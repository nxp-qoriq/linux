// SPDX-License-Identifier: GPL-2.0-only
// Copyright 2026 NXP

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/timekeeping.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/spinlock.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-vmalloc.h>

#define DRIVER_NAME     "stream2v4l2"
#define VIDEO_NAME      "PCIe Host Console"

static int width = 1920;

static int height = 1080;
static int pitch = 0;  /* bytes per row; 0 = use width*BPP */

static DEFINE_SPINLOCK(res_lock);
static bool res_changed;

#define BPP    4
#define FRAME_SIZE (width * height * BPP)

struct stream2v4l2 {
	struct v4l2_device  v4l2_dev;
	struct video_device vdev;
	struct vb2_queue    vb2_q;
	struct mutex        lock;
	struct file *stream_filp;
	void *stream_buf;
	size_t stream_size;
	int mapped_width;
	int mapped_height;
	u8 *tmp_buf;
};

static struct stream2v4l2 *g_dev;

static void stream2v4l2_wait_prepare(struct vb2_queue *vq)
{
	struct stream2v4l2 *dev = vb2_get_drv_priv(vq);

	mutex_unlock(&dev->lock);
}

static void stream2v4l2_wait_finish(struct vb2_queue *vq)
{
	struct stream2v4l2 *dev = vb2_get_drv_priv(vq);

	mutex_lock(&dev->lock);
}

/* Frame copy */
static void stream2v4l2_buf_queue(struct vb2_buffer *vb)
{
	struct stream2v4l2 *dev = vb2_get_drv_priv(vb->vb2_queue);
	void *dst;
	u8 *out;
	u8 *tmp;
	u8 __user *src;
	int pixels;

	if (!dev->stream_buf) {
		pr_err("stream2v4l2: stream not mapped\n");
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		return;
	}

	dst = vb2_plane_vaddr(vb, 0);
	if (!dst) {
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		return;
	}

	tmp = dev->tmp_buf;
	if (!tmp) {
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		return;
	}

	src = (u8 __user *)dev->stream_buf;

	{
		unsigned long flags;

		spin_lock_irqsave(&res_lock, flags);
		pixels = width * height;
		spin_unlock_irqrestore(&res_lock, flags);
	}

	if (copy_from_user(tmp, src, (size_t)pixels * 4) != 0) {
		pr_err("stream2v4l2: copy_from_user failed\n");
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		return;
	}

	int src_stride, dst_stride, x, y, src_idx, dst_idx;

	out = (u8 *)dst;
	/* Use actual FB pitch if set, else default to width*4 */
	src_stride = pitch ? pitch : width * 4;
	dst_stride = width * 4;   /* RGB24 output (always packed) */

	for (y = 0; y < height; y++) {
		const u8 *src_row = tmp + y * src_stride;
		u8 *dst_row = out + y * dst_stride;

		for (x = 0; x < width; x++) {
			src_idx = x * 4;
			dst_idx = x * 4;

			dst_row[dst_idx + 0] = src_row[src_idx + 2];  // R
			dst_row[dst_idx + 1] = src_row[src_idx + 1];  // G
			dst_row[dst_idx + 2] = src_row[src_idx + 0];  // B
			dst_row[dst_idx + 3] = 0x00;                 // padding
		}
	}

	vb2_set_plane_payload(vb, 0, FRAME_SIZE);
	vb->timestamp = ktime_get_ns();

	vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
}

/* Setup */
static int stream2v4l2_queue_setup(struct vb2_queue *vq,
					unsigned int *nbufs,
					unsigned int *nplanes,
					unsigned int sizes[],
					struct device *alloc_devs[])
{
	struct stream2v4l2 *dev = vb2_get_drv_priv(vq);
	unsigned long addr;

	/* Ensure at least 2 buffers */
	if (*nbufs < 2)
		*nbufs = 2;

	/* Single plane RGB24 */
	*nplanes = 1;
	sizes[0] = FRAME_SIZE;

	/* Re-map if resolution changed since last map */
	if (res_changed && dev->stream_buf) {
		unsigned long flags;
		int w, h;

		spin_lock_irqsave(&res_lock, flags);
		w = width;
		h = height;
		spin_unlock_irqrestore(&res_lock, flags);

		vm_munmap((unsigned long)dev->stream_buf, dev->stream_size);
		dev->stream_buf = NULL;
		if (dev->stream_filp) {
			filp_close(dev->stream_filp, NULL);
			dev->stream_filp = NULL;
		}
		kfree(dev->tmp_buf);
		dev->tmp_buf = kmalloc((size_t)w * h * BPP, GFP_KERNEL);
		res_changed = false;
		pr_info("stream2v4l2: resolution updated to %dx%d\n", w, h);
	}

	/* Map /dev/stream only once */
	if (!dev->stream_buf) {
		dev->stream_filp = filp_open("/dev/stream", O_RDWR, 0);
		if (IS_ERR(dev->stream_filp)) {
			pr_err("stream2v4l2: failed to open /dev/stream\n");
		return PTR_ERR(dev->stream_filp);
		}

		/* Map stream buffer (XBGR32 at 4 bytes per pixel) */
		size_t map_size;

		/* Map using actual pitch*height to cover full FB */
		map_size = (size_t)(pitch ? pitch : width * 4) * height;
		addr = vm_mmap(dev->stream_filp, 0, map_size, PROT_READ,
		MAP_SHARED, 0);

		if (IS_ERR_VALUE(addr)) {
			pr_err("stream2v4l2: vm_mmap failed\n");
			filp_close(dev->stream_filp, NULL);
			dev->stream_filp = NULL;
			return -EINVAL;
		}

		dev->stream_buf = (void *)addr;
		dev->stream_size = (size_t)(pitch ? pitch : width * 4) * height;

		pr_info("stream2v4l2: mapped /dev/stream at %p\n",
		dev->stream_buf);
	}

	return 0;
}

static int stream2v4l2_start_streaming(struct vb2_queue *vq,
					unsigned int count)
{
	return 0;
}

static void stream2v4l2_stop_streaming(struct vb2_queue *vq)
{
	struct stream2v4l2 *dev = vb2_get_drv_priv(vq);

	if (dev->stream_buf) {
		vm_munmap((unsigned long)dev->stream_buf,
			dev->stream_size);
		dev->stream_buf = NULL;
	}

	if (dev->stream_filp) {
		filp_close(dev->stream_filp, NULL);
		dev->stream_filp = NULL;
	}
}

static int stream2v4l2_buf_prepare(struct vb2_buffer *vb)
{
	if (vb2_plane_size(vb, 0) < FRAME_SIZE)
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, FRAME_SIZE);
	return 0;
}

static const struct vb2_ops stream2v4l2_vb2_ops = {
	.queue_setup     = stream2v4l2_queue_setup,
	.buf_prepare     = stream2v4l2_buf_prepare,
	.buf_queue       = stream2v4l2_buf_queue,
	.start_streaming = stream2v4l2_start_streaming,
	.stop_streaming  = stream2v4l2_stop_streaming,
	.wait_prepare    = stream2v4l2_wait_prepare,
	.wait_finish     = stream2v4l2_wait_finish,
};

/* V4L2 IOCTLs */
static int stream2v4l2_querycap(struct file *file, void *priv,
				struct v4l2_capability *cap)
{
	strscpy(cap->driver, DRIVER_NAME, sizeof(cap->driver));
	strscpy(cap->card, VIDEO_NAME, sizeof(cap->card));
	strscpy(cap->bus_info, "platform:stream2v4l2", sizeof(cap->bus_info));

	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int stream2v4l2_enum_fmt(struct file *file, void *priv,
	       struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_RGB24;
	strscpy(f->description, "RGB24", sizeof(f->description));

	return 0;
}

static int stream2v4l2_g_fmt(struct file *file, void *priv,
				struct v4l2_format *f)
{
	f->fmt.pix.width        = width;
	f->fmt.pix.height       = height;
	f->fmt.pix.pixelformat  = V4L2_PIX_FMT_RGB24;
	f->fmt.pix.field        = V4L2_FIELD_NONE;
	f->fmt.pix.bytesperline = width * 4;
	f->fmt.pix.sizeimage    = width * height * 4;
	f->fmt.pix.colorspace   = V4L2_COLORSPACE_SRGB;
	f->fmt.pix.ycbcr_enc    = V4L2_YCBCR_ENC_DEFAULT;
	f->fmt.pix.quantization = V4L2_QUANTIZATION_DEFAULT;
	f->fmt.pix.xfer_func    = V4L2_XFER_FUNC_DEFAULT;
	f->fmt.pix.priv = 0;

	return 0;
}

static int stream2v4l2_s_fmt(struct file *file, void *priv,
				struct v4l2_format *f)
{
	f->fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
	return stream2v4l2_g_fmt(file, priv, f);
}

static int stream2v4l2_try_fmt(struct file *file, void *priv,
				struct v4l2_format *f)
{
	f->fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
	f->fmt.pix.width  = width;
	f->fmt.pix.height = height;
	f->fmt.pix.field  = V4L2_FIELD_NONE;

	return 0;
}

static int stream2v4l2_g_parm(struct file *file, void *priv,
				struct v4l2_streamparm *p)
{
	if (p->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	p->parm.capture.timeperframe.numerator = 1;
	p->parm.capture.timeperframe.denominator = 30;

	return 0;
}

static int stream2v4l2_s_parm(struct file *file, void *priv,
				struct v4l2_streamparm *p)
{
	return stream2v4l2_g_parm(file, priv, p);
}

static int stream2v4l2_enum_framesizes(struct file *file, void *priv,
					struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index)
		return -EINVAL;

	if (fsize->pixel_format != V4L2_PIX_FMT_RGB24)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width  = width;
	fsize->discrete.height = height;

	return 0;
}

static int stream2v4l2_enum_frameintervals(struct file *file, void *priv,
					struct v4l2_frmivalenum *fival)
{
	if (fival->index)
		return -EINVAL;

	if (fival->pixel_format != V4L2_PIX_FMT_RGB24)
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = 30;

	return 0;
}

static int stream2v4l2_query_dv_timings(struct file *file, void *priv,
					struct v4l2_dv_timings *timings)
{
	memset(timings, 0, sizeof(*timings));

	timings->type = V4L2_DV_BT_656_1120;
	timings->bt.width = width;
	timings->bt.height = height;
	timings->bt.pixelclock = 148500000;
	timings->bt.hfrontporch = 88;
	timings->bt.hsync = 44;
	timings->bt.hbackporch = 148;
	timings->bt.vfrontporch = 4;
	timings->bt.vsync = 5;
	timings->bt.vbackporch = 36;

	return 0;
}

static int stream2v4l2_s_dv_timings(struct file *file, void *priv,
					struct v4l2_dv_timings *timings)
{
	return 0;
}

static const struct v4l2_ioctl_ops stream2v4l2_ioctl_ops = {
	.vidioc_querycap         = stream2v4l2_querycap,
	.vidioc_enum_fmt_vid_cap = stream2v4l2_enum_fmt,
	.vidioc_g_fmt_vid_cap    = stream2v4l2_g_fmt,
	.vidioc_s_fmt_vid_cap    = stream2v4l2_s_fmt,
	.vidioc_try_fmt_vid_cap  = stream2v4l2_try_fmt,

	.vidioc_reqbufs          = vb2_ioctl_reqbufs,
	.vidioc_querybuf         = vb2_ioctl_querybuf,
	.vidioc_qbuf             = vb2_ioctl_qbuf,
	.vidioc_dqbuf            = vb2_ioctl_dqbuf,
	.vidioc_streamon         = vb2_ioctl_streamon,
	.vidioc_streamoff        = vb2_ioctl_streamoff,

	.vidioc_g_parm = stream2v4l2_g_parm,
	.vidioc_s_parm = stream2v4l2_s_parm,

	.vidioc_enum_framesizes = stream2v4l2_enum_framesizes,
	.vidioc_enum_frameintervals = stream2v4l2_enum_frameintervals,
	.vidioc_query_dv_timings = stream2v4l2_query_dv_timings,
	.vidioc_s_dv_timings     = stream2v4l2_s_dv_timings,
};

static const struct v4l2_file_operations stream2v4l2_fops = {
	.owner          = THIS_MODULE,
	.open           = v4l2_fh_open,
	.release        = v4l2_fh_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap           = vb2_fop_mmap,
	.poll           = vb2_fop_poll,
};

static int __init stream2v4l2_init(void)
{
	struct stream2v4l2 *dev;
	int rc;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->tmp_buf = kmalloc(width * height * 4, GFP_KERNEL);
	if (!dev->tmp_buf) {
		rc = -ENOMEM;
		goto err_free;
	}

	mutex_init(&dev->lock);

	strscpy(dev->v4l2_dev.name, DRIVER_NAME, sizeof(dev->v4l2_dev.name));
	strscpy(dev->vb2_q.name, DRIVER_NAME, sizeof(dev->vb2_q.name));

	rc = v4l2_device_register(NULL, &dev->v4l2_dev);
	if (rc)
		goto err_free;

	dev->vb2_q.dev = dev->v4l2_dev.dev;
	dev->vb2_q.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dev->vb2_q.io_modes = VB2_MMAP;
	dev->vb2_q.ops = &stream2v4l2_vb2_ops;
	dev->vb2_q.mem_ops = &vb2_vmalloc_memops;
	dev->vb2_q.lock = &dev->lock;
	dev->vb2_q.drv_priv = dev;
	dev->vb2_q.buf_struct_size = sizeof(struct vb2_v4l2_buffer);
	dev->vb2_q.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	rc = vb2_queue_init(&dev->vb2_q);
	if (rc)
		goto err_v4l2;

	strscpy(dev->vdev.name, VIDEO_NAME, sizeof(dev->vdev.name));

	dev->vdev.v4l2_dev  = &dev->v4l2_dev;
	dev->vdev.fops      = &stream2v4l2_fops;
	dev->vdev.ioctl_ops = &stream2v4l2_ioctl_ops;
	dev->vdev.queue     = &dev->vb2_q;
	dev->vdev.lock      = &dev->lock;
	dev->vdev.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	dev->vdev.release = video_device_release_empty;

	video_set_drvdata(&dev->vdev, dev);

	rc = video_register_device(&dev->vdev, VFL_TYPE_VIDEO, -1);
	if (rc)
		goto err_v4l2;

	g_dev = dev;

	pr_info("stream2v4l2: registered /dev/video%d\n", dev->vdev.num);
	return 0;

err_v4l2:
	v4l2_device_unregister(&dev->v4l2_dev);
err_free:
	kfree(dev->tmp_buf);
	kfree(dev);

	return rc;
}

static void __exit stream2v4l2_exit(void)
{
	struct stream2v4l2 *dev = g_dev;

	if (!dev)
		return;

	if (dev->stream_buf) {
		vm_munmap((unsigned long)dev->stream_buf,
			dev->stream_size);
	}

	if (dev->stream_filp)
		filp_close(dev->stream_filp, NULL);

	video_unregister_device(&dev->vdev);
	v4l2_device_unregister(&dev->v4l2_dev);

	kfree(dev->tmp_buf);
	kfree(dev);
}

/*
 * Resolution update: dpu_pci_scan writes new width/height directly to
 * /sys/module/stream2v4l2/parameters/width and /height after a
 * RESOLUTION_CHANGE uevent. The module_param callback sets res_changed
 * so queue_setup re-maps /dev/stream at the new size.
 */
static int param_set_width(const char *val, const struct kernel_param *kp)
{
	unsigned long flags;
	int new_w, ret;

	ret = kstrtoint(val, 0, &new_w);
	if (ret || new_w <= 0)
		return ret ?: -EINVAL;

	spin_lock_irqsave(&res_lock, flags);
	if (new_w != width) {
		width = new_w;
		res_changed = true;
		pr_info("stream2v4l2: width updated to %d\n", new_w);
	}
	spin_unlock_irqrestore(&res_lock, flags);
	return 0;
}

static int param_set_height(const char *val, const struct kernel_param *kp)
{
	unsigned long flags;
	int new_h, ret;

	ret = kstrtoint(val, 0, &new_h);
	if (ret || new_h <= 0)
		return ret ?: -EINVAL;

	spin_lock_irqsave(&res_lock, flags);
	if (new_h != height) {
		height = new_h;
		res_changed = true;
		pr_info("stream2v4l2: height updated to %d\n", new_h);
	}
	spin_unlock_irqrestore(&res_lock, flags);
	return 0;
}

static const struct kernel_param_ops width_ops = {
	.set = param_set_width,
	.get = param_get_int,
};

static const struct kernel_param_ops height_ops = {
	.set = param_set_height,
	.get = param_get_int,
};

static int param_set_pitch(const char *val, const struct kernel_param *kp)
{
	unsigned long flags;
	int new_pitch, ret;

	ret = kstrtoint(val, 0, &new_pitch);
	if (ret || new_pitch < 0)
		return ret ?: -EINVAL;

	spin_lock_irqsave(&res_lock, flags);
	if (new_pitch != pitch) {
		pitch = new_pitch;
		res_changed = true;
		pr_info("stream2v4l2: pitch updated to %d\n", new_pitch);
	}
	spin_unlock_irqrestore(&res_lock, flags);
	return 0;
}

static const struct kernel_param_ops pitch_ops = {
	.set = param_set_pitch,
	.get = param_get_int,
};

module_param_cb(width,  &width_ops,  &width,  0644);
MODULE_PARM_DESC(width,  "Frame width - write to update resolution");
module_param_cb(height, &height_ops, &height, 0644);
MODULE_PARM_DESC(height, "Frame height - write to update resolution");
module_param_cb(pitch,  &pitch_ops,  &pitch,  0644);
MODULE_PARM_DESC(pitch,  "Frame pitch (bytes/row) - 0 = width*4");

module_init(stream2v4l2_init);
module_exit(stream2v4l2_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("stream2v4l2 RGB24 V4L2 device (iKVM compatible)");