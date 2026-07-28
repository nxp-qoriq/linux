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
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#include <linux/fs.h>
#include <linux/slab.h>

extern uint64_t pci_epf_get_vram_fb_address(void);
extern int dpu95_convert_bgrx8888_to_rgbx8888(dma_addr_t src, dma_addr_t dst,
				      u32 width, u32 height,
				      u32 src_stride, u32 dst_stride);

/* RES_FILE use for sync the local and remote display resoluion
	Example:- local work 4k and if user start remote 1k then remote display its not work */
#define RES_FILE "/tmp/last_res.conf"

#define DRIVER_NAME     "stream2v4l2"
#define VIDEO_NAME      "PCIe Host Console"
#define BPP_FORMAT       4 
static int width = 1920;

static int height = 1080;
static int pitch = 0;  /* bytes per row; 0 = use width*BPP */

static DEFINE_SPINLOCK(res_lock);
static bool res_changed;

struct stream2v4l2 {
	struct v4l2_device  v4l2_dev;
	struct video_device vdev;
	struct vb2_queue    vb2_q;
	struct mutex        lock;
	bool streaming;
	u8 *stream_buf;
	size_t map_size;
};

enum soc_type {
    SOC_UNKNOWN,
    SOC_IMX94,
    SOC_IMX95,
};

static enum soc_type current_soc = SOC_UNKNOWN;

static struct stream2v4l2 *g_dev;
static struct platform_device *stream_pdev;

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
	u32 w, h, p;
	int ret;

	if (unlikely(!READ_ONCE(dev->streaming))) {
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		return;
	}
	
	w = READ_ONCE(width);
	h = READ_ONCE(height);
	p = READ_ONCE(pitch);

	if (!p)
		p = w * BPP_FORMAT;

	/* Runtime SoC detection */
	switch (current_soc) {
		case SOC_IMX95:
			/* Hardware conversion path for i.MX95 */
			dma_addr_t src = pci_epf_get_vram_fb_address();
			dma_addr_t dst = vb2_dma_contig_plane_dma_addr(vb, 0);

			if (!src || !dst) {
				pr_err("Invalid DMA address src=%pad dst=%pad\n", &src, &dst);
				vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
				return;
			}			
			ret = dpu95_convert_bgrx8888_to_rgbx8888(src, dst, w, h, p, p);
			if (ret) {
				pr_err("DPU convert failed %d\n", ret);
				vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
				return;
			}
			break;

		case SOC_IMX94:
			/* Software conversion path for i.MX94 */

			u8 *src_ptr, *dst_ptr;
			unsigned int x, y;
			int src_stride, dst_stride;

			if (unlikely(READ_ONCE(res_changed)) || !dev->stream_buf) {
				vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
				return;
			}

        	src_ptr = dev->stream_buf;
        	dst_ptr = vb2_plane_vaddr(vb, 0);

        	if (!dst_ptr || !src_ptr) {
                pr_err("stream2v4l2: vb2_plane_vaddr() returned NULL\n");
                vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
                return;
        	}

        	src_stride = p;
    	    dst_stride = p;
	
			for (y = 0; y < h; y++) {
				const u8 *__restrict src = src_ptr + y * src_stride;
				u8 *__restrict dst = dst_ptr + y * dst_stride;
				for (x = 0; x < w; x++) {
					/*BGRX -> RGBX*/
					dst[0] = src[2];  /* R <- B */
					dst[1] = src[1];  /* G <- G */
					dst[2] = src[0];  /* B <- R */
					dst[3] = src[3];  /* padding */

					src += 4;
					dst += 4;
				}
			}
        break;

	    default:
			pr_warn("Unsupported SoC detected, skipping conversion\n");
			vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);

		return;
	}

	vb2_set_plane_payload(vb, 0, w * h * BPP_FORMAT);
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
	unsigned long flags;
	phys_addr_t phys;
	size_t map_size;
	int w, h, p;

	spin_lock_irqsave(&res_lock, flags);
	w = width;
	h = height;
	p = pitch;
	spin_unlock_irqrestore(&res_lock, flags);

	if (!p)
		p = w * BPP_FORMAT;

	if (*nbufs < 4)
		*nbufs = 4;

	/* Single plane RGB24 */
	*nplanes = 1;
	sizes[0] = max_t(size_t, (size_t)h * p, PAGE_SIZE);

	if (current_soc != SOC_IMX94)
		return 0;

	map_size = (size_t)h * p;

	if (dev->stream_buf && dev->map_size != map_size) {
		memunmap(dev->stream_buf);
		dev->stream_buf = NULL;
		dev->map_size = 0;
	}

	if (dev->stream_buf)
		return 0;
		
	if (!dev->stream_buf) {

		phys_addr_t phys = pci_epf_get_vram_fb_address();

		dev->stream_buf = memremap(phys, map_size, MEMREMAP_WB);
		if (!dev->stream_buf)
			return -ENOMEM;
		
		dev->map_size = map_size;
	}

	return 0;
}

static int stream2v4l2_start_streaming(struct vb2_queue *vq,
					unsigned int count)
{
	struct stream2v4l2 *dev = vb2_get_drv_priv(vq);

	dev->streaming = true;
	res_changed = false;

	return 0;
}

static void stream2v4l2_stop_streaming(struct vb2_queue *vq)
{
	struct stream2v4l2 *dev = vb2_get_drv_priv(vq);
	
	if (dev->stream_buf) {
			memunmap(dev->stream_buf);
			dev->stream_buf = NULL;
			dev->map_size = 0;
	}

	dev->streaming = false;
}

static int stream2v4l2_buf_prepare(struct vb2_buffer *vb)
{
	u32 size;
	
	size = READ_ONCE(width) * READ_ONCE(height) * BPP_FORMAT;
	if (vb2_plane_size(vb,0) < size)
		return -EINVAL;

	vb2_set_plane_payload(vb, 0 ,size);
	return 0;
}

static void stream2v4l2_buf_finish(struct vb2_buffer *vb)
{
	
}

static const struct vb2_ops stream2v4l2_vb2_ops = {
	.queue_setup     = stream2v4l2_queue_setup,
	.buf_prepare     = stream2v4l2_buf_prepare,
	.buf_queue       = stream2v4l2_buf_queue,
	.buf_finish      = stream2v4l2_buf_finish,
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
	f->fmt.pix.bytesperline = width * BPP_FORMAT;
	f->fmt.pix.sizeimage    = width * height * BPP_FORMAT;
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
	.release        = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap           = vb2_fop_mmap,
	.poll           = vb2_fop_poll,
};

static void load_resolution(void)
{
    struct file *filp;
    char buf[64];
    loff_t pos = 0;
    ssize_t len;
	
	int last_w;
	int last_h;

    last_w = width;
    last_h = height;

    filp = filp_open(RES_FILE, O_RDONLY, 0);
    if (IS_ERR(filp)) {
        pr_info("No saved resolution, using default %dx%d\n",
                width, height);
        return;
    }

    len = kernel_read(filp, buf, sizeof(buf) - 1, &pos);
    filp_close(filp, NULL);

    if (len <= 0) {
        pr_info("Failed to read resolution file\n");
        return;
    }

    buf[len] = '\0';

    if (sscanf(buf, "%d %d", &width, &height) != 2) {
		width = last_w;
    	height = last_h;
    }

    pr_info("Restored resolution %dx%d\n", width, height);
}

static int __init stream2v4l2_init(void)
{
	struct stream2v4l2 *dev;
	int rc;

    if (of_machine_is_compatible("fsl,imx95"))
        current_soc = SOC_IMX95;
    else if (of_machine_is_compatible("fsl,imx94"))
        current_soc = SOC_IMX94;
    else
        current_soc = SOC_UNKNOWN;

    pr_info("Detected SoC_type: %d\n", current_soc);

	load_resolution();

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	mutex_init(&dev->lock);
	dev->streaming = false;

	strscpy(dev->v4l2_dev.name, DRIVER_NAME, sizeof(dev->v4l2_dev.name));
	strscpy(dev->vb2_q.name, DRIVER_NAME, sizeof(dev->vb2_q.name));

	stream_pdev = platform_device_register_simple(DRIVER_NAME, -1, NULL, 0);
	if (IS_ERR(stream_pdev)) {
			rc = PTR_ERR(stream_pdev);
			goto err_free;
	}

	rc = dma_set_mask_and_coherent(&stream_pdev->dev,DMA_BIT_MASK(64));
	if (rc)
		pr_err("Failed dma set mask");

	rc = v4l2_device_register(&stream_pdev->dev, &dev->v4l2_dev);
	if (rc)
		goto err_pdev;

	dev->vb2_q.dev = &stream_pdev->dev;
	dev->vb2_q.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dev->vb2_q.io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ;
	dev->vb2_q.gfp_flags = GFP_DMA32;
	dev->vb2_q.ops = &stream2v4l2_vb2_ops;
	dev->vb2_q.mem_ops = &vb2_dma_contig_memops;
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

err_pdev:
    platform_device_unregister(stream_pdev);	
err_v4l2:
	v4l2_device_unregister(&dev->v4l2_dev);
err_free:
	kfree(dev);

	return rc;
}

static void __exit stream2v4l2_exit(void)
{
	struct stream2v4l2 *dev = g_dev;

	if (!dev)
		return;

	if (current_soc == SOC_IMX94 && dev->stream_buf) {
		memunmap(dev->stream_buf);
		dev->stream_buf = NULL;
	}

	vb2_queue_release(&dev->vb2_q);
	video_unregister_device(&dev->vdev);
	v4l2_device_unregister(&dev->v4l2_dev);
	platform_device_unregister(stream_pdev);

	kfree(dev);
}

/*
 * Resolution update: dpu_pci_scan writes new width/height directly to
 * /sys/module/stream2v4l2/parameters/width and /height after a
 * RESOLUTION_CHANGE uevent. The module_param callback sets res_changed
 * so queue_setup re-maps /dev/stream at the new size.
 */
static int param_set_resolution(const char *val, const struct kernel_param *kp)
{
	unsigned long flags;
	int w, h, new_pitch;

	if (sscanf(val, "%dx%d:%d", &w, &h, &new_pitch) != 3)
		return -EINVAL;

	if (w <= 0 || h <= 0 || new_pitch <= 0)
		return -EINVAL;

	spin_lock_irqsave(&res_lock, flags);

	if (width != w || height != h || pitch != new_pitch) {
		WRITE_ONCE(width,w);
		WRITE_ONCE(height,h);
		WRITE_ONCE(pitch,new_pitch);
		WRITE_ONCE(res_changed,true);
	}
	spin_unlock_irqrestore(&res_lock, flags);
	return 0;
}

static const struct kernel_param_ops resolution_ops = {
	.set = param_set_resolution,
	.get = param_get_int,
};

module_param_cb(resolution, &resolution_ops, NULL, 0644);
MODULE_PARM_DESC(resolution, "Frame resolution + Frame pitch (bytes/row) - 0 = width*4 (e.g. 1920x1080:7680)");

module_init(stream2v4l2_init);
module_exit(stream2v4l2_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("stream2v4l2 RGB24 V4L2 device (iKVM compatible)");