// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2026 NXP
 *
 * PCIe Endpoint VGA Class Display Driver for i.MX95 iKVM
 *
 * Implements a PCI_BASE_CLASS_DISPLAY endpoint function driver,
 * separated from the PCI EPF test framework.
 *
 * BAR layout (PCI Type-0 config space):
 *
 *   Offset 0x10  BAR0 - 512B   : control registers (pcitest wire-format compatible)
 *   Offset 0x14  BAR1 - 512B   : EDID (DRM sysfs or 1080p/4K fallback blob)
 *   Offset 0x18  BAR2 - 64MB   : VRAM framebuffer (ioremap_wc, DT "vram" node)
 *   Offset 0x1C  BAR3 - 32KB   : PCIe HID ring buffer (keyboard + mouse reports)
 *   Offset 0x20  BAR4 - 4MB    : DPU95 register space (ioremap at DPU95_ADDRESS_SPACE)
 *   Offset 0x24  BAR5 - 1MB    : reserved
 *
 *   Offset 0x30  PCI_ROM_ADDRESS (Expansion ROM BAR, NOT BAR6):
 *                  Size: rounded up to next power-of-2, minimum 64KB per PCI spec.
 *                  Loaded via request_firmware(), selected by kvm_mode:
 *                   pcie_video_usb_hid -> OptionRom.rom
 *                   pcie_kvm          -> OptionRom_pcie_kvm.rom
 */

#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/msi.h>
#include <linux/slab.h>
#include <linux/pci_ids.h>
#include <linux/random.h>
#include <linux/kernel.h>
#include <linux/fb.h>
#include <linux/device.h>
#include <linux/dma-map-ops.h>
#include <linux/firmware.h>

#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/pci-ep-msi.h>
#include <linux/pci_regs.h>

#include <linux/dma-mapping.h>

#if IS_ENABLED(CONFIG_PCIE_KVM_INPUT)
#include "pci-epf-hid.h"
#endif

/* DPU95 register space physical base address on i.MX95 */
#define DPU95_ADDRESS_SPACE		0x4b400000UL

/* EDID sysfs path and size */
#define EDID_PATH	"/sys/class/drm/card0-HDMI-A-1/edid"
#define EDID_SIZE	256

/* IRQ types */
#define IRQ_TYPE_INTX			0
#define IRQ_TYPE_MSI			1
#define IRQ_TYPE_MSIX			2

/* Command bits - base test commands (kept for wire-format compatibility) */
#define COMMAND_RAISE_INTX_IRQ		BIT(0)
#define COMMAND_RAISE_MSI_IRQ		BIT(1)
#define COMMAND_RAISE_MSIX_IRQ		BIT(2)
#define COMMAND_READ			BIT(3)
#define COMMAND_WRITE			BIT(4)
#define COMMAND_COPY			BIT(5)
#define COMMAND_ENABLE_DOORBELL		BIT(6)
#define COMMAND_DISABLE_DOORBELL	BIT(7)
/* Display-specific command bits */
#define COMMAND_CHANGE_RESOLUTION	BIT(8)
#define COMMAND_GET_FB_OFFSET		BIT(9)
#define COMMAND_START_STREAM		BIT(10)
#define COMMAND_STOP_STREAM		BIT(11)

/* Status bits */
#define STATUS_READ_SUCCESS		BIT(0)
#define STATUS_READ_FAIL		BIT(1)
#define STATUS_WRITE_SUCCESS		BIT(2)
#define STATUS_WRITE_FAIL		BIT(3)
#define STATUS_COPY_SUCCESS		BIT(4)
#define STATUS_COPY_FAIL		BIT(5)
#define STATUS_IRQ_RAISED		BIT(6)
#define STATUS_SRC_ADDR_INVALID		BIT(7)
#define STATUS_DST_ADDR_INVALID		BIT(8)
#define STATUS_DOORBELL_SUCCESS		BIT(9)
#define STATUS_DOORBELL_ENABLE_SUCCESS	BIT(10)
#define STATUS_DOORBELL_ENABLE_FAIL	BIT(11)
#define STATUS_DOORBELL_DISABLE_SUCCESS	BIT(12)
#define STATUS_DOORBELL_DISABLE_FAIL	BIT(13)

#define FLAG_USE_DMA			BIT(0)
#define TIMER_RESOLUTION		1

/* Capability bits */
#define CAP_UNALIGNED_ACCESS		BIT(0)
#define CAP_MSI				BIT(1)
#define CAP_MSIX			BIT(2)
#define CAP_INTX			BIT(3)

/* VRAM state - module-level, exported via pci_epf_get_vram_fb_address() */
static DEFINE_SPINLOCK(imx9_vga_offset_lock);
static phys_addr_t vram_phys;
static size_t      vram_size;
static uint64_t    vram_fb_offset;
static uint64_t    vram_fb_addr;

static struct workqueue_struct *kpciimx9vga_workqueue;

/*
 * param_lock: protects param_kvm_mode and param_display_resolution against
 * concurrent sysfs writes after module load.
 */
static DEFINE_MUTEX(param_lock);

/* ------------------------------------------------------------------ */
/* Module parameters: KVM configuration                                */
/* ------------------------------------------------------------------ */

/*
 * kvm_mode: selects the KVM operating mode.
 *
 *   "pcie_kvm"            - Full PCIe KVM mode (video + PCIe HID ring)
 *   "pcie_video_usb_hid"  - PCIe video with USB HID redirection (default)
 *
 * Set at module load time:
 *   modprobe pci-epf-imx9-vga kvm_mode=pcie_kvm
 * or via kernel command line:
 *   pci_epf_imx9_vga.kvm_mode=pcie_kvm
 * or at runtime via sysfs:
 *   echo pcie_kvm > /sys/module/pci_epf_imx9_vga/parameters/kvm_mode
 */
static char param_kvm_mode[32] = "pcie_video_usb_hid";

/*
 * display_resolution: selects the display output resolution.
 *
 *   "1080p" - 1920x1080 FHD (default)
 *   "4k"    - 3840x2160 UHD
 *
 * Set at module load time:
 *   modprobe pci-epf-imx9-vga display_resolution=4k
 * or via kernel command line:
 *   pci_epf_imx9_vga.display_resolution=4k
 * or at runtime via sysfs:
 *   echo 4k > /sys/module/pci_epf_imx9_vga/parameters/display_resolution
 */
static char param_display_resolution[32] = "1080p";

/**
 * enum display_mode - supported display output resolutions
 * @DISPLAY_1080P: Configure the display for 1080p (FHD) resolution.
 * @DISPLAY_4K: Configure the display for 4K resolution.
 */
enum display_mode {
	DISPLAY_1080P,
	DISPLAY_4K,
};

/**
 * enum kvm_mode - supported KVM operating modes
 * @KVM_MODE_PCIE_VIDEO_USB_HID: PCIe video with USB HID redirection.
 * @KVM_MODE_PCIE_KVM: Full PCIe KVM mode.
 */
enum imx9_kvm_mode {
	KVM_MODE_PCIE_VIDEO_USB_HID,
	KVM_MODE_PCIE_KVM,
};

/**
 * struct kvm_config - KVM configuration parameters
 * @display: Selected display resolution.
 * @mode: Selected KVM operating mode.
 */
struct kvm_config {
	enum display_mode display;
	enum imx9_kvm_mode mode;
};

/**
 * struct edid_resolution - display resolution described by EDID
 * @width: Horizontal resolution in pixels.
 * @height: Vertical resolution in pixels.
 */
struct edid_resolution {
	u16 width;
	u16 height;
};

/**
 * struct pci_epf_imx9_vga - private state for the VGA EPF driver instance
 * @reg:          virtual addresses of each BAR's register space
 * @epf:          back-pointer to the EPF device
 * @rom_bar:      Option ROM BAR descriptor
 * @ctrl_reg_bar: BAR number used for the control register window (BAR_0)
 * @msix_table_offset: offset of MSI-X table within ctrl_reg_bar
 * @cmd_handler:  delayed work for polling the command register
 * @epc_features: EPC feature set for this function
 * @cfg: Global KVM configuration instance used to store the current
 *       display resolution and KVM operating mode.
 */
struct pci_epf_imx9_vga {
	void				*reg[PCI_STD_NUM_BARS];
	struct pci_epf			*epf;
	struct pci_epf_bar		 rom_bar;
	enum pci_barno			 ctrl_reg_bar;
	size_t				 msix_table_offset;
	struct delayed_work		 cmd_handler;
	const struct pci_epc_features	*epc_features;
	struct kvm_config cfg;
#if IS_ENABLED(CONFIG_PCIE_KVM_INPUT)	
	atomic_t hid_attached;
#endif	
};

/**
 * struct pci_epf_imx9_vga_reg - control register layout (BAR0)
 *
 * Wire-format compatible with pci_epf_test_reg so that the host-side
 * pcitest utility can exercise the base READ/WRITE/COPY/IRQ commands.
 * The display-specific fields (height, width, dpu_fb_offset) are additive.
 */
struct pci_epf_imx9_vga_reg {
	__le32 magic;
	__le32 command;
	__le32 status;
	__le64 src_addr;
	__le64 dst_addr;
	__le32 size;
	__le32 checksum;
	__le32 irq_type;
	__le32 irq_number;
	__le32 flags;
	__le32 height;		/* display: current frame height in pixels */
	__le32 width;		/* display: current frame width in pixels  */
	__le64 dpu_fb_offset;	/* display: DPU framebuffer offset in VRAM  */
	__le32 caps;
	__le32 doorbell_bar;
	__le32 doorbell_offset;
	__le32 doorbell_data;
} __packed;

/* PCI configuration header: VGA class display device */
static struct pci_epf_header imx9_vga_header = {
	.vendorid	= PCI_VENDOR_ID_IMX9_VGA,	/* 0x1131 (Philips/NXP) */
	.deviceid	= PCI_DEVICE_ID_IMX9_VGA,	/* 0x1234               */
	.baseclass_code	= PCI_BASE_CLASS_DISPLAY,
	.interrupt_pin	= PCI_INTERRUPT_INTA,
};

/*
 * BAR sizes:
 *   BAR0 = 512B   (control regs)
 *   BAR1 = 512B   (EDID)
 *   BAR2 = 64MB   (VRAM - overridden by alloc_bar to DT size)
 *   BAR3 = 32KB   (PCIe HID ring buffer)
 *   BAR4 = 4MB    (DPU95 regs)
 *   BAR5 = 1MB    (reserved)
 */
static size_t imx9_vga_bar_size[] = { 512, 512, 67108864, 32768, 4194304, 1048576 };

/* ------------------------------------------------------------------ */
/* Default 1080P EDID                                                  */
/* ------------------------------------------------------------------ */
static const unsigned char default_edid_1080p[256] = {
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00,
	0x1e, 0x6d, 0x55, 0x5b, 0xd5, 0x56, 0x07, 0x00,
	0x01, 0x21, 0x01, 0x03, 0x80, 0x30, 0x1b, 0x78,
	0xea, 0x31, 0x35, 0xa5, 0x55, 0x4e, 0xa1, 0x26,
	0x0c, 0x50, 0x54, 0xa5, 0x4b, 0x00, 0x71, 0x4f,
	0x81, 0x80, 0x95, 0x00, 0xb3, 0x00, 0xa9, 0xc0,
	0x81, 0x00, 0x81, 0xc0, 0x90, 0x40, 0x02, 0x3a,
	0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c,
	0x45, 0x00, 0xe0, 0x0e, 0x11, 0x00, 0x00, 0x1e,
	0x00, 0x00, 0x00, 0xfd, 0x00, 0x38, 0x4b, 0x1e,
	0x55, 0x12, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20,
	0x20, 0x20, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x4c,
	0x47, 0x20, 0x46, 0x55, 0x4c, 0x4c, 0x20, 0x48,
	0x44, 0x0a, 0x20, 0x20, 0x00, 0x00, 0x00, 0xff,
	0x00, 0x33, 0x30, 0x31, 0x4e, 0x54, 0x5a, 0x4e,
	0x45, 0x34, 0x39, 0x38, 0x31, 0x0a, 0x01, 0xb3,
	0x02, 0x03, 0x1b, 0xf1, 0x48, 0x90, 0x04, 0x03,
	0x01, 0x12, 0x1f, 0x10, 0x13, 0x23, 0x09, 0x07,
	0x07, 0x83, 0x01, 0x00, 0x00, 0x65, 0x03, 0x0c,
	0x00, 0x10, 0x00, 0x02, 0x3a, 0x80, 0x18, 0x71,
	0x38, 0x2d, 0x40, 0x58, 0x2c, 0x45, 0x00, 0xe0,
	0x0e, 0x11, 0x00, 0x00, 0x1e, 0x2a, 0x44, 0x80,
	0xa0, 0x70, 0x38, 0x27, 0x40, 0x30, 0x20, 0x35,
	0x00, 0xe0, 0x0e, 0x11, 0x00, 0x00, 0x1e, 0x01,
	0x1d, 0x00, 0x72, 0x51, 0xd0, 0x1e, 0x20, 0x6e,
	0x28, 0x55, 0x00, 0xe0, 0x0e, 0x11, 0x00, 0x00,
	0x1e, 0x8c, 0x0a, 0xd0, 0x8a, 0x20, 0xe0, 0x2d,
	0x10, 0x10, 0x3e, 0x96, 0x00, 0xe0, 0x0e, 0x11,
	0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4b
};

/* ------------------------------------------------------------------ */
/* Default 4K EDID                                                     */
/* ------------------------------------------------------------------ */
static const unsigned char default_edid_4k[256] = {
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00,
	0x09, 0xd1, 0x49, 0x79, 0x45, 0x54, 0x00, 0x00,
	0x03, 0x20, 0x01, 0x03, 0x80, 0x3e, 0x22, 0x78,
	0x2e, 0x08, 0xa5, 0xa2, 0x57, 0x4f, 0xa2, 0x28,
	0x0f, 0x50, 0x54, 0xa5, 0x6b, 0x80, 0xd1, 0xc0,
	0x81, 0xc0, 0x81, 0x00, 0x81, 0x80, 0xa9, 0xc0,
	0xb3, 0x00, 0xa9, 0x40, 0x01, 0x01, 0x51, 0xd0,
	0x00, 0xa0, 0xf0, 0x70, 0x3e, 0x80, 0x30, 0x20,
	0x35, 0x00, 0x6d, 0x55, 0x21, 0x00, 0x00, 0x1a,
	0x00, 0x00, 0x00, 0xff, 0x00, 0x32, 0x31, 0x4e,
	0x30, 0x37, 0x35, 0x38, 0x36, 0x53, 0x4c, 0x30,
	0x0a, 0x20, 0x00, 0x00, 0x00, 0xfd, 0x00, 0x18,
	0x4c, 0x1e, 0x87, 0x3c, 0x00, 0x0a, 0x20, 0x20,
	0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xfc,
	0x00, 0x42, 0x65, 0x6e, 0x51, 0x20, 0x45, 0x4c,
	0x32, 0x38, 0x37, 0x30, 0x55, 0x0a, 0x01, 0x63,
	0x02, 0x03, 0x4e, 0xf1, 0x56, 0x61, 0x60, 0x5d,
	0x5e, 0x5f, 0x10, 0x05, 0x04, 0x03, 0x02, 0x07,
	0x06, 0x0f, 0x1f, 0x20, 0x21, 0x22, 0x14, 0x13,
	0x12, 0x16, 0x01, 0x23, 0x09, 0x07, 0x07, 0x6d,
	0x03, 0x0c, 0x00, 0x10, 0x00, 0x38, 0x44, 0x20,
	0x00, 0x60, 0x01, 0x02, 0x03, 0x67, 0xd8, 0x5d,
	0xc4, 0x01, 0x78, 0x80, 0x03, 0xe3, 0x05, 0xc3,
	0x01, 0xe4, 0x0f, 0x03, 0x00, 0x00, 0xe6, 0x06,
	0x05, 0x01, 0x5a, 0x53, 0x44, 0x68, 0x1a, 0x00,
	0x00, 0x01, 0x01, 0x28, 0x3c, 0x00, 0x56, 0x5e,
	0x00, 0xa0, 0xa0, 0xa0, 0x29, 0x50, 0x30, 0x20,
	0x35, 0x00, 0x6d, 0x55, 0x21, 0x00, 0x00, 0x1a,
	0xf4, 0x51, 0x00, 0xa0, 0xf0, 0x70, 0x19, 0x80,
	0x30, 0x20, 0x35, 0x00, 0x6d, 0x55, 0x21, 0x00,
	0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfb
};

/**
 * @hid_addr: Virtual __iomem address of the shared memory region used for
 *            PCIe HID report exchange between the endpoint and host.
 */
static void __iomem *hid_addr;

/* ------------------------------------------------------------------ */
/* VRAM helpers                                                         */
/* ------------------------------------------------------------------ */

/**
 * epf_imx9_vga_init_vram() - parse the "vram" DT reserved-memory node
 *
 * Reads the physical base address and size of the VRAM region from the
 * device tree.  Called once at module_init time.
 */
static int epf_imx9_vga_init_vram(void)
{
	struct device_node *vram_node;
	struct resource res;

	vram_node = of_find_node_by_name(NULL, "vram");
	if (!vram_node) {
		pr_err("pci-epf-imx9-vga: VRAM node not found in DT\n");
		return -ENODEV;
	}

	if (of_address_to_resource(vram_node, 0, &res)) {
		pr_err("pci-epf-imx9-vga: Failed to get VRAM resource\n");
		of_node_put(vram_node);
		return -EINVAL;
	}

	of_node_put(vram_node);

	vram_phys = res.start;
	vram_size = resource_size(&res);

	pr_info("pci-epf-imx9-vga: VRAM phys=%pa size=0x%zx\n",
		&vram_phys, vram_size);
	return 0;
}

/**
 * pci_epf_get_vram_fb_address() - return the current VRAM framebuffer address
 *
 * Returns the physical address of the active framebuffer within VRAM.
 * If no explicit offset has been set via the "offset" module parameter,
 * returns the base of the VRAM region.
 *
 * Exported for use by the stream2v4l2 driver.
 */
uint64_t pci_epf_get_vram_fb_address(void)
{
	if (vram_fb_addr)
		return vram_fb_addr;
	return vram_phys;
}
EXPORT_SYMBOL_GPL(pci_epf_get_vram_fb_address);

/* ------------------------------------------------------------------ */
/* Driver-specific BAR allocator (ops->alloc_bar / ops->free_bar)      */
/* ------------------------------------------------------------------ */

/**
 * epf_imx9_vga_alloc_bar() - driver-specific BAR memory allocator
 *
 * BAR_2: ioremap_wc() the VRAM reserved-memory region.
 * BAR_4: ioremap() the DPU95 register space.
 * All other BARs: return NULL so the core falls back to dma_alloc_coherent.
 */
static void *epf_imx9_vga_alloc_bar(struct pci_epf *epf, enum pci_barno bar,
				size_t size, dma_addr_t *phys_addr)
{
	void __iomem *vaddr;

	if (bar == BAR_4) {
		/* DPU95 register space */
		vaddr = ioremap(DPU95_ADDRESS_SPACE, size);
		if (!vaddr) {
			dev_err(&epf->dev,
				"pci-epf-imx9-vga: ioremap DPU95 failed\n");
			return NULL;
		}
		*phys_addr = DPU95_ADDRESS_SPACE;
		dev_info(&epf->dev,
			 "pci-epf-imx9-vga: BAR4 DPU95 mapped phys=0x%08lx\n",
			 DPU95_ADDRESS_SPACE);
		return (__force void *)vaddr;
	}

	if (bar == BAR_2) {
		/* VRAM framebuffer */
		if (!vram_phys || !vram_size) {
			dev_err(&epf->dev,
				"pci-epf-imx9-vga: VRAM not initialised\n");
			return NULL;
		}
		vaddr = ioremap_wc(vram_phys, vram_size);
		if (!vaddr) {
			dev_err(&epf->dev,
				"pci-epf-imx9-vga: ioremap_wc VRAM failed\n");
			return NULL;
		}
		*phys_addr = vram_phys;
		/* Zero the framebuffer on first map */
		memset((__force void *)vaddr, 0x00, vram_size);
		dev_info(&epf->dev,
			 "pci-epf-imx9-vga: BAR2 VRAM phys=%pa size=0x%zx\n",
			 &vram_phys, vram_size);
		return (__force void *)vaddr;
	}

	/* All other BARs: return NULL so pci_epf_alloc_space() falls back
	 * to dma_alloc_coherent(epc->dev.parent) — the correct HW device.
	 * &epf->dev is a virtual bus device with no DMA mask configured.
	 */
	return NULL;
}

/**
 * epf_imx9_vga_free_bar() - driver-specific BAR memory release
 *
 * Returns true for BAR_2 (VRAM) and BAR_4 (DPU95) which were mapped with
 * ioremap* — the core will skip dma_free_coherent for these.
 * Returns false for all other BARs so the core calls
 * dma_free_coherent(epc->dev.parent) to release them.
 */
static bool epf_imx9_vga_free_bar(struct pci_epf *epf, enum pci_barno bar,
			      void *addr, size_t size)
{
	if (bar == BAR_2 || bar == BAR_4) {
		iounmap((void __iomem *)addr);
		return true;
	}
	return false;
}

/* ------------------------------------------------------------------ */
/* EDID helpers                                                         */
/* ------------------------------------------------------------------ */

static int epf_imx9_vga_read_edid_sysfs(u8 *buf)
{
	struct file *filp;
	loff_t pos = 0;
	ssize_t ret;

	filp = filp_open(EDID_PATH, O_RDONLY, 0);
	if (IS_ERR(filp))
		return PTR_ERR(filp);

	ret = kernel_read(filp, buf, EDID_SIZE, &pos);
	filp_close(filp, NULL);

	if (ret != EDID_SIZE)
		return -EINVAL;

	return 0;
}

static int epf_imx9_vga_read_kvm_config(struct kvm_config *cfg)
{
	char kvm_mode_buf[32];
	char disp_res_buf[32];
	char *kvm_mode;
	char *disp_res;

	/* Defaults */
	cfg->display = DISPLAY_1080P;
	cfg->mode    = KVM_MODE_PCIE_VIDEO_USB_HID;

	/*
	 * Copy params and strip trailing whitespace/newlines written by
	 * userspace via sysfs (e.g. echo adds a trailing '\n').
	 */
	mutex_lock(&param_lock);
	strscpy(kvm_mode_buf, param_kvm_mode, sizeof(kvm_mode_buf));
	strscpy(disp_res_buf, param_display_resolution, sizeof(disp_res_buf));
	mutex_unlock(&param_lock);

	kvm_mode = strim(kvm_mode_buf);
	disp_res = strim(disp_res_buf);

	/*
	 * Parse display_resolution module parameter.
	 */
	if (strcmp(disp_res, "4k") == 0) {
		cfg->display = DISPLAY_4K;
	} else if (strcmp(disp_res, "1080p") == 0) {
		cfg->display = DISPLAY_1080P;
	} else {
		pr_warn("pci-epf-imx9-vga: unknown display_resolution=\"%s\", "
			"defaulting to 1080p\n", disp_res);
		cfg->display = DISPLAY_1080P;
	}

	/*
	 * Parse kvm_mode module parameter.
	 */
	if (strcmp(kvm_mode, "pcie_kvm") == 0) {
		cfg->mode = KVM_MODE_PCIE_KVM;
	} else if (strcmp(kvm_mode, "pcie_video_usb_hid") == 0) {
		cfg->mode = KVM_MODE_PCIE_VIDEO_USB_HID;
	} else {
		pr_warn("pci-epf-imx9-vga: unknown kvm_mode=\"%s\", "
			"defaulting to pcie_video_usb_hid\n", kvm_mode);
		cfg->mode = KVM_MODE_PCIE_VIDEO_USB_HID;
	}

	pr_info("===== KVM Configuration =====\n");
	pr_info("Display Resolution : %s\n",
		cfg->display == DISPLAY_4K ?
		"4K (3840x2160)" : "1080p (1920x1080)");
	pr_info("KVM Mode           : %s\n",
		cfg->mode == KVM_MODE_PCIE_KVM ?
		"PCIe KVM" : "PCIe Video + USB HID");
	pr_info("=============================\n");

	return 0;
}

static bool epf_imx9_vga_is_adv7535(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "adi,adv7535");
	if (!np)
		return false;

	of_node_put(np);
	return true;
}

static bool epf_imx9_vga_is_lt9611uxc(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "lontium,lt9611uxc");
	if (!np)
		return false;

	of_node_put(np);
	return true;
}

static void epf_imx9_vga_parse_dtd(const u8 *dtd, struct edid_resolution *max)
{
	u16 hactive, vactive;

	if (dtd[0] == 0 && dtd[1] == 0)
		return;

	hactive = dtd[2] | ((dtd[4] & 0xF0) << 4);
	vactive = dtd[5] | ((dtd[7] & 0xF0) << 4);

	if ((u32)hactive * vactive > (u32)max->width * max->height) {
		max->width = hactive;
		max->height = vactive;
	}
}

static struct edid_resolution epf_imx9_vga_get_max_resolution(const u8 *edid, size_t len)
{
	struct edid_resolution max = { 0, 0 };
	int i, ext;

	if (!edid || len < 128)
		return max;

	for (i = 54; i <= 108; i += 18)
		epf_imx9_vga_parse_dtd(&edid[i], &max);

	/*
	 * Cap ext to the number of complete 128-byte extension blocks that
	 * actually fit in the buffer. edid[126] is device-provided and must
	 * not be trusted blindly (REV-007).
	 */
	ext = min_t(int, edid[126], (int)(len / 128) - 1);

	for (i = 0; i < ext; i++) {
		const u8 *blk;
		int dtd_start;
		int off;

		if (128 * (i + 2) > len)
			break;

		blk = edid + 128 * (i + 1);

		if (blk[0] != 0x02)
			continue;

		dtd_start = blk[2];

		if (dtd_start < 4 || dtd_start >= 128)
			continue;

		for (off = dtd_start; off + 18 <= 128; off += 18)
			epf_imx9_vga_parse_dtd(&blk[off], &max);
	}

	return max;
}

static void epf_imx9_vga_copy_edid_to_bar1(struct pci_epf_imx9_vga *epf_imx9_vga, struct kvm_config *cfg)
{
	u8 *bar1;
	int ret;
	u8 edid[EDID_SIZE];
	struct edid_resolution res;

	if (!epf_imx9_vga->reg[BAR_1]) {
		pr_err("pci-epf-imx9-vga: BAR1 not mapped\n");
		return;
	}

	bar1 = (u8 *)epf_imx9_vga->reg[BAR_1];

	pr_info("========== Select EDID ==========\n");
	pr_info("Configured Resolution : %s\n",
		cfg->display == DISPLAY_4K ? "4K" : "1080p");
	pr_info("Configured KVM Mode   : %s\n",
		cfg->mode == KVM_MODE_PCIE_KVM ?
		"PCIe KVM" : "PCIe Video + USB HID");

	if (epf_imx9_vga_is_adv7535())
		pr_info("Detected DT          : ADV7535\n");
	else if (epf_imx9_vga_is_lt9611uxc())
		pr_info("Detected DT          : LT9611UXC\n");
	else
		pr_info("Detected DT          : Unknown\n");

	ret = epf_imx9_vga_read_edid_sysfs(edid);
	if (ret) {
		pr_warn("pci-epf-imx9-vga: Display EDID not found\n");
		pr_info("pci-epf-imx9-vga: Using fallback %s EDID\n",
			cfg->display == DISPLAY_4K ? "4K" : "1080p");

		if (cfg->display == DISPLAY_4K)
			memcpy(edid, default_edid_4k, EDID_SIZE);
		else
			memcpy(edid, default_edid_1080p, EDID_SIZE);

		goto done;
	}

	res = epf_imx9_vga_get_max_resolution(edid, EDID_SIZE);

	pr_info("Monitor Max Resolution : %ux%u\n",
		res.width, res.height);

	if (cfg->display == DISPLAY_1080P) {
		pr_info("Requested Resolution : 1080p\n");

		if (epf_imx9_vga_is_adv7535())
			pr_info("ADV7535 supports 1080p mode\n");

		if (res.width > 1920 || res.height > 1080) {
			pr_info("Monitor supports higher resolution, advertising 1080p EDID\n");
			memcpy(edid, default_edid_1080p, EDID_SIZE);
		} else {
			pr_info("Using monitor EDID\n");
		}
	} else {
		pr_info("Requested Resolution : 4K\n");

		if (!epf_imx9_vga_is_lt9611uxc()) {
			pr_warn("LT9611UXC DT not detected, using 1080p EDID\n");
			memcpy(edid, default_edid_1080p, EDID_SIZE);
		} else if (res.width >= 3840 && res.height >= 2160) {
			pr_info("Monitor supports 4K, using monitor EDID\n");
		} else {
			pr_warn("Monitor does not support 4K, using 1080p EDID\n");
			memcpy(edid, default_edid_1080p, EDID_SIZE);
		}
	}

done:
	memcpy(bar1, edid, EDID_SIZE);
	pr_info("pci-epf-imx9-vga: EDID copied to BAR1\n");
}

/* ------------------------------------------------------------------ */
/* Option ROM                                                           */
/* ------------------------------------------------------------------ */

static int epf_imx9_vga_load_option_rom(struct pci_epf *epf, struct kvm_config *cfg)
{
	struct pci_epf_imx9_vga *epf_imx9_vga = epf_get_drvdata(epf);
	const struct firmware *fw = NULL;
	void *rom_base;
	size_t rom_size;
	int ret;

	dev_info(&epf->dev, "pci-epf-imx9-vga: Loading Option ROM\n");

	if (cfg->mode == KVM_MODE_PCIE_VIDEO_USB_HID) {
		pr_info("Option ROM     : OptionRom.rom\n");

		ret = request_firmware(&fw, "OptionRom.rom", &epf->dev);
		if (ret) {
			dev_err(&epf->dev,
				"pci-epf-imx9-vga: Failed to load Option ROM (%d)\n", ret);
			goto out;
		}

		dev_info(&epf->dev, "pci-epf-imx9-vga: Option ROM size: %zu\n", fw->size);

	} else if (cfg->mode == KVM_MODE_PCIE_KVM) {
		pr_info("Option ROM     : OptionRom_pcie_kvm.rom\n");

		ret = request_firmware(&fw, "OptionRom_pcie_kvm.rom", &epf->dev);
		if (ret) {
			dev_err(&epf->dev,
				"pci-epf-imx9-vga: Failed to load Option ROM (%d)\n", ret);
			goto out;
		}
		dev_info(&epf->dev, "pci-epf-imx9-vga: Option ROM size: %zu\n", fw->size);
	} else {
		dev_err(&epf->dev,
			"pci-epf-imx9-vga: unknown KVM mode %d, cannot select Option ROM\n",
			cfg->mode);
		ret = -EINVAL;
		goto out;
	}

	/* PCI spec: ROM BAR window >= 64KB */
	rom_size = max_t(size_t, roundup_pow_of_two(fw->size), 65536);

	memset(&epf_imx9_vga->rom_bar, 0, sizeof(epf_imx9_vga->rom_bar));
	epf_imx9_vga->rom_bar.barno = (enum pci_barno)PCI_ROM_RESOURCE;
	epf_imx9_vga->rom_bar.flags = PCI_BASE_ADDRESS_SPACE_MEMORY |
				  PCI_BASE_ADDRESS_MEM_TYPE_32;
	epf_imx9_vga->rom_bar.size = rom_size;

	rom_base = dma_alloc_coherent(epf->epc->dev.parent, rom_size,
				      &epf_imx9_vga->rom_bar.phys_addr, GFP_KERNEL);
	if (!rom_base) {
		dev_err(&epf->dev,
			"pci-epf-imx9-vga: Failed to allocate ROM memory\n");
		ret = -ENOMEM;	
		goto out;
	}

	memcpy(rom_base, fw->data, fw->size);
	epf_imx9_vga->rom_bar.addr = rom_base;

	dev_info(&epf->dev,
		 "pci-epf-imx9-vga: Option ROM virt=%p phys=%pa size=%zu\n",
		 rom_base, &epf_imx9_vga->rom_bar.phys_addr, rom_size);

	ret = pci_epc_set_bar(epf->epc, epf->func_no, epf->vfunc_no,
			      &epf_imx9_vga->rom_bar);
	if (ret) {
		dev_err(&epf->dev,
			"pci-epf-imx9-vga: Failed to configure ROM BAR (%d)\n", ret);
		dma_free_coherent(epf->epc->dev.parent, rom_size, rom_base,
				  epf_imx9_vga->rom_bar.phys_addr);
		goto out;
	}

	dev_info(&epf->dev,
		 "pci-epf-imx9-vga: ROM BAR configured (%zu bytes)\n", rom_size);

	ret = 0;
out:
	if (fw)
		release_firmware(fw);

	return ret;
}

/* ------------------------------------------------------------------ */
/* IRQ helper                                                           */
/* ------------------------------------------------------------------ */

static void epf_imx9_vga_raise_irq(struct pci_epf_imx9_vga *epf_imx9_vga,
			       struct pci_epf_imx9_vga_reg *reg)
{
	struct pci_epf *epf = epf_imx9_vga->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;
	u32 status = le32_to_cpu(reg->status);
	u32 irq_number = le32_to_cpu(reg->irq_number);
	u32 irq_type = le32_to_cpu(reg->irq_type);
	int count;

	status |= STATUS_IRQ_RAISED;
	WRITE_ONCE(reg->status, cpu_to_le32(status));

	switch (irq_type) {
	case IRQ_TYPE_INTX:
		pci_epc_raise_irq(epc, epf->func_no, epf->vfunc_no,
				  PCI_IRQ_INTX, 0);
		break;
	case IRQ_TYPE_MSI:
		count = pci_epc_get_msi(epc, epf->func_no, epf->vfunc_no);
		if (irq_number > count || count <= 0) {
			dev_err(dev, "Invalid MSI IRQ number %d / %d\n",
				irq_number, count);
			return;
		}
		pci_epc_raise_irq(epc, epf->func_no, epf->vfunc_no,
				  PCI_IRQ_MSI, irq_number);
		break;
	case IRQ_TYPE_MSIX:
		count = pci_epc_get_msix(epc, epf->func_no, epf->vfunc_no);
		if (irq_number > count || count <= 0) {
			dev_err(dev, "Invalid MSI-X IRQ number %d / %d\n",
				irq_number, count);
			return;
		}
		pci_epc_raise_irq(epc, epf->func_no, epf->vfunc_no,
				  PCI_IRQ_MSIX, irq_number);
		break;
	default:
		dev_err(dev, "pci-epf-imx9-vga: unknown IRQ type %u\n", irq_type);
		break;
	}
}

/* ------------------------------------------------------------------ */
/* Command handler (delayed work)                                       */
/* ------------------------------------------------------------------ */

static void epf_imx9_vga_cmd_handler(struct work_struct *work)
{
	struct pci_epf_imx9_vga *epf_imx9_vga =
		container_of(work, struct pci_epf_imx9_vga, cmd_handler.work);
	struct pci_epf *epf = epf_imx9_vga->epf;
	struct device *dev = &epf->dev;
	enum pci_barno ctrl_bar = epf_imx9_vga->ctrl_reg_bar;
	struct pci_epf_imx9_vga_reg *reg = epf_imx9_vga->reg[ctrl_bar];
	u32 command;

	command = le32_to_cpu(READ_ONCE(reg->command));
	if (!command)
		goto reschedule;

	WRITE_ONCE(reg->command, 0);
	WRITE_ONCE(reg->status, 0);

	switch (command) {
	case COMMAND_RAISE_INTX_IRQ:
	case COMMAND_RAISE_MSI_IRQ:
	case COMMAND_RAISE_MSIX_IRQ:
		epf_imx9_vga_raise_irq(epf_imx9_vga, reg);
		break;

	case COMMAND_CHANGE_RESOLUTION:
	{
		char *envp[3];
		char res_str[32];
		u32 width  = le32_to_cpu(reg->width);
		u32 height = le32_to_cpu(reg->height);

		dev_info(dev, "pci-epf-imx9-vga: Resolution change: %ux%u\n",
			 width, height);

		snprintf(res_str, sizeof(res_str), "RESOLUTION=%ux%u",
			 width, height);
		envp[0] = "EVENT=RESOLUTION_CHANGE";
		envp[1] = res_str;
		envp[2] = NULL;
		kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);
		epf_imx9_vga_raise_irq(epf_imx9_vga, reg);
		break;
	}

	case COMMAND_GET_FB_OFFSET:
		reg->dpu_fb_offset = cpu_to_le64(vram_fb_offset);
		epf_imx9_vga_raise_irq(epf_imx9_vga, reg);
		break;

	case COMMAND_START_STREAM:
		dev_info(dev, "pci-epf-imx9-vga: START_STREAM\n");
		epf_imx9_vga_raise_irq(epf_imx9_vga, reg);
		break;

	case COMMAND_STOP_STREAM:
		dev_info(dev, "pci-epf-imx9-vga: STOP_STREAM\n");
		epf_imx9_vga_raise_irq(epf_imx9_vga, reg);
		break;

	default:
		dev_err(dev, "pci-epf-imx9-vga: unknown command 0x%x\n", command);
		break;
	}

reschedule:
	queue_delayed_work(kpciimx9vga_workqueue, &epf_imx9_vga->cmd_handler,
			   msecs_to_jiffies(TIMER_RESOLUTION));
}

/* ------------------------------------------------------------------ */
/* BAR set / clear                                                      */
/* ------------------------------------------------------------------ */

static int epf_imx9_vga_set_bar(struct pci_epf *epf)
{
	struct pci_epf_imx9_vga *epf_imx9_vga = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	int bar, ret;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		if (!epf_imx9_vga->reg[bar])
			continue;

		/* Mark BAR2 (VRAM) as prefetchable memory */
		if (bar == BAR_2)
			epf->bar[bar].flags |=
				PCI_BASE_ADDRESS_MEM_PREFETCH |
				PCI_BASE_ADDRESS_SPACE_MEMORY;

		ret = pci_epc_set_bar(epc, epf->func_no, epf->vfunc_no,
				      &epf->bar[bar]);
		if (ret) {
			pci_epf_free_space(epf, epf_imx9_vga->reg[bar], bar,
					   PRIMARY_INTERFACE);
			epf_imx9_vga->reg[bar] = NULL;
			dev_err(dev, "pci-epf-imx9-vga: Failed to set BAR%d\n", bar);
			if (bar == epf_imx9_vga->ctrl_reg_bar)
				return ret;
		}
	}
	return 0;
}

static void epf_imx9_vga_clear_bar(struct pci_epf *epf)
{
	struct pci_epf_imx9_vga *epf_imx9_vga = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;
	int bar;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		if (!epf_imx9_vga->reg[bar])
			continue;
		pci_epc_clear_bar(epc, epf->func_no, epf->vfunc_no,
				  &epf->bar[bar]);
	}
}

/* ------------------------------------------------------------------ */
/* Capabilities                                                         */
/* ------------------------------------------------------------------ */

static void epf_imx9_vga_set_capabilities(struct pci_epf *epf)
{
	struct pci_epf_imx9_vga *epf_imx9_vga = epf_get_drvdata(epf);
	struct pci_epf_imx9_vga_reg *reg = epf_imx9_vga->reg[epf_imx9_vga->ctrl_reg_bar];
	struct pci_epc *epc = epf->epc;
	u32 caps = 0;
	if (epc->ops->align_addr)
		caps |= CAP_UNALIGNED_ACCESS;
	if (epf_imx9_vga->epc_features->msi_capable)
		caps |= CAP_MSI;
	if (epf_imx9_vga->epc_features->msix_capable)
		caps |= CAP_MSIX;
	if (epf_imx9_vga->epc_features->intx_capable)
		caps |= CAP_INTX;

	reg->caps = cpu_to_le32(caps);
}

/* ------------------------------------------------------------------ */
/* EPC event callbacks                                                  */
/* ------------------------------------------------------------------ */

static int epf_imx9_vga_epc_init(struct pci_epf *epf)
{
	struct pci_epf_imx9_vga *epf_imx9_vga = epf_get_drvdata(epf);
	struct pci_epf_header *header = epf->header;
	const struct pci_epc_features *epc_features = epf_imx9_vga->epc_features;
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	char *envp_uevent[] = { "KVM_EVENT=kvm_ready", NULL};
	int ret;

	if (epf->vfunc_no <= 1) {
		ret = pci_epc_write_header(epc, epf->func_no, epf->vfunc_no,
					   header);
		if (ret) {
			dev_err(dev,
				"pci-epf-imx9-vga: config header write failed\n");
			return ret;
		}
	}

	epf_imx9_vga_set_capabilities(epf);

	ret = epf_imx9_vga_set_bar(epf);
	if (ret)
		return ret;

	if (epc_features->msi_capable) {
		ret = pci_epc_set_msi(epc, epf->func_no, epf->vfunc_no,
				      epf->msi_interrupts);
		if (ret) {
			dev_err(dev, "pci-epf-imx9-vga: MSI config failed\n");
			return ret;
		}
	}

	if (epc_features->msix_capable) {
		ret = pci_epc_set_msix(epc, epf->func_no, epf->vfunc_no,
				       epf->msix_interrupts,
				       epf_imx9_vga->ctrl_reg_bar,
				       epf_imx9_vga->msix_table_offset);
		if (ret) {
			dev_err(dev, "pci-epf-imx9-vga: MSI-X config failed\n");
			return ret;
		}
	}

	ret = epf_imx9_vga_read_kvm_config(&epf_imx9_vga->cfg);
	if (ret)
		dev_warn(dev, "pci-epf-imx9-vga: kvm config file not found (%d)\n", ret);

	ret = epf_imx9_vga_load_option_rom(epf, &epf_imx9_vga->cfg);
	if (ret)
		dev_warn(dev, "pci-epf-imx9-vga: Option ROM not loaded (%d)\n", ret);

	/* Populate EDID into BAR1 */
	epf_imx9_vga_copy_edid_to_bar1(epf_imx9_vga, &epf_imx9_vga->cfg);

#if IS_ENABLED(CONFIG_PCIE_KVM_INPUT)
	if (epf_imx9_vga->cfg.mode == KVM_MODE_PCIE_KVM) {
		if (epf_imx9_vga->reg[BAR_3]) {
			WRITE_ONCE(hid_addr, epf_imx9_vga->reg[BAR_3]);
			pr_info("HID BAR3 address=%p\n", hid_addr);
			atomic_set(&epf_imx9_vga->hid_attached, 0);
			ret = epf_hid_attach(hid_addr);
			if (ret) {
				dev_err(&epf->dev,
					"pci-epf-imx9-vga: epf_hid_attach failed (%d)\n",
					ret);
				return ret;
			}
			atomic_set(&epf_imx9_vga->hid_attached, 1);
		} else {
			dev_err(&epf->dev, "BAR3 not allocated\n");
			return -EINVAL;
		}
	}
#endif

	/* Notify userspace that the PCIe endpoint is initialized.
		* Userspace (udev / systemd) is responsible for restarting
		* obmc-ikvm.service in response to this event.
		*/

	kobject_uevent_env(&epf->dev.kobj, KOBJ_CHANGE, envp_uevent);

	dev_info(dev, "pci-epf-imx9-vga: sent kvm_ready uevent\n");

	if (!epc_features->linkup_notifier)
		queue_work(kpciimx9vga_workqueue, &epf_imx9_vga->cmd_handler.work);

	return 0;
}

static void epf_imx9_vga_epc_deinit(struct pci_epf *epf)
{
	struct pci_epf_imx9_vga *epf_imx9_vga = epf_get_drvdata(epf);

	cancel_delayed_work_sync(&epf_imx9_vga->cmd_handler);
#if IS_ENABLED(CONFIG_PCIE_KVM_INPUT)
	if (epf_imx9_vga->cfg.mode == KVM_MODE_PCIE_KVM) {
		if (atomic_cmpxchg(&epf_imx9_vga->hid_attached, 1, 0) == 1) {
			epf_hid_detach();
			WRITE_ONCE(hid_addr, NULL);
		}
	}
#endif
	epf_imx9_vga_clear_bar(epf);
}

static int epf_imx9_vga_link_up(struct pci_epf *epf)
{
	struct pci_epf_imx9_vga *epf_imx9_vga = epf_get_drvdata(epf);

	queue_delayed_work(kpciimx9vga_workqueue, &epf_imx9_vga->cmd_handler,
			   msecs_to_jiffies(TIMER_RESOLUTION));
	return 0;
}

static int epf_imx9_vga_link_down(struct pci_epf *epf)
{
	struct pci_epf_imx9_vga *epf_imx9_vga = epf_get_drvdata(epf);

	cancel_delayed_work_sync(&epf_imx9_vga->cmd_handler);
	return 0;
}

static const struct pci_epc_event_ops pci_epf_imx9_vga_event_ops = {
	.epc_init   = epf_imx9_vga_epc_init,
	.epc_deinit = epf_imx9_vga_epc_deinit,
	.link_up    = epf_imx9_vga_link_up,
	.link_down  = epf_imx9_vga_link_down,
};

/* ------------------------------------------------------------------ */
/* Space allocation / deallocation                                      */
/* ------------------------------------------------------------------ */

static int epf_imx9_vga_alloc_space(struct pci_epf *epf)
{
	struct pci_epf_imx9_vga *epf_imx9_vga = epf_get_drvdata(epf);
	struct device *dev = &epf->dev;
	size_t msix_table_size = 0;
	size_t ctrl_reg_bar_size;
	size_t pba_size = 0;
	void *base;
	enum pci_barno ctrl_bar = epf_imx9_vga->ctrl_reg_bar;
	enum pci_barno bar;
	const struct pci_epc_features *epc_features = epf_imx9_vga->epc_features;
	size_t alloc_size;

	ctrl_reg_bar_size = ALIGN(sizeof(struct pci_epf_imx9_vga_reg), 128);

	if (epc_features->msix_capable) {
		msix_table_size = PCI_MSIX_ENTRY_SIZE * epf->msix_interrupts;
		epf_imx9_vga->msix_table_offset = ctrl_reg_bar_size;
		pba_size = ALIGN(DIV_ROUND_UP(epf->msix_interrupts, 8), 8);
	}
	alloc_size = ctrl_reg_bar_size + msix_table_size + pba_size;

	base = pci_epf_alloc_space(epf, alloc_size, ctrl_bar,
				   epc_features, PRIMARY_INTERFACE);
	if (!base) {
		dev_err(dev, "pci-epf-imx9-vga: Failed to allocate ctrl reg space\n");
		return -ENOMEM;
	}
	epf_imx9_vga->reg[ctrl_bar] = base;

	for (bar = BAR_0; bar < PCI_STD_NUM_BARS; bar++) {
		bar = pci_epc_get_next_free_bar(epc_features, bar);
		if (bar == NO_BAR)
			break;
		if (bar == ctrl_bar)
			continue;

		if (epc_features->bar[bar].type == BAR_FIXED)
			alloc_size = epc_features->bar[bar].fixed_size;
		else
			alloc_size = imx9_vga_bar_size[bar];

		base = pci_epf_alloc_space(epf, alloc_size, bar,
					   epc_features, PRIMARY_INTERFACE);
		if (!base)
			dev_err(dev,
				"pci-epf-imx9-vga: Failed to alloc space for BAR%d\n",
				bar);
		epf_imx9_vga->reg[bar] = base;

		if (bar == BAR_2 && base && vram_size)
			epf->bar[BAR_2].size = vram_size;
	}

	return 0;
}

static void epf_imx9_vga_free_space(struct pci_epf *epf)
{
	struct pci_epf_imx9_vga *epf_imx9_vga = epf_get_drvdata(epf);
	int bar;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		if (!epf_imx9_vga->reg[bar])
			continue;
		pci_epf_free_space(epf, epf_imx9_vga->reg[bar], bar,
				   PRIMARY_INTERFACE);
		epf_imx9_vga->reg[bar] = NULL;
	}
}

/* ------------------------------------------------------------------ */
/* bind / unbind                                                        */
/* ------------------------------------------------------------------ */

static int epf_imx9_vga_bind(struct pci_epf *epf)
{
	struct pci_epf_imx9_vga *epf_imx9_vga = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;
	enum pci_barno ctrl_bar = BAR_0;
	struct pci_epc *epc = epf->epc;
	int ret;

	if (WARN_ON_ONCE(!epc))
		return -EINVAL;

	epc_features = pci_epc_get_features(epc, epf->func_no, epf->vfunc_no);
	if (!epc_features) {
		dev_err(&epf->dev, "pci-epf-imx9-vga: epc_features not implemented\n");
		return -EOPNOTSUPP;
	}

	ctrl_bar = pci_epc_get_first_free_bar(epc_features);
	if (ctrl_bar < 0)
		return -EINVAL;

	epf_imx9_vga->ctrl_reg_bar = ctrl_bar;
	epf_imx9_vga->epc_features = epc_features;

	ret = epf_imx9_vga_alloc_space(epf);
	if (ret)
		return ret;

	return 0;
}

static void epf_imx9_vga_unbind(struct pci_epf *epf)
{
	struct pci_epf_imx9_vga *epf_imx9_vga = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;

	cancel_delayed_work_sync(&epf_imx9_vga->cmd_handler);
#if IS_ENABLED(CONFIG_PCIE_KVM_INPUT)
	if (epf_imx9_vga->cfg.mode == KVM_MODE_PCIE_KVM) {
		if (atomic_cmpxchg(&epf_imx9_vga->hid_attached, 1, 0) == 1) {
			epf_hid_detach();
			WRITE_ONCE(hid_addr, NULL);
		}
	}
#endif
	if (epc->init_complete) {
		epf_imx9_vga_clear_bar(epf);
	}
	epf_imx9_vga_free_space(epf);
}

/* ------------------------------------------------------------------ */
/* EPF driver ops                                                       */
/* ------------------------------------------------------------------ */

static const struct pci_epf_ops epf_imx9_vga_ops = {
	.bind      = epf_imx9_vga_bind,
	.unbind    = epf_imx9_vga_unbind,
	.alloc_bar = epf_imx9_vga_alloc_bar,
	.free_bar  = epf_imx9_vga_free_bar,
};

static const struct pci_epf_device_id pci_epf_imx9_vga_ids[] = {
	{ .name = "pci_epf_imx9_vga" },
	{},
};

static int pci_epf_imx9_vga_probe(struct pci_epf *epf,
			     const struct pci_epf_device_id *id)
{
	struct pci_epf_imx9_vga *epf_imx9_vga;
	struct device *dev = &epf->dev;

	epf_imx9_vga = devm_kzalloc(dev, sizeof(*epf_imx9_vga), GFP_KERNEL);
	if (!epf_imx9_vga)
		return -ENOMEM;

	epf->header    = &imx9_vga_header;
	epf_imx9_vga->epf   = epf;

	INIT_DELAYED_WORK(&epf_imx9_vga->cmd_handler, epf_imx9_vga_cmd_handler);
	epf->event_ops = &pci_epf_imx9_vga_event_ops;

	epf_set_drvdata(epf, epf_imx9_vga);
	return 0;
}

static struct pci_epf_driver pci_epf_imx9_vga_driver = {
	.driver.name = "pci_epf_imx9_vga",
	.probe       = pci_epf_imx9_vga_probe,
	.id_table    = pci_epf_imx9_vga_ids,
	.ops         = &epf_imx9_vga_ops,
	.owner       = THIS_MODULE,
};

/* ------------------------------------------------------------------ */
/* Module parameter: VRAM framebuffer offset                            */
/* ------------------------------------------------------------------ */

static int param_set_imx9_vga_offset(const char *val, const struct kernel_param *kp)
{
	unsigned long flags;
	u64 offset;
	int ret;

	ret = kstrtou64(val, 0, &offset);
	if (ret)
		return ret;

	spin_lock_irqsave(&imx9_vga_offset_lock, flags);
	vram_fb_addr   = offset;
	vram_fb_offset = offset ? offset - vram_phys : 0;
	pr_info("pci-epf-imx9-vga: VRAM fb addr=0x%016llx offset=0x%016llx\n",
		(unsigned long long)vram_fb_addr,
		(unsigned long long)vram_fb_offset);
	spin_unlock_irqrestore(&imx9_vga_offset_lock, flags);

	return 0;
}

static int param_get_imx9_vga_offset(char *buffer, const struct kernel_param *kp)
{
	return sprintf(buffer, "%llu\n",
		       (unsigned long long)vram_fb_offset);
}

static const struct kernel_param_ops vga_offset_ops = {
	.set = param_set_imx9_vga_offset,
	.get = param_get_imx9_vga_offset,
};

module_param_cb(offset, &vga_offset_ops, NULL, 0644);
MODULE_PARM_DESC(offset, "VRAM framebuffer physical address");

static int param_set_kvm_mode(const char *val, const struct kernel_param *kp)
{
	int ret;

	mutex_lock(&param_lock);
	ret = param_set_copystring(val, kp);
	mutex_unlock(&param_lock);
	return ret;
}

static int param_get_kvm_mode(char *buf, const struct kernel_param *kp)
{
	int ret;

	mutex_lock(&param_lock);
	ret = param_get_string(buf, kp);
	mutex_unlock(&param_lock);
	return ret;
}

static const struct kernel_param_ops kvm_mode_ops = {
	.set = param_set_kvm_mode,
	.get = param_get_kvm_mode,
};

/* kvm_mode */
static struct kparam_string kvm_mode_kps = {
	.maxlen = sizeof(param_kvm_mode),
	.string = param_kvm_mode,
};
module_param_cb(kvm_mode, &kvm_mode_ops, &kvm_mode_kps, 0644);
MODULE_PARM_DESC(kvm_mode,"KVM operating mode: pcie_kvm or pcie_video_usb_hid (default)");
	
static int param_set_display_resolution(const char *val, const struct kernel_param *kp)
{
	int ret;

	mutex_lock(&param_lock);
	ret = param_set_copystring(val, kp);
	mutex_unlock(&param_lock);
	return ret;
}

static int param_get_display_resolution(char *buf, const struct kernel_param *kp)
{
	int ret;

	mutex_lock(&param_lock);
	ret = param_get_string(buf, kp);
	mutex_unlock(&param_lock);
	return ret;
}

static const struct kernel_param_ops display_resolution_ops = {
	.set = param_set_display_resolution,
	.get = param_get_display_resolution,
};

/* display_resolution */
static struct kparam_string display_resolution_kps = {
	.maxlen = sizeof(param_display_resolution),
	.string = param_display_resolution,
};

module_param_cb(display_resolution, &display_resolution_ops, &display_resolution_kps, 0644);
MODULE_PARM_DESC(display_resolution,"Display resolution: 1080p (default) or 4k");

/* ------------------------------------------------------------------ */
/* PCI fixup: VGA class quirk                                           */
/* ------------------------------------------------------------------ */

static void imx_pcie_vga_quirk(struct pci_dev *dev)
{
	/*
	 * Placeholder quirk for i.MX95 VGA class EP device.
	 * The cfg_size limiting (dbi_length) is i.MX6-specific and uses
	 * controller-private symbols (struct imx_pcie, to_imx_pcie,
	 * to_dw_pcie_from_pp) that are not accessible from an EPF driver.
	 * On i.MX95, dbi_length = 0 so this would be a no-op regardless.
	 */
}
DECLARE_PCI_FIXUP_CLASS_HEADER(PCI_VENDOR_ID_IMX9_VGA, PCI_DEVICE_ID_IMX9_VGA,
			       PCI_CLASS_DISPLAY_VGA, 8, imx_pcie_vga_quirk);

/* ------------------------------------------------------------------ */
/* Module init / exit                                                   */
/* ------------------------------------------------------------------ */

static int __init pci_epf_imx9_vga_init(void)
{
	int ret;

	ret = epf_imx9_vga_init_vram();
	if (ret)
		return ret;

	imx9_vga_bar_size[2] = vram_size;

	kpciimx9vga_workqueue = alloc_workqueue("kpcivga",
					    WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
	if (!kpciimx9vga_workqueue) {
		pr_err("pci-epf-imx9-vga: Failed to allocate workqueue\n");
		return -ENOMEM;
	}

	ret = pci_epf_register_driver(&pci_epf_imx9_vga_driver);
	if (ret) {
		destroy_workqueue(kpciimx9vga_workqueue);
		pr_err("pci-epf-imx9-vga: Failed to register driver: %d\n", ret);
		return ret;
	}

	return 0;
}
module_init(pci_epf_imx9_vga_init);

static void __exit pci_epf_imx9_vga_exit(void)
{
	pci_epf_unregister_driver(&pci_epf_imx9_vga_driver);
	if (kpciimx9vga_workqueue)
		destroy_workqueue(kpciimx9vga_workqueue);
}
module_exit(pci_epf_imx9_vga_exit);

MODULE_DESCRIPTION("PCIe EP VGA Class Display Driver for i.MX95 iKVM");
MODULE_AUTHOR("Nikhil Gupta <nikhil.gupta_2@nxp.com>");
MODULE_AUTHOR("Shreyash Damanwala <shreyash.damanwala@nxp.com>");
MODULE_LICENSE("GPL v2");

