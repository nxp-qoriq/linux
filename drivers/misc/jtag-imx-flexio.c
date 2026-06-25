// SPDX-License-Identifier: GPL-2.0-only
/*
 * NXP i.MX FlexIO JTAG master driver
 *
 * Implements a JTAG master on the i.MX95 latu-dcsm board using the
 * FlexIO1 IP block.  The hardware mapping is:
 *
 *   FlexIO1 BIT24  GPIO_IO24  HPM_JTAG_TCK   (Timer 0 clock output)
 *   FlexIO1 BIT25  GPIO_IO25  HPM_JTAG_TDI   (Shifter 0 TX output)
 *   FlexIO1 BIT27  GPIO_IO27  HPM_JTAG_TDO   (Shifter 1 RX input)
 *   FlexIO1 BIT28  GPIO_IO28  HPM_JTAG_TMS   (Shifter 2 TX output)
 *   GPIO2_IO29     GPIO_IO29  HPM_JTAG_TRST_N (GPIO, active-low)
 *
 * FlexIO resource allocation:
 *   Timer   0 - TCK baud generator (DUAL_8BIT_COUNTERS_BAUD)
 *   Shifter 0 - TDI transmit (32-bit, MSB first)
 *   Shifter 1 - TDO receive  (32-bit, MSB first)
 *   Shifter 2 - TMS transmit (32-bit, MSB first, parallel with TDI)
 *
 * Minimum supported TCK: ~46.875 kHz (24 MHz FlexIO clock / 512)
 *
 * Copyright 2026 NXP
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/miscdevice.h>
#include <linux/mfd/imx-flexio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>

/* -----------------------------------------------------------------------
 * UAPI definitions (no upstream jtag.h in this kernel tree yet)
 * ----------------------------------------------------------------------- */

#define JTAG_MAGIC      'J'

/* TAP states (IEEE 1149.1) */
enum jtag_tap_state {
	JTAG_STATE_TLRESET   = 0,
	JTAG_STATE_IDLE      = 1,
	JTAG_STATE_SELECTDR  = 2,
	JTAG_STATE_CAPTUREDR = 3,
	JTAG_STATE_SHIFTDR   = 4,
	JTAG_STATE_EXIT1DR   = 5,
	JTAG_STATE_PAUSEDR   = 6,
	JTAG_STATE_EXIT2DR   = 7,
	JTAG_STATE_UPDATEDR  = 8,
	JTAG_STATE_SELECTIR  = 9,
	JTAG_STATE_CAPTUREIR = 10,
	JTAG_STATE_SHIFTIR   = 11,
	JTAG_STATE_EXIT1IR   = 12,
	JTAG_STATE_PAUSEIR   = 13,
	JTAG_STATE_EXIT2IR   = 14,
	JTAG_STATE_UPDATEIR  = 15,
};

enum jtag_xfer_type {
	JTAG_SIR_XFER = 0, /* instruction register */
	JTAG_SDR_XFER = 1, /* data register */
};

enum jtag_xfer_direction {
	JTAG_READ_XFER  = 1,
	JTAG_WRITE_XFER = 2,
	JTAG_READ_WRITE_XFER = 3,
};

struct jtag_tap_state_req {
	__u8  from;
	__u8  endstate;
	__u8  reset;    /* 1 = assert TRST, 0 = TMS walk */
	__u8  padding;
};

struct jtag_xfer {
	__u8   type;       /* JTAG_SIR_XFER / JTAG_SDR_XFER */
	__u8   direction;
	__u8   endstate;
	__u8   padding;
	__u32  length;     /* number of bits */
	__u64  tdio;       /* user pointer to TDI buffer; TDO written here */
};

#define JTAG_SIOCSTATE  _IOW(JTAG_MAGIC, 1, struct jtag_tap_state_req)
#define JTAG_GIOCSTATE  _IOR(JTAG_MAGIC, 2, __u32)
#define JTAG_SIOCFREQ   _IOW(JTAG_MAGIC, 3, __u32)
#define JTAG_GIOCFREQ   _IOR(JTAG_MAGIC, 4, __u32)
#define JTAG_IOCXFER    _IOWR(JTAG_MAGIC, 5, struct jtag_xfer)
#define JTAG_GIOCSTATUS _IOR(JTAG_MAGIC, 6, enum jtag_tap_state)

/* -----------------------------------------------------------------------
 * FlexIO register offsets for shifters/timers beyond what imx-flexio.h
 * covers (imx-flexio.h only defines _0 and _1 variants).
 * -----------------------------------------------------------------------
 * Layout: SHIFTCTL[n] = 0x80 + n*4,  SHIFTCFG[n] = 0x100 + n*4
 *         SHIFTBUF[n] = 0x200 + n*4, SHIFTBUFBIS[n] = 0x280 + n*4
 *         TIMCTL[n]   = 0x400 + n*4, TIMCFG[n]   = 0x480 + n*4
 *         TIMCMP[n]   = 0x500 + n*4
 */
#define FLEXIO_SHIFTCTL(n)    (0x80  + (n) * 4)
#define FLEXIO_SHIFTCFG(n)    (0x100 + (n) * 4)
#define FLEXIO_SHIFTBUF(n)    (0x200 + (n) * 4)
#define FLEXIO_SHIFTBUFBIS(n) (0x280 + (n) * 4) /* bit-inverted/swapped (LSB first) */
#define FLEXIO_TIMCTL(n)      (0x400 + (n) * 4)
#define FLEXIO_TIMCFG(n)      (0x480 + (n) * 4)
#define FLEXIO_TIMCMP(n)      (0x500 + (n) * 4)

/* TIMCMP encoding for DUAL_8BIT_COUNTERS_BAUD:
 *   [15:8] = (bits_per_word * 2) - 1
 *   [7:0]  = (flexio_clk / (2 * target_hz)) - 1
 */
#define FLEXIO_TIMCMP_BAUD(bits, div) (((u32)((bits) * 2 - 1) << 8) | ((div) - 1))

/* Shifter modes */
#define SMOD_DISABLE  0
#define SMOD_RECEIVE  1
#define SMOD_TRANSMIT 2

/* Pin config for shifter/timer pins */
#define PINCFG_DISABLED    0
#define PINCFG_OPEN_DRAIN  1
#define PINCFG_BIDIR       2
#define PINCFG_OUTPUT      3

/* Timer modes */
#define TIMOD_DISABLE         0
#define TIMOD_DUAL8_BAUD      1
#define TIMOD_DUAL8_PWM       2
#define TIMOD_SINGLE16        3

/* TIMENA: timer enable conditions */
#define TIMENA_ALWAYS     0
#define TIMENA_PREV_TIMER 1
#define TIMENA_TRG_RISE   3

/* TIMDIS: timer disable conditions */
#define TIMDIS_NEVER      0
#define TIMDIS_PREV_TIMER 1
#define TIMDIS_COMPARE    2
#define TIMDIS_COMPARE_TRG 3

/* TIMRST: timer reset conditions */
#define TIMRST_PIN_EQUAL 2

/* TIMOUT: initial output value */
#define TIMOUT_ONE       0
#define TIMOUT_ZERO      1

/* Trigger select: internal trigger from shifter N status flag */
#define TRGSEL_SHIFTER(n) (((n) << 2) | 0x1)

/* -----------------------------------------------------------------------
 * Driver private data
 * ----------------------------------------------------------------------- */

#define FLEXIO_JTAG_TIMER_TCK    0
#define FLEXIO_JTAG_SHIFTER_TDI  0
#define FLEXIO_JTAG_SHIFTER_TDO  1
#define FLEXIO_JTAG_SHIFTER_TMS  2
#define FLEXIO_BITS_PER_WORD     32

/* FlexIO clock is IMX95_CLK_BUSWAKEUP = 24 MHz */
#define FLEXIO_CLK_HZ        24000000U
#define FLEXIO_TCK_MIN_HZ    46875U    /* 24 MHz / 512 */
#define FLEXIO_TCK_DEFAULT_HZ FLEXIO_TCK_MIN_HZ

struct flexio_jtag {
	struct miscdevice  miscdev;
	struct device      *dev;
	void __iomem       *base;
	struct clk         *per_clk;
	struct clk         *ipg_clk;
	struct gpio_desc   *trst_gpiod;
	int                irq;
	u32                tck_hz;
	enum jtag_tap_state tap_state;
	struct mutex       lock;
	struct completion  xfer_done;
	/* shadow TDO buffer filled by ISR */
	u32                tdo_shadow;
};

/* -----------------------------------------------------------------------
 * FlexIO hardware helpers
 * ----------------------------------------------------------------------- */

static void fjtag_writel(struct flexio_jtag *fj, u32 val, u32 reg)
{
	writel_relaxed(val, fj->base + reg);
}

static u32 fjtag_readl(struct flexio_jtag *fj, u32 reg)
{
	return readl_relaxed(fj->base + reg);
}

/* Compute baud divider (rounds down, minimum 1) */
static u32 fjtag_baud_div(u32 tck_hz)
{
	u32 div = DIV_ROUND_UP(FLEXIO_CLK_HZ, 2 * tck_hz);

	if (div < 1)
		div = 1;
	if (div > 256)
		div = 256;
	return div;
}

static void fjtag_set_tck_hz(struct flexio_jtag *fj, u32 tck_hz)
{
	u32 div, timcmp;

	if (tck_hz < FLEXIO_TCK_MIN_HZ)
		tck_hz = FLEXIO_TCK_MIN_HZ;

	div = fjtag_baud_div(tck_hz);
	fj->tck_hz = FLEXIO_CLK_HZ / (2 * div);

	/* Program 32 bits per word with the computed baud divider.
	 * Timer will be reprogrammed per-transfer with correct bit count. */
	timcmp = FLEXIO_TIMCMP_BAUD(FLEXIO_BITS_PER_WORD, div);
	fjtag_writel(fj, timcmp, FLEXIO_TIMCMP(FLEXIO_JTAG_TIMER_TCK));
}

/* Configure FlexIO1 for JTAG operation */
static void fjtag_hw_init(struct flexio_jtag *fj)
{
	u32 val;

	/* Software reset FlexIO */
	val = fjtag_readl(fj, CTRL);
	val |= FLEXIO_CTRL_SWRST_MASK;
	fjtag_writel(fj, val, CTRL);
	fjtag_writel(fj, 0, CTRL);

	/* Enable FlexIO, enable in debug mode */
	val = FLEXIO_CTRL_FLEXEN(1) | FLEXIO_CTRL_DBGE(1);
	fjtag_writel(fj, val, CTRL);

	/* --- Shifter 0: TDI TX ---
	 * Shift MSB first on TCK rising edge, output on BIT25 */
	val = SHIFTCTL_TIMSEL(FLEXIO_JTAG_TIMER_TCK) |
	      SHIFTCTL_TIMPOL(0)  |  /* shift on posedge (default) */
	      SHIFTCTL_PINCFG(PINCFG_OUTPUT) |
	      SHIFTCTL_PINSEL(25) |
	      SHIFTCTL_PINPOL(0)  |
	      SHIFTCTL_SMOD(SMOD_TRANSMIT);
	fjtag_writel(fj, val, FLEXIO_SHIFTCTL(FLEXIO_JTAG_SHIFTER_TDI));
	fjtag_writel(fj, 0, FLEXIO_SHIFTCFG(FLEXIO_JTAG_SHIFTER_TDI));

	/* --- Shifter 1: TDO RX ---
	 * Sample on TCK falling edge (TIMPOL=1 = negedge), input on BIT27 */
	val = SHIFTCTL_TIMSEL(FLEXIO_JTAG_TIMER_TCK) |
	      SHIFTCTL_TIMPOL(1)  |  /* sample on negedge */
	      SHIFTCTL_PINCFG(PINCFG_DISABLED) |
	      SHIFTCTL_PINSEL(27) |
	      SHIFTCTL_PINPOL(0)  |
	      SHIFTCTL_SMOD(SMOD_RECEIVE);
	fjtag_writel(fj, val, FLEXIO_SHIFTCTL(FLEXIO_JTAG_SHIFTER_TDO));
	fjtag_writel(fj, 0, FLEXIO_SHIFTCFG(FLEXIO_JTAG_SHIFTER_TDO));

	/* --- Shifter 2: TMS TX ---
	 * Parallel to TDI, same timer, output on BIT28 */
	val = SHIFTCTL_TIMSEL(FLEXIO_JTAG_TIMER_TCK) |
	      SHIFTCTL_TIMPOL(0)  |
	      SHIFTCTL_PINCFG(PINCFG_OUTPUT) |
	      SHIFTCTL_PINSEL(28) |
	      SHIFTCTL_PINPOL(0)  |
	      SHIFTCTL_SMOD(SMOD_TRANSMIT);
	fjtag_writel(fj, val, FLEXIO_SHIFTCTL(FLEXIO_JTAG_SHIFTER_TMS));
	fjtag_writel(fj, 0, FLEXIO_SHIFTCFG(FLEXIO_JTAG_SHIFTER_TMS));

	/* --- Timer 0: TCK ---
	 * DUAL_8BIT_COUNTERS_BAUD mode, output on BIT24.
	 * Internal trigger from Shifter 0 transmit flag (starts on write). */
	val = TIMCTL_TRGSEL(TRGSEL_SHIFTER(FLEXIO_JTAG_SHIFTER_TDI)) |
	      TIMCTL_TRGPOL(1)    |  /* trigger active low */
	      TIMCTL_TRGSRC(1)    |  /* internal trigger */
	      TIMCTL_PINCFG(PINCFG_OUTPUT) |
	      TIMCTL_PINSEL(24)   |
	      TIMCTL_PINPOL(0)    |
	      TIMCTL_TIMOD(TIMOD_DUAL8_BAUD);
	fjtag_writel(fj, val, FLEXIO_TIMCTL(FLEXIO_JTAG_TIMER_TCK));

	val = TIMCFG_TIMOUT(TIMOUT_ONE)    |  /* output high when not active */
	      TIMCFG_TIMDEC(TIMDEC_FLEXIO_CLK) |
	      TIMCFG_TIMRST(TIMRST_NEVER)  |
	      TIMCFG_TIMDIS(TIMDIS_COMPARE) | /* stop after bit count expires */
	      TIMCFG_TIMENA(TIMENA_TRG_HIGH) |
	      TIMCFG_TSTOP(TSTOP_BIT_ENABLE_TIMDIS) |
	      TIMCFG_TSTART(TSTART_BIT_ENABLE);
	fjtag_writel(fj, val, FLEXIO_TIMCFG(FLEXIO_JTAG_TIMER_TCK));

	/* Set default TCK frequency */
	fjtag_set_tck_hz(fj, FLEXIO_TCK_DEFAULT_HZ);

	/* De-assert TRST_N (active-low) */
	if (fj->trst_gpiod)
		gpiod_set_value_cansleep(fj->trst_gpiod, 0);
}

/* Transfer exactly @nbits bits (1..32).
 * tdi_word and tms_word hold the bits to transmit, LSB first.
 * Returns the received TDO bits, LSB in bit 0.
 *
 * SHIFTBUFBIS is the bit-inverted variant of SHIFTBUF, causing FlexIO to
 * shift the LS-bit first instead of the MS-bit first.  Both TDI and TMS
 * use it so that callers can present data LSB-first (standard JTAG).
 */
static u32 fjtag_xfer_word(struct flexio_jtag *fj, u32 nbits,
			    u32 tdi_word, u32 tms_word)
{
	u32 div = fjtag_baud_div(fj->tck_hz);
	u32 timcmp, stat, tdo;
	int ret;

	/* Update TIMCMP for the exact bit count of this transfer */
	timcmp = FLEXIO_TIMCMP_BAUD(nbits, div);
	fjtag_writel(fj, timcmp, FLEXIO_TIMCMP(FLEXIO_JTAG_TIMER_TCK));

	/* Align bits: shift right-justify so the last bit is at MSB position
	 * when using SHIFTBUFBIS (bit-swap): the value written to SHIFTBUFBIS
	 * is stored inverted, so the first bit shifted out is bit 0 of the
	 * written value. */
	fjtag_writel(fj, tms_word, FLEXIO_SHIFTBUFBIS(FLEXIO_JTAG_SHIFTER_TMS));

	/* Writing TDI last triggers Timer 0 (TDI is the trigger source) */
	fjtag_writel(fj, tdi_word, FLEXIO_SHIFTBUFBIS(FLEXIO_JTAG_SHIFTER_TDI));

	/* Wait for the receive shifter to report data ready (SHIFTSTAT bit 1) */
	ret = readl_relaxed_poll_timeout(fj->base + SHIFTSTAT, stat,
					 stat & BIT(FLEXIO_JTAG_SHIFTER_TDO),
					 1, 10000);
	if (ret)
		dev_warn_ratelimited(fj->dev, "TDO timeout\n");

	/* Read TDO (SHIFTBUFBIS converts MSB-first HW to LSB-first result) */
	tdo = fjtag_readl(fj, FLEXIO_SHIFTBUFBIS(FLEXIO_JTAG_SHIFTER_TDO));

	/* Clear status flags */
	fjtag_writel(fj, BIT(FLEXIO_JTAG_SHIFTER_TDO), SHIFTSTAT);

	return tdo;
}

/* -----------------------------------------------------------------------
 * TAP state machine
 * ----------------------------------------------------------------------- */

/* TMS sequence table to navigate between TAP states.
 * Each entry: [from][to] = {tms_bits, bit_count}
 * For simplicity, we use a generic TLR->desired path via known sequences. */

static void fjtag_set_tap_state(struct flexio_jtag *fj,
				enum jtag_tap_state from,
				enum jtag_tap_state to)
{
	/*
	 * Walk the TAP state machine using single-bit TMS transitions.
	 * This is a simplified implementation that walks via Test-Logic-Reset
	 * when the path is not directly known, then navigates to the target.
	 *
	 * For production use, a full TMS path lookup table (as in the
	 * upstream JTAG subsystem) should be substituted here.
	 */
	static const struct {
		u32 tms_bits;
		u8  nbits;
	} state_paths[16][16] = {
		/* From TLR */
		[JTAG_STATE_TLRESET][JTAG_STATE_IDLE]      = { 0x00, 1 },
		[JTAG_STATE_TLRESET][JTAG_STATE_SHIFTDR]   = { 0x02, 4 },
		[JTAG_STATE_TLRESET][JTAG_STATE_SHIFTIR]   = { 0x06, 5 },
		/* From IDLE */
		[JTAG_STATE_IDLE][JTAG_STATE_SHIFTDR]      = { 0x02, 3 },
		[JTAG_STATE_IDLE][JTAG_STATE_SHIFTIR]      = { 0x06, 4 },
		[JTAG_STATE_IDLE][JTAG_STATE_TLRESET]      = { 0x1F, 5 },
		/* From SHIFTDR */
		[JTAG_STATE_SHIFTDR][JTAG_STATE_IDLE]      = { 0x03, 3 },
		[JTAG_STATE_SHIFTDR][JTAG_STATE_UPDATEDR]  = { 0x03, 2 },
		[JTAG_STATE_SHIFTDR][JTAG_STATE_PAUSEDR]   = { 0x01, 2 },
		/* From SHIFTIR */
		[JTAG_STATE_SHIFTIR][JTAG_STATE_IDLE]      = { 0x03, 3 },
		[JTAG_STATE_SHIFTIR][JTAG_STATE_UPDATEIR]  = { 0x03, 2 },
		/* From UPDATEDR */
		[JTAG_STATE_UPDATEDR][JTAG_STATE_IDLE]     = { 0x00, 1 },
		[JTAG_STATE_UPDATEDR][JTAG_STATE_SHIFTDR]  = { 0x01, 2 },
		/* From UPDATEIR */
		[JTAG_STATE_UPDATEIR][JTAG_STATE_IDLE]     = { 0x00, 1 },
		[JTAG_STATE_UPDATEIR][JTAG_STATE_SHIFTDR]  = { 0x01, 2 },
		/* From PAUSEDR */
		[JTAG_STATE_PAUSEDR][JTAG_STATE_SHIFTDR]   = { 0x01, 2 },
	};
	u32 tms;
	u8  nbits;

	if (from == to)
		return;

	tms   = state_paths[from][to].tms_bits;
	nbits = state_paths[from][to].nbits;

	if (!nbits) {
		/* Unknown path: reset to TLR then go to target */
		fjtag_xfer_word(fj, 5, 0x00, 0x1F); /* five TMS=1 -> TLR */
		fj->tap_state = JTAG_STATE_TLRESET;
		if (to == JTAG_STATE_TLRESET)
			return;
		fjtag_set_tap_state(fj, JTAG_STATE_TLRESET, to);
		return;
	}

	fjtag_xfer_word(fj, nbits, 0, tms);
	fj->tap_state = to;
}

/* -----------------------------------------------------------------------
 * Data transfer
 * ----------------------------------------------------------------------- */

static int fjtag_do_xfer(struct flexio_jtag *fj, struct jtag_xfer *xfer,
			  u8 __user *user_buf)
{
	u32 nbits = xfer->length;
	u32 done = 0;
	u32 chunk, tdi_w, tms_w, tdo_w;
	u32 last_tms;
	int ret = 0;
	u8  *buf = NULL;
	u32 buf_bytes;

	if (!nbits || nbits > (1 << 20))
		return -EINVAL;

	buf_bytes = DIV_ROUND_UP(nbits, 8);
	buf = kzalloc(buf_bytes, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (xfer->direction & JTAG_WRITE_XFER) {
		if (copy_from_user(buf, user_buf, buf_bytes)) {
			ret = -EFAULT;
			goto out;
		}
	}

	/* Navigate to the shift state */
	if (xfer->type == JTAG_SIR_XFER)
		fjtag_set_tap_state(fj, fj->tap_state, JTAG_STATE_SHIFTIR);
	else
		fjtag_set_tap_state(fj, fj->tap_state, JTAG_STATE_SHIFTDR);

	while (done < nbits) {
		chunk = min_t(u32, nbits - done, FLEXIO_BITS_PER_WORD);

		/* Assemble TDI word from buffer (LSB-first per bit position) */
		tdi_w = 0;
		if (xfer->direction & JTAG_WRITE_XFER) {
			u32 byte_off = done / 8;
			u32 bit_off  = done % 8;
			u32 i;

			for (i = 0; i < chunk; i++) {
				u32 abs_bit = done + i;
				u32 b = (buf[abs_bit / 8] >> (abs_bit % 8)) & 1;

				tdi_w |= (b << i);
			}
			(void)byte_off; (void)bit_off;
		}

		/* TMS = 0 for all bits except the last (EXIT1 transition) */
		last_tms = ((done + chunk) >= nbits) ? 1 : 0;
		tms_w = last_tms << (chunk - 1);

		tdo_w = fjtag_xfer_word(fj, chunk, tdi_w, tms_w);

		if (xfer->direction & JTAG_READ_XFER) {
			u32 i;

			for (i = 0; i < chunk; i++) {
				u32 abs_bit = done + i;
				u32 b = (tdo_w >> i) & 1;

				if (b)
					buf[abs_bit / 8] |= BIT(abs_bit % 8);
				else
					buf[abs_bit / 8] &= ~BIT(abs_bit % 8);
			}
		}

		done += chunk;
	}

	/* Update TAP state: last TMS=1 moved us to EXIT1 */
	if (xfer->type == JTAG_SIR_XFER)
		fj->tap_state = JTAG_STATE_EXIT1IR;
	else
		fj->tap_state = JTAG_STATE_EXIT1DR;

	/* Navigate to requested end state */
	fjtag_set_tap_state(fj, fj->tap_state, xfer->endstate);

	if ((xfer->direction & JTAG_READ_XFER) && !ret) {
		if (copy_to_user(user_buf, buf, buf_bytes))
			ret = -EFAULT;
	}

out:
	kfree(buf);
	return ret;
}

/* -----------------------------------------------------------------------
 * File operations
 * ----------------------------------------------------------------------- */

static int flexio_jtag_open(struct inode *inode, struct file *file)
{
	struct flexio_jtag *fj =
		container_of(file->private_data, struct flexio_jtag, miscdev);

	if (!mutex_trylock(&fj->lock))
		return -EBUSY;
	return 0;
}

static int flexio_jtag_release(struct inode *inode, struct file *file)
{
	struct flexio_jtag *fj =
		container_of(file->private_data, struct flexio_jtag, miscdev);

	mutex_unlock(&fj->lock);
	return 0;
}

static long flexio_jtag_ioctl(struct file *file, unsigned int cmd,
			       unsigned long arg)
{
	struct flexio_jtag *fj =
		container_of(file->private_data, struct flexio_jtag, miscdev);
	void __user *uarg = (void __user *)arg;
	int ret = 0;

	switch (cmd) {
	case JTAG_GIOCFREQ: {
		u32 hz = fj->tck_hz;

		if (copy_to_user(uarg, &hz, sizeof(hz)))
			ret = -EFAULT;
		break;
	}
	case JTAG_SIOCFREQ: {
		u32 hz;

		if (copy_from_user(&hz, uarg, sizeof(hz))) {
			ret = -EFAULT;
			break;
		}
		if (hz < FLEXIO_TCK_MIN_HZ) {
			dev_err(fj->dev, "TCK %u Hz below minimum %u Hz\n",
				hz, FLEXIO_TCK_MIN_HZ);
			ret = -EINVAL;
			break;
		}
		fjtag_set_tck_hz(fj, hz);
		break;
	}
	case JTAG_GIOCSTATE: {
		u32 state = (u32)fj->tap_state;

		if (copy_to_user(uarg, &state, sizeof(state)))
			ret = -EFAULT;
		break;
	}
	case JTAG_SIOCSTATE: {
		struct jtag_tap_state_req req;

		if (copy_from_user(&req, uarg, sizeof(req))) {
			ret = -EFAULT;
			break;
		}
		if (req.reset) {
			if (fj->trst_gpiod) {
				gpiod_set_value_cansleep(fj->trst_gpiod, 1);
				udelay(10);
				gpiod_set_value_cansleep(fj->trst_gpiod, 0);
				udelay(10);
			} else {
				/* TMS walk to TLR */
				fjtag_xfer_word(fj, 5, 0, 0x1F);
			}
			fj->tap_state = JTAG_STATE_TLRESET;
		}
		if (req.endstate != fj->tap_state)
			fjtag_set_tap_state(fj, fj->tap_state, req.endstate);
		break;
	}
	case JTAG_IOCXFER: {
		struct jtag_xfer xfer;
		u8 __user *user_buf;

		if (copy_from_user(&xfer, uarg, sizeof(xfer))) {
			ret = -EFAULT;
			break;
		}
		user_buf = u64_to_user_ptr(xfer.tdio);
		ret = fjtag_do_xfer(fj, &xfer, user_buf);
		break;
	}
	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

static const struct file_operations flexio_jtag_fops = {
	.owner          = THIS_MODULE,
	.open           = flexio_jtag_open,
	.release        = flexio_jtag_release,
	.unlocked_ioctl = flexio_jtag_ioctl,
};

/* -----------------------------------------------------------------------
 * Platform driver
 * ----------------------------------------------------------------------- */

static int flexio_jtag_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct flexio_ddata *ddata;
	struct flexio_jtag *fj;
	int ret;

	fj = devm_kzalloc(dev, sizeof(*fj), GFP_KERNEL);
	if (!fj)
		return -ENOMEM;

	/* The FlexIO MFD parent provides the register base and clocks */
	ddata = dev_get_drvdata(dev->parent);
	if (!ddata) {
		dev_err(dev, "failed to get FlexIO parent driver data\n");
		return -ENODEV;
	}

	fj->base    = ddata->base;
	fj->per_clk = ddata->per_clk;
	fj->ipg_clk = ddata->ipg_clk;
	fj->dev     = dev;
	fj->tck_hz  = FLEXIO_TCK_DEFAULT_HZ;
	fj->tap_state = JTAG_STATE_TLRESET;
	mutex_init(&fj->lock);
	init_completion(&fj->xfer_done);

	/* TRST_N is an optional active-low GPIO */
	fj->trst_gpiod = devm_gpiod_get_optional(dev, "trst", GPIOD_OUT_HIGH);
	if (IS_ERR(fj->trst_gpiod))
		return dev_err_probe(dev, PTR_ERR(fj->trst_gpiod),
				     "failed to get trst GPIO\n");

	ret = clk_prepare_enable(fj->per_clk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable per_clk\n");

	ret = clk_prepare_enable(fj->ipg_clk);
	if (ret) {
		clk_disable_unprepare(fj->per_clk);
		return dev_err_probe(dev, ret, "failed to enable ipg_clk\n");
	}

	fjtag_hw_init(fj);

	fj->miscdev.minor  = MISC_DYNAMIC_MINOR;
	fj->miscdev.name   = "jtag0";
	fj->miscdev.fops   = &flexio_jtag_fops;
	fj->miscdev.parent = dev;

	ret = misc_register(&fj->miscdev);
	if (ret) {
		clk_disable_unprepare(fj->ipg_clk);
		clk_disable_unprepare(fj->per_clk);
		return dev_err_probe(dev, ret, "failed to register misc device\n");
	}

	platform_set_drvdata(pdev, fj);

	dev_info(dev, "FlexIO JTAG master registered as /dev/%s, TCK %u Hz\n",
		 fj->miscdev.name, fj->tck_hz);
	return 0;
}

static void flexio_jtag_remove(struct platform_device *pdev)
{
	struct flexio_jtag *fj = platform_get_drvdata(pdev);

	misc_deregister(&fj->miscdev);

	/* Reset FlexIO before release */
	fjtag_writel(fj, FLEXIO_CTRL_SWRST_MASK, CTRL);
	fjtag_writel(fj, 0, CTRL);

	clk_disable_unprepare(fj->ipg_clk);
	clk_disable_unprepare(fj->per_clk);
}

static const struct of_device_id flexio_jtag_of_match[] = {
	{ .compatible = "nxp,imx-flexio-jtag" },
	{ }
};
MODULE_DEVICE_TABLE(of, flexio_jtag_of_match);

static struct platform_driver flexio_jtag_driver = {
	.probe  = flexio_jtag_probe,
	.remove = flexio_jtag_remove,
	.driver = {
		.name           = "imx-flexio-jtag",
		.of_match_table = flexio_jtag_of_match,
	},
};
module_platform_driver(flexio_jtag_driver);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("NXP i.MX FlexIO JTAG master driver for latu-dcsm");
MODULE_LICENSE("GPL v2");
