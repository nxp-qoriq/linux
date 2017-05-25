/* Copyright 2013-2015 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the above-listed copyright holders nor the
 * names of any contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/module.h>
#include <linux/ptp_clock_kernel.h>

#include "../../fsl-mc/include/mc.h"
#include "../../fsl-mc/include/mc-sys.h"

#include "dprtc.h"
#include "dprtc-cmd.h"

#define N_EXT_TS	2

struct ptp_clock *clock;
struct fsl_mc_device *rtc_mc_dev;
u32 freqCompensation;

/* PTP clock operations */
static int ptp_dpaa2_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	u64 adj;
	u32 diff, tmr_add;
	int neg_adj = 0;
	int err = 0;
	struct fsl_mc_device *mc_dev = rtc_mc_dev;
	struct device *dev = &mc_dev->dev;

	if (ppb < 0) {
		neg_adj = 1;
		ppb = -ppb;
	}

	tmr_add = freqCompensation;
	adj = tmr_add;
	adj *= ppb;
	diff = div_u64(adj, 1000000000ULL);

	tmr_add = neg_adj ? tmr_add - diff : tmr_add + diff;

	err = dprtc_set_freq_compensation(mc_dev->mc_io, 0,
					  mc_dev->mc_handle, tmr_add);
	if (err)
		dev_err(dev, "dprtc_set_freq_compensation err %d\n", err);
	return 0;
}

static int ptp_dpaa2_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	s64 now;
	int err = 0;
	struct fsl_mc_device *mc_dev = rtc_mc_dev;
	struct device *dev = &mc_dev->dev;

	err = dprtc_get_time(mc_dev->mc_io, 0, mc_dev->mc_handle, &now);
	if (err) {
		dev_err(dev, "dprtc_get_time err %d\n", err);
		return 0;
	}

	now += delta;

	err = dprtc_set_time(mc_dev->mc_io, 0, mc_dev->mc_handle, now);
	if (err) {
		dev_err(dev, "dprtc_set_time err %d\n", err);
		return 0;
	}
	return 0;
}

static int ptp_dpaa2_gettime(struct ptp_clock_info *ptp, struct timespec *ts)
{
	u64 ns;
	u32 remainder;
	int err = 0;
	struct fsl_mc_device *mc_dev = rtc_mc_dev;
	struct device *dev = &mc_dev->dev;

	err = dprtc_get_time(mc_dev->mc_io, 0, mc_dev->mc_handle, &ns);
	if (err) {
		dev_err(dev, "dprtc_get_time err %d\n", err);
		return 0;
	}

	ts->tv_sec = div_u64_rem(ns, 1000000000, &remainder);
	ts->tv_nsec = remainder;
	return 0;
}

static int ptp_dpaa2_settime(struct ptp_clock_info *ptp,
			       const struct timespec *ts)
{
	u64 ns;
	int err = 0;
	struct fsl_mc_device *mc_dev = rtc_mc_dev;
	struct device *dev = &mc_dev->dev;

	ns = ts->tv_sec * 1000000000ULL;
	ns += ts->tv_nsec;

	err = dprtc_set_time(mc_dev->mc_io, 0, mc_dev->mc_handle, ns);
	if (err)
		dev_err(dev, "dprtc_set_time err %d\n", err);
	return 0;
}

static struct ptp_clock_info ptp_dpaa2_caps = {
	.owner		= THIS_MODULE,
	.name		= "dpaa2 clock",
	.max_adj	= 512000,
	.n_alarm	= 0,
	.n_ext_ts	= N_EXT_TS,
	.n_per_out	= 0,
	.n_pins		= 0,
	.pps		= 1,
	.adjfreq	= ptp_dpaa2_adjfreq,
	.adjtime	= ptp_dpaa2_adjtime,
	.gettime64	= ptp_dpaa2_gettime,
	.settime64	= ptp_dpaa2_settime,
};

static int rtc_probe(struct fsl_mc_device *mc_dev)
{
	struct device		*dev;
	int			err = 0;
	int			dpaa2_phc_index;
	u32			tmr_add = 0;

	if (!mc_dev)
		return -EFAULT;

	dev = &mc_dev->dev;

	err = fsl_mc_portal_allocate(mc_dev, 0, &mc_dev->mc_io);
	if (unlikely(err)) {
		dev_err(dev, "fsl_mc_portal_allocate err %d\n", err);
		goto err_exit;
	}
	if (!mc_dev->mc_io) {
		dev_err(dev,
			"fsl_mc_portal_allocate returned null handle but no error\n");
		err = -EFAULT;
		goto err_exit;
	}

	err = dprtc_open(mc_dev->mc_io, 0, mc_dev->obj_desc.id,
			 &mc_dev->mc_handle);
	if (err) {
		dev_err(dev, "dprtc_open err %d\n", err);
		goto err_free_mcp;
	}
	if (!mc_dev->mc_handle) {
		dev_err(dev, "dprtc_open returned null handle but no error\n");
		err = -EFAULT;
		goto err_free_mcp;
	}

	rtc_mc_dev = mc_dev;

	err = dprtc_get_freq_compensation(mc_dev->mc_io, 0,
					  mc_dev->mc_handle, &tmr_add);
	if (err) {
		dev_err(dev, "dprtc_get_freq_compensation err %d\n", err);
		goto err_close;
	}
	freqCompensation = tmr_add;

	clock = ptp_clock_register(&ptp_dpaa2_caps, dev);
	if (IS_ERR(clock)) {
		err = PTR_ERR(clock);
		goto err_close;
	}
	dpaa2_phc_index = ptp_clock_index(clock);

	return 0;
err_close:
	dprtc_close(mc_dev->mc_io, 0, mc_dev->mc_handle);
err_free_mcp:
	fsl_mc_portal_free(mc_dev->mc_io);
err_exit:
	return err;
}

static int rtc_remove(struct fsl_mc_device *mc_dev)
{
	ptp_clock_unregister(clock);
	dprtc_close(mc_dev->mc_io, 0, mc_dev->mc_handle);
	fsl_mc_portal_free(mc_dev->mc_io);

	return 0;
}

static const struct fsl_mc_device_id rtc_match_id_table[] = {
	{
		.vendor = FSL_MC_VENDOR_FREESCALE,
		.obj_type = "dprtc",
	},
	{}
};

static struct fsl_mc_driver rtc_drv = {
	.driver = {
		.name		= KBUILD_MODNAME,
		.owner		= THIS_MODULE,
	},
	.probe		= rtc_probe,
	.remove		= rtc_remove,
	.match_id_table = rtc_match_id_table,
};

module_fsl_mc_driver(rtc_drv);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DPAA2 RTC (PTP 1588 clock) driver (prototype)");
