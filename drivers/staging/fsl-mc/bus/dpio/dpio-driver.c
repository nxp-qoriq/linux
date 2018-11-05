/*
 * Copyright 2014-2016 Freescale Semiconductor Inc.
 * Copyright 2016 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/msi.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>

#include "../../include/mc.h"
#include "../../include/mc-sys.h"
#include "../../include/dpaa2-io.h"

#include "qbman-portal.h"
#include "dpio.h"
#include "dpio-cmd.h"

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Freescale Semiconductor, Inc");
MODULE_DESCRIPTION("DPIO Driver");

struct dpio_priv {
	struct dpaa2_io *io;
};

static cpumask_var_t cpus_unused_mask;
static struct fsl_mc_io *dpio_mc_io = NULL;
static int dpio_num_probed = 0;

static irqreturn_t dpio_irq_handler(int irq_num, void *arg)
{
	struct device *dev = (struct device *)arg;
	struct dpio_priv *priv = dev_get_drvdata(dev);

	return dpaa2_io_irq(priv->io);
}

static void unregister_dpio_irq_handlers(struct fsl_mc_device *dpio_dev)
{
	struct fsl_mc_device_irq *irq;

	irq = dpio_dev->irqs[0];

	/* clear the affinity hint */
	irq_set_affinity_hint(irq->msi_desc->irq, NULL);
}

static int register_dpio_irq_handlers(struct fsl_mc_device *dpio_dev, int cpu)
{
	struct dpio_priv *priv;
	int error;
	struct fsl_mc_device_irq *irq;
	cpumask_t mask;

	priv = dev_get_drvdata(&dpio_dev->dev);

	irq = dpio_dev->irqs[0];
	error = devm_request_irq(&dpio_dev->dev,
				 irq->msi_desc->irq,
				 dpio_irq_handler,
				 0,
				 dev_name(&dpio_dev->dev),
				 &dpio_dev->dev);
	if (error < 0) {
		dev_err(&dpio_dev->dev,
			"devm_request_irq() failed: %d\n",
			error);
		return error;
	}

	/* set the affinity hint */
	cpumask_clear(&mask);
	cpumask_set_cpu(cpu, &mask);
	if (irq_set_affinity_hint(irq->msi_desc->irq, &mask))
		dev_err(&dpio_dev->dev,
			"irq_set_affinity failed irq %d cpu %d\n",
			irq->msi_desc->irq, cpu);

	return 0;
}

static int dpaa2_dpio_probe(struct fsl_mc_device *dpio_dev)
{
	struct dpio_attr dpio_attrs;
	struct dpaa2_io_desc desc;
	struct dpio_priv *priv;
	int err = -ENOMEM;
	struct device *dev = &dpio_dev->dev;
	struct device *root_dprc_dev;
	int next_cpu;

	fsl_mc_get_root_dprc(dev, &root_dprc_dev);
	if (root_dprc_dev->links.status != DL_DEV_DRIVER_BOUND)
		return -EPROBE_DEFER;
	device_link_add(dev, root_dprc_dev, DL_FLAG_AUTOREMOVE);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		goto err_priv_alloc;

	dev_set_drvdata(dev, priv);

	if (dpio_mc_io == NULL) {
		err = fsl_mc_portal_allocate(dpio_dev, 0, &dpio_mc_io);
		if (err) {
			dev_dbg(dev, "MC portal allocation failed\n");
			err = -EPROBE_DEFER;
			goto err_mcportal;
		}
	}
	dpio_dev->mc_io = dpio_mc_io;
	device_link_add(dev, &dpio_mc_io->dpmcp_dev->dev, DL_FLAG_AUTOREMOVE);

	err = dpio_open(dpio_dev->mc_io, 0, dpio_dev->obj_desc.id,
			&dpio_dev->mc_handle);
	if (err) {
		dev_err(dev, "dpio_open() failed\n");
		goto err_open;
	}

	err = dpio_reset(dpio_dev->mc_io, 0, dpio_dev->mc_handle);
	if (err) {
		dev_err(dev, "dpio_reset() failed\n");
		goto err_reset;
	}

	err = dpio_get_attributes(dpio_dev->mc_io, 0, dpio_dev->mc_handle,
				  &dpio_attrs);
	if (err) {
		dev_err(dev, "dpio_get_attributes() failed %d\n", err);
		goto err_get_attr;
	}
	desc.qman_version = dpio_attrs.qbman_version;

	err = dpio_enable(dpio_dev->mc_io, 0, dpio_dev->mc_handle);
	if (err) {
		dev_err(dev, "dpio_enable() failed %d\n", err);
		goto err_get_attr;
	}

	/* initialize DPIO descriptor */
	desc.receives_notifications = dpio_attrs.num_priorities ? 1 : 0;
	desc.has_8prio = dpio_attrs.num_priorities == 8 ? 1 : 0;
	desc.dpio_id = dpio_dev->obj_desc.id;

	/* get the cpu to use for the affinity hint */
	next_cpu = cpumask_first(cpus_unused_mask);

	if (!cpu_possible(next_cpu)) {
		dev_err(dev, "probe failed. Number of DPIOs exceeds NR_CPUS.\n");
		err = -ERANGE;
		goto err_allocate_irqs;
	}
	desc.cpu = next_cpu;
	cpumask_clear_cpu(next_cpu, cpus_unused_mask);

	if (dpio_dev->obj_desc.region_count < 3) {
		/* No support for DDR backed portals, use classic mapping */
		desc.regs_cena = qbman_cena_ioremap(dpio_dev->regions[0].start,
					resource_size(&dpio_dev->regions[0]));
	} else {
		desc.regs_cena = memremap(dpio_dev->regions[2].start,
					resource_size(&dpio_dev->regions[2]),
					MEMREMAP_WB);
		pr_info("Mapped 0x%llx to %p size %llu\n",dpio_dev->regions[2].start, desc.regs_cena, resource_size(&dpio_dev->regions[2]));

	}
	desc.regs_cinh = ioremap(dpio_dev->regions[1].start,
		resource_size(&dpio_dev->regions[1]));

	err = fsl_mc_allocate_irqs(dpio_dev);
	if (err) {
		dev_err(dev, "fsl_mc_allocate_irqs failed. err=%d\n", err);
		goto err_allocate_irqs;
	}

	err = register_dpio_irq_handlers(dpio_dev, desc.cpu);
	if (err)
		goto err_register_dpio_irq;

	priv->io = dpaa2_io_create(&desc);
	if (!priv->io) {
		dev_err(dev, "dpaa2_io_create failed\n");
		goto err_dpaa2_io_create;
	}

	dpio_num_probed++;

	dev_info(dev, "probed\n");
	dev_dbg(dev, "   receives_notifications = %d\n",
		desc.receives_notifications);
	dpio_close(dpio_dev->mc_io, 0, dpio_dev->mc_handle);

	return 0;

err_dpaa2_io_create:
	unregister_dpio_irq_handlers(dpio_dev);
err_register_dpio_irq:
	fsl_mc_free_irqs(dpio_dev);
err_allocate_irqs:
	cpumask_set_cpu(next_cpu, cpus_unused_mask);
	dpio_disable(dpio_dev->mc_io, 0, dpio_dev->mc_handle);
err_get_attr:
err_reset:
	dpio_close(dpio_dev->mc_io, 0, dpio_dev->mc_handle);
err_open:
	if (cpumask_full(cpus_unused_mask)) {
		fsl_mc_portal_free(dpio_dev->mc_io);
		dpio_mc_io = NULL;
	}
err_mcportal:
	dev_set_drvdata(dev, NULL);
err_priv_alloc:
	return err;
}

/* Tear down interrupts for a given DPIO object */
static void dpio_teardown_irqs(struct fsl_mc_device *dpio_dev)
{
	unregister_dpio_irq_handlers(dpio_dev);
	fsl_mc_free_irqs(dpio_dev);
}

static int dpaa2_dpio_remove(struct fsl_mc_device *dpio_dev)
{
	struct device *dev;
	struct dpio_priv *priv;
	int err, cpu;

	dev = &dpio_dev->dev;
	priv = dev_get_drvdata(dev);

	dpaa2_io_down(priv->io);

	dpio_teardown_irqs(dpio_dev);

	err = dpio_open(dpio_dev->mc_io, 0, dpio_dev->obj_desc.id,
			&dpio_dev->mc_handle);
	if (err) {
		dev_err(dev, "dpio_open() failed\n");
		goto err_open;
	}

	dpio_disable(dpio_dev->mc_io, 0, dpio_dev->mc_handle);

	dpio_close(dpio_dev->mc_io, 0, dpio_dev->mc_handle);

err_open:

	cpu = dpaa2_io_get_cpu(priv->io);
	cpumask_set_cpu(cpu, cpus_unused_mask);

	if (dpio_num_probed == 1) {
		fsl_mc_portal_free(dpio_dev->mc_io);
		dpio_mc_io = NULL;
	}
	dpio_num_probed--;

	dev_set_drvdata(dev, NULL);

	return err;
}

static const struct fsl_mc_device_id dpaa2_dpio_match_id_table[] = {
	{
		.vendor = FSL_MC_VENDOR_FREESCALE,
		.obj_type = "dpio",
	},
	{ .vendor = 0x0 }
};

static struct fsl_mc_driver dpaa2_dpio_driver = {
	.driver = {
		.name		= KBUILD_MODNAME,
		.owner		= THIS_MODULE,
	},
	.probe		= dpaa2_dpio_probe,
	.remove		= dpaa2_dpio_remove,
	.match_id_table = dpaa2_dpio_match_id_table
};

static int dpio_driver_init(void)
{
	if (!zalloc_cpumask_var(&cpus_unused_mask, GFP_KERNEL))
		return -ENOMEM;
	cpumask_setall(cpus_unused_mask);
	cpumask_and(cpus_unused_mask, cpus_unused_mask, cpu_online_mask);

	return fsl_mc_driver_register(&dpaa2_dpio_driver);
}

static void dpio_driver_exit(void)
{
	free_cpumask_var(cpus_unused_mask);
	fsl_mc_driver_unregister(&dpaa2_dpio_driver);
}
module_init(dpio_driver_init);
module_exit(dpio_driver_exit);
