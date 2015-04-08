/* Copyright 2014 Freescale Semiconductor Inc.
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
#include <linux/kthread.h>
#include <linux/delay.h>

#include "../../include/mc.h"
#include "../../include/fsl_dpaa2_io.h"

#include "fsl_qbman_portal.h"
#include "fsl_dpio.h"
#include "fsl_dpio_cmd.h"

#include "dpio-drv.h"

#define DPIO_DESCRIPTION "DPIO Driver"

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Freescale Semiconductor, Inc");
MODULE_DESCRIPTION(DPIO_DESCRIPTION);

#define MAX_DPIO_IRQ_NAME 16 /* Big enough for "FSL DPIO %d" */

struct dpio_priv {
	struct dpaa2_io *io;
	char irq_name[MAX_DPIO_IRQ_NAME];
	struct task_struct *thread;
};

static int dpio_thread(void *data)
{
	struct dpaa2_io *io = data;

	while (!kthread_should_stop()) {
		int err = dpaa2_io_poll(io);

		if (err) {
			pr_err("dpaa2_io_poll() failed\n");
			return err;
		}
		msleep(50);
	}
	return 0;
}

static irqreturn_t dpio_irq_handler(int irq_num, void *arg)
{
	struct device *dev = (struct device *)arg;
	struct dpio_priv *priv = dev_get_drvdata(dev);

	return dpaa2_io_irq(priv->io);
}

static void unregister_dpio_irq_handlers(struct fsl_mc_device *ls_dev)
{
	int i;
	struct fsl_mc_device_irq *irq;
	int irq_count = ls_dev->obj_desc.irq_count;

	for (i = 0; i < irq_count; i++) {
		irq = ls_dev->irqs[i];
		devm_free_irq(&ls_dev->dev, irq->msi_desc->irq, &ls_dev->dev);
	}
}

static int register_dpio_irq_handlers(struct fsl_mc_device *ls_dev, int cpu)
{
	struct dpio_priv *priv;
	unsigned int i;
	int error;
	struct fsl_mc_device_irq *irq;
	unsigned int num_irq_handlers_registered = 0;
	int irq_count = ls_dev->obj_desc.irq_count;
	cpumask_t mask;

	priv = dev_get_drvdata(&ls_dev->dev);

	if (WARN_ON(irq_count != 1))
		return -EINVAL;

	for (i = 0; i < irq_count; i++) {
		irq = ls_dev->irqs[i];
		error = devm_request_irq(&ls_dev->dev,
					 irq->msi_desc->irq,
					 dpio_irq_handler,
					 0,
					 priv->irq_name,
					 &ls_dev->dev);
		if (error < 0) {
			dev_err(&ls_dev->dev,
				"devm_request_irq() failed: %d\n",
				error);
			goto error_unregister_irq_handlers;
		}

		/* Set the IRQ affinity */
		cpumask_clear(&mask);
		cpumask_set_cpu(cpu, &mask);
		if (irq_set_affinity(irq->msi_desc->irq, &mask))
			pr_err("irq_set_affinity failed irq %d cpu %d\n",
			       irq->msi_desc->irq, cpu);

		num_irq_handlers_registered++;
	}

	return 0;

error_unregister_irq_handlers:
	for (i = 0; i < num_irq_handlers_registered; i++) {
		irq = ls_dev->irqs[i];
		devm_free_irq(&ls_dev->dev, irq->msi_desc->irq,
			      &ls_dev->dev);
	}

	return error;
}

static int __cold
dpaa2_dpio_probe(struct fsl_mc_device *ls_dev)
{
	struct dpio_attr dpio_attrs;
	struct dpaa2_io_desc desc;
	struct dpio_priv *priv;
	int err = -ENOMEM;
	struct device *dev = &ls_dev->dev;
	struct dpaa2_io *defservice;
	bool irq_allocated = false;
	static int next_cpu;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		goto err_priv_alloc;

	dev_set_drvdata(dev, priv);

	err = fsl_mc_portal_allocate(ls_dev, 0, &ls_dev->mc_io);
	if (err) {
		dev_err(dev, "MC portal allocation failed\n");
		err = -EPROBE_DEFER;
		goto err_mcportal;
	}

	err = dpio_open(ls_dev->mc_io, 0, ls_dev->obj_desc.id,
			&ls_dev->mc_handle);
	if (err) {
		dev_err(dev, "dpio_open() failed\n");
		goto err_open;
	}

	err = dpio_get_attributes(ls_dev->mc_io, 0, ls_dev->mc_handle,
				  &dpio_attrs);
	if (err) {
		dev_err(dev, "dpio_get_attributes() failed %d\n", err);
		goto err_get_attr;
	}
	err = dpio_enable(ls_dev->mc_io, 0, ls_dev->mc_handle);
	if (err) {
		dev_err(dev, "dpio_enable() failed %d\n", err);
		goto err_get_attr;
	}
	pr_info("ce_paddr=0x%llx, ci_paddr=0x%llx, portalid=%d, prios=%d\n",
		ls_dev->regions[0].start,
		ls_dev->regions[1].start,
		dpio_attrs.qbman_portal_id,
		dpio_attrs.num_priorities);

	pr_info("ce_size=0x%llx, ci_size=0x%llx\n",
		resource_size(&ls_dev->regions[0]),
		resource_size(&ls_dev->regions[1]));

	desc.qman_version = dpio_attrs.qbman_version;
	/* Build DPIO driver object out of raw MC object */
	desc.receives_notifications = dpio_attrs.num_priorities ? 1 : 0;
	desc.has_irq = 1;
	desc.will_poll = 1;
	desc.has_8prio = dpio_attrs.num_priorities == 8 ? 1 : 0;
	desc.cpu = next_cpu;
	desc.stash_affinity = 1; /* TODO: Figure out how to determine
				    this setting - will we ever have non-affine
				    portals where we stash to a platform cache? */
	next_cpu = (next_cpu + 1) % num_active_cpus();
	desc.dpio_id = ls_dev->obj_desc.id;
	desc.regs_cena = ioremap_cache_ns(ls_dev->regions[0].start,
		resource_size(&ls_dev->regions[0]));
	desc.regs_cinh = ioremap(ls_dev->regions[1].start,
		resource_size(&ls_dev->regions[1]));

	err = fsl_mc_allocate_irqs(ls_dev);
	if (err) {
		dev_err(dev, "DPIO fsl_mc_allocate_irqs failed\n");
		desc.has_irq = 0;
	} else {
		irq_allocated = true;

		snprintf(priv->irq_name, MAX_DPIO_IRQ_NAME, "FSL DPIO %d",
			 desc.dpio_id);

		err = register_dpio_irq_handlers(ls_dev, desc.cpu);
		if (err)
			desc.has_irq = 0;
	}

	priv->io = dpaa2_io_create(&desc);
	if (!priv->io) {
		dev_err(dev, "DPIO setup failed\n");
		goto err_dpaa2_io_create;
	}

	/* If no irq then go to poll mode */
	if (desc.has_irq == 0) {
		dev_info(dev, "Using polling mode for DPIO %d\n",
			 desc.dpio_id);
		/* goto err_register_dpio_irq; */
		/* TEMP: Start polling if IRQ could not
		   be registered.  This will go away once
		   KVM support for MSI is present */
		if (irq_allocated == true)
			fsl_mc_free_irqs(ls_dev);

		if (desc.stash_affinity)
			priv->thread = kthread_create_on_cpu(dpio_thread,
							     priv->io,
							     desc.cpu,
							     "dpio_aff%u");
		else
			priv->thread =
				kthread_create(dpio_thread,
					       priv->io,
					       "dpio_non%u",
					       dpio_attrs.qbman_portal_id);
		if (IS_ERR(priv->thread)) {
			dev_err(dev, "DPIO thread failure\n");
			err = PTR_ERR(priv->thread);
			goto err_dpaa_thread;
		}
		kthread_unpark(priv->thread);
		wake_up_process(priv->thread);
	}

	defservice = dpaa2_io_default_service();
	err = dpaa2_io_service_add(defservice, priv->io);
	dpaa2_io_down(defservice);
	if (err) {
		dev_err(dev, "DPIO add-to-service failed\n");
		goto err_dpaa2_io_add;
	}

	dev_info(dev, "dpio: probed object %d\n", ls_dev->obj_desc.id);
	dev_info(dev, "   receives_notifications = %d\n",
			desc.receives_notifications);
	dev_info(dev, "   has_irq = %d\n", desc.has_irq);
	dpio_close(ls_dev->mc_io, 0, ls_dev->mc_handle);
	fsl_mc_portal_free(ls_dev->mc_io);
	return 0;

err_dpaa2_io_add:
	unregister_dpio_irq_handlers(ls_dev);
/* TEMP: To be restored once polling is removed
  err_register_dpio_irq:
	fsl_mc_free_irqs(ls_dev);
*/
err_dpaa_thread:
err_dpaa2_io_create:
	dpio_disable(ls_dev->mc_io, 0, ls_dev->mc_handle);
err_get_attr:
	dpio_close(ls_dev->mc_io, 0, ls_dev->mc_handle);
err_open:
	fsl_mc_portal_free(ls_dev->mc_io);
err_mcportal:
	dev_set_drvdata(dev, NULL);
	devm_kfree(dev, priv);
err_priv_alloc:
	return err;
}

/*
 * Tear down interrupts for a given DPIO object
 */
static void dpio_teardown_irqs(struct fsl_mc_device *ls_dev)
{
	/* (void)disable_dpio_irqs(ls_dev); */
	unregister_dpio_irq_handlers(ls_dev);
	fsl_mc_free_irqs(ls_dev);
}

static int __cold
dpaa2_dpio_remove(struct fsl_mc_device *ls_dev)
{
	struct device *dev;
	struct dpio_priv *priv;
	int err;

	dev = &ls_dev->dev;
	priv = dev_get_drvdata(dev);

	/* there is no implementation yet for pulling a DPIO object out of a
	 * running service (and they're currently always running).
	 */
	dev_crit(dev, "DPIO unplugging is broken, the service holds onto it\n");

	if (priv->thread)
		kthread_stop(priv->thread);
	else
		dpio_teardown_irqs(ls_dev);

	err = fsl_mc_portal_allocate(ls_dev, 0, &ls_dev->mc_io);
	if (err) {
		dev_err(dev, "MC portal allocation failed\n");
		goto err_mcportal;
	}

	err = dpio_open(ls_dev->mc_io, 0, ls_dev->obj_desc.id,
			&ls_dev->mc_handle);
	if (err) {
		dev_err(dev, "dpio_open() failed\n");
		goto err_open;
	}

	dev_set_drvdata(dev, NULL);
	dpaa2_io_down(priv->io);

	err = 0;

	dpio_disable(ls_dev->mc_io, 0, ls_dev->mc_handle);
	dpio_close(ls_dev->mc_io, 0, ls_dev->mc_handle);
err_open:
	fsl_mc_portal_free(ls_dev->mc_io);
err_mcportal:
	return err;
}

static const struct fsl_mc_device_match_id dpaa2_dpio_match_id_table[] = {
	{
		.vendor = FSL_MC_VENDOR_FREESCALE,
		.obj_type = "dpio",
		.ver_major = DPIO_VER_MAJOR,
		.ver_minor = DPIO_VER_MINOR
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
	int err;

	err = dpaa2_io_service_driver_init();
	if (!err) {
		err = fsl_mc_driver_register(&dpaa2_dpio_driver);
		if (err)
			dpaa2_io_service_driver_exit();
	}
	return err;
}
static void dpio_driver_exit(void)
{
	fsl_mc_driver_unregister(&dpaa2_dpio_driver);
	dpaa2_io_service_driver_exit();
}
module_init(dpio_driver_init);
module_exit(dpio_driver_exit);
