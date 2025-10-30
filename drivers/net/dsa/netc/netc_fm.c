// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NETC switch Frame Modification driver
 *
 * Copyright 2025 NXP
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/fsl/ntmp.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>

#include "netc_fm.h"
#include "ntmp_private.h"
#include "netc_switch.h"

#define NETC_SWITCH_FM_BUF_SIZE		(2048U)
#define NETC_FM_FMDT_BLOCK_UNIT		(24U)
#define NETC_FM_MAX_DATASETMSG		(32U)

struct netc_fm_priv {
	struct netc_cbdrs *cbdrs;
	struct netc_switch *netc_switch;
	u32 ipft_eid;
	u32 ist_eid;
	u32 ett_eid;
	u32 fmt_eid;
	u32 fmdt_eid;
	struct ntmp_ipft_entry ipft_entry;
	struct ntmp_ist_entry ist_entry;
	struct ett_cfge_data ett_cfg;
	struct fmt_cfge_data fmt_cfg;
	struct netc_fm_conf config;
	/* the number of the DataSetMessages to insert, range from 1 to 32 */
	u32 datasetmsg_cnt;
	void *fmdt_buffer;
	size_t fmdt_size;
	bool valid;
};

static DEFINE_MUTEX(netc_switch_fm_mutex);
static struct netc_switch *netc_switch_g;

static void netc_fm_enable_ipft(struct netc_switch *netc_switch, uint16_t port_id)
{
	struct netc_port *port = NETC_PORT(netc_switch, port_id);
	u32 val;

	val = netc_port_rd(port, NETC_PIPFCR);
	val |= PIPFCR_EN;
	netc_port_wr(port, NETC_PIPFCR, val);
}

static void netc_fm_disable_ipft(struct netc_switch *netc_switch, uint16_t port_id)
{
	struct netc_port *port = NETC_PORT(netc_switch, port_id);
	u32 val;

	val = netc_port_rd(port, NETC_PIPFCR);
	val &= ~(PIPFCR_EN);
	netc_port_wr(port, NETC_PIPFCR, val);
}

static u32 netc_fm_set_ipft(struct netc_fm_priv *priv, u32 ist_eid, struct netc_fm_conf *config)
{
	struct ntmp_ipft_entry *ipft = &priv->ipft_entry;
	struct ipft_keye_data *keye = &ipft->keye;
	struct ipft_cfge_data *cfge = &ipft->cfge;
	struct netc_cbdrs *cbdrs = priv->cbdrs;
	u32 ipft_eid;
	int ret;

	/* set it to the highest priority */
	keye->precedence = 0;

	/* set the filter parameters related to the MAC layer */
	keye->ethertype = htons(config->filter.ethertype);
	keye->ethertype_mask = 0x0ffff;

	cfge->cfg |= FIELD_PREP(IPFT_FLTFA, IPFT_FLTFA_PERMIT);
	cfge->cfg |= FIELD_PREP(IPFT_FLTA, IPFT_FLTA_IS);
	cfge->flta_tgt = ist_eid;

	ret = ntmp_ipft_add_entry(cbdrs, &ipft_eid, ipft);
	if (ret)
		return NTMP_NULL_ENTRY_ID;

	return ipft_eid;
}

static u32 netc_fm_alloc_ist(struct ntmp_priv *ntmp)
{
	u32 entry_id;

	entry_id = ntmp_lookup_free_eid(ntmp->ist_eid_bitmap, ntmp->caps.ist_num_entries);
	return entry_id;
}

static void netc_fm_free_ist(struct ntmp_priv *ntmp, u32 entry_id)
{
	if (entry_id == NTMP_NULL_ENTRY_ID)
		return;

	ntmp_clear_eid_bitmap(ntmp->ist_eid_bitmap, entry_id);
}

static u32 netc_fm_set_ist(struct netc_fm_priv *priv, u32 ett_eid, struct netc_fm_conf *config)
{
	struct ntmp_priv *ntmp = &priv->netc_switch->ntmp;
	struct netc_cbdrs *cbdrs = priv->cbdrs;
	struct ntmp_ist_entry *ist_entry = &priv->ist_entry;
	struct ist_cfge_data *ist_cfge = &ist_entry->cfge;
	u32 ist_eid;
	int ret;

	/* bypass the ingress stream filter table lookup */
	ist_cfge->cfg |= FIELD_PREP(IST_SFE, 0);
	ist_cfge->cfg |= FIELD_PREP(IST_V1_FA, 2);	/* enable stream forwarding */
	ist_cfge->cfg |= FIELD_PREP(IST_SPPD, 1);	/* disable source port pruning */

	/* enable stream forwarding */
	ist_cfge->switch_cfg |= FIELD_PREP(IST_OETEID, 2);
	ist_cfge->bitmap_evmeid |= FIELD_PREP(IST_EGRESS_PORT_BITMAP, 1 << config->egress_port);

	ist_cfge->et_eid = ett_eid;
	/* set unused eid to null */
	ist_cfge->isqg_eid = NTMP_NULL_ENTRY_ID;
	ist_cfge->rp_eid = NTMP_NULL_ENTRY_ID;
	ist_cfge->sgi_eid = NTMP_NULL_ENTRY_ID;
	ist_cfge->ifm_eid = NTMP_NULL_ENTRY_ID;
	ist_cfge->isc_eid = NTMP_NULL_ENTRY_ID;

	ist_eid = netc_fm_alloc_ist(ntmp);
	if (ist_eid == NTMP_NULL_ENTRY_ID)
		return NTMP_NULL_ENTRY_ID;

	ist_entry->entry_id = ist_eid;
	ret = ntmp_ist_add_or_update_entry(cbdrs, ist_entry);
	if (ret)
		goto err;

	return ist_eid;
err:
	netc_fm_free_ist(ntmp, ist_eid);
	return NTMP_NULL_ENTRY_ID;
}

static u32 netc_fm_alloc_ett(struct ntmp_priv *ntmp)
{
	u32 entry_id;

	entry_id = ntmp_lookup_free_eid(ntmp->ett_gid_bitmap, ntmp->ett_bitmap_size);
	return entry_id;
}

static void netc_fm_free_ett(struct ntmp_priv *ntmp, u32 entry_id)
{
	if (entry_id == NTMP_NULL_ENTRY_ID)
		return;

	ntmp_clear_eid_bitmap(ntmp->ett_gid_bitmap, entry_id);
}

static int netc_fm_update_ett(struct netc_fm_priv *priv)
{
	struct ett_cfge_data *ett_cfg = &priv->ett_cfg;
	struct netc_cbdrs *cbdrs = priv->cbdrs;
	int ret;

	ett_cfg->efm_data_len = priv->fmdt_size;
	ett_cfg->efm_eid = priv->fmt_eid;
	ret = ntmp_ett_add_or_update_entry(cbdrs, priv->ett_eid, false, ett_cfg);
	if (ret)
		return ret;

	return 0;
}

static u32 netc_fm_set_ett(struct netc_fm_priv *priv, u32 fmt_eid, struct netc_fm_conf *config)
{
	struct netc_cbdrs *cbdrs = priv->cbdrs;
	struct ntmp_priv *ntmp = &priv->netc_switch->ntmp;
	struct ett_cfge_data *ett_cfg = &priv->ett_cfg;
	u32 ett_eid;
	int ret;

	ett_cfg->efm_cfg |= FIELD_PREP(ETT_EFM_MODE, ETT_EFM_MODE_INSERT);
	ett_cfg->efm_data_len = priv->fmdt_size;

	ett_cfg->efm_eid = fmt_eid;
	ett_cfg->ec_eid = NTMP_NULL_ENTRY_ID;
	ett_cfg->esqa_tgt_eid = NTMP_NULL_ENTRY_ID;

	ett_eid = netc_fm_alloc_ett(ntmp);
	if (ett_eid == NTMP_NULL_ENTRY_ID)
		goto err;

	ret = ntmp_ett_add_or_update_entry(cbdrs, ett_eid, true, ett_cfg);
	if (ret) {
		netc_fm_free_ett(ntmp, ett_eid);
		goto err;
	}
	return ett_eid;

err:
	return NTMP_NULL_ENTRY_ID;
}

static u32 netc_fm_alloc_fmt(struct ntmp_priv *ntmp)
{
	u32 entry_id;

	entry_id = ntmp_lookup_free_eid(ntmp->fmt_eid_bitmap, ntmp->caps.fmt_num_entries);
	return entry_id;
}

static void netc_fm_free_fmt(struct ntmp_priv *ntmp, u32 entry_id)
{
	if (entry_id == NTMP_NULL_ENTRY_ID)
		return;

	ntmp_clear_eid_bitmap(ntmp->fmt_eid_bitmap, entry_id);
}

static u32 netc_fm_set_fmt(struct netc_fm_priv *priv, u32 fmdt_eid, struct netc_fm_conf *config)
{
	struct netc_cbdrs *cbdrs = priv->cbdrs;
	struct ntmp_priv *ntmp = &priv->netc_switch->ntmp;
	struct fmt_cfge_data *fmt_cfg = &priv->fmt_cfg;
	u32 fmt_eid;
	int ret;

	if (priv->datasetmsg_cnt > NETC_FM_MAX_DATASETMSG)
		return NTMP_NULL_ENTRY_ID;

	if (config->fm_action == FMT_PLD_ACT_INSERT_DATASETMSG) {
		fmt_cfg->act1 |= FIELD_PREP(FMT_PLD_ACT, FMT_PLD_ACT_INSERT_DATASETMSG);
		fmt_cfg->act1 |= FIELD_PREP(FMT_OPCUA_MSG_CNT, priv->datasetmsg_cnt - 1);
		fmt_cfg->pld_offset = config->message_count_offset & 0x0ff;
	} else {
		goto err;
	}

	fmt_cfg->fmd_eid = fmdt_eid;
	fmt_cfg->fmd_bytes = priv->fmdt_size;

	fmt_eid = netc_fm_alloc_fmt(ntmp);
	if (fmt_eid == NTMP_NULL_ENTRY_ID)
		goto err;

	ret = ntmp_fmt_add_or_update_entry(cbdrs, fmt_eid, true, fmt_cfg);
	if (ret) {
		netc_fm_free_fmt(ntmp, fmt_eid);
		goto err;
	}
	return fmt_eid;

err:
	return NTMP_NULL_ENTRY_ID;
}

static u32 netc_fm_alloc_fmdt(struct ntmp_priv *ntmp, u32 size)
{
	u32 blocks = DIV_ROUND_UP(size, NETC_FM_FMDT_BLOCK_UNIT);
	u32 entry_id;

	entry_id = ntmp_lookup_free_words(ntmp->fmdt_eid_bitmap, ntmp->caps.fmdt_num_blocks,
					  blocks);
	return entry_id;
}

static void netc_fm_free_fmdt(struct ntmp_priv *ntmp, u32 entry_id, u32 size)
{
	u32 blocks = DIV_ROUND_UP(size, NETC_FM_FMDT_BLOCK_UNIT);

	if (entry_id == NTMP_NULL_ENTRY_ID)
		return;

	ntmp_clear_words_bitmap(ntmp->fmdt_eid_bitmap, entry_id, blocks);
}

static u32 netc_fm_set_fmdt(struct netc_fm_priv *priv, u8 *fmdt_data, u32 fmdt_size)
{
	struct netc_cbdrs *cbdrs = priv->cbdrs;
	struct ntmp_priv *ntmp = &priv->netc_switch->ntmp;
	u32 fmdt_eid = NTMP_NULL_ENTRY_ID;
	int ret;

	fmdt_eid = netc_fm_alloc_fmdt(ntmp, fmdt_size);
	if (fmdt_eid == NTMP_NULL_ENTRY_ID) {
		pr_err("Failed to allocate entry id of Frame Modification Data table.");
		goto err;
	}

	ret = ntmp_fmdt_update_entry(cbdrs, fmdt_eid, fmdt_data, fmdt_size);
	if (ret) {
		pr_err("Failed to update Frame Modification Data table (eid=%d, errno=%d).",
		       fmdt_eid, ret);
		netc_fm_free_fmdt(ntmp, fmdt_eid, fmdt_size);
		goto err;
	}
	return fmdt_eid;

err:
	return NTMP_NULL_ENTRY_ID;
}

static void netc_fm_cleanup(struct netc_fm_priv *priv)
{
	struct ntmp_priv *ntmp = &priv->netc_switch->ntmp;
	struct netc_cbdrs *cbdrs = priv->cbdrs;

	if (priv->ipft_eid != NTMP_NULL_ENTRY_ID)
		ntmp_ipft_delete_entry(cbdrs, priv->ipft_eid);

	priv->ipft_eid = NTMP_NULL_ENTRY_ID;

	if (priv->ist_eid != NTMP_NULL_ENTRY_ID) {
		ntmp_ist_delete_entry(cbdrs, priv->ist_eid);
		netc_fm_free_ist(ntmp, priv->ist_eid);
	}
	priv->ist_eid = NTMP_NULL_ENTRY_ID;

	if (priv->ett_eid != NTMP_NULL_ENTRY_ID) {
		ntmp_ett_delete_entry(cbdrs, priv->ett_eid);
		netc_fm_free_ett(ntmp, priv->ett_eid);
	}
	priv->ett_eid = NTMP_NULL_ENTRY_ID;

	if (priv->fmt_eid != NTMP_NULL_ENTRY_ID) {
		ntmp_fmt_delete_entry(cbdrs, priv->fmt_eid);
		netc_fm_free_fmt(ntmp, priv->fmt_eid);
	}
	priv->fmt_eid = NTMP_NULL_ENTRY_ID;

	if (priv->fmdt_eid != NTMP_NULL_ENTRY_ID)
		netc_fm_free_fmdt(ntmp, priv->fmdt_eid, priv->fmdt_size);

	priv->fmdt_eid = NTMP_NULL_ENTRY_ID;
}

static int netc_fm_config(struct netc_fm_priv *priv, struct netc_fm_conf *config)
{
	u32 entry_id;

	memcpy(&priv->config, config, sizeof(*config));

	entry_id = netc_fm_set_ett(priv, NTMP_NULL_ENTRY_ID, config);
	if (entry_id == NTMP_NULL_ENTRY_ID) {
		pr_err("Egress Treatment Table config error!");
		goto err;
	}
	priv->ett_eid = entry_id;

	entry_id = netc_fm_set_ist(priv, priv->ett_eid, config);
	if (entry_id == NTMP_NULL_ENTRY_ID) {
		pr_err("Ingress Stream Table config error!");
		goto err;
	}
	priv->ist_eid = entry_id;

	entry_id = netc_fm_set_ipft(priv, priv->ist_eid, config);
	if (entry_id == NTMP_NULL_ENTRY_ID) {
		pr_err("Ingress Port Filter Table config error!");
		goto err;
	}
	priv->ipft_eid = entry_id;

	return 0;
err:
	netc_fm_cleanup(priv);
	return -EINVAL;
}

static ssize_t netc_switch_fm_write(struct file *filp, const char __user *buffer,
				    size_t count, loff_t *ppos)
{
	struct netc_fm_priv *priv = filp->private_data;
	struct ntmp_priv *ntmp = &priv->netc_switch->ntmp;
	struct netc_cbdrs *cbdrs = priv->cbdrs;
	u32 new_fmdt_eid = NTMP_NULL_ENTRY_ID;
	u32 new_fmt_eid = NTMP_NULL_ENTRY_ID;
	u32 old_fmdt_eid;
	u32 old_fmt_eid;
	size_t old_fmdt_size;
	int ret;

	if (!count || count > NETC_SWITCH_FM_BUF_SIZE)
		return -EINVAL;

	if (!priv->valid) {
		pr_err("One instance has not been created. Call the IOCTL command NETC_FM_CMD_CREATE to create one.");
		return -EINVAL;
	}

	mutex_lock(&netc_switch_fm_mutex);

	if (copy_from_user(priv->fmdt_buffer, buffer, count)) {
		ret = -EFAULT;
		goto err;
	}

	old_fmdt_eid = priv->fmdt_eid;
	old_fmt_eid = priv->fmt_eid;
	old_fmdt_size = priv->fmdt_size;

	priv->datasetmsg_cnt = *((uint8_t *)priv->fmdt_buffer);
	priv->fmdt_size = count - 1;

	new_fmdt_eid = netc_fm_set_fmdt(priv, priv->fmdt_buffer + 1, priv->fmdt_size);
	if (new_fmdt_eid == NTMP_NULL_ENTRY_ID) {
		pr_err("Frame Modification Data Table config error!");
		ret = -EINVAL;
		goto err;
	}

	priv->fmdt_eid = new_fmdt_eid;

	new_fmt_eid = netc_fm_set_fmt(priv, priv->fmdt_eid, &priv->config);
	if (new_fmt_eid == NTMP_NULL_ENTRY_ID) {
		pr_err("Frame Modification Table config error!");
		ret = -EINVAL;
		goto err;
	}
	priv->fmt_eid = new_fmt_eid;

	ret = netc_fm_update_ett(priv);
	if (ret) {
		pr_err("Egress Treatment Table update error!");
		goto err;
	}

	netc_fm_enable_ipft(priv->netc_switch, priv->config.ingress_port);

	ntmp_fmt_delete_entry(cbdrs, old_fmt_eid);
	netc_fm_free_fmt(ntmp, old_fmt_eid);
	netc_fm_free_fmdt(ntmp, old_fmdt_eid, old_fmdt_size);

	mutex_unlock(&netc_switch_fm_mutex);

	return count;

err:
	netc_fm_cleanup(priv);
	mutex_unlock(&netc_switch_fm_mutex);
	return ret;
}

static int netc_switch_fm_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	const char __user *p = (const char __user *)arg;
	struct netc_fm_priv *priv = filp->private_data;
	struct netc_fm_conf conf;
	int ret;

	switch (cmd) {
	case NETC_FM_CMD_CREATE:
		pr_info("Enter NETC_FM_CMD_CREATE ...");
		if (copy_from_user(&conf, p, sizeof(conf))) {
			ret = -EINVAL;
			goto err;
		}

		ret = netc_fm_config(priv, &conf);
		if (ret)
			goto err;

		priv->valid = true;
		break;
	case NETC_FM_CMD_DESTROY:
		pr_info("Enter NETC_FM_CMD_DESTROY ...");

		if (priv->valid) {
			priv->valid = false;
			netc_fm_disable_ipft(priv->netc_switch, priv->config.ingress_port);
			netc_fm_cleanup(priv);
			netc_fm_enable_ipft(priv->netc_switch, priv->config.ingress_port);
		}
		break;
	default:
		ret = -ENOIOCTLCMD;
		goto err;
	}

	return 0;
err:
	return ret;
}

static long netc_switch_fm_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret;

	mutex_lock(&netc_switch_fm_mutex);
	ret = netc_switch_fm_ioctl(filp, cmd, arg);
	mutex_unlock(&netc_switch_fm_mutex);

	return ret;
}

static int netc_switch_fm_open(struct inode *inode, struct file *filp)
{
	struct netc_fm_priv *priv;

	/* Can NOT read */
	if (filp->f_mode & FMODE_READ)
		return -EINVAL;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		goto err;

	memset(priv, 0, sizeof(*priv));

	priv->valid = false;
	priv->netc_switch = netc_switch_g;
	priv->cbdrs = &netc_switch_g->ntmp.cbdrs;

	priv->ipft_eid = NTMP_NULL_ENTRY_ID;
	priv->ist_eid = NTMP_NULL_ENTRY_ID;
	priv->ett_eid = NTMP_NULL_ENTRY_ID;
	priv->fmt_eid = NTMP_NULL_ENTRY_ID;
	priv->fmdt_eid = NTMP_NULL_ENTRY_ID;

	priv->fmdt_buffer = kmalloc(NETC_SWITCH_FM_BUF_SIZE, GFP_KERNEL);
	if (!priv->fmdt_buffer) {
		kfree(priv);
		goto err;
	}

	filp->private_data = priv;
	return 0;
err:
	return -ENOMEM;
}

static int netc_switch_fm_release(struct inode *inode, struct file *filp)
{
	struct netc_fm_priv *priv = filp->private_data;

	netc_fm_cleanup(priv);
	kfree(priv->fmdt_buffer);
	kfree(priv);
	return 0;
}

static const struct file_operations netc_switch_fm_fops = {
	.owner = THIS_MODULE,
	.open = netc_switch_fm_open,
	.release = netc_switch_fm_release,
	.read = NULL,
	.write = netc_switch_fm_write,
	.unlocked_ioctl = netc_switch_fm_unlocked_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

static struct miscdevice netc_switch_fm_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "netc-switch-fm",
	.fops = &netc_switch_fm_fops,
};

static int __init netc_switch_fm_driver_init(void)
{
	struct pci_dev *pdev = NULL;
	int err;

	pdev = pci_get_device(NETC_SWITCH_VENDOR_ID, NETC_SWITCH_DEVICE_ID, NULL);
	if (!pdev) {
		pr_err("pci_get_device error\n");
		err = -ENODEV;
		goto error;
	}

	netc_switch_g = pci_get_drvdata(pdev);
	if (!netc_switch_g) {
		pr_err("pci_get_drvdata error\n");
		err = -ENODEV;
		goto error;
	}

	err = misc_register(&netc_switch_fm_misc);
	if (err < 0) {
		pr_err("misc_register error\n");
		err = -ENODEV;
		goto error;
	}
	return 0;

error:
	return err;
}

static void __exit netc_switch_fm_driver_exit(void)
{
	misc_deregister(&netc_switch_fm_misc);
}

module_init(netc_switch_fm_driver_init);
module_exit(netc_switch_fm_driver_exit);

MODULE_DESCRIPTION("NETC switch Frame Modification driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
