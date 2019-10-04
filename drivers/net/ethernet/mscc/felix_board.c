// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/* Felix Switch driver
 *
 * Copyright 2018-2019 NXP
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/iopoll.h>
#include <net/switchdev.h>
#include <net/sock.h>
#include "ocelot.h"
#include "felix_tsn.h"

#define FELIX_DRV_VER_MAJ 1
#define FELIX_DRV_VER_MIN 0

#define FELIX_DRV_STR	"Felix Switch driver"
#define FELIX_DRV_VER_STR __stringify(FELIX_DRV_VER_MAJ) "." \
			  __stringify(FELIX_DRV_VER_MIN)

#define PCI_DEVICE_ID_FELIX	0xEEF0

/* Switch register block BAR */
#define FELIX_SWITCH_BAR	4

static struct pci_device_id felix_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, PCI_DEVICE_ID_FELIX) },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, felix_ids);

#ifdef CONFIG_MSCC_FELIX_SWITCH_TSN
const struct tsn_ops switch_tsn_ops = {
	.device_init			= felix_tsn_init,
	.qbv_set			= felix_qbv_set,
	.qbv_get			= felix_qbv_get,
	.qbv_get_status			= felix_qbv_get_status,
	.qbu_set			= felix_qbu_set,
	.qbu_get                        = felix_qbu_get,
	.cb_streamid_set		= felix_cb_streamid_set,
	.cb_streamid_get		= felix_cb_streamid_get,
	.qci_sfi_set			= felix_qci_sfi_set,
	.qci_sfi_get			= felix_qci_sfi_get,
	.qci_sfi_counters_get		= felix_qci_sfi_counters_get,
	.qci_sgi_set			= felix_qci_sgi_set,
	.qci_sgi_get			= felix_qci_sgi_get,
	.qci_sgi_status_get		= felix_qci_sgi_status_get,
	.qci_fmi_set			= felix_qci_fmi_set,
	.qci_fmi_get			= felix_qci_fmi_get,
	.cbs_set			= felix_cbs_set,
	.cbs_get			= felix_cbs_get,
	.ct_set				= felix_cut_thru_set,
	.cbgen_set			= felix_seq_gen_set,
	.cbrec_set			= felix_seq_rec_set,
	.cb_get				= felix_cb_get,
	.dscp_set			= felix_dscp_set,
};
#endif

static struct {
	enum ocelot_target id;
	struct resource res;
} felix_io_res[] = {
	{	.id = ANA,
		{
			.start = 0x0280000,
			.end = 0x028ffff,
			.name = "ana",
		}
	},
	{	.id = PTP,
		{
			.start = 0x0090000,
			.end = 0x00900cb,
			.name = "ptp",
		}
	},
	{	.id = QS,
		{
			.start = 0x0080000,
			.end = 0x00800ff,
			.name = "qs",
		}
	},
	{	.id = QSYS,
		{
			.start = 0x0200000,
			.end = 0x021ffff,
			.name = "qsys",
		}
	},
	{	.id = REW,
		{
			.start = 0x0030000,
			.end = 0x003ffff,
			.name = "rew",
		}
	},
	{	.id = SYS,
		{
			.start = 0x0010000,
			.end = 0x001ffff,
			.name = "sys",
		}
	},
	{	.id = S2,
		{
			.start = 0x0060000,
			.end = 0x00603ff,
			.name = "s2",
		}
	},
	{	.id = GCB,
		{
			.start = 0x0070000,
			.end = 0x00701ff,
			.name = "devcpu_gcb",
		}
	}
};

#define FELIX_MAX_NUM_PHY_PORTS	6

#define FELIX_PORT_RES_START	0x0100000
#define FELIX_PORT_RES_SIZE	0x10000

static void __iomem *regs;

/* Felix header bytes length */
#define FELIX_XFH_LEN 16
#define FELIX_MAX_MTU (VLAN_ETH_FRAME_LEN - XFH_LONG_PREFIX_LEN - VLAN_ETH_HLEN)

static inline void felix_set_xfh_field(u64 *efh, u8 nth_bit, u8 w, u16 v)
{
	u8 i = (8 * FELIX_XFH_LEN - nth_bit) >> 6; /* MSB0 dword index */
	u8 bit = nth_bit & GENMASK(5, 0); /* modulo: field start bit index */
	u64 val = v & GENMASK(w - 1, 0);

	efh[i] |= cpu_to_be64(val << bit);
}

static inline u32 felix_get_xfh_field(u64 *efh, u8 nth_bit, u8 w)
{
	u8 i = (8 * FELIX_XFH_LEN - nth_bit) >> 6; /* MSB0 dword index */
	u8 bit = nth_bit & GENMASK(5, 0); /* modulo: field start bit index */

	return (be64_to_cpu(efh[i]) >> bit) & GENMASK(w - 1, 0);
}

#define FELIX_IFH_FIELD(name, bit, w) \
static inline void felix_set_ifh_##name(u64 *ifh, u16 v) \
{ \
	felix_set_xfh_field(ifh, bit, w, v); \
}

#define FELIX_EFH_FIELD(name, bit, w) \
static inline u32 felix_get_efh_##name(u64 *efh) \
{ \
	return felix_get_xfh_field(efh, bit, w); \
}

/* Felix 128bit-value frame injection header:
 *
 * bit 127: bypass the analyzer processing
 * bit 117-125: Rewriter operation command
 * bit 56-61: destination port mask
 * bit 28-29: pop_cnt: 3 disables all rewriting of the frame
 * bit 20-27: cpu extraction queue mask
 */
FELIX_IFH_FIELD(bypass, 127, 1)
FELIX_IFH_FIELD(rew_op, 117, 9)
FELIX_IFH_FIELD(dstp, 56, 6)
FELIX_IFH_FIELD(srcp, 43, 4)
FELIX_IFH_FIELD(popcnt, 28, 2)
FELIX_IFH_FIELD(cpuq, 20, 8)

#define FELIX_IFH_INJ_POP_CNT_DISABLE 3

/* Felix 128bit-value frame extraction header:
 *
 * bit 85-116: rewriter val
 * bit 43-45: source port id
 */
FELIX_EFH_FIELD(rew_val, 85, 32)
FELIX_EFH_FIELD(srcp, 43, 4)

static void felix_tx_hdr_set(struct sk_buff *skb, struct ocelot_port *port)
{
	u64 *ifh = skb_push(skb, FELIX_XFH_LEN);
	struct ocelot *ocelot = port->ocelot;

	/* fill frame injection header */
	memset(ifh, 0x0, FELIX_XFH_LEN);
	felix_set_ifh_bypass(ifh, 1);
	felix_set_ifh_dstp(ifh, BIT(port->chip_port));
	felix_set_ifh_srcp(ifh, ocelot->cpu_port_id);
	felix_set_ifh_popcnt(ifh, 0);
	felix_set_ifh_cpuq(ifh, 0x0);

	if (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP && port->tx_tstamp) {
		u8 id = (port->tstamp_id++) % 4;
		/* REW_OP[2:0] = 0x3 for two-step PTP timestamp
		 * REW_OP[8:3] for timestamp ID
		 */
		felix_set_ifh_rew_op(ifh, 0x3 | id << 3);
	}
}

static netdev_tx_t felix_cpu_inj_handler(struct sk_buff *skb,
					 struct net_device *ndev)
{
	struct ocelot_port *port = netdev_priv(ndev);
	bool do_tstamp = skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP &&
			 port->tx_tstamp;
	struct net_device *pair_ndev;

	pair_ndev = port->cpu_inj_handler_data;

	if (!netif_running(pair_ndev))
		return NETDEV_TX_BUSY;

	if (do_tstamp) {
		struct ocelot_skb *oskb =
			devm_kzalloc(port->ocelot->dev,
				     sizeof(struct ocelot_skb),
				     GFP_KERNEL);
		oskb->skb = skb_clone(skb, GFP_ATOMIC);
		if (skb->sk)
			skb_set_owner_w(oskb->skb, skb->sk);
		oskb->tstamp_id = port->tstamp_id % 4;
		oskb->tx_port = port->chip_port;
		list_add_tail(&oskb->head, &port->ocelot->skbs);

		skb_shinfo(oskb->skb)->tx_flags |= SKBTX_IN_PROGRESS;
	}

	if (unlikely(skb_headroom(skb) < FELIX_XFH_LEN)) {
		struct sk_buff *skb_orig = skb;

		skb = skb_realloc_headroom(skb, FELIX_XFH_LEN);

		/* TODO: free skb in non irq context */
		if (!skb) {
			dev_kfree_skb_any(skb_orig);
			return NETDEV_TX_OK;
		}

		if (skb_orig->sk)
			skb_set_owner_w(skb, skb_orig->sk);

		skb_copy_queue_mapping(skb, skb_orig);
		skb->priority = skb_orig->priority;
#ifdef CONFIG_NET_SCHED
		skb->tc_index = skb_orig->tc_index;
#endif
		dev_consume_skb_any(skb_orig);
	}
	/* add cpu injection header */
	felix_tx_hdr_set(skb, port);

	skb->dev = pair_ndev;
	dev_queue_xmit(skb);

	return NETDEV_TX_OK;
}

static rx_handler_result_t felix_frm_ext_handler(struct sk_buff **pskb)
{
	struct skb_shared_hwtstamps *shhwtstamps;
	struct net_device *ndev = (*pskb)->dev;
	struct ocelot_port *port = NULL;
	struct sk_buff *skb = *pskb;
	char *start = skb->data;
	struct ocelot *ocelot;
	struct timespec64 ts;
	u32 p, ts_lo, ts_hi;
	u64 *efh, ts_ns;

	/* extraction header offset: assume eth header was consumed */
	efh = (u64 *)(start - ETH_HLEN + XFH_LONG_PREFIX_LEN - FELIX_XFH_LEN);

	/* decode src port */
	p = felix_get_efh_srcp(efh);

	ts_lo = felix_get_efh_rew_val(efh);

       /* don't pass frames with unknown header format back to interface */
	if (unlikely(p >= FELIX_MAX_NUM_PHY_PORTS)) {
		kfree_skb(skb);
		return RX_HANDLER_CONSUMED;
	}

	ocelot = rcu_dereference(ndev->rx_handler_data);
	/* get the intf to fwd the frame */
	if (ocelot && p != ocelot->cpu_port_id) {
		port = ocelot->ports[p];
		if (port)
			ndev = port->dev;
	}

	/* pull the rest of extraction header */
	skb_pull(skb, XFH_LONG_PREFIX_LEN - ETH_HLEN);

	/* get the actual protocol type */
	skb->protocol = eth_type_trans(skb, ndev);

	skb_reset_network_header(skb);
	skb_reset_transport_header(skb);
	skb->pkt_type = PACKET_HOST;

	/* remove from inet csum the extraction and eth headers */
	skb_postpull_rcsum(skb, start, XFH_LONG_PREFIX_LEN);

	/* frame for CPU */
	if (ocelot && p == ocelot->cpu_port_id)
		return RX_HANDLER_PASS;

	if (ocelot->bridge_mask & BIT(p))
		skb->offload_fwd_mark = 1;

	if (port && port->rx_tstamp) {
		felix_ptp_gettime(&ocelot->ptp_caps, &ts);
		ts_ns = ktime_set(ts.tv_sec, ts.tv_nsec);

		ts_hi = ts_ns >> 32;
		if ((ts_ns & 0xffffffff) < ts_lo)
			ts_hi = ts_hi - 1;

		ts_ns = ((u64)ts_hi << 32) | ts_lo;

		shhwtstamps = skb_hwtstamps(skb);
		memset(shhwtstamps, 0, sizeof(struct skb_shared_hwtstamps));
		shhwtstamps->hwtstamp = ts_ns;
	}

	netif_rx(skb);

	return RX_HANDLER_CONSUMED;
}

static void felix_register_rx_handler(struct ocelot *ocelot,
				      struct net_device *pair_ndev)
{
	int err = -EBUSY;

	/* must obtain rtnl mutex first */
	rtnl_lock();
	if (netif_device_present(pair_ndev))
		err = netdev_rx_handler_register(pair_ndev,
						 felix_frm_ext_handler, ocelot);
	rtnl_unlock();
	if (err)
		dev_err(ocelot->dev,
			"pair ndev busy: rx_handler not registered\n");
}

static void felix_release_ports(struct ocelot *ocelot)
{
	struct ocelot_port *ocelot_port;
	struct net_device *pair_ndev;
	struct phy_device *phydev;
	struct device_node *dn;
	int i;

	for (i = 0; i < ocelot->num_phys_ports; i++) {
		ocelot_port = ocelot->ports[i];
		if (!ocelot_port || !ocelot_port->phy || !ocelot_port->dev)
			continue;

		phydev = ocelot_port->phy;
#ifdef CONFIG_MSCC_FELIX_SWITCH_TSN
		tsn_port_unregister(ocelot_port->dev);
#endif
		unregister_netdev(ocelot_port->dev);
		free_netdev(ocelot_port->dev);

		if (phy_is_pseudo_fixed_link(phydev)) {
			dn = phydev->mdio.dev.of_node;
			/* decr refcnt: of_phy_register_fixed_link */
			of_phy_deregister_fixed_link(dn);
		}
		phy_device_free(phydev); /* decr refcnt: of_phy_find_device */

		/* unregister cpu port rx handler */
		if (ocelot->cpu_port_id == i) {
			pair_ndev = ocelot_port->cpu_inj_handler_data;
			if (pair_ndev && netif_device_present(pair_ndev)) {
				rtnl_lock();
				netdev_rx_handler_unregister(pair_ndev);
				rtnl_unlock();
			}
		}
	}
}

static void felix_setup_port_inj(struct ocelot_port *port,
				 struct net_device *pair_ndev)
{
	struct ocelot *ocelot = port->ocelot;
	struct net_device *pdev = port->dev;

	if (port->chip_port == ocelot->cpu_port_id) {
		/* expected frame formats on NPI:
		 * short prefix frame tag on tx and long prefix on rx
		 */
		ocelot_write_rix(ocelot, SYS_PORT_MODE_INCL_XTR_HDR(3) |
				 SYS_PORT_MODE_INCL_INJ_HDR(1), SYS_PORT_MODE,
				 port->chip_port);

		/* register rx handler for decoding tagged frames from NPI */
		felix_register_rx_handler(port->ocelot, pair_ndev);
		/* save for cleanup */
		port->cpu_inj_handler_data = pair_ndev;
		ocelot->cpu_port_ndev = port->dev;
	} else {
		/* set frame injection handler on non-NPI ports */
		port->cpu_inj_handler = felix_cpu_inj_handler;
		port->cpu_inj_handler_data = pair_ndev;
		/* no CPU header, only normal frames */
		ocelot_write_rix(ocelot, 0, SYS_PORT_MODE, port->chip_port);
	}

	/* felix configs */
	pdev->max_mtu = FELIX_MAX_MTU;
	pdev->mtu = pdev->max_mtu;
	pdev->needed_headroom = XFH_LONG_PREFIX_LEN;
}

struct net_device *felix_port_get_pair_ndev(struct device_node *np, u32 *port)
{
	struct device_node *ethnp = of_parse_phandle(np, "ethernet", 0);

	if (!ethnp)
		return NULL;

	if (of_property_read_u32(np, "reg", port))
		return NULL;

	return of_find_net_device_by_node(ethnp);
}

static void felix_get_hwtimestamp(struct ocelot *ocelot, struct timespec64 *ts)
{
	/* Read current PTP time to get seconds */
	u32 val = ocelot_read_rix(ocelot, PTP_PIN_CFG, TOD_ACC_PIN);

	val &= ~(PTP_PIN_CFG_SYNC | PTP_PIN_CFG_ACTION_MASK | PTP_PIN_CFG_DOM);
	val |= PTP_PIN_CFG_ACTION(PTP_PIN_ACTION_SAVE);
	ocelot_write_rix(ocelot, val, PTP_PIN_CFG, TOD_ACC_PIN);
	ts->tv_sec = ocelot_read_rix(ocelot, PTP_TOD_SEC_LSB, TOD_ACC_PIN);

	/* Read packet HW timestamp from FIFO */
	val = ocelot_read(ocelot, SYS_PTP_TXSTAMP);
	ts->tv_nsec = SYS_PTP_TXSTAMP_PTP_TXSTAMP(val);

	/* Sec has incremented since the ts was registered */
	if ((ts->tv_sec & 0x1) != !!(val & SYS_PTP_TXSTAMP_PTP_TXSTAMP_SEC))
		ts->tv_sec--;
}

static bool felix_tx_tstamp_avail(struct ocelot *ocelot)
{
	return (!list_empty(&ocelot->skbs)) &&
	       (ocelot_read(ocelot, SYS_PTP_STATUS) &
		SYS_PTP_STATUS_PTP_MESS_VLD);
}

static void felix_tx_clean(struct ocelot *ocelot)
{
	do {
		struct list_head *pos, *tmp;
		struct ocelot_skb *entry;
		struct sk_buff *skb = NULL;
		struct timespec64 ts;
		struct skb_shared_hwtstamps shhwtstamps;
		u32 val, id, port;

		val = ocelot_read(ocelot, SYS_PTP_STATUS);

		id = SYS_PTP_STATUS_PTP_MESS_ID_X(val);
		port = SYS_PTP_STATUS_PTP_MESS_TXPORT_X(val);

		list_for_each_safe(pos, tmp, &ocelot->skbs) {
			entry = list_entry(pos, struct ocelot_skb, head);
			if (entry->tstamp_id != id ||
			    entry->tx_port != port)
				continue;
			skb = entry->skb;

			list_del(pos);
			devm_kfree(ocelot->dev, entry);
		}

		if (likely(skb)) {
			felix_get_hwtimestamp(ocelot, &ts);
			memset(&shhwtstamps, 0, sizeof(shhwtstamps));
			shhwtstamps.hwtstamp = ktime_set(ts.tv_sec, ts.tv_nsec);
			skb_tstamp_tx(skb, &shhwtstamps);

			dev_kfree_skb_any(skb);
		}

		/* Next tstamp */
		ocelot_write(ocelot, SYS_PTP_NXT_PTP_NXT, SYS_PTP_NXT);

	} while (ocelot_read(ocelot, SYS_PTP_STATUS) &
		 SYS_PTP_STATUS_PTP_MESS_VLD);
}

static void felix_preempt_irq_clean(struct ocelot *ocelot)
{
	struct ocelot_port *ocelot_port;
	int port;
	u32 val;

	val = DEV_GMII_MM_STATISTICS_MM_STATUS_PRMPT_ACTIVE_STICKY;
	for (port = 0; port < FELIX_MAX_NUM_PHY_PORTS; port++) {
		ocelot_port = ocelot->ports[port];
		ocelot_port_rmwl(ocelot_port, val, val,
				 DEV_GMII_MM_STATISTICS_MM_STATUS);
	}
}

static void felix_irq_handle_work(struct work_struct *work)
{
	struct ocelot *ocelot = container_of(work, struct ocelot,
					     irq_handle_work);
	struct pci_dev *pdev = container_of(ocelot->dev, struct pci_dev, dev);

	/* The INTB interrupt is used both for 1588 interrupt and
	 * preemption status change interrupt on each port. So check
	 * which interrupt it is, and clean it.
	 */
	if (felix_tx_tstamp_avail(ocelot))
		felix_tx_clean(ocelot);
	else
		felix_preempt_irq_clean(ocelot);

	enable_irq(pdev->irq);
}

static irqreturn_t felix_isr(int irq, void *data)
{
	struct ocelot *ocelot = (struct ocelot *)data;

	disable_irq_nosync(irq);
	queue_work(ocelot->ocelot_wq, &ocelot->irq_handle_work);

	return IRQ_HANDLED;
}

static int felix_ports_init(struct pci_dev *pdev)
{
	struct ocelot *ocelot = pci_get_drvdata(pdev);
	struct device_node *np = ocelot->dev->of_node;
	struct device_node *phy_node, *portnp;
	struct net_device *pair_ndev = NULL;
	struct ocelot_port *ocelot_port;
	struct phy_device *phydev;
	void __iomem *port_regs;
	resource_size_t base;
	u32 port;
	int err;

	ocelot->num_phys_ports = FELIX_MAX_NUM_PHY_PORTS;

	np = of_get_child_by_name(np, "ports");
	if (!np) {
		dev_err(&pdev->dev, "ports sub-node not found\n");
		return -ENODEV;
	}

	/* check if it's cpu port node, to get the pair ndev */
	portnp = of_find_node_with_property(np, "ethernet");
	if (portnp)
		pair_ndev = felix_port_get_pair_ndev(portnp, &port);

	ocelot->cpu_port_id = port;
	if (!pair_ndev) {
		//TODO: Maybe defer probing
		dev_warn(ocelot->dev, "Pair netdev for port %d not found\n",
			 port);
		ocelot->cpu_port_id = FELIX_MAX_NUM_PHY_PORTS;
	}

	/* alloc netdev for each port */
	err = ocelot_init(ocelot);
	if (err)
		return err;

	base = pci_resource_start(pdev, FELIX_SWITCH_BAR);
	for_each_available_child_of_node(np, portnp) {
		struct resource res = {};
		int phy_mode;

		if (!portnp || !portnp->name ||
		    of_node_cmp(portnp->name, "port") ||
		    of_property_read_u32(portnp, "reg", &port))
			continue;

		if (port >= FELIX_MAX_NUM_PHY_PORTS) {
			dev_err(ocelot->dev, "invalid port num: %d\n", port);
			continue;
		}

		res.start = base + FELIX_PORT_RES_START +
			    FELIX_PORT_RES_SIZE * port;
		res.end = res.start + FELIX_PORT_RES_SIZE - 1;
		res.flags = IORESOURCE_MEM;
		res.name = "port";

		port_regs = devm_ioremap_resource(ocelot->dev, &res);
		if (IS_ERR(port_regs)) {
			dev_err(ocelot->dev,
				"failed to map registers for port %d\n", port);
			continue;
		}

		phy_node = of_parse_phandle(portnp, "phy-handle", 0);
		if (!phy_node) {
			if (!of_phy_is_fixed_link(portnp))
				continue;

			err = of_phy_register_fixed_link(portnp);
			if (err < 0) {
				dev_err(ocelot->dev,
					"can't create fixed link for port:%d\n",
					port);
				continue;
			}
			phydev = of_phy_find_device(portnp);
		} else {
			phydev = of_phy_find_device(phy_node);
		}

		of_node_put(phy_node);


		phy_mode = of_get_phy_mode(portnp);
		if (phy_mode < 0)
			phy_mode = PHY_INTERFACE_MODE_NA;

		err = ocelot_probe_port(ocelot, port, port_regs, phydev);
		if (err) {
			dev_err(ocelot->dev, "failed to probe ports\n");
			goto release_ports;
		}

		/* Felix configs */
		ocelot_port = ocelot->ports[port];
		ocelot_port->phy_mode = phy_mode;
		ocelot_port->portnp = portnp;

		if (pair_ndev)
			felix_setup_port_inj(ocelot_port, pair_ndev);

#ifdef CONFIG_MSCC_FELIX_SWITCH_TSN
		tsn_port_register(ocelot_port->dev,
				  (struct tsn_ops *)&switch_tsn_ops,
				  (u16)pdev->bus->number + GROUP_OFFSET_SWITCH);
#endif
	}

	/* set port for external CPU frame extraction/injection */
	if (pair_ndev)
		ocelot_write(ocelot, QSYS_EXT_CPU_CFG_EXT_CPUQ_MSK_M |
			     QSYS_EXT_CPU_CFG_EXT_CPU_PORT(ocelot->cpu_port_id),
			     QSYS_EXT_CPU_CFG);

	return 0;

release_ports:
	felix_release_ports(ocelot);

	return err;
}

#define FELIX_INIT_TIMEOUT	50000
#define FELIX_GCB_RST_SLEEP	100
#define FELIX_SYS_RAMINIT_SLEEP	80

static int felix_gcb_soft_rst_status(struct ocelot *ocelot)
{
	int val;

	regmap_field_read(ocelot->regfields[GCB_SOFT_RST_SWC_RST], &val);
	return val;
}

static int felix_sys_ram_init_status(struct ocelot *ocelot)
{
	return ocelot_read(ocelot, SYS_RAM_INIT);
}

static int felix_init_switch_core(struct ocelot *ocelot)
{
	int val, err;

	/* soft-reset the switch core */
	regmap_field_write(ocelot->regfields[GCB_SOFT_RST_SWC_RST], 1);

	err = readx_poll_timeout(felix_gcb_soft_rst_status, ocelot, val, !val,
				 FELIX_GCB_RST_SLEEP, FELIX_INIT_TIMEOUT);
	if (err) {
		dev_err(ocelot->dev, "timeout: switch core reset\n");
		return err;
	}

	/* initialize switch mem ~40us */
	ocelot_write(ocelot, SYS_RAM_INIT_RAM_INIT, SYS_RAM_INIT);
	err = readx_poll_timeout(felix_sys_ram_init_status, ocelot, val, !val,
				 FELIX_SYS_RAMINIT_SLEEP, FELIX_INIT_TIMEOUT);
	if (err) {
		dev_err(ocelot->dev, "timeout: switch sram init\n");
		return err;
	}

	/* enable switch core */
	regmap_field_write(ocelot->regfields[SYS_RESET_CFG_CORE_ENA], 1);

	return 0;
}

static int felix_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct ocelot *ocelot;
	resource_size_t base;
	size_t len;
	int i, err;

	if (pdev->dev.of_node && !of_device_is_available(pdev->dev.of_node)) {
		dev_info(&pdev->dev, "device is disabled, skipping\n");
		return -ENODEV;
	}

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "device enable failed\n");
		return err;
	}

	/* set up for high or low dma */
	err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (err) {
		err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
		if (err) {
			dev_err(&pdev->dev,
				"DMA configuration failed: 0x%x\n", err);
			goto err_dma;
		}
	}

	base = pci_resource_start(pdev, FELIX_SWITCH_BAR);

	pci_set_master(pdev);

	ocelot = devm_kzalloc(&pdev->dev, sizeof(*ocelot), GFP_KERNEL);
	if (!ocelot) {
		err = -ENOMEM;
		goto err_alloc_ocelot;
	}

	pci_set_drvdata(pdev, ocelot);
	ocelot->dev = &pdev->dev;

	err = request_irq(pdev->irq, &felix_isr, 0, "felix-intb", ocelot);
	if (err)
		goto err_alloc_irq;

	ocelot->ocelot_wq = alloc_workqueue("ocelot_wq", 0, 0);
	if (!ocelot->ocelot_wq) {
		err = -ENOMEM;
		goto err_alloc_wq;
	}

	INIT_WORK(&ocelot->irq_handle_work, felix_irq_handle_work);

	INIT_LIST_HEAD(&ocelot->skbs);

	len = pci_resource_len(pdev, FELIX_SWITCH_BAR);
	if (!len) {
		err = -EINVAL;
		goto err_resource_len;
	}

	regs = pci_iomap(pdev, FELIX_SWITCH_BAR, len);
	if (!regs) {
		err = -ENXIO;
		dev_err(&pdev->dev, "ioremap() failed\n");
		goto err_iomap;
	}

	for (i = 0; i < ARRAY_SIZE(felix_io_res); i++) {
		struct resource *res = &felix_io_res[i].res;
		struct regmap *target;

		res->flags = IORESOURCE_MEM;
		res->start += base;
		res->end += base;

		target = ocelot_io_init(ocelot, res);
		if (IS_ERR(target)) {
			err = PTR_ERR(target);
			goto err_iomap;
		}

		ocelot->targets[felix_io_res[i].id] = target;
	}

	err = felix_chip_init(ocelot);
	if (err)
		goto err_chip_init;

	err = felix_init_switch_core(ocelot);
	if (err)
		goto err_sw_core_init;

	err = felix_ptp_init(ocelot);
	if (err)
		goto err_ptp_init;

	err = felix_ports_init(pdev);
	if (err)
		goto err_ports_init;

	register_netdevice_notifier(&ocelot_netdevice_nb);

	dev_info(&pdev->dev, "%s v%s\n", FELIX_DRV_STR, FELIX_DRV_VER_STR);
	return 0;

err_ports_init:
err_ptp_init:
err_chip_init:
err_sw_core_init:
	pci_iounmap(pdev, regs);
err_iomap:
err_resource_len:
	destroy_workqueue(ocelot->ocelot_wq);
err_alloc_wq:
	free_irq(pdev->irq, ocelot);
err_alloc_irq:
err_alloc_ocelot:
err_dma:
	pci_disable_device(pdev);

	return err;
}

static void felix_pci_remove(struct pci_dev *pdev)
{
	struct ocelot *ocelot;

	ocelot = pci_get_drvdata(pdev);

	/* stop workqueue thread */
	ocelot_deinit(ocelot);
	free_irq(pdev->irq, ocelot);
	unregister_netdevice_notifier(&ocelot_netdevice_nb);

	felix_release_ports(ocelot);
	felix_ptp_remove(ocelot);

	pci_iounmap(pdev, regs);
	destroy_workqueue(ocelot->ocelot_wq);
	pci_disable_device(pdev);
}

static struct pci_driver felix_pci_driver = {
	.name = KBUILD_MODNAME,
	.id_table = felix_ids,
	.probe = felix_pci_probe,
	.remove = felix_pci_remove,
};

module_pci_driver(felix_pci_driver);

MODULE_DESCRIPTION(FELIX_DRV_STR);
MODULE_LICENSE("Dual MIT/GPL");
