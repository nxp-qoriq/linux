/* SPDX-License-Identifier: (GPL-2.0 OR MIT)
 *
 * TSN_SWITCH driver
 *
 * Copyright 2018-2019 NXP
 */
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/sys_soc.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/phy.h>
#include "ocelot.h"
#include "tsn_switch.h"

/* Round x divided by y to nearest higher integer. x and y are integers */
#define MSCC_DIV_ROUND_UP(x, y) (((x) + (y) - 1) / (y))
#define SE_IX_PORT 64
#define MSCC_QOS_DSCP_MAX 63
#define MSCC_QOS_DP_MAX 1
#ifndef MIN
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#endif
#ifndef MAX
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#endif

static void ocelot_port_rmwl(struct ocelot_port *port, u32 val, u32 mask, u32 reg)
{
	u32 cur = ocelot_port_readl(port, reg);
	ocelot_port_writel(port, (cur & (~mask)) | val, reg);
}

static int qos_port_tas_gcl_set(struct net_device *ndev,
				struct ocelot *ocelot, const u8 gcl_ix,
				struct tsn_qbv_entry *control_list,
				const bool dry_run)
{
	netdev_dbg(ndev, "%s: gcl_ix %u gate_state 0x%02x time_interval %u dry_run %d\n",
		   __func__, gcl_ix, control_list->gate_state,
		   control_list->time_interval, dry_run);
	if (gcl_ix > (SWITCH_TAS_GCL_MAX - 1)) {
		netdev_info(ndev, "Invalid gcl ix %u\n", gcl_ix);
		return -EINVAL;
	}
	if (!control_list->time_interval ||
	    (control_list->time_interval > 1000000000)) {
		netdev_info(ndev, "Invalid time_interval %u\n",
			    control_list->time_interval);
		return -EINVAL;
	}
	if (dry_run)
		return 0;

	ocelot_write(ocelot,
		     QSYS_GCL_CFG_REG_1_GCL_ENTRY_NUM(gcl_ix) |
		     QSYS_GCL_CFG_REG_1_GATE_STATE(control_list->gate_state),
		     QSYS_GCL_CFG_REG_1);

	ocelot_write(ocelot,
		     control_list->time_interval,
		     QSYS_GCL_CFG_REG_2);

	return 0;
}

int switch_qbv_set(struct net_device *ndev, struct tsn_qbv_conf *shaper_config)
{
	struct ocelot_port *port = netdev_priv(ndev);
	struct ocelot *ocelot = port->ocelot;
	int  i, count;
	struct tsn_qbv_basic *admin_basic = &shaper_config->admin;
	struct tsn_qbv_entry *control_list = admin_basic->control_list;
	u32 base_time_nsec = admin_basic->base_time & 0xffffffff;
	u64 base_time_sec = admin_basic->base_time >> 32;
	u64 cur_time;
	u32 val;

	shaper_config->config_change = 1;
	netdev_dbg(ndev, "%s: gate_enabled %d admin_gate_states 0x%02x admin_control_list_length %u admin_cycle_time %u admin_cycle_time_extension %u admin_base_time %llu config_change %d ",
		   __func__, shaper_config->gate_enabled,
		   admin_basic->gate_states,
		   admin_basic->control_list_length,
		   admin_basic->cycle_time,
		   admin_basic->cycle_time_extension,
		   admin_basic->base_time, shaper_config->config_change);

	if (admin_basic->control_list_length > SWITCH_TAS_GCL_MAX) {
		netdev_info(ndev, "Invalid admin_control_list_length %u\n",
			    admin_basic->control_list_length);
		return -EINVAL;
	}

	if (((admin_basic->cycle_time < SWITCH_TAS_CT_MIN) ||
	    (admin_basic->cycle_time > SWITCH_TAS_CT_MAX)) &&
	    shaper_config->gate_enabled) {
		netdev_info(ndev, "Invalid admin_cycle_time %u ns\n",
			    admin_basic->cycle_time);
		return -EINVAL;
	}
	if (admin_basic->cycle_time_extension > SWITCH_TAS_CTE_MAX) {
		netdev_info(ndev, "Invalid admin_cycle_time_extension %u\n",
			    admin_basic->cycle_time_extension);
		return -EINVAL;
	}
	if (base_time_nsec > 999999999) {
		netdev_info(ndev, "Invalid admin_base_time.tv_nsec %u\n",
			    base_time_nsec);
		return -EINVAL;
	}

	cur_time = ocelot_read(ocelot, PTP_CUR_SEC_MSB);
	cur_time = cur_time << 32;
	cur_time += ocelot_read(ocelot, PTP_CUR_SEC_LSB);

	if (base_time_sec < cur_time) {
		base_time_sec = cur_time;
		base_time_nsec = ocelot_read(ocelot, PTP_CUR_NSEC);
	}

	/* Select port */
	ocelot_rmw(ocelot,
		   QSYS_TAS_PARAM_CFG_CTRL_PORT_NUM(port->chip_port),
		   QSYS_TAS_PARAM_CFG_CTRL_PORT_NUM_M,
		   QSYS_TAS_PARAM_CFG_CTRL);

	val = ocelot_read(ocelot, QSYS_PARAM_STATUS_REG_8);
	if (val & QSYS_PARAM_STATUS_REG_8_CONFIG_PENDING) {
		ocelot_rmw_rix(ocelot, 0, QSYS_TAG_CONFIG_ENABLE,
			       QSYS_TAG_CONFIG, port->chip_port);
	}

	ocelot_rmw_rix(ocelot,
		    (shaper_config->gate_enabled ? QSYS_TAG_CONFIG_ENABLE : 0) |
		    QSYS_TAG_CONFIG_INIT_GATE_STATE(admin_basic->gate_states) |
		    QSYS_TAG_CONFIG_SCH_TRAFFIC_QUEUES(0xff) |
		    QSYS_TAG_CONFIG_LINK_SPEED(0x1),
		    QSYS_TAG_CONFIG_ENABLE |
		    QSYS_TAG_CONFIG_INIT_GATE_STATE_M |
		    QSYS_TAG_CONFIG_SCH_TRAFFIC_QUEUES_M |
		    QSYS_TAG_CONFIG_LINK_SPEED_M,
		    QSYS_TAG_CONFIG,
		    port->chip_port);

	ocelot_write_rix(ocelot, shaper_config->maxsdu,
			  QSYS_PORT_MAX_SDU, port->chip_port);
	/* TODO: add queue max SDU set */

	if (shaper_config->gate_enabled && shaper_config->config_change) {
		ocelot_write(ocelot, base_time_nsec,
			     QSYS_PARAM_CFG_REG_1);

		ocelot_write(ocelot, base_time_sec & GENMASK(31, 0),
			     QSYS_PARAM_CFG_REG_2);

		ocelot_write(ocelot,
			     QSYS_PARAM_CFG_REG_3_BASE_TIME_SEC_MSB(base_time_sec >> 32) |
			     QSYS_PARAM_CFG_REG_3_LIST_LENGTH(admin_basic->control_list_length),
			     QSYS_PARAM_CFG_REG_3);
		ocelot_write(ocelot, admin_basic->cycle_time,
			     QSYS_PARAM_CFG_REG_4);

		ocelot_write(ocelot,
			     admin_basic->cycle_time_extension,
			     QSYS_PARAM_CFG_REG_5);

		for (i = 0; i < admin_basic->control_list_length; i++) {
			qos_port_tas_gcl_set(ndev, ocelot, i,
					     control_list, 0);
			control_list++;
		}

		/* Start configuration change */
		ocelot_rmw(ocelot,
			   QSYS_TAS_PARAM_CFG_CTRL_CONFIG_CHANGE,
			   QSYS_TAS_PARAM_CFG_CTRL_CONFIG_CHANGE,
			   QSYS_TAS_PARAM_CFG_CTRL);

		count = 0;
		do { /* Wait until configuration change is complete */
			msleep(20);
			count++;
			if (count == 100) {
				netdev_err(ndev, "Config change timeout on chip_port %u\n", port->chip_port);
				return -ETIMEDOUT;
			}
		} while (ocelot_read(ocelot, QSYS_TAS_PARAM_CFG_CTRL) &
			 QSYS_TAS_PARAM_CFG_CTRL_CONFIG_CHANGE);
	}
	return 0;
}

int switch_qbv_get(struct net_device *ndev, struct tsn_qbv_conf *shaper_config)
{
	struct ocelot_port *port = netdev_priv(ndev);
	struct ocelot *ocelot = port->ocelot;
	u32 val, reg;
	int i;
	u8 p_num = port->chip_port;
	u32 base_timel;
	u32 base_timeh;
	struct tsn_qbv_basic *admin = &shaper_config->admin;
	struct tsn_qbv_entry *list;

	ocelot_field_write(ocelot,
			   QSYS_TAS_PARAM_CFG_CTRL_PORT_NUM_0, p_num);

	val = ocelot_read(ocelot, QSYS_TAG_CONFIG);
	if (val & QSYS_TAG_CONFIG_ENABLE)
		shaper_config->gate_enabled = TRUE;
	else
		shaper_config->gate_enabled = FALSE;

	admin->gate_states = QSYS_TAG_CONFIG_INIT_GATE_STATE_X(val);

	base_timel = ocelot_read(ocelot, QSYS_PARAM_CFG_REG_1);
	base_timeh = ocelot_read(ocelot, QSYS_PARAM_CFG_REG_2);
	reg = ocelot_read(ocelot, QSYS_PARAM_CFG_REG_3);
	admin->base_time = base_timeh | (((u64)QSYS_PARAM_CFG_REG_3_BASE_TIME_SEC_MSB(reg)) << 32);
	admin->base_time = (admin->base_time << 32) | base_timel;

	admin->control_list_length = QSYS_PARAM_CFG_REG_3_LIST_LENGTH_X(reg);

	admin->cycle_time = ocelot_read(ocelot, QSYS_PARAM_CFG_REG_4);
	admin->cycle_time_extension = ocelot_read(ocelot, QSYS_PARAM_CFG_REG_5);

	list = (struct tsn_qbv_entry *)kmalloc(admin->control_list_length *
			sizeof(struct tsn_qbv_entry), GFP_KERNEL);
	admin->control_list = list;

	for (i = 0; i < admin->control_list_length; i++) {
		ocelot_field_write(ocelot,
				   QSYS_GCL_CFG_REG_1_GCL_ENTRY_NUM_0, i);
		list->time_interval = ocelot_read(ocelot, QSYS_GCL_CFG_REG_2);
		reg = ocelot_read(ocelot, QSYS_GCL_CFG_REG_1);
		list->gate_state = QSYS_GCL_CFG_REG_1_GATE_STATE_X(reg);

		list++;
	}

	return 0;
}

void get_operparam(struct ocelot *ocelot, struct tsn_qbv_basic *oper)
{
	u32 base_timel;
	u32 base_timeh;
	u32 val;
	struct tsn_qbv_entry *glist;
	int i;

	base_timel = ocelot_read(ocelot, QSYS_PARAM_STATUS_REG_1);
	base_timeh = ocelot_read(ocelot, QSYS_PARAM_STATUS_REG_2);
	val = ocelot_read(ocelot, QSYS_PARAM_STATUS_REG_3);
	oper->base_time = base_timeh;
	oper->base_time += ((u64)QSYS_PARAM_STATUS_REG_3_BASE_TIME_SEC_MSB(val)) << 32;
	oper->base_time = (oper->base_time << 32) | base_timel;

	oper->control_list_length = QSYS_PARAM_STATUS_REG_3_LIST_LENGTH_X(val);
	oper->cycle_time = ocelot_read(ocelot, QSYS_PARAM_STATUS_REG_4);
	oper->cycle_time_extension = ocelot_read(ocelot,
						 QSYS_PARAM_STATUS_REG_5);

	val = ocelot_read(ocelot, QSYS_PARAM_STATUS_REG_8);
	oper->gate_states = QSYS_PARAM_STATUS_REG_8_OPER_GATE_STATE_X(val);

	glist = (struct tsn_qbv_entry *)kmalloc(oper->control_list_length *
		sizeof(struct tsn_qbv_entry), GFP_KERNEL);

	oper->control_list = glist;

	for (i = 0; i < oper->control_list_length; i++) {
		ocelot_field_write(ocelot,
				   QSYS_GCL_STATUS_REG_1_GCL_ENTRY_NUM_0, i);
		val = ocelot_read(ocelot, QSYS_GCL_STATUS_REG_2);
		oper->control_list->time_interval = val;
		val = ocelot_read(ocelot, QSYS_GCL_STATUS_REG_1);
		glist->gate_state = QSYS_GCL_STATUS_REG_1_GATE_STATE_X(val);

		glist++;
	}
}

int switch_qbv_get_status(struct net_device *ndev,
			  struct tsn_qbv_status *qbvstatus)
{
	struct ocelot_port *port = netdev_priv(ndev);
	struct ocelot *ocelot = port->ocelot;
	struct tsn_qbv_basic *oper = &qbvstatus->oper;
	u8 p_num = port->chip_port;
	u32 val;
	ptptime_t cur_time;

	ocelot_field_write(ocelot, QSYS_TAS_PARAM_CFG_CTRL_PORT_NUM_0, p_num);

	qbvstatus->supported_list_max = 64;

	val = ocelot_read(ocelot, QSYS_PARAM_STATUS_REG_8);
	if (val & QSYS_PARAM_STATUS_REG_8_CONFIG_PENDING)
		qbvstatus->config_pending = TRUE;
	else
		qbvstatus->config_pending = FALSE;

	qbvstatus->config_change_time = ocelot_read(ocelot,
						    QSYS_PARAM_STATUS_REG_7);
	qbvstatus->config_change_time += ((u64)QSYS_PARAM_STATUS_REG_8_CFG_CHG_TIME_SEC_MSB(val)) << 32;
	qbvstatus->config_change_time = (qbvstatus->config_change_time << 32) |
					ocelot_read(ocelot,
						    QSYS_PARAM_STATUS_REG_6);

	qbvstatus->config_change_error = ocelot_read(ocelot,
						     QSYS_PARAM_STATUS_REG_9);

	cur_time = ocelot_read(ocelot, PTP_CUR_SEC_MSB);
	cur_time = cur_time << 32;
	cur_time += ocelot_read(ocelot, PTP_CUR_SEC_LSB);
	cur_time = (cur_time << 32) | ocelot_read(ocelot, PTP_CUR_NSEC);
	qbvstatus->current_time = cur_time;
	get_operparam(ocelot, oper);

	return 0;
}

int switch_cut_thru_set(struct net_device *ndev, u8 cut_thru)
{
	struct ocelot_port *ocelot_port = netdev_priv(ndev);
	struct ocelot *ocelot = ocelot_port->ocelot;
	ocelot_write_rix(ocelot, cut_thru, ANA_CUT_THRU_CFG,
			 ocelot_port->chip_port);

	return 0;
}


int qos_shaper_conf_set(struct net_device *ndev, u32 port_ix, u8 percent)
{
	struct ocelot_port *ocelot_port = netdev_priv(ndev);
	struct ocelot *ocelot = ocelot_port->ocelot;
	u32 cbs = 0;
	u32 cir = 0;

	switch (ndev->phydev->speed) {
	case SPEED_10:
		cir = 10000;
		break;
	case SPEED_100:
		cir = 100000;
		break;
	case SPEED_1000:
		cir = 1000000;
		break;
	case SPEED_2500:
		cir = 2500000;
		break;
	}

	cir = cir * percent / 100;
	cir = MSCC_DIV_ROUND_UP(cir, 100);  /* Rate unit is 100 kbps */
	cir = (cir ? cir : 1);                    /* Avoid using zero rate */
	cbs = MSCC_DIV_ROUND_UP(cbs, 4096); /* Burst unit is 4kB */
	cbs = (cbs ? cbs : 1);                    /* Avoid using zero burst size */
	cir = MIN(GENMASK(15, 0), cir);
	cbs = MIN(GENMASK(6, 0), cbs);
	ocelot_write_gix(ocelot,
			 QSYS_CIR_CFG_CIR_RATE(cir) |
			 QSYS_CIR_CFG_CIR_BURST(cbs),
			 QSYS_CIR_CFG,
			 port_ix);

	return 0;
}

int switch_cbs_set(struct net_device *ndev, u8 tc, u8 bw)
{
	struct ocelot_port *ocelot_port = netdev_priv(ndev);
	struct ocelot *ocelot = ocelot_port->ocelot;

	qos_shaper_conf_set(ndev, ocelot_port->chip_port * 8 + tc, bw);

	ocelot_rmw_gix(ocelot,
		       QSYS_SE_CFG_SE_AVB_ENA,
		       QSYS_SE_CFG_SE_AVB_ENA,
		       QSYS_SE_CFG,
		       ocelot_port->chip_port * 8 + tc);

	return 0;

}

int switch_port_shaper_set(struct net_device *ndev, u8 tc, u8 bw)
{
	struct ocelot_port *ocelot_port = netdev_priv(ndev);
	struct ocelot *ocelot = ocelot_port->ocelot;
	u8 *weight = ocelot_port->cbs_weight;
	int i;
	int percent = 100;
	u8 w_min = 100;
	u32 c_max = 1 << 5;

	qos_shaper_conf_set(ndev, SE_IX_PORT + ocelot_port->chip_port,
			    percent);

	ocelot_rmw_gix(ocelot,
		       QSYS_SE_CFG_SE_DWRR_CNT(7) |
		       QSYS_SE_CFG_SE_AVB_ENA,
		       QSYS_SE_CFG_SE_DWRR_CNT_M |
		       QSYS_SE_CFG_SE_AVB_ENA,
		       QSYS_SE_CFG,
		       SE_IX_PORT + ocelot_port->chip_port);

	weight[tc] = bw;
	for (i = 0; i < NUM_MSCC_QOS_PRIO; i++) {
		if (weight[i])
			/* Find the lowest weight */
			w_min = MIN(w_min, weight[i]);
	}
	for (i = 0; i < NUM_MSCC_QOS_PRIO; i++) {
		if (weight[i]) {
			u32 c = (((c_max << 4) * w_min / weight[i]) + 8) >> 4;
			c = MAX(1, c) - 1;
			ocelot_write_ix(ocelot, c,
					QSYS_SE_DWRR_CFG,
					SE_IX_PORT + ocelot_port->chip_port,
					i);
		} else {
			ocelot_write_ix(ocelot, 0,
					QSYS_SE_DWRR_CFG,
					SE_IX_PORT + ocelot_port->chip_port,
					i);
		}
	}

	return 0;
}

int switch_qbu_set(struct net_device *ndev, u8 preemptable)
{
	struct ocelot_port *ocelot_port = netdev_priv(ndev);
	struct ocelot *ocelot = ocelot_port->ocelot;

	ocelot_port_rmwl(ocelot_port,
		   DEV_GMII_MM_CONFIG_ENABLE_CONFIG_MM_RX_ENA |
		   DEV_GMII_MM_CONFIG_ENABLE_CONFIG_MM_TX_ENA,
		   DEV_GMII_MM_CONFIG_ENABLE_CONFIG_MM_RX_ENA |
		   DEV_GMII_MM_CONFIG_ENABLE_CONFIG_MM_TX_ENA,
		   DEV_GMII_MM_CONFIG_ENABLE_CONFIG);

	ocelot_rmw_rix(ocelot,
		       QSYS_PREEMPTION_CFG_P_QUEUES(preemptable),
		       QSYS_PREEMPTION_CFG_P_QUEUES_M,
		       QSYS_PREEMPTION_CFG,
		       ocelot_port->chip_port);

	return 0;
}

int switch_qbu_get(struct net_device *ndev, struct tsn_preempt_status *c)
{
	struct ocelot_port *ocelot_port = netdev_priv(ndev);
	struct ocelot *ocelot = ocelot_port->ocelot;
	u32 val;

	val = ocelot_read_rix(ocelot,
			      QSYS_PREEMPTION_CFG,
			      ocelot_port->chip_port);

	c->admin_state = QSYS_PREEMPTION_CFG_P_QUEUES(val);
	c->hold_advance = QSYS_PREEMPTION_CFG_HOLD_ADVANCE_X(val);

	val = ocelot_port_readl(ocelot_port,
				DEV_GMII_MM_STATISTICS_MM_STATUS);
	c->preemption_active =
		DEV_GMII_MM_STATISTICS_MM_STATUS_PRMPT_ACTIVE_STATUS & val;

	return 0;
}

/* Qci */
int switch_cb_streamid_get(struct net_device *ndev, u32 index,
			   struct tsn_cb_streamid *streamid)
{
	struct ocelot_port *ocelot_port = netdev_priv(ndev);
	struct ocelot *ocelot = ocelot_port->ocelot;

	u32 m_index = index / 4;
	u32 bucket =  index % 4;
	u32 val, dst, reg;
	u64 dmac;
	u32 ldmac, hdmac;

	streamid->type = 1;
	regmap_field_write(ocelot->regfields[ANA_TABLES_MACTINDX_BUCKET],
			   bucket);
	regmap_field_write(ocelot->regfields[ANA_TABLES_MACTINDX_M_INDEX],
			   m_index);

	/*READ command MACACCESS.VALID(11 bit) must be 0 */
	ocelot_write(ocelot,
		     ANA_TABLES_MACACCESS_MAC_TABLE_CMD(MACACCESS_CMD_READ) ,
		     ANA_TABLES_MACACCESS);

	val = ocelot_read(ocelot, ANA_TABLES_MACACCESS);
	dst = (val & ANA_TABLES_MACACCESS_DEST_IDX_M) >> 3;
	reg = ocelot_read_rix(ocelot, ANA_PGID_PGID, dst);
	streamid->ofac_oport = ANA_PGID_PGID_PGID(reg);

	/*Get the entry's MAC address and VLAN id*/
	ldmac = ocelot_read(ocelot, ANA_TABLES_MACLDATA);
	val = ocelot_read(ocelot, ANA_TABLES_MACHDATA);
	val &= 0x1fffffff;
	hdmac = val & 0xffff;
	dmac = hdmac;
	dmac = (dmac << 32) | ldmac;
	streamid->para.nid.dmac = dmac;

	streamid->para.nid.vid = ANA_TABLES_MACHDATA_VID_X(val);

	val = ocelot_read(ocelot, ANA_TABLES_STREAMDATA);
	if (!(val & ANA_TABLES_STREAMDATA_SFID_VALID))
		return -EINVAL;

	streamid->handle = ANA_TABLES_STREAMDATA_SSID_X(val);

	return 0;
}
u32 lookup_pgid(u32 mask, struct ocelot *ocelot)
{
	int i;
	u32 val, port_mask;

	for (i = 0; i < PGID_AGGR; i++) {
		val = ocelot_read_rix(ocelot, ANA_PGID_PGID, i);
		port_mask = ANA_PGID_PGID_PGID(val);
		if (mask == port_mask)
			return i;
	}
	if (i == PGID_AGGR)
		return PGID_UC;

	return 0;
}
int switch_cb_streamid_set(struct net_device *ndev, u32 index, bool enable,
			   struct tsn_cb_streamid *streamid)
{
	struct ocelot_port *port = netdev_priv(ndev);
	struct ocelot *ocelot = port->ocelot;
	u32 macl, mach;
	u16 vid;
	u64 mac;
	u32 dst_idx;
	int sfid, ssid;
	u32 m_index = index / 4;
	u32 bucket =  index % 4;

	regmap_field_write(ocelot->regfields[ANA_TABLES_MACTINDX_BUCKET],
			   bucket);
	regmap_field_write(ocelot->regfields[ANA_TABLES_MACTINDX_M_INDEX],
			   m_index);
	if (streamid->type != 1)
		return -EINVAL;

	if (enable == TRUE) {
		netdev_dbg(ndev, "index=%d mac=0x%llx vid=0x%x sfid=%d dst=%d\n",
			   index, streamid->para.nid.dmac,
			   streamid->para.nid.vid,
			   streamid->handle,
			   port->chip_port);

		mac = streamid->para.nid.dmac;
		macl = mac & 0xffffffff;
		mach = (mac >> 32) & 0xffff;
		vid = streamid->para.nid.vid;
		ocelot_write(ocelot, macl, ANA_TABLES_MACLDATA);
		ocelot_write(ocelot, ANA_TABLES_MACHDATA_VID(vid) |
			     ANA_TABLES_MACHDATA_MACHDATA(mach),
			     ANA_TABLES_MACHDATA);

		sfid = streamid->handle * 2;
		ssid = streamid->handle;
		ocelot_write(ocelot,
			     ANA_TABLES_STREAMDATA_SFID_VALID |
			     ANA_TABLES_STREAMDATA_SFID(sfid) |
			     ANA_TABLES_STREAMDATA_SSID_VALID |
			     ANA_TABLES_STREAMDATA_SSID(ssid),
			     ANA_TABLES_STREAMDATA);

		dst_idx = port->chip_port;
		ocelot_write(ocelot, ANA_TABLES_MACACCESS_VALID |
			     ANA_TABLES_MACACCESS_ENTRYTYPE(1) |
			     ANA_TABLES_MACACCESS_DEST_IDX(dst_idx) |
			     ANA_TABLES_MACACCESS_MAC_TABLE_CMD(MACACCESS_CMD_WRITE),
			     ANA_TABLES_MACACCESS);

		ocelot_write(ocelot, ANA_TABLES_MACACCESS_VALID |
			     ANA_TABLES_MACACCESS_ENTRYTYPE(1) |
			     ANA_TABLES_MACACCESS_DEST_IDX(dst_idx) |
			     ANA_TABLES_MACACCESS_MAC_TABLE_CMD(MACACCESS_CMD_LEARN),
			     ANA_TABLES_MACACCESS);
	} else {
		netdev_info(ndev, "disable stream set\n");
		mac = streamid->para.nid.dmac;
		macl = mac & 0xffffffff;
		mach = (mac >> 32) & 0xffff;
		vid = streamid->para.nid.vid;
		ocelot_write(ocelot, macl, ANA_TABLES_MACLDATA);
		ocelot_write(ocelot, ANA_TABLES_MACHDATA_VID(vid) |
			     ANA_TABLES_MACHDATA_MACHDATA(mach),
			     ANA_TABLES_MACHDATA);

		ocelot_write(ocelot,
			     ANA_TABLES_MACACCESS_MAC_TABLE_CMD(MACACCESS_CMD_FORGET),
			     ANA_TABLES_MACACCESS);
	}

	return 0;
}

int switch_qci_sfi_get(struct net_device *ndev, u32 index,
		       struct tsn_qci_psfp_sfi_conf *sfi)
{
	struct ocelot_port *port = netdev_priv(ndev);
	struct ocelot *ocelot = port->ocelot;
	u32 val, reg, fmeter_id, max_sdu;

	ocelot_field_write(ocelot, ANA_TABLES_SFIDTIDX_SFID_INDEX_0, index);

	ocelot_write(ocelot,
		     ANA_TABLES_SFIDACCESS_SFID_TBL_CMD(SFIDACCESS_CMD_READ),
		     ANA_TABLES_SFIDACCESS);

	val = ocelot_read(ocelot, ANA_TABLES_SFIDTIDX);
	if (!(val & ANA_TABLES_SFIDTIDX_SGID_VALID))
		return -EINVAL;

	sfi->stream_gate_instance_id = ANA_TABLES_SFIDTIDX_SGID_X(val);
	fmeter_id = ANA_TABLES_SFIDTIDX_POL_IDX_X(val);
	sfi->stream_filter.flow_meter_instance_id = fmeter_id;

	reg = ocelot_read(ocelot, ANA_TABLES_SFIDACCESS);
	max_sdu = ANA_TABLES_SFIDACCESS_MAX_SDU_LEN_X(reg);
	sfi->stream_filter.maximum_sdu_size  = max_sdu;

	if (reg & ANA_TABLES_SFIDACCESS_IGR_PRIO_MATCH_ENA)
		sfi->priority_spec = ANA_TABLES_SFIDACCESS_IGR_PRIO_X(reg);
	else
		netdev_info(ndev, "priority not enable\n");

	return 0;
}

int switch_qci_sfi_set(struct net_device *ndev, u32 index, bool enable,
		       struct tsn_qci_psfp_sfi_conf *sfi)
{
	struct ocelot_port *port = netdev_priv(ndev);
	struct ocelot *ocelot = port->ocelot;
	int igr_prio = sfi->priority_spec;
	u16 sgid  = sfi->stream_gate_instance_id;
	u16 pol_idx = sfi->stream_filter.flow_meter_instance_id;
	u16 max_sdu_len = sfi->stream_filter.maximum_sdu_size;
	int sfid = index * 2;

	netdev_dbg(ndev, "sfid=%d prio=%d sgid=%d pol_idx=%d\n",
		   index, igr_prio, sgid, pol_idx);

	ocelot_write(ocelot, ANA_TABLES_SFIDTIDX_SGID_VALID |
		     ANA_TABLES_SFIDTIDX_SGID(sgid) |
		     ANA_TABLES_SFIDTIDX_POL_ENA |
		     ANA_TABLES_SFIDTIDX_POL_IDX(pol_idx) |
		     ANA_TABLES_SFIDTIDX_SFID_INDEX(sfid),
		     ANA_TABLES_SFIDTIDX);

	ocelot_write(ocelot,
		     ((igr_prio >= 0) ? ANA_TABLES_SFIDACCESS_IGR_PRIO_MATCH_ENA : 0) |
		     ANA_TABLES_SFIDACCESS_IGR_PRIO(igr_prio) |
		     ANA_TABLES_SFIDACCESS_MAX_SDU_LEN(max_sdu_len) |
		     ANA_TABLES_SFIDACCESS_SFID_TBL_CMD(SFIDACCESS_CMD_WRITE),
		     ANA_TABLES_SFIDACCESS);

	return 0;
}

int switch_cb_streamid_counters_get(struct net_device *ndev, u32 index,
				    struct tsn_cb_streamid_counters *s_counters)
{
	return 0;
}

int switch_qci_sfi_counters_get(struct net_device *ndev, u32 index,
				struct tsn_qci_psfp_sfi_counters *sfi_counters)
{
	struct ocelot_port *port = netdev_priv(ndev);
	struct ocelot *ocelot = port->ocelot;
	u32 sfid = index;
	u32 match, not_pass, not_pass_sdu, red;

	ocelot_field_write(ocelot, SYS_STAT_CFG_STAT_VIEW_0, sfid);
	match = ocelot_read_gix(ocelot, SYS_CNT, 0x200);
	not_pass = ocelot_read_gix(ocelot, SYS_CNT, 0x201);
	not_pass_sdu = ocelot_read_gix(ocelot, SYS_CNT, 0x202);
	red = ocelot_read_gix(ocelot, SYS_CNT, 0x203);

	sfi_counters->matching_frames_count = match;
	sfi_counters->not_passing_frames_count = not_pass;
	sfi_counters->not_passing_sdu_count = not_pass_sdu;
	sfi_counters->red_frames_count  =  red;

	sfi_counters->passing_frames_count = match - not_pass;
	sfi_counters->passing_sdu_count = match - not_pass - not_pass_sdu;

	return 0;
}

void write_list(struct ocelot *ocelot,
		struct tsn_qci_psfp_gcl *gcl, uint32_t num)
{
	int i;
	u32 time_sum = 0;

	for (i = 0; i < num; i++) {
		ocelot_write_rix(ocelot,
				 ANA_SG_GCL_GS_CONFIG_IPS((gcl->ipv < 0) ? 0 : gcl->ipv + 8) |
				 (gcl->gate_state ? ANA_SG_GCL_GS_CONFIG_GATE_STATE : 0),
				 ANA_SG_GCL_GS_CONFIG, i);

		time_sum += gcl->time_interval;
		ocelot_write_rix(ocelot, time_sum, ANA_SG_GCL_TI_CONFIG, i);

		gcl++;
	}
}

int switch_qci_sgi_set(struct net_device *ndev, u32 index,
		       struct tsn_qci_psfp_sgi_conf *sgi_conf)
{
	int count;
	struct ocelot_port *port = netdev_priv(ndev);
	struct ocelot *ocelot = port->ocelot;
	struct tsn_qci_sg_control *admin_list = &sgi_conf->admin;
	u32 sgid = index;
	u32 list_length = sgi_conf->admin.control_list_length;
	u32 cycle_time = sgi_conf->admin.cycle_time;
	u32 cycle_time_ex = sgi_conf->admin.cycle_time_extension;
	u32 l_basetime = sgi_conf->admin.base_time & 0x00000000ffffffff;
	u64 h_basetime = (sgi_conf->admin.base_time & 0xffffffff00000000) >> 32;
	u64 cur_time;

	/*configure SGID*/
	ocelot_field_write(ocelot, ANA_SG_ACCESS_CTRL_SGID_0, sgid);

	netdev_info(ndev, "sgid=%d ate_enabled=%d control_list_length=%d cycle_time=0x%x l_basetime=0x%x initipv=%d\n",
		    sgid, sgi_conf->gate_enabled,
		    sgi_conf->admin.control_list_length,
		    cycle_time, l_basetime,
		    sgi_conf->admin.init_ipv);

	/*Enable SG*/
	if (sgi_conf->gate_enabled == FALSE) {
		ocelot_field_write(ocelot,
				   ANA_SG_CONFIG_REG_3_GATE_ENABLE_0, 0);
		return 0;
	}
	/*admin parameters*/
	cur_time = ocelot_read(ocelot, PTP_CUR_SEC_MSB);
	cur_time = cur_time << 32;
	cur_time += ocelot_read(ocelot, PTP_CUR_SEC_LSB);
	if (h_basetime < cur_time) {
		h_basetime = cur_time;
		l_basetime = ocelot_read(ocelot, PTP_CUR_NSEC);
	}

	ocelot_write(ocelot, l_basetime, ANA_SG_CONFIG_REG_1);
	ocelot_write(ocelot, h_basetime, ANA_SG_CONFIG_REG_2);
	if (sgi_conf->admin.init_ipv >= 0)
		ocelot_write(ocelot, ANA_SG_CONFIG_REG_3_IPV_VALID |
			     ANA_SG_CONFIG_REG_3_INIT_IPV(sgi_conf->admin.init_ipv) |
			     ANA_SG_CONFIG_REG_3_GATE_ENABLE |
			     ANA_SG_CONFIG_REG_3_LIST_LENGTH(list_length) |
			     sgi_conf->admin.gate_states << 28 |
			     ANA_SG_CONFIG_REG_3_BASE_TIME_SEC_MSB(h_basetime >> 32),
			     ANA_SG_CONFIG_REG_3);
	else
		ocelot_write(ocelot, ANA_SG_CONFIG_REG_3_IPV_INVALID(0) |
			     ANA_SG_CONFIG_REG_3_INIT_IPV(sgi_conf->admin.init_ipv) |
			     ANA_SG_CONFIG_REG_3_GATE_ENABLE |
			     ANA_SG_CONFIG_REG_3_LIST_LENGTH(list_length) |
			     sgi_conf->admin.gate_states << 28 |
			     ANA_SG_CONFIG_REG_3_BASE_TIME_SEC_MSB(h_basetime >> 32),
			     ANA_SG_CONFIG_REG_3);

	ocelot_write(ocelot, cycle_time, ANA_SG_CONFIG_REG_4);
	ocelot_write(ocelot, cycle_time_ex, ANA_SG_CONFIG_REG_5);

	write_list(ocelot, admin_list->gcl, list_length);

	/*CONG_CHANGE TO 1*/
	ocelot_field_write(ocelot, ANA_SG_ACCESS_CTRL_CONFIG_CHANGE_0, 1);

	count = 0;
	do { /* Wait until configuration change is complete */
		msleep(20);
		count++;
		if (count == 100) {
			netdev_err(ndev, "SGI Config change timeout\n");
			return -ETIMEDOUT;
		}
	} while (ocelot_read(ocelot, ANA_SG_ACCESS_CTRL) &
		 ANA_SG_ACCESS_CTRL_CONFIG_CHANGE);

	return SUCCESS;
}

void get_list(struct ocelot *ocelot, struct tsn_qci_psfp_gcl *gcl, uint32_t num)
{
	int i;
	u16 val;
	u32 time = 0;
	u32 reg;

	for (i = 0; i < num; i++) {
		val = ocelot_read_rix(ocelot, ANA_SG_GCL_GS_CONFIG, i);
		if (val & ANA_SG_GCL_GS_CONFIG_GATE_STATE)
			gcl->gate_state = TRUE;
		else
			gcl->gate_state = FALSE;

		if (val & ANA_SG_GCL_GS_CONFIG_IPV_VALID)
			gcl->ipv = ANA_SG_GCL_GS_CONFIG_IPV(val);
		else
			gcl->ipv = 0;

		reg = ocelot_read_rix(ocelot, ANA_SG_GCL_TI_CONFIG, i);
		gcl->time_interval = (reg - time);
		time = reg;

		gcl++;
	}
}

int switch_qci_sgi_get(struct net_device *ndev, u32 index,
		       struct tsn_qci_psfp_sgi_conf *sgi_conf)
{
	struct ocelot_port *port = netdev_priv(ndev);
	struct ocelot *ocelot = port->ocelot;
	struct tsn_qci_sg_control *admin  = &sgi_conf->admin;
	struct tsn_qci_psfp_gcl *glist;

	u32 val, reg;
	u32 list_num;

	val = ocelot_read(ocelot, ANA_SG_CONFIG_REG_1);
	reg = ocelot_read(ocelot, ANA_SG_CONFIG_REG_2);
	admin->base_time = reg;
	admin->base_time = (admin->base_time << 32) | val;

	admin->cycle_time = ocelot_read(ocelot,  ANA_SG_CONFIG_REG_4);
	admin->cycle_time_extension = ocelot_read(ocelot, ANA_SG_CONFIG_REG_5);

	val = ocelot_read(ocelot, ANA_SG_CONFIG_REG_3);

	if (val & ANA_SG_CONFIG_REG_3_IPV_VALID)
		admin->init_ipv = ANA_SG_CONFIG_REG_3_INIT_IPV_X(val);
	else
		netdev_info(ndev, "IPV specified  in bits [0:2] is not used or invalid");

	admin->control_list_length = ANA_SG_CONFIG_REG_3_LIST_LENGTH_X(val);
	list_num = admin->control_list_length;

	glist = (struct tsn_qci_psfp_gcl *)kmalloc(list_num *
			sizeof(struct tsn_qci_psfp_gcl), GFP_KERNEL);
	admin->gcl = glist;

	get_list(ocelot, glist, list_num);

	return 0;
}

int switch_qci_sgi_status_get(struct net_device *ndev, u16 index,
			      struct tsn_psfp_sgi_status *sgi_status)
{
	struct ocelot_port *port = netdev_priv(ndev);
	struct ocelot *ocelot = port->ocelot;
	u32 val, reg;

	/*SET SGID*/
	ocelot_field_write(ocelot, ANA_SG_ACCESS_CTRL_SGID_0, index);

	val = ocelot_read(ocelot, ANA_SG_STATUS_REG_1);
	reg = ocelot_read(ocelot, ANA_SG_STATUS_REG_2);
	sgi_status->config_change_time = reg;
	sgi_status->config_change_time = sgi_status->config_change_time << 32 |
					 val;

	val = ocelot_read(ocelot, ANA_SG_STATUS_REG_3);
	if (val & ANA_SG_STATUS_REG_3_CONFIG_PENDING)
		sgi_status->config_pending  = TRUE;
	else
		sgi_status->config_pending = FALSE;

	if (val & ANA_SG_STATUS_REG_3_GATE_STATE)
		sgi_status->oper.gate_states  =  TRUE;
	else
		sgi_status->oper.gate_states  =  FALSE;
	/*bit 3 encoding 0:IPV [0:2]is invalid . 1:IPV[0:2] is valid*/
	if (val & ANA_SG_STATUS_REG_3_IPV_VALID)
		sgi_status->oper.init_ipv  = ANA_SG_STATUS_REG_3_IPV_X(val);
	else
		sgi_status->oper.init_ipv = 0;

	return 0;
}

int switch_qci_fmi_set(struct net_device *ndev, u32 index,
		       bool enable, struct tsn_qci_psfp_fmi *fmi)
{
	struct ocelot_port *port = netdev_priv(ndev);
	struct ocelot *ocelot = port->ocelot;
	u32 cir = 0, cbs = 0, pir = 0, pbs = 0;
	u32 cir_ena = 0;
	u32 pbs_max = 0, cbs_max = 0;
	bool cir_discard = 0, pir_discard = 0;

	pir = fmi->eir;
	pbs = fmi->ebs;

	if (!fmi->drop_on_yellow) {
		cir_ena = 1;
	}

	if (cir_ena) {
		cir = fmi->cir;
		cbs = fmi->cbs;
		if (cir == 0 && cbs == 0) {
			cir_discard = 1;
		} else {
			cir = MSCC_DIV_ROUND_UP(cir, 100);
			cir *= 3;  /* Rate unit is 33 1/3 kbps */
			cbs = MSCC_DIV_ROUND_UP(cbs, 4096);
			cbs = (cbs ? cbs : 1);
			cbs_max = 60;
			if (fmi->cf)
				pir += fmi->cir;
		}
	}
	if (pir == 0 && pbs == 0) {
		pir_discard = 1;
	} else {
		pir = MSCC_DIV_ROUND_UP(pir, 100);
		pir *= 3;  /* Rate unit is 33 1/3 kbps */
		pbs = MSCC_DIV_ROUND_UP(pbs, 4096);
		pbs = (pbs ? pbs : 1);
		pbs_max = 60;
	}
	pir = MIN(GENMASK(15, 0), pir);
	cir = MIN(GENMASK(15, 0), cir);
	pbs = MIN(pbs_max, pbs);
	cbs = MIN(cbs_max, cbs);

	ocelot_write_gix(ocelot, (ANA_POL_MODE_CFG_IPG_SIZE(20) |
			 ANA_POL_MODE_CFG_FRM_MODE(1) |
			 (fmi->cf ? ANA_POL_MODE_CFG_DLB_COUPLED : 0) |
			 (cir_ena ? ANA_POL_MODE_CFG_CIR_ENA : 0) |
			 ANA_POL_MODE_CFG_OVERSHOOT_ENA),
			 ANA_POL_MODE_CFG, index);

	ocelot_write_gix(ocelot, ANA_POL_PIR_CFG_PIR_RATE(pir) |
			 ANA_POL_PIR_CFG_PIR_BURST(pbs),
			 ANA_POL_PIR_CFG, index);

	ocelot_write_gix(ocelot,
			 (pir_discard ? GENMASK(22, 0) : 0),
			 ANA_POL_PIR_STATE, index);

	ocelot_write_gix(ocelot, ANA_POL_CIR_CFG_CIR_RATE(cir) |
			 ANA_POL_CIR_CFG_CIR_BURST(cbs),
			 ANA_POL_CIR_CFG, index);

	ocelot_write_gix(ocelot,
			 (cir_discard ? GENMASK(22, 0) : 0),
			 ANA_POL_CIR_STATE, index);

	return 0;
}

int switch_qci_fmi_get(struct net_device *ndev, u32 index,
		       struct tsn_qci_psfp_fmi *fmi,
			   struct tsn_qci_psfp_fmi_counters *counters)
{
	struct ocelot_port *port = netdev_priv(ndev);
	struct ocelot *ocelot = port->ocelot;
	u32 val, reg;

	if (index <= 64)
		index += 64;

	val = ocelot_read_gix(ocelot, ANA_POL_PIR_CFG, index);
	reg = ocelot_read_gix(ocelot, ANA_POL_CIR_CFG, index);

	fmi->eir = ANA_POL_PIR_CFG_PIR_RATE_X(val);
	fmi->ebs = ANA_POL_PIR_CFG_PIR_BURST(val);
	fmi->cir = ANA_POL_CIR_CFG_CIR_RATE_X(reg);
	fmi->cbs = ANA_POL_CIR_CFG_CIR_BURST(reg);
	if (!(fmi->eir | fmi->ebs | fmi->cir | fmi->cbs))
		fmi->mark_red = TRUE;
	else
		fmi->mark_red = FALSE;

	val = ocelot_read_gix(ocelot, ANA_POL_MODE_CFG, index);
	if (val & ANA_POL_MODE_CFG_DLB_COUPLED)
		fmi->cf = TRUE;
	else
		fmi->cf = FALSE;
	if (val & ANA_POL_MODE_CFG_CIR_ENA)
		fmi->drop_on_yellow = FALSE;
	else
		fmi->drop_on_yellow = TRUE;

	return 0;
}

int switch_seq_gen_set(struct net_device *ndev, u32 index,
		  struct tsn_seq_gen_conf *sg_conf)
{
	struct ocelot_port *port = netdev_priv(ndev);
	struct ocelot *ocelot = port->ocelot;
	u8 iport_mask = sg_conf->iport_mask;
	u8 split_mask = sg_conf->split_mask;
	u8 seq_len = sg_conf->seq_len;
	u32 seq_num = sg_conf->seq_num;

	netdev_dbg(ndev, "iport_mask=0x%x split_mask=0x%x seq_len=%d seq_num=%d\n",
		   sg_conf->iport_mask, sg_conf->split_mask,
		   sg_conf->seq_len, sg_conf->seq_num);

	ocelot_write(ocelot,
		     ANA_TABLES_SEQ_MASK_SPLIT_MASK(split_mask) |
		     ANA_TABLES_SEQ_MASK_INPUT_PORT_MASK(iport_mask),
		     ANA_TABLES_SEQ_MASK);

	ocelot_write(ocelot,
		     ANA_TABLES_STREAMTIDX_S_INDEX(index) |
		     ANA_TABLES_STREAMTIDX_STREAM_SPLIT |
		     ANA_TABLES_STREAMTIDX_SEQ_SPACE_LOG2(seq_len),
		     ANA_TABLES_STREAMTIDX);

	ocelot_write(ocelot,
		     ANA_TABLES_STREAMACCESS_GEN_REC_SEQ_NUM(seq_num) |
		     ANA_TABLES_STREAMACCESS_SEQ_GEN_REC_ENA |
		     ANA_TABLES_STREAMACCESS_STREAM_TBL_CMD(SFIDACCESS_CMD_WRITE),
		     ANA_TABLES_STREAMACCESS);

	return 0;
}

int switch_seq_rec_set(struct net_device *ndev, u32 index,
		  struct tsn_seq_rec_conf *sr_conf)
{
	struct ocelot_port *port = netdev_priv(ndev);
	struct ocelot *ocelot = port->ocelot;
	u8 seq_len = sr_conf->seq_len;
	u8 hislen = sr_conf->his_len;

	netdev_dbg(ndev, "seq_len=%d hislen=%d rtag_pop_en=%d\n",
		   sr_conf->seq_len, sr_conf->his_len,
		   sr_conf->rtag_pop_en);

	ocelot_rmw_rix(ocelot, ANA_PORT_MODE_REDTAG_PARSE_CFG,
		       ANA_PORT_MODE_REDTAG_PARSE_CFG,
		       ANA_PORT_MODE, port->chip_port);

	ocelot_write(ocelot,
		     ANA_TABLES_STREAMTIDX_S_INDEX(index) |
		     ANA_TABLES_STREAMTIDX_FORCE_SF_BEHAVIOUR |
		     ANA_TABLES_STREAMTIDX_SEQ_HISTORY_LEN(hislen) |
		     ANA_TABLES_STREAMTIDX_RESET_ON_ROGUE |
		     (sr_conf->rtag_pop_en ?
		      ANA_TABLES_STREAMTIDX_REDTAG_POP : 0) |
		     ANA_TABLES_STREAMTIDX_SEQ_SPACE_LOG2(seq_len),
		     ANA_TABLES_STREAMTIDX);

	ocelot_write(ocelot,
		     ANA_TABLES_STREAMACCESS_SEQ_GEN_REC_ENA |
		     ANA_TABLES_STREAMACCESS_GEN_REC_TYPE |
		     ANA_TABLES_STREAMACCESS_STREAM_TBL_CMD(SFIDACCESS_CMD_WRITE),
		     ANA_TABLES_STREAMACCESS);

	return 0;
}

int switch_cb_get(struct net_device *ndev, u32 index, struct tsn_cb_status *c)
{
	u32 val;
	struct ocelot_port *port = netdev_priv(ndev);
	struct ocelot *ocelot = port->ocelot;

	ocelot_write(ocelot,
		     ANA_TABLES_STREAMTIDX_S_INDEX(index),
		     ANA_TABLES_STREAMTIDX);

	ocelot_write(ocelot,
		     ANA_TABLES_STREAMACCESS_STREAM_TBL_CMD(SFIDACCESS_CMD_READ),
		     ANA_TABLES_STREAMACCESS);

	val = ocelot_read(ocelot, ANA_TABLES_STREAMACCESS);
	c->gen_rec = (ANA_TABLES_STREAMACCESS_GEN_REC_TYPE & val) >> 2;
	c->seq_num = ANA_TABLES_STREAMACCESS_GEN_REC_SEQ_NUM_X(val);

	val = ocelot_read(ocelot, ANA_TABLES_STREAMTIDX);
	c->err = ANA_TABLES_STREAMTIDX_SEQ_GEN_ERR_STATUS_X(val);
	c->his_len = ANA_TABLES_STREAMTIDX_SEQ_HISTORY_LEN_X(val);
	c->seq_len = ANA_TABLES_STREAMTIDX_SEQ_SPACE_LOG2(val);

	val = ocelot_read(ocelot, ANA_TABLES_SEQ_MASK);
	c->split_mask = ANA_TABLES_SEQ_MASK_SPLIT_MASK_X(val);
	c->iport_mask = ANA_TABLES_SEQ_MASK_INPUT_PORT_MASK(val);

	c->seq_his = ocelot_read(ocelot, ANA_TABLES_SEQ_HISTORY);

	return 0;
}

int switch_pcp_map_set(struct net_device *ndev, bool enable)
{
	struct ocelot_port *port = netdev_priv(ndev);
	struct ocelot *ocelot = port->ocelot;
	int i;

	ocelot_rmw_gix(ocelot,
		       (enable ? ANA_PORT_QOS_CFG_QOS_PCP_ENA : 0),
		       ANA_PORT_QOS_CFG_QOS_PCP_ENA,
		       ANA_PORT_QOS_CFG,
		       port->chip_port);

	for (i = 0; i < NUM_MSCC_QOS_PRIO * 2; i++) {
		ocelot_rmw_ix(ocelot,
			      (ANA_PORT_PCP_DEI_MAP_DP_PCP_DEI_VAL & i) |
			      ANA_PORT_PCP_DEI_MAP_QOS_PCP_DEI_VAL(i),
			      ANA_PORT_PCP_DEI_MAP_DP_PCP_DEI_VAL |
			      ANA_PORT_PCP_DEI_MAP_QOS_PCP_DEI_VAL_M,
			      ANA_PORT_PCP_DEI_MAP,
			      port->chip_port, i);
	}

	return 0;
}

int switch_dscp_set(struct net_device *ndev,
		    bool enable,
		    const u8 dscp_ix,
		    struct tsn_qos_switch_dscp_conf *c)
{

	struct ocelot_port *port = netdev_priv(ndev);
	struct ocelot *ocelot = port->ocelot;
	u32 val, ri = dscp_ix;

	c->dscp = 0;
	c->trust = 1;
	c->remark = 0;

	if (dscp_ix > MSCC_QOS_DSCP_MAX) {
		netdev_info(ndev, "%s: Invalid dscp_ix %u\n",
			    __func__, dscp_ix);
		return -EINVAL;
	}
	if (c->cos > MSCC_QOS_PRIO_MAX) {
		netdev_info(ndev, "%s: Invalid cos %d\n",
			    __func__, c->cos);
		return -EINVAL;
	}
	if (c->dpl > MSCC_QOS_DP_MAX) {
		netdev_info(ndev, "%s: Invalid dpl %d\n",
			    __func__, c->dpl);
		return -EINVAL;
	}
	if (c->dscp > MSCC_QOS_DSCP_MAX) {
		netdev_info(ndev, "%s: Invalid dscp %d\n",
			    __func__, c->dscp);
		return -EINVAL;
	}

	ocelot_rmw_gix(ocelot,
		       (enable ? ANA_PORT_QOS_CFG_QOS_DSCP_ENA : 0) |
		       (c->dscp ? ANA_PORT_QOS_CFG_DSCP_TRANSLATE_ENA : 0),
		       ANA_PORT_QOS_CFG_QOS_DSCP_ENA |
		       ANA_PORT_QOS_CFG_DSCP_TRANSLATE_ENA,
		       ANA_PORT_QOS_CFG,
		       port->chip_port);

	val = (c->dpl ? ANA_DSCP_CFG_DP_DSCP_VAL : 0) |
	       ANA_DSCP_CFG_QOS_DSCP_VAL(c->cos) |
	       ANA_DSCP_CFG_DSCP_TRANSLATE_VAL(c->dscp) |
	       (c->trust ? ANA_DSCP_CFG_DSCP_TRUST_ENA : 0) |
	       (c->remark ? ANA_DSCP_CFG_DSCP_REWR_ENA : 0);

	ocelot_write_rix(ocelot, val, ANA_DSCP_CFG, ri);
	return 0;
}
