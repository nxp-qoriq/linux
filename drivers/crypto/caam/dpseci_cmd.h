/*
 * Copyright 2013-2016 Freescale Semiconductor Inc.
 * Copyright 2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the names of the above-listed copyright holders nor the
 *	 names of any contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
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

#ifndef _DPSECI_CMD_H_
#define _DPSECI_CMD_H_

/* DPSECI Version */
#define DPSECI_VER_MAJOR				5
#define DPSECI_VER_MINOR				1

#define DPSECI_VER(maj, min)	(((maj) << 16) | (min))
#define DPSECI_VERSION		DPSECI_VER(DPSECI_VER_MAJOR, DPSECI_VER_MINOR)

/* Command IDs */

#define DPSECI_CMDID_CLOSE                              0x8001
#define DPSECI_CMDID_OPEN                               0x8091
#define DPSECI_CMDID_CREATE                             0x9092
#define DPSECI_CMDID_DESTROY                            0x9891
#define DPSECI_CMDID_GET_API_VERSION                    0xa091

#define DPSECI_CMDID_ENABLE                             0x0021
#define DPSECI_CMDID_DISABLE                            0x0031
#define DPSECI_CMDID_GET_ATTR                           0x0041
#define DPSECI_CMDID_RESET                              0x0051
#define DPSECI_CMDID_IS_ENABLED                         0x0061

#define DPSECI_CMDID_SET_IRQ_ENABLE                     0x0121
#define DPSECI_CMDID_GET_IRQ_ENABLE                     0x0131
#define DPSECI_CMDID_SET_IRQ_MASK                       0x0141
#define DPSECI_CMDID_GET_IRQ_MASK                       0x0151
#define DPSECI_CMDID_GET_IRQ_STATUS                     0x0161
#define DPSECI_CMDID_CLEAR_IRQ_STATUS                   0x0171

#define DPSECI_CMDID_SET_RX_QUEUE                       0x1941
#define DPSECI_CMDID_GET_RX_QUEUE                       0x1961
#define DPSECI_CMDID_GET_TX_QUEUE                       0x1971
#define DPSECI_CMDID_GET_SEC_ATTR                       0x1981
#define DPSECI_CMDID_GET_SEC_COUNTERS                   0x1991
#define DPSECI_CMDID_SET_OPR				0x19A1
#define DPSECI_CMDID_GET_OPR				0x19B1

#define DPSECI_CMDID_SET_CONGESTION_NOTIFICATION	0x1701
#define DPSECI_CMDID_GET_CONGESTION_NOTIFICATION	0x1711

/* Macros for accessing command fields smaller than 1 byte */
#define DPSECI_MASK(field)	\
	GENMASK(DPSECI_##field##_SHIFT + DPSECI_##field##_SIZE - 1,	\
		DPSECI_##field##_SHIFT)

#define dpseci_set_field(var, field, val)	\
	((var) |= (((val) << DPSECI_##field##_SHIFT) & DPSECI_MASK(field)))

#define dpseci_get_field(var, field)	\
	(((var) & DPSECI_MASK(field)) >> DPSECI_##field##_SHIFT)

struct dpseci_cmd_open {
	__le32 dpseci_id;
};

struct dpseci_cmd_create {
	u8 priorities[8];
	u8 num_tx_queues;
	u8 num_rx_queues;
	__le16 pad;
	__le32 options;
};

struct dpseci_cmd_destroy {
	__le32 object_id;
};

struct dpseci_rsp_is_enabled {
	__le32 is_enabled;
};

struct dpseci_cmd_irq_enable {
	u8 enable_state;
	u8 pad[3];
	u8 irq_index;
};

struct dpseci_rsp_get_irq_enable {
	u8 enable_state;
};

struct dpseci_cmd_irq_mask {
	__le32 mask;
	u8 irq_index;
};

struct dpseci_cmd_irq_status {
	__le32 status;
	u8 irq_index;
};

struct dpseci_rsp_get_attributes {
	__le32 id;
	__le32 pad0;
	u8 num_tx_queues;
	u8 num_rx_queues;
	u8 pad1[6];
	__le32 options;
};

struct dpseci_cmd_queue {
	__le32 dest_id;
	u8 priority;
	u8 queue;
	u8 dest_type;
	u8 pad;
	__le64 user_ctx;
	union {
		__le32 options;
		__le32 fqid;
	};
	__le32 order_preservation_en;
};

struct dpseci_rsp_get_tx_queue {
	__le32 pad;
	__le32 fqid;
	u8 priority;
};

struct dpseci_rsp_get_sec_attr {
	__le16 ip_id;
	u8 major_rev;
	u8 minor_rev;
	u8 era;
	u8 pad0[3];
	u8 deco_num;
	u8 zuc_auth_acc_num;
	u8 zuc_enc_acc_num;
	u8 pad1;
	u8 snow_f8_acc_num;
	u8 snow_f9_acc_num;
	u8 crc_acc_num;
	u8 pad2;
	u8 pk_acc_num;
	u8 kasumi_acc_num;
	u8 rng_acc_num;
	u8 pad3;
	u8 md_acc_num;
	u8 arc4_acc_num;
	u8 des_acc_num;
	u8 aes_acc_num;
};

struct dpseci_rsp_get_sec_counters {
	__le64 dequeued_requests;
	__le64 ob_enc_requests;
	__le64 ib_dec_requests;
	__le64 ob_enc_bytes;
	__le64 ob_prot_bytes;
	__le64 ib_dec_bytes;
	__le64 ib_valid_bytes;
};

struct dpseci_rsp_get_api_version {
	__le16 major;
	__le16 minor;
};

struct dpseci_cmd_opr {
	__le16 pad;
	u8 index;
	u8 options;
	u8 pad1[7];
	u8 oloe;
	u8 oeane;
	u8 olws;
	u8 oa;
	u8 oprrws;
};

#define DPSECI_OPR_RIP_SHIFT		0
#define DPSECI_OPR_RIP_SIZE		1
#define DPSECI_OPR_ENABLE_SHIFT		1
#define DPSECI_OPR_ENABLE_SIZE		1
#define DPSECI_OPR_TSEQ_NLIS_SHIFT	1
#define DPSECI_OPR_TSEQ_NLIS_SIZE	1
#define DPSECI_OPR_HSEQ_NLIS_SHIFT	1
#define DPSECI_OPR_HSEQ_NLIS_SIZE	1

struct dpseci_rsp_get_opr {
	__le64 pad;
	u8 rip_enable;
	u8 pad0[2];
	u8 oloe;
	u8 oeane;
	u8 olws;
	u8 oa;
	u8 oprrws;
	__le16 nesn;
	__le16 pad1;
	__le16 ndsn;
	__le16 pad2;
	__le16 ea_tseq;
	u8 tseq_nlis;
	u8 pad3;
	__le16 ea_hseq;
	u8 hseq_nlis;
	u8 pad4;
	__le16 ea_hptr;
	__le16 pad5;
	__le16 ea_tptr;
	__le16 pad6;
	__le16 opr_vid;
	__le16 pad7;
	__le16 opr_id;
};

#define DPSECI_CGN_DEST_TYPE_SHIFT	0
#define DPSECI_CGN_DEST_TYPE_SIZE	4
#define DPSECI_CGN_UNITS_SHIFT		4
#define DPSECI_CGN_UNITS_SIZE		2

struct dpseci_cmd_congestion_notification {
	__le32 dest_id;
	__le16 notification_mode;
	u8 priority;
	u8 options;
	__le64 message_iova;
	__le64 message_ctx;
	__le32 threshold_entry;
	__le32 threshold_exit;
};

#endif /* _DPSECI_CMD_H_ */
