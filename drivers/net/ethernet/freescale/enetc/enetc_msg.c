/*
 * Copyright 2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the names of the above-listed copyright holders nor the
 *       names of any contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
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

#include "enetc_pf.h"

static void enetc_msg_disable_mr_int(struct enetc_hw *hw)
{
	u32 psiier = enetc_rd(hw, ENETC_PSIIER);
	/* disable MR int source(s) */
	enetc_wr(hw, ENETC_PSIIER, psiier & ~ENETC_PSIIER_MR_MASK);
}

static void enetc_msg_enable_mr_int(struct enetc_hw *hw)
{
	u32 psiier = enetc_rd(hw, ENETC_PSIIER);

	enetc_wr(hw, ENETC_PSIIER, psiier | ENETC_PSIIER_MR_MASK);
}

static irqreturn_t enetc_msg_psi_msix(int irq, void *data)
{
	struct enetc_si *si = (struct enetc_si *)data;
	struct enetc_pf *pf = enetc_si_priv(si);

	enetc_msg_disable_mr_int(&si->hw);
	schedule_work(&pf->msg_task);

	return IRQ_HANDLED;
}

static u16 enetc_msg_psi_set_vf_primary_mac_addr(struct enetc_pf *pf,
						 int mbox_id)
{
	struct enetc_msg_swbd *msg = &pf->rxmsg[mbox_id];
	struct enetc_msg_cmd_set_primary_mac *cmd;
	struct enetc_si *si = pf->si;
	u16 cmd_id;
	char *addr;

	cmd = (struct enetc_msg_cmd_set_primary_mac *)msg->vaddr;
	cmd_id = cmd->header.id;
	if (cmd_id != ENETC_MSG_CMD_MNG_ADD)
		return ENETC_MSG_CMD_STATUS_FAIL;

	addr = cmd->mac.sa_data;
	dev_info(&si->pdev->dev, "Request from VF#%d for %x:%x:%x:%x:%x:%x\n",
		 mbox_id, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

	return ENETC_MSG_CMD_STATUS_OK;
}

static void enetc_msg_handle_rxmsg(struct enetc_pf *pf, int mbox_id,
				   u16 *status)
{
	struct enetc_msg_swbd *msg = &pf->rxmsg[mbox_id];
	struct enetc_msg_cmd_header *cmd_hdr;
	u16 cmd_type;

	*status = ENETC_MSG_CMD_STATUS_OK;
	/* TODO: dispatch command */
	cmd_hdr = (struct enetc_msg_cmd_header *)msg->vaddr;
	cmd_type = cmd_hdr->type;
	pr_info("cmd_type: 0x%x\n", cmd_type);

	switch (cmd_type) {
	case ENETC_MSG_CMD_MNG_MAC:
		*status = enetc_msg_psi_set_vf_primary_mac_addr(pf, mbox_id);
		break;
	default:
		pr_err("command not supported (cmd_type: 0x%x)\n", cmd_type);
	}
}

static void enetc_msg_task(struct work_struct *work)
{
	struct enetc_pf *pf = container_of(work, struct enetc_pf, msg_task);
	struct enetc_hw *hw = &pf->si->hw;
	unsigned long mr_mask;
	int i;

	for (;;) {
		mr_mask = enetc_rd(hw, ENETC_PSIMSGRR) & ENETC_PSIMSGRR_MR_MASK;
		if (!mr_mask) {
			/* re-arm MR interrupts, w1c the IDR reg */
			enetc_wr(hw, ENETC_PSIIDR, ENETC_PSIIER_MR_MASK);
			enetc_msg_enable_mr_int(hw);
			return;
		}

		for (i = 0; i < pf->num_vfs; i++) {
			u32 psimsgrr;
			u16 msg_code;

			if (!(ENETC_PSIMSGRR_MR(i) & mr_mask))
				continue;

			enetc_msg_handle_rxmsg(pf, i, &msg_code);

			psimsgrr = ENETC_SIMSGSR_SET_MC(msg_code);
			psimsgrr |= ENETC_PSIMSGRR_MR(i); /* w1c */
			enetc_wr(hw, ENETC_PSIMSGRR, psimsgrr);
		}
	}
}

/* Init */

static int enetc_msg_alloc_mbx(struct enetc_si *si, int idx)
{
	struct enetc_pf *pf = enetc_si_priv(si);
	struct device *dev = &si->pdev->dev;
	struct enetc_hw *hw = &si->hw;
	struct enetc_msg_swbd *msg;
	u32 val;

	msg = &pf->rxmsg[idx];
	/* allocate and set receive buffer */
	msg->size = ENETC_DEFAULT_MSG_SIZE;

	msg->vaddr = dma_zalloc_coherent(dev, msg->size, &msg->dma,
					 GFP_KERNEL);
	if (!msg->vaddr) {
		dev_err(dev, "msg: fail to alloc dma buffer of size: %d\n",
			msg->size);
		return -ENOMEM;
	}

	/* set multiple of 32 bytes */
	val = lower_32_bits(msg->dma);
	enetc_wr(hw, ENETC_PSIVMSGRCVAR0(idx), val);
	val = upper_32_bits(msg->dma);
	enetc_wr(hw, ENETC_PSIVMSGRCVAR1(idx), val);

	return 0;
}

static void enetc_msg_free_mbx(struct enetc_si *si, int idx)
{
	struct enetc_pf *pf = enetc_si_priv(si);
	struct enetc_hw *hw = &si->hw;
	struct enetc_msg_swbd *msg;

	msg = &pf->rxmsg[idx];
	dma_free_coherent(&si->pdev->dev, msg->size, msg->vaddr, msg->dma);
	memset(msg, 0, sizeof(*msg));

	enetc_wr(hw, ENETC_PSIVMSGRCVAR0(idx), 0);
	enetc_wr(hw, ENETC_PSIVMSGRCVAR1(idx), 0);
}

int enetc_msg_psi_init(struct enetc_pf *pf)
{
	struct enetc_si *si = pf->si;
	int vector, i, err;

	/* register message passing interrupt handler */
	sprintf(pf->msg_int_name, "%s-vfmsg", si->ndev->name);
	vector = pci_irq_vector(si->pdev, ENETC_SI_INT_IDX);
	err = request_irq(vector, enetc_msg_psi_msix, 0, pf->msg_int_name, si);
	if (err) {
		dev_err(&si->pdev->dev,
			"PSI messaging: request_irq() failed!\n");
		return err;
	}

	/* set one IRQ entry for PSI message receive notification (SI int) */
	enetc_wr(&si->hw, ENETC_SIMSIVR, ENETC_SI_INT_IDX);

	/* initialize PSI mailbox */
	INIT_WORK(&pf->msg_task, enetc_msg_task);

	for (i = 0; i < pf->num_vfs; i++) {
		err = enetc_msg_alloc_mbx(si, i);
		if (err)
			goto err_init_mbx;
	}

	/* enable MR interrupts */
	enetc_msg_enable_mr_int(&si->hw);

	return 0;

err_init_mbx:
	for (i--; i >= 0; i--)
		enetc_msg_free_mbx(si, i);

	free_irq(vector, si);

	return err;
}

void enetc_msg_psi_free(struct enetc_pf *pf)
{
	struct enetc_si *si = pf->si;
	int i;

	cancel_work_sync(&pf->msg_task);

	/* disable MR interrupts */
	enetc_msg_disable_mr_int(&si->hw);

	for (i = 0; i < pf->num_vfs; i++)
		enetc_msg_free_mbx(si, i);

	/* de-register message passing interrupt handler */
	free_irq(pci_irq_vector(si->pdev, ENETC_SI_INT_IDX), si);
}
