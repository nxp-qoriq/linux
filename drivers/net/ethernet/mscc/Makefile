# SPDX-License-Identifier: (GPL-2.0 OR MIT)
obj-$(CONFIG_MSCC_OCELOT_SWITCH) += mscc_ocelot_common.o
mscc_ocelot_common-y := ocelot.o ocelot_io.o
obj-$(CONFIG_MSCC_OCELOT_SWITCH_OCELOT) += ocelot_board.o ocelot_regs.o

obj-$(CONFIG_MSCC_FELIX_SWITCH) += mscc_felix.o
mscc_felix-y := felix_regs.o felix_board.o
ifeq ($(CONFIG_MSCC_FELIX_SWITCH_PTP_CLOCK),y)
	mscc_felix-y += felix_ptp.o
endif
obj-$(CONFIG_MSCC_FELIX_SWITCH_TSN) += tsn_switch.o
