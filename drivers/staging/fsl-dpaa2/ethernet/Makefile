#
# Makefile for the Freescale DPAA Ethernet controllers
#
# Copyright (C) 2014-2015 Freescale Semiconductor, Inc.
#
# This file is released under the GPLv2
#

ccflags-y += -DVERSION=\"\"

obj-$(CONFIG_FSL_DPAA2_ETH) += fsl-dpaa2-eth.o

fsl-dpaa2-eth-objs    := dpaa2-eth.o dpaa2-ethtool.o dpni.o
fsl-dpaa2-eth-${CONFIG_FSL_DPAA2_ETH_DEBUGFS} += dpaa2-eth-debugfs.o

#Needed by the tracing framework
CFLAGS_dpaa2-eth.o := -I$(src)

ifeq ($(CONFIG_FSL_DPAA2_ETH_GCOV),y)
	GCOV_PROFILE := y
endif
