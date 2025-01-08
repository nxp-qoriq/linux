# SPDX-License-Identifier: GPL-2.0-only

obj-$(CONFIG_NET_DSA_HMS_SWITCH) += hmsdsa.o

hmsdsa-objs := \
    hms_spi.o \
    hms_config.o \
    hms_ethtool.o \
    hms_devlink.o \
    hms_main.o

ifdef CONFIG_NET_DSA_HMS_PTP
hmsdsa-objs += hms_ptp.o
endif