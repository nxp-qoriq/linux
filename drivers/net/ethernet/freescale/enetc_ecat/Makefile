# SPDX-License-Identifier: GPL-2.0

obj-$(CONFIG_FSL_ECAT_ENETC_CORE) += fsl-ecat-enetc-core.o
fsl-ecat-enetc-core-y := enetc_ecat.o enetc_ecat_cbdr.o enetc_ecat_ethtool.o

obj-$(CONFIG_FSL_ECAT_ENETC4) += fsl-ecat-enetc4.o
fsl-ecat-enetc4-y := enetc4_ecat_pf.o enetc_ecat_pf_common.o enetc_ecat_devlink.o
fsl-ecat-enetc4-$(CONFIG_PCI_IOV) += enetc_msg.o
fsl-ecat-enetc4-$(CONFIG_FSL_ENETC_QOS) += enetc_qos.o
fsl-ecat-enetc4-$(CONFIG_DEBUG_FS) += enetc_debugfs.o

