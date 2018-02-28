#
# Makefile for the Freescale DPAA2 Ethernet controller
#

obj-$(CONFIG_FSL_DPAA2_ETH) += fsl-dpaa2-eth.o

fsl-dpaa2-eth-objs    := dpaa2-eth.o dpni.o
fsl-dpaa2-eth-${CONFIG_FSL_DPAA2_ETH_DEBUGFS} += dpaa2-eth-debugfs.o

# Needed by the tracing framework
CFLAGS_dpaa2-eth.o := -I$(src)
