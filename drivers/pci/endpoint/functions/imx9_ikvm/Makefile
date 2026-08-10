# SPDX-License-Identifier: GPL-2.0
# Copyright 2026 NXP
#
# Makefile for PCI KVM input
#

obj-$(CONFIG_PCI_EPF_IMX9_VGA)	+= pci-epf-imx9-vga.o

obj-$(CONFIG_PCIE_KVM_INPUT) += pcie-input.o

pcie-input-y := pcie-kvm-hid.o \
		pcie_keyboard.o \
		pcie-rel-mouse.o \
		pcie-abs-mouse.o \
		pcie-local-keyboard.o

