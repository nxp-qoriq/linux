# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the CAAM backend and dependent components
#
ifeq ($(CONFIG_CRYPTO_DEV_FSL_CAAM_DEBUG), y)
	ccflags-y := -DDEBUG
endif

ccflags-y += -DVERSION=\"\"

obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_COMMON) += error.o
obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM) += caam.o
obj-$(CONFIG_CRYPTO_DEV_FSL_CAAM_JR) += caam_jr.o

caam-y := ctrl.o
caam_jr-y := jr.o key_gen.o error.o
caam_jr-$(CONFIG_CRYPTO_DEV_FSL_CAAM_CRYPTO_API) += caamalg.o caamalg_desc.o
caam_jr-$(CONFIG_CRYPTO_DEV_FSL_CAAM_CRYPTO_API_QI) += caamalg_qi.o \
						       caamalg_desc.o
caam_jr-$(CONFIG_CRYPTO_DEV_FSL_CAAM_AHASH_API) += caamhash.o caamhash_desc.o
caam_jr-$(CONFIG_CRYPTO_DEV_FSL_CAAM_RNG_API) += caamrng.o
caam_jr-$(CONFIG_CRYPTO_DEV_FSL_CAAM_PKC_API) += caampkc.o pkc_desc.o

caam-$(CONFIG_CRYPTO_DEV_FSL_CAAM_CRYPTO_API_QI) += qi.o
ifneq ($(CONFIG_CRYPTO_DEV_FSL_CAAM_CRYPTO_API_QI),)
	ccflags-y += -DCONFIG_CAAM_QI
endif

obj-$(CONFIG_CRYPTO_DEV_FSL_DPAA2_CAAM) += dpaa2_caam.o

dpaa2_caam-y    := caamalg_qi2.o dpseci.o caamalg_desc.o caamhash_desc.o
