# SPDX-License-Identifier: GPL-2.0-only
ccflags-y       := -I $(srctree)/mm
obj-$(CONFIG_HYBRIDSWAP_ZRAM) += oplus_bsp_hybridswap_zram.o

obj-$(CONFIG_CRYPTO_ZSTDN) += zstd/

oplus_bsp_hybridswap_zram-y	:=	zcomp.o zram_drv.o
oplus_bsp_hybridswap_zram-$(CONFIG_HYBRIDSWAP) += hybridswap/hybridmain.o
oplus_bsp_hybridswap_zram-$(CONFIG_HYBRIDSWAP_SWAPD) += hybridswap/hybridswapd.o
oplus_bsp_hybridswap_zram-$(CONFIG_CONT_PTE_HUGEPAGE) += hybridswap/hybridswapd_chp.o
oplus_bsp_hybridswap_zram-$(CONFIG_HYBRIDSWAP_CORE) += hybridswap/hybridswap.o
