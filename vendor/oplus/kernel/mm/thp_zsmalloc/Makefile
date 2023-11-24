# SPDX-License-Identifier: GPL-2.0-only
ccflags-y       := -I $(srctree)/mm
obj-$(CONFIG_THP_ZSMALLOC) += oplus_bsp_zsmalloc.o

oplus_bsp_zsmalloc-y   := zsmalloc.o
oplus_bsp_zsmalloc-$(CONFIG_CONT_PTE_HUGEPAGE_64K_ZRAM) += thp_zsmalloc.o
