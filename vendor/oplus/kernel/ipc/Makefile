# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 2018-2020 Oplus. All rights reserved.

LINUXINCLUDE += -I$(srctree)/

obj-$(CONFIG_OPLUS_BINDER_STRATEGY) += oplus_binder_strategy.o
oplus_binder_strategy-y += binder_main.o
oplus_binder_strategy-y += binder_sched.o
oplus_binder_strategy-y += binder_sysfs.o
oplus_binder_strategy-$(CONFIG_OPLUS_BINDER_TRANS_CTRL) += trans_ctrl.o