# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 2020-2022 Oplus. All rights reserved.

LINUXINCLUDE += -I$(srctree)/

obj-$(CONFIG_OPLUS_LOCKING_STRATEGY) += oplus_locking_strategy.o
oplus_locking_strategy-y += locking_main.o
oplus_locking_strategy-y += mutex.o
oplus_locking_strategy-y += rwsem.o
oplus_locking_strategy-y += futex.o
oplus_locking_strategy-y += sysfs.o
oplus_locking_strategy-$(CONFIG_OPLUS_LOCKING_MONITOR) += kern_lock_stat.o
