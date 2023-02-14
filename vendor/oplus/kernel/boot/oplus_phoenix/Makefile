# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 2018-2020 Oplus. All rights reserved.
ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
obj-$(CONFIG_OPLUS_FEATURE_PHOENIX) += oplus_phoenix.o
oplus_phoenix-y := op_bootprof.o phoenix_dump.o phoenix_watchdog.o phoenix_base.o
obj-$(CONFIG_OPLUS_FEATURE_PHOENIX_REBOOT_SPEED) += phoenix_reboot_speed.o
else
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
obj-$(CONFIG_OPLUS_FEATURE_PHOENIX) += oplus_phoenix.o
oplus_phoenix-y := op_bootprof.o phoenix_dump.o phoenix_watchdog.o phoenix_base.o
endif #CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
obj-$(CONFIG_OPLUS_FEATURE_KMSG_WB) += oplus_kmsg_wb.o
