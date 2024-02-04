# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 2018-2020 Oplus. All rights reserved.

LINUXINCLUDE += -I$(srctree)/

obj-$(CONFIG_OPLUS_FEATURE_SCHED_ASSIST) += oplus_bsp_sched_assist.o
oplus_bsp_sched_assist-y += sched_assist.o
oplus_bsp_sched_assist-y += sa_common.o
oplus_bsp_sched_assist-y += sa_sysfs.o
oplus_bsp_sched_assist-y += sa_exec.o
oplus_bsp_sched_assist-y += sa_fair.o
oplus_bsp_sched_assist-y += sa_jankinfo.o
oplus_bsp_sched_assist-y += sa_oemdata.o
oplus_bsp_sched_assist-y += sa_priority.o
oplus_bsp_sched_assist-y += sched_assist_locking.o
oplus_bsp_sched_assist-$(CONFIG_OPLUS_CPU_AUDIO_PERF) += sa_audio.o
oplus_bsp_sched_assist-$(CONFIG_OPLUS_FEATURE_LOADBALANCE) += sa_balance.o
oplus_bsp_sched_assist-$(CONFIG_OPLUS_FEATURE_PIPELINE) += sa_pipeline.o
