# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 2018-2020 Oplus. All rights reserved.

ifeq ($(CONFIG_OPLUS_SYSTEM_KERNEL_QCOM), y)
obj-$(CONFIG_OPLUS_FEATURE_GKI_CPUFREQ_BOUNCING) += cpufreq_bouncing.o
else
obj-$(CONFIG_OPLUS_FEATURE_GKI_CPUFREQ_BOUNCING) += cpufreq_bouncing.o
cpufreq_bouncing-y := cpufreq_bouncing_mtk.o
endif
