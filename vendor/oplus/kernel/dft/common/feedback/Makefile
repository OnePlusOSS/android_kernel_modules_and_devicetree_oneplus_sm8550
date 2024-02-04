# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 2018-2020 Oplus. All rights reserved.
ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
obj-$(CONFIG_OPLUS_FEATURE_FEEDBACK) += kernel_fb.o
else
ifeq ($(CONFIG_OPLUS_FEATURE_FEEDBACK), m)
obj-$(CONFIG_OPLUS_FEATURE_FEEDBACK) += gki_mtk/kernel_fb.o
endif #CONFIG_OPLUS_FEATURE_FEEDBACK
endif #CONFIG_OPLUS_SYSTEM_KERNEL_QCOM

