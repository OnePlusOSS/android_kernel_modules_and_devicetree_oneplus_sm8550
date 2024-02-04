# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 2018-2020 Oplus. All rights reserved.
ifeq ($(CONFIG_KRETPROBES), y)
obj-$(CONFIG_OPLUS_FEATURE_FAULT_INJECT) +=  oplus_inject.o
oplus_inject-objs :=  oplus_inject_hook.o oplus_inject_proc.o
obj-$(CONFIG_OPLUS_FEATURE_FAULT_INJECT_SAMPLE) +=  oplus_inject_sample.o
endif
