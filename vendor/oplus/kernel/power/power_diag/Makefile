# SPDX-License-Identifier: GPL-2.0

KBUILD_OPTIONS += CONFIG_OPLUS_FEATURE_POWER_DIAG=m

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)
modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) $(@)