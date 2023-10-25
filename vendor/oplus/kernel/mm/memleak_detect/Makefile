# SPDX-License-Identifier: GPL-2.0-only
## only use in out_of_tree
KBUILD_OPTIONS += CONFIG_MEMLEAK_DETECT=m

KERNEL_SRC ?= lib/module/$(shell uname -r)/build
M ?= $(shell pwd)

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) $(@)
