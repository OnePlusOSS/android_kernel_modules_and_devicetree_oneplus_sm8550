#
# Makefile for the wifi cap center.
#
ifeq ($(OPLUS_VND_BUILD_PLATFORM), SM8550)
KBUILD_OPTIONS += OPLUS_FEATURE_WIFI_SMARTANTENNA=y
endif

KBUILD_OPTIONS += CONFIG_OPLUS_FEATURE_WIFI_SMARTANTENNA=m
KBUILD_OPTIONS += OPLUS_OUT_OF_TREE_KO=y

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) W=1 $(@)
