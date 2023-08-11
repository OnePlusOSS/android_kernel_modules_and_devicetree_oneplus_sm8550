KBUILD_OPTIONS += CONFIG_PSEUDO_SENSOR=m
KBUILD_OPTIONS += CONFIG_OPLUS_SENSOR_DEBUG_KIT=y

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)
modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) $(@)

