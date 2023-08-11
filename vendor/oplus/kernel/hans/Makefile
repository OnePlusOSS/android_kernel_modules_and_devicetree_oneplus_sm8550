KBUILD_OPTIONS += OPLUS_OUT_OF_TREE_KO=y

ifeq ($(CHIPSET_COMPANY), QCOM)
KBUILD_OPTIONS += CONFIG_OPLUS_FEATURE_HANS=m
else ifeq ($(CHIPSET_COMPANY), MTK)
KBUILD_OPTIONS += CONFIG_OPLUS_FEATURE_HANS_GKI=m
endif

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)
modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) $(@)
