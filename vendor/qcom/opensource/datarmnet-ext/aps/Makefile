#By default build for CLD
RMNET_APS_SELECT := CONFIG_RMNET_APS=m
KBUILD_OPTIONS += RMNET_CORE_ROOT=$(KERNEL_SRC)/$(M)/../../datarmnet/core
KBUILD_OPTIONS += RMNET_CORE_INC_DIR=$(KERNEL_SRC)/$(M)/../../datarmnet/core/
KBUILD_OPTIONS += $(RMNET_APS_SELECT)
KBUILD_OPTIONS += $(KBUILD_EXTRA) # Extra config if any
KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS=$(KERNEL_SRC)/../vendor/qcom/opensource/datarmnet/core/Module.symvers

M ?= $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(M) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean
