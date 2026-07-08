#By default build for CLD
RMNET_WLAN_SELECT := CONFIG_RMNET_WLAN=m
DATARMNET_CORE_PATH=datarmnet/core
KBUILD_OPTIONS += RMNET_CORE_ROOT=$(KERNEL_SRC)/$(M)/../../datarmnet/core
KBUILD_OPTIONS += RMNET_CORE_INC_DIR=$(KERNEL_SRC)/$(M)/../../datarmnet/core/
KBUILD_OPTIONS += $(RMNET_WLAN_SELECT)
KBUILD_OPTIONS += $(KBUILD_EXTRA) # Extra config if any
KBUILD_EXTRA_SYMBOLS := $(M)/../../$(DATARMNET_CORE_PATH)/Module.symvers

M ?= $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS) KBUILD_EXTRA_SYMBOLS=$(KBUILD_EXTRA_SYMBOLS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(M) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean
