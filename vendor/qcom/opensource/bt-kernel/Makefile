KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

M=$(PWD)
BT_ROOT=$(KERNEL_SRC)/$(M)

KBUILD_OPTIONS+=  BT_ROOT=$(BT_ROOT)
KBUILD_OPTIONS += CONFIG_MSM_BT_POWER=m
KBUILD_OPTIONS += CONFIG_BTFM_SLIM=m
KBUILD_OPTIONS += CONFIG_I2C_RTC6226_QCA=m
KBUILD_EXTRA_SYMBOLS=$(call intermediates-dir-for,DLKM,wlan-platform-module-symvers)/Module.symvers
KBUILD_EXTRA_SYMBOLS=$(M)/../wlan/platform/Module.symvers
ccflags-y += -I$(BT_ROOT)/../wlan/platform/inc

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(M) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean
