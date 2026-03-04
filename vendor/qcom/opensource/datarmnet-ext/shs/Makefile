KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build

KBUILD_OPTIONS := RMNET_SHS_ROOT=$(PWD)
KBUILD_OPTIONS += RMNET_CORE_ROOT=$(KERNEL_SRC)/$(M)/../../datarmnet/core
KBUILD_OPTIONS += RMNET_CORE_INC_DIR=$(KERNEL_SRC)/$(M)/../../datarmnet/core/
KBUILD_OPTIONS += MODNAME?=rmnet_shs
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
