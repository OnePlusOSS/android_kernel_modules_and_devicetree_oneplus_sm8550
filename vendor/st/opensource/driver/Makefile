KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build

KBUILD_OPTIONS+= NFC_ROOT=$(KERNEL_SRC)/$(M)
M ?= $(shell pwd)

KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS=$(KERNEL_SRC)/../vendor/qcom/opensource/securemsm-kernel/Module.symvers

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(M) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean
