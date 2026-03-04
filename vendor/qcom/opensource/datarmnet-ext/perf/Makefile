#By default build for CLD
RMNET_PERF_SELECT := CONFIG_RMNET_PERF=m
KBUILD_OPTIONS += RMNET_CORE_ROOT=$(KERNEL_SRC)/$(M)/../../datarmnet/core
KBUILD_OPTIONS += RMNET_SHS_PATH=$(KERNEL_SRC)/$(M)/../shs
KBUILD_OPTIONS += RMNET_CORE_INC_DIR=$(KERNEL_SRC)/$(M)/../../datarmnet/core/
KBUILD_OPTIONS += $(RMNET_PERF_SELECT)
KBUILD_OPTIONS += $(KBUILD_EXTRA) # Extra config if any
EXTRA_SYMVERS := \
    $(KERNEL_SRC)/../vendor/qcom/opensource/datarmnet/core/Module.symvers \
    $(KERNEL_SRC)/$(M)/../shs/Module.symvers

KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMVERS)"

M ?= $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(M) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean
