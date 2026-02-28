# SPDX-License-Identifier: GPL-2.0

KBUILD_OPTIONS+= DISPLAY_ROOT=$(KERNEL_SRC)/$(M)/../

EXTRA_SYMVERS := \
    $(KERNEL_SRC)/../vendor/qcom/opensource/mm-drivers/sync_fence/Module.symvers \
    $(KERNEL_SRC)/../vendor/qcom/opensource/securemsm-kernel/Module.symvers \
    $(KERNEL_SRC)/../vendor/qcom/opensource/mm-drivers/msm_ext_display/Module.symvers \
    $(KERNEL_SRC)/../vendor/qcom/opensource/mm-drivers/hw_fence/Module.symvers \
    $(KERNEL_SRC)/../vendor/qcom/opensource/mmrm-driver/Module.symvers

KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMVERS)"

all: modules

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(M) modules_install

%:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $@ $(KBUILD_OPTIONS)

clean:
	rm -f *.o *.ko *.mod.c *.mod.o *~ .*.cmd Module.symvers
	rm -rf .tmp_versions

