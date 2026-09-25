# SPDX-License-Identifier: GPL-2.0-only

KBUILD_OPTIONS+= DISPLAY_ROOT=$(KERNEL_SRC)/$(M)

KBUILD_EXTRA_SYMBOLS := $(M)/../mm-drivers/hw_fence/Module.symvers \
						$(M)/../mm-drivers/msm_ext_display/Module.symvers \
						$(M)/../mm-drivers/sync_fence/Module.symvers \
						$(M)/../mmrm-driver/Module.symvers \
						$(M)/../securemsm-kernel/Module.symvers

KBUILD_OPTIONS+=CONFIG_DRM_MSM=y
KBUILD_OPTIONS+= KBUILD_EXTRA_SYMBOLS="$(KBUILD_EXTRA_SYMBOLS)"

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(M) modules_install

%:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $@ $(KBUILD_OPTIONS)

clean:
	rm -f *.o *.ko *.mod.c *.mod.o *~ .*.cmd Module.symvers
	rm -rf .tmp_versions
