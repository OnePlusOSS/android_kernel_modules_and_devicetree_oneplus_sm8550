M=$(PWD)
AUDIO_ROOT=$(KERNEL_SRC)/$(M)

KBUILD_OPTIONS+=  AUDIO_ROOT=$(AUDIO_ROOT)
KBUILD_OPTIONS += KERNEL_BUILD=0
KBUILD_OPTIONS += BOARD_PLATFORM=kalama

EXTRA_SYMVERS := \
    $(KERNEL_SRC)/../vendor/qcom/opensource/mm-drivers/sync_fence/Module.symvers \
    $(KERNEL_SRC)/../vendor/qcom/opensource/securemsm-kernel/Module.symvers \
    $(KERNEL_SRC)/../vendor/qcom/opensource/mm-drivers/msm_ext_display/Module.symvers \
    $(KERNEL_SRC)/../vendor/qcom/opensource/mm-drivers/hw_fence/Module.symvers \
    $(KERNEL_SRC)/../vendor/qcom/opensource/mmrm-driver/Module.symvers

KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMVERS)"

all: modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean

%:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $@ $(KBUILD_OPTIONS)

