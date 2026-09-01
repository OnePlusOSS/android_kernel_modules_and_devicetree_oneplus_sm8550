M=$(PWD)
AUDIO_ROOT=$(KERNEL_SRC)/$(M)

KBUILD_OPTIONS += MODNAME=audio_dlkm
KBUILD_OPTIONS+= AUDIO_ROOT=$(AUDIO_ROOT)
KBUILD_OPTIONS+= KBUILD_EXTRA_SYMBOLS=$(M)/../mm-drivers/msm_ext_display/Module.symvers

all: modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean

%:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $@ $(KBUILD_OPTIONS)

