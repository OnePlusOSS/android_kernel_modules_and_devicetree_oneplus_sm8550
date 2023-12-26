# Build audio kernel driver
ifeq ($(call is-board-platform-in-list,kalama), true)
AUDIO_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/oplus_audio_extend.ko
AUDIO_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/oplus_audio_tfa98xx_v6.ko
endif

ifeq ($(call is-board-platform-in-list,pineapple), true)
AUDIO_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/oplus_audio_extend.ko
AUDIO_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/oplus_audio_tfa98xx_v6.ko
endif
ifeq ($(call is-board-platform-in-list,bengal), true)
AUDIO_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/oplus_audio_sipa.ko
AUDIO_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/oplus_audio_sipa_tuning.ko
AUDIO_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/oplus_audio_pa_manager.ko
endif # bengal supported