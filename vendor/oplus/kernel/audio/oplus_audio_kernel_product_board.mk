# Build audio kernel driver
ifeq ($(call is-board-platform-in-list,kalama), true)
PRODUCT_PACKAGES += $(KERNEL_MODULES_OUT)/oplus_audio_extend.ko
PRODUCT_PACKAGES += $(KERNEL_MODULES_OUT)/oplus_audio_tfa98xx_v6.ko
endif

ifeq ($(call is-board-platform-in-list,pineapple), true)
PRODUCT_PACKAGES += $(KERNEL_MODULES_OUT)/oplus_audio_extend.ko
PRODUCT_PACKAGES += $(KERNEL_MODULES_OUT)/oplus_audio_tfa98xx_v6.ko
endif
ifeq ($(call is-board-platform-in-list,bengal), true)
PRODUCT_PACKAGES += $(KERNEL_MODULES_OUT)/oplus_audio_sipa.ko
PRODUCT_PACKAGES += $(KERNEL_MODULES_OUT)/oplus_audio_sipa_tuning.ko
PRODUCT_PACKAGES += $(KERNEL_MODULES_OUT)/oplus_audio_pa_manager.ko
endif # bengal supported
