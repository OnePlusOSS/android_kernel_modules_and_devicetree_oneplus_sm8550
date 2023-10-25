# Build audio kernel driver
#add for tfa98xx bringup
PRODUCT_PACKAGES += $(KERNEL_MODULES_OUT)/tfa98xx-v6_dlkm.ko
PRODUCT_PACKAGES += $(KERNEL_MODULES_OUT)/audio_extend_dlkm.ko
