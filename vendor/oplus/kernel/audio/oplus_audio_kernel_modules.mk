# Build audio kernel driver
#add for tfa98xx bringup
AUDIO_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/tfa98xx-v6_dlkm.ko
AUDIO_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/audio_extend_dlkm.ko
