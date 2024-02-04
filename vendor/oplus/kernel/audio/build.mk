MY_LOCAL_PATH := $(call my-dir)

# This makefile is only for DLKM
ifneq ($(findstring vendor,$(MY_LOCAL_PATH)),)

#This makefile is only for soft links
ifneq ($(findstring audio-kernel,$(MY_LOCAL_PATH)),)

DLKM_DIR := $(TOP)/device/qcom/common/dlkm

OPLUS_AUDIO_SRC_FILES := \
	$(wildcard $(MY_LOCAL_PATH)/*) \
	$(wildcard $(MY_LOCAL_PATH)/*/*) \
	$(wildcard $(MY_LOCAL_PATH)/*/*/*) \
	$(wildcard $(MY_LOCAL_PATH)/*/*/*/*) \
	$(wildcard $(MY_LOCAL_PATH)/*/*/*/*/*)

# Build/Package only in case of supported target
########################### Audio extend driver  ###########################
#Add for audio extend dirver
include $(CLEAR_VARS)
LOCAL_SRC_FILES           := $(OPLUS_AUDIO_SRC_FILES)
LOCAL_MODULE              := oplus_audio_extend.ko
LOCAL_MODULE_KBUILD_NAME  := oplus/qcom/oplus_audio_extend.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
###########################################################

########################### TFA98xx-v6 CODEC  ###########################
#ifdef OPLUS_ARCH_EXTENDS
#add for tfa98xx bringup
include $(CLEAR_VARS)
LOCAL_SRC_FILES           := $(OPLUS_AUDIO_SRC_FILES)
LOCAL_MODULE              := oplus_audio_tfa98xx_v6.ko
LOCAL_MODULE_KBUILD_NAME  := oplus/codecs/tfa98xx-v6/oplus_audio_tfa98xx_v6.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
#endif /* OPLUS_ARCH_EXTENDS */
###########################################################

########################### SIPA  ###########################
#add for sia pa bringup
include $(CLEAR_VARS)
LOCAL_SRC_FILES           := $(OPLUS_AUDIO_SRC_FILES)
LOCAL_MODULE              := oplus_audio_sipa.ko
LOCAL_MODULE_KBUILD_NAME  := oplus/codecs/sipa/oplus_audio_sipa.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
###########################################################
########################### SIPA TUNING  ###########################
#add for sia pa bringup
include $(CLEAR_VARS)
LOCAL_SRC_FILES           := $(OPLUS_AUDIO_SRC_FILES)
LOCAL_MODULE              := oplus_audio_sipa_tuning.ko
LOCAL_MODULE_KBUILD_NAME  := oplus/codecs/sipa/oplus_audio_sipa_tuning.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
###########################################################
########################### PA MANAGER  ###########################
#add for pa manager
include $(CLEAR_VARS)
LOCAL_SRC_FILES           := $(OPLUS_AUDIO_SRC_FILES)
LOCAL_MODULE              := oplus_audio_pa_manager.ko
LOCAL_MODULE_KBUILD_NAME  := oplus/qcom/oplus_speaker_manager/oplus_audio_pa_manager.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
###########################################################
endif # audio-kernel
endif # DLKM check
