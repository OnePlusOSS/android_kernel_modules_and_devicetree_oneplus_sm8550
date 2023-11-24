ifneq ($(findstring $(TARGET_BOARD_PLATFORM),taro lahaina kalama),)
$(warning explorer uses android.mk for QCOM)
###########################################################
#For explorer on QCOM platform
MMRM_BOARDS := taro kalama

LOCAL_PATH:= $(call my-dir)

# Path to DLKM make scripts
DLKM_DIR := $(TOP)/device/qcom/common/dlkm

EXPLORER_BLD_DIR := vendor/oplus/kernel

# This is set once per LOCAL_PATH, not per (kernel) module
KBUILD_OPTIONS := EXPLORER_ROOT=$(EXPLORER_BLD_DIR)
KBUILD_OPTIONS += MODNAME=explorer
KBUILD_OPTIONS += BOARD_PLATFORM=$(TARGET_BOARD_PLATFORM)

ifeq ($(call is-board-platform-in-list, $(MMRM_BOARDS)),true)
	KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS=$(shell pwd)/$(call intermediates-dir-for,DLKM,camera-module-symvers)/Module.symvers
endif

EXPLORER_SRC_FILES := \
        $(wildcard $(LOCAL_PATH)/*) \
        $(wildcard $(LOCAL_PATH)/*/*) \
        $(wildcard $(LOCAL_PATH)/*/*/*) \
        $(wildcard $(LOCAL_PATH)/*/*/*/*)

# DLKM_DIR was moved for JELLY_BEAN (PLATFORM_SDK 16)
#ifeq ($(call is-platform-sdk-version-at-least,16),true)
#       DLKM_DIR := $(TOP)/device/qcom/common/dlkm
#else
#       DLKM_DIR := build/dlkm
#endif # platform-sdk-version

include $(CLEAR_VARS)
LOCAL_SRC_FILES := $(EXPLORER_SRC_FILES)
LOCAL_MODULE := explorer.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_DEBUG_ENABLE := true
#LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_PATH   := $(KERNEL_MODULES_OUT)

ifeq ($(call is-board-platform-in-list, $(MMRM_BOARDS)),true)
	LOCAL_REQUIRED_MODULES        := camera-module-symvers
	LOCAL_ADDITIONAL_DEPENDENCIES := $(call intermediates-dir-for,DLKM,camera-module-symvers)/Module.symvers
endif

ifeq ($(TARGET_BOARD_PLATFORM), lahaina)
# Include Kernel DLKM Android.mk target to place generated .ko file in image
include $(DLKM_DIR)/AndroidKernelModule.mk
else
include $(DLKM_DIR)/Build_external_kernelmodule.mk
endif

#End for QCOM
###########################################################

else
$(warning explorer uses android.mk for MTK)

###########################################################
#For explorer on MTK platform

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := explorer.ko
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE_OWNER := oplus
LOCAL_INIT_RC := init.explorer_mtk.rc

ifeq ($(TARGET_OUT_VENDOR),)
LOCAL_MODULE_PATH := $(ALPS_OUT)/vendor/lib/modules
else
LOCAL_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules
endif

include $(MTK_KERNEL_MODULE)

#End for MTK
###########################################################

endif