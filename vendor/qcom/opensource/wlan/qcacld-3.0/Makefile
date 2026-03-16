KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build

# The Make variable $(M) must point to the directory that contains the module
# source code (which includes this Makefile). It can either be an absolute or a
# relative path. If it is a relative path, then it must be relative to the
# kernel source directory (KERNEL_SRC). An absolute path can be obtained very
# easily through $(shell pwd). Generating a path relative to KERNEL_SRC is
# difficult and we accept some outside help by letting the caller override the
# variable $(M). Allowing a relative path for $(M) enables us to have the build
# system put output/object files (.o, .ko.) into a directory different from the
# module source directory.
M ?= $(shell pwd)

ifeq ($(WLAN_ROOT),)

KBUILD_OPTIONS += \
    WLAN_ROOT=$(KERNEL_SRC)/$(M) \
    WLAN_PLATFORM_ROOT=$(KERNEL_SRC)/$(M)/../platform \
    WLAN_COMMON_ROOT=cmn \
    WLAN_COMMON_INC=$(KERNEL_SRC)/$(M)/cmn \
    WLAN_FW_API=$(KERNEL_SRC)/$(M)/../fw-api \
    CONFIG_QCA_CLD_WLAN=m \
    CONFIG_CNSS_KIWI_V2=y \
    CONFIG_CNSS_OUT_OF_TREE=y \
    CONFIG_CNSS2=m \
    CONFIG_CNSS2_QMI=y \
    CONFIG_CNSS_QMI_SVC=m \
    CONFIG_CNSS_PLAT_IPC_QMI_SVC=m \
    CONFIG_CNSS_GENL=m \
    CONFIG_WCNSS_MEM_PRE_ALLOC=m \
    CONFIG_CNSS_UTILS=m \
    CONFIG_CNSS_HW_SECURE_DISABLE=y \
    CONFIG_BUS_AUTO_SUSPEND=y \
    WLAN_PROFILE=kiwi_v2 \
    DYNAMIC_SINGLE_CHIP=kiwi_v2 \
    MODNAME=qca_cld3_kiwi_v2 \
    DEVNAME=kiwi_v2 \
    BOARD_PLATFORM=kalama \
    WLAN_CTRL_NAME=wlan
endif

KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS=$(KERNEL_SRC)/../vendor/qcom/opensource/wlan/platform/Module.symvers

KBUILD_OPTIONS += $(KBUILD_EXTRA) # Extra config if any

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean $(KBUILD_OPTIONS)
