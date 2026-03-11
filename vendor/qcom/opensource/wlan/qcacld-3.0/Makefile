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
# WLAN_ROOT must contain an absolute path (i.e. not a relative path)
KBUILD_OPTIONS := WLAN_ROOT=$(shell cd $(KERNEL_SRC); readlink -e $(M))

# MODNAME should be qca_cld3_wlan for helium based wear target
ifeq (qca_cld3, $(WLAN_WEAR_CHIPSET))
KBUILD_OPTIONS += MODNAME?=$(WLAN_WEAR_CHIPSET)_wlan
else
KBUILD_OPTIONS += MODNAME?=wlan
endif

#By default build for CLD
WLAN_SELECT := CONFIG_QCA_CLD_WLAN=m
KBUILD_OPTIONS += CONFIG_QCA_WIFI_ISOC=0
KBUILD_OPTIONS += CONFIG_QCA_WIFI_2_0=1
KBUILD_OPTIONS += CONFIG_CNSS_KIWI_V2=y CONFIG_CNSS_OUT_OF_TREE=y CONFIG_CNSS2=m CONFIG_CNSS2_QMI=y CONFIG_CNSS_QMI_SVC=m CONFIG_CNSS_PLAT_IPC_QMI_SVC=m CONFIG_CNSS_GENL=m CONFIG_WCNSS_MEM_PRE_ALLOC=m CONFIG_CNSS_UTILS=m WLAN_PROFILE=kiwi_v2 MODNAME=kiwi_v2 DEVNAME=kiwi_v2 BOARD_PLATFORM=kalama WLAN_CTRL_NAME=wlan CFG80211_RU_PUNCT_NOTIFY=y SUPPORT_WDEV_CFG80211_VENDOR_EVENT_ALLOC=y KCFLAGS="-DLINUX_VERSION_CODE=331687 -DCFG80211_SINGLE_NETDEV_MULTI_LINK_SUPPORT"
KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS=$(KERNEL_SRC)/../vendor/qcom/opensource/wlan/platform/Module.symvers
KBUILD_OPTIONS += WLAN_ROOT=$(KERNEL_SRC)/../../vendor/qcom/opensource/wlan/qcacld-3.0/.kiwi_v2 WLAN_COMMON_INC=$(KERNEL_SRC)/../../vendor/qcom/opensource/wlan/qca-wifi-host-cmn WLAN_FW_API=$(KERNEL_SRC)/../../vendor/qcom/opensource/wlan/fw-api
KBUILD_OPTIONS += $(WLAN_SELECT)
KBUILD_OPTIONS += $(KBUILD_EXTRA) # Extra config if any
endif

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean $(KBUILD_OPTIONS)
