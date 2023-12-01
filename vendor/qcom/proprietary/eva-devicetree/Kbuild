ifneq ($(CONFIG_ARCH_QTI_VM), y)

ifeq ($(CONFIG_ARCH_KALAMA), y)
dtbo-y += kalama-eva.dtbo
endif

ifeq ($(CONFIG_ARCH_WAIPIO), y)
dtbo-y += waipio-eva.dtbo
endif

ifeq ($(CONFIG_ARCH_CAPE), y)
dtbo-y += cape-eva.dtbo
endif

else

ifeq ($(CONFIG_ARCH_KALAMA), y)
dtbo-y += trustedvm-kalama-eva-mtp.dtbo \
	trustedvm-kalama-eva-qrd.dtbo
endif

endif

always-y	:= $(dtb-y) $(dtbo-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
