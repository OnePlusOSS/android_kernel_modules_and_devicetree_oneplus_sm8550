ifeq ($(CONFIG_ARCH_WAIPIO),y)
dtbo-y += waipio-cnss.dtbo
dtbo-y += waipio-kiwi-cnss.dtbo
endif

ifeq ($(CONFIG_ARCH_KALAMA),y)
dtbo-y += kalama-cnss.dtbo
endif

ifeq ($(CONFIG_ARCH_KHAJE),y)
dtbo-y += khaje-cnss.dtbo
endif

ifeq ($(CONFIG_ARCH_SA8155),y)
dtbo-y += sa8155p-cnss.dtbo
endif

ifeq ($(CONFIG_ARCH_SA8195),y)
dtbo-y += sa8195p-cnss.dtbo
endif

ifeq ($(CONFIG_ARCH_SDXPINN),y)
dtbo-y += sdxpinn-cnss.dtbo
endif

always-y	:= $(dtb-y) $(dtbo-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
