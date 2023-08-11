ifeq ($(CONFIG_ARCH_KALAMA),y)
dtbo-y += nxp/kalama-nfc.dtbo \
	  nxp/kalama-nfc-mtp.dtbo \
	  nxp/kalama-nfc-qrd.dtbo \
	  nxp/kalama-nfc-cdp.dtbo

dtbo-y += nxp/kalama-v2-nfc.dtbo \
	  nxp/kalama-v2-nfc-mtp.dtbo \
	  nxp/kalama-v2-nfc-qrd.dtbo \
	  nxp/kalama-v2-nfc-cdp.dtbo

dtbo-y += st/kalama-nfc.dtbo \
	  st/kalama-nfc-mtp.dtbo \
	  st/kalama-nfc-cdp.dtbo

dtbo-y += st/kalama-v2-nfc.dtbo \
	  st/kalama-v2-nfc-mtp.dtbo \
	  st/kalama-v2-nfc-cdp.dtbo
endif

ifeq ($(CONFIG_ARCH_KHAJE),y)
dtbo-y += nxp/khaje-nfc-idp.dtbo \
          nxp/khaje-nfc-qrd.dtbo \
          nxp/khaje-nfc-qrd-hvdcp3p5.dtbo \
          nxp/khaje-nfc-qrd-nowcd9375.dtbo
endif

always-y	:= $(dtb-y) $(dtbo-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
