KALAMA_BASE_DTB += ../qcom/kalama.dtb ../qcom/kalama-v2.dtb
KALAMA_APQ_BASE_DTB += ../qcom/kalamap.dtb ../qcom/kalamap-v2.dtb

OPLUS_KALAMA_BOARDS += \
		wukong-21131-kalama-overlay.dtbo \
		salami-22811-kalama-overlay.dtbo \
		salami-22811-kalama-overlay-EVB.dtbo \
		salami-22811-kalama-overlay-T0.dtbo \
		wukong-21131-kalama-overlay-EVB.dtbo \
		xueying-22003-kalama-overlay.dtbo

kalama-overlays-dtb-y += $(OPLUS_KALAMA_BOARDS) $(KALAMA_BASE_DTB) $(KALAMA_APQ_BASE_DTB)
dtb-y += $(OPLUS_KALAMA_BOARDS) $(KALAMA_BASE_DTB) $(KALAMA_APQ_BASE_DTB)

always-y	:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
