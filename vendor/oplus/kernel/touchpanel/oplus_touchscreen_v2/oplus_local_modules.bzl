load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def define_oplus_local_modules():

    define_oplus_ddk_module(
        name = "oplus_bsp_tp_syna_common",
        srcs = native.glob([
            "**/*.h",
            "Synaptics/synaptics_touch_panel_remote.c",
            "Synaptics/synaptics_common.c",
        ]),
        includes = ["."],
        ko_deps = [
            "//vendor/oplus/kernel/touchpanel/oplus_touchscreen_v2:oplus_bsp_tp_custom",
            "//vendor/oplus/kernel/touchpanel/oplus_touchscreen_v2:oplus_bsp_tp_common",
        ],
#        local_defines = ["CONFIG_REMOVE_OPLUS_FUNCTION"],
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_tp_tcm_S3910",
        srcs = native.glob([
            "**/*.h",
            "Synaptics/Syna_tcm_S3910/synaptics_tcm_S3910.c",
            "Synaptics/Syna_tcm_S3910/synaptics_tcm_device_S3910.c",
        ]),
        ko_deps = [
            "//vendor/oplus/kernel/touchpanel/oplus_touchscreen_v2:oplus_bsp_tp_custom",
            "//vendor/oplus/kernel/touchpanel/oplus_touchscreen_v2:oplus_bsp_tp_common",
            "//vendor/oplus/kernel/touchpanel/oplus_touchscreen_v2:oplus_bsp_tp_syna_common",
        ],
        includes = ["."],
#        local_defines = ["CONFIG_REMOVE_OPLUS_FUNCTION"],
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_tp_custom",
        srcs = native.glob([
            "**/*.h",
            "touch_custom/touch.c"
        ]),
        includes = ["."],
        local_defines = ["CONFIG_OPLUS_FEATURE_OPROJECT"],
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_tp_common",
        srcs = native.glob([
            "**/*.h",
            "util_interface/touch_interfaces.c",
            "touch_comon_api/touch_comon_api.c",
            "touchpanel_healthinfo/touchpanel_healthinfo.c",
            "touchpanel_healthinfo/touchpanel_exception.c",
            "touchpanel_autotest/touchpanel_autotest.c",
            "touchpanel_prevention/touchpanel_prevention.c",
            "touchpanel_common_driver.c",
            "touchpanel_proc.c",
            "tp_ioctl.c",
            "touchpanel_tui_support/touchpanel_tui_support.c",
            "message_list.c",
            "touch_pen/touch_pen_core.c",
            "touch_pen/touch_pen_algo.c",
        ]),
        ko_deps = [
            "//vendor/oplus/kernel/touchpanel/oplus_touchscreen_v2:oplus_bsp_tp_custom",
#            "//vendor/oplus/kernel/touchpanel/oplus_touchscreen_v2:oplus_bsp_fw_update",	#built in-tree
#            "//vendor/oplus/kernel/touchpanel/oplus_touchscreen_v2:oplus_bsp_tp_notify",	#built in-tree
#            "//vendor/oplus/kernel/device_info/device_info:oplus_bsp_device_info",		#built in-tree
        ],
        includes = ["."],
        local_defines = ["CONFIG_TOUCHPANEL_NOTIFY", "CONFIG_TOUCHPANEL_OPLUS_MODULE"],
        conditional_defines = {
            "qcom":  ["CONFIG_QCOM_PANEL_EVENT_NOTIFIER"],
            "mtk":  ["CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY"],
        },
    )

    ddk_headers(
        name = "config_headers",
        hdrs  = native.glob([
            "**/*.h",
        ]),
        includes = [".","touchpanel_notify"],
    )

    ddk_copy_to_dist_dir(
        name = "oplus_bsp_tp",
        module_list = [
            "oplus_bsp_tp_syna_common",
            "oplus_bsp_tp_tcm_S3910",
            "oplus_bsp_tp_custom",
            "oplus_bsp_tp_common",
        ],
    )
