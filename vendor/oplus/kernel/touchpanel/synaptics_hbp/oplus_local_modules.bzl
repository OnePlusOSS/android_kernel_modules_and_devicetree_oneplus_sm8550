load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def define_oplus_local_modules():

    define_oplus_ddk_module(
        name = "oplus_bsp_synaptics_tcm2",
        srcs = native.glob([
            "**/*.h",
            "syna_tcm2.c",
            "tcm/synaptics_touchcom_core_v1.c",
            "tcm/synaptics_touchcom_core_v2.c",
            "tcm/synaptics_touchcom_func_base.c",
            "tcm/synaptics_touchcom_func_touch.c",
            "tcm/synaptics_touchcom_func_reflash.c",
            "tcm/synaptics_touchcom_func_romboot.c",
            "syna_tcm2_platform_spi.c",
            "syna_tcm2_sysfs.c",
            "syna_tcm2_testing.c",
            "synaptics_common.c",
            "touchpanel_proc.c",
            "touch_comon_api/touch_comon_api.c",
            "touchpanel_autotest/touchpanel_autotest.c",
            "touchpanel_healthinfo/touchpanel_healthinfo.c",
            "touchpanel_healthinfo/touchpanel_exception.c",
        ]),
        ko_deps = [
            "//vendor/oplus/kernel/touchpanel/oplus_touchscreen_v2:oplus_bsp_tp_custom",
#            "//vendor/oplus/kernel/touchpanel/oplus_touchscreen_v2:oplus_bsp_tp_notify",	#built in-tree
        ],
        includes = ["."],
        local_defines = [
			"CONFIG_TOUCHPANEL_NOTIFY",
			"CONFIG_TOUCHPANEL_OPLUS_MODULE",
			"CONFIG_OF",
			"BUILD_BY_BAZEL",
		],
        conditional_defines = {
            "mtk":  ["CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY"],
            "qcom":  ["CONFIG_QCOM_PANEL_EVENT_NOTIFIER"],
        },
        header_deps = [
            "//vendor/oplus/kernel/touchpanel/oplus_touchscreen_v2:config_headers",
        ],
    )

    ddk_copy_to_dist_dir(
        name = "oplus_bsp_synaptics_tcm2",
        module_list = [
            "oplus_bsp_synaptics_tcm2",
        ],
    )
