load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def define_oplus_local_modules():

    define_oplus_ddk_module(
        name = "oplus_bsp_tri_key",
        srcs = native.glob([
            "**/*.h",
            "oplus_tri_key.c",
        ]),
        includes = ["."],
        conditional_defines = {
            "qcom":  ["CONFIG_QCOM_PANEL_EVENT_NOTIFIER"],
            "mtk":  ["CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY"],
        },
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_mxm_up",
        srcs = native.glob([
            "**/*.h",
            "hall_ic/hall_mxm1120_up.c"
        ]),
        ko_deps = [
            "//vendor/oplus/kernel/device_info/tri_state_key:oplus_bsp_tri_key",
        ],
        includes = ["."],
    )
    define_oplus_ddk_module(
        name = "oplus_bsp_mxm_down",
        srcs = native.glob([
            "**/*.h",
            "hall_ic/hall_mxm1120_down.c"
        ]),
        ko_deps = [
            "//vendor/oplus/kernel/device_info/tri_state_key:oplus_bsp_tri_key",
        ],
        includes = ["."],
    )
    
    define_oplus_ddk_module(
        name = "oplus_bsp_ist_up",
        srcs = native.glob([
            "**/*.h",
            "ist_hall_ic/hall_ist8801_up.c"
        ]),
        ko_deps = [
            "//vendor/oplus/kernel/device_info/tri_state_key:oplus_bsp_tri_key",
        ],
        includes = ["."],
    )
    define_oplus_ddk_module(
        name = "oplus_bsp_ist_down",
        srcs = native.glob([
            "**/*.h",
            "ist_hall_ic/hall_ist8801_down.c"
        ]),
        ko_deps = [
            "//vendor/oplus/kernel/device_info/tri_state_key:oplus_bsp_tri_key",
        ],
        includes = ["."],
    )
    
    ddk_copy_to_dist_dir(
        name = "oplus_bsp_tri_key",
        module_list = [
            "oplus_bsp_tri_key",
            "oplus_bsp_mxm_up",
            "oplus_bsp_mxm_down",
            "oplus_bsp_ist_up",
            "oplus_bsp_ist_down",
        ],
    )