load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def define_oplus_local_modules():

    define_oplus_ddk_module(
        name = "oplus_sensor_ir_core",
        srcs = native.glob([
            "**/*.h",
            "oplus_consumer_ir/oplus_ir_core.c",
        ]),
        includes = ["oplus_consumer_ir"],
    )

    define_oplus_ddk_module(
        name = "oplus_sensor_kookong_ir_spi",
        srcs = native.glob([
            "**/*.h",
            "oplus_consumer_ir/oplus_ir_spi.c",
        ]),
        includes = ["oplus_consumer_ir"],
        ko_deps = [
            "//vendor/oplus/sensor/kernel/qcom:oplus_sensor_ir_core",
        ],
    )

    define_oplus_ddk_module(
        name = "oplus_sensor_deviceinfo",
        srcs = native.glob([
            "**/*.h",
            "sensor/oplus_sensor_devinfo.c",
            "sensor/oplus_press_cali_info.c",
            "sensor/oplus_pad_als_info.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_sensor_interact",
        srcs = native.glob([
            "**/*.h",
            "sensor/oplus_ssc_interact/oplus_ssc_interact.c",
        ]),
        includes = ["."],
        local_defines = ["CONFIG_OPLUS_SENSOR_FB_QC",
                         "CONFIG_OPLUS_SENSOR_DRM_PANEL_NOTIFY",
                         "CONFIG_OPLUS_SENSOR_DRM_PANEL_ADFR_MIN_FPS"],
        ko_deps = [
            "//vendor/oplus/sensor/kernel/qcom:oplus_sensor_feedback",
        ],
    )

    define_oplus_ddk_module(
        name = "oplus_sensor_feedback",
        srcs = native.glob([
            "**/*.h",
            "sensor/oplus_sensor_feedback/sensor_feedback.c",
        ]),
        includes = ["."],
        local_defines = ["CONFIG_OPLUS_SENSOR_DRM_PANEL_NOTIFY"],
    )

    define_oplus_ddk_module(
        name = "pseudo_sensor",
        srcs = native.glob([
            "**/*.h",
            "pseudo-sensor/pseudo_sensor.c",
        ]),
        includes = [],
        local_defines = ["CFG_OPLUS_ARCH_IS_QCOM"],
    )

    ddk_copy_to_dist_dir(
        name = "oplus_bsp_sensor",
        module_list = [
            "oplus_sensor_ir_core",
            "oplus_sensor_kookong_ir_spi",
            "oplus_sensor_deviceinfo",
            "oplus_sensor_interact",
            "oplus_sensor_feedback",
            "pseudo_sensor",
        ],
    )
