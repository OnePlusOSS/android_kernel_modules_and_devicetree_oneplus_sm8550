load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def define_oplus_local_modules():

    define_oplus_ddk_module(
        name = "oplus_sensor_kookong_ir_pwm",
        srcs = native.glob([
            "**/*.h",
            "oplus_consumer_ir/oplus_ir_pwm.c",
        ]),
        includes = ["oplus_consumer_ir"],
        ko_deps = [
            "//vendor/oplus/sensor/kernel/sensorhub:oplus_sensor_ir_core",
        ],
        local_defines = [],
        conditional_defines = {
            "mtk":  ["CONFIG_OPLUS_SENSOR_CONSUMER_IR_MTK"],
        },
    )

    define_oplus_ddk_module(
        name = "oplus_sensor_ir_core",
        srcs = native.glob([
            "**/*.h",
            "oplus_consumer_ir/oplus_ir_core.c",
        ]),
        includes = ["oplus_consumer_ir"],
        local_defines = [],
    )

    define_oplus_ddk_module(
        name = "pseudo_sensor",
        srcs = native.glob([
            "**/*.h",
            "pseudo-sensor/pseudo_sensor.c",
        ]),
        includes = [],
        local_defines = ["CFG_OPLUS_ARCH_IS_MTK"],
    )

    ddk_copy_to_dist_dir(
        name = "oplus_sensor_consumer_ir",
        module_list = [
            "oplus_sensor_ir_core",
            "oplus_sensor_kookong_ir_pwm",
            "pseudo_sensor",
        ],
    )
