load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def define_oplus_local_modules():

    define_oplus_ddk_module(
        name = "oplus_sensor_deviceinfo",
        srcs = native.glob([
            "*.h",
            "*.c",
        ]),
        includes = ["."],
        ko_deps = [
        ],
        local_defines = ["CONFIG_OPLUS_SENSOR_MTK68XX"],
    )

    ddk_copy_to_dist_dir(
        name = "oplus_sensor_deviceinfo",
        module_list = [
            "oplus_sensor_deviceinfo",
        ],
    )
