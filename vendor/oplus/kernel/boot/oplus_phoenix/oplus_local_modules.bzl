load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def define_oplus_local_modules():

    define_oplus_ddk_module(
        name = "oplus_bsp_dfr_kmsg_wb",
        srcs = native.glob([
            "**/*.h",
            "oplus_kmsg_wb.c",
        ]),
        ko_deps = [
           "//vendor/oplus/kernel/boot/oplus_phoenix:oplus_bsp_dfr_phoenix",
        ],

        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_dfr_shutdown_speed",
        srcs = native.glob([
            "shutdown_speed.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_dfr_reboot_speed",
        srcs = native.glob([
            "**/*.h",
            "phoenix_reboot_speed.c",
        ]),
        ko_deps = [
           "//vendor/oplus/kernel/boot/oplus_phoenix:oplus_bsp_dfr_shutdown_speed",
        ],
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_dfr_phoenix",
        srcs = native.glob([
            "**/*.h",
            "op_bootprof.c",
            "phoenix_dump.c",
            "phoenix_watchdog.c",
            "phoenix_base.c",
        ]),

        includes = ["."],
        local_defines = ["TRACK_TASK_COMM","CONFIG_OPLUS_FEATURE_PHOENIX_MODULE"],
    )


    ddk_copy_to_dist_dir(
        name = "oplus_bsp_oplus_phoenix",
        module_list = [
            "oplus_bsp_dfr_kmsg_wb",
            "oplus_bsp_dfr_reboot_speed",
            "oplus_bsp_dfr_phoenix",
            "oplus_bsp_dfr_shutdown_speed"
        ],
    )
