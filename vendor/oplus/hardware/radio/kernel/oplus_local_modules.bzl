load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def define_oplus_local_modules():

    define_oplus_ddk_module(
        name = "oplus_mdmfeature",
        srcs = native.glob([
            "mdmfeature/oplus_mdmfeature.h",
            "mdmfeature/oplus_mdmfeature.c",
        ]),
        includes = ["."],
    )

    ddk_copy_to_dist_dir(
        name = "oplus_hardware_radio",
        module_list = [
            "oplus_mdmfeature",
        ]
    )
