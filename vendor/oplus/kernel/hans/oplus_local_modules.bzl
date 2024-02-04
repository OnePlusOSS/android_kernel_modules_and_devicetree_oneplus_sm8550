load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def define_oplus_local_modules():

    define_oplus_ddk_module(
        name = "oplus_sys_hans",
        srcs = native.glob([
            "hans.h",
            "hans.c",
            "hans_help.c",
            "hans_netfilter.c",
        ]),

        conditional_defines = {
            "mtk":  ["CONFIG_OPLUS_FEATURE_HANS_GKI"],
            "qcom": ["CONFIG_OPLUS_FEATURE_HANS"],
        },
        includes = ["."],
    )

    ddk_copy_to_dist_dir(
        name = "oplus_sys_hans",
        module_list = ["oplus_sys_hans"],
    )
