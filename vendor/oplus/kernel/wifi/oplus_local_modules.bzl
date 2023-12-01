load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def define_oplus_local_modules():

    define_oplus_ddk_module(
        name = "oplus_connectivity_routerboost",
        srcs = native.glob([
            "**/*.h",
            "oplus_connectivity_routerboost/oplus_routerboost.c",
            "oplus_connectivity_routerboost/oplus_routerboost_game_monitor.c"
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_connectivity_sla",
        srcs = native.glob([
            "oplus_connectivity_sla/oplus_connectivity_sla.c"
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_wifi_wsa",
        srcs = native.glob([
            "oplus_wifi_wsa/oplus_wifismartantenna.c"
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_wificapcenter",
        srcs = native.glob([
            "**/*.h",
            "oplus_wificapcenter/oplus_wificapcenter.c"
        ]),
        includes = ["."],
    )

    ddk_headers(
        name = "config_headers",
        hdrs  = native.glob([
            "**/*.h",
        ]),
        includes = ["."],
    )

    ddk_copy_to_dist_dir(
        name = "oplus_wifi",
        module_list = [
            "oplus_connectivity_routerboost",
            "oplus_connectivity_sla",
            "oplus_wifi_wsa",
            "oplus_wificapcenter"
        ],
    )
