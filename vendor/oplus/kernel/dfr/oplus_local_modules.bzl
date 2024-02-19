load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def define_oplus_local_modules():

    define_oplus_ddk_module(
        name = "oplus_bsp_dfr_combkey_monitor",
        srcs = native.glob([
            "**/*.h",
            "common/combkey_monitor/combkey_monitor.c",
        ]),
        includes = ["."],
        ko_deps = [
            "//vendor/oplus/kernel/dfr:oplus_bsp_dfr_keyevent_handler",
            "//vendor/oplus/kernel/dfr:oplus_bsp_dfr_theia",
        ],
        local_defines = ["CONFIG_OPLUS_FEATURE_THEIA","CONFIG_OPLUS_FEATURE_KEYEVENT_HANDLER"],
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_dfr_hung_task_enhance",
        srcs = native.glob([
            "**/*.h",
            "common/hung_task_enhance/hung_task_enhance.c",
        ]),

        conditional_defines = {
            "mtk":  ["CONFIG_OPLUS_SYSTEM_KERNEL_MTK"],
            "qcom": ["CONFIG_OPLUS_SYSTEM_KERNEL_QCOM"],
        },
        includes = ["."],
        ko_deps = [
            "//vendor/oplus/kernel/dfr:oplus_bsp_dfr_theia",
        ],
        local_defines = [
            "CONFIG_OPLUS_FEATURE_HUNG_TASK_ENHANCE",
            "CONFIG_OPLUS_FEATURE_THEIA",
            "CONFIG_OPLUS_FEATURE_DEATH_HEALER",
            "CONFIG_OPLUS_BSP_DFR_USERSPACE_BACKTRACE",
        ],
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_dfr_init_watchdog",
        srcs = native.glob([
            "**/*.h",
            "common/init_watchdog/init_watchdog.c",
        ]),
        includes = ["."],
        ko_deps = [
            "//vendor/oplus/kernel/dfr:oplus_bsp_dfr_theia",
        ],
        local_defines = ["CONFIG_OPLUS_FEATURE_THEIA"],
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_dfr_keyevent_handler",
        srcs = native.glob([
            "**/*.h",
            "common/keyevent_handler/keyevent_handler.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_dfr_last_boot_reason",
        srcs = native.glob([
            "**/*.h",
            "common/last_boot_reason/last_boot_reason.c",
        ]),
        includes = ["."],
        local_defines = ["CONFIG_OPLUS_FEATURE_SHUTDOWN_DETECT"],
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_dfr_fdleak_check",
        srcs = native.glob([
            "**/*.h",
            "common/oplus_fdleak/oplus_fdleak_check.c",
        ]),
        conditional_defines = {
            "mtk":  ["CONFIG_OPLUS_SYSTEM_KERNEL_MTK"],
            "qcom": ["CONFIG_OPLUS_SYSTEM_KERNEL_QCOM"],
        },
#        header_deps = [
#            "//vendor/oplus/kernel/cpu:config_headers",
#        ],
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_dfr_oplus_saupwk",
        srcs = native.glob([
            "**/*.h",
            "common/oplus_saupwk/oplus_saupwk.c",
        ]),
        conditional_defines = {
            "mtk":  ["CONFIG_OPLUS_SYSTEM_KERNEL_MTK"],
            "qcom": ["CONFIG_OPLUS_SYSTEM_KERNEL_QCOM"],
        },
        includes = ["."],
        local_defines = ["CONFIG_OPLUS_FEATURE_SAUPWK"],
    )


    define_oplus_ddk_module(
        name = "oplus_bsp_dfr_shutdown_detect",
        srcs = native.glob([
            "**/*.h",
            "common/shutdown_detect/shutdown_detect.c",
        ]),
        includes = ["."],
        local_defines = ["CONFIG_OPLUS_FEATURE_SHUTDOWN_DETECT","CONFIG_OPLUS_BSP_DFR_USERSPACE_BACKTRACE"],
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_dfr_ubt",
        srcs = native.glob([
            "**/*.h",
            "common/oplus_bsp_dfr_ubt/oplus_bsp_dfr_ubt.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_dfr_theia",
        srcs = native.glob([
            "**/*.h",
            "common/theia/black_screen_check.c",
            "common/theia/bright_screen_check.c",
            "common/theia/theia_kevent_kernel.c",
            "common/theia/powerkey_monitor.c",
            "common/theia/theia_send_event.c",
        ]),
        conditional_defines = {
            "mtk":  ["CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY"],
            "qcom": ["CONFIG_QCOM_PANEL_EVENT_NOTIFIER"],
        },
        includes = ["."],
        local_defines = ["CONFIG_OPLUS_FEATURE_THEIA_MODULE"],
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_dfr_force_shutdown",
        srcs = native.glob([
            "**/*.h",
        ]),
        conditional_srcs = {
            "CONFIG_OPLUS_DDK_MTK": {
                True:  ["mtk/mtk_shutdown_reboot/mtk_force_shutdown.c"],
                False: ["qcom/force_shutdown/force_shutdown.c"],
            }
        },
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_dfr_pmic_monitor",
        srcs = native.glob([
            "**/*.h",
        ]),
        conditional_srcs = {
            "CONFIG_OPLUS_DDK_MTK": {
                True:  ["mtk/oplus_pmic_monitor_mtk/oplus_pmic_info_get_mtk.c","mtk/oplus_pmic_monitor_mtk/main.c"],
                False: ["qcom/qcom_pmic_monitor/oplus_pmic_info_smem.c","qcom/qcom_pmic_monitor/main.c","qcom/qcom_pmic_monitor/oplus_pmic_machine_state.c"],
            }
        },
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_dfr_dump_device_info",
        srcs = native.glob([
            "**/*.h",
            "qcom/dump_device_info/dump_device_info.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_dfr_dump_reason",
        srcs = native.glob([
            "**/*.h",
            "qcom/dump_reason/dump_reason.c",
        ]),
        ko_deps = [
            "//vendor/oplus/kernel/dfr:oplus_bsp_dfr_dump_device_info",
        ],
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_dfr_pmic_watchdog",
        srcs = native.glob([
            "**/*.h",
            "qcom/qcom_pmicwd/qcom_pmicwd.c",
            "qcom/qcom_pmicwd/qcom_pwkpwr.c",
        ]),
        includes = ["."],
    )

    ddk_copy_to_dist_dir(
        name = "oplus_bsp_dfr",
        module_list = [
            "oplus_bsp_dfr_combkey_monitor",
            "oplus_bsp_dfr_hung_task_enhance",
            "oplus_bsp_dfr_init_watchdog",
            "oplus_bsp_dfr_keyevent_handler",
            "oplus_bsp_dfr_last_boot_reason",
            "oplus_bsp_dfr_fdleak_check",
            "oplus_bsp_dfr_shutdown_detect",
            "oplus_bsp_dfr_ubt",
            "oplus_bsp_dfr_theia",
            "oplus_bsp_dfr_force_shutdown",
            "oplus_bsp_dfr_pmic_monitor",
            "oplus_bsp_dfr_dump_device_info",
            "oplus_bsp_dfr_dump_reason",
            "oplus_bsp_dfr_pmic_watchdog",
        ],
    )
