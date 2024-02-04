// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
/*
###############################################################################
## File: phoenix_watchdog.c
## Description : add for project phenix(hang )
##
## Version:  1.0
################################################################################
*/
#include "oplus_phoenix.h"
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/version.h>
#include <soc/oplus/boot/boot_mode.h>
#include <soc/oplus/boot/oplus_project.h>

#define DELAY_TIME 30
#define MAX_CMD_LENGTH 32
#define DEFAULT_PHX_WD_PET_TIME (60 * 4)

static int hang_oplus_main_on = 1; /* default on */
static int hang_oplus_recovery_method = RESTART_AND_RECOVERY;
static int phx_hlos_wd_pet_time = DEFAULT_PHX_WD_PET_TIME;

static bool is_qcom_hardware;
static bool is_userdebug;

static char *bootargs_ptr;
#define BOOTARGS_LEN (4 * 1024) /* 4K */

static void reinitialze_pet_time_for_debug_build(void)
{
	if (is_userdebug) {
		phx_hlos_wd_pet_time = 60 * 5;
	}
	if (AGING == get_eng_version()) {
		phx_hlos_wd_pet_time = 60 * 10; /* aging test version */
	}
	if (HIGH_TEMP_AGING == get_eng_version()) {
		phx_hlos_wd_pet_time = 60 * 10; /* hightemp aging test version */
	}

#ifdef CONFIG_MEMLEAK_DETECT_THREAD
	if (AGING != get_eng_version()) {
		phx_hlos_wd_pet_time += 60 * 5;
	}
#endif
}

int is_phoenix_enable(void)
{
	return hang_oplus_main_on;
}

EXPORT_SYMBOL(is_phoenix_enable);

extern char boot_from[];
static void phx_pet(void)
{
	/* give abnormal boot another timeslice */
	if (strncmp(boot_from, "normal", 6)) {
		schedule_timeout_interruptible(phx_hlos_wd_pet_time * HZ);
	}

	if (phx_is_system_boot_completed()) {
		return;
	}

	/* for GSI detecting, boot stage will not have any string */
	if (phx_get_current_stage_len() == 0) {
		return;
	}

	/* for debug/eng ver, not reboot ... */
	if (is_userdebug || (get_eng_version() == AGING)) {
		PHX_KLOG_INFO(
			"will not set ERROR_HANG_OPLUS with userdebug/aging version\n");
		return;
	}

	PHX_KLOG_INFO("phoenix watchdog pet!\n");
	phx_set_boot_error(ERROR_HANG_OPLUS);
	return;
}

/* start phoenix high level os watchdog */
static int phoenix_watchdog_kthread(void *dummy)
{
	schedule_timeout_interruptible(phx_hlos_wd_pet_time * HZ);
	phx_pet();
	return 0;
}

static int phx_is_normal_mode_qcom(void)
{
#if IS_ENABLED(CONFIG_OPLUS_SYSTEM_KERNEL_QCOM)
	if (op_is_monitorable_boot()) {
		return 1;
	} else {
		return 0;
	}
#else
	return 1;
#endif /* CONFIG_OPLUS_SYSTEM_KERNEL_QCOM */
}

/* copy mtk_boot_common.h */
#define NORMAL_BOOT 0
#define ALARM_BOOT 7
#define SILENCE_BOOT 12

static int phx_is_normal_mode_mtk(void)
{
	int mtk_boot_mode = 0;

	mtk_boot_mode = get_boot_mode();
	PHX_KLOG_INFO("mtk_boot_mode: %d\n", mtk_boot_mode);

	if ((mtk_boot_mode == NORMAL_BOOT) || (mtk_boot_mode == ALARM_BOOT) ||(mtk_boot_mode == SILENCE_BOOT)) {
		return 1;
	} else {
		return 0;
	}
}

static const char *phx_get_machine_name(void)
{
	int ret;
	const char *str;
	struct device_node *np;

	np = of_find_node_by_path("/");
	ret = of_property_read_string(np, "model", &str);
	if (!ret) {
		return str;
	}

	ret = of_property_read_string(np, "compatible", &str);
	if (!ret) {
		return str;
	}

	return "unknown";
}

static int phx_is_qcom_platform(void)
{
	const char *platform_name;
	char *substr;
	int i;
	const char *const qcom_platform_keywords[] = { "Qualcomm", "SDM" };

	platform_name = phx_get_machine_name();
	for (i = 0; i < ARRAY_SIZE(qcom_platform_keywords); i++) {
		substr = strstr(platform_name, qcom_platform_keywords[i]);
		if (substr) {
			PHX_KLOG_INFO("Qcom platform");
			return 1;
		}
	}

	if (is_qcom_hardware) {
		PHX_KLOG_INFO("Qcom platform hardware");
		return 1;
	}

	PHX_KLOG_INFO("MTK platform");
	return 0;
}

int phx_is_normal_mode(void)
{
	return phx_is_qcom_platform() ? phx_is_normal_mode_qcom() :
					phx_is_normal_mode_mtk();
}
EXPORT_SYMBOL(phx_is_normal_mode);

int phx_is_userdebug(void)
{
	return is_userdebug;
}
EXPORT_SYMBOL(phx_is_userdebug);

int phx_is_boot_from_normal(void)
{
        if (strncmp(boot_from, "normal", 6)) {
		return 0;
	} else {
		return 1;
	}
}
EXPORT_SYMBOL(phx_is_boot_from_normal);

static void phx_parse_cmdline(void)
{
	char *str;

	if (strstr(bootargs_ptr, "androidboot.hardware=qcom")) {
		is_qcom_hardware = true;
	}

	if (strstr(bootargs_ptr, "buildvariant=userdebug")) {
		is_userdebug = true;
	}

	str = strstr(bootargs_ptr, "phx_rus_conf.main_on=");
	if (str) {
		str += strlen("phx_rus_conf.main_on=");
		get_option(&str, &hang_oplus_main_on);
		PHX_KLOG_INFO("hang_oplus_main_on %d\n", hang_oplus_main_on);
	}

	/* nouse ? */
	str = strstr(bootargs_ptr, "phx_rus_conf.recovery_method=");
	if (str) {
		str += strlen("phx_rus_conf.recovery_method=");
		get_option(&str, &hang_oplus_recovery_method);
		PHX_KLOG_INFO("hang_oplus_recovery_method %d\n",
			      hang_oplus_recovery_method);
	}

	str = strstr(bootargs_ptr, "phx_rus_conf.kernel_time=");
	if (str) {
		str += strlen("phx_rus_conf.kernel_time=");
		get_option(&str, &phx_hlos_wd_pet_time);
		PHX_KLOG_INFO("phx_hlos_wd_pet_time %d\n",
			      phx_hlos_wd_pet_time);
	}
}

static int phx_prepare_bootargs(void)
{
	int ret;
	struct device_node *of_chosen;

	bootargs_ptr = (char *)kmalloc(BOOTARGS_LEN, GFP_KERNEL);
	if (!bootargs_ptr) {
		PHX_KLOG_ERROR("kmalloc for bootargs ptr failed !\n");
		return -1;
	}

	of_chosen = of_find_node_by_path("/chosen");
	if (!of_chosen) {
		of_chosen = of_find_node_by_path("/chosen@0");
	}

	if (of_chosen) {
		ret = of_property_read_string(of_chosen, "bootargs",
					      (const char **)&bootargs_ptr);
		if (ret) {
			PHX_KLOG_ERROR("try to get bootargs failed !");
			return -1;
		}
	}

	phx_parse_cmdline();

	return 0;
}

void __init phoenix_hlos_watchdog_init(void)
{
	int ret;

	ret = phx_prepare_bootargs();
	if (ret) {
		BUG_ON(1);
	}

	PHX_KLOG_INFO("phoenix hlos watchdog: %s\n",
		      hang_oplus_main_on ? "on" : "off");
	if (hang_oplus_main_on && phx_is_normal_mode()) {
		reinitialze_pet_time_for_debug_build();
		PHX_KLOG_ERROR("phoenix hlos watchdog pet time: %d\n",
			       phx_hlos_wd_pet_time);
		kthread_run(phoenix_watchdog_kthread, NULL,
			    "phoenix_hlos_watchdog");
	}
}
/* arch_initcall(phoenix_hlos_watchdog_init); */

MODULE_DESCRIPTION("PHOENIX HLOS WATCHDOG");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Bright.Zhang <bright.zhang>");
