// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022-2030 Oplus. All rights reserved.
 * Description : combination key monitor, such as volup + pwrkey
 * Version : 1.0
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": %s: %d: " fmt, __func__, __LINE__

#include <linux/types.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/workqueue.h>
#include <linux/notifier.h>
#include <linux/input.h>
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_KEYEVENT_HANDLER)
#include "../../include/keyevent_handler.h"
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_THEIA)
#include "../../include/theia_send_event.h"
#include "../../include/theia_bright_black_check.h"
#endif

#define KEY_DOWN_VALUE 1
#define KEY_UP_VALUE 0

static struct delayed_work g_check_combkey_long_press_work;
static struct delayed_work g_check_pwrkey_long_press_work;
#define CHECK_COMBKEY_LONG_PRESS_MS 6000
#define CHECK_PWRKEY_LONG_PRESS_MS 8000

static bool is_pwrkey_down;
static bool is_volumup_down;
static bool is_volumup_pwrkey_down;

static unsigned int combkey_monitor_events[] = {
	KEY_POWER,
	KEY_VOLUMEUP,
};
static size_t combkey_monitor_events_nr = ARRAY_SIZE(combkey_monitor_events);

static void combkey_long_press_callback(struct work_struct *work)
{
	pr_info("called. send pwr_resin_bark to theia.\n");
	theia_send_event(THEIA_EVENT_KPDPWR_RESIN_BARK, THEIA_LOGINFO_KERNEL_LOG | THEIA_LOGINFO_ANDROID_LOG,
		0, "kpdpwr_resin_bark happen");
}

static void pwrkey_long_press_callback(struct work_struct *work)
{
	pr_info("called. send long press pwrkey to theia.\n");
	theia_send_event(THEIA_EVENT_PWK_LONGPRESS, THEIA_LOGINFO_KERNEL_LOG
		 | THEIA_LOGINFO_ANDROID_LOG | THEIA_LOGINFO_DUMPSYS_SF | THEIA_LOGINFO_BINDER_INFO,
		0, "pwrkey long press happen");
}

static int combkey_monitor_notifier_call(struct notifier_block *nb, unsigned long type, void *data)
{
	struct keyevent_notifier_param *param = data;

	pr_info("called. event_code = %u, value = %d\n", param->keycode, param->down);

	if (param->keycode == KEY_POWER) {
		pr_info("pwrkey handle enter.\n");
		if (param->down == KEY_DOWN_VALUE) {
			is_pwrkey_down = true;
			pr_info("pwrkey pressed, call pwrkey monitor checker.\n");
			black_screen_timer_restart();
			bright_screen_timer_restart();
		} else if (param->down == KEY_UP_VALUE) {
			is_pwrkey_down = false;
		}
	} else if (param->keycode == KEY_VOLUMEUP) {
		pr_info("volumup key handle enter\n");
		if (param->down == KEY_DOWN_VALUE)
			is_volumup_down = true;
		else if (param->down == KEY_UP_VALUE)
			is_volumup_down = false;
	}

	/* combination key pressed, start to calculate duration */
	if (is_pwrkey_down && is_volumup_down) {
		is_volumup_pwrkey_down = true;
		pr_info("volup_pwrkey combination key pressed.\n");
		schedule_delayed_work(&g_check_combkey_long_press_work, msecs_to_jiffies(CHECK_COMBKEY_LONG_PRESS_MS));
	} else {
		if (is_volumup_pwrkey_down) {
			is_volumup_pwrkey_down = false;
			pr_info("volup_pwrkey combination key canceled.\n");
			cancel_delayed_work(&g_check_combkey_long_press_work);
		}
	}

	/* only power key pressed, start to calculate duration */
	if (is_pwrkey_down && !is_volumup_down) {
		pr_info("power key pressed.\n");
		schedule_delayed_work(&g_check_pwrkey_long_press_work, msecs_to_jiffies(CHECK_PWRKEY_LONG_PRESS_MS));
	} else {
		pr_info("power key canceled.\n");
		cancel_delayed_work(&g_check_pwrkey_long_press_work);
	}

	return NOTIFY_DONE;
}

static struct notifier_block combkey_monitor_notifier = {
	.notifier_call = combkey_monitor_notifier_call,
	.priority = 128,
};

static int __init combkey_monitor_init(void)
{
	pr_info("called.\n");
	keyevent_register_notifier(&combkey_monitor_notifier, combkey_monitor_events, combkey_monitor_events_nr);
	INIT_DELAYED_WORK(&g_check_combkey_long_press_work, combkey_long_press_callback);
	INIT_DELAYED_WORK(&g_check_pwrkey_long_press_work, pwrkey_long_press_callback);
	return 0;
}

static void __exit combkey_monitor_exit(void)
{
	pr_info("called.\n");
	cancel_delayed_work_sync(&g_check_combkey_long_press_work);
	cancel_delayed_work_sync(&g_check_pwrkey_long_press_work);
	keyevent_unregister_notifier(&combkey_monitor_notifier, combkey_monitor_events, combkey_monitor_events_nr);
}

module_init(combkey_monitor_init);
module_exit(combkey_monitor_exit);

MODULE_DESCRIPTION("oplus_combkey_monitor");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
