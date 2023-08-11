// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#include "powerkey_monitor.h"
#include "theia_kevent_kernel.h"

#define BLACK_MAX_WRITE_NUMBER            50
#define BLACK_SLOW_STATUS_TIMEOUT_MS    5000
#define PROC_BLACK_SWITCH "blackSwitch"

#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
static struct delayed_work g_check_dt_work;
static int g_check_dt_retry_count;
#define CHECK_DT_DELAY_MS 20000
#endif

#define BLACK_DEBUG_PRINTK(a, arg...)\
	do {\
		printk("[black_screen_check]: " a, ##arg);\
	} while (0)

struct pwrkey_monitor_data g_black_data = {
	.is_panic = 0,
	.status = BLACK_STATUS_INIT,
	.blank = THEIA_PANEL_BLANK_VALUE,
	.timeout_ms = BLACK_SLOW_STATUS_TIMEOUT_MS,
	.get_log = 1,
	.error_count = 0,
#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	.active_panel = NULL,
	.cookie = NULL,
#endif
};

static int bl_start_check_systemid = -1;

bool is_dual_screen(void)
{
	struct device_node *node = NULL;
	bool is_dual_screen_dev;
	node = of_find_node_by_name(NULL, "oplus-theia");
	if (!node) {
		BLACK_DEBUG_PRINTK("%s, No oplus-theia node in dtsi\n", __func__);
		return false;
	} else {
		BLACK_DEBUG_PRINTK("%s, oplus-theia node in dtsi\n", __func__);
	}
	is_dual_screen_dev = of_property_read_bool(node, "is-dual-screen-pwk-monitor-switch");
	return is_dual_screen_dev;
}

int black_screen_timer_restart(void)
{
	BLACK_DEBUG_PRINTK("%s enter: blank = %d, status = %d\n", __func__, g_black_data.blank, g_black_data.status);

	if (g_black_data.status != BLACK_STATUS_CHECK_ENABLE && g_black_data.status != BLACK_STATUS_CHECK_DEBUG) {
		BLACK_DEBUG_PRINTK("%s unsupported status, return, status = %d\n", __func__, g_black_data.status);
		return g_black_data.status;
	}

	/* Remove for MTK functioning */
	if (!is_system_boot_completed()) {
		BLACK_DEBUG_PRINTK("boot not complete, %s just return\n", __func__);
		/* return -1; */
	}

	if (is_dual_screen()) {
		BLACK_DEBUG_PRINTK("dual screen not adapted, %s just return\n", __func__);
		return 0;
	} else {
		BLACK_DEBUG_PRINTK("Not dual screen, %s running\n", __func__);
	}

	if (g_black_data.blank == THEIA_PANEL_BLANK_VALUE) {
		bl_start_check_systemid = get_systemserver_pid();
		mod_timer(&g_black_data.timer, jiffies + msecs_to_jiffies(g_black_data.timeout_ms));
		BLACK_DEBUG_PRINTK("%s: BL check start, timeout = %u\n", __func__, g_black_data.timeout_ms);
		theia_pwk_stage_start("POWERKEY_START_BL");
		return 0;
	}
	return g_black_data.blank;
}
EXPORT_SYMBOL_GPL(black_screen_timer_restart);

/*
logmap format:
logmap{key1:value1;key2:value2;key3:value3 ...}
*/
static void get_blackscreen_check_dcs_logmap(char *logmap)
{
	char stages[512] = {0};
	int stages_len;

	stages_len = get_pwkey_stages(stages);
	snprintf(logmap, 512, "logmap{logType:%s;error_id:%s;error_count:%u;systemserver_pid:%d;stages:%s}",
		PWKKEY_BLACK_SCREEN_DCS_LOGTYPE, g_black_data.error_id, g_black_data.error_count,
		get_systemserver_pid(), stages);
}

void send_black_screen_dcs_msg(void)
{
	char logmap[512] = {0};
	get_blackscreen_check_dcs_logmap(logmap);
	theia_send_event(THEIA_EVENT_BLACK_SCREEN_HANG, THEIA_LOGINFO_KERNEL_LOG | THEIA_LOGINFO_ANDROID_LOG,
		current->pid, logmap);
}

static void delete_timer(char *reason, bool cancel)
{
	del_timer(&g_black_data.timer);

	if (cancel && g_black_data.error_count != 0) {
		g_black_data.error_count = 0;
		sprintf(g_black_data.error_id, "%s", "null");
	}

	theia_pwk_stage_end(reason);
}

static int black_screen_cancel_proc_show(struct seq_file *seq_file, void *data)
{
	seq_printf(seq_file, "%s called\n", __func__);
	return 0;
}

static int black_screen_cancel_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, black_screen_cancel_proc_show, NULL);
}

static ssize_t black_screen_cancel_proc_write(struct file *file, const char __user *buf,
	size_t count, loff_t *off)
{
	char buffer[40] = {0};
	char cancel_str[64] = {0};

	if (g_black_data.status == BLACK_STATUS_INIT || g_black_data.status == BLACK_STATUS_INIT_FAIL) {
		BLACK_DEBUG_PRINTK("%s init not finish: status = %d\n", __func__, g_black_data.status);
		return count;
	}

	if (count >= 40)
		count = 39;

	if (copy_from_user(buffer, buf, count)) {
		BLACK_DEBUG_PRINTK("%s: read proc input error.\n", __func__);
		return count;
	}

	snprintf(cancel_str, sizeof(cancel_str), "CANCELED_BL_%s", buffer);
	delete_timer(cancel_str, true);

	return count;
}

static ssize_t black_screen_cancel_proc_read(struct file *file, char __user *buf,
	size_t count, loff_t *off)
{
	return 0;
}

static const struct proc_ops black_screen_cancel_proc_fops = {
	.proc_open = black_screen_cancel_proc_open,
	.proc_read = black_screen_cancel_proc_read,
	.proc_write = black_screen_cancel_proc_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static void dump_freeze_log(void)
{
	send_black_screen_dcs_msg();
}

static void black_error_happen_work(struct work_struct *work)
{
	struct pwrkey_monitor_data *bla_data = container_of(work, struct pwrkey_monitor_data, error_happen_work);
	struct timespec64 ts;

	if (bla_data->error_count == 0) {
		ktime_get_real_ts64(&ts);
		sprintf(g_black_data.error_id, "%d.%lld.%ld", get_systemserver_pid(), ts.tv_sec, ts.tv_nsec);
	}

	if (bla_data->error_count < BLACK_MAX_WRITE_NUMBER) {
		bla_data->error_count++;
		dump_freeze_log();
	}
	BLACK_DEBUG_PRINTK("black_error_happen_work error_id = %s, error_count = %d\n",
		bla_data->error_id, bla_data->error_count);

	delete_timer("BL_SCREEN_ERROR_HAPPEN", false);

	if(bla_data->is_panic)
		doPanic();
}

static void black_timer_func(struct timer_list *t)
{
	struct pwrkey_monitor_data *p = from_timer(p, t, timer);

	BLACK_DEBUG_PRINTK("black_timer_func is called\n");

#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	if (g_black_data.active_panel == NULL || g_black_data.cookie == NULL) {
		BLACK_DEBUG_PRINTK("bl check register panel not ready\n");
		return;
	}
#endif

	if (bl_start_check_systemid == get_systemserver_pid())
		schedule_work(&p->error_happen_work);
	else
		BLACK_DEBUG_PRINTK("black_timer_func, not valid for check, skip\n");
}

#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
static void black_fb_notifier_callback(enum panel_event_notifier_tag tag,
	struct panel_event_notification *notification, void *client_data)
{
	if (!notification) {
		BLACK_DEBUG_PRINTK("black_fb_notifier_callback, invalid notify\n");
		return;
	}

	switch (notification->notif_type) {
	case THEIA_PANEL_BLANK_EVENT:
		g_black_data.blank = THEIA_PANEL_BLANK_VALUE;
		break;
	case THEIA_PANEL_UNBLANK_EVENT:
		g_black_data.blank = THEIA_PANEL_UNBLANK_VALUE;
		if (g_black_data.status != BLACK_STATUS_CHECK_DEBUG) {
			delete_timer("FINISH_FB", true);
			BLACK_DEBUG_PRINTK("black_fb_notifier_callback: del timer, status:%d, blank:%d\n",
				g_black_data.status, g_black_data.blank);
		} else {
			BLACK_DEBUG_PRINTK("black_fb_notifier_callback debug: status:%d, blank:%d\n",
				g_black_data.status, g_black_data.blank);
		}
		break;
	default:
		break;
	}
}
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
static int black_fb_notifier_callback(struct notifier_block *self,
	unsigned long event, void *data)
{
	switch (event) {
	case THEIA_PANEL_BLANK_EVENT:
		g_black_data.blank = *(int *)data;
		if (g_black_data.status != BLACK_STATUS_CHECK_DEBUG) {
			if (g_black_data.blank == THEIA_PANEL_UNBLANK_VALUE) {
				delete_timer("FINISH_FB", true);
				BLACK_DEBUG_PRINTK("black_fb_notifier_callback: del timer, status:%d, blank:%d\n",
					g_black_data.status, g_black_data.blank);
			}
		} else {
			BLACK_DEBUG_PRINTK("black_fb_notifier_callback debug: status:%d, blank:%d\n",
				g_black_data.status, g_black_data.blank);
		}
		break;
	default:
		break;
	}

	return 0;
}
#endif

#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
static int bl_register_panel_event_notify(void)
{
	void *data = NULL;
	void *cookie = NULL;

	cookie = panel_event_notifier_register(PANEL_EVENT_NOTIFICATION_PRIMARY,
				PANEL_EVENT_NOTIFIER_CLIENT_PRIMARY_THEIA_BLACK,
				g_black_data.active_panel, black_fb_notifier_callback, data);

	if (!cookie) {
		BLACK_DEBUG_PRINTK("bl_register_panel_event_notify failed\n");
		return -1;
	}
	g_black_data.cookie = cookie;
	return 0;
}

static struct drm_panel *theia_check_panel_dt(void)
{
	int i;
	int count;
	struct device_node *node = NULL;
	struct drm_panel *panel = NULL;
	struct device_node *np = NULL;

	np = of_find_node_by_name(NULL, "oplus,dsi-display-dev");
	if (!np) {
		BLACK_DEBUG_PRINTK("Device tree info missing.\n");
		goto fail;
	} else {
		BLACK_DEBUG_PRINTK("Device tree info found.\n");
	}

	/* for furture mliti-panel extend, need to add code for other panel parse and register */
	count = of_count_phandle_with_args(np, "oplus,dsi-panel-primary", NULL);
	BLACK_DEBUG_PRINTK("Device tree oplus,dsi-panel-primary count = %d.\n", count);
	if (count <= 0)
		goto fail;

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "oplus,dsi-panel-primary", i);
		panel = of_drm_find_panel(node);
		of_node_put(node);
		if (!IS_ERR(panel)) {
			BLACK_DEBUG_PRINTK("Found active panel.\n");
			break;
		}
	}

	if (IS_ERR(panel))
		panel = NULL;

fail:
	return panel;
}

static struct drm_panel *theia_get_active_panel(void)
{
	return theia_check_panel_dt();
}

static int register_panel_event(void)
{
	int ret = -1;
	struct drm_panel *panel = NULL;

	panel = theia_get_active_panel();
	if (panel) {
		g_black_data.active_panel = panel;
		g_bright_data.active_panel = panel;
		ret = bl_register_panel_event_notify();
		ret |= br_register_panel_event_notify();
	} else {
		BLACK_DEBUG_PRINTK("theia_check_panel_dt failed, get no active panel\n");
	}
	return ret;
}

static void check_dt_work_func(struct work_struct *work)
{
	if (register_panel_event()) {
		BLACK_DEBUG_PRINTK("register_panel_event failed, retry, retry_count = %d\n", g_check_dt_retry_count);
		if (g_check_dt_retry_count) {
			schedule_delayed_work(&g_check_dt_work, msecs_to_jiffies(CHECK_DT_DELAY_MS));
			g_check_dt_retry_count--;
		} else {
			g_black_data.status = BLACK_STATUS_INIT_FAIL;
			g_bright_data.status = BRIGHT_STATUS_INIT_FAIL;
			BLACK_DEBUG_PRINTK("register_panel_event failed, the pwrkey monitor function disabled\n");
		}
	}
}
#endif

void black_screen_check_init(void)
{
	BLACK_DEBUG_PRINTK("%s called\n", __func__);

	g_black_data.status = BLACK_STATUS_INIT;

#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	g_black_data.active_panel = NULL;
	g_black_data.cookie = NULL;
	g_check_dt_retry_count = 2;
	INIT_DELAYED_WORK(&g_check_dt_work, check_dt_work_func);
	schedule_delayed_work(&g_check_dt_work, msecs_to_jiffies(CHECK_DT_DELAY_MS));
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	g_black_data.fb_notif.notifier_call = black_fb_notifier_callback;
	if (mtk_disp_notifier_register("oplus_theia", &g_black_data.fb_notif)) {
		g_black_data.status = BLACK_STATUS_INIT_FAIL;
		BLACK_DEBUG_PRINTK("black_screen_check_init, register fb notifier fail\n");
		return;
	}
#endif
	sprintf(g_black_data.error_id, "%s", "null");

	/* the node for cancel black screen check */
	if (proc_create(PROC_BLACK_SWITCH, S_IRWXUGO, NULL, &black_screen_cancel_proc_fops) == NULL)
		BLACK_DEBUG_PRINTK("brightSwitch proc node create failed\n");

	INIT_WORK(&g_black_data.error_happen_work, black_error_happen_work);
	timer_setup((&g_black_data.timer), (black_timer_func), TIMER_DEFERRABLE);
	g_black_data.status = BLACK_STATUS_CHECK_ENABLE;
}

void black_screen_exit(void)
{
	BLACK_DEBUG_PRINTK("%s called\n", __func__);

	remove_proc_entry(PROC_BLACK_SWITCH, NULL);
	delete_timer("FINISH_DRIVER_EXIT", true);
	cancel_work_sync(&g_black_data.error_happen_work);
#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	cancel_delayed_work_sync(&g_check_dt_work);
	if (g_black_data.active_panel && g_black_data.cookie)
		panel_event_notifier_unregister(g_black_data.cookie);
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	mtk_disp_notifier_unregister(&g_black_data.fb_notif);
#endif
}
