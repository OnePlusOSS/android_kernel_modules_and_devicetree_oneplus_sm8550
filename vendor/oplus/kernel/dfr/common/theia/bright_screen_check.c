// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#include "powerkey_monitor.h"
#include "theia_kevent_kernel.h"

#define BRIGHT_MAX_WRITE_NUMBER            50
#define BRIGHT_SLOW_TIMEOUT_MS            5000
#define PROC_BRIGHT_SWITCH "brightSwitch"

#define BRIGHT_DEBUG_PRINTK(a, arg...)\
	do {\
		printk("[bright_screen_check]: " a, ##arg);\
	} while (0)

struct pwrkey_monitor_data g_bright_data = {
	.is_panic = 0,
	.status = BRIGHT_STATUS_INIT,
	.blank = THEIA_PANEL_UNBLANK_VALUE,
	.timeout_ms = BRIGHT_SLOW_TIMEOUT_MS,
	.get_log = 1,
	.error_count = 0,
#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	.active_panel = NULL,
	.cookie = NULL,
#endif
};

/* if last stage in this array, skip */
static char bright_last_skip_block_stages[][64] = {
	{ "POWERKEY_interceptKeyBeforeQueueing" }, /* framework policy may not goto sleep when bright check, skip */
};

/* if contain stage in this array, skip */
static char bright_skip_stages[][64] = {
	{ "POWER_wakeUpInternal" }, /* quick press powerkey, power decide wakeup when bright check, skip */
	{ "POWERKEY_wakeUpFromPowerKey" }, /* quick press powerkey, power decide wakeup when bright check, skip */
	{ "CANCELED_" }, /* if CANCELED_ event write in bright check stage, skip */
};

static int br_start_check_systemid = -1;

int bright_screen_timer_restart(void)
{
	BRIGHT_DEBUG_PRINTK("%s enter: blank = %d, status = %d\n", __func__, g_bright_data.blank, g_bright_data.status);

	if (g_bright_data.status != BRIGHT_STATUS_CHECK_ENABLE && g_bright_data.status != BRIGHT_STATUS_CHECK_DEBUG) {
		BRIGHT_DEBUG_PRINTK("%s unsupported status, return, status = %d\n", __func__, g_bright_data.status);
		return g_bright_data.status;
	}

	/* Remove for MTK functioning */
	if (!is_system_boot_completed()) {
		BRIGHT_DEBUG_PRINTK("boot not complete, %s just return\n", __func__);
		/* return -1; */
	}

	if (g_bright_data.blank == THEIA_PANEL_UNBLANK_VALUE) {
		br_start_check_systemid = get_systemserver_pid();
		mod_timer(&g_bright_data.timer, jiffies + msecs_to_jiffies(g_bright_data.timeout_ms));
		BRIGHT_DEBUG_PRINTK("%s: BR check start, timeout = %u\n", __func__, g_bright_data.timeout_ms);
		theia_pwk_stage_start("POWERKEY_START_BR");
		return 0;
	}
	return g_bright_data.blank;
}
EXPORT_SYMBOL_GPL(bright_screen_timer_restart);

/*
logmap format:
logmap{key1:value1;key2:value2;key3:value3 ...}
*/
static void get_brightscreen_check_dcs_logmap(char *logmap)
{
	char stages[512] = {0};
	int stages_len;

	stages_len = get_pwkey_stages(stages);
	snprintf(logmap, 512, "logmap{logType:%s;error_id:%s;error_count:%u;systemserver_pid:%d;stages:%s}",
		PWKKEY_BRIGHT_SCREEN_DCS_LOGTYPE, g_bright_data.error_id, g_bright_data.error_count,
		get_systemserver_pid(), stages);
}

static void send_bright_screen_dcs_msg(void)
{
	char logmap[512] = {0};
	get_brightscreen_check_dcs_logmap(logmap);
	theia_send_event(THEIA_EVENT_BRIGHT_SCREEN_HANG, THEIA_LOGINFO_KERNEL_LOG | THEIA_LOGINFO_ANDROID_LOG,
		current->pid, logmap);
}

static void dump_freeze_log(void)
{
	send_bright_screen_dcs_msg();
}

static bool is_bright_last_stage_skip()
{
	int i = 0, nLen;
	char stage[64] = {0};;
	get_last_pwkey_stage(stage);

	nLen = ARRAY_SIZE(bright_last_skip_block_stages);

	for (i = 0; i < nLen; i++) {
		if (!strcmp(stage, bright_last_skip_block_stages[i])) {
			BRIGHT_DEBUG_PRINTK("is_bright_last_stage_skip return true, stage:%s", stage);
			return true;
		}
	}

	return false;
}

static bool is_bright_contain_skip_stage()
{
	char stages[512] = {0};
	int i = 0, nArrayLen;
	get_pwkey_stages(stages);

	nArrayLen = ARRAY_SIZE(bright_skip_stages);
	for (i = 0; i < nArrayLen; i++) {
		if (strstr(stages, bright_skip_stages[i]) != NULL) {
			BRIGHT_DEBUG_PRINTK("is_bright_contain_skip_stage return true, stages:%s", stages);
			return true;
		}
	}

	return false;
}

static bool is_need_skip()
{
	if (is_bright_last_stage_skip())
		return true;

	if (is_bright_contain_skip_stage())
		return true;

	return false;
}

static void delete_timer(char *reason, bool cancel)
{
	del_timer(&g_bright_data.timer);

	if (cancel && g_bright_data.error_count != 0) {
		g_bright_data.error_count = 0;
		sprintf(g_bright_data.error_id, "%s", "null");
	}

	theia_pwk_stage_end(reason);
}

static void bright_error_happen_work(struct work_struct *work)
{
	struct pwrkey_monitor_data *bri_data = container_of(work, struct pwrkey_monitor_data, error_happen_work);
	struct timespec64 ts;

	/* for bright screen check, check if need skip, we direct return */
	if (is_need_skip())
		return;

	if (bri_data->error_count == 0) {
		ktime_get_real_ts64(&ts);
		sprintf(g_bright_data.error_id, "%d.%lld.%ld", get_systemserver_pid(), ts.tv_sec, ts.tv_nsec);
	}

	if (bri_data->error_count < BRIGHT_MAX_WRITE_NUMBER) {
		bri_data->error_count++;
		dump_freeze_log();
	}
	BRIGHT_DEBUG_PRINTK("bright_error_happen_work error_id = %s, error_count = %d\n",
		bri_data->error_id, bri_data->error_count);

	delete_timer("BR_SCREEN_ERROR_HAPPEN", false);

	if(bri_data->is_panic)
		doPanic();
}

static void bright_timer_func(struct timer_list *t)
{
	struct pwrkey_monitor_data *p = from_timer(p, t, timer);

	BRIGHT_DEBUG_PRINTK("bright_timer_func is called\n");

#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	if (g_bright_data.active_panel == NULL || g_bright_data.cookie == NULL) {
		BRIGHT_DEBUG_PRINTK("br check register panel not ready\n");
		return;
	}
#endif

	if (br_start_check_systemid == get_systemserver_pid())
		schedule_work(&p->error_happen_work);
	else
		BRIGHT_DEBUG_PRINTK("bright_timer_func, not valid for check, skip\n");
}

#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
static void bright_fb_notifier_callback(enum panel_event_notifier_tag tag,
	struct panel_event_notification *notification, void *client_data)
{
	if (!notification) {
		BRIGHT_DEBUG_PRINTK("bright_fb_notifier_callback, invalid notify\n");
		return;
	}

	switch (notification->notif_type) {
	case DRM_PANEL_EVENT_BLANK:
		g_bright_data.blank = THEIA_PANEL_BLANK_VALUE;
		if (g_bright_data.status != BLACK_STATUS_CHECK_DEBUG) {
			delete_timer("FINISH_FB", true);
			BRIGHT_DEBUG_PRINTK("bright_fb_notifier_callback: del timer, status:%d, blank:%d\n",
				g_bright_data.status, g_bright_data.blank);
		} else {
			BRIGHT_DEBUG_PRINTK("bright_fb_notifier_callback debug: status:%d, blank:%d\n",
				g_bright_data.status, g_bright_data.blank);
		}
		break;
	case DRM_PANEL_EVENT_UNBLANK:
		g_bright_data.blank = THEIA_PANEL_UNBLANK_VALUE;
		break;
	default:
		break;
	}
}
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
static int bright_fb_notifier_callback(struct notifier_block *self,
	unsigned long event, void *data)
{
	switch (event) {
	case THEIA_PANEL_BLANK_EVENT:
		g_bright_data.blank = *(int *)data;
		if (g_bright_data.status != BLACK_STATUS_CHECK_DEBUG) {
			if (g_bright_data.blank == THEIA_PANEL_BLANK_VALUE) {
				delete_timer("FINISH_FB", true);
				BRIGHT_DEBUG_PRINTK("bright_fb_notifier_callback: del timer, status:%d blank:%d\n",
					g_bright_data.status, g_bright_data.blank);
			}
		} else {
			BRIGHT_DEBUG_PRINTK("bright_fb_notifier_callback debug: status:%d blank:%d\n",
				g_bright_data.status, g_bright_data.blank);
		}
		break;
	default:
		break;
	}

	return 0;
}
#endif

static int bright_screen_cancel_proc_show(struct seq_file *seq_file, void *data)
{
	seq_printf(seq_file, "%s called\n", __func__);
	return 0;
}

static int bright_screen_cancel_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, bright_screen_cancel_proc_show, NULL);
}

static ssize_t bright_screen_cancel_proc_write(struct file *file, const char __user *buf,
	size_t count, loff_t *off)
{
	char buffer[40] = {0};
	char cancel_str[64] = {0};

	if(g_bright_data.status == BRIGHT_STATUS_INIT || g_bright_data.status == BRIGHT_STATUS_INIT_FAIL) {
		BRIGHT_DEBUG_PRINTK("%s init not finish: status = %d\n", __func__, g_bright_data.status);
		return count;
	}

	if (count >= 40)
		count = 39;

	if (copy_from_user(buffer, buf, count)) {
		BRIGHT_DEBUG_PRINTK("%s: read proc input error.\n", __func__);
		return count;
	}

	snprintf(cancel_str, sizeof(cancel_str), "CANCELED_BR_%s", buffer);
	delete_timer(cancel_str, true);

	return count;
}
static ssize_t bright_screen_cancel_proc_read(struct file *file, char __user *buf,
	size_t count, loff_t *off)
{
	return 0;
}

static const struct proc_ops bright_screen_cancel_proc_fops = {
	.proc_open = bright_screen_cancel_proc_open,
	.proc_read = bright_screen_cancel_proc_read,
	.proc_write = bright_screen_cancel_proc_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
int br_register_panel_event_notify(void)
{
	void *data = NULL;
	void *cookie = NULL;

	cookie = panel_event_notifier_register(PANEL_EVENT_NOTIFICATION_PRIMARY,
				PANEL_EVENT_NOTIFIER_CLIENT_PRIMARY_THEIA_BRIGHT,
				g_bright_data.active_panel, bright_fb_notifier_callback, data);

	if (!cookie) {
		BRIGHT_DEBUG_PRINTK("br_register_panel_event_notify failed\n");
		return -1;
	}
	g_bright_data.cookie = cookie;
	return 0;
}
#endif

void bright_screen_check_init(void)
{
	BRIGHT_DEBUG_PRINTK("%s called\n", __func__);
	g_bright_data.status = BRIGHT_STATUS_INIT;

#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	g_bright_data.active_panel = NULL;
	g_bright_data.cookie = NULL;
	/* register notify done in black_screen_check_init for reduce get_active_panel redundant code */
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	g_bright_data.fb_notif.notifier_call = bright_fb_notifier_callback;
	if (mtk_disp_notifier_register("oplus_theia", &g_bright_data.fb_notif)) {
		g_bright_data.status = BRIGHT_STATUS_INIT_FAIL;
		BRIGHT_DEBUG_PRINTK("bright_screen_check_init, register fb notifier fail\n");
		return;
	}
#endif

	sprintf(g_bright_data.error_id, "%s", "null");
	/* the node for cancel bright screen check */
	if (proc_create(PROC_BRIGHT_SWITCH, S_IRWXUGO, NULL, &bright_screen_cancel_proc_fops) == NULL)
		BRIGHT_DEBUG_PRINTK("brightSwitch proc node create failed\n");

	INIT_WORK(&g_bright_data.error_happen_work, bright_error_happen_work);
	timer_setup((&g_bright_data.timer), (bright_timer_func), TIMER_DEFERRABLE);
	g_bright_data.status = BRIGHT_STATUS_CHECK_ENABLE;
}

void bright_screen_exit(void)
{
	BRIGHT_DEBUG_PRINTK("%s called\n", __func__);
	remove_proc_entry(PROC_BRIGHT_SWITCH, NULL);
	delete_timer("FINISH_DRIVER_EXIT", true);
	cancel_work_sync(&g_bright_data.error_happen_work);
#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	if (g_bright_data.active_panel && g_bright_data.cookie)
		panel_event_notifier_unregister(g_bright_data.cookie);
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	mtk_disp_notifier_unregister(&g_bright_data.fb_notif);
#endif
}
