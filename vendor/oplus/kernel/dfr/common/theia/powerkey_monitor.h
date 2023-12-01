// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (C) 2018-2020 Oplus. All rights reserved.
*/
#ifndef __POWERKEY_MONITOR_H_
#define __POWERKEY_MONITOR_H_

#include <linux/module.h>
#include <linux/fb.h>
#include <linux/notifier.h>
#include <linux/timer.h>
#include <linux/kthread.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#ifdef CONFIG_ARM
#include <linux/sched.h>
#else
#include <linux/wait.h>
#endif
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/sched/debug.h>
#include <linux/nmi.h>
#if IS_MODULE(CONFIG_OPLUS_FEATURE_THEIA)
#include <linux/sysrq.h>
#endif
#include <linux/sched/signal.h>
#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
#include <linux/soc/qcom/panel_event_notifier.h>
#include <drm/drm_panel.h>
#endif
#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
#include <../drivers/gpu/drm/mediatek/mediatek_v2/mtk_disp_notify.h>
#endif
#include <soc/oplus/dfr/theia_send_event.h>

/* Only CONFIG_DRM_PANEL_NOTIFY or CONFIG_QCOM_PANEL_EVENT_NOTIFIER seperates BLANK event and UNBLANK event */
/* CONFIG_DRM_PANEL_NOTIFY for msm-5.10
 * CONFIG_QCOM_PANEL_EVENT_NOTIFIER is for msm-5.15
 * CONFIG_DRM_MSM for msm-5.4 or early
 * the other for mtk before gki2.0
 */
#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
#define THEIA_PANEL_BLANK_EVENT              DRM_PANEL_EVENT_BLANK
#define THEIA_PANEL_EARLY_BLANK_EVENT        DRM_PANEL_EVENT_BLANK_LP
#define THEIA_PANEL_UNBLANK_EVENT            DRM_PANEL_EVENT_UNBLANK
#define THEIA_PANEL_UNBLANK_VALUE            DRM_PANEL_EVENT_UNBLANK
#define THEIA_PANEL_BLANK_VALUE              DRM_PANEL_EVENT_BLANK
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
#define THEIA_PANEL_BLANK_EVENT              MTK_DISP_EARLY_EVENT_BLANK
#define THEIA_PANEL_EARLY_BLANK_EVENT        MTK_DISP_EVENT_BLANK
#define THEIA_PANEL_UNBLANK_VALUE            MTK_DISP_BLANK_UNBLANK  /* display power on */
#define THEIA_PANEL_BLANK_VALUE              MTK_DISP_BLANK_POWERDOWN
#endif

#define PWKKEY_DCS_TAG                      "CriticalLog"
#define PWKKEY_DCS_EVENTID                  "Theia"
#define PWKKEY_BLACK_SCREEN_DCS_LOGTYPE     "black_screen_monitor"
#define PWKKEY_BRIGHT_SCREEN_DCS_LOGTYPE    "bright_screen_monitor"

#define BRIGHT_STATUS_INIT                1
#define BRIGHT_STATUS_INIT_FAIL           2
#define BRIGHT_STATUS_INIT_SUCCEES        3
#define BRIGHT_STATUS_CHECK_ENABLE        4
#define BRIGHT_STATUS_CHECK_DISABLE       5
#define BRIGHT_STATUS_CHECK_DEBUG         6

struct pwrkey_monitor_data {
	int is_panic;
	int status;
	int blank;
	int get_log;
	unsigned int timeout_ms;
	unsigned int error_count;
#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	struct notifier_block fb_notif;
#endif
	struct timer_list timer;
	struct work_struct error_happen_work;
#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	struct drm_panel *active_panel;
	void *cookie;
#endif
	char error_id[64]; /*format: systemserver_pid:time_sec:time_usec*/
};

#define BLACK_STATUS_INIT                 1
#define BLACK_STATUS_INIT_FAIL            2
#define BLACK_STATUS_INIT_SUCCEES         3
#define BLACK_STATUS_CHECK_ENABLE         4
#define BLACK_STATUS_CHECK_DISABLE        5
#define BLACK_STATUS_CHECK_DEBUG          6

extern struct pwrkey_monitor_data g_bright_data;
extern struct pwrkey_monitor_data g_black_data;
extern struct pwrkey_monitor_data g_recovery_data;

void bright_screen_check_init(void);
void bright_screen_exit(void);
void black_screen_check_init(void);
void black_screen_exit(void);
void theia_pwk_stage_start(char *reason);
void theia_pwk_stage_end(char *reason);
void send_black_screen_dcs_msg(void);
ssize_t get_last_pwkey_stage(char *buf);
ssize_t get_pwkey_stages(char *buf);
void record_stage(const char *buf);
int get_systemserver_pid(void);
void doPanic(void);
void recovery_timer_func(struct timer_list *t);
void theia_send_event_init(void);
void theia_send_event_exit(void);
#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
int br_register_panel_event_notify(void);
#endif
bool is_system_boot_completed(void);
#endif /* __POWERKEY_MONITOR_H_ */
