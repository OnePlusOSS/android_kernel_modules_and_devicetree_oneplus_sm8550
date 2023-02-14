// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#include "healthinfo.h"
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/ratelimit.h>
#include <linux/ktime.h>
#include <linux/seq_file.h>
#include <linux/version.h>
#include <trace/events/sched.h>
#include <trace/hooks/mm.h>
#include <trace/hooks/sched.h>
#define BUFFER_SIZE_S 256
#define BUFFER_SIZE_M 512
#define BUFFER_SIZE_L 2048

#define MAX_OHMEVENT_PARAM 4
#define OH_MSG_LEN 256
#include <linux/version.h>

/****  Ctrl init  ****/
#define OHM_LIST_MAGIC          0x5a000000
#define OHM_CTRL_MAX            32
#define OHM_INT_MAX             20
#define OHM_CTRL_IOWAIT         BIT(OHM_SCHED_IOWAIT)
#define OHM_CTRL_SCHEDLATENCY   BIT(OHM_SCHED_SCHEDLATENCY)
#define OHM_CTRL_FSYNC          BIT(OHM_SCHED_FSYNC)
#define OHM_CTRL_EMMCIO         BIT(OHM_SCHED_EMMCIO)
#define OHM_CTRL_DSTATE         BIT(OHM_SCHED_DSTATE)
#define OHM_CTRL_SCHEDTOTAL     (OHM_CTRL_EMMCIO | OHM_CTRL_FSYNC | OHM_CTRL_SCHEDLATENCY | OHM_CTRL_IOWAIT | OHM_CTRL_DSTATE)
#define OHM_CTRL_CPU_CUR        BIT(OHM_CPU_LOAD_CUR)
#define OHM_CTRL_MEMMON         BIT(OHM_MEM_MON)
#define OHM_CTRL_IOPANIC_MON    BIT(OHM_IOPANIC_MON)
#define OHM_CTRL_SVM			BIT(OHM_SVM_MON)
#define OHM_CTRL_RLIMIT			BIT(OHM_RLIMIT_MON)
#define OHM_CTRL_IONMON         BIT(OHM_ION_MON)
/******  Para Update  *****/
#define LOW_THRESH_MS_DEFAULT   100
#define HIGH_THRESH_MS_DEFAULT  500
/* low thresh 10~1000ms*/
#define LOW_THRESH_MS_LOW       10
#define LOW_THRESH_MS_HIGH      1000
/* high thresh 100~5000ms*/
#define HIGH_THRESH_MS_LOW      50
#define HIGH_THRESH_MS_HIGH     5000

#define LATENCY_STRING_FORMAT(BUF, MODULE, SCHED_STAT) sprintf(BUF, \
	#MODULE"_ctrl: %s\n"#MODULE"_logon: %s\n"#MODULE"_trig: %s\n" \
	#MODULE"_delta_ms: %llu\n"#MODULE"_low_thresh_ms: %d\n"#MODULE"_high_thresh_ms: %d\n" \
	#MODULE"_max_ms: %llu\n"#MODULE"_high_cnt: %llu\n"#MODULE"_low_cnt: %llu\n" \
	#MODULE"_total_ms: %llu\n"#MODULE"_total_cnt: %llu\n" \
	#MODULE"_fg_max_ms: %llu\n"#MODULE"_fg_high_cnt: %llu\n"#MODULE"_fg_low_cnt: %llu\n" \
	#MODULE"_fg_total_ms: %llu\n"#MODULE"_fg_total_cnt: %llu\n" \
	#MODULE"_ux_max_ms: %llu\n"#MODULE"_ux_high_cnt: %llu\n"#MODULE"_ux_low_cnt: %llu\n" \
	#MODULE"_ux_total_ms: %llu\n"#MODULE"_ux_total_cnt: %llu\n" \
	#MODULE"_top_app_max_ms: %llu\n"#MODULE"_top_app_high_cnt: %llu\n"#MODULE"_top_app_low_cnt: %llu\n" \
	#MODULE"_top_app_total_ms: %llu\n"#MODULE"_top_app_total_cnt: %llu\n" \
	#MODULE"_bg_max_ms: %llu\n"#MODULE"_bg_high_cnt: %llu\n"#MODULE"_bg_low_cnt: %llu\n" \
	#MODULE"_bg_total_ms: %llu\n"#MODULE"_bg_total_cnt: %llu\n" \
	#MODULE"_sys_bg_max_ms: %llu\n"#MODULE"_sys_bg_high_cnt: %llu\n"#MODULE"_sys_bg_low_cnt: %llu\n" \
	#MODULE"_sys_bg_total_ms: %llu\n"#MODULE"_sys_bg_total_cnt: %llu\n", \
	SCHED_STAT->ctrl ? "true":"false", \
	SCHED_STAT->logon ? "true":"false", \
	SCHED_STAT->trig ? "true":"false", \
	SCHED_STAT->delta_ms, \
	SCHED_STAT->low_thresh_ms, \
	SCHED_STAT->high_thresh_ms, \
	SCHED_STAT->all.max_ms, \
	SCHED_STAT->all.high_cnt, \
	SCHED_STAT->all.low_cnt, \
	SCHED_STAT->all.total_ms, \
	SCHED_STAT->all.total_cnt, \
	SCHED_STAT->fg.max_ms, \
	SCHED_STAT->fg.high_cnt, \
	SCHED_STAT->fg.low_cnt, \
        SCHED_STAT->fg.total_ms, \
	SCHED_STAT->fg.total_cnt, \
	SCHED_STAT->ux.max_ms, \
	SCHED_STAT->ux.high_cnt, \
	SCHED_STAT->ux.low_cnt, \
	SCHED_STAT->ux.total_ms, \
	SCHED_STAT->ux.total_cnt, \
	SCHED_STAT->top.max_ms, \
	SCHED_STAT->top.high_cnt, \
	SCHED_STAT->top.low_cnt, \
	SCHED_STAT->top.total_ms, \
	SCHED_STAT->top.total_cnt, \
	SCHED_STAT->bg.max_ms, \
	SCHED_STAT->bg.high_cnt, \
	SCHED_STAT->bg.low_cnt, \
	SCHED_STAT->bg.total_ms, \
	SCHED_STAT->bg.total_cnt, \
	SCHED_STAT->sysbg.max_ms, \
	SCHED_STAT->sysbg.high_cnt, \
	SCHED_STAT->sysbg.low_cnt, \
	SCHED_STAT->sysbg.total_ms, \
	SCHED_STAT->sysbg.total_cnt)

#define REGISTER_TRACE_VH(vender_hook, handler) \
		{ \
			ret = register_##vender_hook(handler, NULL); \
			if (ret) { \
				ohm_err("failed to register_"#vender_hook", ret=%d\n", ret); \
				return ret; \
			} \
		}

#define UNREGISTER_TRACE_VH(vender_hook, handler) \
		{ \
			ret = unregister_##vender_hook(handler, NULL); \
			printk("%s handler register success", #handler);\
		}


/*
ohm_ctrl_list    = 0x5a0fffff
ohm_logon_list = 0x5a002005
ohm_trig_list    = 0x5a002000
*/

/*Default init*/
static int ohm_ctrl_list = OHM_LIST_MAGIC | OHM_CTRL_CPU_CUR | OHM_CTRL_MEMMON |  OHM_CTRL_IONMON | OHM_CTRL_SCHEDTOTAL;
static int ohm_logon_list = OHM_LIST_MAGIC;
static int ohm_trig_list = OHM_LIST_MAGIC;

bool ohm_cpu_ctrl = true;
bool ohm_cpu_logon;
bool ohm_cpu_trig;

bool ohm_memmon_ctrl = true;
bool ohm_memmon_logon;
bool ohm_memmon_trig;

bool ohm_iopanic_mon_ctrl = true;
bool ohm_iopanic_mon_logon;
bool ohm_iopanic_mon_trig;

bool ohm_ionmon_ctrl = true;
bool ohm_ionmon_logon;
bool ohm_ionmon_trig;

struct sched_stat_para sched_para[OHM_SCHED_TOTAL];
static char *sched_list[OHM_TYPE_TOTAL] = {
	/* SCHED_STATS 0 -11 */
	"iowait",
	"sched_latency",
	"fsync",
	"emmcio",
	"dstate",
	"sched_default_05",
	"sched_default_06",
	"sched_default_07",
	"sched_default_08",
	"sched_default_09",
	"sched_default_10",
	"sched_default_11",
	/* OTHER_TYPE 12 - */
	"cur_cpu_load",
	"memory_monitor",
	"io_panic",
	"svm_monitor",
	"rlimit_monitor",
	"ionwait_monitor"
};

struct thresh_para {
	int l_ms;
	int h_ms;
};

struct thresh_para ohm_thresh_para[OHM_SCHED_TOTAL] = {
	{ LOW_THRESH_MS_DEFAULT, HIGH_THRESH_MS_DEFAULT},
	{ LOW_THRESH_MS_DEFAULT, HIGH_THRESH_MS_DEFAULT},
	{ LOW_THRESH_MS_DEFAULT, HIGH_THRESH_MS_DEFAULT},
	{ 100, 200},
	{ LOW_THRESH_MS_DEFAULT, HIGH_THRESH_MS_DEFAULT},
};

static struct kobject *ohm_kobj;
static struct work_struct ohm_detect_ws;
static char *ohm_detect_env[MAX_OHMEVENT_PARAM] = { "OHMACTION=uevent", NULL };
static bool ohm_action_ctrl;
static char msg_buf[OH_MSG_LEN] = {0};


#if IS_ENABLED(CONFIG_CGROUP_SCHED)
static inline int get_task_cgroup_id(struct task_struct *task)
{
	struct cgroup_subsys_state *css = task_css(task, cpu_cgrp_id);

	return css ? css->id : -1;
}
#else
inline int get_task_cgroup_id(struct task_struct *task) { return 0; }
#endif

#if  IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_ASSIST)
// todo add ux type
#endif
static int test_task_top_app(struct task_struct *task)
{
	return (SA_CGROUP_TOP_APP == get_task_cgroup_id(task)) ? 1 : 0;
}
static int test_task_fg(struct task_struct *task)
{
	return (SA_CGROUP_FOREGROUND == get_task_cgroup_id(task)) ? 1 : 0;
}
static int test_task_sys_bg(struct task_struct *task)
{
	 return (SA_CGROUP_SYS_BACKGROUND == get_task_cgroup_id(task)) ? 1 : 0;
}
static int test_task_bg(struct task_struct *task)
{
	return (SA_CGROUP_BACKGROUND == get_task_cgroup_id(task)) ? 1 : 0;
}

void ohm_trig_init(void)
{
	int i;

	ohm_memmon_trig = (ohm_trig_list & OHM_CTRL_MEMMON) ? true : false;
	ohm_cpu_trig = (ohm_trig_list & OHM_CTRL_CPU_CUR) ? true : false;
	ohm_iopanic_mon_trig = (ohm_trig_list & OHM_CTRL_IOPANIC_MON) ? true : false;
		ohm_ionmon_trig = (ohm_trig_list & OHM_CTRL_IONMON) ? true : false;
	for (i = 0; i < OHM_SCHED_TOTAL; i++) {
		sched_para[i].trig = (ohm_trig_list & BIT(i)) ? true : false;
	}
	return;
}

void ohm_logon_init(void)
{
	int i;

	ohm_cpu_logon = (ohm_logon_list & OHM_CTRL_CPU_CUR) ? true : false;
	ohm_memmon_logon = (ohm_logon_list & OHM_CTRL_MEMMON) ? true : false;
	ohm_iopanic_mon_logon = (ohm_logon_list & OHM_CTRL_IOPANIC_MON) ? true : false;
		ohm_ionmon_logon = (ohm_logon_list & OHM_CTRL_IONMON) ? true : false;
	for (i = 0; i < OHM_SCHED_TOTAL; i++) {
		sched_para[i].logon = (ohm_logon_list & BIT(i)) ? true : false;
	}
	return;
}

void ohm_ctrl_init(void)
{
	int i;

	ohm_cpu_ctrl = (ohm_ctrl_list & OHM_CTRL_CPU_CUR) ? true : false;
	ohm_memmon_ctrl = (ohm_ctrl_list & OHM_CTRL_MEMMON) ? true : false;
	ohm_iopanic_mon_ctrl = (ohm_ctrl_list & OHM_CTRL_IOPANIC_MON) ? true : false;
		ohm_ionmon_ctrl = (ohm_ctrl_list & OHM_CTRL_IONMON) ? true : false;
	for (i = 0; i < OHM_SCHED_TOTAL; i++) {
		sched_para[i].ctrl = (ohm_ctrl_list & BIT(i)) ? true : false;
	}
	return;
}



void ohm_action_trig(int type)
{
	if (!ohm_action_ctrl) {
		ohm_err_deferred("ctrl off\n");
		return;
	}
	ohm_debug_deferred("%s trig action\n", sched_list[type]);
	if (OHM_MEM_MON == type || OHM_SCHED_FSYNC == type) {
		if (!ohm_kobj) {
			ohm_err_deferred("kobj NULL\n");
			return;
                }
		sprintf(ohm_detect_env[1], "OHMTYPE=%s", sched_list[type]);
		ohm_detect_env[MAX_OHMEVENT_PARAM - 2] = NULL;
		ohm_detect_env[MAX_OHMEVENT_PARAM - 1] = NULL;
		schedule_work(&ohm_detect_ws);
	}
}

void ohm_action_trig_with_msg(int type, char *msg)
{
	int len;

	if (!ohm_action_ctrl) {
		ohm_err("ctrl off\n");
		return;
	}

	if (!ohm_kobj) {
		ohm_err("kobj NULL\n");
		return;
	}
	if (OHM_SVM_MON == type || OHM_RLIMIT_MON == type || OHM_MEM_VMA_ALLOC_ERR == type) {
		sprintf(ohm_detect_env[1], "OHMTYPE=%s", sched_list[type]);
		len = snprintf(msg_buf, OH_MSG_LEN-1, "OHMMSG=%s", msg);
		msg_buf[len] = '\0';
		ohm_detect_env[MAX_OHMEVENT_PARAM - 2] = msg_buf;
		ohm_detect_env[MAX_OHMEVENT_PARAM - 1] = NULL;
		schedule_work(&ohm_detect_ws);
	}
}
EXPORT_SYMBOL_GPL(ohm_action_trig_with_msg);

void ohm_detect_work(struct work_struct *work)
{
	ohm_debug("Uevent Para: %s, %s\n", ohm_detect_env[0], ohm_detect_env[1]);
	kobject_uevent_env(ohm_kobj, KOBJ_CHANGE, ohm_detect_env);
	ohm_debug("Uevent Done!\n");
}

static inline void ohm_sched_stat_record_common(struct sched_stat_para *sched_stat,struct sched_stat_common *stat_common, u64 delta_ms)
{
	stat_common->total_ms += delta_ms;
	stat_common->total_cnt++;

	if (delta_ms > stat_common->max_ms) {
		stat_common->max_ms = delta_ms;
	}
	if (delta_ms >= sched_stat->high_thresh_ms) {
		stat_common->high_cnt++;
	} else if (delta_ms >= sched_stat->low_thresh_ms) {
		stat_common->low_cnt++;
	}
}
void ohm_schedstats_record(int sched_type, struct task_struct *task, u64 delta_ms)
{
	struct sched_stat_para *sched_stat = &sched_para[sched_type];
	static DEFINE_RATELIMIT_STATE(ratelimit, 60*HZ, 1);

	if (unlikely(!sched_stat->ctrl)) {
		return;
	}

	sched_stat->delta_ms = delta_ms;
	ohm_sched_stat_record_common(sched_stat, &sched_stat->all, delta_ms);

	if (test_task_fg(task)) {
	ohm_sched_stat_record_common(sched_stat, &sched_stat->fg, delta_ms);
		if (unlikely(delta_ms >= sched_stat->high_thresh_ms)) {
			if (sched_para[sched_type].logon  && __ratelimit(&ratelimit)) {
				ohm_debug_deferred("[%s / %s] high_cnt, delay = %llu ms\n",
				sched_list[sched_type], "fg", delta_ms);
			}
			if (sched_para[sched_type].trig)
				ohm_action_trig(sched_type);
		}
	}
#if  IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_ASSIST)
//To do add ux type
#endif
	if (test_task_top_app(task)) {
		ohm_sched_stat_record_common(sched_stat, &sched_stat->top, delta_ms);
	}
	if (test_task_bg(task)) {
		ohm_sched_stat_record_common(sched_stat, &sched_stat->bg, delta_ms);
	}
	if (test_task_sys_bg(task)) {
		ohm_sched_stat_record_common(sched_stat, &sched_stat->sysbg, delta_ms);
	}
	return;
	}

static inline void _ohm_para_init(struct sched_stat_para *sched_para)
{
	sched_para->delta_ms = 0;
	memset(&sched_para->all, 0, sizeof(struct sched_stat_common));
	memset(&sched_para->ux, 0, sizeof(struct sched_stat_common));
	memset(&sched_para->fg, 0, sizeof(struct sched_stat_common));

	return;
}

void ohm_para_init(void)
{
	int i;

	for (i = 0; i < OHM_SCHED_TOTAL; i++) {
		_ohm_para_init(&sched_para[i]);
		sched_para[i].low_thresh_ms = 100;
		sched_para[i].high_thresh_ms = 500;
	}
	sched_para[OHM_SCHED_EMMCIO].low_thresh_ms = 100;
	sched_para[OHM_SCHED_EMMCIO].high_thresh_ms = 200;
	ohm_ctrl_init();
	ohm_logon_init();
	ohm_trig_init();
	ohm_debug("origin list: ctrl 0x%08x, logon 0x%08x, trig 0x%08x\n", ohm_ctrl_list, ohm_logon_list, ohm_trig_list);
	return;
}

void ohm_para_update(void)
{
	int i;

	for (i = 0; i < OHM_SCHED_TOTAL; i++) {
		if (ohm_thresh_para[i].l_ms < LOW_THRESH_MS_LOW
			|| ohm_thresh_para[i].l_ms > LOW_THRESH_MS_HIGH
			|| ohm_thresh_para[i].h_ms < HIGH_THRESH_MS_LOW
			|| ohm_thresh_para[i].h_ms > HIGH_THRESH_MS_HIGH) {
/********** Legal Check **********/
			ohm_err("Para illegal: sched_type %s, l_ms %d, h_ms %d\n",
				sched_list[i], ohm_thresh_para[i].l_ms, ohm_thresh_para[i].h_ms);
			ohm_thresh_para[i].l_ms = LOW_THRESH_MS_DEFAULT;
			ohm_thresh_para[i].h_ms = HIGH_THRESH_MS_DEFAULT;
			return;
		}
		sched_para[i].low_thresh_ms = ohm_thresh_para[i].l_ms;
		sched_para[i].high_thresh_ms = ohm_thresh_para[i].h_ms;
	}
	ohm_debug("Success update ohm_para!\n");
}

static inline ssize_t sched_data_to_user(char __user *buff, size_t count, loff_t *off, char *format_str, int len)
{
	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}
	if (copy_to_user(buff, format_str, (len < count ? len : count))) {
		return -EFAULT;
	}
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

/******  Cur cpuloading  ******/
static ssize_t cpu_load_read(struct file *filp, char __user *buff, size_t count, loff_t *off)
{
	char page[BUFFER_SIZE_S] = {0};
	int len = 0;
	int load = ohm_get_cur_cpuload(ohm_cpu_ctrl);

	if(load < 0){
		load = 0;
		printk_deferred("ohm get cpu load error");
	}
	len = sprintf( page, "cur_cpuloading: %d\n""cur_cpu_ctrl: %s\n""cur_cpu_logon: %s\n""cur_cpu_trig: %s\n",
	load, (ohm_cpu_ctrl ? "true" : "false"), (ohm_cpu_logon ? "true" : "false"), (ohm_cpu_trig ? "true" : "false"));

	return sched_data_to_user(buff, count, off, page, len);
}

static const struct proc_ops proc_cpu_load_fops = {
	.proc_read = cpu_load_read,
	.proc_lseek = default_llseek,
};

/******  Sched latency stat  *****/
static ssize_t sched_latency_read(struct file *filp, char __user *buff, size_t count, loff_t *off)
{
	int len = 0;

	struct sched_stat_para *sched_stat = &sched_para[OHM_SCHED_SCHEDLATENCY];
	char *page = kzalloc(2048, GFP_KERNEL);

	if (!page)
		return -ENOMEM;

	len = LATENCY_STRING_FORMAT(page, sched_latency, sched_stat);

	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}
	if (copy_to_user(buff, page, (len < count ? len : count))) {
		kfree(page);
		return -EFAULT;
	}
	kfree(page);
	*off += len < count ? len : count;
	return (len < count ? len : count);

}

static const struct proc_ops proc_sched_latency_fops = {
	.proc_read = sched_latency_read,
	.proc_lseek = default_llseek,
};

/****** Sched iowait stat  *****/
static ssize_t iowait_read(struct file *filp, char __user *buff, size_t count, loff_t *off)
{
	int len = 0;
	struct sched_stat_para *sched_stat = &sched_para[OHM_SCHED_IOWAIT];
	char *page = kzalloc(2048, GFP_KERNEL);

	if (!page)
		return -ENOMEM;
	len = LATENCY_STRING_FORMAT(page, iowait, sched_stat);

	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}

	if (copy_to_user(buff, page, (len < count ? len : count))) {
		kfree(page);
	return -EFAULT;
	}
	kfree(page);
	*off += len < count ? len : count;

	return (len < count ? len : count);

}

static const struct proc_ops proc_iowait_fops = {
	.proc_read = iowait_read,
	.proc_lseek = default_llseek,
};

/****** Sched sync wait stat  ******/
static ssize_t fsync_wait_read(struct file *filp, char __user *buff, size_t count, loff_t *off)
{
	int len = 0;

	struct sched_stat_para *sched_stat = &sched_para[OHM_SCHED_FSYNC];
	char *page = kzalloc(2048, GFP_KERNEL);

	if (!page)
		return -ENOMEM;

	len = LATENCY_STRING_FORMAT(page, fsync, sched_stat);

	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}
	if (copy_to_user(buff, page, (len < count ? len : count))) {
		kfree(page);
		return -EFAULT;
	}
	kfree(page);
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

static const struct proc_ops proc_fsync_wait_fops = {
	.proc_read = fsync_wait_read,
	.proc_lseek = default_llseek,
};

/****** dstat statistics  ******/
static ssize_t dstate_read(struct file *filp, char __user *buff, size_t count, loff_t *off)
{
	int len = 0;

	struct sched_stat_para *sched_stat = &sched_para[OHM_SCHED_DSTATE];
	char *page = kzalloc(2048, GFP_KERNEL);
	if (!page)
		return -ENOMEM;

 	len = LATENCY_STRING_FORMAT(page, dstate, sched_stat);

	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}
	if (copy_to_user(buff, page, (len < count ? len : count))) {
		kfree(page);
		return -EFAULT;
	}
	kfree(page);
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

static const struct proc_ops proc_dstate_fops = {
	.proc_read = dstate_read,
	.proc_lseek = default_llseek,

};

/*******alloc wait ************/

static ssize_t alloc_wait_read(struct file *filp, char __user *buff, size_t count, loff_t *off)
{
	char page[1024] = {0};
	int len = 0;

	len = sprintf(page, "total_alloc_wait_h_cnt: %lld\n""total_alloc_wait_l_cnt: %lld\n"
		"ux_alloc_wait_h_cnt: %lld\n""ux_alloc_wait_l_cnt: %lld\n"
		"ux_alloc_wait_max_ms: %lld\n""ux_alloc_wait_max_order: %lld\n"
		"fg_alloc_wait_h_cnt: %lld\n""fg_alloc_wait_l_cnt: %lld\n"
		"total_alloc_wait_max_ms: %lld\n""total_alloc_wait_max_order: %lld\n"
		"fg_alloc_wait_max_ms: %lld\n""fg_alloc_wait_max_order: %lld\n"
		"alloc_wait_ctrl: %s\n""alloc_wait_logon: %s\n""alloc_wait_trig: %s\n",
		allocwait_para.total_alloc_wait.high_cnt, allocwait_para.total_alloc_wait.low_cnt,
		allocwait_para.ux_alloc_wait.high_cnt, allocwait_para.ux_alloc_wait.low_cnt,
		allocwait_para.ux_alloc_wait.max_ms, allocwait_para.ux_alloc_wait_max_order,
		allocwait_para.fg_alloc_wait.high_cnt, allocwait_para.fg_alloc_wait.low_cnt,
		allocwait_para.total_alloc_wait.max_ms, allocwait_para.total_alloc_wait_max_order,
		allocwait_para.fg_alloc_wait.max_ms, allocwait_para.fg_alloc_wait_max_order,
		ohm_memmon_ctrl ? "true":"false", ohm_memmon_logon ? "true":"false", ohm_memmon_trig ? "true":"false");

	return sched_data_to_user(buff, count, off, page, len);
}

static const struct proc_ops proc_alloc_wait_fops = {
	.proc_read = alloc_wait_read,
	.proc_lseek = default_llseek,
};

/******  Proc para   ******/
static ssize_t ohm_para_read(struct file *filp, char __user *buff, size_t count, loff_t *off)
{
	char page[BUFFER_SIZE_S] = {0};
	int len = 0;

	len = sprintf(page, "action: %s\n""ctrl: 0x%08x\n""logon: 0x%08x\n""trig: 0x%08x\n",
		(ohm_action_ctrl ? "true":"false"), ohm_ctrl_list, ohm_logon_list, ohm_trig_list);

	return sched_data_to_user(buff, count, off, page, len);
}

static ssize_t ohm_para_write(struct file *file, const char __user *buff, size_t len, loff_t *ppos)
{
	char write_data[32] = {0};
	char ctrl_list[32] = {0};
	int action_ctrl;

	if (len > 31 || len == 0)
		return -EFAULT;

	if (copy_from_user(&write_data, buff, len)) {
		ohm_err("write error.\n");
		return -EFAULT;
	}
	write_data[len] = '\0';
	if (write_data[len - 1] == '\n') {
		write_data[len - 1] = '\0';
	}
	if (0 == strncmp(write_data, "ohmctrl", 7)) {
		strncpy(ctrl_list, &write_data[7], OHM_INT_MAX);
		ctrl_list[OHM_INT_MAX] = '\0';
		ohm_ctrl_list = (int)simple_strtol(ctrl_list, NULL, 10);
		ohm_ctrl_init();
	} else if (0 == strncmp(write_data, "ohmlogon", 8)) {
		strncpy(ctrl_list, &write_data[8], OHM_INT_MAX);
		ctrl_list[OHM_INT_MAX] = '\0';
		ohm_logon_list = (int)simple_strtol(ctrl_list, NULL, 10);
		ohm_logon_init();
	} else if (0 == strncmp(write_data, "ohmtrig", 7)) {
		strncpy(ctrl_list, &write_data[7], OHM_INT_MAX);
		ctrl_list[OHM_INT_MAX] = '\0';
		ohm_trig_list = (int)simple_strtol(ctrl_list, NULL, 10);
		ohm_trig_init();
	} else if (0 == strncmp(write_data, "ohmparaupdate", 13)) {
		ohm_para_update();
		return len;
	} else if (0 == strncmp(write_data, "ohmacitonctrl", 13)) {
		strncpy(ctrl_list, &write_data[13], OHM_INT_MAX);
		ctrl_list[OHM_INT_MAX] = '\0';
		action_ctrl =  (int)simple_strtol(ctrl_list, NULL, 10);
		ohm_action_ctrl = action_ctrl != 0 ? 1 : 0;
	} else {
		ohm_err("input illegal\n");
		return -EFAULT;
	}
	ohm_debug("write: %s, set: %s, ctrl: 0x%08x, logon: 0x%08x, trig: 0x%08x\n",
		write_data, ctrl_list, ohm_ctrl_list, ohm_logon_list, ohm_trig_list);
	return len;
}

static const struct proc_ops proc_para_fops = {
	.proc_read = ohm_para_read,
	.proc_write = ohm_para_write,
	.proc_lseek = default_llseek,
};

/******  iowait hung show  ******/
unsigned int  iowait_hung_cnt = 0;
unsigned int  iowait_panic_cnt = 0;
static ssize_t iowait_hung_read(struct file *filp, char __user *buff, size_t count, loff_t *off)
{
	char page[BUFFER_SIZE_M] = {0};
	int len = 0;

	len = sprintf(page, "iowait_hung_cnt: %u\n""iowait_panic_cnt: %u\n"
		"ohm_iopanic_mon_ctrl: %s\n""ohm_iopanic_mon_logon: %s\n""ohm_iopanic_mon_trig: %s\n",
		iowait_hung_cnt, iowait_panic_cnt,
		(ohm_iopanic_mon_ctrl ? "true" : "false"), (ohm_iopanic_mon_logon ? "true"
		 : "false"), (ohm_iopanic_mon_trig ? "true" : "false"));

	return sched_data_to_user(buff, count, off, page, len);
}

static const struct proc_ops proc_iowait_hung_fops = {
	.proc_read = iowait_hung_read,
	.proc_lseek = default_llseek,
};

/******  cpu info show  ******/
extern unsigned int cpufreq_quick_get_max(unsigned int cpu);
static ssize_t cpu_info_read(struct file *filp, char __user *buff, size_t count, loff_t *off)
{
	char page[BUFFER_SIZE_L] = {0};
	int len = 0;
	unsigned int cpu;
	unsigned long scale_capacity = 0, last_capacity = 0;

	for_each_possible_cpu(cpu) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	scale_capacity = arch_scale_cpu_capacity(NULL,cpu);
#else
	scale_capacity = arch_scale_cpu_capacity(cpu);
#endif
	if (scale_capacity == last_capacity) {
		continue;
	}
	last_capacity = scale_capacity;
	len += snprintf(page + len, sizeof(page) - len, "%u ", cpufreq_quick_get_max(cpu));
	}

	return sched_data_to_user(buff, count, off, page, len);
}

static const struct proc_ops proc_cpu_info_fops = {
	.proc_read = cpu_info_read,
	.proc_lseek = default_llseek,
};

/* thresh read/write */
static ssize_t ohm_thresh_read_common(struct file *filp, char __user *buff, size_t count, loff_t *off,int type)
{
	char page[BUFFER_SIZE_S] = {0};
	int len = 0;

	len = sprintf(page, " %s_thresh:\n h_ms = %dms,l_ms = %dms\n",
		sched_list[type], ohm_thresh_para[type].h_ms, ohm_thresh_para[type].l_ms);

	return sched_data_to_user(buff, count, off, page, len);
}

static ssize_t ohm_thresh_read_iowait(struct file *filp, char __user *buff, size_t count, loff_t *off)
{
	return ohm_thresh_read_common(filp, buff, count, off, OHM_SCHED_IOWAIT);
}

static ssize_t ohm_thresh_read_sched_latency(struct file *filp, char __user *buff, size_t count, loff_t *off)
{
	return ohm_thresh_read_common(filp, buff, count, off, OHM_SCHED_SCHEDLATENCY);
}

static ssize_t ohm_thresh_read_fsync(struct file *filp, char __user *buff, size_t count, loff_t *off)
{
	return ohm_thresh_read_common(filp, buff, count, off, OHM_SCHED_FSYNC);
}

static ssize_t ohm_thresh_write_common(struct file *file, const char __user *buff, size_t len, loff_t *ppos, int type)
{
	char write_data[32] = {0};
	char thresh_list[32] = {0};
	int thresh = 0;

	if (len > 31 || len == 0)
		return -EFAULT;

	if (copy_from_user(&write_data, buff, len)) {
		ohm_err("write error.\n");
		return -EFAULT;
	}
	write_data[len] = '\0';
	if (write_data[len - 1] == '\n') {
		write_data[len - 1] = '\0';
	}

	if (0 == strncmp(write_data, "highthresh", 10)) {
		strncpy(thresh_list, &write_data[9], OHM_INT_MAX);
		thresh_list[OHM_INT_MAX] = '\0';
		thresh = (int)simple_strtol(thresh_list, NULL, 10);
		if (thresh < 0 || thresh < ohm_thresh_para[type].l_ms)
			goto input_err;
		ohm_thresh_para[type].h_ms = thresh;
	} else if (0 == strncmp(write_data, "lowthresh", 9)) {
		strncpy(thresh_list, &write_data[9], OHM_INT_MAX);
		thresh_list[OHM_INT_MAX] = '\0';
		thresh = (int)simple_strtol(thresh_list, NULL, 10);
		if (thresh < 0 || thresh > ohm_thresh_para[type].h_ms)
			goto input_err;
		ohm_thresh_para[type].l_ms = thresh;
	} else {
		goto input_err;
	}

	ohm_debug("thresh update success!Now %s h_ms= %dms,l_ms= %dms\n",
		sched_list[type], ohm_thresh_para[type].h_ms, ohm_thresh_para[type].l_ms);
	return len;

input_err:
	ohm_err("input illegal\n");
	return -EFAULT;
}

static ssize_t ohm_thresh_write_iowait(struct file *file, const char __user *buff, size_t len, loff_t *ppos)
{
	return ohm_thresh_write_common(file, buff, len, ppos, OHM_SCHED_IOWAIT);
}

static ssize_t ohm_thresh_write_sched_latency(struct file *file, const char __user *buff, size_t len, loff_t *ppos)
{
	return ohm_thresh_write_common(file, buff, len, ppos, OHM_SCHED_SCHEDLATENCY);
}

static ssize_t ohm_thresh_write_fsync(struct file *file, const char __user *buff, size_t len, loff_t *ppos)
{
	return ohm_thresh_write_common(file, buff, len, ppos, OHM_SCHED_FSYNC);
}

static const struct proc_ops proc_iowait_thresh_fops = {
	.proc_read = ohm_thresh_read_iowait,
	.proc_write = ohm_thresh_write_iowait,
	.proc_lseek = default_llseek,
};

static const struct proc_ops proc_sched_latency_thresh_fops = {
	.proc_read = ohm_thresh_read_sched_latency,
	.proc_write = ohm_thresh_write_sched_latency,
	.proc_lseek = default_llseek,
};

static const struct proc_ops proc_fsync_thresh_fops = {
	.proc_read = ohm_thresh_read_fsync,
	.proc_write = ohm_thresh_write_fsync,
	.proc_lseek = default_llseek,
};

/*
//ion totoal
atomic_long_t ion_total_size;
bool ion_cnt_enable = true;

unsigned long ion_total(void)
{
	if (!ion_cnt_enable)
		return 0;
	return (unsigned long)atomic_long_read(&ion_total_size);
}

*/

static void probe_sched_latency_handler(void *data, struct task_struct *tsk, u64 delay)
{     
	ohm_schedstats_record(OHM_SCHED_SCHEDLATENCY, tsk, (delay >> 20));
}

static void probe_sched_stat_iowait_handler(void *data, struct task_struct *tsk, u64 delay)
{     
	ohm_schedstats_record(OHM_SCHED_IOWAIT, tsk, (delay >> 20));
}

static void probe_sched_stat_blocked(void *data, struct task_struct *tsk, u64 delay)
{
	ohm_schedstats_record(OHM_SCHED_DSTATE, tsk, (delay >> 20));
}

/*
static void healthinfo_print_slabinfo_header(void *data, struct seq_file *m){
	printk("wlh slabinfo_header access");
	seq_puts(m, " <reclaim>");
}

static void healthinfo_cache_show(void *data, struct seq_file *m, struct slabinfo *sinfo, struct kmem_cache *s)
{
	printk("wlh cache show success");
	seq_printf(m, " : slabdata %6lu %6lu %6lu %1d",
		sinfo->active_slabs, sinfo->num_slabs, sinfo->shared_avail,
		((s->flags & SLAB_RECLAIM_ACCOUNT) == SLAB_RECLAIM_ACCOUNT) ? 1: 0);
}

static void healthinfo_alloc_pages_slowpath(void *data, gfp_t gfp_mask, unsigned int order, unsigned long alloc_start)
{
	printk("wlh slow path alloc success");
	memory_alloc_monitor(gfp_mask, order, jiffies_to_msecs(jiffies - alloc_start));
}

static void healthinfo_show_val_kb(struct seq_file *m, const char *s, unsigned long num)
{
	printk("wlh show val kb");
	seq_put_decimal_ull_width(m, s, num << (PAGE_SHIFT - 10), 8);
	seq_write(m, " kB\n", 4);
}

static unsigned long healthinfo_global_zone_page_state(enum n_zone_stat_item item)
{

	long x = atomic_long_read(&vm_zone_stat[item]);
	printk("wlh global zone page");
#ifdef CONFIG_SMP
	if (x < 0)
		x = 0;
#endif
	
	return x;
}

static void healthinfo_meminfo_proc_show(void *data, struct seq_file *m)
{
	printk("wlh meminfo proc show");
	healthinfo_show_val_kb(m, "IonTotalCache:   ", healthinfo_global_zone_page_state(N_IONCACHE_PAGES));;	
	healthinfo_show_val_kb(m, "IonTotalUsed:   ", ion_total() >> PAGE_SHIFT);
//	healthinfo_show_val_kb(m, "GPUTotalUsed:   ", gpu_total() >> PAGE_SHIFT);
}

static void healthinfo_show_mem(void *data, unsigned int filter, nodemask_t *nodemask)
{
	printk("wlh %lu pages ion total used\n", ion_total()>> PAGE_SHIFT);
}

*/

static int register_healthinfo_vendor_hooks(void)
{
	int ret = 0;
	REGISTER_TRACE_VH(trace_sched_stat_wait, probe_sched_latency_handler);
	REGISTER_TRACE_VH(trace_sched_stat_blocked, probe_sched_stat_blocked);
	REGISTER_TRACE_VH(trace_sched_stat_iowait, probe_sched_stat_iowait_handler);
	/*
	REGISTER_TRACE_VH(trace_android_vh_print_slabinfo_header, healthinfo_print_slabinfo_header);
	REGISTER_TRACE_VH(trace_android_vh_cache_show, healthinfo_cache_show);
	REGISTER_TRACE_VH(trace_android_vh_alloc_pages_slowpath, healthinfo_alloc_pages_slowpath);
	REGISTER_TRACE_VH(trace_android_vh_meminfo_proc_show, healthinfo_meminfo_proc_show);
	REGISTER_TRACE_VH(trace_android_vh_show_mem, healthinfo_show_mem);
	*/
	return ret;
}

static int unregister_healthinfo_vendor_hooks(void)
{
	int ret =0;
	UNREGISTER_TRACE_VH(trace_sched_stat_wait, probe_sched_latency_handler);
	UNREGISTER_TRACE_VH(trace_sched_stat_blocked, probe_sched_stat_blocked);
	UNREGISTER_TRACE_VH(trace_sched_stat_iowait, probe_sched_stat_iowait_handler);
	/*
	UNREGISTER_TRACE_VH(trace_android_vh_print_slabinfo_header, healthinfo_print_slabinfo_header);
	UNREGISTER_TRACE_VH(trace_android_vh_cache_show, healthinfo_cache_show);
	UNREGISTER_TRACE_VH(trace_android_vh_alloc_pages_slowpath, healthinfo_alloc_pages_slowpath);
	UNREGISTER_TRACE_VH(trace_android_vh_meminfo_proc_show, healthinfo_meminfo_proc_show);
	UNREGISTER_TRACE_VH(trace_android_vh_show_mem, healthinfo_show_mem);
	UNREGISTER_TRACE_VH(trace_android_vh_do_fsync,healthinfo_sched_stat_do_fsync);
	*/
	return ret;
}

//fg_uid
#define FG_RW (S_IWUSR|S_IRUSR|S_IWGRP|S_IRGRP|S_IWOTH|S_IROTH)
#define MAX_ARRAY_LENGTH 256
#define FS_FG_INFO_PATH "fg_info"
#define FS_FG_UIDS "fg_uids"

struct fg_info {
	int fg_num;
	int fg_uids;
};

struct fg_info fginfo = {
	.fg_num = 0,
	.fg_uids = -555,
};

bool is_fg(int uid)
{
	bool ret = false;
	if (uid == fginfo.fg_uids)
		ret = true;
	return ret;
}


inline int current_is_fg(void)
{
	int cur_uid;
	cur_uid = current_uid().val;
	if (is_fg(cur_uid))
		return 1;
	return 0;
}

inline int task_is_fg(struct task_struct *tsk)
{
	int cur_uid;
	cur_uid = task_uid(tsk).val;
	if (is_fg(cur_uid))
		return 1;
	return 0;
}

static int fg_uids_show(struct seq_file *m, void *v)
{
	seq_printf(m, "fg_uids: %d\n", fginfo.fg_uids);
	return 0;
}

static int fg_uids_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, fg_uids_show, inode);
}

#if IS_ENABLED(CONFIG_OPLUS_BINDER_TRANS_CTRL)
extern void oblist_dequeue_topapp_change(uid_t topuid);
#endif
static ssize_t fg_uids_write(struct file *file, const char __user *buf,
                        size_t count, loff_t *ppos)
{
	char buffer[MAX_ARRAY_LENGTH];
	int err = 0;

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user((void *)buffer, buf, count)) {
		err = -EFAULT;
		goto out;
	}

	fginfo.fg_uids = simple_strtol(buffer, NULL, 0);
	fginfo.fg_num = 1;
#if IS_ENABLED(CONFIG_OPLUS_BINDER_TRANS_CTRL)
	if (fginfo.fg_uids > 0)
		oblist_dequeue_topapp_change(fginfo.fg_uids);
#endif
out:
	return err < 0 ? err : count;
}

static const struct proc_ops proc_fg_uids_operations = {
	.proc_open       = fg_uids_open,
	.proc_read       = seq_read,
	.proc_write      = fg_uids_write,
	.proc_lseek     = seq_lseek,
	.proc_release    = single_release,
};

static void uids_proc_fs_init(struct proc_dir_entry *p_parent)
{
	struct proc_dir_entry *p_temp;

	if (!p_parent)
		goto out_p_temp;

	p_temp = proc_create(FS_FG_UIDS, FG_RW, p_parent, &proc_fg_uids_operations);
	if (!p_temp)
		goto out_p_temp;

out_p_temp:
	return ;
}


#define HEALTHINFO_PROC_NODE "oplus_healthinfo"
static struct proc_dir_entry *fg_dir = NULL;
static struct proc_dir_entry *healthinfo = NULL;
static struct proc_dir_entry *sched_thresh = NULL;


static __init int oplus_healthinfo_init(void)
{
	int ret = 0;
	struct proc_dir_entry *pentry;
	ohm_para_init();
	register_healthinfo_vendor_hooks();
	healthinfo =  proc_mkdir(HEALTHINFO_PROC_NODE, NULL);
	if(!healthinfo) {
		ohm_err("can't create healthinfo proc\n");
		goto ERROR_INIT_VERSION;
	}
/******  ctrl  *****/
	pentry = proc_create("para_update", S_IRUGO | S_IWUGO, healthinfo, &proc_para_fops);
        if(!pentry) {
                ohm_err("create para_update proc failed.\n");
                goto ERROR_INIT_VERSION;
        }

/******  Stat  ******/

	pentry = proc_create("cpu_loading", S_IRUGO, healthinfo, &proc_cpu_load_fops);
	if(!pentry) {
		ohm_err("create cpu_loading proc failed.\n");
		goto ERROR_INIT_VERSION;
	}

	pentry = proc_create("iowait", S_IRUGO, healthinfo, &proc_iowait_fops);
	if(!pentry) {
		ohm_err("create iowait proc failed.\n");
		goto ERROR_INIT_VERSION;
	}

	pentry = proc_create("sched_latency", S_IRUGO, healthinfo, &proc_sched_latency_fops);
	if(!pentry) {
		ohm_err("create sched_latency proc failed.\n");
		goto ERROR_INIT_VERSION;
	}

	pentry = proc_create("dstate", S_IRUGO, healthinfo, &proc_dstate_fops);
	if(!pentry) {
		ohm_err("create dstate proc failed.\n");
		goto ERROR_INIT_VERSION;
	}

	pentry = proc_create("iowait_hung", S_IRUGO, healthinfo, &proc_iowait_hung_fops);
	if(!pentry) {
		ohm_err("create iowait_hung proc failed.\n");
		goto ERROR_INIT_VERSION;
	}

	pentry = proc_create("alloc_wait", S_IRUGO, healthinfo, &proc_alloc_wait_fops);
	if(!pentry) {
		ohm_err("create alloc_wait proc failed.\n");
		goto ERROR_INIT_VERSION;
	}

	pentry = proc_create("cpu_info", S_IRUGO, healthinfo, &proc_cpu_info_fops);
	if(!pentry) {
		ohm_err("create cpu info proc failed.\n");
	goto ERROR_INIT_VERSION;
	}

	/****** thresh update ******/
	sched_thresh =  proc_mkdir("sched_thresh", healthinfo);
	if(!healthinfo) {
		ohm_err("can't create healthinfo proc\n");
		goto ERROR_INIT_VERSION;
	}

	pentry = proc_create("iowait_thresh", S_IRUGO | S_IWUGO, sched_thresh, &proc_iowait_thresh_fops);
	if(!pentry) {
		ohm_err("create iowait_thresh proc failed.\n");
		goto ERROR_INIT_VERSION;
	}
	pentry = proc_create("sched_latency_thresh", S_IRUGO | S_IWUGO, sched_thresh, &proc_sched_latency_thresh_fops);
	if(!pentry) {
		ohm_err("create sched_latency_thresh proc failed.\n");
		goto ERROR_INIT_VERSION;
	}

	fg_dir = proc_mkdir(FS_FG_INFO_PATH, NULL);
	if (!fg_dir){
		goto ERROR_INIT_VERSION;
	}
	else {
		uids_proc_fs_init(fg_dir);
	}
	ohm_debug("Success \n");
	return ret;

ERROR_INIT_VERSION:
	remove_proc_entry(HEALTHINFO_PROC_NODE, NULL);
	return -ENOENT;
}

static __exit void oplus_healthinfo_exit(void)
{
	unregister_healthinfo_vendor_hooks();
	if(healthinfo){
		remove_proc_entry(HEALTHINFO_PROC_NODE, NULL);
		healthinfo = NULL;
		if (sched_thresh){
			remove_proc_entry("sched_thresh", healthinfo);
			sched_thresh = NULL;
		}
	}
	if (fg_dir){
		remove_proc_entry(FS_FG_INFO_PATH, NULL);
		return;
	}
}

module_init(oplus_healthinfo_init);
module_exit(oplus_healthinfo_exit);

module_param_named(ohm_action_ctrl, ohm_action_ctrl, bool, S_IRUGO | S_IWUSR);
MODULE_LICENSE("GPL v2");
