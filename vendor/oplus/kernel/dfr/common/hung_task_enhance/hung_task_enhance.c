/***************************************************************
** Copyright (C),  2019-2020,  OPLUS Mobile Comm Corp.,  Ltd
** VENDOR_EDIT
** File : uboot_log.c
** Description : BSP uboot_log back up xbl uefi kernel boot log , cat /proc/boot_dmesg
** Version : 1.0
** Date : 2020/02/25
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
******************************************************************/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cred.h>
#include <linux/module.h>
#include <linux/nmi.h>
#include <linux/utsname.h>
#include <trace/events/sched.h>
#include <linux/sched/debug.h>
#include <linux/sched/sysctl.h>
#include <linux/sched/signal.h>
#include <linux/version.h>
#include <linux/string.h>

#include "hung_task_enhance.h"
#include "oplus_signal.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
#include <hooks/hung_task.h>
#else
#include <trace/hooks/hung_task.h>
#endif
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_THEIA) && (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
#include <soc/oplus/dfr/theia_send_event.h> /* for theia_send_event etc */
#endif

#if IS_ENABLED (CONFIG_OPLUS_BSP_DFR_USERSPACE_BACKTRACE)
#include <soc/oplus/dfr/oplus_bsp_dfr_ubt.h>
#endif /* CONFIG_OPLUS_BSP_DFR_USERSPACE_BACKTRACE */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
#define GET_STATE(t) (t->__state)
#include <linux/printk.h>
#else
#define GET_STATE(t) (t->state)
#endif


#if IS_ENABLED (CONFIG_OPLUS_FEATURE_DEATH_HEALER)
/* 
 * format: task_name,reason. e.g. system_server,uninterruptible for 60 secs
 */
#define HUNG_TASK_KILL_LEN	128
char __read_mostly sysctl_hung_task_kill[HUNG_TASK_KILL_LEN];
#define TWICE_DEATH_PERIOD	300000000000ULL	 /* 300s */
#define MAX_DEATH_COUNT	3
#define MAX_DEATH_COUNT_FOR_AGING 2
#define DISP_TASK_COMM_LEN_MASK 10

/* Foreground background optimization,change max io count */
#define MAX_IO_WAIT_HUNG 5
int __read_mostly sysctl_hung_task_maxiowait_count = MAX_IO_WAIT_HUNG;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int io_wait_count = 0;
#endif

#endif

/* key process:zygote system_server surfaceflinger*/
static bool is_usersapce_key_process(struct task_struct *t)
{
	const struct cred *tcred = __task_cred(t);
	if(!strcmp(t->comm, "main") && (tcred->uid.val == 0) && (t->parent != 0 && !strcmp(t->parent->comm,"init")))
		return true;
	if(!strncmp(t->comm,"system_server", TASK_COMM_LEN)
			|| !strncmp(t->comm,"surfaceflinger", TASK_COMM_LEN) )
		return true;
	if (!strncmp(t->comm, "Binder:", 7) && (t->group_leader->pid == t->pid)
			&& (tcred->uid.val == 1000) && (t->parent != 0 && !strcmp(t->parent->comm, "main")))
		return true;

	return false;
}


static bool is_ignore_process(struct task_struct *t)
{
	if(!strncmp(t->comm,"mdss_dsi_event", TASK_COMM_LEN)||
		!strncmp(t->comm,"msm-core:sampli", TASK_COMM_LEN)||
		!strncmp(t->comm,"mdss_fb0", TASK_COMM_LEN)||
		!strncmp(t->comm,"mdss_fb_ffl0", TASK_COMM_LEN)||
		!strncmp(t->comm,"hdcp_2x", TASK_COMM_LEN)||
		!strncmp(t->comm,"dp_hdcp2p2", TASK_COMM_LEN)||
		!strncmp(t->comm,"opmonitor_boot", TASK_COMM_LEN)||
		!strncmp(t->comm,"panic_flush", TASK_COMM_LEN)||
		!strncmp(t->comm,"fb_flush", TASK_COMM_LEN)||
		!strncmp(t->comm,"crtc_commit", DISP_TASK_COMM_LEN_MASK)||
		!strncmp(t->comm,"crtc_event", DISP_TASK_COMM_LEN_MASK)){
		return true;
	}
	return false;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
/* because of KMI, define local val */
int __read_mostly sysctl_hung_task_warnings = 10;
/*
 * Should we panic (and reboot, if panic_timeout= is set) when a
 * hung task is detected:
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
unsigned int __read_mostly sysctl_hung_task_panic = 0;
#else
unsigned int __read_mostly sysctl_hung_task_panic = CONFIG_BOOTPARAM_HUNG_TASK_PANIC_VALUE;
#endif
extern int send_sig_info(int sig, struct kernel_siginfo *info, struct task_struct *p);

static void oplus_check_hung_task(struct task_struct *t, unsigned long timeout, bool *need_check)
#else
static void oplus_check_hung_task(struct task_struct *t, unsigned long timeout, unsigned int *iowait_count, bool *show_lock, bool *call_panic)
#endif

{
	unsigned long switch_count = t->nvcsw + t->nivcsw;

#if IS_ENABLED (CONFIG_OPLUS_FEATURE_DEATH_HEALER)
	static unsigned long long last_death_time = 0;
	unsigned long long cur_death_time = 0;
	static int death_count = 0;
	unsigned int local_iowait = 0;
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_THEIA) && (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	char extra_info[64];
#endif

	if(is_ignore_process(t))
		return;

	/*
	 * Ensure the task is not frozen.
	 * Also, skip vfork and any other user process that freezer should skip.
	 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
	if (unlikely(t->__state == TASK_FROZEN))
#else
	if (unlikely(t->flags & (PF_FROZEN | PF_FREEZER_SKIP)))
#endif
	{
#if IS_ENABLED (CONFIG_OPLUS_FEATURE_DEATH_HEALER)
/* DeathHealer, kill D/T/t state tasks */
		if (is_usersapce_key_process(t)) 
		{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
			if (t->__state == TASK_FROZEN)
#else
			if (t->flags & PF_FROZEN)
#endif
				return;
		}
		else
#endif
		return;
	}

	/*
	 * When a freshly created task is scheduled once, changes its state to
	 * TASK_UNINTERRUPTIBLE without having ever been switched out once, it
	 * musn't be checked.
	 */
	if (unlikely(!switch_count))
		return;

	if (switch_count != t->last_switch_count) {
		t->last_switch_count = switch_count;
		#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)) || defined(CONFIG_OPLUS_SYSTEM_KERNEL_QCOM)
		t->last_switch_time = jiffies;
		#endif
		return;
	}
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)) || defined(CONFIG_OPLUS_SYSTEM_KERNEL_QCOM)
	if (time_is_after_jiffies(t->last_switch_time + timeout * HZ))
		return;
	#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
    trace_sched_process_hang(t);
#endif

#if IS_ENABLED (CONFIG_OPLUS_FEATURE_DEATH_HEALER)
	/* kill D/T/t state tasks ,if this task blocked at iowait. so maybe we should reboot system first */
	if(t->in_iowait){
		printk(KERN_ERR "DeathHealer task %s:%d io wait too long time\n", t->comm, t->pid);
                if(t->mm != NULL && t == t->group_leader)// only work on user main thread
                {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
                        io_wait_count = io_wait_count + 1;
#else
                        *iowait_count = *iowait_count + 1;
#endif
                        local_iowait = 1;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_THEIA) && (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
			memset(extra_info, 0, sizeof(extra_info));
			snprintf(extra_info, 64, "DeathHealer task %s:%d io wait too long time", t->comm, t->pid);
			theia_send_event(THEIA_EVENT_HUNGTASK, THEIA_LOGINFO_KERNEL_LOG, t->pid, extra_info);
#endif
                }
	}
	if (is_usersapce_key_process(t))
	{
		if (GET_STATE(t) == TASK_UNINTERRUPTIBLE)
			snprintf(sysctl_hung_task_kill, HUNG_TASK_KILL_LEN, "%s,uninterruptible for %lu seconds", t->comm, timeout);
		else if (GET_STATE(t) == TASK_STOPPED)
			snprintf(sysctl_hung_task_kill, HUNG_TASK_KILL_LEN, "%s,stopped for %lu seconds", t->comm, timeout);
		else if (GET_STATE(t) == TASK_TRACED)
			snprintf(sysctl_hung_task_kill, HUNG_TASK_KILL_LEN, "%s,traced for %lu seconds", t->comm, timeout);
		else
			snprintf(sysctl_hung_task_kill, HUNG_TASK_KILL_LEN, "%s,unknown hung for %lu seconds", t->comm, timeout);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
		printk(KERN_ERR "DeathHealer: task %s:%d blocked for more than %lu seconds in state %u. Count:%d\n",
			t->comm, t->pid, timeout, GET_STATE(t), death_count+1);
#else
		printk(KERN_ERR "DeathHealer: task %s:%d blocked for more than %lu seconds in state %lu. Count:%d\n",
			t->comm, t->pid, timeout, GET_STATE(t), death_count+1);
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_THEIA) && (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
		memset(extra_info, 0, sizeof(extra_info));
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
		snprintf(extra_info, 64, "DeathHealer: task %s:%d blocked for more than %lu seconds in state 0x%x. Count:%d\n",
			t->comm, t->pid, timeout, GET_STATE(t), death_count + 1);
#else
		snprintf(extra_info, 64, "DeathHealer: task %s:%d blocked for more than %lu seconds in state 0x%lx. Count:%d\n",
			t->comm, t->pid, timeout, GET_STATE(t), death_count + 1);
#endif
		theia_send_event(THEIA_EVENT_HUNGTASK, THEIA_LOGINFO_KERNEL_LOG, t->pid, extra_info);
#endif

#if IS_ENABLED (CONFIG_OPLUS_BSP_DFR_USERSPACE_BACKTRACE)
		dump_userspace_bt(t);
#endif /* CONFIG_OPLUS_BSP_DFR_USERSPACE_BACKTRACE */
		sched_show_task(t);
		debug_show_held_locks(t);
		trigger_all_cpu_backtrace();

		death_count++;
		cur_death_time = local_clock();
		if ((death_count >= MAX_DEATH_COUNT) 
			|| (death_count >= MAX_DEATH_COUNT_FOR_AGING && get_eng_version() == AGING)) {
			if (cur_death_time - last_death_time < TWICE_DEATH_PERIOD) {
				printk(KERN_ERR "DeathHealer has been triggered %d times, \
					last time at: %llu\n", death_count, last_death_time);
				BUG();
			}else{
				death_count = 0;
				printk(KERN_ERR "DeathHealer reset death_count to 0");
			}
		}
		last_death_time = cur_death_time;

        if (get_eng_version() == AGING)
            BUG();

		t->flags |= PF_KILLING;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
		send_sig_info(SIGKILL, SEND_SIG_PRIV, t);
#else
		send_sig_info(SIGKILL, SEND_SIG_FORCED, t);
#endif
		wake_up_process(t);
	}
#endif

	if (sysctl_hung_task_panic) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
		console_verbose();
#endif
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
		*show_lock = true;
		*call_panic = true;
#endif

		/* Panic on critical process D-state */
		if (is_usersapce_key_process(t))
		{
			trigger_all_cpu_backtrace();
			panic("hung_task: blocked tasks");
		}

	}

	/*
	 * Ok, the task did not get scheduled for more than 2 minutes,
	 * complain:
	 */
#if IS_ENABLED (CONFIG_OPLUS_FEATURE_DEATH_HEALER)
    /* Modify for make sure we could print the stack of iowait thread before panic */
	if (sysctl_hung_task_warnings || local_iowait)
#else
	if (sysctl_hung_task_warnings)
#endif
	{
		if (sysctl_hung_task_warnings > 0)
			sysctl_hung_task_warnings--;
		pr_err("INFO: task %s:%d blocked for more than %lu seconds.\n",
			t->comm, t->pid, timeout);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
		pr_err("      %s %s %.*s\n",
			print_tainted(), init_utsname()->release,
			(int)strcspn(init_utsname()->version, " "),
			init_utsname()->version);
		pr_err("\"echo 0 > /proc/sys/kernel/hung_task_timeout_secs\""
			" disables this message.\n");
#endif
		sched_show_task(t);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
        *show_lock = true;
#endif
	}
	touch_nmi_watchdog();
}


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
void io_check_hung_detection(void *ignore, struct task_struct *t, unsigned long timeout, bool *need_check)
{
#if IS_ENABLED (CONFIG_OPLUS_FEATURE_DEATH_HEALER)
	/* add io wait monitor */
	if (GET_STATE(t) == TASK_UNINTERRUPTIBLE || GET_STATE(t) == TASK_STOPPED || GET_STATE(t) == TASK_TRACED)
		oplus_check_hung_task(t, timeout, need_check);
#endif
	return;
}
EXPORT_SYMBOL(io_check_hung_detection);

void io_block_panic(void *ignore, void *extra)
{
#if IS_ENABLED (CONFIG_OPLUS_FEATURE_DEATH_HEALER)
/* Foreground background optimization,change max io count */
	if (io_wait_count >= sysctl_hung_task_maxiowait_count) {
		panic("hung_task:[%u]IO blocked too long time", io_wait_count);
       }
	io_wait_count = 0;
#endif
	return;
}
EXPORT_SYMBOL(io_block_panic);

static int __init hung_task_enhance_init(void)
{
	int ret;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
	ret = register_trace_android_vh_check_uninterrupt_tasks(
						io_check_hung_detection, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_check_uninterrupt_tasks_done(
						io_block_panic, NULL);
	if (ret) {
		unregister_trace_android_vh_check_uninterrupt_tasks(
						io_check_hung_detection, NULL);
		return ret;
	}
#else
	ret = register_trace_android_vh_check_uninterruptible_tasks(
						io_check_hung_detection, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_check_uninterruptible_tasks_dn(
						io_block_panic, NULL);
	if (ret) {
		unregister_trace_android_vh_check_uninterruptible_tasks(
						io_check_hung_detection, NULL);
		return ret;
	}
#endif

#if IS_ENABLED (CONFIG_OPLUS_BSP_DFR_USERSPACE_BACKTRACE)
	dump_userspace_init("ubt,hung_task");
#endif /* CONFIG_OPLUS_BSP_DFR_USERSPACE_BACKTRACE */
	return 0;
}
late_initcall(hung_task_enhance_init);

static void __exit hung_task_enh_exit(void)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
	unregister_trace_android_vh_check_uninterrupt_tasks(
						io_check_hung_detection, NULL);
	unregister_trace_android_vh_check_uninterrupt_tasks_done(
						io_block_panic, NULL);
#else
	unregister_trace_android_vh_check_uninterruptible_tasks(
						io_check_hung_detection, NULL);
	unregister_trace_android_vh_check_uninterruptible_tasks_dn(
						io_block_panic, NULL);
#endif
}
module_exit(hung_task_enh_exit);
MODULE_LICENSE("GPL v2");
#else
void io_check_hung_detection(struct task_struct *t, unsigned long timeout, unsigned int *iowait_count, bool *show_lock, bool *call_panic)
{
#ifdef CONFIG_OPLUS_FEATURE_DEATH_HEALER
	/* add io wait monitor */
	if (GET_STATE(t) == TASK_UNINTERRUPTIBLE || GET_STATE(t) == TASK_STOPPED || GET_STATE(t) == TASK_TRACED)
		/* Check for selective monitoring */
		if (!sysctl_hung_task_selective_monitoring ||
			t->hang_detection_enabled)
			oplus_check_hung_task(t, timeout, iowait_count, show_lock, call_panic);
#endif
	return;
}
EXPORT_SYMBOL(io_check_hung_detection);

void io_block_panic(unsigned int *iowait_count, unsigned int sys_mamxiowait_count)
{
#ifdef CONFIG_OPLUS_FEATURE_DEATH_HEALER
/* Foreground background optimization,change max io count */
	if(*iowait_count >= sysctl_hung_task_maxiowait_count){
		panic("hung_task:[%u]IO blocked too long time",*iowait_count);
       }
	*iowait_count = 0;
#endif
	return;
}
EXPORT_SYMBOL(io_block_panic);
#endif
