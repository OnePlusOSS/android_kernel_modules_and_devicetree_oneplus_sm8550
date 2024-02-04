// SPDX-License-Identifier: GPL-2.0-only
/**************************************************************
* Copyright (c)  2019- 2039   Oplus. All rights reserved.
*
* File       : oplus_secure_harden.c
* Description: Security for kernel harden.
* Version   : 1.0
* Date        : 2019-12-19
* Author    :
* TAG         :
****************************************************************/

#include <asm/pgtable.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/pid.h>
//#include <linux/current.h> 	/*current*/
#include <linux/sched.h>	/*is_global_init, find_task_by_vpid ()*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kprobes.h>
#include <linux/uaccess.h>
#include <linux/ktime.h>
#include <linux/bootconfig.h>

/* Used to determine whether it is MTK platform or kernel version >= 5.15*/
#include <linux/device.h>
#include <linux/version.h>
#if !defined(CONFIG_OPLUS_SYSTEM_KERNEL_QCOM) || (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
/* for dx1 bringup */
#ifdef __KERNEL__
#ifndef _STRUCT_TIMESPEC
#define _STRUCT_TIMESPEC
struct timespec {
	__kernel_old_time_t	tv_sec;		/* seconds */
	long			tv_nsec;	/* nanoseconds */
};
#endif /* _STRUCT_TIMESPEC */
#endif /* __KERNEL__ */

#if __BITS_PER_LONG == 64
/* timespec64 is defined as timespec here */
static inline struct timespec timespec64_to_timespec(const struct timespec64 ts64) {
	return *(const struct timespec *)&ts64;
}

static inline struct timespec64 timespec_to_timespec64(const struct timespec ts) {
	return *(const struct timespec64 *)&ts;
}

#else
static inline struct timespec timespec64_to_timespec(const struct timespec64 ts64) {
	struct timespec ret;

	ret.tv_sec = (time_t)ts64.tv_sec;
	ret.tv_nsec = ts64.tv_nsec;
	return ret;
}

static inline struct timespec64 timespec_to_timespec64(const struct timespec ts) {
	struct timespec64 ret;

	ret.tv_sec = ts.tv_sec;
	ret.tv_nsec = ts.tv_nsec;
	return ret;
}
#endif /* __BITS_PER_LONG */

static inline void getnstimeofday(struct timespec *ts) {
	struct timespec64 ts64;

	ktime_get_real_ts64(&ts64);
	*ts = timespec64_to_timespec(ts64);
}
#endif /* LINUX_VERSION_CODE */
#endif /* MTK or NEW_KERNEL_VERSION */

#include "oplus_guard_general.h"
#include "oplus_secure_harden.h"
#include "oplus_kevent.h"

/********************** Feature Variable Start**********************/

/* Heapspary layout[x]: PPID, COUNT, TIME*/
unsigned int heapspary_ip4[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned int heapspary_cpuinfo[3] = {0, 0, 0};
unsigned int heapspary_xttr[3] = {0, 0, 0};
unsigned int heapspary_ip6[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

/*Used for hook function name by Kprobe.*/
static char func_name_sepolicy_reload[NAME_MAX] = "sel_write_load";
#if 0
/* MSM8450's kallsyms is not include do_ip_setsockopt , so upstream check point */
static char func_name_socket[NAME_MAX] = "do_ip_setsockopt";/* ipv4 */
#else
static char func_name_socket[NAME_MAX] = "ip_setsockopt";/* ipv4 */
#endif
static char func_name_socket_ip6[NAME_MAX] = "do_ipv6_setsockopt";/* ipv6 */
static char func_name_cpu_info[NAME_MAX] = "cpuinfo_open";
static char func_name_setxattr[NAME_MAX] = "setxattr";



/********************** Feature Variable End**********************/


/********************** Feature Code Start**********************/

extern char verified_bootstate[];
static int oplus_is_unlocked(void) {
	if (strstr(verified_bootstate, "orange"))
		return 1;

	return 0;
}

/* ---CONFIG_OPLUS_KEVENT_UOLOAD --- */
/**
* report_secuiry_event:
* This is a kernel api for burying points, which transfers the key information of security events to Native for analysis and burying points.
* @event_name: Event type of security check. (e.g..root_check/capa/heapspary...)
* @event_type: Security event fuctions type. (e.g. MCAST_JOIN_GROUP/IP_M.. of Heapsparty...)
* @more: (Additional information)
* @Return: Void type, non-return.
**/

void report_secuiry_event(const char* event_name, unsigned int event_type, const char* more) {
	struct kernel_packet_info* dcs_event;
	char dcs_stack[sizeof(struct kernel_packet_info) + 256];
	const char *dcs_event_tag = "kernel_event";
	const char *dcs_event_id = event_name;
	char *dcs_event_payload = NULL;

	dcs_event = (struct kernel_packet_info*)dcs_stack;
	dcs_event_payload = dcs_stack + sizeof(struct kernel_packet_info);
	dcs_event->type = event_type;/*set type of security event*/
	strncpy(dcs_event->log_tag, dcs_event_tag,
	sizeof(dcs_event->log_tag));
	strncpy(dcs_event->event_id, dcs_event_id,
	sizeof(dcs_event->event_id));
	/*accrding type, chosse array*/
	dcs_event->payload_length = snprintf(dcs_event_payload, 256, "$$uid@@%d$$EVENT_TYPE@@%d$$current_name@@%s$$additional@@%s\n", current_uid().val, event_type, current->comm, more);
	if (dcs_event->payload_length < 256) {
		dcs_event->payload_length += 1;
	}
	kevent_send_to_user(dcs_event);
}
EXPORT_SYMBOL(report_secuiry_event);

#if 1//fdef CONFIG_OPLUS_HP_CHECK

/* ---CONFIG_OPLUS_HP_CHECK --- */
/**
* oplus_heapspray_check:
* By detecting how often the function is called by the same process, judge whether is a heapspary.
* If an exception is detected, exception handling it.(e.g.. kill process)
* @type: Type of entered
* @Return: Void type, non-return.
**/
void oplus_heapspray_check(unsigned int type) {
	struct timespec ts;
	const char* event_type = "heapspray";
	unsigned int new_ppid = current->real_parent->pid;

	/*bypass if root process && unlocked state */
	if ((!current_uid().val) || oplus_is_unlocked()) {
		return;
	}
  	getnstimeofday(&ts);
	/*
	task_ppid_nr() calls init_ppid_ns. But init_ppid_ns is not include into whilelist by Android R + K5.4.
	May support Android S + Kernel 5.10.
	unsigned int new_ppid = task_ppid_nr(current);
	*/
	switch(type) {
	case CPU_INFO:
		/* Only detect the same caller. */
		if (new_ppid == heapspary_cpuinfo[0]) {
			/* The time interval greater than 10s, then count again to avoid normal process being intercepted. */
			if ((ts.tv_sec - heapspary_cpuinfo[2]) >= HEAPSPARY_TIME_LIMIT) {
				heapspary_cpuinfo[1] = 0;
				heapspary_cpuinfo[2] = ts.tv_sec;
			/*For the first record, the initial value needs to be set.*/
			} else if (!heapspary_cpuinfo[2]) {
				heapspary_cpuinfo[2] = ts.tv_sec;
			/*Detect abnormal process: 1.Exceed the limit of 200 times within 10s of time intercal.*/
			} else if (heapspary_cpuinfo[1] > HEAPSPART_COUNT_LIMIT) {
				printk("[ROOTCHECK-HS-ERROR]%s:Detected the CPU_INFO may be abnormal, marked it! (tiem diff: %d)\n", __func__, ts.tv_sec - heapspary_cpuinfo[2]);
				heapspary_cpuinfo[1] = 0;
				heapspary_cpuinfo[2] = 0;
				/*force_sig is not include in whilelist*/
				//do_exit(SIGKILL);
				report_secuiry_event(event_type, type, "");
			}
		heapspary_cpuinfo[1]++;
		} else {
			/*Record the first call of different process.*/
			heapspary_cpuinfo[0] = new_ppid;
			heapspary_cpuinfo[1] = 0;
			heapspary_cpuinfo[2] = 0;
		}
		break;

		case SET_XATTR:
		if (new_ppid == heapspary_xttr[0]) {
			if ((ts.tv_sec - heapspary_xttr[2]) >= HEAPSPARY_TIME_LIMIT) {
				heapspary_xttr[1] = 0;
				heapspary_xttr[2] = ts.tv_sec;
			} else if (!heapspary_xttr[2]) {
				heapspary_xttr[2] = ts.tv_sec;
			} else if (heapspary_xttr[1] > HEAPSPART_COUNT_LIMIT) {
				printk("[ROOTCHECK-HS-ERROR]%s:Detected the SET_XATTR may be abnormal, intercepted it! (tiem diff: %d)\n", __func__, ts.tv_sec - heapspary_xttr[2]);
				heapspary_xttr[1] = 0;
				heapspary_xttr[2] = 0;
				//do_exit(SIGKILL);
				report_secuiry_event(event_type, type, "");
			}
			heapspary_xttr[1]++;
		} else {
			heapspary_xttr[0] = new_ppid;
			heapspary_xttr[1] = 0;
			heapspary_xttr[2] = 0;
		}
		break;

		case MCAST_MSFILTER_IP4:
		if (new_ppid == heapspary_ip4[0]) {
			if ((ts.tv_sec - heapspary_ip4[2]) >= HEAPSPARY_TIME_LIMIT) {
				heapspary_ip4[1] = 0;
				heapspary_ip4[2] = ts.tv_sec;
			} else if (!heapspary_ip4[2]) {
				heapspary_ip4[2] = ts.tv_sec;
			} else if (heapspary_ip4[1] > HEAPSPART_COUNT_LIMIT) {
				printk("[ROOTCHECK-HS-ERROR]%s:Detected the MCAST_MSFILTER_IP4 may be abnormal, intercepted it! (tiem diff: %d)\n", __func__, ts.tv_sec - heapspary_ip4[2]);
				heapspary_ip4[1] = 0;
				heapspary_ip4[2] = 0;
				//do_exit(SIGKILL);
				report_secuiry_event(event_type, type, "");
			}
			heapspary_ip4[1]++;
		} else {
			heapspary_ip4[0] = new_ppid;
			heapspary_ip4[1] = 0;
			heapspary_ip4[2] = 0;
		}
		break;

		case MCAST_JOIN_GROUP_IP4:
		if (new_ppid == heapspary_ip4[3]) {
			if ((ts.tv_sec - heapspary_ip4[5]) >= HEAPSPARY_TIME_LIMIT) {
				heapspary_ip4[4] = 0;
				heapspary_ip4[5] = ts.tv_sec;
			} else if (!heapspary_ip4[5]) {
				heapspary_ip4[5] = ts.tv_sec;
			} else if (heapspary_ip4[4] > HEAPSPART_COUNT_LIMIT) {
				printk("[ROOTCHECK-HS-ERROR]%s:Detected the MCAST_JOIN_GROUP_IP4 may be abnormal, intercepted it! (tiem diff: %d)\n", __func__, ts.tv_sec - heapspary_ip4[5]);
				heapspary_ip4[4] = 0;
				heapspary_ip4[5] = 0;
				//do_exit(SIGKILL);
				report_secuiry_event(event_type, type, "");
			}
			heapspary_ip4[4]++;
		} else {
			heapspary_ip4[3] = new_ppid;
			heapspary_ip4[4] = 0;
			heapspary_ip4[5] = 0;
		}
		break;

		case IP_MSFILTER_IP4:
		if (new_ppid == heapspary_ip4[6]) {
			if ((ts.tv_sec - heapspary_ip4[8]) >= HEAPSPARY_TIME_LIMIT) {
				heapspary_ip4[7] = 0;
				heapspary_ip4[8] = ts.tv_sec;
			} else if (!heapspary_ip4[8]) {
				heapspary_ip4[8] = ts.tv_sec;
			} else if (heapspary_ip4[7] > HEAPSPART_COUNT_LIMIT) {
				printk("[ROOTCHECK-HS-ERROR]%s:Detected the IP_MSFILTER_IP4 may be abnormal, intercepted it! (tiem diff: %d)\n", __func__, ts.tv_sec - heapspary_ip4[8]);
				heapspary_ip4[7] = 0;
				heapspary_ip4[8] = 0;
				//do_exit(SIGKILL);
				report_secuiry_event(event_type, type, "");
			}
			heapspary_ip4[7]++;
		} else {
			heapspary_ip4[6] = new_ppid;
			heapspary_ip4[7] = 0;
			heapspary_ip4[8] = 0;
		}
		break;

		case MCAST_JOIN_GROUP_IP6:
		if (new_ppid == heapspary_ip6[0]) {
			if ((ts.tv_sec - heapspary_ip6[2]) >= HEAPSPARY_TIME_LIMIT) {
				heapspary_ip6[1] = 0;
				heapspary_ip6[2] = ts.tv_sec;
			} else if (!heapspary_ip6[2]) {
				heapspary_ip6[2] = ts.tv_sec;
			} else if (heapspary_ip6[1] > HEAPSPART_COUNT_LIMIT) {
				printk("[ROOTCHECK-HS-ERROR]%s:Detected the MCAST_JOIN_GROUP_IP6 may be abnormal, intercepted it! (tiem diff: %d)\n", __func__, ts.tv_sec - heapspary_ip6[2]);
				heapspary_ip6[1] = 0;
				heapspary_ip6[2] = 0;
				//do_exit(SIGKILL);
				report_secuiry_event(event_type, type, "");
			}
			heapspary_ip6[1]++;
		} else {
			heapspary_ip6[0] = new_ppid;
			heapspary_ip6[1] = 0;
			heapspary_ip6[2] = 0;
		}
		break;

		case MCAST_MSFILTER_IP6:
		if (new_ppid == heapspary_ip6[3]) {
			if ((ts.tv_sec - heapspary_ip6[5]) >= HEAPSPARY_TIME_LIMIT) {
				heapspary_ip6[4] = 0;
				heapspary_ip6[5] = ts.tv_sec;
			} else if (!heapspary_ip6[5]) {
				heapspary_ip6[5] = ts.tv_sec;
			} else if (heapspary_ip6[4] > HEAPSPART_COUNT_LIMIT) {
				printk("[ROOTCHECK-HS-ERROR]%s:Detected the MCAST_MSFILTER_IP6 may be abnormal, intercepted it! (tiem diff: %d)\n", __func__, ts.tv_sec - heapspary_ip6[5]);
				heapspary_ip6[4] = 0;
				heapspary_ip6[5] = 0;
				//do_exit(SIGKILL);
				report_secuiry_event(event_type, type, "");
			}
			heapspary_ip6[4]++;
		} else {
			heapspary_ip6[3] = new_ppid;
			heapspary_ip6[4] = 0;
			heapspary_ip6[5] = 0;
		}
		break;
	} /*switch(type)*/
}
#endif /*CONFIG_OPLUS_HP_CHECK*/

/* ---CONFIG_OPLUS_SEPOLICY_RELOAD --- */
/**
*oplus_sepolicy_reload:
*Only the init process is allowed to load sepolicy, and the other process calls will be bolcked.
*@type: Void
*@Return: Void type, non-return.
**/
void oplus_sepolicy_reload(void) {
	const char* event_type = "spolicy_reload";

  	if (oplus_is_unlocked())
		return;

	if(!is_global_init(current)) {
		printk("[ROOTCHECK-SR-ERROR]%s:Detected illegal porcess reload policy!!!\n", __func__);
		//do_exit(SIGKILL);
		report_secuiry_event(event_type, SEPOLICY_RL, "");
		}
}

/********************** Feature Code End**********************/


/********************** Feature Hook Code Start**********************/

/* ---CONFIG_OPLUS_HP_CHECK --- */
static int entry_handler_socket(struct kretprobe_instance *ri, struct pt_regs *regs) {
	int socket_type = regs->regs[1];

	if (socket_type == MCAST_MSFILTER){
		oplus_heapspray_check(MCAST_MSFILTER_IP4);
	}else if (socket_type == MCAST_JOIN_GROUP){
		oplus_heapspray_check(MCAST_JOIN_GROUP_IP4);
	}else if (socket_type == IP_MSFILTER){
		oplus_heapspray_check(IP_MSFILTER_IP4);
	}
	return 0;
}
static int entry_handler_socket_ip6(struct kretprobe_instance *ri, struct pt_regs *regs) {
	int socket_type = regs->regs[1];

	if (socket_type == MCAST_JOIN_GROUP){
		oplus_heapspray_check(MCAST_JOIN_GROUP_IP4);
	}else if (socket_type == IP_MSFILTER){
		oplus_heapspray_check(IP_MSFILTER_IP4);
	}
	return 0;
}
static int entry_handler_cpuinfo(struct kretprobe_instance *ri, struct pt_regs *regs) {
	oplus_heapspray_check(CPU_INFO);
	return 0;
}
static int entry_handler_setxattr(struct kretprobe_instance *ri, struct pt_regs *regs) {
	oplus_heapspray_check(SET_XATTR);
	return 0;
}

/* ---CONFIG_OPLUS_SEPOLICY_RELOAD --- */
static int entry_handler_sepolicy_reload(struct kretprobe_instance *ri, struct pt_regs *regs) {
	oplus_sepolicy_reload();
	return 0;

}

/* ---CONFIG_OPLUS_SEPOLICY_RELOAD --- */
static struct kretprobe sepolicy_reload_kretprobe = {
//	.handler		= ret_handler,
	.entry_handler  = entry_handler_sepolicy_reload,
//	.data_size		= sizeof(struct my_data),
	.maxactive		= 20,
};

/* ---CONFIG_OPLUS_HP_CHECK --- */
static struct kretprobe socket_kretprobe = {
	.entry_handler  = entry_handler_socket,
	.data_size		= sizeof(struct pt_regs),
	.maxactive		= 300,
};
static struct kretprobe socket_ip6_kretprobe = {
	.entry_handler  = entry_handler_socket_ip6,
	.data_size		= sizeof(struct pt_regs),
	.maxactive		= 300,
};
static struct kretprobe cpuinfo_kretprobe = {
	.entry_handler  = entry_handler_cpuinfo,
	.data_size		= sizeof(struct pt_regs),
	.maxactive		= 300,
};
static struct kretprobe setxattr_kretprobe = {
	.entry_handler  = entry_handler_setxattr,
	.data_size		= sizeof(struct pt_regs),
	.maxactive		= 300,
};

static int __init kretprobe_init(void) {
	int ret = 0;

	sepolicy_reload_kretprobe.kp.symbol_name = func_name_sepolicy_reload;
	socket_kretprobe.kp.symbol_name = func_name_socket;
	socket_ip6_kretprobe.kp.symbol_name = func_name_socket_ip6;
	cpuinfo_kretprobe.kp.symbol_name = func_name_cpu_info;
	setxattr_kretprobe.kp.symbol_name = func_name_setxattr;

	#ifdef CONFIG_OPLUS_FEATURE_SECURE_SRGUARD
	ret = register_kretprobe(&sepolicy_reload_kretprobe);
	if (ret < 0) {
		printk("[ROOTCHECK-SR-ERROR]%s:register sepolicy_write_load FAILED.!\n", __func__);
		goto sepolicy_reload_kretprobe_failed;
	}
	#endif /* CONFIG_OPLUS_FEATURE_SECURE_SRGUARD */
	#ifdef CONFIG_OPLUS_FEATURE_SECURE_SOCKETGUARD
	ret = register_kretprobe(&socket_kretprobe);
	if (ret < 0) {
		printk("[ROOTCHECK-HS-ERROR]%s:register do_setsocket_opt FAILED.!\n", __func__);
		goto socket_kretprobe_failed;
	}
	ret = register_kretprobe(&socket_ip6_kretprobe);
	if (ret < 0) {
		printk("[ROOTCHECK-HS-ERROR]%s:register do_ip6_setsocket_opt FAILED.!\n", __func__);
		goto socket_ip6_kretprobe_failed;
	}
	ret = register_kretprobe(&cpuinfo_kretprobe);
	if (ret < 0) {
		printk("[ROOTCHECK-HS-ERROR]%s:register func_name_cpu_info FAILED.!\n", __func__);
		goto cpuinfo_kretprobe_failed;
	}

	ret = register_kretprobe(&setxattr_kretprobe);
	if (ret < 0) {
		printk("[ROOTCHECK-HS-ERROR]%s:register func_name_setxattr FAILED.!\n", __func__);
		goto setxattr_kretprobe_failed;
	}
	#endif /* CONFIG_OPLUS_FEATURE_SECURE_SOCKETGUARD */
	printk("[ROOTCHECK-INFO]%s:secure_harden has been register.\n", __func__);
	return ret;

setxattr_kretprobe_failed:
	unregister_kretprobe(&cpuinfo_kretprobe);
cpuinfo_kretprobe_failed:
	unregister_kretprobe(&socket_ip6_kretprobe);
socket_ip6_kretprobe_failed:
	unregister_kretprobe(&socket_kretprobe);
socket_kretprobe_failed:
	unregister_kretprobe(&sepolicy_reload_kretprobe);
sepolicy_reload_kretprobe_failed:
	return 0;
}
static void __exit kretprobe_exit(void) {
	#ifdef CONFIG_OPLUS_FEATURE_SECURE_SRGUARD
	unregister_kretprobe(&sepolicy_reload_kretprobe);
	#endif /* CONFIG_OPLUS_FEATURE_SECURE_SRGUARD */
	#ifdef CONFIG_OPLUS_FEATURE_SECURE_SOCKETGUARD
	unregister_kretprobe(&socket_kretprobe);
	unregister_kretprobe(&socket_ip6_kretprobe);
	unregister_kretprobe(&cpuinfo_kretprobe);
	unregister_kretprobe(&setxattr_kretprobe);
	#endif /* CONFIG_OPLUS_FEATURE_SECURE_SOCKETGUARD */
	printk("[ROOTCHECK-INFO]%s:Module of secure_harden has been unregister.\n", __func__);
}

/********************** Feature Hook Code End**********************/
MODULE_PARM_DESC(func, "This is a module developed by OPLUS's AMBER LAB to use a strengthened kernel to enhance the security capabilities of the kernel.");
module_init(kretprobe_init)
module_exit(kretprobe_exit)
MODULE_LICENSE("GPL v2");
