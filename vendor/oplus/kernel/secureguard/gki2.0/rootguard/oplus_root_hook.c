// SPDX-License-Identifier: GPL-2.0-only
/**************************************************************
* Copyright (c)  2017- 2030  OPLUS Mobile communication Corp.ltd All rights reserved.
*
* File		: oplus_root_hooc.c
* Description	: For root check
* Version   	: 1.0
* Date      	: 2021-6-9
* Author    	:
* TAG       	:
****************************************************************/

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/cred.h>
#include <linux/version.h>
#include <trace/events/syscalls.h>
#include "oplus_root_hook.h"
#include "oplus_kevent.h"
#include "oplus_guard_general.h"

#if defined(WHITE_LIST_SUPPORT)
#include <linux/string.h>
#include <linux/sched/task.h>
#endif /* WHITE_LIST_SUPPORT */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
static int selinux_enabled = 1;
#else
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
unsigned int get_fs(void)
{
	return 0;
}
#endif

extern int kevent_send_to_user(struct kernel_packet_info *userinfo);
void oplus_root_check_succ(uid_t uid, uid_t euid, uid_t egid, uid_t callnum)
{
	struct kernel_packet_info *dcs_event;
	char dcs_stack[sizeof(struct kernel_packet_info) + 256];
	const char* dcs_event_tag = "kernel_event";
	const char* dcs_event_id = "root_check";
	char* dcs_event_payload = NULL;

	int ret = -1;
	char comm[TASK_COMM_LEN], nameofppid[TASK_COMM_LEN];
	struct task_struct * parent_task = NULL;
	int ppid = -1;

	memset(comm, 0, TASK_COMM_LEN);
	memset(nameofppid, 0, TASK_COMM_LEN);

	//ppid = task_ppid_nr(current);
	parent_task = rcu_dereference(current->real_parent);
	if (parent_task){
		get_task_comm(nameofppid, parent_task);
		ppid = parent_task->pid;
	}

	dcs_event = (struct kernel_packet_info*)dcs_stack;
	dcs_event->type = 0;
	strncpy(dcs_event->log_tag, dcs_event_tag, sizeof(dcs_event->log_tag));
	strncpy(dcs_event->event_id, dcs_event_id, sizeof(dcs_event->event_id));
	dcs_event_payload = kmalloc(256, GFP_ATOMIC);
	if (NULL == dcs_event_payload){
		return;
	}
	memset(dcs_event_payload, 0, 256);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	dcs_event->payload_length = snprintf(dcs_event_payload,256,
	    "%d$$old_euid@@%d$$old_fsuid@@%d$$sys_call_number@@%d$$addr_limit@@%lx$$curr_uid@@%d$$curr_euid@@%d$$curr_fsuid@@%d$$curr_name@@%s$$ppid@@%d$$ppidname@@%s$$enforce@@%d\n",
	    uid,euid,egid,callnum,
	    get_fs(),current_uid().val,current_euid().val,current_fsuid().val,get_task_comm(comm, current), ppid, nameofppid,selinux_is_enabled());
#else
	dcs_event->payload_length = snprintf(dcs_event_payload,256,
	    "%d$$old_euid@@%d$$old_egid@@%d$$sys_call_number@@%d$$addr_limit@@%lx$$curr_uid@@%d$$curr_euid@@%d$$curr_egid@@%d$$curr_name@@%s$$ppid@@%d$$ppidname@@%s$$enforce@@%d\n",
	    uid,euid,egid,callnum,
	    get_fs(),current_uid().val,current_euid().val,current_egid().val,get_task_comm(comm, current), ppid, nameofppid,selinux_enabled);
#endif
	printk(KERN_INFO "[ROOTCHECK-RC-INFO]oplus_root_check_succ,payload:%s\n",dcs_event_payload);
	memcpy(dcs_event->payload, dcs_event_payload, strlen(dcs_event_payload));

	ret = kevent_send_to_user(dcs_event);
	if (ret != 0 ){
		printk(KERN_INFO "[ROOTCHECK-RC-ERROR]Send to user failed\n");
	}

	kfree(dcs_event_payload);

	return;
}

void oplus_root_killed(void)
{
	printk(KERN_INFO "[ROOTCHECK-RC-ERROR]:Kill the process of escalation...");
	do_exit(SIGKILL);
	//printk(KERN_INFO "[OPLUS_ROOT_CHECK]:ptr_exit is 0x%lx", (unsigned long)ptr_exit);

}


void oplus_root_check_pre_handler(void *data, struct pt_regs *regs, long id)
{

	current->android_kabi_reserved4 = regs->syscallno;
	current->android_kabi_reserved5 = current_uid().val;
	current->android_kabi_reserved6 = current_euid().val;
	current->android_kabi_reserved7 = current_gid().val;
	current->android_kabi_reserved8 = current_egid().val;

    return ;
}

void oplus_root_check_post_handler(void *data, struct pt_regs *regs, long ret)
{
	uid_t scno =current->android_kabi_reserved4;

#if defined(WHITE_LIST_SUPPORT)
	char nameofppid[TASK_COMM_LEN];
	struct task_struct * parent_task = NULL;
#endif /* WHITE_LIST_SUPPORT */

	if ((0 != current->android_kabi_reserved5) && (is_unlocked() == 0)) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
		if ((current->android_kabi_reserved5 > current_uid().val) || (current->android_kabi_reserved6 > current_euid().val)\
					|| (current->android_kabi_reserved7 > current_gid().val)\
					|| (current->android_kabi_reserved8 > current_egid().val)) {
#else
		if ((current->android_kabi_reserved5 > current_uid().val) || (current->android_kabi_reserved6 > current_euid().val)\
					|| (current->android_kabi_reserved7 > current_gid().val)\
					|| (current->android_kabi_reserved8 > current_egid().val)\
					|| (get_fs() > KERNEL_ADDR_LIMIT)) {
#endif
			if((scno != __NR_setreuid32) && (scno != __NR_setregid32) && (scno != __NR_setresuid32) && (scno != __NR_setresgid32) && (scno != __NR_setuid32) && (scno != __NR_setgid32)
				&& (scno != __NR_setreuid) && (scno != __NR_setregid) && (scno != __NR_setresuid) && (scno != __NR_setresgid) && (scno != __NR_setuid) && (scno != __NR_setgid)){
					#if defined(WHITE_LIST_SUPPORT)
					memset(nameofppid, 0, TASK_COMM_LEN);
					parent_task = rcu_dereference(current->real_parent);
					if (parent_task) {
						get_task_comm(nameofppid, parent_task);
					}
					if (strncmp(nameofppid, "dumpstate", 9)) {
						oplus_root_check_succ(current->android_kabi_reserved5, current->android_kabi_reserved6, current->android_kabi_reserved8, current->android_kabi_reserved4);
						oplus_root_killed();
					}
					#else
					oplus_root_check_succ(current->android_kabi_reserved5, current->android_kabi_reserved6, current->android_kabi_reserved8, current->android_kabi_reserved4);
					oplus_root_killed();
					#endif  /* WHITE_LIST_SUPPORT */
			}
		}
	}
	return ;
}

MODULE_LICENSE("GPL");


