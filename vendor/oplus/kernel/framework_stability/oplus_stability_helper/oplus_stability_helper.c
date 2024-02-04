// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2030 Oplus. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cred.h>
#include <trace/hooks/signal.h>

#define ROOT_UID	0
#define SYSTEM_UID	1000

static bool is_root_process(struct task_struct *t)
{
	if (task_uid(t).val == ROOT_UID) {
		if ((!strcmp(t->comm, "main")
			&& (t->parent != NULL && !strcmp(t->parent->comm, "init"))) /*zygote*/
			|| !strcmp(t->comm, "Binder:netd")) {
			return true;
		}
	}
	return false;
}

static bool is_system_process(struct task_struct *t)
{
	if (task_uid(t).val == SYSTEM_UID) {
		if (!strcmp(t->comm, "system_server")
			|| !strcmp(t->comm, "surfaceflinger")
			|| !strcmp(t->comm, "servicemanager")
			|| !strcmp(t->comm, "hwservicemanage")
			|| !strncmp(t->comm, "composer", 8)) {
			return true;
		}
	}
	return false;
}

static bool is_key_process(struct task_struct *t)
{
	struct pid *pgrp;
	struct task_struct *taskp;

	if (t->pid == t->tgid) {
		if (is_system_process(t) || is_root_process(t)) {
			return true;
		}
	} else {
		pgrp = get_task_pid(t->group_leader, PIDTYPE_PID);
		if (pgrp != NULL) {
			taskp = pid_task(pgrp, PIDTYPE_PID);
			if (taskp != NULL && (is_system_process(taskp) || is_root_process(taskp))) {
				return true;
			}
		}
	}
	return false;
}

void send_signal_catcher(void *ignore, int sig, struct task_struct *src, struct task_struct *dst)
{
	if (sig == 33 && src->tgid == dst->tgid) {
		return; /*exclude system_server's "Signal Catcher" sending to itself*/
	} else if (sig == SIGQUIT && !strcmp(dst->comm, "system_server")) {
		return; /*exclude aee sending signal 3 to system_server*/
	} else if ((sig == 33 || sig == SIGQUIT || sig == SIGKILL || sig == SIGABRT || sig == SIGHUP
		|| sig == SIGSTOP || sig == SIGTERM || sig == SIGPIPE || sig == SIGCONT)
		&& is_key_process(dst)) {
		printk("<critical>Some other process %d:%s(ppid %d:%s) want to send sig:%d to pid:%d tgid:%d comm:%s\n",
			src->pid, src->comm, src->parent != NULL ? src->parent->pid : -1, src->parent != NULL ? src->parent->comm : "null",
			sig, dst->pid, dst->tgid, dst->comm);
	}
}

static int __init helper_init(void)
{
	int ret = register_trace_android_vh_do_send_sig_info(send_signal_catcher, NULL);
	if (ret != 0) {
		pr_err("register send_signal_catcher failed, ret=%d\n", ret);
		return ret;
	}
	return 0;
}

static void __exit helper_exit(void)
{
	unregister_trace_android_vh_do_send_sig_info(send_signal_catcher, NULL);
}

module_init(helper_init);
module_exit(helper_exit);

MODULE_DESCRIPTION("oplus stability helper");
MODULE_LICENSE("GPL v2");
