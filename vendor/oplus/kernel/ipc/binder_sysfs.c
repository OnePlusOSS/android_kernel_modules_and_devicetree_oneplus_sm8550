// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

#include "binder_sched.h"
#include "binder_sysfs.h"

#define OPLUS_BINDER_PROC_DIR	"oplus_binder"

static struct proc_dir_entry *d_oplus_binder;

enum {
	BINDER_OPT_STR_TYPE = 0,
	BINDER_OPT_STR_PID,
	BINDER_OPT_STR_VAL,
	BINDER_OPT_STR_MAX = 3,
};

static ssize_t proc_async_ux_enable_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[8];
	int err, val;

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	err = kstrtoint(strstrip(buffer), 10, &val);
	if (err)
		return err;

	g_async_ux_enable = val;

	return count;
}

static ssize_t proc_async_ux_enable_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[20];
	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "%d\n", g_async_ux_enable);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static ssize_t proc_set_last_async_ux_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[8];
	int err, val;

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	err = kstrtoint(strstrip(buffer), 10, &val);
	if (err)
		return err;

	g_set_last_async_ux = val;

	return count;
}

static ssize_t proc_set_last_async_ux_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[20];
	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "%d\n", g_set_last_async_ux);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static ssize_t proc_async_ux_flag_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[64];
	char *str, *token;
	char opt_str[BINDER_OPT_STR_MAX][8];
	int cnt = 0;
	int pid = 0;
	int ux_flag = 0;
	int err = 0;

	if (unlikely(!g_async_ux_enable)) {
		return count;
	}
	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	str = strstrip(buffer);
	while ((token = strsep(&str, " ")) && *token && (cnt < BINDER_OPT_STR_MAX)) {
		strlcpy(opt_str[cnt], token, sizeof(opt_str[cnt]));
		cnt += 1;
	}

	if (cnt != BINDER_OPT_STR_MAX) {
		if (cnt == (BINDER_OPT_STR_MAX - 1) && !strncmp(opt_str[BINDER_OPT_STR_TYPE], "r", 1)) {
			err = kstrtoint(strstrip(opt_str[BINDER_OPT_STR_PID]), 10, &pid);
			if (err)
				return err;

			return count;
		} else {
			return -EFAULT;
		}
	}

	err = kstrtoint(strstrip(opt_str[BINDER_OPT_STR_PID]), 10, &pid);
	if (err)
		return err;

	err = kstrtoint(strstrip(opt_str[BINDER_OPT_STR_VAL]), 10, &ux_flag);
	if (err)
		return err;

	if (!strncmp(opt_str[BINDER_OPT_STR_TYPE], "p", 1)) {
		set_task_async_ux_enable(pid, ux_flag);
	}

	return count;
}

static ssize_t proc_async_ux_flag_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[256];
	size_t len = 0;
	struct task_struct *task = current;

	len = snprintf(buffer, sizeof(buffer), "comm=%s pid=%d tgid=%d enable=%d\n",
		task->comm, task->pid, task->tgid, get_task_async_ux_enable(CURRENT_TASK_PID));

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static ssize_t proc_binder_all_tasks_async_ux_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[256];
	size_t len = 0;

	get_all_tasks_async_ux_enable();
	len = snprintf(buffer, sizeof(buffer), "pls check dmesg [async_ux_tasks]\n");

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops proc_async_ux_enable_fops = {
	.proc_write		= proc_async_ux_enable_write,
	.proc_read		= proc_async_ux_enable_read,
	.proc_lseek		= default_llseek,
};

static const struct proc_ops proc_set_last_async_ux_fops = {
	.proc_write		= proc_set_last_async_ux_write,
	.proc_read		= proc_set_last_async_ux_read,
	.proc_lseek		= default_llseek,
};

static const struct proc_ops proc_binder_async_ux_flag_fops = {
	.proc_write		= proc_async_ux_flag_write,
	.proc_read		= proc_async_ux_flag_read,
	.proc_lseek		= default_llseek,
};

static const struct proc_ops proc_binder_all_tasks_ux_sts_fops = {
	.proc_write		= NULL,
	.proc_read		= proc_binder_all_tasks_async_ux_read,
	.proc_lseek		= default_llseek,
};

int oplus_binder_sysfs_init(void)
{
	struct proc_dir_entry *proc_node;

	d_oplus_binder = proc_mkdir(OPLUS_BINDER_PROC_DIR, NULL);
	if (!d_oplus_binder) {
		pr_err("failed to create proc dir d_oplus_binder\n");
		goto err_create_d_oplus_binder;
	}

	proc_node = proc_create("async_ux_enable", 0666, d_oplus_binder, &proc_async_ux_enable_fops);
	if (!proc_node) {
		pr_err("failed to create proc node proc_async_ux_enable_fops\n");
		goto err_create_async_ux_enable;
	}

	proc_node = proc_create("set_last_async_ux", 0666, d_oplus_binder, &proc_set_last_async_ux_fops);
	if (!proc_node) {
		pr_err("failed to create proc node proc_set_last_async_ux_fops\n");
		goto err_create_unset_async_ux_time;
	}

	proc_node = proc_create("ux_flag", 0666, d_oplus_binder, &proc_binder_async_ux_flag_fops);
	if (!proc_node) {
		pr_err("failed to create proc node proc_unset_async_ux_time_fops\n");
		goto err_create_async_ux_flag;
	}

	proc_node = proc_create("all_tasks_ux_sts", 0444, d_oplus_binder, &proc_binder_all_tasks_ux_sts_fops);
	if (!proc_node) {
		pr_err("failed to create proc node proc_binder_all_tasks_ux_sts_fops\n");
		goto err_create_all_tasks_ux_sts;
	}

	pr_info("%s success\n", __func__);
	return 0;

err_create_all_tasks_ux_sts:
	remove_proc_entry("async_ux_flag", d_oplus_binder);

err_create_async_ux_flag:
	remove_proc_entry("unset_async_ux_time", d_oplus_binder);

err_create_unset_async_ux_time:
	remove_proc_entry("async_ux_enable", d_oplus_binder);

err_create_async_ux_enable:
	remove_proc_entry(OPLUS_BINDER_PROC_DIR, NULL);

err_create_d_oplus_binder:
	return -ENOENT;
}

void oplus_binder_sysfs_deinit(void)
{
	remove_proc_entry("unset_async_ux_time", d_oplus_binder);
	remove_proc_entry("async_ux_enable", d_oplus_binder);
	remove_proc_entry("sched_assist_enabled", d_oplus_binder);
	remove_proc_entry(OPLUS_BINDER_PROC_DIR, NULL);
}
