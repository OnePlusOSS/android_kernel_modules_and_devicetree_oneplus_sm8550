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

#include "sa_common.h"
#include "sa_sysfs.h"

#define OPLUS_SCHEDULER_PROC_DIR		"oplus_scheduler"
#define OPLUS_SCHEDASSIST_PROC_DIR		"sched_assist"

#define MAX_SET (128)
#define MAX_THREAD_INPUT (6)
#define TPD_HASH_BITS (6)

int global_debug_enabled;
EXPORT_SYMBOL(global_debug_enabled);
int global_sched_assist_enabled;
EXPORT_SYMBOL(global_sched_assist_enabled);
int global_sched_assist_scene;
EXPORT_SYMBOL(global_sched_assist_scene);

pid_t global_ux_task_pid = -1;
pid_t global_im_flag_pid = -1;

pid_t save_audio_tgid;
pid_t save_top_app_tgid;
unsigned int top_app_type;

struct proc_dir_entry *d_oplus_scheduler;
struct proc_dir_entry *d_sched_assist;

enum {
	OPT_STR_TYPE = 0,
	OPT_STR_PID,
	OPT_STR_VAL,
	OPT_STR_MAX = 3,
};

DECLARE_HASHTABLE(tpd_hash_table, TPD_HASH_BITS);
struct tpd_entry {
	int pid;
	char name[TASK_COMM_LEN];
	int tpd;
	struct hlist_node hash;
};

static ssize_t proc_debug_enabled_write(struct file *file, const char __user *buf,
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

	global_debug_enabled = val;

	return count;
}

static ssize_t proc_debug_enabled_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[20];
	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "debug_enabled=%d\n", global_debug_enabled);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static ssize_t proc_sched_assist_enabled_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[13];
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

	global_sched_assist_enabled = val;

	return count;
}

static ssize_t proc_sched_assist_enabled_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[13];
	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "enabled=%d\n", global_sched_assist_enabled);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static ssize_t proc_sched_assist_scene_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[13];
	int err, val;
	static DEFINE_MUTEX(sa_scene_mutex);

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	err = kstrtoint(strstrip(buffer), 10, &val);
	if (err)
		return err;

	mutex_lock(&sa_scene_mutex);

	if (val == SA_SCENE_OPT_CLEAR) {
		global_sched_assist_scene = val;
		goto out;
	}

	if (val & SA_SCENE_OPT_SET)
		global_sched_assist_scene |= val & (~SA_SCENE_OPT_SET);
	else if (val & global_sched_assist_scene)
		global_sched_assist_scene &= ~val;

out:
	mutex_unlock(&sa_scene_mutex);
	return count;
}

static ssize_t proc_sched_assist_scene_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[13];
	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "scene=%d\n", global_sched_assist_scene);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

/*
 * Example:
 * adb shell "echo "p 1611 130" > proc/oplus_scheduler/sched_assist/ux_task"
 * 'p' means pid, '1611' is thread pid, '130' means '128 + 2', set ux state as '2'
 *
 * adb shell "echo "r 1611" > proc/oplus_scheduler/sched_assist/ux_task"
 * "r" means we want to read thread "1611"'s info
 */
static ssize_t proc_ux_task_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[MAX_SET];
	char *str, *token;
	char opt_str[OPT_STR_MAX][8] = {"0", "0", "0"};
	int cnt = 0;
	int pid = 0;
	int ux_state = 0, ux_orig = 0;
	int err = 0;
	static DEFINE_MUTEX(sa_ux_mutex);

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	str = strstrip(buffer);
	while ((token = strsep(&str, " ")) && *token && (cnt < OPT_STR_MAX)) {
		strlcpy(opt_str[cnt], token, sizeof(opt_str[cnt]));
		cnt += 1;
	}

	if (cnt != OPT_STR_MAX) {
		if (cnt == (OPT_STR_MAX - 1) && !strncmp(opt_str[OPT_STR_TYPE], "r", 1)) {
			err = kstrtoint(strstrip(opt_str[OPT_STR_PID]), 10, &pid);
			if (err)
				return err;

			if (pid > 0 && pid <= PID_MAX_DEFAULT)
				global_ux_task_pid = pid;
		}

		return -EFAULT;
	}

	err = kstrtoint(strstrip(opt_str[OPT_STR_PID]), 10, &pid);
	if (err)
		return err;

	err = kstrtoint(strstrip(opt_str[OPT_STR_VAL]), 10, &ux_state);
	if (err)
		return err;

	mutex_lock(&sa_ux_mutex);
	if (!strncmp(opt_str[OPT_STR_TYPE], "p", 1) && (ux_state >= 0)) {
		struct task_struct *ux_task = NULL;

		if (pid > 0 && pid <= PID_MAX_DEFAULT) {
			rcu_read_lock();
			ux_task = find_task_by_vpid(pid);
			if (ux_task)
				get_task_struct(ux_task);
			rcu_read_unlock();

			if (ux_task) {
				ux_orig = oplus_get_ux_state(ux_task);

				if ((ux_state & SA_OPT_SET) && oplus_get_inherit_ux(ux_task)) {
					clear_all_inherit_type(ux_task);
					ux_orig = 0;
				}

				if (ux_state == SA_OPT_CLEAR) { /* clear all ux type but animator type */
					if (ux_orig & SA_TYPE_ANIMATOR)
						ux_orig &= SA_TYPE_ANIMATOR;
					else
						ux_orig = 0;

					if (oplus_set_ux_state(ux_task, ux_orig))
						count = -EFAULT;

				} else if (ux_state & SA_OPT_SET) { /* set target ux type and clear set opt */
					ux_orig |= ux_state & (~SA_OPT_SET);
					if (oplus_set_ux_state(ux_task, ux_orig))
						count = -EFAULT;

				} else if (ux_orig & ux_state) { /* reset target ux type */
					ux_orig &= ~ux_state;
					if (oplus_set_ux_state(ux_task, ux_orig))
						count = -EFAULT;
				}

				put_task_struct(ux_task);
			}
		}
	}

	mutex_unlock(&sa_ux_mutex);
	return count;
}

static ssize_t proc_ux_task_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[256];
	size_t len = 0;
	struct task_struct *task = NULL;

	task = find_task_by_vpid(global_ux_task_pid);
	if (task) {
		get_task_struct(task);
		len = snprintf(buffer, sizeof(buffer), "comm=%s pid=%d tgid=%d ux_state=%d inherit=%lld(bi:%d rw:%d mu:%d) im_flag=%d\n",
			task->comm, task->pid, task->tgid, oplus_get_ux_state(task), oplus_get_inherit_ux(task),
			test_inherit_ux(task, INHERIT_UX_BINDER), test_inherit_ux(task, INHERIT_UX_RWSEM), test_inherit_ux(task, INHERIT_UX_MUTEX),
			oplus_get_im_flag(task));
		put_task_struct(task);
	} else
		len = snprintf(buffer, sizeof(buffer), "Can not find task\n");

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops proc_debug_enabled_fops = {
	.proc_write		= proc_debug_enabled_write,
	.proc_read		= proc_debug_enabled_read,
	.proc_lseek		= default_llseek,
};

static int im_flag_set_handle(struct task_struct *task, int im_flag)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(task);

	if (IS_ERR_OR_NULL(ots))
		return 0;

	ots->im_flag = im_flag;

	switch (ots->im_flag) {
	case IM_FLAG_LAUNCHER_NON_UX_RENDER:
		ots->ux_state |= SA_TYPE_HEAVY;
		break;
	default:
		break;
	}

	return 0;
}

static ssize_t proc_im_flag_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[MAX_SET];
	char *str, *token;
	char opt_str[OPT_STR_MAX][8];
	int cnt = 0;
	int pid = 0;
	int im_flag = 0;
	int err = 0;
	static DEFINE_MUTEX(sa_im_mutex);

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	str = strstrip(buffer);
	while ((token = strsep(&str, " ")) && *token && (cnt < OPT_STR_MAX)) {
		strlcpy(opt_str[cnt], token, sizeof(opt_str[cnt]));
		cnt += 1;
	}

	if (cnt != OPT_STR_MAX) {
		if (cnt == (OPT_STR_MAX - 1) && !strncmp(opt_str[OPT_STR_TYPE], "r", 1)) {
			err = kstrtoint(strstrip(opt_str[OPT_STR_PID]), 10, &pid);
			if (err)
				return err;

			if (pid > 0 && pid <= PID_MAX_DEFAULT)
				global_im_flag_pid = pid;

			return count;
		} else {
			return -EFAULT;
		}
	}

	err = kstrtoint(strstrip(opt_str[OPT_STR_PID]), 10, &pid);
	if (err)
		return err;

	err = kstrtoint(strstrip(opt_str[OPT_STR_VAL]), 10, &im_flag);
	if (err)
		return err;

	mutex_lock(&sa_im_mutex);
	if (!strncmp(opt_str[OPT_STR_TYPE], "p", 1)) {
		struct task_struct *task = NULL;

		if (pid > 0 && pid <= PID_MAX_DEFAULT) {
			rcu_read_lock();
			task = find_task_by_vpid(pid);
			if (task) {
				get_task_struct(task);
				im_flag_set_handle(task, im_flag);
				put_task_struct(task);
			} else
				ux_debug("Can not find task with pid=%d", pid);
			rcu_read_unlock();
		}
	}

	mutex_unlock(&sa_im_mutex);
	return count;
}

static ssize_t proc_im_flag_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[256];
	size_t len = 0;
	struct task_struct *task = NULL;

	task = find_task_by_vpid(global_im_flag_pid);
	if (task) {
		get_task_struct(task);
		len = snprintf(buffer, sizeof(buffer), "comm=%s pid=%d tgid=%d im_flag=%d\n",
			task->comm, task->pid, task->tgid, oplus_get_im_flag(task));
		put_task_struct(task);
	} else
		len = snprintf(buffer, sizeof(buffer), "Can not find task\n");

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static ssize_t proc_sched_impt_task_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char temp_buf[32];
	char *temp_str, *token;
	char in_str[2][16];
	int cnt, err, pid;

	static DEFINE_MUTEX(impt_thd_mutex);

	mutex_lock(&impt_thd_mutex);

	memset(temp_buf, 0, sizeof(temp_buf));

	if (count > sizeof(temp_buf) - 1) {
		mutex_unlock(&impt_thd_mutex);
		return -EFAULT;
	}

	if (copy_from_user(temp_buf, buf, count)) {
		mutex_unlock(&impt_thd_mutex);
		return -EFAULT;
	}

	cnt = 0;
	temp_buf[count] = '\0';
	temp_str = strstrip(temp_buf);
	while ((token = strsep(&temp_str, " ")) && *token && (cnt < 2)) {
		strlcpy(in_str[cnt], token, sizeof(in_str[cnt]));
		cnt += 1;
	}

	if (cnt != 2) {
		mutex_unlock(&impt_thd_mutex);
		return -EFAULT;
	}

	err = kstrtoint(strstrip(in_str[1]), 10, &pid);
	if (err) {
		mutex_unlock(&impt_thd_mutex);
		return err;
	}

	if (pid < 0 || pid > PID_MAX_DEFAULT) {
		mutex_unlock(&impt_thd_mutex);
		return -EINVAL;
	}

	/* set top app */
	if (!strncmp(in_str[0], "fg", 2)) {
		save_top_app_tgid = pid;
		top_app_type = 0;
		if (!strncmp(in_str[0], "fgLauncher", 10))
			top_app_type = 1; /* 1 is launcher */
		goto out;
	}

	/* set audio app */
	if (!strncmp(in_str[0], "au", 2)) {
		save_audio_tgid = pid;
		goto out;
	}

out:
	mutex_unlock(&impt_thd_mutex);

	return count;
}

static ssize_t proc_sched_impt_task_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[32];
	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "top(%d %u) au(%d)\n", save_top_app_tgid, top_app_type, save_audio_tgid);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops proc_sched_assist_enabled_fops = {
	.proc_write		= proc_sched_assist_enabled_write,
	.proc_read		= proc_sched_assist_enabled_read,
	.proc_lseek		= default_llseek,
};

static const struct proc_ops proc_sched_assist_scene_fops = {
	.proc_write		= proc_sched_assist_scene_write,
	.proc_read		= proc_sched_assist_scene_read,
	.proc_lseek		= default_llseek,
};

static const struct proc_ops proc_ux_task_fops = {
	.proc_write		= proc_ux_task_write,
	.proc_read		= proc_ux_task_read,
	.proc_lseek		= default_llseek,
};

static const struct proc_ops proc_im_flag_fops = {
	.proc_write		= proc_im_flag_write,
	.proc_read		= proc_im_flag_read,
	.proc_lseek		= default_llseek,
};

static const struct proc_ops proc_sched_impt_task_fops = {
	.proc_write		= proc_sched_impt_task_write,
	.proc_read		= proc_sched_impt_task_read,
	.proc_lseek		= default_llseek,
};

static void tpd_add_record(struct task_struct *tsk, int decision)
{
	struct tpd_entry *tpd_e;

	tpd_e = kzalloc(sizeof(struct tpd_entry), GFP_ATOMIC);
	if (!tpd_e)
		return;

	tpd_e->pid = tsk->pid;
	strlcpy(tpd_e->name, tsk->comm, TASK_COMM_LEN);
	tpd_e->tpd = decision;
	hash_add(tpd_hash_table, &tpd_e->hash, tsk->pid);
}

static void tpd_remove_record(struct task_struct *tsk)
{
	struct tpd_entry *tpd_entry;
	unsigned long bkt_task;
	struct hlist_node *tmp_task;

	hash_for_each_safe(tpd_hash_table, bkt_task,
			tmp_task, tpd_entry, hash) {
		if (tpd_entry->pid == tsk->pid && !strncmp(tpd_entry->name, tsk->comm, TASK_COMM_LEN)) {
			hash_del(&tpd_entry->hash);
			kfree(tpd_entry);
		}
	}
}

static inline void tagging(struct task_struct *tsk, int decision)
{
	if (!tsk) {
		ux_err("task cannot set");
		return;
	}

	oplus_set_tpd(tsk, decision);
	if (decision)
		tpd_add_record(tsk, decision);
	else
		tpd_remove_record(tsk);
	ux_debug("task: %s pid:%d decision:%d\n", tsk->comm, tsk->pid, decision);
}

static inline void tagging_by_name(struct task_struct *tsk, char *name, int decision, int *cnt)
{
	size_t tlen = 0, len = 0;

	tlen = strlen(name);
	if (tlen == 0)
		return;

	len = strlen(tsk->comm);

	if (len != tlen)
		return;

	if (!strncmp(tsk->comm, name, tlen)) {
		ux_debug("task: %s pid:%d decision:%d name=%s\n", tsk->comm, tsk->pid, decision, name);
		oplus_set_tpd(tsk, decision);
		if (decision)
			tpd_add_record(tsk, decision);
		else
			tpd_remove_record(tsk);
		*cnt = *cnt + 1;
	}
}

static void tag_from_tid(unsigned int pid, unsigned int tid, int decision)
{
	struct task_struct *p;

	rcu_read_lock();
	p = find_task_by_vpid(tid);
	if (p) {
		get_task_struct(p);
		if (p->group_leader && (p->group_leader->pid == pid)) {
			ux_debug("tpd tagging task pid= %d\n", pid);
			tagging(p, decision);
		}
		put_task_struct(p);
	} else
		ux_err("Can not find task!!! pid = %d", tid);

	rcu_read_unlock();
}

static void tag_from_tgid(unsigned int tgid, int decision, char *thread_name, int *cnt)
{
	struct task_struct *p, *t;

	rcu_read_lock();
	p = find_task_by_vpid(tgid);
	if (p) {
		get_task_struct(p);
		for_each_thread(p, t)
			tagging_by_name(t, thread_name, decision, cnt);
		put_task_struct(p);
	}
	rcu_read_unlock();
}

static ssize_t proc_tpd_set_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	unsigned int pid = 0;
	unsigned int tid = 0;
	int tpdenable = 0;
	int tp_decision = -1;
	int ret;
	char buffer[18];

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	ret = sscanf(buffer, "%u,%u,%d,%d\n",
		&pid, &tid, &tpdenable, &tp_decision);

	ux_debug("tpd param pid:%u tid:%u, tpd_enable:%d decision:%d from %s %d\n",
		pid, tid, tpdenable, tp_decision, current->comm, current->pid);

	if (ret != 4) {
		ux_err("Invalid params!!!!!!");
		return -EFAULT;
	}

	tag_from_tid(pid, tid, tpdenable ? tp_decision : 0);

	return count;
}

static ssize_t proc_tpd_cmds_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	unsigned int tgid = 0;
	int tp_decision = -1;
	char threads[MAX_THREAD_INPUT][TASK_COMM_LEN] = {{0}, {0}, {0}, {0}, {0}, {0}};
	int ret, i, cnt = 0;
	char buffer[128];

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	ret = sscanf(buffer, "%u %d %s %s %s %s %s %s\n",
		&tgid, &tp_decision,
		threads[0], threads[1], threads[2], threads[3], threads[4], threads[5]);

	ux_debug("tpd params: %u %d %s %s %s %s %s %s, from %s %d, total=%d\n",
		tgid, tp_decision, threads[0], threads[1], threads[2], threads[3], threads[4], threads[5],
		current->comm, current->pid, ret);

	for (i = 0; i < MAX_THREAD_INPUT; i++) {
		if (strlen(threads[i]) > 0)
			tag_from_tgid(tgid, tp_decision, threads[i], &cnt);
	}

	ux_debug("tpd tagging count = %d\n", cnt);

	return count;
}

static int tpd_hash_show(struct seq_file *m, void *v)
{
	struct tpd_entry *tpd_e;
	unsigned long bkt;

	hash_for_each(tpd_hash_table, bkt, tpd_e, hash)
		seq_printf(m, "pid=%d,name=%s,tpd=%d\n", tpd_e->pid, tpd_e->name, tpd_e->tpd);

	return 0;
}

static int tpd_ignore_show(struct seq_file *seq_filp, void *v)
{
	seq_printf(seq_filp, "%s\n", "Invalid Operation");
	return 0;
}

static int tpd_open(struct inode *inode, struct file *file)
{
	int ret;

	ret = single_open(file, tpd_ignore_show, NULL);
	return ret;
}

static const struct proc_ops proc_tpd_set_fops = {
	.proc_open		= tpd_open,
	.proc_write		= proc_tpd_set_write,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release		= single_release,
};

static const struct proc_ops proc_tpd_cmds_fops = {
	.proc_open		= tpd_open,
	.proc_write		= proc_tpd_cmds_write,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release		= single_release,
};

static int proc_tpd_task_read(struct inode *inode, struct file *file)
{
	return single_open(file, tpd_hash_show, NULL);
}

static const struct proc_ops proc_tpd_task_fops = {
	.proc_open		= proc_tpd_task_read,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release		= single_release,
};

int oplus_sched_assist_proc_init(void)
{
	struct proc_dir_entry *proc_node;

	d_oplus_scheduler = proc_mkdir(OPLUS_SCHEDULER_PROC_DIR, NULL);
	if (!d_oplus_scheduler) {
		ux_err("failed to create proc dir oplus_scheduler\n");
		goto err_creat_d_oplus_scheduler;
	}

	d_sched_assist = proc_mkdir(OPLUS_SCHEDASSIST_PROC_DIR, d_oplus_scheduler);
	if (!d_sched_assist) {
		ux_err("failed to create proc dir sched_assist\n");
		goto err_creat_d_sched_assist;
	}

	proc_node = proc_create("debug_enabled", 0666, d_sched_assist, &proc_debug_enabled_fops);
	if (!proc_node) {
		ux_err("failed to create proc node debug_enabled\n");
		goto err_creat_debug_enabled;
	}

	proc_node = proc_create("sched_assist_enabled", 0666, d_sched_assist, &proc_sched_assist_enabled_fops);
	if (!proc_node) {
		ux_err("failed to create proc node sched_assist_enabled\n");
		goto err_creat_sched_assist_enabled;
	}

	proc_node = proc_create("sched_assist_scene", 0666, d_sched_assist, &proc_sched_assist_scene_fops);
	if (!proc_node) {
		ux_err("failed to create proc node sched_assist_scene\n");
		goto err_creat_sched_assist_scene;
	}

	proc_node = proc_create("ux_task", 0666, d_sched_assist, &proc_ux_task_fops);
	if (!proc_node) {
		ux_err("failed to create proc node ux_task\n");
		goto err_creat_ux_task;
	}

	proc_node = proc_create("im_flag", 0666, d_sched_assist, &proc_im_flag_fops);
	if (!proc_node) {
		ux_err("failed to create proc node im_flag\n");
		remove_proc_entry("im_flag", d_sched_assist);
	}

	proc_node = proc_create("sched_impt_task", 0666, d_sched_assist, &proc_sched_impt_task_fops);
	if (!proc_node) {
		ux_err("failed to create proc node sched_impt_task\n");
		remove_proc_entry("sched_impt_task", d_sched_assist);
	}

	/* proc of tpd feature */
	proc_node = proc_create("tpd_id", 0666, d_sched_assist, &proc_tpd_set_fops);
	if (!proc_node) {
		ux_err("failed to create proc node tpd_id\n");
		remove_proc_entry("tpd_id", d_sched_assist);
	}

	proc_node = proc_create("tpd_cmds", 0666, d_sched_assist, &proc_tpd_cmds_fops);
	if (!proc_node) {
		ux_err("failed to create proc node tpd_cmds\n");
		remove_proc_entry("tpd_cmds", d_sched_assist);
	}

	proc_node = proc_create_data("tpd_task", 0666, d_sched_assist, &proc_tpd_task_fops, NULL);
	if (!proc_node) {
		ux_err("failed to create proc node tpd_task\n");
		remove_proc_entry("tpd_task", d_sched_assist);
	}

	return 0;

err_creat_ux_task:
	remove_proc_entry("sched_assist_scene", d_sched_assist);

err_creat_sched_assist_scene:
	remove_proc_entry("sched_assist_enabled", d_sched_assist);

err_creat_sched_assist_enabled:
	remove_proc_entry("debug_enabled", d_sched_assist);

err_creat_debug_enabled:
	remove_proc_entry(OPLUS_SCHEDASSIST_PROC_DIR, d_oplus_scheduler);

err_creat_d_sched_assist:
	remove_proc_entry(OPLUS_SCHEDULER_PROC_DIR, NULL);

err_creat_d_oplus_scheduler:
	return -ENOENT;
}

void oplus_sched_assist_proc_deinit(void)
{
	/* proc of tpd feature */
	remove_proc_entry("tpd_task", d_sched_assist);
	remove_proc_entry("tpd_cmds", d_sched_assist);
	remove_proc_entry("tpd_id", d_sched_assist);

	remove_proc_entry("ux_task", d_sched_assist);
	remove_proc_entry("sched_assist_scene", d_sched_assist);
	remove_proc_entry("sched_assist_enabled", d_sched_assist);
	remove_proc_entry(OPLUS_SCHEDASSIST_PROC_DIR, d_oplus_scheduler);
	remove_proc_entry(OPLUS_SCHEDULER_PROC_DIR, NULL);
}

