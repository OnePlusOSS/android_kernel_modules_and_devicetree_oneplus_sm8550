// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#include <linux/fdtable.h>
#include <linux/file.h>
#include <linux/freezer.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nsproxy.h>
#include <linux/poll.h>
#include <linux/debugfs.h>
#include <linux/rbtree.h>
#include <linux/sched/signal.h>
#include <linux/sched/mm.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/pid_namespace.h>
#include <linux/security.h>
#include <linux/spinlock.h>
#include <linux/ratelimit.h>
#include <linux/syscalls.h>
#include <linux/sched/clock.h>
#include <linux/task_work.h>
#include <linux/android_vendor.h>
#include <trace/hooks/binder.h>

#include <uapi/linux/sched/types.h>
#include <uapi/linux/android/binder.h>

#include <asm/cacheflush.h>
#include <drivers/android/binder_alloc.h>
#include <drivers/android/binder_internal.h>

#include "trans_ctrl.h"

struct ob_struct ob_target;
pid_t ob_pid;

struct binder_proc_status system_server_proc_status;
struct mutex *binder_procs_lock = NULL;
struct hlist_head *binder_procs = NULL;
static struct work_struct obs_ws;

unsigned int g_control_enable = 1;
unsigned int g_status_record = 1;
int sysctl_debug_obs;

void obtarget_init(struct binder_proc *proc);
void work_for_init_target(struct work_struct *work)
{
	struct binder_proc *target_proc;

	if (binder_procs_lock == NULL || binder_procs == NULL)
		return;

	if (ob_target.init)
		return;

	mutex_lock(binder_procs_lock);
	hlist_for_each_entry(target_proc, binder_procs, proc_node) {
		if (ob_target.init) {
			mutex_unlock(binder_procs_lock);
			return;
		}
		obtarget_init(target_proc);
	}
	mutex_unlock(binder_procs_lock);
}

void obs_work_init(void)
{
	INIT_WORK(&obs_ws, work_for_init_target);
}

noinline void ob_tracing_mark_write(const char *buf)
{
	trace_printk(buf);
}

void ob_systrace_begin(struct binder_proc *proc)
{
	char buf[256];

	if (!sysctl_debug_obs)
		return;

	snprintf(buf, sizeof(buf), "B|%d|obs_limit", proc->pid);
	ob_tracing_mark_write(buf);
}

void ob_systrace_end(void)
{
	if (!sysctl_debug_obs)
		return;

	ob_tracing_mark_write("E\n");
}

int get_task_cgroup_id(struct task_struct *task)
{
	struct cgroup_subsys_state *css = task_css(task, cpu_cgrp_id);

	return css ? css->id : -1;
}

bool test_task_bg(struct task_struct *task)
{
	return (BT_CGROUP_BACKGROUND == get_task_cgroup_id(task)) ? 1 : 0;
}

bool obtrans_is_from_background(struct binder_transaction *t)
{
	return test_task_bg(t->from->task);
}

bool obtrans_is_from_third_party(struct binder_transaction *t)
{
	return (from_kuid(current_user_ns(), t->sender_euid) % 100000) >= 10000;
}

bool obtrans_is_from_main(struct binder_transaction *t)
{
	return t->from->proc->pid == t->from->pid;
}

bool obwork_is_ux_boost(struct binder_transaction *t)
{
	if ((!t->from) || (!t->from->proc))
		return false;
	else
		return true;
}

bool obwork_is_async_boost(struct binder_transaction *t)
{
	if (!g_control_enable)
		return false;

	if (t->flags & TF_ASYNC_BOOST)
		return true;

	return false;
}

bool obwork_is_restrict(struct binder_transaction *t)
{
	if ((!t->from) || (!t->from->proc))
		return false;

	return obtrans_is_from_background(t) && obtrans_is_from_third_party(t)
	    && !obtrans_is_from_main(t);
}

/*
static void
binder_dequeue_work_ilocked(struct binder_work *work)
{
	list_del_init(&work->entry);
}
*/

static bool binder_worklist_empty_ilocked(struct list_head *list)
{
	return list_empty(list);
}

static void binder_dequeue_work_ilocked(struct binder_work *work)
{
	list_del_init(&work->entry);
}

static void
binder_enqueue_work_ilocked(struct binder_work *work,
			    struct list_head *target_list)
{
	BUG_ON(target_list == NULL);
	BUG_ON(work->entry.next && !list_empty(&work->entry));
	list_add_tail(&work->entry, target_list);
}

/**
 * binder_inner_proc_lock() - Acquire inner lock for given binder_proc
 * @proc:         struct binder_proc to acquire
 *
 * Acquires proc->inner_lock. Used to protect todo lists
 */
#define binder_inner_proc_lock(proc) _binder_inner_proc_lock(proc, __LINE__)
static void
_binder_inner_proc_lock(struct binder_proc *proc, int line)
__acquires(&proc->inner_lock)
{
	spin_lock(&proc->inner_lock);
}

/**
 * binder_inner_proc_unlock() - Release inner lock for given binder_proc
 * @proc:         struct binder_proc to acquire
 *
 * Release lock acquired via binder_inner_proc_lock()
 */
#define binder_inner_proc_unlock(proc) _binder_inner_proc_unlock(proc, __LINE__)
static void
_binder_inner_proc_unlock(struct binder_proc *proc, int line)
__releases(&proc->inner_lock)
{
	spin_unlock(&proc->inner_lock);
}

/**
 * binder_select_thread_ilocked() - selects a thread for doing proc work.
 * @proc:	process to select a thread from
 *
 * Note that calling this function moves the thread off the waiting_threads
 * list, so it can only be woken up by the caller of this function, or a
 * signal. Therefore, callers *should* always wake up the thread this function
 * returns.
 *
 * Return:	If there's a thread currently waiting for process work,
 *		returns that thread. Otherwise returns NULL.
 */
static struct binder_thread *binder_select_thread_ilocked(struct binder_proc *proc)
{
	struct binder_thread *thread;

	assert_spin_locked(&proc->inner_lock);
	thread = list_first_entry_or_null(&proc->waiting_threads,
					  struct binder_thread,
					  waiting_thread_node);

	if (thread)
		list_del_init(&thread->waiting_thread_node);

	return thread;
}

/*
 * Try to boost async which need large allocated buffer by list_add.
*/
static struct binder_buffer *binder_buffer_next(struct binder_buffer *buffer)
{
	return list_entry(buffer->entry.next, struct binder_buffer, entry);
}

static size_t binder_alloc_buffer_size(struct binder_alloc *alloc,
				       struct binder_buffer *buffer)
{
	if (list_is_last(&buffer->entry, &alloc->buffers))
		return alloc->buffer + alloc->buffer_size - buffer->user_data;

	return binder_buffer_next(buffer)->user_data - buffer->user_data;
}

static inline struct oplus_binder_transaction *get_oplus_binder_transaction_struct(struct binder_transaction *t)
{
	return (struct oplus_binder_transaction *)t->android_oem_data1;
}

struct binder_thread *obthread_get(struct binder_proc *proc,
				   struct binder_transaction *t, bool oneway)
{
	struct binder_thread *thread = NULL;

	if (g_control_enable && (proc == ob_target.ob_proc)
	    && obwork_is_restrict(t)) {
		list_for_each_entry(thread, &proc->waiting_threads,
				    waiting_thread_node)
		    if (thread
			&& (thread->looper & BINDER_LOOPER_STATE_BACKGROUND)) {
				list_del_init(&thread->waiting_thread_node);
				return thread;
		}
		return NULL;
	}
	return binder_select_thread_ilocked(proc);
}

/*
 * Check amount and size of buffers allocated for
 * async and sync binder transactions.
 * Warn if free_async_space low has been used lower than 1/5 * (alloc->buffer_size * 1/2).
 * Warn if total_alloc_size_unoneway has been larger than 9 / 10 * alloc->buffer_size.
 */
void obinder_low_mem_check(struct binder_alloc *alloc)
{
	struct rb_node *n = alloc->free_buffers.rb_node;
	struct binder_buffer *buffer;
	size_t total_alloc_size_oneway = 0;
	size_t total_alloc_size_unoneway = 0;
	size_t oneway_buffers = 0;
	size_t unoneway_buffers = 0;

	if (alloc->pid != ob_target.pid)
		return;

	if (!g_status_record)
		return;

	mutex_lock(&alloc->mutex);
	for (n = rb_first(&alloc->allocated_buffers); n != NULL; n = rb_next(n)) {
		buffer = rb_entry(n, struct binder_buffer, rb_node);
		if (buffer->async_transaction) {
			total_alloc_size_oneway +=
			    binder_alloc_buffer_size(alloc, buffer)
			    + sizeof(struct binder_buffer);
			oneway_buffers++;
		} else {
			total_alloc_size_unoneway +=
			    binder_alloc_buffer_size(alloc, buffer)
			    + sizeof(struct binder_buffer);
			unoneway_buffers++;
		}
	}

	if (alloc->free_async_space < alloc->buffer_size / 10) {
		system_server_proc_status.async_mem_over_low++;
		if (alloc->free_async_space < alloc->buffer_size / 20)
			system_server_proc_status.async_mem_over_high++;
	}

	if (total_alloc_size_unoneway > alloc->buffer_size * 70 / 100) {
		system_server_proc_status.sync_mem_over_low++;
		if (total_alloc_size_unoneway > alloc->buffer_size * 90 / 100)
			system_server_proc_status.sync_mem_over_high++;
	}
	mutex_unlock(&alloc->mutex);
	return;
}

void obwork_check_restrict_off(struct binder_proc *proc)
{
	struct binder_work *w = NULL;
	struct binder_work *tmp = NULL;
	struct binder_transaction *t = NULL;
	struct oplus_binder_transaction *obt = NULL;
	u64 now = sched_clock();

	if (proc != ob_target.ob_proc
	    || binder_worklist_empty_ilocked(&ob_target.ob_list)
	    || (now - ob_target.ob_check_ts) < OBPROC_CHECK_CYCLE_NS)
		return;

	list_for_each_entry_safe(w, tmp, &ob_target.ob_list, entry) {
		if (!w)
			continue;

		t = container_of(w, struct binder_transaction, work);
		obt = get_oplus_binder_transaction_struct(t);
		if ((now - obt->ob_begin < OBWORK_TIMEOUT_NS))
			break;

		list_del_init(&w->entry);
		binder_enqueue_work_ilocked(w, &proc->todo);
	}

	ob_target.ob_check_ts = sched_clock();
}

void obtrans_restrict_start(struct binder_proc *proc,
			    struct binder_transaction *t)
{
	struct oplus_binder_transaction *obt = NULL;

	if (!g_control_enable)
		return;

	if (obwork_is_restrict(t)) {
		obt = get_oplus_binder_transaction_struct(t);
		obt->ob_begin = sched_clock();
	}

	obwork_check_restrict_off(proc);
}

void oblist_dequeue_all(void)
{
	struct binder_work *w;
	struct binder_work *w_tmp;

	if (ob_target.ob_proc == NULL)
		return;

	binder_inner_proc_lock(ob_target.ob_proc);

	if (binder_worklist_empty_ilocked(&ob_target.ob_list)) {
		binder_inner_proc_unlock(ob_target.ob_proc);
		return;
	}

	pr_info("dequeue all");

	list_for_each_entry_safe(w, w_tmp, &ob_target.ob_list, entry) {
		if (!w)
			continue;
		list_del_init(&w->entry);
		binder_enqueue_work_ilocked(w, &ob_target.ob_proc->todo);
	}

	binder_inner_proc_unlock(ob_target.ob_proc);
	return;
}

/*
 * When top app changed, we need to check if it's binder had been pushed to ob list before
 */
void oblist_dequeue_topapp_change(uid_t topuid)
{
	struct binder_work *w;
	struct binder_work *w_tmp;
	struct binder_transaction *t;

	if (ob_target.ob_proc == NULL)
		return;

	binder_inner_proc_lock(ob_target.ob_proc);
	if (!binder_worklist_empty_ilocked(&ob_target.ob_list)) {
		list_for_each_entry_safe(w, w_tmp, &ob_target.ob_list, entry) {
			if (!w)
				continue;
			t = container_of(w, struct binder_transaction, work);
			if (from_kuid(current_user_ns(), t->sender_euid) !=
			    topuid)
				continue;
			if (sysctl_debug_obs)
				pr_info("binder from %d released",
					t->from->task->pid);
			list_del_init(&w->entry);
			binder_enqueue_work_ilocked(w,
						    &ob_target.ob_proc->todo);
		}
	}
	binder_inner_proc_unlock(ob_target.ob_proc);
	return;
}

EXPORT_SYMBOL_GPL(oblist_dequeue_topapp_change);

void obinder_enqueue_work_head_ilocked(struct binder_work *work,
				       struct list_head *target_list)
{
	BUG_ON(target_list == NULL);
	BUG_ON(work->entry.next && !list_empty(&work->entry));
	list_add(&work->entry, target_list);
}

void obbinder_thread_check_status(struct binder_proc *proc)
{
	struct binder_thread *thread;
	struct rb_node *n;
	struct binder_transaction *tmp;
	int doing_bg = 0;
	int doing_fg = 0;

	if (!g_status_record)
		return;

	if (proc != ob_target.ob_proc)
		return;

	for (n = rb_first(&proc->threads); n != NULL; n = rb_next(n)) {
		thread = rb_entry(n, struct binder_thread, rb_node);
		if (thread && thread->looper & BINDER_LOOPER_STATE_REGISTERED) {
			tmp = thread->transaction_stack;
			if (tmp && !(tmp->flags & TF_ONE_WAY)) {
				spin_lock(&tmp->lock);
				if (!obwork_is_restrict(tmp))
					doing_fg++;
				else {
					doing_bg++;
				}
				spin_unlock(&tmp->lock);
			}
		}
	}

	system_server_proc_status.warning++;
	if (doing_bg > doing_fg)
		system_server_proc_status.warning_cg_bg++;
}

void obtarget_init(struct binder_proc *proc)
{
	if (!proc->tsk || !proc->context || !proc->context->name)
		return;

	if (ob_target.init)
		return;

	if ((!strncmp(proc->tsk->comm, "system_server", TASK_COMM_LEN))
	    && !strcmp(proc->context->name, "binder")) {
		ob_target.ob_proc = proc;
		ob_target.ob_check_ts = sched_clock();
		INIT_LIST_HEAD(&ob_target.ob_list);
		ob_target.pid = proc->pid;
		ob_target.init = true;
		pr_info("obsproc init");
	}
}

void obwork_restrict(struct binder_proc *proc, struct binder_transaction *t,
		     bool pending_async)
{
	struct binder_node *node = t->buffer->target_node;

	if (!pending_async) {
		obbinder_thread_check_status(proc);
		if (!g_control_enable || (proc != ob_target.ob_proc)
		    || !obwork_is_restrict(t)) {
			binder_enqueue_work_ilocked(&t->work, &proc->todo);
		} else {
			ob_systrace_begin(proc);
			binder_enqueue_work_ilocked(&t->work,
						    &ob_target.ob_list);
			ob_systrace_end();
		}
	} else {
		if (!g_control_enable || (proc != ob_target.ob_proc))
			binder_enqueue_work_ilocked(&t->work,
						    &node->async_todo);
		else if (obwork_is_async_boost(t))
			obinder_enqueue_work_head_ilocked(&t->work,
							  &node->async_todo);
		else
			binder_enqueue_work_ilocked(&t->work,
						    &node->async_todo);
	}
}

void obthread_init(struct binder_proc *proc, struct binder_thread *thread)
{
	if (proc != ob_target.ob_proc)
		return;

	if (proc->requested_threads_started == BG_THREAD) {
		thread->looper |= BINDER_LOOPER_STATE_BACKGROUND;
		ob_pid = thread->task->pid;
		pr_info("obthread init!! %d", ob_pid);
	}
}

bool obthread_has_work(struct binder_thread *thread)
{
	if (!g_control_enable
	    || !(thread->looper & BINDER_LOOPER_STATE_BACKGROUND)
	    || (thread->proc != ob_target.ob_proc))
		return false;

	if (!binder_worklist_empty_ilocked(&ob_target.ob_list))
		return true;

	return false;
}

void obproc_has_work(struct binder_proc *proc)
{
	struct binder_thread *thread = NULL;

	binder_inner_proc_lock(proc);
	if (!binder_worklist_empty_ilocked(&ob_target.ob_list)) {
		list_for_each_entry(thread, &proc->waiting_threads,
				    waiting_thread_node) {
			if (thread
			    && (thread->
				looper & BINDER_LOOPER_STATE_BACKGROUND)) {
				list_del_init(&thread->waiting_thread_node);
				wake_up_interruptible(&thread->wait);
			}
		}
	}
	binder_inner_proc_unlock(proc);
	return;
}

void obproc_free(struct binder_proc *proc)
{
	if (proc == ob_target.ob_proc) {
		BUG_ON(!list_empty(&ob_target.ob_list));
		ob_target.ob_check_ts = 0;
		ob_target.ob_proc = NULL;
	}
}

/* hook registered implemention in drivers/android/binder.c*/
static void android_vh_binder_looper_state_registered_handler(void *unused,
						struct binder_thread *thread, struct binder_proc *proc)
{
	obthread_init(proc, thread);
}

static void android_vh_binder_free_proc_handler(void *unused,
						struct binder_proc *proc)
{
	obproc_free(proc);
}

static void android_vh_binder_thread_release_handler(void *unused,
						     struct binder_proc *proc,
						     struct binder_thread
						     *thread)
{
	if (proc == ob_target.ob_proc && thread
	    && (thread->looper & BINDER_LOOPER_STATE_BACKGROUND))
		oblist_dequeue_all();
}

static void android_vh_binder_has_work_ilocked_handler(void *unused,
						       struct binder_thread *thread, bool do_proc_work, int *ret)
{
	if (thread->process_todo || thread->looper_need_return ||
	    (do_proc_work && obthread_has_work(thread)) ||
	    (do_proc_work
	     && !binder_worklist_empty_ilocked(&thread->proc->todo)))
		*ret = 1;
}

static void android_vh_binder_read_done_handler(void *unused,
						struct binder_proc *proc, struct binder_thread *thread)
{
	obproc_has_work(proc);
}

static void android_vh_binder_restore_priority_handler(void *unused,
						struct binder_transaction *t, struct task_struct *task)
{
	if (!t || !t->to_proc)
		return;

	binder_inner_proc_lock(t->to_proc);
	obwork_check_restrict_off(t->to_proc);
	binder_inner_proc_unlock(t->to_proc);
}

static void android_vh_binder_preset_handler(void *unused,
					     struct hlist_head *hhead,
					     struct mutex *lock)
{
	if (binder_procs_lock == NULL)
		binder_procs_lock = lock;

	if (binder_procs == NULL)
		binder_procs = hhead;

	if (ob_target.init)
		return;

	pr_info("try to queue work");
	schedule_work(&obs_ws);
}

static void android_vh_binder_proc_transaction_entry_handler(void *unused,
						struct binder_proc *proc, struct binder_transaction *t,
						struct binder_thread **binder_th, int node_debug_id, bool pending_async, bool sync, bool *skip)
{
	if (!g_control_enable)
		return;

	obtrans_restrict_start(proc, t);
	if (!pending_async && *binder_th == NULL) {
		*binder_th = obthread_get(proc, t, !sync);
		*skip = true;
	}
}

static void android_vh_binder_proc_transaction_finish_handler(void *unused,
								struct binder_proc *proc, struct binder_transaction *t, struct task_struct *binder_th_task,
								bool pending_async, bool sync)
{
	if (!g_control_enable)
		return;

	if (binder_th_task)
		return;

	if (pending_async)
		return;

	binder_dequeue_work_ilocked(&t->work);
	obwork_restrict(proc, t, pending_async);
}

static void android_vh_binder_select_worklist_ilocked_handler(void *unused,
								struct list_head **list, struct binder_thread *thread, struct binder_proc *proc,
								int wait_for_proc_work)
{
	if (!g_control_enable)
		return;

	if (!binder_worklist_empty_ilocked(&thread->todo)) {
		*list = &thread->todo;
	} else if (obthread_has_work(thread) && wait_for_proc_work) {
		*list = &ob_target.ob_list;
		if (sysctl_debug_obs)
			pr_info("ob thread:%d working", thread->task->pid);
	} else if (!binder_worklist_empty_ilocked(&proc->todo) &&
		wait_for_proc_work) {
		*list = &proc->todo;
	}
}

#define OPLUS_OEM_DATA_SIZE_TEST(ostruct, kstruct)		\
	BUILD_BUG_ON(sizeof(ostruct) > (sizeof(u64) *		\
		ARRAY_SIZE(((kstruct *)0)->android_oem_data1)))

#define OPLUS_BINDER_PROC_DIR "oplus_binder"
struct proc_dir_entry *oplus_binder_dir;

static ssize_t proc_obs_enabled_read(struct file *file, char __user *buf,
				     size_t count, loff_t *ppos)
{
	char buffer[13];
	size_t len = 0;

	len =
	    snprintf(buffer, sizeof(buffer), "obs enable=%u\n",
		     g_control_enable);
	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static ssize_t proc_obs_enabled_write(struct file *file,
				      const char __user *buf, size_t count,
				      loff_t *ppos)
{
	char buffer[13];
	int err, val;

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	err = kstrtoint(strstrip(buffer), 10, &val);
	if (err)
		return err;

	g_control_enable = val;
	if (!g_control_enable)
		oblist_dequeue_all();

	return count;
}

static const struct proc_ops proc_obs_enabled_fops = {
	.proc_write = proc_obs_enabled_write,
	.proc_read = proc_obs_enabled_read,
	.proc_lseek = default_llseek,
};

static ssize_t proc_obs_debug_read(struct file *file, char __user *buf,
				   size_t count, loff_t *ppos)
{
	char buffer[32];
	size_t len = 0;

	len =
	    snprintf(buffer, sizeof(buffer), "obs debug=%d\n",
		     sysctl_debug_obs);
	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static ssize_t proc_obs_debug_write(struct file *file, const char __user *buf,
				    size_t count, loff_t *ppos)
{
	char buffer[13];
	int err, val;

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	err = kstrtoint(strstrip(buffer), 10, &val);
	if (err)
		return err;

	sysctl_debug_obs = val;
	return count;
}

static const struct proc_ops proc_obs_debug_fops = {
	.proc_write = proc_obs_debug_write,
	.proc_read = proc_obs_debug_read,
	.proc_lseek = default_llseek,
};

static ssize_t proc_obs_bg_thread_read(struct file *file, char __user *buf,
				       size_t count, loff_t *ppos)
{
	char buffer[32];
	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "obs bg_thread pid=%d\n", ob_pid);
	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops proc_obs_bg_thread_fops = {
	.proc_read = proc_obs_bg_thread_read,
	.proc_lseek = default_llseek,
};

static ssize_t ss_binder_proc_read(struct file *filp, char __user *buff,
				   size_t count, loff_t *off)
{
	char page[1024] = { 0 };
	int len = 0;

	len = sprintf(page,
		    "async_mem_over_high: %llu\nasync_mem_over_low: %llu\nsync_mem_over_high: %llu\nsync_mem_over_low: %llu\nwarning_cg_bg: %llu\nwarning: %llu\n",
		    system_server_proc_status.async_mem_over_high,
		    system_server_proc_status.async_mem_over_low,
		    system_server_proc_status.sync_mem_over_high,
		    system_server_proc_status.sync_mem_over_low,
		    system_server_proc_status.warning_cg_bg,
		    system_server_proc_status.warning);

	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}

	if (copy_to_user(buff, page, (len < count ? len : count))) {
		return -EFAULT;
	}
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

static const struct proc_ops proc_ss_fops = {
	.proc_read = ss_binder_proc_read,
	.proc_lseek = default_llseek,
};

static int oplus_binder_proc_init(void)
{
	struct proc_dir_entry *proc_node;
	int ret = 0;

	oplus_binder_dir = proc_mkdir(OPLUS_BINDER_PROC_DIR, NULL);
	if (!oplus_binder_dir) {
		pr_err("failed to create proc dir oplus_binder_dir.\n");
		goto err_create_oplus_binder_proc;
	}

	proc_node =
	    proc_create("obs_enabled", 0660, oplus_binder_dir,
			&proc_obs_enabled_fops);
	if (!proc_node) {
		pr_err("failed to create proc node obs_enabled.\n");
		goto err_create_oplus_binder_proc;
	}

	proc_node =
	    proc_create("obs_bg_thread", 0660, oplus_binder_dir,
			&proc_obs_bg_thread_fops);
	if (!proc_node) {
		pr_err("failed to create proc node obs_bg_thread.\n");
		goto err_create_oplus_binder_proc;
	}

	proc_node =
	    proc_create("obs_debug", 0660, oplus_binder_dir,
			&proc_obs_debug_fops);
	if (!proc_node) {
		pr_err("failed to create proc node obs_debug.\n");
		goto err_create_oplus_binder_proc;
	}

	proc_node =
	    proc_create("systemserver", 0440, oplus_binder_dir, &proc_ss_fops);
	if (!proc_node) {
		pr_err("failed to create proc node obs_debug ss_status.\n");
		goto err_create_oplus_binder_proc;
	}
	return ret;
err_create_oplus_binder_proc:
	remove_proc_entry(OPLUS_BINDER_PROC_DIR, NULL);
	return -ENOENT;
}

static int register_trans_ctrl_vendor_hooks(void)
{
	int ret = 0;

	register_trace_android_vh_binder_looper_state_registered(
			  android_vh_binder_looper_state_registered_handler, NULL);
	register_trace_android_vh_binder_free_proc(
			  android_vh_binder_free_proc_handler, NULL);
	register_trace_android_vh_binder_thread_release(
			  android_vh_binder_thread_release_handler, NULL);
	register_trace_android_vh_binder_has_work_ilocked(
			  android_vh_binder_has_work_ilocked_handler, NULL);
	register_trace_android_vh_binder_read_done(
			  android_vh_binder_read_done_handler, NULL);
	register_trace_android_vh_binder_restore_priority(
			  android_vh_binder_restore_priority_handler, NULL);
	register_trace_android_vh_binder_preset(
			  android_vh_binder_preset_handler, NULL);
	register_trace_android_vh_binder_proc_transaction_entry(
			  android_vh_binder_proc_transaction_entry_handler, NULL);
	register_trace_android_vh_binder_proc_transaction_finish(
			  android_vh_binder_proc_transaction_finish_handler, NULL);
	register_trace_android_vh_binder_select_worklist_ilocked(
			  android_vh_binder_select_worklist_ilocked_handler, NULL);

	return ret;
}

int oplus_trans_ctrl_init(void)
{
	int ret;

	OPLUS_OEM_DATA_SIZE_TEST(struct oplus_binder_transaction,
				 struct binder_transaction);

	ret = register_trans_ctrl_vendor_hooks();
	if (ret != 0)
		return ret;

	ret = oplus_binder_proc_init();

	obs_work_init();

	return ret;
}

module_param_named(trans_ctrl_enable, g_control_enable, uint, 0660);
module_param_named(record_enable, g_status_record, uint, 0660);
