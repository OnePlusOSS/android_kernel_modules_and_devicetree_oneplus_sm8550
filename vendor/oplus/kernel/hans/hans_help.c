#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>
#include <net/sock.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include "hans.h"

struct hlist_head *binder_procs = NULL;
struct mutex *binder_procs_lock = NULL;

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

static bool binder_worklist_empty_ilocked(struct list_head *list)
{
	return list_empty(list);
}


void binder_preset_handler(void *data, struct hlist_head *hhead, struct mutex *lock)
{
	if (binder_procs == NULL)
		binder_procs = hhead;

	if (binder_procs_lock == NULL)
		binder_procs_lock = lock;
}

void binder_trans_handler(void *data, struct binder_proc *target_proc,
				struct binder_proc *proc,
				struct binder_thread *thread,
				struct binder_transaction_data *tr)
{
	char buf_data[INTERFACETOKEN_BUFF_SIZE];
	size_t buf_data_size;
	char buf[INTERFACETOKEN_BUFF_SIZE] = {0};
	int i = 0;
	int j = 0;

	if (!(tr->flags & TF_ONE_WAY) /*report sync binder call*/
		&& target_proc
		&& (NULL != target_proc->tsk)
		&& (NULL != proc->tsk)
		&& (task_uid(target_proc->tsk).val > MIN_USERAPP_UID)
		&& (proc->pid != target_proc->pid)
		&& is_frozen_tg(target_proc->tsk)) {
		hans_report(SYNC_BINDER, task_tgid_nr(proc->tsk), task_uid(proc->tsk).val, task_tgid_nr(target_proc->tsk), task_uid(target_proc->tsk).val, "SYNC_BINDER", -1);
	}

#if defined(CONFIG_CFS_BANDWIDTH)
	if (!(tr->flags & TF_ONE_WAY) /*report sync binder call*/
		&& target_proc
		&& (NULL != target_proc->tsk)
		&& (NULL != proc->tsk)
		&& (task_uid(target_proc->tsk).val > MIN_USERAPP_UID || task_uid(target_proc->tsk).val == HANS_SYSTEM_UID) //uid >10000
		&& is_belong_cpugrp(target_proc->tsk)) {
		hans_report(SYNC_BINDER_CPUCTL, task_tgid_nr(proc->tsk), task_uid(proc->tsk).val, task_tgid_nr(target_proc->tsk), task_uid(target_proc->tsk).val, "SYNC_BINDER_CPUCTL", -1);
            }
#endif

	if ((tr->flags & TF_ONE_WAY) /*report async binder call*/
		&& target_proc
		&& (NULL != target_proc->tsk)
		&& (NULL != proc->tsk)
		&& (task_uid(target_proc->tsk).val > MIN_USERAPP_UID)
		&& (proc->pid != target_proc->pid)
		&& is_frozen_tg(target_proc->tsk)) {
		buf_data_size = tr->data_size>INTERFACETOKEN_BUFF_SIZE ?INTERFACETOKEN_BUFF_SIZE:tr->data_size;
		if (!copy_from_user(buf_data, (char*)tr->data.ptr.buffer, buf_data_size)) {
			/*1.skip first PARCEL_OFFSET bytes (useless data)
			  2.make sure the invalid address issue is not occuring(j =PARCEL_OFFSET+1, j+=2)
			  3.java layer uses 2 bytes char. And only the first bytes has the data.(p+=2)*/
			if (buf_data_size > PARCEL_OFFSET) {
				char *p = (char *)(buf_data) + PARCEL_OFFSET;
				j = PARCEL_OFFSET + 1;
				while (i < INTERFACETOKEN_BUFF_SIZE && j < buf_data_size && *p != '\0') {
					buf[i++] = *p;
					j += 2;
					p += 2;
				}
				if (i == INTERFACETOKEN_BUFF_SIZE) buf[i-1] = '\0';
			}
			hans_report(ASYNC_BINDER, task_tgid_nr(proc->tsk), task_uid(proc->tsk).val, task_tgid_nr(target_proc->tsk), task_uid(target_proc->tsk).val, buf, tr->code);
		}
	}


#if 0 /* ONLY FOR DEBUG*/
        if ((!(tr->flags & TF_ONE_WAY)) /*report sync binder call*/
                && target_proc
                && (task_uid(target_proc->tsk).val > MIN_USERAPP_UID)
                && (proc->pid != target_proc->pid)) {
                buf_data_size = tr->data_size>INTERFACETOKEN_BUFF_SIZE ?INTERFACETOKEN_BUFF_SIZE:tr->data_size;
                if (!copy_from_user(buf_data, (char*)tr->data.ptr.buffer, buf_data_size)) {
                        /*1.skip first PARCEL_OFFSET bytes (useless data)
                          2.make sure the invalid address issue is not occuring(j =PARCEL_OFFSET+1, j+=2)
                          3.java layer uses 2 bytes char. And only the first bytes has the data.(p+=2)*/
                        if (buf_data_size > PARCEL_OFFSET) {
                                char *p = (char *)(buf_data) + PARCEL_OFFSET;
                                j = PARCEL_OFFSET + 1;
                                while (i < INTERFACETOKEN_BUFF_SIZE && j < buf_data_size && *p != '\0') {
                                        buf[i++] = *p;
                                        j += 2;
                                        p += 2;
                                }
                                if (i == INTERFACETOKEN_BUFF_SIZE) buf[i-1] = '\0';
                        }
			printk(KERN_ERR "HANS: %d:%d-->%d:%d, %s, %u\n", task_tgid_nr(proc->tsk), task_uid(proc->tsk).val, task_tgid_nr(target_proc->tsk), task_uid(target_proc->tsk).val, buf, tr->code);
                }
        }
#endif


}

void binder_reply_handler(void *data, struct binder_proc *target_proc,
				struct binder_proc *proc,
				struct binder_thread *thread,
				struct binder_transaction_data *tr)
{
	/*only sync binder call has BC_REPLY*/
	if (target_proc
		&& (NULL != target_proc->tsk)
		&& (NULL != proc->tsk)
		&& (task_uid(target_proc->tsk).val <= MAX_SYSTEM_UID)
		&& (proc->pid != target_proc->pid)
		&& is_frozen_tg(target_proc->tsk)) {
		hans_report(SYNC_BINDER, task_tgid_nr(proc->tsk), task_uid(proc->tsk).val, task_tgid_nr(target_proc->tsk), task_uid(target_proc->tsk).val, "SYNC_BINDER_REPLY", -1);
	}
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
void binder_alloc_handler(void *data, size_t size, size_t *free_async_space, int is_async)
{
	struct task_struct *p = NULL;
	struct binder_alloc *alloc = NULL;

	alloc = container_of(free_async_space, struct binder_alloc, free_async_space);
	if (alloc == NULL) {
		return;
	}

	/*
	* async buffer free space bigger 1/3 buffer or buffer free lower 100K
	*/
	if (is_async
		&& (alloc->free_async_space < 3 * (size + sizeof(struct binder_buffer))
		|| (alloc->free_async_space < 100*1024))) {
		rcu_read_lock();
		p = find_task_by_vpid(alloc->pid);
		rcu_read_unlock();
		if (p != NULL && is_frozen_tg(p)) {
			hans_report(ASYNC_BINDER, task_tgid_nr(current), task_uid(current).val, task_tgid_nr(p), task_uid(p).val, "free_buffer_full", -1);
		}
	}
}
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
void binder_alloc_handler(void *data, size_t size, struct binder_alloc *alloc, int is_async)
{
	struct task_struct *p = NULL;

	/*
	* async buffer free space bigger 1/3 buffer or buffer free lower 100K
	*/
	if (is_async
		&& (alloc->free_async_space < 3 * (size + sizeof(struct binder_buffer))
		|| (alloc->free_async_space < 100*1024))) {
		rcu_read_lock();
		p = find_task_by_vpid(alloc->pid);
		rcu_read_unlock();
		if (p != NULL && is_frozen_tg(p)) {
			hans_report(ASYNC_BINDER, task_tgid_nr(current), task_uid(current).val, task_tgid_nr(p), task_uid(p).val, "free_buffer_full", -1);
		}
	}
}
#endif

void send_signal_handler(void *data, int sig, struct task_struct *killer, struct task_struct *dst)
{
	if (!dst || !killer) {
		return;
	}
	if (is_frozen_tg(dst)  /*signal receiver thread group is frozen?*/
		&& (sig == SIGKILL || sig == SIGTERM || sig == SIGABRT || sig == SIGQUIT)) {
		if (hans_report(SIGNAL, task_tgid_nr(killer), task_uid(killer).val, task_tgid_nr(dst), task_uid(dst).val, "signal", -1) == HANS_ERROR) {
			printk(KERN_ERR "HANS: report signal-freeze failed, sig = %d, caller = %d, target_uid = %d\n", sig, task_tgid_nr(killer), task_uid(dst).val);
		}
	}

#if defined(CONFIG_CFS_BANDWIDTH)
	if (is_belong_cpugrp(dst)  /*signal receiver thread group is cpuctl?*/
		&& (sig == SIGKILL || sig == SIGTERM || sig == SIGABRT || sig == SIGQUIT || sig == SIGIO)) {
			if (hans_report(SIGNAL, task_tgid_nr(killer), task_uid(killer).val, task_tgid_nr(dst), task_uid(dst).val, "signal", -1) == HANS_ERROR) {
			printk(KERN_ERR "HANS: report signal-cpuctl failed, sig = %d, caller = %d, target_uid = %d\n", sig, task_tgid_nr(killer), task_uid(dst).val);
		}
	}
#endif

}

static void get_uid_pid(int *from_pid, int *from_uid, int *to_pid, int *to_uid, struct binder_transaction *tr)
{
	*from_pid = tr->from ? tr->from->proc->pid : -1;
	*from_uid = tr->sender_euid.val;
	*to_pid = tr->to_proc ?  tr->to_proc->pid : -1;
	*to_uid = tr->to_thread ? task_uid(tr->to_thread->task).val : -1;
	if (*to_uid == -1)
		*to_uid = tr->to_proc ? task_uid(tr->to_proc->tsk).val : -1;
}

/* Only used for debuging, controled by FW */
void hans_check_uid_proc_status_debug(struct binder_proc *proc, enum message_type type)
{
	struct rb_node *n = NULL;
	struct binder_thread *thread = NULL;
	int from_uid = -1;
	int from_pid = -1;
	int to_uid = -1;
	int to_pid = -1;
	struct binder_transaction *btrans = NULL;
	bool empty = true;
	int need_reply = -1;
	struct binder_work *w = NULL;

	/*check binder_thread/transaction_stack/binder_proc ongoing transaction*/
	binder_inner_proc_lock(proc);
	for (n = rb_first(&proc->threads); n != NULL; n = rb_next(n)) {
		thread = rb_entry(n, struct binder_thread, rb_node);
		empty = binder_worklist_empty_ilocked(&thread->todo);

		if (thread->task != NULL) {
			/*has "todo" binder thread in worklist?*/
			to_uid = task_uid(thread->task).val;
			if (!empty) {
				/*scan thread->todo list*/
				list_for_each_entry(w, &thread->todo, entry) {
					if (w != NULL && w->type == BINDER_WORK_TRANSACTION) {
						btrans = container_of(w, struct binder_transaction, work);
						spin_lock(&btrans->lock);
						if (btrans->to_thread == thread) {
							need_reply = (int)(btrans->need_reply == 1);
							get_uid_pid(&from_pid, &from_uid, &to_pid, &to_uid, btrans);
							spin_unlock(&btrans->lock);
							hans_report(type, from_pid, from_uid, to_pid, to_uid, "FROZEN_TRANS_THREAD", need_reply);
						} else {
							spin_unlock(&btrans->lock);
						}
					}
				}
			}

			/*has transcation in transaction_stack?*/
			btrans = thread->transaction_stack;
			if (btrans) {
				spin_lock(&btrans->lock);
				if (btrans->to_thread == thread) {
					/*only report incoming binder call*/
					need_reply = (int)(btrans->need_reply == 1);
					get_uid_pid(&from_pid, &from_uid, &to_pid, &to_uid, btrans);
					spin_unlock(&btrans->lock);
					hans_report(type, from_pid, from_uid, to_pid, to_uid, "FROZEN_TRANS_STACK", need_reply);
				} else {
					spin_unlock(&btrans->lock);
				}
			}
		}
	}

	/*has "todo" binder proc in worklist*/
	empty = binder_worklist_empty_ilocked(&proc->todo);
	if (proc->tsk != NULL && !empty) {
		to_uid = task_uid(proc->tsk).val;
		list_for_each_entry(w, &proc->todo, entry) {
			if (w != NULL && w->type == BINDER_WORK_TRANSACTION) {
				btrans = container_of(w, struct binder_transaction, work);
				spin_lock(&btrans->lock);
				if (btrans->to_thread == thread) {
					need_reply = (int)(btrans->need_reply == 1);
					get_uid_pid(&from_pid, &from_uid, &to_pid, &to_uid, btrans);
					spin_unlock(&btrans->lock);
					hans_report(type, from_pid, from_uid, to_pid, to_uid, "FROZEN_TRANS_PROC", need_reply);
				} else {
					spin_unlock(&btrans->lock);
				}
			}
		}
	}
	binder_inner_proc_unlock(proc);
}

static void hans_check_uid_proc_status(struct binder_proc *proc, enum message_type type)
{
	struct rb_node *n = NULL;
	struct binder_thread *thread = NULL;
	int uid = -1;
	struct binder_transaction *btrans = NULL;
	bool empty = true;

	/* check binder_thread/transaction_stack/binder_proc ongoing transaction */
	binder_inner_proc_lock(proc);
	for (n = rb_first(&proc->threads); n != NULL; n = rb_next(n)) {
		thread = rb_entry(n, struct binder_thread, rb_node);
		empty = binder_worklist_empty_ilocked(&thread->todo);

		if (thread->task != NULL) {
			/* has "todo" binder thread in worklist? */
			uid = task_uid(thread->task).val;
			if (!empty) {
				binder_inner_proc_unlock(proc);
				hans_report(type, -1, -1, -1, uid, "FROZEN_TRANS_THREAD", 1);
				return;
			}

			/* has transcation in transaction_stack? */
			btrans = thread->transaction_stack;
			if (btrans) {
				spin_lock(&btrans->lock);
				if (btrans->to_thread == thread) {
					/* only report incoming binder call */
					spin_unlock(&btrans->lock);
					binder_inner_proc_unlock(proc);
					hans_report(type, -1, -1, -1, uid, "FROZEN_TRANS_STACK", 1);
					return;
				}
				spin_unlock(&btrans->lock);
			}
		}
	}

	/* has "todo" binder proc in worklist */
	empty = binder_worklist_empty_ilocked(&proc->todo);
	if (proc->tsk != NULL && !empty) {
		uid = task_uid(proc->tsk).val;
		binder_inner_proc_unlock(proc);
		hans_report(type, -1, -1, -1, uid, "FROZEN_TRANS_PROC", 1);
		return;
	}
	binder_inner_proc_unlock(proc);

}


void hans_check_frozen_transcation(uid_t uid, enum message_type type)
{
	struct binder_proc *proc;

	if (uid == TRANS_BINDER_DEBUG_ON) {
		trans_binder_debug = true;
	} else if (uid == TRANS_BINDER_DEBUG_OFF) {
		trans_binder_debug = false;
	}

	mutex_lock(binder_procs_lock);
	hlist_for_each_entry(proc, binder_procs, proc_node) {
		if (proc != NULL && (proc->tsk != NULL) && (task_uid(proc->tsk).val == uid)) {
			if (trans_binder_debug == true) {
				hans_check_uid_proc_status_debug(proc, type);
			} else {
				hans_check_uid_proc_status(proc, type);
			}
		}
	}
	mutex_unlock(binder_procs_lock);
}

