// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Oplus. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/plist.h>
#include <linux/sort.h>
#include <linux/completion.h>
#include <uapi/linux/sched/types.h>
#include <trace/hooks/sched.h>

#include "osi_hotthread.h"
#include "osi_topology.h"
#include "osi_tasktrack.h"
#include "osi_netlink.h"


struct rq_num {
	int num;
	int avg_num;
};

DEFINE_PER_CPU(struct rq_num, percpu_rq_num);
struct kmem_cache *hot_thread_struct_cachep;

extern unsigned long high_load_switch;
extern g_over_load;

static struct task_track_cpu task_track[MAX_CLUSTER];
struct hot_thread_struct  hot_thread_top[JANK_WIN_CNT][TOP_THREAD_CNT];

struct hot_thread_node {
	struct plist_node node;
	struct hot_thread_struct hot_thread_struct;
};

static PLIST_HEAD(hot_thread_head);
static DEFINE_RAW_SPINLOCK(hot_thread_lock);
static struct work_struct rqlen_notify_work;

#ifdef CONFIG_DEBUG_PLIST
static void plist_check_head(struct plist_head *head)
{
	if (!plist_head_empty(head))
		plist_check_list(&plist_first(head)->prio_list);
	plist_check_list(&head->node_list);
}

#else
# define plist_check_head(h)	do { } while (0)
#endif

void plist_add(struct plist_node *node, struct plist_head *head)
{
	struct plist_node *first, *iter, *prev = NULL;
	struct list_head *node_next = &head->node_list;

	WARN_ON(!plist_node_empty(node));
	WARN_ON(!list_empty(&node->prio_list));
	if (plist_head_empty(head))
		goto ins_node;
	first = iter = plist_first(head);
	do {
		if (node->prio < iter->prio) {
			node_next = &iter->node_list;
			break;
		}
		prev = iter;
		iter = list_entry(iter->prio_list.next, struct plist_node, prio_list);
	} while (iter != first);
	if (!prev || prev->prio != node->prio)
		list_add_tail(&node->prio_list, &iter->prio_list);
ins_node:
	list_add_tail(&node->node_list, node_next);
}

/**
 * plist_del - Remove a @node from plist.
 *
 * @node:	&struct plist_node pointer - entry to be removed
 * @head:	&struct plist_head pointer - list head
 */
void plist_del(struct plist_node *node, struct plist_head *head)
{
	plist_check_head(head);

	if (!list_empty(&node->prio_list)) {
		if (node->node_list.next != &head->node_list) {
			struct plist_node *next;

			next = list_entry(node->node_list.next,
					struct plist_node, node_list);

			/* add the next plist_node into prio_list */
			if (list_empty(&next->prio_list))
				list_add(&next->prio_list, &node->prio_list);
		}
		list_del_init(&node->prio_list);
	}

	list_del_init(&node->node_list);

	plist_check_head(head);
}

int find_in_plist(struct task_struct *p,  struct oplus_task_struct *ots)
{
	struct hot_thread_node *tmp;
	bool is_find = false;

	plist_for_each_entry(tmp, &hot_thread_head, node) {
		if (tmp->hot_thread_struct.pid == p->pid) {
			/*find the same node, just update hot thread info*/
			if (is_topapp(p))
				tmp->hot_thread_struct.top_app_cnt++;
			else
				tmp->hot_thread_struct.non_topapp_cnt++;
			tmp->hot_thread_struct.total_cnt++;
			is_find = true;
			break;
		}
	}
	if (is_find) {
		plist_del(&tmp->node, &hot_thread_head);
		plist_node_init(&tmp->node, INT_MAX - tmp->hot_thread_struct.total_cnt);
		plist_add(&tmp->node, &hot_thread_head);
		return 0;
	}
	return -ESRCH;
}

static int insert_hot_thread(struct oplus_task_struct *ots, struct task_struct *p, struct task_struct *leader, u32 now_idx)
{
	struct hot_thread_node  *hot_thread_node;
	unsigned long flags;
	const struct cred *tcred;
	uid_t uid;

	rcu_read_lock();
	tcred = __task_cred(p);
	if (!tcred) {
		rcu_read_unlock();
		pr_info("cpuload: tcred is NULL!");
		return -1;
	}
	uid = __kuid_val(tcred->uid);
	rcu_read_unlock();
	raw_spin_lock_irqsave(&hot_thread_lock, flags);
	if (find_in_plist(p, ots) == -ESRCH) {
			hot_thread_node = kmem_cache_zalloc(hot_thread_struct_cachep, GFP_ATOMIC);
			if (IS_ERR_OR_NULL(hot_thread_node))
				goto done;
			memcpy(hot_thread_node->hot_thread_struct.comm, p->comm, TASK_COMM_LEN);
			memcpy(hot_thread_node->hot_thread_struct.leader_comm, leader->comm, TASK_COMM_LEN);
			hot_thread_node->hot_thread_struct.pid = p->pid;
			hot_thread_node->hot_thread_struct.tgid = p->tgid;
			hot_thread_node->hot_thread_struct.uid = uid;
			hot_thread_node->hot_thread_struct.top_app_cnt = 0;
			hot_thread_node->hot_thread_struct.non_topapp_cnt = 0;
			if (is_topapp(p))
				hot_thread_node->hot_thread_struct.top_app_cnt = 1;
			else
				hot_thread_node->hot_thread_struct.non_topapp_cnt = 1;
			hot_thread_node->hot_thread_struct.total_cnt = 1;
			plist_node_init(&hot_thread_node->node, INT_MAX - hot_thread_node->hot_thread_struct.total_cnt);
			plist_add(&hot_thread_node->node, &hot_thread_head);
	}
done:
	raw_spin_unlock_irqrestore(&hot_thread_lock, flags);
	return 0;
}

static void  get_hot_thread(u32 now_idx, u64 now)
{
	struct hot_thread_node *hot_node, *tmp;
	unsigned long flags;
	int i = 0;

	memset(&hot_thread_top[now_idx][0], 0, TOP_THREAD_CNT * sizeof(struct hot_thread_struct));
	raw_spin_lock_irqsave(&hot_thread_lock, flags);
	plist_for_each_entry_safe(hot_node, tmp, &hot_thread_head, node) {
		if (!hot_node) {
			goto out;
		}
		if (i < TOP_THREAD_CNT) {
			memcpy(&hot_thread_top[now_idx][i], &hot_node->hot_thread_struct,
				sizeof(struct hot_thread_struct));
			i++;
		}
		plist_del(&hot_node->node, &hot_thread_head);
		kmem_cache_free(hot_thread_struct_cachep, hot_node);
	}
out:
	raw_spin_unlock_irqrestore(&hot_thread_lock, flags);
}

static void notify_rqlen_fn(struct work_struct *work)
{
	int rq_len[CPU_NUMS];
	int cpu;
	for_each_possible_cpu(cpu) {
		rq_len[cpu] = per_cpu(percpu_rq_num, cpu).avg_num;
	}
	send_to_user(NOTIFY_CPU_RQ, ARRAY_SIZE(rq_len), (int *)&rq_len);
}

static inline void count_rq_num(int cpu)
{
	struct rq_num *cur_rq;
	cur_rq = &per_cpu(percpu_rq_num, cpu);
	cur_rq->num += cpu_rq(cpu)->nr_running;
}

static void cal_rq_num(void)
{
	struct rq_num *cur_rq;
	int cpu;
	for_each_possible_cpu(cpu) {
		cur_rq = &per_cpu(percpu_rq_num, cpu);
		cur_rq->avg_num = cur_rq->num / TICK_PER_WIN;
		cur_rq->num = 0;
	}
}

static struct task_record *get_task_record(struct task_struct *t,
			u32 cluster)
{
	struct task_record *rc = NULL;
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	if (IS_ERR_OR_NULL(ots))
		return NULL;

	rc = (struct task_record *) (&(ots->record));
	return (struct task_record *) (&rc[cluster]);
}

/* updated in each tick */
void jank_hotthread_update_tick(struct task_struct *p, u64 now)
{
	struct task_record *record_p, *record_b;
	struct oplus_task_struct *ots;
	u64 timestamp, timestamp_prewin;
	u32 now_idx;
	u32 cpu, cluster_id;
	static int cal_rq_cnt;

	if (!p)
		return;
	ots = get_oplus_task_struct(p);
	cpu = p->cpu;
	cluster_id = get_cluster_id(cpu);
	record_p = get_task_record(p, cluster_id);

	if (!record_p)
		return;
	if (record_p->winidx == get_record_winidx(now)) {
		record_p->count++;
	} else {
		record_p->count = 1;
	}

	record_p->winidx = get_record_winidx(now);

	now_idx = time2winidx(now);
	if (unlikely(g_over_load)) {
		insert_hot_thread(ots, p, p->group_leader, now_idx);
		count_rq_num(cpu);
	}
	record_b = &task_track[cluster_id].track[now_idx].record;
	timestamp = task_track[cluster_id].track[now_idx].timestamp;
	timestamp_prewin = hot_thread_top[now_idx][0].timestamp;
	if (!is_same_idx(timestamp_prewin, now)) {
		if (unlikely(g_over_load)) {
			get_hot_thread(now_idx, now);
			cal_rq_num();
			if (cal_rq_cnt++ >= TICK_PER_WIN) {
				queue_work(system_wq, &rqlen_notify_work);
				cal_rq_cnt = 0;
			}
		}
		hot_thread_top[now_idx][0].timestamp = now;
	}
	if (!is_same_idx(timestamp, now) || (record_p->count > record_b->count)) {
		task_track[cluster_id].track[now_idx].pid = p->pid;
		task_track[cluster_id].track[now_idx].tgid = p->tgid;

		memcpy(task_track[cluster_id].track[now_idx].comm,
			p->comm, TASK_COMM_LEN);
		memcpy(record_b, record_p, sizeof(struct task_record));

		task_track[cluster_id].track[now_idx].timestamp = now;
	}
}

static void init_hot_thread_struct(void *ptr)
{
	struct hot_thread_node *hot_thread_node = ptr;
	memset(hot_thread_node, 0, sizeof(struct hot_thread_node));
}

void hotthread_show(struct seq_file *m, u32 win_idx, u64 now)
{
	u32 i, now_index, idx;
	bool nospace = false;
	struct hot_thread_struct  *tmp_track;
	u64 timestamp;
	pid_t  pid, tgid;
	uid_t uid;

	now_index =  time2winidx(now);
	for (i = 0; i < TOP_THREAD_CNT; i++) {
		nospace = (i == TOP_THREAD_CNT-1) ? true : false;
		idx = winidx_sub(now_index, win_idx);
		tmp_track = &hot_thread_top[idx][i];
		pid = tmp_track->pid;
		tgid = tmp_track->tgid;
		uid = tmp_track->uid;
		timestamp = tmp_track->timestamp;
		if (tmp_track->total_cnt) {
			seq_printf(m, "%d$%d$%s$%d$%s$%d$%d%s", uid, tgid, tmp_track->leader_comm,
			pid, tmp_track->comm, tmp_track->top_app_cnt, tmp_track->non_topapp_cnt,
			nospace ? "" : "  ");
		}
	}
}

void jank_hotthread_show(struct seq_file *m, u32 win_idx, u64 now)
{
	u32 i, idx, now_index;
	u64 timestamp;
	struct task_track *track_p;
	struct task_struct *leader;
	char *comm;
	bool nospace = false;

	now_index =  time2winidx(now);

	for (i = 0; i < MAX_CLUSTER; i++) {
		nospace = (i == MAX_CLUSTER-1) ? true : false;


		idx = winidx_sub(now_index, win_idx);
		track_p = &task_track[i].track[idx];
		timestamp = track_p->timestamp;

		leader = jank_find_get_task_by_vpid(track_p->tgid);
		comm = leader ? leader->comm : track_p->comm;

		/*
		 * The following situations indicate that this thread is not hot
		 *  a) task is null, which means no suitable task, or task is dead
		 *  b) the time stamp is overdue
		 *  c) count did not reach the threshold
		 */
		if (!timestamp_is_valid(timestamp, now))
			seq_printf(m, "-%s", nospace ? "" : " ");
		else
			seq_printf(m, "%s$%d%s", comm,
						track_p->record.count,
						nospace ? "" : " ");

		if (leader)
			put_task_struct(leader);
	}
}

static int  top_hotthread_dump_win(struct seq_file *m, void *v, u32 win_cnt)
{
	u32 i;
	u64 now = jiffies_to_nsecs(jiffies);

	for (i = 0; i < win_cnt; i++) {
		hotthread_show(m, i, now);
		seq_puts(m, "\n");
	}
	return 0;
}

static int proc_top_hotthread_show(struct seq_file *m, void *v)
{
	return top_hotthread_dump_win(m, v, JANK_WIN_CNT/2);
}
static int proc_top_hotthread_open(struct inode *inode,
		struct file *file)
{
	return single_open(file, proc_top_hotthread_show, inode);
}

static const struct proc_ops proc_top_hotthread_info_operations = {
	.proc_open	=	proc_top_hotthread_open,
	.proc_read	=	seq_read,
	.proc_lseek	=	seq_lseek,
	.proc_release   =	single_release,
};


int osi_hotthread_proc_init(struct proc_dir_entry *pde)
{
	struct proc_dir_entry *entry = NULL;

	entry = proc_create("top_hotthread", S_IRUGO,
				pde, &proc_top_hotthread_info_operations);
	if (!entry) {
		osi_err("create top_hotthread fail\n");
		return -1;
	}
	hot_thread_struct_cachep = kmem_cache_create("hot_thread_node",
			sizeof(struct hot_thread_node), 0,
			SLAB_PANIC|SLAB_ACCOUNT, init_hot_thread_struct);
	if (!hot_thread_struct_cachep)
		return -ENOMEM;

	INIT_WORK(&rqlen_notify_work, notify_rqlen_fn);
	return 0;
}

void osi_hotthread_proc_deinit(struct proc_dir_entry *pde)
{
	remove_proc_entry("top_hotthread", pde);
}

