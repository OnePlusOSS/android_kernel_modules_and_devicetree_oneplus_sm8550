// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/sched/cpufreq.h>
#include <linux/syscore_ops.h>
#include <drivers/android/binder_internal.h>
#include <kernel/sched/sched.h>
#include <linux/reciprocal_div.h>

#include <trace/hooks/binder.h>
#include <trace/hooks/sched.h>
#include <trace/events/sched.h>
#include <trace/events/task.h>
#include <trace/events/power.h>

#include <../kernel/oplus_cpu/sched/sched_assist/sa_common.h>
#include "frame_boost.h"
#include "cluster_boost.h"
#include "frame_debug.h"

#define NONE_FRAME_TASK      (0)
#define STATIC_FRAME_TASK    (1 << 0)
#define BINDER_FRAME_TASK    (1 << 1)
#define FRAME_COMPOSITION    (1 << 2)
#define FRAME_GAME           (1 << 3)

#define DEFAULT_BINDER_DEPTH (2)
#define MAX_BINDER_THREADS   (6)
#define INVALID_FBG_DEPTH    (-1)

#define DEFAULT_FRAME_RATE   (60)

#define DEFAULT_FREQ_UPDATE_MIN_INTERVAL    (2 * NSEC_PER_MSEC)
#define DEFAULT_UTIL_INVALID_INTERVAL       (32 * NSEC_PER_MSEC)

/*********************************
 * frame group global data
 *********************************/
struct frame_group {
	struct list_head tasks;

	u64 window_start;
	u64 prev_window_size;
	u64 window_size;

	u64 curr_window_scale;
	u64 curr_window_exec;
	u64 prev_window_scale;
	u64 prev_window_exec;

	unsigned int window_busy;

	/* nr_running:
	 *     The number of running threads in the group
	 * mark_start:
	 *     Mark the start time of next load track
	 */
	int nr_running;
	u64 mark_start;

	unsigned int frame_zone;

	u64 last_freq_update_time;
	u64 last_util_update_time;

	/* For Surfaceflinger Process:
	 *     ui is "surfaceflinger", render is "RenderEngine"
	 * For Top Application:
	 *     ui is "UI Thread", render is "RenderThread"
	 */
	int ui_pid, render_pid;
	struct task_struct *ui, *render;

	/* Binder frame task information */
	int binder_thread_num;

	/* Frame group task should be placed on these clusters */
	struct oplus_sched_cluster *preferred_cluster;
	struct oplus_sched_cluster *available_cluster;
	/* Util used to adjust cpu frequency */
	unsigned long policy_util;
	unsigned long curr_util;
};

static DEFINE_RAW_SPINLOCK(def_fbg_lock);
static struct frame_group default_frame_boost_group;

static DEFINE_RAW_SPINLOCK(sf_fbg_lock);
static struct frame_group sf_composition_group;

static DEFINE_RAW_SPINLOCK(game_fbg_lock);
static struct frame_group game_frame_boost_group;

static DEFINE_RAW_SPINLOCK(freq_protect_lock);

static atomic_t fbg_initialized = ATOMIC_INIT(0);

__read_mostly int num_sched_clusters;

unsigned long ed_task_boost_mid_duration;
unsigned long ed_task_boost_max_duration;
unsigned long ed_task_boost_timeout_duration;
unsigned int ed_task_boost_type;
EXPORT_SYMBOL_GPL(ed_task_boost_type);

struct list_head cluster_head;
#define for_each_sched_cluster(cluster) \
	list_for_each_entry_rcu(cluster, &cluster_head, list)

/*********************************
 * frame group common function
 *********************************/
static inline void move_list(struct list_head *dst, struct list_head *src)
{
	struct list_head *first, *last;

	first = src->next;
	last = src->prev;

	first->prev = dst;
	dst->prev = last;
	last->next = dst;

	/* Ensure list sanity before making the head visible to all CPUs. */
	smp_mb();
	dst->next = first;
}

static void get_possible_siblings(int cpuid, struct cpumask *cluster_cpus)
{
	int cpu;
	struct cpu_topology *cpu_topo, *cpuid_topo = &cpu_topology[cpuid];

	if (cpuid_topo->package_id == -1)
		return;

	for_each_possible_cpu(cpu) {
		cpu_topo = &cpu_topology[cpu];

		if (cpuid_topo->package_id != cpu_topo->package_id)
			continue;
		cpumask_set_cpu(cpu, cluster_cpus);
	}
}

static void insert_cluster(struct oplus_sched_cluster *cluster, struct list_head *head)
{
	struct oplus_sched_cluster *tmp;
	struct list_head *iter = head;

	list_for_each_entry(tmp, head, list) {
		if (arch_scale_cpu_capacity(cpumask_first(&cluster->cpus))
			< arch_scale_cpu_capacity(cpumask_first(&tmp->cpus)))
			break;
		iter = &tmp->list;
	}

	list_add(&cluster->list, iter);
}

static void cleanup_clusters(struct list_head *head)
{
	struct oplus_sched_cluster *cluster, *tmp;

	list_for_each_entry_safe(cluster, tmp, head, list) {
		list_del(&cluster->list);
		num_sched_clusters--;
		kfree(cluster);
	}
}

static struct oplus_sched_cluster *alloc_new_cluster(const struct cpumask *cpus)
{
	struct oplus_sched_cluster *cluster = NULL;

	cluster = kzalloc(sizeof(struct oplus_sched_cluster), GFP_ATOMIC);
	BUG_ON(!cluster);

	INIT_LIST_HEAD(&cluster->list);
	cluster->cpus = *cpus;

	return cluster;
}

struct oplus_sched_cluster *fb_cluster[MAX_CLS_NUM];

static inline void add_cluster(const struct cpumask *cpus, struct list_head *head)
{
	unsigned long capacity = 0, insert_capacity = 0;
	struct oplus_sched_cluster *cluster = NULL;

	capacity = arch_scale_cpu_capacity(cpumask_first(cpus));
	/* If arch_capacity is no different between mid cluster and max cluster,
	 * just combind them
	 */
	list_for_each_entry_rcu(cluster, head, list) {
		insert_capacity = arch_scale_cpu_capacity(cpumask_first(&cluster->cpus));
		if (capacity == insert_capacity) {
			ofb_debug("insert cluster=%*pbl is same as exist cluster=%*pbl\n",
				cpumask_pr_args(cpus), cpumask_pr_args(&cluster->cpus));
			cpumask_or(&cluster->cpus, &cluster->cpus, cpus);
			return;
		}
	}

	cluster = alloc_new_cluster(cpus);
	insert_cluster(cluster, head);

	fb_cluster[num_sched_clusters] = cluster;

	num_sched_clusters++;
}

static inline void assign_cluster_ids(struct list_head *head)
{
	struct oplus_sched_cluster *cluster;
	int pos = 0;

	list_for_each_entry(cluster, head, list)
		cluster->id = pos++;
}

static bool build_clusters(void)
{
	struct cpumask cpus = *cpu_possible_mask;
	struct cpumask cluster_cpus;
	struct list_head new_head;
	int i;

	INIT_LIST_HEAD(&cluster_head);
	INIT_LIST_HEAD(&new_head);

	/* If this work failed, our cluster_head can still used with only one cluster struct */
	for_each_cpu(i, &cpus) {
		cpumask_clear(&cluster_cpus);
		get_possible_siblings(i, &cluster_cpus);
		if (cpumask_empty(&cluster_cpus)) {
			cleanup_clusters(&new_head);
			return false;
		}
		cpumask_andnot(&cpus, &cpus, &cluster_cpus);
		add_cluster(&cluster_cpus, &new_head);
	}

	assign_cluster_ids(&new_head);
	move_list(&cluster_head, &new_head);
	return true;
}

/* We use these flag(FRAME_COMPOSITION, FRAME_GAME) to check which group @task is in
 * instead of traversing the whole group list
 */
static inline struct frame_group *task_get_frame_group(struct oplus_task_struct *ots)
{
	if (ots->fbg_state & FRAME_COMPOSITION)
		return &sf_composition_group;
	else if (ots->fbg_state & FRAME_GAME)
		return &game_frame_boost_group;
	else
		return &default_frame_boost_group;
}

static inline raw_spinlock_t *task_get_frame_group_lock(struct oplus_task_struct *ots)
{
	if (ots->fbg_state & FRAME_COMPOSITION)
		return &sf_fbg_lock;
	else if (ots->fbg_state & FRAME_GAME)
		return &game_fbg_lock;
	else
		return &def_fbg_lock;
}

static inline int get_frame_group_id(struct frame_group *grp)
{
	if (grp == &sf_composition_group)
		return SF_FRAME_GROUP_ID;
	else if (grp == &game_frame_boost_group)
		return GAME_FRAME_GROUP_ID;
	else /* grp == &default_frame_boost_group */
		return DEFAULT_FRAME_GROUP_ID;
}

static inline void frame_grp_with_lock_assert(struct frame_group *grp)
{
	if (grp == &sf_composition_group)
		lockdep_assert_held(&sf_fbg_lock);
	else if (grp == &game_frame_boost_group)
		lockdep_assert_held(&game_fbg_lock);
	else /* grp == &default_frame_boost_group */
		lockdep_assert_held(&def_fbg_lock);
}

static inline bool __frame_boost_enabled(void)
{
	return likely(sysctl_frame_boost_enable);
}

bool frame_boost_enabled(void)
{
	return __frame_boost_enabled();
}
EXPORT_SYMBOL_GPL(frame_boost_enabled);

bool is_fbg_task(struct task_struct *p)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(p);

	if (IS_ERR_OR_NULL(ots))
		return false;

	return ots->fbg_state;
}
EXPORT_SYMBOL_GPL(is_fbg_task);

/*********************************
 * frame group clock
 *********************************/
static ktime_t ktime_last;
static bool fbg_ktime_suspended;

u64 fbg_ktime_get_ns(void)
{
	if (unlikely(fbg_ktime_suspended))
		return ktime_to_ns(ktime_last);

	return ktime_get_ns();
}
EXPORT_SYMBOL_GPL(fbg_ktime_get_ns);

static void fbg_resume(void)
{
	fbg_ktime_suspended = false;
}

static int fbg_suspend(void)
{
	ktime_last = ktime_get();
	fbg_ktime_suspended = true;
	return 0;
}

static struct syscore_ops fbg_syscore_ops = {
	.resume		= fbg_resume,
	.suspend	= fbg_suspend
};

/***************************************************
 * add/remove static frame task to/from frame group
 ***************************************************/
static void remove_task_from_frame_group(struct task_struct *tsk)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(tsk);
	struct frame_group *grp = NULL;

	if (IS_ERR_OR_NULL(ots))
		return;

	grp = task_get_frame_group(ots);
	frame_grp_with_lock_assert(grp);

	raw_spin_lock(&ots->fbg_list_entry_lock);

	if (ots->fbg_state & STATIC_FRAME_TASK) {
		list_del_init(&ots->fbg_list);
		ots->fbg_state = NONE_FRAME_TASK;
		ots->fbg_depth = INVALID_FBG_DEPTH;

		if (tsk == grp->ui) {
			grp->ui = NULL;
			grp->ui_pid = 0;
		} else if (tsk == grp->render) {
			grp->render = NULL;
			grp->render_pid = 0;
		}

		if (ots->fbg_running) {
			ots->fbg_running = false;
			grp->nr_running--;
			if (unlikely(grp->nr_running < 0))
				grp->nr_running = 0;
		}

		put_task_struct(tsk);
	}

	raw_spin_unlock(&ots->fbg_list_entry_lock);

	if (list_empty(&grp->tasks)) {
		grp->preferred_cluster = NULL;
		grp->available_cluster = NULL;
		grp->policy_util = 0;
		grp->curr_util = 0;
		grp->nr_running = 0;
	}
}

static void clear_all_static_frame_task(struct frame_group *grp)
{
	struct oplus_task_struct *ots = NULL;
	struct oplus_task_struct *tmp = NULL;
	struct task_struct *p = NULL;

	list_for_each_entry_safe(ots, tmp, &grp->tasks, fbg_list) {
		p = ots_to_ts(ots);

		raw_spin_lock(&ots->fbg_list_entry_lock);

		if (ots->fbg_state & STATIC_FRAME_TASK) {
			list_del_init(&ots->fbg_list);
			ots->fbg_state = NONE_FRAME_TASK;
			ots->fbg_depth = INVALID_FBG_DEPTH;

			if (p == grp->ui) {
				grp->ui = NULL;
				grp->ui_pid = 0;
			} else if (p == grp->render) {
				grp->render = NULL;
				grp->render_pid = 0;
			}

			if (ots->fbg_running) {
				ots->fbg_running = false;
				grp->nr_running--;
				if (unlikely(grp->nr_running < 0))
					grp->nr_running = 0;
			}

			put_task_struct(p);
		}

		raw_spin_unlock(&ots->fbg_list_entry_lock);
	}

	if (list_empty(&grp->tasks)) {
		grp->preferred_cluster = NULL;
		grp->available_cluster = NULL;
		grp->policy_util = 0;
		grp->curr_util = 0;
		grp->nr_running = 0;
	}
}

static void add_task_to_frame_group(struct frame_group *grp, struct task_struct *task)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(task);

	if (IS_ERR_OR_NULL(ots))
		return;

	raw_spin_lock(&ots->fbg_list_entry_lock);

	if (ots->fbg_state || task->flags & PF_EXITING) {
		raw_spin_unlock(&ots->fbg_list_entry_lock);
		return;
	}

	list_add(&ots->fbg_list, &grp->tasks);
	ots->fbg_state = STATIC_FRAME_TASK;

	if (grp == &sf_composition_group)
		ots->fbg_state |= FRAME_COMPOSITION;
	else if (grp == &game_frame_boost_group)
		ots->fbg_state |= FRAME_GAME;

	/* Static frame task's depth is zero */
	ots->fbg_depth = 0;

	raw_spin_unlock(&ots->fbg_list_entry_lock);
}

void set_ui_thread(int pid, int tid)
{
	unsigned long flags;
	struct task_struct *ui;
	struct frame_group *grp = &default_frame_boost_group;

	raw_spin_lock_irqsave(&def_fbg_lock, flags);
	if (pid <= 0 || pid == grp->ui_pid) {
		raw_spin_unlock_irqrestore(&def_fbg_lock, flags);
		return;
	}

	rcu_read_lock();
	ui = find_task_by_vpid(pid);
	if (ui)
		get_task_struct(ui);
	rcu_read_unlock();

	if (grp->ui)
		clear_all_static_frame_task(grp);

	if (ui) {
		grp->ui = ui;
		grp->ui_pid = pid;
		add_task_to_frame_group(grp, ui);
	}

	raw_spin_unlock_irqrestore(&def_fbg_lock, flags);
}
EXPORT_SYMBOL_GPL(set_ui_thread);

void set_render_thread(int pid, int tid)
{
	unsigned long flags;
	struct task_struct *render;
	struct frame_group *grp = &default_frame_boost_group;

	raw_spin_lock_irqsave(&def_fbg_lock, flags);
	if (tid <= 0 || pid != grp->ui_pid || tid == grp->render_pid) {
		raw_spin_unlock_irqrestore(&def_fbg_lock, flags);
		return;
	}

	rcu_read_lock();
	render = find_task_by_vpid(tid);
	if (render)
		get_task_struct(render);
	rcu_read_unlock();

	if (grp->render)
		remove_task_from_frame_group(grp->render);

	if (render) {
		grp->render = render;
		grp->render_pid = tid;
		add_task_to_frame_group(grp, render);
	}

	raw_spin_unlock_irqrestore(&def_fbg_lock, flags);
}
EXPORT_SYMBOL_GPL(set_render_thread);

int get_frame_group_ui(void)
{
	return default_frame_boost_group.ui_pid;
}
EXPORT_SYMBOL_GPL(get_frame_group_ui);

void set_sf_thread(int pid, int tid)
{
	unsigned long flags;
	struct task_struct *ui;
	struct frame_group *grp = &sf_composition_group;

	raw_spin_lock_irqsave(&sf_fbg_lock, flags);
	if (pid <= 0 || pid == grp->ui_pid) {
		raw_spin_unlock_irqrestore(&sf_fbg_lock, flags);
		return;
	}

	rcu_read_lock();
	ui = find_task_by_vpid(pid);
	if (ui)
		get_task_struct(ui);
	rcu_read_unlock();

	if (grp->ui)
		clear_all_static_frame_task(grp);

	if (ui) {
		grp->ui = ui;
		grp->ui_pid = pid;
		add_task_to_frame_group(grp, ui);
	}

	raw_spin_unlock_irqrestore(&sf_fbg_lock, flags);
}
EXPORT_SYMBOL_GPL(set_sf_thread);

void set_renderengine_thread(int pid, int tid)
{
	unsigned long flags;
	struct task_struct *render;
	struct frame_group *grp = &sf_composition_group;

	raw_spin_lock_irqsave(&sf_fbg_lock, flags);
	if (tid <= 0 || pid != grp->ui_pid || tid == grp->render_pid) {
		raw_spin_unlock_irqrestore(&sf_fbg_lock, flags);
		return;
	}

	rcu_read_lock();
	render = find_task_by_vpid(tid);
	if (render)
		get_task_struct(render);
	rcu_read_unlock();

	if (grp->render)
		remove_task_from_frame_group(grp->render);

	if (render) {
		grp->render = render;
		grp->render_pid = tid;
		add_task_to_frame_group(grp, render);
	}

	raw_spin_unlock_irqrestore(&sf_fbg_lock, flags);
}
EXPORT_SYMBOL_GPL(set_renderengine_thread);

static inline bool is_same_uid(struct task_struct *p, struct task_struct *grp_ui)
{
	int p_uid, ui_uid;

	if (p == NULL || grp_ui == NULL)
		return false;

	p_uid = task_uid(p).val;
	ui_uid = task_uid(grp_ui).val;

	return (p_uid == ui_uid);
}

bool add_rm_related_frame_task(int pid, int tid, int add, int r_depth, int r_width)
{
	unsigned long flags;
	struct task_struct *tsk = NULL;
	struct frame_group *grp = NULL;
	bool success = false;

	rcu_read_lock();
	tsk = find_task_by_vpid(tid);
	if (!tsk)
		goto out;

	grp = &default_frame_boost_group;
	raw_spin_lock_irqsave(&def_fbg_lock, flags);
	if (add && is_same_uid(tsk, grp->ui)) {
		get_task_struct(tsk);
		add_task_to_frame_group(grp, tsk);
	} else if (!add) {
		remove_task_from_frame_group(tsk);
	}
	raw_spin_unlock_irqrestore(&def_fbg_lock, flags);

	if (r_depth > 0 && r_width > 0) {
		/* TODO: find related threads and set them as frame task */
	}

	success = true;
out:
	rcu_read_unlock();
	return success;
}
EXPORT_SYMBOL_GPL(add_rm_related_frame_task);

bool add_task_to_game_frame_group(int tid, int add)
{
	unsigned long flags;
	struct task_struct *tsk = NULL;
	struct frame_group *grp = NULL;
	struct oplus_task_struct *ots = NULL;
	bool success = false;

	rcu_read_lock();
	tsk = find_task_by_vpid(tid);
	/* game_frame_boost_group not add binder task */
	if (!tsk || strstr(tsk->comm, "binder:") || strstr(tsk->comm, "HwBinder:"))
		goto out;

	ots = get_oplus_task_struct(tsk);
	if (IS_ERR_OR_NULL(ots))
		goto out;

	raw_spin_lock(&ots->fbg_list_entry_lock);
	/*
	 * only the task which not belonged to any group can be added to game_frame_boost_group in this func,
	 * only the task which belonged to game_frame_boost_group can be removed in this func.
	 */
	if ((add && ots->fbg_state)
		|| (!add && !(ots->fbg_state & FRAME_GAME))) {
		raw_spin_unlock(&ots->fbg_list_entry_lock);
		goto out;
	}
	raw_spin_unlock(&ots->fbg_list_entry_lock);

	grp = &game_frame_boost_group;
	raw_spin_lock_irqsave(&game_fbg_lock, flags);
	if (add) {
		get_task_struct(tsk);
		add_task_to_frame_group(grp, tsk);
	} else if (!add) {
		remove_task_from_frame_group(tsk);
	}
	raw_spin_unlock_irqrestore(&game_fbg_lock, flags);

	success = true;
out:
	rcu_read_unlock();
	return success;
}
EXPORT_SYMBOL_GPL(add_task_to_game_frame_group);

/**********************************************************
 * add/remove dynamic binder frame task to/from frame group
 **********************************************************/
static void remove_binder_from_frame_group(struct task_struct *binder)
{
	struct oplus_task_struct *ots_binder = get_oplus_task_struct(binder);
	struct frame_group *grp = NULL;

	if (IS_ERR_OR_NULL(ots_binder) || !(ots_binder->fbg_state & BINDER_FRAME_TASK))
		return;

	grp = task_get_frame_group(ots_binder);
	frame_grp_with_lock_assert(grp);

	raw_spin_lock(&ots_binder->fbg_list_entry_lock);

	/* judge two times for hot path performance */
	if (!(ots_binder->fbg_state & BINDER_FRAME_TASK)) {
		raw_spin_unlock(&ots_binder->fbg_list_entry_lock);
		return;
	}

	list_del_init(&ots_binder->fbg_list);
	ots_binder->fbg_state = NONE_FRAME_TASK;
	ots_binder->fbg_depth = INVALID_FBG_DEPTH;

	raw_spin_unlock(&ots_binder->fbg_list_entry_lock);

	grp->binder_thread_num--;

	if (grp->binder_thread_num < 0)
		ofb_err("group binder num is less than 0, binder_num=%d, group_id=%d, prio=%d",
			grp->binder_thread_num, get_frame_group_id(grp), binder->prio);

	put_task_struct(binder);
}

static void add_binder_to_frame_group(struct task_struct *binder, struct task_struct *from)
{
	struct oplus_task_struct *ots_binder, *ots_from;
	unsigned long flags;
	struct frame_group *grp = NULL;
	bool composition_part = false;

	if (binder == NULL || from == NULL)
		return;

	ots_binder = get_oplus_task_struct(binder);
	ots_from = get_oplus_task_struct(from);

	if (IS_ERR_OR_NULL(ots_binder) || IS_ERR_OR_NULL(ots_from))
		return;

	/* game_frame_boost_group not add binder task */
	if (ots_from->fbg_state & FRAME_GAME)
		return;

	if (ots_from->fbg_state & FRAME_COMPOSITION) {
		composition_part = true;
		grp = &sf_composition_group;
		raw_spin_lock_irqsave(&sf_fbg_lock, flags);
	} else {
		grp = &default_frame_boost_group;
		raw_spin_lock_irqsave(&def_fbg_lock, flags);
	}

	if (ots_from->fbg_state == NONE_FRAME_TASK || ots_binder->fbg_state)
		goto unlock;

	if (grp->binder_thread_num > MAX_BINDER_THREADS)
		goto unlock;

	if ((ots_from->fbg_state & BINDER_FRAME_TASK) &&
		ots_from->fbg_depth >= DEFAULT_BINDER_DEPTH) {
		goto unlock;
	}

	raw_spin_lock(&ots_binder->fbg_list_entry_lock);

	/* judge two times for hot path performance */
	if (ots_binder->fbg_state) {
		raw_spin_unlock(&ots_binder->fbg_list_entry_lock);
		goto unlock;
	}

	get_task_struct(binder);
	list_add(&ots_binder->fbg_list, &grp->tasks);
	ots_binder->fbg_state = BINDER_FRAME_TASK;

	if (composition_part)
		ots_binder->fbg_state |= FRAME_COMPOSITION;

	ots_binder->fbg_depth = ots_from->fbg_depth + 1;

	raw_spin_unlock(&ots_binder->fbg_list_entry_lock);

	grp->binder_thread_num++;

unlock:
	if (composition_part)
		raw_spin_unlock_irqrestore(&sf_fbg_lock, flags);
	else
		raw_spin_unlock_irqrestore(&def_fbg_lock, flags);
}

/*
 * task_rename_hook - check if the binder thread should be add to
 *                            frame group
 * @task: binder thread that to be wokeup.
 * @sync: whether to do a synchronous wake-up.
 *       the other paramenter is unused
 */
void task_rename_hook(void *unused, struct task_struct *p, const char *buf)
{
}

#ifdef TODO_DELME
/*
 * fbg_binder_wakeup_hook - check if the binder thread should be add to
 *                            frame group
 * @task: binder thread that to be wokeup.
 * @sync: whether to do a synchronous wake-up.
 *       the other paramenter is unused
 */
static void fbg_binder_wakeup_hook(void *unused, struct task_struct *caller_task,
	struct task_struct *binder_proc_task, struct task_struct *binder_th_task,
	bool pending_async, bool sync)
{
	if (sync)
		add_binder_to_frame_group(binder_th_task, current);
}
#endif
/*
 * fbg_binder_restore_priority_hook - check if the binder thread should be remove from
 *             frame group after finishing their work
 * @task: binder thread that finished binder request and restore to saved priority.
 * @t: binder transaction that to be finished
 *       the other paramenter is unused
 */
static void fbg_binder_restore_priority_hook(void *unused, struct binder_transaction *t,
	struct task_struct *task)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(task);
	unsigned long flags;
	raw_spinlock_t *lock = NULL;

	if (IS_ERR_OR_NULL(ots))
		return;

	lock = task_get_frame_group_lock(ots);

	if (task != NULL) {
		raw_spin_lock_irqsave(lock, flags);
		remove_binder_from_frame_group(task);
		raw_spin_unlock_irqrestore(lock, flags);
	}
}

/*
 * fbg_binder_wait_for_work_hook - check if the binder thread should be remove from
 *             frame group before insert to idle binder list
 * @task: binder thread
 * @do_proc_work: whether the binder thread is waiting for new request
 *       the other paramenter is unused
 */
static void fbg_binder_wait_for_work_hook(void *unused, bool do_proc_work,
	struct binder_thread *tsk, struct binder_proc *proc)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(tsk->task);
	unsigned long flags;
	raw_spinlock_t *lock = NULL;

	if (IS_ERR_OR_NULL(ots))
		return;

	if (do_proc_work) {
		lock = task_get_frame_group_lock(ots);

		raw_spin_lock_irqsave(lock, flags);
		remove_binder_from_frame_group(tsk->task);
		raw_spin_unlock_irqrestore(lock, flags);
	}
}

static void fbg_sync_txn_recvd_hook(void *unused, struct task_struct *tsk, struct task_struct *from)
{
	add_binder_to_frame_group(tsk, from);
}


/*********************************
 * load tracking for frame group
 *********************************/
static inline unsigned int get_cur_freq(unsigned int cpu)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get_raw(cpu);

	return (policy == NULL) ? 0 : policy->cur;
}

static inline unsigned int get_max_freq(unsigned int cpu)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get_raw(cpu);

	return (policy == NULL) ? 0 : policy->cpuinfo.max_freq;
}

void update_ed_task_boost_mid_duration(unsigned int ed_task_boost_mid_duration_scale)
{
	struct frame_group *grp = NULL;

	grp = &default_frame_boost_group;
	if (grp && grp->window_size)
		ed_task_boost_mid_duration = mult_frac(grp->window_size, ed_task_boost_mid_duration_scale, 100);
}
EXPORT_SYMBOL_GPL(update_ed_task_boost_mid_duration);

void update_ed_task_boost_max_duration(unsigned int ed_task_boost_max_duration_scale)
{
	struct frame_group *grp = NULL;

	grp = &default_frame_boost_group;
	if (grp && grp->window_size)
		ed_task_boost_max_duration = mult_frac(grp->window_size, ed_task_boost_max_duration_scale, 100);
}
EXPORT_SYMBOL_GPL(update_ed_task_boost_max_duration);

void update_ed_task_boost_timeout_duration(unsigned int ed_task_boost_timeout_duration_scale)
{
	struct frame_group *grp = NULL;

	grp = &default_frame_boost_group;
	if (grp && grp->window_size)
		ed_task_boost_timeout_duration = mult_frac(grp->window_size, ed_task_boost_timeout_duration_scale, 100);
}
EXPORT_SYMBOL_GPL(update_ed_task_boost_timeout_duration);

void set_frame_group_window_size(unsigned int window_size)
{
	struct frame_group *grp = NULL;
	unsigned long flags;

	ed_task_boost_mid_duration = mult_frac(window_size, stune_boost[BOOST_ED_TASK_MID_DURATION], 100);
	ed_task_boost_max_duration = mult_frac(window_size, stune_boost[BOOST_ED_TASK_MAX_DURATION], 100);
	ed_task_boost_timeout_duration = mult_frac(window_size, stune_boost[BOOST_ED_TASK_TIME_OUT_DURATION], 100);

	grp = &default_frame_boost_group;
	raw_spin_lock_irqsave(&def_fbg_lock, flags);
	grp->window_size = window_size;
	raw_spin_unlock_irqrestore(&def_fbg_lock, flags);

	grp = &sf_composition_group;
	raw_spin_lock_irqsave(&sf_fbg_lock, flags);
	grp->window_size = window_size;
	raw_spin_unlock_irqrestore(&sf_fbg_lock, flags);

	grp = &game_frame_boost_group;
	raw_spin_lock_irqsave(&game_fbg_lock, flags);
	grp->window_size = window_size;
	raw_spin_unlock_irqrestore(&game_fbg_lock, flags);
}
EXPORT_SYMBOL_GPL(set_frame_group_window_size);

#define DIV64_U64_ROUNDUP(X, Y) div64_u64((X) + (Y - 1), Y)
static inline u64 scale_exec_time(u64 delta, struct rq *rq)
{
	u64 task_exec_scale;
	unsigned int cur_freq, max_freq;
	int cpu = cpu_of(rq);

	/* TODO:
	 * Use freq_avg instead of freq_cur, because freq may trans when task running.
	 * Can we use this hook trace_android_rvh_cpufreq_transition?
	 */
	cur_freq = get_cur_freq(cpu);
	max_freq = get_max_freq(cpu);

	if (unlikely(cur_freq <= 0) || unlikely(max_freq <= 0) || unlikely(cur_freq > max_freq)) {
		ofb_err("cpu=%d cur_freq=%lu max_freq=%lu\n", cpu, cur_freq, max_freq);
		return delta;
	}

	task_exec_scale = DIV64_U64_ROUNDUP(cur_freq *
				arch_scale_cpu_capacity(cpu),
				max_freq);

	return (delta * task_exec_scale) >> 10;
}

static s64 update_window_start(u64 wallclock, struct frame_group *grp, int group_id)
{
	s64 delta;

	frame_grp_with_lock_assert(grp);

	delta = wallclock - grp->window_start;

	if (delta <= 0) {
		ofb_debug("wallclock=%llu is lesser than window_start=%llu, group_id=%d",
			wallclock, grp->window_start, group_id);
		return delta;
	}

	grp->window_start = wallclock;
	grp->prev_window_size = grp->window_size;
	grp->window_busy = (grp->curr_window_exec * 100) / delta;

	if (unlikely(sysctl_frame_boost_debug)) {
		static unsigned long main_count, sf_count;
		unsigned long *count;
		char *msg;

		trace_printk("window_start=%llu window_size=%llu delta=%lld group_id=%d\n",
			grp->window_start, grp->window_size, delta, group_id);

		if (group_id == SF_FRAME_GROUP_ID) {
			count = &sf_count;
			msg = "sf";
		} else if (group_id == DEFAULT_FRAME_GROUP_ID) {
			count = &main_count;
			msg = "main";
		} else {
			return delta;
		}

		*count += 1;
		val_systrace_c((*count)%2, msg);
	}

	return delta;
}

static void update_group_exectime(struct frame_group *grp, int group_id)
{
	frame_grp_with_lock_assert(grp);

	grp->prev_window_scale = grp->curr_window_scale;
	grp->curr_window_scale = 0;
	grp->prev_window_exec = grp->curr_window_exec;
	grp->curr_window_exec = 0;

	if (unlikely(sysctl_frame_boost_debug) && (group_id == DEFAULT_FRAME_GROUP_ID)) {
		val_systrace_c(grp->prev_window_exec, "prev_window_exec");
		val_systrace_c(get_frame_putil(grp->prev_window_scale, grp->frame_zone),
			"prev_window_util");
		val_systrace_c(grp->curr_window_exec, "curr_window_exec");
		val_systrace_c(get_frame_putil(grp->curr_window_scale, grp->frame_zone),
			"curr_window_util");
	}
}

static void update_util_before_window_rollover(int group_id);
int rollover_frame_group_window(int group_id)
{
	u64 wallclock;
	unsigned long flags;
	raw_spinlock_t *lock = NULL;
	struct frame_group *grp;

	update_util_before_window_rollover(group_id);

	if (group_id == SF_FRAME_GROUP_ID) {
		grp = &sf_composition_group;
		lock = &sf_fbg_lock;
	} else if (group_id == GAME_FRAME_GROUP_ID) {
		grp = &game_frame_boost_group;
		lock = &game_fbg_lock;
	} else { /* DEFAULT_FRAME_GROUP_ID */
		grp = &default_frame_boost_group;
		lock = &def_fbg_lock;
	}

	raw_spin_lock_irqsave(lock, flags);

	wallclock = fbg_ktime_get_ns();
	update_window_start(wallclock, grp, group_id);

	if (unlikely(sysctl_frame_boost_debug) && (group_id == DEFAULT_FRAME_GROUP_ID))
		val_systrace_c(get_frame_rate(), "framerate");

	/* We set curr_window_* as prev_window_* and clear curr_window_*,
	 * but prev_window_* now may belong to old_frame_app, and curr_window_*
	 * belong to new_frame_app, when called from ioctl(BOOST_MOVE_FG).
	 */
	update_group_exectime(grp, group_id);

	raw_spin_unlock_irqrestore(lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(rollover_frame_group_window);

/**************************************
 * cpu frequence adjust for frame group
 ***************************************/
static void update_frame_zone(struct frame_group *grp, u64 wallclock)
{
	s64 delta;

	grp->frame_zone = 0;

	delta = wallclock - grp->window_start;
	if (delta <= (2 * grp->window_size))
		grp->frame_zone |= FRAME_ZONE;

#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	if (sysctl_slide_boost_enabled || sysctl_input_boost_enabled)
		grp->frame_zone |= USER_ZONE;
#endif

	if (!grp->frame_zone)
		set_frame_state(FRAME_END);

	if (unlikely(sysctl_frame_boost_debug)) {
		if (grp == &sf_composition_group)
			sf_zone_systrace_c(grp->frame_zone);
		else
			def_zone_systrace_c(grp->frame_zone);
	}
}

extern struct reciprocal_value reciprocal_value(u32 d);
struct reciprocal_value schedtune_spc_rdiv;
static long schedtune_margin(unsigned long util, long boost)
{
	long long margin = 0;

	/*
	 * Signal proportional compensation (SPC)
	 *
	 * The Boost (B) value is used to compute a Margin (M) which is
	 * proportional to the complement of the original Signal (S):
	 *   M = B * (SCHED_CAPACITY_SCALE - S)
	 * The obtained M could be used by the caller to "boost" S.
	 */
	if (boost >= 0) {
		margin = SCHED_CAPACITY_SCALE - util;
		margin *= boost;
	} else
		margin = -util * boost;

	margin = reciprocal_divide(margin, schedtune_spc_rdiv);

	if (boost < 0)
		margin *= -1;

	return margin;
}

static long schedtune_grp_margin(unsigned long util, int stune_boost)
{
	if (stune_boost == 0 || util == 0)
		return 0;

	return schedtune_margin(util, stune_boost);
}

static struct oplus_sched_cluster *best_cluster(struct frame_group *grp)
{
	int cpu;
	unsigned long max_cap = 0, cap = 0, best_cap = 0;
	struct oplus_sched_cluster *cluster = NULL, *max_cluster = NULL, *best_cluster = NULL;
	unsigned long util = grp->policy_util;
	unsigned long boosted_grp_util = util;

	if (grp == &default_frame_boost_group)
		boosted_grp_util += schedtune_grp_margin(util, stune_boost[BOOST_DEF_MIGR]);

	for_each_sched_cluster(cluster) {
		cpu = cpumask_first(&cluster->cpus);
		cap = capacity_orig_of(cpu);
		/* We sort cluster list by using arch_scale_cpu_capacity() when
		 * build_clusters(). But here we consider freqlimit case and use
		 * capacity_orig_of() to find the max cluster
		 */
		if (cap > max_cap) {
			max_cap = cap;
			max_cluster = cluster;
		}

		if (boosted_grp_util <= cap) {
			best_cap = cap;
			best_cluster = cluster;
			break;
		}
	}

	if (!best_cluster) {
		best_cap = max_cap;
		best_cluster = max_cluster;
	}

	/* We hope to spread frame group task, if preferred_cluster has only
	 * one core and platform has 3 clusters, try to find available_cluster
	 */
	if (num_sched_clusters <= 2) {
		grp->available_cluster = NULL;
	} else {
		if (fb_cluster[num_sched_clusters-1]->id == best_cluster->id) {
			/* if best_cluster is cpu7, then available_cluster is cpu4-6 */
			grp->available_cluster = fb_cluster[num_sched_clusters-2];

		} else if (fb_cluster[num_sched_clusters-2]->id == best_cluster->id) {
			/* if best_cluster is cpu4-6, then available_cluster is cpu7 */
			grp->available_cluster = fb_cluster[num_sched_clusters-1];
		}
	}

	if (unlikely(sysctl_frame_boost_debug)) {
		pref_cpus_systrace_c(cpumask_first(&best_cluster->cpus));
		if (grp->available_cluster)
			avai_cpus_systrace_c(cpumask_first(&grp->available_cluster->cpus));
		else
			avai_cpus_systrace_c(0);
	}

	/* Now we get preferred_cluster */
	return best_cluster;
}

static unsigned long update_freq_policy_util(struct frame_group *grp, u64 wallclock,
	unsigned int flags)
{
	unsigned long prev_putil = 0, curr_putil = 0, vutil = 0, frame_util = 0;
	u64 timeline;
	bool use_vutil = true;
	u64 check_timeline = 0;

	frame_grp_with_lock_assert(grp);
	update_frame_zone(grp, wallclock);

	if (!grp->frame_zone)
		return 0;

	timeline = wallclock - grp->window_start;
	vutil = get_frame_vutil(timeline);

	prev_putil = get_frame_putil(grp->prev_window_scale, grp->frame_zone);
	curr_putil = get_frame_putil(grp->curr_window_scale, grp->frame_zone);

	grp->curr_util = curr_putil;
	frame_util = max_t(unsigned long, prev_putil, curr_putil);

	/* We allow vendor governor's freq-query using vutil, but we only updating
	 * last_util_update_time when called from new hook update_curr()
	 */
	if (flags & SCHED_CPUFREQ_DEF_FRAMEBOOST || flags & SCHED_CPUFREQ_SF_FRAMEBOOST)
		grp->last_util_update_time = wallclock;

	/* Be carefully using vtuil */
	if (grp->frame_zone & FRAME_ZONE && grp->frame_zone & USER_ZONE) {
		if (is_high_frame_rate())
			check_timeline = grp->window_size - (grp->window_size >> 3);
		else
			check_timeline = grp->window_size - (grp->window_size >> 2);
	}

#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	if (curr_putil < (vutil >> 1))
#else
	if (timeline > check_timeline && curr_putil < (vutil >> 1))
#endif
		use_vutil = false;

	/* For rt_uinon_group, vutil is useless */
	if (grp == &sf_composition_group)
		use_vutil = false;

	/* We consider virtual util in frame zone */
	if (!(grp->frame_zone & FRAME_ZONE))
		use_vutil = false;

	if (use_vutil)
		frame_util = max_t(unsigned long, frame_util, vutil);

	if (unlikely(sysctl_frame_boost_debug)) {
		if (grp != &sf_composition_group) {
			val_systrace_c(vutil, "vutil");
			val_systrace_c(use_vutil, "use_vutil");
		}

		trace_printk("flags=%u wallclock=%llu window_start=%llu timeline=%llu prev_putil=%lu curr_util=%lu(curr_exec_util=%llu) vutil=%lu use_vutil=%d rt_grp=%d\n",
			flags, wallclock, grp->window_start, timeline, prev_putil, curr_putil,
			get_frame_putil(grp->curr_window_exec, grp->frame_zone), vutil,
			use_vutil, grp == &sf_composition_group);
	}

	return frame_uclamp(frame_util);
}

void fbg_get_frame_scale(unsigned long *frame_scale)
{
	struct frame_group *grp = &game_frame_boost_group;

	*frame_scale = grp->prev_window_scale;
}
EXPORT_SYMBOL_GPL(fbg_get_frame_scale);

void fbg_get_frame_busy(unsigned int *frame_busy)
{
	struct frame_group *grp = &game_frame_boost_group;

	*frame_busy = grp->window_busy;
}
EXPORT_SYMBOL_GPL(fbg_get_frame_busy);

void fbg_get_prev_util(unsigned long *prev_util)
{
	struct frame_group *grp = &default_frame_boost_group;

	if (!grp->frame_zone)
		return;

	*prev_util = get_frame_putil(grp->prev_window_scale, grp->frame_zone);
}
EXPORT_SYMBOL_GPL(fbg_get_prev_util);

void fbg_get_curr_util(unsigned long *curr_util)
{
	struct frame_group *grp = &default_frame_boost_group;

	if (!grp->frame_zone)
		return;

	*curr_util = get_frame_putil(grp->curr_window_scale, grp->frame_zone);
}
EXPORT_SYMBOL_GPL(fbg_get_curr_util);

bool check_putil_over_thresh(unsigned long thresh)
{
	struct frame_group *grp = &default_frame_boost_group;
	unsigned long putil = 0;

	putil = get_frame_putil(grp->curr_window_scale, FRAME_ZONE);
	return putil >= thresh;
}
EXPORT_SYMBOL_GPL(check_putil_over_thresh);

static bool valid_freq_querys(const struct cpumask *query_cpus, struct frame_group *grp)
{
	int count = 0;
	struct task_struct *p = NULL;
#if (0)
	cpumask_t grp_cpus = CPU_MASK_NONE;
#endif
	cpumask_t on_cpus = CPU_MASK_NONE;
	struct rq *rq;
	struct oplus_task_struct *ots = NULL;
	u64 now = fbg_ktime_get_ns();
	int cpu;

	frame_grp_with_lock_assert(grp);

	if (list_empty(&grp->tasks))
		return false;

	if ((now - grp->last_util_update_time) >= (2 * grp->window_size))
		return false;

#if (0)
	cpumask_copy(&grp_cpus, &grp->preferred_cluster->cpus);
	if (grp->available_cluster)
		cpumask_or(&grp_cpus, &grp_cpus, &grp->available_cluster->cpus);

	if (!cpumask_intersects(query_cpus, &grp_cpus))
		return false;
#endif

	/* Make sure our group task is running on query_cpus now,
	 * otherwise we don't need to update freq.
	 */
	list_for_each_entry(ots, &grp->tasks, fbg_list) {
		p = ots_to_ts(ots);

		cpu = task_cpu(p);
		rq = cpu_rq(cpu);
		if (task_running(rq, p))
			cpumask_set_cpu(task_cpu(p), &on_cpus);

		/* detect infinite loop */
		BUG_ON(++count > 1000);
	}

	return cpumask_intersects(&on_cpus, query_cpus);
}

bool fbg_freq_policy_util(unsigned int policy_flags, const struct cpumask *query_cpus,
	unsigned long *util)
{
	unsigned long flags;
	unsigned long policy_util = 0, raw_util = *util;
	struct frame_group *grp = NULL;
	unsigned long boosted_policy_util = 0;
	u64 wallclock = fbg_ktime_get_ns();
	unsigned int first_cpu = cpumask_first(query_cpus);

	if (!__frame_boost_enabled())
		return false;

	/* Adjust governor util with default_frame_boost_group's policy util */
	grp = &default_frame_boost_group;

	raw_spin_lock_irqsave(&def_fbg_lock, flags);

	if (!grp->preferred_cluster)
		goto unlock_fbg;

	/* We should always check if group task running on query_cpus. If preferred_clusters
	 * just update recently and group task has completed the migration. We will clear
	 * prev_preferred's policy util from default_group_update_cpufreq()
	 * with need_update_prev_freq=true.
	 */
	if (valid_freq_querys(query_cpus, grp)) {
		/* We allow cfs group used vutil, so we should always update vtuil no matter
		 * it's vendor governor's query or frame boost group's query.
		 */
		if (!(policy_flags & SCHED_CPUFREQ_DEF_FRAMEBOOST) &&
			!(policy_flags & SCHED_CPUFREQ_SF_FRAMEBOOST))
			grp->policy_util = update_freq_policy_util(grp, wallclock, policy_flags);

		policy_util = grp->policy_util;
		boosted_policy_util = policy_util +
			schedtune_grp_margin(policy_util, stune_boost[BOOST_DEF_FREQ]);
	}

unlock_fbg:
	raw_spin_unlock_irqrestore(&def_fbg_lock, flags);
	*util = max(raw_util, boosted_policy_util);

	if (unlikely(sysctl_frame_boost_debug)) {
		cpu_util_systrace_c(boosted_policy_util, first_cpu, "cfs_policy_util");
		cpu_util_systrace_c(policy_util ? grp->curr_util : 0, first_cpu,  "cfs_curr_util");
	}

	/* Adjust governor util with sf_composition_group's policy util */
	grp = &sf_composition_group;

	raw_spin_lock_irqsave(&sf_fbg_lock, flags);

	policy_util = 0;
	boosted_policy_util = 0;
	if (valid_freq_querys(query_cpus, grp)) {
		policy_util = grp->policy_util;
		boosted_policy_util = policy_util +
			schedtune_grp_margin(policy_util, stune_boost[BOOST_SF_FREQ]);
	}

	raw_spin_unlock_irqrestore(&sf_fbg_lock, flags);
	*util = max(*util, boosted_policy_util);

	if (unlikely(sysctl_frame_boost_debug)) {
		cpu_util_systrace_c(boosted_policy_util, first_cpu, "rt_policy_util");
		cpu_util_systrace_c(policy_util ? grp->curr_util : 0, first_cpu, "rt_curr_util");
		cpu_util_systrace_c(raw_util, first_cpu, "raw_util");
	}

	return (raw_util != *util);
}
EXPORT_SYMBOL_GPL(fbg_freq_policy_util);

static inline bool should_update_cpufreq(u64 wallclock, struct frame_group *grp,
	raw_spinlock_t *lock)
{
	s64 delta = 0;

	lockdep_assert_held(lock);

	if (list_empty(&grp->tasks))
		return false;

	delta = wallclock - grp->last_freq_update_time;
	if (delta < DEFAULT_FREQ_UPDATE_MIN_INTERVAL)
		return false;

	return true;
}

static inline void cpufreq_update_util_wrap(struct rq *rq, unsigned int flags)
{
	unsigned long lock_flags;

	raw_spin_lock_irqsave(&freq_protect_lock, lock_flags);
	cpufreq_update_util(rq, flags);
	raw_spin_unlock_irqrestore(&freq_protect_lock, lock_flags);
}

bool default_group_update_cpufreq(void)
{
	struct frame_group *grp = &default_frame_boost_group;
	unsigned long flags;
	bool ret = false;
	bool need_update_prev_freq = false;
	bool need_update_next_freq = false;
	int prev_cpu, next_cpu;
	struct oplus_sched_cluster *preferred_cluster = NULL;
	struct rq *rq = NULL;
	u64 wallclock = fbg_ktime_get_ns();

	raw_spin_lock_irqsave(&def_fbg_lock, flags);

	if (list_empty(&grp->tasks))
		goto unlock;

	grp->policy_util = update_freq_policy_util(grp, wallclock, SCHED_CPUFREQ_DEF_FRAMEBOOST);
	/*
	 *Update frame group preferred cluster before updating cpufreq,
	 * so we can make decision target cluster.
	 */
	preferred_cluster = best_cluster(grp);
	if (!grp->preferred_cluster)
		grp->preferred_cluster = preferred_cluster;
	else if (grp->preferred_cluster != preferred_cluster) {
		prev_cpu = cpumask_first(&grp->preferred_cluster->cpus);
		grp->preferred_cluster = preferred_cluster;
		/*
		 * Once preferred_cluster changed, update prev_cluster's cpufreq without any limit.
		 * And then get_freq_policy_util() will return 0 in this update call.
		 */
		need_update_prev_freq = true;
		ret = true;
	}
	next_cpu = cpumask_first(&grp->preferred_cluster->cpus);

	if (should_update_cpufreq(wallclock, grp, &def_fbg_lock)) {
		grp->last_freq_update_time = wallclock;
		need_update_next_freq = true;
	}

unlock:
	raw_spin_unlock_irqrestore(&def_fbg_lock, flags);

	if (need_update_prev_freq) {
		rq = cpu_rq(prev_cpu);
		if (fbg_hook.update_freq)
			fbg_hook.update_freq(rq, SCHED_CPUFREQ_DEF_FRAMEBOOST);
		else
			cpufreq_update_util_wrap(rq, SCHED_CPUFREQ_DEF_FRAMEBOOST);
	}

	if (need_update_next_freq) {
		rq = cpu_rq(next_cpu);
		if (fbg_hook.update_freq)
			fbg_hook.update_freq(rq, SCHED_CPUFREQ_DEF_FRAMEBOOST);
		else
			cpufreq_update_util_wrap(rq, SCHED_CPUFREQ_DEF_FRAMEBOOST);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(default_group_update_cpufreq);

bool sf_composition_update_cpufreq(struct task_struct *tsk)
{
	struct frame_group *grp = &sf_composition_group;
	unsigned long flags;
	bool ret = false;
	bool need_update = false;
	struct rq *rq = NULL;
	u64 wallclock = fbg_ktime_get_ns();

	raw_spin_lock_irqsave(&sf_fbg_lock, flags);

	if (list_empty(&grp->tasks))
		goto unlock;

	grp->policy_util = update_freq_policy_util(grp, wallclock, SCHED_CPUFREQ_SF_FRAMEBOOST);

	if (should_update_cpufreq(wallclock, grp, &sf_fbg_lock)) {
		grp->last_freq_update_time = wallclock;
		need_update = true;
	}

unlock:
	raw_spin_unlock_irqrestore(&sf_fbg_lock, flags);

	if (need_update) {
		rq = task_rq(tsk);
		if (fbg_hook.update_freq)
			fbg_hook.update_freq(rq, SCHED_CPUFREQ_SF_FRAMEBOOST);
		else
			cpufreq_update_util_wrap(rq, SCHED_CPUFREQ_SF_FRAMEBOOST);
	}

	return ret;
}

/*
 * update_frame_group_util - update frame group utility if the task is drawing frame
 * @task: task that is updating running time.
 * @delta: running time is nano second
 *       the other paramenter is unused
 */
static void update_frame_group_util(struct task_struct *p, u64 running,
	u64 wallclock, bool default_part, struct frame_group *grp)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(p);
	u64 adjusted_running;
	u64 window_start;
	u64 delta_wc_ws;
	u64 prev_exec, exec_scale;
	struct rq *rq = task_rq(p);

	if (IS_ERR_OR_NULL(ots))
		return;

	frame_grp_with_lock_assert(grp);

	window_start = grp->window_start;
	if (unlikely(wallclock < window_start)) {
		ofb_debug("failed to update util with wc=%llu ws=%llu\n",
			wallclock,
			window_start);
		return;
	}

	delta_wc_ws = wallclock - window_start;

	/*
	 * adjust the running time, for serial load track.
	 * only adjust STATIC_FRAME_TASK tasks, not BINDER_FRAME_TASK tasks,
	 * matched with the logic of update_group_nr_running().
	 */
	if (ots->fbg_state & STATIC_FRAME_TASK) {
		if (grp->mark_start <= 0)
			return;

		adjusted_running = wallclock - grp->mark_start;
		if (unlikely(adjusted_running <= 0)) {
			ofb_debug("adjusted_running <= 0 with wc=%llu ms=%llu\n",
				wallclock, grp->mark_start);
			return;
		}

		if (unlikely(sysctl_frame_boost_debug)) {
			trace_printk("raw_running=%llu, adjusted_running=%llu,"
				" old_mark_start=%llu, new_mark_start=%llu\n",
				running, adjusted_running, grp->mark_start, wallclock);
		}

		grp->mark_start = wallclock;
		running = adjusted_running;
	}

	if (running <= 0)
		return;

	/* Per group load tracking in FBG */
	if (likely(delta_wc_ws >= running)) {
		grp->curr_window_exec += running;

		exec_scale = scale_exec_time(running, rq);
		grp->curr_window_scale += exec_scale;
		if (unlikely(sysctl_frame_boost_debug) && default_part) {
			val_systrace_c(grp->curr_window_exec, "curr_window_exec");
			val_systrace_c(get_frame_putil(grp->curr_window_scale, grp->frame_zone),
				"curr_window_util");
		}
	} else {
		/* Prev window group statistic */
		prev_exec = running - delta_wc_ws;
		grp->prev_window_exec += prev_exec;

		exec_scale = scale_exec_time(prev_exec, rq);
		grp->prev_window_scale += exec_scale;

		/* Curr window group statistic */
		grp->curr_window_exec += delta_wc_ws;

		exec_scale = scale_exec_time(delta_wc_ws, rq);
		grp->curr_window_scale += exec_scale;

		if (unlikely(sysctl_frame_boost_debug) && default_part) {
			val_systrace_c(grp->prev_window_exec, "prev_window_exec");
			val_systrace_c(get_frame_putil(grp->prev_window_scale, grp->frame_zone), "prev_window_util");
			val_systrace_c(grp->curr_window_exec, "curr_window_exec");
			val_systrace_c(get_frame_putil(grp->curr_window_scale, grp->frame_zone), "curr_window_util");
		}
	}

	grp->last_util_update_time = wallclock;
}

static inline void fbg_update_task_util(struct task_struct *tsk, u64 runtime,
	bool need_freq_update)
{
	struct frame_group *grp = NULL;
	struct oplus_task_struct *ots = NULL;
	raw_spinlock_t *lock = NULL;
	unsigned long flags;
	u64 wallclock;
	bool composition_part = false;
	bool default_part = false;

	ots = get_oplus_task_struct(tsk);
	if (IS_ERR_OR_NULL(ots) || ots->fbg_state == NONE_FRAME_TASK)
		return;

	if (ots->fbg_state & FRAME_COMPOSITION) {
		composition_part = true;
		grp = &sf_composition_group;
		lock = &sf_fbg_lock;
	} else if (ots->fbg_state & FRAME_GAME) {
		grp = &game_frame_boost_group;
		lock = &game_fbg_lock;
	} else {
		default_part = true;
		grp = &default_frame_boost_group;
		lock = &def_fbg_lock;
	}

	raw_spin_lock_irqsave(lock, flags);
	/* When task update running time, doing following works:
	 * 1) update frame group util;
	 * 2) update frame group's frame zone;
	 * 3) try to update cpufreq.
	 */
	wallclock = fbg_ktime_get_ns();
	update_frame_group_util(tsk, runtime, wallclock, default_part, grp);

	raw_spin_unlock_irqrestore(lock, flags);

	if (need_freq_update) {
		if (composition_part)
			sf_composition_update_cpufreq(tsk);
		else if (default_part)
			default_group_update_cpufreq();
	}
}

static void fbg_update_cfs_util_hook(void *unused, struct task_struct *tsk,
	u64 runtime, u64 vruntime)
{
	fbg_update_task_util(tsk, runtime, true);
}

static void fbg_update_rt_util_hook(void *unused, struct task_struct *tsk, u64 runtime)
{
	fbg_update_task_util(tsk, runtime, true);
}

enum task_event {
	PUT_PREV_TASK	= 0,
	PICK_NEXT_TASK	= 1,
};

/*
 * Update the number of running threads in the group.
 *
 * If thread belonging to a group start running, nr_running of group +1.
 * If thread belonging to a group stop running, nr_running of group -1.
 *
 * We only consider STATIC_FRAME_TASK tasks, not BINDER_FRAME_TASK tasks,
 * because same BINDER_FRAME_TASK tasks bouncing between different group.
 *
 * When nr_running form 0 to 1, the mark_start set to the current time.
 */
static void update_group_nr_running(struct task_struct *p, int event)
{
	struct oplus_task_struct *ots = NULL;
	struct frame_group *grp = NULL;
	raw_spinlock_t *lock = NULL;
	int group_id;
	unsigned long flags;

	ots = get_oplus_task_struct(p);
	if (IS_ERR_OR_NULL(ots))
		return;

	grp = task_get_frame_group(ots);
	lock = task_get_frame_group_lock(ots);
	group_id = get_frame_group_id(grp);

	raw_spin_lock_irqsave(lock, flags);
	if (event == PICK_NEXT_TASK && (ots->fbg_state & STATIC_FRAME_TASK)) {
		ots->fbg_running = true;
		grp->nr_running++;
		if (grp->nr_running == 1)
			grp->mark_start = max(grp->mark_start, fbg_ktime_get_ns());
		if (unlikely(sysctl_frame_boost_debug)) {
			trace_printk("group_id=%d, next->comm=%s, next->tid=%d, nr_running=%d, mark_start=%llu\n",
				group_id, p->comm, p->pid, grp->nr_running, grp->mark_start);
		}
	} else if (event == PUT_PREV_TASK && ots->fbg_running) {
		ots->fbg_running = false;
		grp->nr_running--;
		if (unlikely(grp->nr_running < 0))
			grp->nr_running = 0;
		if (unlikely(sysctl_frame_boost_debug)) {
			trace_printk("group_id=%d, prev->comm=%s, prev->tid=%d, nr_running=%d\n",
				group_id, p->comm, p->pid, grp->nr_running);
		}
	}
	raw_spin_unlock_irqrestore(lock, flags);
}

void fbg_android_rvh_schedule_handler(struct task_struct *prev,
	struct task_struct *next, struct rq *rq)
{
	if (atomic_read(&fbg_initialized) == 0)
		return;

	if (unlikely(prev == next))
		return;

	/* prev task */
	update_group_nr_running(prev, PUT_PREV_TASK);
	/* next task */
	update_group_nr_running(next, PICK_NEXT_TASK);
}
EXPORT_SYMBOL_GPL(fbg_android_rvh_schedule_handler);

void fbg_android_rvh_cpufreq_transition(struct cpufreq_policy *policy)
{
	struct task_struct *curr_task;
	struct rq *rq;
	int cpu;

	if (atomic_read(&fbg_initialized) == 0)
		return;

	for_each_cpu(cpu, policy->cpus) {
		rq = cpu_rq(cpu);

		rcu_read_lock();
		curr_task = rcu_dereference(rq->curr);
		if (curr_task)
			get_task_struct(curr_task);
		rcu_read_unlock();

		if (curr_task) {
			fbg_update_task_util(curr_task, 0, false);
			put_task_struct(curr_task);
		}
	}
}
EXPORT_SYMBOL_GPL(fbg_android_rvh_cpufreq_transition);

static void update_util_before_window_rollover(int group_id)
{
	struct oplus_sched_cluster *cluster;
	struct task_struct *curr_task;
	struct rq *rq;
	int cpu;

	if (group_id != GAME_FRAME_GROUP_ID)
		return;

	for_each_sched_cluster(cluster) {
		for_each_cpu(cpu, &cluster->cpus) {
			rq = cpu_rq(cpu);

			rcu_read_lock();
			curr_task = rcu_dereference(rq->curr);
			if (curr_task)
				get_task_struct(curr_task);
			rcu_read_unlock();

			if (curr_task) {
				fbg_update_task_util(curr_task, 0, false);
				put_task_struct(curr_task);
			}
		}
	}
}

/*********************************
 * task placement for frame group
 *********************************/
/* If task util arrive (max * 80%), it's misfit */
#define fits_capacity(util, max)	((util) * 1280 < (max) * 1024)

/*
 * group_task_fits_cluster_cpus - check if frame group preferred cluster is suitable for
 *             frame task
 *
 * We should know that our preferred cluster comes from util-tracing with frame window,
 * which may not fit original load-tracing with larger window size.
 */
static bool group_task_fits_cluster_cpus(struct task_struct *tsk,
	struct oplus_sched_cluster *cluster)
{
	/* If group task prefer silver core, just let it go */
	if (!cluster || !cluster->id)
		return false;

	return true;
}

inline unsigned long capacity_of(int cpu)
{
	return cpu_rq(cpu)->cpu_capacity;
}

#define lsub_positive(_ptr, _val) do {				\
	typeof(_ptr) ptr = (_ptr);				\
	*ptr -= min_t(typeof(*ptr), *ptr, _val);		\
} while (0)

static inline unsigned long cpu_util(int cpu)
{
	struct cfs_rq *cfs_rq;
	unsigned int util;

	cfs_rq = &cpu_rq(cpu)->cfs;
	util = READ_ONCE(cfs_rq->avg.util_avg);

	if (sched_feat(UTIL_EST))
		util = max(util, READ_ONCE(cfs_rq->avg.util_est.enqueued));

	return min_t(unsigned long, util, capacity_orig_of(cpu));
}

static inline unsigned long task_util(struct task_struct *p)
{
	return READ_ONCE(p->se.avg.util_avg);
}

static inline unsigned long _task_util_est(struct task_struct *p)
{
	struct util_est ue = READ_ONCE(p->se.avg.util_est);

	return max(ue.ewma, (ue.enqueued & ~UTIL_AVG_UNCHANGED));
}

unsigned long cpu_util_without(int cpu, struct task_struct *p)
{
	struct cfs_rq *cfs_rq;
	unsigned int util;

	/* Task has no contribution or is new */
	if (cpu != task_cpu(p) || !READ_ONCE(p->se.avg.last_update_time))
		return cpu_util(cpu);

	cfs_rq = &cpu_rq(cpu)->cfs;
	util = READ_ONCE(cfs_rq->avg.util_avg);

	/* Discount task's util from CPU's util */
	lsub_positive(&util, task_util(p));

	/*
	 * Covered cases:
	 *
	 * a) if *p is the only task sleeping on this CPU, then:
	 *      cpu_util (== task_util) > util_est (== 0)
	 *    and thus we return:
	 *      cpu_util_without = (cpu_util - task_util) = 0
	 *
	 * b) if other tasks are SLEEPING on this CPU, which is now exiting
	 *    IDLE, then:
	 *      cpu_util >= task_util
	 *      cpu_util > util_est (== 0)
	 *    and thus we discount *p's blocked utilization to return:
	 *      cpu_util_without = (cpu_util - task_util) >= 0
	 *
	 * c) if other tasks are RUNNABLE on that CPU and
	 *      util_est > cpu_util
	 *    then we use util_est since it returns a more restrictive
	 *    estimation of the spare capacity on that CPU, by just
	 *    considering the expected utilization of tasks already
	 *    runnable on that CPU.
	 *
	 * Cases a) and b) are covered by the above code, while case c) is
	 * covered by the following code when estimated utilization is
	 * enabled.
	 */
	if (sched_feat(UTIL_EST)) {
		unsigned int estimated =
			READ_ONCE(cfs_rq->avg.util_est.enqueued);

		/*
		 * Despite the following checks we still have a small window
		 * for a possible race, when an execl's select_task_rq_fair()
		 * races with LB's detach_task():
		 *
		 *   detach_task()
		 *     p->on_rq = TASK_ON_RQ_MIGRATING;
		 *     ---------------------------------- A
		 *     deactivate_task()                   \
		 *       dequeue_task()                     + RaceTime
		 *         util_est_dequeue()              /
		 *     ---------------------------------- B
		 *
		 * The additional check on "current == p" it's required to
		 * properly fix the execl regression and it helps in further
		 * reducing the chances for the above race.
		 */
		if (unlikely(task_on_rq_queued(p) || current == p))
			lsub_positive(&estimated, _task_util_est(p));

		util = max(util, estimated);
	}

	/*
	 * Utilization (estimated) can exceed the CPU capacity, thus let's
	 * clamp to the maximum CPU capacity to ensure consistency with
	 * the cpu_util call.
	 */
	return min_t(unsigned long, util, capacity_orig_of(cpu));
}

static bool task_is_rt(struct task_struct *task)
{
	BUG_ON(!task);

	/* valid RT priority is 0..MAX_RT_PRIO-1 */
	if ((task->prio <= MAX_RT_PRIO-1) && (task->prio >= 0))
		return true;

	return false;
}

bool set_frame_group_task_to_perfer_cpu(struct task_struct *p, int *target_cpu)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(p);
	struct frame_group *grp = &default_frame_boost_group;
	struct oplus_sched_cluster *cluster = NULL;
	cpumask_t search_cpus = CPU_MASK_NONE;
	unsigned long spare_cap = 0, max_spare_cap = 0;
	int iter_cpu;
	int max_spare_cap_cpu = -1, backup_cpu = -1;
	bool ret = false;
	bool use_avail_cls = true;
	int fb_num = 0;
	int cpu_nums = 0;
	bool walk_next_cls = false;

	if (IS_ERR_OR_NULL(ots))
		return false;

	if (!__frame_boost_enabled())
		return false;

	if (fbg_cluster_boost(p, target_cpu))
		return true;

	/* Some threads created before moduler working, just init them here. */
	if (ots->fbg_list.prev == 0 && ots->fbg_list.next == 0) {
		ots->fbg_state = NONE_FRAME_TASK;
		ots->fbg_depth = INVALID_FBG_DEPTH;
		INIT_LIST_HEAD(&ots->fbg_list);
	}

	if (!ots->fbg_state)
		return false;

	cluster = grp->preferred_cluster;
	if (!group_task_fits_cluster_cpus(p, cluster))
		return false;

	/* Note that *target_cpu maybe invalid */
	if ((*target_cpu > 0) && (*target_cpu < num_possible_cpus())
		&& cpumask_test_cpu(*target_cpu, &cluster->cpus))
		return false;

retry:
	cpumask_and(&search_cpus, p->cpus_ptr, cpu_online_mask);
#ifdef CONFIG_OPLUS_ADD_CORE_CTRL_MASK
	if (fbg_cpu_halt_mask)
		cpumask_andnot(&search_cpus, &search_cpus, fbg_cpu_halt_mask);
#endif /* CONFIG_OPLUS_ADD_CORE_CTRL_MASK */
	cpumask_and(&search_cpus, &search_cpus, &cluster->cpus);
	cpu_nums = cpumask_weight(&search_cpus);

	for_each_cpu(iter_cpu, &search_cpus) {
		struct rq *rq = NULL;
		struct task_struct *curr = NULL;

		rq = cpu_rq(iter_cpu);
		curr = rq->curr;
		if (curr) {
			struct oplus_task_struct *ots_curr = get_oplus_task_struct(curr);

			/* Avoid puting group task on the same cpu */
			if (!IS_ERR_OR_NULL(ots_curr) && ots_curr->fbg_state) {
				if ((backup_cpu == -1) && task_is_rt(curr))
					backup_cpu = iter_cpu;

				fb_num++;

				if (fb_num == cpu_nums)
					walk_next_cls = true;

				continue;
			}
		}


		backup_cpu = iter_cpu;
		walk_next_cls = false;

		if (available_idle_cpu(iter_cpu)
			|| (iter_cpu == task_cpu(p) && p->__state == TASK_RUNNING)) {
			*target_cpu = iter_cpu;
			ret = true;
			goto out;
		}

		spare_cap = max_t(long, capacity_of(iter_cpu) - cpu_util_without(iter_cpu, p), 0);
		if (spare_cap > max_spare_cap) {
			max_spare_cap = spare_cap;
			max_spare_cap_cpu = iter_cpu;
		}
	}

	if (max_spare_cap_cpu != -1) {
		*target_cpu = max_spare_cap_cpu;
		ret = true;
	} else if (!walk_next_cls && backup_cpu != -1) {
		*target_cpu = backup_cpu;
		ret = true;
	}

	if (!ret && walk_next_cls && grp->available_cluster && use_avail_cls) {
		cluster = grp->available_cluster;
		cpumask_clear(&search_cpus);
		use_avail_cls = false;
		walk_next_cls = false;
		fb_num = 0;
		goto retry;
	}

out:
	return ret;
}
EXPORT_SYMBOL_GPL(set_frame_group_task_to_perfer_cpu);

/*
 * fbg_need_up_migration - check if frame group task @p fits this cpu @rq
 *
 * This function is only used for default_frame_boost_group.
 */
bool fbg_need_up_migration(struct task_struct *p, struct rq *rq)
{
	unsigned long cpu_capacity = capacity_orig_of(cpu_of(rq));
	struct frame_group *grp = NULL;
	struct oplus_sched_cluster *cluster = NULL;
	struct oplus_task_struct *ots = NULL;
	unsigned long flags;

	if (!__frame_boost_enabled())
		return false;

	ots = get_oplus_task_struct(p);
	if (IS_ERR_OR_NULL(ots) || !ots->fbg_state || ots->fbg_state & (FRAME_COMPOSITION | FRAME_GAME))
		return false;

	grp = &default_frame_boost_group;
	raw_spin_lock_irqsave(&def_fbg_lock, flags);
	cluster = grp->preferred_cluster;
	raw_spin_unlock_irqrestore(&def_fbg_lock, flags);

	return group_task_fits_cluster_cpus(p, cluster) &&
		(cpu_capacity < capacity_orig_of(cpumask_first(&cluster->cpus)));
}
EXPORT_SYMBOL_GPL(fbg_need_up_migration);

/*
 * fbg_skip_migration - check if frame group task @p can be migrated from src_cpu
 *             to dst_cpu
 *
 * This function is only used for default_frame_boost_group.
 */
bool fbg_skip_migration(struct task_struct *tsk, int src_cpu, int dst_cpu)
{
	struct oplus_sched_cluster *cluster = NULL;
	struct frame_group *grp = NULL;
	struct oplus_task_struct *ots = NULL, *dst_ots = NULL;
	struct rq *dst_rq = cpu_rq(dst_cpu);
	unsigned long flags;

	if (!__frame_boost_enabled())
		return false;

	ots = get_oplus_task_struct(tsk);
	if (IS_ERR_OR_NULL(ots) || !ots->fbg_state || ots->fbg_state & (FRAME_COMPOSITION | FRAME_GAME))
		return false;

	dst_ots = get_oplus_task_struct(dst_rq->curr);
	if (!IS_ERR_OR_NULL(dst_ots) && dst_ots->fbg_state)
		return true;

	grp = &default_frame_boost_group;
	raw_spin_lock_irqsave(&def_fbg_lock, flags);
	cluster = grp->preferred_cluster;
	raw_spin_unlock_irqrestore(&def_fbg_lock, flags);

	if (!group_task_fits_cluster_cpus(tsk, cluster))
		return false;

	return capacity_orig_of(dst_cpu) < capacity_orig_of(cpumask_first(&cluster->cpus));
}
EXPORT_SYMBOL_GPL(fbg_skip_migration);

bool fbg_rt_task_fits_capacity(struct task_struct *tsk, int cpu)
{
	struct oplus_task_struct *ots = NULL;
	struct frame_group *grp = NULL;
	unsigned long grp_util = 0, raw_util = 0;
	bool fits = true;
	u64 now = fbg_ktime_get_ns();

	if (!__frame_boost_enabled())
		return true;

	ots = get_oplus_task_struct(tsk);
	if (IS_ERR_OR_NULL(ots))
		return true;

	/* Some threads created before moduler working, just init them here. */
	if (ots->fbg_list.prev == 0 && ots->fbg_list.next == 0) {
		ots->fbg_state = NONE_FRAME_TASK;
		ots->fbg_depth = INVALID_FBG_DEPTH;
		INIT_LIST_HEAD(&ots->fbg_list);
	}

	if (!(ots->fbg_state & FRAME_COMPOSITION))
		return true;

	grp = &sf_composition_group;
	if (!grp->frame_zone || (now - grp->last_util_update_time) >= (2 * grp->window_size))
		return true;

	raw_util = grp->policy_util;
	grp_util = raw_util + schedtune_grp_margin(raw_util, stune_boost[BOOST_SF_MIGR]);

	fits = capacity_orig_of(cpu) >= grp_util;

	if (unlikely(sysctl_frame_boost_debug))
		trace_printk("comm=%-12s pid=%d tgid=%d cpu=%d grp_util=%llu raw_util=%lu cpu_cap=%lu fits=%d\n",
			tsk->comm, tsk->pid, tsk->tgid, cpu, grp_util, raw_util,
			capacity_orig_of(cpu), fits);

	return fits;
}
EXPORT_SYMBOL_GPL(fbg_rt_task_fits_capacity);

bool fbg_skip_rt_sync(struct rq *rq, struct task_struct *p, bool *sync)
{
	int cpu = cpu_of(rq);

	if (*sync && !fbg_rt_task_fits_capacity(p, cpu)) {
		*sync = false;
		return true;
	}

	return false;
}
EXPORT_SYMBOL_GPL(fbg_skip_rt_sync);

/*********************************
 * frame group initialize
 *********************************/
static void fbg_flush_task_hook(void *unused, struct task_struct *tsk)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(tsk);
	unsigned long flags;
	raw_spinlock_t *lock = NULL;

	if (IS_ERR_OR_NULL(ots))
		return;

	lock = task_get_frame_group_lock(ots);

	raw_spin_lock_irqsave(lock, flags);
	/* game group task also removed here */
	remove_task_from_frame_group(tsk);
	remove_binder_from_frame_group(tsk);
	raw_spin_unlock_irqrestore(lock, flags);
}

static void fbg_sched_fork_hook(void *unused, struct task_struct *tsk)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(tsk);

	if (IS_ERR_OR_NULL(ots))
		return;

	ots->fbg_state = NONE_FRAME_TASK;
	ots->fbg_depth = INVALID_FBG_DEPTH;
	ots->fbg_running = false;
	ots->preferred_cluster_id = -1;
	INIT_LIST_HEAD(&ots->fbg_list);
	raw_spin_lock_init(&ots->fbg_list_entry_lock);
}

void fbg_add_update_freq_hook(void (*func)(struct rq *rq, unsigned int flags))
{
	if (fbg_hook.update_freq == NULL)
		fbg_hook.update_freq = func;
}
EXPORT_SYMBOL_GPL(fbg_add_update_freq_hook);


bool fbg_is_ed_task(struct task_struct *tsk, u64 wall_clock)
{
	struct oplus_task_struct *ots = NULL;
	u64 exec_time = 0;

	if (!__frame_boost_enabled() || (!(sysctl_slide_boost_enabled || sysctl_input_boost_enabled)))
		return false;

	ots = get_oplus_task_struct(tsk);
	if (IS_ERR_OR_NULL(ots) || !ots->fbg_state || ots->fbg_state & (FRAME_COMPOSITION | FRAME_GAME))
		return false;

	if (ots->last_wake_ts && wall_clock > ots->last_wake_ts)
		exec_time = wall_clock - ots->last_wake_ts;

	if (exec_time >= ed_task_boost_max_duration && exec_time < ed_task_boost_timeout_duration) {
		ed_task_boost_type = ED_TASK_BOOST_MAX;
		return true;
	} else if (exec_time >= ed_task_boost_mid_duration && exec_time < ed_task_boost_max_duration) {
		ed_task_boost_type = ED_TASK_BOOST_MID;
		return true;
	}

	return false;
}
EXPORT_SYMBOL_GPL(fbg_is_ed_task);

void update_wake_up(struct task_struct *p)
{
	struct oplus_task_struct *ots = NULL;

	ots = get_oplus_task_struct(p);
	if (IS_ERR_OR_NULL(ots))
		return;

	ots->last_wake_ts = fbg_ktime_get_ns();
}
EXPORT_SYMBOL_GPL(update_wake_up);

#ifdef CONFIG_ARCH_MEDIATEK
static void fbg_new_task_stats(void *unused, struct task_struct *p)
{
	if (!__frame_boost_enabled())
		return;

	update_wake_up(p);
}

static void fbg_try_to_wake_up(void *unused, struct task_struct *p)
{
	if (!__frame_boost_enabled())
		return;

	update_wake_up(p);
}
#endif

static void fbg_update_freq_hook(void *unused, bool preempt, struct task_struct *prev,
	struct task_struct *next)
{
	u64 fbg_wall_clock = fbg_ktime_get_ns();
	struct rq *rq = NULL;

	if (unlikely(fbg_hook.update_freq == NULL))
		return;

	if (fbg_is_ed_task(next, fbg_wall_clock)) {
		rq = cpu_rq(prev->cpu);
		fbg_hook.update_freq(rq, SCHED_CPUFREQ_EARLY_DET);
	}
}

static void fbg_update_suspend_resume(void *unused, const char *action, int val, bool start)
{
	if (!__frame_boost_enabled())
		return;

	if (!strncmp(action, "timekeeping_freeze", 18)) {
		if (start)
			fbg_suspend();
		else
			fbg_resume();
	}
}

void register_frame_group_vendor_hooks(void)
{
	/* Register vender hook in driver/android/binder.c */
#ifdef TODO_DELME
	register_android_vh_binder_proc_transaction_finish(fbg_binder_wakeup_hook, NULL);
#endif
	register_trace_android_vh_binder_restore_priority(fbg_binder_restore_priority_hook, NULL);
	register_trace_android_vh_binder_wait_for_work(fbg_binder_wait_for_work_hook, NULL);
	register_trace_android_vh_sync_txn_recvd(fbg_sync_txn_recvd_hook, NULL);

	/* Register vendor hook in fs/exec.c */
	register_trace_task_rename(task_rename_hook, NULL);

	/* Register vender hook in kernel/sched/fair.c */
	register_trace_sched_stat_runtime(fbg_update_cfs_util_hook, NULL);

	/* Register vender hook in kernel/sched/rt.c */
	register_trace_android_vh_sched_stat_runtime_rt(fbg_update_rt_util_hook, NULL);

	/* Register vender hook in kernel/sched/core.c */
	register_trace_android_rvh_flush_task(fbg_flush_task_hook, NULL);
	register_trace_android_rvh_sched_fork(fbg_sched_fork_hook, NULL);

#ifdef CONFIG_ARCH_MEDIATEK
	register_trace_android_rvh_try_to_wake_up(fbg_try_to_wake_up, NULL);
	register_trace_android_rvh_new_task_stats(fbg_new_task_stats, NULL);
#endif

	/* Register vender hook in kernel/sched/core.c */
	register_trace_sched_switch(fbg_update_freq_hook, NULL);

	/* Register vender hook in kernel/time/tick-common.c */
	register_trace_suspend_resume(fbg_update_suspend_resume, NULL);
}

int info_show(struct seq_file *m, void *v)
{
	unsigned long flags;
	struct frame_group *grp = NULL;
	struct task_struct *tsk = NULL;
	struct oplus_task_struct *ots = NULL;

	seq_puts(m, "---- DEFAULT FRAME GROUP ----\n");
	grp = &default_frame_boost_group;
	raw_spin_lock_irqsave(&def_fbg_lock, flags);
	list_for_each_entry(ots, &grp->tasks, fbg_list) {
		tsk = ots_to_ts(ots);
		seq_printf(m, "comm=%-16s  pid=%-6d  tgid=%-6d  state=%d  depth=%d\n",
			tsk->comm, tsk->pid, tsk->tgid, ots->fbg_state, ots->fbg_depth);
	}
	raw_spin_unlock_irqrestore(&def_fbg_lock, flags);

	seq_puts(m, "\n---- SF COMPOSITION GROUP ----\n");
	grp = &sf_composition_group;
	raw_spin_lock_irqsave(&sf_fbg_lock, flags);
	list_for_each_entry(ots, &grp->tasks, fbg_list) {
		tsk = ots_to_ts(ots);
		seq_printf(m, "comm=%-16s  pid=%-6d  tgid=%-6d  state=%d  depth=%d\n",
			tsk->comm, tsk->pid, tsk->tgid, ots->fbg_state, ots->fbg_depth);
	}
	raw_spin_unlock_irqrestore(&sf_fbg_lock, flags);

	seq_puts(m, "\n---- GAME FRAME GROUP ----\n");
	grp = &game_frame_boost_group;
	raw_spin_lock_irqsave(&game_fbg_lock, flags);
	list_for_each_entry(ots, &grp->tasks, fbg_list) {
		tsk = ots_to_ts(ots);
		seq_printf(m, "comm=%-16s  pid=%-6d  tgid=%-6d  state=%d  depth=%d\n",
			tsk->comm, tsk->pid, tsk->tgid, ots->fbg_state, ots->fbg_depth);
	}
	raw_spin_unlock_irqrestore(&game_fbg_lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(info_show);

#ifdef CONFIG_OPLUS_ADD_CORE_CTRL_MASK
struct cpumask *fbg_cpu_halt_mask = NULL;
void init_fbg_halt_mask(struct cpumask *halt_mask)
{
	fbg_cpu_halt_mask = halt_mask;
}
EXPORT_SYMBOL_GPL(init_fbg_halt_mask);
#endif

int frame_group_init(void)
{
	struct frame_group *grp = NULL;
	int ret = 0;
	struct oplus_sched_cluster *cluster = NULL;

	/* Default frame group initialization */
	grp = &default_frame_boost_group;
	INIT_LIST_HEAD(&grp->tasks);
	grp->window_size = NSEC_PER_SEC / DEFAULT_FRAME_RATE;
	grp->window_start = 0;
	grp->nr_running = 0;
	grp->mark_start = 0;
	grp->preferred_cluster = NULL;
	grp->available_cluster = NULL;
	ed_task_boost_type = 0;
	ed_task_boost_mid_duration = mult_frac(grp->window_size, 60, 100);
	ed_task_boost_max_duration = mult_frac(grp->window_size, 80, 100);
	ed_task_boost_timeout_duration = grp->window_size << 1;

	/* Sf composition group initialization */
	grp = &sf_composition_group;
	INIT_LIST_HEAD(&grp->tasks);
	grp->window_size = NSEC_PER_SEC / DEFAULT_FRAME_RATE;
	grp->window_start = 0;
	grp->nr_running = 0;
	grp->mark_start = 0;
	grp->preferred_cluster = NULL;
	grp->available_cluster = NULL;

	/* Game frame group initialization */
	grp = &game_frame_boost_group;
	INIT_LIST_HEAD(&grp->tasks);
	grp->window_size = NSEC_PER_SEC / DEFAULT_FRAME_RATE;
	grp->window_start = 0;
	grp->nr_running = 0;
	grp->mark_start = 0;
	grp->preferred_cluster = NULL;
	grp->available_cluster = NULL;

	schedtune_spc_rdiv = reciprocal_value(100);

	if (!build_clusters()) {
		ret = -1;
		ofb_err("failed to build sched cluster\n");
		goto out;
	}

	for_each_sched_cluster(cluster)
		ofb_debug("num_cluster=%d id=%d cpumask=%*pbl capacity=%lu num_cpus=%d\n",
			num_sched_clusters, cluster->id, cpumask_pr_args(&cluster->cpus),
			arch_scale_cpu_capacity(cpumask_first(&cluster->cpus)),
			num_possible_cpus());

	register_syscore_ops(&fbg_syscore_ops);

	atomic_set(&fbg_initialized, 1);

out:
	return ret;
}
