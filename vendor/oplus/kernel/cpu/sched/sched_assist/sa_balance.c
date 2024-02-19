// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 Oplus. All rights reserved.
 */
#include <linux/version.h>
#include <linux/plist.h>
#include <linux/sched/cputime.h>
#include <linux/cpuidle.h>
#include <trace/hooks/sched.h>
#include <kernel/sched/sched.h>

#include "sa_common.h"
#include "sa_fair.h"
#include "sa_balance.h"

/*
 * When the following macros are enabled, some debug information
 * will be output, which is very helpful for finding bugs.
 */
#ifdef DEBUG_LB
#define DEBUG_LB_TICK
#define DEBUG_LB_TICK_RUNNABLE_TIME
#define DEBUG_LB_NEWIDLE
#define DEBUG_LB_NEWIDLE_HIT
#define DEBUG_LB_RT_TICK
#define DEBUG_LB_RT_RUNNABLE_TIME
#define DEBUG_LB_EXEC_TIME
#define DEBUG_LB_TRACKME
#endif

#define OPLUS_LB_SYSTRACE_PID		9999

/*
 * The maximum exit delay allowed for latency-sensitive tasks.
 */
#define OPLUS_LB_EXIT_LATENCY_US	2000		/* 2ms */

#ifdef DEBUG_LB_TRACKME
#define TRACKME_RUNNING_THRES		1000000		/* 1ms */
#define TRACKME_RUNNABLE_THRES		1000000		/* 1ms */

#define MAX_TRACKME_NUM				20

struct trackme {
	struct task_struct *tsk;
	u64 max_running_time;
	u64 max_runnable_time;
	int pid;
};

struct trackme tk_array[MAX_TRACKME_NUM];
int tk_num;

void show_trackme_stats(void);
#endif

struct lb_statistic {
	/* statistics related to tick balance. */
	atomic64_t tick_hit;
	atomic64_t tick_running_rt_boost_succ;
	atomic64_t tick_runnable_rt_boost_succ;
	atomic64_t tick_running_ux_succ;
	atomic64_t tick_runnable_ux_succ;
	atomic64_t tick_running_normal_rt_succ;
	atomic64_t tick_fail;

	/* statistics related to newidle balance. */
	atomic64_t newidle_hit;
	atomic64_t newidle_runnable_ux_succ;
	atomic64_t newidle_runnable_rt_boost_succ;
	atomic64_t newidle_runnable_normal_rt_succ;
	atomic64_t newidle_fail;
};

struct lb_statistic lb_stat;

static unsigned int lb_enable __read_mostly = 1;
static unsigned int lb_debug __read_mostly = 0;

void migr_running_task_systrace(
			unsigned int cpu, struct task_struct *p)
{
	int ux_state;
	char buf[256];
	struct oplus_task_struct *ots;

	if (unlikely(global_debug_enabled & DEBUG_SYSTRACE) && p) {
		ots = get_oplus_task_struct(p);
		if (IS_ERR_OR_NULL(ots))
			return;

		ux_state = ots->ux_state & (SCHED_ASSIST_UX_MASK | SA_TYPE_INHERIT);
		if (!ux_state)
			return;

		snprintf(buf, sizeof(buf), "C|%d|tick_lb_running_cpu%d|%d\n",
						OPLUS_LB_SYSTRACE_PID, cpu, p->pid);
		tracing_mark_write(buf);
	}
}

void migr_running_rt_systrace(
			unsigned int cpu, struct task_struct *p)
{
	char buf[256];

	if (unlikely(global_debug_enabled & DEBUG_SYSTRACE) && p) {
		snprintf(buf, sizeof(buf), "C|%d|tick_lb_running_rt_cpu%d|%d\n",
						OPLUS_LB_SYSTRACE_PID, cpu, p->pid);
		tracing_mark_write(buf);
	}
}

void migr_runnable_task_systrace(
			unsigned int cpu, struct task_struct *p)
{
	int ux_state;
	char buf[256];
	struct oplus_task_struct *ots;

	if (unlikely(global_debug_enabled & DEBUG_SYSTRACE) && p) {
		ots = get_oplus_task_struct(p);
		if (IS_ERR_OR_NULL(ots))
			return;

		ux_state = ots->ux_state & (SCHED_ASSIST_UX_MASK | SA_TYPE_INHERIT);
		if (!ux_state)
			return;

		snprintf(buf, sizeof(buf), "C|%d|tick_lb_runnable_cpu%d|%d\n",
						OPLUS_LB_SYSTRACE_PID, cpu, p->pid);
		tracing_mark_write(buf);
	}
}

__maybe_unused void runnable_time_systrace(
			unsigned int cpu, struct task_struct *p, u64 time)
{
	int ux_state;
	char buf[256];
	struct oplus_task_struct *ots;

	if (unlikely(global_debug_enabled & DEBUG_SYSTRACE) && p) {
		ots = get_oplus_task_struct(p);
		if (IS_ERR_OR_NULL(ots))
			return;

		ux_state = ots->ux_state & (SCHED_ASSIST_UX_MASK | SA_TYPE_INHERIT);
		if (!ux_state)
			return;

		snprintf(buf, sizeof(buf), "C|%d|runnable_time_cpu%d|%llu\n",
						OPLUS_LB_SYSTRACE_PID, cpu, time);
		tracing_mark_write(buf);
	}
}

__maybe_unused void record_runnable_time_systrace(
			unsigned int cpu, struct task_struct *p, u64 time)
{
	int ux_state;
	char buf[256];
	struct oplus_task_struct *ots;

	if (unlikely(global_debug_enabled & DEBUG_SYSTRACE) && p) {
		ots = get_oplus_task_struct(p);
		if (IS_ERR_OR_NULL(ots))
			return;

		ux_state = ots->ux_state & (SCHED_ASSIST_UX_MASK | SA_TYPE_INHERIT);
		if (!ux_state)
			return;

		snprintf(buf, sizeof(buf), "C|%d|runnable_time_cpu%d_pid%d|%llu\n",
						OPLUS_LB_SYSTRACE_PID, cpu, p->pid, time);
		tracing_mark_write(buf);
	}
}

__maybe_unused void record_exec_time_systrace(
			unsigned int cpu, struct task_struct *p, u64 time)
{
	int ux_state;
	char buf[256];
	struct oplus_task_struct *ots;

	if (unlikely(global_debug_enabled & DEBUG_SYSTRACE) && p) {
		ots = get_oplus_task_struct(p);
		if (IS_ERR_OR_NULL(ots))
			return;

		ux_state = ots->ux_state & (SCHED_ASSIST_UX_MASK | SA_TYPE_INHERIT);
		if (!ux_state)
			return;

		snprintf(buf, sizeof(buf), "C|%d|exec_time_cpu%d_pid%d|%llu\n",
						OPLUS_LB_SYSTRACE_PID, cpu, p->pid, time);
		tracing_mark_write(buf);
	}
}

void runnable_time_rt_systrace(
			unsigned int cpu, struct task_struct *p, u64 time)
{
	char buf[256];

	if (unlikely(global_debug_enabled & DEBUG_SYSTRACE) && p) {
		if (!test_task_is_rt(p))
			return;

		snprintf(buf, sizeof(buf), "C|%d|runnable_time_rt_cpu%d|%llu\n",
						OPLUS_LB_SYSTRACE_PID, cpu, time);
		tracing_mark_write(buf);
	}
}

void newidle_migr_ux_systrace(
			unsigned int cpu, struct task_struct *p)
{
	int ux_state;
	char buf[256];
	struct oplus_task_struct *ots;

	if (unlikely(global_debug_enabled & DEBUG_SYSTRACE) && p) {
		ots = get_oplus_task_struct(p);
		if (IS_ERR_OR_NULL(ots))
			return;

		ux_state = ots->ux_state & (SCHED_ASSIST_UX_MASK | SA_TYPE_INHERIT);
		if (!ux_state)
			return;

		snprintf(buf, sizeof(buf), "C|%d|newidle_migr_ux_cpu%d|%d\n",
						OPLUS_LB_SYSTRACE_PID, cpu, p->pid);
		tracing_mark_write(buf);
	}
}

void oplus_loadbalance_systrace_print(
			u32 pid, char *tag, u32 cpu, u64 val)
{
	char buf[256];

	if (unlikely(global_debug_enabled & DEBUG_SYSTRACE)) {
		snprintf(buf, sizeof(buf), "C|%d|%s%d|%llu\n", pid, tag, cpu, val);
		tracing_mark_write(buf);
	}
}

/******** The following code is copied from lib/plist.c. ********/
#ifdef CONFIG_DEBUG_PLIST

static struct plist_head test_head;

static void plist_check_prev_next(struct list_head *t, struct list_head *p,
				  struct list_head *n)
{
	WARN(n->prev != p || p->next != n,
			"top: %p, n: %p, p: %p\n"
			"prev: %p, n: %p, p: %p\n"
			"next: %p, n: %p, p: %p\n",
			 t, t->next, t->prev,
			p, p->next, p->prev,
			n, n->next, n->prev);
}

static void plist_check_list(struct list_head *top)
{
	struct list_head *prev = top, *next = top->next;

	plist_check_prev_next(top, prev, next);
	while (next != top) {
		prev = next;
		next = prev->next;
		plist_check_prev_next(top, prev, next);
	}
}

static void plist_check_head(struct plist_head *head)
{
	if (!plist_head_empty(head))
		plist_check_list(&plist_first(head)->prio_list);
	plist_check_list(&head->node_list);
}

#else
# define plist_check_head(h)	do { } while (0)
#endif

/**
 * plist_add - add @node to @head
 *
 * @node:	&struct plist_node pointer
 * @head:	&struct plist_head pointer
 */
void plist_add(struct plist_node *node, struct plist_head *head)
{
	struct plist_node *first, *iter, *prev = NULL;
	struct list_head *node_next = &head->node_list;

	plist_check_head(head);
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
		iter = list_entry(iter->prio_list.next,
				struct plist_node, prio_list);
	} while (iter != first);

	if (!prev || prev->prio != node->prio)
		list_add_tail(&node->prio_list, &iter->prio_list);
ins_node:
	list_add_tail(&node->node_list, node_next);

	plist_check_head(head);
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

/******** The code above is copied from lib/plist.c. ********/

/******** The following code is copied from fair.c ********/
enum fbq_type { regular, remote, all };

enum migration_type {
	migrate_load = 0,
	migrate_util,
	migrate_task,
	migrate_misfit
};

#define LBF_ALL_PINNED	0x01
#define LBF_NEED_BREAK	0x02
#define LBF_DST_PINNED  0x04
#define LBF_SOME_PINNED	0x08
#define LBF_ACTIVE_LB	0x10

struct lb_env {
	struct sched_domain	*sd;

	struct rq		*src_rq;
	int			src_cpu;

	int			dst_cpu;
	struct rq		*dst_rq;

	struct cpumask		*dst_grpmask;
	int			new_dst_cpu;
	enum cpu_idle_type	idle;
	long			imbalance;
	/* The set of CPUs under consideration for load-balancing */
	struct cpumask		*cpus;

	unsigned int		flags;

	unsigned int		loop;
	unsigned int		loop_break;
	unsigned int		loop_max;

	enum fbq_type		fbq_type;
	enum migration_type	migration_type;
	struct list_head	tasks;
	struct rq_flags		*src_rq_rf;
};

#ifdef CONFIG_NUMA_BALANCING
/*
 * Returns 1, if task migration degrades locality
 * Returns 0, if task migration improves locality i.e migration preferred.
 * Returns -1, if task migration is not affected by locality.
 */
static int migrate_degrades_locality(struct task_struct *p, struct lb_env *env)
{
	struct numa_group *numa_group = rcu_dereference(p->numa_group);
	unsigned long src_weight, dst_weight;
	int src_nid, dst_nid, dist;

	if (!static_branch_likely(&sched_numa_balancing))
		return -1;

	if (!p->numa_faults || !(env->sd->flags & SD_NUMA))
		return -1;

	src_nid = cpu_to_node(env->src_cpu);
	dst_nid = cpu_to_node(env->dst_cpu);

	if (src_nid == dst_nid)
		return -1;

	/* Migrating away from the preferred node is always bad. */
	if (src_nid == p->numa_preferred_nid) {
		if (env->src_rq->nr_running > env->src_rq->nr_preferred_running)
			return 1;
		else
			return -1;
	}

	/* Encourage migration to the preferred node. */
	if (dst_nid == p->numa_preferred_nid)
		return 0;

	/* Leaving a core idle is often worse than degrading locality. */
	if (env->idle == CPU_IDLE)
		return -1;

	dist = node_distance(src_nid, dst_nid);
	if (numa_group) {
		src_weight = group_weight(p, src_nid, dist);
		dst_weight = group_weight(p, dst_nid, dist);
	} else {
		src_weight = task_weight(p, src_nid, dist);
		dst_weight = task_weight(p, dst_nid, dist);
	}

	return dst_weight < src_weight;
}

#else
static inline int migrate_degrades_locality(struct task_struct *p,
					     struct lb_env *env)
{
	return -1;
}
#endif

const_debug unsigned int sysctl_sched_migration_cost	= 500000UL;

/*
 * Is this task likely cache-hot:
 */
static int task_hot(struct task_struct *p, struct lb_env *env)
{
	s64 delta;

	lockdep_assert_rq_held(env->src_rq);

	/*
	 * Because the fair_sched_class symbol cannot be exported,
	 * use the following interface instead
	 *
	 * if (p->sched_class != &fair_sched_class)
	 *	return 0;
	 */
	if (!test_task_is_fair(p))
		return 0;


	if (unlikely(task_has_idle_policy(p)))
		return 0;

	/* SMT siblings share cache */
	if (env->sd->flags & SD_SHARE_CPUCAPACITY)
		return 0;

	/*
	 * Buddy candidates are cache hot:
	 */
	if (sched_feat(CACHE_HOT_BUDDY) && env->dst_rq->nr_running &&
			(&p->se == cfs_rq_of(&p->se)->next ||
			 &p->se == cfs_rq_of(&p->se)->last))
		return 1;

	if (sysctl_sched_migration_cost == -1)
		return 1;

	/*
	 * Don't migrate task if the task's cookie does not match
	 * with the destination CPU's core cookie.
	 */
	if (!sched_core_cookie_match(cpu_rq(env->dst_cpu), p))
		return 1;

	if (sysctl_sched_migration_cost == 0)
		return 0;

	delta = rq_clock_task(env->src_rq) - p->se.exec_start;

	return delta < (s64)sysctl_sched_migration_cost;
}

enum KTHREAD_BITS {
	KTHREAD_IS_PER_CPU = 0,
	KTHREAD_SHOULD_STOP,
	KTHREAD_SHOULD_PARK,
};

struct kthread {
	unsigned long flags;
	unsigned int cpu;
	int (*threadfn)(void *);
	void *data;
	mm_segment_t oldfs;
	struct completion parked;
	struct completion exited;
#ifdef CONFIG_BLK_CGROUP
	struct cgroup_subsys_state *blkcg_css;
#endif
};

/*
 * Variant of to_kthread() that doesn't assume @p is a kthread.
 *
 * Per construction; when:
 *
 *   (p->flags & PF_KTHREAD) && p->set_child_tid
 *
 * the task is both a kthread and struct kthread is persistent. However
 * PF_KTHREAD on it's own is not, kernel_thread() can exec() (See umh.c and
 * begin_new_exec()).
 */
static inline struct kthread *__to_kthread(struct task_struct *p)
{
	void *kthread = (__force void *)p->set_child_tid;
	if (kthread && !(p->flags & PF_KTHREAD))
		kthread = NULL;
	return kthread;
}

bool kthread_is_per_cpu(struct task_struct *p)
{
	struct kthread *kthread = __to_kthread(p);
	if (!kthread)
		return false;

	return test_bit(KTHREAD_IS_PER_CPU, &kthread->flags);
}

/*
 * can_migrate_task - may task p from runqueue rq be migrated to this_cpu?
 */
static
int can_migrate_task(struct task_struct *p, struct lb_env *env)
{
	int tsk_cache_hot;
	/*
	 * del by oplus.
	 * int can_migrate = 1;
	 */

	lockdep_assert_rq_held(env->src_rq);

	/*
	 * del by oplus.
	 * trace_android_rvh_can_migrate_task(p, env->dst_cpu, &can_migrate);
	 * if (!can_migrate)
	 * 	return 0;
	 */

	/*
	 * We do not migrate tasks that are:
	 * 1) throttled_lb_pair, or
	 * 2) cannot be migrated to this CPU due to cpus_ptr, or
	 * 3) running (obviously), or
	 * 4) are cache-hot on their current CPU.
	 */
	/*
	 * del by oplus.
	 * if (throttled_lb_pair(task_group(p), env->src_cpu, env->dst_cpu))
	 * 	return 0;
	 */

	/* Disregard pcpu kthreads; they are where they need to be. */
	if (kthread_is_per_cpu(p))
		return 0;

	if (!cpumask_test_cpu(env->dst_cpu, p->cpus_ptr)) {
		int cpu;

		/*
		 * del by oplus.
		 * schedstat_inc(p->se.statistics.nr_failed_migrations_affine);
		 */

		env->flags |= LBF_SOME_PINNED;

		/*
		 * Remember if this task can be migrated to any other CPU in
		 * our sched_group. We may want to revisit it if we couldn't
		 * meet load balance goals by pulling other tasks on src_cpu.
		 *
		 * Avoid computing new_dst_cpu
		 * - for NEWLY_IDLE
		 * - if we have already computed one in current iteration
		 * - if it's an active balance
		 */
		if (env->idle == CPU_NEWLY_IDLE ||
		    env->flags & (LBF_DST_PINNED | LBF_ACTIVE_LB))
			return 0;

		/* Prevent to re-select dst_cpu via env's CPUs: */
		for_each_cpu_and(cpu, env->dst_grpmask, env->cpus) {
			if (cpumask_test_cpu(cpu, p->cpus_ptr)) {
				env->flags |= LBF_DST_PINNED;
				env->new_dst_cpu = cpu;
				break;
			}
		}

		return 0;
	}

	/* Record that we found at least one task that could run on dst_cpu */
	env->flags &= ~LBF_ALL_PINNED;

	if (task_running(env->src_rq, p)) {
		/*
		 * del by oplus.
		 * schedstat_inc(p->se.statistics.nr_failed_migrations_running);
		 */
		return 0;
	}

	/*
	 * Aggressive migration if:
	 * 1) active balance
	 * 2) destination numa is preferred
	 * 3) task is cache cold, or
	 * 4) too many balance attempts have failed.
	 */
	if (env->flags & LBF_ACTIVE_LB)
		return 1;

	tsk_cache_hot = migrate_degrades_locality(p, env);
	if (tsk_cache_hot == -1)
		tsk_cache_hot = task_hot(p, env);

	if (tsk_cache_hot <= 0 ||
	    env->sd->nr_balance_failed > env->sd->cache_nice_tries) {
		/*
		 * del by oplus.
		 * if (tsk_cache_hot == 1) {
		 * 	schedstat_inc(env->sd->lb_hot_gained[env->idle]);
		 * 	schedstat_inc(p->se.statistics.nr_forced_migrations);
		 * }
		 */
		return 1;
	}

	/*
	 * del by oplus.
	 * schedstat_inc(p->se.statistics.nr_failed_migrations_hot);
	 */
	return 0;
}

/*
 * detach_task() -- detach the task for the migration specified in env
 */
static void detach_task(struct task_struct *p, struct lb_env *env)
{
	/*
	 * del by oplus.
	 * int detached = 0;
	 */

	lockdep_assert_rq_held(env->src_rq);

	/*
	 * The vendor hook may drop the lock temporarily, so
	 * pass the rq flags to unpin lock. We expect the
	 * rq lock to be held after return.
	 */
	/*
	 * del by oplus.
	 * trace_android_rvh_migrate_queued_task(env->src_rq, env->src_rq_rf, p,
	 * 				      env->dst_cpu, &detached);
	 * if (detached)
	 * 	return;
	 */

	deactivate_task(env->src_rq, p, DEQUEUE_NOCLOCK);
	set_task_cpu(p, env->dst_cpu);
}

/*
 * detach_one_task() -- tries to dequeue exactly one task from env->src_rq, as
 * part of active balancing operations within "domain".
 *
 * Returns a task if successful and NULL otherwise.
 */
__maybe_unused static struct task_struct *detach_one_task(struct lb_env *env)
{
	struct task_struct *p;

	lockdep_assert_rq_held(env->src_rq);

	list_for_each_entry_reverse(p,
			&env->src_rq->cfs_tasks, se.group_node) {
		if (!can_migrate_task(p, env))
			continue;

		detach_task(p, env);

		/*
		 * Right now, this is only the second place where
		 * lb_gained[env->idle] is updated (other is detach_tasks)
		 * so we can safely collect stats here rather than
		 * inside detach_tasks().
		 */
		/*
		 * del by oplus.
		 * schedstat_inc(env->sd->lb_gained[env->idle]);
		 */
		return p;
	}
	return NULL;
}

/*
 * attach_task() -- attach the task detached by detach_task() to its new rq.
 */
static void attach_task(struct rq *rq, struct task_struct *p)
{
	lockdep_assert_rq_held(rq);

	WARN_ON(task_rq(p) != rq);
	activate_task(rq, p, ENQUEUE_NOCLOCK);
	check_preempt_curr(rq, p, 0);
}

/*
 * attach_one_task() -- attaches the task returned from detach_one_task() to
 * its new rq.
 */
static void attach_one_task(struct rq *rq, struct task_struct *p)
{
	struct rq_flags rf;

	rq_lock(rq, &rf);
	update_rq_clock(rq);
	attach_task(rq, p);
	rq_unlock(rq, &rf);
}

/******** The above code is copied from fair.c ********/


bool test_task_ux_lb(struct task_struct *task)
{
	if (!task || !test_task_is_fair(task))
		return false;

	/*
	 * If the task is already blocked or only allowed to
	 * run on this cpu, there is no need to perform an up
	 * migration operation.
	 */
	if (READ_ONCE(task->__state) != TASK_RUNNING ||
		task->nr_cpus_allowed == 1)
		return false;

	/*
	 * Only the following 3 types of ux threads need to
	 * be balanced.
	 */
	if (get_ux_state(task) & SCHED_ASSIST_LB_UX)
		return true;

	return false;
}

enum threshold_type {
	/* migrate running ux task. */
	migr_running_ux = 1,

	/* migrate runnable ux task. */
	migr_runnable_ux,

	/* migrate tasks in the rt_boost group. */
	migr_running_rt_boost,

	/* migrate audio-related tasks in the rt_boost group. */
	rt_boost_runnable_audio,

	/* migrate surfaceflinger-related tasks in the rt_boost group. */
	rt_boost_runnable_surfaceflinger,

	/* migrate renderengine-related tasks in the rt_boost group. */
	rt_boost_runnable_renderengine,

	/* migrate normal_rt task*/
	normal_rt_runnable,
};

u64 get_threshold_time(enum threshold_type type)
{
	u64 threshold = ULLONG_MAX;

	switch (type) {
	case migr_running_ux:
		threshold = 7000000U;			/* 7ms */
		break;

	case migr_runnable_ux:
		threshold = 2000000U;			/* 2ms */
		break;

	case migr_running_rt_boost:
		threshold = 7000000U;			/* 7ms */
		break;

	case rt_boost_runnable_audio:
		threshold = 400000U;			/* 0.4ms */
		break;

	case rt_boost_runnable_surfaceflinger:
		threshold = 800000U;			/* 0.8ms */
		break;

	case rt_boost_runnable_renderengine:
		threshold = 800000U;			/* 0.8ms */
		break;

	case normal_rt_runnable:
		threshold = 2000000U;			/* 2ms */
		break;

	default:
		threshold = ULLONG_MAX;
		break;
	}

#if defined(DEBUG_LB_TICK) || \
	defined(DEBUG_LB_TICK_RUNNABLE_TIME) || \
	defined(DEBUG_LB_NEWIDLE) || \
	defined(DEBUG_LB_NEWIDLE_HIT) || \
	defined(DEBUG_LB_RT_TICK) || \
	defined(DEBUG_LB_RT_RUNNABLE_TIME)
	/*
	 * We shrink the threshold value to make it easier to
	 * trigger a valid balance in debug mode.
	 */
	threshold /= 100;
	min_t(u64, threshold, 100000);		/* 0.1ms */
#endif

	return threshold;
}

bool task_is_on_runqueue(struct task_struct *tsk)
{
	if (!tsk)
		return false;

	if (READ_ONCE((tsk)->__state) != TASK_RUNNING)
		return false;

	return (tsk->on_rq == TASK_ON_RQ_QUEUED);
}

bool task_is_runnnig_on_cpu(struct task_struct *tsk)
{
	if (!tsk)
		return false;

	if (READ_ONCE((tsk)->__state) != TASK_RUNNING)
		return false;

	return ((tsk->on_rq == TASK_ON_RQ_QUEUED) && smp_load_acquire(&tsk->on_cpu));
}

bool task_is_runnable_on_runqueue(struct task_struct *tsk)
{
	if (!tsk)
		return false;

	if (READ_ONCE((tsk)->__state) != TASK_RUNNING)
		return false;

	return ((tsk->on_rq == TASK_ON_RQ_QUEUED) && !smp_load_acquire(&tsk->on_cpu));
}

#ifdef DEBUG_LB_TICK
u64 max_runnable_time;
u64 max_running_time;
#endif

/*
 * Get the time a task stays in the running or runnable state.
 *
 * This interface is valid for all tasks including cfs and rt tasks.
 * The timer_sel parameter is used to select the time to be returned.
 * true returns runnable time while false returns running time.
*/
u64 __get_time(struct task_struct *tsk, bool time_sel)
{
	struct oplus_task_struct *ots;
	struct rq *rq;
	u64 runnable_time = 0, running_time = 0;
	u64 now;
	struct sched_info sched_info;

	ots = get_oplus_task_struct(tsk);
	if (IS_ERR_OR_NULL(ots))
		return 0;

	/*
	 * Idle tasks are not considered.
	 */
	if (!tsk->pid)
		return 0;

	/*
	 * Backup to prevent the sched_info of the task from
	 * changing during the following calculations.
	 */
	sched_info = tsk->sched_info;

	/*
	 * As long as the task is on the runqueue, it will continue to
	 * execute the following logic no matter what state it is in.
	 * If the task is already blocked (perhaps in sleep or disk sleep
	 * state), there is no need to pay attention to its running or
	 * runnable time.
	 */
	if (!task_is_on_runqueue(tsk))
		return 0;

	rq = cpu_rq(task_cpu(tsk));
	now = rq->clock;

	if (task_is_runnnig_on_cpu(tsk)) {
		/*
		 * Scenario 1:
		 * the task has completed at least one context switch.
		 */
		runnable_time = sched_info.run_delay - ots->snap_run_delay;

		if (sched_info.last_queued) {
			/*
			 * The value of rq->lock may be less than ots->enqueue_time
			 * because rq->lock is not the latest value.
			 */
			running_time = now - ots->enqueue_time;
			if (running_time >= runnable_time)
				running_time = running_time - runnable_time;
		} else {
			running_time = 0;
		}
	} else if (task_is_runnable_on_runqueue(tsk)) {
		if (ots->snap_pcount == sched_info.pcount) {
			/*
			 * Scenario 2:
			 * task has not been executed since it was enqueued.
			 */
			runnable_time = now - ots->enqueue_time;
			running_time = 0;

		} else if (sched_info.last_queued) {
			/*
			 * Scenario 3:
			 * task has completed one or more context switches.
			 */
			runnable_time = now - sched_info.last_queued +
					(sched_info.run_delay - ots->snap_run_delay);

			/*
			 * The value of rq->lock may be less than ots->enqueue_time
			 * because rq->lock is not the latest value.
			 */
			running_time = now - ots->enqueue_time;
			if (running_time >= runnable_time)
				running_time = running_time - runnable_time;
		} else {
			/*
			 * This task is performing context switching, and there
			 * is no need to pay attention to the running and runnable
			 * times at this time.
			 */
			running_time = runnable_time = 0;
		}
	}

#ifdef DEBUG_LB_TICK
	max_runnable_time = max_t(u64, max_runnable_time, runnable_time);
	max_running_time = max_t(u64, max_running_time, running_time);

	trace_printk("OPLUS_LB_TICK[%d]: cpu=%d, tsk=%s$%d, "
		"state=%d, on_cpu=%d, on_rq=%d "
		"runnable_time=%llu, running_time=%llu, RN=%d-%d, "
		"snap_run_delay=%llu, snap_pcount=%lu, enqueue_time=%llu "
		"now=%llu, run_delay=%llu, pcount=%lu, last_queued=%llu "
		"true=%d-%d-%d-%d-%d "
		"max_runnable_time=%llu, max_running_time=%llu\n",
	__LINE__, smp_processor_id(), tsk->comm, tsk->pid,
	tsk->__state, tsk->on_cpu, tsk->on_rq,
	runnable_time, running_time,
	task_is_runnnig_on_cpu(tsk), task_is_runnable_on_runqueue(tsk),
	ots->snap_run_delay, ots->snap_pcount, ots->enqueue_time,
	now, sched_info.run_delay, sched_info.pcount,
	sched_info.last_queued,
	(s64)running_time < 0, (s64)runnable_time < 0, runnable_time == now,
	running_time > 80000000000, runnable_time > 80000000000,
	max_runnable_time, max_running_time);

	oplus_loadbalance_systrace_print(OPLUS_LB_SYSTRACE_PID,
			"A-runnable_time_pid", tsk->pid, runnable_time);
	oplus_loadbalance_systrace_print(OPLUS_LB_SYSTRACE_PID,
			"B-running_time_pid", tsk->pid, running_time);
#endif

	return time_sel ? runnable_time : running_time;
}

u64 get_runnable_time(struct task_struct *tsk)
{
	return __get_time(tsk, true);
}

u64 get_running_time(struct task_struct *tsk)
{
	return __get_time(tsk, false);
}

bool ux_need_up_migration(struct task_struct *p, struct rq *rq)
{
	struct oplus_task_struct *ots;
	u64 threshold_time = ULLONG_MAX;
	u64 running_time = 0;

	if (!p || !rq)
		return false;

	/*
	 * only allow ux tasks to migrate from silver to gold or prime core.
	 */
	if (topology_physical_package_id(cpu_of(rq)))
		return false;

	/*
	 * If a ux thread runs continuously for more than 7ms,
	 * consider migrating it to a cpu with stronger capacity.
	 */
	if (!test_task_ux_lb(p))
		return false;

	ots = get_oplus_task_struct(p);
	if (IS_ERR_OR_NULL(ots))
		return false;

	running_time = get_running_time(p);
	threshold_time = get_threshold_time(migr_running_ux);

#ifdef DEBUG_LB_TICK
	trace_printk("OPLUS_LB_TICK[%d]: task=%s$%d, mask=[%*pbl], "
		"cpu=%d, cls=%d, running_time=%llu, threshold_time=%llu, true=%d\n",
		__LINE__, p->comm, p->pid, cpumask_pr_args(p->cpus_ptr),
		cpu_of(rq), topology_physical_package_id(cpu_of(rq)),
		running_time, threshold_time, running_time >= threshold_time);
#endif

	return running_time >= threshold_time;
}

/*
 * down_migr     : tasks allow migration to smaller cores.
 * up_migr       : tasks are allowed to migrate from small cores to
 *                 medium or large cores, and medium cores do not
 *                 need to be balanced.
 * normal_migr   : similar to up_migr, but can be in the same cluster.
 * newidle_migr  : migrate tasks from CPUs of smaller or same capacity.
 * tickpull_migr : Only used in tick balance to pull tasks from cpus
 *                 with the same capacity or less to the local cpu.
 */
enum migr_type {
	down_migr = 1,
	up_migr,
	normal_migr,
	newidle_migr,
	tickpull_migr,

	/* Add the new type above this line. */
	invalid_migr_type
};

/*
 * The order of the walk cluster is determined according to the type
 * of migration. The detailed rules are described as follows:
 *
 * oplus_cpu_array[][]
 * 0 -> 1 -> 2
 * 1 -> 2 -> 0
 * 2 -> 1 -> 0
 *
 * 0 -> 1 -> 2
 * 1 -> 0 -> 2
 * 2 -> 1 -> 0
 *
 * up_migr
 * cur_idx    order_idx   walk_cnt
 *    0            1          2
 *    1            X          X
 *    2            X          X
 *
 *
 * normal_migr
 * cur_idx    order_idx   walk_cnt
 *    0            0          3
 *    1            1          2
 *    2            2          2
 *
 *
 * down_migr
 * cur_idx    order_idx   walk_cnt
 *    0            X          X
 *    1            3          1
 *    2            4          2
 *
 *
 * newidle_migr
 * cur_idx    order_idx   walk_cnt
 *    0            0          1
 *    1            1          3    (!SA_LANUNCH)
 *    1            4          2    ( SA_LANUNCH)
 *    2            2          3
 *
 *
 * tickpull_migr
 * cur_idx    order_idx   walk_cnt
 *    0            0          1
 *    1            1          3
 *    2            2          1
 *
 *
 * Return true if a suitable order_idx has been found,
 * otherwise return false.
 */
bool calc_order_idx(enum migr_type type,
			int curr_cls, int *order_idx, int *walk_cnt)
{
	struct ux_sched_cputopo ux_cputopo = ux_sched_cputopo;
	int cls_nr = ux_cputopo.cls_nr;

	/* Returns directly when there is only one cls */
	if (unlikely(cls_nr <= 1))
		return false;

	/* valid type? */
	if (type < down_migr || type >= invalid_migr_type)
		return false;

	/*
	 * Only tasks on the silver core are allowed to perform
	 * the up_migr operation.
	 */
	if ((type == up_migr) && curr_cls > 0)
		return false;

	/*
	 * If p is already running on the silver core, there is
	 * no need to perform the down_migr operation.
	 */
	if ((type == down_migr) && curr_cls == 0)
		return false;

	if (type == down_migr) {
		*order_idx = curr_cls + cls_nr -1;
		*walk_cnt = curr_cls;
	} else if (type == up_migr) {
		*order_idx = curr_cls + 1;
		*walk_cnt = cls_nr - *order_idx;
	} else if (type == normal_migr) {
		*order_idx = curr_cls;
		if ((curr_cls == cls_nr-1) && (cls_nr >= 3)) {
			*walk_cnt = cls_nr - 1;
		} else {
			*walk_cnt = cls_nr - *order_idx;
		}
	} else if (type == newidle_migr) {
		if (sched_assist_scene(SA_LAUNCH) &&
			curr_cls != 0 && curr_cls != cls_nr -1) {
			*order_idx = curr_cls + cls_nr;
			*walk_cnt = cls_nr - curr_cls;
		} else {
			*order_idx = curr_cls;
			if (!curr_cls) {
				*walk_cnt = 1;
			} else {
				*walk_cnt = cls_nr;
			}
		}
	} else if (type == tickpull_migr) {
		*order_idx = curr_cls;
		if ((cls_nr >= 3) && (curr_cls == cls_nr-2)) {
			*walk_cnt = cls_nr;
		} else {
			*walk_cnt = 1;
		}
	}
	return true;
}

int find_cpu_in_migration(struct task_struct *p,
			int prev_cpu, enum migr_type type, bool latency_sensive)
{
	struct rq *rq = NULL;
	struct task_struct *curr = NULL;
	struct oplus_rq *orq = NULL;
	struct ux_sched_cputopo ux_cputopo = ux_sched_cputopo;
	struct cpuidle_state *idle = NULL;
	cpumask_t search_cpus = CPU_MASK_NONE;
	int prev_cls = topology_physical_package_id(prev_cpu);
	int order_idx = -1;
	int walk_cnt = -1, idx;
	int cpu = -1;
	bool ret;

	ret = calc_order_idx(type, prev_cls, &order_idx, &walk_cnt);
#ifdef DEBUG_LB_TICK
	trace_printk("OPLUS_LB_TICK[%d]: migr_type=%d, prev_cpu=%d, prev_cls=%d, "
		"order_idx=%d, walk_cnt=%d, ret=%d\n",
		__LINE__, type, prev_cpu, prev_cls, order_idx, walk_cnt, ret);
#endif
	if (!ret)
		return -1;

	for (idx = 0; idx < walk_cnt; idx++) {
		cpumask_copy(&search_cpus, &ux_cputopo.oplus_cpu_array[order_idx][idx]);

#ifdef DEBUG_LB_TICK
		trace_printk("OPLUS_LB_TICK[%d]: migr_type=%d, prev_cpu=%d, prev_cls=%d, "
			"order_idx=%d, walk_cnt=%d, idx=%d, mask=[%*pbl]\n",
			__LINE__, type, prev_cpu, prev_cls,
			order_idx, walk_cnt, idx,
			cpumask_pr_args(&search_cpus));
#endif

		for_each_cpu(cpu, &search_cpus) {
			rq = cpu_rq(cpu);
			curr = rq->curr;
			orq = (struct oplus_rq *) rq->android_oem_data1;

#ifdef DEBUG_LB_TICK
			trace_printk("OPLUS_LB_TICK[%d]: cpu=%d, curr=%s$%d$%d, "
				"oah=%d-%d-%d, has_ux=%d, has_rt=%d\n",
				__LINE__, cpu, curr->comm, curr->pid, get_ux_state(curr),
				cpu_online(cpu), cpu_active(cpu), oplus_cpu_halted(cpu),
				orq_has_ux_tasks(orq), rt_rq_is_runnable(&rq->rt));
#endif

			/* cannot migrate to itself. */
			if (cpu == prev_cpu)
				continue;

			/* affinify */
			if (!cpumask_test_cpu(cpu, p->cpus_ptr))
				continue;

			if (!cpu_online(cpu) || !cpu_active(cpu))
				continue;

#ifdef CONFIG_OPLUS_ADD_CORE_CTRL_MASK
			if (oplus_cpu_halted(cpu))
				continue;
#endif

			/*
			 * Ignore those CPUs if there is a ux/rt task running on it.
			 */
			if (get_ux_state(curr) & POSSIBLE_UX_MASK)
				continue;

			if (curr->prio < MAX_RT_PRIO)
				continue;

			/*
			 * Ignore those CPUs if there is a ux/rt task in runnable
			 * state on it.
			 */
			if (orq_has_ux_tasks(orq))
				continue;

			if (rt_rq_is_runnable(&rq->rt))
				continue;

			/*
			 * Avoid placing latency-sensitive tasks on a deep-sleeping CPU.
			 */
			idle = idle_get_state(cpu_rq(cpu));
#ifdef DEBUG_LB_TICK
			trace_printk("DEBUG_LB_TICK[%d]: latency_sensive=%d, cpu=%d,"
					" name=%s, desc=%s, exit_latency=%dus,"
					" online=%d, active=%d, idle=%d,"
					" available_idle=%d, nr_running=%d\n",
				__LINE__, latency_sensive, cpu,
				idle?idle->name:"NULL", idle?idle->desc:"NULL",
				idle?idle->exit_latency:0, cpu_online(cpu), cpu_active(cpu),
				oplus_idle_cpu(cpu), available_idle_cpu(cpu),
				cpu_rq(cpu)->nr_running);
#endif
			if (latency_sensive &&
				idle && idle->exit_latency > OPLUS_LB_EXIT_LATENCY_US)
				continue;

#ifdef DEBUG_LB_TICK
			trace_printk("OPLUS_LB_TICK[%d]: cpu=%d\n", __LINE__, cpu);
#endif
			return cpu;
		}
	}

	/* Failed, no available cpu found. */
	return -1;
}

__maybe_unused void debug_find_cpu_in_migration(struct rq *rq,
			struct task_struct *task)
{
	unsigned int prev_cpu = cpu_of(rq);
	unsigned int new_cpu = -1;

	if (cpumask_weight(task->cpus_ptr) != 8)
		return;

	new_cpu = find_cpu_in_migration(task, prev_cpu, down_migr, false);
	trace_printk("OPLUS_LB_TICK[%d]: down_migr : curr=%s$%d, "
		"mask=[%*pbl], prev_cpu=%d, new_cpu=%d\n",
		__LINE__, task->comm, task->pid,
		cpumask_pr_args(task->cpus_ptr),
		prev_cpu, new_cpu);

	new_cpu = find_cpu_in_migration(task, prev_cpu, up_migr, false);
	trace_printk("OPLUS_LB_TICK[%d]: up_migr : curr=%s$%d, "
		"mask=[%*pbl], prev_cpu=%d, new_cpu=%d\n",
		__LINE__, task->comm, task->pid,
		cpumask_pr_args(task->cpus_ptr),
		prev_cpu, new_cpu);

	new_cpu = find_cpu_in_migration(task, prev_cpu, normal_migr, false);
	trace_printk("OPLUS_LB_TICK[%d]: normal_migr : curr=%s$%d, "
		"mask=[%*pbl], prev_cpu=%d, new_cpu=%d\n",
		__LINE__, task->comm, task->pid,
		cpumask_pr_args(task->cpus_ptr),
		prev_cpu, new_cpu);
}

static inline int same_cluster(int prev_cpu, int new_cpu)
{
	return topology_physical_package_id(new_cpu) == topology_physical_package_id(prev_cpu);
}

int oplus_kick_active_balance(struct rq *rq,
			struct task_struct *p, int new_cpu)
{
	unsigned long flags;
	bool ret = false;
	struct oplus_rq *orq = (struct oplus_rq *) rq->android_oem_data1;

	/* Invoke active balance to force migrate currently running task */
	raw_spin_lock_irqsave(&rq->__lock, flags);
	if (!rq->active_balance) {
		rq->active_balance = 1;
		rq->push_cpu = new_cpu;
		orq->lb.pid = p->pid;

		/*
		 * Note:
		 * No need for get_task_struct here. It doesn't matter if p
		 * is destroyed, because in this case the migrate operation
		 * is not performed in the active balance.
		 */
		/* get_task_struct(p); */
		ret = true;
	}
	raw_spin_unlock_irqrestore(&rq->__lock, flags);

	return ret;
}

__maybe_unused void debug_dump_cfs_tasks(struct lb_env *env)
{
	struct task_struct *p;
	unsigned int i = 0;

	lockdep_assert_rq_held(env->src_rq);

	list_for_each_entry(p, &env->src_rq->cfs_tasks, se.group_node) {
		trace_printk("OPLUS_LB_TICK[%d]: i=%d, task=%s$%d\n",
			__LINE__, ++i, p->comm, p->pid);
	}

	list_for_each_entry_reverse(p, &env->src_rq->cfs_tasks, se.group_node) {
		trace_printk("OPLUS_LB_TICK[%d]: i=%d, task=%s$%d\n",
			__LINE__, i--, p->comm, p->pid);
	}
}

/*
 * oplus_detach_running_task() -- Pick the task that was just running on the cpu.
 *
 * Returns a task if successful and NULL otherwise.
 */
static struct task_struct *oplus_detach_running_task(struct lb_env *env)
{
	struct task_struct *p;
	struct oplus_rq *orq = (struct oplus_rq *) env->src_rq->android_oem_data1;
	pid_t pid = orq->lb.pid;

#ifdef DEBUG_LB_TICK
	debug_dump_cfs_tasks(env);
#endif

	lockdep_assert_rq_held(env->src_rq);

	list_for_each_entry(p,
			&env->src_rq->cfs_tasks, se.group_node) {
		/* Pick the task that was just running on the cpu. */
		if (p->pid != pid)
			continue;

		if (!can_migrate_task(p, env))
			continue;

		detach_task(p, env);

		/*
		 * Right now, this is only the second place where
		 * lb_gained[env->idle] is updated (other is detach_tasks)
		 * so we can safely collect stats here rather than
		 * inside detach_tasks().
		 */
		/*
		 * del by oplus.
		 * schedstat_inc(env->sd->lb_gained[env->idle]);
		 */
		return p;
	}
	return NULL;
}

/*
 * active_load_balance_cpu_stop is run by the CPU stopper. It pushes
 * running tasks off the busiest CPU onto idle CPUs. It requires at
 * least 1 task to be running on each physical CPU where possible, and
 * avoids physical / logical imbalances.
 */
static int oplus_active_load_balance_cpu_stop(void *data)
{
	struct rq *busiest_rq = data;
	int busiest_cpu = cpu_of(busiest_rq);
	int target_cpu = busiest_rq->push_cpu;
	struct rq *target_rq = cpu_rq(target_cpu);
	struct sched_domain *sd;
	struct task_struct *p = NULL;
	struct rq_flags rf;
	struct oplus_rq *orq = (struct oplus_rq *) busiest_rq->android_oem_data1;

	rq_lock_irq(busiest_rq, &rf);

#ifdef DEBUG_LB_TICK
	trace_printk("OPLUS_LB_TICK[%d]: src_cpu=%d, dst_cpu=%d, "
		"active=%d-%d, cpu=%d, ab=%d, nr=%d, pid=%d\n",
		__LINE__, busiest_cpu, target_cpu,
		cpu_active(busiest_cpu), cpu_active(target_cpu), smp_processor_id(),
		busiest_rq->active_balance, busiest_rq->nr_running, orq->lb.pid);
#endif

	/*
	 * Between queueing the stop-work and running it is a hole in which
	 * CPUs can become inactive. We should not move tasks from or to
	 * inactive CPUs.
	 */
	if (!cpu_active(busiest_cpu) || !cpu_active(target_cpu))
		goto out_unlock;

	/* Make sure the requested CPU hasn't gone down in the meantime: */
	if (unlikely(busiest_cpu != smp_processor_id() ||
		     !busiest_rq->active_balance))
		goto out_unlock;

	/* Is there any task to move? */
	if (busiest_rq->nr_running <= 1)
		goto out_unlock;

	/*
	 * This condition is "impossible", if it occurs
	 * we need to fix it. Originally reported by
	 * Bjorn Helgaas on a 128-CPU setup.
	 */
	WARN_ON(busiest_rq == target_rq);

	/* Search for an sd spanning us and the target CPU. */
	rcu_read_lock();
	for_each_domain(target_cpu, sd) {
		if (cpumask_test_cpu(busiest_cpu, sched_domain_span(sd)))
			break;
	}

	if (likely(sd)) {
		struct lb_env env = {
			.sd		= sd,
			.dst_cpu	= target_cpu,
			.dst_rq		= target_rq,
			.src_cpu	= busiest_rq->cpu,
			.src_rq		= busiest_rq,
			.idle		= CPU_IDLE,
			.flags		= LBF_ACTIVE_LB,
			.src_rq_rf	= &rf,
		};

		/*
		 * del by oplus.
		 * schedstat_inc(sd->alb_count);
		 */
		update_rq_clock(busiest_rq);

		/*
		 * The native kernel picks a task from the end of the rq->cfs_tasks
		 * list through detach_one_task, but this task is not the one that
		 * is just running on the cpu, so we made a little adjustment here.
		 * p = detach_one_task(&env);
		 */
		p = oplus_detach_running_task(&env);
		if (p) {
			/*
			 * del by oplus.
			 * schedstat_inc(sd->alb_pushed);
			 */
			/* Active balancing done, reset the failure counter. */
			sd->nr_balance_failed = 0;
#ifdef DEBUG_LB_TICK
			trace_printk("OPLUS_LB_TICK[%d]: Success!!! "
				"src_cpu=%d, dst_cpu=%d, p=%s$%d, pid=%d\n",
				__LINE__, busiest_cpu, target_cpu,
				p->comm, p->pid, orq->lb.pid);
#endif
			orq->lb.pid = INVALID_PID;
		} else {
			/*
			 * del by oplus.
			 * schedstat_inc(sd->alb_failed);
			 */
#ifdef DEBUG_LB_TICK
			trace_printk("OPLUS_LB_TICK[%d]: Failed!!! "
				"src_cpu=%d, dst_cpu=%d\n",
				__LINE__, busiest_cpu, target_cpu);
#endif
		}
	}
	rcu_read_unlock();

out_unlock:
	busiest_rq->active_balance = 0;
	rq_unlock(busiest_rq, &rf);

	if (p)
		attach_one_task(target_rq, p);

	local_irq_enable();

	return 0;
}

static inline int has_runnable_rt_tasks(struct rq *rq)
{
	return !plist_head_empty(&rq->rt.pushable_tasks);
}

/*
 * Check whether the specified task is on the pushable list.
 */
bool is_task_on_pushable_task(struct rq *rq,
			struct task_struct *rt_task)
{
	struct plist_head *head = &rq->rt.pushable_tasks;
	struct task_struct *p;

	if (!has_runnable_rt_tasks(rq))
		return false;

	plist_for_each_entry(p, head, pushable_tasks) {
		if (p == rt_task)
			return true;
	}
	return false;
}

__maybe_unused void debug_dump_rt_pushable_tasks(struct lb_env *env)
{
	struct rq *src_rq = env->src_rq;
	struct plist_head *head = &src_rq->rt.pushable_tasks;
	struct task_struct *p;
	unsigned int i = 0;

	lockdep_assert_rq_held(src_rq);

	if (!has_runnable_rt_tasks(src_rq))
		return;

	plist_for_each_entry(p, head, pushable_tasks) {
		trace_printk("OPLUS_LB_RT[%d]: i=%d, task=%s$%d\n",
				__LINE__, ++i, p->comm, p->pid);
	}
}

/*
 * oplus_detach_running_task() -- Pick the task that was just running on the cpu.
 *
 * Returns a task if successful and NULL otherwise.
 */
static struct task_struct *oplus_detach_running_task_for_rt(struct lb_env *env)
{
	struct rq *rq = env->src_rq;
	struct plist_head *head = &rq->rt.pushable_tasks;
	struct oplus_rq *orq = (struct oplus_rq *) rq->android_oem_data1;
	pid_t pid = orq->lb.pid;
	struct task_struct *p;

#ifdef DEBUG_LB_RT_TICK
	debug_dump_rt_pushable_tasks(env);
#endif

	lockdep_assert_rq_held(env->src_rq);

	/*
	 * NOTE:
	 * The tasks to be migrated may have been enqueued to
	 * the runqueue of other CPUs when the migration thread
	 * is awakened for execution, but anyway, orq->lb.pid
	 * needs to be cleared here.
	 */
	orq->lb.pid = INVALID_PID;

	plist_for_each_entry(p, head, pushable_tasks) {
#ifdef DEBUG_LB_RT_TICK
		trace_printk("DEBUG_LB_RT_TICK[%d]: p=%s$%d, lb.pid=%d "
			"state=%d, on_cpu=%d, on_rq=%d, task_cpu=%d\n",
			__LINE__, p->comm, p->pid, pid,
			READ_ONCE(p->__state), p->on_cpu, p->on_rq, task_cpu(p));
#endif

		/* Pick the task that was just running on the cpu. */
		if (p->pid != pid)
			continue;

		if (!can_migrate_task(p, env))
			continue;

		detach_task(p, env);

		/*
		 * Right now, this is only the second place where
		 * lb_gained[env->idle] is updated (other is detach_tasks)
		 * so we can safely collect stats here rather than
		 * inside detach_tasks().
		 */
		/*
		 * del by oplus.
		 * schedstat_inc(env->sd->lb_gained[env->idle]);
		 */
		return p;
	}
	return NULL;
}

/*
 * active_load_balance_cpu_stop is run by the CPU stopper. It pushes
 * running tasks off the busiest CPU onto idle CPUs. It requires at
 * least 1 task to be running on each physical CPU where possible, and
 * avoids physical / logical imbalances.
 */
static int oplus_active_load_balance_cpu_stop_for_rt(void *data)
{
	struct rq *busiest_rq = data;
	int busiest_cpu = cpu_of(busiest_rq);
	int target_cpu = busiest_rq->push_cpu;
	struct rq *target_rq = cpu_rq(target_cpu);
	struct sched_domain *sd;
	struct task_struct *p = NULL;
	struct rq_flags rf;
	struct oplus_rq *orq = (struct oplus_rq *) busiest_rq->android_oem_data1;
#ifdef DEBUG_LB_RT_TICK
	struct task_struct *task = NULL;
#endif

	rq_lock_irq(busiest_rq, &rf);

#ifdef DEBUG_LB_RT_TICK
	trace_printk("DEBUG_LB_RT_TICK[%d]: src_cpu=%d, dst_cpu=%d, local_cpu=%d, "
		"active=%d-%d, active_balance=%d-%d, nr_running=%d-%d, pid=%d\n",
		__LINE__, busiest_cpu, target_cpu, smp_processor_id(),
		cpu_active(busiest_cpu), cpu_active(target_cpu),
		busiest_rq->active_balance, target_rq->active_balance,
		busiest_rq->nr_running, target_rq->nr_running,
		orq->lb.pid);

	task = find_task_by_vpid(orq->lb.pid);
	trace_printk("DEBUG_LB_RT_TICK[%d]: task=0x%llx\n", __LINE__, (unsigned long long)task);
	if (task) {
		trace_printk("DEBUG_LB_RT_TICK[%d]: task=%s$%d, state=%d, on_cpu=%d, on_rq=%d, task_cpu=%d\n",
			__LINE__, task->comm, task->pid,
			READ_ONCE(task->__state), task->on_cpu, task->on_rq, task_cpu(task));
	}
#endif

	/*
	 * Between queueing the stop-work and running it is a hole in which
	 * CPUs can become inactive. We should not move tasks from or to
	 * inactive CPUs.
	 */
	if (!cpu_active(busiest_cpu) || !cpu_active(target_cpu))
		goto out_unlock;

	/* Make sure the requested CPU hasn't gone down in the meantime: */
	if (unlikely(busiest_cpu != smp_processor_id() ||
		     !busiest_rq->active_balance))
		goto out_unlock;

	/* Is there any task to move? */
	if (busiest_rq->nr_running < 1)
		goto out_unlock;

	/*
	 * This condition is "impossible", if it occurs
	 * we need to fix it. Originally reported by
	 * Bjorn Helgaas on a 128-CPU setup.
	 */
	WARN_ON(busiest_rq == target_rq);

	/* Search for an sd spanning us and the target CPU. */
	rcu_read_lock();
	for_each_domain(target_cpu, sd) {
		if (cpumask_test_cpu(busiest_cpu, sched_domain_span(sd)))
			break;
	}

	if (likely(sd)) {
		struct lb_env env = {
			.sd		= sd,
			.dst_cpu	= target_cpu,
			.dst_rq		= target_rq,
			.src_cpu	= busiest_rq->cpu,
			.src_rq		= busiest_rq,
			.idle		= CPU_IDLE,
			.flags		= LBF_ACTIVE_LB,
			.src_rq_rf	= &rf,
		};

		/*
		 * del by oplus.
		 * schedstat_inc(sd->alb_count);
		 */
		update_rq_clock(busiest_rq);

#ifdef DEBUG_LB_RT_TICK
		trace_printk("DEBUG_LB_RT_TICK[%d]: detach!!!\n", __LINE__);
#endif

		/*
		 * The native kernel picks a task from the end of the rq->cfs_tasks
		 * list through detach_one_task, but this task is not the one that
		 * is just running on the cpu, so we made a little adjustment here.
		 * p = detach_one_task(&env);
		 */
		p = oplus_detach_running_task_for_rt(&env);
		if (p) {
			/*
			 * del by oplus.
			 * schedstat_inc(sd->alb_pushed);
			 */
			/* Active balancing done, reset the failure counter. */
			sd->nr_balance_failed = 0;
#ifdef DEBUG_LB_RT_TICK
			trace_printk("DEBUG_LB_RT_TICK[%d]: Success!!! "
				"src_cpu=%d, dst_cpu=%d, p=%s$%d, pid=%d\n",
				__LINE__, busiest_cpu, target_cpu,
				p->comm, p->pid, orq->lb.pid);
#endif
			orq->lb.pid = INVALID_PID;
		} else {
			/*
			 * del by oplus.
			 * schedstat_inc(sd->alb_failed);
			 */
#ifdef DEBUG_LB_RT_TICK
			trace_printk("DEBUG_LB_RT_TICK[%d]: Failed!!! "
				"src_cpu=%d, dst_cpu=%d\n",
				__LINE__, busiest_cpu, target_cpu);
#endif
		}
	}
	rcu_read_unlock();

out_unlock:
	busiest_rq->active_balance = 0;
	rq_unlock(busiest_rq, &rf);

#ifdef DEBUG_LB_RT_TICK
	trace_printk("DEBUG_LB_RT_TICK[%d]: p=%s, src_nr=%d, dst_nr=%d\n",
		__LINE__, p ? p->comm : "NULL",
		busiest_rq->nr_running, target_rq->nr_running);

	if (task) {
		trace_printk("DEBUG_LB_RT_TICK[%d]: task=%s$%d, state=%d, on_cpu=%d, on_rq=%d, task_cpu=%d\n",
			__LINE__, task->comm, task->pid,
			READ_ONCE(task->__state), task->on_cpu, task->on_rq, task_cpu(task));
	}
#endif
	if (p)
		attach_one_task(target_rq, p);

	local_irq_enable();

	return 0;
}

static bool oplus_migrate_running_ux(void *data, struct rq *rq)
{
	unsigned int prev_cpu = cpu_of(rq);
	struct task_struct *curr = rq->curr;
	int new_cpu = -1;
	bool ret = false;

	if (!test_task_is_fair(curr))
		return false;

	if (!ux_need_up_migration(curr, rq))
		return false;

	/*
	 * No need to do load balance if no suitable cpu is found.
	 */
	new_cpu = find_cpu_in_migration(curr, prev_cpu, up_migr, true);
	if ((new_cpu < 0) || (same_cluster(new_cpu, prev_cpu)))
		return false;

#ifdef DEBUG_LB_TICK
	trace_printk("OPLUS_LB_TICK[%d]: curr=%s$%d, "
		"prev_cpu=%d, new_cpu=%d, valid=%d\n",
		__LINE__, curr->comm, curr->pid, prev_cpu, new_cpu,
		topology_physical_package_id(new_cpu) > topology_physical_package_id(prev_cpu));
#endif

	/*
	 * Check if the migration/X can be woken up.
	 */
	if (!oplus_kick_active_balance(rq, curr, new_cpu))
		return false;

	migr_running_task_systrace(prev_cpu, curr);

	/*
	 * Wake up the migration/X to migrate the running task.
	 */
	ret = stop_one_cpu_nowait(prev_cpu,
		oplus_active_load_balance_cpu_stop, rq,
		&rq->active_balance_work);
	if (ret)
		wake_up_if_idle(prev_cpu);

#ifdef DEBUG_LB_TICK
	trace_printk("OPLUS_LB_TICK[%d]: curr=%s$%d, ret=%d\n",
		__LINE__, curr->comm, curr->pid, ret);
#endif
	return ret;
}

u64 oplus_get_runnable_time_for_rt(int cpu, struct task_struct *p)
{
	if (!p || !test_task_is_rt(p))
		return 0;

	return get_runnable_time(p);
}

u64 oplus_get_runnable_time(int cpu, struct task_struct *p)
{
	struct oplus_task_struct *ots;

	if (!p || !test_task_is_fair(p))
		return 0;

	ots = get_oplus_task_struct(p);
	if (IS_ERR_OR_NULL(ots))
		return 0;

	if (!(ots->ux_state & SCHED_ASSIST_LB_UX))
		return 0;

	return get_runnable_time(p);
}

static struct task_struct *oplus_pick_runnable_ux(
			int src_cpu, int dst_cpu, u64 *time)
{
	struct task_struct *task = NULL;
	struct oplus_task_struct *ots = NULL;
	struct rq *src_rq = cpu_rq(src_cpu);
	struct oplus_rq *orq = (struct oplus_rq *) src_rq->android_oem_data1;
	u64 runnable_time, threshold_time = ULLONG_MAX;
	unsigned long irqflag;
	struct rb_node *node;

	spin_lock_irqsave(orq->ux_list_lock, irqflag);
	if (!orq_has_ux_tasks(orq))
		goto out;

	for (node = rb_first_cached(&orq->ux_list); node; node = rb_next(node)) {
		ots = rb_entry(node, struct oplus_task_struct, ux_entry);
		if (IS_ERR_OR_NULL(ots))
			continue;

		/*
		 * NOTE:
		 * The task may have been destroyed during the shutdown
		 * process, so it is necessary to determine whether the
		 * task is NULL.
		 */
		task = ots_to_ts(ots);
		if (IS_ERR_OR_NULL(task))
			continue;

		/* this ux type does not require balance. */
		if (!(ots->ux_state & SCHED_ASSIST_LB_UX))
			continue;

		/* affinify */
		if (dst_cpu >= 0 && !cpumask_test_cpu(dst_cpu, task->cpus_ptr))
			continue;

		/* pick a task in runnable state. */
		if (!task_is_runnable(task))
			continue;

		/* Determine whether the runnable time has exceeded the threshold. */
		runnable_time = oplus_get_runnable_time(src_cpu, task);
		threshold_time = get_threshold_time(migr_runnable_ux);
#ifdef DEBUG_LB_TICK
		trace_printk("OPLUS_LB_TICK[%d]: task=%s$%d, ux_state=%d, "
			"on_cpu=%d, on_rq=%d, "
			"runnable_time=%llu, threshold_time=%llu, true=%d\n",
			__LINE__, task->comm, task->pid, ots->ux_state,
			task->on_cpu, task->on_rq,
			runnable_time, threshold_time, runnable_time >= threshold_time);
#endif
		if (runnable_time >= threshold_time) {
			if (time)
				*time = runnable_time;

			spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
			return task;
		} else {
			continue;
		}
	}

out:
	spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
	return NULL;
}

static bool oplus_migrate_runnable_ux(void *data, struct rq *rq)
{
	struct task_struct *ux_task = NULL;
	unsigned int this_cpu = cpu_of(rq);
	int new_cpu = -1;
	bool ret = false;

	/*
	 * Pick a ux_task that has been in the runnable state for a long time.
	 */
	ux_task = oplus_pick_runnable_ux(this_cpu, -1, NULL);
	if (!ux_task)
		return false;

	/*
	 * Choose a suitable cpu for this ux_task.
	 */
	new_cpu = find_cpu_in_migration(ux_task, this_cpu, normal_migr, false);
	if (new_cpu < 0)
		return false;

	/*
	 * Check if the migration/X can be woken up.
	 */
	if (!oplus_kick_active_balance(rq, ux_task, new_cpu))
		return false;

	migr_runnable_task_systrace(this_cpu, ux_task);

	/*
	 * Wake up the migration/X to migrate the runnable task.
	 */
	ret = stop_one_cpu_nowait(this_cpu,
		oplus_active_load_balance_cpu_stop, rq,
		&rq->active_balance_work);

	return ret;
}


/**** rt-boost implementation. ****/

DEFINE_SPINLOCK(rbt_lock);
struct plist_head rt_boost_task;

static inline int has_rt_boost_tasks(void)
{
	return !plist_head_empty(&rt_boost_task);
}

int im_flag_to_prio(int im_flag)
{
	/*
	 * Currently, we only boost the following 3 types of im_flag tasks.
	 */
	if (im_flag == IM_FLAG_AUDIO)
		return 1;
	else if (im_flag == IM_FLAG_SURFACEFLINGER)
		return 2;
	else if (im_flag == IM_FLAG_RENDERENGINE)
		return 3;

	return -1;
}

void add_rt_boost_task(struct task_struct *p)
{
	int im_flag, prio;
	unsigned long irqflag;
	struct oplus_task_struct *ots;

	if(!p || !test_task_is_rt(p))
		return;

	im_flag = oplus_get_im_flag(p);
	prio = im_flag_to_prio(im_flag);
	if (prio < 0)
		return;

	ots = get_oplus_task_struct(p);
	if (IS_ERR_OR_NULL(ots))
		return;

	/* Sort according to the priority obtained by im_flag mapping. */
	spin_lock_irqsave(&rbt_lock, irqflag);
	plist_del(&ots->rtb, &rt_boost_task);
	plist_node_init(&ots->rtb, prio);
	plist_add(&ots->rtb, &rt_boost_task);
	spin_unlock_irqrestore(&rbt_lock, irqflag);
}

/*
 * NOTE:
 * When the task is destroyed, the task needs to be removed from the
 * rt_boost linked list, otherwise it may cause a crash due to access
 * to an illegal address.
 */
void remove_rt_boost_task(struct task_struct *p)
{
	int im_flag, prio;
	unsigned long irqflag;
	struct oplus_task_struct *ots;

	/*
	 * FBI WARNING
	 * If an rt_boost task has changed from an rt task to cfs task
	 * before being destroyed, the !test_task_is_rt here will cause
	 * the task to be unable to be removed from the rt_boost linked
	 * list. However, the memory space corresponding to ots has been
	 * released at this time, which may cause unexpected things to
	 * happen, such as memory out-of-bounds access or infinite loop.
	 */
	if(!p /* || !test_task_is_rt(p) */)
		return;

	ots = get_oplus_task_struct(p);
	if (IS_ERR_OR_NULL(ots))
		return;

	im_flag = oplus_get_im_flag(p);
	prio = im_flag_to_prio(im_flag);
	if (prio < 0)
		return;

	/* Sort according to the priority obtained by im_flag mapping. */
	spin_lock_irqsave(&rbt_lock, irqflag);
	plist_del(&ots->rtb, &rt_boost_task);
	plist_node_init(&ots->rtb, MAX_IM_FLAG_PRIO);

	/*
	 * NOTE:
	 * A memory barrier is needed here to prevent access to the
	 * rt_boost_task linked list operation in the task_is_rt_boost
	 * function prior to the plist_del operation above.
	 *
	 * See link below and ALM:5878635 for more information.
	 * http://lkml.kernel.org/r/20170414223138.GA4222@fury
	 * http://lkml.kernel.org/r/20170322104151.604296452@infradead.org
	 */
	smp_mb();
	spin_unlock_irqrestore(&rbt_lock, irqflag);
}

struct task_struct *pick_rt_boost_task(void)
{
	unsigned long irqflag;
	struct oplus_task_struct *ots;
	struct plist_head *plist = &rt_boost_task;

	spin_lock_irqsave(&rbt_lock, irqflag);
	if (!has_rt_boost_tasks()) {
		spin_unlock_irqrestore(&rbt_lock, irqflag);
		return NULL;
	}

	ots = plist_first_entry(plist,
				struct oplus_task_struct, rtb);
	spin_unlock_irqrestore(&rbt_lock, irqflag);

	/*
	 * NOTE:
	 * The return value of this function may be NULL, so the
	 * caller must check the return value.
	 */
	return ots_to_ts(ots);
}

bool task_is_rt_boost(struct task_struct *task)
{
	struct task_struct *tmp;
	struct oplus_task_struct *ots, *tmp_ots;
	unsigned long irqflag;
	bool ret = false;

	if (!task || !test_task_is_rt(task))
		return false;

	if (!has_rt_boost_tasks())
		return false;

	spin_lock_irqsave(&rbt_lock, irqflag);
	plist_for_each_entry_safe(ots, tmp_ots, &rt_boost_task, rtb) {
		/*
		 * There is no need to check whether tmp is NULL here.
		 */
		tmp = ots_to_ts(ots);
		if (tmp == task) {
			ret = true;
			break;
		}
	}
	spin_unlock_irqrestore(&rbt_lock, irqflag);

	return ret;
}

void release_rt_boost_task(void)
{
	unsigned long irqflag;
	struct oplus_task_struct *ots, *tmp;

	if (!has_rt_boost_tasks())
		return;

	spin_lock_irqsave(&rbt_lock, irqflag);
	plist_for_each_entry_safe(ots, tmp, &rt_boost_task, rtb) {
		plist_del(&ots->rtb, &rt_boost_task);
		plist_node_init(&ots->rtb, MAX_IM_FLAG_PRIO);
	}
	plist_head_init(&rt_boost_task);
	spin_unlock_irqrestore(&rbt_lock, irqflag);
	return;
}

/**** Tick balance implementation. ****/

bool rt_need_up_migration(struct task_struct *p, struct rq *rq)
{
	u64 running_time, threshold_time = ULLONG_MAX;

	if (!p || !rq)
		return false;

	/* We only allow rt to migrate from silver to gold or prime core. */
	if (topology_physical_package_id(cpu_of(rq)))
		return false;

	running_time = get_running_time(p);
	threshold_time = get_threshold_time(migr_running_rt_boost);

#ifdef DEBUG_LB_RT_TICK
	oplus_loadbalance_systrace_print(OPLUS_LB_SYSTRACE_PID,
				"rt_running_time", cpu_of(rq), running_time);

	trace_printk("DEBUG_LB_RT_TICK[%d]: task=%s$%d, mask=[%*pbl], "
		"cpu=%d, cls=%d, running_time=%llu, threshold_time=%llu, true=%d\n",
		__LINE__, p->comm, p->pid, cpumask_pr_args(p->cpus_ptr),
		cpu_of(rq), topology_physical_package_id(cpu_of(rq)),
		running_time, threshold_time, running_time >= threshold_time);
#endif

	return running_time >= threshold_time;
}

static bool oplus_migrate_running_rt(void *data,
			struct rq *rq, bool rt_boost)
{
	unsigned int prev_cpu = cpu_of(rq);
	struct task_struct *curr = rq->curr;
	int new_cpu = -1;
	bool ret = false;

	if(!test_task_is_rt(curr))
		return false;

	/*
	 * Migrate tasks in the rt_boost group.
	 */
	if (rt_boost && !task_is_rt_boost(curr))
		return false;

	if (!rt_need_up_migration(curr, rq))
		return false;

	/*
	 * No need to do load balance if no suitable cpu is found.
	 */
	new_cpu = find_cpu_in_migration(curr, prev_cpu, up_migr, true);
	if ((new_cpu < 0) || (same_cluster(new_cpu, prev_cpu)))
		return false;

#ifdef DEBUG_LB_RT_TICK
	trace_printk("DEBUG_LB_RT_TICK[%d]: curr=%s$%d, "
		"rt_boost=%d, prev_cpu=%d, new_cpu=%d, valid=%d\n",
		__LINE__, curr->comm, curr->pid,
		task_is_rt_boost(curr), prev_cpu, new_cpu,
		topology_physical_package_id(new_cpu) > topology_physical_package_id(prev_cpu));
#endif

	/*
	 * Check if the migration/X can be woken up.
	 */
	if (!oplus_kick_active_balance(rq, curr, new_cpu))
		return false;

	migr_running_rt_systrace(prev_cpu, curr);

	/*
	 * Wake up the migration/X to migrate the running task.
	 */
	ret = stop_one_cpu_nowait(prev_cpu,
		oplus_active_load_balance_cpu_stop_for_rt, rq,
		&rq->active_balance_work);

#ifdef DEBUG_LB_RT_TICK
	trace_printk("DEBUG_LB_RT_TICK[%d]: curr=%s$%d, "
		"state=%d, on_cpu=%d, on_rq=%d, task_cpu=%d, "
		"rt_boost=%d, ret=%d\n",
		__LINE__, curr->comm, curr->pid,
		READ_ONCE(curr->__state), curr->on_cpu, curr->on_rq, task_cpu(curr),
		task_is_rt_boost(curr), ret);
#endif
	return ret;
}

static struct task_struct *oplus_pick_runnable_rt_boost(
				int src_cpu, int dst_cpu, u64 *time);

static bool oplus_migrate_runnable_rt(void *data,
			struct rq *rq, bool rt_boost)
{
	struct ux_sched_cputopo ux_cputopo = ux_sched_cputopo;
	cpumask_t search_cpus = CPU_MASK_NONE;
	struct task_struct *curr = rq->curr;
	struct task_struct *rt_task = NULL;
	struct rq *iter_rq = NULL;
	struct rq *busiest_rq = NULL;
	struct oplus_rq *iter_orq = NULL;
	struct oplus_rq *orq = (struct oplus_rq *)rq->android_oem_data1;
	struct rq_flags rf;
	int this_cpu = cpu_of(rq);
	int cur_cls = topology_physical_package_id(this_cpu);
	int order_idx = -1, walk_cnt = -1, idx = -1;
	int iter_cpu = -1, busiest_cpu = -1;

	/*
	 * Do not pull tasks from other CPUs if the running task on
	 * the CPU is rt or ux.
	 */
	if(test_task_is_rt(curr))
		return false;

	if (get_ux_state(curr) & POSSIBLE_UX_MASK)
		return false;

	/*
	 * Do not pull tasks from other CPUs if there is a ux or rt
	 * tasks in runnable state on this_cpu.
	 */
	if (orq_has_ux_tasks(orq))
		return false;

	if (rt_rq_is_runnable(&rq->rt))
		return false;

	/*
	 * Calculate order_idx and walk_cnt.
	 */
	if (!calc_order_idx(tickpull_migr, cur_cls, &order_idx, &walk_cnt))
		return false;

#ifdef DEBUG_LB_TICK
	trace_printk("OPLUS_LB_TICKPULL[%d]: migr_type=%d, this_cpu=%d, cur_cls=%d, "
		"order_idx=%d, walk_cnt=%d\n",
		__LINE__, tickpull_migr, this_cpu, cur_cls, order_idx, walk_cnt);
#endif

	/*
	 * Find a cpu on which there is a task in rt_boost group that has been
	 * in the runnable state for a long time.
	 */
	for (idx = 0; idx < walk_cnt; idx++) {
		cpumask_copy(&search_cpus, &ux_cputopo.oplus_cpu_array[order_idx][idx]);

		for_each_cpu(iter_cpu, &search_cpus) {
			iter_rq = cpu_rq(iter_cpu);
			iter_orq = (struct oplus_rq *) iter_rq->android_oem_data1;

			/*
			 * Cannot migrate to itself.
			 */
			if (iter_cpu == this_cpu)
				continue;

			/*
			 * Only allow to pull tasks from the active cpu.
			 */
			if (!cpu_online(iter_cpu) || !cpu_active(iter_cpu))
				continue;

#ifdef CONFIG_OPLUS_ADD_CORE_CTRL_MASK
			if (oplus_cpu_halted(iter_cpu))
				continue;
#endif

			busiest_cpu = iter_cpu;
			busiest_rq = cpu_rq(busiest_cpu);
			rq_lock(busiest_rq, &rf);

			/*
			 * Skip if there is no rt task in the runnable state on this cpu.
			 */
			if (!has_runnable_rt_tasks(busiest_rq)) {
				rq_unlock(busiest_rq, &rf);
				continue;
			}

			/*
			 * Skip if there is no rt_boost task in the runnable state on this cpu.
			 */
			if (!has_rt_boost_tasks()) {
				rq_unlock(busiest_rq, &rf);
				continue;
			}

			/*
			 * Step 1:
			 * Pick a ux_task that has been in the runnable state for a long time.
			 */
			rt_task = oplus_pick_runnable_rt_boost(iter_cpu, this_cpu, NULL);
			if (!rt_task) {
				rq_unlock(busiest_rq, &rf);
				continue;
			}

#ifdef DEBUG_LB_RT_TICK
			trace_printk("OPLUS_LB_TICKPULL[%d]: this_cpu=%d, curr=%s$%d, "
				"busiest_cpu=%d, rt_task=%s$%d,\n",
				__LINE__, this_cpu, curr->comm, curr->pid,
				busiest_cpu, rt_task->comm, rt_task->pid);
#endif

			/*
			 * Step 2:
			 * Ha, rt_task can be migrated to this_cpu to perform enqueue
			 * and dequeue operations.
			 */
			deactivate_task(busiest_rq, rt_task, DEQUEUE_NOCLOCK);
			set_task_cpu(rt_task, this_cpu);
			rq_unlock(busiest_rq, &rf);

			/*
			 * NOTE:
			 * A CPU that has entered the idle state may be selected
			 * in tick_balance, so you need to actively call preempt_curr
			 * to send an ipi interrupt to wake it up.
			 */
			rq_lock(rq, &rf);
			attach_task(rq, rt_task);
			rq_unlock(rq, &rf);

			oplus_loadbalance_systrace_print(OPLUS_LB_SYSTRACE_PID,
					"tick_lb_runnable_rt_cpu", busiest_cpu, rt_task->pid);

			return true;
		}
	}

	return false;
}

#if defined(DEBUG_LB_TICK)
static void cpuidle_exit_latency_systrace(
			unsigned int cpu, unsigned int exit_latency)
{
	char buf[256];

	if (unlikely(global_debug_enabled & DEBUG_SYSTRACE)) {
		snprintf(buf, sizeof(buf), "C|%d|cpuidle_exit_latency[%d]|%d\n",
						OPLUS_LB_SYSTRACE_PID, cpu, exit_latency);
		tracing_mark_write(buf);
	}
}

static void dump_cpu_state(void)
{
	struct cpuidle_state *idle;
	int i;

	for (i = 0; i < OPLUS_NR_CPUS; i++) {
		idle = idle_get_state(cpu_rq(i));

		trace_printk("DEBUG_LB_TICK[%d]: cpu=%d, name=%s, desc=%s,"
				" exit_latency=%dus, online=%d, active=%d, idle=%d,"
				" available_idle=%d, nr_running=%d, h_nr_running=%d\n",
			__LINE__, i,
			idle?idle->name:"NULL", idle?idle->desc:"NULL",
			idle?idle->exit_latency:0, cpu_online(i), cpu_active(i),
			oplus_idle_cpu(i), available_idle_cpu(i),
			cpu_rq(i)->nr_running, cpu_rq(i)->cfs.h_nr_running);

		cpuidle_exit_latency_systrace(i, idle?idle->exit_latency:0);
	}
}
#endif

bool __oplus_tick_balance(void *data, struct rq *rq)
{
	if (unlikely(!lb_enable))
		return false;

#if defined(DEBUG_LB_TICK)
	dump_cpu_state();
	(void) __get_time(rq->curr, true);
#endif

	/*
	 * Update the total number of ticks.
	 */
	atomic64_inc(&lb_stat.tick_hit);

	/*
	 * Migrate long-running/runnable tasks.
	 *
	 * NOTE:
	 * Only one type of task can be migrated at a time, because
	 * tick_balance needs to wake up migration/X tasks to migrate
	 * running/runnable tasks. Migration operations are performed
	 * in the following order:
	 * a) tasks in the rt_boost group.
	 * b) long running ux tasks.
	 * c) long runnable ux tasks.
	 * d) Normal rt tasks NOT in the rt_boost group.
	 */
	if (oplus_migrate_running_rt(data, rq, true)) {
		atomic64_inc(&lb_stat.tick_running_rt_boost_succ);
		return true;
	}

	if (oplus_migrate_runnable_rt(data, rq, true)) {
		atomic64_inc(&lb_stat.tick_runnable_rt_boost_succ);
		return true;
	}

	if (oplus_migrate_running_ux(data, rq)) {
		atomic64_inc(&lb_stat.tick_running_ux_succ);
		return true;
	}

	if (oplus_migrate_runnable_ux(data, rq)) {
		atomic64_inc(&lb_stat.tick_runnable_ux_succ);
		return true;
	}

	if (oplus_migrate_running_rt(data, rq, false)) {
		atomic64_inc(&lb_stat.tick_running_normal_rt_succ);
		return true;
	}

	atomic64_inc(&lb_stat.tick_fail);

	return false;
}
EXPORT_SYMBOL(__oplus_tick_balance);

__maybe_unused static void oplus_tick_balance(void *data, struct rq *rq)
{
	__oplus_tick_balance(data, rq);
}

/**** Newidle balance implementation. ****/

/*
 * idle threshold = 0.5ms
 * The newidle balance may be triggered only when the average
 * idle time exceeds 0.5ms, which means that it may be difficult
 * to trigger in high-load scenarios.
 *
 * cluster util threshold = 90%
 * Although the average sleep time is not enough, this threshold
 * ensures that there is still remaining capacity in the cluster
 * where the CPU is located.
 */
#define NEWIDLE_BALANCE_IDLE_THRESHOLD		500000
#define NEWIDLE_BALANCE_CLS_UTIL_MINMAX_THRESHOLD	95
#define NEWIDLE_BALANCE_CLS_UTIL_MID_THRESHOLD	80
#define MIN_CPU		0
#define MAX_CPU		(OPLUS_NR_CPUS - 1)
#define PERCENTAGE	100

inline unsigned long capacity_of(int cpu)
{
	return cpu_rq(cpu)->cpu_capacity;
}

inline unsigned long cpu_util(int cpu)
{
	struct cfs_rq *cfs_rq;
	unsigned int util;

	cfs_rq = &cpu_rq(cpu)->cfs;
	util = READ_ONCE(cfs_rq->avg.util_avg);

	if (sched_feat(UTIL_EST))
		util = max(util, READ_ONCE(cfs_rq->avg.util_est.enqueued));

	return min_t(unsigned long, util, capacity_orig_of(cpu));
}

#define fits_capacity(util, max)	((util) * 1280 < (max) * 1024)


static struct task_struct *oplus_pick_runnable_rt_boost(
				int src_cpu, int dst_cpu, u64 *time)
{
	struct task_struct *task = NULL;
	struct oplus_task_struct *ots = NULL, *tmp_ots;
	struct plist_head *phead;
	u64 runnable_time, threshold_time = ULLONG_MAX;
	unsigned long irqflag;
	int im_flag;

	phead = &rt_boost_task;
	spin_lock_irqsave(&rbt_lock, irqflag);

	/*
	 * no task in the rt_boost group.
	 */
	if (!has_rt_boost_tasks()) {
		spin_unlock_irqrestore(&rbt_lock, irqflag);
		return NULL;
	}

	/*
	 * Check all tasks in the rt_boost group in turn according to
	 * the priority obtained from the im_flag mapping relationship.
	 */
	plist_for_each_entry_safe(ots, tmp_ots, phead, rtb) {
		/*
		 * NOTE:
		 * The task may have been destroyed during the shutdown
		 * process, so it is necessary to determine whether the
		 * task is NULL.
		 */
		task = ots_to_ts(ots);
		if (IS_ERR_OR_NULL(task))
			continue;

		/*
		 * Remove task from rt_boost group if it is in TASK_DEAD state.
		 */
		if (READ_ONCE(task->__state) == TASK_DEAD) {
			plist_del(&ots->rtb, &rt_boost_task);
			plist_node_init(&ots->rtb, MAX_IM_FLAG_PRIO);
			continue;
		}

		/*
		 * The tasks on rt_boost may belong to other cpus because
		 * re_boost is shared globally. We only focus on tasks on
		 * src_cpu.
		 */
		if (src_cpu != task_cpu(task))
			continue;

		/*
		 * It is possible that tasks in the rt_boost group are not
		 * on the runqueue and have entered a blocked state.
		 */
		if (!task_is_runnable(task))
			continue;

		/*
		 * check affinify
		 */
		if (!cpumask_test_cpu(dst_cpu, &task->cpus_mask))
			continue;

		/*
		 * NOTE:
		 * The system may crash because the two variables pus_ptr
		 * and cpus_mask are not equal.
		 */
		if (!cpumask_test_cpu(dst_cpu, task->cpus_ptr)) {
			continue;
		}

		/*
		 * Pick a task that has been in the runnable state for a long time.
		 */
		im_flag = oplus_get_im_flag(task);
		if (im_flag == IM_FLAG_AUDIO) {
			threshold_time = get_threshold_time(rt_boost_runnable_audio);
		} else if (im_flag == IM_FLAG_SURFACEFLINGER) {
			threshold_time = get_threshold_time(rt_boost_runnable_surfaceflinger);
		} else if (im_flag == IM_FLAG_RENDERENGINE) {
			threshold_time = get_threshold_time(rt_boost_runnable_renderengine);
		} else {
			threshold_time = ULLONG_MAX;
		}
		runnable_time = oplus_get_runnable_time_for_rt(src_cpu, task);

#ifdef DEBUG_LB_NEWIDLE
		trace_printk("DEBUG_LB_NEWIDLE[%d]: task=%s$%d, "
			"runnable_time=%llu, threshold_time=%llu, true=%d\n",
			__LINE__, task->comm, task->pid,
			runnable_time, threshold_time, runnable_time >= threshold_time);
#endif
		if (runnable_time >= threshold_time) {
			if (time)
				*time = runnable_time;

			spin_unlock_irqrestore(&rbt_lock, irqflag);
			return task;
		} else {
			continue;
		}
	}
	spin_unlock_irqrestore(&rbt_lock, irqflag);

	return NULL;
}

static struct task_struct *oplus_pick_runnable_rt_normal(
				int src_cpu, int dst_cpu, u64 *time)
{
	struct rq *src_rq = cpu_rq(src_cpu);
	struct plist_head *head = &src_rq->rt.pushable_tasks;
	struct task_struct *p = NULL;
	u64 runnable_time, threshold_time = ULLONG_MAX;

	/*
	 * Skip if there is no normal_rt task in the runnable
	 * state on this cpu.
	 */
	if (!has_runnable_rt_tasks(src_rq))
		return NULL;

	/*
	 * Check the RT threads in the runnable state on this CPU one by one.
	 */
	plist_for_each_entry(p, head, pushable_tasks) {
		/*
		 * skip running rt thread.
		 */
		if (task_running(src_rq, p))
			continue;

		/*
		 * check affinify
		 */
		if (!cpumask_test_cpu(dst_cpu, &p->cpus_mask))
			continue;

		/*
		 * NOTE:
		 * The system may crash because the two variables pus_ptr
		 * and cpus_mask are not equal.
		 */
		if (!cpumask_test_cpu(dst_cpu, p->cpus_ptr)) {
			continue;
		}

		/*
		 * Skip the rt_boost task.
		 */
		if (task_is_rt_boost(p)) {
			continue;
		}

		/*
		 * Whether the runnable time of the normal_rt task exceeds
		 * the threshold.
		 *
		 * Note:
		 * Frequent migration of normal_rt tasks may cause performance
		 * problems, so we have added a limit to the runnable time here.
		 */
		threshold_time = get_threshold_time(normal_rt_runnable);
		runnable_time = oplus_get_runnable_time_for_rt(src_cpu, p);
		if (runnable_time < threshold_time) {
			continue;
		}

#ifdef DEBUG_LB_NEWIDLE
		trace_printk("DEBUG_LB_NEWIDLE[%d]: task=%s$%d$%d, "
			"runnable_time=%llu, threshold_time=%llu, true=%d\n",
			__LINE__, p->comm, p->pid, p->prio,
			runnable_time, threshold_time, runnable_time >= threshold_time);
#endif

		return p;
	}

	return NULL;
}

/*
 * pick a rt task from rt_boost group.
 *
 * NOTE:
 * When checking the tasks in the rt_boost group, as long as a rt_task
 * in the runnable state is found to be timed out, the check will be
 * exited immediately.
 */
static bool oplus_newidle_balance_pull_runnable_rt_boost(
					struct rq *this_rq, struct rq_flags *rf,
					int *pulled_task, int *done,
					int order_idx, int walk_cnt)
{
	struct ux_sched_cputopo ux_cputopo = ux_sched_cputopo;
	cpumask_t search_cpus = CPU_MASK_NONE;
	struct rq *rq = NULL;
	struct rq *busiest_rq = NULL;
	struct oplus_rq *orq = NULL;
	struct task_struct *rt_task = NULL;
	int this_cpu = cpu_of(this_rq);
	int cpu = -1;
	int busiest_cpu = -1;
	int idx = -1;

	/* can't help if this has a runnable RT */
	if (sched_rt_runnable(this_rq))
		return false;

#ifdef DEBUG_LB_NEWIDLE_HIT
	trace_printk("DEBUG_LB_NEWIDLE_HIT[%d]: cpu=%d, "
		"order_idx=%d, walk_cnt=%d\n",
		__LINE__, this_cpu, order_idx, walk_cnt);
#endif

	/*
	 * Find a cpu on which there is a task in rt_boost group that has been
	 * in the runnable state for a long time.
	 */
	for (idx = 0; idx < walk_cnt; idx++) {
		cpumask_copy(&search_cpus, &ux_cputopo.oplus_cpu_array[order_idx][idx]);

		for_each_cpu(cpu, &search_cpus) {
			rq = cpu_rq(cpu);
			orq = (struct oplus_rq *) rq->android_oem_data1;

			/*
			 * Cannot migrate to itself.
			 */
			if (cpu == this_cpu)
				continue;

			/*
			 * Only allow to pull tasks from the active cpu.
			 */
			if (!cpu_online(cpu) || !cpu_active(cpu))
				continue;

#ifdef CONFIG_OPLUS_ADD_CORE_CTRL_MASK
			if (oplus_cpu_halted(cpu))
				continue;
#endif

			busiest_cpu = cpu;
			busiest_rq = cpu_rq(busiest_cpu);
			double_lock_balance(this_rq, busiest_rq);

			/*
			 * Skip if there is no rt task in the runnable state on this cpu.
			 */
			if (!has_runnable_rt_tasks(busiest_rq)) {
				double_unlock_balance(this_rq, busiest_rq);
				continue;
			}

			/*
			 * Skip if there is no rt_boost task in the runnable state on this cpu.
			 */
			if (!has_rt_boost_tasks()) {
				double_unlock_balance(this_rq, busiest_rq);
				continue;
			}

			/*
			 * Step 1:
			 * Pick a ux_task that has been in the runnable state for a long time.
			 */
			rt_task = oplus_pick_runnable_rt_boost(cpu, this_cpu, NULL);
			if (!rt_task) {
				double_unlock_balance(this_rq, busiest_rq);
				continue;
			}

			/*
			 * Step 2:
			 * Ha, rt_task can be migrated to this_cpu to perform enqueue
			 * and dequeue operations.
			 */
			deactivate_task(busiest_rq, rt_task, 0);
			set_task_cpu(rt_task, this_cpu);
			activate_task(this_rq, rt_task, 0);

			/*
			 * Only pull one task at a time.
			 */
			*pulled_task = 1;
			*done = 1;

			/*
			 * reset the idle time stamp if we pulled any task.
			 */
			this_rq->idle_stamp = 0;

			oplus_loadbalance_systrace_print(OPLUS_LB_SYSTRACE_PID,
					"newidle_migr_rt_boost_cpu", busiest_cpu, rt_task->pid);

			double_unlock_balance(this_rq, busiest_rq);
			return true;
		}
	}

	return false;
}

/*
 * pick a normal_rt task.
 *
 * Note:
 * We only pay attention to the throughput of the rt task,
 * not whether the runnable of this rt task is timeout.
 */
static bool oplus_newidle_balance_pull_runnable_rt_normal(
					struct rq *this_rq, struct rq_flags *rf,
					int *pulled_task, int *done,
					int order_idx, int walk_cnt)
{
	struct ux_sched_cputopo ux_cputopo = ux_sched_cputopo;
	cpumask_t search_cpus = CPU_MASK_NONE;
	struct rq *rq = NULL;
	struct rq *busiest_rq = NULL;
	struct oplus_rq *orq = NULL;
	struct task_struct *rt_task = NULL;
	int this_cpu = cpu_of(this_rq);
	int cpu = -1, busiest_cpu = -1;
	int idx;

	/* can't help if this has a runnable RT */
	if (sched_rt_runnable(this_rq))
		return false;

#ifdef DEBUG_LB_NEWIDLE_HIT
	trace_printk("DEBUG_LB_NEWIDLE_HIT[%d]: cpu=%d, "
		"order_idx=%d, walk_cnt=%d\n",
		__LINE__, this_cpu, order_idx, walk_cnt);
#endif

	/*
	 * Find a cpu whose runqueue has a normal_rt task in the
	 * runnable state.
	 *
	 * Note:
	 * We only pay attention to the throughput of the rt task,
	 * not whether the runnable of this rt task is timeout.
	 */
	for (idx = 0; idx < walk_cnt; idx++) {
		cpumask_copy(&search_cpus, &ux_cputopo.oplus_cpu_array[order_idx][idx]);

		for_each_cpu(cpu, &search_cpus) {
			rq = cpu_rq(cpu);
			orq = (struct oplus_rq *) rq->android_oem_data1;

			/*
			 * Cannot migrate to itself.
			 */
			if (cpu == this_cpu)
				continue;

			/*
			 * Only allow to pull tasks from the active cpu.
			 */
			if (!cpu_online(cpu) || !cpu_active(cpu))
				continue;

#ifdef CONFIG_OPLUS_ADD_CORE_CTRL_MASK
			if (oplus_cpu_halted(cpu))
				continue;
#endif

			/* got it! */
			busiest_cpu = cpu;
			busiest_rq = cpu_rq(busiest_cpu);
			double_lock_balance(this_rq, busiest_rq);

			/*
			 * pick an rt task that is in the runnable state
			 * and has the highest priority.
			 */
			rt_task = oplus_pick_runnable_rt_normal(cpu, this_cpu, NULL);
			if (!rt_task) {
				double_unlock_balance(this_rq, busiest_rq);
				continue;
			}

			/*
			 * Migrate this rt task to this_cpu.
			 */
			deactivate_task(busiest_rq, rt_task, 0);
			set_task_cpu(rt_task, this_cpu);
			activate_task(this_rq, rt_task, 0);

			/*
			 * Only pull one task at a time.
			 */
			*pulled_task = 1;
			*done = 1;

			/*
			 * reset the idle time stamp if we pulled any task
			 */
			this_rq->idle_stamp = 0;

			oplus_loadbalance_systrace_print(OPLUS_LB_SYSTRACE_PID,
					"newidle_migr_rt_normal_cpu", busiest_cpu, rt_task->pid);

			double_unlock_balance(this_rq, busiest_rq);

			return true;
		}
	}
	return false;
}

static bool oplus_newidle_balance_pull_runnable_ux(
			struct rq *this_rq, struct rq_flags *rf,
			int *pulled_task, int *done,
			int order_idx, int walk_cnt)
{
	struct ux_sched_cputopo ux_cputopo = ux_sched_cputopo;
	cpumask_t search_cpus = CPU_MASK_NONE;
	struct rq *rq;
	struct rq *busiest_rq = NULL;
	struct oplus_rq *orq = NULL;
	struct task_struct *ux_task = NULL;
	int this_cpu = cpu_of(this_rq);
	int cpu = -1;
	int busiest_cpu = -1;
	int idx;

#ifdef DEBUG_LB_NEWIDLE_HIT
	trace_printk("DEBUG_LB_NEWIDLE_HIT[%d]: cpu=%d, "
		"order_idx=%d, walk_cnt=%d\n",
		__LINE__, this_cpu, order_idx, walk_cnt);
#endif

	/*
	 * Find a cpu on which there is a task in rt_boost group that has been
	 * in the runnable state for a long time.
	 */
	for (idx = 0; idx < walk_cnt; idx++) {
		cpumask_copy(&search_cpus, &ux_cputopo.oplus_cpu_array[order_idx][idx]);

		for_each_cpu(cpu, &search_cpus) {
			rq = cpu_rq(cpu);
			orq = (struct oplus_rq *) rq->android_oem_data1;

			/*
			 * Cannot migrate to itself.
			 */
			if (cpu == this_cpu)
				continue;

			/*
			 * Only allow to pull tasks from the active cpu.
			 */
			if (!cpu_online(cpu) || !cpu_active(cpu))
				continue;

#ifdef CONFIG_OPLUS_ADD_CORE_CTRL_MASK
			if (oplus_cpu_halted(cpu))
				continue;
#endif

			busiest_cpu = cpu;
			busiest_rq = cpu_rq(busiest_cpu);
			double_lock_balance(this_rq, busiest_rq);

			/*
			 * Skip if there is no ux task on this cpu.
			 */
			if (!orq_has_ux_tasks(orq)) {
				double_unlock_balance(this_rq, busiest_rq);
				continue;
			}

			/*
			 * In this step, we pay more attention to the rapid response
			 * capability of the ux task.
			 */
			ux_task = oplus_pick_runnable_ux(cpu, this_cpu, NULL);
			if (!ux_task) {
				double_unlock_balance(this_rq, busiest_rq);
				continue;
			}

			/*
			 * Ha, ux_task can be migrated to this_cpu to perform enqueue
			 * and dequeue operations.
			 */
			deactivate_task(busiest_rq, ux_task, 0);
			set_task_cpu(ux_task, this_cpu);
			activate_task(this_rq, ux_task, 0);

			/*
			 * Only pull one task at a time.
			 */
			*pulled_task = 1;
			*done = 1;

			/*
			 * reset the idle time stamp if we pulled any task.
			 */
			this_rq->idle_stamp = 0;

			double_unlock_balance(this_rq, busiest_rq);

			oplus_loadbalance_systrace_print(OPLUS_LB_SYSTRACE_PID,
					"newidle_migr_ux_cpu", busiest_cpu, ux_task->pid);

#ifdef DEBUG_LB_NEWIDLE
			trace_printk("DEBUG_LB_NEWIDLE[%d]: "
				"busiest_cpu=%d, this_cpu=%d, task=%s$%d$%d\n",
				__LINE__, busiest_cpu, this_cpu,
				ux_task->comm, ux_task->pid, ux_task->prio);
#endif
			return true;
		}
	}
	return false;
}

static bool oplus_newidle_balance_pull_task(struct rq *this_rq,
					struct rq_flags *rf, int *pulled_task, int *done)
{
	int this_cpu = cpu_of(this_rq);
	int cur_cls = topology_physical_package_id(this_cpu);
	int order_idx = -1, walk_cnt = -1;
	bool ret = false;

	/*
	 * Calculate order_idx and walk_cnt.
	 */
	if (!calc_order_idx(newidle_migr, cur_cls, &order_idx, &walk_cnt))
		return false;

	atomic64_inc(&lb_stat.newidle_hit);

	/*
	 * Step1: migrate rt_boost first.
	 * In this step, we pay more attention to the rapid response
	 * capability of the rt_boost task.
	 */
	if (oplus_newidle_balance_pull_runnable_rt_boost(this_rq, rf,
				pulled_task, done, order_idx, walk_cnt)) {
		atomic64_inc(&lb_stat.newidle_runnable_rt_boost_succ);
		ret = true;
		goto out;
	}

	/*
	 * Step2: then runnable ux task.
	 * In this step, we pay more attention to the rapid response
	 * capability of the ux task.
	 */
	if (oplus_newidle_balance_pull_runnable_ux(this_rq, rf,
				pulled_task, done, order_idx, walk_cnt)) {
		atomic64_inc(&lb_stat.newidle_runnable_ux_succ);
		ret = true;
		goto out;
	}

	/*
	 * Step3: last is normal_rt
	 * In this step we focus on the throughput of the normal_rt task.
	 */
	if (oplus_newidle_balance_pull_runnable_rt_normal(this_rq, rf,
				pulled_task, done, order_idx, walk_cnt)) {
		atomic64_inc(&lb_stat.newidle_runnable_normal_rt_succ);
		ret = true;
		goto out;
	}

	atomic64_inc(&lb_stat.newidle_fail);

out:
	/*
	 * WARNING:
	 * Although we may failed to pull tasks from other CPUs through
	 * newidle balance. But there is a possibility that other cpus
	 * have successfully enqueue a cfs task to the runqueue of
	 * this_cpu through the active balance logic. In this case,
	 * the newidle balance is considered successful, and the variable
	 * 'done' needs to be set to true. Because only in this way, the
	 * logic of 'goto again' in pick_next_task_fair will be executed,
	 * and a task will be re-selected from the runqueue of this_cpu
	 * as next_task. Otherwise, the newly migrated task will be in
	 * the runnable state and will not get a chance to run for a
	 * long time.
	 */
	if (this_rq->cfs.h_nr_running && !*pulled_task)
		*pulled_task = 1;

	/*
	 * Is there a task of a high priority class?
	 */
	if (this_rq->nr_running != this_rq->cfs.h_nr_running)
		*pulled_task = -1;

	if (*pulled_task)
		this_rq->idle_stamp = 0;

	if (*pulled_task != 0)
		*done = 1;

	/*
	 * WARNING:
	 * Skip the newidle logic of QCOM or MTK and pick the higher
	 * priority task directly when there is a higher priority task.
	 *
	 * For example:
	 * the following logic exists in Qualcomm's walt_newidle_balance
	 * function (implemented in kernel\sched\walt\walt_lb.c).
	 * The pulled_task variable will be cleared here. If this_cpu is
	 * inactive at this time, the walt_newidle_balance function will
	 * return directly, which results in The RETRY_TASK logic will
	 * not be executed in pick_next_task_fair. This will causes high
	 * priority tasks to fail to run.
	 *
	 *  * newly idle load balance is completely handled here, so
	 *  * set done to skip the load balance by the caller.
	 *
	 * *done = 1;
	 * *pulled_task = 0;
	 *
	 *
	 *  * This CPU is about to enter idle, so clear the
	 *  * misfit_task_load and mark the idle stamp.
	 *
	 * this_rq->misfit_task_load = 0;
	 * this_rq->idle_stamp = rq_clock(this_rq);
	 *
	 * if (!cpu_active(this_cpu))
	 * 	 return;
	 *
	 * Note:
	 * The execution logic in the pause_cpus function (implemented in
	 * \kernel\cpu.c), will first mark the cpu as inactive, and then
	 * wake up the migration thread. This will also cause the active
	 * status judgment result of this_cpu in walt_newidle_balance to
	 * be false. However, there may be a high-priority migration task
	 * in the runqueue of this_cpu.
	 *
	 * 	for_each_cpu(cpu, cpus)
	 * 	set_cpu_active(cpu, false);
	 * 	err = __pause_drain_rq(cpus);
	 */
	if (*done != 0)
		ret = true;

#ifdef DEBUG_LB_NEWIDLE_HIT
	trace_printk("DEBUG_LB_NEWIDLE_HIT[%d]: cpu=%d, pulled_task=%d, "
		"done=%d, nr_running=%d, h_nr_running=%d\n",
		__LINE__, this_cpu, *pulled_task, *done,
		this_rq->nr_running, this_rq->cfs.h_nr_running);
#endif

	return ret;
}

static int oplus_get_cluster_util(int cpu)
{
	struct ux_sched_cputopo ux_cputopo = ux_sched_cputopo;
	cpumask_t search_cpus = CPU_MASK_NONE;

	/* Only the current cluster is counted. */
	int order_idx = topology_physical_package_id(cpu);
	unsigned long total_capacity = 0, total_util = 0;
	unsigned long util = PERCENTAGE;

	cpumask_copy(&search_cpus, &ux_cputopo.oplus_cpu_array[order_idx][0]);
	for_each_cpu(cpu, &search_cpus) {
		/* active cpu. */
		if (!cpu_online(cpu) || !cpu_active(cpu))
			continue;

#ifdef CONFIG_OPLUS_ADD_CORE_CTRL_MASK
		if (oplus_cpu_halted(cpu))
			continue;
#endif
		total_util += cpu_util(cpu);
		total_capacity += capacity_orig_of(cpu);
	}

	/* in percent format. */
	util = (total_capacity == 0) ? PERCENTAGE :
			total_util * PERCENTAGE / total_capacity;
	util = min_t(unsigned int, util, PERCENTAGE);

	return util;
}

/* in percent format. */
__maybe_unused static int oplus_get_cpu_util(int cpu)
{
	return cpu_util(cpu) * PERCENTAGE / 1024;
}

bool has_enough_capacity(int this_cpu)
{
#ifdef DEBUG_LB_NEWIDLE_HIT
	int i;
#endif

	/*
	 * Only consider the case where this_cpu belongs to mid_cluster.
	 */
	if (!is_mid_cluster(this_cpu))
		return false;

#ifdef DEBUG_LB_NEWIDLE_HIT
	for (i=0; i < OPLUS_NR_CPUS; i++) {
		oplus_loadbalance_systrace_print(OPLUS_LB_SYSTRACE_PID,
						"newidle_cpu_util",
						i, oplus_get_cpu_util(i));
	}

	oplus_loadbalance_systrace_print(OPLUS_LB_SYSTRACE_PID,
					"newidle_cls_util",
					topology_physical_package_id(MIN_CPU),
					oplus_get_cluster_util(MIN_CPU));
	oplus_loadbalance_systrace_print(OPLUS_LB_SYSTRACE_PID,
					"newidle_cls_util",
					topology_physical_package_id(this_cpu),
					oplus_get_cluster_util(this_cpu));
	oplus_loadbalance_systrace_print(OPLUS_LB_SYSTRACE_PID,
					"newidle_cls_util",
					topology_physical_package_id(MAX_CPU),
					oplus_get_cluster_util(MAX_CPU));
#endif

	/*
	 * When the following two conditions are met at the same time,
	 * it means that there is still some spare capacity:
	 * a) The util value of mid_cluster is less than 80%;
	 * b) The util value of min_cluster or max_cluster is greater
	 *    than 95%;
	 */
	if (oplus_get_cluster_util(this_cpu) > NEWIDLE_BALANCE_CLS_UTIL_MID_THRESHOLD)
		return false;

	if (oplus_get_cluster_util(MIN_CPU) > NEWIDLE_BALANCE_CLS_UTIL_MINMAX_THRESHOLD ||
		oplus_get_cluster_util(MAX_CPU) > NEWIDLE_BALANCE_CLS_UTIL_MINMAX_THRESHOLD)
		return true;

	return false;
}

bool __oplus_newidle_balance(void *data, struct rq *this_rq,
					struct rq_flags *rf, int *pulled_task, int *done)
{
	int this_cpu = cpu_of(this_rq);
	struct root_domain *rd = cpu_rq(this_cpu)->rd;
	bool enough_idle, enough_capacity;
	bool ret = false;
#ifdef DEBUG_LB_NEWIDLE_HIT
	bool is_migration;
#endif

	if (unlikely(!lb_enable))
		return false;

	/*
	 * Do not pull tasks towards unavailable cpus.
	 */
	if (!cpu_online(this_cpu) || !cpu_active(this_cpu))
		return false;

#ifdef CONFIG_OPLUS_ADD_CORE_CTRL_MASK
	if (oplus_cpu_halted(this_cpu))
		return false;
#endif

	/*
	 * If there are tasks to be run on this cpu, there
	 * is no need to perform a balance operation.
	 * nr_running is the sum of the number of cfs, rt,
	 * dl tasks.
	 */
	if (this_rq->nr_running)
		return false;

	/*
	 * no tasks to be pulled.
	 */
	if (!READ_ONCE(rd->overload))
		return false;

	enough_idle = (this_rq->avg_idle > NEWIDLE_BALANCE_IDLE_THRESHOLD);
	enough_capacity = has_enough_capacity(this_cpu);

#ifdef DEBUG_LB_TRACKME
	show_trackme_stats();
#endif

#ifdef DEBUG_LB_NEWIDLE_HIT
	is_migration = !strncmp(this_rq->curr->comm, "migration/", 10);

	trace_printk("DEBUG_LB_NEWIDLE_HIT[%d]: cpu=%d, current=%s$%d, "
			"nr_running=%d, h_nr_running=%d, high_prio=%d "
			"is_migration=%d\n",
		__LINE__, this_cpu, this_rq->curr->comm, this_rq->curr->pid,
		this_rq->nr_running, this_rq->cfs.h_nr_running,
		this_rq->nr_running != this_rq->cfs.h_nr_running,
		is_migration);

	oplus_loadbalance_systrace_print(OPLUS_LB_SYSTRACE_PID,
					"newidle_enough_idle",
					this_cpu, enough_idle);
	oplus_loadbalance_systrace_print(OPLUS_LB_SYSTRACE_PID,
					"newidle_enough_capacity",
					this_cpu, enough_capacity);
#endif

	/*
	 * Trigger newidle balance when any of the following conditions are met:
	 * a) The cpu has been sleeping for a long time;
	 * b) Although the average sleep time is not enough, there is still
	 *    remaining capacity in the cluster where the cpu is located.
	 */
	if (!enough_idle && !enough_capacity)
		return false;

#ifdef DEBUG_LB_NEWIDLE_HIT
	/*
	 * Output a random number less than 10.
	 * Data changes indicate that newidle balance has been triggered.
	 */
	oplus_loadbalance_systrace_print(OPLUS_LB_SYSTRACE_PID,
				"newidle_balance_hit_cpu", this_cpu, jiffies%10);
#endif

	/*
	 * We must set idle_stamp _before_ calling idle_balance(), such that we
	 * measure the duration of idle_balance() as idle time.
	 */
	this_rq->idle_stamp = rq_clock(this_rq);

	/*
	 * See commit 46f69fa33712a for more information.
	 */
	rq_unpin_lock(this_rq, rf);

	/*
	 * Pull tasks from other CPUs.
	 */
	ret = oplus_newidle_balance_pull_task(this_rq, rf, pulled_task, done);

	rq_repin_lock(this_rq, rf);

#ifdef DEBUG_LB_NEWIDLE_HIT
	trace_printk("DEBUG_LB_NEWIDLE_HIT[%d]: cpu=%d, ret=%d, "
		"pulled_task=%d, done=%d\n",
		__LINE__, this_cpu, ret,
		*pulled_task, *done);
#endif

	return ret;
}
EXPORT_SYMBOL(__oplus_newidle_balance);

void oplus_newidle_balance(void *data, struct rq *this_rq,
					struct rq_flags *rf, int *pulled_task, int *done)
{
	__oplus_newidle_balance(data, this_rq, rf, pulled_task, done);
}

/**** Interface exported to the proc file system. ****/

#ifdef DEBUG_LB_TRACKME
void show_trackme_stats(void)
{
	struct task_struct *tsk;
	struct task_struct *curr;
	struct rq *rq;
	u64 running_time, runnable_time;
	int i;
	int pid;
	int cpu;
	int preempt;

	/*
	 * Ignore preempt_disable scenarios.
	 */
	int is_idle;

	if (!tk_num)
		return;

	for (i = 0; i < tk_num; i++) {
		pid = tk_array[i].pid;
		tsk = tk_array[i].tsk;
		if (!tsk)
			continue;

		get_task_struct(tsk);

		cpu = task_cpu(tsk);
		rq = cpu_rq(cpu);
		curr = rq->curr;
		preempt = task_thread_info(curr)->preempt.count;
		is_idle = curr == rq->idle;

		runnable_time = get_runnable_time(tsk);
		running_time = get_running_time(tsk);

		if (is_idle) {
			tk_array[i].max_runnable_time = max_t(u64,
					tk_array[i].max_runnable_time, runnable_time);
			tk_array[i].max_running_time = max_t(u64,
					tk_array[i].max_running_time, running_time);
		}

		trace_printk("TRACK[%d]: task=%s$%d, state=%d, on_cpu=%d, on_rq=%d, "
			"running=%llu, runnable=%llu, thres=%d-%d, "
			"max_running_time=%llu, max_runnable_time=%llu, "
			"curr=%s$%d, is_idle=%d, preempt=%d, tif=%d, "
			"nr_running=%d, h_nr_running=%d\n",
			i, tsk->comm, tsk->pid,
			tsk->__state, tsk->on_cpu, tsk->on_rq,
			running_time, runnable_time,
			running_time >= TRACKME_RUNNING_THRES,
			runnable_time >= TRACKME_RUNNABLE_THRES,
			tk_array[i].max_running_time,
			tk_array[i].max_runnable_time,
			curr->comm, curr->pid, is_idle, preempt,
			test_ti_thread_flag(task_thread_info(curr), TIF_NEED_RESCHED),
			rq->nr_running, rq->cfs.h_nr_running);

		/*
		 * No log if the task is in the running/runnable state
		 * for a long time due to preempt_diasble.
		 */
		if (running_time >= TRACKME_RUNNING_THRES && is_idle) {
			oplus_loadbalance_systrace_print(OPLUS_LB_SYSTRACE_PID,
					"trackme_running_pid", tsk->pid, running_time);
		}

		if (runnable_time >= TRACKME_RUNNABLE_THRES && is_idle) {
			oplus_loadbalance_systrace_print(OPLUS_LB_SYSTRACE_PID,
					"trackme_runnable_pid", tsk->pid, runnable_time);
		}

		put_task_struct(tsk);
	}
}

static int proc_trackme_read(struct seq_file *m, void *v)
{
	struct task_struct *tsk;
	int pid;
	int i;

	if (!tk_num) {
		seq_printf(m, "TRACK: No task being tracked yet!\n");
		return 0;
	}

	for (i = 0; i < tk_num; i++) {
		pid = tk_array[i].pid;
		tsk = tk_array[i].tsk;

		seq_printf(m, "TRACK[%d]: pid=%d, comm=%s\n",
			i, pid, tsk->comm);
	}

	return 0;
}

static int proc_trackme_open(struct inode *inode,
			struct file *file)
{
	return single_open(file, proc_trackme_read, inode);
}

static ssize_t proc_trackme_write(struct file *file,
					const char __user *buf, size_t count, loff_t *offset)
{
	struct task_struct *tsk;
	char buffer[256];
	char *token, *p = buffer;
	int pid;
	int i = 0;

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	tk_num = 0;
	memset(tk_array, 0, sizeof(struct trackme));
	while ((token = strsep(&p, " ")) != NULL) {
		if (tk_num >= MAX_TRACKME_NUM)
			break;

		if (kstrtoint(strstrip(token), 10, &tk_array[tk_num].pid))
			return -EINVAL;

		tk_num++;
	}

	for (i = 0; i < tk_num; i++) {
		pid = tk_array[i].pid;
		tsk = find_task_by_vpid(pid);
		if (!tsk)
			continue;

		get_task_struct(tsk);
		tk_array[i].tsk = tsk;
		trace_printk("DEBUG_LB_TRACKME[%d]: TRACK[%d]: pid=%d, comm=%s, "
				"state=%d, on_cpu=%d, on_rq=%d\n",
			__LINE__, i, pid, tsk->comm,
			tsk->__state, tsk->on_cpu, tsk->on_rq);
		put_task_struct(tsk);
	}

	return count;
}

const struct proc_ops proc_trackme_operations = {
	.proc_open = proc_trackme_open,
	.proc_read = seq_read,
	.proc_write = proc_trackme_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

struct proc_dir_entry *oplus_trackme_proc_init(
			struct proc_dir_entry *pde)
{
	return proc_create("trackme", S_IRUGO | S_IWUGO, pde, &proc_trackme_operations);
}

void oplus_trackme_proc_deinit(struct proc_dir_entry *pde)
{
	remove_proc_entry("trackme", pde);
}
#endif

int dump_rt_boost_task(struct seq_file *m, void *v)
{
	struct oplus_task_struct *ots;
	struct task_struct *p;
	struct plist_head *plist = &rt_boost_task;
	unsigned long irqflag;
	int idx = 0;

	spin_lock_irqsave(&rbt_lock, irqflag);
	if (!has_rt_boost_tasks()) {
		spin_unlock_irqrestore(&rbt_lock, irqflag);
		return 0;
	}

	plist_for_each_entry(ots, plist, rtb) {
		/*
		 * NOTE:
		 * The task may have been destroyed during the shutdown
		 * process, so it is necessary to determine whether the
		 * task is NULL.
		 */
		p = ots_to_ts(ots);
		if (IS_ERR_OR_NULL(p))
			continue;

		seq_printf(m, "RT_BOOST: idx=%d, prio=%d, im_flag=%d, task=%s$%d$%d",
			++idx, im_flag_to_prio(ots->im_flag), ots->im_flag,
			p->comm, p->pid, p->prio);

#if defined(DEBUG_LB_TICK) || \
		defined(DEBUG_LB_TICK_RUNNABLE_TIME) || \
		defined(DEBUG_LB_NEWIDLE) || \
		defined(DEBUG_LB_NEWIDLE_HIT) || \
		defined(DEBUG_LB_RT_TICK) || \
		defined(DEBUG_LB_RT_RUNNABLE_TIME)

		seq_printf(m, ", uage=%d, state=%d, task_cpu=%d",
			refcount_read(&p->usage), READ_ONCE(p->__state), task_cpu(p));
#endif
		seq_printf(m, "\n");
	}
	spin_unlock_irqrestore(&rbt_lock, irqflag);
	return 0;
}

static int proc_rt_boost_read(struct seq_file *m, void *v)
{
	return dump_rt_boost_task(m, v);
}

static int proc_rt_boost_open(struct inode *inode,
			struct file *file)
{
	return single_open(file, proc_rt_boost_read, inode);
}

/*
 * Execute the following command to add/remove a task to the rt_boost group.
 * Remove this rt task from the rt_boost group if the specified im_flag is
 * not of the type we care about.
 *
 * command:
 * echo im_flag task_pid > /proc/oplus_scheduler/sched_assist/rt_boost
 *
 * eg:
 * add    : echo 1 2345 > /proc/oplus_scheduler/sched_assist/rt_boost
 * remove : echo 0 2345 > /proc/oplus_scheduler/sched_assist/rt_boost
 */
#define PARACNT			2
static ssize_t proc_rt_boost_write(struct file *file,
					const char __user *buf, size_t count, loff_t *offset)
{
	char buffer[256];
	char *token, *p = buffer;
	struct task_struct *task;
	struct oplus_task_struct *ots;
	int im_flag, pid;
	int para[PARACNT];
	int cnt = 0;

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

#ifdef DEBUG_LB_RT_TICK
	trace_printk("DEBUG_LB_RT_TICK[%d]: buffer=%s\n",
			__LINE__, buffer);
#endif

	while ((token = strsep(&p, " ")) != NULL) {
		if (cnt >= PARACNT)
			break;

		if (kstrtoint(strstrip(token), 10, &para[cnt]))
			return -EINVAL;

		cnt++;
	}
	im_flag = para[0];
	pid = abs(para[1]);

#ifdef DEBUG_LB_RT_TICK
	trace_printk("DEBUG_LB_RT_TICK[%d]: prio=%d, pid=%d\n",
			__LINE__, im_flag, pid);
#endif

	rcu_read_lock();
	task = find_task_by_vpid(pid);
	if (task) {
		get_task_struct(task);

		ots = get_oplus_task_struct(task);
		if (IS_ERR_OR_NULL(ots)) {
			put_task_struct(task);
			rcu_read_unlock();
			return -EFAULT;
		}

		/*
		 * Note:
		 * The following operations are order sensitive.
		 */
		if (im_flag_to_prio(im_flag) < 0) {
			remove_rt_boost_task(task);
			ots->im_flag = im_flag;
		} else {
			ots->im_flag = im_flag;
			add_rt_boost_task(task);
		}

		put_task_struct(task);
	}
	rcu_read_unlock();

	return count;
}

const struct proc_ops proc_rt_boost_operations = {
	.proc_open = proc_rt_boost_open,
	.proc_read = seq_read,
	.proc_write = proc_rt_boost_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

struct proc_dir_entry *oplus_rt_boost_proc_init(
			struct proc_dir_entry *pde)
{
	return proc_create("rt_boost", S_IRUGO | S_IWUGO, pde, &proc_rt_boost_operations);
}

void oplus_rt_boost_proc_deinit(struct proc_dir_entry *pde)
{
	remove_proc_entry("rt_boost", pde);
}

static int proc_lb_enable_read(struct seq_file *m, void *v)
{
	seq_printf(m, "lb_enable: %d\n", lb_enable);
	return 0;
}

static int proc_lb_enable_open(struct inode *inode,
			struct file *file)
{
	return single_open(file, proc_lb_enable_read, inode);
}

static ssize_t proc_lb_enable_write(struct file *file,
					const char __user *buf, size_t count, loff_t *offset)
{
	char buffer[256];
	char *token, *p = buffer;
	int para[PARACNT];
	int cnt = 0;

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	while ((token = strsep(&p, " ")) != NULL) {
		if (cnt >= PARACNT)
			break;

		if (kstrtoint(strstrip(token), 10, &para[cnt]))
			return -EINVAL;

		cnt++;
	}
	lb_enable = !!para[0];

	return count;
}

const struct proc_ops proc_lb_enable_operations = {
	.proc_open = proc_lb_enable_open,
	.proc_read = seq_read,
	.proc_write = proc_lb_enable_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

struct proc_dir_entry *oplus_lb_enable_proc_init(
			struct proc_dir_entry *pde)
{
	return proc_create("lb_enable", S_IRUGO | S_IWUGO, pde, &proc_lb_enable_operations);
}

void oplus_lb_enable_proc_deinit(struct proc_dir_entry *pde)
{
	remove_proc_entry("lb_enable", pde);
}

static int proc_lb_debug_read(struct seq_file *m, void *v)
{
	seq_printf(m, "lb_debug: %d\n", lb_debug);
	return 0;
}

static int proc_lb_debug_open(struct inode *inode,
			struct file *file)
{
	return single_open(file, proc_lb_debug_read, inode);
}

static ssize_t proc_lb_debug_write(struct file *file,
					const char __user *buf, size_t count, loff_t *offset)
{
	char buffer[256];
	char *token, *p = buffer;
	int para[PARACNT];
	int cnt = 0;

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	while ((token = strsep(&p, " ")) != NULL) {
		if (cnt >= PARACNT)
			break;

		if (kstrtoint(strstrip(token), 10, &para[cnt]))
			return -EINVAL;

		cnt++;
	}
	lb_debug = !!para[0];

	return count;
}

const struct proc_ops proc_lb_debug_operations = {
	.proc_open = proc_lb_debug_open,
	.proc_read = seq_read,
	.proc_write = proc_lb_debug_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

struct proc_dir_entry *oplus_lb_debug_proc_init(
			struct proc_dir_entry *pde)
{
	return proc_create("lb_debug", S_IRUGO | S_IWUGO, pde, &proc_lb_debug_operations);
}

void oplus_lb_debug_proc_deinit(struct proc_dir_entry *pde)
{
	remove_proc_entry("lb_debug", pde);
}

static int proc_lb_stat_read(struct seq_file *m, void *v)
{
	seq_printf(m, "tick_hit:                   %10llu\n",
		atomic64_read(&lb_stat.tick_hit));
	seq_printf(m, "tick_running_rt_boost:      %10llu\n",
		atomic64_read(&lb_stat.tick_running_rt_boost_succ));
	seq_printf(m, "tick_runnable_rt_boost:     %10llu\n",
		atomic64_read(&lb_stat.tick_runnable_rt_boost_succ));
	seq_printf(m, "tick_running_ux:            %10llu\n",
		atomic64_read(&lb_stat.tick_running_ux_succ));
	seq_printf(m, "tick_runnable_ux:           %10llu\n",
		atomic64_read(&lb_stat.tick_runnable_ux_succ));
	seq_printf(m, "tick_running_normal_rt:     %10llu\n",
		atomic64_read(&lb_stat.tick_running_normal_rt_succ));
	seq_printf(m, "tick_fail:                  %10llu\n",
		atomic64_read(&lb_stat.tick_fail));
	seq_printf(m, "\n");

	seq_printf(m, "newidle_hit:                %10llu\n",
		atomic64_read(&lb_stat.newidle_hit));
	seq_printf(m, "newidle_runnable_rt_boost:  %10llu\n",
		atomic64_read(&lb_stat.newidle_runnable_rt_boost_succ));
	seq_printf(m, "newidle_runnable_ux:        %10llu\n",
		atomic64_read(&lb_stat.newidle_runnable_ux_succ));
	seq_printf(m, "newidle_runnable_normal_rt: %10llu\n",
		atomic64_read(&lb_stat.newidle_runnable_normal_rt_succ));
	seq_printf(m, "newidle_fail:               %10llu\n",
		atomic64_read(&lb_stat.newidle_fail));
	seq_printf(m, "\n");

	return 0;
}

static int proc_lb_stat_open(struct inode *inode,
			struct file *file)
{
	return single_open(file, proc_lb_stat_read, inode);
}

static const struct proc_ops proc_lb_stat_operations = {
	.proc_open = proc_lb_stat_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

struct proc_dir_entry *oplus_lb_stat_proc_init(
			struct proc_dir_entry *pde)
{
	return proc_create("lb_stat", S_IRUGO, pde, &proc_lb_stat_operations);
}

void oplus_lb_stat_proc_deinit(struct proc_dir_entry *pde)
{
	remove_proc_entry("lb_stat", pde);
}

/**** Interface related to loadbalance initialization. ****/

void oplus_lb_proc_init(struct proc_dir_entry *pde)
{
	oplus_rt_boost_proc_init(pde);
	oplus_lb_stat_proc_init(pde);
	oplus_lb_enable_proc_init(pde);
	oplus_lb_debug_proc_init(pde);
#ifdef DEBUG_LB_TRACKME
	oplus_trackme_proc_init(pde);
#endif
}

void oplus_lb_proc_deinit(struct proc_dir_entry *pde)
{
	oplus_rt_boost_proc_deinit(pde);
	oplus_lb_stat_proc_deinit(pde);
	oplus_lb_enable_proc_deinit(pde);
	oplus_lb_debug_proc_deinit(pde);
#ifdef DEBUG_LB_TRACKME
	oplus_trackme_proc_deinit(pde);
#endif
}

void oplus_loadbalance_init(void)
{
	plist_head_init(&rt_boost_task);

	/*
	 * NOTE:
	 * In order to prevent conflicts, we directly insert the following
	 * hook operation into Qualcomm's walt_lb_tick function.
	 *
	 *register_trace_android_vh_scheduler_tick(oplus_tick_balance, NULL);
	 */

#if !IS_ENABLED(CONFIG_OPLUS_SYSTEM_KERNEL_QCOM) && !IS_ENABLED(CONFIG_MTK_NEWIDLE_BALANCE)
	/*
	 * for newidle balance.
	 * rvh hook cannot be uninstalled.
	 */
	register_trace_android_rvh_sched_newidle_balance(oplus_newidle_balance, NULL);
#endif
}

void oplus_loadbalance_deinit(void)
{
	release_rt_boost_task();

	/*
	 * NOTE:
	 * In order to prevent conflicts, we directly insert the following
	 * hook operation into Qualcomm's walt_lb_tick function.
	 *
	 * unregister_trace_android_vh_scheduler_tick(oplus_tick_balance, NULL);
	 */
}

