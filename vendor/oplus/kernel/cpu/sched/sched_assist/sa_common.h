/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */


#ifndef _OPLUS_SA_COMMON_H_
#define _OPLUS_SA_COMMON_H_

#include <linux/sched.h>
#include <linux/list.h>
#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/hashtable.h>
#include <linux/cgroup-defs.h>
#if IS_ENABLED(CONFIG_SCHED_WALT)
#include <linux/sched/walt.h>
#endif
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include "sa_oemdata.h"


#define ux_err(fmt, ...) \
		pr_err("[sched_assist][%s]"fmt, __func__, ##__VA_ARGS__)
#define ux_warn(fmt, ...) \
		pr_warn("[sched_assist][%s]"fmt, __func__, ##__VA_ARGS__)
#define ux_debug(fmt, ...) \
		pr_info("[sched_assist][%s]"fmt, __func__, ##__VA_ARGS__)

#define UX_MSG_LEN		64
#define UX_DEPTH_MAX		5

/* define for debug */
#define DEBUG_SYSTRACE (1 << 0)
#define DEBUG_FTRACE   (1 << 1)
#define DEBUG_FBG	(1 << 2)

/* define for sched assist feature */
#define FEATURE_COMMON (1 << 0)
#define FEATURE_SPREAD (1 << 1)

#define UX_EXEC_SLICE (4000000U)

/* define for sched assist thread type, keep same as the define in java file */
#define SA_OPT_CLEAR				(0)
#define SA_TYPE_LIGHT				(1 << 0)
#define SA_TYPE_HEAVY				(1 << 1)
#define SA_TYPE_ANIMATOR			(1 << 2)
#define SA_TYPE_LISTPICK			(1 << 3)
#define SA_TYPE_ONCE				(1 << 4) /* clear ux type when dequeue */
#define SA_OPT_SET					(1 << 7)
#define SA_TYPE_INHERIT				(1 << 8)

#define SCHED_ASSIST_UX_MASK		(0xFF)

/* define for sched assist scene type, keep same as the define in java file */
#define SA_SCENE_OPT_CLEAR			(0)
#define SA_LAUNCH					(1 << 0)
#define SA_SLIDE					(1 << 1)
#define SA_CAMERA					(1 << 2)
#define SA_ANIM_START				(1 << 3) /* we care about both launcher and top app */
#define SA_ANIM						(1 << 4) /* we only care about launcher */
#define SA_INPUT					(1 << 5)
#define SA_LAUNCHER_SI				(1 << 6)
#define SA_SCENE_OPT_SET			(1 << 7)

#define  FIRST_APPLICATION_UID  10000
#define  LAST_APPLICATION_UID   19999

extern pid_t save_audio_tgid;
extern pid_t save_top_app_tgid;
extern unsigned int top_app_type;

/* define for boost threshold unit */
#define BOOST_THRESHOLD_UNIT (51)

enum UX_STATE_TYPE {
	UX_STATE_INVALID = 0,
	UX_STATE_NONE,
	UX_STATE_SCHED_ASSIST,
	UX_STATE_INHERIT,
	MAX_UX_STATE_TYPE,
};

enum INHERIT_UX_TYPE {
	INHERIT_UX_BINDER = 0,
	INHERIT_UX_RWSEM,
	INHERIT_UX_MUTEX,
	INHERIT_UX_FUTEX,
	INHERIT_UX_MAX,
};

/* WANNING: new flag should be add before MAX_IM_FLAG_TYPE, never change the value of those existed flag type. */
enum IM_FLAG_TYPE {
	INVALID_IM_FLAG = -1,
	IM_FLAG_NONE = 0,
	IM_FLAG_SURFACEFLINGER,
	IM_FLAG_HWC,			/* Discarded */
	IM_FLAG_RENDERENGINE,
	IM_FLAG_WEBVIEW,
	IM_FLAG_CAMERA_HAL,
	IM_FLAG_3RD_AUDIO,
	IM_FLAG_HWBINDER,
	IM_FLAG_LAUNCHER,
	IM_FLAG_LAUNCHER_NON_UX_RENDER,
	IM_FLAG_SS_LOCK_OWNER,
	IM_FLAG_FORBID_SET_CPU_AFFINITY, /* forbid setting cpu affinity from app */
	MAX_IM_FLAG_TYPE,
};

DECLARE_PER_CPU(struct list_head, ux_thread_list);

struct ux_sched_cluster {
	struct cpumask cpus;
	unsigned long capacity;
};

#define OPLUS_NR_CPUS (8)
struct ux_sched_cputopo {
	int cls_nr;
	struct ux_sched_cluster sched_cls[OPLUS_NR_CPUS];
};

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CPU_JANKINFO)
/* hot-thread */
struct task_record {
#define RECOED_WINSIZE			(1 << 8)
#define RECOED_WINIDX_MASK		(RECOED_WINSIZE - 1)
	u8 winidx;
	u8 count;
};
#endif

#if IS_ENABLED(CONFIG_OPLUS_LOCKING_STRATEGY)
struct locking_info {
	u64 waittime_stamp;
	u64 holdtime_stamp;
	struct task_struct *holder;
	u32 waittype;
	bool ux_contrib;
};
#endif

/* Please add your own members of task_struct here :) */
struct oplus_task_struct {
	/* CONFIG_OPLUS_FEATURE_SCHED_ASSIST */
	struct list_head ux_entry;

	struct task_struct *task;
	atomic64_t inherit_ux;
	u64 enqueue_time;
	u64 inherit_ux_start;
	u64 sum_exec_baseline;
	u64 total_exec;
	int ux_state;
	int ux_depth;
	int im_flag;
	int tpd; /* task placement decision */
	/* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */
	int lb_state;
	int ld_flag;
	/* CONFIG_OPLUS_FEATURE_TASK_LOAD */
	int is_update_runtime;
	int target_process;
	u64 wake_tid;
	u64 running_start_time;
	bool update_running_start_time;
	u64 exec_calc_runtime;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CPU_JANKINFO)
	struct task_record record[OPLUS_NR_CPUS];	/* 2*u64 */
#endif
	/* CONFIG_OPLUS_FEATURE_FRAME_BOOST */
	struct list_head fbg_list;
	unsigned int fbg_state;
	int fbg_depth;
	bool fbg_running; /* task belongs to a group, and in running */
	int preferred_cluster_id;
	u64 last_wake_ts;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FDLEAK_CHECK)
	unsigned int fdleak_flag;
#endif
#if IS_ENABLED(CONFIG_OPLUS_LOCKING_STRATEGY)
	struct locking_info lkinfo;
#endif
} ____cacheline_aligned;

struct oplus_rq {
	/* CONFIG_OPLUS_FEATURE_SCHED_ASSIST */
	struct list_head ux_list;
};

extern int global_debug_enabled;
extern int global_sched_assist_enabled;
extern int global_sched_assist_scene;

struct rq;

/* attention: before insert .ko, task's list->prev/next will be init with 0 */
static inline bool oplus_list_empty(struct list_head *list)
{
	return list_empty(list) || (list->prev == 0 && list->next == 0);
}

/**
 * Check if there are ux tasks waiting to run on the specified cpu
 */
static inline bool orq_has_ux_tasks(struct oplus_rq *orq)
{
	bool ret = false;

	if (!orq)
		return false;

	ret = !oplus_list_empty(&orq->ux_list);
	return ret;
}

typedef bool (*migrate_task_callback_t)(struct task_struct *tsk, int src_cpu, int dst_cpu);
extern migrate_task_callback_t fbg_migrate_task_callback;
typedef void (*android_rvh_schedule_handler_t)(struct task_struct *prev,
	struct task_struct *next, struct rq *rq);
extern android_rvh_schedule_handler_t fbg_android_rvh_schedule_callback;

extern struct kmem_cache *oplus_task_struct_cachep;

#define ots_to_ts(ots)	(ots->task)
#define OTS_IDX			0

static inline struct oplus_task_struct *get_oplus_task_struct(struct task_struct *t)
{
	struct oplus_task_struct *ots = NULL;

	/* Skip idle thread */
	if (!t || !t->pid)
		return NULL;

	/*
	 * the ots space is allocated here for threads
	 * that were created before the module loaded.
	 */
	ots = (struct oplus_task_struct *) READ_ONCE(t->android_oem_data1[OTS_IDX]);
	if (IS_ERR_OR_NULL(ots)) {
		/*
		 * If the thread is about to enter the TASK_DEAD state,
		 * there is no need to allocate space for it.
		 */
		if (READ_ONCE(t->__state) == TASK_DEAD)
			return NULL;

		/*
		 * Do not handle the kswapd0 thread, otherwise deadlock will occur.
		 * More details can be found at ALMID: 4469972.
		 */
		if (t->flags & PF_KSWAPD)
			return NULL;

		ots = kmem_cache_alloc(oplus_task_struct_cachep, GFP_ATOMIC);
		if (IS_ERR_OR_NULL(ots))
			return NULL;

		ots->task = t;
		smp_mb();

		WRITE_ONCE(t->android_oem_data1[OTS_IDX], (u64) ots);
	}

	return ots;
}

static inline int oplus_get_im_flag(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	if (IS_ERR_OR_NULL(ots))
		return INVALID_IM_FLAG;

	return ots->im_flag;
}

static inline bool is_optimized_audio_thread(struct task_struct *t)
{
	if (oplus_get_im_flag(t) == IM_FLAG_3RD_AUDIO)
		return true;

	return false;
}

static inline void oplus_set_im_flag(struct task_struct *t, int im_flag)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	if (IS_ERR_OR_NULL(ots))
		return;

	ots->im_flag = im_flag;
}

static inline int oplus_get_ux_state(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	if (IS_ERR_OR_NULL(ots))
		return 0;

	return ots->ux_state;
}

static inline int oplus_set_ux_state(struct task_struct *t, int ux_state)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	if (IS_ERR_OR_NULL(ots))
		return -EFAULT;

	ots->ux_state = ux_state;

	return 0;
}

static inline s64 oplus_get_inherit_ux(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	if (IS_ERR_OR_NULL(ots))
		return 0;

	return atomic64_read(&ots->inherit_ux);
}

static inline void oplus_set_inherit_ux(struct task_struct *t, s64 inherit_ux)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	if (IS_ERR_OR_NULL(ots))
		return;

	atomic64_set(&ots->inherit_ux, inherit_ux);
}

static inline struct list_head *oplus_get_ux_entry(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	if (IS_ERR_OR_NULL(ots))
		return NULL;

	return &ots->ux_entry;
}

static inline int oplus_get_ux_depth(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	if (IS_ERR_OR_NULL(ots))
		return 0;

	return ots->ux_depth;
}

static inline void oplus_set_ux_depth(struct task_struct *t, int ux_depth)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	if (IS_ERR_OR_NULL(ots))
		return;

	ots->ux_depth = ux_depth;
}

static inline u64 oplus_get_enqueue_time(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	if (IS_ERR_OR_NULL(ots))
		return 0;

	return ots->enqueue_time;
}

static inline void oplus_set_enqueue_time(struct task_struct *t, u64 enqueue_time)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	if (IS_ERR_OR_NULL(ots))
		return;

	ots->enqueue_time = enqueue_time;
}

static inline void oplus_set_inherit_ux_start(struct task_struct *t, u64 start_time)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	if (IS_ERR_OR_NULL(ots))
		return;

	ots->inherit_ux_start = start_time;
}

static inline int oplus_get_tpd(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	if (IS_ERR_OR_NULL(ots))
		return 0;

	return (ots->tpd > 0) ? ots->tpd : 0;
}

static inline void oplus_set_tpd(struct task_struct *t, int tpd)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	if (IS_ERR_OR_NULL(ots))
		return;

	ots->tpd = tpd;
}

static inline void init_task_ux_info(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	if (IS_ERR_OR_NULL(ots))
		return;

	INIT_LIST_HEAD(&ots->ux_entry);
	ots->ux_state = 0;
	atomic64_set(&ots->inherit_ux, 0);
	ots->ux_depth = 0;
	ots->enqueue_time = 0;
	ots->inherit_ux_start = 0;
	ots->tpd = 0;
#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
	ots->lb_state = 0;
	ots->ld_flag = 0;
#endif
	ots->exec_calc_runtime = 0;
	ots->is_update_runtime = 0;
	ots->target_process = -1;
	ots->wake_tid = 0;
	ots->running_start_time = 0;
	ots->update_running_start_time = false;
	ots->last_wake_ts = 0;
#if IS_ENABLED(CONFIG_OPLUS_LOCKING_STRATEGY)
	memset(&ots->lkinfo, 0, sizeof(struct locking_info));
#endif
}

static inline bool test_sched_assist_ux_type(struct task_struct *task, unsigned int sa_ux_type)
{
	return oplus_get_ux_state(task) & sa_ux_type;
}

static inline bool is_tpd_task(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	if (IS_ERR_OR_NULL(ots))
		return false;

	return (ots->tpd > 0);
}

static inline bool is_heavy_ux_task(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	if (IS_ERR_OR_NULL(ots))
		return false;

	return ots->ux_state & SA_TYPE_HEAVY;
}

static inline bool is_anim_ux_task(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	if (IS_ERR_OR_NULL(ots))
		return false;

	return ots->ux_state & SA_TYPE_ANIMATOR;
}

static inline bool sched_assist_scene(unsigned int scene)
{
	if (unlikely(!global_sched_assist_enabled))
		return false;

	return global_sched_assist_scene & scene;
}

static inline unsigned long oplus_task_util(struct task_struct *p)
{
#if IS_ENABLED(CONFIG_SCHED_WALT)
	struct walt_task_struct *wts = (struct walt_task_struct *) p->android_vendor_data1;

	return wts->demand_scaled;
#else
	return READ_ONCE(p->se.avg.util_avg);
#endif
}

#if IS_ENABLED(CONFIG_SCHED_WALT)
static inline u32 task_wts_sum(struct task_struct *tsk)
{
	struct walt_task_struct *wts = (struct walt_task_struct *) tsk->android_vendor_data1;

	return wts->sum;
}
#endif
void hwbinder_systrace_c(unsigned int cpu, int flag);
void sched_assist_init_oplus_rq(void);
void queue_ux_thread(struct rq *rq, struct task_struct *p, int enqueue);

void inherit_ux_inc(struct task_struct *task, int type);
void inherit_ux_sub(struct task_struct *task, int type, int value);
void set_inherit_ux(struct task_struct *task, int type, int depth, int inherit_val);
void reset_inherit_ux(struct task_struct *inherit_task, struct task_struct *ux_task, int reset_type);
void unset_inherit_ux(struct task_struct *task, int type);
void unset_inherit_ux_value(struct task_struct *task, int type, int value);
void inc_inherit_ux_refs(struct task_struct *task, int type);
void clear_all_inherit_type(struct task_struct *p);

bool test_task_is_fair(struct task_struct *task);
bool test_task_is_rt(struct task_struct *task);

bool prio_higher(int a, int b);
bool test_task_ux(struct task_struct *task);
bool test_task_ux_depth(int ux_depth);
bool test_inherit_ux(struct task_struct *task, int type);
bool test_set_inherit_ux(struct task_struct *task);
bool test_task_identify_ux(struct task_struct *task, int id_type_ux);
bool test_list_pick_ux(struct task_struct *task);
int get_ux_state_type(struct task_struct *task);
void sched_assist_target_comm(struct task_struct *task, const char *buf);

void update_ux_sched_cputopo(void);
bool is_task_util_over(struct task_struct *tsk, int threshold);
bool oplus_task_misfit(struct task_struct *tsk, int cpu);
ssize_t oplus_show_cpus(const struct cpumask *mask, char *buf);
void adjust_rt_lowest_mask(struct task_struct *p, struct cpumask *local_cpu_mask, int ret, bool force_adjust);
bool sa_skip_rt_sync(struct rq *rq, struct task_struct *p, bool *sync);

void account_ux_runtime(struct rq *rq, struct task_struct *curr);

/* register vender hook in kernel/sched/topology.c */
void android_vh_build_sched_domains_handler(void *unused, bool has_asym);

/* register vender hook in kernel/sched/rt.c */
void android_rvh_select_task_rq_rt_handler(void *unused, struct task_struct *p, int prev_cpu, int sd_flag, int wake_flags, int *new_cpu);
void android_rvh_find_lowest_rq_handler(void *unused, struct task_struct *p, struct cpumask *local_cpu_mask, int ret, int *best_cpu);

/* register vender hook in kernel/sched/core.c */
void android_rvh_sched_fork_handler(void *unused, struct task_struct *p);
void android_rvh_enqueue_task_handler(void *unused, struct rq *rq, struct task_struct *p, int flags);
void android_rvh_dequeue_task_handler(void *unused, struct rq *rq, struct task_struct *p, int flags);
void android_rvh_schedule_handler(void *unused, struct task_struct *prev, struct task_struct *next, struct rq *rq);
void android_vh_scheduler_tick_handler(void *unused, struct rq *rq);

/* register vendor hook in kernel/cgroup/cgroup-v1.c */
void android_vh_cgroup_set_task_handler(void *unused, int ret, struct task_struct *task);
/* register vendor hook in kernel/signal.c  */
void android_vh_exit_signal_handler(void *unused, struct task_struct *p);
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_BAN_APP_SET_AFFINITY)
void android_vh_sched_setaffinity_early_handler(void *unused, struct task_struct *task, const struct cpumask *new_mask, int *skip);
#endif

#endif /* _OPLUS_SA_COMMON_H_ */
