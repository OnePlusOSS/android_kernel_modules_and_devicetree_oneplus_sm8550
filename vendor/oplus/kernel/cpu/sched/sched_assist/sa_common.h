/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
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

#define SA_DEBUG_ON 0

#if (SA_DEBUG_ON >= 1)
#define DEBUG_BUG_ON(x) BUG_ON(x)
#define DEBUG_WARN_ON(x) WARN_ON(x)
#else
#define DEBUG_BUG_ON(x)
#define DEBUG_WARN_ON(x)
#endif

#define ux_err(fmt, ...) \
		pr_err("[sched_assist][%s]"fmt, __func__, ##__VA_ARGS__)
#define ux_warn(fmt, ...) \
		pr_warn("[sched_assist][%s]"fmt, __func__, ##__VA_ARGS__)
#define ux_debug(fmt, ...) \
		pr_info("[sched_assist][%s]"fmt, __func__, ##__VA_ARGS__)

#define UX_MSG_LEN		64
#define UX_DEPTH_MAX		5

/* define for debug */
#define DEBUG_SYSTRACE  (1 << 0)
#define DEBUG_FTRACE    (1 << 1)
#define DEBUG_PIPELINE  (1 << 2)

/* define for sched assist feature */
#define FEATURE_COMMON (1 << 0)
#define FEATURE_SPREAD (1 << 1)

#define UX_EXEC_SLICE (4000000U)

/* define for sched assist thread type, keep same as the define in java file */
#define SA_OPT_CLEAR				(0)
#define SA_TYPE_LIGHT				(1 << 0)
#define SA_TYPE_HEAVY				(1 << 1)
#define SA_TYPE_ANIMATOR			(1 << 2)
/* SA_TYPE_LISTPICK for camera */
#define SA_TYPE_LISTPICK			(1 << 3)
#define SA_OPT_SET					(1 << 7)
#define SA_OPT_SET_PRIORITY			(1 << 9)

/* The following ux value only used in kernel */
#define SA_TYPE_SWIFT				(1 << 14)
/* clear ux type when dequeue */
#define SA_TYPE_ONCE				(1 << 15)
#define SA_TYPE_INHERIT				(1 << 16)
#define SA_TYPE_URGENT_MASK	(SA_TYPE_LIGHT|SA_TYPE_ANIMATOR|SA_TYPE_SWIFT)
#define SCHED_ASSIST_UX_MASK	(SA_TYPE_LIGHT|SA_TYPE_HEAVY|SA_TYPE_ANIMATOR|SA_TYPE_LISTPICK|SA_TYPE_SWIFT)

/* load balance operation is performed on the following ux types */
#define SCHED_ASSIST_LB_UX			(SA_TYPE_SWIFT | SA_TYPE_ANIMATOR | SA_TYPE_LIGHT | SA_TYPE_HEAVY)
#define POSSIBLE_UX_MASK			(SA_TYPE_SWIFT | \
									SA_TYPE_LIGHT | \
									SA_TYPE_HEAVY | \
									SA_TYPE_ANIMATOR | \
									SA_TYPE_LISTPICK | \
									SA_TYPE_ONCE | \
									SA_TYPE_INHERIT)

#define SCHED_ASSIST_UX_PRIORITY_MASK	(0xFF000000)
#define SCHED_ASSIST_UX_PRIORITY_SHIFT	24

#define UX_PRIORITY_TOP_APP		0x0A000000
#define UX_PRIORITY_AUDIO		0x0A000000
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_PIPELINE)
#define UX_PRIORITY_PIPELINE	0x09000000
#endif

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

#define SYSTEM_UID             1000
#define FIRST_APPLICATION_UID  10000
#define LAST_APPLICATION_UID   19999

extern pid_t save_audio_tgid;
extern pid_t save_top_app_tgid;
extern unsigned int top_app_type;

/* define for boost threshold unit */
#define BOOST_THRESHOLD_UNIT (51)

#define MAX_CLUSTER          (3)

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

/*
 * WANNING:
 * new flag should be add before MAX_IM_FLAG_TYPE, never change
 * the value of those existed flag type.
 */
enum IM_FLAG_TYPE {
	INVALID_IM_FLAG = -1,
	IM_FLAG_NONE = 0,
	IM_FLAG_SURFACEFLINGER,
	IM_FLAG_HWC,			/* Discarded */
	IM_FLAG_RENDERENGINE,
	IM_FLAG_WEBVIEW,
	IM_FLAG_CAMERA_HAL,
	IM_FLAG_AUDIO,
	IM_FLAG_HWBINDER,
	IM_FLAG_LAUNCHER,
	IM_FLAG_LAUNCHER_NON_UX_RENDER,
	IM_FLAG_SS_LOCK_OWNER,
	IM_FLAG_FORBID_SET_CPU_AFFINITY, /* forbid setting cpu affinity from app */
	IM_FLAG_SYSTEMSERVER_PID,
	IM_FLAG_MIDASD,
	MAX_IM_FLAG_TYPE,
};

#define MAX_IM_FLAG_PRIO	MAX_IM_FLAG_TYPE

struct ux_sched_cluster {
	struct cpumask cpus;
	unsigned long capacity;
};

#define OPLUS_NR_CPUS (8)
#define OPLUS_MAX_CLS (5)
struct ux_sched_cputopo {
	int cls_nr;
	struct ux_sched_cluster sched_cls[OPLUS_NR_CPUS];

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
	cpumask_t oplus_cpu_array[2*OPLUS_MAX_CLS][OPLUS_MAX_CLS];
#endif
};

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CPU_JANKINFO)
/* hot-thread */
struct task_record {
#define RECOED_WINSIZE			(1 << 8)
#define RECOED_WINIDX_MASK		(RECOED_WINSIZE - 1)
	u8 winidx;
	u8 count;
};

struct uid_struct {
	uid_t uid;
	u64 uid_total_cycle;
	u64 uid_total_inst;
	spinlock_t lock;
	char leader_comm[TASK_COMM_LEN];
};

struct  amu_uid_entry {
	uid_t uid;
	struct uid_struct *uid_struct;
	struct hlist_node node;
};

#endif

#if IS_ENABLED(CONFIG_OPLUS_LOCKING_STRATEGY)
struct locking_info {
	u64 waittime_stamp;
	u64 holdtime_stamp;
	/*
	 * mutex or rwsem optimistic spin start time. Because a task
	 * can't spin both on mutex and rwsem at one time, use one common
	 * threshold time is OK.
	 */
	u64 opt_spin_start_time;
	struct task_struct *holder;
	u32 waittype;
	bool ux_contrib;
	/*
	 * Whether task is ux when it's going to be added to mutex or
	 * rwsem waiter list. It helps us check whether there is ux
	 * task on mutex or rwsem waiter list. Also, a task can't be
	 * added to both mutex and rwsem at one time, so use one common
	 * field is OK.
	 */
	bool is_block_ux;
};
#endif

/* Please add your own members of task_struct here :) */
struct oplus_task_struct {
	/* CONFIG_OPLUS_FEATURE_SCHED_ASSIST */
	struct rb_node ux_entry;
	struct rb_node exec_time_node;
	struct task_struct *task;
	atomic64_t inherit_ux;
	u64 enqueue_time;
	u64 inherit_ux_start;
	/* u64 sum_exec_baseline; */
	u64 total_exec;
	u64 vruntime;
	u64 preset_vruntime;
	int ux_state;
	u8 ux_depth;
	s8 ux_priority;
	s8 ux_nice;
	s8 im_flag;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_ABNORMAL_FLAG)
	int abnormal_flag;
#endif
	/* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */
	int lb_state;
	int ld_flag:1;
	/* CONFIG_OPLUS_FEATURE_TASK_LOAD */
	int is_update_runtime:1;
	int target_process;
	u64 wake_tid;
	u64 running_start_time;
	bool update_running_start_time;
	u64 exec_calc_runtime;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CPU_JANKINFO)
	struct task_record record[MAX_CLUSTER];	/* 2*u64 */
	u64 block_start_time;
#endif
	/* CONFIG_OPLUS_FEATURE_FRAME_BOOST */
	struct list_head fbg_list;
	raw_spinlock_t fbg_list_entry_lock;
	bool fbg_running; /* task belongs to a group, and in running */
	u8 fbg_state;
	s8 preferred_cluster_id;
	s8 fbg_depth;
	u64 last_wake_ts;
#ifdef CONFIG_LOCKING_PROTECT
	unsigned long locking_start_time;
	struct list_head locking_entry;
	int locking_depth;
#endif

#if IS_ENABLED(CONFIG_OPLUS_LOCKING_STRATEGY)
	struct locking_info lkinfo;
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FDLEAK_CHECK)
	u8 fdleak_flag;
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
	/* for loadbalance */
	struct plist_node rtb;		/* rt boost task */

	/*
	 * The following variables are used to calculate the time
	 * a task spends in the running/runnable state.
	 */
	u64 snap_run_delay;
	unsigned long snap_pcount;
#endif
#if IS_ENABLED(CONFIG_ARM64_AMU_EXTN) && IS_ENABLED(CONFIG_OPLUS_FEATURE_CPU_JANKINFO)
	struct uid_struct *uid_struct;
	u64 amu_instruct;
	u64 amu_cycle;
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_PIPELINE)
	u64 pipeline_enqueue_ts;
	u64 pipeline_switch_out_ts;
	int pipeline_cpu;
#endif
	/* for binder ux */
	int binder_async_ux_enable;
	bool binder_async_ux_sts;
	int binder_thread_mode;
	struct binder_node *binder_thread_node;
} ____cacheline_aligned;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
#define INVALID_PID						(-1)
struct oplus_lb {
	/* used for active_balance to record the running task. */
	pid_t pid;
};
#endif

struct oplus_rq {
	/* CONFIG_OPLUS_FEATURE_SCHED_ASSIST */
	struct rb_root_cached ux_list;
	/* a tree to track minimum exec vruntime */
	struct rb_root_cached exec_timeline;
	/* malloc this spinlock_t instead of built-in to shrank size of oplus_rq */
	spinlock_t *ux_list_lock;
	int nr_running;
	u64 min_vruntime;
	u64 load_weight;
#ifdef CONFIG_LOCKING_PROTECT
	struct list_head locking_thread_list;
	spinlock_t *locking_list_lock;
	int rq_locking_task;
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
	/* for loadbalance */
	struct oplus_lb lb;
#endif
};

extern int global_debug_enabled;
extern int global_sched_assist_enabled;
extern int global_sched_assist_scene;

struct rq;

/* attention: before insert .ko, task's list->prev/next is init with 0 */
static inline bool oplus_list_empty(struct list_head *list)
{
	return list_empty(list) || (list->prev == 0 && list->next == 0);
}

static inline bool oplus_rbtree_empty(struct rb_root_cached *list)
{
	return (rb_first_cached(list) == NULL);
}

/**
 * Check if there are ux tasks waiting to run on the specified cpu
 */
static inline bool orq_has_ux_tasks(struct oplus_rq *orq)
{
	return !oplus_rbtree_empty(&orq->ux_list);
}

/* attention: before insert .ko, task's node->__rb_parent_color is init with 0 */
static inline bool oplus_rbnode_empty(struct rb_node *node) {
	return RB_EMPTY_NODE(node) || (node->__rb_parent_color == 0);
}

static inline struct oplus_task_struct *ux_list_first_entry(struct rb_root_cached *list) {
	struct rb_node *leftmost = rb_first_cached(list);
	if (leftmost == NULL) {
		return NULL;
	}
	return rb_entry(leftmost, struct oplus_task_struct, ux_entry);
}

static inline struct oplus_task_struct *exec_timeline_first_entry(struct rb_root_cached *tree) {
	struct rb_node *leftmost = rb_first_cached(tree);
	if (leftmost == NULL) {
		return NULL;
	}
	return rb_entry(leftmost, struct oplus_task_struct, exec_time_node);
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

	/* not Skip idle thread */
	if (!t)
		return NULL;

	ots = (struct oplus_task_struct *) READ_ONCE(t->android_oem_data1[OTS_IDX]);
	if (IS_ERR_OR_NULL(ots))
		return NULL;

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
	if (oplus_get_im_flag(t) == IM_FLAG_AUDIO)
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

void oplus_set_ux_state_lock(struct task_struct *t, int ux_state, bool need_lock_rq);

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

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
	/*
	 * Record the number of task context switches
	 * and scheduling delays when enqueueing a task.
	 */
	ots->snap_pcount = t->sched_info.pcount;
	ots->snap_run_delay = t->sched_info.run_delay;
#endif
}

static inline void oplus_set_inherit_ux_start(struct task_struct *t, u64 start_time)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	if (IS_ERR_OR_NULL(ots))
		return;

	ots->inherit_ux_start = start_time;
}

static inline void init_task_ux_info(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	if (IS_ERR_OR_NULL(ots))
		return;

	RB_CLEAR_NODE(&ots->ux_entry);
	RB_CLEAR_NODE(&ots->exec_time_node);
	ots->ux_state = 0;
	atomic64_set(&ots->inherit_ux, 0);
	ots->ux_depth = 0;
	ots->enqueue_time = 0;
	ots->inherit_ux_start = 0;
	ots->ux_priority = -1;
	ots->ux_nice = -1;
	ots->vruntime = 0;
	ots->preset_vruntime = 0;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_ABNORMAL_FLAG)
	ots->abnormal_flag = 0;
#endif
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
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CPU_JANKINFO)
	ots->block_start_time = 0;
#endif
#ifdef CONFIG_LOCKING_PROTECT
	INIT_LIST_HEAD(&ots->locking_entry);
	ots->locking_start_time = 0;
	ots->locking_depth = 0;
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
	/* for loadbalance */
	ots->snap_pcount = 0;
	ots->snap_run_delay = 0;
	plist_node_init(&ots->rtb, MAX_IM_FLAG_PRIO);
#endif

#if IS_ENABLED(CONFIG_ARM64_AMU_EXTN) && IS_ENABLED(CONFIG_OPLUS_FEATURE_CPU_JANKINFO)
	ots->amu_cycle = 0;
	ots->amu_instruct = 0;
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_PIPELINE)
	ots->pipeline_enqueue_ts = 0;
	ots->pipeline_switch_out_ts = 0;
	ots->pipeline_cpu = -1;
#endif
};

static inline bool test_sched_assist_ux_type(struct task_struct *task, unsigned int sa_ux_type)
{
	return oplus_get_ux_state(task) & sa_ux_type;
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

bool is_min_cluster(int cpu);
bool is_max_cluster(int cpu);
bool is_mid_cluster(int cpu);
bool task_is_runnable(struct task_struct *task);
int get_ux_state(struct task_struct * task);

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
int ux_mask_to_prio(int ux_mask);
int ux_prio_to_mask(int prio);
#endif

noinline int tracing_mark_write(const char *buf);
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
unsigned int ux_task_exec_limit(struct task_struct *p);

void update_ux_sched_cputopo(void);
bool is_task_util_over(struct task_struct *tsk, int threshold);
bool oplus_task_misfit(struct task_struct *tsk, int cpu);
ssize_t oplus_show_cpus(const struct cpumask *mask, char *buf);
void adjust_rt_lowest_mask(struct task_struct *p, struct cpumask *local_cpu_mask, int ret, bool force_adjust);
bool sa_skip_rt_sync(struct rq *rq, struct task_struct *p, bool *sync);
bool sa_rt_skip_ux_cpu(int cpu);

/* s64 account_ux_runtime(struct rq *rq, struct task_struct *curr); */
unsigned int ux_task_exec_limit(struct task_struct *p);
void opt_ss_lock_contention(struct task_struct *p, int old_im, int new_im);

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
extern struct notifier_block process_exit_notifier_block;
#endif /* _OPLUS_SA_COMMON_H_ */
