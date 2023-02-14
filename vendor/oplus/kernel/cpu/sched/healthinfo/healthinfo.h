/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#ifndef _HEALTHINFO_H_
#define _HEALTHINFO_H_

#include <linux/latencytop.h>
#include <linux/sched.h>
#include <linux/cpumask.h>
#include <linux/cpuidle.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/mm.h>
#include <linux/cgroup.h>
#include <../../../mm/slab.h>
#if IS_ENABLED(CONFIG_CGROUP_SCHED)
#define SA_CGROUP_SYS_BACKGROUND	(1)
#define SA_CGROUP_FOREGROUND		(2)
#define SA_CGROUP_BACKGROUND		(3)
#define SA_CGROUP_TOP_APP			(4)
#define SA_CGROUP_UX				(9)
#endif
#define ohm_err(fmt, ...) \
        printk(KERN_ERR "[OHM_ERR][%s]"fmt, __func__, ##__VA_ARGS__)
#define ohm_debug(fmt, ...) \
        printk(KERN_INFO "[OHM_INFO][%s]"fmt, __func__, ##__VA_ARGS__)
#define ohm_debug_deferred(fmt, ...) \
		printk_deferred(KERN_INFO "[OHM_INFO][%s]"fmt, __func__, ##__VA_ARGS__)
#define ohm_err_deferred(fmt, ...) \
        printk_deferred(KERN_ERR "[OHM_ERR][%s]"fmt, __func__, ##__VA_ARGS__)

#define OHM_FLASH_TYPE_EMC 1
#define OHM_FLASH_TYPE_UFS 2

#define OHM_SCHED_TYPE_MAX 12

enum {
    JANK_TRACE_RUNNABLE = 0,
    JANK_TRACE_DSTATE,
    JANK_TRACE_SSTATE,
    JANK_TRACE_RUNNING,
};

struct jank_d_state {
	u64 iowait_ns;
	u64 downread_ns;
	u64 downwrite_ns;
	u64 mutex_ns;
	u64 other_ns;
	int cnt;
};

struct jank_s_state{
	u64 binder_ns;
	u64 epoll_ns;
	u64 futex_ns;
	u64 other_ns;
	int cnt;
};

struct jank_monitor_info {
	u64 runnable_state;
	u64 ltt_running_state; /* ns */
	u64 big_running_state; /* ns */
	struct jank_d_state d_state;
	struct jank_s_state s_state;
};

enum {
        /* SCHED_STATS 0 -11 */
        OHM_SCHED_IOWAIT = 0,
        OHM_SCHED_SCHEDLATENCY,
        OHM_SCHED_FSYNC,
        OHM_SCHED_EMMCIO,
        OHM_SCHED_DSTATE,
        OHM_SCHED_TOTAL,
        /* OTHER_TYPE 12 - */
        OHM_CPU_LOAD_CUR = OHM_SCHED_TYPE_MAX,
        OHM_MEM_MON,
        OHM_IOPANIC_MON,
        OHM_SVM_MON,
        OHM_RLIMIT_MON,
        OHM_ION_MON,
		OHM_MEM_VMA_ALLOC_ERR,
        OHM_TYPE_TOTAL
};

enum n_zone_stat_item {
	/* First 128 byte cacheline (assuming 64 bit words) */
	N_FREE_PAGES,
	N_ZONE_LRU_BASE, /* Used only for compaction and reclaim retry */
	N_ZONE_INACTIVE_ANON = N_ZONE_LRU_BASE,
	N_ZONE_ACTIVE_ANON,
	N_ZONE_INACTIVE_FILE,
	N_ZONE_ACTIVE_FILE,
	N_ZONE_UNEVICTABLE,
	N_ZONE_WRITE_PENDING,	/* Count of dirty, writeback and unstable pages */
	N_MLOCK,		/* mlock()ed pages found and moved off LRU */
	N_PAGETABLE,		/* used for pagetables */
	N_KERNEL_STACK_KB,	/* measured in KiB */
#if IS_ENABLED(CONFIG_SHADOW_CALL_STACK)
	N_KERNEL_SCS_BYTES,	/* measured in bytes */
#endif
	/* Second 128 byte cacheline */
	N_BOUNCE,
#if IS_ENABLED(CONFIG_ZSMALLOC)
	N_ZSPAGES,		/* allocated in zsmalloc */
#endif
	N_FREE_CMA_PAGES,
    N_IONCACHE_PAGES,
	N_VM_ZONE_STAT_ITEMS };

struct sched_stat_common {
        u64 max_ms;
        u64 high_cnt;
        u64 low_cnt;
        u64 total_ms;
        u64 total_cnt;
};

struct sched_stat_para {
        bool ctrl;
        bool logon;
        bool trig;
        int low_thresh_ms;
        int high_thresh_ms;
        u64 delta_ms;
        struct sched_stat_common all;
        struct sched_stat_common fg;
        struct sched_stat_common ux;
        struct sched_stat_common top;
        struct sched_stat_common bg;
        struct sched_stat_common sysbg;		
};

struct alloc_wait_para {
	u64 total_alloc_wait_max_order;
	u64 fg_alloc_wait_max_order;
	u64 ux_alloc_wait_max_order;
	struct sched_stat_common total_alloc_wait;
	struct sched_stat_common fg_alloc_wait;
	struct sched_stat_common ux_alloc_wait;
};

struct ion_wait_para {
	struct sched_stat_common ux_ion_wait;
	struct sched_stat_common fg_ion_wait;
	struct sched_stat_common total_ion_wait;
};

extern void ohm_schedstats_record(int sched_type, struct task_struct *task, u64 delta_ms);
extern int ohm_get_cur_cpuload(bool ctrl);
extern void ohm_action_trig_with_msg(int type, char *msg);
extern atomic_long_t ion_total_size;
extern bool ion_cnt_enable;
extern unsigned long ion_total(void);
extern unsigned long slabs_node(struct kmem_cache *s, int node);
extern unsigned long node_nr_slabs(struct kmem_cache_node *n);
extern void inc_slabs_node(struct kmem_cache *s, int node, int objects);
extern void dec_slabs_node(struct kmem_cache *s, int node, int objects);
extern int count_free(struct page *page);
extern unsigned long node_nr_objs(struct kmem_cache_node *n);
extern const struct file_operations proc_jank_trace_operations;
extern struct ion_wait_para ionwait_para;
extern void memory_alloc_monitor(gfp_t gfp_mask, unsigned int order, u64 wait_ms);
extern struct alloc_wait_para allocwait_para;
extern void ionwait_monitor(u64 wait_ms);
struct fg_info;
extern struct fg_info fginfo;
extern bool is_fg(int uid);
extern inline int current_is_fg(void);
extern inline int task_is_fg(struct task_struct *tsk);
extern void memory_alloc_monitor(gfp_t gfp_mask, unsigned int order, u64 wait_ms);

#endif /* _HEALTHINFO_H_*/

