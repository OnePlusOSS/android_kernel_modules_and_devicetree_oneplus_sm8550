/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */


#ifndef _OPLUS_TRANS_CTRL_H_
#define _OPLUS_TRANS_CTRL_H_

#define OBPROC_CHECK_CYCLE_NS 100000000
#define OBWORK_TIMEOUT_NS 800000000
#define BG_THREAD (2)
#define TF_ASYNC_BOOST 0x40
#define BT_CGROUP_BACKGROUND (3)

enum {
	BINDER_LOOPER_STATE_REGISTERED  = 0x01,
	BINDER_LOOPER_STATE_ENTERED     = 0x02,
	BINDER_LOOPER_STATE_BACKGROUND  = 0x40,
};


struct ob_struct {
	struct binder_proc *ob_proc;
	struct list_head ob_list;
	pid_t pid;
	u64 ob_check_ts;
	bool init;
};

/* Please add your own members of binder_transaction here */
struct oplus_binder_transaction {
	u64 ob_begin;
};

/* kmi mismatch now, use until hook was added in binder_proc */
struct oplus_binder_proc {
	struct list_head ux_todo;
	uint32_t ux_count;
};

struct binder_proc_status{
	u64 warning;
	u64 warning_cg_bg;
	u64 async_mem_over_high;
	u64 async_mem_over_low;
	u64 sync_mem_over_high;
	u64 sync_mem_over_low;
};
#endif /* _OPLUS_TRANS_CTRL_H_ */
