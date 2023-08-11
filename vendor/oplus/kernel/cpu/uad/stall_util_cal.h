// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Oplus. All rights reserved.
 */

#ifndef __STALL_UTIL_CAL_H__
#define __STALL_UTIL_CAL_H__

#include "cpufreq_uag.h"

enum amu_counters {
	SYS_AMU_CONST_CYC,
	SYS_AMU_CORE_CYC,
	SYS_AMU_INST_RET,
	SYS_AMU_STALL_MEM,
	SYS_AMU_MAX,
};

struct amu_data {
	u64 val[SYS_AMU_MAX];
};

enum amu_report_policy {
	REPORT_NONE,
	REPORT_MAX_UTIL,
	REPORT_MIN_UTIL,
	REPORT_DIRECT,
	REPORT_REDUCE_STALL,
	REPORT_NORMAL_REDUCE_STALL,
	REPORT_MAX_TYPES
};

extern void uag_adjust_util(int cpu, unsigned long *util, void *pol);
extern void uag_update_counter(struct uag_gov_policy *sg_pol);
extern void uag_register_stall_update(void);
extern void uag_unregister_stall_update(void);
#endif
