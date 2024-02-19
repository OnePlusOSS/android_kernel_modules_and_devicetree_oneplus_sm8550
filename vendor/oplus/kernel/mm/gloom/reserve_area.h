// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef _RESERVE_AREA_H_
#define _RESERVE_AREA_H_
#include <linux/mm.h>
#include <linux/ratelimit.h>

#include "va_feature_hash.h"

#define SIZE_10M	0xA00000
#define ZYGOTE_HEAP_DEFAULT_SIZE	64
#define GET_UNMMAPED_AREA_FIRST_LOW_LIMIT	0x12c00000

#define FLAG_BITS			4
#define RESERVE_AREA		0x001
#define ANTI_FRAGMENT_AREA	0x002
#define ZYGOTE_HEAP			0x004
#define RESERVE_LOGGING		0x008

#define RESERVE_FEATURES 	(RESERVE_AREA | ANTI_FRAGMENT_AREA | ZYGOTE_HEAP | RESERVE_LOGGING)
#define HEAP_SIZE_MASK		(~((1 << FLAG_BITS) - 1))

#define ANTI_FRAGMENT_AREA_BASE_SIZE	0x4900000
#define ANTI_FRAGMENT_AREA_MASK			0x1e00000
#define ANTI_FRAGMENT_AREA_ALIGN		(~(0xffff))

#define OHM_SCHED_TYPE_MAX 12

extern void get_unmapped_area_from_fragment_pool_handler(void *data, struct mm_struct *mm,
		struct vm_unmapped_area_info *info, unsigned long *addr);
extern void get_unmapped_area_exclude_reserved_zone_handler(void *data, struct mm_struct *mm,
		struct vm_unmapped_area_info *info);
extern void get_unmapped_area_include_reserved_zone_handler(void *data, struct mm_struct *mm,
		struct vm_unmapped_area_info *info, unsigned long *addr);
extern void exit_mm_handler(void *data, struct mm_struct *mm);
extern void trigger_svm_oom_event(struct va_metadata *data, struct mm_struct *mm,
		bool brk_risk, bool is_locked);
extern int get_va_feature_value(unsigned int feature);
extern void update_oom_pid_and_time(unsigned long len, unsigned long val, unsigned long flags);

#ifdef CONFIG_MMU
extern void special_arch_pick_mmap_layout(struct mm_struct *mm);
#else
static inline void special_arch_pick_mmap_layout(struct mm_struct *mm) {}
#endif

#endif /* _RESERVE_AREA_H_ */
