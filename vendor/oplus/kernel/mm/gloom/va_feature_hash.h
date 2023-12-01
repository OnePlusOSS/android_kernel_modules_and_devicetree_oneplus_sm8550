// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef _VA_FEATURE_HASH_H_
#define _VA_FEATURE_HASH_H_

#include <linux/types.h>

struct va_metadata {
	pid_t nr;
	int va_feature;
	unsigned int zygoteheap_in_mb;
	unsigned long va_feature_rnd;
	struct hlist_node pid_chain;
};

extern int pid_va_feature_hash_init(void);
extern void pid_va_feature_hash_deinit(void);
extern void oplus_gloom_va_feature_add_hash(pid_t pid, struct va_metadata *data);
extern struct va_metadata *oplus_gloom_va_feature_search_hash(pid_t pid);
extern void oplus_gloom_va_feature_delete_hash_and_trigger_event(pid_t pid, struct mm_struct *mm);

#endif /* _VA_FEATURE_HASH_H_ */
