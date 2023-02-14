/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#ifndef _OPLUS_LOCKING_MAIN_H_
#define _OPLUS_LOCKING_MAIN_H_

#define cond_trace_printk(cond, fmt, ...)	\
do {										\
	if (cond)								\
		trace_printk(fmt, ##__VA_ARGS__);	\
} while (0)

#define LK_MUTEX_ENABLE (1 << 0)
#define LK_RWSEM_ENABLE (1 << 1)
#define LK_FUTEX_ENABLE (1 << 2)

#define LK_DEBUG_PRINTK (1 << 0)
#define LK_DEBUG_FTRACE (1 << 1)

extern unsigned int g_opt_enable;
extern unsigned int g_opt_debug;

extern atomic64_t futex_inherit_set_times;
extern atomic64_t futex_inherit_unset_times;
extern atomic64_t futex_inherit_useless_times;
extern atomic64_t futex_low_count;
extern atomic64_t futex_high_count;

static inline bool locking_opt_enable(unsigned int enable)
{
	return g_opt_enable & enable;
}

static inline bool locking_opt_debug(int debug)
{
	return g_opt_debug & debug;
}

void register_rwsem_vendor_hooks(void);
void register_mutex_vendor_hooks(void);
void register_futex_vendor_hooks(void);
void register_monitor_vendor_hooks(void);
void lk_sysfs_init(void);

void unregister_rwsem_vendor_hooks(void);
void unregister_mutex_vendor_hooks(void);
void unregister_futex_vendor_hooks(void);
void unregister_monitor_vendor_hooks(void);
void lk_sysfs_exit(void);
#endif /* _OPLUS_LOCKING_MAIN_H_ */
