// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2023 Oplus. All rights reserved.
 */

/* This file is used internally */

#ifndef __OPLUS_UFCS_NOTIFY_H__
#define __OPLUS_UFCS_NOTIFY_H__

struct ufcs_class;

enum ufcs_notify_state {
	UFCS_NOTIFY_SOURCE_HW_RESET = 0,
	UFCS_NOTIFY_CABLE_HW_RESET,
	UFCS_NOTIFY_POWER_CHANGE,
	UFCS_NOTIFY_EXIT,
};

#if IS_ENABLED(CONFIG_OPLUS_UFCS_CLASS)
int ufcs_reg_event_notifier(struct notifier_block *nb);
void ufcs_unreg_event_notifier(struct notifier_block *nb);
void ufcs_send_state(struct ufcs_class *class, enum ufcs_notify_state state);
#else

__maybe_unused
static inline int ufcs_reg_event_notifier(struct notifier_block *nb)
{
	return -EINVAL;
}

__maybe_unused
static inline void ufcs_unreg_event_notifier(struct notifier_block *nb)
{
	return;
}

__maybe_unused
static inline void ufcs_send_state(struct ufcs_class *class, enum ufcs_notify_state state)
{
	return;
}

#endif /* CONFIG_OPLUS_UFCS_CLASS */

#endif /* __OPLUS_UFCS_NOTIFY_H__ */
