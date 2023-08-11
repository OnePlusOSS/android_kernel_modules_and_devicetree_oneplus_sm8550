// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022-2030 Oplus. All rights reserved.
 * Description : generic key event handler, use for oplus code to catch key event
 * Version : 1.0
 */

#ifndef __KEY_EVENT_HANDLER__
#define __KEY_EVENT_HANDLER__
#include <linux/notifier.h>

struct keyevent_notifier_param {
	unsigned int keycode;
	int down;
};

int keyevent_register_notifier(struct notifier_block *nb, const unsigned int *events, const size_t nr);
int keyevent_unregister_notifier(struct notifier_block *nb, const unsigned int *events, const size_t nr);
#endif /*__KEY_EVENT_HANDLER__*/
