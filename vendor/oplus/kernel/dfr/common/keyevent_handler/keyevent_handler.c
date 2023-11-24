// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022-2030 Oplus. All rights reserved.
 * Description : generic key event handler, use for oplus code to catch key event
 * Version : 1.0
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: %d: " fmt, __func__, __LINE__

#include <linux/types.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/notifier.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include "../../include/keyevent_handler.h"

static ATOMIC_NOTIFIER_HEAD(keyevent_notifier_chain);
static atomic_t keyevent_handler_registered_events[KEY_MAX] __read_mostly;

static void update_registered_event(unsigned int event_code, bool is_add)
{
	if (is_add)
		atomic_inc(&keyevent_handler_registered_events[event_code]);
	else
		atomic_dec(&keyevent_handler_registered_events[event_code]);
}

int keyevent_register_notifier(struct notifier_block *nb, const unsigned int *events, const size_t nr)
{
	size_t i = 0;

	pr_info("called.\n");
	for (; i < nr; i++)
		update_registered_event(events[i], true);

	return atomic_notifier_chain_register(&keyevent_notifier_chain, nb);
}
EXPORT_SYMBOL_GPL(keyevent_register_notifier);

int keyevent_unregister_notifier(struct notifier_block *nb, const unsigned int *events, const size_t nr)
{
	size_t i = 0;

	pr_info("called.\n");
	for (; i < nr; i++)
		update_registered_event(events[i], false);

	return atomic_notifier_chain_unregister(&keyevent_notifier_chain, nb);
}
EXPORT_SYMBOL_GPL(keyevent_unregister_notifier);

static bool event_is_registered(unsigned int event_type, unsigned int event_code)
{
	if (event_type != EV_KEY || event_code >= KEY_MAX)
		return false;

	return !!atomic_read(&keyevent_handler_registered_events[event_code]);
}

static void keyevent_handler_event(struct input_handle *handle, unsigned int event_type,
	unsigned int event_code, int value)
{
	struct keyevent_notifier_param param = {
		.keycode = event_code,
		.down = value,
	};

	//pr_info("called. before filter, event_type: %u, event_code: %u, value: %d\n", event_type, event_code, value);
	if (!event_is_registered(event_type, event_code))
		return;

	pr_info("after filter, event_type: %u, event_code: %u, value: %d\n", event_type, event_code, value);
	atomic_notifier_call_chain(&keyevent_notifier_chain, 0, &param);
}

static int keyevent_handler_connect(struct input_handler *handler, struct input_dev *dev,
	const struct input_device_id *id)
{
	struct input_handle *handle;
	int err;

	pr_info("called.\n");

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "oplus_keyevent_notifier";

	err = input_register_handle(handle);
	if (err)
		goto err_free_handle;

	err = input_open_device(handle);
	if (err)
		goto err_unregister_handle;

	return 0;

err_unregister_handle:
	input_unregister_handle(handle);
err_free_handle:
	kfree(handle);
	return err;
}

static void keyevent_handler_disconnect(struct input_handle *handle)
{
	pr_info("called.\n");
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id keyevent_handler_id_table[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ }, /* Terminating entry */
};

static struct input_handler keyevent_input_handler = {
	.event = keyevent_handler_event,
	.connect = keyevent_handler_connect,
	.disconnect = keyevent_handler_disconnect,
	.name = "oplus_keyevent_handler",
	.id_table = keyevent_handler_id_table,
};

static int __init keyevent_handler_init(void)
{
	size_t i = 0;

	pr_info("called.\n");
	for (; i < KEY_MAX; i++)
		atomic_set(&keyevent_handler_registered_events[i], 0);

	(void)input_register_handler(&keyevent_input_handler);
	return 0;
}

static void __exit keyevent_handler_exit(void)
{
	pr_info("called.\n");
	input_unregister_handler(&keyevent_input_handler);
}

module_init(keyevent_handler_init);
module_exit(keyevent_handler_exit);

MODULE_DESCRIPTION("oplus_keyevent_handler");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
