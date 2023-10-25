// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#include <linux/proc_fs.h>
#include <linux/kmemleak.h>

unsigned int sysctl_frame_boost_enable;
unsigned int sysctl_frame_boost_debug;
EXPORT_SYMBOL_GPL(sysctl_frame_boost_debug);
unsigned int sysctl_slide_boost_enabled;
EXPORT_SYMBOL_GPL(sysctl_slide_boost_enabled);
unsigned int sysctl_input_boost_enabled;
EXPORT_SYMBOL_GPL(sysctl_input_boost_enabled);

#define INPUT_BOOST_DURATION 1500000000
static struct hrtimer ibtimer;
static int intput_boost_duration;
static ktime_t ib_last_time;

void enable_input_boost_timer(void)
{
	ktime_t ktime;

	ib_last_time = ktime_get();
	ktime = ktime_set(0, intput_boost_duration);

	hrtimer_start(&ibtimer, ktime, HRTIMER_MODE_REL);
}

void disable_input_boost_timer(void)
{
	hrtimer_cancel(&ibtimer);
}

enum hrtimer_restart input_boost_timeout(struct hrtimer *timer)
{
	ktime_t now, delta;

	now = ktime_get();
	delta = ktime_sub(now, ib_last_time);

	ib_last_time = now;
	sysctl_input_boost_enabled = 0;

	return HRTIMER_NORESTART;
}

static int input_boost_ctrl_handler(struct ctl_table *table, int write, void __user *buffer,
	size_t *lenp, loff_t *ppos)
{
	int result;

	result = proc_dointvec(table, write, buffer, lenp, ppos);

	if (!write)
		goto out;

	disable_input_boost_timer();
	enable_input_boost_timer();
out:
	return result;
}

static int slide_boost_ctrl_handler(struct ctl_table *table, int write, void __user *buffer,
	size_t *lenp, loff_t *ppos)
{
	int result;

	result = proc_dointvec(table, write, buffer, lenp, ppos);

	if (!write)
		goto out;

	if (sysctl_input_boost_enabled && sysctl_slide_boost_enabled) {
		disable_input_boost_timer();
		sysctl_input_boost_enabled = 0;
	}

out:
	return result;
}

struct ctl_table frame_boost_table[] = {
	{
		.procname	= "frame_boost_enabled",
		.data		= &sysctl_frame_boost_enable,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0666,
		.proc_handler	= proc_dointvec,
	},
	{
		.procname	= "frame_boost_debug",
		.data		= &sysctl_frame_boost_debug,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0666,
		.proc_handler	= proc_dointvec,
	},
	{
		.procname	= "slide_boost_enabled",
		.data		= &sysctl_slide_boost_enabled,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0666,
		.proc_handler	= slide_boost_ctrl_handler,
	},
	{
		.procname	= "input_boost_enabled",
		.data		= &sysctl_input_boost_enabled,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0666,
		.proc_handler	= input_boost_ctrl_handler,
	},
	{ }
};

struct ctl_table fbg_base_table[] = {
	{
		.procname	= "fbg",
		.mode		= 0555,
		.child		= frame_boost_table,
	},
	{ },
};

void fbg_sysctl_init(void)
{
	struct ctl_table_header *hdr;

	sysctl_frame_boost_enable = 1;
	sysctl_frame_boost_debug = 0;
	sysctl_slide_boost_enabled = 0;
	sysctl_input_boost_enabled = 0;

	ib_last_time = ktime_get();
	intput_boost_duration = INPUT_BOOST_DURATION;
	hrtimer_init(&ibtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ibtimer.function = &input_boost_timeout;

	hdr = register_sysctl_table(fbg_base_table);
	kmemleak_not_leak(hdr);
}
