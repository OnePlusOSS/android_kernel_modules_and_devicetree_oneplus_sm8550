// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#include <linux/string.h>
#include <linux/kernel.h>

#ifndef __UAG_SYSTRACE_H_
#define __UAG_SYSTRACE_H_

static noinline int tracing_mark_write(const char *buf)
{
	trace_printk(buf);
	return 0;
}

static void uag_systrace_c(u64 util, unsigned int cpu, char *msg)
{
	char buf[256];

	snprintf(buf, sizeof(buf), "C|10000|%s_cpu%d|%llu\n", msg, cpu, util);
	tracing_mark_write(buf);
}
#endif /* __UAG_SYSTRACE_H_ */

