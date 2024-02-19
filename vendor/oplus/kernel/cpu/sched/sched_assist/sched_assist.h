/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */


#ifndef _OPLUS_SCHED_ASSIST_H_
#define _OPLUS_SCHED_ASSIST_H_

#include <trace/hooks/sched.h>
#include <trace/hooks/topology.h>
#include <trace/events/task.h>

#define REGISTER_TRACE_VH(vender_hook, handler) \
	{ \
		ret = register_trace_##vender_hook(handler, NULL); \
		if (ret) { \
			ux_err("failed to register_trace_"#vender_hook", ret=%d\n", ret); \
			return ret; \
		} \
	}

#define UNREGISTER_TRACE_VH(vender_hook, handler) \
	{ \
		unregister_trace_##vender_hook(handler, NULL); \
	}

#define REGISTER_TRACE_RVH		REGISTER_TRACE_VH


#ifdef VENDOR_DEBUG
#define UNREGISTER_TRACE_RVH	UNREGISTER_TRACE_VH
#else
#define UNREGISTER_TRACE_RVH(vender_hook, handler)
#endif

#endif /* _OPLUS_SCHED_ASSIST_H_ */
