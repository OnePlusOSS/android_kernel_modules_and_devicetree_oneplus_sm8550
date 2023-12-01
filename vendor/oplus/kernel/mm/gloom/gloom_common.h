// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef _GLOOM_COMMON_H_
#define _GLOOM_COMMON_H_

#define REGISTER_TRACE_VH(vendor_hook, handler) \
	 { \
		ret = register_trace_##vendor_hook(handler, NULL); \
		if (ret) { \
			pr_err("Failed to register_trace_"#vendor_hook", ret=%d\n", ret); \
			return ret; \
		} \
	 }

#define UNREGISTER_TRACE_VH(vendor_hook, handler) \
	 { \
		unregister_trace_##vendor_hook(handler, NULL); \
	 }

#endif /* _GLOOM_COMMON_H_ */
