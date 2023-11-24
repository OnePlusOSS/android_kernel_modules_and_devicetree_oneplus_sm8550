/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 */

#if !defined(OPLUS_CAM_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define OPLUS_CAM_TRACE_H

#undef TRACE_SYSTEM
#define TRACE_SYSTEM camera
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE ./cam_trace

#include <linux/tracepoint.h>

TRACE_EVENT(cam_tracing_mark_write,
	TP_PROTO(const char *string1),
	TP_ARGS(string1),
	TP_STRUCT__entry(
		__string(string1, string1)
	),
	TP_fast_assign(
		__assign_str(string1, string1);
	),
	TP_printk(
		"%s",
		__get_str(string1)
	)
);

#define STR_BUFFER_MAX_LENGTH  512
const char* GetFileName(const char* pFilePath);
extern pid_t camera_provider_pid;
#define trace_begin(...)                                                                                                        \
do {                                                                                                                            \
	char str_buffer[STR_BUFFER_MAX_LENGTH/2] = {0};                                                                             \
	snprintf(str_buffer, STR_BUFFER_MAX_LENGTH/2, "B|%d|%s:%d KMD ", camera_provider_pid, GetFileName(__FILE__), __LINE__);     \
	snprintf(str_buffer+strlen(str_buffer), STR_BUFFER_MAX_LENGTH/2 - strlen(str_buffer), __VA_ARGS__);                         \
	trace_cam_tracing_mark_write(str_buffer);                                                                                   \
} while (0)

#define trace_end()                                                                                                         \
do {                                                                                                                        \
	char str_buffer[STR_BUFFER_MAX_LENGTH/16] = {0};                                                                        \
	snprintf(str_buffer, STR_BUFFER_MAX_LENGTH/16, "E|%d", camera_provider_pid);                                            \
	trace_cam_tracing_mark_write(str_buffer);                                                                               \
} while (0)

#define trace_begin_end(...)                                                                                                \
do {                                                                                                                        \
	trace_begin(__VA_ARGS__);                                                                                               \
	trace_end();                                                                                                            \
} while (0)

#define trace_int(string, value)                                                                                            \
do {                                                                                                                        \
	char str_buffer[STR_BUFFER_MAX_LENGTH/4] = {0};                                                                         \
	snprintf(str_buffer, STR_BUFFER_MAX_LENGTH/4, "C|%d|%s|%d", camera_provider_pid, string, value);                        \
	trace_cam_tracing_mark_write(str_buffer);                                                                               \
} while (0)

#define trace_int_tag(string, value)                                                                                            \
do {                                                                                                                            \
    trace_int(string, value);                                                                                                   \
    trace_int(string, 0);                                                                                                       \
} while (0)

#endif /* _CAM_TRACE_H */
