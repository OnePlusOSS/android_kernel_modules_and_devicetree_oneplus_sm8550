/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef __DSI_IRIS_TIMING_SWITCH_DEF__
#define __DSI_IRIS_TIMING_SWITCH_DEF__
#include "dsi_iris_log.h"

enum {
	LOG_NORMAL_LEVEL,
	LOG_DEBUG_LEVEL,
	LOG_VERBOSE_LEVEL,
	LOG_VERY_VERBOSE_LEVEL,
	LOG_DUMP_CMDLIST_LEVEL = 10,
};

inline uint32_t iris_get_tm_sw_loglevel(void);
#define LOG_NORMAL_INFO	(IRIS_IF_LOGI())
#define LOG_DEBUG_INFO		\
	((iris_get_tm_sw_loglevel() >= LOG_DEBUG_LEVEL) || IRIS_IF_LOGD())
#define LOG_VERBOSE_INFO	\
	((iris_get_tm_sw_loglevel() >= LOG_VERBOSE_LEVEL) || IRIS_IF_LOGV())
#define LOG_VERY_VERBOSE_INFO	\
	((iris_get_tm_sw_loglevel() >= LOG_VERY_VERBOSE_LEVEL) || IRIS_IF_LOGVV())


/* Internal interface for I7 */
void iris_init_timing_switch_i7(void);
void iris_send_dynamic_seq_i7(void);


/* Internal interface for I7P */
void iris_init_timing_switch_i7p(void);
void iris_send_dynamic_seq_i7p(void);

#endif //__DSI_IRIS_TIMING_SWITCH_DEF__
