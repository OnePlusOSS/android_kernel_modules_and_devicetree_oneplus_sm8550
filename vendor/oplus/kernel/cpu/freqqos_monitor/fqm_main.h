// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 Oplus. All rights reserved.
 */

#ifndef _FREQQOS_MONITOR_H
#define _FREQQOS_MONITOR_H

#include <linux/pm_qos.h>

void android_vh_freq_qos_add_request_handler(void *unused, struct freq_constraints *qos,
                struct freq_qos_request *req, enum freq_qos_req_type type, int value, int ret);
void android_vh_freq_qos_remove_request_handler(void *unused, struct freq_qos_request *req);
void android_vh_freq_qos_update_request_handler(void *unused, struct freq_qos_request *req, int new_value);

void fqm_set_threshold(int vaule, unsigned int cluster_id);
int fqm_get_threshold(unsigned int cluster_id);

extern int g_fqm_monitor_enable;
extern int g_fqm_debug_enable;
extern int max_cluster_num;
extern int default_fqm_threshold;


#endif /*_FREQQOS_MONITOR_H */
