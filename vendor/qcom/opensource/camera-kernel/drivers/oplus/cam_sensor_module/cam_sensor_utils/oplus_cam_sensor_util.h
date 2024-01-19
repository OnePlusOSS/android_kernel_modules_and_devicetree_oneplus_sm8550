/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023, The Linux Foundation. All rights reserved.
 */

#ifndef _OPLUS_CAM_SENSOR_UTIL_H_
#define _OPLUS_CAM_SENSOR_UTIL_H_
int cam_sensor_core_power_up_vio(struct cam_sensor_power_setting *power_setting,
		struct cam_hw_soc_info *soc_info, int32_t vreg_idx);
#endif //_OPLUS_CAM_SENSOR_UTIL_H_
