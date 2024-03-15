// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023, The Linux Foundation. All rights reserved.
 */
#include <linux/kernel.h>
#include <clocksource/arm_arch_timer.h>
#include "cam_sensor_util.h"
#include "cam_mem_mgr.h"
#include "cam_res_mgr_api.h"

int cam_sensor_core_power_up_vio(struct cam_sensor_power_setting *power_setting,
		struct cam_hw_soc_info *soc_info, int32_t vreg_idx)
{
	int rc = 0, i = 0;
	if(power_setting->seq_type == SENSOR_VIO && power_setting->config_val == 3){
		CAM_DBG(CAM_SENSOR, "seq_type:%d config_val:%d rgltr_delay = %d",
				power_setting->seq_type,
				power_setting->config_val, soc_info->rgltr_delay[vreg_idx]);
		for(i = 0 ; i < 2 ; i++) {
			rc =  cam_soc_util_regulator_enable(
				soc_info->rgltr[vreg_idx],
				soc_info->rgltr_name[vreg_idx],
				soc_info->rgltr_min_volt[vreg_idx],
				soc_info->rgltr_max_volt[vreg_idx],
				soc_info->rgltr_op_mode[vreg_idx],
				soc_info->rgltr_delay[vreg_idx]);
			if (rc) {
				CAM_ERR(CAM_SENSOR,
					"Reg Enable failed for %s",
					soc_info->rgltr_name[vreg_idx]);
				return rc;
			}
			usleep_range((power_setting->delay * 1000),
				(power_setting->delay * 1000));

			CAM_DBG(CAM_SENSOR, "SENSOR_VIO seq_type:%d power_setting->delay = %d",
				power_setting->seq_type, power_setting->delay);

			rc = cam_soc_util_regulator_disable(
				soc_info->rgltr[vreg_idx],
				soc_info->rgltr_name[vreg_idx],
				soc_info->rgltr_min_volt[vreg_idx],
				soc_info->rgltr_max_volt[vreg_idx],
				soc_info->rgltr_op_mode[vreg_idx],
				soc_info->rgltr_delay[vreg_idx]);
			if (rc) {
				CAM_ERR(CAM_SENSOR,
					"Reg: %s disable failed",
					soc_info->rgltr_name[vreg_idx]);
				return rc;
			}
			usleep_range((power_setting->delay * 1000) + 2000,
				(power_setting->delay * 1000) + 2000);

			CAM_DBG(CAM_SENSOR, "SENSOR_VIO seq_type:%d regulator_disable",
				power_setting->seq_type, power_setting->delay);
		}
	}

	return rc;
}
