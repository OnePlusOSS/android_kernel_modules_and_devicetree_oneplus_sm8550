/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#ifndef _CAM_OIS_CORE_H_
#define _CAM_OIS_CORE_H_

#include <linux/cma.h>
#include "cam_ois_dev.h"

#define OIS_NAME_LEN 32

/**
 * @power_info: power setting info to control the power
 *
 * This API construct the default ois power setting.
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int32_t cam_ois_construct_default_power_setting(
	struct cam_sensor_power_ctrl_t *power_info);


int cam_ois_driver_cmd(struct cam_ois_ctrl_t *e_ctrl, void *arg);

/**
 * @o_ctrl: OIS ctrl structure
 *
 * This API handles the shutdown ioctl/close
 */
void cam_ois_shutdown(struct cam_ois_ctrl_t *o_ctrl);
#ifdef OPLUS_FEATURE_CAMERA_COMMON
int cam_ois_power_up(struct cam_ois_ctrl_t *o_ctrl);
int cam_ois_power_down(struct cam_ois_ctrl_t *o_ctrl);
int ois_power_down_thread(void *arg);
#endif

struct completion *cam_ois_get_i3c_completion(uint32_t index);

#endif
/* _CAM_OIS_CORE_H_ */
