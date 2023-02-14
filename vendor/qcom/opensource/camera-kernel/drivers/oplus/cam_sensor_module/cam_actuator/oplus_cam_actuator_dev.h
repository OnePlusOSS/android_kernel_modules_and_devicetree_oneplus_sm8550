/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
 */


#ifndef _OPLUS_CAM_ACTUATOR_DEV_H_
#define _OPLUS_CAM_ACTUATOR_DEV_H_

#include "cam_actuator_dev.h"

void oplus_cam_actuator_sds_on(struct cam_actuator_ctrl_t *a_ctrl);
int32_t oplus_cam_actuator_lock_on(struct cam_actuator_ctrl_t *a_ctrl);
int32_t oplus_cam_actuator_lock_off_by_state(struct cam_actuator_ctrl_t *a_ctrl, uint32_t state);
int32_t oplus_cam_actuator_lock_off(struct cam_actuator_ctrl_t *a_ctrl);
int32_t oplus_cam_actuator_power_up(struct cam_actuator_ctrl_t *a_ctrl);
int32_t oplus_cam_actuator_power_down(struct cam_actuator_ctrl_t *a_ctrl);
int32_t oplus_cam_actuator_stand_by(struct cam_actuator_ctrl_t *a_ctrl);
int oplus_cam_actuator_ram_write(struct cam_actuator_ctrl_t *a_ctrl, uint32_t addr, uint32_t data);
int oplus_cam_actuator_ram_read(struct cam_actuator_ctrl_t *a_ctrl, uint32_t addr, uint32_t* data);

#endif /* _CAM_ACTUATOR_CORE_H_ */

