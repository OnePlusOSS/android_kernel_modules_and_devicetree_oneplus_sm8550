/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
 */


#ifndef _OPLUS_CAM_ACTUATOR_DEV_H_
#define _OPLUS_CAM_ACTUATOR_DEV_H_

#include "cam_actuator_dev.h"
#include "oplus_cam_sensor_core.h"

#define AK7316_SLAVE_ADDR 0x18
#define AK7316_STATE_ADDR 0x02
#define AK7316_PID_VER_ADDR 0x16
#define AK7316_STANDBY_STATE 0x40
#define AK7316_WRITE_CONTROL_ADDR 0xAE
#define AK7316_STORE_ADDR 0x03
#define AK7316_DATA_CHECK_ADDR 0x4B
#define AK7316_DATA_CHECK_BIT 0x04
#define AK7316_ID_ADDR 0x03
#define AK7316_ID_DATA 0x1C
#define AK7316_PID_LENGTH 44

void oplus_cam_actuator_sds_enable(struct cam_actuator_ctrl_t *a_ctrl);
int32_t oplus_cam_actuator_lock(struct cam_actuator_ctrl_t *a_ctrl);
int32_t oplus_cam_actuator_unlock(struct cam_actuator_ctrl_t *a_ctrl);
int32_t oplus_cam_actuator_power_up(struct cam_actuator_ctrl_t *a_ctrl);
int32_t oplus_cam_actuator_power_down(struct cam_actuator_ctrl_t *a_ctrl);
int oplus_cam_actuator_ram_write(struct cam_actuator_ctrl_t *a_ctrl, uint32_t addr, uint32_t data);
int oplus_cam_actuator_ram_read(struct cam_actuator_ctrl_t *a_ctrl, uint32_t addr, uint32_t* data);
int oplus_cam_actuator_update_pid_ak7316(struct cam_actuator_ctrl_t *a_ctrl);
int oplus_cam_actuator_update_pid(void *arg);
int oplus_cam_actuator_ram_write_extend(struct cam_actuator_ctrl_t *a_ctrl,
	uint32_t addr, uint32_t data,unsigned short mdelay,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type);
int oplus_cam_actuator_ram_read_extend(struct cam_actuator_ctrl_t *a_ctrl,
	uint32_t addr, uint32_t *data,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type);


#endif /* _CAM_ACTUATOR_CORE_H_ */

