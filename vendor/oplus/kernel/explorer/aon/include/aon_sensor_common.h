/*
 * Copyright (c) 2021 ZEKU Technology(Shanghai) Corp.,Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * Basecode Created :        2021/5/27 Author: wangyingju@zeku.com
 *
 */

#ifndef _AON_SENSOR_COMMON_H_
#define _AON_SENSOR_COMMON_H_

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include "aon_uapi.h"

#define CCI_MASTER 1
#define I2C_MASTER 2

#define MAX_REGULATOR 5
#define MAX_POWER_CONFIG 12
#define MAX_PER_FRAME_ARRAY 32
#define INVALID_VREG 100

enum aon_sensor_state_t {
	AON_SENSOR_INIT,
	AON_SENSOR_ACQUIRE,
	AON_SENSOR_CONFIG,
	AON_SENSOR_START,
};

enum aon_sensor_packet_opcodes {
	AON_SENSOR_PACKET_OPCODE_STREAMON,
	AON_SENSOR_PACKET_OPCODE_UPDATE,
	AON_SENSOR_PACKET_OPCODE_INITIAL_CONFIG,
	AON_SENSOR_PACKET_OPCODE_PROBE,
	AON_SENSOR_PACKET_OPCODE_CONFIG,
	AON_SENSOR_PACKET_OPCODE_STREAMOFF,
	AON_SENSOR_PACKET_OPCODE_GETMEM,
	AON_SENSOR_PACKET_OPCODE_OPMCLK,
	AON_SENSOR_PACKET_OPCODE_NOP = 127
};

enum camera_sensor_i2c_type {
	CAMERA_SENSOR_I2C_TYPE_INVALID,
	CAMERA_SENSOR_I2C_TYPE_BYTE,
	CAMERA_SENSOR_I2C_TYPE_WORD,
	CAMERA_SENSOR_I2C_TYPE_3B,
	CAMERA_SENSOR_I2C_TYPE_DWORD,
	CAMERA_SENSOR_I2C_TYPE_MAX,
};

enum i2c_freq_mode {
	I2C_STANDARD_MODE,
	I2C_FAST_MODE,
	I2C_CUSTOM_MODE,
	I2C_FAST_PLUS_MODE,
	I2C_MAX_MODES,
};

enum cam_sensor_i2c_cmd_type {
	CAM_SENSOR_I2C_WRITE_RANDOM,
	CAM_SENSOR_I2C_WRITE_BURST,
	CAM_SENSOR_I2C_WRITE_SEQ,
	CAM_SENSOR_I2C_READ_RANDOM,
	CAM_SENSOR_I2C_READ_SEQ,
	CAM_SENSOR_I2C_POLL
};

struct cam_sensor_i2c_reg_array {
	u32 reg_addr;
	u32 reg_data;
	u32 delay;
	u32 data_mask;
};

struct cam_sensor_i2c_reg_setting {
	struct cam_sensor_i2c_reg_array *reg_setting;
	u32                             size;
	enum                            camera_sensor_i2c_type addr_type;
	enum                            camera_sensor_i2c_type data_type;
	unsigned short                  delay;
	u8                              *read_buff;
	u32                             read_buff_len;
};

struct cam_sensor_i2c_seq_reg {
	u32                         reg_addr;
	u8                          *reg_data;
	u32                         size;
	enum camera_sensor_i2c_type addr_type;
};

struct i2c_settings_list {
	struct cam_sensor_i2c_reg_setting i2c_settings;
	struct cam_sensor_i2c_seq_reg     seq_settings;
	enum cam_sensor_i2c_cmd_type      op_code;
	struct list_head                  list;
};

struct i2c_settings_array {
	struct list_head list_head;
	s32              is_settings_valid;
};

struct i2c_data_settings {
	struct i2c_settings_array init_settings;
	struct i2c_settings_array config_settings;
	struct i2c_settings_array streamon_settings;
	struct i2c_settings_array streamoff_settings;
};

struct aon_camera_slave_info {
	u16 sensor_slave_addr;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	u16 sub_sensor_slave_addr;
#endif
	u16 sensor_id_reg_addr;
	u16 sensor_id;
	u16 sensor_id_mask;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	u8  addr_type;
	u8  data_type;
	u8  needI2cSwitch;
#endif
};

void aon_sensor_release_stream_resource(struct i2c_data_settings *i2c_data);
s32 delete_request(struct i2c_settings_array *i2c_array);

#endif /* _AON_SENSOR_COMMON_H_ */
