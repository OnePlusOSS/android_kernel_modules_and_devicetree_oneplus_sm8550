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

#ifndef __UAPI_AON_SENSOR_H__
#define __UAPI_AON_SENSOR_H__

#include <linux/types.h>

#ifndef u8
#define u8 unsigned char
#endif

#ifndef u16
#define u16 unsigned short
#endif

#ifndef u32
#define u32 unsigned int
#endif

#ifndef u64
#define u64 unsigned long long
#endif

enum aon_sensor_cmd_opcode {
	AON_SENSOR_CMD_PROBE = 0,
	AON_SENSOR_CMD_ACQUIRE,
	AON_SENSOR_CMD_CONFIG,
	AON_SENSOR_CMD_START,
	AON_SENSOR_CMD_STOP,
	AON_SENSOR_CMD_RELEASE,
	AON_SENSOR_CMD_READREG,
	AON_SENSOR_CMD_GETMEM,
	AON_SENSOR_CMD_OPMCLK,
#ifdef MTK_AON
	AON_SENSOR_CMD_VCM_POWER,
	AON_SENSOR_CMD_GET_EEPROM_INFO,
#endif
	AON_SENSOR_CMD_MAX
};

struct aon_sensor_cmd_buf_desc {
	int slave_info_offset;
	int powerup_setting_offset;
	int powerup_setting_count;
	int powerdown_setting_offset;
	int powerdown_setting_count;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	int shiftmode_setting_offset;
	int shiftmode_setting_count;
#endif
#ifdef MTK_AON
	int powerup_vcm_setting_offset;
	int powerup_vcm_setting_count;
	int eeprom_slave_info_offset;
#endif
	int config_regsetting_offset;
	int config_regsetting_count;
	int read_regs_offset;
	int read_regs_count;
	u64 read_vals_addr;
	u8  is_init_config_valid;
	u8  is_streamon_config_valid;
	u8  is_streamoff_config_valid;
	u8  is_res_config_valid;
	u8 mclk_enabled;
#ifdef MTK_AON
	u8 vcm_power_on;
#endif
};

struct aon_sensor_cmd {
	enum aon_sensor_cmd_opcode     aon_cmd_opcode;
	struct aon_sensor_cmd_buf_desc aon_cmd_bufdesc;
	unsigned long                  aon_cmd_bufhandle;
	unsigned long                  aon_cmd_bufsize;
};

struct aon_slave_info_data {
	unsigned int  slave_address;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	unsigned int  sub_slave_address;
#endif
	unsigned int  sensor_id_regaddr;
	unsigned int  sensor_id;
	unsigned int  sensor_id_mask;
	unsigned char i2c_frequency_mode;
	unsigned char reg_addrtype;
	unsigned char reg_datatype;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	unsigned char needI2cSwitch;
#endif
	unsigned char reserve;
};

/*
 * struct aon_power_setting: Description of AON sensor power setting
 * @configType:  Config type
 *                 MCLK         = 0,
 *                 VANA         = 1,
 *                 VDIG         = 2,
 *                 VIO          = 3,
 *                 VAF          = 4,
 *                 VAF_PWDN     = 5,
 *                 CUSTOM_REG1  = 6,
 *                 CUSTOM_REG2  = 7,
 *                 RESET        = 8,
 *                 STANDBY      = 9,
 *                 CUSTOM_GPIO1 = 10,
 *                 CUSTOM_GPIO2 = 11
*/
struct aon_power_setting {
	unsigned int config_type;
	unsigned int config_value;
	unsigned int delayMs;
};

/*
 * struct aon_reg_setting: AON sensor register setting
 * @regAddrType: Register address type in bytes
 * @regDataType: Register data type in bytes
 * @operation:   Register operation type
 *                  WRITE            = 0,
 *                  WRITE_BURST      = 1,
 *                  WRITE_SEQUENTIAL = 2,
 *                  READ             = 3,
 *                  POLL             = 4
 * @reserve:     Reserve data for alignment
*/
struct aon_reg_setting {
	unsigned int  register_addr;
	unsigned int  register_data;
	unsigned char regAddr_type;
	unsigned char regData_type;
	unsigned char operation;
	unsigned char reserve;
	unsigned int  delayus;
};

/* camera op codes */
#define CAM_OPCODE_BASE                     0x100
#define CAM_QUERY                           (CAM_OPCODE_BASE + 0x1)
#define CAM_ACQUIRE                         (CAM_OPCODE_BASE + 0x2)
#define CAM_START                           (CAM_OPCODE_BASE + 0x3)
#define CAM_STOP                            (CAM_OPCODE_BASE + 0x4)
#define CAM_CONFIG                          (CAM_OPCODE_BASE + 0x5)
#define CAM_RELEASE                         (CAM_OPCODE_BASE + 0x6)
#define CAM_PROBE                           (CAM_OPCODE_BASE + 0x7)


/**
 * struct i2c_rdwr_header - header of READ/WRITE I2C command
 *
 * @ count           :   Number of registers / data / reg-data pairs
 * @ data_type       :   I2C data type
 * @ addr_type       :   I2C address type

 */
struct i2c_rdwr_header {
	u16    count;
	u8     data_type;
	u8     addr_type;

} __attribute__((packed));

/**
 * struct aon_control - Structure used by ioctl control for aon sensor
 *
 * @op_code:            This is the op code for aon sensor control
 * @size:               Control command size
 * @handle_type:        User pointer or shared memory handle
 * @reserved:           Reserved field for 64 bit alignment
 * @handle:             Control command payload
 */
struct aon_control {
	u32        op_code;
	u32        size;
	u32        handle_type;
	u32        reserved;
	u64        handle;
};

/**
 * struct aon_cmd_i2c_info - Contains slave I2C related info
 *
 * @slave_addr      :    Slave address
 * @i2c_freq_mode   :    4 bits are used for I2c freq mode
 */
struct aon_cmd_i2c_info {
	u16    slave_addr;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	u16    sub_slave_addr;
#endif
	u8     i2c_freq_mode;
} __attribute__((packed));

/**
 * struct aon_cmd_probe - Contains sensor slave info
 *
 * @data_type       :   Slave register data type
 * @addr_type       :   Slave register address type
 * @reg_addr        :   Slave register address
 * @expected_data   :   Data expected at slave register address
 * @data_mask       :   Data mask if only few bits are valid
 */
struct aon_cmd_probe {
	u8     data_type;
	u8     addr_type;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	u8     needI2cSwitch;
#endif
	u32    reg_addr;
	u32    expected_data;
	u32    data_mask;
} __attribute__((packed));

#endif
