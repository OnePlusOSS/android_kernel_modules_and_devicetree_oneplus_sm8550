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

#ifndef _QCOM_AON_SENSOR_CORE_H_
#define _QCOM_AON_SENSOR_CORE_H_

#include <linux/i2c.h>
#include <linux/version.h>
#include "qcom_aon_soc_util.h"

#define MAX_POLL_DELAY_MS 100
#define I2C_COMPARE_MATCH 0
#define I2C_COMPARE_MISMATCH 1

enum cam_cci_cmd_type {
	MSM_CCI_INIT,
	MSM_CCI_RELEASE,
	MSM_CCI_SET_SID,
	MSM_CCI_SET_FREQ,
	MSM_CCI_SET_SYNC_CID,
	MSM_CCI_I2C_READ,
	MSM_CCI_I2C_WRITE,
	MSM_CCI_I2C_WRITE_SEQ,
	MSM_CCI_I2C_WRITE_BURST,
	MSM_CCI_I2C_WRITE_ASYNC,
	MSM_CCI_GPIO_WRITE,
	MSM_CCI_I2C_WRITE_SYNC,
	MSM_CCI_I2C_WRITE_SYNC_BLOCK,
};

enum cci_i2c_master_t {
	MASTER_0,
	MASTER_1,
	MASTER_MAX,
};

struct cam_cci_wait_sync_cfg {
	u16 cid;
	s16 csid;
	u16 line;
	u16 delay;
};

struct cam_cci_gpio_cfg {
	u16 gpio_queue;
	u16 i2c_queue;
};

struct cam_cci_read_cfg {
	u32 addr;
	u16 addr_type;
	u8  *data;
	u16 num_byte;
	u16 data_type;
};

struct cam_sensor_cci_client {
	struct v4l2_subdev    *cci_subdev;
	u32                   freq;
	enum i2c_freq_mode    i2c_freq_mode;
	enum cci_i2c_master_t cci_i2c_master;
	u16                   sid;
	u16                   cid;
	u32                   timeout;
	u16                   retries;
	u16                   id_map;
	u16                   cci_device;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,15,0)
        bool                  is_probing;
#endif
};

struct cam_cci_ctrl {
	s32                                   status;
	struct cam_sensor_cci_client          *cci_info;
	enum cam_cci_cmd_type                 cmd;
	union {
		struct cam_sensor_i2c_reg_setting cci_i2c_write_cfg;
		struct cam_cci_read_cfg           cci_i2c_read_cfg;
		struct cam_cci_wait_sync_cfg      cci_wait_sync_cfg;
		struct cam_cci_gpio_cfg           gpio_cfg;
	} cfg;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,15,0)
        bool                                   is_probing;
#endif
};

extern struct v4l2_subdev *cam_cci_get_subdev(int cci_dev_index);

#define VIDIOC_MSM_CCI_CFG \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 23, struct cam_cci_ctrl)


enum msm_camera_power_seq_type {
	SENSOR_MCLK,
	SENSOR_VANA,
	SENSOR_VDIG,
	SENSOR_VIO,
	SENSOR_VAF,
	SENSOR_VAF_PWDM,
	SENSOR_CUSTOM_REG1,
	SENSOR_CUSTOM_REG2,
	SENSOR_RESET,
	SENSOR_STANDBY,
	SENSOR_CUSTOM_GPIO1,
	SENSOR_CUSTOM_GPIO2,
	SENSOR_VANA1,
	SENSOR_SEQ_TYPE_MAX,
};

enum cci_device_num {
	CCI_DEVICE_0,
	CCI_DEVICE_1,
	CCI_DEVICE_MAX,
};

enum msm_camera_vreg_name_t {
	CAM_VDIG,
	CAM_VIO,
	CAM_VANA,
	CAM_VAF,
	CAM_V_CUSTOM1,
	CAM_V_CUSTOM2,
	CAM_VREG_MAX,
};

enum sensor_sub_module {
	SUB_MODULE_SENSOR,
	SUB_MODULE_ACTUATOR,
	SUB_MODULE_EEPROM,
	SUB_MODULE_LED_FLASH,
	SUB_MODULE_CSID,
	SUB_MODULE_CSIPHY,
	SUB_MODULE_OIS,
	SUB_MODULE_EXT,
	SUB_MODULE_MAX,
};

enum msm_sensor_camera_id_t {
	CAMERA_0,
	CAMERA_1,
	CAMERA_2,
	CAMERA_3,
	CAMERA_4,
	CAMERA_5,
	CAMERA_6,
	CAMERA_7,
	CAMERA_8,
	CAMERA_9,
	MAX_CAMERAS,
};

struct aon_sensor_power_setting {
	enum msm_camera_power_seq_type seq_type;
	unsigned short                 seq_val;
	long                           config_val;
	unsigned short                 delay;
	void                           *data[10];
};

struct aon_camera_gpio_num_info {
	u16 gpio_num[SENSOR_SEQ_TYPE_MAX];
	u8  valid[SENSOR_SEQ_TYPE_MAX];
};

struct aon_pinctrl_info {
	struct pinctrl       *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
};

struct aon_sensor_power_ctrl_t {
	struct device                     *dev;
	struct aon_sensor_power_setting   *power_setting;
	u16                               power_setting_size;
	struct aon_sensor_power_setting   *power_down_setting;
	u16                               power_down_setting_size;
	struct aon_camera_gpio_num_info   *gpio_num_info;
	struct aon_pinctrl_info           pinctrl_info;
	u8                                cam_pinctrl_status;
};

struct aon_sensor_board_info {
	struct aon_camera_slave_info   slave_info;
	s32                            sensor_mount_angle;
	s32                            secure_mode;
	int                            modes_supported;
	s32                            pos_roll;
	s32                            pos_yaw;
	s32                            pos_pitch;
	s32                            subdev_id[SUB_MODULE_MAX];
	s32                            subdev_intf[SUB_MODULE_MAX];
	const                          char *misc_regulator;
	struct aon_sensor_power_ctrl_t power_info;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	struct cam_sensor_i2c_reg_setting shift_mode_setting;
#endif
};

struct aon_sensor_ctrl_t {
	char                         device_name[20];
	struct platform_device       *pdev;
	struct aon_hw_soc_info       soc_info;
	struct mutex                 aon_sensor_mutex;
	struct aon_sensor_board_info *sensordata;
	int master_type;
	int is_cci_on;
	enum cci_i2c_master_t        cci_i2c_master;
	enum cci_device_num          cci_num;
	struct cam_sensor_cci_client *cci_client;
	enum aon_sensor_state_t      sensor_state;
	u8                           is_probe_succeed;
	u8                           is_power_on;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	u8                           isAuxSensor;
#endif
	u32                          id;
	struct device_node           *of_node;
	struct i2c_data_settings     i2c_data;
	u32                          streamon_count;
	u32                          streamoff_count;
	int                          bob_reg_index;
	bool                         bob_pwm_switch;
};

int explorer_aon_drv_cmd(void **ctrl, void *arg);
#ifdef OPLUS_FEATURE_CAMERA_COMMON
int explorer_aon_init(struct aon_sensor_ctrl_t **s_ctrl,bool isAuxSensor);
#else
int explorer_aon_init(struct aon_sensor_ctrl_t **s_ctrl);
#endif
void explorer_aon_exit(struct aon_sensor_ctrl_t **s_ctrl);

s32 aon_fill_vreg_params(struct aon_hw_soc_info *soc_info,
	struct aon_sensor_power_setting *power_setting,
	u16 power_setting_size);
s32 aon_sensor_parse_dt(struct aon_sensor_ctrl_t *s_ctrl);

s32 aon_sensor_power_up(struct aon_sensor_ctrl_t *s_ctrl);
s32 aon_sensor_power_down(struct aon_sensor_ctrl_t *s_ctrl);

s32 aon_sensor_update_i2c_info(struct aon_cmd_i2c_info *i2c_info,
	struct aon_sensor_ctrl_t *s_ctrl);

s32 aon_sensor_modes_util(struct aon_sensor_ctrl_t *s_ctrl,
	struct i2c_settings_list *i2c_list);

s32 aon_io_read(struct aon_sensor_ctrl_t *s_ctrl,
	u32 addr, u32 *data,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type);

#ifdef OPLUS_FEATURE_CAMERA_COMMON
s32 aon_io_dev_write(struct cam_sensor_cci_client *client,
    struct cam_sensor_i2c_reg_setting *write_setting);
#endif

s32 aon_disable_mclk(struct aon_sensor_power_ctrl_t *ctrl,
	struct aon_hw_soc_info *soc_info);
s32 aon_enable_mclk(struct aon_sensor_power_ctrl_t *ctrl,
	struct aon_hw_soc_info *soc_info);
void aon_dinit_variables(void);

s32 aon_cci_release(struct aon_sensor_ctrl_t *s_ctrl);
s32 aon_cci_init(struct aon_sensor_ctrl_t *s_ctrl);

#endif /* _QCOM_AON_SENSOR_DEV_H_ */
