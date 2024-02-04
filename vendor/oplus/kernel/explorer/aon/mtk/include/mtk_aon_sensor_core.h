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

#ifndef _MTK_AON_SENSOR_CORE_H_
#define _MTK_AON_SENSOR_CORE_H_
#include <linux/i2c.h>

#include "mtk_aon_soc_util.h"

#define MAX_MCLK_CURRENT_NUM 10

enum mtk_camera_power_seq_type {
	MP_MCLK,
	MP_RST,
	MP_PDN,
	MP_AVDD,
	MP_AFVDD,
	MP_DVDD,
	MP_DOVDD,
	MP_MCLK_DRIVER_CURRENT,
	MP_SEQ_MAX,
};

enum mtk_camera_vreg_name_t {
	CAM_AVDD,
	CAM_AFVDD,
	CAM_DVDD,
	CAM_DOVDD,
	CAM_VREG_MAX,
};

struct aon_sensor_power_setting {
	enum mtk_camera_power_seq_type seq_type;
	unsigned short                 seq_val;
	long                           config_val;
	unsigned short                 delay;
};

struct aon_pinctrl_info {
	struct pinctrl       *pinctrl;
	struct pinctrl_state *gpio_state_mclk[MAX_MCLK_CURRENT_NUM];
	struct pinctrl_state *gpio_state_rst[2];
	struct pinctrl_state *gpio_state_pdn[2];
};

struct aon_sensor_power_ctrl_t {
	struct device                     *dev;
	struct aon_sensor_power_setting   *power_setting;
	u16                               power_setting_size;
	struct aon_sensor_power_setting   *power_down_setting;
	u16                               power_down_setting_size;

	struct aon_sensor_power_setting    *vcm_power_setting;
	u16                                vcm_power_setting_size;
	s32                                vcm_on_idx;
	u8                                 vcm_is_on;

	struct aon_pinctrl_info           pinctrl_info;
	u8                                cam_pinctrl_status;
};

struct aon_sensor_board_info {
	struct aon_camera_slave_info   slave_info;
	struct aon_sensor_power_ctrl_t power_info;
};

struct aon_sensor_ctrl_t {
	char                         device_name[20];
	struct platform_device       *pdev;
	struct aon_hw_soc_info       soc_info;
	struct mutex                 aon_sensor_mutex;
	struct aon_sensor_board_info *sensordata;
	int master_type;
	struct i2c_client *i2c_client;
	struct i2c_client *eeprom_i2c_client;
	enum aon_sensor_state_t      sensor_state;
	u8                           is_probe_succeed;
	u8                           is_power_on;
	u32                          id;
	struct device_node           *of_node;
	struct i2c_data_settings     i2c_data;
	u32                          streamon_count;
	u32                          streamoff_count;
};

int explorer_aon_drv_cmd(void **ctrl, void *arg);
int explorer_aon_init(struct aon_sensor_ctrl_t **s_ctrl);
void explorer_aon_exit(struct aon_sensor_ctrl_t **s_ctrl);

s32 aon_fill_vreg_params(struct aon_hw_soc_info *soc_info,
		struct aon_sensor_power_setting *power_setting,
		u16 power_setting_size);
s32 aon_sensor_parse_dt_mtk(struct aon_sensor_ctrl_t *s_ctrl);

s32 aon_sensor_power_up(struct aon_sensor_ctrl_t *s_ctrl);
s32 aon_sensor_power_down(struct aon_sensor_ctrl_t *s_ctrl);

s32 aon_sensor_update_i2c_info(struct aon_cmd_i2c_info *i2c_info,
	struct aon_sensor_ctrl_t *s_ctrl);
s32 aon_sensor_update_eeprom_i2c_info(struct aon_cmd_i2c_info *i2c_info,
	struct aon_sensor_ctrl_t *s_ctrl);

s32 aon_sensor_modes_util(struct aon_sensor_ctrl_t *s_ctrl,
	struct i2c_settings_list *i2c_list);

s32 aon_io_read(struct aon_sensor_ctrl_t *s_ctrl,
	u32 addr, u32 *data,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type);

s32 aon_eeprom_io_read(struct aon_sensor_ctrl_t *s_ctrl,
		u32 addr, u32 *data,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type);

s32 aon_disable_mclk(struct aon_sensor_power_ctrl_t *ctrl,
		struct aon_hw_soc_info *soc_info);
s32 aon_enable_mclk(struct aon_sensor_power_ctrl_t *ctrl,
		struct aon_hw_soc_info *soc_info);
s32 aon_disable_mclk_cur(struct aon_sensor_power_ctrl_t *ctrl,
		struct aon_hw_soc_info *soc_info);
s32 aon_enable_mclk_cur(struct aon_sensor_power_ctrl_t * ctrl,
	struct aon_hw_soc_info * soc_info);

s32 aon_vcm_power_on(struct aon_sensor_power_ctrl_t *ctrl,
		struct aon_hw_soc_info *soc_info);
s32 aon_vcm_power_off(struct aon_sensor_power_ctrl_t *ctrl,
		struct aon_hw_soc_info *soc_info);

void aon_dinit_variables(void);

#endif /* _MTK_AON_SENSOR_DEV_H_ */
