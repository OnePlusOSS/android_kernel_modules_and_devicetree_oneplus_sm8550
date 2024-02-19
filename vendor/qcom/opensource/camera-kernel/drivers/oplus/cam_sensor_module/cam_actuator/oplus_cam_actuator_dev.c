// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 */

#include "cam_actuator_dev.h"
#include "cam_req_mgr_dev.h"
#include "cam_actuator_soc.h"
#include "cam_actuator_core.h"
#include "cam_trace.h"
#include "camera_main.h"

#include "oplus_cam_actuator_dev.h"
#include "oplus_cam_actuator_core.h"

struct cam_sensor_i2c_reg_setting_array ak7316_new_pid = {
	.reg_setting =
	{
		{.reg_addr = 0x10, .reg_data = 0x3A, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x11, .reg_data = 0x42, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x12, .reg_data = 0x87, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x13, .reg_data = 0x1E, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x14, .reg_data = 0x1E, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x15, .reg_data = 0x00, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x16, .reg_data = 0x17, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x17, .reg_data = 0x37, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x18, .reg_data = 0x8B, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x1A, .reg_data = 0x00, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x1B, .reg_data = 0x7F, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x1C, .reg_data = 0xFA, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x1D, .reg_data = 0xD2, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x1E, .reg_data = 0xF0, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x1F, .reg_data = 0x1E, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x20, .reg_data = 0x78, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x21, .reg_data = 0x28, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x22, .reg_data = 0x28, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x23, .reg_data = 0x0A, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x24, .reg_data = 0xB0, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x25, .reg_data = 0xFA, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x26, .reg_data = 0xC3, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x27, .reg_data = 0x81, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x28, .reg_data = 0x7C, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x29, .reg_data = 0x4D, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x2A, .reg_data = 0x2E, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x2B, .reg_data = 0xA1, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x2C, .reg_data = 0xA0, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x2D, .reg_data = 0xDB, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x2E, .reg_data = 0x30, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x2F, .reg_data = 0xB6, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x30, .reg_data = 0x00, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x31, .reg_data = 0x4D, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x32, .reg_data = 0x00, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x33, .reg_data = 0x0A, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x34, .reg_data = 0x70, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x35, .reg_data = 0x32, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x36, .reg_data = 0x77, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x3C, .reg_data = 0x50, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x3D, .reg_data = 0xC5, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x52, .reg_data = 0x0F, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x53, .reg_data = 0x64, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x54, .reg_data = 0x00, .delay = 0x00, .data_mask = 0x00},
		{.reg_addr = 0x55, .reg_data = 0x00, .delay = 0x00, .data_mask = 0x00},
	},
	.size = AK7316_PID_LENGTH,
	.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.delay = 1,
};

uint32_t ak7316_old_pid_v2[AK7316_PID_LENGTH][2]={
	{0x10, 0x42},
	{0x11, 0x42},
	{0x12, 0xA4},
	{0x13, 0x2C},
	{0x14, 0x16},
	{0x15, 0x08},
	{0x16, 0x17},
	{0x17, 0x36},
	{0x18, 0xAA},
	{0x1A, 0x00},
	{0x1B, 0x50},
	{0x1C, 0xE6},
	{0x1D, 0xCA},
	{0x1E, 0xCA},
	{0x1F, 0x0A},
	{0x20, 0x7F},
	{0x21, 0x18},
	{0x22, 0x00},
	{0x23, 0x36},
	{0x24, 0x00},
	{0x25, 0xE6},
	{0x26, 0xC2},
	{0x27, 0x47},
	{0x28, 0x7D},
	{0x29, 0xA4},
	{0x2A, 0x29},
	{0x2B, 0x74},
	{0x2C, 0xAB},
	{0x2D, 0xE8},
	{0x2E, 0x2A},
	{0x2F, 0xB9},
	{0x30, 0x00},
	{0x31, 0x24},
	{0x32, 0x00},
	{0x33, 0x00},
	{0x34, 0x00},
	{0x35, 0x00},
	{0x36, 0x70},
	{0x3C, 0x50},
	{0x3D, 0xC1},
	{0x52, 0x14},
	{0x53, 0x50},
	{0x54, 0x14},
	{0x55, 0x32},
};

uint32_t ak7316_old_pid_v3[AK7316_PID_LENGTH][2]={
	{0x10, 0x3A},
	{0x11, 0x3F},
	{0x12, 0x87},
	{0x13, 0x2C},
	{0x14, 0x1E},
	{0x15, 0x05},
	{0x16, 0x17},
	{0x17, 0x3F},
	{0x18, 0x9B},
	{0x1A, 0x00},
	{0x1B, 0x6E},
	{0x1C, 0xFA},
	{0x1D, 0xCD},
	{0x1E, 0xDC},
	{0x1F, 0x0F},
	{0x20, 0x78},
	{0x21, 0x0D},
	{0x22, 0x28},
	{0x23, 0x25},
	{0x24, 0x1E},
	{0x25, 0xE3},
	{0x26, 0xC2},
	{0x27, 0x47},
	{0x28, 0x7D},
	{0x29, 0xA4},
	{0x2A, 0x29},
	{0x2B, 0x74},
	{0x2C, 0xAB},
	{0x2D, 0xE8},
	{0x2E, 0x2A},
	{0x2F, 0xB9},
	{0x30, 0x00},
	{0x31, 0x31},
	{0x32, 0x00},
	{0x33, 0x50},
	{0x34, 0x77},
	{0x35, 0x3C},
	{0x36, 0x97},
	{0x3C, 0x50},
	{0x3D, 0xC1},
	{0x52, 0x14},
	{0x53, 0x50},
	{0x54, 0x14},
	{0x55, 0x32},
};

void oplus_cam_actuator_sds_enable(struct cam_actuator_ctrl_t *a_ctrl)
{
	mutex_lock(&(a_ctrl->actuator_mutex));
	a_ctrl->camera_actuator_shake_detect_enable = true;
	mutex_unlock(&(a_ctrl->actuator_mutex));
}

int32_t oplus_cam_actuator_lock(struct cam_actuator_ctrl_t *a_ctrl)
{
	int rc = 0;

	mutex_lock(&(a_ctrl->actuator_mutex));
	if (a_ctrl->camera_actuator_shake_detect_enable && a_ctrl->cam_act_last_state == CAM_ACTUATOR_INIT)
	{
		CAM_INFO(CAM_ACTUATOR, "SDS Actuator:%s lock start", a_ctrl->actuator_name);
		a_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;
		if (strstr(a_ctrl->actuator_name, "sem1217s"))
		{
			a_ctrl->io_master_info.cci_client->sid = (0xC2 >> 1);
		}
		else
		{
			a_ctrl->io_master_info.cci_client->sid = (0x18 >> 1);
		}
		a_ctrl->io_master_info.cci_client->retries = 0;
		a_ctrl->io_master_info.cci_client->id_map = 0;

		rc = oplus_cam_actuator_power_up(a_ctrl);
		if (rc < 0)
		{
			CAM_ERR(CAM_ACTUATOR, "Failed for Actuator Power up failed: %d", rc);
			mutex_unlock(&(a_ctrl->actuator_mutex));
			return rc;
		}

		if (strstr(a_ctrl->actuator_name, "sem1217s"))
		{
			rc = oplus_cam_actuator_ram_write_extend(a_ctrl, (uint32_t)0x0200, (uint32_t)0x01, 5,
				CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
			CAM_DBG(CAM_ACTUATOR, "SDS set reg: 0x0200, data: 0x01, rc = %d", rc);
			msleep(5);
			rc = oplus_cam_actuator_ram_write_extend(a_ctrl, (uint32_t)0x0200, (uint32_t)0x01, 5,
				CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
			CAM_DBG(CAM_ACTUATOR, "SDS set reg: 0x0200, data: 0x01, rc = %d", rc);
			msleep(5);

			rc = oplus_cam_actuator_ram_write_extend(a_ctrl, (uint32_t)0x0080, (uint32_t)0x01, 0,
				CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
			CAM_DBG(CAM_ACTUATOR, "SDS set reg: 0x0080, data: 0x01, rc = %d", rc);
			rc = oplus_cam_actuator_ram_write_extend(a_ctrl, (uint32_t)0x0080, (uint32_t)0x00, 0,
				CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
			CAM_DBG(CAM_ACTUATOR, "SDS set reg: 0x0080, data: 0x00, rc = %d", rc);
		}
		else
		{
			rc = oplus_cam_actuator_ram_write(a_ctrl, 0x02, 0x40);
			CAM_DBG(CAM_ACTUATOR, "SDS set reg: 0x02, data: 0x40, rc = %d", rc);
			msleep(5);
			rc = oplus_cam_actuator_ram_write(a_ctrl, 0x02, 0x40);
			CAM_DBG(CAM_ACTUATOR, "SDS set reg: 0x02, data: 0x40, rc = %d", rc);
			msleep(5);

			rc = oplus_cam_actuator_ram_write(a_ctrl, 0x00, 0x80);
			CAM_DBG(CAM_ACTUATOR, "SDS set reg: 0x00, data: 0x80, rc = %d", rc);
			rc = oplus_cam_actuator_ram_write(a_ctrl, 0x02, 0x00);
			CAM_DBG(CAM_ACTUATOR, "SDS set reg: 0x02, data: 0x00, rc = %d", rc);
		}

		if (rc < 0)
		{
			int rc_power_down = oplus_cam_actuator_power_down(a_ctrl);
			if (rc_power_down < 0)
			{
				CAM_ERR(CAM_ACTUATOR, "SDS oplus_cam_actuator_power_down fail, rc_power_down = %d", rc_power_down);
			}
		}
		else
		{
			a_ctrl->cam_act_last_state = CAM_ACTUATOR_LOCK;
		}
	}
	else
	{
		CAM_ERR(CAM_ACTUATOR, "do not support SDS(shake detect service)");
	}
	mutex_unlock(&(a_ctrl->actuator_mutex));

	return rc;
}

int32_t oplus_cam_actuator_unlock(struct cam_actuator_ctrl_t *a_ctrl)
{
	int rc = 0;

	struct cam_actuator_soc_private *soc_private;
	struct cam_sensor_power_ctrl_t *power_info;

	if (!a_ctrl) {
		CAM_ERR(CAM_ACTUATOR, "failed: a_ctrl %pK", a_ctrl);
		return -EINVAL;
	}

	soc_private =
		(struct cam_actuator_soc_private *)a_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	if (!power_info) {
		CAM_ERR(CAM_ACTUATOR, "failed: power_info %pK", power_info);
		return -EINVAL;
	}

	mutex_lock(&(a_ctrl->actuator_mutex));
	if (a_ctrl->camera_actuator_shake_detect_enable && a_ctrl->cam_act_last_state == CAM_ACTUATOR_LOCK)
	{
		rc = oplus_cam_actuator_power_down(a_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "Actuator Power down failed");
		} else {
			a_ctrl->cam_act_last_state = CAM_ACTUATOR_INIT;
			kfree(power_info->power_setting);
			kfree(power_info->power_down_setting);
			power_info->power_setting = NULL;
			power_info->power_down_setting = NULL;
			power_info->power_setting_size = 0;
			power_info->power_down_setting_size = 0;
		}
	}
	else
	{
		CAM_ERR(CAM_ACTUATOR, "do not support SDS(shake detect service)");
	}

	mutex_unlock(&(a_ctrl->actuator_mutex));

	return rc;
}

int32_t oplus_cam_actuator_power_up(struct cam_actuator_ctrl_t *a_ctrl)
{
	int rc = 0;
	struct cam_hw_soc_info  *soc_info =
		&a_ctrl->soc_info;
	struct cam_actuator_soc_private  *soc_private;
	struct cam_sensor_power_ctrl_t *power_info;

	soc_private =
		(struct cam_actuator_soc_private *)a_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	if ((power_info->power_setting == NULL) &&
		(power_info->power_down_setting == NULL)) {
		CAM_INFO(CAM_ACTUATOR,
			"Using default power settings");
		rc = oplus_cam_actuator_construct_default_power_setting(a_ctrl, power_info);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Construct default actuator power setting failed.");
			return rc;
		}
	}

	/* Parse and fill vreg params for power up settings */
	rc = msm_camera_fill_vreg_params(
		&a_ctrl->soc_info,
		power_info->power_setting,
		power_info->power_setting_size);
	if (rc) {
		CAM_ERR(CAM_ACTUATOR,
			"failed to fill vreg params for power up rc:%d", rc);
		return rc;
	}

	/* Parse and fill vreg params for power down settings*/
	rc = msm_camera_fill_vreg_params(
		&a_ctrl->soc_info,
		power_info->power_down_setting,
		power_info->power_down_setting_size);
	if (rc) {
		CAM_ERR(CAM_ACTUATOR,
			"failed to fill vreg params power down rc:%d", rc);
		return rc;
	}

	power_info->dev = soc_info->dev;

	rc = cam_sensor_core_power_up(power_info, soc_info, NULL, &(a_ctrl->io_master_info));
	if (rc) {
		CAM_ERR(CAM_ACTUATOR,
			"failed in actuator power up rc %d", rc);
		return rc;
	} else {
		CAM_INFO(CAM_ACTUATOR,
			"actuator Power Up success for cci_device:%d, cci_i2c_master:%d, sid:0x%x",
			a_ctrl->io_master_info.cci_client->cci_device,
			a_ctrl->io_master_info.cci_client->cci_i2c_master,
			a_ctrl->io_master_info.cci_client->sid);
	}

	rc = camera_io_init(&a_ctrl->io_master_info);
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "cci init failed: rc: %d", rc);
		goto cci_failure;
	}

	return rc;
cci_failure:
	if (cam_sensor_util_power_down(power_info, soc_info, &(a_ctrl->io_master_info))){
		CAM_ERR(CAM_ACTUATOR, "Power down failure");
	} else {
		CAM_INFO(CAM_ACTUATOR,
			   "actuator Power Down success for cci_device:%d, cci_i2c_master:%d, sid:0x%x",
			   a_ctrl->io_master_info.cci_client->cci_device,
			   a_ctrl->io_master_info.cci_client->cci_i2c_master,
			   a_ctrl->io_master_info.cci_client->sid);
	}

	return rc;
}

int32_t oplus_cam_actuator_power_down(struct cam_actuator_ctrl_t *a_ctrl)
{
	int32_t rc = 0;
	struct cam_sensor_power_ctrl_t *power_info;
	struct cam_hw_soc_info *soc_info = &a_ctrl->soc_info;
	struct cam_actuator_soc_private  *soc_private;

	if (!a_ctrl) {
		CAM_ERR(CAM_ACTUATOR, "failed: a_ctrl %pK", a_ctrl);
		return -EINVAL;
	}

	soc_private =
		(struct cam_actuator_soc_private *)a_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;
	soc_info = &a_ctrl->soc_info;

	if (!power_info) {
		CAM_ERR(CAM_ACTUATOR, "failed: power_info %pK", power_info);
		return -EINVAL;
	}

	if ((power_info->power_setting == NULL) &&
		(power_info->power_down_setting == NULL)) {
		CAM_INFO(CAM_ACTUATOR,
			"Using default power settings");
		rc = oplus_cam_actuator_construct_default_power_setting(a_ctrl, power_info);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Construct default actuator power setting failed.");
			return rc;
		}
		/* Parse and fill vreg params for power up settings */
		rc = msm_camera_fill_vreg_params(
			&a_ctrl->soc_info,
			power_info->power_setting,
			power_info->power_setting_size);
		if (rc) {
			CAM_ERR(CAM_ACTUATOR,
				"failed to fill vreg params for power up rc:%d", rc);
			return rc;
		}

		/* Parse and fill vreg params for power down settings*/
		rc = msm_camera_fill_vreg_params(
			&a_ctrl->soc_info,
			power_info->power_down_setting,
			power_info->power_down_setting_size);
		if (rc) {
			CAM_ERR(CAM_ACTUATOR,
				"failed to fill vreg params power down rc:%d", rc);
		}

	}


	rc = cam_sensor_util_power_down(power_info, soc_info, &(a_ctrl->io_master_info));
	if (rc) {
		CAM_ERR(CAM_ACTUATOR, "power down the core is failed:%d", rc);
		return rc;
	} else {
		CAM_INFO(CAM_ACTUATOR,
				"actuator Power Down success for cci_device:%d, cci_i2c_master:%d, sid:0x%x",
				a_ctrl->io_master_info.cci_client->cci_device,
				a_ctrl->io_master_info.cci_client->cci_i2c_master,
				a_ctrl->io_master_info.cci_client->sid);
	}

	camera_io_release(&a_ctrl->io_master_info);

	return rc;
}

int oplus_cam_actuator_ram_write(struct cam_actuator_ctrl_t *a_ctrl, uint32_t addr, uint32_t data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;

	struct cam_sensor_i2c_reg_array i2c_write_setting = {
		.reg_addr = addr,
		.reg_data = data,
		.delay = 5,
		.data_mask = 0x00,
	};
	struct cam_sensor_i2c_reg_setting i2c_write = {
		.reg_setting = &i2c_write_setting,
		.size = 1,
		.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
		.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
		.delay = 0x00,
	};

	if (a_ctrl == NULL)
	{
		CAM_ERR(CAM_ACTUATOR, "Invalid Args");
		return -EINVAL;
	}

	for(i = 0; i < retry; i++) {
		rc = camera_io_dev_write(&(a_ctrl->io_master_info), &i2c_write);
		if (rc < 0)
		{
			CAM_ERR(CAM_ACTUATOR, "actuator write 0x%x failed, retry:%d", addr, i+1);
		}
		else
		{
			CAM_DBG(CAM_ACTUATOR, "actuator write success 0x%x, data:=0x%x", addr, data);
			return rc;
		}
	}
	return rc;
}

int oplus_cam_actuator_ram_read(struct cam_actuator_ctrl_t *a_ctrl, uint32_t addr, uint32_t* data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;

	if (a_ctrl == NULL)
	{
		CAM_ERR(CAM_ACTUATOR, "Invalid Args");
		return -EINVAL;
	}

	for(i = 0; i < retry; i++)
	{
		rc = camera_io_dev_read(&(a_ctrl->io_master_info), (uint32_t)addr, (uint32_t *)data,
		                        CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE, false);
		if (rc < 0)
		{
			CAM_ERR(CAM_ACTUATOR, "read 0x%x failed, retry:%d", addr, i+1);
		}
		else
		{
			return rc;
		}
	}
	return rc;
}

int oplus_cam_actuator_update_pid_ak7316(struct cam_actuator_ctrl_t *a_ctrl)
{
	struct device_node *of_node = NULL;
	struct cam_hw_soc_info *soc_info = &a_ctrl->soc_info;
	bool is_update_pid = false;
	int32_t rc = 0, i = 0, pid_register_size = 0;
	uint8_t* pid_register = NULL;
	uint32_t ic_id = 0, pid_version = 0;
	uint32_t register_value = 0, data_check_value = 0;

	if (a_ctrl == NULL) {
		CAM_ERR(CAM_ACTUATOR, "oplus_cam_actuator_update_pid Invalid Args");
		return -EINVAL;
	}

	a_ctrl->io_master_info.cci_client->cci_i2c_master = a_ctrl->cci_i2c_master;
	a_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;
	a_ctrl->io_master_info.cci_client->sid = (AK7316_SLAVE_ADDR >> 1);
	a_ctrl->io_master_info.cci_client->retries = 0;
	a_ctrl->io_master_info.cci_client->id_map = 0;

	rc = oplus_cam_actuator_power_up(a_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "Failed for Actuator Power up failed: %d", rc);
		return rc;
	}
	msleep(10);
	rc = oplus_cam_actuator_ram_write(a_ctrl, AK7316_STATE_ADDR, AK7316_STANDBY_STATE);
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "Failed write standby state: %d", rc);
		goto error_hand;
	}
	msleep(5);

	rc = oplus_cam_actuator_ram_read(a_ctrl, AK7316_ID_ADDR, &ic_id);
	if (rc < 0 || ic_id != AK7316_ID_DATA) {
		CAM_INFO(CAM_ACTUATOR, "not ak7316, no need update pid, id:0x%x", ic_id);
		goto error_hand;
	}

	of_node = soc_info->dev->of_node;
	rc = of_property_read_u32(of_node, "pid_version", &pid_version);
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "Failed to get pid version: %d, exit", rc);
		goto error_hand;
	}

	rc = oplus_cam_actuator_ram_read(a_ctrl, AK7316_PID_VER_ADDR, &register_value);
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "Failed to read ak7316 pid version: %d, exit", rc);
		goto error_hand;
	}

	if(register_value != pid_version) {
		is_update_pid = true;
		CAM_INFO(CAM_ACTUATOR, "before updating, pid ver:0x%x", register_value);
	}

	if(is_update_pid) {
		//1.get pid register
		pid_register_size = of_property_count_u8_elems(of_node, "pid_register");
		if (pid_register_size < 1) {
			CAM_ERR(CAM_OIS, "pid_register_size < 1, is %d, return", pid_register_size);
			goto error_hand;
		}

		pid_register = (uint8_t*)kzalloc(pid_register_size, GFP_KERNEL);
		if (pid_register == NULL) {
			CAM_ERR(CAM_OIS, "allocate pid register buffer failed");
			goto error_hand;
		}

		rc = of_property_read_u8_array(of_node, "pid_register", pid_register, pid_register_size);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Invalid fw_data params");
			goto error_hand;
		}

		//2.change to setting mode
		rc = oplus_cam_actuator_ram_write(a_ctrl, AK7316_WRITE_CONTROL_ADDR, 0x3B);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "write setting mode failed");
			goto error_hand;
		}

		//3.write pid register
		for (i = 0; i < pid_register_size; i = i + 2)
		{
			rc = oplus_cam_actuator_ram_write(a_ctrl, pid_register[i], pid_register[i + 1]);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "write pid setting failed");
				goto error_hand;
			}
		}

		//4.write store instruction
		rc = oplus_cam_actuator_ram_write(a_ctrl, AK7316_STORE_ADDR, 0x01);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "store pid setting failed");
			goto error_hand;
		}
		msleep(110);

		rc = oplus_cam_actuator_ram_write(a_ctrl, AK7316_STORE_ADDR, 0x02);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "store pid setting failed");
			goto error_hand;
		}
		msleep(190);

		rc = oplus_cam_actuator_ram_write(a_ctrl, AK7316_STORE_ADDR, 0x04);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "store pid setting failed");
			goto error_hand;
		}
		msleep(160);

		rc = oplus_cam_actuator_ram_write(a_ctrl, AK7316_STORE_ADDR, 0x08);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "store pid setting failed");
			goto error_hand;
		}
		msleep(170);

		rc = oplus_cam_actuator_ram_read(a_ctrl, AK7316_DATA_CHECK_ADDR, &data_check_value);
		CAM_ERR(CAM_OIS, "check pid setting 0x%x", data_check_value);
		if (rc < 0 || (data_check_value & AK7316_DATA_CHECK_BIT) != 0) {
			CAM_ERR(CAM_OIS, "check pid setting failed");
			goto error_hand;
		}

		//4.go to release mode
		rc = oplus_cam_actuator_ram_write(a_ctrl, AK7316_WRITE_CONTROL_ADDR, 0x00);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "write release mode setting failed");
			goto error_hand;
		}

		rc = oplus_cam_actuator_ram_read(a_ctrl, AK7316_PID_VER_ADDR, &register_value);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "Failed to read ak7316 pid version, rc: %d", rc);
			goto error_hand;
		}
		CAM_INFO(CAM_ACTUATOR, "after updating, pid ver:0x%x", register_value);
		if (register_value != pid_version) {
			CAM_ERR(CAM_ACTUATOR, "Failed to update ak7316 pid, version: %d", register_value);
			goto error_hand;
		} else {
			CAM_INFO(CAM_ACTUATOR, "update ak7316 pid successfully");
		}
	} else {
		CAM_INFO(CAM_ACTUATOR, "no need to update ak7316 pid");
		rc = oplus_cam_actuator_power_down(a_ctrl);
		return rc;
	}

	kfree(pid_register);
	pid_register = NULL;
	rc = oplus_cam_actuator_power_down(a_ctrl);
	return rc;

error_hand:
	kfree(pid_register);
	pid_register = NULL;
	CAM_ERR(CAM_ACTUATOR, "update pid failed, rc: %d", rc);
	rc = oplus_cam_actuator_power_down(a_ctrl);
	return rc;
}

int oplus_cam_actuator_update_pid(void *arg)
{
	int rc = 0;
	struct cam_actuator_ctrl_t *a_ctrl = (struct cam_actuator_ctrl_t *)arg;

	if (a_ctrl == NULL) {
		CAM_ERR(CAM_ACTUATOR, "oplus_cam_actuator_update_pid Invalid Args");
		return -EINVAL;
	}

	if (strstr(a_ctrl->actuator_name, "ak7316")) {
		rc = oplus_cam_actuator_update_pid_ak7316(a_ctrl);
	} else {
		CAM_ERR(CAM_ACTUATOR, "no actuator pid need update");
	}

	return rc;
}

int oplus_cam_actuator_ram_write_extend(struct cam_actuator_ctrl_t *a_ctrl,
	uint32_t addr, uint32_t data,unsigned short mdelay,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type)
{
	int32_t rc = 0;
	int retry = 3;
	int i;

	struct cam_sensor_i2c_reg_array i2c_write_setting = {
		.reg_addr = addr,
		.reg_data = data,
		.delay = 5,
		.data_mask = mdelay,
	};
	struct cam_sensor_i2c_reg_setting i2c_write = {
		.reg_setting = &i2c_write_setting,
		.size = 1,
		.addr_type = addr_type,
		.data_type = data_type,
		.delay = 0x00,
	};

	if (a_ctrl == NULL)
	{
		CAM_ERR(CAM_ACTUATOR, "Invalid Args");
		return -EINVAL;
	}

	if ((addr_type <= CAMERA_SENSOR_I2C_TYPE_INVALID) || (addr_type >= CAMERA_SENSOR_I2C_TYPE_MAX)
		|| (data_type <= CAMERA_SENSOR_I2C_TYPE_INVALID) || (data_type >= CAMERA_SENSOR_I2C_TYPE_MAX))
	{
		CAM_ERR(CAM_ACTUATOR, "addr_type: %d, data_type: %d, is not invalid", addr_type, data_type);
		return -EINVAL;
	}

	for(i = 0; i < retry; i++) {
		rc = camera_io_dev_write(&(a_ctrl->io_master_info), &i2c_write);
		if (rc < 0)
		{
			CAM_ERR(CAM_ACTUATOR, "actuator write 0x%x failed, retry:%d", addr, i+1);
		}
		else
		{
			CAM_DBG(CAM_ACTUATOR, "actuator write success 0x%x, data:=0x%x", addr, data);
			return rc;
		}
	}
	return rc;
}

int oplus_cam_actuator_ram_read_extend(struct cam_actuator_ctrl_t *a_ctrl,
	uint32_t addr, uint32_t *data,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type)
{
	int32_t rc = 0;
	int retry = 3;
	int i;

	if (a_ctrl == NULL)
	{
		CAM_ERR(CAM_ACTUATOR, "Invalid Args");
		return -EINVAL;
	}

	if ((addr_type <= CAMERA_SENSOR_I2C_TYPE_INVALID) || (addr_type >= CAMERA_SENSOR_I2C_TYPE_MAX)
		|| (data_type <= CAMERA_SENSOR_I2C_TYPE_INVALID) || (data_type >= CAMERA_SENSOR_I2C_TYPE_MAX))
	{
		CAM_ERR(CAM_ACTUATOR, "addr_type: %d, data_type: %d, is not invalid", addr_type, data_type);
		return -EINVAL;
	}

	for(i = 0; i < retry; i++)
	{
		rc = camera_io_dev_read(&(a_ctrl->io_master_info), (uint32_t)addr, (uint32_t *)data,
			addr_type, data_type, false);
		if (rc < 0)
		{
			CAM_ERR(CAM_ACTUATOR, "read 0x%x failed, retry:%d", addr, i+1);
		}
		else
		{
			return rc;
		}
	}
	return rc;
}
