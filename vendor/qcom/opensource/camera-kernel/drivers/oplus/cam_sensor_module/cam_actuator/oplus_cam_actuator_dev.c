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

void oplus_cam_actuator_sds_on(struct cam_actuator_ctrl_t *a_ctrl)
{
	mutex_lock(&(a_ctrl->actuator_mutex));
	a_ctrl->camera_actuator_shake_detect_on = true;
	a_ctrl->ssd_actuator_cci_i2c_master_num = 0;
	a_ctrl->ssd_actuator_cci_num = 1;
	mutex_unlock(&(a_ctrl->actuator_mutex));
}

int32_t oplus_cam_actuator_lock_on(struct cam_actuator_ctrl_t *a_ctrl)
{
	uint32_t data = 0;
	int rc = -1;
	mutex_lock(&(a_ctrl->actuator_mutex));
	if (a_ctrl->camera_actuator_shake_detect_on)
	{
		if ((CAM_ACTUATOR_START == a_ctrl->cam_act_state) || (CAM_ACTUATOR_ACQUIRE == a_ctrl->cam_act_state))
		{
			CAM_ERR(CAM_ACTUATOR, "SDS Actuator active, can not lock");
			mutex_unlock(&(a_ctrl->actuator_mutex));
			return -1;
		}

		CAM_DBG(CAM_ACTUATOR, "SDS Actuator lock start");
		a_ctrl->io_master_info.cci_client->cci_device = CCI_DEVICE_1;
		a_ctrl->io_master_info.cci_client->cci_i2c_master = MASTER_0;
		a_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;
		a_ctrl->io_master_info.cci_client->sid = (0x18 >> 1);
		a_ctrl->io_master_info.cci_client->retries = 0;
		a_ctrl->io_master_info.cci_client->id_map = 0;

		rc = oplus_cam_actuator_power_up(a_ctrl);
		if (rc < 0)
		{
			CAM_ERR(CAM_ACTUATOR, "Failed for Actuator Power up failed: %d", rc);
		}

		msleep(8);
		oplus_cam_actuator_ram_write(a_ctrl,0x02,0x40);
		rc = oplus_cam_actuator_ram_read(a_ctrl,0x09,&data);
		if(data != 0x80)
		{
			rc = oplus_cam_actuator_ram_write(a_ctrl,0xAE,0x3B);
			rc = oplus_cam_actuator_ram_write(a_ctrl,0x09,0x80);
			rc = oplus_cam_actuator_ram_write(a_ctrl,0x03,0x01);
			msleep(110);
		}

		/*rc = oplus_cam_actuator_ram_read(a_ctrl,0x56,&data1);
		CAM_DBG(CAM_ACTUATOR, "read 0x56 state 0x%x rc = %d", data1,rc);
		rc = oplus_cam_actuator_ram_read(a_ctrl,0x57,&data2);
		CAM_DBG(CAM_ACTUATOR, "read 0x57 state 0x%x rc = %d", data2,rc);
		rc = oplus_cam_actuator_ram_read(a_ctrl,0x58,&data3);
		CAM_DBG(CAM_ACTUATOR, "read 0x58 state 0x%x rc = %d", data3,rc);

		rc = oplus_cam_actuator_ram_write(a_ctrl,0xAE,0x3B);
		rc = oplus_cam_actuator_ram_write(a_ctrl,0x57,0xA4);
		msleep(10);
		rc = oplus_cam_actuator_ram_read(a_ctrl,0x57,&data4);
		CAM_DBG(CAM_ACTUATOR, "read 0x57 state 0x%x rc = %d", data4,rc);*/
		oplus_cam_actuator_ram_write(a_ctrl,0x02,0x20);

		a_ctrl->cam_act_last_state = a_ctrl->cam_act_state;
		a_ctrl->cam_act_state = CAM_ACTUATOR_LOCK;
	}
	else
	{
		CAM_ERR(CAM_ACTUATOR, "do not support SDS(shake detect service)");
	}
	mutex_unlock(&(a_ctrl->actuator_mutex));

	return rc;
}

int32_t oplus_cam_actuator_lock_off_by_state(struct cam_actuator_ctrl_t *a_ctrl, uint32_t state)
{
	int rc = -1;

	if (CAM_ACTUATOR_INIT == state)
	{
		//power_off
		rc = oplus_cam_actuator_power_down(a_ctrl);
		if (rc < 0)
		{
			CAM_ERR(CAM_ACTUATOR, "SDS Failed for Actuator Power down failed: %d", rc);
		}
		else
		{
			msleep(2);
			CAM_DBG(CAM_ACTUATOR, "SDS ioctl actuator power off success");
		}
	}
	else if (CAM_ACTUATOR_CONFIG == state)
	{
		//standby
		rc = oplus_cam_actuator_stand_by(a_ctrl);
		if (rc < 0)
		{
			CAM_ERR(CAM_ACTUATOR, "SDS Failed for Actuator stand by failed: %d", rc);
		}
		else
		{
			CAM_DBG(CAM_ACTUATOR, "SDS ioctl actuator stand by success");
		}
	}

	return rc;
}

int32_t oplus_cam_actuator_lock_off(struct cam_actuator_ctrl_t *a_ctrl)
{
	int rc = 0;
	mutex_lock(&(a_ctrl->actuator_mutex));
	if (a_ctrl->camera_actuator_shake_detect_on)
	{
		CAM_DBG(CAM_ACTUATOR, "SDS Actuator lock off start");
		a_ctrl->io_master_info.cci_client->cci_device = CCI_DEVICE_1;
		a_ctrl->io_master_info.cci_client->cci_i2c_master = MASTER_0;
		a_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;
		a_ctrl->io_master_info.cci_client->sid = (0x18 >> 1);
		a_ctrl->io_master_info.cci_client->retries = 0;
		a_ctrl->io_master_info.cci_client->id_map = 0;

		if (CAM_ACTUATOR_LOCK == a_ctrl->cam_act_state)
		{
			rc = oplus_cam_actuator_lock_off_by_state(a_ctrl, a_ctrl->cam_act_last_state);
			if (rc < 0)
			{
				CAM_ERR(CAM_ACTUATOR, "SDS cam_act_state: %d, cam_act_last_state: %d, not recongnize, do nothing",
						a_ctrl->cam_act_state, a_ctrl->cam_act_last_state);
			}
			else
			{
				a_ctrl->cam_act_state = a_ctrl->cam_act_last_state;
				CAM_DBG(CAM_ACTUATOR, "SDS actuator from lock to lock off success, last state: %d", a_ctrl->cam_act_last_state);
			}
		}
		else if ((CAM_ACTUATOR_INIT == a_ctrl->cam_act_state) || (CAM_ACTUATOR_CONFIG == a_ctrl->cam_act_state))
		{
			oplus_cam_actuator_lock_off_by_state(a_ctrl, a_ctrl->cam_act_state);
		}
		else
		{
			//do nothing
			CAM_WARN(CAM_ACTUATOR, "SDS a_ctrl->cam_act_state: %d, not recongnize, do nothing", a_ctrl->cam_act_state);
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
		CAM_ERR(CAM_ACTUATOR,
			"Using default power settings");
		rc = oplus_cam_actuator_construct_default_power_setting(power_info);
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

	rc = cam_sensor_core_power_up(power_info, soc_info, NULL);
	if (rc) {
		CAM_ERR(CAM_ACTUATOR,
			"failed in actuator power up rc %d", rc);
		return rc;
	}

	rc = camera_io_init(&a_ctrl->io_master_info);
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "cci init failed: rc: %d", rc);
		goto cci_failure;
	}

	return rc;
cci_failure:
	if (cam_sensor_util_power_down(power_info, soc_info))
		CAM_ERR(CAM_ACTUATOR, "Power down failure");

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
	rc = cam_sensor_util_power_down(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_ACTUATOR, "power down the core is failed:%d", rc);
		return rc;
	}

	camera_io_release(&a_ctrl->io_master_info);

	return rc;
}

int32_t oplus_cam_actuator_stand_by(struct cam_actuator_ctrl_t *a_ctrl)
{
	int32_t rc = 0;
	uint32_t data = 0;

	if (!a_ctrl) {
		CAM_ERR(CAM_ACTUATOR, "failed: a_ctrl %pK", a_ctrl);
		return -EINVAL;
	}

	rc = oplus_cam_actuator_power_up(a_ctrl);
	if (rc < 0)
	{
		CAM_ERR(CAM_ACTUATOR, "Failed for Actuator Power up failed: %d", rc);
		return rc;
	}

	rc = oplus_cam_actuator_ram_read(a_ctrl,0x09,&data);
	if(data & 0x80)
	{
		rc = oplus_cam_actuator_ram_write(a_ctrl,0x02,0x40);
		msleep(5);
		rc = oplus_cam_actuator_ram_write(a_ctrl,0x02,0x40);
		msleep(5);
	}
	else
	{
		rc = oplus_cam_actuator_ram_write(a_ctrl,0x02,0x40);
		msleep(5);
	}

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
