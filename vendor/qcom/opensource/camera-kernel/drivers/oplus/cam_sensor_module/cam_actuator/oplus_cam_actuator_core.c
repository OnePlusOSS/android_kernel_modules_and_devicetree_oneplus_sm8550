#include <linux/module.h>
#include "cam_sensor_cmn_header.h"
#include "cam_actuator_core.h"
#include "cam_sensor_util.h"
#include "cam_trace.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"

#include "oplus_cam_actuator_core.h"
#include "oplus_cam_actuator_dev.h"

void cam_actuator_poll_setting_update(struct cam_actuator_ctrl_t *a_ctrl) {

        struct i2c_settings_list *i2c_list = NULL;

        a_ctrl->is_actuator_ready = true;
        memset(&(a_ctrl->poll_register), 0, sizeof(struct cam_sensor_i2c_reg_array));
        list_for_each_entry(i2c_list,
                &(a_ctrl->i2c_data.init_settings.list_head), list) {
                if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
                        a_ctrl->poll_register.reg_addr = i2c_list->i2c_settings.reg_setting[0].reg_addr;
                        a_ctrl->poll_register.reg_data = i2c_list->i2c_settings.reg_setting[0].reg_data;
                        a_ctrl->poll_register.data_mask = i2c_list->i2c_settings.reg_setting[0].data_mask;
                        a_ctrl->poll_register.delay = 100; //i2c_list->i2c_settings.reg_setting[0].delay; // The max delay should be 100
                        a_ctrl->addr_type = i2c_list->i2c_settings.addr_type;
                        a_ctrl->data_type = i2c_list->i2c_settings.data_type;
                }
        }
}

void cam_actuator_poll_setting_apply(struct cam_actuator_ctrl_t *a_ctrl) {
        int ret = 0;
        if (!a_ctrl->is_actuator_ready) {
                if (a_ctrl->poll_register.reg_addr || a_ctrl->poll_register.reg_data) {
                        ret = camera_io_dev_poll(
                                &(a_ctrl->io_master_info),
                                a_ctrl->poll_register.reg_addr,
                                a_ctrl->poll_register.reg_data,
                                a_ctrl->poll_register.data_mask,
                                a_ctrl->addr_type,
                                a_ctrl->data_type,
                                a_ctrl->poll_register.delay);
                        if (ret < 0) {
                                CAM_ERR(CAM_ACTUATOR,"i2c poll apply setting Fail: %d, is_actuator_ready %d", ret, a_ctrl->is_actuator_ready);
                        } else {
                                CAM_DBG(CAM_ACTUATOR,"is_actuator_ready %d, ret %d", a_ctrl->is_actuator_ready, ret);
                        }
                        a_ctrl->is_actuator_ready = true; //Just poll one time
                }
        }
}

int32_t oplus_cam_actuator_construct_default_power_setting(
	struct cam_sensor_power_ctrl_t *power_info)
{
	int rc = 0;

	power_info->power_setting_size = 2;
	power_info->power_setting =
		kzalloc(sizeof(struct cam_sensor_power_setting) * 2,
			GFP_KERNEL);
	if (!power_info->power_setting)
		return -ENOMEM;

	power_info->power_setting[0].seq_type = SENSOR_VIO;
	power_info->power_setting[0].seq_val = CAM_VIO;
	power_info->power_setting[0].config_val = 1;
	power_info->power_setting[0].delay = 2;

	power_info->power_setting[1].seq_type = SENSOR_VAF;
	power_info->power_setting[1].seq_val = CAM_VAF;
	power_info->power_setting[1].config_val = 1;
	power_info->power_setting[1].delay = 0;

	power_info->power_down_setting_size = 2;
	power_info->power_down_setting =
		kzalloc(sizeof(struct cam_sensor_power_setting) * 2,
			GFP_KERNEL);
	if (!power_info->power_down_setting) {
		rc = -ENOMEM;
		goto free_power_settings;
	}

	power_info->power_down_setting[0].seq_type = SENSOR_VIO;
	power_info->power_down_setting[0].seq_val = CAM_VIO;
	power_info->power_down_setting[0].config_val = 0;

	power_info->power_down_setting[1].seq_type = SENSOR_VAF;
	power_info->power_down_setting[1].seq_val = CAM_VAF;
	power_info->power_down_setting[1].config_val = 0;

	return rc;

free_power_settings:
	kfree(power_info->power_setting);
	power_info->power_setting = NULL;
	power_info->power_setting_size = 0;
	return rc;
}

bool  oplus_cam_actuator_sds_match_id(struct cam_actuator_ctrl_t *a_ctrl)
{
	if ((a_ctrl->ssd_actuator_cci_i2c_master_num == a_ctrl->cci_i2c_master) && (a_ctrl->ssd_actuator_cci_num == a_ctrl->cci_num))
	{
		return true;
	}
	else
	{
		return false;
	}
}

void oplus_cam_actuator_sds_adjust_from_acquire(struct cam_actuator_ctrl_t *a_ctrl)
{
	int rc = -1;
	if ((a_ctrl->camera_actuator_shake_detect_on)
			&& oplus_cam_actuator_sds_match_id(a_ctrl)
			&& (CAM_ACTUATOR_LOCK == a_ctrl->cam_act_state))
	{
		//power_off
		a_ctrl->io_master_info.cci_client->cci_device = CCI_DEVICE_1;
		a_ctrl->io_master_info.cci_client->cci_i2c_master = MASTER_0;
		a_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;
		a_ctrl->io_master_info.cci_client->sid = (0x18 >> 1);
		a_ctrl->io_master_info.cci_client->retries = 0;
		a_ctrl->io_master_info.cci_client->id_map = 0;

		rc = oplus_cam_actuator_power_down(a_ctrl);
		if (rc < 0)
		{
			CAM_ERR(CAM_ACTUATOR, "SDS Failed for Actuator Power down failed: %d", rc);
		}
		else
		{
			msleep(2);
			CAM_DBG(CAM_ACTUATOR, "SDS Actuator Power down success");
		}

		//set init
		a_ctrl->cam_act_state = CAM_ACTUATOR_INIT;
	}
}

void oplus_cam_actuator_sds_adjust_from_start(struct cam_actuator_ctrl_t *a_ctrl)
{
	int rc = -1;
	if ((a_ctrl->camera_actuator_shake_detect_on)
			&& oplus_cam_actuator_sds_match_id(a_ctrl)
			&& (CAM_ACTUATOR_LOCK == a_ctrl->cam_act_state))
	{
		//standby
		a_ctrl->io_master_info.cci_client->cci_device = CCI_DEVICE_1;
		a_ctrl->io_master_info.cci_client->cci_i2c_master = MASTER_0;
		a_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;
		a_ctrl->io_master_info.cci_client->sid = (0x18 >> 1);
		a_ctrl->io_master_info.cci_client->retries = 0;
		a_ctrl->io_master_info.cci_client->id_map = 0;

		rc = oplus_cam_actuator_stand_by(a_ctrl);
		if (rc < 0)
		{
			CAM_ERR(CAM_ACTUATOR, "SDS Failed for Actuator Power down failed: %d", rc);
		}
		else
		{
			CAM_DBG(CAM_ACTUATOR, "SDS Actuator stand by success");
		}

		//set config
		a_ctrl->cam_act_state = CAM_ACTUATOR_CONFIG;
	}
}