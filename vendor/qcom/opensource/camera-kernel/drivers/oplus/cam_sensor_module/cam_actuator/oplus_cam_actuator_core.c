#include <linux/module.h>
#include "cam_sensor_cmn_header.h"
#include "cam_actuator_core.h"
#include "cam_sensor_util.h"
#include "cam_trace.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"

#include "oplus_cam_actuator_core.h"
#include "oplus_cam_actuator_dev.h"

#define ACTUATOR_REGISTER_SIZE 10
#define AK7316_DAC_ADDR 0x84
uint32_t AK7316_PARKLENS_DOWN[ACTUATOR_REGISTER_SIZE][2] = {
	{0x00, 0xC8},
	{0x00, 0xD0},
	{0x00, 0xDC},
	{0x00, 0xF0},
	{0x00, 0xFE},
	{0xff, 0xff},
};

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
	power_info->power_setting[1].delay = 10;

	power_info->power_down_setting_size = 2;
	power_info->power_down_setting =
		kzalloc(sizeof(struct cam_sensor_power_setting) * 2,
			GFP_KERNEL);
	if (!power_info->power_down_setting) {
		rc = -ENOMEM;
		goto free_power_settings;
	}

	power_info->power_down_setting[0].seq_type = SENSOR_VAF;
	power_info->power_down_setting[0].seq_val = CAM_VIO;
	power_info->power_down_setting[0].config_val = 0;
	power_info->power_down_setting[0].delay = 1;

	power_info->power_down_setting[1].seq_type = SENSOR_VIO;
	power_info->power_down_setting[1].seq_val = CAM_VAF;
	power_info->power_down_setting[1].config_val = 0;
	power_info->power_down_setting[0].delay = 0;

	return rc;

free_power_settings:
	kfree(power_info->power_setting);
	power_info->power_setting = NULL;
	power_info->power_setting_size = 0;
	return rc;
}

int actuator_power_down_thread(void *arg)
{
	int rc = 0;
	int i;
	uint32_t read_val = 0;
	struct cam_actuator_ctrl_t *a_ctrl = (struct cam_actuator_ctrl_t *)arg;
	struct cam_actuator_soc_private *soc_private;
	struct cam_sensor_power_ctrl_t *power_info;

	msleep(5);
	if (!a_ctrl) {
		CAM_ERR(CAM_ACTUATOR, "failed: a_ctrl %pK", a_ctrl);
		return -EINVAL;
	}

	soc_private = (struct cam_actuator_soc_private *)a_ctrl->soc_info.soc_private;
	if (!soc_private) {
		CAM_ERR(CAM_ACTUATOR, "failed:soc_private %pK", soc_private);
		return -EINVAL;
	}
	else{
		power_info  = &soc_private->power_info;
		if (!power_info){
			CAM_ERR(CAM_ACTUATOR, "failed: power_info %pK", a_ctrl, power_info);
			return -EINVAL;
		}
	}

	down(&a_ctrl->actuator_sem);

	CAM_INFO(CAM_ACTUATOR,"actuator_power_down_thread start ");
	camera_io_dev_read(&(a_ctrl->io_master_info), AK7316_DAC_ADDR, &read_val,
			CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_WORD, false);

	for (i = 0; i < ACTUATOR_REGISTER_SIZE; i++) {
		if ((AK7316_PARKLENS_DOWN[i][0] != 0xff) && (AK7316_PARKLENS_DOWN[i][1] != 0xff)) {
			if(read_val <= (AK7316_PARKLENS_DOWN[i][1] << 8)) {
				CAM_INFO(CAM_ACTUATOR,"read_val: 0x%x, Ak7316_PARKLENS_DOWN: 0x%x", read_val, AK7316_PARKLENS_DOWN[i][1]);
				break;
			}
		} else {
			break;
		}
	}

	for (; i < ACTUATOR_REGISTER_SIZE; i++)
	{
		if ( (AK7316_PARKLENS_DOWN[i][0] != 0xff) && (AK7316_PARKLENS_DOWN[i][1] != 0xff) )
		{
			rc = oplus_cam_actuator_ram_write(a_ctrl, (uint32_t)AK7316_PARKLENS_DOWN[i][0], (uint32_t)AK7316_PARKLENS_DOWN[i][1]);
			if(rc < 0)
			{
				CAM_ERR(CAM_ACTUATOR,"write failed ret : %d", rc);
				goto free_power_settings;
			} else {
				CAM_INFO(CAM_ACTUATOR,"write register 0x%x: 0x%x sucess", AK7316_PARKLENS_DOWN[i][0], AK7316_PARKLENS_DOWN[i][1]);
			}
			msleep(5);
		}
		else
		{
			CAM_INFO(CAM_ACTUATOR,"set parklens success ");
			break;
		}
	}


free_power_settings:
	rc = oplus_cam_actuator_power_down(a_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "Actuator Power down failed");
	} else {
		kfree(power_info->power_setting);
		kfree(power_info->power_down_setting);
		power_info->power_setting = NULL;
		power_info->power_down_setting = NULL;
		power_info->power_setting_size = 0;
		power_info->power_down_setting_size = 0;
	}
	CAM_INFO(CAM_ACTUATOR, "actuator_power_down_thread exit");
	up(&a_ctrl->actuator_sem);
	return rc;
}

void oplus_cam_actuator_parklens_power_down(struct cam_actuator_ctrl_t *a_ctrl)
{
	a_ctrl->actuator_parklens_thread = kthread_run(actuator_power_down_thread, a_ctrl, "actuator_power_down_thread");
}
