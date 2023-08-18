// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/firmware.h>

#include "cam_sensor_cmn_header.h"
#include "cam_ois_core.h"
#include "cam_ois_soc.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "cam_res_mgr_api.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"
#ifdef OPLUS_FEATURE_CAMERA_COMMON
#include "fw_download_interface.h"
extern bool chip_version_old;
#endif

#ifdef ENABLE_OIS_DELAY_POWER_DOWN

int ois_power_down_thread(void *arg)
{
	int rc = 0;
	int i;
	struct cam_ois_ctrl_t *o_ctrl = (struct cam_ois_ctrl_t *)arg;
	struct cam_ois_soc_private *soc_private;
	struct cam_sensor_power_ctrl_t *power_info;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "failed: o_ctrl %pK", o_ctrl);
		return -EINVAL;
	}

	soc_private = (struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	if (!soc_private) {
		CAM_ERR(CAM_OIS, "failed:soc_private %pK", soc_private);
		return -EINVAL;
	}
	else{
		power_info  = &soc_private->power_info;
		if (!power_info){
			CAM_ERR(CAM_OIS, "failed: power_info %pK", o_ctrl, power_info);
			return -EINVAL;
		}
	}

	mutex_lock(&(o_ctrl->ois_power_down_mutex));
	o_ctrl->ois_power_down_thread_state = CAM_OIS_POWER_DOWN_THREAD_RUNNING;
	mutex_unlock(&(o_ctrl->ois_power_down_mutex));

	for (i = 0; i < (OIS_POWER_DOWN_DELAY/50); i++) {
#ifdef OPLUS_FEATURE_CAMERA_COMMON
		if (o_ctrl->ois_power_state == CAM_OIS_POWER_OFF) {
			CAM_WARN(CAM_OIS, "ois type=%d has powered down", o_ctrl->ois_type);
			break;
		}
		if(!IsOISReady(o_ctrl)){
			CAM_ERR(CAM_OIS, "ois type=%d, is not ready!", o_ctrl->ois_type);
			break;
		}
#endif
		msleep(50);// sleep 50ms every time, and sleep OIS_POWER_DOWN_DELAY/50 times.

		mutex_lock(&(o_ctrl->ois_power_down_mutex));
		if (o_ctrl->ois_power_down_thread_exit) {
			mutex_unlock(&(o_ctrl->ois_power_down_mutex));
			break;
		}
		mutex_unlock(&(o_ctrl->ois_power_down_mutex));
	}

	mutex_lock(&(o_ctrl->ois_power_down_mutex));
	if ((!o_ctrl->ois_power_down_thread_exit) && (o_ctrl->ois_power_state == CAM_OIS_POWER_ON)) {
		cam_set_ois_disable(o_ctrl);
		rc = cam_ois_power_down(o_ctrl);
		if (!rc){
			kfree(power_info->power_setting);
			kfree(power_info->power_down_setting);
			power_info->power_setting = NULL;
			power_info->power_down_setting = NULL;
			power_info->power_down_setting_size = 0;
			power_info->power_setting_size = 0;
			CAM_INFO(CAM_OIS, "ois type=%d,cam_ois_power_down successfully",o_ctrl->ois_type);
		} else {
			CAM_ERR(CAM_OIS, "ois type=%d,cam_ois_power_down failed",o_ctrl->ois_type);
		}
		o_ctrl->ois_power_state = CAM_OIS_POWER_OFF;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
		if(o_ctrl->cam_ois_download_fw_in_advance)
		{
			mutex_lock(&(o_ctrl->do_ioctl_ois));
			o_ctrl->ois_downloadfw_thread = NULL;
			o_ctrl->ois_download_fw_done = CAM_OIS_FW_NOT_DOWNLOAD;
			o_ctrl->ois_fd_have_close_state = CAM_OIS_IS_CLOSE;
			mutex_unlock(&(o_ctrl->do_ioctl_ois));
			CAM_INFO(CAM_OIS, "ois type=%d,cam_ois_power_down,so reset state",o_ctrl->ois_type);
		}
#endif

	} else {
		CAM_INFO(CAM_OIS, "ois type=%d,No need to do power down, ois_power_down_thread_exit %d, ois_power_state %d",o_ctrl->ois_type, o_ctrl->ois_power_down_thread_exit, o_ctrl->ois_power_state);
	}
	o_ctrl->ois_power_down_thread_state = CAM_OIS_POWER_DOWN_THREAD_STOPPED;
	mutex_unlock(&(o_ctrl->ois_power_down_mutex));

	return rc;
}
#endif

int32_t cam_ois_construct_default_power_setting(
	struct cam_sensor_power_ctrl_t *power_info)
{
	int rc = 0;

	power_info->power_setting_size = 1;
	power_info->power_setting =
		kzalloc(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_setting)
		return -ENOMEM;

	power_info->power_setting[0].seq_type = SENSOR_VAF;
	power_info->power_setting[0].seq_val = CAM_VAF;
	power_info->power_setting[0].config_val = 1;
	power_info->power_setting[0].delay = 2;

	power_info->power_down_setting_size = 1;
	power_info->power_down_setting =
		kzalloc(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_down_setting) {
		rc = -ENOMEM;
		goto free_power_settings;
	}

	power_info->power_down_setting[0].seq_type = SENSOR_VAF;
	power_info->power_down_setting[0].seq_val = CAM_VAF;
	power_info->power_down_setting[0].config_val = 0;

	return rc;

free_power_settings:
	kfree(power_info->power_setting);
	power_info->power_setting = NULL;
	power_info->power_setting_size = 0;
	return rc;
}


/**
 * cam_ois_get_dev_handle - get device handle
 * @o_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int cam_ois_get_dev_handle(struct cam_ois_ctrl_t *o_ctrl,
	void *arg)
{
	struct cam_sensor_acquire_dev    ois_acq_dev;
	struct cam_create_dev_hdl        bridge_params;
	struct cam_control              *cmd = (struct cam_control *)arg;

	if (o_ctrl->bridge_intf.device_hdl != -1) {
		CAM_ERR(CAM_OIS, "Device is already acquired");
		return -EFAULT;
	}
	if (copy_from_user(&ois_acq_dev, u64_to_user_ptr(cmd->handle),
		sizeof(ois_acq_dev)))
		return -EFAULT;

	bridge_params.session_hdl = ois_acq_dev.session_handle;
	bridge_params.ops = &o_ctrl->bridge_intf.ops;
	bridge_params.v4l2_sub_dev_flag = 0;
	bridge_params.media_entity_flag = 0;
	bridge_params.priv = o_ctrl;
	bridge_params.dev_id = CAM_OIS;

	ois_acq_dev.device_handle =
		cam_create_device_hdl(&bridge_params);
	if (ois_acq_dev.device_handle <= 0) {
		CAM_ERR(CAM_OIS, "Can not create device handle");
		return -EFAULT;
	}
	o_ctrl->bridge_intf.device_hdl = ois_acq_dev.device_handle;
	o_ctrl->bridge_intf.session_hdl = ois_acq_dev.session_handle;

	CAM_DBG(CAM_OIS, "Device Handle: %d", ois_acq_dev.device_handle);
	if (copy_to_user(u64_to_user_ptr(cmd->handle), &ois_acq_dev,
		sizeof(struct cam_sensor_acquire_dev))) {
		CAM_ERR(CAM_OIS, "ACQUIRE_DEV: copy to user failed");
		return -EFAULT;
	}
	return 0;
}

#ifdef OPLUS_FEATURE_CAMERA_COMMON
int cam_ois_power_up(struct cam_ois_ctrl_t *o_ctrl)
#else
static int cam_ois_power_up(struct cam_ois_ctrl_t *o_ctrl)
#endif
{
	int                                     rc = 0;
	struct cam_hw_soc_info                 *soc_info = &o_ctrl->soc_info;
	struct cam_ois_soc_private             *soc_private;
	struct cam_sensor_power_ctrl_t         *power_info;
	struct completion                      *i3c_probe_completion = NULL;

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	CAM_INFO(CAM_OIS, "cam_ois_power_up");
#endif
	soc_private = (struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	if ((power_info->power_setting == NULL) &&
		(power_info->power_down_setting == NULL)) {
		CAM_INFO(CAM_OIS,
			"Using default power settings");
#ifdef OPLUS_FEATURE_CAMERA_COMMON
		if(strstr(o_ctrl->ois_name, "129")){
			rc = oplus_cam_ois_construct_default_power_setting_129(power_info);
			CAM_DBG(CAM_OIS,"Using 129 power settings");
		} else if(strstr(o_ctrl->ois_name, "bu24721")){
			oplus_cam_ois_construct_default_power_setting_bu24721(power_info);
			CAM_DBG(CAM_OIS,"Using bu24271 power settings");
		}else if (strstr(o_ctrl->ois_name, "sem1217s")) {
			rc = oplus_cam_ois_construct_default_power_setting_1217s(power_info);
			CAM_INFO(CAM_OIS,"Using 1217 power settings");
		} else{
			rc = oplus_cam_ois_construct_default_power_setting(power_info);
		}
#else
		rc = cam_ois_construct_default_power_setting(power_info);
#endif
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Construct default ois power setting failed.");
			return rc;
		}
	}

	/* Parse and fill vreg params for power up settings */
	rc = msm_camera_fill_vreg_params(
		soc_info,
		power_info->power_setting,
		power_info->power_setting_size);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"failed to fill vreg params for power up rc:%d", rc);
		return rc;
	}

	/* Parse and fill vreg params for power down settings*/
	rc = msm_camera_fill_vreg_params(
		soc_info,
		power_info->power_down_setting,
		power_info->power_down_setting_size);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"failed to fill vreg params for power down rc:%d", rc);
		return rc;
	}

	power_info->dev = soc_info->dev;

	if (o_ctrl->io_master_info.master_type == I3C_MASTER)
		i3c_probe_completion = cam_ois_get_i3c_completion(o_ctrl->soc_info.index);

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	rc = cam_sensor_core_power_up(power_info, soc_info, i3c_probe_completion, &(o_ctrl->io_master_info));
#else
	rc = cam_sensor_core_power_up(power_info, soc_info, i3c_probe_completion);
#endif
	if (rc) {
		CAM_ERR(CAM_OIS, "failed in ois power up rc %d", rc);
		return rc;
	}

	rc = camera_io_init(&o_ctrl->io_master_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "cci_init failed: rc: %d", rc);
		goto cci_failure;
	}

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	InitOIS(o_ctrl);
#endif

	return rc;
cci_failure:
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if (cam_sensor_util_power_down(power_info, soc_info, &(o_ctrl->io_master_info)))
#else
	if (cam_sensor_util_power_down(power_info, soc_info))
#endif
		CAM_ERR(CAM_OIS, "Power Down failed");

	return rc;
}

/**
 * cam_ois_power_down - power down OIS device
 * @o_ctrl:     ctrl structure
 *
 * Returns success or failure
 */
#ifdef OPLUS_FEATURE_CAMERA_COMMON
int cam_ois_power_down(struct cam_ois_ctrl_t *o_ctrl)
#else
static int cam_ois_power_down(struct cam_ois_ctrl_t *o_ctrl)
#endif
{
	int32_t                         rc = 0;
	struct cam_sensor_power_ctrl_t  *power_info;
	struct cam_hw_soc_info          *soc_info =
		&o_ctrl->soc_info;
	struct cam_ois_soc_private *soc_private;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "failed: o_ctrl %pK", o_ctrl);
		return -EINVAL;
	}

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	CAM_INFO(CAM_OIS, "cam_ois_power_down");
#endif
	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;
	soc_info = &o_ctrl->soc_info;

	if (!power_info) {
		CAM_ERR(CAM_OIS, "failed: power_info %pK", power_info);
		return -EINVAL;
	}

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	DeinitOIS(o_ctrl);

	rc = cam_sensor_util_power_down(power_info, soc_info, &(o_ctrl->io_master_info));
#else
	rc = cam_sensor_util_power_down(power_info, soc_info);
#endif
	if (rc) {
		CAM_ERR(CAM_OIS, "power down the core is failed:%d", rc);
		return rc;
	}

	camera_io_release(&o_ctrl->io_master_info);

	return rc;
}

static int cam_ois_update_time(struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0;
	uint32_t size = 0;
	uint32_t i = 0;
	uint64_t qtime_ns = 0;

	if (i2c_set == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	rc = cam_sensor_util_get_current_qtimer_ns(&qtime_ns);
	if (rc < 0) {
		CAM_ERR(CAM_OIS,
			"Failed to get current qtimer value: %d",
			rc);
		return rc;
	}

	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
		if (i2c_list->op_code ==  CAM_SENSOR_I2C_WRITE_SEQ) {
			size = i2c_list->i2c_settings.size;
			/* qtimer is 8 bytes so validate here*/
			if (size < 8) {
				CAM_ERR(CAM_OIS, "Invalid write time settings");
				return -EINVAL;
			}
			for (i = 0; i < size; i++) {
				CAM_DBG(CAM_OIS, "time: reg_data[%d]: 0x%x",
					i, (qtime_ns & 0xFF));
				i2c_list->i2c_settings.reg_setting[i].reg_data =
					(qtime_ns & 0xFF);
				qtime_ns >>= 8;
			}
		}
	}

	return rc;
}

static int cam_ois_apply_settings(struct cam_ois_ctrl_t *o_ctrl,
	struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0;
	uint32_t i, size;

	if (o_ctrl == NULL || i2c_set == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	if (i2c_set->is_settings_valid != 1) {
		CAM_ERR(CAM_OIS, " Invalid settings");
		return -EINVAL;
	}

	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
		if (i2c_list->op_code ==  CAM_SENSOR_I2C_WRITE_RANDOM) {
			rc = camera_io_dev_write(&(o_ctrl->io_master_info),
				&(i2c_list->i2c_settings));
#ifdef OPLUS_FEATURE_CAMERA_COMMON
			CAM_INFO(CAM_OIS,"type=%d write ois register addr=0x%x data=0x%x ",o_ctrl->ois_type,i2c_list->i2c_settings.reg_setting->reg_addr,i2c_list->i2c_settings.reg_setting->reg_data);
#endif
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"Failed in Applying i2c wrt settings");
				return rc;
			}
		} else if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_SEQ) {
			rc = camera_io_dev_write_continuous(
				&(o_ctrl->io_master_info),
				&(i2c_list->i2c_settings),
				CAM_SENSOR_I2C_WRITE_SEQ);
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"Failed to seq write I2C settings: %d",
					rc);
				return rc;
			}
		} else if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
			size = i2c_list->i2c_settings.size;
			for (i = 0; i < size; i++) {
				rc = camera_io_dev_poll(
				&(o_ctrl->io_master_info),
				i2c_list->i2c_settings.reg_setting[i].reg_addr,
				i2c_list->i2c_settings.reg_setting[i].reg_data,
				i2c_list->i2c_settings.reg_setting[i].data_mask,
				i2c_list->i2c_settings.addr_type,
				i2c_list->i2c_settings.data_type,
				i2c_list->i2c_settings.reg_setting[i].delay);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"i2c poll apply setting Fail");
					return rc;
				}
			}
		}
	}

	return rc;
}

static int cam_ois_slaveInfo_pkt_parser(struct cam_ois_ctrl_t *o_ctrl,
	uint32_t *cmd_buf, size_t len)
{
	int32_t rc = 0;
	struct cam_cmd_ois_info *ois_info;

	if (!o_ctrl || !cmd_buf || len < sizeof(struct cam_cmd_ois_info)) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	ois_info = (struct cam_cmd_ois_info *)cmd_buf;
	if (o_ctrl->io_master_info.master_type == CCI_MASTER) {
		o_ctrl->io_master_info.cci_client->i2c_freq_mode =
			ois_info->i2c_freq_mode;
		o_ctrl->io_master_info.cci_client->sid =
			ois_info->slave_addr >> 1;
		o_ctrl->ois_fw_flag = ois_info->ois_fw_flag;
		o_ctrl->is_ois_calib = ois_info->is_ois_calib;
		memcpy(o_ctrl->ois_name, ois_info->ois_name, OIS_NAME_LEN);
		o_ctrl->ois_name[OIS_NAME_LEN - 1] = '\0';
		o_ctrl->io_master_info.cci_client->retries = 3;
		o_ctrl->io_master_info.cci_client->id_map = 0;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
		if(o_ctrl->ois_change_cci) {
			if( o_ctrl->ois_type == 0){
				if(chip_version_old){
					o_ctrl->io_master_info.cci_client->cci_device = CCI_DEVICE_0;
					o_ctrl->io_master_info.cci_client->cci_i2c_master = MASTER_0;
					o_ctrl->cci_i2c_master = MASTER_0;
					CAM_INFO(CAM_OIS, "change old module cci");
				}
			}
		}
#endif
		memcpy(&(o_ctrl->opcode), &(ois_info->opcode),
			sizeof(struct cam_ois_opcode));
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x Freq Mode: %d",
			ois_info->slave_addr, ois_info->i2c_freq_mode);
	} else if (o_ctrl->io_master_info.master_type == I2C_MASTER) {
		o_ctrl->io_master_info.client->addr = ois_info->slave_addr;
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x", ois_info->slave_addr);
	} else {
		CAM_ERR(CAM_OIS, "Invalid Master type : %d",
			o_ctrl->io_master_info.master_type);
		rc = -EINVAL;
	}

	return rc;
}

static int cam_ois_fw_download(struct cam_ois_ctrl_t *o_ctrl)
{
	uint16_t                           total_bytes = 0;
	uint8_t                           *ptr = NULL;
	int32_t                            rc = 0, cnt;
	uint32_t                           fw_size;
	const struct firmware             *fw = NULL;
	const char                        *fw_name_prog = NULL;
	const char                        *fw_name_coeff = NULL;
	char                               name_prog[32] = {0};
	char                               name_coeff[32] = {0};
	struct device                     *dev = &(o_ctrl->pdev->dev);
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	void                              *vaddr = NULL;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	snprintf(name_coeff, 32, "%s.coeff", o_ctrl->ois_name);

	snprintf(name_prog, 32, "%s.prog", o_ctrl->ois_name);

	/* cast pointer as const pointer*/
	fw_name_prog = name_prog;
	fw_name_coeff = name_coeff;

	/* Load FW */
	rc = request_firmware(&fw, fw_name_prog, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_prog);
		return rc;
	}

	total_bytes = fw->size;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	fw_size = (sizeof(struct cam_sensor_i2c_reg_array) * total_bytes);
	vaddr = vmalloc(fw_size);
	if (!vaddr) {
		CAM_ERR(CAM_OIS,
			"Failed in allocating i2c_array: fw_size: %u", fw_size);
		release_firmware(fw);
		return -ENOMEM;
	}

	CAM_DBG(CAM_OIS, "FW prog size:%d.", total_bytes);

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (
		vaddr);

	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;
		cnt++, ptr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr =
			o_ctrl->opcode.prog;
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		&i2c_reg_setting, CAM_SENSOR_I2C_WRITE_BURST);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS FW(prog) size(%d) download failed. %d",
			total_bytes, rc);
		goto release_firmware;
	}
	vfree(vaddr);
	vaddr = NULL;
	fw_size = 0;
	release_firmware(fw);

	rc = request_firmware(&fw, fw_name_coeff, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_coeff);
		return rc;
	}

	total_bytes = fw->size;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	fw_size = (sizeof(struct cam_sensor_i2c_reg_array) * total_bytes);
	vaddr = vmalloc(fw_size);
	if (!vaddr) {
		CAM_ERR(CAM_OIS,
			"Failed in allocating i2c_array: fw_size: %u", fw_size);
		release_firmware(fw);
		return -ENOMEM;
	}

	CAM_DBG(CAM_OIS, "FW coeff size:%d", total_bytes);

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (
		vaddr);

	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;
		cnt++, ptr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr =
			o_ctrl->opcode.coeff;
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		&i2c_reg_setting, CAM_SENSOR_I2C_WRITE_BURST);

	if (rc < 0)
		CAM_ERR(CAM_OIS, "OIS FW(coeff) size(%d) download failed rc: %d",
			total_bytes, rc);

release_firmware:
	vfree(vaddr);
	vaddr = NULL;
	fw_size = 0;
	release_firmware(fw);
	return rc;
}

/**
 * cam_ois_pkt_parse - Parse csl packet
 * @o_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int cam_ois_pkt_parse(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{
	int32_t                         rc = 0;
	int32_t                         i = 0;
	uint32_t                        total_cmd_buf_in_bytes = 0;
	struct common_header           *cmm_hdr = NULL;
	uintptr_t                       generic_ptr;
	struct cam_control             *ioctl_ctrl = NULL;
	struct cam_config_dev_cmd       dev_config;
	struct i2c_settings_array      *i2c_reg_settings = NULL;
	struct cam_cmd_buf_desc        *cmd_desc = NULL;
	uintptr_t                       generic_pkt_addr;
	size_t                          pkt_len;
	size_t                          remain_len = 0;
	struct cam_packet              *csl_packet = NULL;
	size_t                          len_of_buff = 0;
	uint32_t                       *offset = NULL, *cmd_buf;
	struct cam_ois_soc_private     *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t  *power_info = &soc_private->power_info;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	int count=0;
	int enable=0;
#endif

	ioctl_ctrl = (struct cam_control *)arg;
	if (copy_from_user(&dev_config,
		u64_to_user_ptr(ioctl_ctrl->handle),
		sizeof(dev_config)))
		return -EFAULT;
	rc = cam_mem_get_cpu_buf(dev_config.packet_handle,
		&generic_pkt_addr, &pkt_len);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"error in converting command Handle Error: %d", rc);
		return rc;
	}

	remain_len = pkt_len;
	if ((sizeof(struct cam_packet) > pkt_len) ||
		((size_t)dev_config.offset >= pkt_len -
		sizeof(struct cam_packet))) {
		CAM_ERR(CAM_OIS,
			"Inval cam_packet strut size: %zu, len_of_buff: %zu",
			 sizeof(struct cam_packet), pkt_len);
		return -EINVAL;
	}

	remain_len -= (size_t)dev_config.offset;
	csl_packet = (struct cam_packet *)
		(generic_pkt_addr + (uint32_t)dev_config.offset);

	if (cam_packet_util_validate_packet(csl_packet,
		remain_len)) {
		CAM_ERR(CAM_OIS, "Invalid packet params");
		return -EINVAL;
	}

	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_OIS_PACKET_OPCODE_INIT:
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);

		CAM_DBG(CAM_OIS, "num_cmd_buf %d",
			csl_packet->num_cmd_buf);

		/* Loop through multiple command buffers */
		for (i = 0; i < csl_packet->num_cmd_buf; i++) {
			total_cmd_buf_in_bytes = cmd_desc[i].length;
			if (!total_cmd_buf_in_bytes)
				continue;

			rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
				&generic_ptr, &len_of_buff);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "Failed to get cpu buf : 0x%x",
					cmd_desc[i].mem_handle);
				return rc;
			}
			cmd_buf = (uint32_t *)generic_ptr;
			if (!cmd_buf) {
				CAM_ERR(CAM_OIS, "invalid cmd buf");
				return -EINVAL;
			}

			if ((len_of_buff < sizeof(struct common_header)) ||
				(cmd_desc[i].offset > (len_of_buff -
				sizeof(struct common_header)))) {
				CAM_ERR(CAM_OIS,
					"Invalid length for sensor cmd");
				return -EINVAL;
			}
			remain_len = len_of_buff - cmd_desc[i].offset;
			cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);
			cmm_hdr = (struct common_header *)cmd_buf;

			switch (cmm_hdr->cmd_type) {
			case CAMERA_SENSOR_CMD_TYPE_I2C_INFO:
				rc = cam_ois_slaveInfo_pkt_parser(
					o_ctrl, cmd_buf, remain_len);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"Failed in parsing slave info");
					return rc;
				}
				break;
			case CAMERA_SENSOR_CMD_TYPE_PWR_UP:
			case CAMERA_SENSOR_CMD_TYPE_PWR_DOWN:
				CAM_DBG(CAM_OIS,
					"Received power settings buffer");
#ifdef ENABLE_OIS_DELAY_POWER_DOWN
				mutex_lock(&(o_ctrl->ois_power_down_mutex));
				if (o_ctrl->ois_power_state == CAM_OIS_POWER_OFF){
					rc = cam_sensor_update_power_settings(
						cmd_buf,
						total_cmd_buf_in_bytes,
						power_info, remain_len);
					if (!rc) {
						CAM_INFO(CAM_OIS, "ois type=%d,cam_sensor_update_power_settings successfully",o_ctrl->ois_type);
					} else {
						CAM_ERR(CAM_OIS, "ois type=%d,cam_sensor_update_power_settings failed",o_ctrl->ois_type);
						mutex_unlock(&(o_ctrl->ois_power_down_mutex));
						return rc;
					}
				} else {
					CAM_ERR(CAM_OIS, "ois type=%d,OIS already power on, no need to update power setting",o_ctrl->ois_type);
				}
				mutex_unlock(&(o_ctrl->ois_power_down_mutex));
#else
				rc = cam_sensor_update_power_settings(
					cmd_buf,
					total_cmd_buf_in_bytes,
					power_info, remain_len);
#endif
				if (rc) {
					CAM_ERR(CAM_OIS,
					"Failed: parse power settings");
					return rc;
				}
				break;
			default:
			if (o_ctrl->i2c_init_data.is_settings_valid == 0) {
				CAM_DBG(CAM_OIS,
				"Received init/config settings");
				i2c_reg_settings =
					&(o_ctrl->i2c_init_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"init parsing failed: %d", rc);
					return rc;
				}
			} else if ((o_ctrl->is_ois_calib != 0) &&
				(o_ctrl->i2c_calib_data.is_settings_valid ==
				0)) {
				CAM_DBG(CAM_OIS,
					"Received calib settings");
				i2c_reg_settings = &(o_ctrl->i2c_calib_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"Calib parsing failed: %d", rc);
					return rc;
				}
			} else if (o_ctrl->i2c_fwinit_data.is_settings_valid == 0) {
				CAM_DBG(CAM_OIS, "received fwinit settings");
				i2c_reg_settings =
					&(o_ctrl->i2c_fwinit_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"fw init parsing failed: %d", rc);
					return rc;
				}
			}
			break;
			}
		}

		if (o_ctrl->cam_ois_state != CAM_OIS_CONFIG) {
#ifdef ENABLE_OIS_DELAY_POWER_DOWN
			mutex_lock(&(o_ctrl->ois_power_down_mutex));
			o_ctrl->ois_power_down_thread_exit = true;
			if (o_ctrl->ois_power_state == CAM_OIS_POWER_OFF){
				rc = cam_ois_power_up(o_ctrl);
				if (!rc){
					o_ctrl->ois_power_state = CAM_OIS_POWER_ON;
					CAM_INFO(CAM_OIS, "ois type=%d,cam_ois_power_up successfully",o_ctrl->ois_type);
				} else {
					CAM_ERR(CAM_OIS, "ois type=%d,cam_ois_power_up failed",o_ctrl->ois_type);
					mutex_unlock(&(o_ctrl->ois_power_down_mutex));
					return rc;
				}
			} else {
				if(o_ctrl->ois_switch_spi_mode) {
					spi_mode_switch(o_ctrl);//modify for ois not power down, but switch scene, should switch master and monitor mode
				}
				CAM_INFO(CAM_OIS, "ois type=%d,OIS already power on, no need to power on again",o_ctrl->ois_type);
			}
			mutex_unlock(&(o_ctrl->ois_power_down_mutex));
#else
			rc = cam_ois_power_up(o_ctrl);
#endif
			if (rc) {
				CAM_ERR(CAM_OIS, " OIS Power up failed");
				return rc;
			}
			o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		}

		if (o_ctrl->i2c_fwinit_data.is_settings_valid == 1) {
			rc = cam_ois_apply_settings(o_ctrl,
				&o_ctrl->i2c_fwinit_data);
			if ((rc == -EAGAIN) &&
				(o_ctrl->io_master_info.master_type == CCI_MASTER)) {
				CAM_WARN(CAM_OIS,
					"CCI HW is restting: Reapplying fwinit settings");
				usleep_range(1000, 1010);
				rc = cam_ois_apply_settings(o_ctrl,
					&o_ctrl->i2c_fwinit_data);
			}
			if (rc) {
				CAM_ERR(CAM_OIS,
					"Cannot apply fwinit data %d",
					rc);
				goto pwr_dwn;
			} else {
				CAM_DBG(CAM_OIS, "OIS fwinit settings success");
			}
		}

#ifdef OPLUS_FEATURE_CAMERA_COMMON
		trace_begin("%d_%d_%s Download FW", o_ctrl->cci_num, o_ctrl->cci_i2c_master, o_ctrl->ois_name);
#endif
		if (o_ctrl->ois_fw_flag) {
			CAM_DBG(CAM_OIS, "read ois_name %s", o_ctrl->ois_name);
			if (strstr(o_ctrl->ois_name, "lc898")) {
#ifdef OPLUS_FEATURE_CAMERA_COMMON
				o_ctrl->ois_module_vendor = (o_ctrl->opcode.pheripheral & 0xFF00) >> 8;
				o_ctrl->ois_actuator_vendor = o_ctrl->opcode.pheripheral & 0xFF;
				if(o_ctrl->cam_ois_download_fw_in_advance)
				{
					mutex_lock(&(o_ctrl->do_ioctl_ois));
					if(o_ctrl->ois_download_fw_done == CAM_OIS_FW_NOT_DOWNLOAD)
					{
						rc = DownloadFW(o_ctrl);
					}
					else
					{
						CAM_INFO(CAM_OIS, "OIS FW Have Download");
					}
					if(rc)
						o_ctrl->ois_download_fw_done = CAM_OIS_FW_NOT_DOWNLOAD;
					else
						o_ctrl->ois_download_fw_done = CAM_OIS_FW_DOWNLOAD_DONE;
					mutex_unlock(&(o_ctrl->do_ioctl_ois));
				}
				else
					rc = DownloadFW(o_ctrl);
			} else if(strstr(o_ctrl->ois_name, "bu24721_tele")) {
				if(o_ctrl->cam_ois_download_fw_in_advance)
				{
					//OIS for imx766_bu24721gwz
					mutex_lock(&(o_ctrl->do_ioctl_ois));
					if(o_ctrl->ois_download_fw_done == CAM_OIS_FW_NOT_DOWNLOAD){
						rc = DownloadFW(o_ctrl);
					}
					else
					{
						CAM_INFO(CAM_OIS, "Tele OIS FW Have Download");
					}
					if(rc)
						o_ctrl->ois_download_fw_done = CAM_OIS_FW_NOT_DOWNLOAD;
					else
						o_ctrl->ois_download_fw_done = CAM_OIS_FW_DOWNLOAD_DONE;
					mutex_unlock(&(o_ctrl->do_ioctl_ois));
				}
#endif
			} else {
				rc = cam_ois_fw_download(o_ctrl);
			}

			if (rc) {
				CAM_ERR(CAM_OIS, "Failed OIS FW Download");
#ifdef OPLUS_FEATURE_CAMERA_COMMON
				trace_end();
#endif
				goto pwr_dwn;
			}
		}
#ifdef OPLUS_FEATURE_CAMERA_COMMON
		trace_end();
#endif

		rc = cam_ois_apply_settings(o_ctrl, &o_ctrl->i2c_init_data);
		if ((rc == -EAGAIN) &&
			(o_ctrl->io_master_info.master_type == CCI_MASTER)) {
			CAM_WARN(CAM_OIS,
				"CCI HW is restting: Reapplying INIT settings");
			usleep_range(1000, 1010);
			rc = cam_ois_apply_settings(o_ctrl,
				&o_ctrl->i2c_init_data);
		}
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Cannot apply Init settings: rc = %d",
				rc);
			goto pwr_dwn;
		} else {
			CAM_DBG(CAM_OIS, "apply Init settings success");
		}

		if (o_ctrl->is_ois_calib) {
			rc = cam_ois_apply_settings(o_ctrl,
				&o_ctrl->i2c_calib_data);
			if ((rc == -EAGAIN) &&
				(o_ctrl->io_master_info.master_type == CCI_MASTER)) {
				CAM_WARN(CAM_OIS,
					"CCI HW is restting: Reapplying calib settings");
				usleep_range(1000, 1010);
				rc = cam_ois_apply_settings(o_ctrl,
					&o_ctrl->i2c_calib_data);
			}
			if (rc) {
				CAM_ERR(CAM_OIS, "Cannot apply calib data");
				goto pwr_dwn;
			} else {
				CAM_DBG(CAM_OIS, "apply calib data settings success");
			}
		}

		rc = delete_request(&o_ctrl->i2c_fwinit_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting fwinit data: rc: %d", rc);
			rc = 0;
		}

		rc = delete_request(&o_ctrl->i2c_init_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting Init data: rc: %d", rc);
			rc = 0;
		}
		rc = delete_request(&o_ctrl->i2c_calib_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting Calibration data: rc: %d", rc);
			rc = 0;
		}
		break;
	case CAM_OIS_PACKET_OPCODE_OIS_CONTROL:
		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Not in right state to control OIS: %d",
				o_ctrl->cam_ois_state);
			return rc;
		}
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_reg_settings = &(o_ctrl->i2c_mode_data);
		i2c_reg_settings->is_settings_valid = 1;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1, NULL);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS pkt parsing failed: %d", rc);
			return rc;
		}

#ifdef OPLUS_FEATURE_CAMERA_COMMON
		if (!IsOISReady(o_ctrl)) {
			CAM_ERR(CAM_OIS, "OIS is not ready, apply setting may fail");
			for(count=0;count<o_ctrl->soc_info.num_rgltr;count++){
				enable=regulator_is_enabled(regulator_get(o_ctrl->soc_info.dev,o_ctrl->soc_info.rgltr_name[count]));
				CAM_ERR(CAM_OIS, "regulator enable=%d,name[%d]=%s",enable,count,o_ctrl->soc_info.rgltr_name[count]);
			}
		}
		o_ctrl->ois_poll_thread_control_cmd = CAM_OIS_START_POLL_THREAD;
		OISControl(o_ctrl);
#endif

		rc = cam_ois_apply_settings(o_ctrl, i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot apply mode settings");
			return rc;
		}

		rc = delete_request(i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Fail deleting Mode data: rc: %d", rc);
			return rc;
		}
		break;
	case CAM_OIS_PACKET_OPCODE_READ: {
		uint64_t qtime_ns;
		struct cam_buf_io_cfg *io_cfg;
		struct i2c_settings_array i2c_read_settings;

		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Not in right state to read OIS: %d",
				o_ctrl->cam_ois_state);
			return rc;
		}
		CAM_DBG(CAM_OIS, "number of I/O configs: %d:",
			csl_packet->num_io_configs);
		if (csl_packet->num_io_configs == 0) {
			CAM_ERR(CAM_OIS, "No I/O configs to process");
			rc = -EINVAL;
			return rc;
		}

		INIT_LIST_HEAD(&(i2c_read_settings.list_head));

		io_cfg = (struct cam_buf_io_cfg *) ((uint8_t *)
			&csl_packet->payload +
			csl_packet->io_configs_offset);

		/* validate read data io config */
		if (io_cfg == NULL) {
			CAM_ERR(CAM_OIS, "I/O config is invalid(NULL)");
			rc = -EINVAL;
			return rc;
		}

		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_read_settings.is_settings_valid = 1;
		i2c_read_settings.request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			&i2c_read_settings,
			cmd_desc, 1, &io_cfg[0]);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS read pkt parsing failed: %d", rc);
			return rc;
		}

		rc = cam_sensor_util_get_current_qtimer_ns(&qtime_ns);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "failed to get qtimer rc:%d");
			return rc;
		}

#ifdef OPLUS_FEATURE_CAMERA_COMMON
		if(o_ctrl->ois_eis_function == 1) {
			rc = OIS_READ_HALL_DATA_TO_UMD(o_ctrl,&i2c_read_settings);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "cannot read data rc: %d", rc);
				delete_request(&i2c_read_settings);
				return rc;
			}
		}else if(o_ctrl->ois_eis_function == 2) {
			rc = OIS_READ_HALL_DATA_TO_UMD_NEW(o_ctrl,&i2c_read_settings);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "cannot read data rc: %d", rc);
				delete_request(&i2c_read_settings);
				return rc;
			}
		}else if(o_ctrl->ois_eis_function == 3) {
			rc = OIS_READ_HALL_DATA_TO_UMD_129(o_ctrl,&i2c_read_settings);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "cannot read data rc: %d", rc);
				delete_request(&i2c_read_settings);
				return rc;
			}
		}else if(o_ctrl->ois_eis_function == 4) {
			rc = OIS_READ_HALL_DATA_TO_UMD_TELE124(o_ctrl,&i2c_read_settings);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "cannot read data rc: %d", rc);
				delete_request(&i2c_read_settings);
				return rc;
			}
		}else if(o_ctrl->ois_eis_function == 5) {
			rc = OIS_READ_HALL_DATA_TO_UMD_Bu24721(o_ctrl,&i2c_read_settings);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "cannot read data rc: %d", rc);
				delete_request(&i2c_read_settings);
				return rc;
			}
		}else if(o_ctrl->ois_eis_function == 6) {
			rc = OIS_READ_HALL_DATA_TO_UMD_SEM1217S(o_ctrl,&i2c_read_settings);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "cannot read data rc: %d", rc);
				delete_request(&i2c_read_settings);
				return rc;
			}
		}else {
			rc = cam_sensor_i2c_read_data(
				&i2c_read_settings,
				&o_ctrl->io_master_info);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "cannot read data rc: %d", rc);
				delete_request(&i2c_read_settings);
				return rc;
			}
		}
#else
		rc = cam_sensor_i2c_read_data(
			&i2c_read_settings,
			&o_ctrl->io_master_info);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "cannot read data rc: %d", rc);
			delete_request(&i2c_read_settings);
			return rc;
		}
#endif
		if (csl_packet->num_io_configs > 1) {
			rc = cam_sensor_util_write_qtimer_to_io_buffer(
				qtime_ns, &io_cfg[1]);
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"write qtimer failed rc: %d", rc);
				delete_request(&i2c_read_settings);
				return rc;
			}
		}

		rc = delete_request(&i2c_read_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Failed in deleting the read settings");
			return rc;
		}
		break;
	}
	case CAM_OIS_PACKET_OPCODE_WRITE_TIME: {
		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_ERR(CAM_OIS,
				"Not in right state to write time to OIS: %d",
				o_ctrl->cam_ois_state);
			return rc;
		}
#ifdef OPLUS_FEATURE_CAMERA_COMMON
		if(o_ctrl->ois_eis_function == 1 || o_ctrl->ois_eis_function == 5) {
			return 0;
		}else if(o_ctrl->ois_eis_function == 2 || o_ctrl->ois_eis_function == 4) {
			rc = WRITE_QTIMER_TO_OIS(o_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "Cannot update time");
				return rc;
			}
			break;
		}
#endif
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_reg_settings = &(o_ctrl->i2c_time_data);
		i2c_reg_settings->is_settings_valid = 1;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1, NULL);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS pkt parsing failed: %d", rc);
			return rc;
		}

		rc = cam_ois_update_time(i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot update time");
			return rc;
		}

		rc = cam_ois_apply_settings(o_ctrl, i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot apply mode settings");
			return rc;
		}

		rc = delete_request(i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Fail deleting Mode data: rc: %d", rc);
			return rc;
		}
		break;
	}
	default:
		CAM_ERR(CAM_OIS, "Invalid Opcode: %d",
			(csl_packet->header.op_code & 0xFFFFFF));
		return -EINVAL;
	}

	if (!rc)
		return rc;
pwr_dwn:
	cam_ois_power_down(o_ctrl);
	return rc;
}

void cam_ois_shutdown(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
#ifndef ENABLE_OIS_DELAY_POWER_DOWN
	struct cam_ois_soc_private *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	o_ctrl->ois_poll_thread_control_cmd = CAM_OIS_STOP_POLL_THREAD;
	OISControl(o_ctrl);
#endif

	if (o_ctrl->cam_ois_state == CAM_OIS_INIT)
		return;

	if (o_ctrl->cam_ois_state >= CAM_OIS_CONFIG) {
#ifdef ENABLE_OIS_DELAY_POWER_DOWN
		mutex_lock(&(o_ctrl->ois_power_down_mutex));
		if (o_ctrl->ois_power_state == CAM_OIS_POWER_ON && o_ctrl->ois_power_down_thread_state == CAM_OIS_POWER_DOWN_THREAD_STOPPED) {
			o_ctrl->ois_power_down_thread_exit = false;
			kthread_run(ois_power_down_thread, o_ctrl, "ois_power_down_thread");
			CAM_INFO(CAM_OIS, "ois type=%d,ois_power_down_thread created",o_ctrl->ois_type);
		} else {
			CAM_INFO(CAM_OIS, "ois type=%d,no need to create ois_power_down_thread, ois_power_state %d, ois_power_down_thread_state %d",o_ctrl->ois_type, o_ctrl->ois_power_state, o_ctrl->ois_power_down_thread_state);
		}
		mutex_unlock(&(o_ctrl->ois_power_down_mutex));
#else
		rc = cam_ois_power_down(o_ctrl);
#endif

		if (rc < 0)
			CAM_ERR(CAM_OIS, "OIS Power down failed");
		o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;
	}

	if (o_ctrl->cam_ois_state >= CAM_OIS_ACQUIRE) {
		rc = cam_destroy_device_hdl(o_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "destroying the device hdl");
		o_ctrl->bridge_intf.device_hdl = -1;
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.session_hdl = -1;
	}

	if (o_ctrl->i2c_fwinit_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_fwinit_data);

	if (o_ctrl->i2c_mode_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_mode_data);

	if (o_ctrl->i2c_calib_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_calib_data);

	if (o_ctrl->i2c_init_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_init_data);

#ifndef ENABLE_OIS_DELAY_POWER_DOWN
	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_down_setting_size = 0;
	power_info->power_setting_size = 0;
#endif

	o_ctrl->cam_ois_state = CAM_OIS_INIT;
}

/**
 * cam_ois_driver_cmd - Handle ois cmds
 * @e_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
int cam_ois_driver_cmd(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{
	int                              rc = 0;
	struct cam_ois_query_cap_t       ois_cap = {0};
	struct cam_control              *cmd = (struct cam_control *)arg;
#ifndef ENABLE_OIS_DELAY_POWER_DOWN
	struct cam_ois_soc_private      *soc_private = NULL;
	struct cam_sensor_power_ctrl_t  *power_info = NULL;
#endif

	if (!o_ctrl || !cmd) {
		CAM_ERR(CAM_OIS, "Invalid arguments");
		return -EINVAL;
	}

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_OIS, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}

#ifndef ENABLE_OIS_DELAY_POWER_DOWN
	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;
#endif

	mutex_lock(&(o_ctrl->ois_mutex));
	switch (cmd->op_code) {
	case CAM_QUERY_CAP:
		ois_cap.slot_info = o_ctrl->soc_info.index;

		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&ois_cap,
			sizeof(struct cam_ois_query_cap_t))) {
			CAM_ERR(CAM_OIS, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
		CAM_DBG(CAM_OIS, "ois_cap: ID: %d", ois_cap.slot_info);
		break;
	case CAM_ACQUIRE_DEV:
#ifdef OPLUS_FEATURE_CAMERA_COMMON
		if(o_ctrl->cam_ois_download_fw_in_advance)
		{
			mutex_lock(&(o_ctrl->ois_power_down_mutex));
			if (o_ctrl->ois_power_state == CAM_OIS_POWER_ON)
			{
				CAM_INFO(CAM_OIS, "ois need to exit power down thread");
				o_ctrl->ois_power_down_thread_exit = true;
			}
			mutex_unlock(&(o_ctrl->ois_power_down_mutex));
			mutex_lock(&(o_ctrl->do_ioctl_ois));
			o_ctrl->ois_fd_have_close_state = CAM_OIS_IS_OPEN;
			mutex_unlock(&(o_ctrl->do_ioctl_ois));
		}
#endif
		rc = cam_ois_get_dev_handle(o_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_OIS, "Failed to acquire dev");
			goto release_mutex;
		}
#ifdef OPLUS_FEATURE_CAMERA_COMMON
		CAM_INFO(CAM_OIS,
			"CAM_ACQUIRE_DEV Success, ID: %d", o_ctrl->soc_info.index);
#endif
		o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;
		break;
	case CAM_START_DEV:
		if (o_ctrl->cam_ois_state != CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
			"Not in right state for start : %d",
			o_ctrl->cam_ois_state);
			goto release_mutex;
		}
#ifdef OPLUS_FEATURE_CAMERA_COMMON
		CAM_INFO(CAM_OIS,
			"CAM_START_DEV Success, ID: %d", o_ctrl->soc_info.index);
#endif
		o_ctrl->cam_ois_state = CAM_OIS_START;
		break;
	case CAM_CONFIG_DEV:
#ifdef OPLUS_FEATURE_CAMERA_COMMON
		if(o_ctrl->cam_ois_download_fw_in_advance)
		{
			mutex_lock(&(o_ctrl->do_ioctl_ois));
			if(o_ctrl->ois_fd_have_close_state != CAM_OIS_IS_OPEN)
			{
				CAM_INFO(CAM_OIS, "ois have closing");
				mutex_unlock(&(o_ctrl->do_ioctl_ois));
				break;
			}
		mutex_unlock(&(o_ctrl->do_ioctl_ois));
		}
#endif
		rc = cam_ois_pkt_parse(o_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_OIS, "Failed in ois pkt Parsing");
			goto release_mutex;
		}
		break;
	case CAM_RELEASE_DEV:
#ifdef OPLUS_FEATURE_CAMERA_COMMON
		if(o_ctrl->cam_ois_download_fw_in_advance)
		{
			mutex_lock(&(o_ctrl->do_ioctl_ois));
			if(o_ctrl->ois_fd_have_close_state != CAM_OIS_IS_OPEN)
			{
				rc = 0;
				CAM_INFO(CAM_OIS,"ois have release");
				mutex_unlock(&(o_ctrl->do_ioctl_ois));
				goto release_mutex;
			}
			mutex_unlock(&(o_ctrl->do_ioctl_ois));
		}
#endif
		if (o_ctrl->cam_ois_state == CAM_OIS_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Cant release ois: in start state");
			goto release_mutex;
		}

		if (o_ctrl->cam_ois_state == CAM_OIS_CONFIG) {

#ifdef OPLUS_FEATURE_CAMERA_COMMON
			o_ctrl->ois_poll_thread_control_cmd = CAM_OIS_STOP_POLL_THREAD;
			OISControl(o_ctrl);
#endif
#ifdef ENABLE_OIS_DELAY_POWER_DOWN
			mutex_lock(&(o_ctrl->ois_power_down_mutex));
			if (o_ctrl->ois_power_state == CAM_OIS_POWER_ON && o_ctrl->ois_power_down_thread_state == CAM_OIS_POWER_DOWN_THREAD_STOPPED) {
				o_ctrl->ois_power_down_thread_exit = false;
				kthread_run(ois_power_down_thread, o_ctrl, "ois_power_down_thread");
				CAM_INFO(CAM_OIS, "ois type=%d,ois_power_down_thread created",o_ctrl->ois_type);
			} else {
				CAM_INFO(CAM_OIS, "ois type=%d,no need to create ois_power_down_thread, ois_power_state %d, ois_power_down_thread_state %d",o_ctrl->ois_type, o_ctrl->ois_power_state, o_ctrl->ois_power_down_thread_state);
			}
			mutex_unlock(&(o_ctrl->ois_power_down_mutex));
#else
			rc = cam_ois_power_down(o_ctrl);
#endif
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "OIS Power down failed");
				goto release_mutex;
			}
		}

		if (o_ctrl->bridge_intf.device_hdl == -1) {
			CAM_ERR(CAM_OIS, "link hdl: %d device hdl: %d",
				o_ctrl->bridge_intf.device_hdl,
				o_ctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = cam_destroy_device_hdl(o_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "destroying the device hdl");
		o_ctrl->bridge_intf.device_hdl = -1;
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.session_hdl = -1;
		o_ctrl->cam_ois_state = CAM_OIS_INIT;

#ifndef ENABLE_OIS_DELAY_POWER_DOWN
		kfree(power_info->power_setting);
		kfree(power_info->power_down_setting);
		power_info->power_setting = NULL;
		power_info->power_down_setting = NULL;
		power_info->power_down_setting_size = 0;
		power_info->power_setting_size = 0;
#endif

		if (o_ctrl->i2c_mode_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_mode_data);

		if (o_ctrl->i2c_calib_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_calib_data);

		if (o_ctrl->i2c_init_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_init_data);

		if (o_ctrl->i2c_fwinit_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_fwinit_data);

#ifdef OPLUS_FEATURE_CAMERA_COMMON
		CAM_INFO(CAM_OIS,
			"CAM_RELEASE_DEV, ID: %d", o_ctrl->soc_info.index);
#endif
		break;
	case CAM_STOP_DEV:
#ifdef OPLUS_FEATURE_CAMERA_COMMON
		if(o_ctrl->cam_ois_download_fw_in_advance)
		{
			mutex_lock(&(o_ctrl->do_ioctl_ois));
			if(o_ctrl->ois_fd_have_close_state != CAM_OIS_IS_OPEN)
			{
				rc = 0;
				CAM_INFO(CAM_OIS,"ois have stop");
				mutex_unlock(&(o_ctrl->do_ioctl_ois));
				break;
			}
			mutex_unlock(&(o_ctrl->do_ioctl_ois));
		}
#endif
		if (o_ctrl->cam_ois_state != CAM_OIS_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
			"Not in right state for stop : %d",
			o_ctrl->cam_ois_state);
		}
#ifdef OPLUS_FEATURE_CAMERA_COMMON
		CAM_INFO(CAM_OIS,
			"CAM_STOP_DEV, ID: %d", o_ctrl->soc_info.index);
#endif
		o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		break;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	case CAM_GET_OIS_EIS_HALL: {
		int get_hall_version;
		get_hall_version = cmd->reserved;
		if (o_ctrl->cam_ois_state == CAM_OIS_START || o_ctrl->cam_ois_state == CAM_OIS_CONFIG) {
			if (get_hall_version == GET_HALL_DATA_VERSION_V2){
				ReadOISHALLDataV2(o_ctrl, u64_to_user_ptr(cmd->handle));
			} else if (get_hall_version == GET_HALL_DATA_VERSION_V3){
				ReadOISHALLDataV3(o_ctrl, u64_to_user_ptr(cmd->handle));
			} else {
				ReadOISHALLData(o_ctrl, u64_to_user_ptr(cmd->handle));
			}
		} else {
			CAM_DBG(CAM_OIS, "OIS in wrong state %d", o_ctrl->cam_ois_state);
		}
		break;
	}

	case CAM_STORE_DUALOIS_GYRO_GAIN: {
		int m_result = 1;
		int enable = (int)cmd->reserved;

		if(enable) {
			m_result = StoreOisGyroGian(o_ctrl);
			CAM_ERR(CAM_OIS, "start CAM_STORE_DUALOIS_GYRO_GAIN m_result:%d", m_result);
		}else {
			CAM_ERR(CAM_OIS, "no need CAM_STORE_DUALOIS_GYRO_GAIN");
		}

		m_result = copy_to_user((void __user *) cmd->handle, &m_result, sizeof(m_result));
		if (m_result != 0) {
			CAM_ERR(CAM_OIS, "Failed set power status:%d !!!", enable);
			rc = -EFAULT;
			goto release_mutex;
		}
		break;
	}
	case CAM_WRITE_DUALOIS_GYRO_GAIN: {
		int m_result = 1;
		if(strstr(o_ctrl->ois_name, "imx766_bu24721_tele")) {
			OIS_GYROGAIN current_gyro_gain;
			if (copy_from_user(&current_gyro_gain, (void __user *)cmd->handle,
				sizeof(struct ois_gyrogain_t))) {
				CAM_ERR(CAM_OIS,
					"Fail in copy oem control infomation form user data");
				rc = -1;
				goto release_mutex;
			}else{
				WriteOisGyroGian(o_ctrl,&current_gyro_gain);
			}
		}else {
			DUAL_OIS_CALI_RESULTS current_gyro_gain;
			if (copy_from_user(&current_gyro_gain, (void __user *)cmd->handle,
				sizeof(struct dual_ois_calibration_t))) {
				CAM_ERR(CAM_OIS,
					"Fail in copy oem control infomation form user data");
				rc = -1;
				goto release_mutex;
			}else{
				WriteOisGyroGianToRam(o_ctrl,&current_gyro_gain);
			}

			current_gyro_gain.successed = 1;
			m_result = copy_to_user((void __user *) cmd->handle, &current_gyro_gain, sizeof(struct dual_ois_calibration_t));
			if (m_result != 0) {
				CAM_ERR(CAM_OIS, "Failed Copy multi hall data to User:%d !!!", m_result);
				rc = -1;
				goto release_mutex;
			}
		}

		break;
	}
	case CAM_DO_DUALOIS_GYRO_OFFSET: {
		uint32_t gyro_offset = 0;
		DoDualOisGyroOffset(o_ctrl, &gyro_offset);
		CAM_ERR(CAM_OIS, "CAM_DO_DUALOIS_GYRO_OFFSET:0x%x !!!", gyro_offset);
		if (copy_to_user((void __user *) cmd->handle, &gyro_offset,
			sizeof(gyro_offset))) {
			CAM_ERR(CAM_OIS, "Failed Copy to User");
			rc = -1;
			goto release_mutex;
		}

		break;
	}
	case CAM_QUERY_DUALOIS_SMA_WIRE_STATUS: {
		uint32_t sma_wire_broken = 0;
		QueryDualOisSmaWireStatus(o_ctrl, &sma_wire_broken);
		CAM_ERR(CAM_OIS, "QueryDualOisSmaWireStatus sma_wire_broken: 0x%x", sma_wire_broken);
		if (copy_to_user((void __user *) cmd->handle, &sma_wire_broken,
			sizeof(sma_wire_broken))) {
			CAM_ERR(CAM_OIS, "Failed Copy to User");
			rc = -1;
			goto release_mutex;
		}

		break;
	}
	case CAM_GET_DUALOIS_INITIAL_GYRO_GAIN: {
		int m_result = 1;
		DUAL_OIS_GYROGAIN initial_gyro_gain;

		GetDualOisGyroGain(o_ctrl, &initial_gyro_gain);
		m_result = copy_to_user((void __user *) cmd->handle, &initial_gyro_gain, sizeof(DUAL_OIS_GYROGAIN));
		if (m_result != 0) {
			CAM_ERR(CAM_OIS, "Failed Copy multi hall data to User:%d !!!", m_result);
			rc = -1;
			goto release_mutex;
		}
		break;
	}
	case CAM_WRITE_SHIFT_OIS_REGISTER: {
		int m_result = 1;
		uint32_t distance;
		if (copy_from_user(&distance, (void __user *)cmd->handle,
			sizeof(int32_t))) {
			CAM_ERR(CAM_OIS,
				"Fail in copy oem control infomation form user data");
			rc = -1;
			goto release_mutex;
		}else{
			WriteDualOisShiftRegister(o_ctrl, distance);
			CAM_DBG(CAM_OIS, "CAM_WRITE_SHIFT_OIS_REGISTER distance = %d", distance);
		}

		m_result = copy_to_user((void __user *) cmd->handle, &m_result, sizeof(int));
		if (m_result != 0) {
			CAM_ERR(CAM_OIS, "Failed Copy multi hall data to User:%d !!!", m_result);
			rc = -1;
			goto release_mutex;
		}
		break;
	}
	case CAM_FIRMWARE_CALI_GYRO_OFFSET: {
		uint32_t gyro_offset = 0;
		if(strstr(o_ctrl->ois_name, "bu24721")) {
			DoBU24721GyroOffset(o_ctrl, &gyro_offset);
		} else if (strstr(o_ctrl->ois_name, "sem1217s")) {
			DoSEM1217SGyroOffset(o_ctrl, &gyro_offset);
		}
		CAM_ERR(CAM_OIS, "[GyroOffsetCaliByFirmware] gyro_offset: 0x%x !!!", gyro_offset);
		if (copy_to_user((void __user *) cmd->handle, &gyro_offset,
			sizeof(gyro_offset))) {
			CAM_ERR(CAM_OIS, "Failed Copy to User");
			rc = -1;
			goto release_mutex;
		}

		break;
	}
	case CAM_TELE_OIS_USE_MONITOR: {
		uint32_t isTeleOisUseMonitor = 0;
		if (copy_from_user(&isTeleOisUseMonitor, (void __user *)cmd->handle,
			sizeof(isTeleOisUseMonitor))) {
			CAM_ERR(CAM_OIS,
				"Fail in copy oem control infomation form user data");
			rc = -1;
			goto release_mutex;
		}else{
			o_ctrl->pre_isTeleOisUseMonitor = o_ctrl->isTeleOisUseMonitor;
			o_ctrl->isTeleOisUseMonitor = isTeleOisUseMonitor ? true : false;
		}

		break;
	}
#endif

	default:
		CAM_ERR(CAM_OIS, "invalid opcode");
		goto release_mutex;
	}
release_mutex:
	mutex_unlock(&(o_ctrl->ois_mutex));
	return rc;
}
