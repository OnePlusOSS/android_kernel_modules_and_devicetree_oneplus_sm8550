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

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/dma-buf.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/clk.h>

#include "include/aon_uapi.h"

#ifdef MTK_AON
#include "mtk/include/mtk_aon_sensor_core.h"
#endif

#ifdef QCOM_AON
#include "qcom/include/qcom_aon_sensor_core.h"
#endif

s32 delete_request(struct i2c_settings_array *i2c_array)
{
	struct i2c_settings_list *i2c_list = NULL, *i2c_next = NULL;
	s32 rc = 0;

	if (i2c_array == NULL) {
		pr_err("%s FATAL: Invalid argument", __func__);
		return -EINVAL;
	}

	list_for_each_entry_safe(i2c_list, i2c_next,
		&(i2c_array->list_head), list) {
		pr_info("%s mm-vfree i2c_list->i2c_settings.reg_setting: %p",
			__func__, i2c_list->i2c_settings.reg_setting);
		vfree(i2c_list->i2c_settings.reg_setting);
		i2c_list->i2c_settings.reg_setting = NULL;
		list_del(&(i2c_list->list));
		pr_info("%s mm-kfree i2c_list: %p", __func__, i2c_list);
		kfree(i2c_list);
		i2c_list = NULL;
	}
	INIT_LIST_HEAD(&(i2c_array->list_head));
	i2c_array->is_settings_valid = 0;

	return rc;
}

void aon_sensor_release_stream_resource(struct i2c_data_settings *i2c_data)
{
	struct i2c_settings_array *i2c_set = NULL;
	s32 rc = 0;

	i2c_set = &(i2c_data->streamoff_settings);
	if (i2c_set->is_settings_valid == 1) {
		i2c_set->is_settings_valid = -1;
		rc = delete_request(i2c_set);
		if (rc < 0)
			pr_err("%s failed while deleting Streamoff settings", __func__);
	}

	i2c_set = &(i2c_data->streamon_settings);
	if (i2c_set->is_settings_valid == 1) {
		i2c_set->is_settings_valid = -1;
		rc = delete_request(i2c_set);
		if (rc < 0)
			pr_err("%s failed while deleting Streamon settings", __func__);
	}
}

static s32 aon_sensor_update_slave_info(struct aon_cmd_probe *probe_info,
	struct aon_sensor_ctrl_t *s_ctrl)
{
	s32 rc = 0;
	struct aon_sensor_board_info *data = NULL;

	if (!probe_info || !s_ctrl || !s_ctrl->sensordata) {
		pr_err("%s invalid params", __func__);
		return -EINVAL;
	}

	data = s_ctrl->sensordata;

	data->slave_info.sensor_id_reg_addr = probe_info->reg_addr;
	data->slave_info.sensor_id = probe_info->expected_data;
	data->slave_info.sensor_id_mask = probe_info->data_mask;

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	data->slave_info.addr_type = probe_info->addr_type;
	data->slave_info.data_type = probe_info->data_type;
	data->slave_info.needI2cSwitch = probe_info->needI2cSwitch;
#endif

	pr_info("%s Sensor Addr: 0x%x sensor_id: 0x%x sensor_mask: 0x%x",
		__func__,
		data->slave_info.sensor_id_reg_addr,
		data->slave_info.sensor_id,
		data->slave_info.sensor_id_mask);
	return rc;
}

static u16 aon_sensor_id_by_mask(struct aon_sensor_ctrl_t *s_ctrl, u32 chipid)
{
	u16 sensor_id = (u16)(chipid & 0xFFFF);
	s16 sensor_id_mask = 0;

	if (!s_ctrl) {
		pr_err("%s invalid param", __func__);
		return -EINVAL;
	}

	sensor_id_mask = s_ctrl->sensordata->slave_info.sensor_id_mask;
	pr_info("%s sensor_id_mask = 0x%x", __func__, sensor_id_mask);
	if (!sensor_id_mask)
		sensor_id_mask = ~sensor_id_mask;

	sensor_id &= sensor_id_mask;
	sensor_id_mask &= -sensor_id_mask;
	sensor_id_mask -= 1;
	while (sensor_id_mask) {
		sensor_id_mask >>= 1;
		sensor_id >>= 1;
	}
	pr_info("%s sensor_id = %d", __func__, sensor_id);
	return sensor_id;
}

static s32 aon_sensor_match_id(struct aon_sensor_ctrl_t *s_ctrl)
{
	s32 rc = 0;
	u32 chipid = 0;
	struct aon_camera_slave_info *slave_info = NULL;

	if (!s_ctrl || !s_ctrl->sensordata) {
		pr_err("%s invalid param", __func__);
		return -EINVAL;
	}

	slave_info = &(s_ctrl->sensordata->slave_info);

#ifdef QCOM_AON
		rc = aon_cci_init(s_ctrl);
		if (rc < 0) {
			pr_err("%s aon_cci_init failed, rc: %d", __func__, rc);
			return rc;
		}
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if(slave_info->needI2cSwitch && s_ctrl->sensordata->shift_mode_setting.size != 0) {
		pr_info("%s sid: %x",__func__, s_ctrl->cci_client->sid);
		aon_io_dev_write(s_ctrl->cci_client,&s_ctrl->sensordata->shift_mode_setting);
		s_ctrl->cci_client->sid = slave_info->sub_sensor_slave_addr >> 1;
	}
#endif

	rc = aon_io_read(s_ctrl,
			slave_info->sensor_id_reg_addr,
			&chipid, CAMERA_SENSOR_I2C_TYPE_WORD,
			CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0) {
		pr_err("%s aon_io_read failed, rc = %d", __func__, rc);
		goto out;
	}

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if(slave_info->needI2cSwitch && slave_info->sensor_slave_addr != 0) {
		s_ctrl->cci_client->sid = slave_info->sensor_slave_addr >> 1;
	}
#endif

	pr_info("%s read id: 0x%x expected id 0x%x",
			 __func__, chipid, slave_info->sensor_id);
	if (aon_sensor_id_by_mask(s_ctrl, chipid) != slave_info->sensor_id) {
		pr_err("%s chip id %x does not match %x", __func__,
				chipid, slave_info->sensor_id);
		rc = -ENODEV;
	}

out:
#ifdef QCOM_AON
	if (aon_cci_release(s_ctrl) < 0) {
		pr_err("%s aon_cci_release failed", __func__);
		rc = -1;
	}
#endif
	return rc;
}

static s32 aon_sensor_apply_settings(struct aon_sensor_ctrl_t *s_ctrl,
	enum aon_sensor_packet_opcodes opcode)
{
	s32 rc = 0;
	struct i2c_settings_array *i2c_set = NULL;
	struct i2c_settings_list *i2c_list = NULL;

	if (!s_ctrl)
		return -EINVAL;

	switch (opcode) {
	case AON_SENSOR_PACKET_OPCODE_STREAMON: {
		i2c_set = &s_ctrl->i2c_data.streamon_settings;
		pr_info("%s AON_SENSOR_PACKET_OPCODE_STREAMON", __func__);
		break;
	}
	case AON_SENSOR_PACKET_OPCODE_INITIAL_CONFIG: {
		i2c_set = &s_ctrl->i2c_data.init_settings;
		pr_info("%s AON_SENSOR_PACKET_OPCODE_INITIAL_CONFIG", __func__);
		break;
	}
	case AON_SENSOR_PACKET_OPCODE_CONFIG: {
		i2c_set = &s_ctrl->i2c_data.config_settings;
		pr_info("%s AON_SENSOR_PACKET_OPCODE_CONFIG", __func__);
		break;
	}
	case AON_SENSOR_PACKET_OPCODE_STREAMOFF: {
		i2c_set = &s_ctrl->i2c_data.streamoff_settings;
		pr_info("%s AON_SENSOR_PACKET_OPCODE_STREAMOFF", __func__);
		break;
	}
	case AON_SENSOR_PACKET_OPCODE_UPDATE:
	case AON_SENSOR_PACKET_OPCODE_PROBE:
	default:
		pr_info("%s opcode=%d", __func__, opcode);
		return 0;
	}

	if (i2c_set->is_settings_valid != 1)
		return rc;

#ifdef QCOM_AON
	rc = aon_cci_init(s_ctrl);
	if (rc < 0) {
		pr_err("%s aon_cci_init failed", __func__);
		return rc;
	}
#endif

	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
		rc = aon_sensor_modes_util(s_ctrl,
				i2c_list);
		if (rc < 0) {
			pr_err("%s Failed to apply settings: %d", __func__, rc);
			goto out;
		}
	}

out:
#ifdef QCOM_AON
	if (aon_cci_release(s_ctrl) < 0) {
		pr_err("%s aon_cci_release failed", __func__);
		rc = -1;
	}
#endif

	return rc;
}

#ifdef OPLUS_FEATURE_CAMERA_COMMON
static s32 aon_sensor_update_shift_settings(s32 count,
    struct aon_reg_setting *shift_settings,
    struct cam_sensor_i2c_reg_setting *pshift_mode_setting)
{
	s32 rc = 0;
	s32 i = 0;

	if (!shift_settings || !pshift_mode_setting || count <= 0 || count > MAX_POWER_CONFIG) {
		pr_err("%s Invalid Args, count: %d, shift_settings: %p, pshift_mode_setting: %p",
			__func__, count, shift_settings, pshift_mode_setting);
		return -EINVAL;
	}

	pshift_mode_setting->size = count;
	pshift_mode_setting->reg_setting =
		(struct cam_sensor_i2c_reg_array *)
		kzalloc(sizeof(struct cam_sensor_i2c_reg_array) *
			pshift_mode_setting->size, GFP_KERNEL);
	if (!pshift_mode_setting->reg_setting) {
		pr_err("%s shift_mode_setting kzalloc failed", __func__);
		return -ENOMEM;
	}
	pr_info("%s mm-kzalloc shift_mode_setting %p, size: %d count %d",
		__func__, pshift_mode_setting->reg_setting,
		sizeof(struct cam_sensor_i2c_reg_array) * pshift_mode_setting->size,count);

	for (i = 0; i < count; i++) {
		pshift_mode_setting->reg_setting[i].reg_addr  = shift_settings[i].register_addr;
		pshift_mode_setting->reg_setting[i].reg_data  = shift_settings[i].register_data;
		pshift_mode_setting->reg_setting[i].delay     = shift_settings[i].delayus;
		pshift_mode_setting->reg_setting[i].data_mask = 0x0;
		pshift_mode_setting->addr_type = shift_settings[i].regAddr_type;
		pshift_mode_setting->data_type = shift_settings[i].regData_type;
		pr_info("%s [%d], reg_addr: 0x%x, reg_data: 0x%x", __func__, i,
			pshift_mode_setting->reg_setting[i].reg_addr,
			pshift_mode_setting->reg_setting[i].reg_data);
	}
	return rc;
}
#endif

static s32 aon_sensor_update_power_up_settings(s32 count,
	struct aon_power_setting *pwr_settings,
	struct aon_sensor_power_ctrl_t *power_info)
{
	s32 rc = 0;
	s32 i = 0;

	if (!pwr_settings || !power_info || count <= 0 || count > MAX_POWER_CONFIG) {
		pr_err("%s Invalid Args, count: %d, pwr_settings: %p, power_info: %p",
			__func__, count, pwr_settings, power_info);
		return -EINVAL;
	}

	power_info->power_setting_size = 0;
	power_info->power_setting =
		(struct aon_sensor_power_setting *)
		kzalloc(sizeof(struct aon_sensor_power_setting) *
			MAX_POWER_CONFIG, GFP_KERNEL);
	if (!power_info->power_setting) {
		pr_err("%s power_setting kzalloc failed", __func__);
		return -ENOMEM;
	}
	pr_info("%s mm-kzalloc power_setting %p, size: %d",
		__func__, power_info->power_setting,
		sizeof(struct aon_sensor_power_setting) * MAX_POWER_CONFIG);

	for (i = 0; i < count; i++) {
		power_info->power_setting[i].seq_type = pwr_settings[i].config_type;
		power_info->power_setting[i].config_val = pwr_settings[i].config_value;
		power_info->power_setting[i].delay = pwr_settings[i].delayMs;
		pr_info("%s [%d], seq_type: %d, config_val: %ld, delay:%d", __func__, i,
			power_info->power_setting[i].seq_type,
			power_info->power_setting[i].config_val,
			power_info->power_setting[i].delay);
	}
	power_info->power_setting_size = count;
	return rc;
}

static s32 aon_sensor_update_power_down_settings(s32 count,
	struct aon_power_setting *pwr_settings,
	struct aon_sensor_power_ctrl_t *power_info)
{
	s32 rc = 0;
	s32 i = 0;

	if (!pwr_settings || !power_info || count <= 0 || count > MAX_POWER_CONFIG) {
		pr_err("%s Invalid Args, count: %d, pwr_settings: %p, power_info: %p",
			__func__, count, pwr_settings, power_info);
		return -EINVAL;
	}

	power_info->power_down_setting_size = 0;
	power_info->power_down_setting =
		(struct aon_sensor_power_setting *)
		kzalloc(sizeof(struct aon_sensor_power_setting) *
			MAX_POWER_CONFIG, GFP_KERNEL);
	if (!power_info->power_down_setting) {
		pr_err("%s power_down_setting kzalloc failed", __func__);
		return -ENOMEM;
	}
	pr_info("%s mm-kzalloc power_down_setting %p, size: %d",
		__func__, power_info->power_down_setting,
		sizeof(struct aon_sensor_power_setting) * MAX_POWER_CONFIG);
	for (i = 0; i < count; i++) {
		power_info->power_down_setting[i].seq_type = pwr_settings[i].config_type;
		power_info->power_down_setting[i].config_val = pwr_settings[i].config_value;
		power_info->power_down_setting[i].delay = pwr_settings[i].delayMs;
		pr_info("%s [%d], seq_type: %d, config_val: %ld, delay:%d", __func__, i,
			power_info->power_down_setting[i].seq_type,
			power_info->power_down_setting[i].config_val,
			power_info->power_down_setting[i].delay);
	}
	power_info->power_down_setting_size = count;
	return rc;
}

static struct i2c_settings_list *aon_sensor_get_i2c_ptr(
	struct i2c_settings_array *i2c_reg_settings, u32 size)
{
	struct i2c_settings_list *tmp = NULL;

	if (!i2c_reg_settings) {
		pr_err("%s i2c_reg_settings is null", __func__);
		return NULL;
	}

	tmp = (struct i2c_settings_list *)
		kzalloc(sizeof(struct i2c_settings_list), GFP_KERNEL);

	if (tmp != NULL) {
		pr_info("%s mm-kzalloc tmp %p, size: %d",
		__func__, tmp, sizeof(struct i2c_settings_list));
		list_add_tail(&(tmp->list),
			&(i2c_reg_settings->list_head));
	} else {
		pr_err("%s kzalloc failed", __func__);
		return NULL;
	}

	tmp->i2c_settings.reg_setting = (struct cam_sensor_i2c_reg_array *)
		vzalloc(size * sizeof(struct cam_sensor_i2c_reg_array));
	if (tmp->i2c_settings.reg_setting == NULL) {
		pr_err("%s vzalloc failed", __func__);
		list_del(&(tmp->list));
		pr_info("%s mm-kfree tmp %p", __func__, tmp);
		kfree(tmp);
		tmp = NULL;
		return NULL;
	}
	tmp->i2c_settings.size = size;
	pr_info("%s mm-vzalloc reg_setting %p, size: %d",
		__func__, tmp->i2c_settings.reg_setting,
		size * sizeof(struct cam_sensor_i2c_reg_array));

	return tmp;
}

static s32 aon_sensor_handle_random_write(struct i2c_rdwr_header hr,
	struct aon_reg_setting *configs,
	struct i2c_settings_array *i2c_reg_settings)
{
	struct i2c_settings_list *i2c_list = NULL;
	s32 rc = 0, cnt = 0;

	if (!configs || !i2c_reg_settings)
		return -EINVAL;

	i2c_list = aon_sensor_get_i2c_ptr(i2c_reg_settings, hr.count);
	if (i2c_list == NULL || i2c_list->i2c_settings.reg_setting == NULL) {
		pr_err("%s Failed in allocating i2c_list", __func__);
		return -ENOMEM;
	}

	i2c_list->op_code = CAM_SENSOR_I2C_WRITE_RANDOM;
	i2c_list->i2c_settings.addr_type = hr.addr_type;
	i2c_list->i2c_settings.data_type = hr.data_type;

	for (cnt = 0; cnt < hr.count; cnt++) {
		i2c_list->i2c_settings.reg_setting[cnt].reg_addr =
			configs[cnt].register_addr;
		i2c_list->i2c_settings.reg_setting[cnt].reg_data =
			configs[cnt].register_data;
		i2c_list->i2c_settings.reg_setting[cnt].data_mask = 0;
	}

	return rc;
}

static s32 aon_prepare_config_cmd(u32 op_code,
	struct aon_sensor_ctrl_t *s_ctrl, s32 count,
	struct aon_reg_setting *configs)
{
	s32 rc = 0;
	struct i2c_data_settings *i2c_data = NULL;
	struct i2c_settings_array *i2c_reg_settings = NULL;
	struct i2c_rdwr_header hr = {0};

	if (!s_ctrl || !configs || count <= 0) {
		pr_err("%s invalid param, s_ctrl: %p, configs: %p, count: %d",
			__func__, s_ctrl, configs, count);
		return -EINVAL;
	}

	i2c_data = &(s_ctrl->i2c_data);
	switch (op_code & 0xFFFFFF) {
	case AON_SENSOR_PACKET_OPCODE_INITIAL_CONFIG: {
		pr_info("%s AON_SENSOR_PACKET_OPCODE_INITIAL_CONFIG", __func__);
		i2c_reg_settings = &i2c_data->init_settings;
		i2c_reg_settings->is_settings_valid = 1;
		break;
	}
	case AON_SENSOR_PACKET_OPCODE_CONFIG: {
		pr_info("%s AON_SENSOR_PACKET_OPCODE_CONFIG", __func__);
		i2c_reg_settings = &i2c_data->config_settings;
		i2c_reg_settings->is_settings_valid = 1;
		break;
	}
	case AON_SENSOR_PACKET_OPCODE_STREAMON: {
		pr_info("%s AON_SENSOR_PACKET_OPCODE_STREAMON", __func__);
		if (s_ctrl->streamon_count > 0) {
			pr_info("%s s_ctrl->streamon_count > 0", __func__);
			return rc;
		}

		s_ctrl->streamon_count = s_ctrl->streamon_count + 1;
		i2c_reg_settings = &i2c_data->streamon_settings;
		i2c_reg_settings->is_settings_valid = 1;
		break;
	}
	case AON_SENSOR_PACKET_OPCODE_STREAMOFF: {
		pr_info("%s AON_SENSOR_PACKET_OPCODE_STREAMOFF", __func__);
		if (s_ctrl->streamoff_count > 0) {
			pr_info("%s s_ctrl->streamon_count > 0", __func__);
			return rc;
		}

		s_ctrl->streamoff_count = s_ctrl->streamoff_count + 1;
		i2c_reg_settings = &i2c_data->streamoff_settings;
		i2c_reg_settings->is_settings_valid = 1;
		break;
	}
	default:
		pr_err("%s not supported op_code", __func__);
		rc = -EINVAL;
		return rc;
	}

	hr.count = count;
	hr.data_type = configs[0].regData_type;
	hr.addr_type = configs[0].regAddr_type;

	rc = aon_sensor_handle_random_write(hr,
		configs, i2c_reg_settings);
	return rc;
}

static s32 aon_show_slaveinfo(struct aon_slave_info_data *pslave_info)
{
	s32 rc = 0;

	if (!pslave_info) {
		pr_err("%s pslave_info is null", __func__);
		return -EINVAL;
	}

	pr_info("aon slaveinfo: slaveaddr: 0x%x, sensor_id_regaddr: 0x%x,"
		" sensor_id: 0x%x, sensor_id_mask: 0x%x, "
		"i2c_frequency_mode: %d, reg_addrtype: %d, reg_datatype: %d\n",
		pslave_info->slave_address,
		pslave_info->sensor_id_regaddr,
		pslave_info->sensor_id,
		pslave_info->sensor_id_mask,
		(int)(pslave_info->i2c_frequency_mode),
		(int)(pslave_info->reg_addrtype),
		(int)(pslave_info->reg_datatype));

	return rc;
}

static s32 aon_process_cmd_release(struct aon_sensor_ctrl_t **pp_ctrl);

static s32 aon_process_cmd_probe(struct aon_sensor_ctrl_t **pp_ctrl,
	struct aon_sensor_cmd_buf_desc *buf_desc, void *buf_data)
{
	s32 rc = 0;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	bool isAuxSensor = 0;
	struct aon_reg_setting *paon_shift_reg_setting = NULL;
#endif
	struct aon_sensor_power_ctrl_t *power_info = NULL;
	struct aon_power_setting *paon_powerup_setting = NULL;
	struct aon_power_setting *paon_powerdown_setting = NULL;
	struct aon_cmd_i2c_info i2c_info = {0};
	struct aon_cmd_probe probe_info = {0};
	struct aon_slave_info_data *pslave_info = NULL;

	if (!pp_ctrl || !buf_desc || !buf_data) {
		pr_err("%s invalid param, pp_ctrl: %p, buf_desc: %p, buf_data: %p",
			__func__, pp_ctrl, buf_desc, buf_data);
		return -EINVAL;
	}

	pr_info("%s *pp_ctrl: %p", __func__, *pp_ctrl);
	if (!(*pp_ctrl)) {
		/*
		* pp_ctrl is actually a member of explorer driver private data,
		* we use it to determine whether aon has been initialized or not:
		*    NULL: has not been initialized
		*    not NULL: has been initialized
		*/
		pr_info("%s *pp_ctrl is null, to init...", __func__);
#ifdef OPLUS_FEATURE_CAMERA_COMMON
		if (buf_desc->slave_info_offset >= 0) {
			pslave_info = (struct aon_slave_info_data *)
				(buf_data + buf_desc->slave_info_offset);
			isAuxSensor = pslave_info->reserve;
			pr_info("%s slave_info isAuxSensor %d", __func__,(int)(pslave_info->reserve));
		}
#ifdef MTK_AON
		rc = explorer_aon_init(pp_ctrl);
#else
		rc = explorer_aon_init(pp_ctrl,isAuxSensor);
#endif

#else
		rc = explorer_aon_init(pp_ctrl);
#endif
		if (rc < 0 || !(*pp_ctrl)) {
			pr_err("%s explorer_aon_init failed, rc = %d", __func__, rc);
			return -1;
		}
	}

	mutex_lock(&((*pp_ctrl)->aon_sensor_mutex));

	if ((*pp_ctrl)->is_power_on == 1) {
		pr_info("%s Sensor already power on, to power down\n", __func__);
		rc = aon_sensor_power_down((*pp_ctrl));
		if (rc < 0) {
			pr_err("%s sensor failed to power down", __func__);
			goto free_power_settings;
		}
		(*pp_ctrl)->is_power_on = 0;
	}

	if (buf_desc->slave_info_offset >= 0) {
		pslave_info = (struct aon_slave_info_data *)
			(buf_data + buf_desc->slave_info_offset);
		rc = aon_show_slaveinfo(pslave_info);
		if (rc < 0)
			goto release_mutex;

		i2c_info.slave_addr = pslave_info->slave_address;
		i2c_info.i2c_freq_mode = pslave_info->i2c_frequency_mode;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
		i2c_info.sub_slave_addr = pslave_info->sub_slave_address;
#endif

		probe_info.data_type = pslave_info->reg_datatype;
		probe_info.addr_type = pslave_info->reg_addrtype;
		probe_info.reg_addr = pslave_info->sensor_id_regaddr;
		probe_info.expected_data = pslave_info->sensor_id;
		probe_info.data_mask = pslave_info->sensor_id_mask;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
		probe_info.needI2cSwitch = pslave_info->needI2cSwitch;
#endif
		rc = aon_sensor_update_i2c_info(&i2c_info, (*pp_ctrl));
		pr_info("%s aon_sensor_update_i2c_info rc=%d", __func__, rc);
		if (rc)
			goto release_mutex;

		rc = aon_sensor_update_slave_info(&probe_info, (*pp_ctrl));
		pr_info("%s aon_sensor_update_slave_info rc=%d", __func__, rc);
		if (rc)
			goto release_mutex;
	}

	if (buf_desc->powerup_setting_offset >= 0) {
		paon_powerup_setting = (struct aon_power_setting *)
			(buf_data + buf_desc->powerup_setting_offset);
		rc = aon_sensor_update_power_up_settings(
				buf_desc->powerup_setting_count,
				paon_powerup_setting,
				&((*pp_ctrl)->sensordata->power_info));
		if (rc) {
			pr_info("aon_sensor_update_power_up_settings failed");
			goto release_mutex;
		}
	}

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if (buf_desc->shiftmode_setting_offset >= 0 && buf_desc->shiftmode_setting_count > 0) {
		paon_shift_reg_setting = (struct aon_reg_setting *)
			(buf_data + buf_desc->shiftmode_setting_offset);
		rc = aon_sensor_update_shift_settings(
				buf_desc->shiftmode_setting_count,
				paon_shift_reg_setting,
				&((*pp_ctrl)->sensordata->shift_mode_setting));
		if (rc) {
			pr_info("aon_sensor_update_power_up_settings failed");
			goto release_mutex;
		}
	}
#endif

	if (buf_desc->powerdown_setting_offset >= 0) {
		paon_powerdown_setting = (struct aon_power_setting *)
			(buf_data + buf_desc->powerdown_setting_offset);
		rc = aon_sensor_update_power_down_settings(
				buf_desc->powerdown_setting_count,
				paon_powerdown_setting,
				&(*pp_ctrl)->sensordata->power_info);
		if (rc) {
			pr_info("aon_sensor_update_power_down_settings failed");
			pr_info("%s mm-kfree power_setting %p",
				__func__,
				(*pp_ctrl)->sensordata->power_info.power_setting);
			kfree((*pp_ctrl)->sensordata->power_info.power_setting);
			(*pp_ctrl)->sensordata->power_info.power_setting = NULL;
			(*pp_ctrl)->sensordata->power_info.power_setting_size = 0;
			goto release_mutex;
		}
	}

	/* Parse and fill vreg params for powerup settings */
	rc = aon_fill_vreg_params(
		&(*pp_ctrl)->soc_info,
		(*pp_ctrl)->sensordata->power_info.power_setting,
		(*pp_ctrl)->sensordata->power_info.power_setting_size);
	if (rc < 0) {
		pr_err("%s aon_fill_vreg_params for power on rc: %d", __func__, rc);
		goto free_power_settings;
	}

	/* Parse and fill vreg params for powerdown settings*/
	rc = aon_fill_vreg_params(
		&(*pp_ctrl)->soc_info,
		(*pp_ctrl)->sensordata->power_info.power_down_setting,
		(*pp_ctrl)->sensordata->power_info.power_down_setting_size);
	if (rc < 0) {
		pr_err("%s aon_fill_vreg_params for power down rc: %d", __func__, rc);
		goto free_power_settings;
	}

	/* Power up and probe sensor */
	rc = aon_sensor_power_up((*pp_ctrl));
	if (rc < 0) {
		pr_err("%s aon_sensor_power_up failed", __func__);
		goto free_power_settings;
	} else {
		pr_info("%s aon_sensor_power_up succeeded",  __func__);
		(*pp_ctrl)->is_power_on = 1;
	}

	/* Match sensor ID */
	rc = aon_sensor_match_id((*pp_ctrl));
	if (rc < 0) {
		aon_sensor_power_down((*pp_ctrl));
		(*pp_ctrl)->is_power_on = 0;
		msleep(20);
		goto free_power_settings;
	}

	pr_info("%s Probe succeeded, slave_addr:0x%x, sensor_id:0x%x",
		__func__,
		(*pp_ctrl)->sensordata->slave_info.sensor_slave_addr,
		(*pp_ctrl)->sensordata->slave_info.sensor_id);

	rc = aon_sensor_power_down((*pp_ctrl));
	if (rc < 0) {
		pr_err("%s aon_sensor_power_down, rc: %d", __func__, rc);
		goto free_power_settings;
	}

	(*pp_ctrl)->is_probe_succeed = 1;
	(*pp_ctrl)->is_power_on = 0;
	(*pp_ctrl)->sensor_state = AON_SENSOR_INIT;
	goto release_mutex;

free_power_settings:
	power_info = &((*pp_ctrl)->sensordata->power_info);
	pr_info("%s mm-kfree power_setting %p",
		__func__, power_info->power_setting);
	kfree(power_info->power_setting);
	power_info->power_setting = NULL;
	power_info->power_setting_size = 0;

	pr_info("%s mm-kfree power_setting %p",
		__func__, power_info->power_down_setting);
	kfree(power_info->power_down_setting);
	power_info->power_down_setting = NULL;
	power_info->power_down_setting_size = 0;

release_mutex:
	if ((*pp_ctrl))
		mutex_unlock(&((*pp_ctrl)->aon_sensor_mutex));

	/*to release aon device if probe failed*/
	if (rc < 0) {
		if (aon_process_cmd_release(pp_ctrl) < 0)
			pr_info("%s aon_process_cmd_release failed", __func__);
	}
	return rc;
}

static s32 aon_process_cmd_acquire(struct aon_sensor_ctrl_t *ctrl)
{
	s32 rc = 0;

	if (!ctrl) {
		pr_err("%s, ctrl is null", __func__);
		return -EINVAL;
	}
	mutex_lock(&(ctrl->aon_sensor_mutex));

	if ((ctrl->is_probe_succeed == 0) ||
		(ctrl->sensor_state != AON_SENSOR_INIT)) {
		pr_err("Not in right state to aquire %d", ctrl->sensor_state);
		rc = -EINVAL;
		goto release_mutex;
	}

	rc = aon_sensor_power_up(ctrl);
	if (rc < 0) {
		pr_err("Sensor Power up failed");
		goto release_mutex;
	}
	ctrl->is_power_on = 1;
	ctrl->sensor_state = AON_SENSOR_ACQUIRE;

release_mutex:
	if (ctrl)
		mutex_unlock(&(ctrl->aon_sensor_mutex));
	return rc;
}

static s32 aon_process_cmd_config(struct aon_sensor_ctrl_t *ctrl,
	struct aon_sensor_cmd_buf_desc *buf_desc, void *buf_data)
{
	s32 rc = 0;
	s32 cnt = 0;
	struct aon_reg_setting *paon_reg_setting = NULL;
	u32 op_code = AON_SENSOR_PACKET_OPCODE_NOP;

	if (!ctrl || (ctrl->sensor_state == AON_SENSOR_INIT)) {
		pr_err("%s, ctrl is null", __func__);
		return -EINVAL;
	}
	mutex_lock(&(ctrl->aon_sensor_mutex));

	/* there is only one config valid at one time */
	if (buf_desc->is_init_config_valid) {
		op_code = AON_SENSOR_PACKET_OPCODE_INITIAL_CONFIG;
		pr_info("%s INITIAL CONFIG", __func__);
		cnt++;
	}
	if (buf_desc->is_res_config_valid) {
		op_code = AON_SENSOR_PACKET_OPCODE_CONFIG;
		pr_info("%s RES CONFIG", __func__);
		cnt++;
	}
	if (buf_desc->is_streamon_config_valid) {
		op_code = AON_SENSOR_PACKET_OPCODE_STREAMON;
		pr_info("%s STREAMON CONFIG", __func__);
		cnt++;
	}
	if (buf_desc->is_streamoff_config_valid) {
		op_code = AON_SENSOR_PACKET_OPCODE_STREAMOFF;
		pr_info("%s STREAMOFF CONFIG", __func__);
		cnt++;
	}
	if (cnt != 1) {
		pr_err("%s %s is_xx_config_valid", __func__,
			(cnt > 1) ? "too much" : "no");
		rc = -EINVAL;
		goto release_mutex;
	}
	paon_reg_setting = (struct aon_reg_setting *)
		(buf_data + buf_desc->config_regsetting_offset);
	rc = aon_prepare_config_cmd(op_code, ctrl,
		buf_desc->config_regsetting_count, paon_reg_setting);
	if (rc < 0) {
		pr_err("%s aon_prepare_config_cmd failed", __func__);
		goto release_mutex;
	}

	if (ctrl->i2c_data.init_settings.is_settings_valid) {
		rc = aon_sensor_apply_settings(ctrl,
			AON_SENSOR_PACKET_OPCODE_INITIAL_CONFIG);

		if (rc < 0) {
			pr_err("cannot apply init settings");
			delete_request(&ctrl->i2c_data.init_settings);
			goto release_mutex;
		}
		rc = delete_request(&ctrl->i2c_data.init_settings);
		if (rc < 0) {
			pr_err("Fail in deleting the Init settings");
			goto release_mutex;
		}
	}

	if (ctrl->i2c_data.config_settings.is_settings_valid) {
		pr_info("%s to config resolution settings", __func__);
		rc = aon_sensor_apply_settings(ctrl,
			AON_SENSOR_PACKET_OPCODE_CONFIG);

		if (rc < 0) {
			pr_err("cannot apply config settings");
			delete_request(&ctrl->i2c_data.config_settings);
			goto release_mutex;
		}
		rc = delete_request(&ctrl->i2c_data.config_settings);
		if (rc < 0) {
			pr_err("Fail in deleting the config settings");
			goto release_mutex;
		}
		if (ctrl->sensor_state == AON_SENSOR_START) {
			pr_info("%s keep AON_SENSOR_START after config cmd", __func__);
		} else {
			pr_info("%s enter state AON_SENSOR_CONFIG", __func__);
			ctrl->sensor_state = AON_SENSOR_CONFIG;
		}
	}
release_mutex:
	if (ctrl)
		mutex_unlock(&(ctrl->aon_sensor_mutex));
	return rc;
}

static s32 aon_process_cmd_start(struct aon_sensor_ctrl_t *ctrl)
{
	s32 rc = 0;

	if (!ctrl) {
		pr_err("%s, ctrl is null", __func__);
		return -EINVAL;
	}

	mutex_lock(&(ctrl->aon_sensor_mutex));
	if ((ctrl->sensor_state == AON_SENSOR_INIT) ||
		(ctrl->sensor_state == AON_SENSOR_START)) {
		rc = -EINVAL;
		pr_err("%s Not in right state to start : %d", __func__,
		ctrl->sensor_state);
		goto release_mutex;
	}

	if (ctrl->i2c_data.streamon_settings.is_settings_valid) {
		pr_info("%s to apply streamon setting", __func__);
		rc = aon_sensor_apply_settings(ctrl,
			AON_SENSOR_PACKET_OPCODE_STREAMON);
		if (rc < 0) {
			pr_err("aon cannot apply streamon settings");
			goto release_mutex;
		}
	}
	ctrl->sensor_state = AON_SENSOR_START;
	pr_info("%s START Success, sensor_id:0x%x,sensor_slave_addr:0x%x",
		__func__,
		ctrl->sensordata->slave_info.sensor_id,
		ctrl->sensordata->slave_info.sensor_slave_addr);

release_mutex:
	if (ctrl)
		mutex_unlock(&(ctrl->aon_sensor_mutex));
	return rc;
}

static s32 aon_process_cmd_stop(struct aon_sensor_ctrl_t *ctrl)
{
	s32 rc = 0;

	if (!ctrl) {
		pr_err("%s, ctrl is null", __func__);
		return -EINVAL;
	}
	mutex_lock(&(ctrl->aon_sensor_mutex));
	if (ctrl->sensor_state != AON_SENSOR_START) {
		rc = -EINVAL;
		pr_err("%s Not in right state to stop : %d", __func__,
		ctrl->sensor_state);
		goto release_mutex;
	}

	if (ctrl->i2c_data.streamoff_settings.is_settings_valid) {
		pr_info("%s to apply streamoff setting", __func__);
		rc = aon_sensor_apply_settings(ctrl,
			AON_SENSOR_PACKET_OPCODE_STREAMOFF);
		if (rc < 0)
			pr_err("%s cannot apply streamoff settings, rc = %d", __func__, rc);
	}

	ctrl->sensor_state = AON_SENSOR_ACQUIRE;
	pr_info("%s STOP Success, sensor_id:0x%x,sensor_slave_addr:0x%x",
		__func__,
		ctrl->sensordata->slave_info.sensor_id,
		ctrl->sensordata->slave_info.sensor_slave_addr);

release_mutex:
	if (ctrl)
		mutex_unlock(&(ctrl->aon_sensor_mutex));
	return rc;
}

static s32 aon_process_cmd_release(struct aon_sensor_ctrl_t **pp_ctrl)
{
	s32 rc = 0;

	if (!(*pp_ctrl)) {
		pr_err("%s, s_ctrl is already null", __func__);
		return rc;
	}

	mutex_lock(&((*pp_ctrl)->aon_sensor_mutex));

	if ((*pp_ctrl)->is_power_on == 1) {
		pr_info("%s, to power down", __func__);
		rc = aon_sensor_power_down((*pp_ctrl));
		if (rc < 0) {
			pr_err("%s Sensor Power Down failed", __func__);
			goto release_mutex;
		}
		(*pp_ctrl)->is_power_on = 0;
		pr_info("%s, power down OK", __func__);
	}

	aon_sensor_release_stream_resource(&((*pp_ctrl)->i2c_data));

	(*pp_ctrl)->sensor_state = AON_SENSOR_INIT;
	pr_info("%s RELEASE Success, sensor_id:0x%x, sensor_slave_addr:0x%x",
		__func__,
		(*pp_ctrl)->sensordata->slave_info.sensor_id,
		(*pp_ctrl)->sensordata->slave_info.sensor_slave_addr);
	(*pp_ctrl)->streamon_count = 0;
	(*pp_ctrl)->streamoff_count = 0;

	aon_dinit_variables();
	if ((*pp_ctrl))
		mutex_unlock(&((*pp_ctrl)->aon_sensor_mutex));
	explorer_aon_exit(pp_ctrl);
	return rc;

release_mutex:
	if ((*pp_ctrl))
		mutex_unlock(&((*pp_ctrl)->aon_sensor_mutex));
	return rc;
}

static s32 aon_sensor_read_register(struct aon_sensor_ctrl_t *s_ctrl,
	struct aon_sensor_cmd_buf_desc *buf_desc,
	struct aon_reg_setting *reg_settings)
{
	s32 rc = 0;
	s32 i = 0;
	u32 *regs_val_buf = NULL;
	s32 count = 0;

	if (!s_ctrl || !buf_desc || !reg_settings) {
		pr_err("%s invalid param", __func__);
		return -EINVAL;
	}

	count = buf_desc->read_regs_count;
	if (count <= 0) {
		pr_err("%s register count = %d", __func__, count);
		return -EINVAL;
	}

	regs_val_buf = (u32 *)
		kzalloc(count * sizeof(u32), GFP_KERNEL);
	if (!regs_val_buf) {
		pr_err("%s, alloc aon buffer data memory failed", __func__);
		return -ENOMEM;
	}
	pr_info("%s mm-kzalloc regs_val_buf: %p, size: %d",
		__func__, regs_val_buf, count * sizeof(u32));

#ifdef QCOM_AON
	rc = aon_cci_init(s_ctrl);
	if (rc < 0) {
		pr_err("%s aon_cci_init failed, rc: %d", __func__, rc);
		return rc;
	}
#endif

	for (i = 0; i < count; i++) {
		rc = aon_io_read(s_ctrl,
			reg_settings[i].register_addr,
			&regs_val_buf[i],
			reg_settings[i].regAddr_type,
			reg_settings[i].regData_type);
		if (rc < 0) {
			pr_err("%s aon_io_read error, return code %d", __func__, rc);
			goto out;
		}
		pr_debug("%s, register addr %d vals %d", __func__,
			reg_settings[i].register_addr,
			regs_val_buf[i]);
	}

	rc = copy_to_user((char __user *)(buf_desc->read_vals_addr),
		regs_val_buf, count * sizeof(u32));
	if (rc)
		pr_err("%s, copy_to_user failed, ret = 0x%x", __func__, rc);

out:
#ifdef QCOM_AON
	if (aon_cci_release(s_ctrl) < 0) {
		pr_err("%s aon_cci_release failed", __func__);
		rc = -1;
	}
#endif

	pr_info("%s mm-kfree regs_val_buf: %p", __func__, regs_val_buf);
	kfree(regs_val_buf);
	regs_val_buf = NULL;
	return rc;
}

static s32 aon_process_cmd_readreg(struct aon_sensor_ctrl_t *ctrl,
	struct aon_sensor_cmd_buf_desc *buf_desc, void *buf_data)
{
	s32 rc = 0;
	struct aon_reg_setting *paon_reg_setting = NULL;

	if (!ctrl || !buf_desc || !buf_data) {
		pr_err("%s, invalid params, ctrl: %p, buf_desc: %p, buf_data: %p",
			__func__, ctrl, buf_desc, buf_data);
		return -EINVAL;
	}

	mutex_lock(&(ctrl->aon_sensor_mutex));

	if (buf_desc->read_regs_offset >= 0) {
		paon_reg_setting = (struct aon_reg_setting *)
			(buf_data + buf_desc->read_regs_offset);

		rc = aon_sensor_read_register(ctrl, buf_desc, paon_reg_setting);
		if (rc) {
			pr_err("%s aon_sensor_read_register error", __func__);
			goto release_mutex;
		}
	} else {
		pr_err("%s read regs offset has wrong value %d", __func__,
			buf_desc->read_regs_offset);
		goto release_mutex;
	}

release_mutex:
	if (ctrl)
		mutex_unlock(&(ctrl->aon_sensor_mutex));
	return rc;
}

static s32 aon_process_cmd_opmclk(struct aon_sensor_ctrl_t *ctrl,
	struct aon_sensor_cmd_buf_desc *buf_desc)
{
	s32 rc = 0;

	if (!ctrl || !buf_desc) {
		pr_err("%s, invalid params, ctrl: %p, buf_desc: %p",
			__func__, ctrl, buf_desc);
		return -EINVAL;
	}
	pr_info("%s, AON_SENSOR_CMD_OPMCLK: %d", __func__, buf_desc->mclk_enabled);

	mutex_lock(&(ctrl->aon_sensor_mutex));

	if (buf_desc->mclk_enabled == 1) {
		rc = aon_enable_mclk(&(ctrl->sensordata->power_info),
			&(ctrl->soc_info));
		if (rc < 0) {
			pr_err("%s aon_enable_mclk failed, rc = %d", __func__, rc);
			goto release_mutex;
		}
	} else if (buf_desc->mclk_enabled == 0) {
		rc = aon_disable_mclk(&(ctrl->sensordata->power_info),
			&(ctrl->soc_info));
		if (rc < 0) {
			pr_err("%s aon_disable_mclk failed, rc = %d", __func__, rc);
			goto release_mutex;
		}
	} else if (buf_desc->mclk_enabled == 3) {
#ifdef MTK_AON
		rc = aon_enable_mclk_cur(&(ctrl->sensordata->power_info),
			&(ctrl->soc_info));
		if (rc < 0) {
			pr_err("%s aon_ensable_mclk_cur failed, rc = %d", __func__, rc);
			goto release_mutex;
		}
#endif
	} else if (buf_desc->mclk_enabled == 2) {
#ifdef MTK_AON
		rc = aon_disable_mclk_cur(&(ctrl->sensordata->power_info),
			&(ctrl->soc_info));
		if (rc < 0) {
			pr_err("%s aon_disable_mclk_cur failed, rc = %d", __func__, rc);
			goto release_mutex;
		}
#endif
	} else {
		pr_info("%s unsupported MCLK operation code", __func__);
	}

release_mutex:
	if (ctrl)
		mutex_unlock(&(ctrl->aon_sensor_mutex));
	return rc;
}

#ifdef MTK_AON
static void aon_free_power_setting(struct aon_sensor_power_ctrl_t *power_info)
{
	if (!power_info)
		return;

	pr_info("%s mm-kfree vcm_power_setting %p",
		__func__, power_info->vcm_power_setting);
	kfree(power_info->vcm_power_setting);
	power_info->vcm_power_setting = NULL;
	power_info->vcm_power_setting_size = 0;
}

static s32 aon_vcm_update_power_up_settings(s32 count,
	struct aon_power_setting *pwr_settings,
	struct aon_sensor_power_ctrl_t *power_info)
{
	s32 rc = 0;
	s32 i = 0;

	if (!pwr_settings || !power_info || count <= 0 || count > MAX_POWER_CONFIG) {
		pr_err("%s Invalid Args, count: %d, pwr_settings: %p, power_info: %p",
			__func__, count, pwr_settings, power_info);
		return -EINVAL;
	}

	if (power_info->vcm_power_setting) {
		pr_info("%s to clear previous VCM power setting", __func__);
		aon_free_power_setting(power_info);
	}

	power_info->vcm_power_setting_size = 0;
	power_info->vcm_power_setting =
		(struct aon_sensor_power_setting *)
		kzalloc(sizeof(struct aon_sensor_power_setting) *
			MAX_POWER_CONFIG, GFP_KERNEL);
	if (!power_info->vcm_power_setting) {
		pr_err("%s vcm_power_setting kzalloc failed", __func__);
		return -ENOMEM;
	}
	pr_info("%s mm-kzalloc vcm_power_setting %p, size: %d",
		__func__, power_info->vcm_power_setting,
		sizeof(struct aon_sensor_power_setting) * MAX_POWER_CONFIG);

	for (i = 0; i < count; i++) {
		power_info->vcm_power_setting[i].seq_type = pwr_settings[i].config_type;
		power_info->vcm_power_setting[i].config_val = pwr_settings[i].config_value;
		power_info->vcm_power_setting[i].delay = pwr_settings[i].delayMs;
		power_info->vcm_power_setting[i].seq_val = INVALID_VREG;
		pr_info("%s [%d], seq_type: %d, config_val: %ld, delay:%d", __func__, i,
			power_info->vcm_power_setting[i].seq_type,
			power_info->vcm_power_setting[i].config_val,
			power_info->vcm_power_setting[i].delay);
	}
	power_info->vcm_power_setting_size = count;
	return rc;
}

static s32 aon_process_cmd_opvcm(struct aon_sensor_ctrl_t *ctrl,
	struct aon_sensor_cmd_buf_desc *buf_desc, void *buf_data)
{
	s32 rc = 0;
	u8 on = 0;
	struct aon_power_setting *pvcm_pw_setting = NULL;

	if (!ctrl || !buf_desc || !ctrl->sensordata) {
		pr_err("%s, invalid params, ctrl: %p, buf_desc: %p",
			__func__, ctrl, buf_desc);
		return -EINVAL;
	}
	pr_info("%s, AON_SENSOR_CMD_OPVCM: %d", __func__, buf_desc->vcm_power_on);

	mutex_lock(&(ctrl->aon_sensor_mutex));

	on = !!buf_desc->vcm_power_on;
	pr_info("%s %s", __func__, (on == 1) ? "power on" : "power off");

	if (on) {
		if (ctrl->sensordata->power_info.vcm_power_setting &&
			ctrl->sensordata->power_info.vcm_is_on == 1) {
			pr_info("%s aon vcm is already on", __func__);
			rc = 0;
			goto release_mutex;
		}
		if (!buf_data) {
			pr_err("%s VCM power up info missing", __func__);
			rc = -EINVAL;
			goto release_mutex;
		}
		if (buf_desc->powerup_vcm_setting_offset >= 0) {
			pvcm_pw_setting = (struct aon_power_setting *)
				(buf_data + buf_desc->powerup_vcm_setting_offset);
			rc = aon_vcm_update_power_up_settings(
					buf_desc->powerup_vcm_setting_count,
					pvcm_pw_setting,
					&(ctrl->sensordata->power_info));
			if (rc) {
				pr_info("aon_sensor_update_power_up_settings failed");
				goto release_mutex;
			}
		} else {
			rc = -EINVAL;
			pr_err("%s VCM power setting offset invalid", __func__);
			goto release_mutex;
		}

		/* Parse and fill vreg params for powerup settings */
		rc = aon_fill_vreg_params(
			&ctrl->soc_info,
			ctrl->sensordata->power_info.vcm_power_setting,
			ctrl->sensordata->power_info.vcm_power_setting_size);
		if (rc < 0) {
			pr_err("%s aon_fill_vreg_params for vcm power on rc: %d", __func__, rc);
			aon_free_power_setting(&(ctrl->sensordata->power_info));
			goto release_mutex;
		}

		rc = aon_vcm_power_on(&(ctrl->sensordata->power_info),
			&(ctrl->soc_info));
		if (rc < 0) {
			pr_err("%s aon_fill_vreg_params for vcm power on rc: %d", __func__, rc);
			aon_free_power_setting(&(ctrl->sensordata->power_info));
		}
	} else {
		rc = aon_vcm_power_off(&(ctrl->sensordata->power_info),
			&(ctrl->soc_info));
		if (!rc)
			aon_free_power_setting(&(ctrl->sensordata->power_info));
	}
	pr_info("%s rc = %d", __func__, rc);

release_mutex:
	if (ctrl)
		mutex_unlock(&(ctrl->aon_sensor_mutex));
	return rc;
}

static s32 aon_process_cmd_get_eeprom_info(struct aon_sensor_ctrl_t *ctrl,
	struct aon_sensor_cmd_buf_desc *buf_desc, void *buf_data)
{
	s32 rc = 0;
	s32 i = 0;
	u32 *regs_val_buf = NULL;
	s32 count = 0;
	struct aon_slave_info_data *pslave_info = NULL;
	struct aon_cmd_i2c_info i2c_info = {0};
	struct aon_reg_setting *reg_settings = NULL;

	if (!ctrl || !buf_desc || !buf_data) {
		pr_err("%s, invalid params, ctrl: %p, buf_desc: %p",
			__func__, ctrl, buf_desc);
		return -EINVAL;
	}

	count = buf_desc->read_regs_count;
	if (count <= 0) {
		pr_err("%s register count = %d", __func__, count);
		return -EINVAL;
	}

	if (buf_desc->eeprom_slave_info_offset < 0 ||
		buf_desc->read_regs_offset < 0) {
		pr_info("%s invalid offset, eeprom_slave_info: %d, read_regs: %d",
			__func__,
			buf_desc->eeprom_slave_info_offset,
			buf_desc->read_regs_offset);
		return -EINVAL;
	}

	mutex_lock(&(ctrl->aon_sensor_mutex));

	pslave_info = (struct aon_slave_info_data *)
		(buf_data + buf_desc->eeprom_slave_info_offset);
	i2c_info.slave_addr = pslave_info->slave_address;
	i2c_info.i2c_freq_mode = pslave_info->i2c_frequency_mode;
	rc = aon_sensor_update_eeprom_i2c_info(&i2c_info, ctrl);
	if (rc)
		goto release_mutex;

	reg_settings = (struct aon_reg_setting *)
				(buf_data + buf_desc->read_regs_offset);
	if (!reg_settings) {
		pr_err("%s regs to read is null", __func__);
		rc = -EINVAL;
		goto release_mutex;
	}

	regs_val_buf = (u32 *)kzalloc(count * sizeof(u32), GFP_KERNEL);
	if (!regs_val_buf) {
		pr_err("%s, alloc aon buffer data memory failed", __func__);
		rc = -ENOMEM;
		goto release_mutex;
	}
	pr_info("%s mm-kzalloc regs_val_buf: %p, size: %d",
		__func__, regs_val_buf, count * sizeof(u32));

	for (i = 0; i < count; i++) {
		rc = aon_eeprom_io_read(ctrl,
			reg_settings[i].register_addr,
			&regs_val_buf[i],
			reg_settings[i].regAddr_type,
			reg_settings[i].regData_type);
		if (rc < 0) {
			pr_err("%s aon_io_read error, return code %d", __func__, rc);
			goto out;
		}
		pr_debug("%s, register addr %d vals %d", __func__,
			reg_settings[i].register_addr,
			regs_val_buf[i]);
	}

	rc = copy_to_user((char __user *)(buf_desc->read_vals_addr),
		regs_val_buf, count * sizeof(u32));
	if (rc)
		pr_err("%s, copy_to_user failed, ret = 0x%x", __func__, rc);
out:
	pr_info("%s mm-kfree regs_val_buf: %p", __func__, regs_val_buf);
	kfree(regs_val_buf);
	regs_val_buf = NULL;
release_mutex:
	mutex_unlock(&(ctrl->aon_sensor_mutex));
	return rc;
}
#endif

static s32 aon_show_bufdesc(struct aon_sensor_cmd_buf_desc *buf_desc)
{
	s32 rc = 0;

	if (!buf_desc) {
		pr_info("%s buf_desc is null", __func__);
		return -EINVAL;
	}

	pr_info("aon cmd buf desc: slave_offset: %d, "
		"powerup_count: %d, powerup_offset: %d, "
		"powerdown_count: %d, powerdown_offset: %d, "
		"readregs_offset: %d, readregs_count: %d, readvals_addr: %lld, "
		"init_valid: %u, streamon_valid: %u, streamoff_valid: %u, "
		"res_valid: %u, mclk_enabled: %u",
		buf_desc->slave_info_offset,
		buf_desc->powerup_setting_count,
		buf_desc->powerup_setting_offset,
		buf_desc->powerdown_setting_count,
		buf_desc->powerdown_setting_offset,
		buf_desc->read_regs_offset,
		buf_desc->read_regs_count,
		buf_desc->read_vals_addr,
		buf_desc->is_init_config_valid,
		buf_desc->is_streamon_config_valid,
		buf_desc->is_streamoff_config_valid,
		buf_desc->is_res_config_valid,
		buf_desc->mclk_enabled);
#ifdef MTK_AON
	pr_info("aon cmd buf desc: vcm_power_on: %u", buf_desc->vcm_power_on);
#endif
	return rc;
}

int explorer_aon_drv_cmd(void **pp_param, void *arg)
{
	struct aon_sensor_cmd *aon_data = NULL;
	struct aon_sensor_cmd_buf_desc *buf_desc = NULL;
	void *buf_data = NULL;
	s32 rc = 0;

	if (!arg || !pp_param) {
		pr_err("%s invalid args, pp_param: %p, arg: %p\n",
			__func__, pp_param, arg);
		return -EINVAL;
	}

	aon_data = (struct aon_sensor_cmd *)arg;
	buf_desc = &aon_data->aon_cmd_bufdesc;
	rc = aon_show_bufdesc(buf_desc);
	if (rc < 0)
		return rc;

	if (aon_data->aon_cmd_bufsize > 0) {
		buf_data = kzalloc(aon_data->aon_cmd_bufsize, GFP_KERNEL);
		if (!buf_data) {
			pr_err("%s, kzalloc aon buf_data memory failed.\n", __func__);
			return -ENOMEM;
		}
		pr_info("%s mm-kzalloc buf_data: %p, size: %ld",
			__func__, buf_data, aon_data->aon_cmd_bufsize);
		rc = copy_from_user(buf_data, (char __user *)(aon_data->aon_cmd_bufhandle),
			aon_data->aon_cmd_bufsize);
		if (rc) {
			pr_err("%s, copy_from_user failed, rc = 0x%x.\n", __func__, rc);
			pr_info("%s mm-kfree buf_data :%p", __func__, buf_data);
			kfree(buf_data);
			buf_data = NULL;
			return rc;
		}
	}

	switch (aon_data->aon_cmd_opcode) {
	case AON_SENSOR_CMD_PROBE:
		pr_info("%s, AON_SENSOR_CMD_PROBE.\n", __func__);
		rc = aon_process_cmd_probe((struct aon_sensor_ctrl_t **)pp_param,
			buf_desc, buf_data);
		if (rc < 0) {
			pr_info("%s aon_process_cmd_probe failed", __func__);
		}
		break;
	case AON_SENSOR_CMD_ACQUIRE:
		pr_info("%s, AON_SENSOR_CMD_ACQUIRE.\n", __func__);
		rc = aon_process_cmd_acquire((struct aon_sensor_ctrl_t *)(*pp_param));
		if (rc < 0)
			pr_info("%s aon_process_cmd_acquire failed", __func__);
		break;
	case AON_SENSOR_CMD_CONFIG:
		pr_info("%s, AON_SENSOR_CMD_CONFIG.\n", __func__);
		rc = aon_process_cmd_config((struct aon_sensor_ctrl_t *)(*pp_param),
			buf_desc, buf_data);
		if (rc < 0)
			pr_info("%s aon_process_cmd_config failed", __func__);
		break;
	case AON_SENSOR_CMD_START:
		pr_info("%s, AON_SENSOR_CMD_START.\n", __func__);
		rc = aon_process_cmd_start((struct aon_sensor_ctrl_t *)(*pp_param));
		if (rc < 0)
			pr_info("%s aon_process_cmd_start failed", __func__);
		break;
	case AON_SENSOR_CMD_STOP:
		pr_info("%s, AON_SENSOR_CMD_STOP.\n", __func__);
		rc = aon_process_cmd_stop((struct aon_sensor_ctrl_t *)(*pp_param));
		if (rc < 0)
			pr_info("%s aon_process_cmd_stop failed", __func__);
		break;
	case AON_SENSOR_CMD_RELEASE:
		pr_info("%s, AON_SENSOR_CMD_RELEASE.\n", __func__);
		rc = aon_process_cmd_release((struct aon_sensor_ctrl_t **)pp_param);
		if (rc < 0)
			pr_info("%s aon_process_cmd_release failed", __func__);
		break;
	case AON_SENSOR_CMD_READREG:
		pr_info("%s, AON_SENSOR_CMD_READREG.\n", __func__);
		rc = aon_process_cmd_readreg((struct aon_sensor_ctrl_t *)(*pp_param),
			buf_desc, buf_data);
		if (rc < 0)
			pr_info("%s aon_process_cmd_readreg failed", __func__);
		break;
	case AON_SENSOR_CMD_OPMCLK:
		pr_info("%s, AON_SENSOR_CMD_OPMCLK.\n", __func__);
		rc = aon_process_cmd_opmclk((struct aon_sensor_ctrl_t *)(*pp_param),
			buf_desc);
		break;
#ifdef MTK_AON
	case AON_SENSOR_CMD_VCM_POWER:
		pr_info("%s, AON_SENSOR_CMD_VCM_POWER.\n", __func__);
		rc = aon_process_cmd_opvcm((struct aon_sensor_ctrl_t *)(*pp_param),
			buf_desc, buf_data);
		break;
	case AON_SENSOR_CMD_GET_EEPROM_INFO:
		pr_info("%s, AON_SENSOR_CMD_GET_EEPROM_INFO.\n", __func__);
		rc = aon_process_cmd_get_eeprom_info((struct aon_sensor_ctrl_t *)(*pp_param),
			buf_desc, buf_data);
		break;
#endif
	default:
		pr_info("%s, unsupported cmd.\n", __func__);
		rc = -EINVAL;
		break;
	}
	pr_info("%s mm-kfree buf_data :%p", __func__, buf_data);
	kfree(buf_data);
	buf_data = NULL;
	return rc;
}
