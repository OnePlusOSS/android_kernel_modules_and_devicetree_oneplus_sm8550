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


#include "include/mtk_aon_sensor_core.h"

#define I2C_REG_MAX_BUF_SIZE 8

static int32_t aon_i2c_send_data(struct i2c_client *client,
	unsigned char *data,
	int length)
{
	s32 rc = 0;
	u16 saddr = client->addr >> 1;
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = data,
		 },
	};
	rc = i2c_transfer(client->adapter, msg, 1);
	if (rc < 0)
		pr_err("%s failed 0x%x", __func__, saddr);
	return rc;
}

static int32_t aon_i2c_write(struct i2c_client *client,
	struct cam_sensor_i2c_reg_array *reg_setting,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type)
{
	s32 rc = 0;
	u8 *buf = NULL;
	u8 len = 0;

	buf = kzalloc(I2C_REG_MAX_BUF_SIZE, GFP_KERNEL | GFP_DMA);
	if (!buf) {
		pr_err("%s Buffer memory allocation failed", __func__);
		return -ENOMEM;
	}
	/*if needed, change pr_debug to pr_info*/
	pr_debug("%s mm-kzalloc buf %p, size: %d", __func__,
		buf, I2C_REG_MAX_BUF_SIZE);

	pr_debug("%s reg addr = 0x%x addr type: %d", __func__,
			reg_setting->reg_addr, addr_type);
	if (addr_type == CAMERA_SENSOR_I2C_TYPE_BYTE) {
		buf[0] = reg_setting->reg_addr;
		pr_err("%s byte %d: 0x%x", __func__, len, buf[len]);
		len = 1;
	} else if (addr_type == CAMERA_SENSOR_I2C_TYPE_WORD) {
		buf[0] = reg_setting->reg_addr >> 8;
		buf[1] = reg_setting->reg_addr;
		pr_debug("%s byte %d: 0x%x", __func__, len, buf[len]);
		pr_debug("%s byte %d: 0x%x", __func__, len+1, buf[len+1]);
		len = 2;
	} else if (addr_type == CAMERA_SENSOR_I2C_TYPE_3B) {
		buf[0] = reg_setting->reg_addr >> 16;
		buf[1] = reg_setting->reg_addr >> 8;
		buf[2] = reg_setting->reg_addr;
		len = 3;
	} else if (addr_type == CAMERA_SENSOR_I2C_TYPE_DWORD) {
		buf[0] = reg_setting->reg_addr >> 24;
		buf[1] = reg_setting->reg_addr >> 16;
		buf[2] = reg_setting->reg_addr >> 8;
		buf[3] = reg_setting->reg_addr;
		len = 4;
	} else {
		pr_err("%s Invalid I2C addr type", __func__);
		rc = -EINVAL;
		goto free_buffer;
	}

	pr_debug("%s Data: 0x%x", __func__, reg_setting->reg_data);
	if (data_type == CAMERA_SENSOR_I2C_TYPE_BYTE) {
		buf[len] = reg_setting->reg_data;
		pr_debug("%s Byte %d: 0x%x", __func__, len, buf[len]);
		len += 1;
	} else if (data_type == CAMERA_SENSOR_I2C_TYPE_WORD) {
		buf[len] = reg_setting->reg_data >> 8;
		buf[len+1] = reg_setting->reg_data;
		pr_debug("%s Byte %d: 0x%x", __func__, len, buf[len]);
		pr_debug("%s Byte %d: 0x%x", __func__, len+1, buf[len+1]);
		len += 2;
	} else if (data_type == CAMERA_SENSOR_I2C_TYPE_3B) {
		buf[len] = reg_setting->reg_data >> 16;
		buf[len + 1] = reg_setting->reg_data >> 8;
		buf[len + 2] = reg_setting->reg_data;
		pr_debug("%s Byte %d: 0x%x", __func__, len, buf[len]);
		pr_debug("%s Byte %d: 0x%x", __func__, len+1, buf[len+1]);
		pr_debug("%s Byte %d: 0x%x", __func__, len+2, buf[len+2]);
		len += 3;
	} else if (data_type == CAMERA_SENSOR_I2C_TYPE_DWORD) {
		buf[len] = reg_setting->reg_data >> 24;
		buf[len + 1] = reg_setting->reg_data >> 16;
		buf[len + 2] = reg_setting->reg_data >> 8;
		buf[len + 3] = reg_setting->reg_data;
		pr_debug("%s Byte %d: 0x%x", __func__, len, buf[len]);
		pr_debug("%s Byte %d: 0x%x", __func__, len+1, buf[len+1]);
		pr_debug("%s Byte %d: 0x%x", __func__, len+2, buf[len+2]);
		pr_debug("%s Byte %d: 0x%x", __func__, len+3, buf[len+3]);
		len += 4;
	} else {
		pr_err("%s Invalid Data Type", __func__);
		rc = -EINVAL;
		goto free_buffer;
	}

	rc = aon_i2c_send_data(client, buf, len);
	if (rc < 0)
		pr_err("%s aon_i2c_send_data failed rc: %d", __func__, rc);

free_buffer:
	/*if needed, change pr_debug to pr_info*/
	pr_debug("%s mm-kfree buf %p", __func__, buf);
	kfree(buf);
	buf = NULL;
	return rc;
}

static s32 aon_sensor_i2c_modes_util(struct i2c_client *client,
	struct i2c_settings_list *i2c_list)
{
	s32 rc = 0, i;
	struct cam_sensor_i2c_reg_setting *write_setting = NULL;
	struct cam_sensor_i2c_reg_array *reg_setting = NULL;

	if (!client || !i2c_list)
		return -EINVAL;
	write_setting = &(i2c_list->i2c_settings);

	if (write_setting->addr_type <= CAMERA_SENSOR_I2C_TYPE_INVALID
		|| write_setting->addr_type >= CAMERA_SENSOR_I2C_TYPE_MAX
		|| write_setting->data_type <= CAMERA_SENSOR_I2C_TYPE_INVALID
		|| write_setting->data_type >= CAMERA_SENSOR_I2C_TYPE_MAX) {
		pr_err("%s invalid addr/data_type", __func__);
		return -EINVAL;
		}

	reg_setting = write_setting->reg_setting;
	for (i = 0; i < write_setting->size; i++) {
		pr_debug("%s addr 0x%x data 0x%x", __func__,
			reg_setting->reg_addr, reg_setting->reg_data);

		rc = aon_i2c_write(client, reg_setting,
			write_setting->addr_type, write_setting->data_type);
		if (rc < 0)
			break;
		reg_setting++;
	}

	if (write_setting->delay > 20)
		msleep(write_setting->delay);
	else if (write_setting->delay)
		usleep_range(write_setting->delay * 1000,
			(write_setting->delay * 1000) + 1000);
	
	return rc;
}

s32 aon_sensor_modes_util(struct aon_sensor_ctrl_t *s_ctrl,
	struct i2c_settings_list *i2c_list)
{
	s32 rc = 0;

	if (!s_ctrl || !s_ctrl->i2c_client)
		return -EINVAL;

	if (s_ctrl->master_type == I2C_MASTER) {
		rc = aon_sensor_i2c_modes_util(s_ctrl->i2c_client, i2c_list);
		if (rc < 0)
			pr_err("%s aon_sensor_i2c_modes_util failed", __func__);
	} else {
		rc = -EINVAL;
		pr_err("%s unsupported master type", __func__);
	}
	return rc;
}

static s32 aon_i2c_recv_data(struct i2c_client *client,
	unsigned char *data,
	enum camera_sensor_i2c_type addr_type,
	int data_length)
{
	s32 rc = 0;
	u16 saddr = client->addr >> 1;
	struct i2c_msg msgs[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len   = addr_type,
			.buf   = data,
		},
		{
			.addr  = saddr,
			.flags = I2C_M_RD,
			.len   = data_length,
			.buf   = data,
		},
	};
	rc = i2c_transfer(client->adapter, msgs, 2);
	if (rc < 0)
		pr_err("%s failed 0x%x", __func__, saddr);
	return rc;
}

s32 aon_io_read_inner(struct i2c_client *client,
		u32 addr, u32 *data,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type)
{
	s32 rc = -EINVAL;
	u8 *buf = NULL;

	if (!client || !data) {
		pr_err("%s client or data is null", __func__);
		return -EINVAL;
	}

	if (addr_type <= CAMERA_SENSOR_I2C_TYPE_INVALID
		|| addr_type >= CAMERA_SENSOR_I2C_TYPE_MAX
		|| data_type <= CAMERA_SENSOR_I2C_TYPE_INVALID
		|| data_type >= CAMERA_SENSOR_I2C_TYPE_MAX) {
		pr_err("%s invalid addr/data_type", __func__);
		return -EINVAL;
	}

	buf = kzalloc(addr_type + data_type, GFP_KERNEL);
	if (!buf) {
		pr_err("%s kzalloc failed", __func__);
		return -ENOMEM;
	}
	/*if needed, change pr_debug to pr_info*/
	pr_debug("%s mm-kzalloc buf %p, size: %d",
		__func__, buf, addr_type + data_type);

	if (addr_type == CAMERA_SENSOR_I2C_TYPE_BYTE) {
		buf[0] = addr;
	} else if (addr_type == CAMERA_SENSOR_I2C_TYPE_WORD) {
		buf[0] = addr >> 8;
		buf[1] = addr;
	} else if (addr_type == CAMERA_SENSOR_I2C_TYPE_3B) {
		buf[0] = addr >> 16;
		buf[1] = addr >> 8;
		buf[2] = addr;
	} else {
		buf[0] = addr >> 24;
		buf[1] = addr >> 16;
		buf[2] = addr >> 8;
		buf[3] = addr;
	}

	rc = aon_i2c_recv_data(client, buf, addr_type, data_type);
	if (rc < 0) {
		pr_err("%s failed rc: %d", __func__, rc);
		goto read_fail;
	}

	if (data_type == CAMERA_SENSOR_I2C_TYPE_BYTE)
		*data = buf[0];
	else if (data_type == CAMERA_SENSOR_I2C_TYPE_WORD)
		*data = buf[0] << 8 | buf[1];
	else if (data_type == CAMERA_SENSOR_I2C_TYPE_3B)
		*data = buf[0] << 16 | buf[1] << 8 | buf[2];
	else
		*data = buf[0] << 24 | buf[1] << 16 |
			buf[2] << 8 | buf[3];

	pr_debug("%s addr = 0x%x data: 0x%x", __func__, addr, *data);
read_fail:
	/*if needed, change pr_debug to pr_info*/
	pr_debug("%s mm-kfree buf %p", __func__, buf);
	kfree(buf);
	buf = NULL;
	return rc;
}

s32 aon_eeprom_io_read(struct aon_sensor_ctrl_t *s_ctrl,
		u32 addr, u32 *data,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type)
{
	s32 rc = -EINVAL;
	struct i2c_client *client = NULL;

	if (!s_ctrl)
		return rc;

	client = s_ctrl->eeprom_i2c_client;
	if (!client || (s_ctrl->master_type != I2C_MASTER)) {
		pr_err("%s Invalid Args", __func__);
		return -EINVAL;
	}

	return aon_io_read_inner(client, addr, data, addr_type, data_type);
}

s32 aon_io_read(struct aon_sensor_ctrl_t *s_ctrl,
		u32 addr, u32 *data,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type)
{
	s32 rc = -EINVAL;
	struct i2c_client *client = NULL;

	if (!s_ctrl)
		return rc;

	client = s_ctrl->i2c_client;
	if (!client || (s_ctrl->master_type != I2C_MASTER)) {
		pr_err("%s Invalid Args", __func__);
		return -EINVAL;
	}

	return aon_io_read_inner(client, addr, data, addr_type, data_type);
}

