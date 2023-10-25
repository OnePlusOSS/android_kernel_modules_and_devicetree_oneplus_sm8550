// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */

#include <linux/i2c.h>
#include "dsi_display.h"
#include "dsi_iris_api.h"
#include "dsi_iris_i2c.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_log.h"


#define IRIS_PURE_COMPATIBLE_NAME  "pixelworks,iris-i2c"
#define IRIS_PURE_I2C_DRIVER_NAME  "pixelworks-i2c"

/*iris i2c handle*/
static struct i2c_client  *iris_pure_i2c_handle;

int iris_pure_i2c_single_read(uint32_t addr, uint32_t *val)
{
	int ret = -1;
	uint8_t *w_data_list = NULL;
	uint8_t *r_data_list = NULL;
	struct i2c_msg msgs[2];

	if (!iris_pure_i2c_handle || !val) {
		IRIS_LOGE("%s, %d: the parameter is not right\n", __func__, __LINE__);
		return -EINVAL;
	}

	memset(msgs, 0, 2 * sizeof(msgs[0]));

	w_data_list = kmalloc(5, GFP_KERNEL);
	if (!w_data_list) {
		IRIS_LOGE("%s, %d: allocate memory fails\n", __func__, __LINE__);
		return -ENOMEM;
	}

	r_data_list = kmalloc(4, GFP_KERNEL);
	if (!r_data_list) {
		IRIS_LOGE("%s, %d: allocate memory fails\n", __func__, __LINE__);
		kfree(w_data_list);
		return -ENOMEM;
	}

	w_data_list[0] = 0xcc;
	w_data_list[1] = (addr >> 0) & 0xff;
	w_data_list[2] = (addr >> 8) & 0xff;
	w_data_list[3] = (addr >> 16) & 0xff;
	w_data_list[4] = (addr >> 24) & 0xff;

	r_data_list[0] = 0x00;
	r_data_list[1] = 0x00;
	r_data_list[2] = 0x00;
	r_data_list[3] = 0x00;

	msgs[0].addr = (iris_pure_i2c_handle->addr & 0xff);
	msgs[0].flags = 0;
	msgs[0].buf = w_data_list;
	msgs[0].len = 5;

	msgs[1].addr = (iris_pure_i2c_handle->addr & 0xff);
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = r_data_list;
	msgs[1].len = 4;

	ret = i2c_transfer(iris_pure_i2c_handle->adapter, &msgs[0], 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		IRIS_LOGE("%s, %d: i2c_transfer failed, write cmd, addr = 0x%08x, ret = %d\n",
			__func__, __LINE__, addr, ret);
	}

	if (ret == 0) {
		udelay(20);
		ret = i2c_transfer(iris_pure_i2c_handle->adapter, &msgs[1], 1);
		if (ret == 1) {
			ret = 0;
		} else {
			ret = ret < 0 ? ret : -EIO;
			IRIS_LOGE("%s, %d: i2c_transfer failed, read cmd, addr = 0x%08x, ret = %d\n",
				__func__, __LINE__, addr, ret);
		}
	}

	if (ret == 0) {
		*val = (r_data_list[0] << 24) |
			(r_data_list[1] << 16) |
			(r_data_list[2] << 8) |
			(r_data_list[3] << 0);
	}

	kfree(w_data_list);
	kfree(r_data_list);
	w_data_list = NULL;
	r_data_list = NULL;

	return ret;

}

int iris_pure_i2c_single_write(uint32_t addr, uint32_t val)
{

	int ret = 0;
	struct i2c_msg msg;
	uint8_t data_list[9] = {0};

	if (!iris_pure_i2c_handle) {
		IRIS_LOGE("%s, %d: the parameter is not right\n", __func__, __LINE__);
		return -EINVAL;
	}

	memset(&msg, 0, sizeof(msg));

	data_list[0] = 0xcc;
	data_list[1] = (addr >> 0) & 0xff;
	data_list[2] = (addr >> 8) & 0xff;
	data_list[3] = (addr >> 16) & 0xff;
	data_list[4] = (addr >> 24) & 0xff;
	data_list[5] = (val >> 24) & 0xff;
	data_list[6] = (val >> 16) & 0xff;
	data_list[7] = (val >> 8) & 0xff;
	data_list[8] = (val >> 0) & 0xff;

	msg.addr = (iris_pure_i2c_handle->addr & 0xff);
	msg.flags = 0;
	msg.buf = data_list;
	msg.len = 9;

	ret = i2c_transfer(iris_pure_i2c_handle->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		IRIS_LOGE("%s, %d: i2c_transfer failed, write cmd, addr = 0x%08x, ret = %d\n",
			__func__, __LINE__, addr, ret);
	}

	return ret;
}

int iris_pure_i2c_burst_write(uint32_t addr, uint32_t *val, uint32_t reg_num)
{

	int i;
	int ret = -1;
	u32 msg_len = 0;
	struct i2c_msg msg;
	uint8_t *data_list = NULL;

	if (!val || reg_num < 1 || !iris_pure_i2c_handle) {
		IRIS_LOGE("%s, %d: the parameter is not right\n", __func__, __LINE__);
		return -EINVAL;
	}

	memset(&msg, 0x00, sizeof(msg));

	msg_len = 5 + reg_num * 4;

	data_list = kmalloc(msg_len, GFP_KERNEL);
	if (data_list == NULL) {
		IRIS_LOGE("%s, %d: allocate memory fails\n", __func__, __LINE__);
		return -ENOMEM;
	}

	data_list[0] = 0xfc;
	data_list[1] = (addr >> 0) & 0xff;
	data_list[2] = (addr >> 8) & 0xff;
	data_list[3] = (addr >> 16) & 0xff;
	data_list[4] = (addr >> 24) & 0xff;

	for (i = 0; i < reg_num; i++) {
		data_list[i*4 + 5] = (val[i] >> 24) & 0xff;
		data_list[i*4 + 6] = (val[i] >> 16) & 0xff;
		data_list[i*4 + 7] = (val[i] >> 8) & 0xff;
		data_list[i*4 + 8] = (val[i] >> 0) & 0xff;
	}

	msg.addr = (iris_pure_i2c_handle->addr & 0xff);
	msg.flags = 0;
	msg.buf = data_list;
	msg.len = msg_len;

	ret = i2c_transfer(iris_pure_i2c_handle->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		IRIS_LOGE("%s, %d: i2c_transfer failed, write cmd, addr = 0x%08x, ret = %d\n",
			__func__, __LINE__, addr, ret);
	}

	kfree(data_list);
	data_list = NULL;
	return ret;

}

static int iris_pure_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	iris_pure_i2c_handle = client;
	IRIS_LOGI("%s,%d: %p\n", __func__, __LINE__, iris_pure_i2c_handle);
	return 0;
}

static int iris_pure_i2c_remove(struct i2c_client *client)
{
	iris_pure_i2c_handle = NULL;
	return 0;
}

static const struct i2c_device_id iris_pure_i2c_id_table[] = {
	{IRIS_PURE_I2C_DRIVER_NAME, 0},
	{},
};


static const struct of_device_id iris_pure_match_table[] = {
	{.compatible = IRIS_PURE_COMPATIBLE_NAME,},
	{ },
};

static struct i2c_driver plx_pure_i2c_driver = {
	.driver = {
		.name = IRIS_PURE_I2C_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = iris_pure_match_table,
	},
	.probe = iris_pure_i2c_probe,
	.remove =  iris_pure_i2c_remove,
	.id_table = iris_pure_i2c_id_table,
};


int iris_pure_i2c_bus_init(void)
{
	int ret;

	IRIS_LOGD("%s()\n", __func__);
	iris_pure_i2c_handle = NULL;
	ret = i2c_add_driver(&plx_pure_i2c_driver);
	if (ret != 0)
		IRIS_LOGE("iris pure i2c add driver fail: %d\n", ret);
	return 0;
}

void iris_pure_i2c_bus_exit(void)
{

	i2c_del_driver(&plx_pure_i2c_driver);
	iris_pure_i2c_remove(iris_pure_i2c_handle);
}
