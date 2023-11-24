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
#include <linux/dma-buf.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/clk.h>

#include "include/qcom_aon_sensor_core.h"

static struct aon_sensor_ctrl_t *g_ctrl = NULL;

static void aon_sensor_shutdown(struct aon_sensor_ctrl_t *s_ctrl)
{
	struct aon_sensor_power_ctrl_t *power_info = NULL;

	if (!s_ctrl) {
		pr_info("%s s_ctrl is already null, nothing to do.", __func__);
		return;
	}

	if ((s_ctrl->sensor_state == AON_SENSOR_INIT) &&
		(s_ctrl->is_probe_succeed == 0)) {
		pr_info("stat is AON_SENSOR_INIT && is_probe_succeed == 0");
		return;
	}

	aon_sensor_release_stream_resource(&(s_ctrl->i2c_data));

	if (s_ctrl->sensor_state != AON_SENSOR_INIT)
		aon_sensor_power_down(s_ctrl);

	power_info = &s_ctrl->sensordata->power_info;
	pr_info("%s mm-kfree power_info->power_setting %p",
		__func__, power_info->power_setting);
	kfree(power_info->power_setting);
	power_info->power_setting = NULL;
	power_info->power_setting_size = 0;

	pr_info("%s mm-kfree power_info->power_down_setting %p",
		__func__, power_info->power_down_setting);
	kfree(power_info->power_down_setting);
	power_info->power_down_setting = NULL;
	power_info->power_down_setting_size = 0;

	s_ctrl->streamon_count = 0;
	s_ctrl->streamoff_count = 0;
	s_ctrl->is_probe_succeed = 0;
	s_ctrl->is_power_on = 0;
	s_ctrl->sensor_state = AON_SENSOR_INIT;
}

static void aon_sensor_platform_remove(void)
{
	s32 i = 0;
	struct aon_sensor_ctrl_t *s_ctrl = NULL;
	struct aon_hw_soc_info *soc_info = NULL;

	s_ctrl = g_ctrl;
	if (!s_ctrl) {
		pr_err("%s aon sensor device is NULL, nothing to do.", __func__);
		return;
	}

	mutex_lock(&(s_ctrl->aon_sensor_mutex));
	aon_sensor_shutdown(s_ctrl);

	pr_info("%s mm-kfree sensordata->power_info.gpio_num_info: %p",
		__func__, s_ctrl->sensordata->power_info.gpio_num_info);
	kfree(s_ctrl->sensordata->power_info.gpio_num_info);
	s_ctrl->sensordata->power_info.gpio_num_info = NULL;

	pr_info("%s mm-kfree s_ctrl->sensordata: %p",
		__func__, s_ctrl->sensordata);
	kfree(s_ctrl->sensordata);
	s_ctrl->sensordata = NULL;

	soc_info = &s_ctrl->soc_info;
	for (i = 0; i < soc_info->num_clk; i++)
		devm_clk_put(soc_info->dev, soc_info->clk[i]);
	pr_info("%s after devm_clk_put", __func__);

	if (soc_info->gpio_data) {
		pr_info("%s mm-kfree soc_info->gpio_data->cam_gpio_common_tbl: %p",
			__func__, soc_info->gpio_data->cam_gpio_common_tbl);
		kfree(soc_info->gpio_data->cam_gpio_common_tbl);
		soc_info->gpio_data->cam_gpio_common_tbl = NULL;
		soc_info->gpio_data->cam_gpio_common_tbl_size = 0;

		pr_info("%s mm-kfree gpio_data->cam_gpio_req_tbl: %p",
			__func__, soc_info->gpio_data->cam_gpio_req_tbl);
		kfree(soc_info->gpio_data->cam_gpio_req_tbl);
		soc_info->gpio_data->cam_gpio_req_tbl = NULL;
		soc_info->gpio_data->cam_gpio_req_tbl_size = 0;

		pr_info("%s mm-kfree soc_info->gpio_data: %p",
			__func__, soc_info->gpio_data);
		kfree(soc_info->gpio_data);
		soc_info->gpio_data = NULL;
	}

	pr_info("%s mm-kfree s_ctrl->cci_client: %p",
		__func__, s_ctrl->cci_client);
	kfree(s_ctrl->cci_client);
	s_ctrl->cci_client = NULL;
	mutex_unlock(&(s_ctrl->aon_sensor_mutex));

	pr_info("%s s_ctrl mm-kfree: %p", __func__, s_ctrl);
	kfree(s_ctrl);
	s_ctrl = NULL;
	g_ctrl = s_ctrl;
}

#ifdef OPLUS_FEATURE_CAMERA_COMMON
static s32 aon_sensor_driver_platform_probe(bool isAuxSensor)
#else
static s32 aon_sensor_driver_platform_probe(void)
#endif
{
	s32 rc = 0;
	struct aon_sensor_ctrl_t *s_ctrl = NULL;
	struct aon_hw_soc_info *soc_info = NULL;

	s_ctrl = kzalloc(sizeof(struct aon_sensor_ctrl_t), GFP_KERNEL);
	if (!s_ctrl) {
		pr_err("%s kzalloc failed", __func__);
		return -ENOMEM;
	}
	pr_info("%s mm-kzalloc s_ctrl %p, size: %d",
		__func__, s_ctrl, sizeof(struct aon_sensor_ctrl_t));

	s_ctrl->master_type = CCI_MASTER;
	s_ctrl->is_probe_succeed = 0;
	s_ctrl->is_power_on = 0;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	s_ctrl->isAuxSensor = isAuxSensor;
#endif

	rc = aon_sensor_parse_dt(s_ctrl);
	if (rc < 0) {
		pr_err("aon failed: cam_sensor_parse_dt rc %d", rc);
		goto free_s_ctrl;
	}

	/* Fill platform device id*/
	soc_info = &s_ctrl->soc_info;
	s_ctrl->pdev->id = soc_info->index;

	INIT_LIST_HEAD(&(s_ctrl->i2c_data.init_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.config_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.streamon_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.streamoff_settings.list_head));

	if (s_ctrl->sensordata)
		s_ctrl->sensordata->power_info.dev = &s_ctrl->pdev->dev;
	s_ctrl->sensor_state = AON_SENSOR_INIT;

	g_ctrl = s_ctrl;

	return rc;
free_s_ctrl:
	pr_info("%s mm-kfree s_ctrl: %p", __func__, s_ctrl);
	kfree(s_ctrl);
	s_ctrl = NULL;
	g_ctrl = s_ctrl;
	return rc;
}

#ifdef OPLUS_FEATURE_CAMERA_COMMON
int explorer_aon_init(struct aon_sensor_ctrl_t **ctrl,bool isAuxSensor)
#else
int explorer_aon_init(struct aon_sensor_ctrl_t **ctrl)
#endif
{
	int rc = 0;

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	rc = aon_sensor_driver_platform_probe(isAuxSensor);
#else
	rc = aon_sensor_driver_platform_probe();
#endif
	*ctrl = g_ctrl;

	return rc;
}

void explorer_aon_exit(struct aon_sensor_ctrl_t **ctrl)
{
	aon_sensor_platform_remove();
	*ctrl = g_ctrl;
}
