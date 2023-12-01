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
#include <linux/of.h>
#include <linux/module.h>
#include <linux/clk.h>

#include "include/mtk_aon_sensor_core.h"

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

	pr_info("%s mm-kfree power_info->vcm_power_setting %p",
		__func__, power_info->vcm_power_setting);
	kfree(power_info->vcm_power_setting);
	power_info->vcm_power_setting = NULL;
	power_info->vcm_power_setting_size = 0;

	s_ctrl->streamon_count = 0;
	s_ctrl->streamoff_count = 0;
	s_ctrl->is_probe_succeed = 0;
	s_ctrl->is_power_on = 0;
	s_ctrl->sensor_state = AON_SENSOR_INIT;
}

static s32 aon_sensor_driver_i2c_remove(struct i2c_client *client)
{
	int i;
	struct aon_sensor_ctrl_t *s_ctrl = i2c_get_clientdata(client);
	struct aon_hw_soc_info *soc_info = NULL;

	if (!s_ctrl) {
		pr_err("%s aon sensor device is NULL, nothing to do.", __func__);
		return 0;
	}

	pr_info("%s i2c remove invoked", __func__);
	mutex_lock(&(s_ctrl->aon_sensor_mutex));
	aon_sensor_shutdown(s_ctrl);

	pr_info("%s mm-kfree s_ctrl->sensordata:%p",
		__func__, s_ctrl->sensordata);
	kfree(s_ctrl->sensordata);
	s_ctrl->sensordata = NULL;

	soc_info = &s_ctrl->soc_info;
	for (i = 0; i < soc_info->num_clk; i++)
		devm_clk_put(soc_info->dev, soc_info->clk[i]);
	pr_info("%s after devm_clk_put", __func__);

	pr_info("%s mm-kfree s_ctrl->eeprom_i2c_client: %p",
		__func__, s_ctrl->eeprom_i2c_client);
	kfree(s_ctrl->eeprom_i2c_client);
	s_ctrl->eeprom_i2c_client = NULL;
	mutex_unlock(&(s_ctrl->aon_sensor_mutex));

	kfree(s_ctrl);
	pr_info("%s mm-kfree s_ctrl: %p", __func__, s_ctrl);
	s_ctrl = NULL;
	g_ctrl = s_ctrl;

	return 0;
}

static s32 aon_sensor_driver_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	s32 rc = 0;
	struct aon_sensor_ctrl_t *s_ctrl = NULL;
	struct aon_hw_soc_info *soc_info = NULL;

	pr_info("%s begin", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s %s :i2c_check_functionality failed", __func__, client->name);
		return -EFAULT;
	}

	/* Create sensor control structure */
	s_ctrl = kzalloc(sizeof(*s_ctrl), GFP_KERNEL);
	if (!s_ctrl) {
		pr_err("%s kzalloc failed", __func__);
		return -ENOMEM;
	}
	pr_info("%s mm-kzalloc s_ctrl %p, size: %d", __func__,
		s_ctrl, sizeof(*s_ctrl));

	s_ctrl->i2c_client = client;
	soc_info = &s_ctrl->soc_info;
	soc_info->dev = &client->dev;
	soc_info->dev_name = client->name;
	i2c_set_clientdata(client, s_ctrl);

	/* Initialize eeprom_i2c_client */
	s_ctrl->eeprom_i2c_client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!(s_ctrl->eeprom_i2c_client)) {
		pr_err("%s kzalloc failed", __func__);
		rc = -ENOMEM;
		goto free_s_ctrl;
	}
	pr_info("%s s_ctrl->eeprom_i2c_client: %p mm-kzalloc-size: %d",
		__func__, s_ctrl->eeprom_i2c_client, sizeof(struct i2c_client));
	memcpy(s_ctrl->eeprom_i2c_client, s_ctrl->i2c_client, sizeof(struct i2c_client));

	/* Initialize sensor device type */
	s_ctrl->of_node = client->dev.of_node;
	s_ctrl->master_type = I2C_MASTER;
	s_ctrl->is_probe_succeed = 0;
	s_ctrl->is_power_on = 0;

	rc = aon_sensor_parse_dt_mtk(s_ctrl);
	if (rc < 0) {
		pr_err("aon failed: cam_sensor_parse_dt rc %d", rc);
		goto free_s_ctrl;
	}

	INIT_LIST_HEAD(&(s_ctrl->i2c_data.init_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.config_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.streamon_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.streamoff_settings.list_head));

	if (s_ctrl->sensordata)
		s_ctrl->sensordata->power_info.dev = soc_info->dev;
	s_ctrl->sensor_state = AON_SENSOR_INIT;

	g_ctrl = s_ctrl;

	return rc;
free_s_ctrl:
	pr_info("%s mm-kfree s_ctrl %p", __func__, s_ctrl);
	kfree(s_ctrl);
	s_ctrl = NULL;
	g_ctrl = s_ctrl;
	return rc;
}

static struct of_device_id i2c_of[] = {
	{.compatible = "zeku,aon-sensor"},
	{}
};

static const struct i2c_device_id i2c_id[] = {
	{"i2c_aon_sensor", (kernel_ulong_t)NULL},
	{}
};
MODULE_DEVICE_TABLE(i2c, i2c_id);

static struct i2c_driver aon_sensor_driver_i2c = {
	.id_table = i2c_id,
	.probe = aon_sensor_driver_i2c_probe,
	.remove = aon_sensor_driver_i2c_remove,
	.driver = {
		.name = "i2c_aon_sensor",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(i2c_of),
	}
};

int explorer_aon_init(struct aon_sensor_ctrl_t **ctrl)
{
	int rc = 0;

	pr_info("%s before i2c_add_driver", __func__);
	rc = i2c_add_driver(&aon_sensor_driver_i2c);
	if (rc) {
		pr_err("%s i2c_add_driver failed rc = %d", __func__, rc);
		return -1;
	}
	pr_info("%s after i2c_add_driver, rc = %d", __func__, rc);
	*ctrl = g_ctrl;

	if (!(*ctrl)) {
		pr_err("%s *ctrl is NULL after i2c_add_driver, to revert...", __func__);
		pr_info("%s before i2c_del_driver", __func__);
		i2c_del_driver(&aon_sensor_driver_i2c);
		pr_info("%s after i2c_del_driver", __func__);
		return -1;
	}

	return 0;
}

void explorer_aon_exit(struct aon_sensor_ctrl_t **ctrl)
{
	pr_info("%s before i2c_del_driver", __func__);
	i2c_del_driver(&aon_sensor_driver_i2c);
	pr_info("%s after i2c_del_driver", __func__);
	*ctrl = g_ctrl;
}
