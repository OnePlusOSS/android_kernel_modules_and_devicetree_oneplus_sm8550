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

#include <linux/of.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include "include/mtk_aon_sensor_core.h"

static s32 mclk_up_index = -1;
static s32 mclk_cur_index = -1;
static s32 mclk_on = -1;
static s32 mclk_cur_on = -1;

enum pw_type_mclk {
	MCLK_UP = 0,
	MCLK_DOWN,
	MCLK_CUR_UP,
	MCLK_CUR_DOWN,
	MCLK_MAX
};

void aon_dinit_variables(void)
{
	mclk_up_index = -1;
	mclk_cur_index = -1;
	mclk_on = -1;
	mclk_cur_on = -1;
}

static s32 aon_soc_util_get_dt_regulator_info(struct aon_sensor_ctrl_t *s_ctrl)
{
	int rc = 0, count = 0, i = 0;
	struct device_node *of_node = NULL;
	struct aon_hw_soc_info *soc_info = NULL;

	if (!s_ctrl) {
		pr_err("%s Invalid parameters", __func__);
		return -EINVAL;
	}

	soc_info = &s_ctrl->soc_info;
	of_node = s_ctrl->of_node;

	soc_info->num_rgltr = 0;
	count = of_property_count_strings(of_node, "regulator-names");
	pr_info("%s %d regulators found", __func__, count);
	if (count != -EINVAL) {
		if (count <= 0) {
			pr_err("%s no regulators found", __func__);
			count = 0;
			return -EINVAL;
		}

		soc_info->num_rgltr = count;
	} else {
		pr_info("%s No regulators node found", __func__);
		return 0;
	}

	for (i = 0; i < soc_info->num_rgltr; i++) {
		rc = of_property_read_string_index(of_node,
			"regulator-names", i, &soc_info->rgltr_name[i]);
		pr_info("%s rgltr_name[%d] = %s", __func__,
			i, soc_info->rgltr_name[i]);
		if (rc) {
			pr_err("%s no regulator resource at cnt=%d", __func__, i);
			return -ENODEV;
		}
	}
	return rc;
}

static s32 aon_soc_util_get_dt_clk_info(struct aon_sensor_ctrl_t *s_ctrl)
{
	struct device_node *of_node = NULL;
	s32 count = 0, i = 0, rc = 0;
	struct aon_hw_soc_info *soc_info = NULL;

	if (!s_ctrl) {
		pr_err("%s invalid params", __func__);
		return -EINVAL;
	}

	soc_info = &s_ctrl->soc_info;
	of_node = s_ctrl->of_node;

	count = of_property_count_strings(of_node, "clock-names");

	pr_info("%s E: count = %d", __func__, count);
	if (count > CAM_SOC_MAX_CLK) {
		pr_err("%s too many count of clocks, count=%d", __func__, count);
		rc = -EINVAL;
		return rc;
	}
	if (count <= 0) {
		pr_info("%s No clock-names found", __func__);
		count = 0;
		soc_info->num_clk = count;
		return 0;
	}
	soc_info->num_clk = count;

	for (i = 0; i < count; i++) {
		rc = of_property_read_string_index(of_node, "clock-names",
				i, &(soc_info->clk_name[i]));
		pr_info("%s clock-names[%d] = %s",
			__func__, i, soc_info->clk_name[i]);
		if (rc) {
			pr_err("%s i= %d count= %d reading clock-names failed",
				__func__, i, count);
			return rc;
		}
	}
	return rc;
}

static s32 aon_soc_util_get_dt_properties_mtk(struct aon_sensor_ctrl_t *s_ctrl)
{
	struct device_node *of_node = NULL;
	int rc = 0;
	struct aon_hw_soc_info *soc_info = NULL;

	if (!s_ctrl) {
		pr_err("%s invalid params", __func__);
		return -EINVAL;
	}

	of_node = s_ctrl->of_node;
	soc_info = &s_ctrl->soc_info;

	rc = aon_soc_util_get_dt_regulator_info(s_ctrl);
	if (rc)
		return rc;
	pr_info("%s num_rgltr: %d", __func__, soc_info->num_rgltr);

	rc = aon_soc_util_get_dt_clk_info(s_ctrl);
	pr_info("%s aon_soc_util_get_dt_clk_info rc = %d", __func__, rc);

	return rc;
}

static s32 aon_sensor_driver_get_dt_data_mtk(struct aon_sensor_ctrl_t *s_ctrl)
{
	s32 rc = 0;
	struct aon_sensor_board_info *sensordata = NULL;
	struct device_node *of_node = NULL;
	struct aon_hw_soc_info *soc_info = NULL;

	if (!s_ctrl || !s_ctrl->of_node) {
		pr_err("%s invalid param", __func__);
		return -EINVAL;
	}

	soc_info = &s_ctrl->soc_info;
	of_node = s_ctrl->of_node;

	s_ctrl->sensordata = kzalloc(sizeof(*sensordata), GFP_KERNEL);
	if (!s_ctrl->sensordata) {
		pr_err("%s s_ctrl->sensordata kzalloc failed\n", __func__);
		return -ENOMEM;
	}
	pr_info("%s mm-kzalloc s_ctrl->sensordata %p, size: %d",
		__func__, s_ctrl->sensordata, sizeof(*sensordata));

	sensordata = s_ctrl->sensordata;

	rc = aon_soc_util_get_dt_properties_mtk(s_ctrl);
	if (rc < 0) {
		pr_err("%s Failed to read DT properties rc %d\n", __func__, rc);
		goto free_sensor_data;
	}
	pr_info("%s aon_soc_util_get_dt_properties ok\n", __func__);

	return rc;

free_sensor_data:
	pr_info("%s mm-kfree sensordata %p", __func__, sensordata);
	kfree(sensordata);
	sensordata = NULL;
	return rc;
}

s32 aon_sensor_parse_dt_mtk(struct aon_sensor_ctrl_t *s_ctrl)
{
	s32 i = 0;
	s32 j = 0;
	s32 rc = 0;
	struct aon_hw_soc_info *soc_info = NULL;

	pr_info("%s begin", __func__);
	if (!s_ctrl) {
		pr_err("%s invalid args", __func__);
		return -EINVAL;
	}

	soc_info = &s_ctrl->soc_info;

	/* Parse dt information and store in sensor control structure */
	rc = aon_sensor_driver_get_dt_data_mtk(s_ctrl);
	if (rc < 0) {
		pr_err("%s Failed to get dt data rc %d", __func__, rc);
		return rc;
	}
	pr_info("%s aon_sensor_driver_get_dt_data_mtk, rc = %d", __func__, rc);

	/* Initialize mutex */
	mutex_init(&(s_ctrl->aon_sensor_mutex));

	/* Initialize default parameters */
	for (i = 0; i < soc_info->num_clk; i++) {
		soc_info->clk[i] = devm_clk_get(soc_info->dev,
					soc_info->clk_name[i]);
		if (!soc_info->clk[i]) {
			pr_err("%s get clk failed for %s", __func__, soc_info->clk_name[i]);
			for (j = 0; j < i; j++) {
				devm_clk_put(soc_info->dev, soc_info->clk[j]);
				pr_info("%s after put soc_info->clk[%d]", __func__, i);
			}
			rc = -ENOENT;
			goto free_dt_data;
		}
	}

	return rc;

free_dt_data:
	pr_info("%s mm-kfree s_ctrl->sensordata %p", __func__, s_ctrl->sensordata);
	kfree(s_ctrl->sensordata);
	s_ctrl->sensordata = NULL;

	return rc;
}

static s32 aon_enable_regulator(struct aon_sensor_power_setting *power_setting,
		struct aon_hw_soc_info *soc_info)
{
	s32 vreg_idx = -1;
	s32 rc = 0;

	if (!power_setting || !soc_info)
		return -EINVAL;

	vreg_idx = power_setting->seq_val;
	pr_info("%s Enable Regulator %d", __func__, vreg_idx);

	soc_info->rgltr[vreg_idx] = regulator_get(soc_info->dev,
			soc_info->rgltr_name[vreg_idx]);
	if (IS_ERR_OR_NULL(soc_info->rgltr[vreg_idx])) {
		rc = PTR_ERR(soc_info->rgltr[vreg_idx]);
		rc = rc ? rc : -EINVAL;
		pr_err("%s %s get failed %d", __func__,
			soc_info->rgltr_name[vreg_idx], rc);

		soc_info->rgltr[vreg_idx] = NULL;
		return -1;
	}

	rc = regulator_set_load(soc_info->rgltr[vreg_idx],
			power_setting->config_val);
	if (rc) {
		pr_err("%s regulator %s set_load failed",
			__func__, soc_info->rgltr_name[vreg_idx]);
		regulator_put(soc_info->rgltr[vreg_idx]);
		soc_info->rgltr[vreg_idx] = NULL;
		return -1;
	}

	rc = regulator_enable(soc_info->rgltr[vreg_idx]);
	if (rc) {
		pr_err("%s %s regulator_enable failed",
			__func__, soc_info->rgltr_name[vreg_idx]);
		regulator_set_load(soc_info->rgltr[vreg_idx], 0);
		regulator_put(soc_info->rgltr[vreg_idx]);
		soc_info->rgltr[vreg_idx] = NULL;
		rc = -1;
	}

	return rc;
}

static s32 aon_disable_regulator(struct aon_sensor_power_setting *power_setting,
		struct aon_hw_soc_info *soc_info)
{
	s32 vreg_idx = -1;
	s32 rc = 0;

	if (!power_setting || !soc_info)
		return -EINVAL;

	vreg_idx = power_setting->seq_val;
	if (soc_info->rgltr[vreg_idx]) {
		pr_info("%s Disable Regulator %d", __func__, vreg_idx);
		rc = regulator_disable(soc_info->rgltr[vreg_idx]);
		if (rc) {
			pr_err("%s Reg: %s regulator_disable failed", __func__,
				soc_info->rgltr_name[vreg_idx]);
			return rc;
		}

		regulator_set_load(soc_info->rgltr[vreg_idx], 0);
		regulator_put(soc_info->rgltr[vreg_idx]);
		soc_info->rgltr[vreg_idx] = NULL;
	}
	return rc;
}

static void aon_power_delay(unsigned short delay)
{
	if (delay > 20)
		msleep(delay);
	else if (delay > 0)
		usleep_range(delay * 1000, (delay * 1000) + 1000);
}

s32 aon_vcm_power_on(struct aon_sensor_power_ctrl_t *ctrl,
		struct aon_hw_soc_info *soc_info)
{
	s32 rc = -1;
	s32 i = 0;
	s32 find = 0;
	struct aon_sensor_power_setting *power_setting = NULL;

	if (!ctrl || !soc_info || !ctrl->vcm_power_setting)
		return -EINVAL;

	if (ctrl->vcm_is_on == 1) {
		pr_info("%s aon vcm is already on", __func__);
		return 0;
	}

	for (i = 0; i < ctrl->vcm_power_setting_size; i++) {
		power_setting = &(ctrl->vcm_power_setting[i]);
		if (!power_setting)
			return -EINVAL;
		switch (power_setting->seq_type) {
		case MP_AFVDD:
			ctrl->vcm_on_idx = i;
			rc = aon_enable_regulator(power_setting, soc_info);
			if (rc) {
				pr_err("aon_enable_regulator failed");
			} else {
				aon_power_delay(power_setting->delay);
				ctrl->vcm_is_on = 1;
			}
			find = 1;
			break;
		default:
			pr_info("%s not supported power type for VCM", __func__);
			break;
		}
		if (find == 1)
			break;
	}
	return rc;
}

s32 aon_vcm_power_off(struct aon_sensor_power_ctrl_t *ctrl,
		struct aon_hw_soc_info *soc_info)
{
	s32 rc = 0;
	struct aon_sensor_power_setting *ps = NULL;

	if (!ctrl || !soc_info || !ctrl->power_setting)
		return -EINVAL;

	if (ctrl->vcm_is_on == 0) {
		pr_info("%s aon vcm is already off", __func__);
		return 0;
	}

	if ((ctrl->vcm_is_on != 1) || (ctrl->vcm_on_idx < 0)
		|| (ctrl->vcm_on_idx >= ctrl->vcm_power_setting_size))
		return -EINVAL;

	ps = &ctrl->vcm_power_setting[ctrl->vcm_on_idx];

	rc = aon_disable_regulator(ps, soc_info);
	if (rc) {
		pr_err("aon_disable_regulator failed");
	} else {
		aon_power_delay(ps->delay);
		ctrl->vcm_is_on = 0;
		ctrl->vcm_on_idx = -1;
	}

	return rc;
}

static s32 aon_get_clk_idx(long val)
{
	s32 idx = -1, i = 0;
	long rate[] = {6000000, 12000000, 13000000, 19200000,
					24000000, 26000000, 52000000};
	s32 size = sizeof(rate) / sizeof(rate[0]);

	for (i = 0; i < size; i++) {
		if (val == rate[i]) {
			idx = i;
			break;
		}
	}

	return idx;
}

static s32 aon_get_mclk_cur_idx(long val)
{
	s32 idx = -1, i = 0;
	long cur[] = {0, 2, 4, 6, 8};
	s32 size = sizeof(cur) / sizeof(cur[0]);

	for (i = 0; i < size; i++) {
		if (val == cur[i]) {
			idx = i;
			break;
		}
	}

	return idx;
}

static s32 aon_pinctrl_init(struct aon_pinctrl_info *sensor_pctrl,
			struct device *dev)
{
	char *cur[5] = {"mclk_off", "mclk_2mA", "mclk_4mA", "mclk_6mA", "mclk_8mA"};
	char *rst[2] = {"rst_low", "rst_high"};
	char *pdn[2] = {"pdn_low", "pdn_high"};
	s32 i = 0;

	if (!sensor_pctrl || !dev) {
		pr_err("%s invalid params, sensor_pctrl %p, dev %p",
			__func__, sensor_pctrl, dev);
		return -EINVAL;
	}
	sensor_pctrl->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(sensor_pctrl->pinctrl)) {
		pr_err("%s Getting pinctrl handle failed", __func__);
		return -EINVAL;
	}

	for (i = 0; i < 5; i++) {
		sensor_pctrl->gpio_state_mclk[i] =
			pinctrl_lookup_state(sensor_pctrl->pinctrl, cur[i]);
		if (IS_ERR_OR_NULL(sensor_pctrl->gpio_state_mclk[i])) {
			pr_err("%s Failed to get the gpio_state_mclk[%d] handle", __func__, i);
			return -EINVAL;
		}
	}

	for (i = 0; i < 2; i++) {
		sensor_pctrl->gpio_state_rst[i]
			= pinctrl_lookup_state(sensor_pctrl->pinctrl, rst[i]);
		if (IS_ERR_OR_NULL(sensor_pctrl->gpio_state_rst[i])) {
			pr_err("%s Failed to get the gpio_state_rst[%d] handle", __func__, i);
			return -EINVAL;
		}
	}

	for (i = 0; i < 2; i++) {
		sensor_pctrl->gpio_state_pdn[i]
			= pinctrl_lookup_state(sensor_pctrl->pinctrl, pdn[i]);
		if (IS_ERR_OR_NULL(sensor_pctrl->gpio_state_pdn[i])) {
			pr_err("%s Failed to get the gpio_state_pdn[%d] handle", __func__, i);
			return -EINVAL;
		}
	}

	return 0;
}

static void aon_update_mclk_index(struct aon_sensor_power_ctrl_t *ctrl,
			enum pw_type_mclk type, s32 *idx)
{
	s32 index = 0;
	struct aon_sensor_power_setting *pd = NULL;

	if (!ctrl || !idx || type < MCLK_UP || type > MCLK_MAX) {
		pr_err("%s invalid args", __func__);
		return;
	}

	switch (type) {
	case MCLK_UP:
		for (index = 0; index < ctrl->power_setting_size; index++) {
			pr_debug("%s power_setting_index %d", __func__, index);
			pd = &ctrl->power_setting[index];
			if (!pd) {
				pr_err("%s Invalid power settings for index %d", __func__, index);
				return;
			}

			pr_debug("%s power seq_type %d", __func__, pd->seq_type);
			if (pd->seq_type == MP_MCLK) {
				*idx = index;
				pr_info("%s mclk up index: %d", __func__, index);
				break;
			}
		}
		break;
	case MCLK_DOWN:
		for (index = 0; index < ctrl->power_down_setting_size; index++) {
			pr_debug("%s power_down_setting_index %d", __func__, index);
			pd = &ctrl->power_down_setting[index];
			if (!pd) {
				pr_err("%s Invalid power down settings for index %d",
					__func__, index);
				return;
			}

			pr_debug("%s power down seq_type %d", __func__, pd->seq_type);
			if (pd->seq_type == MP_MCLK) {
				*idx = index;
				pr_info("%s mclk down index: %d", __func__, index);
				break;
			}
		}
		break;
	case MCLK_CUR_UP:
		for (index = 0; index < ctrl->power_setting_size; index++) {
			pr_debug("%s power_setting_index %d", __func__, index);
			pd = &ctrl->power_setting[index];
			if (!pd) {
				pr_err("%s Invalid power settings for index %d",
					__func__, index);
				return;
			}

			pr_debug("%s power seq_type %d", __func__, pd->seq_type);
			if (pd->seq_type == MP_MCLK_DRIVER_CURRENT) {
				*idx = index;
				pr_info("%s mclk_up_index: %d", __func__, index);
				break;
			}
		}
		break;
	case MCLK_CUR_DOWN:
		for (index = 0; index < ctrl->power_down_setting_size; index++) {
			pr_debug("%s power_down_setting_index %d", __func__, index);
			pd = &ctrl->power_down_setting[index];
			if (!pd) {
				pr_err("%s Invalid power down settings for index %d",
					__func__, index);
				return;
			}

			pr_debug("%s power seq_type %d", __func__, pd->seq_type);
			if (pd->seq_type == MP_MCLK_DRIVER_CURRENT) {
				*idx = index;
				pr_info("%s mclk_down_index: %d", __func__, index);
				break;
			}
		}
		break;
	default:
		pr_err("%s not supported type", __func__);
		break;
	}
}

s32 aon_disable_mclk(struct aon_sensor_power_ctrl_t *ctrl,
		struct aon_hw_soc_info *soc_info)
{
	s32 rc = 0;
	s32 i = 0;

	if (mclk_on != 1) {
		pr_info("%s MCLK already disabled", __func__);
		return 0;
	}

	if (!ctrl || !soc_info) {
		pr_err("%s invalid args", __func__);
		return -EINVAL;
	}

	for (i = soc_info->num_clk - 1; i >= 0; i--) {
		if (soc_info->clk_enabled[i] == 1 && soc_info->clk_name[i]) {
			clk_disable_unprepare(soc_info->clk[i]);
			pr_info("%s disable %s", __func__, soc_info->clk_name[i]);
			soc_info->clk_enabled[i] = 0;
		}
	}
	clk_disable_unprepare(soc_info->clk[soc_info->num_clk - 1]);

	mclk_on = 0;
	return rc;
}

s32 aon_enable_mclk(struct aon_sensor_power_ctrl_t *ctrl,
		struct aon_hw_soc_info *soc_info)
{
	s32 mclk_idx = -1;
	s32 rc = 0;
	struct aon_sensor_power_setting *power_setting = NULL;

	if (mclk_on == 1) {
		pr_info("%s MCLK already enabled", __func__);
		return 0;
	}

	if (!ctrl || !soc_info) {
		pr_err("%s invalid args", __func__);
		return -EINVAL;
	}

	if (mclk_up_index < 0 || mclk_up_index >= ctrl->power_setting_size) {
		aon_update_mclk_index(ctrl, MCLK_UP, &mclk_up_index);
		if (mclk_up_index < 0 || mclk_up_index >= ctrl->power_setting_size) {
			pr_err("%s not found MCLK in power on sequence", __func__);
			return -EINVAL;
		}
	}
	power_setting = &ctrl->power_setting[mclk_up_index];
	mclk_idx = aon_get_clk_idx(power_setting->config_val);
	if (mclk_idx < 0 || mclk_idx >= soc_info->num_clk) {
		pr_err("%s invalid mclk value", __func__);
		mclk_up_index = -1;
		return -EINVAL;
	}
	pr_info("%s found idx: %d for clock rate: %ld",
		__func__, mclk_idx, power_setting->config_val);

	rc = clk_prepare_enable(soc_info->clk[soc_info->num_clk - 1]);
	if (rc < 0) {
		pr_err("%s clk_prepare_enable mclk base failed", __func__);
		mclk_up_index = -1;
		return rc;
	}
	rc = clk_prepare_enable(soc_info->clk[mclk_idx]);
	if (rc < 0) {
		pr_err("%s clk_prepare_enable failed", __func__);
		clk_disable_unprepare(soc_info->clk[soc_info->num_clk - 1]);
		mclk_up_index = -1;
		return rc;
	}
	rc = clk_set_parent(soc_info->clk[soc_info->num_clk - 1], soc_info->clk[mclk_idx]);
	if (rc < 0) {
		pr_info("%s clock_set_parent failed rc = %d", __func__, rc);
		clk_disable_unprepare(soc_info->clk[mclk_idx]);
		clk_disable_unprepare(soc_info->clk[soc_info->num_clk - 1]);
		mclk_up_index = -1;
		return rc;
	}

	soc_info->clk_enabled[mclk_idx] = 1;
	pr_info("%s enable clk %d", __func__, mclk_idx);
	mclk_on = 1;

	return 0;
}

s32 aon_disable_mclk_cur(struct aon_sensor_power_ctrl_t *ctrl,
		struct aon_hw_soc_info *soc_info)
{
	s32 rc = 0;

	if (mclk_cur_on != 1) {
		pr_info("%s MCLK current already disabled", __func__);
		return 0;
	}

	if (!ctrl || !soc_info) {
		pr_err("%s invalid args", __func__);
		return -EINVAL;
	}

	pinctrl_select_state(ctrl->pinctrl_info.pinctrl,
		ctrl->pinctrl_info.gpio_state_mclk[0]);
	pr_info("%s end", __func__);
	mclk_cur_on = 0;

	return rc;
}

s32 aon_enable_mclk_cur(struct aon_sensor_power_ctrl_t * ctrl,
	struct aon_hw_soc_info * soc_info)
{
	s32 rc = 0;
	s32 sta = -1;
	struct aon_sensor_power_setting *power_setting = NULL;

	if (mclk_cur_on == 1) {
		pr_info("%s MCLK current already enabled", __func__);
		return 0;
	}

	if (!ctrl || !soc_info) {
		pr_err("%s invalid args", __func__);
		return -EINVAL;
	}

	if (mclk_cur_index < 0 || mclk_cur_index >= ctrl->power_setting_size) {
		aon_update_mclk_index(ctrl, MCLK_CUR_UP, &mclk_cur_index);
		if (mclk_cur_index < 0 || mclk_cur_index >= ctrl->power_setting_size) {
			pr_err("%s not found MCLK current in power on sequence", __func__);
			return -EINVAL;
		}
	}
	power_setting = &ctrl->power_setting[mclk_cur_index];
	sta = aon_get_mclk_cur_idx(power_setting->config_val);
	if (sta == -1) {
		pr_err("%s can not found compatible state for mclk", __func__);
		mclk_cur_index = -1;
		return -EINVAL;
	}
	rc = pinctrl_select_state(
		ctrl->pinctrl_info.pinctrl,
		ctrl->pinctrl_info.gpio_state_mclk[sta]);
	if (rc) {
		pr_err("%s cannot set mclk pin to expected state", __func__);
		mclk_cur_index = -1;
		return -EINVAL;
	}
	mclk_cur_on = 1;
	pr_info("%s end", __func__);

	return rc;
}

static s32 aon_sensor_util_power_up(struct aon_sensor_power_ctrl_t *ctrl,
		struct aon_hw_soc_info *soc_info)
{
	s32 rc = 0, index = 0, ret = 0, num_vreg, i = 0;
	s32 sta = -1;
	s32 mclk_idx = -1;
	struct aon_sensor_power_setting *power_setting = NULL;

	if (!ctrl || !soc_info) {
		pr_err("%s Invalid handle, ctrl %p soc_info %p",
			__func__, ctrl, soc_info);
		return -EINVAL;
	}

	num_vreg = soc_info->num_rgltr;

	if ((num_vreg <= 0) || (num_vreg > CAM_SOC_MAX_REGULATOR)) {
		pr_err("%s failed: num_vreg %d", __func__, num_vreg);
		return -EINVAL;
	}

	ret = aon_pinctrl_init(&(ctrl->pinctrl_info), ctrl->dev);
	if (ret < 0) {
		/* Some sensor subdev no pinctrl. */
		pr_info("%s Initialization of pinctrl failed", __func__);
		ctrl->cam_pinctrl_status = 0;
	} else {
		ctrl->cam_pinctrl_status = 1;
	}

	pr_info("%s power setting size: %d", __func__, ctrl->power_setting_size);
	for (index = 0; index < ctrl->power_setting_size; index++) {
		pr_info("%s index: %d", __func__, index);
		power_setting = &ctrl->power_setting[index];
		if (!power_setting) {
			pr_err("%s Invalid power up settings for index %d", __func__, index);
			return -EINVAL;
		}

		pr_info("%s seq_type %d", __func__, power_setting->seq_type);

		switch (power_setting->seq_type) {
		case MP_MCLK:{
			rc = clk_prepare_enable(soc_info->clk[soc_info->num_clk - 1]);
			if (rc < 0) {
				pr_info("%s enable base mclk failed, rc = %d", __func__, rc);
				goto power_up_failed;
			}
			mclk_idx = aon_get_clk_idx(power_setting->config_val);
			if (mclk_idx < 0 || mclk_idx >= soc_info->num_clk) {
				pr_err("%s invalid mclk value", __func__);
				rc = -EINVAL;
				clk_disable_unprepare(soc_info->clk[soc_info->num_clk - 1]);
				goto power_up_failed;
			}
			pr_info("%s found idx: %d for clock rate: %ld",
				__func__, mclk_idx, power_setting->config_val);

			mclk_up_index = index;

			rc = clk_prepare_enable(soc_info->clk[mclk_idx]);
			if (rc < 0) {
				pr_err("%s clk_prepare_enable failed", __func__);
				clk_disable_unprepare(soc_info->clk[soc_info->num_clk - 1]);
				mclk_up_index = -1;
				goto power_up_failed;
			}
			rc = clk_set_parent(soc_info->clk[soc_info->num_clk - 1], soc_info->clk[mclk_idx]);
			if (rc < 0) {
				pr_info("%s clock_set_parent failed rc = %d", __func__, rc);
				clk_disable_unprepare(soc_info->clk[mclk_idx]);
				clk_disable_unprepare(soc_info->clk[soc_info->num_clk - 1]);
				mclk_up_index = -1;
				goto power_up_failed;
			}
			soc_info->clk_enabled[mclk_idx] = 1;
			mclk_on = 1;
			break;
		}
		case MP_AVDD:
		case MP_DVDD:
		case MP_DOVDD:
			if (power_setting->seq_val == INVALID_VREG) {
				pr_info("%s invalid power seq_val, ignore...", __func__);
				break;
			}

			if (power_setting->seq_val >= CAM_VREG_MAX) {
				pr_err("%s seq_val %d >= max %d", __func__,
					power_setting->seq_val, CAM_VREG_MAX);
				goto power_up_failed;
			}

			if (power_setting->seq_val >= num_vreg) {
				pr_info("%s seq_val >= dts num_vreg (%d >= %d), ignore...",
					__func__, power_setting->seq_val, num_vreg);
				break;
			}
			rc = aon_enable_regulator(power_setting, soc_info);
			if (rc) {
				pr_err("aon_enable_regulator failed");
				goto power_up_failed;
			} else {
				pr_info("aon_enable_regulator successfully");
			}

			break;
		case MP_RST:
			if (power_setting->config_val == 0) {
				sta = 0;
			} else {
				sta = 1;
			}
			ret = pinctrl_select_state(
				ctrl->pinctrl_info.pinctrl,
				ctrl->pinctrl_info.gpio_state_rst[sta]);
			if (ret) {
				pr_err("%s cannot set rst pin to state", __func__);
				goto power_up_failed;
			}
			break;

		case MP_PDN:
			if (power_setting->config_val == 0) {
				sta = 0;
			} else {
				sta = 1;
			}
			ret = pinctrl_select_state(
				ctrl->pinctrl_info.pinctrl,
				ctrl->pinctrl_info.gpio_state_pdn[sta]);
			if (ret) {
				pr_err("%s cannot set pdn pin to state", __func__);
				goto power_up_failed;
			}
			break;

		case MP_MCLK_DRIVER_CURRENT: {
			sta = aon_get_mclk_cur_idx(power_setting->config_val);
			if (sta == -1) {
				pr_err("%s can not found compatible state for mclk", __func__);
				ret = -EINVAL;
				mclk_cur_index = -1;
				goto power_up_failed;
			}
			ret = pinctrl_select_state(
				ctrl->pinctrl_info.pinctrl,
				ctrl->pinctrl_info.gpio_state_mclk[sta]);
			if (ret) {
				pr_err("%s cannot set mclk pin to expected state", __func__);
				goto power_up_failed;
			}
			mclk_cur_index = index;
			mclk_cur_on = 1;
			break;
		}

		case MP_AFVDD:
			pr_err("%s MP_AFVDD ignored", __func__);
			break;

		default:
			pr_err("%s invalid power seq type %d", __func__,
				power_setting->seq_type);
			break;
		}
		aon_power_delay(power_setting->delay);

	}
	return 0;
power_up_failed:
	pr_err("%s failed, now to recover", __func__);
	for (index--; index >= 0; index--) {
		pr_info("%s index %d", __func__, index);
		power_setting = &ctrl->power_setting[index];
		pr_info("%s type %d", __func__, power_setting->seq_type);
		switch (power_setting->seq_type) {
		case MP_MCLK:
			for (i = soc_info->num_clk - 1; i >= 0; i--) {
				if (soc_info->clk_enabled[i] == 1 && soc_info->clk_name[i]) {
					clk_disable_unprepare(soc_info->clk[i]);
					soc_info->clk_enabled[i] = 0;
				}
			}
			clk_disable_unprepare(soc_info->clk[soc_info->num_clk - 1]);
			mclk_up_index = -1;
			mclk_on = 0;
			break;
		case MP_RST:
			pinctrl_select_state(
				ctrl->pinctrl_info.pinctrl,
				ctrl->pinctrl_info.gpio_state_rst[0]);
			break;
		case MP_PDN:
			pinctrl_select_state(
				ctrl->pinctrl_info.pinctrl,
				ctrl->pinctrl_info.gpio_state_pdn[0]);
			break;
		case MP_MCLK_DRIVER_CURRENT:
				pinctrl_select_state(
				ctrl->pinctrl_info.pinctrl,
				ctrl->pinctrl_info.gpio_state_mclk[0]);
				mclk_cur_index = -1;
				mclk_cur_on = 0;
			break;
		case MP_AVDD:
		case MP_DVDD:
		case MP_DOVDD:
			if (power_setting->seq_val < num_vreg) {
				pr_info("%s Disable Regulator", __func__);
				aon_disable_regulator(power_setting, soc_info);
			} else {
				pr_err("%s seq_val:%d > num_vreg: %d", __func__,
					power_setting->seq_val, num_vreg);
			}
			break;
		case MP_AFVDD:
			pr_err("%s MP_AFVDD ignored", __func__);
			break;
		default:
			pr_err("%s error power seq type %d", __func__,
				power_setting->seq_type);
			break;
		}
		aon_power_delay(power_setting->delay);
	}
	if (ctrl->cam_pinctrl_status) {
		pr_info("%s before devm_pinctrl_put", __func__);
		devm_pinctrl_put(ctrl->pinctrl_info.pinctrl);
	}
	ctrl->cam_pinctrl_status = 0;

	return -1;
}

s32 aon_sensor_power_up(struct aon_sensor_ctrl_t *s_ctrl)
{
	s32 rc = 0;
	struct aon_sensor_power_ctrl_t *power_info = NULL;
	struct aon_camera_slave_info *slave_info = NULL;
	struct aon_hw_soc_info *soc_info = NULL;

	if (!s_ctrl || !s_ctrl->sensordata) {
		pr_err("%s invalid param, s_ctrl: %p", __func__, s_ctrl);
		return -EINVAL;
	}

	soc_info = &s_ctrl->soc_info;
	power_info = &(s_ctrl->sensordata->power_info);
	slave_info = &(s_ctrl->sensordata->slave_info);

	if (!power_info || !slave_info) {
		pr_err("%s failed: %p %p", __func__, power_info, slave_info);
		return -EINVAL;
	}

	rc = aon_sensor_util_power_up(power_info, soc_info);
	if (rc < 0)
		pr_err("%s aon_sensor_util_power_up failed:%d", __func__, rc);

	return rc;
}

static struct aon_sensor_power_setting *aon_get_power_settings(
				struct aon_sensor_power_ctrl_t *ctrl,
				enum mtk_camera_power_seq_type seq_type,
				u16 seq_val)
{
	struct aon_sensor_power_setting *power_setting = NULL, *ps = NULL;
	int idx = 0;

	if (!ctrl) {
		pr_err("%s invalid params", __func__);
		return NULL;
	}

	for (idx = 0; idx < ctrl->power_setting_size; idx++) {
		power_setting = &ctrl->power_setting[idx];
		if (power_setting->seq_type == seq_type &&
			power_setting->seq_val == seq_val) {
			ps = power_setting;
			return ps;
		}
	}

	return ps;
}

static s32 aon_sensor_util_power_down(struct aon_sensor_power_ctrl_t *ctrl,
		struct aon_hw_soc_info *soc_info)
{
	s32 index = 0, num_vreg = 0, i = 0;
	struct aon_sensor_power_setting *pd = NULL;
	struct aon_sensor_power_setting *ps = NULL;

	if (!ctrl || !soc_info) {
		pr_err("%s failed, ctrl %p, soc_info %p", __func__, ctrl, soc_info);
		return -EINVAL;
	}

	num_vreg = soc_info->num_rgltr;

	if ((num_vreg <= 0) || (num_vreg > CAM_SOC_MAX_REGULATOR)) {
		pr_err("%s failed: num_vreg %d", __func__, num_vreg);
		return -EINVAL;
	}

	if (ctrl->power_down_setting_size > MAX_POWER_CONFIG) {
		pr_err("%s Invalid: power setting size %d", __func__,
			ctrl->power_setting_size);
		return -EINVAL;
	}

	pr_info("%s power_down_setting_size = %d",
		__func__, ctrl->power_down_setting_size);
	for (index = 0; index < ctrl->power_down_setting_size; index++) {
		pr_err("%s power_down_index %d", __func__, index);
		pd = &ctrl->power_down_setting[index];
		if (!pd) {
			pr_err("%s Invalid power down setting for index %d", __func__,
				index);
			return -EINVAL;
		}

		ps = NULL;
		pr_info("%s power down seq_type %d", __func__, pd->seq_type);
		switch (pd->seq_type) {
		case MP_MCLK:
			for (i = soc_info->num_clk - 1; i >= 0; i--) {
				if (soc_info->clk_enabled[i] == 1 && soc_info->clk_name[i]) {
					clk_disable_unprepare(soc_info->clk[i]);
					soc_info->clk_enabled[i] = 0;
				}
			}
			clk_disable_unprepare(soc_info->clk[soc_info->num_clk - 1]);
			mclk_on = 0;
			break;
		case MP_RST:
			pinctrl_select_state(
				ctrl->pinctrl_info.pinctrl,
				ctrl->pinctrl_info.gpio_state_rst[0]);
			break;
		case MP_PDN:
			pinctrl_select_state(
				ctrl->pinctrl_info.pinctrl,
				ctrl->pinctrl_info.gpio_state_pdn[0]);
			break;
		case MP_MCLK_DRIVER_CURRENT:
			pinctrl_select_state(
				ctrl->pinctrl_info.pinctrl,
				ctrl->pinctrl_info.gpio_state_mclk[0]);
			mclk_cur_on = 0;
			break;
		case MP_AVDD:
		case MP_DVDD:
		case MP_DOVDD:
			if (pd->seq_val == INVALID_VREG)
				break;

			ps = aon_get_power_settings(ctrl, pd->seq_type, pd->seq_val);
			if (ps) {
				if (pd->seq_val < num_vreg) {
					pr_err("%s Disable Regulator", __func__);
					aon_disable_regulator(ps, soc_info);
				} else {
					pr_err("%s seq_val:%d > num_vreg: %d", __func__,
						pd->seq_val, num_vreg);
				}
			} else
				pr_err("%s error in power up/down seq", __func__);

			break;
		case MP_AFVDD:
			pr_err("%s MP_AFVDD ignored", __func__);
			break;
		default:
			pr_err("%s error power seq type %d", __func__, pd->seq_type);
			break;
		}
		aon_power_delay(pd->delay);
	}

	devm_pinctrl_put(ctrl->pinctrl_info.pinctrl);
	ctrl->cam_pinctrl_status = 0;

	return 0;
}

s32 aon_sensor_power_down(struct aon_sensor_ctrl_t *s_ctrl)
{
	struct aon_sensor_power_ctrl_t *power_info = NULL;
	struct aon_hw_soc_info *soc_info = NULL;
	s32 rc = 0;

	if (!s_ctrl || !s_ctrl->sensordata) {
		pr_err("%s invalid param", __func__);
		return -EINVAL;
	}

	power_info = &s_ctrl->sensordata->power_info;
	soc_info = &s_ctrl->soc_info;

	if (!power_info) {
		pr_err("%s failed: power_info %p", __func__, power_info);
		return -EINVAL;
	}

	rc = aon_sensor_util_power_down(power_info, soc_info);
	if (rc < 0) {
		pr_err("%s aon_sensor_util_power_down failed, rc = %d", __func__, rc);
	}
	return rc;
}

s32 aon_sensor_update_eeprom_i2c_info(struct aon_cmd_i2c_info *i2c_info,
	struct aon_sensor_ctrl_t *s_ctrl)
{
	s32 rc = 0;

	if (!i2c_info || !s_ctrl || !s_ctrl->eeprom_i2c_client) {
		pr_err("%s invalid param", __func__);
		return -EINVAL;
	}

	s_ctrl->eeprom_i2c_client->addr = i2c_info->slave_addr;
	pr_info("%s eeprom slave info: %d", __func__, i2c_info->slave_addr);
	return rc;
}

s32 aon_sensor_update_i2c_info(struct aon_cmd_i2c_info *i2c_info,
	struct aon_sensor_ctrl_t *s_ctrl)
{
	s32 rc = 0;

	if (!i2c_info || !s_ctrl || !s_ctrl->sensordata) {
		pr_err("%s invalid param", __func__);
		return -EINVAL;
	}

	s_ctrl->sensordata->slave_info.sensor_slave_addr = i2c_info->slave_addr;
	s_ctrl->i2c_client->addr = i2c_info->slave_addr;
	return rc;
}

s32 aon_fill_vreg_params(struct aon_hw_soc_info *soc_info,
	struct aon_sensor_power_setting *power_setting,
	u16 power_setting_size)
{
	s32 rc = 0, j = 0, i = 0;
	s32 num_vreg;

	/* Validate input parameters */
	if (!soc_info || !power_setting || power_setting_size <= 0) {
		pr_err("%s %d failed: soc_info %p power_setting %p",
			__func__, __LINE__, soc_info, power_setting);
		return -EINVAL;
	}

	num_vreg = soc_info->num_rgltr;

	if ((num_vreg <= 0) || (num_vreg > CAM_SOC_MAX_REGULATOR)) {
		pr_err("%s %d failed: num_vreg %d", __func__, __LINE__, num_vreg);
		return -EINVAL;
	}

	for (i = 0; i < power_setting_size; i++) {
		pr_info("%s [%d] seq_type: %d", __func__, i, power_setting[i].seq_type);

		if (power_setting[i].seq_type < MP_MCLK ||
			power_setting[i].seq_type >= MP_SEQ_MAX) {
			pr_err("%s %d failed: Invalid Seq type: %d", __func__, __LINE__,
				power_setting[i].seq_type);
			return -EINVAL;
		}

		switch (power_setting[i].seq_type) {
		case MP_AVDD:
			for (j = 0; j < num_vreg; j++) {
				if (!strcmp(soc_info->rgltr_name[j], "avdd")) {
					pr_info("%s i: %d j: %d avdd", __func__, i, j);
					power_setting[i].seq_val = j;
					break;
				}
			}
			if (j == num_vreg)
				power_setting[i].seq_val = INVALID_VREG;
			break;
		case MP_AFVDD:
			for (j = 0; j < num_vreg; j++) {
				if (!strcmp(soc_info->rgltr_name[j], "afvdd")) {
					pr_info("%s i: %d j: %d afvdd", __func__, i, j);
					power_setting[i].seq_val = j;
					break;
				}
			}
			if (j == num_vreg)
				power_setting[i].seq_val = INVALID_VREG;
			break;
		case MP_DVDD:
			for (j = 0; j < num_vreg; j++) {
				if (!strcmp(soc_info->rgltr_name[j], "dvdd")) {
					pr_info("%s i: %d j: %d dvdd", __func__, i, j);
					power_setting[i].seq_val = j;
					break;
				}
			}
			if (j == num_vreg)
				power_setting[i].seq_val = INVALID_VREG;
			break;

		case MP_DOVDD:
			for (j = 0; j < num_vreg; j++) {
				if (!strcmp(soc_info->rgltr_name[j], "dovdd")) {
					pr_info("%s i: %d j: %d dovdd", __func__, i, j);
					power_setting[i].seq_val = j;
					break;
				}
			}
			if (j == num_vreg)
				power_setting[i].seq_val = INVALID_VREG;
			break;

		default:
			break;
		}
	}

	return rc;
}

