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
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>

#include "include/qcom_aon_sensor_core.h"

#define MAX_CAMERAS 16

#define AON_SENSOR_PINCTRL_STATE_SLEEP "cam_suspend"
#define AON_SENSOR_PINCTRL_STATE_DEFAULT "cam_default"

#define VALIDATE_VOLTAGE(min, max, config_val) ((config_val) && \
	(config_val >= min) && (config_val <= max))

static s32 mclk_up_index = -1;
static s32 mclk_down_index = -1;
static s32 mclk_enabled = -1;

enum msm_sensor_power_seq_gpio_t {
	SENSOR_GPIO_RESET,
	SENSOR_GPIO_STANDBY,
	SENSOR_GPIO_AF_PWDM,
	SENSOR_GPIO_VIO,
	SENSOR_GPIO_VANA,
	SENSOR_GPIO_VDIG,
	SENSOR_GPIO_VAF,
	SENSOR_GPIO_FL_EN,
	SENSOR_GPIO_FL_NOW,
	SENSOR_GPIO_FL_RESET,
	SENSOR_GPIO_CUSTOM1,
	SENSOR_GPIO_CUSTOM2,
	SENSOR_GPIO_CUSTOM3,
	SENSOR_GPIO_MAX,
};

void aon_dinit_variables(void)
{
	mclk_up_index = -1;
	mclk_down_index = -1;
	mclk_enabled = -1;
}

static int aon_pinctrl_init(struct aon_pinctrl_info *sensor_pctrl,
	struct device *dev)
{
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
	sensor_pctrl->gpio_state_active =
		pinctrl_lookup_state(sensor_pctrl->pinctrl,
				AON_SENSOR_PINCTRL_STATE_DEFAULT);
	if (IS_ERR_OR_NULL(sensor_pctrl->gpio_state_active)) {
		pr_err("%s Failed to get the active state pinctrl handle", __func__);
		return -EINVAL;
	}
	sensor_pctrl->gpio_state_suspend
		= pinctrl_lookup_state(sensor_pctrl->pinctrl,
				AON_SENSOR_PINCTRL_STATE_SLEEP);
	if (IS_ERR_OR_NULL(sensor_pctrl->gpio_state_suspend)) {
		pr_err("%s Failed to get the suspend state pinctrl handle", __func__);
		return -EINVAL;
	}
	return 0;
}

static int aon_res_mgr_gpio_request(struct device *dev, uint gpio,
	unsigned long flags, const char *label)
{
	int rc = 0;

	rc = gpio_request_one(gpio, flags, label);
	if (rc) {
		pr_err("%s gpio %d:%s request fails", __func__,
			gpio, label);
		return rc;
	}
	return rc;
}

static void aon_res_mgr_gpio_free_arry(const struct gpio *array, size_t num)
{
	while (num--) {
		pr_info("%s free array[%d].gpio", __func__, num);
		gpio_free(array[num].gpio);
	}
}

static int aon_sensor_util_request_gpio_table(struct aon_hw_soc_info *soc_info,
	int gpio_en)
{
	int rc = 0, i = 0;
	u8 size = 0;
	struct aon_soc_gpio_data *gpio_conf = NULL;
	struct gpio *gpio_tbl = NULL;

	if (!soc_info) {
		pr_err("%s invalid params", __func__);
		return -EINVAL;
	}
	gpio_conf = soc_info->gpio_data;

	if (!gpio_conf) {
		pr_err("%s No GPIO data", __func__);
		return 0;
	}

	if (gpio_conf->cam_gpio_common_tbl_size <= 0) {
		pr_err("%s No GPIO entry", __func__);
		return -EINVAL;
	}

	gpio_tbl = gpio_conf->cam_gpio_req_tbl;
	size = gpio_conf->cam_gpio_req_tbl_size;

	if (!gpio_tbl || !size) {
		pr_err("%s invalid gpio_tbl %p / size %d", __func__,
			gpio_tbl, size);
		return -EINVAL;
	}

	for (i = 0; i < size; i++) {
		pr_info("%s i: %d, gpio %d dir %ld", __func__, i,
			gpio_tbl[i].gpio, gpio_tbl[i].flags);
	}

	if (gpio_en) {
		for (i = 0; i < size; i++) {
			rc = aon_res_mgr_gpio_request(soc_info->dev,
					gpio_tbl[i].gpio,
					gpio_tbl[i].flags, gpio_tbl[i].label);
			if (rc) {
				pr_err("%s gpio %d:%s request fails", __func__,
					gpio_tbl[i].gpio, gpio_tbl[i].label);
			}
		}
	} else {
		aon_res_mgr_gpio_free_arry(gpio_tbl, size);
	}
	return rc;
}

static int aon_soc_util_regulator_enable(struct regulator *rgltr,
	const char *rgltr_name,
	u32 rgltr_min_volt, u32 rgltr_max_volt,
	u32 rgltr_op_mode, u32 rgltr_delay)
{
	s32 rc = 0;

	if (!rgltr) {
		pr_err("%s Invalid NULL parameter", __func__);
		return -EINVAL;
	}

	if (regulator_count_voltages(rgltr) > 0) {
		pr_info("%s voltage min=%d, max=%d", __func__,
			rgltr_min_volt, rgltr_max_volt);

		rc = regulator_set_voltage(
			rgltr, rgltr_min_volt, rgltr_max_volt);
		if (rc) {
			pr_err("%s %s set voltage failed", __func__, rgltr_name);
			return rc;
		}

		rc = regulator_set_load(rgltr, rgltr_op_mode);
		if (rc) {
			pr_err("%s %s set optimum mode failed", __func__,
				rgltr_name);
			return rc;
		}
	}

	rc = regulator_enable(rgltr);
	if (rc) {
		pr_err("%s %s regulator_enable failed", __func__,
				rgltr_name);
		return rc;
	}

	if (rgltr_delay > 20)
		msleep(rgltr_delay);
	else if (rgltr_delay)
		usleep_range(rgltr_delay * 1000,
			(rgltr_delay * 1000) + 1000);

	return rc;
}

static int aon_soc_util_set_clk_rate(struct clk *clk, const char *clk_name,
	s64 clk_rate)
{
	int rc = 0;
	long clk_rate_round;

	if (!clk || !clk_name) {
		pr_err("%s invalid params, clk %p, clk_name %p",
			__func__, clk, clk_name);
		return -EINVAL;
	}

	pr_info("%s set %s, rate %lld", __func__, clk_name, clk_rate);
	if (clk_rate > 0) {
		clk_rate_round = clk_round_rate(clk, clk_rate);
		pr_info("%s new_rate %ld", __func__, clk_rate_round);
		if (clk_rate_round < 0) {
			pr_err("%s round failed for clock %s rc = %ld", __func__,
				clk_name, clk_rate_round);
			return clk_rate_round;
		}
		rc = clk_set_rate(clk, clk_rate_round);
		if (rc) {
			pr_err("%s set_rate failed on %s", __func__, clk_name);
			return rc;
		}
	} else if (clk_rate == INIT_RATE) {
		clk_rate_round = clk_get_rate(clk);
		pr_info("%s init new_rate %ld", __func__, clk_rate_round);
		if (clk_rate_round == 0) {
			clk_rate_round = clk_round_rate(clk, 0);
			if (clk_rate_round <= 0) {
				pr_err("%s round rate failed on %s",__func__, clk_name);
				return clk_rate_round;
			}
		}
		rc = clk_set_rate(clk, clk_rate_round);
		if (rc) {
			pr_err("%s set_rate failed on %s", __func__, clk_name);
			return rc;
		}
	}

	return rc;
}

static int aon_soc_util_clk_enable(struct clk *clk,
	const char *clk_name, s32 clk_rate)
{
	int rc = 0;

	if (!clk || !clk_name) {
		pr_err("%s invalid params, clk %p, clk_name %p",
			__func__, clk, clk_name);
		return -EINVAL;
	}

	rc = aon_soc_util_set_clk_rate(clk, clk_name, clk_rate);
	if (rc)
		return rc;

	rc = clk_prepare_enable(clk);
	if (rc) {
		pr_err("%s enable failed for %s: rc(%d)", __func__, clk_name, rc);
		return rc;
	}

	return rc;
}

static int aon_soc_util_clk_disable(struct clk *clk, const char *clk_name)
{
	if (!clk || !clk_name) {
		pr_err("%s invalid params, clk %p, clk_name %p",
			__func__, clk, clk_name);
		return -EINVAL;
	}

	pr_info("%s disable %s", __func__, clk_name);
	clk_disable_unprepare(clk);

	return 0;
}

static int aon_soc_util_regulator_disable(struct regulator *rgltr,
	const char *rgltr_name, u32 rgltr_min_volt,
	u32 rgltr_max_volt, u32 rgltr_op_mode,
	u32 rgltr_delay_ms)
{
	s32 rc = 0;

	if (!rgltr) {
		pr_err("%s Invalid NULL parameter", __func__);
		return -EINVAL;
	}

	rc = regulator_disable(rgltr);
	if (rc) {
		pr_err("%s %s regulator disable failed", __func__, rgltr_name);
		return rc;
	}

	if (rgltr_delay_ms > 20)
		msleep(rgltr_delay_ms);
	else if (rgltr_delay_ms)
		usleep_range(rgltr_delay_ms * 1000,
			(rgltr_delay_ms * 1000) + 1000);

	if (regulator_count_voltages(rgltr) > 0) {
		regulator_set_load(rgltr, 0);
		regulator_set_voltage(rgltr, 0, rgltr_max_volt);
	}

	return rc;
}

static int aon_config_mclk_reg(struct aon_sensor_power_ctrl_t *ctrl,
	struct aon_hw_soc_info *soc_info, s32 index)
{
	s32 num_vreg = 0, j = 0, rc = 0, idx = 0;
	struct aon_sensor_power_setting *ps = NULL;
	struct aon_sensor_power_setting *pd = NULL;

	if (!ctrl || !soc_info) {
		pr_err("%s invalid params, ctrl %p, soc_info %p",
			 __func__, ctrl, soc_info);
		return -EINVAL;
	}

	num_vreg = soc_info->num_rgltr;

	pd = &ctrl->power_down_setting[index];

	for (j = 0; j < num_vreg; j++) {
		if (!strcmp(soc_info->rgltr_name[j], "cam_clk")) {
			pr_info("%s match cam_clk, j=%d", __func__, j);
			ps = NULL;
			for (idx = 0; idx < ctrl->power_setting_size; idx++) {
				if (ctrl->power_setting[idx].seq_type == pd->seq_type) {
					ps = &ctrl->power_setting[idx];
					break;
				}
			}

			if (ps != NULL) {
				pr_info("%s Disable MCLK Regulator", __func__);
				rc = aon_soc_util_regulator_disable(
					soc_info->rgltr[j],
					soc_info->rgltr_name[j],
					soc_info->rgltr_min_volt[j],
					soc_info->rgltr_max_volt[j],
					soc_info->rgltr_op_mode[j],
					soc_info->rgltr_delay[j]);

				if (rc) {
					pr_err("%s MCLK REG DISALBE FAILED: %d", __func__, rc);
					return rc;
				}

				ps->data[0] = soc_info->rgltr[j];

				regulator_put(soc_info->rgltr[j]);
				soc_info->rgltr[j] = NULL;
			}
		}
	}

	return rc;
}

static void aon_res_mgr_gpio_set_value(unsigned int gpio, int value)
{
	gpio_set_value_cansleep(gpio, value);
}

static void aon_cam_sensor_handle_reg_gpio(int seq_type,
	struct aon_camera_gpio_num_info *gpio_num_info, int val)
{
	int gpio_offset = -1;

	if (!gpio_num_info) {
		pr_info("%s Input Parameters are not proper", __func__);
		return;
	}

	pr_info("%s Seq type: %d, config: %d", __func__, seq_type, val);

	gpio_offset = seq_type;

	if (gpio_num_info->valid[gpio_offset] == 1) {
		pr_info("%s VALID GPIO offset: %d, seqtype: %d", __func__,
			 gpio_offset, seq_type);
		aon_res_mgr_gpio_set_value(gpio_num_info->gpio_num[gpio_offset], val);
	}
}

static int aon_res_mgr_shared_pinctrl_init(void)
{
	return 0;
}

static struct aon_sensor_power_setting *aon_get_power_settings(
	struct aon_sensor_power_ctrl_t *ctrl,
	enum msm_camera_power_seq_type seq_type,
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

static int aon_sensor_handle_reg_gpio(int seq_type,
	struct aon_camera_gpio_num_info *gpio_num_info, int val)
{
	int gpio_offset = -1;

	if (!gpio_num_info) {
		pr_info("%s: Input Parameters are not proper", __func__);
		return 0;
	}

	pr_info("%s: Seq type: %d, config: %d", __func__, seq_type, val);

	gpio_offset = seq_type;

	if (gpio_num_info->valid[gpio_offset] == 1) {
		pr_info("%s: VALID GPIO offset: %d, seqtype: %d", __func__,
			 gpio_offset, seq_type);
		aon_res_mgr_gpio_set_value(gpio_num_info->gpio_num[gpio_offset], val);
	}

	return 0;
}

static int aon_soc_util_get_dt_regulator_info(struct aon_sensor_ctrl_t *s_ctrl)
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

	if (!of_property_read_bool(of_node, "rgltr-cntrl-support")) {
		pr_info("%s No regulator control parameter defined", __func__);
		soc_info->rgltr_ctrl_support = false;
		return 0;
	}

	soc_info->rgltr_ctrl_support = true;

	rc = of_property_read_u32_array(of_node, "rgltr-min-voltage",
		soc_info->rgltr_min_volt, soc_info->num_rgltr);
	if (rc) {
		pr_err("%s No minimum volatage value found, rc=%d", __func__, rc);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(of_node, "rgltr-max-voltage",
		soc_info->rgltr_max_volt, soc_info->num_rgltr);
	if (rc) {
		pr_err("%s No maximum volatage value found, rc=%d", __func__, rc);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(of_node, "rgltr-load-current",
		soc_info->rgltr_op_mode, soc_info->num_rgltr);
	if (rc) {
		pr_err("%s No Load curent found rc=%d", __func__, rc);
		return -EINVAL;
	}

	return rc;
}

static int aon_soc_util_get_level_from_string(const char *string,
	enum cam_vote_level *level)
{
	if (!level) {
		pr_err("%s invalid params", __func__);
		return -EINVAL;
	}

	if (!strcmp(string, "suspend")) {
		*level = CAM_SUSPEND_VOTE;
	} else if (!strcmp(string, "minsvs")) {
		*level = CAM_MINSVS_VOTE;
	} else if (!strcmp(string, "lowsvs")) {
		*level = CAM_LOWSVS_VOTE;
	} else if (!strcmp(string, "svs")) {
		*level = CAM_SVS_VOTE;
	} else if (!strcmp(string, "svs_l1")) {
		*level = CAM_SVSL1_VOTE;
	} else if (!strcmp(string, "nominal")) {
		*level = CAM_NOMINAL_VOTE;
	} else if (!strcmp(string, "nominal_l1")) {
		*level = CAM_NOMINALL1_VOTE;
	} else if (!strcmp(string, "turbo")) {
		*level = CAM_TURBO_VOTE;
	} else {
		pr_err("%s Invalid string %s", __func__, string);
		return -EINVAL;
	}

	return 0;
}

static s32 aon_soc_util_get_dt_clk_info(struct aon_sensor_ctrl_t *s_ctrl)
{
	struct device_node *of_node = NULL;
	s32 count = 0;
	s32 num_clk_rates = 0, num_clk_levels = 0;
	s32 i = 0 , j = 0, rc = 0;
	s32 num_clk_level_strings = 0;
	const char *src_clk_str = NULL;
	const char *clk_control_debugfs = NULL;
	const char *clk_cntl_lvl_string = NULL;
	enum cam_vote_level level;
	struct aon_hw_soc_info *soc_info = NULL;

	if (!s_ctrl) {
		pr_err("%s invalid params", __func__);
		return -EINVAL;
	}

	soc_info = &s_ctrl->soc_info;
	of_node = s_ctrl->of_node;

	if (!of_property_read_bool(of_node, "use-shared-clk")) {
		pr_info("%s No shared clk parameter defined", __func__);
		soc_info->use_shared_clk = false;
	} else {
		soc_info->use_shared_clk = true;
	}

	count = of_property_count_strings(of_node, "clock-names");

	pr_info("%s E: count = %d", __func__, count);
	if (count > CAM_SOC_MAX_CLK) {
		pr_err("%s invalid count of clocks, count=%d", __func__, count);
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

	num_clk_rates = of_property_count_u32_elems(of_node, "clock-rates");
	if (num_clk_rates <= 0) {
		pr_err("%s reading clock-rates count failed", __func__);
		return -EINVAL;
	}

	if ((num_clk_rates % soc_info->num_clk) != 0) {
		pr_err("%s mismatch clk/rates, No of clocks=%d, No of rates=%d",
			__func__, soc_info->num_clk, num_clk_rates);
		return -EINVAL;
	}

	num_clk_levels = (num_clk_rates / soc_info->num_clk);

	num_clk_level_strings = of_property_count_strings(of_node,
		"clock-cntl-level");
	if (num_clk_level_strings != num_clk_levels) {
		pr_err("%s Mismatch No of levels=%d, No of level string=%d",
			__func__, num_clk_levels, num_clk_level_strings);
		return -EINVAL;
	}
	pr_info("%s num_clk_rates=%d soc_info->num_clk=%d num_clk_level_strings=%d",
		__func__,num_clk_rates, soc_info->num_clk, num_clk_level_strings);

	for (i = 0; i < num_clk_levels; i++) {
		rc = of_property_read_string_index(of_node,
			"clock-cntl-level", i, &clk_cntl_lvl_string);
		if (rc) {
			pr_err("%s Error reading clock-cntl-level, rc=%d", __func__, rc);
			return rc;
		}

		rc = aon_soc_util_get_level_from_string(clk_cntl_lvl_string, &level);
		if (rc) {
			pr_err("%s aon_soc_util_get_level_from_string rc=%d", __func__, rc);
			return rc;
		}

		pr_info("%s [%d] : %s %d", __func__, i, clk_cntl_lvl_string, level);
		soc_info->clk_level_valid[level] = true;
		for (j = 0; j < soc_info->num_clk; j++) {
			rc = of_property_read_u32_index(of_node, "clock-rates",
				((i * soc_info->num_clk) + j),
				&soc_info->clk_rate[level][j]);
			if (rc) {
				pr_err("%s Error reading clock-rates, rc=%d", __func__, rc);
				return rc;
			}

			soc_info->clk_rate[level][j] =
				(soc_info->clk_rate[level][j] == 0) ?
				(s32)NO_SET_RATE :
				soc_info->clk_rate[level][j];

			pr_info("%s soc_info->clk_rate[%d][%d] = %d",
				__func__, level, j,
				soc_info->clk_rate[level][j]);
		}
	}

	soc_info->src_clk_idx = -1;
	rc = of_property_read_string_index(of_node, "src-clock-name", 0,
		&src_clk_str);
	if (rc || !src_clk_str) {
		pr_info("%s No src_clk_str found", __func__);
		rc = 0;
		goto end;
	}

	for (i = 0; i < soc_info->num_clk; i++) {
		if (strcmp(soc_info->clk_name[i], src_clk_str) == 0) {
			soc_info->src_clk_idx = i;
			pr_info("%s src clock = %s, index = %d",
				__func__, src_clk_str, i);
			break;
		}
	}

	rc = of_property_read_string_index(of_node,
		"clock-control-debugfs", 0, &clk_control_debugfs);
	if (rc || !clk_control_debugfs) {
		pr_info("%s No clock_control_debugfs property found", __func__);
		rc = 0;
		goto end;
	}

	if (strcmp("true", clk_control_debugfs) == 0)
		soc_info->clk_control_enable = true;

	pr_info("%s X: dev_name = %s count = %d",
		__func__, soc_info->dev_name, count);
end:
	return rc;
}

static int aon_soc_util_get_dt_gpio_req_tbl(struct device_node *of_node,
	struct aon_soc_gpio_data *gconf, u16 *gpio_array,
	u16 gpio_array_size)
{
	s32 rc = 0, i = 0;
	u32 count = 0;
	u32 *val_array = NULL;

	if (!of_node || !gconf || !gpio_array) {
		pr_err("%s invalid params, of_node %p, gconf %p, gpio_array %p",
			__func__, of_node, gconf, gpio_array);
		return -EINVAL;
	}

	if (!of_get_property(of_node, "gpio-req-tbl-num", &count))
		return 0;

	count /= sizeof(u32);
	if (!count) {
		pr_err("%s gpio-req-tbl-num 0", __func__);
		return 0;
	}

	val_array = kcalloc(count, sizeof(u32), GFP_KERNEL);
	if (!val_array)
		return -ENOMEM;
	pr_info("%s val_array: %p mm-kcalloc-size: %d",
		__func__, val_array, count * sizeof(u32));

	gconf->cam_gpio_req_tbl = kcalloc(count, sizeof(struct gpio),
		GFP_KERNEL);
	if (!gconf->cam_gpio_req_tbl) {
		rc = -ENOMEM;
		goto free_val_array;
	}
	pr_info("%s gconf->cam_gpio_req_tbl: %p mm-kcalloc-size: %d",
		__func__, gconf->cam_gpio_req_tbl, count * sizeof(struct gpio));
	gconf->cam_gpio_req_tbl_size = count;

	rc = of_property_read_u32_array(of_node, "gpio-req-tbl-num",
		val_array, count);
	if (rc) {
		pr_err("%s failed in reading gpio-req-tbl-num, rc=%d\n", __func__, rc);
		goto free_gpio_req_tbl;
	}

	for (i = 0; i < count; i++) {
		if (val_array[i] >= gpio_array_size) {
			pr_err("%s gpio req tbl index %d invalid", __func__, val_array[i]);
			goto free_gpio_req_tbl;
		}
		gconf->cam_gpio_req_tbl[i].gpio = gpio_array[val_array[i]];
		pr_info("%s cam_gpio_req_tbl[%d].gpio = %d\n", __func__, i,
			gconf->cam_gpio_req_tbl[i].gpio);
	}

	rc = of_property_read_u32_array(of_node, "gpio-req-tbl-flags",
		val_array, count);
	if (rc) {
		pr_err("%s Failed in gpio-req-tbl-flags, rc %d\n", __func__, rc);
		goto free_gpio_req_tbl;
	}

	for (i = 0; i < count; i++) {
		gconf->cam_gpio_req_tbl[i].flags = val_array[i];
		pr_info("%s cam_gpio_req_tbl[%d].flags = %ld\n", __func__, i,
			gconf->cam_gpio_req_tbl[i].flags);
	}

	for (i = 0; i < count; i++) {
		rc = of_property_read_string_index(of_node,
			"gpio-req-tbl-label", i,
			&gconf->cam_gpio_req_tbl[i].label);
		if (rc) {
			pr_err("%s Failed rc %d\n", __func__, rc);
			goto free_gpio_req_tbl;
		}
		pr_info("%s cam_gpio_req_tbl[%d].label = %s\n", __func__, i,
			gconf->cam_gpio_req_tbl[i].label);
	}

	pr_info("%s mm-kfree val_array: %p", __func__, val_array);
	kfree(val_array);
	val_array = NULL;

	return rc;

free_gpio_req_tbl:
	pr_info("%s mm-kfree gconf->cam_gpio_req_tbl: %p",
		__func__, gconf->cam_gpio_req_tbl);
	kfree(gconf->cam_gpio_req_tbl);
	gconf->cam_gpio_req_tbl = NULL;
free_val_array:
	pr_info("%s mm-kfree val_array:%p", __func__, val_array);
	kfree(val_array);
	val_array = NULL;

	gconf->cam_gpio_req_tbl_size = 0;

	return rc;
}

static int aon_soc_util_get_gpio_info(struct aon_sensor_ctrl_t *s_ctrl)
{
	s32 rc = 0, i = 0;
	u16 *gpio_array = NULL;
	s16 gpio_array_size = 0;
	struct aon_soc_gpio_data *gconf = NULL;
	struct device_node *of_node = NULL;
	struct aon_hw_soc_info *soc_info = NULL;

	if (!s_ctrl) {
		pr_err("%s invalid args", __func__);
		return -EINVAL;
	}

	of_node = s_ctrl->of_node;
	soc_info = &s_ctrl->soc_info;

	/* Validate input parameters */
	if (!of_node) {
		pr_err("%s Invalid param of_node", __func__);
		return -EINVAL;
	}

	gpio_array_size = of_gpio_count(of_node);

	if (gpio_array_size <= 0) {
		pr_info("%s gpio count %d", __func__, gpio_array_size);
		return 0;
	}

	pr_info("%s gpio count %d", __func__, gpio_array_size);
	gpio_array = kcalloc(gpio_array_size, sizeof(u16), GFP_KERNEL);
	if (!gpio_array) {
		pr_err("%s kcalloc failed", __func__);
		goto free_gpio_conf;
	}
	pr_info("%s gpio_array:%p mm-kcalloc-size: %d",
		__func__, gpio_array, gpio_array_size * sizeof(u16));

	for (i = 0; i < gpio_array_size; i++) {
		gpio_array[i] = of_get_gpio(of_node, i);
		pr_info("%s gpio_array[%d] = %d\n", __func__, i, gpio_array[i]);
	}

	gconf = kzalloc(sizeof(*gconf), GFP_KERNEL);
	if (!gconf) {
		pr_err("%s kcalloc failed", __func__);
		return -ENOMEM;
	}
	pr_info("%s gconf: %p mm-kzalloc-size: %d",
		__func__, gconf, sizeof(*gconf));

	rc = aon_soc_util_get_dt_gpio_req_tbl(of_node, gconf, gpio_array,
		gpio_array_size);
	if (rc) {
		pr_err("%s failed in msm_camera_get_dt_gpio_req_tbl\n", __func__);
		goto free_gpio_array;
	}

	gconf->cam_gpio_common_tbl = kcalloc(gpio_array_size,
				sizeof(struct gpio), GFP_KERNEL);
	if (!gconf->cam_gpio_common_tbl) {
		rc = -ENOMEM;
		goto free_gpio_array;
	}
	pr_info("%s gconf->cam_gpio_common_tbl:%p mm-kcalloc-size: %d",
		__func__, gconf->cam_gpio_common_tbl,
		gpio_array_size * sizeof(struct gpio));

	for (i = 0; i < gpio_array_size; i++)
		gconf->cam_gpio_common_tbl[i].gpio = gpio_array[i];

	gconf->cam_gpio_common_tbl_size = gpio_array_size;
	soc_info->gpio_data = gconf;

	pr_info("%s mm-kfree gpio_array: %p", __func__, gpio_array);
	kfree(gpio_array);
	gpio_array = NULL;

	return rc;

free_gpio_array:
	pr_info("%s mm-kfree gpio_array: %p", __func__, gpio_array);
	kfree(gpio_array);
	gpio_array = NULL;

free_gpio_conf:
	pr_info("%s mm-kfree gconf: %p", __func__, gconf);
	kfree(gconf);
	gconf = NULL;
	soc_info->gpio_data = NULL;

	return rc;
}

static int aon_soc_util_get_dt_properties(struct aon_sensor_ctrl_t *s_ctrl)
{
	struct device_node *of_node = NULL;
	int count = 0, i = 0, rc = 0;
	struct aon_hw_soc_info *soc_info = NULL;

	if (!s_ctrl) {
		pr_err("%s invalid params", __func__);
		return -EINVAL;
	}

	of_node = s_ctrl->of_node;
	soc_info = &s_ctrl->soc_info;

	soc_info->pdev = of_find_device_by_node(of_node);
	if (!soc_info->pdev) {
		pr_err("%s of_find_device_by_node failed", __func__);
		return -ENODEV;
	}

	s_ctrl->pdev = soc_info->pdev;
	soc_info->dev = &soc_info->pdev->dev;
	soc_info->dev_name = soc_info->pdev->name;

	rc = of_property_read_u32(of_node, "cell-index", &soc_info->index);
	if (rc) {
		pr_err("%s device failed to read cell-index", __func__);
		return rc;
	}
	pr_info("%s soc index=%d", __func__, soc_info->index);

	count = of_property_count_strings(of_node, "reg-names");
	if (count <= 0) {
		pr_info("%s no reg-names found", __func__);
		count = 0;
	}
	soc_info->num_mem_block = count;
	pr_info("%s soc_info->num_mem_block=%d\n", __func__, soc_info->num_mem_block);

	for (i = 0; i < soc_info->num_mem_block; i++) {
		rc = of_property_read_string_index(of_node, "reg-names", i,
			&soc_info->mem_block_name[i]);
		if (rc) {
			pr_err("%s failed to read reg-names at %d", __func__, i);
			return rc;
		}
		soc_info->mem_block[i] =
			platform_get_resource_byname(soc_info->pdev,
			IORESOURCE_MEM, soc_info->mem_block_name[i]);

		if (!soc_info->mem_block[i]) {
			pr_err("%s no mem resource by name %s", __func__,
				soc_info->mem_block_name[i]);
			rc = -ENODEV;
			return rc;
		}
	}

	if (soc_info->num_mem_block > 0) {
		rc = of_property_read_u32_array(of_node, "reg-cam-base",
			soc_info->mem_block_cam_base, soc_info->num_mem_block);
		if (rc) {
			pr_err("%s Error reading register offsets", __func__);
			return rc;
		}
	}

	rc = of_property_read_string_index(of_node, "interrupt-names", 0,
		&soc_info->irq_name);
	if (rc) {
		pr_info("%s No interrupt line preset", __func__);
		rc = 0;
	} else {
		soc_info->irq_line =
			platform_get_resource_byname(soc_info->pdev,
			IORESOURCE_IRQ, soc_info->irq_name);
		if (!soc_info->irq_line) {
			pr_err("%s no irq resource", __func__);
			rc = -ENODEV;
			return rc;
		}
	}

	rc = aon_soc_util_get_dt_regulator_info(s_ctrl);
	if (rc)
		return rc;

	rc = aon_soc_util_get_dt_clk_info(s_ctrl);
	if (rc)
		return rc;

	rc = aon_soc_util_get_gpio_info(s_ctrl);

	return rc;
}

static int aon_sensor_util_init_gpio_pin_tbl(
	struct aon_sensor_ctrl_t *s_ctrl,
	struct aon_camera_gpio_num_info **pgpio_num_info)
{
	int rc = 0, val = 0;
	u32 gpio_array_size;
	struct device_node *of_node = NULL;
	struct aon_soc_gpio_data *gconf = NULL;
	struct aon_camera_gpio_num_info *gpio_num_info = NULL;
	struct aon_hw_soc_info *soc_info = NULL;

	if (!s_ctrl || !pgpio_num_info) {
		pr_err("%s device node NULL, s_ctrl %p, pgpio_num_info %p\n",
			__func__, s_ctrl, pgpio_num_info);
		return -EINVAL;
	}

	soc_info = &s_ctrl->soc_info;
	of_node = s_ctrl->of_node;

	gconf = soc_info->gpio_data;
	if (!gconf) {
		pr_err("%s No gpio_common_table is found", __func__);
		return -EINVAL;
	}

	if (!gconf->cam_gpio_common_tbl) {
		pr_err("%s gpio_common_table is not initialized", __func__);
		return -EINVAL;
	}

	gpio_array_size = gconf->cam_gpio_common_tbl_size;

	if (!gpio_array_size) {
		pr_err("%s invalid size of gpio table", __func__);
		return -EINVAL;
	}

	*pgpio_num_info = kzalloc(sizeof(struct aon_camera_gpio_num_info),
		GFP_KERNEL);
	if (!*pgpio_num_info)
		return -ENOMEM;
	gpio_num_info = *pgpio_num_info;
	pr_info("%s gpio_num_info: %p mm-kzalloc-size: %d",
		__func__, gpio_num_info, sizeof(struct aon_camera_gpio_num_info));

	rc = of_property_read_u32(of_node, "gpio-vana", &val);
	if (rc != -EINVAL) {
		if (rc < 0) {
			pr_err("%s read gpio-vana failed rc %d", __func__, rc);
			goto free_gpio_info;
		} else if (val >= gpio_array_size) {
			pr_err("%s gpio-vana invalid %d", __func__, val);
			rc = -EINVAL;
			goto free_gpio_info;
		}
		gpio_num_info->gpio_num[SENSOR_VANA] =
				gconf->cam_gpio_common_tbl[val].gpio;
		gpio_num_info->valid[SENSOR_VANA] = 1;

		pr_info("%s gpio-vana %d", __func__,
			gpio_num_info->gpio_num[SENSOR_VANA]);
	}

	rc = of_property_read_u32(of_node, "gpio-vio", &val);
	if (rc != -EINVAL) {
		if (rc < 0) {
			pr_err("%s read gpio-vio failed rc %d", __func__, rc);
			goto free_gpio_info;
		} else if (val >= gpio_array_size) {
			pr_err("%s gpio-vio invalid %d", __func__, val);
			goto free_gpio_info;
		}
		gpio_num_info->gpio_num[SENSOR_VIO] =
			gconf->cam_gpio_common_tbl[val].gpio;
		gpio_num_info->valid[SENSOR_VIO] = 1;

		pr_info("%s gpio-vio %d", __func__, gpio_num_info->gpio_num[SENSOR_VIO]);
	}

	rc = of_property_read_u32(of_node, "gpio-vaf", &val);
	if (rc != -EINVAL) {
		if (rc < 0) {
			pr_err("%s read gpio-vaf failed rc %d", __func__, rc);
			goto free_gpio_info;
		} else if (val >= gpio_array_size) {
			pr_err("%s gpio-vaf invalid %d", __func__, val);
			rc = -EINVAL;
			goto free_gpio_info;
		}
		gpio_num_info->gpio_num[SENSOR_VAF] =
			gconf->cam_gpio_common_tbl[val].gpio;
		gpio_num_info->valid[SENSOR_VAF] = 1;

		pr_info("%s gpio-vaf %d", __func__,
			gpio_num_info->gpio_num[SENSOR_VAF]);
	}

	rc = of_property_read_u32(of_node, "gpio-vdig", &val);
	if (rc != -EINVAL) {
		if (rc < 0) {
			pr_err("%s read gpio-vdig failed rc %d", __func__, rc);
			goto free_gpio_info;
		} else if (val >= gpio_array_size) {
			pr_err("%s gpio-vdig invalid %d", __func__, val);
			rc = -EINVAL;
			goto free_gpio_info;
		}
		gpio_num_info->gpio_num[SENSOR_VDIG] =
			gconf->cam_gpio_common_tbl[val].gpio;
		gpio_num_info->valid[SENSOR_VDIG] = 1;

		pr_info("%s gpio-vdig %d", __func__,
				gpio_num_info->gpio_num[SENSOR_VDIG]);
	}

	rc = of_property_read_u32(of_node, "gpio-reset", &val);
	if (rc != -EINVAL) {
		if (rc < 0) {
			pr_err("%s read gpio-reset failed rc %d", __func__, rc);
			goto free_gpio_info;
		} else if (val >= gpio_array_size) {
			pr_err("%s gpio-reset invalid %d", __func__, val);
			rc = -EINVAL;
			goto free_gpio_info;
		}
		gpio_num_info->gpio_num[SENSOR_RESET] =
			gconf->cam_gpio_common_tbl[val].gpio;
		gpio_num_info->valid[SENSOR_RESET] = 1;

		pr_info("%s gpio-reset %d", __func__,
			gpio_num_info->gpio_num[SENSOR_RESET]);
	}

	rc = of_property_read_u32(of_node, "gpio-standby", &val);
	if (rc != -EINVAL) {
		if (rc < 0) {
			pr_err("%s read gpio-standby failed rc %d", __func__, rc);
			goto free_gpio_info;
		} else if (val >= gpio_array_size) {
			pr_err("%s gpio-standby invalid %d", __func__, val);
			rc = -EINVAL;
			goto free_gpio_info;
		}
		gpio_num_info->gpio_num[SENSOR_STANDBY] =
			gconf->cam_gpio_common_tbl[val].gpio;
		gpio_num_info->valid[SENSOR_STANDBY] = 1;

		pr_info("%s gpio-standby %d", __func__,
			gpio_num_info->gpio_num[SENSOR_STANDBY]);
	}

	rc = of_property_read_u32(of_node, "gpio-af-pwdm", &val);
	if (rc != -EINVAL) {
		if (rc < 0) {
			pr_err("%s read gpio-af-pwdm failed rc %d", __func__, rc);
			goto free_gpio_info;
		} else if (val >= gpio_array_size) {
			pr_err("%s gpio-af-pwdm invalid %d", __func__, val);
			rc = -EINVAL;
			goto free_gpio_info;
		}
		gpio_num_info->gpio_num[SENSOR_VAF_PWDM] =
			gconf->cam_gpio_common_tbl[val].gpio;
		gpio_num_info->valid[SENSOR_VAF_PWDM] = 1;

		pr_info("%s gpio-af-pwdm %d", __func__,
			gpio_num_info->gpio_num[SENSOR_VAF_PWDM]);
	}

	rc = of_property_read_u32(of_node, "gpio-custom1", &val);
	if (rc != -EINVAL) {
		if (rc < 0) {
			pr_err("%s read gpio-custom1 failed rc %d", __func__, rc);
			goto free_gpio_info;
		} else if (val >= gpio_array_size) {
			pr_err("%s gpio-custom1 invalid %d", __func__, val);
			rc = -EINVAL;
			goto free_gpio_info;
		}
		gpio_num_info->gpio_num[SENSOR_CUSTOM_GPIO1] =
			gconf->cam_gpio_common_tbl[val].gpio;
		gpio_num_info->valid[SENSOR_CUSTOM_GPIO1] = 1;

		pr_info("%s gpio-custom1 %d", __func__,
			gpio_num_info->gpio_num[SENSOR_CUSTOM_GPIO1]);
	}

	rc = of_property_read_u32(of_node, "gpio-custom2", &val);
	if (rc != -EINVAL) {
		if (rc < 0) {
			pr_err("%s read gpio-custom2 failed rc %d", __func__, rc);
			goto free_gpio_info;
		} else if (val >= gpio_array_size) {
			pr_err("%s gpio-custom2 invalid %d", __func__, val);
			rc = -EINVAL;
			goto free_gpio_info;
		}
		gpio_num_info->gpio_num[SENSOR_CUSTOM_GPIO2] =
			gconf->cam_gpio_common_tbl[val].gpio;
		gpio_num_info->valid[SENSOR_CUSTOM_GPIO2] = 1;

		pr_info("%s gpio-custom2 %d", __func__,
			gpio_num_info->gpio_num[SENSOR_CUSTOM_GPIO2]);
	} else {
		rc = 0;
	}

	return rc;

free_gpio_info:
	pr_info("%s mm-kfree gpio_num_info: %p", __func__, gpio_num_info);
	kfree(gpio_num_info);
	gpio_num_info = NULL;
	return rc;
}

static s32 aon_sensor_get_sub_module_index(struct device_node *of_node,
	struct aon_sensor_board_info *s_info)
{
	int rc = 0, i = 0;
	u32 val = 0;
	struct device_node *src_node = NULL;
	struct aon_sensor_board_info *sensor_info = NULL;

	if (!of_node || !s_info) {
		pr_err("%s invalid params, of_node %p, s_info %p",
			__func__, of_node, s_info);
		return -EINVAL;
	}

	sensor_info = s_info;

	for (i = 0; i < SUB_MODULE_MAX; i++)
		sensor_info->subdev_id[i] = -1;

	src_node = of_parse_phandle(of_node, "actuator-src", 0);
	if (!src_node) {
		pr_err("%s src_node NULL", __func__);
	} else {
		rc = of_property_read_u32(src_node, "cell-index", &val);
		pr_info("%s actuator cell index %d, rc %d", __func__, val, rc);
		if (rc < 0) {
			pr_err("%s failed %d", __func__, rc);
			of_node_put(src_node);
			return rc;
		}
		sensor_info->subdev_id[SUB_MODULE_ACTUATOR] = val;
		of_node_put(src_node);
	}

	src_node = of_parse_phandle(of_node, "ois-src", 0);
	if (!src_node) {
		pr_info("%s src_node NULL", __func__);
	} else {
		rc = of_property_read_u32(src_node, "cell-index", &val);
		pr_info("%s ois cell index %d, rc %d", __func__, val, rc);
		if (rc < 0) {
			pr_err("%s failed %d", __func__,  rc);
			of_node_put(src_node);
			return rc;
		}
		sensor_info->subdev_id[SUB_MODULE_OIS] = val;
		of_node_put(src_node);
	}

	src_node = of_parse_phandle(of_node, "eeprom-src", 0);
	if (!src_node) {
		pr_info("%s eeprom src_node NULL", __func__);
	} else {
		rc = of_property_read_u32(src_node, "cell-index", &val);
		pr_info("%s eeprom cell index %d, rc %d", __func__, val, rc);
		if (rc < 0) {
			pr_err("%s failed %d", __func__, rc);
			of_node_put(src_node);
			return rc;
		}
		sensor_info->subdev_id[SUB_MODULE_EEPROM] = val;
		of_node_put(src_node);
	}

	src_node = of_parse_phandle(of_node, "led-flash-src", 0);
	if (!src_node) {
		pr_info("%s src_node NULL", __func__);
	} else {
		rc = of_property_read_u32(src_node, "cell-index", &val);
		pr_info("%s led flash cell index %d, rc %d", __func__, val, rc);
		if (rc < 0) {
			pr_err("%s failed %d", __func__, rc);
			of_node_put(src_node);
			return rc;
		}
		sensor_info->subdev_id[SUB_MODULE_LED_FLASH] = val;
		of_node_put(src_node);
	}

	if (of_property_read_u32(of_node, "csiphy-sd-index", &val) < 0)
		pr_err("%s paring the dt node for csiphy rc %d", __func__, rc);
	else
		sensor_info->subdev_id[SUB_MODULE_CSIPHY] = val;

	return rc;
}

static s32 aon_sensor_driver_get_dt_data(struct aon_sensor_ctrl_t *s_ctrl)
{
	s32 rc = 0;
	int i = 0;
	struct aon_sensor_board_info *sensordata = NULL;
	struct device_node *of_node = NULL;
	struct device_node *of_parent = NULL;
	struct aon_hw_soc_info *soc_info = NULL;

	if (!s_ctrl) {
		pr_err("%s invalid param", __func__);
		return -EINVAL;
	}

	soc_info = &s_ctrl->soc_info;

	pr_info("%s before of_find_compatible_node", __func__);
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if(s_ctrl->isAuxSensor){
		s_ctrl->of_node = of_find_compatible_node(NULL, NULL, "zeku,aon-auxsensor");
	}else{
		s_ctrl->of_node = of_find_compatible_node(NULL, NULL, "zeku,aon-sensor");
	}
#else
	s_ctrl->of_node = of_find_compatible_node(NULL, NULL, "zeku,aon-sensor");
#endif
	if (!s_ctrl->of_node) {
		pr_err("%s of_find_compatible_node failed", __func__);
		return -ENODEV;
	}
	pr_info("%s after of_find_compatible_node", __func__);
	of_node = s_ctrl->of_node;

	pr_info("%s before s_ctrl->sensordata kzalloc", __func__);
	s_ctrl->sensordata = kzalloc(sizeof(*sensordata), GFP_KERNEL);
	if (!s_ctrl->sensordata) {
		pr_err("%s s_ctrl->sensordata kzalloc failed\n", __func__);
		return -ENOMEM;
	}
	pr_info("%s s_ctrl->sensordata: %p mm-kzalloc-size: %d",
		__func__, s_ctrl->sensordata, sizeof(*sensordata));

	sensordata = s_ctrl->sensordata;
	pr_info("%s s_ctrl->sensordata kzalloc ok\n", __func__);

	rc = aon_soc_util_get_dt_properties(s_ctrl);
	if (rc < 0) {
		pr_err("%s Failed to read DT properties rc %d\n", __func__, rc);
		goto free_sensor_data;
	}
	pr_info("%s aon_soc_util_get_dt_properties ok\n", __func__);

	rc = aon_sensor_util_init_gpio_pin_tbl(s_ctrl,
			&sensordata->power_info.gpio_num_info);
	if (rc < 0) {
		pr_err("%s Failed to read gpios %d\n", __func__, rc);
		goto free_sensor_data;
	}
	pr_info("%s aon_sensor_util_init_gpio_pin_tbl ok\n", __func__);

	s_ctrl->id = soc_info->index;

	/* Validate cell_id */
	if (s_ctrl->id >= MAX_CAMERAS) {
		pr_err("%s Failed invalid cell_id %d", __func__, s_ctrl->id);
		rc = -EINVAL;
		goto free_sensor_data;
	}

	pr_info("%s num_rgltr: %d", __func__, soc_info->num_rgltr);
	/* Store the index of BoB regulator if it is available */
	for (i = 0; i < soc_info->num_rgltr; i++) {
		if (!strcmp(soc_info->rgltr_name[i], "cam_bob")) {
			pr_info("%s i: %d cam_bob", __func__, i);
			s_ctrl->bob_reg_index = i;
			soc_info->rgltr[i] = devm_regulator_get(soc_info->dev,
				soc_info->rgltr_name[i]);
			if (IS_ERR_OR_NULL(soc_info->rgltr[i])) {
				pr_err("%s Regulator: %s get failed", __func__,
					soc_info->rgltr_name[i]);
				soc_info->rgltr[i] = NULL;
			} else {
				if (!of_property_read_bool(of_node, "pwm-switch")) {
					pr_info("%s No BoB PWM switch param defined", __func__);
					s_ctrl->bob_pwm_switch = false;
				} else {
					s_ctrl->bob_pwm_switch = true;
				}
			}
		}
	}

	/* Read subdev info */
	rc = aon_sensor_get_sub_module_index(of_node, sensordata);
	if (rc < 0) {
		pr_err("%s failed to get sub module index, rc=%d", __func__, rc);
		goto free_sensor_data;
	}

	/* Get CCI master */
	rc = of_property_read_u32(of_node, "cci-master",
		&s_ctrl->cci_i2c_master);
	pr_info("%s cci-master %d, rc %d", __func__, s_ctrl->cci_i2c_master, rc);
	if (rc < 0) {
		/* Set default master 0 */
		s_ctrl->cci_i2c_master = MASTER_0;
		rc = 0;
	}

	pr_info("%s to read parent cell-index", __func__);
	of_parent = of_get_parent(of_node);
	if (of_property_read_u32(of_parent, "cell-index",
			&s_ctrl->cci_num) < 0) {
		/* Set default master 0 */
		pr_err("%s read parent cell-index failed", __func__);
		s_ctrl->cci_num = CCI_DEVICE_0;
	}
	pr_info("%s cci-device %d, rc %d", __func__, s_ctrl->cci_num, rc);


	if (of_property_read_u32(of_node, "sensor-position-pitch",
		&sensordata->pos_pitch) < 0) {
		pr_info("Invalid sensor position");
		sensordata->pos_pitch = 360;
	}
	if (of_property_read_u32(of_node, "sensor-position-roll",
		&sensordata->pos_roll) < 0) {
		pr_info("Invalid sensor position");
		sensordata->pos_roll = 360;
	}
	if (of_property_read_u32(of_node, "sensor-position-yaw",
		&sensordata->pos_yaw) < 0) {
		pr_info("Invalid sensor position");
		sensordata->pos_yaw = 360;
	}

	return rc;

free_sensor_data:
	pr_info("%s sensordata: %p mm-kfree", __func__, sensordata);
	kfree(sensordata);
	return rc;
}

static s32 aon_sensor_init_default_params(struct aon_sensor_ctrl_t *s_ctrl)
{
	/* Validate input parameters */
	if (!s_ctrl) {
		pr_err("aon failed: invalid params s_ctrl %p", s_ctrl);
		return -EINVAL;
	}

	/* Initialize cci_client */
	s_ctrl->cci_client = kzalloc(sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
	if (!(s_ctrl->cci_client)) {
		pr_err("%s kzalloc failed", __func__);
		return -ENOMEM;
	}
	pr_info("%s s_ctrl->cci_client: %p mm-kzalloc-size: %d",
		__func__, s_ctrl->cci_client, sizeof(struct cam_sensor_cci_client));
	s_ctrl->cci_client->cci_device = s_ctrl->cci_num;
	pr_info("%s cci_device: %d", __func__, s_ctrl->cci_num);

	return 0;
}

s32 aon_sensor_parse_dt(struct aon_sensor_ctrl_t *s_ctrl)
{
	s32 i = 0;
	int rc = 0;
	struct aon_hw_soc_info *soc_info = NULL;

	if (!s_ctrl) {
		pr_err("%s invalid args", __func__);
		return -EINVAL;
	}

	soc_info = &s_ctrl->soc_info;

	/* Parse dt information and store in sensor control structure */
	rc = aon_sensor_driver_get_dt_data(s_ctrl);
	if (rc < 0) {
		pr_err("%s Failed to get dt data rc %d", __func__, rc);
		return rc;
	}

	/* Initialize mutex */
	mutex_init(&(s_ctrl->aon_sensor_mutex));

	/* Initialize default parameters */
	for (i = 0; i < soc_info->num_clk; i++) {
		soc_info->clk[i] = devm_clk_get(soc_info->dev,
					soc_info->clk_name[i]);
		if (!soc_info->clk[i]) {
			pr_err("%s get clk failed for %s", __func__, soc_info->clk_name[i]);
			rc = -ENOENT;
			return rc;
		}
	}

	rc = aon_sensor_init_default_params(s_ctrl);
	if (rc < 0) {
		pr_err("%s failed: aon_sensor_init_default_params rc %d", __func__, rc);
		goto free_dt_data;
	}

	return rc;

free_dt_data:
	pr_info("%s s_ctrl->sensordata: %p mm-kfree", __func__, s_ctrl->sensordata);
	kfree(s_ctrl->sensordata);
	s_ctrl->sensordata = NULL;

	return rc;
}

s32 aon_fill_vreg_params(
	struct aon_hw_soc_info *soc_info,
	struct aon_sensor_power_setting *power_setting,
	u16 power_setting_size)
{
	s32 rc = 0, j = 0, i = 0;
	int num_vreg;

	/* Validate input parameters */
	if (!soc_info || !power_setting || (power_setting_size <= 0)) {
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

		if (power_setting[i].seq_type < SENSOR_MCLK ||
			power_setting[i].seq_type >= SENSOR_SEQ_TYPE_MAX) {
			pr_err("%s %d failed: Invalid Seq type: %d", __func__, __LINE__,
				power_setting[i].seq_type);
			return -EINVAL;
		}

		switch (power_setting[i].seq_type) {
		case SENSOR_VDIG:
			for (j = 0; j < num_vreg; j++) {
				if (!strcmp(soc_info->rgltr_name[j], "cam_vdig")) {
					pr_info("%s i: %d j: %d cam_vdig", __func__, i, j);
					power_setting[i].seq_val = j;

					if (VALIDATE_VOLTAGE(
						soc_info->rgltr_min_volt[j],
						soc_info->rgltr_max_volt[j],
						power_setting[i].config_val)) {
						soc_info->rgltr_min_volt[j] =
						soc_info->rgltr_max_volt[j] =
						power_setting[i].config_val;
					}
					break;
				}
			}
			if (j == num_vreg)
				power_setting[i].seq_val = INVALID_VREG;
			break;

		case SENSOR_VIO:
			for (j = 0; j < num_vreg; j++) {
				if (!strcmp(soc_info->rgltr_name[j], "cam_vio")) {
					pr_info("%s i: %d j: %d cam_vio", __func__, i, j);
					power_setting[i].seq_val = j;

					if (VALIDATE_VOLTAGE(
						soc_info->rgltr_min_volt[j],
						soc_info->rgltr_max_volt[j],
						power_setting[i].config_val)) {
						soc_info->rgltr_min_volt[j] =
						soc_info->rgltr_max_volt[j] =
						power_setting[i].config_val;
					}
					break;
				}
			}
			if (j == num_vreg)
				power_setting[i].seq_val = INVALID_VREG;
			break;

		case SENSOR_VANA:
			for (j = 0; j < num_vreg; j++) {
				if (!strcmp(soc_info->rgltr_name[j], "cam_vana")) {
					pr_info("%s i: %d j: %d cam_vana", __func__, i, j);
					power_setting[i].seq_val = j;

					if (VALIDATE_VOLTAGE(
						soc_info->rgltr_min_volt[j],
						soc_info->rgltr_max_volt[j],
						power_setting[i].config_val)) {
						soc_info->rgltr_min_volt[j] =
						soc_info->rgltr_max_volt[j] =
						power_setting[i].config_val;
					}
					break;
				}
			}
			if (j == num_vreg)
				power_setting[i].seq_val = INVALID_VREG;
			break;

		case SENSOR_VAF:
			for (j = 0; j < num_vreg; j++) {
				if (!strcmp(soc_info->rgltr_name[j], "cam_vaf")) {
					pr_info("%s i: %d j: %d cam_vaf", __func__, i, j);
					power_setting[i].seq_val = j;

					if (VALIDATE_VOLTAGE(
						soc_info->rgltr_min_volt[j],
						soc_info->rgltr_max_volt[j],
						power_setting[i].config_val)) {
						soc_info->rgltr_min_volt[j] =
						soc_info->rgltr_max_volt[j] =
						power_setting[i].config_val;
					}

					break;
				}
			}
			if (j == num_vreg)
				power_setting[i].seq_val = INVALID_VREG;
			break;

		case SENSOR_CUSTOM_REG1:
			for (j = 0; j < num_vreg; j++) {
				if (!strcmp(soc_info->rgltr_name[j], "cam_v_custom1")) {
					pr_info("%s i: %d j: %d cam_vcustom1", __func__, i, j);
					power_setting[i].seq_val = j;

					if (VALIDATE_VOLTAGE(
						soc_info->rgltr_min_volt[j],
						soc_info->rgltr_max_volt[j],
						power_setting[i].config_val)) {
						soc_info->rgltr_min_volt[j] =
						soc_info->rgltr_max_volt[j] =
						power_setting[i].config_val;
					}
					break;
				}
			}
			if (j == num_vreg)
				power_setting[i].seq_val = INVALID_VREG;
			break;
		case SENSOR_CUSTOM_REG2:
			for (j = 0; j < num_vreg; j++) {
				if (!strcmp(soc_info->rgltr_name[j], "cam_v_custom2")) {
					pr_info("%s i: %d j: %d cam_vcustom2", __func__, i, j);
					power_setting[i].seq_val = j;

					if (VALIDATE_VOLTAGE(
						soc_info->rgltr_min_volt[j],
						soc_info->rgltr_max_volt[j],
						power_setting[i].config_val)) {
						soc_info->rgltr_min_volt[j] =
						soc_info->rgltr_max_volt[j] =
						power_setting[i].config_val;
					}
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

static void aon_update_mclk_down_index(struct aon_sensor_power_ctrl_t *ctrl)
{
	s32 index = 0;
	struct aon_sensor_power_setting *pd = NULL;

	if (!ctrl) {
		pr_err("%s power ctrl is null", __func__);
		return;
	}

	for (index = 0; index < ctrl->power_down_setting_size; index++) {
		pr_debug("%s power_down_index %d", __func__, index);
		pd = &ctrl->power_down_setting[index];
		if (!pd) {
			pr_err("%s Invalid power down settings for index %d", __func__, index);
			return;
		}

		pr_debug("%s power down seq_type %d", __func__, pd->seq_type);
		if (pd->seq_type == SENSOR_MCLK) {
			mclk_down_index = index;
			pr_info("%s mclk_down_index: %d", __func__, mclk_down_index);
			break;
		}
	}
}

s32 aon_disable_mclk(struct aon_sensor_power_ctrl_t *ctrl,
		struct aon_hw_soc_info *soc_info)
{
	s32 i = 0;
	s32 ret = 0;

	if (mclk_enabled != 1) {
		pr_info("%s MCLK already disabled", __func__);
		return 0;
	}

	if (!ctrl || !soc_info) {
		pr_err("%s power ctrl or soc_info is null", __func__);
		return -1;
	}

	if (mclk_down_index < 0) {
		aon_update_mclk_down_index(ctrl);
		if (mclk_down_index < 0) {
			pr_err("%s mclk_down_index = %d", __func__, mclk_down_index);
			return -1;
		}
	}

	for (i = soc_info->num_clk - 1; i >= 0; i--) {
		aon_soc_util_clk_disable(soc_info->clk[i],
			soc_info->clk_name[i]);
	}
	ret = aon_config_mclk_reg(ctrl, soc_info, mclk_down_index);
	if (ret < 0) {
		pr_err("%s config clk reg failed rc: %d", __func__, ret);
	} else
		mclk_enabled = 0;

	return ret;
}

static void aon_update_mclk_up_index(struct aon_sensor_power_ctrl_t *ctrl,
					struct aon_hw_soc_info *soc_info)
{
	s32 index = 0;
	struct aon_sensor_power_setting *power_setting = NULL;

	if (!ctrl || !soc_info) {
		pr_err("%s power ctrl or soc_info is null", __func__);
		return;
	}

	for (index = 0; index < ctrl->power_setting_size; index++) {
		pr_info("%s index: %d", __func__, index);
		power_setting = &ctrl->power_setting[index];
		if (!power_setting) {
			pr_err("%s Invalid power up settings for index %d", __func__, index);
			return;
		}

		pr_debug("%s seq_type %d", __func__, power_setting->seq_type);
		if (power_setting->seq_type == SENSOR_MCLK) {
			if (power_setting->seq_val >= soc_info->num_clk) {
				pr_err("%s clk index %d >= max %u", __func__,
						power_setting->seq_val, soc_info->num_clk);
				break;
			}
			mclk_up_index = index;
			break;
		}
	}
}

s32 aon_enable_mclk(struct aon_sensor_power_ctrl_t *ctrl,
		struct aon_hw_soc_info *soc_info)
{
	s32 j = 0;
	s32 num_vreg = 0;
	s32 rc = 0;
	struct aon_sensor_power_setting *power_setting = NULL;

	if (mclk_enabled == 1) {
		pr_info("%s MCLK already enabled", __func__);
		return 0;
	}

	if (!ctrl || !soc_info) {
		pr_err("%s power ctrl or soc_info is null", __func__);
		return -1;
	}

	if (mclk_up_index < 0 || mclk_up_index >= ctrl->power_setting_size) {
		aon_update_mclk_up_index(ctrl, soc_info);
		if (mclk_up_index < 0 || mclk_up_index >= ctrl->power_setting_size) {
			pr_err("%s mclk_up_index = %d", __func__, mclk_up_index);
			return -1;
		}
	}

	power_setting = &ctrl->power_setting[mclk_up_index];
	if (!power_setting) {
		pr_err("%s Invalid power up settings for index %d",
			__func__, mclk_up_index);
		return -EINVAL;
	}

	num_vreg = soc_info->num_rgltr;

	for (j = 0; j < num_vreg; j++) {
		if (!strcmp(soc_info->rgltr_name[j], "cam_clk")) {
			pr_info("%s Enable cam_clk: %d", __func__, j);

			soc_info->rgltr[j] =
				regulator_get(soc_info->dev, soc_info->rgltr_name[j]);

			if (IS_ERR_OR_NULL(soc_info->rgltr[j])) {
				rc = PTR_ERR(soc_info->rgltr[j]);
				rc = rc ? rc : -EINVAL;
				pr_err("%s vreg %s %d", __func__,
					soc_info->rgltr_name[j], rc);
				soc_info->rgltr[j] = NULL;
				return rc;
			}

			rc = aon_soc_util_regulator_enable(
				soc_info->rgltr[j],
				soc_info->rgltr_name[j],
				soc_info->rgltr_min_volt[j],
				soc_info->rgltr_max_volt[j],
				soc_info->rgltr_op_mode[j],
				soc_info->rgltr_delay[j]);
			if (rc) {
				pr_err("%s Reg enable failed", __func__);
				regulator_put(soc_info->rgltr[j]);
				soc_info->rgltr[j] = NULL;
				return rc;
			}
			power_setting->data[0] = soc_info->rgltr[j];
			break;
		}
	}
	if (power_setting->config_val)
		soc_info->clk_rate[0][power_setting->seq_val] =
			power_setting->config_val;

	for (j = 0; j < soc_info->num_clk; j++) {
		rc = aon_soc_util_clk_enable(soc_info->clk[j],
			soc_info->clk_name[j],
			soc_info->clk_rate[0][j]);
		if (rc)
			break;
	}
	if (power_setting->delay > 20)
		msleep(power_setting->delay);
	else if (power_setting->delay)
		usleep_range(power_setting->delay * 1000,
			(power_setting->delay * 1000) + 1000);

	if (rc < 0) {
		pr_err("%s clk enable failed", __func__);
		goto disable_mclk;
	}
	mclk_enabled = 1;
	return rc;

disable_mclk:
	rc = aon_config_mclk_reg(ctrl, soc_info, mclk_down_index);
	if (rc < 0) {
		pr_err("%s config clk reg failed rc: %d", __func__, rc);
	}

	return rc;
}

static int aon_sensor_util_power_up(struct aon_sensor_power_ctrl_t *ctrl,
	struct aon_hw_soc_info *soc_info)
{
	s32 rc = 0, index = 0, no_gpio = 0, ret = 0, num_vreg, j = 0;
	s32 mclk_reg_idx = -1;
	s32 vreg_idx = -1;
	struct aon_sensor_power_setting *power_setting = NULL;
	struct aon_camera_gpio_num_info *gpio_num_info = NULL;

	pr_debug("%s begin", __func__);
	if (!ctrl || !soc_info) {
		pr_err("%s Invalid handle, ctrl %p soc_info %p",
			__func__, ctrl, soc_info);
		return -EINVAL;
	}

	gpio_num_info = ctrl->gpio_num_info;
	num_vreg = soc_info->num_rgltr;

	if ((num_vreg <= 0) || (num_vreg > CAM_SOC_MAX_REGULATOR)) {
		pr_err("%s failed: num_vreg %d", __func__, num_vreg);
		return -EINVAL;
	}

	if (soc_info->use_shared_clk)
		pr_info("%s use shared clk", __func__);

	ret = aon_pinctrl_init(&(ctrl->pinctrl_info), ctrl->dev);
	if (ret < 0) {
		/* Some sensor subdev no pinctrl. */
		pr_info("%s Initialization of pinctrl failed", __func__);
		ctrl->cam_pinctrl_status = 0;
	} else {
		ctrl->cam_pinctrl_status = 1;
	}

	if (aon_res_mgr_shared_pinctrl_init()) {
		pr_err("%s Failed to init shared pinctrl",  __func__);
		return -EINVAL;
	}

	rc = aon_sensor_util_request_gpio_table(soc_info, 1);
	if (rc < 0)
		no_gpio = rc;

	if (ctrl->cam_pinctrl_status) {
		ret = pinctrl_select_state(
			ctrl->pinctrl_info.pinctrl,
			ctrl->pinctrl_info.gpio_state_active);
		if (ret)
			pr_err("%s cannot set pin to active state", __func__);
	}

	pr_info("%s power setting size: %d", __func__, ctrl->power_setting_size);

	for (index = 0; index < ctrl->power_setting_size; index++) {
		pr_info("%s index: %d", __func__, index);
		power_setting = &ctrl->power_setting[index];
		if (!power_setting) {
			pr_err("%s Invalid power up settings for index %d", __func__,
				index);
			return -EINVAL;
		}

		pr_info("%s seq_type %d", __func__, power_setting->seq_type);

		switch (power_setting->seq_type) {
		case SENSOR_MCLK:
			if (power_setting->seq_val >= soc_info->num_clk) {
				pr_err("%s clk index %d >= max %u", __func__,
					power_setting->seq_val, soc_info->num_clk);
				goto power_up_failed;
			}
			mclk_up_index = index;
			for (j = 0; j < num_vreg; j++) {
				if (!strcmp(soc_info->rgltr_name[j], "cam_clk")) {
					pr_info("%s Enable cam_clk: %d", __func__, j);

					soc_info->rgltr[j] =
						regulator_get(soc_info->dev, soc_info->rgltr_name[j]);

					if (IS_ERR_OR_NULL(soc_info->rgltr[j])) {
						rc = PTR_ERR(soc_info->rgltr[j]);
						rc = rc ? rc : -EINVAL;
						pr_err("%s vreg %s %d", __func__,
							soc_info->rgltr_name[j], rc);
						soc_info->rgltr[j] = NULL;
						goto power_up_failed;
					}

					rc = aon_soc_util_regulator_enable(
						soc_info->rgltr[j],
						soc_info->rgltr_name[j],
						soc_info->rgltr_min_volt[j],
						soc_info->rgltr_max_volt[j],
						soc_info->rgltr_op_mode[j],
						soc_info->rgltr_delay[j]);
					if (rc) {
						pr_err("%s Reg enable failed", __func__);
						regulator_put(soc_info->rgltr[j]);
						soc_info->rgltr[j] = NULL;
						goto power_up_failed;
					}
					power_setting->data[0] = soc_info->rgltr[j];
					mclk_reg_idx = j;
					break;
				}
			}
			if (power_setting->config_val)
				soc_info->clk_rate[0][power_setting->seq_val] =
					power_setting->config_val;

			for (j = 0; j < soc_info->num_clk; j++) {
				rc = aon_soc_util_clk_enable(soc_info->clk[j],
					soc_info->clk_name[j],
					soc_info->clk_rate[0][j]);
				if (rc)
					break;
			}

			if (rc < 0) {
				pr_err("%s clk enable failed", __func__);
				if (mclk_reg_idx >= 0 && mclk_reg_idx < num_vreg) {
					ret = aon_soc_util_regulator_disable(
						soc_info->rgltr[mclk_reg_idx],
						soc_info->rgltr_name[mclk_reg_idx],
						soc_info->rgltr_min_volt[mclk_reg_idx],
						soc_info->rgltr_max_volt[mclk_reg_idx],
						soc_info->rgltr_op_mode[mclk_reg_idx],
						soc_info->rgltr_delay[mclk_reg_idx]);
					regulator_put(soc_info->rgltr[mclk_reg_idx]);
					soc_info->rgltr[mclk_reg_idx] = NULL;
				}
				goto power_up_failed;
			}
			mclk_enabled = 1;

			break;
		case SENSOR_RESET:
		case SENSOR_STANDBY:
		case SENSOR_CUSTOM_GPIO1:
		case SENSOR_CUSTOM_GPIO2:
			if (no_gpio) {
				pr_err("%s request gpio failed", __func__);
				return no_gpio;
			}
			if (!gpio_num_info) {
				pr_err("%s Invalid gpio_num_info", __func__);
				goto power_up_failed;
			}
			pr_info("%s gpio set val %d", __func__,
				gpio_num_info->gpio_num
				[power_setting->seq_type]);

			aon_cam_sensor_handle_reg_gpio(
				power_setting->seq_type,
				gpio_num_info,
				(int) power_setting->config_val);
			break;
		case SENSOR_VANA:
		case SENSOR_VDIG:
		case SENSOR_VIO:
		case SENSOR_VAF:
		case SENSOR_VAF_PWDM:
		case SENSOR_CUSTOM_REG1:
		case SENSOR_CUSTOM_REG2:
			if (power_setting->seq_val == INVALID_VREG) {
				pr_err("%s invalid power setting seq_val", __func__);
				break;
			}

			if (power_setting->seq_val >= CAM_VREG_MAX) {
				pr_err("%s vreg index %d >= max %d", __func__,
					power_setting->seq_val,
					CAM_VREG_MAX);
				goto power_up_failed;
			}
			if (power_setting->seq_val < num_vreg) {
				pr_info("%s Enable Regulator", __func__);
				vreg_idx = power_setting->seq_val;

				soc_info->rgltr[vreg_idx] =
					regulator_get(soc_info->dev,
						soc_info->rgltr_name[vreg_idx]);
				if (IS_ERR_OR_NULL(
					soc_info->rgltr[vreg_idx])) {
					rc = PTR_ERR(soc_info->rgltr[vreg_idx]);
					rc = rc ? rc : -EINVAL;

					pr_err("%s %s get failed %d", __func__,
						soc_info->rgltr_name[vreg_idx],
						rc);

					soc_info->rgltr[vreg_idx] = NULL;
					goto power_up_failed;
				}

				rc = aon_soc_util_regulator_enable(
					soc_info->rgltr[vreg_idx],
					soc_info->rgltr_name[vreg_idx],
					soc_info->rgltr_min_volt[vreg_idx],
					soc_info->rgltr_max_volt[vreg_idx],
					soc_info->rgltr_op_mode[vreg_idx],
					soc_info->rgltr_delay[vreg_idx]);
				if (rc) {
					pr_err("%s Reg Enable failed for %s, rc=%d", __func__,
						soc_info->rgltr_name[vreg_idx], rc);
					regulator_put(soc_info->rgltr[vreg_idx]);
					soc_info->rgltr[vreg_idx] = NULL;
					goto power_up_failed;
				}
				power_setting->data[0] =
						soc_info->rgltr[vreg_idx];
			} else {
				pr_err("%s usr_idx:%d dts_idx:%d", __func__,
					power_setting->seq_val, num_vreg);
			}

			aon_cam_sensor_handle_reg_gpio(
				power_setting->seq_type,
				gpio_num_info, 1);
			break;
		default:
			pr_err("%s error power seq type %d", __func__,
				power_setting->seq_type);
			break;
		}
		if (power_setting->delay > 20)
			msleep(power_setting->delay);
		else if (power_setting->delay)
			usleep_range(power_setting->delay * 1000,
				(power_setting->delay * 1000) + 1000);
	}
	pr_debug("%s end", __func__);
	return 0;
power_up_failed:
	pr_err("%s failed", __func__);
	for (index--; index >= 0; index--) {
		pr_info("%s index %d", __func__,  index);
		power_setting = &ctrl->power_setting[index];
		pr_info("%s type %d", __func__,
			power_setting->seq_type);
		switch (power_setting->seq_type) {
		case SENSOR_MCLK:
			ret = aon_disable_mclk(ctrl, soc_info);
			pr_info("%s aon_disable_mclk ret = %d", __func__, ret);
			break;
		case SENSOR_RESET:
		case SENSOR_STANDBY:
		case SENSOR_CUSTOM_GPIO1:
		case SENSOR_CUSTOM_GPIO2:
			if (!gpio_num_info)
				continue;
			if (!gpio_num_info->valid[power_setting->seq_type])
				continue;
			aon_res_mgr_gpio_set_value(
				gpio_num_info->gpio_num[power_setting->seq_type],
				GPIOF_OUT_INIT_LOW);
			break;
		case SENSOR_VANA:
		case SENSOR_VDIG:
		case SENSOR_VIO:
		case SENSOR_VAF:
		case SENSOR_VAF_PWDM:
		case SENSOR_CUSTOM_REG1:
		case SENSOR_CUSTOM_REG2:
			if (power_setting->seq_val < num_vreg) {
				pr_info("%s Disable Regulator", __func__);
				vreg_idx = power_setting->seq_val;

				rc = aon_soc_util_regulator_disable(
					soc_info->rgltr[vreg_idx],
					soc_info->rgltr_name[vreg_idx],
					soc_info->rgltr_min_volt[vreg_idx],
					soc_info->rgltr_max_volt[vreg_idx],
					soc_info->rgltr_op_mode[vreg_idx],
					soc_info->rgltr_delay[vreg_idx]);

				if (rc) {
					pr_err("%s Fail to disalbe reg: %s", __func__,
					soc_info->rgltr_name[vreg_idx]);
					soc_info->rgltr[vreg_idx] = NULL;
					aon_cam_sensor_handle_reg_gpio(
						power_setting->seq_type,
						gpio_num_info,
						GPIOF_OUT_INIT_LOW);
					continue;
				}
				power_setting->data[0] =
						soc_info->rgltr[vreg_idx];

				regulator_put(soc_info->rgltr[vreg_idx]);
				soc_info->rgltr[vreg_idx] = NULL;
			} else {
				pr_err("%s seq_val:%d > num_vreg: %d", __func__,
					power_setting->seq_val, num_vreg);
			}

			aon_cam_sensor_handle_reg_gpio(power_setting->seq_type,
				gpio_num_info, GPIOF_OUT_INIT_LOW);

			break;
		default:
			pr_err("%s error power seq type %d", __func__,
				power_setting->seq_type);
			break;
		}
		if (power_setting->delay > 20) {
			msleep(power_setting->delay);
		} else if (power_setting->delay) {
			usleep_range(power_setting->delay * 1000,
				(power_setting->delay * 1000) + 1000);
		}
	}

	if (ctrl->cam_pinctrl_status) {
		ret = pinctrl_select_state(
			ctrl->pinctrl_info.pinctrl,
			ctrl->pinctrl_info.gpio_state_suspend);
		if (ret)
			pr_err("%s cannot set pin to suspend state", __func__);
		devm_pinctrl_put(ctrl->pinctrl_info.pinctrl);
	}

	ctrl->cam_pinctrl_status = 0;

	aon_sensor_util_request_gpio_table(soc_info, 0);

	pr_debug("%s end", __func__);
	return rc;
}

static s32 aon_sensor_util_power_down(struct aon_sensor_power_ctrl_t *ctrl,
	struct aon_hw_soc_info *soc_info)
{
	s32 index = 0, ret = 0, num_vreg = 0;
	struct aon_sensor_power_setting *pd = NULL;
	struct aon_sensor_power_setting *ps = NULL;
	struct aon_camera_gpio_num_info *gpio_num_info = NULL;

	pr_debug("%s begin", __func__);
	if (!ctrl || !soc_info) {
		pr_err("%s failed, ctrl %p, soc_info %p", __func__, ctrl, soc_info);
		return -EINVAL;
	}

	gpio_num_info = ctrl->gpio_num_info;
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
			pr_err("%s Invalid power down settings for index %d", __func__,
				index);
			return -EINVAL;
		}

		ps = NULL;
		pr_info("%s power down seq_type %d", __func__, pd->seq_type);
		switch (pd->seq_type) {
		case SENSOR_MCLK:
			mclk_down_index = index;
			ret = aon_disable_mclk(ctrl, soc_info);
			pr_info("%s aon_disable_mclk ret = %d", __func__, ret);
			break;
		case SENSOR_RESET:
		case SENSOR_STANDBY:
		case SENSOR_CUSTOM_GPIO1:
		case SENSOR_CUSTOM_GPIO2:
			if (!gpio_num_info->valid[pd->seq_type])
				continue;

			aon_res_mgr_gpio_set_value(gpio_num_info->gpio_num[pd->seq_type],
				(int)pd->config_val);

			break;
		case SENSOR_VANA:
		case SENSOR_VDIG:
		case SENSOR_VIO:
		case SENSOR_VAF:
		case SENSOR_VAF_PWDM:
		case SENSOR_CUSTOM_REG1:
		case SENSOR_CUSTOM_REG2:
			if (pd->seq_val == INVALID_VREG)
				break;

			ps = aon_get_power_settings(ctrl, pd->seq_type, pd->seq_val);
			if (ps) {
				if (pd->seq_val < num_vreg) {
					pr_err("%s Disable Regulator", __func__);
					ret = aon_soc_util_regulator_disable(
						soc_info->rgltr[ps->seq_val],
						soc_info->rgltr_name[ps->seq_val],
						soc_info->rgltr_min_volt[ps->seq_val],
						soc_info->rgltr_max_volt[ps->seq_val],
						soc_info->rgltr_op_mode[ps->seq_val],
						soc_info->rgltr_delay[ps->seq_val]);
					if (ret) {
						pr_err("%s Reg: %s disable failed", __func__,
							soc_info->rgltr_name[ps->seq_val]);
						soc_info->rgltr[ps->seq_val] = NULL;
						aon_sensor_handle_reg_gpio(
							pd->seq_type,
							gpio_num_info,
							GPIOF_OUT_INIT_LOW);
						continue;
					}
					ps->data[0] = soc_info->rgltr[ps->seq_val];
					regulator_put(soc_info->rgltr[ps->seq_val]);
					soc_info->rgltr[ps->seq_val] = NULL;
				} else {
					pr_err("%s seq_val:%d > num_vreg: %d", __func__,
						pd->seq_val, num_vreg);
				}
			} else
				pr_err("%s error in power up/down seq", __func__);

			ret = aon_sensor_handle_reg_gpio(pd->seq_type,
				gpio_num_info, GPIOF_OUT_INIT_LOW);

			if (ret < 0)
				pr_err("%s Error disabling VREG GPIO", __func__);
			break;
		default:
			pr_err("%s error power seq type %d", __func__, pd->seq_type);
			break;
		}
		if (pd->delay > 20)
			msleep(pd->delay);
		else if (pd->delay)
			usleep_range(pd->delay * 1000, (pd->delay * 1000) + 1000);
	}

	if (ctrl->cam_pinctrl_status) {
		ret = pinctrl_select_state(
				ctrl->pinctrl_info.pinctrl,
				ctrl->pinctrl_info.gpio_state_suspend);
		if (ret)
			pr_err("%s cannot set pin to suspend state", __func__);

		devm_pinctrl_put(ctrl->pinctrl_info.pinctrl);
	}

	ctrl->cam_pinctrl_status = 0;

	aon_sensor_util_request_gpio_table(soc_info, 0);

	pr_debug("%s end", __func__);
	return 0;
}

static s32 aon_sensor_bob_pwm_mode_switch(struct aon_hw_soc_info *soc_info,
	s32 bob_reg_idx, bool flag)
{
	s32 rc = 0;
	u32 op_current = 0;

	if (!soc_info)
		return -EINVAL;

	op_current = (flag == true) ? soc_info->rgltr_op_mode[bob_reg_idx] : 0;

	if (soc_info->rgltr[bob_reg_idx] != NULL) {
		rc = regulator_set_load(soc_info->rgltr[bob_reg_idx],
			op_current);
		if (rc)
			pr_err("%s BoB PWM SetLoad failed rc: %d", __func__, rc);
	}

	return rc;
}

s32 aon_sensor_power_down(struct aon_sensor_ctrl_t *s_ctrl)
{
	struct aon_sensor_power_ctrl_t *power_info = NULL;
	struct aon_hw_soc_info *soc_info = NULL;
	s32 rc = 0;

	if (!s_ctrl || !(s_ctrl->sensordata)) {
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
		pr_err("%s power down the core is failed:%d", __func__, rc);
		return rc;
	}

	if (s_ctrl->bob_pwm_switch) {
		pr_info("%s before aon_sensor_bob_pwm_mode_switch", __func__);
		rc = aon_sensor_bob_pwm_mode_switch(soc_info,
			s_ctrl->bob_reg_index, false);
		if (rc) {
			pr_err("%s BoB PWM setup failed rc: %d", __func__, rc);
			rc = 0;
		}
	}

	rc = aon_cci_release(s_ctrl);
	pr_info("%s aon_cci_release rc = %d", __func__, rc);

	return rc;
}

s32 aon_sensor_power_up(struct aon_sensor_ctrl_t *s_ctrl)
{
	s32 rc = 0;
	struct aon_sensor_power_ctrl_t *power_info = NULL;
	struct aon_camera_slave_info *slave_info = NULL;
	struct aon_hw_soc_info *soc_info = NULL;

	if (!s_ctrl) {
		pr_err("%s s_ctrl is null: %p", __func__, s_ctrl);
		return -EINVAL;
	}

	soc_info = &s_ctrl->soc_info;
	if (!s_ctrl->sensordata) {
		pr_err("%s sensordata is null: %p", __func__, s_ctrl->sensordata);
		return -EINVAL;
	}
	power_info = &s_ctrl->sensordata->power_info;
	slave_info = &(s_ctrl->sensordata->slave_info);

	if (!power_info || !slave_info) {
		pr_err("%s failed: %p %p", __func__, power_info, slave_info);
		return -EINVAL;
	}

	if (s_ctrl->bob_pwm_switch) {
		pr_info("%s before aon_sensor_bob_pwm_mode_switch", __func__);
		rc = aon_sensor_bob_pwm_mode_switch(soc_info,
			s_ctrl->bob_reg_index, true);
		if (rc) {
			pr_err("%s BoB PWM setup failed rc: %d", __func__, rc);
			rc = 0;
		}
	}

	rc = aon_sensor_util_power_up(power_info, soc_info);
	if (rc < 0) {
		pr_err("%s power up the core is failed:%d", __func__, rc);
		return rc;
	}

	return rc;
}

s32 aon_sensor_update_i2c_info(struct aon_cmd_i2c_info *i2c_info,
	struct aon_sensor_ctrl_t *s_ctrl)
{
	s32 rc = 0;
	struct cam_sensor_cci_client *cci_client = NULL;

	if (!i2c_info || !s_ctrl)
		return -EINVAL;

	cci_client = s_ctrl->cci_client;
	if (!cci_client) {
		pr_err("failed: cci_client %p", cci_client);
		return -EINVAL;
	}
	cci_client->cci_i2c_master = s_ctrl->cci_i2c_master;
	cci_client->sid = i2c_info->slave_addr >> 1;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->i2c_freq_mode = i2c_info->i2c_freq_mode;
	pr_info(" Master: %d sid: %d freq_mode: %d",
		cci_client->cci_i2c_master, i2c_info->slave_addr,
		i2c_info->i2c_freq_mode);

	s_ctrl->sensordata->slave_info.sensor_slave_addr = i2c_info->slave_addr;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	s_ctrl->sensordata->slave_info.sub_sensor_slave_addr = i2c_info->sub_slave_addr;
#endif
	return rc;
}
