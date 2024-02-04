/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
** File : oplus_display_temp_compensation.c
** Description : oplus_display_temp_compensation implement
** Version : 1.0
** Date : 2022/11/20
** Author : Display
***************************************************************/

#include "oplus_display_temp_compensation.h"
#include "oplus_display_panel_common.h"
#include "sde_trace.h"
#include <linux/thermal.h>
#include <linux/iio/consumer.h>

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
#include "oplus_adfr.h"
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "oplus_onscreenfingerprint.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

/* -------------------- macro -------------------- */
/* config bit setting */
#define OPLUS_TEMP_COMPENSAITON_CONFIG_ENABLE				(BIT(0))
#define OPLUS_TEMP_COMPENSAITON_CONFIG_DRY_RUN				(BIT(1))
#define OPLUS_TEMP_COMPENSAITON_CONFIG_DATA_UPDATE			(BIT(2))
/* get config value */
#define OPLUS_TEMP_COMPENSATION_GET_ENABLE_CONFIG(config)	((config) & OPLUS_TEMP_COMPENSAITON_CONFIG_ENABLE)
#define OPLUS_TEMP_COMPENSATION_GET_DRY_RUN_CONFIG(config)	((config) & OPLUS_TEMP_COMPENSAITON_CONFIG_DRY_RUN)
#define OPLUS_TEMP_COMPENSATION_GET_DATA_UPDATE_CONFIG(config)	((config) & OPLUS_TEMP_COMPENSAITON_CONFIG_DATA_UPDATE)

#define OPLUS_TEMP_COMPENSAITON_MAX_INIT_RETRY_TIME			100

/* -------------------- parameters -------------------- */
struct LCM_setting_table {
	size_t count;
	unsigned char *para_list;
};

/* log level config */
unsigned int oplus_temp_compensation_log_level = OPLUS_TEMP_COMPENSATION_LOG_LEVEL_DEBUG;
EXPORT_SYMBOL(oplus_temp_compensation_log_level);
/* temp compensation global structure */
static struct oplus_temp_compensation_params g_oplus_temp_compensation_params = {0};

/* -------------------- extern -------------------- */
/* extern params */

/* extern functions */

/* -------------------- function implementation -------------------- */
static struct oplus_temp_compensation_params *oplus_temp_compensation_get_params(void)
{
	return &g_oplus_temp_compensation_params;
}

bool oplus_temp_compensation_is_supported(void)
{
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	if (IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return false;
	}

	/* global config */
	return (bool)(OPLUS_TEMP_COMPENSATION_GET_ENABLE_CONFIG(p_oplus_temp_compensation_params->config));
}

static bool oplus_temp_compensation_dry_run_is_enabled(void)
{
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	if (IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return false;
	}

	if (!oplus_temp_compensation_is_supported()) {
		TEMP_COMPENSATION_DEBUG("temp compensation is not support, dry run is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_TEMP_COMPENSATION_GET_DRY_RUN_CONFIG(p_oplus_temp_compensation_params->config));
}

static bool oplus_temp_compensation_data_update_is_enabled(void)
{
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	if (IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return false;
	}

	if (!oplus_temp_compensation_is_supported()) {
		TEMP_COMPENSATION_DEBUG("temp compensation is not support, data updating is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_TEMP_COMPENSATION_GET_DATA_UPDATE_CONFIG(p_oplus_temp_compensation_params->config));
}

int oplus_temp_compensation_init(void *dsi_panel)
{
	static bool registered = false;
	const char *data1 = NULL;
	unsigned char *data2 = NULL;
	int rc = 0;
	int length = 0;
	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int k = 0;
	unsigned int value = 0;
	static unsigned int failure_count = 0;
	struct dsi_panel *panel = dsi_panel;
	struct dsi_parser_utils *utils = NULL;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (registered) {
		TEMP_COMPENSATION_DEBUG("temp compensation has been already initialized\n");
		return rc;
	}

	if (IS_ERR_OR_NULL(panel) || IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid panel or p_oplus_temp_compensation_params params\n");
		return -EINVAL;
	}

	if (!strcmp(panel->type, "secondary")) {
		TEMP_COMPENSATION_INFO("ignore secondary panel function calls\n");
		return rc;
	}

	utils = &panel->utils;
	if (!utils) {
		TEMP_COMPENSATION_ERR("Invalid utils params\n");
		return -EINVAL;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_init");

	/* oplus,temp-compensation-config */
	rc = utils->read_u32(utils->data, "oplus,temp-compensation-config", &value);
	if (rc) {
		TEMP_COMPENSATION_INFO("failed to read oplus,temp-compensation-config, rc=%d\n", rc);
	} else {
		p_oplus_temp_compensation_params->config = value;
	}
	TEMP_COMPENSATION_INFO("oplus_temp_compensation_config:0x%x\n", p_oplus_temp_compensation_params->config);
	OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_config", p_oplus_temp_compensation_params->config);

	if (oplus_temp_compensation_is_supported()) {
		/* oplus,temp-compensation-reg-repeat */
		rc = utils->read_u32(utils->data, "oplus,temp-compensation-reg-repeat", &value);
		if (rc) {
			TEMP_COMPENSATION_INFO("failed to read oplus,temp-compensation-reg-repeat, rc=%d, set reg_repeat to default value 4\n", rc);
			p_oplus_temp_compensation_params->reg_repeat = 4;
		} else {
			p_oplus_temp_compensation_params->reg_repeat = value;
		}
		TEMP_COMPENSATION_INFO("oplus_temp_compensation_reg_repeat:%u\n", p_oplus_temp_compensation_params->reg_repeat);
		OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_reg_repeat", p_oplus_temp_compensation_params->reg_repeat);

		/* oplus,temp-compensation-dbv-group */
		length = utils->count_u32_elems(utils->data, "oplus,temp-compensation-dbv-group");
		if (length < 0) {
			TEMP_COMPENSATION_ERR("failed to get the count of oplus,temp-compensation-dbv-group\n");
			rc = -EINVAL;
			goto error;
		}
		p_oplus_temp_compensation_params->dbv_group = kzalloc(length * sizeof(unsigned int), GFP_KERNEL);
		if (!p_oplus_temp_compensation_params->dbv_group) {
			TEMP_COMPENSATION_ERR("failed to kzalloc dbv_group\n");
			rc = -ENOMEM;
			goto error;
		}
		rc = utils->read_u32_array(utils->data, "oplus,temp-compensation-dbv-group", p_oplus_temp_compensation_params->dbv_group, length);
		if (rc) {
			TEMP_COMPENSATION_ERR("failed to read oplus,temp-compensation-dbv-group\n");
			rc = -EINVAL;
			goto error_free_dbv_group;
		}
		TEMP_COMPENSATION_INFO("property:oplus,temp-compensation-dbv-group,length:%u\n", length);
		for (i = 0; i < length; i++) {
			TEMP_COMPENSATION_INFO("dbv_group[%u]=%u\n", i, p_oplus_temp_compensation_params->dbv_group[i]);
		}
		p_oplus_temp_compensation_params->dbv_group_count = length + 1;
		TEMP_COMPENSATION_INFO("oplus_temp_compensation_dbv_group_count:%u\n", p_oplus_temp_compensation_params->dbv_group_count);
		OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_dbv_group_count", p_oplus_temp_compensation_params->dbv_group_count);

		/* oplus,temp-compensation-temp-group */
		length = utils->count_u32_elems(utils->data, "oplus,temp-compensation-temp-group");
		if (length < 0) {
			TEMP_COMPENSATION_ERR("failed to get the count of oplus,temp-compensation-temp-group\n");
			rc = -EINVAL;
			goto error_free_dbv_group;
		}
		p_oplus_temp_compensation_params->temp_group = kzalloc(length * sizeof(int), GFP_KERNEL);
		if (!p_oplus_temp_compensation_params->temp_group) {
			TEMP_COMPENSATION_ERR("failed to kzalloc temp_group\n");
			rc = -ENOMEM;
			goto error_free_dbv_group;
		}
		rc = utils->read_u32_array(utils->data, "oplus,temp-compensation-temp-group", (unsigned int *)p_oplus_temp_compensation_params->temp_group, length);
		if (rc) {
			TEMP_COMPENSATION_ERR("failed to read oplus,temp-compensation-temp-group\n");
			rc = -EINVAL;
			goto error_free_temp_group;
		}
		TEMP_COMPENSATION_INFO("property:oplus,temp-compensation-temp-group,length:%u\n", length);
		for (i = 0; i < length; i++) {
			TEMP_COMPENSATION_INFO("temp_group[%u]=%d\n", i, p_oplus_temp_compensation_params->temp_group[i]);
		}
		p_oplus_temp_compensation_params->temp_group_count = length + 1;
		TEMP_COMPENSATION_INFO("oplus_temp_compensation_temp_group_count:%u\n", p_oplus_temp_compensation_params->temp_group_count);
		OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_temp_group_count", p_oplus_temp_compensation_params->temp_group_count);

		/* oplus,temp-compensation-data */
		data1 = utils->get_property(utils->data, "oplus,temp-compensation-data", &length);
		if (!data1) {
			TEMP_COMPENSATION_ERR("failed to get oplus,temp-compensation-data\n");
			rc = -ENOTSUPP;
			goto error_free_temp_group;
		}
		TEMP_COMPENSATION_INFO("property:oplus,temp-compensation-data,length:%u\n", length);
		if (length % (p_oplus_temp_compensation_params->dbv_group_count * p_oplus_temp_compensation_params->temp_group_count)) {
			TEMP_COMPENSATION_ERR("format error\n");
			rc = -EINVAL;
			goto error_free_temp_group;
		}
		p_oplus_temp_compensation_params->voltage_group_count =
			length / (p_oplus_temp_compensation_params->dbv_group_count * p_oplus_temp_compensation_params->temp_group_count);
		TEMP_COMPENSATION_INFO("oplus_temp_compensation_voltage_group_count:%u\n", p_oplus_temp_compensation_params->voltage_group_count);
		OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_voltage_group_count", p_oplus_temp_compensation_params->voltage_group_count);
		data2 = kzalloc(length * sizeof(unsigned char), GFP_KERNEL);
		if (!data2) {
			TEMP_COMPENSATION_ERR("failed to kzalloc data2\n");
			rc = -ENOMEM;
			goto error_free_temp_group;
		}
		memcpy(data2, data1, length * sizeof(unsigned char));
		p_oplus_temp_compensation_params->data = kzalloc(p_oplus_temp_compensation_params->dbv_group_count * sizeof(unsigned char**), GFP_KERNEL);
		if (!p_oplus_temp_compensation_params->data) {
			TEMP_COMPENSATION_ERR("failed to kzalloc data\n");
			rc = -ENOMEM;
			goto error_free_data2;
		}
		for (i = 0; i < p_oplus_temp_compensation_params->dbv_group_count; i++) {
			p_oplus_temp_compensation_params->data[i] = kzalloc(p_oplus_temp_compensation_params->temp_group_count * sizeof(unsigned char*), GFP_KERNEL);
			if (!p_oplus_temp_compensation_params->data[i]) {
				TEMP_COMPENSATION_ERR("failed to kzalloc data[%u]\n", i);
				rc = -ENOMEM;
				goto error_free_data;
			}
			for (j = 0; j < p_oplus_temp_compensation_params->temp_group_count; j++) {
				p_oplus_temp_compensation_params->data[i][j] =
					&data2[i * p_oplus_temp_compensation_params->temp_group_count * p_oplus_temp_compensation_params->voltage_group_count +
							j * p_oplus_temp_compensation_params->voltage_group_count + 0];
				for (k = 0; k < (p_oplus_temp_compensation_params->voltage_group_count); k++) {
					TEMP_COMPENSATION_DEBUG("data[%u][%u][%u]=0x%02X\n", i, j, k, p_oplus_temp_compensation_params->data[i][j][k]);
				}
			}
		}

		/* add workqueue to send temp compensation cmd in the first half frame */
		p_oplus_temp_compensation_params->first_half_frame_cmd_set_wq = create_singlethread_workqueue("first_half_frame_temp_compensation_cmd_set");
		INIT_WORK(&p_oplus_temp_compensation_params->first_half_frame_cmd_set_work, oplus_temp_compensation_first_half_frame_cmd_set_work_handler);

		registered = true;
		TEMP_COMPENSATION_INFO("init temp compensation successfully\n");
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_init");

	TEMP_COMPENSATION_DEBUG("end\n");

	return 0;

error_free_data:
	for (i = i - 1; i >= 0; i--) {
		kfree(p_oplus_temp_compensation_params->data[i]);
		p_oplus_temp_compensation_params->data[i] = NULL;
	}
	kfree(p_oplus_temp_compensation_params->data);
	p_oplus_temp_compensation_params->data = NULL;
error_free_data2:
	kfree(data2);
	data2 = NULL;
error_free_temp_group:
	kfree(p_oplus_temp_compensation_params->temp_group);
	p_oplus_temp_compensation_params->temp_group = NULL;
error_free_dbv_group:
	kfree(p_oplus_temp_compensation_params->dbv_group);
	p_oplus_temp_compensation_params->dbv_group = NULL;
error:
	p_oplus_temp_compensation_params->config = 0;
	TEMP_COMPENSATION_INFO("oplus_temp_compensation_config:0x%x\n", p_oplus_temp_compensation_params->config);
	OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_config", p_oplus_temp_compensation_params->config);

	/* set max retry time to OPLUS_TEMP_COMPENSAITON_MAX_INIT_RETRY_TIME */
	if (!registered) {
		failure_count++;
		TEMP_COMPENSATION_INFO("failure_count:%u\n", failure_count);
		if (failure_count == OPLUS_TEMP_COMPENSAITON_MAX_INIT_RETRY_TIME) {
			registered = true;
		}
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_init");

	TEMP_COMPENSATION_DEBUG("end\n");

	return rc;
}

int oplus_temp_compensation_register_ntc_channel(void *dsi_display)
{
	static bool registered = false;
	int rc = 0;
	unsigned int i = 0;
	static unsigned int failure_count = 0;
	struct dsi_display *display = dsi_display;
	struct device *dev = NULL;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (registered) {
		TEMP_COMPENSATION_DEBUG("ntc channel has been already registered\n");
		return rc;
	}

	if (IS_ERR_OR_NULL(display) || IS_ERR_OR_NULL(display->panel) || IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid display or p_oplus_temp_compensation_params params\n");
		return -EINVAL;
	}

	if (!strcmp(display->panel->type, "secondary")) {
		TEMP_COMPENSATION_INFO("ignore secondary panel function calls\n");
		return rc;
	}

	dev = &display->pdev->dev;
	if (IS_ERR_OR_NULL(dev)) {
		TEMP_COMPENSATION_ERR("Invalid dev params\n");
		return -EINVAL;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_register_ntc_channel");

	for (i = 0; i < 5; i++) {
		p_oplus_temp_compensation_params->ntc_temp_chan = iio_channel_get(dev, "disp0_con_therm_adc");
		rc = IS_ERR_OR_NULL(p_oplus_temp_compensation_params->ntc_temp_chan);
		if (rc) {
			TEMP_COMPENSATION_ERR("failed to get panel channel\n");
			p_oplus_temp_compensation_params->ntc_temp_chan = NULL;
		} else {
			p_oplus_temp_compensation_params->ntc_temp = 29;
			TEMP_COMPENSATION_INFO("oplus_temp_compensation_ntc_temp:%d\n", p_oplus_temp_compensation_params->ntc_temp);
			OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_ntc_temp", p_oplus_temp_compensation_params->ntc_temp);
			p_oplus_temp_compensation_params->shell_temp = 29;
			TEMP_COMPENSATION_INFO("oplus_temp_compensation_shell_temp:%d\n", p_oplus_temp_compensation_params->shell_temp);
			OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_shell_temp", p_oplus_temp_compensation_params->shell_temp);
			TEMP_COMPENSATION_INFO("register ntc channel successfully\n");
			registered = true;
			break;
		}
	}


	/* set max retry time to OPLUS_TEMP_COMPENSAITON_MAX_INIT_RETRY_TIME */
	if (!registered) {
		failure_count++;
		TEMP_COMPENSATION_INFO("failure_count:%u\n", failure_count);
		if (failure_count == OPLUS_TEMP_COMPENSAITON_MAX_INIT_RETRY_TIME) {
			registered = true;
		}
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_register_ntc_channel");

	TEMP_COMPENSATION_DEBUG("end\n");

	return rc;
}

int oplus_temp_compensation_get_ntc_temp(void)
{
	int rc = 0;
	int val_avg = 0;
	int val[3] = {0, 0, 0};
	unsigned int i = 0;
	unsigned int count = 0;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid p_oplus_temp_compensation_params params\n");
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(p_oplus_temp_compensation_params->ntc_temp_chan)) {
		TEMP_COMPENSATION_ERR("Invalid ntc_temp_chan params\n");
		p_oplus_temp_compensation_params->ntc_temp = 29;
		TEMP_COMPENSATION_INFO("oplus_temp_compensation_ntc_temp:%d\n", p_oplus_temp_compensation_params->ntc_temp);
		OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_ntc_temp", p_oplus_temp_compensation_params->ntc_temp);
		return -EINVAL;
	}

	if (p_oplus_temp_compensation_params->fake_ntc_temp) {
		TEMP_COMPENSATION_DEBUG("fake panel ntc temp is %d\n", p_oplus_temp_compensation_params->ntc_temp);
		return p_oplus_temp_compensation_params->ntc_temp;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_get_ntc_temp");

	do {
		for (i = 0; i < 3; i++) {
			rc = iio_read_channel_processed(p_oplus_temp_compensation_params->ntc_temp_chan, &val[i]);
			if (rc < 0) {
				TEMP_COMPENSATION_ERR("read ntc_temp_chan value failed, rc=%d\n", rc);
			} else {
				TEMP_COMPENSATION_DEBUG("ntc_temp:%d\n", val[i]);
			}
		}

		if (count) {
			TEMP_COMPENSATION_ERR("retry %u\n", count);
		}
		count++;
	} while (((abs(val[0] - val[1]) >= 2000) || (abs(val[0] - val[2]) >= 2000)
				|| (abs(val[1] - val[2]) >= 2000) || (rc < 0)) && (count < 5));

	if (count == 5) {
		TEMP_COMPENSATION_ERR("use last panel ntc temp %d\n", p_oplus_temp_compensation_params->ntc_temp);
		return p_oplus_temp_compensation_params->ntc_temp;
	} else {
		val_avg = (val[0] + val[1] + val[2]) / 3;
	}

	p_oplus_temp_compensation_params->ntc_temp = val_avg / 1000;
	TEMP_COMPENSATION_DEBUG("oplus_temp_compensation_ntc_temp:%d\n", p_oplus_temp_compensation_params->ntc_temp);
	OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_ntc_temp", p_oplus_temp_compensation_params->ntc_temp);

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_get_ntc_temp");

	TEMP_COMPENSATION_DEBUG("end\n");

	return p_oplus_temp_compensation_params->ntc_temp;
}

static int oplus_temp_compensation_get_shell_temp(void)
{
	int temp = -127000;
	int max_temp = -127000;
	unsigned int i = 0;
	const char *shell_tz[] = {"shell_front", "shell_frame", "shell_back"};
	struct thermal_zone_device *tz = NULL;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (p_oplus_temp_compensation_params->fake_shell_temp) {
		TEMP_COMPENSATION_DEBUG("fake shell temp is %d\n", p_oplus_temp_compensation_params->shell_temp);
		return p_oplus_temp_compensation_params->shell_temp;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_get_shell_temp");

	for (i = 0; i < ARRAY_SIZE(shell_tz); i++) {
		tz = thermal_zone_get_zone_by_name(shell_tz[i]);
		thermal_zone_get_temp(tz, &temp);
		if (max_temp < temp) {
			max_temp = temp;
		}
	}

	p_oplus_temp_compensation_params->shell_temp = max_temp / 1000;
	TEMP_COMPENSATION_DEBUG("oplus_temp_compensation_shell_temp:%d\n", p_oplus_temp_compensation_params->shell_temp);
	OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_shell_temp", p_oplus_temp_compensation_params->shell_temp);

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_get_shell_temp");

	TEMP_COMPENSATION_DEBUG("end\n");

	return p_oplus_temp_compensation_params->shell_temp;
}

int oplus_temp_compensation_data_update(void *dsi_display)
{
	static bool calibrated = false;
	int rc = 0;
	int delta1 = 0;
	int delta2 = 0;
	unsigned char rx_buf[7] = {0};
	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int k = 0;
	static unsigned int failure_count = 0;
	struct dsi_display *display = dsi_display;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (!oplus_temp_compensation_data_update_is_enabled()) {
		TEMP_COMPENSATION_DEBUG("data updating is not enabled\n");
		return 0;
	}

	if (calibrated || failure_count > 10) {
		TEMP_COMPENSATION_DEBUG("calibrated:%u,failure_count:%u, no need to update temp compensation data\n", calibrated, failure_count);
		return 0;
	}

	if (IS_ERR_OR_NULL(display) || IS_ERR_OR_NULL(display->panel) || IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (!strcmp(display->panel->type, "secondary")) {
		TEMP_COMPENSATION_INFO("ignore secondary panel function calls\n");
		return rc;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_data_update");

	rc = oplus_display_tx_cmd_set_lock(display, DSI_CMD_READ_TEMP_COMPENSATION_REG);
	if (rc) {
		TEMP_COMPENSATION_ERR("[%s] failed to send DSI_CMD_READ_TEMP_COMPENSATION_REG cmds, rc=%d\n", display->name, rc);
		OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_data_update");
		return rc;
	}
	rc = dsi_display_read_panel_reg(display, 0xE0, rx_buf, 7);
	if (rc) {
		TEMP_COMPENSATION_ERR("failed to read panel reg 0xE0, rc=%d\n", rc);
		OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_data_update");
		return rc;
	}

	TEMP_COMPENSATION_INFO("rx_buf[0]=%u,rx_buf[6]=%u\n", rx_buf[0], rx_buf[6]);
	if ((rx_buf[0] < 25) || (rx_buf[0] > 29) || (rx_buf[6] < 26) || (rx_buf[6] > 30)) {
		failure_count++;
		TEMP_COMPENSATION_ERR("invalid rx_buf,failure_count:%u\n", failure_count);
		OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_data_update");
		return -EINVAL;
	}

	delta1 = rx_buf[0] - 0x1B;
	delta2 = rx_buf[6] - 0x1C;
	TEMP_COMPENSATION_DEBUG("delta1=%d,delta2=%d\n", delta1, delta2);

	for (i = OPLUS_TEMP_COMPENSATION_1511_1604_DBV_INDEX; i <= OPLUS_TEMP_COMPENSATION_1212_1328_DBV_INDEX; i++) {
		for (j = OPLUS_TEMP_COMPENSATION_LESS_THAN_MINUS10_TEMP_INDEX; j <= OPLUS_TEMP_COMPENSATION_GREATER_THAN_50_TEMP_INDEX; j++) {
			for (k = 11; k <= 13; k++) {
				p_oplus_temp_compensation_params->data[i][j][k] += delta2;
				TEMP_COMPENSATION_DEBUG("data[%u][%u][%u]=0x%02X\n", i, j, k, p_oplus_temp_compensation_params->data[i][j][k]);
			}
			for (k = 14; k <= 16; k++) {
				p_oplus_temp_compensation_params->data[i][j][k] += delta1;
				TEMP_COMPENSATION_DEBUG("data[%u][%u][%u]=0x%02X\n", i, j, k, p_oplus_temp_compensation_params->data[i][j][k]);
			}
		}
	}

	i = OPLUS_TEMP_COMPENSATION_1604_3515_DBV_INDEX;
	for (j = OPLUS_TEMP_COMPENSATION_LESS_THAN_MINUS10_TEMP_INDEX; j <= OPLUS_TEMP_COMPENSATION_GREATER_THAN_50_TEMP_INDEX; j++) {
		for (k = 8; k <= 10; k++) {
			p_oplus_temp_compensation_params->data[i][j][k] += delta1;
			TEMP_COMPENSATION_DEBUG("data[%u][%u][%u]=0x%02X\n", i, j, k, p_oplus_temp_compensation_params->data[i][j][k]);
		}
	}

	calibrated = true;
	TEMP_COMPENSATION_INFO("update temp compensation data successfully\n");

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_data_update");

	TEMP_COMPENSATION_DEBUG("end\n");

	return 0;
}

static unsigned int oplus_temp_compensation_get_dbv_index(unsigned int bl_lvl)
{
	unsigned int i = 0;
	unsigned int dbv_index = 0;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		/* set default dbv index to 2 */
		return 2;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_get_dbv_index");

	for (i = 0; i < (p_oplus_temp_compensation_params->dbv_group_count - 1); i++) {
		if (bl_lvl >= p_oplus_temp_compensation_params->dbv_group[i]) {
			break;
		}
	}
	dbv_index = i;
	TEMP_COMPENSATION_DEBUG("dbv_index:%u\n", dbv_index);

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_get_dbv_index");

	TEMP_COMPENSATION_DEBUG("end\n");

	return dbv_index;
}

static unsigned int oplus_temp_compensation_get_temp_index(int temp)
{
	unsigned int i = 0;
	unsigned int temp_index = 0;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		/* set default temp index to 6 */
		return 6;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_get_temp_index");

	for (i = 0; i < (p_oplus_temp_compensation_params->temp_group_count - 1); i++) {
		if (temp < p_oplus_temp_compensation_params->temp_group[i]) {
			break;
		}
	}
	temp_index = i;
	TEMP_COMPENSATION_DEBUG("temp_index:%u\n", temp_index);

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_get_temp_index");

	TEMP_COMPENSATION_DEBUG("end\n");

	return temp_index;
}

int oplus_temp_compensation_cmd_set(void *dsi_panel, unsigned int setting_mode)
{
	int rc = 0;
	int ntc_temp = 0;
	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int lcm_cmd_count = 0;
	unsigned int refresh_rate = 120;
	unsigned int dbv_index = 0;
	unsigned int temp_index = 0;
	unsigned int bl_lvl = 0;
	static unsigned int last_dbv_index = 0;
	static unsigned int last_temp_index = 0;
	static unsigned int last_bl_lvl = 0;		/* Force sending temp compensation cmd when booting up */
	static unsigned int onepulse_enable_last = 0;
	unsigned int onepulse_enable_changed = 0;
	unsigned int onepulse_used = 0;
	struct dsi_panel *panel = dsi_panel;
	struct dsi_cmd_desc *cmds = NULL;
	struct LCM_setting_table temp_compensation_cmd[50] = {0};
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(panel) || IS_ERR_OR_NULL(&(panel->bl_config)) || IS_ERR_OR_NULL(panel->cur_mode)
			|| IS_ERR_OR_NULL(panel->cur_mode->priv_info) || IS_ERR_OR_NULL(&(panel->cur_mode->priv_info->cmd_sets[DSI_CMD_TEMPERATURE_COMPENSATION]))
				|| IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (!strcmp(panel->type, "secondary")) {
		TEMP_COMPENSATION_INFO("ignore secondary panel function calls\n");
		return rc;
	}

	if (!dsi_panel_initialized(panel)) {
		TEMP_COMPENSATION_ERR("should not set temp compensation cmd if panel is not initialized\n");
		return -EINVAL;
	}

	cmds = panel->cur_mode->priv_info->cmd_sets[DSI_CMD_TEMPERATURE_COMPENSATION].cmds;
	lcm_cmd_count = panel->cur_mode->priv_info->cmd_sets[DSI_CMD_TEMPERATURE_COMPENSATION].count;
	if (!lcm_cmd_count) {
		TEMP_COMPENSATION_ERR("Invalid DSI_CMD_TEMPERATURE_COMPENSATION cmd sets\n");
		return -EINVAL;
	}
	for(i = 0; i < lcm_cmd_count; i++) {
		temp_compensation_cmd[i].count = cmds[i].msg.tx_len;
		temp_compensation_cmd[i].para_list = (unsigned char *)cmds[i].msg.tx_buf;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_cmd_set");

	bl_lvl = panel->bl_config.bl_level;

	TEMP_COMPENSATION_DEBUG("setting_mode:%u\n", setting_mode);

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	if (oplus_ofp_is_supported()) {
		if ((setting_mode == OPLUS_TEMP_COMPENSATION_FOD_ON_SETTING)
				|| (oplus_ofp_get_hbm_state() && (setting_mode != OPLUS_TEMP_COMPENSATION_FOD_OFF_SETTING))) {
			bl_lvl = 3840;
		}
	}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

	refresh_rate = panel->cur_mode->timing.refresh_rate;
	TEMP_COMPENSATION_DEBUG("refresh_rate:%u\n", refresh_rate);

	dbv_index = oplus_temp_compensation_get_dbv_index(bl_lvl);

	if (IS_ERR_OR_NULL(p_oplus_temp_compensation_params->ntc_temp_chan)) {
		TEMP_COMPENSATION_DEBUG("panel ntc is not exist, use default ntc temp value\n");
		ntc_temp = 29;
	} else {
		ntc_temp = p_oplus_temp_compensation_params->ntc_temp;
	}

	temp_index = oplus_temp_compensation_get_temp_index(ntc_temp);

	onepulse_enable_changed = oplus_panel_pwm_onepulse_is_enabled(panel) != onepulse_enable_last;
	onepulse_enable_last = oplus_panel_pwm_onepulse_is_enabled(panel);
	onepulse_used = oplus_panel_pwm_onepulse_is_used(panel);
	TEMP_COMPENSATION_DEBUG("onepulse_enable:%u,changed:%u,used:%u\n", onepulse_enable_last, onepulse_enable_changed, onepulse_used);

	TEMP_COMPENSATION_DEBUG("last_bl_lvl:%u,bl_lvl:%u,last_dbv_index:%u,dbv_index:%u,shell_temp:%d,ntc_temp:%d,last_temp_index:%u,temp_index:%u\n",
			last_bl_lvl, bl_lvl, last_dbv_index, dbv_index, oplus_temp_compensation_get_shell_temp(), ntc_temp,
				last_temp_index, temp_index);

	if ((last_dbv_index != dbv_index) || (last_temp_index != temp_index) || (!last_bl_lvl && bl_lvl)
			|| (setting_mode == OPLUS_TEMP_COMPENSATION_ESD_SETTING) || (setting_mode == OPLUS_TEMP_COMPENSATION_FIRST_HALF_FRAME_SETTING)
			|| onepulse_enable_changed) {
		if (((refresh_rate == 60) && (setting_mode == OPLUS_TEMP_COMPENSATION_BACKLIGHT_SETTING)
			&& (bl_lvl != 0) && (bl_lvl != 1)) || oplus_temp_compensation_wait_for_vsync_set) {
			p_oplus_temp_compensation_params->need_to_set_in_first_half_frame = true;
			TEMP_COMPENSATION_DEBUG("oplus_temp_compensation_need_to_set_in_first_half_frame:%d\n", p_oplus_temp_compensation_params->need_to_set_in_first_half_frame);
			OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_need_to_set_in_first_half_frame",
												p_oplus_temp_compensation_params->need_to_set_in_first_half_frame);
		} else {
			for (i = 0; i < p_oplus_temp_compensation_params->reg_repeat; i++) {
				/*
				 * the left side is cmd register
				 * the right side is dtsi data, it is 3-dimen array selected by current brightness and temperature
				 * cmd register 2 4 6 is low freq 120HZ EM Duty, cmd register 12 14 16 is high freq 120HZ EM Duty
				 * dtsi data 0 1 2 is low  freq 120HZ EM Duty
				 * dtsi data 5 6 7 is high freq 120HZ EM Duty if not 1pulse, if 1pulse, it is REUSED as low 120HZ EM Duty of 1pulse
				 * cmd register len is 4, dtsi data will be repeated reg_repeat(3 or 4) times to set cmd register
				 * when 1pulse support
				 *  cmd register 12 14 16 is seted 0
				 *  if onepulse_used cmd register 2 4 6 is seted from dtsi data 5 6 7, else seted from dtsi data 0 1 2
				 * when 1pulse NOT support
				 *  cmd register 2 4 6 is seted from dtsi data 0 1 2
				 *  cmd register 12 14 16 is seted from dtsi data 5 6 7
				 */
				if (panel->oplus_priv.pwm_onepulse_support) {
					if (onepulse_used) {
						temp_compensation_cmd[2].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][5];
						temp_compensation_cmd[4].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][6];
						temp_compensation_cmd[6].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][7];
					} else {
						temp_compensation_cmd[2].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][0];
						temp_compensation_cmd[4].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][1];
						temp_compensation_cmd[6].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][2];
					}
					temp_compensation_cmd[12].para_list[i+1] = 0;
					temp_compensation_cmd[14].para_list[i+1] = 0;
					temp_compensation_cmd[16].para_list[i+1] = 0;
				} else {
					temp_compensation_cmd[2].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][0];
					temp_compensation_cmd[4].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][1];
					temp_compensation_cmd[6].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][2];
					temp_compensation_cmd[12].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][5];
					temp_compensation_cmd[14].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][6];
					temp_compensation_cmd[16].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][7];
				}
			}

			for (i = 0; i < 3; i++) {
				/*
				 * the left side is cmd register
				 * the right side is dtsi data, it is 3-dimen array selected by current brightness and temperature
				 * cmd register 8 10 is low freq 90HZ EM Duty, dtsi data 3 4 is low freq 90HZ EM Duty
				 * cmd register 20 is low freq 90HZ Vref2, dtsi data 11 is low freq 90HZ Vref2
				 * cmd register 18 is low freq 120HZ Vref2, cmd register 22 is high freq 120HZ Vref2
				 * dtsi data 8 is low  freq 120HZ Vref2
				 * dtsi data 14 is high freq 120HZ Vref2 if not 1pulse, if 1pulse, it is REUSED as low freq 120HZ Vref2 of 1pulse
				 * cmd register len is 4, for register 8 10, dtsi data will be repeated 3 times to set cmd register
				 * cmd register len is 3, for register 20 18 22, dtsi data will be picked 3 len data to set cmd register
				 * when 1pulse support
				 *  cmd register 22 is seted 0
				 *  if onepulse_used cmd register 18 is seted from dtsi data 14-16, else seted from dtsi data 8-10
				 * when 1pulse NOT support
				 *  cmd register 18 is seted from dtsi data 8-10
				 *  cmd register 22 is seted from dtsi data 14-16
				 */
				temp_compensation_cmd[8].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][3];
				temp_compensation_cmd[10].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][4];
				temp_compensation_cmd[20].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][11+i];
				if (panel->oplus_priv.pwm_onepulse_support) {
					if (onepulse_used) {
						temp_compensation_cmd[18].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][14+i];
					} else {
						temp_compensation_cmd[18].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][8+i];
					}
					temp_compensation_cmd[22].para_list[i+1] = 0;
				} else {
					temp_compensation_cmd[18].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][8+i];
					temp_compensation_cmd[22].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][14+i];
				}
			}

			for (i = 0; i < 4; i++) {
				/*
				 * the left side is cmd register
				 * the right side is dtsi data, it is 3-dimen array selected by current brightness and temperature
				 * cmd register 25 is Vdata
				 * dtsi data 17-20 is low  freq Vdata
				 * dtsi data 21-24 is high freq Vdata if not 1pulse, if 1pulse, it is REUSED as low freq Vdata of 1pulse
				 * cmd register len is 4, dtsi data will be picked 4 len data to set cmd register
				 * when pwm_turbo or onepulse_used
				 *  cmd register 25 is seted from dtsi data 21-24
				 * when else
				 *  cmd register 25 is seted from dtsi data 17-20
				 */
				if (oplus_panel_pwm_turbo_is_enabled(panel) || onepulse_used) {
					temp_compensation_cmd[25].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][21+i];
				} else {
					temp_compensation_cmd[25].para_list[i+1] = p_oplus_temp_compensation_params->data[dbv_index][temp_index][17+i];
				}
			}

			TEMP_COMPENSATION_INFO("refresh_rate:%u,setting_mode:%u\n", refresh_rate, setting_mode);
			TEMP_COMPENSATION_INFO("last_bl_lvl:%u,bl_lvl:%u,last_dbv_index:%u,dbv_index:%u,ntc_temp:%d,last_temp_index:%u,temp_index:%u,set temp compensation cmd\n",
				last_bl_lvl, bl_lvl, last_dbv_index, dbv_index, ntc_temp, last_temp_index, temp_index);

			if (!oplus_temp_compensation_dry_run_is_enabled()) {
				rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_TEMPERATURE_COMPENSATION);
				if (rc) {
					TEMP_COMPENSATION_ERR("[%s] failed to send DSI_CMD_TEMPERATURE_COMPENSATION cmds rc=%d\n", panel->name, rc);
				}
			} else {
				TEMP_COMPENSATION_INFO("dry running\n");
			}

			for (i = 0; i < lcm_cmd_count; i++) {
				for (j = 0; j < temp_compensation_cmd[i].count; j++) {
					TEMP_COMPENSATION_DEBUG("temp_compensation_cmd[%u][%u]=0x%02X\n", i, j, temp_compensation_cmd[i].para_list[j]);
				}
			}
		}
	}

	last_dbv_index = dbv_index;
	last_temp_index = temp_index;
	last_bl_lvl = bl_lvl;

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_cmd_set");

	TEMP_COMPENSATION_DEBUG("end\n");

	return rc;
}

void oplus_temp_compensation_first_half_frame_cmd_set_work_handler(struct work_struct *work_item)
{
	int rc = 0;
	struct dsi_display *display = oplus_display_get_current_display();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(display) || IS_ERR_OR_NULL(display->panel)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return;
	}

	if (!strcmp(display->panel->type, "secondary")) {
		TEMP_COMPENSATION_INFO("ignore secondary panel function calls\n");
		return;
	}

	if (!dsi_panel_initialized(display->panel)) {
		TEMP_COMPENSATION_ERR("should not set temp compensation cmd in the first half frame if panel is not initialized\n");
		return;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_first_half_frame_cmd_set_work_handler");

	oplus_sde_early_wakeup(display->panel);
	oplus_wait_for_vsync(display->panel);

	mutex_lock(&display->panel->panel_lock);
	TEMP_COMPENSATION_DEBUG("OPLUS_TEMP_COMPENSATION_FIRST_HALF_FRAME_SETTING\n");
	rc = oplus_temp_compensation_cmd_set(display->panel, OPLUS_TEMP_COMPENSATION_FIRST_HALF_FRAME_SETTING);
	if (rc) {
		TEMP_COMPENSATION_ERR("failed to set temp compensation cmd, rc=%d\n", rc);
	}
	mutex_unlock(&display->panel->panel_lock);

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_first_half_frame_cmd_set_work_handler");

	TEMP_COMPENSATION_DEBUG("end\n");

	return;
}

int oplus_temp_compensation_first_half_frame_cmd_set(void *dsi_panel)
{
	struct dsi_panel *panel = dsi_panel;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(panel) || IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (!strcmp(panel->type, "secondary")) {
		TEMP_COMPENSATION_INFO("ignore secondary panel function calls\n");
		return 0;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_first_half_frame_cmd_set");

	/*
	 temp compensation cmd should be sent in the first half frame in 60hz timing.
	 otherwise,there would be a low probability backlight flash issue.
	*/
	if (p_oplus_temp_compensation_params->need_to_set_in_first_half_frame) {
		TEMP_COMPENSATION_DEBUG("need to set temp compensation cmd in the first half frame\n");
		queue_work(p_oplus_temp_compensation_params->first_half_frame_cmd_set_wq, &p_oplus_temp_compensation_params->first_half_frame_cmd_set_work);

		p_oplus_temp_compensation_params->need_to_set_in_first_half_frame = false;
		TEMP_COMPENSATION_DEBUG("oplus_temp_compensation_need_to_set_in_first_half_frame:%d\n", p_oplus_temp_compensation_params->need_to_set_in_first_half_frame);
		OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_need_to_set_in_first_half_frame",
											p_oplus_temp_compensation_params->need_to_set_in_first_half_frame);
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_first_half_frame_cmd_set");

	TEMP_COMPENSATION_DEBUG("end\n");

	return 0;
}

int oplus_temp_compensation_temp_check(void *dsi_display)
{
	int rc = 0;
	int ntc_temp = 0;
	int shell_temp = 0;
	int last_ntc_temp = 0;
	struct dsi_display *display = dsi_display;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(display) || IS_ERR_OR_NULL(display->panel) || IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (!strcmp(display->panel->type, "secondary")) {
		TEMP_COMPENSATION_INFO("ignore secondary panel function calls\n");
		return rc;
	}

	if (!dsi_panel_initialized(display->panel)) {
		TEMP_COMPENSATION_ERR("should not check temperature if panel is not initialized\n");
		return -EINVAL;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_temp_check");

	last_ntc_temp = p_oplus_temp_compensation_params->ntc_temp;
	ntc_temp = oplus_temp_compensation_get_ntc_temp();
	shell_temp = oplus_temp_compensation_get_shell_temp();

	if (oplus_temp_compensation_get_temp_index(last_ntc_temp) != oplus_temp_compensation_get_temp_index(ntc_temp)) {
		mutex_lock(&display->panel->panel_lock);
		rc = oplus_temp_compensation_cmd_set(display->panel, OPLUS_TEMP_COMPENSATION_TEMPERATURE_SETTING);
		if (rc) {
			TEMP_COMPENSATION_ERR("failed to set temp compensation cmd, rc=%d\n", rc);
		}
		mutex_unlock(&display->panel->panel_lock);

		TEMP_COMPENSATION_INFO("last_ntc_temp:%d,current_ntc_temp:%d,bl_lvl:%u,update temp compensation cmd\n",
									last_ntc_temp, ntc_temp, display->panel->bl_config.bl_level);
	}

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	rc = oplus_adfr_temperature_detection_handle(display, ntc_temp, shell_temp);
	if (rc) {
		TEMP_COMPENSATION_ERR("failed to handle temperature detection\n");
	}
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_temp_check");

	TEMP_COMPENSATION_DEBUG("end\n");

	return rc;
}

/* -------------------- node -------------------- */
/* config */
ssize_t oplus_temp_compensation_set_config_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int config = 0;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(buf) || IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return count;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_set_config_attr");

	sscanf(buf, "%u", &config);

	p_oplus_temp_compensation_params->config = config;
	TEMP_COMPENSATION_INFO("oplus_temp_compensation_config:0x%x\n", p_oplus_temp_compensation_params->config);
	OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_config", p_oplus_temp_compensation_params->config);

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_set_config_attr");

	TEMP_COMPENSATION_DEBUG("end\n");

	return count;
}

ssize_t oplus_temp_compensation_get_config_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(buf) || IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_get_config_attr");

	TEMP_COMPENSATION_INFO("config:0x%x\n", p_oplus_temp_compensation_params->config);

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_get_config_attr");

	TEMP_COMPENSATION_DEBUG("end\n");

	return sprintf(buf, "%u\n", p_oplus_temp_compensation_params->config);
}

/* ntc temp */
ssize_t oplus_temp_compensation_set_ntc_temp_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ntc_temp = 0;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (!oplus_temp_compensation_is_supported()) {
		TEMP_COMPENSATION_ERR("temp compensation is not supported\n");
		return count;
	}

	if (IS_ERR_OR_NULL(buf) || IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return count;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_set_ntc_temp_attr");

	sscanf(buf, "%d", &ntc_temp);
	TEMP_COMPENSATION_INFO("set ntc temp to %d\n", ntc_temp);

	if (ntc_temp == -1) {
		p_oplus_temp_compensation_params->fake_ntc_temp = false;
	} else {
		p_oplus_temp_compensation_params->ntc_temp = ntc_temp;
		p_oplus_temp_compensation_params->fake_ntc_temp = true;
	}

	TEMP_COMPENSATION_INFO("oplus_temp_compensation_ntc_temp:%d\n", p_oplus_temp_compensation_params->ntc_temp);
	OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_ntc_temp", p_oplus_temp_compensation_params->ntc_temp);
	TEMP_COMPENSATION_INFO("oplus_temp_compensation_fake_ntc_temp:%d\n", p_oplus_temp_compensation_params->fake_ntc_temp);
	OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_fake_ntc_temp", p_oplus_temp_compensation_params->fake_ntc_temp);

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_set_ntc_temp_attr");

	TEMP_COMPENSATION_DEBUG("end\n");

	return count;
}

ssize_t oplus_temp_compensation_get_ntc_temp_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	int ntc_temp = 0;

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(buf)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_get_ntc_temp_attr");

	if (!oplus_temp_compensation_is_supported()) {
		ntc_temp = -EINVAL;
		TEMP_COMPENSATION_ERR("temp compensation is not supported\n");
	} else {
		ntc_temp = oplus_temp_compensation_get_ntc_temp();
		TEMP_COMPENSATION_INFO("ntc_temp:%d\n", ntc_temp);
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_get_ntc_temp_attr");

	TEMP_COMPENSATION_DEBUG("end\n");

	return sprintf(buf, "%d\n", ntc_temp);
}

/* shell temp */
ssize_t oplus_temp_compensation_set_shell_temp_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int shell_temp = 0;
	struct oplus_temp_compensation_params *p_oplus_temp_compensation_params = oplus_temp_compensation_get_params();

	TEMP_COMPENSATION_DEBUG("start\n");

	if (!oplus_temp_compensation_is_supported()) {
		TEMP_COMPENSATION_ERR("temp compensation is not supported\n");
		return count;
	}

	if (IS_ERR_OR_NULL(buf) || IS_ERR_OR_NULL(p_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return count;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_set_shell_temp_attr");

	sscanf(buf, "%d", &shell_temp);
	TEMP_COMPENSATION_INFO("set shell temp to %d\n", shell_temp);

	if (shell_temp == -1) {
		p_oplus_temp_compensation_params->fake_shell_temp = false;
	} else {
		p_oplus_temp_compensation_params->shell_temp = shell_temp;
		p_oplus_temp_compensation_params->fake_shell_temp = true;
	}

	TEMP_COMPENSATION_INFO("oplus_temp_compensation_shell_temp:%d\n", p_oplus_temp_compensation_params->shell_temp);
	OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_shell_temp", p_oplus_temp_compensation_params->shell_temp);
	TEMP_COMPENSATION_INFO("oplus_temp_compensation_fake_shell_temp:%d\n", p_oplus_temp_compensation_params->fake_shell_temp);
	OPLUS_TEMP_COMPENSAITON_TRACE_INT("oplus_temp_compensation_fake_shell_temp", p_oplus_temp_compensation_params->fake_shell_temp);

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_set_shell_temp_attr");

	TEMP_COMPENSATION_DEBUG("end\n");

	return count;
}

ssize_t oplus_temp_compensation_get_shell_temp_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	int shell_temp = 0;

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(buf)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN("oplus_temp_compensation_get_shell_temp_attr");

	if (!oplus_temp_compensation_is_supported()) {
		shell_temp = -EINVAL;
		TEMP_COMPENSATION_ERR("temp compensation is not supported\n");
	} else {
		shell_temp = oplus_temp_compensation_get_shell_temp();
		TEMP_COMPENSATION_INFO("shell_temp:%d\n", shell_temp);
	}

	OPLUS_TEMP_COMPENSAITON_TRACE_END("oplus_temp_compensation_get_shell_temp_attr");

	TEMP_COMPENSATION_DEBUG("end\n");

	return sprintf(buf, "%d\n", shell_temp);
}
