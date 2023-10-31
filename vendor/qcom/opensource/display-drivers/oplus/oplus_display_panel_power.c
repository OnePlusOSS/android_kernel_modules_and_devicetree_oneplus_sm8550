/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
**
** File : oplus_display_panel_power.c
** Description : oplus display panel power control
** Version : 1.0
** Date : 2020/06/13
** Author : Display
******************************************************************/
#include "oplus_display_panel_power.h"

PANEL_VOLTAGE_BAK panel_vol_bak[PANEL_VOLTAGE_ID_MAX] = {{0}, {0}, {2, 0, 1, 2, ""}};
u32 panel_pwr_vg_base = 0;
extern int oplus_request_power_status;
DEFINE_MUTEX(oplus_power_status_lock);

static int oplus_panel_find_vreg_by_name(const char *name)
{
	int count = 0, i = 0;
	struct dsi_vreg *vreg = NULL;
	struct dsi_regulator_info *dsi_reg = NULL;
	struct dsi_display *display = get_main_display();

	if (!display) {
		return -ENODEV;
	}

	if (!display->panel) {
		return -EINVAL;
	}

	dsi_reg = &display->panel->power_info;
	count = dsi_reg->count;

	for (i = 0; i < count; i++) {
		vreg = &dsi_reg->vregs[i];
		LCD_INFO("finding: %s\n", vreg->vreg_name);

		if (!strcmp(vreg->vreg_name, name)) {
			LCD_INFO("find the vreg: %s\n", name);
			return i;

		} else {
			continue;
		}
	}

	LCD_ERR("dose not find the vreg: %s\n", name);

	return -EINVAL;
}


int dsi_panel_parse_panel_power_cfg(struct dsi_panel *panel)
{
	int rc = 0, i = 0;
	const char *name_vddi = NULL;
	const char *name_vddr = NULL;
	u32 *panel_vol = NULL;
	struct dsi_parser_utils *utils = &panel->utils;

	LCD_INFO("parse panel power config\n");

	if (!strcmp(panel->type, "primary")) {
		panel_vol = &panel_vol_bak[PANEL_VOLTAGE_ID_VDDI].voltage_id;
		rc = utils->read_u32_array(utils->data, "qcom,panel_voltage_vddi",
				panel_vol, PANEL_VOLTAGE_VALUE_COUNT);

		if (rc) {
			LCD_ERR("[%s] failed to parse panel_voltage vddi\n",
					panel->oplus_priv.vendor_name);
			goto error;
		}

		rc = utils->read_string(utils->data, "qcom,panel_voltage_vddi_name",
				&name_vddi);

		if (rc) {
			LCD_ERR("[%s] failed to parse vddi name\n",
					panel->oplus_priv.vendor_name);
			goto error;

		} else {
			LCD_INFO("[%s] surccess to parse vddi name %s\n",
					panel->oplus_priv.vendor_name, name_vddi);
			strcpy(panel_vol_bak[PANEL_VOLTAGE_ID_VDDI].pwr_name, name_vddi);
		}

		panel_vol = &panel_vol_bak[PANEL_VOLTAGE_ID_VDDR].voltage_id;
		rc = utils->read_u32_array(utils->data, "qcom,panel_voltage_vddr",
				panel_vol, PANEL_VOLTAGE_VALUE_COUNT);

		if (rc) {
			LCD_ERR("[%s] failed to parse panel_voltage vddr\n",
					panel->oplus_priv.vendor_name);
			goto error;
		}

		rc = utils->read_string(utils->data, "qcom,panel_voltage_vddr_name",
				&name_vddr);

		if (rc) {
			LCD_ERR("[%s] failed to parse vddr name\n",
					panel->oplus_priv.vendor_name);
			goto error;

		} else {
			LCD_INFO("[%s] surccess to parse vddr name %s\n",
					panel->oplus_priv.vendor_name, name_vddr);
			strcpy(panel_vol_bak[PANEL_VOLTAGE_ID_VDDR].pwr_name, name_vddr);
		}
		/* add for debug */
		for (i = 0; i < PANEL_VOLTAGE_ID_MAX; i++) {
			LCD_INFO("panel_voltage[%d] = %d,%d,%d,%d,%s\n", i,
					panel_vol_bak[i].voltage_id,
					panel_vol_bak[i].voltage_min, panel_vol_bak[i].voltage_current,
					panel_vol_bak[i].voltage_max, panel_vol_bak[i].pwr_name);
		}
	}

error:
	return rc;
}

static u32 oplus_panel_update_current_voltage(u32 id)
{
	int vol_current = 0, pwr_id = 0;
	struct dsi_vreg *dsi_reg = NULL;
	struct dsi_regulator_info *dsi_reg_info = NULL;
	struct dsi_display *display = get_main_display();

	if (!display) {
		return -ENODEV;
	}

	if (!display->panel || !display->drm_conn) {
		return -EINVAL;
	}

	dsi_reg_info = &display->panel->power_info;
	pwr_id = oplus_panel_find_vreg_by_name(panel_vol_bak[id].pwr_name);

	if (pwr_id < 0) {
		LCD_ERR("can't find the pwr_id, please check the vreg name\n");
		return pwr_id;
	}

	dsi_reg = &dsi_reg_info->vregs[pwr_id];

	if (!dsi_reg) {
		return -EINVAL;
	}

	vol_current = regulator_get_voltage(dsi_reg->vreg);

	return vol_current;
}

int oplus_display_panel_get_pwr(void *data)
{
	int ret = 0;
	struct panel_vol_get *panel_vol = data;
	u32 vol_id = ((panel_vol->panel_id & 0x0F) - 1);

	if (vol_id < 0 || vol_id >= PANEL_VOLTAGE_ID_MAX) {
		LCD_ERR("error id: %d\n", vol_id);
		return -EINVAL;
	}

	LCD_INFO("id = %d\n", vol_id);
	panel_vol->panel_min = panel_vol_bak[vol_id].voltage_min;
	panel_vol->panel_max = panel_vol_bak[vol_id].voltage_max;
	panel_vol->panel_cur = panel_vol_bak[vol_id].voltage_current;

	if (vol_id < PANEL_VOLTAGE_ID_VG_BASE &&
		vol_id >= PANEL_VOLTAGE_ID_VDDI) {
		ret = oplus_panel_update_current_voltage(vol_id);
		if (ret < 0) {
			LCD_ERR("update_current_voltage error = %d\n", ret);
			return ret;
		} else {
			panel_vol->panel_cur = ret;
			LCD_ERR("[id min cur max] = [%u32, %u32, %u32, %u32]\n",
				vol_id, panel_vol->panel_min,
				panel_vol->panel_cur, panel_vol->panel_max);
			return 0;
		}
	}

	return ret;
}

int oplus_display_panel_set_pwr(void *data)
{
	struct panel_vol_set *panel_vol = data;
	int panel_vol_value = 0, rc = 0, panel_vol_id = 0, pwr_id = 0;
	struct dsi_vreg *dsi_reg = NULL;
	struct dsi_regulator_info *dsi_reg_info = NULL;
	struct dsi_display *display = get_main_display();

	panel_vol_id = ((panel_vol->panel_id & 0x0F)-1);
	panel_vol_value = panel_vol->panel_vol;

	if (panel_vol_id < 0 || panel_vol_id >= PANEL_VOLTAGE_ID_MAX) {
		LCD_ERR("error id: %d\n", panel_vol_id);
		return -EINVAL;
	}

	LCD_INFO("id = %d, value = %d\n",
			panel_vol_id, panel_vol_value);
	if (panel_vol_value < panel_vol_bak[panel_vol_id].voltage_min ||
			panel_vol_value > panel_vol_bak[panel_vol_id].voltage_max)
		return -EINVAL;

	if (!display) {
		return -ENODEV;
	}

	if (!display->panel || !display->drm_conn) {
		return -EINVAL;
	}

	if (panel_vol_id == PANEL_VOLTAGE_ID_VG_BASE) {
		LCD_ERR("set the VGH_L pwr = %d \n", panel_vol_value);
		panel_pwr_vg_base = panel_vol_value;
		return rc;
	}

	dsi_reg_info = &display->panel->power_info;

	pwr_id = oplus_panel_find_vreg_by_name(panel_vol_bak[panel_vol_id].pwr_name);
	if (pwr_id < 0) {
		LCD_ERR("can't find the vreg name, please re-check vreg name: %s \n",
			panel_vol_bak[panel_vol_id].pwr_name);
		return pwr_id;
	}

	dsi_reg = &dsi_reg_info->vregs[pwr_id];

	rc = regulator_set_voltage(dsi_reg->vreg, panel_vol_value, panel_vol_value);

	if (rc) {
		LCD_ERR("Set voltage(%s) fail, rc=%d\n",
			 dsi_reg->vreg_name, rc);
		return -EINVAL;
	}

	return rc;
}

int __oplus_set_request_power_status(int status)
{
	mutex_lock(&oplus_power_status_lock);

	if (status != oplus_request_power_status) {
		oplus_request_power_status = status;
	}

	mutex_unlock(&oplus_power_status_lock);
	return 0;
}

int oplus_display_panel_get_power_status(void *data) {
	uint32_t *power_status = data;

	LCD_INFO("oplus_display_get_power_status = %d\n", __oplus_get_power_status());
	(*power_status) = __oplus_get_power_status();

	return 0;
}

int oplus_display_panel_set_power_status(void *data) {
	uint32_t *temp_save = data;

	LCD_INFO("oplus_display_set_power_status = %d\n", (*temp_save));
	__oplus_set_request_power_status((*temp_save));

	return 0;
}

int oplus_display_panel_regulator_control(void *data) {
	uint32_t *temp_save_user = data;
	uint32_t temp_save = (*temp_save_user);
	struct dsi_display *temp_display;

	LCD_INFO("oplus_display_regulator_control = %d\n", temp_save);
	if(get_main_display() == NULL) {
		LCD_ERR("display is null\n");
		return -1;
	}
	temp_display = get_main_display();
	if(temp_save == 0) {
		dsi_pwr_enable_regulator(&temp_display->panel->power_info, false);
	} else if (temp_save == 1) {
		dsi_pwr_enable_regulator(&temp_display->panel->power_info, true);
	}

	return 0;
}
