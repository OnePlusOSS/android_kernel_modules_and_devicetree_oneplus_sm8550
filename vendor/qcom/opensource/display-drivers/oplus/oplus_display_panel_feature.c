/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
**
** File : oplus_display_panel_feature.c
** Description : oplus display panel char dev  /dev/oplus_panel
** Version : 1.0
** Date : 2021/11/17
** Author : Display
******************************************************************/
#include <drm/drm_mipi_dsi.h>
#include "dsi_parser.h"
#include "dsi_display.h"
#include "dsi_panel.h"
#include "dsi_clk.h"
#include "oplus_bl.h"
/* OPLUS_FEATURE_ADFR, include header file */
#include "oplus_adfr.h"
#include "oplus_display_panel_feature.h"
#include "oplus_display_private_api.h"
#include "oplus_display_interface.h"

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "oplus_onscreenfingerprint.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

#if defined(CONFIG_PXLW_IRIS)
#include "dsi_iris_api.h"
#endif

extern int lcd_closebl_flag;
extern u32 oplus_last_backlight;

int oplus_panel_get_serial_number_info(struct dsi_panel *panel)
{
	struct dsi_parser_utils *utils = NULL;
	int ret = 0;
	if (!panel) {
		LCD_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}
	utils = &panel->utils;

	panel->oplus_ser.serial_number_support = utils->read_bool(utils->data,
			"oplus,dsi-serial-number-enabled");
	LCD_INFO("oplus,dsi-serial-number-enabled: %s\n", panel->oplus_ser.serial_number_support ? "true" : "false");

	if (panel->oplus_ser.serial_number_support) {
		panel->oplus_ser.is_reg_lock = utils->read_bool(utils->data, "oplus,dsi-serial-number-lock");
		LCD_INFO("oplus,dsi-serial-number-lock: %s\n", panel->oplus_ser.is_reg_lock ? "true" : "false");

		ret = utils->read_u32(utils->data, "oplus,dsi-serial-number-reg",
				&panel->oplus_ser.serial_number_reg);
		if (ret) {
			LCD_INFO("failed to get oplus,dsi-serial-number-reg\n");
			panel->oplus_ser.serial_number_reg = 0xA1;
		}

		ret = utils->read_u32(utils->data, "oplus,dsi-serial-number-index",
				&panel->oplus_ser.serial_number_index);
		if (ret) {
			LCD_INFO("failed to get oplus,dsi-serial-number-index\n");
			/* Default sync start index is set 5 */
			panel->oplus_ser.serial_number_index = 7;
		}

		ret = utils->read_u32(utils->data, "oplus,dsi-serial-number-read-count",
				&panel->oplus_ser.serial_number_conut);
		if (ret) {
			LCD_INFO("failed to get oplus,dsi-serial-number-read-count\n");
			/* Default  read conut 5 */
			panel->oplus_ser.serial_number_conut = 5;
		}
	}
	return 0;
}

int oplus_panel_features_config(struct dsi_panel *panel)
{
	struct dsi_parser_utils *utils = NULL;
	if (!panel) {
		LCD_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		LCD_INFO("iris secondary panel no need config\n");
		return 0;
	}
#endif

	utils = &panel->utils;
	panel->oplus_priv.dp_support = utils->get_property(utils->data,
			"oplus,dp-enabled", NULL);

	if (!panel->oplus_priv.dp_support) {
		LCD_INFO("Failed to found panel dp support, using null dp config\n");
		panel->oplus_priv.dp_support = false;
	}

	panel->oplus_priv.cabc_enabled = utils->read_bool(utils->data,
			"oplus,dsi-cabc-enabled");
	LCD_INFO("oplus,dsi-cabc-enabled: %s\n", panel->oplus_priv.cabc_enabled ? "true" : "false");

	panel->oplus_priv.dre_enabled = utils->read_bool(utils->data,
			"oplus,dsi-dre-enabled");
	LCD_INFO("oplus,dsi-dre-enabled: %s\n", panel->oplus_priv.dre_enabled ? "true" : "false");

	panel->oplus_priv.pwm_turbo_support = utils->read_bool(utils->data,
			"oplus,pwm-turbo-support");
	LCD_INFO("oplus,pwm-turbo-support: %s\n",
			panel->oplus_priv.pwm_turbo_support ? "true" : "false");
	panel->oplus_priv.pwm_turbo_enabled = utils->read_bool(utils->data,
			"oplus,pwm-turbo-enabled-default");
	LCD_INFO("oplus,pwm-turbo-enabled-default: %s\n",
			panel->oplus_priv.pwm_turbo_enabled ? "true" : "false");

	oplus_panel_get_serial_number_info(panel);

	return 0;
}

int oplus_panel_post_on_backlight(void *display, struct dsi_panel *panel, u32 bl_lvl)
{
	struct dsi_display *dsi_display = display;
	int rc = 0;

	if (!panel || !dsi_display) {
		LCD_ERR("oplus post backlight No panel device\n");
		return -ENODEV;
	}

	LCD_DEBUG_BACKLIGHT("[%s] display backlight changed: %d -> %d\n",
			panel->oplus_priv.vendor_name, panel->bl_config.bl_level, bl_lvl);

	/* Add some delay to avoid screen flash */
	if (panel->need_power_on_backlight && bl_lvl) {
		panel->need_power_on_backlight = false;
		rc = dsi_display_clk_ctrl(dsi_display->dsi_clk_handle,
			DSI_CORE_CLK, DSI_CLK_ON);
		rc |= dsi_panel_tx_cmd_set(panel, DSI_CMD_POST_ON_BACKLIGHT);
		rc |= dsi_display_clk_ctrl(dsi_display->dsi_clk_handle,
			DSI_CORE_CLK, DSI_CLK_OFF);
		if (rc) {
			LCD_ERR("[%s] failed to send %s, rc=%d\n",
				panel->oplus_priv.vendor_name,
				cmd_set_prop_map[DSI_CMD_POST_ON_BACKLIGHT],
				rc);
		}

		atomic_set(&panel->esd_pending, 0);

		if ((!strcmp(panel->name, "BOE AB319 NT37701B UDC") ||
				!strcmp(panel->name, "BOE AB241 NT37701A")) &&
				__oplus_get_power_status() == OPLUS_DISPLAY_POWER_ON)
			oplus_panel_event_data_notifier_trigger(panel,
					DRM_PANEL_EVENT_UNBLANK, 0, true);
	}
	return 0;
}

u32 oplus_panel_silence_backlight(struct dsi_panel *panel, u32 bl_lvl)
{
	u32 bl_temp = 0;
	if (!panel) {
		LCD_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}

	bl_temp = bl_lvl;

	if (lcd_closebl_flag) {
		LCD_INFO("silence reboot we should set backlight to zero\n");
		bl_temp = 0;
	}
	return bl_temp;
}

void oplus_panel_restore_auto_mode(struct dsi_panel *panel) {
	static bool adfr_init;
	int ret;

	if (!adfr_init && oplus_adfr_is_support() &&
			((!strcmp(panel->oplus_priv.vendor_name, "AMB670YF07_CS")) ||
			(!strcmp(panel->oplus_priv.vendor_name, "AMB670YF07_FS")) ||
			(!strcmp(panel->oplus_priv.vendor_name, "AMB670YF08_CS")) ||
			(!strcmp(panel->oplus_priv.vendor_name, "AMB670YF08_FS")))) {
		LCD_INFO("[DISP] Restore adfr auto parameter\n");

		mutex_lock(&panel->panel_tx_lock);
		ret = dsi_panel_tx_cmd_set(panel, DSI_CMD_SET_QSYNC_OFF);
		mutex_unlock(&panel->panel_tx_lock);

		mutex_lock(&panel->panel_tx_lock);
		ret = dsi_panel_tx_cmd_set(panel, DSI_CMD_QSYNC_MIN_FPS_0);
		mutex_unlock(&panel->panel_tx_lock);
		adfr_init = true;
	}
}

void oplus_panel_update_backlight(struct dsi_panel *panel,
		struct mipi_dsi_device *dsi, u32 bl_lvl)
{
	int rc = 0;
	u64 inverted_dbv_bl_lvl = 0;

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	if (oplus_ofp_is_supported()) {
		if (oplus_ofp_backlight_filter(panel, bl_lvl)) {
			return;
		}
	}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

	/* backlight value mapping */
	oplus_panel_global_hbm_mapping(panel, &bl_lvl);

	/* will inverted display brightness value */
	if (panel->bl_config.bl_inverted_dbv)
		inverted_dbv_bl_lvl = (((bl_lvl & 0xff) << 8) | (bl_lvl >> 8));
	else
		inverted_dbv_bl_lvl = bl_lvl;

	/* OPLUS_FEATURE_ADFR, backlight filter for OA */
	if (oplus_adfr_is_support()) {
		/* if backlight cmd is set after qsync window setting and qsync is enable, filter it
			otherwise tearing issue happen */
		if ((oplus_adfr_backlight_cmd_filter_get() == true) && (inverted_dbv_bl_lvl != 0)) {
			LCD_INFO("kVRR filter backlight cmd\n");
		} else {
			mutex_lock(&panel->panel_tx_lock);
#if defined(CONFIG_PXLW_IRIS)
			if (iris_is_chip_supported() && iris_is_pt_mode(panel))
				rc = iris_update_backlight(inverted_dbv_bl_lvl);
			else
#endif
			rc = mipi_dsi_dcs_set_display_brightness(dsi, inverted_dbv_bl_lvl);
			if (rc < 0)
				LCD_ERR("failed to update dcs backlight:%d\n", bl_lvl);
			mutex_unlock(&panel->panel_tx_lock);
		}
	} else {
#if defined(CONFIG_PXLW_IRIS)
		if (iris_is_chip_supported() && iris_is_pt_mode(panel))
			rc = iris_update_backlight(inverted_dbv_bl_lvl);
		else
#endif
		rc = mipi_dsi_dcs_set_display_brightness(dsi, inverted_dbv_bl_lvl);
		if (rc < 0)
			LCD_ERR("failed to update dcs backlight:%d\n", bl_lvl);
	}
#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && !iris_is_pt_mode(panel))
		rc = iris_update_backlight_value(bl_lvl);
#endif

	oplus_panel_restore_auto_mode(panel);

	LCD_DEBUG_BACKLIGHT("[%s] panel backlight changed: %d -> %d\n",
			panel->oplus_priv.vendor_name, oplus_last_backlight, bl_lvl);

	oplus_last_backlight = bl_lvl;
}

