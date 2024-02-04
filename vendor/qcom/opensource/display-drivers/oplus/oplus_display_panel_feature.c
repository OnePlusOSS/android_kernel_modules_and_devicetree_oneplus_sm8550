/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
**
** File : oplus_display_panel_feature.c
** Description : oplus display panel char dev  /dev/oplus_panel
** Version : 1.0
** Date : 2022/08/01
** Author : Display
******************************************************************/
#include <drm/drm_mipi_dsi.h>
#include "sde_trace.h"
#include "dsi_parser.h"
#include "dsi_display.h"
#include "dsi_panel.h"
#include "dsi_clk.h"
#include "oplus_bl.h"
#include <linux/ktime.h>
#include "oplus_display_panel_feature.h"
#include "oplus_display_private_api.h"
#include "oplus_display_interface.h"
#include "oplus_display_panel_common.h"

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
#include "oplus_adfr.h"
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
#include "oplus_display_temp_compensation.h"
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "oplus_onscreenfingerprint.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

#if defined(CONFIG_PXLW_IRIS)
#include "dsi_iris_api.h"
#endif

extern int lcd_closebl_flag;
extern u32 oplus_last_backlight;
int switch_pwm_in_pre_bl = 0;

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

		panel->oplus_ser.is_switch_page = utils->read_bool(utils->data,
			"oplus,dsi-serial-number-switch-page");
		LCD_INFO("oplus,dsi-serial-number-switch-page: %s", panel->oplus_ser.is_switch_page ? "true" : "false");
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

	panel->oplus_priv.pwm_switch_support = utils->read_bool(utils->data,
			"oplus,pwm-switch-support");
	LCD_INFO("oplus,pwm-switch-support: %s\n",
			panel->oplus_priv.pwm_switch_support ? "true" : "false");

	panel->oplus_priv.pwm_switch_restore_support = utils->read_bool(utils->data,
			"oplus,pwm-switch-restore-support");
	LCD_INFO("oplus,pwm-switch-restore-support: %s\n",
			panel->oplus_priv.pwm_switch_restore_support ? "true" : "false");

	panel->oplus_priv.pwm_onepulse_support = utils->read_bool(utils->data,
			"oplus,pwm-onepulse-support");
	LCD_INFO("oplus,pwm-onepulse-support: %s\n",
			panel->oplus_priv.pwm_onepulse_support ? "true" : "false");
	panel->oplus_priv.pwm_onepulse_enabled = utils->read_bool(utils->data,
			"oplus,pwm-onepulse-default-enabled");
	LCD_INFO("oplus,pwm-onepulse-default-enabled: %s\n",
			panel->oplus_priv.pwm_onepulse_enabled ? "true" : "false");

	if(panel->oplus_priv.pwm_onepulse_support) {
		panel->oplus_priv.directional_onepulse_switch = utils->read_bool(utils->data,
			"oplus,directional-onepulse-switch");
	} else {
		panel->oplus_priv.directional_onepulse_switch = false;
	}
	LCD_INFO("oplus,directional-onepulse-switch: %s\n",
			panel->oplus_priv.directional_onepulse_switch ? "true" : "false");
	if (panel->oplus_priv.directional_onepulse_switch) {
		panel->oplus_priv.pwm_sw_cmd_te_cnt = 0;
		panel->oplus_pwm_switch_send_next_cmdq_wq = create_singlethread_workqueue("oplus_pwm_switch_send_next_cmdq_wq");
		INIT_WORK(&panel->oplus_pwm_switch_send_next_cmdq_work, oplus_pwm_switch_send_next_cmdq_work_handler);
	}

	panel->oplus_priv.dynamic_demua_support = utils->read_bool(utils->data,
			"oplus,dynamic-demua-support");
	LCD_INFO("oplus,dynamic-demua-supportt: %s\n",
			panel->oplus_priv.dynamic_demua_support ? "true" : "false");

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
		panel->post_power_on = true;
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

void oplus_panel_update_backlight(struct dsi_panel *panel,
		struct mipi_dsi_device *dsi, u32 bl_lvl)
{
	int rc = 0;
	u64 inverted_dbv_bl_lvl = 0;
	int pack_51 = 0;

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	if (oplus_adfr_osync_backlight_filter(panel, bl_lvl)) {
		return;
	}
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	if (oplus_ofp_is_supported()) {
		oplus_ofp_lhbm_backlight_update(NULL, panel, &bl_lvl);
		if (oplus_ofp_backlight_filter(panel, bl_lvl)) {
			return;
		}
	}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	SDE_ATRACE_BEGIN("oplus_panel_update_backlight");

#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
	if (oplus_temp_compensation_is_supported()) {
		oplus_temp_compensation_cmd_set(panel, OPLUS_TEMP_COMPENSATION_BACKLIGHT_SETTING);
	}
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */

	oplus_temp_compensation_wait_for_vsync_set = false;

	/* backlight value mapping */
	oplus_panel_global_hbm_mapping(panel, &bl_lvl);

	oplus_panel_pwm_switch_backlight(panel, bl_lvl, &pack_51);

#ifdef OPLUS_FEATURE_DISPLAY_HIGH_PRECISION
	oplus_adfr_high_precision_switch_state(panel);
#endif /* OPLUS_FEATURE_DISPLAY_HIGH_PRECISION */

	if (!panel->oplus_priv.need_sync && panel->cur_mode->priv_info->async_bl_delay && !pack_51) {
		if (panel->oplus_priv.disable_delay_bl_count > 0) {
			panel->oplus_priv.disable_delay_bl_count--;
		} else if (panel->oplus_priv.disable_delay_bl_count == 0) {
			oplus_apollo_async_bl_delay(panel);
		} else {
			DSI_INFO("invalid disable_delay_bl_count\n");
			panel->oplus_priv.disable_delay_bl_count = 0;
		}
	}

	/* will inverted display brightness value */
	if (panel->bl_config.bl_inverted_dbv)
		inverted_dbv_bl_lvl = (((bl_lvl & 0xff) << 8) | (bl_lvl >> 8));
	else
		inverted_dbv_bl_lvl = bl_lvl;

	if (!strcmp(panel->oplus_priv.vendor_name, "TM_NT37705") && (bl_lvl > 1)) {
		oplus_display_update_dbv(panel);
	}

	/* need to delay 51 to the next frame of pwm switch cmd */
	if (switch_pwm_in_pre_bl == 1 && panel->oplus_priv.pwm_sw_cmd_te_cnt > 1) {
		if (oplus_ofp_get_hbm_state()) {
			LCD_INFO("lhbm on state, cancel delay 51 to next frame operation\n");
		} else {
			oplus_sde_early_wakeup(panel);
			oplus_wait_for_vsync(panel);
			if (panel->cur_mode->timing.refresh_rate == 90 || panel->cur_mode->timing.refresh_rate == 60) {
				oplus_need_to_sync_te(panel);
			}
			LCD_INFO("bl_lvl %d delay to next frame for avoiding same frame with pwm_switch\n", bl_lvl);
		}
	}
	switch_pwm_in_pre_bl = 0;

	if(!pack_51) {
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
	} else {
		LCD_INFO("The cmd of backlight %d was packed in other cmd\n", bl_lvl);
		switch_pwm_in_pre_bl = 1;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && !iris_is_pt_mode(panel))
		rc = iris_update_backlight_value(bl_lvl);
#endif

#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
	if (oplus_temp_compensation_is_supported()) {
		oplus_temp_compensation_first_half_frame_cmd_set(panel);
	}
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */

	SDE_ATRACE_INT("current_bl_lvl", bl_lvl);

	oplus_panel_backlight_demura_dbv_switch(panel, bl_lvl);
	LCD_DEBUG_BACKLIGHT("[%s] panel backlight changed: %d -> %d\n",
			panel->oplus_priv.vendor_name, oplus_last_backlight, bl_lvl);
	oplus_last_backlight = bl_lvl;
	SDE_ATRACE_END("oplus_panel_update_backlight");
}

void oplus_printf_backlight_log(struct dsi_display *display, u32 bl_lvl) {
	struct timespec64 now;
	struct tm broken_time;
	static time64_t time_last = 0;
	struct backlight_log *bl_log;
	int i = 0;
	int len = 0;
	char backlight_log_buf[1024];

	ktime_get_real_ts64(&now);
	time64_to_tm(now.tv_sec, 0, &broken_time);
	if (now.tv_sec - time_last >= 60) {
		pr_info("<%s> dsi_display_set_backlight time:%02d:%02d:%02d.%03ld,bl_lvl:%d\n",
			display->panel->oplus_priv.vendor_name, broken_time.tm_hour, broken_time.tm_min,
			broken_time.tm_sec, now.tv_nsec / 1000000, bl_lvl);
		time_last = now.tv_sec;
	}

	if (!strcmp(display->display_type, "secondary")) {
		bl_log = &oplus_bl_log[DISPLAY_SECONDARY];
	} else {
		bl_log = &oplus_bl_log[DISPLAY_PRIMARY];
	}

	bl_log->backlight[bl_log->bl_count] = bl_lvl;
	bl_log->past_times[bl_log->bl_count] = now;
	bl_log->bl_count++;
	if (bl_log->bl_count >= BACKLIGHT_CACHE_MAX) {
		bl_log->bl_count = 0;
		memset(backlight_log_buf, 0, sizeof(backlight_log_buf));
		for (i = 0; i < BACKLIGHT_CACHE_MAX; i++) {
			time64_to_tm(bl_log->past_times[i].tv_sec, 0, &broken_time);
			len += snprintf(backlight_log_buf + len, sizeof(backlight_log_buf) - len,
				"%02d:%02d:%02d.%03ld:%d,", broken_time.tm_hour, broken_time.tm_min,
				broken_time.tm_sec, bl_log->past_times[i].tv_nsec / 1000000, bl_log->backlight[i]);
		}
		pr_info("<%s> len:%d dsi_display_set_backlight %s\n", display->panel->oplus_priv.vendor_name, len, backlight_log_buf);
	}
}

