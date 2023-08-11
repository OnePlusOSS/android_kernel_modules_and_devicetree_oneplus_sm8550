/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
**
** File : oplus_display_interface.c
** Description : oplus display interface
** Version : 1.0
** Date : 2022/05/30
** Author : Display
******************************************************************/
#include <drm/drm_print.h>
#include <drm/drm_connector.h>
#include <linux/msm_drm_notify.h>
#include <linux/module.h>

#include "oplus_display_interface.h"
#include "oplus_adfr.h"
#include "sde_color_processing.h"

extern int dc_apollo_enable;
extern int oplus_dimlayer_hbm;
extern unsigned int oplus_dsi_log_type;
extern const char *cmd_set_prop_map[];
extern int oplus_debug_max_brightness;

int oplus_panel_cmd_print(struct dsi_panel *panel, enum dsi_cmd_set_type type)
{
	switch (type) {
	case DSI_CMD_READ_SAMSUNG_PANEL_REGISTER_ON:
	case DSI_CMD_SET_ROI:
	case DSI_CMD_READ_SAMSUNG_PANEL_REGISTER_OFF:
	case DSI_CMD_FAKEFRAME:
	case DSI_CMD_QSYNC_MIN_FPS_0:
	case DSI_CMD_QSYNC_MIN_FPS_1:
	case DSI_CMD_ESD_SWITCH_PAGE:
		/* Do nothing */
		break;
	default:
		LCD_INFO("[%s] dsi_cmd: %s\n", panel->oplus_priv.vendor_name,
				cmd_set_prop_map[type]);
		break;
	}

	return 0;
}

int oplus_panel_cmd_switch(struct dsi_panel *panel, enum dsi_cmd_set_type *type)
{
	enum dsi_cmd_set_type type_store = *type;
	u32 count;

	/* switch the command when pwm turbo is enabled */
	if (oplus_panel_pwm_turbo_is_enabled(panel)) {
		switch (*type) {
		case DSI_CMD_HBM_ON:
			*type = DSI_CMD_PWM_TURBO_HBM_ON;
			break;
		case DSI_CMD_HBM_OFF:
			*type = DSI_CMD_PWM_TURBO_HBM_OFF;
			break;
		case DSI_CMD_AOR_ON:
			*type = DSI_CMD_PWM_TURBO_AOR_ON;
			break;
		case DSI_CMD_AOR_OFF:
			*type = DSI_CMD_PWM_TURBO_AOR_OFF;
			break;
		case DSI_CMD_SET_TIMING_SWITCH:
			*type = DSI_CMD_PWM_TURBO_TIMING_SWITCH;
		default:
			break;
		}
	}

	count = panel->cur_mode->priv_info->cmd_sets[*type].count;
	if (count == 0) {
		LCD_DEBUG("[%s] %s is undefined, restore to %s\n",
				panel->oplus_priv.vendor_name,
				cmd_set_prop_map[*type],
				cmd_set_prop_map[type_store]);
		*type = type_store;
	}

	return 0;
}

void oplus_ctrl_print_cmd_desc(struct dsi_ctrl *dsi_ctrl, const struct mipi_dsi_msg *msg)
{
	char buf[512];
	int len = 0;
	size_t i;
	char *tx_buf = (char*)msg->tx_buf;

	memset(buf, 0, sizeof(buf));

	/* Packet Info */
	len += snprintf(buf, sizeof(buf) - len,  "%02X ", msg->type);
	/* Last bit */
	/* len += snprintf(buf + len, sizeof(buf) - len, "%02X ", (msg->flags & MIPI_DSI_MSG_LASTCOMMAND) ? 1 : 0); */
	len += snprintf(buf + len, sizeof(buf) - len, "%02X ", (msg->flags) ? 1 : 0);
	len += snprintf(buf + len, sizeof(buf) - len, "%02X ", msg->channel);
	len += snprintf(buf + len, sizeof(buf) - len, "%02X ", (unsigned int)msg->flags);
	/* Delay */
	/* len += snprintf(buf + len, sizeof(buf) - len, "%02X ", msg->wait_ms); */
	len += snprintf(buf + len, sizeof(buf) - len, "%02X %02X ", msg->tx_len >> 8, msg->tx_len & 0x00FF);

	/* Packet Payload */
	for (i = 0 ; i < msg->tx_len ; i++) {
		len += snprintf(buf + len, sizeof(buf) - len, "%02X ", tx_buf[i]);
		/* Break to prevent show too long command */
		if (i > 160)
			break;
	}

	/* DSI_CTRL_ERR(dsi_ctrl, "%s\n", buf); */
	LCD_DEBUG_CMD("dsi_cmd: %s\n", buf);
}

int oplus_panel_init(struct dsi_panel *panel)
{
	int rc = 0;
	static bool panel_need_init = true;

	if (!panel_need_init)
		return 0;

	LCD_INFO("Send panel init dcs\n");

	mutex_lock(&panel->panel_lock);

	rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_PANEL_INIT);
	if (!rc)
		panel_need_init = false;

	mutex_unlock(&panel->panel_lock);

	return rc;
}

int oplus_panel_gpio_request(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_panel_reset_config *r_config;
	if (!panel) {
		LCD_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}

	r_config = &panel->reset_config;

	if (gpio_is_valid(r_config->panel_vout_gpio)) {
		rc = gpio_request(r_config->panel_vout_gpio, "panel_vout_gpio");
		if (rc) {
			LCD_ERR("request for panel_vout_gpio failed, rc=%d\n", rc);
			if (gpio_is_valid(r_config->panel_vout_gpio))
				gpio_free(r_config->panel_vout_gpio);
		}
	}
	if (gpio_is_valid(r_config->panel_vddr_aod_en_gpio)) {
		rc = gpio_request(r_config->panel_vddr_aod_en_gpio, "panel_vddr_aod_en_gpio");
		if (rc) {
			LCD_ERR("request for panel_vddr_aod_en_gpio failed, rc=%d\n", rc);
			if (gpio_is_valid(r_config->panel_vddr_aod_en_gpio))
				gpio_free(r_config->panel_vddr_aod_en_gpio);
		}
	}
	if (oplus_adfr_is_support()) {
		if (gpio_is_valid(panel->vsync_switch_gpio)) {
			rc = gpio_request(panel->vsync_switch_gpio, "vsync_switch_gpio");
			if (rc) {
				LCD_ERR("adfr request for vsync_switch_gpio failed, rc=%d\n", rc);
			}
		}
	}

	return rc;
}

int oplus_panel_gpio_release(struct dsi_panel *panel)
{
	struct dsi_panel_reset_config *r_config;
	if (!panel) {
		LCD_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}

	r_config = &panel->reset_config;

	if (gpio_is_valid(r_config->panel_vout_gpio))
		gpio_free(r_config->panel_vout_gpio);
	if (gpio_is_valid(r_config->panel_vddr_aod_en_gpio))
		gpio_free(r_config->panel_vddr_aod_en_gpio);

	if (oplus_adfr_is_support()) {
		if (gpio_is_valid(panel->vsync_switch_gpio))
			gpio_free(panel->vsync_switch_gpio);
	}

	return 0;
}

int oplus_panel_gpio_pre_on(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_panel_reset_config *r_config;
	if (!panel) {
		LCD_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}

	r_config = &panel->reset_config;

	if (panel->oplus_priv.gpio_pre_on &&
			gpio_is_valid(panel->reset_config.panel_vout_gpio)) {
		rc = gpio_direction_output(panel->reset_config.panel_vout_gpio, 1);
		if (rc)
			LCD_ERR("unable to set dir for panel_vout_gpio rc=%d\n", rc);
		gpio_set_value(panel->reset_config.panel_vout_gpio, 1);
	}

	return rc;
}

int oplus_panel_gpio_on(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_panel_reset_config *r_config;
	if (!panel) {
		LCD_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}

	r_config = &panel->reset_config;

	if (!panel->oplus_priv.gpio_pre_on &&
			gpio_is_valid(panel->reset_config.panel_vout_gpio)) {
		rc = gpio_direction_output(panel->reset_config.panel_vout_gpio, 1);
		if (rc)
			LCD_ERR("unable to set dir for panel_vout_gpio rc=%d\n", rc);
		gpio_set_value(panel->reset_config.panel_vout_gpio, 1);
	}
	if (gpio_is_valid(panel->reset_config.panel_vddr_aod_en_gpio)) {
		rc = gpio_direction_output(panel->reset_config.panel_vddr_aod_en_gpio, 1);
		if (rc)
			LCD_ERR("unable to set dir for panel_vddr_aod_en_gpio rc=%d\n", rc);
		gpio_set_value(panel->reset_config.panel_vddr_aod_en_gpio, 1);
	}

	return rc;
}

int oplus_panel_gpio_off(struct dsi_panel *panel)
{
	struct dsi_panel_reset_config *r_config;
	if (!panel) {
		LCD_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}

	r_config = &panel->reset_config;

	if (gpio_is_valid(panel->reset_config.panel_vout_gpio))
		gpio_set_value(panel->reset_config.panel_vout_gpio, 0);
	if (gpio_is_valid(panel->reset_config.panel_vddr_aod_en_gpio))
		gpio_set_value(panel->reset_config.panel_vddr_aod_en_gpio, 0);

	return 0;
}

int oplus_panel_gpio_parse(struct dsi_panel *panel)
{
	struct dsi_parser_utils *utils;
	struct dsi_panel_reset_config *r_config;
	if (!panel) {
		LCD_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}
	utils = &panel->utils;
	r_config = &panel->reset_config;

	panel->reset_config.panel_vout_gpio = utils->get_named_gpio(utils->data,
								"qcom,platform-panel-vout-gpio", 0);

	if (!gpio_is_valid(panel->reset_config.panel_vout_gpio)) {
		LCD_ERR("[%s] failed get panel_vout_gpio\n", panel->oplus_priv.vendor_name);
	}
	panel->reset_config.panel_vddr_aod_en_gpio = utils->get_named_gpio(utils->data,
								"qcom,platform-panel-vddr-aod-en-gpio", 0);

	if (!gpio_is_valid(panel->reset_config.panel_vddr_aod_en_gpio)) {
		LCD_ERR("[%s] failed get panel_vddr_aod_en_gpio\n", panel->oplus_priv.vendor_name);
	}

	return 0;
}

int oplus_panel_parse_vsync_config(
				struct dsi_display_mode *mode,
				struct dsi_parser_utils *utils)
{
	int rc;
	struct dsi_display_mode_priv_info *priv_info;

	priv_info = mode->priv_info;

	rc = utils->read_u32(utils->data, "oplus,apollo-panel-vsync-period",
				  &priv_info->vsync_period);
	if (rc) {
		LCD_DEBUG("panel prefill lines are not defined rc=%d\n", rc);
		priv_info->vsync_period = 1000000 / mode->timing.refresh_rate;
	}

	rc = utils->read_u32(utils->data, "oplus,apollo-panel-vsync-width",
				  &priv_info->vsync_width);
	if (rc) {
		LCD_DEBUG("panel vsync width not defined rc=%d\n", rc);
		priv_info->vsync_width = priv_info->vsync_period >> 1;
	}

	LCD_INFO("vsync width = %d, vsync period = %d\n", priv_info->vsync_width, priv_info->vsync_period);

	return 0;
}

int oplus_panel_mult_frac(int bright)
{
	int bl_lvl = 0;
	struct dsi_display *display = get_main_display();

	if (!display || !display->drm_conn || !display->drm_conn->state) {
		LCD_ERR("failed to find dsi display\n");
		return false;
	}

	if (oplus_debug_max_brightness) {
		bl_lvl = mult_frac(bright, oplus_debug_max_brightness,
			display->panel->bl_config.brightness_max_level);
	} else if (bright == 0) {
			bl_lvl = 0;
	} else {
		if (display->panel->oplus_priv.bl_remap && display->panel->oplus_priv.bl_remap_count) {
			int i = 0;
			int count = display->panel->oplus_priv.bl_remap_count;
			struct oplus_brightness_alpha *lut = display->panel->oplus_priv.bl_remap;

			for (i = 0; i < display->panel->oplus_priv.bl_remap_count; i++) {
				if (display->panel->oplus_priv.bl_remap[i].brightness >= bright)
					break;
			}

			if (i == 0)
				bl_lvl = lut[0].alpha;
			else if (i == count)
				bl_lvl = lut[count - 1].alpha;
			else
				bl_lvl = oplus_interpolate(bright, lut[i-1].brightness,
						lut[i].brightness, lut[i-1].alpha, lut[i].alpha);
		} else if (bright > display->panel->bl_config.brightness_normal_max_level) {
			bl_lvl = oplus_interpolate(bright,
					display->panel->bl_config.brightness_normal_max_level,
					display->panel->bl_config.brightness_max_level,
					display->panel->bl_config.bl_normal_max_level,
					display->panel->bl_config.bl_max_level);
		} else {
			bl_lvl = mult_frac(bright, display->panel->bl_config.bl_normal_max_level,
					display->panel->bl_config.brightness_normal_max_level);
		}
	}

	return bl_lvl;
}

int oplus_panel_event_data_notifier_trigger(struct dsi_panel *panel,
		enum panel_event_notification_type notif_type,
		u32 data,
		bool early_trigger)
{
	struct panel_event_notification notifier;
	enum panel_event_notifier_tag panel_type;

	if (!panel) {
		LCD_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}

	if (!strcmp(panel->type, "secondary")) {
		panel_type = PANEL_EVENT_NOTIFICATION_SECONDARY;
	} else {
		panel_type = PANEL_EVENT_NOTIFICATION_PRIMARY;
	}

	LCD_DEBUG_COMMON("[%s] type=0x%X, data=%d, early_trigger=%d\n",
			panel->type, notif_type, data, early_trigger);

	memset(&notifier, 0, sizeof(notifier));

	notifier.panel = &panel->drm_panel;
	notifier.notif_type = notif_type;
	notifier.notif_data.data = data;
	notifier.notif_data.early_trigger = early_trigger;

	panel_event_notification_trigger(panel_type, &notifier);

	return 0;
}
EXPORT_SYMBOL(oplus_panel_event_data_notifier_trigger);

int oplus_event_data_notifier_trigger(
		enum panel_event_notification_type notif_type,
		u32 data,
		bool early_trigger)
{
	struct dsi_display *display = oplus_display_get_current_display();

	if (!display || !display->panel) {
		LCD_ERR("Oplus Features config No display device\n");
		return -ENODEV;
	}

	oplus_panel_event_data_notifier_trigger(display->panel,
			notif_type, data, early_trigger);

	return 0;
}
EXPORT_SYMBOL(oplus_event_data_notifier_trigger);

int oplus_panel_backlight_notifier(struct dsi_panel *panel, u32 bl_lvl)
{
	u32 threshold = panel->bl_config.dc_backlight_threshold;
	bool dc_mode = panel->bl_config.oplus_dc_mode;

	if (dc_mode && (bl_lvl > 1 && bl_lvl < threshold)) {
		dc_mode = false;
		oplus_panel_event_data_notifier_trigger(panel,
				DRM_PANEL_EVENT_DC_MODE, dc_mode, true);
	} else if (!dc_mode && bl_lvl >= threshold) {
		dc_mode = true;
		oplus_panel_event_data_notifier_trigger(panel,
				DRM_PANEL_EVENT_DC_MODE, dc_mode, true);
	}

	oplus_panel_event_data_notifier_trigger(panel,
			DRM_PANEL_EVENT_BACKLIGHT, bl_lvl, true);

	return 0;
}
EXPORT_SYMBOL(oplus_panel_backlight_notifier);

int oplus_panel_set_pinctrl_state(struct dsi_panel *panel, bool enable)
{
	int rc = 0;
	struct pinctrl_state *state;

	if (panel->host_config.ext_bridge_mode)
		return 0;

	if (!panel->pinctrl.pinctrl)
		return 0;

	/* OPLUS_FEATURE_ADFR, for two TE source */
	if (oplus_adfr_is_support() &&
			(oplus_adfr_get_vsync_mode() == OPLUS_DOUBLE_TE_VSYNC) &&
			!strcmp(panel->type, "primary")) {
		if (enable)
			state = panel->pinctrl.te1_active;
		else
			state = panel->pinctrl.te1_suspend;

		rc = pinctrl_select_state(panel->pinctrl.pinctrl, state);
		if (rc)
			LCD_ERR("[%s] failed to set te1 pin state, rc=%d\n",
					panel->oplus_priv.vendor_name, rc);
	}

	/* oplus panel pinctrl */
	if (panel->oplus_priv.pinctrl_enabled) {
		if (enable)
			state = panel->pinctrl.oplus_panel_active;
		else
			state = panel->pinctrl.oplus_panel_suspend;

		rc = pinctrl_select_state(panel->pinctrl.pinctrl, state);
		if (rc)
			LCD_ERR("[%s] failed to set oplus pin state, rc=%d\n",
					panel->oplus_priv.vendor_name, rc);
	}

	return rc;
}

int oplus_panel_pinctrl_init(struct dsi_panel *panel)
{
	int rc = 0, count = 0;
	const char *pinctrl_name;

	if (panel->host_config.ext_bridge_mode)
		return 0;

	panel->pinctrl.pinctrl = devm_pinctrl_get(panel->parent);
	if (IS_ERR_OR_NULL(panel->pinctrl.pinctrl)) {
		rc = PTR_ERR(panel->pinctrl.pinctrl);
		LCD_ERR("failed to get pinctrl, rc=%d\n", rc);
		goto error;
	}

	/* OPLUS_FEATURE_ADFR, for two TE source */
	if (oplus_adfr_is_support() &&
			(oplus_adfr_get_vsync_mode() == OPLUS_DOUBLE_TE_VSYNC) &&
			!strcmp(panel->type, "primary")) {
		panel->pinctrl.te1_active =
				pinctrl_lookup_state(panel->pinctrl.pinctrl, "te1_active");
		if (IS_ERR_OR_NULL(panel->pinctrl.te1_active)) {
			rc = PTR_ERR(panel->pinctrl.te1_active);
			LCD_ERR("[%s] failed to get pinctrl: te1_active, rc=%d\n",
					panel->oplus_priv.vendor_name, rc);
			goto error;
		}

		panel->pinctrl.te1_suspend =
				pinctrl_lookup_state(panel->pinctrl.pinctrl, "te1_suspend");
		if (IS_ERR_OR_NULL(panel->pinctrl.te1_suspend)) {
			rc = PTR_ERR(panel->pinctrl.te1_suspend);
			LCD_ERR("[%s] failed to get pinctrl: te1_suspend, rc=%d\n",
					panel->oplus_priv.vendor_name, rc);
			goto error;
		}

		LCD_INFO("kVRR: [%s] successfully init te1 pinctrl, rc=%d\n",
				panel->oplus_priv.vendor_name, rc);
	}

	/* oplus panel pinctrl */
	count = of_property_count_strings(panel->panel_of_node,
			"oplus,dsi-pinctrl-names");
	if (OPLUS_PINCTRL_NAMES_COUNT == count) {
		of_property_read_string_index(panel->panel_of_node,
				"oplus,dsi-pinctrl-names", 0, &pinctrl_name);
		panel->pinctrl.oplus_panel_active =
				pinctrl_lookup_state(panel->pinctrl.pinctrl, pinctrl_name);
		if (IS_ERR_OR_NULL(panel->pinctrl.oplus_panel_active)) {
			rc = PTR_ERR(panel->pinctrl.oplus_panel_active);
			LCD_ERR("[%s] failed to get pinctrl: %s, rc=%d\n",
					panel->oplus_priv.vendor_name, pinctrl_name, rc);
			goto error;
		}

		of_property_read_string_index(panel->panel_of_node,
				"oplus,dsi-pinctrl-names", 1, &pinctrl_name);
		panel->pinctrl.oplus_panel_suspend =
				pinctrl_lookup_state(panel->pinctrl.pinctrl, pinctrl_name);
		if (IS_ERR_OR_NULL(panel->pinctrl.oplus_panel_suspend)) {
			rc = PTR_ERR(panel->pinctrl.oplus_panel_suspend);
			LCD_ERR("[%s] failed to get pinctrl: %s, rc=%d\n",
					panel->oplus_priv.vendor_name, pinctrl_name, rc);
			goto error;
		}

		panel->oplus_priv.pinctrl_enabled = true;
		LCD_INFO("[%s] successfully init oplus panel pinctrl, rc=%d\n",
				panel->oplus_priv.vendor_name, rc);
	} else if (count >= 0) {
		LCD_ERR("[%s] invalid oplus,dsi-pinctrl-names, count=%d\n",
				panel->oplus_priv.vendor_name, count);
	}

error:
	return rc;
}

