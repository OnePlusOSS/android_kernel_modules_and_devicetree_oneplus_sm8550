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
#include "oplus_display_panel_common.h"
#include "sde_color_processing.h"
#include "sde_encoder_phys.h"
#include "sde_trace.h"

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
#include "oplus_adfr.h"
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

extern int dc_apollo_enable;
extern int oplus_dimlayer_hbm;
extern unsigned int oplus_dsi_log_type;
extern const char *cmd_set_prop_map[];
extern int oplus_debug_max_brightness;
bool is_evt_panel = false;
bool is_dvt_0_panel = false;

int oplus_panel_cmd_print(struct dsi_panel *panel, enum dsi_cmd_set_type type)
{
	switch (type) {
	case DSI_CMD_READ_SAMSUNG_PANEL_REGISTER_ON:
	case DSI_CMD_SET_ROI:
	case DSI_CMD_READ_SAMSUNG_PANEL_REGISTER_OFF:
	case DSI_CMD_ESD_SWITCH_PAGE:
	case DSI_CMD_SKIPFRAME_DBV:
#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	case DSI_CMD_ADFR_MIN_FPS_0:
	case DSI_CMD_ADFR_MIN_FPS_1:
	case DSI_CMD_ADFR_MIN_FPS_2:
	case DSI_CMD_ADFR_MIN_FPS_3:
	case DSI_CMD_ADFR_MIN_FPS_4:
	case DSI_CMD_ADFR_MIN_FPS_5:
	case DSI_CMD_ADFR_MIN_FPS_6:
	case DSI_CMD_ADFR_MIN_FPS_7:
	case DSI_CMD_ADFR_MIN_FPS_8:
	case DSI_CMD_ADFR_MIN_FPS_9:
	case DSI_CMD_ADFR_MIN_FPS_10:
	case DSI_CMD_ADFR_MIN_FPS_11:
	case DSI_CMD_ADFR_MIN_FPS_12:
	case DSI_CMD_ADFR_MIN_FPS_13:
	case DSI_CMD_ADFR_MIN_FPS_14:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_0:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_1:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_2:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_3:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_4:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_5:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_6:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_7:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_8:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_9:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_10:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_11:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_12:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_13:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_14:
	case DSI_CMD_ADFR_FAKEFRAME:
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */
		/* Do nothing */
		break;
	default:
		LCD_INFO("[%s] dsi_cmd: %s\n", panel->oplus_priv.vendor_name,
				cmd_set_prop_map[type]);
		break;
	}

	return 0;
}

int oplus_panel_get_id(struct dsi_display *display, char *boot_str)
{
	char *keyword = ".";
	char *pos = NULL;

	if (!display) {
		LCD_ERR("Oplus display null\n");
		return -ENODEV;
	}

	LCD_INFO("boot_str = %s\n", boot_str);

	if (strstr(boot_str, "mdss_dsi_panel_AA551_P_3_A0004_dsc_cmd")) {
		pos = strstr(boot_str, keyword);
		if (pos != NULL) {
			char hex[10];
			int panel_id;

			sscanf(pos + 1, "%s", hex);
			sscanf(hex, "%x", &panel_id);

			display->panel_id_da = (panel_id & 0xFF0000) >> 16;
			display->panel_id_db = (panel_id & 0x00FF00) >> 8;
			display->panel_id_dc = (panel_id & 0x0000FF) >> 0;
			LCD_INFO("%x %x %x\n", display->panel_id_da, display->panel_id_db, display->panel_id_dc);

			if((0x3F == display->panel_id_da) && (0x04 == display->panel_id_db)
				&& ((0x00 == display->panel_id_dc) || (0xAA == display->panel_id_dc))) {
				is_evt_panel = true;
				LCD_INFO("is_evt_panel true\n");
			}
			if((0x05 == display->panel_id_db) && (0x00 == display->panel_id_dc)) {
				is_dvt_0_panel = true;
				LCD_INFO("is_dvt_panel true\n");
			}
		} else {
			LCD_INFO("fail to parse panel id\n");
		}
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

	/* switch the command when pwm onepulse is enabled */
	if (oplus_panel_pwm_onepulse_is_enabled(panel)) {
		switch (*type) {
		case DSI_CMD_PWM_SWITCH_HIGH:
			*type = DSI_CMD_PWM_SWITCH_ONEPULSE;
			break;
		case DSI_CMD_TIMMING_PWM_SWITCH_HIGH:
			*type = DSI_CMD_TIMMING_PWM_SWITCH_ONEPULSE;
			break;
		case DSI_CMD_HBM_ON:
			*type = DSI_CMD_HBM_ON_ONEPULSE;
		default:
			break;
		}
	}

	if (!strcmp(panel->name, "AA551 P 3 A0004 dsc cmd mode panel")) {
		if (is_evt_panel) {
			switch (*type) {
			case DSI_CMD_SET_ON:
				*type = DSI_CMD_SET_ON_EVT;
				break;
			case DSI_CMD_LOADING_EFFECT_MODE1:
			case DSI_CMD_LOADING_EFFECT_MODE2:
			case DSI_CMD_LOADING_EFFECT_OFF:
				*type = DSI_CMD_DEFAULT_SWITCH_PAGE;
				break;
			default:
				break;
			}
		}
		if (is_dvt_0_panel) {
			switch (*type) {
			case DSI_CMD_SET_ON:
				*type = DSI_CMD_SET_ON_DVT;
				break;
			default:
				break;
			}
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

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	oplus_adfr_gpio_release(panel);
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

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

	if (panel->oplus_priv.gpio_pre_on
		|| !strcmp(panel->name, "AC052 P 1 A0002 dsc cmd mode panel")
		|| !strcmp(panel->name, "AC052 P 3 A0003 dsc cmd mode panel")) {
		/* do nothing */
	} else if (gpio_is_valid(panel->reset_config.panel_vout_gpio)) {
		rc = gpio_direction_output(panel->reset_config.panel_vout_gpio, 1);
		if (rc)
			LCD_ERR("unable to set dir for panel_vout_gpio rc=%d\n", rc);
		gpio_set_value(panel->reset_config.panel_vout_gpio, 1);
		if (!strcmp(panel->name, "AC052 S 3 A0001 dsc cmd mode panel")
			|| !strcmp(panel->name, "AA536 P 3 A0001 dsc cmd mode panel"))
			usleep_range(2*1000, (2*1000)+100);
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

	if (!strcmp(panel->name, "AC052 P 1 A0002 dsc cmd mode panel")
		|| !strcmp(panel->name, "AC052 P 3 A0003 dsc cmd mode panel")
		|| !strcmp(panel->name, "AA536 P 3 A0001 dsc cmd mode panel")
		|| !strcmp(panel->name, "AC052 S 3 A0001 dsc cmd mode panel")
		|| !strcmp(panel->name, "AA551 P 3 A0004 dsc cmd mode panel")) {
		/*do nothing */
	} else if (gpio_is_valid(panel->reset_config.panel_vout_gpio)) {
		gpio_set_value(panel->reset_config.panel_vout_gpio, 0);
	}
	if (gpio_is_valid(panel->reset_config.panel_vddr_aod_en_gpio))
		gpio_set_value(panel->reset_config.panel_vddr_aod_en_gpio, 0);
	if (gpio_is_valid(panel->reset_config.panel_vout_gpio)
		&& !strcmp(panel->name, "AC052 S 3 A0001 dsc cmd mode panel")) {
		usleep_range(1*1000, (1*1000)+100);
		gpio_set_value(panel->reset_config.panel_vout_gpio, 0);
	}
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

	rc = utils->read_u32(utils->data, "oplus,apollo-panel-async-bl-delay",
				  &priv_info->async_bl_delay);
	if (rc) {
		LCD_DEBUG("panel async backlight delay to bottom of frame was disabled rc=%d\n", rc);
		priv_info->async_bl_delay = 0;
	} else {
		if(priv_info->async_bl_delay >= priv_info->vsync_period) {
			LCD_ERR("async backlight delay value was out of vsync period\n");
			priv_info->async_bl_delay = priv_info->vsync_width;
		}
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

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	rc = oplus_adfr_te_source_vsync_switch_set_pinctrl_state(panel, enable);
	if (rc) {
		goto error;
	}
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

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

error:
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

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	rc = oplus_adfr_te_source_vsync_switch_pinctrl_init(panel);
	if (rc) {
		goto error;
	}
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

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

int oplus_panel_vddr_on(struct dsi_display *display, const char *vreg_name)
{
	int rc = 0;

	if (!display || !display->panel) {
		LCD_ERR("display or display panel is null, power vddr failed!\n");
		return -ENODEV;
	}

	if ((!strcmp(display->panel->name, "AC052 P 3 A0003 dsc cmd mode panel")
		|| !strcmp(display->panel->name, "AC052 P 1 A0002 dsc cmd mode panel")
		|| !strcmp(display->panel->name, "AA536 P 3 A0001 dsc cmd mode panel")
		|| !strcmp(display->panel->name, "AA551 P 3 A0004 dsc cmd mode panel"))
		&& !strcmp(vreg_name, "vddio")) {
		if (gpio_is_valid(display->panel->reset_config.panel_vout_gpio)) {
			rc = gpio_direction_output(display->panel->reset_config.panel_vout_gpio, 1);
			if (rc)
				LCD_ERR("unable to set dir for panel_vout_gpio rc=%d\n", rc);
			gpio_set_value(display->panel->reset_config.panel_vout_gpio, 1);
		}
	}

	return rc;
}

int oplus_panel_vddr_off(struct dsi_display *display, const char *vreg_name)
{
	int rc = 0;

	if (!display || !display->panel) {
		LCD_ERR("display or display panel is null, power vddr failed!\n");
		return -ENODEV;
	}

	if ((!strcmp(display->panel->name, "AC052 P 3 A0003 dsc cmd mode panel")
		|| !strcmp(display->panel->name, "AC052 P 1 A0002 dsc cmd mode panel")
		|| !strcmp(display->panel->name, "AA536 P 3 A0001 dsc cmd mode panel")
		|| !strcmp(display->panel->name, "AA551 P 3 A0004 dsc cmd mode panel"))
		&& !strcmp(vreg_name, "vci")) {
		usleep_range(2*1000, (2*1000)+100);
		if (gpio_is_valid(display->panel->reset_config.panel_vout_gpio)) {
			gpio_set_value(display->panel->reset_config.panel_vout_gpio, 0);
		}
	}

	if (display->panel->oplus_priv.oplus_disp_hw_seq_modify_flag && !strcmp(vreg_name, "vci")) {
		usleep_range(2*1000, (2*1000)+100);
		if (gpio_is_valid(display->panel->reset_config.panel_vout_gpio)) {
			gpio_set_value(display->panel->reset_config.panel_vout_gpio, 0);
		}
	}

	return rc;
}


void notify_work_handler(struct kthread_work *work)
{
	struct dsi_panel *panel = container_of(work, struct dsi_panel, work);

	SDE_ATRACE_BEGIN("notify_work_handler");
	panel->notify_done.done = 0;
	panel->need_to_wait_notify_done = 1;
	panel_event_notification_trigger(panel->panel_event,
					&panel->notification);
	panel->need_to_wait_notify_done = 0;
	complete(&panel->notify_done);
	SDE_ATRACE_END("notify_work_handler");
}

void oplus_panel_event_notification_trigger(enum panel_event_notifier_tag panel_event,
	struct panel_event_notification *notification)
{
	struct dsi_panel *panel = container_of(notification->panel, struct dsi_panel, drm_panel);

	if (IS_ERR_OR_NULL(panel->notify_worker)) {
		panel->notify_worker = kthread_create_worker(0, "notify_worker");
		kthread_init_work(&panel->work, notify_work_handler);
		init_completion(&panel->notify_done);
	}

	if (!IS_ERR_OR_NULL(panel->notify_worker)) {
		/* only power off use kthread */
		if (notification->notif_type == DRM_PANEL_EVENT_BLANK) {
			panel->panel_event = panel_event;
			panel->notification = *notification;
			kthread_queue_work(panel->notify_worker, &panel->work);
		} else {
			panel_event_notification_trigger(panel_event, notification);
		}
	} else {
		panel_event_notification_trigger(panel_event, notification);
	}
}

void oplus_wait_for_notify_done(struct dsi_display *display)
{
	char tag_name[32];

	if (!display || !display->panel)
		return;

	snprintf(tag_name, sizeof(tag_name), "need_to_wait_notify_done %d", display->panel->need_to_wait_notify_done);
	SDE_ATRACE_BEGIN(tag_name);
	if (1 == display->panel->need_to_wait_notify_done)
		wait_for_completion(&display->panel->notify_done);
	SDE_ATRACE_END(tag_name);
}

void oplus_sde_cp_crtc_apply_properties(struct drm_crtc *crtc, struct drm_encoder *encoder)
{
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct sde_encoder_virt *sde_enc = NULL;
	char tag_name[64];
	list_for_each_entry(encoder, &crtc->dev->mode_config.encoder_list, head) {
		if (encoder->crtc != crtc)
			continue;

		sde_enc = to_sde_encoder_virt(encoder);
	}

	if (!sde_enc || !sde_enc->cur_master || !sde_enc->cur_master->connector) {
		LCD_DEBUG("Invalid sde_enc params\n");
		goto apply_properties;
	}

	c_conn = to_sde_connector(sde_enc->cur_master->connector);
	if (!c_conn) {
		LCD_DEBUG("Invalid c_conn params\n");
		goto apply_properties;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		LCD_DEBUG("No in dsi mode\n");
		goto apply_properties;
	}

	display = c_conn->display;
	if (!display) {
		LCD_DEBUG("Invalid display params\n");
		goto apply_properties;
	}

	if (display) {
		snprintf(tag_name, sizeof(tag_name), "dspp_lock_%s", display->display_type);
		SDE_ATRACE_BEGIN(tag_name);
		mutex_lock(&display->dspp_lock);
	}
apply_properties:
	sde_cp_crtc_apply_properties(crtc);

	if (display) {
		mutex_unlock(&display->dspp_lock);
		SDE_ATRACE_END(tag_name);
	}
}

#define to_sde_encoder_phys_cmd(x) \
	container_of(x, struct sde_encoder_phys_cmd, base)

int oplus_set_osc_status(struct drm_encoder *drm_enc) {
	struct sde_encoder_virt *sde_enc = NULL;
	struct sde_encoder_phys *phys_encoder = NULL;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct sde_connector_state *c_state;
	int rc = 0;
	struct sde_encoder_phys_cmd *cmd_enc = NULL;
	bool sync_osc;
	u32 osc_status;

	sde_enc = to_sde_encoder_virt(drm_enc);
	phys_encoder = sde_enc->phys_encs[0];

	if (phys_encoder == NULL)
		return -EFAULT;
	if (phys_encoder->connector == NULL)
		return -EFAULT;

	c_conn = to_sde_connector(phys_encoder->connector);
	if (c_conn == NULL)
		return -EFAULT;

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI)
		return 0;

	c_state = to_sde_connector_state(c_conn->base.state);

	display = c_conn->display;
	if (display == NULL)
		return -EFAULT;

	cmd_enc = to_sde_encoder_phys_cmd(phys_encoder);
	if (cmd_enc == NULL) {
		return -EFAULT;
	}

	sync_osc = c_conn->osc_need_update;
	c_conn->osc_need_update = false;

	if (sync_osc) {
		char tag_name[32];
		osc_status = sde_connector_get_property(c_conn->base.state, CONNECTOR_PROP_SET_OSC_STATUS);
		snprintf(tag_name, sizeof(tag_name), "sync_osc_status:%d", osc_status);
		SDE_ATRACE_BEGIN(tag_name);
		LCD_INFO("osc_status = %d", osc_status);
		oplus_display_panel_set_osc_track(osc_status);
		SDE_ATRACE_END(tag_name);
	}

	return rc;
}

int oplus_display_send_dcs_lock(struct dsi_display *display,
		enum dsi_cmd_set_type type)
{
	int rc = 0;

	if (!display || !display->panel) {
		LCD_ERR("invalid display panel\n");
		return -ENODEV;
	}

	if (display->panel->power_mode == SDE_MODE_DPMS_OFF) {
		LCD_WARN("display panel is in off status\n");
		return -EINVAL;
	}

	if (type < DSI_CMD_SET_MAX) {
		mutex_lock(&display->display_lock);
		/* enable the clk vote for CMD mode panels */
		if (display->config.panel_mode == DSI_OP_CMD_MODE) {
			rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_ON);
			if (rc) {
				LCD_ERR("failed to enable DSI clocks, rc=%d\n", rc);
				mutex_unlock(&display->display_lock);
				return -EFAULT;
			}
		}

		mutex_lock(&display->panel->panel_lock);
		rc = dsi_panel_tx_cmd_set(display->panel, type);
		mutex_unlock(&display->panel->panel_lock);

		/* disable the clk vote for CMD mode panels */
		if (display->config.panel_mode == DSI_OP_CMD_MODE) {
			rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_OFF);
			if (rc) {
				LCD_ERR("failed to disable DSI clocks, rc=%d\n", rc);
			}
		}
		mutex_unlock(&display->display_lock);
	} else {
		LCD_ERR("dcs[%d] is out of range", type);
		return -EINVAL;
	}

	return rc;
}

int oplus_panel_cmdq_pack_handle(void *dsi_panel, enum dsi_cmd_set_type type, bool before_cmd)
{
	struct dsi_panel *panel = dsi_panel;
	struct dsi_display_mode *mode;

	LCD_DEBUG("start\n");

	if (!panel || !panel->cur_mode) {
		LCD_ERR("invalid panel param\n");
		return -EINVAL;
	}
	mode = panel->cur_mode;

	if(!panel->oplus_priv.cmdq_pack_support || !mode->priv_info->cmd_sets[type].pack) {
		return 0;
	}

	OPLUS_LCD_TRACE_BEGIN("oplus_panel_cmdq_pack_handle");
	OPLUS_LCD_TRACE_INT("oplus_dsi_cmd_set_type", type);
	OPLUS_LCD_TRACE_INT("before_cmd", before_cmd);

	if (before_cmd) {
		if (panel->oplus_priv.cmdq_pack_state) {
			LCD_INFO("[%s] dsi_cmd: %s block to the next frame\n",
					panel->oplus_priv.vendor_name,
					cmd_set_prop_map[type]);
			oplus_sde_early_wakeup(panel);
			oplus_wait_for_vsync(panel);
			if (panel->cur_mode->timing.refresh_rate == 60) {
				oplus_need_to_sync_te(panel);
			}
		}
	} else {
		panel->oplus_priv.cmdq_pack_state = true;
	}

	OPLUS_LCD_TRACE_END("oplus_panel_cmdq_pack_handle");
	LCD_DEBUG("end\n");

	return 0;
}

int oplus_panel_cmdq_pack_status_reset(void *sde_connector)
{
	struct sde_connector *c_conn = sde_connector;
	struct dsi_display *display = NULL;

	LCD_DEBUG("start\n");

	if (!c_conn) {
		LCD_ERR("invalid c_conn param\n");
		return -EINVAL;
	}

	display = c_conn->display;
	if (!display || !display->panel) {
		LCD_DEBUG("invalid display params\n");
		return -EINVAL;
	}

	if(!display->panel->oplus_priv.cmdq_pack_support) {
		return 0;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		LCD_WARN("not in dsi mode, should not reset cmdq pack status\n");
		return 0;
	}

	display->panel->oplus_priv.cmdq_pack_state = false;

	LCD_DEBUG("cmdq pack state is %d\n", display->panel->oplus_priv.cmdq_pack_state);
	SDE_ATRACE_INT("oplus_panel_cmdq_pack_status_reset",
			display->panel->oplus_priv.cmdq_pack_state);

	LCD_DEBUG("end\n");

	return 0;
}

