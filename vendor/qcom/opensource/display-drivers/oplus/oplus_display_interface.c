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

static void print_cmd_desc(struct dsi_ctrl *dsi_ctrl, const struct mipi_dsi_msg *msg)
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
	pr_err("kVRR: dsi %s\n", buf);
}

void oplus_print_cmd_desc(struct dsi_ctrl *dsi_ctrl, const struct mipi_dsi_msg *msg)
{
	if (OPLUS_DEBUG_LOG_CMD & oplus_dsi_log_type)
		print_cmd_desc(dsi_ctrl, msg);
}

int oplus_panel_init(struct dsi_panel *panel)
{
	int rc = 0;
	static bool panel_need_init = true;

	if (!panel) {
		DSI_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}

	if (panel_need_init) {
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_PANEL_INIT);
		if (rc)
			DSI_ERR("[%s] failed to send DSI_CMD_PANEL_INIT cmds, rc=%d\n",
					panel->name, rc);
		else
			panel_need_init = false;
	}

	return rc;
}

int oplus_panel_gpio_request(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_panel_reset_config *r_config;
	if (!panel) {
		DSI_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}

	r_config = &panel->reset_config;

	if (gpio_is_valid(r_config->panel_vout_gpio)) {
		rc = gpio_request(r_config->panel_vout_gpio, "panel_vout_gpio");
		if (rc) {
			DSI_ERR("request for panel_vout_gpio failed, rc=%d\n", rc);
			if (gpio_is_valid(r_config->panel_vout_gpio))
				gpio_free(r_config->panel_vout_gpio);
		}
	}
	if (gpio_is_valid(r_config->panel_vddr_aod_en_gpio)) {
		rc = gpio_request(r_config->panel_vddr_aod_en_gpio, "panel_vddr_aod_en_gpio");
		if (rc) {
			DSI_ERR("request for panel_vddr_aod_en_gpio failed, rc=%d\n", rc);
			if (gpio_is_valid(r_config->panel_vddr_aod_en_gpio))
				gpio_free(r_config->panel_vddr_aod_en_gpio);
		}
	}
	if (oplus_adfr_is_support()) {
		if (gpio_is_valid(panel->vsync_switch_gpio)) {
			rc = gpio_request(panel->vsync_switch_gpio, "vsync_switch_gpio");
			if (rc) {
				DSI_ERR("adfr request for vsync_switch_gpio failed, rc=%d\n", rc);
			}
		}
	}

	return rc;
}

int oplus_panel_gpio_release(struct dsi_panel *panel)
{
	struct dsi_panel_reset_config *r_config;
	if (!panel) {
		DSI_ERR("Oplus Features config No panel device\n");
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
		DSI_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}

	r_config = &panel->reset_config;

	if (panel->oplus_priv.gpio_pre_enabled &&
			gpio_is_valid(panel->reset_config.panel_vout_gpio)) {
		rc = gpio_direction_output(panel->reset_config.panel_vout_gpio, 1);
		if (rc)
			DSI_ERR("unable to set dir for panel_vout_gpio rc=%d", rc);
		gpio_set_value(panel->reset_config.panel_vout_gpio, 1);
	}

	return rc;
}

int oplus_panel_gpio_on(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_panel_reset_config *r_config;
	if (!panel) {
		DSI_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}

	r_config = &panel->reset_config;

	if (!panel->oplus_priv.gpio_pre_enabled &&
			gpio_is_valid(panel->reset_config.panel_vout_gpio)) {
		rc = gpio_direction_output(panel->reset_config.panel_vout_gpio, 1);
		if (rc)
			DSI_ERR("unable to set dir for panel_vout_gpio rc=%d", rc);
		gpio_set_value(panel->reset_config.panel_vout_gpio, 1);
	}
	if (gpio_is_valid(panel->reset_config.panel_vddr_aod_en_gpio)) {
		rc = gpio_direction_output(panel->reset_config.panel_vddr_aod_en_gpio, 1);
		if (rc)
			DSI_ERR("unable to set dir for panel_vddr_aod_en_gpio rc=%d", rc);
		gpio_set_value(panel->reset_config.panel_vddr_aod_en_gpio, 1);
	}

	return rc;
}

int oplus_panel_gpio_off(struct dsi_panel *panel)
{
	struct dsi_panel_reset_config *r_config;
	if (!panel) {
		DSI_ERR("Oplus Features config No panel device\n");
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
		DSI_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}
	utils = &panel->utils;
	r_config = &panel->reset_config;

	panel->reset_config.panel_vout_gpio = utils->get_named_gpio(utils->data,
								"qcom,platform-panel-vout-gpio", 0);

	if (!gpio_is_valid(panel->reset_config.panel_vout_gpio)) {
		DSI_ERR("[%s] failed get panel_vout_gpio\n", panel->name);
	}
	panel->reset_config.panel_vddr_aod_en_gpio = utils->get_named_gpio(utils->data,
								"qcom,platform-panel-vddr-aod-en-gpio", 0);

	if (!gpio_is_valid(panel->reset_config.panel_vddr_aod_en_gpio)) {
		DSI_ERR("[%s] failed get panel_vddr_aod_en_gpio\n", panel->name);
	}

	return 0;
}

int oplus_tx_cmd_print(enum dsi_cmd_set_type type)
{
	if (type != DSI_CMD_READ_SAMSUNG_PANEL_REGISTER_ON
	&& type != DSI_CMD_SET_ROI
	&& type != DSI_CMD_READ_SAMSUNG_PANEL_REGISTER_OFF
	&& type != DSI_CMD_FAKEFRAME
	&& type != DSI_CMD_QSYNC_MIN_FPS_0
	&& type != DSI_CMD_QSYNC_MIN_FPS_1
	&& type != DSI_CMD_ESD_SWITCH_PAGE) {
		pr_err("dsi_cmd %s\n", cmd_set_prop_map[type]);
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
		DSI_DEBUG("panel prefill lines are not defined rc=%d\n", rc);
		priv_info->vsync_period = 1000000 / mode->timing.refresh_rate;
	}

	rc = utils->read_u32(utils->data, "oplus,apollo-panel-vsync-width",
				  &priv_info->vsync_width);
	if (rc) {
		DSI_DEBUG("panel vsync width not defined rc=%d\n", rc);
		priv_info->vsync_width = priv_info->vsync_period >> 1;
	}

	DSI_INFO("vsync width = %d, vsync period = %d\n", priv_info->vsync_width, priv_info->vsync_period);

	return 0;
}

int oplus_panel_mult_frac(int bright)
{
	int bl_lvl = 0;
	struct dsi_display *display = get_main_display();

	if (!display || !display->drm_conn || !display->drm_conn->state) {
		pr_err("failed to find dsi display\n");
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
				bl_lvl = interpolate(bright, lut[i-1].brightness,
						lut[i].brightness, lut[i-1].alpha, lut[i].alpha);
		} else if (bright > display->panel->bl_config.brightness_normal_max_level) {
			bl_lvl = interpolate(bright,
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

int oplus_panel_event_notification_trigger(struct dsi_display *display, enum panel_event_notification_type notif_type)
{
	struct panel_event_notification notification;
	struct drm_panel *panel = dsi_display_get_drm_panel(display);
	enum panel_event_notifier_tag panel_type;

	if (!panel)
		return -ENOLINK;

	panel_type = PANEL_EVENT_NOTIFICATION_PRIMARY;

	memset(&notification, 0, sizeof(notification));

	notification.notif_type = notif_type;
	notification.panel = panel;
	notification.notif_data.early_trigger = true;
	panel_event_notification_trigger(panel_type, &notification);
	return 0;
}

EXPORT_SYMBOL(oplus_panel_event_notification_trigger);

int oplus_display_event_data_notifier_trigger(struct dsi_display *display,
		enum panel_event_notifier_tag panel_type,
		enum panel_event_notification_type notif_type,
		u32 data)
{
	struct drm_panel *panel = dsi_display_get_drm_panel(display);
	struct panel_event_notification notifier;

	LCD_DEBUG_COMMON("Trigger notifier, panel:%d, type:%d, data:%d\n",
			panel_type, notif_type, data);

	if (!panel) {
		pr_err("[%s] invalid panel\n", __func__);
		return -EINVAL;
	}

	memset(&notifier, 0, sizeof(notifier));

	notifier.panel = panel;
	notifier.notif_type = notif_type;
	notifier.notif_data.early_trigger = true;
	notifier.notif_data.data = data;

	panel_event_notification_trigger(panel_type, &notifier);
	return 0;
}
EXPORT_SYMBOL(oplus_display_event_data_notifier_trigger);

