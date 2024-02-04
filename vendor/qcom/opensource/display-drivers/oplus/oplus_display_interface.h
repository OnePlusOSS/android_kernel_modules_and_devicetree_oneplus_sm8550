/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
**
** File : oplus_display_interface.c
** Description : oplus display interface
** Version : 1.0
** Date : 2022/05/30
** Author : Display
******************************************************************/

#ifndef __OPLUS_DISPLAY_INTERFACE_H__
#define __OPLUS_DISPLAY_INTERFACE_H__
#include "dsi_ctrl.h"
#include <linux/soc/qcom/panel_event_notifier.h>
#include "dsi_panel.h"
#include "dsi_defs.h"
#include "oplus_dsi_support.h"
#include "oplus_display_private_api.h"

#define OPLUS_PINCTRL_NAMES_COUNT 2
#define OPLUS_BACKLIGHT_WINDOW_SIZE 5

/**
 * oplus_panel_cmd_print() - oplus panel command printf
 * @panel: Display panel
 * @type:  Command type
 * Return: Zero on Success
 */
int oplus_panel_cmd_print(struct dsi_panel *panel, enum dsi_cmd_set_type type);

/**
 * oplus_panel_cmd_switch() - oplus panel command switch
 * @panel: Display panel
 * @type:  Pointer of command type
 * Return: Zero on Success
 */
int oplus_panel_cmd_switch(struct dsi_panel *panel, enum dsi_cmd_set_type *type);

/**
 * oplus_ctrl_print_cmd_desc() - oplus command desc printf
 * @dsi_ctrl: Dsi ctrl
 * @cmd: Dsi command set
 * Return: void
 */
void oplus_ctrl_print_cmd_desc(struct dsi_ctrl *dsi_ctrl, struct dsi_cmd_desc *cmd);

/**
 * oplus_panel_event_data_notifier_trigger() - oplus panel event notification with data
 * @panel:         Display panel
 * @notif_type:    Type of notifier
 * @data:          Data to be notified
 * @early_trigger: Whether support early trigger
 * Return: Zero on Success
 */
int oplus_panel_event_data_notifier_trigger(struct dsi_panel *panel,
		enum panel_event_notification_type notif_type,
		u32 data,
		bool early_trigger);

/**
 * oplus_event_data_notifier_trigger() - oplus event notification with data
 * @notif_type:    Type of notifier
 * @data:          Data to be notified
 * @early_trigger: Whether support early trigger
 * Return: Zero on Success
 */
int oplus_event_data_notifier_trigger(
		enum panel_event_notification_type notif_type,
		u32 data,
		bool early_trigger);

/**
 * oplus_panel_backlight_notifier() - oplus panel backlight notifier
 * @panel:  Display panel
 * @bl_lvl: Backlight level
 * Return: Zero on Success
 */
int oplus_panel_backlight_notifier(struct dsi_panel *panel, u32 bl_lvl);

/**
 * oplus_panel_init() - oplus panel init
 * @panel: Display panel
 * Return: Zero on Success
 */
int oplus_panel_init(struct dsi_panel *panel);

/**
 * oplus_panel_gpio_request() - oplus panel config gpio request
 * @panel: Display panel
 * Return: Zero on Success
 */
int oplus_panel_gpio_request(struct dsi_panel *panel);

/**
 * oplus_panel_gpio_release() - oplus panel config gpio release
 * @panel: Display panel
 * Return: Zero on Success
 */
int oplus_panel_gpio_release(struct dsi_panel *panel);

/**
 * oplus_panel_gpio_pre_on() - oplus panel config gpio pre on
 * @panel: Display panel
 * Return: Zero on Success
 */
int oplus_panel_gpio_pre_on(struct dsi_panel *panel);

/**
 * oplus_panel_gpio_on() - oplus panel config gpio on
 * @panel: Display panel
 * Return: Zero on Success
 */
int oplus_panel_gpio_on(struct dsi_panel *panel);

/**
 * oplus_panel_gpio_off() - oplus panel config gpio off
 * @panel: Display panel
 * Return: Zero on Success
 */
int oplus_panel_gpio_off(struct dsi_panel *panel);

/**
 * oplus_panel_gpio_parse() - oplus panel config gpio parse
 * @panel: Display panel
 * Return: Zero on Success
 */
int oplus_panel_gpio_parse(struct dsi_panel *panel);

/**
 * oplus_panel_parse_vsync_config() - oplus panel parse vsync config
 * @mode:  Display mode
 * @utils: Parser utils
 * Return: Zero on Success
 */
int oplus_panel_parse_vsync_config(
				struct dsi_display_mode *mode,
				struct dsi_parser_utils *utils);

/**
 * oplus_mult_frac() - oplus display backlight mult brightness
 * @bright: Display config brightness
 * Return: Int level
 */
int oplus_panel_mult_frac(int bright);

/**
 * oplus_panel_set_pinctrl_state() - oplus panel set pinctrl state
 * @panel:  Display panel
 * @enable: Pinctrl state
 * Return: Zero on Success
 */
int oplus_panel_set_pinctrl_state(struct dsi_panel *panel, bool enable);

/**
 * oplus_panel_pinctrl_init() - oplus panel pinctrl init
 * @panel: Display panel
 * Return: Zero on Success
 */
int oplus_panel_pinctrl_init(struct dsi_panel *panel);

/**
 * oplus_vddr_power_on_after_vddio() - oplus panel power on vddr as vddio vddr vci
 * @panel: Display panel
 * Return: Zero on Success
 */
int oplus_panel_vddr_on(struct dsi_display *display, const char *vreg_name);

/**
 * oplus_vddr_power_on_after_vddio() - oplus panel power off vddr as vci vddr vddio
 * @panel: Display panel
 * Return: Zero on Success
 */
int oplus_panel_vddr_off(struct dsi_display *display, const char *vreg_name);

void oplus_sde_cp_crtc_apply_properties(struct drm_crtc *crtc, struct drm_encoder *encoder);
int oplus_set_osc_status(struct drm_encoder *drm_enc);

/**
 * oplus_display_send_dcs_lock() - send dcs with lock
 */
int oplus_display_send_dcs_lock(struct dsi_display *display,
        enum dsi_cmd_set_type type);

int oplus_panel_cmdq_pack_handle(void *dsi_panel, enum dsi_cmd_set_type type, bool before_cmd);
int oplus_panel_cmdq_pack_status_reset(void *sde_connector);
int oplus_panel_get_id(struct dsi_display *display, char *boot_str);
int oplus_panel_pwm_switch_cmdq_delay_handle(void *dsi_panel, enum dsi_cmd_set_type type);
#endif /* __OPLUS_DISPLAY_INTERFACE_H__ */

