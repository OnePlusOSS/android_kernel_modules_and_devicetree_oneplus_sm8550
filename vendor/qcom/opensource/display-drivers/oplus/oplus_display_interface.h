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
#include "oplus_display_private_api.h"

#define OPLUS_BACKLIGHT_WINDOW_SIZE 5

/**
 * oplus_print_cmd_desc() - display cmd printf
 * @display:       display dsi cmd printf
 * @dsi_ctrl:      dsi ctrl
 * @mipi_dsi_msg:  mipi dsi msg
 * Return: void
 */
void oplus_print_cmd_desc(struct dsi_ctrl *dsi_ctrl, const struct mipi_dsi_msg *msg);

/**
 * oplus_panel_event_notification_trigger() - display notifi
 * @display:       dsi_display to be compared
 * @panel_event_notification_type:       notif_type to be compared
 * Return: Zero on Success
 */
int oplus_panel_event_notification_trigger(struct dsi_display *display, enum panel_event_notification_type notif_type);

/**
 * oplus_display_event_data_notifier_trigger() - oplus event notification with data
 * @display:                       Point to dsi_display
 * @panel_event_notifier_tag:      Type of panel
 * @panel_event_notification_type: Type of notifier
 * @data:                          Data to be notified
 * Return: Zero on Success
 */
int oplus_display_event_data_notifier_trigger(struct dsi_display *display,
		enum panel_event_notifier_tag panel_type,
		enum panel_event_notification_type notif_type,
		u32 data);

/**
 * oplus_panel_init() - oplus panel init
 * @dsi_panel:  Display panel
 * Return: Zero on Success
 */
int oplus_panel_init(struct dsi_panel *panel);

/**
 * oplus_panel_gpio_request() - oplus panel config gpio request
 * @dsi_panel:  Display panel
 * Return: Zero on Success
 */
int oplus_panel_gpio_request(struct dsi_panel *panel);

/**
 * oplus_panel_gpio_release() - oplus panel config gpio release
 * @dsi_panel:  Display panel
 * Return: Zero on Success
 */
int oplus_panel_gpio_release(struct dsi_panel *panel);

/**
 * oplus_panel_gpio_pre_on() - oplus panel config gpio pre on
 * @dsi_panel:  Display panel
 * Return: Zero on Success
 */
int oplus_panel_gpio_pre_on(struct dsi_panel *panel);

/**
 * oplus_panel_gpio_on() - oplus panel config gpio on
 * @dsi_panel:  Display panel
 * Return: Zero on Success
 */
int oplus_panel_gpio_on(struct dsi_panel *panel);

/**
 * oplus_panel_gpio_off() - oplus panel config gpio off
 * @dsi_panel:  Display panel
 * Return: Zero on Success
 */
int oplus_panel_gpio_off(struct dsi_panel *panel);

/**
 * oplus_panel_gpio_parse() - oplus panel config gpio parse
 * @dsi_panel:  Display panel
 * Return: Zero on Success
 */
int oplus_panel_gpio_parse(struct dsi_panel *panel);

/**
 * oplus_panel_gpio_parse() - oplus panel config gpio parse
 * @dsi_panel:  Display panel
 * Return: Zero on Success
 */
int oplus_tx_cmd_print(enum dsi_cmd_set_type type);

/**
 * oplus_panel_parse_vsync_config() - oplus panel parse vsync config
 * @dsi_panel:  Display panel
 * Return: Zero on Success
 */
int oplus_panel_parse_vsync_config(
				struct dsi_display_mode *mode,
				struct dsi_parser_utils *utils);

/**
 * oplus_mult_frac() - oplus display backlight mult brightness
 * @int: Display config brightness
 * Return: Int level
 */
int oplus_panel_mult_frac(int bright);

#endif /* __OPLUS_DISPLAY_INTERFACE_H__ */

