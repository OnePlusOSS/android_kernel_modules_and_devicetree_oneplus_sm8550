/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
**
** File : oplus_display_panel_feature.c
** Description : oplus display panel char dev  /dev/oplus_panel
** Version : 1.0
** Date : 2021/11/17
** Author : Display
******************************************************************/
#ifndef _OPLUS_DISPLAY_PANEL_FEATURE_H_
#define _OPLUS_DISPLAY_PANEL_FEATURE_H_

/**
 * oplus_panel_features - list of oplus panel features
 */
enum oplus_panel_features {
	/* Append new panle features before OPLUS_PANEL_MAX_FEATURES */
	/* Oplus Features start */
	OPLUS_PANLE_DP_SUPPORT,
	OPLUS_PANLE_CABC_SUPPORT,
	OPLUS_PANLE_SERIAL_NUM_SUPPORT,
	OPLUS_PANLE_DC_BACKLIGHT_SUPPORT,
	/* Oplus Features end */
	OPLUS_PANEL_MAX_FEATURES,
};

int oplus_panel_features_config(struct dsi_panel *panel);
int oplus_panel_post_on_backlight(void *display, struct dsi_panel *panel, u32 bl_lvl);
void oplus_panel_update_backlight(struct dsi_panel *panel,
		struct mipi_dsi_device *dsi, u32 bl_lvl);
u32 oplus_panel_silence_backlight(struct dsi_panel *panel, u32 bl_lvl);
void oplus_panel_backlight_notifier(struct dsi_panel *panel, u32 bl_lvl);

#endif /* _OPLUS_DISPLAY_PANEL_FEATURE_H_ */

