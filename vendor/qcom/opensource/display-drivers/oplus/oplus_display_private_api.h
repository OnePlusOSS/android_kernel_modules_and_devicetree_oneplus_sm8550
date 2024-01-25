/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
**
** File : oplus_display_private_api.h
** Description : oplus display private api implement
** Version : 1.0
** Date : 2022/08/01
** Author : Display
******************************************************************/
#ifndef _OPLUS_DISPLAY_PRIVATE_API_H_
#define _OPLUS_DISPLAY_PRIVATE_API_H_

#include <linux/err.h>
#include <linux/list.h>
#include <linux/of.h>
#include "msm_drv.h"
#include "sde_connector.h"
#include "sde_crtc.h"
#include "sde_hw_dspp.h"
#include "sde_plane.h"
#include "msm_mmu.h"
#include "dsi_display.h"
#include "dsi_panel.h"
#include "dsi_ctrl.h"
#include "dsi_ctrl_hw.h"
#include "dsi_drm.h"
#include "dsi_clk.h"
#include "dsi_pwr.h"
#include "sde_dbg.h"
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <drm/drm_mipi_dsi.h>
#include "oplus_dsi_support.h"
#include "oplus_display_esd.h"

#define DISPLAY_TOOL_CMD_KEYWORD "[display:sh]"

extern u32 oplus_last_backlight;

int oplus_display_set_vendor(struct dsi_display *display);

int oplus_display_panel_update_spr_mode(void);

void oplus_panel_process_dimming_v2_post(struct dsi_panel *panel, bool force_disable);

int oplus_panel_process_dimming_v2(struct dsi_panel *panel, int bl_lvl, bool force_disable);

int oplus_panel_process_dimming_v3(struct dsi_panel *panel, int brightness);

int oplus_interpolate(int x, int xa, int xb, int ya, int yb);

int oplus_display_set_power(struct drm_connector *connector, int power_mode, void *disp);

bool oplus_is_support_panel_dither(const char *panel_name);

int dsi_panel_read_panel_reg_unlock(struct dsi_display_ctrl *ctrl,
		struct dsi_panel *panel, u8 cmd, void *rbuf,  size_t len);

int dsi_display_read_panel_reg(struct dsi_display *display, u8 cmd, void *data, size_t len);

#endif /* _OPLUS_DISPLAY_PRIVATE_API_H_ */

