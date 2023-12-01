/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
**
** File : oplus_dc_diming.c
** Description : oplus dc_diming feature
** Version : 1.0
** Date : 2020/04/15
** Author : Display
******************************************************************/

#include "oplus_display_private_api.h"
#include "oplus_dc_diming.h"
#include "dsi_defs.h"
#include "sde_trace.h"
#include "oplus_display_panel.h"
#include "oplus/oplus_bl.h"


int oplus_dimlayer_bl = 0;
EXPORT_SYMBOL(oplus_dimlayer_bl);
int oplus_dimlayer_bl_enabled = 0;
EXPORT_SYMBOL(oplus_dimlayer_bl_enabled);
int oplus_datadimming_v3_skip_frame = 2;
int oplus_panel_alpha = 0;
int oplus_underbrightness_alpha = 0;
EXPORT_SYMBOL(oplus_underbrightness_alpha);
int dc_apollo_enable = 0;
EXPORT_SYMBOL(dc_apollo_enable);

static struct dsi_panel_cmd_set oplus_priv_seed_cmd_set;

/* DC setting for onscreenfinger */
extern int oplus_dimlayer_bl_enable;
extern int oplus_dimlayer_bl_alpha_value;
extern int oplus_dimlayer_bl;
extern ktime_t oplus_backlight_time;
extern u32 oplus_backlight_delta;
extern bool oplus_ffl_trigger_finish;
extern int oplus_dimlayer_bl_on_vblank;
extern int oplus_dimlayer_bl_off_vblank;
extern int oplus_dimlayer_bl_delay;
extern int oplus_dimlayer_bl_delay_after;
extern int oplus_dimlayer_bl_enable_v2;
extern int oplus_dimlayer_bl_enable_v2_real;
extern int oplus_dimlayer_bl_enable_v3;
extern int oplus_dimlayer_bl_enable_v3_real;
extern int oplus_datadimming_vblank_count;
extern atomic_t oplus_datadimming_vblank_ref;
extern bool oplus_skip_datadimming_sync;
extern int oplus_dc2_alpha;
extern int oplus_seed_backlight;
extern int seed_mode;
extern int dsi_panel_seed_mode(struct dsi_panel *panel, int mode);
extern struct oplus_apollo_backlight_list *p_apollo_backlight;

static struct oplus_brightness_alpha brightness_seed_alpha_lut_dc[] = {
	{0, 0xff},
	{1, 0xfc},
	{2, 0xfb},
	{3, 0xfa},
	{4, 0xf9},
	{5, 0xf8},
	{6, 0xf7},
	{8, 0xf6},
	{10, 0xf4},
	{15, 0xf0},
	{20, 0xea},
	{30, 0xe0},
	{45, 0xd0},
	{70, 0xbc},
	{100, 0x98},
	{120, 0x80},
	{140, 0x70},
	{160, 0x58},
	{180, 0x48},
	{200, 0x30},
	{220, 0x20},
	{240, 0x10},
	{260, 0x00},
};

static struct oplus_brightness_alpha brightness_alpha_lut_dc[] = {
	{0, 0xff},
	{1, 0xE0},
	{2, 0xd1},
	{3, 0xd0},
	{4, 0xcf},
	{5, 0xc9},
	{6, 0xc7},
	{8, 0xbe},
	{10, 0xb6},
	{15, 0xaa},
	{20, 0x9c},
	{30, 0x92},
	{45, 0x7c},
	{70, 0x5c},
	{100, 0x40},
	{120, 0x2c},
	{140, 0x20},
	{160, 0x1c},
	{180, 0x16},
	{200, 0x8},
	{223, 0x0},
};

int oplus_panel_parse_dc_config(struct dsi_panel *panel)
{
	int rc = 0;
	int i;
	u32 length = 0;
	u32 count = 0;
	u32 size = 0;
	u32 *arr_32 = NULL;
	const u32 *arr;
	struct dsi_parser_utils *utils = &panel->utils;
	struct oplus_brightness_alpha *seq;

	arr = utils->get_property(utils->data, "oplus,dsi-dc-brightness", &length);
	if (!arr) {
		LCD_ERR("[%s] oplus,dsi-dc-brightness  not found\n", panel->name);
		return -EINVAL;
	}

	if (length & 0x1) {
		LCD_ERR("[%s] oplus,dsi-dc-brightness length error\n", panel->name);
		return -EINVAL;
	}

	LCD_DEBUG("RESET SEQ LENGTH = %d\n", length);
	length = length / sizeof(u32);
	size = length * sizeof(u32);

	arr_32 = kzalloc(size, GFP_KERNEL);
	if (!arr_32) {
		rc = -ENOMEM;
		goto error;
	}

	rc = utils->read_u32_array(utils->data, "oplus,dsi-dc-brightness",
					arr_32, length);
	if (rc) {
		LCD_ERR("[%s] cannot read dsi-dc-brightness\n", panel->name);
		goto error_free_arr_32;
	}

	count = length / 2;
	size = count * sizeof(*seq);
	seq = kzalloc(size, GFP_KERNEL);
	if (!seq) {
		rc = -ENOMEM;
		goto error_free_arr_32;
	}

	panel->dc_ba_seq = seq;
	panel->dc_ba_count = count;

	for (i = 0; i < length; i += 2) {
		seq->brightness = arr_32[i];
		seq->alpha = arr_32[i + 1];
		seq++;
	}

error_free_arr_32:
	kfree(arr_32);
error:
	return rc;
}

int sde_connector_update_backlight(struct drm_connector *connector, bool post)
{
	struct sde_connector *c_conn = to_sde_connector(connector);
	struct dsi_display *dsi_display;

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		return 0;
	}

	dsi_display = c_conn->display;

	if (!dsi_display || !dsi_display->panel || !dsi_display->panel->cur_mode) {
		LCD_ERR("Invalid params(s) dsi_display %pK, panel %pK\n",
			  dsi_display,
			  ((dsi_display) ? dsi_display->panel : NULL));
		return -EINVAL;
	}

	if (!connector->state || !connector->state->crtc) {
		return 0;
	}

	if ((!strcmp(dsi_display->panel->oplus_priv.vendor_name, "S6E3HC3")) ||
			(!strcmp(dsi_display->panel->oplus_priv.vendor_name, "S6E3HC4")) ||
			(!strcmp(dsi_display->panel->oplus_priv.vendor_name, "AMB670YF01"))) {
		return 0;
	}

	if (oplus_dimlayer_bl != oplus_dimlayer_bl_enabled) {
		struct sde_connector *c_conn = to_sde_connector(connector);
		struct drm_crtc *crtc = connector->state->crtc;
		struct dsi_panel *panel = dsi_display->panel;
		int bl_lvl = dsi_display->panel->bl_config.bl_level;
		u32 current_vblank;
		int on_vblank = 0;
		int off_vblank = 0;
		int vblank = 0;
		int ret = 0;
		int vblank_get = -EINVAL;
		int on_delay = 0, on_delay_after = 0;
		int off_delay = 0, off_delay_after = 0;
		int delay = 0, delay_after = 0;

		if (panel->cur_mode->timing.refresh_rate == 120) {
			if (bl_lvl < 103) {
				on_vblank = 0;
				off_vblank = 2;

			} else {
				on_vblank = 0;
				off_vblank = 0;
			}

		} else {
			if (bl_lvl < 103) {
				on_vblank = -1;
				off_vblank = 1;
				on_delay = 11000;

			} else {
				on_vblank = -1;
				off_vblank = -1;
				on_delay = 11000;
				off_delay = 11000;
			}
		}

		if (oplus_dimlayer_bl_on_vblank != INT_MAX) {
			on_vblank = oplus_dimlayer_bl_on_vblank;
		}

		if (oplus_dimlayer_bl_off_vblank != INT_MAX) {
			off_vblank = oplus_dimlayer_bl_off_vblank;
		}


		if (oplus_dimlayer_bl) {
			vblank = on_vblank;
			delay = on_delay;
			delay_after = on_delay_after;

		} else {
			vblank = off_vblank;
			delay = off_delay;
			delay_after = off_delay_after;
		}

		if (oplus_dimlayer_bl_delay >= 0) {
			delay = oplus_dimlayer_bl_delay;
		}

		if (oplus_dimlayer_bl_delay_after >= 0) {
			delay_after = oplus_dimlayer_bl_delay_after;
		}

		vblank_get = drm_crtc_vblank_get(crtc);

		if (vblank >= 0) {
			if (!post) {
				oplus_dimlayer_bl_enabled = oplus_dimlayer_bl;
				current_vblank = drm_crtc_vblank_count(crtc);
				ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
							 current_vblank != drm_crtc_vblank_count(crtc),
							 msecs_to_jiffies(34));
				current_vblank = drm_crtc_vblank_count(crtc) + vblank;

				if (delay > 0) {
					usleep_range(delay, delay + 100);
				}

				_sde_connector_update_bl_scale_(c_conn);

				if (delay_after) {
					usleep_range(delay_after, delay_after + 100);
				}

				if (vblank > 0) {
					ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
								 current_vblank == drm_crtc_vblank_count(crtc),
								 msecs_to_jiffies(17 * 3));
				}
			}

		} else {
			if (!post) {
				current_vblank = drm_crtc_vblank_count(crtc);
				ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
							 current_vblank != drm_crtc_vblank_count(crtc),
							 msecs_to_jiffies(34));

			} else {
				if (vblank < -1) {
					current_vblank = drm_crtc_vblank_count(crtc) + 1 - vblank;
					ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
								 current_vblank == drm_crtc_vblank_count(crtc),
								 msecs_to_jiffies(17 * 3));
				}

				oplus_dimlayer_bl_enabled = oplus_dimlayer_bl;

				if (delay > 0) {
					usleep_range(delay, delay + 100);
				}

				_sde_connector_update_bl_scale_(c_conn);

				if (delay_after) {
					usleep_range(delay_after, delay_after + 100);
				}
			}
		}

		if (!vblank_get) {
			drm_crtc_vblank_put(crtc);
		}
	}

	if (oplus_dimlayer_bl_enable_v2 != oplus_dimlayer_bl_enable_v2_real) {
		struct sde_connector *c_conn = to_sde_connector(connector);

		oplus_dimlayer_bl_enable_v2_real = oplus_dimlayer_bl_enable_v2;
		_sde_connector_update_bl_scale_(c_conn);
	}

	if (oplus_dimlayer_bl_enable_v3 != oplus_dimlayer_bl_enable_v3_real) {
		struct sde_connector *c_conn = to_sde_connector(connector);

		if (oplus_datadimming_v3_skip_frame > 0) {
			oplus_datadimming_v3_skip_frame--;

		} else {
			oplus_dimlayer_bl_enable_v3_real = oplus_dimlayer_bl_enable_v3;
			_sde_connector_update_bl_scale_(c_conn);
			oplus_datadimming_v3_skip_frame = 2;
		}
	}

	if (post) {
		if (oplus_datadimming_vblank_count > 0) {
			oplus_datadimming_vblank_count--;

		} else {
			while (atomic_read(&oplus_datadimming_vblank_ref) > 0) {
				drm_crtc_vblank_put(connector->state->crtc);
				atomic_dec(&oplus_datadimming_vblank_ref);
			}
		}
	}

	return 0;
}
EXPORT_SYMBOL(sde_connector_update_backlight);

int oplus_seed_bright_to_alpha(int brightness)
{
	struct dsi_display *display = get_main_display();
	struct oplus_brightness_alpha *lut = NULL;
	int count = 0;
	int i = 0;
	int alpha;

	if (!display)
		return 0;

	if (oplus_panel_alpha)
		return oplus_panel_alpha;

	if (display->panel->dc_ba_seq && display->panel->dc_ba_count) {
		count = display->panel->dc_ba_count;
		lut = display->panel->dc_ba_seq;
	} else {
		count = ARRAY_SIZE(brightness_seed_alpha_lut_dc);
		lut = brightness_seed_alpha_lut_dc;
	}

	for (i = 0; i < count; i++) {
		if (lut[i].brightness >= brightness)
			break;
	}

	if (i == 0)
		alpha = lut[0].alpha;
	else if (i == count)
		alpha = lut[count - 1].alpha;
	else
		alpha = oplus_interpolate(brightness, lut[i-1].brightness,
				lut[i].brightness, lut[i-1].alpha,
				lut[i].alpha);

	return alpha;
}

struct dsi_panel_cmd_set *
oplus_panel_update_seed_backlight(struct dsi_panel *panel, int brightness,
		enum dsi_cmd_set_type type)
{
	enum dsi_cmd_set_state state;
	struct dsi_cmd_desc *cmds;
	struct dsi_cmd_desc *oplus_cmd;
	u8 *tx_buf;
	int count, rc = 0;
	int i = 0;
	int k = 0;
	int alpha = oplus_seed_bright_to_alpha(brightness);

	if (type != DSI_CMD_SEED_MODE0 &&
			type != DSI_CMD_SEED_MODE1 &&
			type != DSI_CMD_SEED_MODE2 &&
			type != DSI_CMD_SEED_MODE3 &&
			type != DSI_CMD_SEED_MODE4 &&
			type != DSI_CMD_SEED_OFF) {
		return NULL;
	}

	if (type == DSI_CMD_SEED_OFF) {
		type = DSI_CMD_SEED_MODE0;
	}

	cmds = panel->cur_mode->priv_info->cmd_sets[type].cmds;
	count = panel->cur_mode->priv_info->cmd_sets[type].count;
	state = panel->cur_mode->priv_info->cmd_sets[type].state;

	oplus_cmd = kmemdup(cmds, sizeof(*cmds) * count, GFP_KERNEL);

	if (!oplus_cmd) {
		rc = -ENOMEM;
		goto error;
	}

	for (i = 0; i < count; i++) {
		oplus_cmd[i].msg.tx_buf = NULL;
	}

	for (i = 0; i < count; i++) {
		u32 size;

		size = oplus_cmd[i].msg.tx_len * sizeof(u8);

		oplus_cmd[i].msg.tx_buf = kmemdup(cmds[i].msg.tx_buf, size, GFP_KERNEL);

		if (!oplus_cmd[i].msg.tx_buf) {
			rc = -ENOMEM;
			goto error;
		}
	}

	for (i = 0; i < count; i++) {
		if (oplus_cmd[i].msg.tx_len != 0x16) {
			continue;
		}

		tx_buf = (u8 *)oplus_cmd[i].msg.tx_buf;

		for (k = 0; k < oplus_cmd[i].msg.tx_len; k++) {
			if (k == 0) {
				continue;
			}

			tx_buf[k] = tx_buf[k] * (255 - alpha) / 255;
		}
	}

	if (oplus_priv_seed_cmd_set.cmds) {
		for (i = 0; i < oplus_priv_seed_cmd_set.count; i++) {
			kfree(oplus_priv_seed_cmd_set.cmds[i].msg.tx_buf);
		}

		kfree(oplus_priv_seed_cmd_set.cmds);
	}

	oplus_priv_seed_cmd_set.cmds = oplus_cmd;
	oplus_priv_seed_cmd_set.count = count;
	oplus_priv_seed_cmd_set.state = state;
	oplus_dc2_alpha = alpha;

	return &oplus_priv_seed_cmd_set;

error:

	if (oplus_cmd) {
		for (i = 0; i < count; i++) {
			kfree(oplus_cmd[i].msg.tx_buf);
		}

		kfree(oplus_cmd);
	}

	return ERR_PTR(rc);
}
EXPORT_SYMBOL(oplus_panel_update_seed_backlight);

int oplus_display_panel_get_dim_alpha(void *buf)
{
	unsigned int *temp_alpha = buf;

	if (__oplus_get_power_status() != OPLUS_DISPLAY_POWER_ON) {
		(*temp_alpha) = 0;
		return 0;
	}

	(*temp_alpha) = oplus_underbrightness_alpha;
	return 0;
}

int oplus_display_panel_set_dim_alpha(void *buf)
{
	unsigned int *temp_alpha = buf;

	oplus_panel_alpha = *temp_alpha;

	return 0;
}

int oplus_display_panel_get_dim_dc_alpha(void *buf)
{
	int ret = 0;
	unsigned int *temp_dim_alpha = buf;

	if (__oplus_get_power_status() != OPLUS_DISPLAY_POWER_ON) {
		ret = 0;
	}

	if (oplus_dc2_alpha != 0) {
		ret = oplus_dc2_alpha;

	} else if (oplus_underbrightness_alpha != 0) {
		ret = oplus_underbrightness_alpha;

	} else if (oplus_dimlayer_bl_enable_v3_real) {
		ret = 1;
	}

	(*temp_dim_alpha) = ret;
	return 0;
}


int oplus_display_panel_set_dimlayer_enable(void *data)
{
	struct dsi_display *display = NULL;
	struct drm_connector *dsi_connector = NULL;
	uint32_t *dimlayer_enable = data;

	display = get_main_display();
	if (!display) {
		return -EINVAL;
	}

	if (!strcmp(display->panel->name, "samsung S6E3HC3 dsc cmd mode panel 21631")) {
		dc_apollo_enable = *dimlayer_enable;
		LCD_INFO("DC BKL %s\n", *dimlayer_enable?"ON":"OFF");
		return 0;
	}

	dsi_connector = display->drm_conn;
	if (display && display->name) {
		int enable = (*dimlayer_enable);
		int err = 0;

		mutex_lock(&display->display_lock);
		if (!dsi_connector || !dsi_connector->state || !dsi_connector->state->crtc) {
			LCD_ERR("display not ready\n");
		} else {
			err = drm_crtc_vblank_get(dsi_connector->state->crtc);
			if (err) {
				LCD_ERR("failed to get crtc vblank, error=%d\n", err);
			} else {
				/* do vblank put after 7 frames */
				oplus_datadimming_vblank_count = 7;
				atomic_inc(&oplus_datadimming_vblank_ref);
			}
		}

		usleep_range(17000, 17100);
		if (!strcmp(display->panel->oplus_priv.vendor_name, "ANA6706")) {
			oplus_dimlayer_bl_enable = enable;
		} else {
			if (!strcmp(display->name, "qcom,mdss_dsi_oplus19101boe_nt37800_1080_2400_cmd"))
				oplus_dimlayer_bl_enable_v3 = enable;
			else
				oplus_dimlayer_bl_enable_v2 = enable;
		}
		mutex_unlock(&display->display_lock);
	}

	return 0;
}

int oplus_display_panel_get_dimlayer_enable(void *data)
{
	uint32_t *dimlayer_bl_enable = data;

	(*dimlayer_bl_enable) = oplus_dimlayer_bl_enable_v2;

	return 0;
}

int oplus_bl_to_alpha_dc(int brightness)
{
	int level = ARRAY_SIZE(brightness_alpha_lut_dc);
	int i = 0;
	int alpha;

	for (i = 0; i < ARRAY_SIZE(brightness_alpha_lut_dc); i++) {
		if (brightness_alpha_lut_dc[i].brightness >= brightness) {
			break;
		}
	}

	if (i == 0) {
		alpha = brightness_alpha_lut_dc[0].alpha;
	} else if (i == level) {
		alpha = brightness_alpha_lut_dc[level - 1].alpha;
	} else
		alpha = oplus_interpolate(brightness,
				    brightness_alpha_lut_dc[i - 1].brightness,
				    brightness_alpha_lut_dc[i].brightness,
				    brightness_alpha_lut_dc[i - 1].alpha,
				    brightness_alpha_lut_dc[i].alpha);

	return alpha;
}

int oplus_find_index_invmaplist(uint32_t bl_level)
{
	int index = 0;
	int ret = -1;

	if (bl_level == p_apollo_backlight->bl_level_last) {
		index = p_apollo_backlight->bl_index_last;
		return index;
	}

	if (!p_apollo_backlight->bl_fix) {
		ret = oplus_display_fix_apollo_level();
		if (ret < 0) {
			return ret;
		}
	}

	for (; index < p_apollo_backlight->bl_id_lens; index++) {
		if (p_apollo_backlight->apollo_bl_list[index] == bl_level) {
			p_apollo_backlight->bl_index_last = index;
			p_apollo_backlight->bl_level_last = bl_level;
			return index;
		}
	}

	return -1;
}


int oplus_get_panel_brightness(void)
{
	struct dsi_display *display = get_main_display();

	if (!display) {
		return 0;
	}

	return display->panel->bl_config.bl_level;
}
EXPORT_SYMBOL(oplus_get_panel_brightness);
