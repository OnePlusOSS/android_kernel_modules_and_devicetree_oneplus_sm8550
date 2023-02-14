/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
**
** File : oplus_display_panel_seed.c
** Description : oplus display panel seed feature
** Version : 1.1
** Date : 2020/06/13
** Author : Display
******************************************************************/
#include "oplus_display_panel_seed.h"
#include "oplus_dsi_support.h"
#include "oplus_display_private_api.h"

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "oplus_onscreenfingerprint.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

int seed_mode = 0;
extern int oplus_seed_backlight;
/* outdoor hbm flag*/

DEFINE_MUTEX(oplus_seed_lock);

int oplus_display_get_seed_mode(void)
{
	return seed_mode;
}

int __oplus_display_set_seed(int mode)
{
	mutex_lock(&oplus_seed_lock);

	if (mode != seed_mode) {
		seed_mode = mode;
	}

	mutex_unlock(&oplus_seed_lock);
	return 0;
}

int dsi_panel_seed_mode_unlock(struct dsi_panel *panel, int mode)
{
	int rc = 0;

	if (!dsi_panel_initialized(panel)) {
		return -EINVAL;
	}

	switch (mode) {
	case 0:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SEED_MODE0);

		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_SEED_MODE0 cmds, rc=%d\n",
					panel->name, rc);
		}

		break;

	case 1:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SEED_MODE1);

		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_SEED_MODE1 cmds, rc=%d\n",
					panel->name, rc);
		}

		break;

	case 2:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SEED_MODE2);

		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_SEED_MODE2 cmds, rc=%d\n",
					panel->name, rc);
		}

		break;

	case 3:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SEED_MODE3);

		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_SEED_MODE3 cmds, rc=%d\n",
					panel->name, rc);
		}

		break;

	case 4:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SEED_MODE4);

		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_SEED_MODE4 cmds, rc=%d\n",
					panel->name, rc);
		}

		break;

	default:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SEED_OFF);

		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_SEED_OFF cmds, rc=%d\n",
					panel->name, rc);
		}

		pr_err("[%s] seed mode Invalid %d\n",
				panel->name, mode);
	}

	return rc;
}

int dsi_panel_loading_effect_mode_unlock(struct dsi_panel *panel, int mode)
{
	int rc = 0;

	if (!dsi_panel_initialized(panel)) {
		return -EINVAL;
	}

	switch (mode) {
	case PANEL_LOADING_EFFECT_MODE1:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_LOADING_EFFECT_MODE1);

		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_SEED_MODE0 cmds, rc=%d\n",
					panel->name, rc);
		}

		break;

	case PANEL_LOADING_EFFECT_MODE2:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_LOADING_EFFECT_MODE2);

		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_SEED_MODE1 cmds, rc=%d\n",
					panel->name, rc);
		}

		break;

	case PANEL_LOADING_EFFECT_OFF:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_LOADING_EFFECT_OFF);

		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_SEED_MODE2 cmds, rc=%d\n",
					panel->name, rc);
		}

		break;

	default:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_LOADING_EFFECT_OFF);

		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_LOADING_EFFECT_OFF cmds, rc=%d\n",
					panel->name, rc);
		}

		pr_err("[%s] loading effect mode Invalid %d\n",
				panel->name, mode);
	}

	return rc;
}

int dsi_panel_seed_mode(struct dsi_panel *panel, int mode)
{
	int rc = 0;

	if (!panel) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	if (mode >= PANEL_LOADING_EFFECT_FLAG) {
		rc = dsi_panel_loading_effect_mode_unlock(panel, mode);
	} else {
		rc = dsi_panel_seed_mode_unlock(panel, mode);
	}

	return rc;
}

int dsi_display_seed_mode_lock(struct dsi_display *display, int mode)
{
	int rc = 0;

	if (!display || !display->panel) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&display->display_lock);
	mutex_lock(&display->panel->panel_lock);

	/* enable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_ON);
	}

	rc = dsi_panel_seed_mode(display->panel, mode);

	if (rc) {
		pr_err("[%s] failed to dsi_panel_seed_or_loading_effect_on, rc=%d\n",
				display->name, rc);
	}

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_OFF);
	}

	mutex_unlock(&display->panel->panel_lock);
	mutex_unlock(&display->display_lock);
	return rc;
}

int oplus_display_panel_get_seed(void *data)
{
	uint32_t *temp = data;
	printk(KERN_INFO "oplus_display_get_seed = %d\n", seed_mode);

	(*temp) = seed_mode;
	return 0;
}

int oplus_display_panel_set_seed(void *data)
{
	uint32_t *temp_save = data;
	uint32_t panel_id = (*temp_save >> 12);
	struct dsi_display *display = oplus_display_get_current_display();
	seed_mode = (*temp_save & 0x0fff);
	printk(KERN_INFO "%s oplus_display_set_seed = %d, panel_id = %d\n", __func__, seed_mode, panel_id);

	__oplus_display_set_seed(seed_mode);

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	if (oplus_ofp_is_supported() && !oplus_ofp_oled_capacitive_is_enabled()
			&& !oplus_ofp_local_hbm_is_enabled()) {
		if (oplus_ofp_get_hbm_state()) {
			OFP_INFO("should not set seed in hbm state\n");
			return 0;
		}
	}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

	if (1 == panel_id) {
		display = get_sec_display();
	}

	if (display == NULL) {
		printk(KERN_INFO "oplus_display_set_seed and main display is null");
		return -EINVAL;
	}
	if (display->panel->power_mode != SDE_MODE_DPMS_ON) {
		printk(KERN_ERR
				"<%s> %s oplus_display_set_seed = %d, but now display panel power_mode is not on\n",
				display->panel->oplus_priv.vendor_name, __func__, seed_mode);
		return -EINVAL;
	}

	dsi_display_seed_mode_lock(display, seed_mode);

	return 0;
}

