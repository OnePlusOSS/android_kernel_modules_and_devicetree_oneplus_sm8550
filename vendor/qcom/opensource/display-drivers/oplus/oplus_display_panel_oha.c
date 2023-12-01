/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
**
** File : oplus_display_panel_oha.c
** Description : oplus display panel 1hz aod feature
** Version : 1.0
** Date : 2021/11/30
** Author : Display
******************************************************************/
#include "oplus_display_panel_oha.h"
#include "oplus_dsi_support.h"
#include "oplus_display_panel_common.h"
#include "sde_crtc.h"
#include "sde_trace.h"

#define OPLUS_OHA_CONFIG_GLOBAL (1<<0)
#define OHA_GET_GLOBAL_CONFIG(config) ((config) & OPLUS_OHA_CONFIG_GLOBAL)

/* Add for oplus 1hz aod */
static u32 oplus_oha_config = 0;
u32 oplus_oha_enable = OHA_SWITCH_OFF;
DEFINE_MUTEX(oplus_oha_lock);
/*int oplus_oha_vblank_count;
atomic_t oplus_oha_vblank_ref = ATOMIC_INIT(0);
static struct task_struct *oha_update_task;
static wait_queue_head_t oha_update_task_wq;
static atomic_t oha_update_task_wakeup = ATOMIC_INIT(0);*/

void oplus_panel_oha_init(struct dsi_panel *panel)
{
	static bool inited = false;
	u32 config = 0;
	int rc = 0;
	struct dsi_parser_utils *utils = NULL;

	if (!panel)
		return;

	utils = &panel->utils;
	if (!utils)
		return;

	if (inited && oplus_oha_config) {
		LCD_WARN("kOFP oha config = %#X already\n", oplus_oha_config);
		return;
	}

	rc = utils->read_u32(utils->data, "oplus,oha-config", &config);
	if (rc == 0) {
		oplus_oha_config = config;
	} else {
		oplus_oha_config = 0;
	}

	inited = true;

	LCD_INFO("kOFP oha config = %#X\n", oplus_oha_config);
}

inline bool oplus_oha_is_support(void)
{
	return (bool) (OHA_GET_GLOBAL_CONFIG(oplus_oha_config));
}

int oplus_panel_update_oha_mode_unlock(struct drm_crtc_state *cstate,
		struct dsi_panel *panel)
{
	int rc = 0;
	int count;
	enum dsi_cmd_set_type type;
	struct dsi_display_mode *mode;

	if (!cstate) {
		LCD_ERR("kOFP, Invalid params cstate null\n");
		return 0;
	}

	if (!panel || !panel->cur_mode) {
		LCD_ERR("Invalid params\n");
		return -EINVAL;
	}
	mode = panel->cur_mode;

	if (OHA_SWITCH_ON == oplus_crtc_get_oha_mode(cstate)) {
		type = DSI_CMD_OHA_ON;
	} else {
		type = DSI_CMD_OHA_OFF;
	}

	count = mode->priv_info->cmd_sets[type].count;
	if (!count) {
		LCD_ERR("This panel does not support oplus oha mode\n");
		goto error;
	}

	rc = dsi_panel_tx_cmd_set(panel, type);

error:
	return rc;
}

int oplus_display_update_oha_mode(struct drm_crtc_state *cstate,
		struct dsi_display *display)
{
	int rc = 0;

	if (!cstate) {
		LCD_ERR("kOFP, Invalid params cstate null\n");
		return 0;
	}

	if (!display || !display->panel) {
		LCD_ERR("kOFP, Invalid params(s) display %pK, panel %pK\n",
				display, ((display) ? display->panel : NULL));
		return -EINVAL;
	}

/*
	if (display->panel->is_hbm_enabled) {
		LCD_ERR("error panel->is_hbm_enabled\n");
		return -EINVAL;
	}
*/

	if (get_oplus_display_scene() != OPLUS_DISPLAY_AOD_SCENE) {
		LCD_ERR("get_oplus_display_scene = %d, return\n",
				get_oplus_display_scene());
		return -EFAULT;
	}

	mutex_lock(&display->display_lock);

	/* enable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_ON);
		if (rc) {
			LCD_ERR("[%s] failed to disable DSI core clocks, rc=%d\n",
				display->name, rc);
			goto error;
		}
	}

	mutex_lock(&display->panel->panel_lock);

	if (!dsi_panel_initialized(display->panel)) {
		LCD_ERR("oplus_display_update_oha_mode is not init\n");
		rc = -EINVAL;
		goto error;
	}

	rc = oplus_panel_update_oha_mode_unlock(cstate, display->panel);

	if (rc) {
		LCD_ERR("[%s] failed to oplus_panel_update_oha_mode_unlock, rc=%d\n",
				display->name, rc);
		goto error;
	}

error:
	mutex_unlock(&display->panel->panel_lock);

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_OFF);
	}

	mutex_unlock(&display->display_lock);

	return rc;
}

/* static int oha_update_worker_kthread(void *data)
{
	int ret = 0;

	while (!kthread_should_stop()) {
		ret = wait_event_interruptible(oha_update_task_wq, atomic_read(&oha_update_task_wakeup));
		atomic_set(&oha_update_task_wakeup, 0);
		oplus_display_update_oha_mode();
	}
	return 0;
}

static void oplus_oha_update_init(void)
{
	if (!oha_update_task) {
		oha_update_task = kthread_create(oha_update_worker_kthread, NULL, "oha_update");
		init_waitqueue_head(&oha_update_task_wq);
		wake_up_process(oha_update_task);
		LCD_INFO("[oha_update] init CREATE\n");
	}
}

void oplus_oha_update(void)
{
	if (oha_update_task != NULL) {
		atomic_set(&oha_update_task_wakeup, 1);
		wake_up_interruptible(&oha_update_task_wq);
	} else {
		LCD_INFO("[oha_update] update is NULL\n");
	}
}*/

static int oplus_ofp_update_oha_enter(struct drm_crtc_state *cstate,
		struct dsi_display *dsi_display)
{
	int rc = 0;

	if (!cstate) {
		LCD_ERR("kOFP, Invalid params cstate null\n");
		return 0;
	}

	if (!dsi_display || !dsi_display->panel) {
		LCD_ERR("kOFP, Invalid params(s) dsi_display %pK, panel %pK\n",
			  dsi_display,
			  ((dsi_display) ? dsi_display->panel : NULL));
		return -EINVAL;
	}

	if (!dsi_display->panel->panel_initialized) {
		dsi_display->panel->is_oha_enabled = false;
		LCD_ERR("kOFP, panel not initialized, failed to Enter Oha\n");
		return 0;
	}

	rc = oplus_display_update_oha_mode(cstate, dsi_display);

	if (rc) {
		LCD_ERR("kOFP, oplus_display_update_oha_mode failed, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static int oplus_ofp_update_oha_exit(struct drm_crtc_state *cstate,
		struct dsi_display *dsi_display)
{
	int rc = 0;

	if (!cstate) {
		LCD_ERR("kOFP, Invalid params cstate null\n");
		return 0;
	}

	if (!dsi_display || !dsi_display->panel) {
		LCD_ERR("kOFP, Invalid params(s) dsi_display %pK, panel %pK\n",
			  dsi_display,
			  ((dsi_display) ? dsi_display->panel : NULL));
		return -EINVAL;
	}

	if (!dsi_display->panel->panel_initialized) {
		dsi_display->panel->is_oha_enabled = true;
		LCD_ERR("kOFP, panel not initialized, failed to Exit Oha\n");
		return 0;
	}

	rc = oplus_display_update_oha_mode(cstate, dsi_display);

	if (rc) {
		LCD_ERR("kOFP, oplus_display_update_oha_mode failed, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

int oplus_connector_ofp_update_oha(struct drm_connector *connector)
{
	struct sde_connector *c_conn = to_sde_connector(connector);
	struct dsi_display *dsi_display;
	int rc = 0;
	int oha_mode;

	if (!c_conn) {
		LCD_ERR("kOFP, Invalid params oha sde_connector null\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		return 0;
	}

	dsi_display = c_conn->display;

	if (!dsi_display || !dsi_display->panel) {
		LCD_ERR("kOFP, Invalid params(s) oha dsi_display %pK, panel %pK\n",
			  dsi_display,
			  ((dsi_display) ? dsi_display->panel : NULL));
		return -EINVAL;
	}

	if (!c_conn->encoder || !c_conn->encoder->crtc ||
			!c_conn->encoder->crtc->state) {
		return 0;
	}

	oha_mode = oplus_crtc_get_oha_mode(c_conn->encoder->crtc->state);

	if (oha_mode != dsi_display->panel->is_oha_enabled) {
		if (OPLUS_DISPLAY_AOD_SCENE == get_oplus_display_scene()) {
			char oha_trace_name[60];
			LCD_ERR("kOFP, Oha mode: %s\n",
					oha_mode ? "Enter" : "Exit");

			sprintf(oha_trace_name, "Oha_%s",
					oha_mode ? "Enter" : "Exit");
			dsi_display->panel->is_oha_enabled = oha_mode;
			/*oplus_oha_update_notify_init();*/

			SDE_ATRACE_BEGIN(oha_trace_name);
			if (oha_mode) {
				rc = oplus_ofp_update_oha_enter(c_conn->encoder->crtc->state, dsi_display);
				if (rc) {
					LCD_ERR("kOFP, failed to oplus_ofp_update_oha_enter, rc=%d\n", rc);
				}
			} else {
				rc = oplus_ofp_update_oha_exit(c_conn->encoder->crtc->state, dsi_display);
				if (rc) {
					LCD_ERR("kOFP, failed to oplus_ofp_update_oha_exit, rc=%d\n", rc);
				}
			}
			SDE_ATRACE_END(oha_trace_name);
		}
	}

	return 0;
}
EXPORT_SYMBOL(oplus_connector_ofp_update_oha);

int __oplus_set_oha_mode(int mode)
{
	mutex_lock(&oplus_oha_lock);

	if (mode != oplus_oha_enable) {
		oplus_oha_enable = mode;
	}
	mutex_unlock(&oplus_oha_lock);
	return 0;
}

int oplus_display_panel_update_oha_mode_unlock(struct dsi_panel *panel)
{
	int rc = 0;
	int count;
	enum dsi_cmd_set_type type;
	struct dsi_display_mode *mode;

	if (!panel || !panel->cur_mode) {
		LCD_ERR("Invalid params\n");
		return -EINVAL;
	}
	mode = panel->cur_mode;

	if (OHA_SWITCH_ON == oplus_oha_enable) {
		type = DSI_CMD_OHA_ON;
	} else {
		type = DSI_CMD_OHA_OFF;
	}

	count = mode->priv_info->cmd_sets[type].count;
	if (!count) {
		LCD_ERR("This panel does not support oplus oha mode\n");
		goto error;
	}

	rc = dsi_panel_tx_cmd_set(panel, type);

error:
	return rc;
}

int oplus_display_panel_update_oha_mode(void)
{
	int rc = 0;
	struct dsi_display *display = get_main_display();

	if (!display || !display->panel) {
		LCD_ERR("kOFP, Invalid params(s) display %pK, panel %pK\n",
			  display, ((display) ? display->panel : NULL));
		return -EINVAL;
	}

/*
	if (display->panel->is_hbm_enabled) {
		LCD_ERR("error panel->is_hbm_enabled\n");
		return -EINVAL;
	}
*/

	if (get_oplus_display_scene() != OPLUS_DISPLAY_AOD_SCENE) {
		LCD_ERR("get_oplus_display_scene = %d, return\n",
				get_oplus_display_scene());
		return -EFAULT;
	}

	mutex_lock(&display->display_lock);

	/* enable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_ON);
		if (rc) {
			LCD_ERR("[%s] failed to disable DSI core clocks, rc=%d\n",
				display->name, rc);
			goto error;
		}
	}

	mutex_lock(&display->panel->panel_lock);

	if (!dsi_panel_initialized(display->panel)) {
		LCD_ERR("oplus_display_update_oha_mode is not init\n");
		rc = -EINVAL;
		goto error;
	}

	rc = oplus_panel_update_oha_mode_unlock(display->panel);

	if (rc) {
		LCD_ERR("[%s] failed to oplus_panel_update_oha_mode_unlock, rc=%d\n",
				display->name, rc);
		goto error;
	}

error:
	mutex_unlock(&display->panel->panel_lock);

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_OFF);
	}

	mutex_unlock(&display->display_lock);

	return rc;
}

bool oplus_crtc_get_oha_mode(struct drm_crtc_state *crtc_state)
{
	struct sde_crtc_state *cstate;

	if (!crtc_state) {
		return false;
	}

	cstate = to_sde_crtc_state(crtc_state);
	return !!cstate->oha_mode;
}

int oplus_display_panel_get_oha_enable(void *data)
{
	uint32_t *oha_enable = data;

	(*oha_enable) = oplus_oha_enable;

	return 0;
}

int oplus_display_panel_set_oha_enable(void *data)
{
	uint32_t *oha_enable = data;
	int value = (*oha_enable);
	struct dsi_display *display = get_main_display();

	if (!display || !display->panel) {
		LCD_ERR("invalid display/panel\n");
		return -EINVAL;
	}

	LCD_ERR("kOFP, systemui set oha_enable = %d\n", value);

	__oplus_set_oha_mode(value);

	if (!display->panel->is_ofp_enabled)
		oplus_display_panel_update_oha_mode();

	return 0;
}

