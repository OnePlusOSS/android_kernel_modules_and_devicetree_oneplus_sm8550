/***************************************************************
** Copyright (C), 2023, OPLUS Mobile Comm Corp., Ltd
** File : oplus_adfr.c
** Description : oplus_adfr implement
** Version : 2.0
** Date : 2023/05/04
** Author : Display
***************************************************************/

#include "oplus_adfr.h"
#include "oplus_bl.h"
#include "oplus_display_panel_common.h"
#include "dsi_display.h"
#include "sde_trace.h"
#include "sde_encoder_phys.h"

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "oplus_onscreenfingerprint.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

#if defined(CONFIG_PXLW_IRIS)
#include "dsi_iris_api.h"
#endif /* CONFIG_PXLW_IRIS */

/* -------------------- macro -------------------- */
/* config bit setting */
#define OPLUS_ADFR_CONFIG_GLOBAL							(BIT(0))
#define OPLUS_ADFR_CONFIG_FAKEFRAME							(BIT(1))
#define OPLUS_ADFR_CONFIG_VSYNC_SWITCH						(BIT(2))
#define OPLUS_ADFR_CONFIG_VSYNC_SWITCH_MODE					(BIT(3))
#define OPLUS_ADFR_CONFIG_IDLE_MODE							(BIT(4))
#define OPLUS_ADFR_CONFIG_TEMPERATURE_DETECTION				(BIT(5))
#define OPLUS_ADFR_CONFIG_OA_BL_MUTUAL_EXCLUSION			(BIT(6))
#define OPLUS_ADFR_CONFIG_SA_MODE_RESTORE					(BIT(7))
#define OPLUS_ADFR_CONFIG_DRY_RUN							(BIT(8))
#define OPLUS_ADFR_CONFIG_HIGH_PRECISION_SA_MODE			(BIT(9))
#define OPLUS_ADFR_CONFIG_HIGH_PRECISION_OA_MODE			(BIT(10))
#define OPLUS_ADFR_CONFIG_HIGH_PRECISION_SWITCH				(BIT(11))

/* get config value */
#define OPLUS_ADFR_GET_GLOBAL_CONFIG(config)				((config) & OPLUS_ADFR_CONFIG_GLOBAL)
#define OPLUS_ADFR_GET_FAKEFRAME_CONFIG(config)				((config) & OPLUS_ADFR_CONFIG_FAKEFRAME)
#define OPLUS_ADFR_GET_VSYNC_SWITCH_CONFIG(config)			((config) & OPLUS_ADFR_CONFIG_VSYNC_SWITCH)
#define OPLUS_ADFR_GET_VSYNC_SWITCH_MODE_CONFIG(config)		((config) & OPLUS_ADFR_CONFIG_VSYNC_SWITCH_MODE)
#define OPLUS_ADFR_GET_IDLE_MODE_CONFIG(config)				((config) & OPLUS_ADFR_CONFIG_IDLE_MODE)
#define OPLUS_ADFR_GET_TEMPERATURE_DETECTION_CONFIG(config)	((config) & OPLUS_ADFR_CONFIG_TEMPERATURE_DETECTION)
#define OPLUS_ADFR_GET_OA_BL_MUTUAL_EXCLUSION_CONFIG(config) ((config) & OPLUS_ADFR_CONFIG_OA_BL_MUTUAL_EXCLUSION)
#define OPLUS_ADFR_GET_SA_MODE_RESTORE_CONFIG(config)		((config) & OPLUS_ADFR_CONFIG_SA_MODE_RESTORE)
#define OPLUS_ADFR_GET_DRY_RUN_CONFIG(config)				((config) & OPLUS_ADFR_CONFIG_DRY_RUN)
#define ADFR_GET_HIGH_PRECISION_SA_MODE_CONFIG(config)		((config) & OPLUS_ADFR_CONFIG_HIGH_PRECISION_SA_MODE)
#define ADFR_GET_HIGH_PRECISION_OA_MODE_CONFIG(config)		((config) & OPLUS_ADFR_CONFIG_HIGH_PRECISION_OA_MODE)
#define ADFR_GET_HIGH_PRECISION_SWITCH_CONFIG(config)		((config) & OPLUS_ADFR_CONFIG_HIGH_PRECISION_SWITCH)

/* SA property value */
#define OPLUS_ADFR_SA_MAGIC									0x00800000
#define OPLUS_ADFR_AUTO_MODE_MAGIC							0x00400000
#define OPLUS_ADFR_AUTO_MODE_VALUE(value)					(((value) & 0x003F0000) >> 16)
#define OPLUS_ADFR_FAKEFRAME_MAGIC							0x00008000
#define OPLUS_ADFR_FAKEFRAME_VALUE(value)					(((value) & 0x00007F00) >> 8)
#define OPLUS_ADFR_SA_MIN_FPS_MAGIC							0x00000080
#define OPLUS_ADFR_SA_MIN_FPS_VALUE(value)					((value) & 0x0000007F)
#define OPLUS_ADFR_HIGH_PRECISION_SA_MODE_MAGIC				0x00800000
#define OPLUS_ADFR_HIGH_PRECISION_OA_MODE_MAGIC				0x00000800
#define OPLUS_ADFR_HIGH_PRECISION_SA_VALUE(value)			(((value)&0X007FF000)>>12)
#define OPLUS_ADFR_HIGH_PRECISION_OA_VALUE(value)			((value)&0X000007FF)

#define to_sde_encoder_phys_cmd(x)							container_of(x, struct sde_encoder_phys_cmd, base)

/* -------------------- parameters -------------------- */
/* log level config */
unsigned int oplus_adfr_log_level = OPLUS_ADFR_LOG_LEVEL_DEBUG;
EXPORT_SYMBOL(oplus_adfr_log_level);
/* dual display id */
unsigned int oplus_adfr_display_id = OPLUS_ADFR_PRIMARY_DISPLAY;
EXPORT_SYMBOL(oplus_adfr_display_id);
/* adfr global structure */
static struct oplus_adfr_params g_oplus_adfr_params[2] = {0};

/* -------------------- extern -------------------- */
/* extern params */
/* dsi cmd set prop map */
extern const char *cmd_set_prop_map[DSI_CMD_SET_MAX];
extern u32 oplus_last_backlight;

/* extern functions */
int _get_tearcheck_threshold(struct sde_encoder_phys *phys_enc);

/* -------------------- oplus_adfr_params -------------------- */
static int oplus_adfr_set_display_id(unsigned int display_id)
{
	ADFR_DEBUG("start\n");

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_set_display_id");

	oplus_adfr_display_id = display_id;
	ADFR_DEBUG("oplus_adfr_display_id:%u\n", oplus_adfr_display_id);
	OPLUS_ADFR_TRACE_INT("oplus_adfr_display_id", oplus_adfr_display_id);

	OPLUS_ADFR_TRACE_END("oplus_adfr_set_display_id");

	ADFR_DEBUG("end\n");

	return 0;
}

/* update display id for dual panel */
int oplus_adfr_update_display_id(void)
{
	struct dsi_display *display = oplus_display_get_current_display();

	ADFR_DEBUG("start\n");

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_update_display_id");

	if (!display) {
		ADFR_ERR("failed to get current display, set default display id to 0\n");
		oplus_adfr_set_display_id(OPLUS_ADFR_PRIMARY_DISPLAY);
	} else {
		if (!strcmp(display->display_type, "primary")) {
		oplus_adfr_set_display_id(OPLUS_ADFR_PRIMARY_DISPLAY);
		} else if (!strcmp(display->display_type, "secondary")) {
			oplus_adfr_set_display_id(OPLUS_ADFR_SECONDARY_DISPLAY);
		} else {
			ADFR_ERR("unknown display type:%s\n", display->display_type);
		}
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_update_display_id");

	ADFR_DEBUG("end\n");

	return 0;
}

static struct oplus_adfr_params *oplus_adfr_get_params(void *dsi_panel)
{
	struct dsi_panel *panel = dsi_panel;

	if (!panel) {
		ADFR_ERR("invalid panel param\n");
		return NULL;
	}

	if (!strcmp(panel->type, "primary")) {
		oplus_adfr_set_display_id(OPLUS_ADFR_PRIMARY_DISPLAY);
		return &g_oplus_adfr_params[OPLUS_ADFR_PRIMARY_DISPLAY];
	} else if (!strcmp(panel->type, "secondary")) {
		oplus_adfr_set_display_id(OPLUS_ADFR_SECONDARY_DISPLAY);
		return &g_oplus_adfr_params[OPLUS_ADFR_SECONDARY_DISPLAY];
	} else {
		ADFR_ERR("unknown panel type:%s\n", panel->type);
		return NULL;
	}
}

/* get config value from panel dtsi */
int oplus_adfr_init(void *dsi_panel)
{
	int rc = 0;
	unsigned int value = 0;
	struct dsi_panel *panel = dsi_panel;
	struct dsi_parser_utils *utils = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!panel) {
		ADFR_ERR("invalid panel param\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		ADFR_INFO("no need to init for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */


	p_oplus_adfr_params = oplus_adfr_get_params(panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	utils = &panel->utils;
	if (!utils) {
		ADFR_ERR("invalid utils param\n");
		return -EINVAL;
	}

	ADFR_INFO("init %s display adfr params\n", panel->type);

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_init");

	/* oplus,adfr-config */
	rc = utils->read_u32(utils->data, "oplus,adfr-config", &value);
	if (rc) {
		ADFR_INFO("failed to read oplus,adfr-config, rc=%d\n", rc);
		/* set default value to 0 */
		p_oplus_adfr_params->config = 0;
	} else {
		p_oplus_adfr_params->config = value;
	}

	if (oplus_is_factory_boot()) {
		p_oplus_adfr_params->config = 0;
		ADFR_INFO("disable adfr in factory mode\n");
	}

	if (oplus_adfr_is_supported(p_oplus_adfr_params)) {
		if (oplus_adfr_vsync_switch_is_enabled(p_oplus_adfr_params)
				&& (oplus_adfr_get_vsync_switch_mode(p_oplus_adfr_params) == OPLUS_ADFR_MUX_VSYNC_SWITCH)) {
			/* oplus,adfr-mux-vsync-switch-gpio */
			p_oplus_adfr_params->mux_vsync_switch_gpio = utils->get_named_gpio(utils->data, "oplus,adfr-mux-vsync-switch-gpio", 0);
			if (!gpio_is_valid(p_oplus_adfr_params->mux_vsync_switch_gpio)) {
				p_oplus_adfr_params->config &= ~(OPLUS_ADFR_CONFIG_VSYNC_SWITCH);
				ADFR_ERR("[%s] oplus,adfr-mux-vsync-switch-gpio is not set, disable vsync switch config\n", panel->name);
			} else {
				rc = gpio_request(p_oplus_adfr_params->mux_vsync_switch_gpio, "mux_vsync_switch_gpio");
				if (rc) {
					p_oplus_adfr_params->config &= ~(OPLUS_ADFR_CONFIG_VSYNC_SWITCH);
					ADFR_ERR("failed to request mux_vsync_switch_gpio %d, disable vsync switch config, rc=%d\n", p_oplus_adfr_params->mux_vsync_switch_gpio, rc);
				} else {
					ADFR_INFO("[%s] oplus,adfr-mux-vsync-switch-gpio is %d\n", panel->name, p_oplus_adfr_params->mux_vsync_switch_gpio);
				}
			}
		}

		/* oplus,adfr-test-te-gpio */
		p_oplus_adfr_params->test_te.gpio = utils->get_named_gpio(utils->data, "oplus,adfr-test-te-gpio", 0);
		if (!gpio_is_valid(p_oplus_adfr_params->test_te.gpio)) {
			ADFR_INFO("[%s] oplus,adfr-test-te-gpio is not set\n", panel->name);
		} else {
			ADFR_INFO("[%s] oplus,adfr-test-te-gpio is %d\n", panel->name, p_oplus_adfr_params->test_te.gpio);

			/* test te timer init */
			hrtimer_init(&p_oplus_adfr_params->test_te.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
			p_oplus_adfr_params->test_te.timer.function = oplus_adfr_test_te_timer_handler;
		}

		if (oplus_adfr_oa_bl_mutual_exclusion_is_enabled(p_oplus_adfr_params)) {
			/* osync mode timer init */
			hrtimer_init(&p_oplus_adfr_params->osync_mode_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
			p_oplus_adfr_params->osync_mode_timer.function = oplus_adfr_osync_mode_timer_handler;
		}
	}

	ADFR_INFO("oplus_adfr_config:0x%x\n", p_oplus_adfr_params->config);
	OPLUS_ADFR_TRACE_INT("oplus_adfr_config", p_oplus_adfr_params->config);

	if (!strcmp(panel->type, "secondary")) {
		/* set default display id to primary display */
		oplus_adfr_set_display_id(OPLUS_ADFR_PRIMARY_DISPLAY);
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_init");

	ADFR_DEBUG("end\n");

	return rc;
}

bool oplus_adfr_is_supported(void *oplus_adfr_params)
{
	struct oplus_adfr_params *p_oplus_adfr_params = oplus_adfr_params;

	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return false;
	}

	/* global config, support adfr panel */
	return (bool)(OPLUS_ADFR_GET_GLOBAL_CONFIG(p_oplus_adfr_params->config));
}

static bool oplus_adfr_fakeframe_is_enabled(void *oplus_adfr_params)
{
	struct oplus_adfr_params *p_oplus_adfr_params = oplus_adfr_params;

	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return false;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported, fakeframe is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_ADFR_GET_FAKEFRAME_CONFIG(p_oplus_adfr_params->config));
}

bool oplus_adfr_vsync_switch_is_enabled(void *oplus_adfr_params)
{
	struct oplus_adfr_params *p_oplus_adfr_params = oplus_adfr_params;

	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return false;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported, vsync switch is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_ADFR_GET_VSYNC_SWITCH_CONFIG(p_oplus_adfr_params->config));
}

enum oplus_adfr_vsync_switch_mode oplus_adfr_get_vsync_switch_mode(void *oplus_adfr_params)
{
	struct oplus_adfr_params *p_oplus_adfr_params = oplus_adfr_params;

	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return OPLUS_ADFR_INVALID_VSYNC_SWITCH;
	}

	if (!oplus_adfr_vsync_switch_is_enabled(p_oplus_adfr_params)) {
		ADFR_DEBUG("vsync switch is not enabled, vsync switch mode is invalid\n");
		return OPLUS_ADFR_INVALID_VSYNC_SWITCH;
	}

	return (enum oplus_adfr_vsync_switch_mode)OPLUS_ADFR_GET_VSYNC_SWITCH_MODE_CONFIG(p_oplus_adfr_params->config);
}

static bool oplus_adfr_idle_mode_is_enabled(void *oplus_adfr_params)
{
	struct oplus_adfr_params *p_oplus_adfr_params = oplus_adfr_params;

	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return false;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported, idle mode is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_ADFR_GET_IDLE_MODE_CONFIG(p_oplus_adfr_params->config));
}

static bool oplus_adfr_temperature_detection_is_enabled(void *oplus_adfr_params)
{
	struct oplus_adfr_params *p_oplus_adfr_params = oplus_adfr_params;

	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return false;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported, temperature detection is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_ADFR_GET_TEMPERATURE_DETECTION_CONFIG(p_oplus_adfr_params->config));
}

bool oplus_adfr_oa_bl_mutual_exclusion_is_enabled(void *oplus_adfr_params)
{
	struct oplus_adfr_params *p_oplus_adfr_params = oplus_adfr_params;

	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return false;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported, oa bl mutual exclusion is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_ADFR_GET_OA_BL_MUTUAL_EXCLUSION_CONFIG(p_oplus_adfr_params->config));
}

static bool oplus_adfr_sa_mode_restore_is_enabled(void *oplus_adfr_params)
{
	struct oplus_adfr_params *p_oplus_adfr_params = oplus_adfr_params;

	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return false;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported, sa mode restore is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_ADFR_GET_SA_MODE_RESTORE_CONFIG(p_oplus_adfr_params->config));
}

static bool oplus_adfr_dry_run_is_enabled(void *oplus_adfr_params)
{
	struct oplus_adfr_params *p_oplus_adfr_params = oplus_adfr_params;

	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return false;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported, dry run is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_ADFR_GET_DRY_RUN_CONFIG(p_oplus_adfr_params->config));
}

/* --------------- high precision mode ---------------*/
static bool oplus_adfr_high_precision_sa_mode_is_enabled(void *oplus_adfr_params)
{
	struct oplus_adfr_params *p_oplus_adfr_params = oplus_adfr_params;

	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return false;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported, high precision sa mode is also not supported\n");
		return false;
	}

	return (bool)(ADFR_GET_HIGH_PRECISION_SA_MODE_CONFIG(p_oplus_adfr_params->config));
}

static bool oplus_adfr_high_precision_oa_mode_is_enabled(void *oplus_adfr_params)
{
	struct oplus_adfr_params *p_oplus_adfr_params = oplus_adfr_params;

	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return false;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported, high precision oa mode is also not supported\n");
		return false;
	}

	return (bool)(ADFR_GET_HIGH_PRECISION_OA_MODE_CONFIG(p_oplus_adfr_params->config));
}

static bool oplus_adfr_high_precision_switch_is_enabled(void *oplus_adfr_params)
{
	struct oplus_adfr_params *p_oplus_adfr_params = oplus_adfr_params;

	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return false;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported, high precision switch is also not supported\n");
		return false;
	}

	return (bool)(ADFR_GET_HIGH_PRECISION_SWITCH_CONFIG(p_oplus_adfr_params->config));
}

int oplus_adfr_get_panel_high_precision_state(void *dsi_display)
{
	struct dsi_display *display = dsi_display;
	unsigned int h_skew = STANDARD_ADFR;
	unsigned int refresh_rate = 120;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	if (!display || !display->panel || !display->panel->cur_mode) {
		ADFR_ERR("invalid display or panel params\n");
		return -EINVAL;
	}

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_get_panel_high_precision_state");
	h_skew = display->panel->cur_mode->timing.h_skew;
	refresh_rate = display->panel->cur_mode->timing.refresh_rate;

	if (h_skew == STANDARD_ADFR && refresh_rate == 120) {
		OPLUS_ADFR_TRACE_INT("oplus_adfr_panel_high_precision_state", p_oplus_adfr_params->panel_high_precision_state);
		OPLUS_ADFR_TRACE_END("oplus_adfr_get_panel_high_precision_state");
		return p_oplus_adfr_params->panel_high_precision_state;
	} else {
		OPLUS_ADFR_TRACE_INT("oplus_adfr_panel_high_precision_state", p_oplus_adfr_params->panel_high_precision_state);
		OPLUS_ADFR_TRACE_END("oplus_adfr_get_panel_high_precision_state");
		return -EINVAL;
	}
}

static int oplus_adfr_panel_cmd_switch(struct dsi_panel *panel, enum dsi_cmd_set_type *type)
{
	enum dsi_cmd_set_type type_store = *type;
	u32 count;

	/* switch the command when switch to hpwm state */
	if (oplus_panel_pwm_turbo_switch_state(panel) != PWM_SWITCH_DC_STATE) {
		switch (*type) {
		case DSI_CMD_ADFR_MIN_FPS_0:
			*type = DSI_CMD_HPWM_ADFR_MIN_FPS_0;
			break;
		case DSI_CMD_ADFR_MIN_FPS_1:
			*type = DSI_CMD_HPWM_ADFR_MIN_FPS_1;
			break;
		case DSI_CMD_ADFR_MIN_FPS_2:
			*type = DSI_CMD_HPWM_ADFR_MIN_FPS_2;
			break;
		case DSI_CMD_ADFR_MIN_FPS_3:
			*type = DSI_CMD_HPWM_ADFR_MIN_FPS_3;
			break;
		case DSI_CMD_ADFR_MIN_FPS_4:
			*type = DSI_CMD_HPWM_ADFR_MIN_FPS_4;
			break;
		case DSI_CMD_ADFR_MIN_FPS_5:
			*type = DSI_CMD_HPWM_ADFR_MIN_FPS_5;
			break;
		case DSI_CMD_ADFR_MIN_FPS_6:
			*type = DSI_CMD_HPWM_ADFR_MIN_FPS_6;
			break;
		default:
			break;
		}
	}

	count = panel->cur_mode->priv_info->cmd_sets[*type].count;
	if (count == 0) {
		ADFR_DEBUG("[%s] %s is undefined, restore to %s\n",
				panel->oplus_priv.vendor_name,
				cmd_set_prop_map[*type],
				cmd_set_prop_map[type_store]);
		*type = type_store;
	}

	return 0;
}

/* -------------------- standard adfr -------------------- */
int oplus_adfr_parse_dtsi_config(void *dsi_panel, void *dsi_display_mode, void *dsi_parser_utils)
{
	int rc = 0;
	int length = 0;
	unsigned int i = 0;
	unsigned int value = 0;
	struct dsi_panel *panel = dsi_panel;
	struct dsi_display_mode *mode = dsi_display_mode;
	struct dsi_parser_utils *utils = dsi_parser_utils;
	struct dsi_display_mode_priv_info *priv_info = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!panel) {
		ADFR_ERR("invalid panel param\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		ADFR_INFO("no need to parse dtsi config for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return 0;
	}

	if (!mode || !mode->priv_info || !utils) {
		ADFR_ERR("invalid mode or utils params\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_parse_dtsi_config");

	priv_info = mode->priv_info;

	ADFR_INFO("mode_idx:%u,h_active:%u,v_active:%u,refresh_rate:%u,h_skew:%u\n",
				mode->mode_idx, mode->timing.h_active, mode->timing.v_active, mode->timing.refresh_rate, mode->timing.h_skew);

	/* oplus,adfr-fakeframe-config */
	if (oplus_adfr_fakeframe_is_enabled(p_oplus_adfr_params)) {
		/* bit(0):indicates whether fakeframe cmds need to be sent or not if fakeframe is enabled */
		rc = utils->read_u32(utils->data, "oplus,adfr-fakeframe-config", &value);
		if (rc) {
			ADFR_DEBUG("failed to read oplus,adfr-fakeframe-config, rc=%d\n", rc);
			/* set default value to 0 */
			priv_info->oplus_adfr_fakeframe_config = 0;
		} else {
			priv_info->oplus_adfr_fakeframe_config = value;
		}
		ADFR_INFO("oplus_adfr_fakeframe_config:%u\n", priv_info->oplus_adfr_fakeframe_config);
	}

	/* oplus,adfr-idle-off-min-fps */
	if (oplus_adfr_idle_mode_is_enabled(p_oplus_adfr_params)) {
		/* the minimum fps setting which can be set in idle off if idle mode is enabled */
		rc = utils->read_u32(utils->data, "oplus,adfr-idle-off-min-fps", &value);
		if (rc) {
			ADFR_DEBUG("failed to read oplus,adfr-idle-off-min-fps, rc=%d\n", rc);
			/* set default value to 0 */
			priv_info->oplus_adfr_idle_off_min_fps = 0;
		} else {
			priv_info->oplus_adfr_idle_off_min_fps = value;
		}
		ADFR_INFO("oplus_adfr_idle_off_min_fps:%u\n", priv_info->oplus_adfr_idle_off_min_fps);
	}

	/* high precision dtsi parse */
	if (oplus_adfr_high_precision_oa_mode_is_enabled(p_oplus_adfr_params)) {
		/* oplus,adfr-osync-sw-stabilize-frame-config-table */
		length = utils->count_u32_elems(utils->data, "oplus,adfr-osync-sw-stabilize-frame-config-table");
		if (length > 0) {
			priv_info->oplus_adfr_sw_stabilize_frame_config_table = kzalloc(length * sizeof(unsigned int), GFP_KERNEL);
			if (priv_info->oplus_adfr_sw_stabilize_frame_config_table) {
				/* store the adfr osync sw stabilize frame config table supported by each timing, represented by a refresh rate (in hz) from high to low */
				rc = utils->read_u32_array(utils->data, "oplus,adfr-osync-sw-stabilize-frame-config-table", priv_info->oplus_adfr_sw_stabilize_frame_config_table, length);
				if (rc) {
					ADFR_ERR("failed to read oplus,adfr-osync-sw-stabilize-frame-config-table\n");
					rc = 0;
					kfree(priv_info->oplus_adfr_sw_stabilize_frame_config_table);
				} else {
					ADFR_INFO("property:oplus,adfr-osync-sw-stabilize-frame-config-table,length:%u\n", length);
					for (i = 0; i < length; i++) {
						ADFR_INFO("oplus_adfr_sw_stabilize_frame_config_table[%u]=%u\n", i, priv_info->oplus_adfr_sw_stabilize_frame_config_table[i]);
					}
					/* can get maximum and minimum adfr osync sw stabilize frame config by oplus_adfr_sw_stabilize_frame_config_table_count */
					priv_info->oplus_adfr_sw_stabilize_frame_config_table_count = length;
					ADFR_INFO("oplus_adfr_sw_stabilize_frame_config_table_count:%u\n", priv_info->oplus_adfr_sw_stabilize_frame_config_table_count);
					/* oplus,adfr-osync-sw-stabilize-frame-threshold-us */
					rc = utils->read_u32(utils->data, "oplus,adfr-osync-sw-stabilize-frame-threshold-us", &value);
					if (rc) {
						priv_info->oplus_adfr_sw_stabilize_frame_threshold_us = 400;
					} else {
						priv_info->oplus_adfr_sw_stabilize_frame_threshold_us = value;
					}
					ADFR_INFO("oplus_adfr_sw_stabilize_frame_threshold_us:%u\n", priv_info->oplus_adfr_sw_stabilize_frame_threshold_us);
				}
			} else {
				ADFR_ERR("failed to kzalloc oplus_adfr_sw_stabilize_frame_config_table\n");
				rc = -ENOMEM;
				priv_info->oplus_adfr_sw_stabilize_frame_config_table = NULL;
				priv_info->oplus_adfr_sw_stabilize_frame_config_table_count = 0;
			}
		} else {
			ADFR_ERR("failed to get the count of oplus,adfr-osync-sw-stabilize-frame-config-table\n");
			rc = 0;
			priv_info->oplus_adfr_sw_stabilize_frame_config_table = NULL;
			priv_info->oplus_adfr_sw_stabilize_frame_config_table_count = 0;
		}

		/* oplus,adfr-osync-hw-stabilize-frame-config-table */
		length = utils->count_u32_elems(utils->data, "oplus,adfr-osync-hw-stabilize-frame-config-table");
		if (length > 0) {
			priv_info->oplus_adfr_hw_stabilize_frame_config_table = kzalloc(length * sizeof(unsigned int), GFP_KERNEL);
			if (priv_info->oplus_adfr_hw_stabilize_frame_config_table) {
				/* store the adfr osync hw stabilize frame config table supported by each timing, represented by a refresh rate (in hz) from high to low */
				rc = utils->read_u32_array(utils->data, "oplus,adfr-osync-hw-stabilize-frame-config-table", priv_info->oplus_adfr_hw_stabilize_frame_config_table, length);
				if (rc) {
					ADFR_ERR("failed to read oplus,adfr-osync-hw-stabilize-frame-config-table\n");
					rc = 0;
					kfree(priv_info->oplus_adfr_hw_stabilize_frame_config_table);
				} else {
					ADFR_INFO("property:oplus,adfr-osync-hw-stabilize-frame-config-table,length:%u\n", length);
					for (i = 0; i < length; i++) {
						ADFR_INFO("oplus_adfr_hw_stabilize_frame_config_table[%u]=%u\n", i, priv_info->oplus_adfr_hw_stabilize_frame_config_table[i]);
					}
					/* can get maximum and minimum adfr osync hw stabilize frame config by oplus_adfr_hw_stabilize_frame_config_table_count */
					priv_info->oplus_adfr_hw_stabilize_frame_config_table_count = length;
					ADFR_INFO("oplus_adfr_hw_stabilize_frame_config_table_count:%u\n", priv_info->oplus_adfr_hw_stabilize_frame_config_table_count);
				}
			} else {
				ADFR_ERR("failed to kzalloc oplus_adfr_hw_stabilize_frame_config_table\n");
				rc = -ENOMEM;
				priv_info->oplus_adfr_hw_stabilize_frame_config_table = NULL;
				priv_info->oplus_adfr_hw_stabilize_frame_config_table_count = 0;
			}
		} else {
			ADFR_ERR("failed to get the count of oplus,adfr-osync-hw-stabilize-frame-config-table\n");
			rc = 0;
			priv_info->oplus_adfr_hw_stabilize_frame_config_table = NULL;
			priv_info->oplus_adfr_hw_stabilize_frame_config_table_count = 0;
		}
	}

	/* oplus,adfr-high-precision-fps-mapping-table */
	length = utils->count_u32_elems(utils->data, "oplus,adfr-high-precision-fps-mapping-table");
	if (length > 0) {
		priv_info->oplus_adfr_high_precision_fps_mapping_table = kzalloc(length * sizeof(unsigned int), GFP_KERNEL);
		if (priv_info->oplus_adfr_high_precision_fps_mapping_table) {
			/* store the adfr high-precision fps mapping table supported by each timing, represented by a refresh rate (in hz) from high to low */
			rc = utils->read_u32_array(utils->data, "oplus,adfr-high-precision-fps-mapping-table", priv_info->oplus_adfr_high_precision_fps_mapping_table, length);
			if (rc) {
				ADFR_ERR("failed to read oplus,adfr-high-precision-fps-mapping-table\n");
				rc = 0;
				kfree(priv_info->oplus_adfr_high_precision_fps_mapping_table);
			} else {
				ADFR_INFO("property:oplus,adfr-high-precision-fps-mapping-table,length:%u\n", length);
				for (i = 0; i < length; i++) {
					ADFR_INFO("oplus_adfr_high_precision_fps_mapping_table[%u]=%u\n", i, priv_info->oplus_adfr_high_precision_fps_mapping_table[i]);
				}
				/* can get maximum and minimum adfr high precision fps by oplus_adfr_high_precision_fps_mapping_table_count */
				priv_info->oplus_adfr_high_precision_fps_mapping_table_count = length;
				ADFR_INFO("oplus_adfr_high_precision_fps_mapping_table_count:%u\n", priv_info->oplus_adfr_high_precision_fps_mapping_table_count);
			}
		} else {
			ADFR_ERR("failed to kzalloc oplus_adfr_high_precision_fps_mapping_table\n");
			rc = -ENOMEM;
			priv_info->oplus_adfr_high_precision_fps_mapping_table = NULL;
			priv_info->oplus_adfr_high_precision_fps_mapping_table_count = 0;
		}
	} else {
		ADFR_ERR("failed to get the count of oplus,adfr-high-precision-fps-mapping-table\n");
		rc = 0;
		priv_info->oplus_adfr_high_precision_fps_mapping_table = NULL;
		priv_info->oplus_adfr_high_precision_fps_mapping_table_count = 0;
	}

	/* oplus,adfr-min-fps-mapping-table */
	length = utils->count_u32_elems(utils->data, "oplus,adfr-min-fps-mapping-table");
	if (length < 0) {
		ADFR_ERR("failed to get the count of oplus,adfr-min-fps-mapping-table\n");
		rc = 0;
		goto reset;
	}
	priv_info->oplus_adfr_min_fps_mapping_table = kzalloc(length * sizeof(unsigned int), GFP_KERNEL);
	if (!priv_info->oplus_adfr_min_fps_mapping_table) {
		ADFR_ERR("failed to kzalloc oplus_adfr_min_fps_mapping_table\n");
		rc = -ENOMEM;
		goto reset;
	}
	/* store the adfr min fps mapping table supported by each timing, represented by a refresh rate (in hz) from high to low */
	rc = utils->read_u32_array(utils->data, "oplus,adfr-min-fps-mapping-table", priv_info->oplus_adfr_min_fps_mapping_table, length);
	if (rc) {
		ADFR_ERR("failed to read oplus,adfr-min-fps-mapping-table\n");
		rc = 0;
		goto free_oplus_adfr_min_fps_mapping_table;
	}
	ADFR_INFO("property:oplus,adfr-min-fps-mapping-table,length:%u\n", length);
	for (i = 0; i < length; i++) {
		ADFR_INFO("oplus_adfr_min_fps_mapping_table[%u]=%u\n", i, priv_info->oplus_adfr_min_fps_mapping_table[i]);
	}
	/* can get maximum and minimum adfr min fps by oplus_adfr_min_fps_mapping_table_count */
	priv_info->oplus_adfr_min_fps_mapping_table_count = length;
	ADFR_INFO("oplus_adfr_min_fps_mapping_table_count:%u\n", priv_info->oplus_adfr_min_fps_mapping_table_count);

	goto end;

free_oplus_adfr_min_fps_mapping_table:
	kfree(priv_info->oplus_adfr_min_fps_mapping_table);
reset:
	priv_info->oplus_adfr_min_fps_mapping_table = NULL;
	priv_info->oplus_adfr_min_fps_mapping_table_count = 0;
end:
	OPLUS_ADFR_TRACE_END("oplus_adfr_parse_dtsi_config");

	ADFR_DEBUG("end\n");

	return rc;
}

/* cmd set */
static int oplus_adfr_panel_cmd_set_nolock(void *dsi_panel, enum dsi_cmd_set_type type)
{
	int rc = 0;
	struct dsi_panel *panel = dsi_panel;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!panel) {
		ADFR_ERR("invalid panel param\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		ADFR_INFO("no need to set panel cmd for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_panel_cmd_set_nolock");

	if (!dsi_panel_initialized(panel)) {
		rc = -EINVAL;
		ADFR_DEBUG("should not send cmd sets if panel is not initialized\n");
		goto error;
	}

	if (!oplus_adfr_dry_run_is_enabled(p_oplus_adfr_params)) {
		if (oplus_adfr_high_precision_switch_is_enabled(p_oplus_adfr_params)) {
			oplus_adfr_panel_cmd_switch(panel, &type);
		}
		OPLUS_ADFR_TRACE_BEGIN("dsi_panel_tx_cmd_set");
		rc = dsi_panel_tx_cmd_set(panel, type);
		OPLUS_ADFR_TRACE_END("dsi_panel_tx_cmd_set");
		if (rc) {
			ADFR_ERR("[%s] failed to send %s, rc=%d\n",
				panel->name, cmd_set_prop_map[type], rc);
			goto error;
		}
	}

	/* after tx cmd set */
	switch (type) {
	case DSI_CMD_ADFR_AUTO_ON:
		OPLUS_ADFR_TRACE_INT("oplus_adfr_auto_mode_cmd", OPLUS_ADFR_AUTO_ON);
		break;

	case DSI_CMD_ADFR_AUTO_OFF:
		OPLUS_ADFR_TRACE_INT("oplus_adfr_auto_mode_cmd", OPLUS_ADFR_AUTO_OFF);

		/* after auto off cmd was sent, auto on cmd filter start */
		p_oplus_adfr_params->need_filter_auto_on_cmd = true;
		ADFR_DEBUG("oplus_adfr_need_filter_auto_on_cmd:%d\n", p_oplus_adfr_params->need_filter_auto_on_cmd);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_need_filter_auto_on_cmd", p_oplus_adfr_params->need_filter_auto_on_cmd);
		break;
	case DSI_CMD_ADFR_HIGH_PRECISION_FPS_0:
	case DSI_CMD_HPWM_ADFR_HIGH_PRECISION_FPS_0:
		p_oplus_adfr_params->high_precision_state = OPLUS_ADFR_HIGH_PRECISION_FPS_120;
		OPLUS_ADFR_TRACE_INT("oplus_adfr_high_precision_panel_cmd_set_nolock", type);
		break;
	case DSI_CMD_ADFR_HIGH_PRECISION_FPS_1:
	case DSI_CMD_HPWM_ADFR_HIGH_PRECISION_FPS_1:
		p_oplus_adfr_params->high_precision_state = OPLUS_ADFR_HIGH_PRECISION_FPS_90;
		OPLUS_ADFR_TRACE_INT("oplus_adfr_high_precision_panel_cmd_set_nolock", type);
		break;
	case DSI_CMD_ADFR_HIGH_PRECISION_FPS_2:
	case DSI_CMD_HPWM_ADFR_HIGH_PRECISION_FPS_2:
		p_oplus_adfr_params->high_precision_state = OPLUS_ADFR_HIGH_PRECISION_FPS_72;
		OPLUS_ADFR_TRACE_INT("oplus_adfr_high_precision_panel_cmd_set_nolock", type);
		break;
	case DSI_CMD_ADFR_HIGH_PRECISION_FPS_3:
	case DSI_CMD_HPWM_ADFR_HIGH_PRECISION_FPS_3:
		p_oplus_adfr_params->high_precision_state = OPLUS_ADFR_HIGH_PRECISION_FPS_60;
		OPLUS_ADFR_TRACE_INT("oplus_adfr_high_precision_panel_cmd_set_nolock", type);
		break;
	default:
		break;
	}

error:
	OPLUS_ADFR_TRACE_END("oplus_adfr_panel_cmd_set_nolock");

	ADFR_DEBUG("end\n");

	return rc;
}

static int oplus_adfr_panel_cmd_set(void *dsi_panel, enum dsi_cmd_set_type type)
{
	int rc = 0;
	struct dsi_panel *panel = dsi_panel;

	ADFR_DEBUG("start\n");

	if (!panel) {
		ADFR_ERR("invalid panel param\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		ADFR_INFO("no need to set panel cmd for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_panel_cmd_set");

	mutex_lock(&panel->panel_lock);

	if (!dsi_panel_initialized(panel)) {
		rc = -EINVAL;
		ADFR_DEBUG("should not send cmd sets if panel is not initialized\n");
		goto error;
	}

	rc = oplus_adfr_panel_cmd_set_nolock(panel, type);
	if (rc) {
		ADFR_ERR("[%s] failed to send %s, rc=%d\n",
			panel->name, cmd_set_prop_map[type], rc);
	}

error:
	mutex_unlock(&panel->panel_lock);

	OPLUS_ADFR_TRACE_END("oplus_adfr_panel_cmd_set");

	ADFR_DEBUG("end\n");

	return rc;
}

/* uniform interface for cmd set */
static int oplus_adfr_display_cmd_set(void *dsi_display, enum dsi_cmd_set_type type)
{
	int rc = 0;
	struct dsi_display *display = dsi_display;

	ADFR_DEBUG("start\n");

	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to set display cmd for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_display_cmd_set");

	mutex_lock(&display->display_lock);

	/* enable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
			DSI_CORE_CLK, DSI_CLK_ON);
		if (rc) {
			ADFR_ERR("[%s] failed to enable DSI clocks, rc=%d\n", display->name, rc);
			goto error;
		}
	}

	rc = oplus_adfr_panel_cmd_set(display->panel, type);
	if (rc) {
		ADFR_ERR("[%s] failed to send %s, rc=%d\n",
			display->name, cmd_set_prop_map[type], rc);
	}

	/* disable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
			DSI_CORE_CLK, DSI_CLK_OFF);
		if (rc) {
			ADFR_ERR("[%s] failed to disable DSI clocks, rc=%d\n", display->name, rc);
		}
	}

error:
	mutex_unlock(&display->display_lock);

	OPLUS_ADFR_TRACE_END("oplus_adfr_display_cmd_set");

	ADFR_DEBUG("end\n");

	return rc;
}

/* distinguish modes by skew for adfr */
bool oplus_adfr_h_skew_is_different(void *dsi_display, void *dsi_display_mode_0, void *dsi_display_mode_1)
{
	bool rc = false;
	struct dsi_display *display = dsi_display;
	struct dsi_display_mode *cmp = dsi_display_mode_0;
	struct dsi_display_mode *m = dsi_display_mode_1;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return false;
	}

	if (!cmp || !m) {
		ADFR_ERR("invalid dsi_display_mode params\n");
		return false;
	}

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return false;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return false;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_h_skew_is_different");

	if (cmp->timing.h_skew != m->timing.h_skew) {
		ADFR_DEBUG("h_skew is different\n");
		rc = true;
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_h_skew_is_different");

	ADFR_DEBUG("end\n");

	return rc;
}

/* handle CONNECTOR_PROP_ADFR_MIN_FPS property value */
int oplus_adfr_property_update(void *sde_connector, void *sde_connector_state, int prop_id, uint64_t prop_val)
{
	unsigned int handled = 0;
	struct sde_connector *c_conn = sde_connector;
	struct sde_connector_state *c_state = sde_connector_state;
	struct dsi_display *display = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!c_conn || !c_state) {
		ADFR_ERR("invalid input params\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		ADFR_DEBUG("not in dsi mode, should not update adfr properties\n");
		return 0;
	}

	display = c_conn->display;
	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to update properties for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return 0;
	}

	if (!display->panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_property_update");

	switch (prop_id) {
	case CONNECTOR_PROP_ADFR_MIN_FPS:
		/* minfps maybe disappear after state change, so handle it early */
		ADFR_DEBUG("CONNECTOR_PROP_ADFR_MIN_FPS:0x%08x\n", prop_val);

		if (prop_val & OPLUS_ADFR_SA_MAGIC) {
			handled = BIT(0);

			if (prop_val & OPLUS_ADFR_AUTO_MODE_MAGIC) {
				if (p_oplus_adfr_params->need_filter_auto_on_cmd
						&& (OPLUS_ADFR_AUTO_MODE_VALUE(prop_val) == OPLUS_ADFR_AUTO_ON)) {
					ADFR_INFO("auto off cmds and auto on cmds could not be sent in the same frame, filter it out\n");
					handled |= BIT(1);
					goto end;
				} else if (OPLUS_ADFR_AUTO_MODE_VALUE(prop_val) == OPLUS_ADFR_AUTO_IDLE) {
					/* min fps value is sw fps if auto idle is set */
					if (prop_val & OPLUS_ADFR_SA_MIN_FPS_MAGIC) {
						if (display->panel->cur_mode->timing.refresh_rate == 144) {
							p_oplus_adfr_params->sw_fps = 144 / (120 / OPLUS_ADFR_SA_MIN_FPS_VALUE(prop_val));
						} else {
							p_oplus_adfr_params->sw_fps = OPLUS_ADFR_SA_MIN_FPS_VALUE(prop_val);
						}
						/* fakeframe need to be updated */
						p_oplus_adfr_params->fakeframe_updated = true;
					} else {
						ADFR_ERR("failed to update sw fps because of the sa min fps magic error,prop_val=0x%08x\n", prop_val);
					}
					handled |= BIT(2);
					goto end;
				} else if (OPLUS_ADFR_AUTO_MODE_VALUE(prop_val) != p_oplus_adfr_params->auto_mode) {
					p_oplus_adfr_params->auto_mode = OPLUS_ADFR_AUTO_MODE_VALUE(prop_val);
					/* filter repeat auto mode setting */
					p_oplus_adfr_params->auto_mode_updated = true;
					/* when auto mode changes, write the corresponding min fps again */
					p_oplus_adfr_params->sa_min_fps_updated = true;
					handled |= BIT(3);
				}
			}

			if (prop_val & OPLUS_ADFR_FAKEFRAME_MAGIC) {
				if (OPLUS_ADFR_FAKEFRAME_VALUE(prop_val) != p_oplus_adfr_params->fakeframe) {
					/* no need to get fakeframe value as fakeframe is control by kerenl driver */
					handled |= BIT(4);
				}
			}

			if (prop_val & OPLUS_ADFR_SA_MIN_FPS_MAGIC) {
				if (display->panel->cur_mode->timing.refresh_rate == 144) {
					/*
					 in 144hz timing, sf is still use 120hz computational formula to calculate minfps value,
					 and minfps only takes 7 bits so that the max value is 127,
					 so it should be converted back to 144hz minfps value in kernel
					*/
					if ((144 / (120 / OPLUS_ADFR_SA_MIN_FPS_VALUE(prop_val))) != p_oplus_adfr_params->sa_min_fps) {
						p_oplus_adfr_params->sa_min_fps = 144 / (120 / OPLUS_ADFR_SA_MIN_FPS_VALUE(prop_val));
						p_oplus_adfr_params->sa_min_fps_updated = true;
						handled |= BIT(5);
					}
				} else {
					if (OPLUS_ADFR_SA_MIN_FPS_VALUE(prop_val) != p_oplus_adfr_params->sa_min_fps) {
						p_oplus_adfr_params->sa_min_fps = OPLUS_ADFR_SA_MIN_FPS_VALUE(prop_val);
						p_oplus_adfr_params->sa_min_fps_updated = true;
						handled |= BIT(5);
					}
				}
			}

			end:
			/* latest setting */
			ADFR_INFO("auto_mode:%u[%d],fakeframe:%u[%d],sa_min_fps:%u[%d],sw_fps:%u,handled:0x%02x\n",
						p_oplus_adfr_params->auto_mode, p_oplus_adfr_params->auto_mode_updated,
							p_oplus_adfr_params->fakeframe, p_oplus_adfr_params->fakeframe_updated,
								p_oplus_adfr_params->sa_min_fps, p_oplus_adfr_params->sa_min_fps_updated,
									p_oplus_adfr_params->sw_fps, handled);
			OPLUS_ADFR_TRACE_INT("oplus_adfr_auto_mode", p_oplus_adfr_params->auto_mode);
			OPLUS_ADFR_TRACE_INT("oplus_adfr_auto_mode_updated", p_oplus_adfr_params->auto_mode_updated);
			OPLUS_ADFR_TRACE_INT("oplus_adfr_fakeframe", p_oplus_adfr_params->fakeframe);
			OPLUS_ADFR_TRACE_INT("oplus_adfr_fakeframe_updated", p_oplus_adfr_params->fakeframe_updated);
			OPLUS_ADFR_TRACE_INT("oplus_adfr_sa_min_fps", p_oplus_adfr_params->sa_min_fps);
			OPLUS_ADFR_TRACE_INT("oplus_adfr_sa_min_fps_updated", p_oplus_adfr_params->sa_min_fps_updated);
			OPLUS_ADFR_TRACE_INT("oplus_adfr_sw_fps", p_oplus_adfr_params->sw_fps);
			OPLUS_ADFR_TRACE_INT("oplus_adfr_handled", handled);
		}

		msm_property_set_dirty(&c_conn->property_info, &c_state->property_state, prop_id);
		break;

	default:
		break;
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_property_update");

	ADFR_DEBUG("end\n");

	return handled;
}

/* handle irq function */
int oplus_adfr_irq_handler(void *sde_encoder_phys, unsigned int irq_type)
{
	/* The initial value is 0, but we don't care about the first calculation error */
	static unsigned long last_rd_ptr_timestamp_us = 0;
	static unsigned long last_wr_ptr_timestamp_us = 0;
	static unsigned long rd_ptr_timestamp_us = 0;
	static unsigned long wr_ptr_timestamp_us = 0;
	static unsigned int last_panel_high_precision_state = 0;
	struct sde_encoder_phys *phys_enc = sde_encoder_phys;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!phys_enc || !phys_enc->connector) {
		ADFR_ERR("invalid phys_enc params\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(phys_enc->connector);
	if (!c_conn) {
		ADFR_ERR("invalid c_conn param\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		ADFR_DEBUG("not in dsi mode, should not handle irq function\n");
		return 0;
	}

	display = c_conn->display;
	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to handle irq function for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_irq_handler");

	if (irq_type == OPLUS_ADFR_RD_PTR) {
		ADFR_DEBUG("rd_ptr_irq interval:%lu\n", ((unsigned long)ktime_to_us(ktime_get()) - rd_ptr_timestamp_us));
		last_rd_ptr_timestamp_us = rd_ptr_timestamp_us;
		rd_ptr_timestamp_us = (unsigned long)ktime_to_us(ktime_get());
		ADFR_DEBUG("rd_ptr_timestamp_us:%u, last_rd_ptr_timestamp_us:%u\n", rd_ptr_timestamp_us, last_rd_ptr_timestamp_us);
		OPLUS_ADFR_TRACE_INT("rd_ptr_timestamp_us", rd_ptr_timestamp_us);
		OPLUS_ADFR_TRACE_INT("last_rd_ptr_timestamp_us", last_rd_ptr_timestamp_us);

		/* high precision cmds are taking effect in panel module */
		if (last_panel_high_precision_state != p_oplus_adfr_params->high_precision_state) {
			p_oplus_adfr_params->panel_high_precision_state = p_oplus_adfr_params->high_precision_state;
			ADFR_DEBUG("oplus_adfr_panel_high_precision_status_update:%d\n", p_oplus_adfr_params->panel_high_precision_state);
			OPLUS_ADFR_TRACE_INT("oplus_adfr_panel_high_precision_state", p_oplus_adfr_params->panel_high_precision_state);
		}
		last_panel_high_precision_state = p_oplus_adfr_params->high_precision_state;

		/* when the rd_ptr_irq comes there is no need to filter auto on cmds anymore */
		if (p_oplus_adfr_params->need_filter_auto_on_cmd) {
			p_oplus_adfr_params->need_filter_auto_on_cmd = false;
			ADFR_DEBUG("oplus_adfr_need_filter_auto_on_cmd:%d\n", p_oplus_adfr_params->need_filter_auto_on_cmd);
			OPLUS_ADFR_TRACE_INT("oplus_adfr_need_filter_auto_on_cmd", p_oplus_adfr_params->need_filter_auto_on_cmd);
		}

		if (p_oplus_adfr_params->osync_frame_status == OPLUS_ADFR_PP_DONE) {
			p_oplus_adfr_params->osync_frame_status = OPLUS_ADFR_RD_PTR;
			ADFR_DEBUG("oplus_adfr_osync_frame_status:%u\n", p_oplus_adfr_params->osync_frame_status);
			OPLUS_ADFR_TRACE_INT("oplus_adfr_osync_frame_status", p_oplus_adfr_params->osync_frame_status);
		}

		if (oplus_adfr_oa_bl_mutual_exclusion_is_enabled(p_oplus_adfr_params)) {
			/* when the rd_ptr_irq comes there is no need to filter osync backlight cmd anymore */
			if (p_oplus_adfr_params->need_filter_osync_backlight_cmd) {
				p_oplus_adfr_params->need_filter_osync_backlight_cmd = false;
				ADFR_DEBUG("oplus_adfr_need_filter_osync_backlight_cmd:%d\n", p_oplus_adfr_params->need_filter_osync_backlight_cmd);
				OPLUS_ADFR_TRACE_INT("oplus_adfr_need_filter_osync_backlight_cmd", p_oplus_adfr_params->need_filter_osync_backlight_cmd);
			}
		}
	} else if (irq_type == OPLUS_ADFR_WD_PTR) {
		ADFR_DEBUG("wr_ptr_irq interval:%lu\n", ((unsigned long)ktime_to_us(ktime_get()) - wr_ptr_timestamp_us));
		last_wr_ptr_timestamp_us = wr_ptr_timestamp_us;
		wr_ptr_timestamp_us = (unsigned long)ktime_to_us(ktime_get());

		p_oplus_adfr_params->osync_frame_status = OPLUS_ADFR_WD_PTR;
		ADFR_DEBUG("oplus_adfr_osync_frame_status:%u\n", p_oplus_adfr_params->osync_frame_status);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_osync_frame_status", p_oplus_adfr_params->osync_frame_status);
		ADFR_DEBUG("wr_ptr_timestamp_us:%u, last_wr_ptr_timestamp_us:%u\n", wr_ptr_timestamp_us, last_wr_ptr_timestamp_us);
		OPLUS_ADFR_TRACE_INT("current_wr_ptr_timestamp_us", wr_ptr_timestamp_us);
		OPLUS_ADFR_TRACE_INT("last_wr_ptr_timestamp_us", last_wr_ptr_timestamp_us);
	} else if (irq_type == OPLUS_ADFR_PP_DONE) {
		p_oplus_adfr_params->osync_frame_status = OPLUS_ADFR_PP_DONE;
		ADFR_DEBUG("oplus_adfr_osync_frame_status:%u\n", p_oplus_adfr_params->osync_frame_status);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_osync_frame_status", p_oplus_adfr_params->osync_frame_status);
	}

	p_oplus_adfr_params->current_wr_rd_irq_interval = abs(wr_ptr_timestamp_us - rd_ptr_timestamp_us);
	p_oplus_adfr_params->last_wr_rd_irq_interval = abs(last_wr_ptr_timestamp_us - last_rd_ptr_timestamp_us);
	OPLUS_ADFR_TRACE_INT("current_wr_rd_irq_interval", p_oplus_adfr_params->current_wr_rd_irq_interval);
	OPLUS_ADFR_TRACE_INT("last_wr_rd_irq_interval", p_oplus_adfr_params->last_wr_rd_irq_interval);
	OPLUS_ADFR_TRACE_END("oplus_adfr_irq_handler");

	ADFR_DEBUG("end\n");

	return 0;
}

static int oplus_adfr_auto_mode_update(void *dsi_display, unsigned int auto_mode)
{
	int rc = 0;
	struct dsi_display *display = dsi_display;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to update auto mode for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_auto_mode_update");

	if (auto_mode) {
		/* send the commands to enable auto mode */
		rc = oplus_adfr_display_cmd_set(display, DSI_CMD_ADFR_AUTO_ON);
		if (rc) {
			ADFR_ERR("[%s] failed to send DSI_CMD_ADFR_AUTO_ON cmds, rc=%d\n", display->name, rc);
		}
	} else {
		/* send the commands to disbale auto mode */
		rc = oplus_adfr_display_cmd_set(display, DSI_CMD_ADFR_AUTO_OFF);
		if (rc) {
			ADFR_ERR("[%s] failed to send DSI_CMD_ADFR_AUTO_OFF cmds, rc=%d\n", display->name, rc);
		}
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_auto_mode_update");

	ADFR_DEBUG("end\n");

	return rc;
}

/* prevent the wrong min fps setting */
static int oplus_adfr_min_fps_check(void *dsi_panel, unsigned int min_fps)
{
	unsigned char min_fps_mapping_table_count = 0;
	unsigned int refresh_rate = 120;
	unsigned int h_skew = STANDARD_ADFR;
	unsigned int idle_off_min_fps = 0;
	struct dsi_panel *panel = dsi_panel;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!panel) {
		ADFR_ERR("invalid panel param\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		ADFR_INFO("no need to check min fps for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return 0;
	}

	if (!panel->cur_mode || !panel->cur_mode->priv_info) {
		ADFR_ERR("invalid cur_mode params\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_min_fps_check");

	refresh_rate = panel->cur_mode->timing.refresh_rate;
	h_skew = panel->cur_mode->timing.h_skew;
	min_fps_mapping_table_count = panel->cur_mode->priv_info->oplus_adfr_min_fps_mapping_table_count;
	idle_off_min_fps = panel->cur_mode->priv_info->oplus_adfr_idle_off_min_fps;
	ADFR_DEBUG("refresh_rate:%u,h_skew:%u,min_fps_mapping_table_count:%u,idle_off_min_fps:%u\n",
					refresh_rate, h_skew, min_fps_mapping_table_count, idle_off_min_fps);

	if (!min_fps_mapping_table_count) {
		/* fixed max min fps */
		min_fps = refresh_rate;
	} else if ((h_skew == OPLUS_ADFR) && !min_fps) {
		/* no need to send min fps cmds in oa mode */
		min_fps = 0;
	} else if ((min_fps > panel->cur_mode->priv_info->oplus_adfr_min_fps_mapping_table[0])
					|| (min_fps < panel->cur_mode->priv_info->oplus_adfr_min_fps_mapping_table[min_fps_mapping_table_count - 1])) {
		/* the highest frame rate is the most stable */
		min_fps = panel->cur_mode->priv_info->oplus_adfr_min_fps_mapping_table[0];
	} else if (oplus_adfr_idle_mode_is_enabled(p_oplus_adfr_params)
					&& idle_off_min_fps
					&& (p_oplus_adfr_params->auto_mode == OPLUS_ADFR_AUTO_OFF)
					&& (p_oplus_adfr_params->idle_mode == OPLUS_ADFR_IDLE_OFF)
					&& (min_fps < idle_off_min_fps)) {
		/* the refresh rate is reduced step by step. before entering mipi idle, the min fps could not be less than idle_off_min_fps */
		min_fps = idle_off_min_fps;
	}

	ADFR_DEBUG("min fps is %u after check\n", min_fps);

	OPLUS_ADFR_TRACE_END("oplus_adfr_min_fps_check");

	ADFR_DEBUG("end\n");

	return min_fps;
}

static int oplus_adfr_min_fps_update(void *dsi_display, unsigned int min_fps)
{
	int rc = 0;
	unsigned int i = 0;
	struct dsi_display *display = dsi_display;
	struct dsi_display_mode_priv_info *priv_info = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to update min fps for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return 0;
	}

	if (!display->panel->cur_mode || !display->panel->cur_mode->priv_info) {
		ADFR_ERR("invalid panel params\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_min_fps_update");

	priv_info = display->panel->cur_mode->priv_info;

	/* check minfps */
	min_fps = oplus_adfr_min_fps_check(display->panel, min_fps);
	if (min_fps <= 0) {
		rc = min_fps;
		ADFR_ERR("failed to check min fps, rc=%d\n", rc);
		goto end;
	}

	/* find the min fps mapping cmd set */
	if (!priv_info->oplus_adfr_min_fps_mapping_table_count) {
		ADFR_DEBUG("fixed max min fps setting\n");
		goto end;
	} else {
		for (i = 0; i < priv_info->oplus_adfr_min_fps_mapping_table_count - 1; i++) {
			if ((min_fps <= priv_info->oplus_adfr_min_fps_mapping_table[i])
					&& (min_fps > priv_info->oplus_adfr_min_fps_mapping_table[i + 1])) {
				break;
			}
		}
	}

	/* send the commands to set min fps */
	if (oplus_panel_pwm_turbo_is_enabled(display->panel)) {
		rc = oplus_adfr_display_cmd_set(display, DSI_CMD_HPWM_ADFR_MIN_FPS_0 + i);
		if (rc) {
			ADFR_ERR("[%s] failed to send DSI_CMD_HPWM_ADFR_MIN_FPS_%d cmds, rc=%d\n", display->name, i, rc);
		}
	} else {
		rc = oplus_adfr_display_cmd_set(display, DSI_CMD_ADFR_MIN_FPS_0 + i);
		if (rc) {
			ADFR_ERR("[%s] failed to send DSI_CMD_ADFR_MIN_FPS_%d cmds, rc=%d\n", display->name, i, rc);
		}
	}

	ADFR_DEBUG("oplus_adfr_min_fps_cmd:%u\n", min_fps);
	OPLUS_ADFR_TRACE_INT("oplus_adfr_min_fps_cmd", min_fps);

end:
	OPLUS_ADFR_TRACE_END("oplus_adfr_min_fps_update");

	ADFR_DEBUG("end\n");

	return rc;
}

/* some panel should resend sa cmd when finished booting, otherwise sa mode cannot take effect */
int oplus_adfr_sa_mode_restore(void *dsi_display)
{
	static bool primary_panel_restored = false;
	static bool secondary_panel_retored = false;
	int rc = 0;
	unsigned int h_skew = STANDARD_ADFR;
	unsigned int refresh_rate = 120;
	struct dsi_display *display = dsi_display;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to restore sa mode for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_sa_mode_restore_is_enabled(p_oplus_adfr_params)) {
		ADFR_DEBUG("sa mode restore is not enabled\n");
		return 0;
	}

	if (!display->panel->cur_mode) {
		ADFR_ERR("invalid panel params\n");
		return -EINVAL;
	}

	if (((!strcmp(display->display_type, "primary")) && primary_panel_restored)
			|| ((!strcmp(display->display_type, "secondary")) && secondary_panel_retored)) {
		ADFR_DEBUG("already restored sa mode\n");
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_sa_mode_restore");

	h_skew = display->panel->cur_mode->timing.h_skew;
	refresh_rate = display->panel->cur_mode->timing.refresh_rate;

	if ((h_skew == STANDARD_ADFR) || (h_skew == STANDARD_MFR)) {
		rc = oplus_adfr_auto_mode_update(display, OPLUS_ADFR_AUTO_OFF);
		if (rc) {
			ADFR_ERR("failed to update auto mode, rc=%d\n", rc);
		}

		rc = oplus_adfr_min_fps_update(display, refresh_rate);
		if (rc) {
			ADFR_ERR("failed to update sa min fps, rc=%d\n", rc);
		}

		if (!strcmp(display->display_type, "primary")) {
			primary_panel_restored = true;
		} else if (!strcmp(display->display_type, "secondary")) {
			secondary_panel_retored = true;
		}

		ADFR_INFO("restore sa mode\n");
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_sa_mode_restore");

	ADFR_DEBUG("end\n");

	return rc;
}

int oplus_adfr_sa_handle(void *sde_encoder_virt)
{
	int rc = 0;
	unsigned int h_skew = STANDARD_ADFR;
	struct sde_encoder_virt *sde_enc = sde_encoder_virt;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!sde_enc || !sde_enc->cur_master || !sde_enc->cur_master->connector) {
		ADFR_ERR("invalid sde_enc params\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(sde_enc->cur_master->connector);
	if (!c_conn) {
		ADFR_ERR("invalid c_conn param\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		ADFR_DEBUG("not in dsi mode, should not handle sa\n");
		return 0;
	}

	display = c_conn->display;
	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to handle sa for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return 0;
	}

	if (!display->panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return -EINVAL;
	}

	/* auto mode, fakeframe and min fps are available only after power on */
	if (display->panel->power_mode != SDE_MODE_DPMS_ON) {
		ADFR_DEBUG("should not handle sa when power mode is %u\n", display->panel->power_mode);
		return 0;
	}

	h_skew = display->panel->cur_mode->timing.h_skew;

	if ((h_skew != STANDARD_ADFR) && (h_skew != STANDARD_MFR)) {
		ADFR_DEBUG("should not handle sa in oa mode\n");
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_sa_handle");

	if (p_oplus_adfr_params->auto_mode_updated) {
		rc = oplus_adfr_auto_mode_update(display, p_oplus_adfr_params->auto_mode);
		if (rc) {
			ADFR_ERR("failed to update auto mode, rc=%d\n", rc);
		}
		p_oplus_adfr_params->auto_mode_updated = false;
		ADFR_DEBUG("oplus_adfr_auto_mode_updated:%d\n", p_oplus_adfr_params->auto_mode_updated);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_auto_mode_updated", p_oplus_adfr_params->auto_mode_updated);
	}

	if (p_oplus_adfr_params->fakeframe_updated) {
		/* update fakeframe status after sa handle */
		rc = oplus_adfr_fakeframe_status_update(display->panel, false);
		if (rc) {
			ADFR_ERR("failed to update fakeframe status, rc=%d\n", rc);
		}
		p_oplus_adfr_params->fakeframe_updated = false;
		ADFR_DEBUG("oplus_adfr_fakeframe_updated:%d\n", p_oplus_adfr_params->fakeframe_updated);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_fakeframe_updated", p_oplus_adfr_params->fakeframe_updated);
	}

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	/* fixed max min fps can be set in hbm on, and update it after hbm off */
	if (oplus_ofp_is_supported() && !oplus_ofp_oled_capacitive_is_enabled()
			&& !oplus_ofp_local_hbm_is_enabled() && !oplus_ofp_ultrasonic_is_enabled()) {
		if (p_oplus_adfr_params->sa_min_fps_updated && !oplus_ofp_get_hbm_state()) {
			if (p_oplus_adfr_params->skip_min_fps_setting) {
				ADFR_INFO("skip min fps %u setting\n", p_oplus_adfr_params->sa_min_fps);
			} else {
				rc = oplus_adfr_min_fps_update(display, p_oplus_adfr_params->sa_min_fps);
				if (rc) {
					ADFR_ERR("failed to update sa min fps, rc=%d\n", rc);
				}
			}
			p_oplus_adfr_params->sa_min_fps_updated = false;
			ADFR_DEBUG("oplus_adfr_sa_min_fps_updated:%d\n", p_oplus_adfr_params->sa_min_fps_updated);
			OPLUS_ADFR_TRACE_INT("oplus_adfr_sa_min_fps_updated", p_oplus_adfr_params->sa_min_fps_updated);
		}
	} else
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	 {
		if (p_oplus_adfr_params->sa_min_fps_updated) {
			if (p_oplus_adfr_params->skip_min_fps_setting) {
				ADFR_INFO("skip min fps %u setting\n", p_oplus_adfr_params->sa_min_fps);
			} else {
				rc = oplus_adfr_min_fps_update(display, p_oplus_adfr_params->sa_min_fps);
				if (rc) {
					ADFR_ERR("failed to update sa min fps, rc=%d\n", rc);
				}
			}
			p_oplus_adfr_params->sa_min_fps_updated = false;
			ADFR_DEBUG("oplus_adfr_sa_min_fps_updated:%d\n", p_oplus_adfr_params->sa_min_fps_updated);
			OPLUS_ADFR_TRACE_INT("oplus_adfr_sa_min_fps_updated", p_oplus_adfr_params->sa_min_fps_updated);
		}
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_sa_handle");

	ADFR_DEBUG("end\n");

	return rc;
}

/* reset adfr status as panel power on or timing switch */
int oplus_adfr_status_reset(void *dsi_panel)
{
	int rc = 0;
	unsigned int h_skew = STANDARD_ADFR;
	unsigned int refresh_rate = 120;
	struct dsi_panel *panel = dsi_panel;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!panel || !panel->cur_mode) {
		ADFR_ERR("invalid panel params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		ADFR_INFO("no need to reset adfr status for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_status_reset");

	h_skew = panel->cur_mode->timing.h_skew;
	refresh_rate = panel->cur_mode->timing.refresh_rate;

	if ((h_skew == STANDARD_ADFR) || (h_skew == STANDARD_MFR)) {
		p_oplus_adfr_params->auto_mode = OPLUS_ADFR_AUTO_OFF;
		/* after auto off cmd was sent, auto on cmd filter start */
		p_oplus_adfr_params->need_filter_auto_on_cmd = true;
		ADFR_DEBUG("oplus_adfr_need_filter_auto_on_cmd:%d\n", p_oplus_adfr_params->need_filter_auto_on_cmd);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_need_filter_auto_on_cmd", p_oplus_adfr_params->need_filter_auto_on_cmd);

		rc = oplus_adfr_fakeframe_status_update(panel, false);
		if (rc) {
			ADFR_ERR("failed to update fakeframe status, rc=%d\n", rc);
		}

		p_oplus_adfr_params->sa_min_fps = refresh_rate;

		if (oplus_adfr_high_precision_sa_mode_is_enabled(p_oplus_adfr_params)) {
			p_oplus_adfr_params->sa_high_precision_fps = refresh_rate;
			ADFR_INFO("sa status reset: auto_mode:%u,fakeframe:%u,sa_min_fps:%u,sa_high_precision_fps:%u\n",
					p_oplus_adfr_params->auto_mode, p_oplus_adfr_params->fakeframe, p_oplus_adfr_params->sa_min_fps, p_oplus_adfr_params->sa_high_precision_fps);
		} else {
			ADFR_INFO("sa status reset: auto_mode:%u,fakeframe:%u,sa_min_fps:%u\n",
					p_oplus_adfr_params->auto_mode, p_oplus_adfr_params->fakeframe, p_oplus_adfr_params->sa_min_fps);
		}

		OPLUS_ADFR_TRACE_INT("oplus_adfr_auto_mode", p_oplus_adfr_params->auto_mode);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_fakeframe", p_oplus_adfr_params->fakeframe);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_sa_min_fps", p_oplus_adfr_params->sa_min_fps);

		OPLUS_ADFR_TRACE_INT("oplus_adfr_auto_mode_cmd", p_oplus_adfr_params->auto_mode);
	} else {
		if (oplus_adfr_high_precision_oa_mode_is_enabled(p_oplus_adfr_params)) {
			p_oplus_adfr_params->oa_high_precision_fps = refresh_rate;
			ADFR_INFO("oa status reset: oa_high_precision_fps:%u\n", p_oplus_adfr_params->oa_high_precision_fps);
			OPLUS_ADFR_TRACE_INT("oplus_adfr_oa_high_precision_fps", p_oplus_adfr_params->oa_high_precision_fps);
		}
		OPLUS_ADFR_TRACE_INT("oplus_adfr_auto_mode_cmd", OPLUS_ADFR_AUTO_OFF);
	}

	/* update sa and osync para when timing switch or panel enable for debug */
	OPLUS_ADFR_TRACE_INT("oplus_adfr_osync_mode_cmd", 0);
	OPLUS_ADFR_TRACE_INT("oplus_adfr_min_fps_cmd", refresh_rate);
	OPLUS_ADFR_TRACE_INT("oplus_adfr_h_skew", h_skew);
	ADFR_DEBUG("oplus_adfr_h_skew:%u\n", h_skew);

	OPLUS_ADFR_TRACE_END("oplus_adfr_status_reset");

	ADFR_DEBUG("end\n");

	return rc;
}

/* -------------------- fakeframe -------------------- */
int oplus_adfr_fakeframe_check(void *sde_encoder_virt)
{
	struct sde_encoder_virt *sde_enc = sde_encoder_virt;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!sde_enc || !sde_enc->crtc || !sde_enc->cur_master || !sde_enc->cur_master->connector) {
		ADFR_ERR("invalid sde_enc params\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(sde_enc->cur_master->connector);
	if (!c_conn) {
		ADFR_ERR("invalid c_conn param\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		ADFR_DEBUG("not in dsi mode, should not check fakeframe\n");
		return 0;
	}

	display = c_conn->display;
	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to check fakeframe for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_fakeframe_is_enabled(p_oplus_adfr_params)) {
		ADFR_DEBUG("fakeframe is not enabled\n");
		return 0;
	}

	if (!display->panel->cur_mode || !display->panel->cur_mode->priv_info) {
		ADFR_ERR("invalid panel params\n");
		return -EINVAL;
	}

	if (!display->panel->cur_mode->priv_info->oplus_adfr_fakeframe_config) {
		ADFR_DEBUG("fakeframe is not configured\n");
		return 0;
	}

	if (!p_oplus_adfr_params->fakeframe) {
		ADFR_DEBUG("fakeframe is not set\n");
		return 0;
	}

	/* fakeframe is available only after power on */
	if (display->panel->power_mode != SDE_MODE_DPMS_ON) {
		ADFR_DEBUG("should not check fakeframe when power mode is %u\n", display->panel->power_mode);
		return -EFAULT;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_fakeframe_check");

	/*
	 send fakeframe cmds before commit to triger ddic flush panel next frame
	 but if pre-frame is pending, ignore this time
	 because pre-frame is a real frame which include fakeframe cmds
	*/
	if (!sde_crtc_frame_pending(sde_enc->crtc)) {
		p_oplus_adfr_params->need_send_fakeframe_cmd = true;
		ADFR_DEBUG("oplus_adfr_need_send_fakeframe_cmd:%d\n", p_oplus_adfr_params->need_send_fakeframe_cmd);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_need_send_fakeframe_cmd", p_oplus_adfr_params->need_send_fakeframe_cmd);
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_fakeframe_check");

	ADFR_DEBUG("end\n");

	return 0;
}

int oplus_adfr_fakeframe_handle(void *sde_encoder_virt)
{
	int rc = 0;
	struct sde_encoder_virt *sde_enc = sde_encoder_virt;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!sde_enc || !sde_enc->cur_master || !sde_enc->cur_master->connector) {
		ADFR_ERR("invalid sde_enc params\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(sde_enc->cur_master->connector);
	if (!c_conn) {
		ADFR_ERR("invalid c_conn param\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		ADFR_DEBUG("not in dsi mode, should not handle fakeframe\n");
		return 0;
	}

	display = c_conn->display;
	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to handle fakeframe for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_fakeframe_is_enabled(p_oplus_adfr_params)) {
		ADFR_DEBUG("fakeframe is not enabled\n");
		return 0;
	}

	if (!display->panel->cur_mode || !display->panel->cur_mode->priv_info) {
		ADFR_ERR("invalid panel params\n");
		return -EINVAL;
	}

	if (!display->panel->cur_mode->priv_info->oplus_adfr_fakeframe_config) {
		ADFR_DEBUG("fakeframe is not configured\n");
		return 0;
	}

	if (!p_oplus_adfr_params->fakeframe) {
		ADFR_DEBUG("fakeframe is not set\n");
		return 0;
	}

	/* fakeframe is available only after power on */
	if (display->panel->power_mode != SDE_MODE_DPMS_ON) {
		ADFR_DEBUG("should not handle fakeframe when power mode is %u\n", display->panel->power_mode);
		return -EFAULT;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_fakeframe_handle");

	/*
	 send fakeframe cmds before commit to triger ddic flush panel next frame
	 but if pre-frame is pending, ignore this time
	 because pre-frame is a real frame which include fakeframe cmds
	*/
	if (p_oplus_adfr_params->need_send_fakeframe_cmd) {
		/* send the commands to simulate a frame transmission */
		rc = oplus_adfr_display_cmd_set(display, DSI_CMD_ADFR_FAKEFRAME);
		if (rc) {
			ADFR_ERR("[%s] failed to send DSI_CMD_ADFR_FAKEFRAME cmds, rc=%d\n", display->name, rc);
		}

		ADFR_DEBUG("send DSI_CMD_ADFR_FAKEFRAME cmds\n");

		p_oplus_adfr_params->need_send_fakeframe_cmd = false;
		ADFR_DEBUG("oplus_adfr_need_send_fakeframe_cmd:%d\n", p_oplus_adfr_params->need_send_fakeframe_cmd);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_need_send_fakeframe_cmd", p_oplus_adfr_params->need_send_fakeframe_cmd);
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_fakeframe_handle");

	ADFR_DEBUG("end\n");

	return rc;
}

/* update fakeframe status according to different situation */
int oplus_adfr_fakeframe_status_update(void *dsi_panel, bool force_disable)
{
	unsigned int refresh_rate = 120;
	unsigned int h_skew = STANDARD_ADFR;
	static unsigned int last_h_active = 1080;
	struct dsi_panel *panel = dsi_panel;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!panel) {
		ADFR_ERR("invalid panel param\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		ADFR_INFO("no need to update fakeframe status for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_fakeframe_is_enabled(p_oplus_adfr_params)) {
		ADFR_DEBUG("fakeframe is not enabled\n");
		return 0;
	}

	if (!panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_fakeframe_status_update");

	refresh_rate = panel->cur_mode->timing.refresh_rate;
	h_skew = panel->cur_mode->timing.h_skew;

	if (force_disable) {
		p_oplus_adfr_params->fakeframe = OPLUS_ADFR_FAKEFRAME_OFF;
	} else if ((h_skew == OPLUS_ADFR) || (h_skew == OPLUS_MFR)) {
		/* no need to enable fakeframe in osync mode */
		p_oplus_adfr_params->fakeframe = OPLUS_ADFR_FAKEFRAME_OFF;
	} else {
		/* if fakeframe is sent after resolution switch, local garbage issue will happen in low probability */
		if (last_h_active != panel->cur_mode->timing.h_active) {
			p_oplus_adfr_params->fakeframe = OPLUS_ADFR_FAKEFRAME_OFF;
			last_h_active = panel->cur_mode->timing.h_active;
		} else {
			if (refresh_rate == 120 || refresh_rate == 90) {
				if (p_oplus_adfr_params->sw_fps == 60) {
					p_oplus_adfr_params->fakeframe = OPLUS_ADFR_FAKEFRAME_OFF;
					ADFR_INFO("sw fps is %u, no need to send fakeframe\n", p_oplus_adfr_params->sw_fps);
				} else {
					p_oplus_adfr_params->fakeframe = OPLUS_ADFR_FAKEFRAME_ON;
				}
			} else {
				p_oplus_adfr_params->fakeframe = OPLUS_ADFR_FAKEFRAME_OFF;
			}
		}
	}

	ADFR_INFO("h_active:%u,refresh_rate:%u,h_skew:%u,sw_fps:%u,fakeframe:%u\n", panel->cur_mode->timing.h_active, refresh_rate,
				h_skew, p_oplus_adfr_params->sw_fps, p_oplus_adfr_params->fakeframe);
	OPLUS_ADFR_TRACE_INT("oplus_adfr_fakeframe", p_oplus_adfr_params->fakeframe);

	OPLUS_ADFR_TRACE_END("oplus_adfr_fakeframe_status_update");

	ADFR_DEBUG("end\n");

	return 0;
}

/* -------------------- pre switch -------------------- */
int oplus_adfr_pre_switch_send(void *dsi_panel)
{
	int rc = 0;
	struct dsi_panel *panel = dsi_panel;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!panel) {
		ADFR_ERR("invalid panel param\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		ADFR_INFO("no need to send pre switch for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_pre_switch_send");

	if (panel->cur_mode->timing.refresh_rate == 60) {
		ADFR_DEBUG("start to delay to second half frame in 60hz timing\n");
		oplus_need_to_sync_te(panel);
	}

	/*
	 some panel should send pre switch cmd to set max min fps before timing switch,
	 otherwise some flicker issue would occur
	*/
	rc = oplus_adfr_panel_cmd_set(panel, DSI_CMD_ADFR_PRE_SWITCH);
	if (rc) {
		ADFR_ERR("failed to send DSI_CMD_ADFR_PRE_SWITCH cmds, rc=%d\n", rc);
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_pre_switch_send");

	ADFR_DEBUG("end\n");

	return rc;
}

/* -------------------- vsync switch -------------------- */
/* ----- te source vsync switch ----- */
int oplus_adfr_te_source_vsync_switch_mode_fixup(void *dsi_display, void *dsi_display_mode_0, void *dsi_display_mode_1)
{
	struct dsi_display *display = dsi_display;
	struct dsi_display_mode *dsi_mode = dsi_display_mode_0;
	struct dsi_display_mode *panel_dsi_mode = dsi_display_mode_1;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

	if (!dsi_mode || !panel_dsi_mode) {
		ADFR_ERR("invalid dsi_display_mode params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to do te source vsync switch mode fixup for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_te_source_vsync_switch_mode_fixup");

	/*
	 qcom patch for two te source
	 add vsync source info from panel_dsi_mode to dsi_mode
	*/
	dsi_mode->vsync_source = panel_dsi_mode->vsync_source;

	OPLUS_ADFR_TRACE_END("oplus_adfr_te_source_vsync_switch_mode_fixup");

	ADFR_DEBUG("end\n");

	return 0;
}

int oplus_adfr_te_source_vsync_switch_get_modes_helper(void *dsi_display, void *dsi_display_mode)
{
	struct dsi_display *display = dsi_display;
	struct dsi_display_mode *display_mode = dsi_display_mode;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

	if (!display_mode) {
		ADFR_ERR("invalid display_mode param\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to do te source vsync switch get modes helper for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_vsync_switch_is_enabled(p_oplus_adfr_params)
			|| (oplus_adfr_get_vsync_switch_mode(p_oplus_adfr_params) != OPLUS_ADFR_TE_SOURCE_VSYNC_SWITCH)) {
		ADFR_DEBUG("te source vsync switch is not enabled\n");
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_te_source_vsync_switch_get_modes_helper");

	/*
	 qcom patch for two te source
	 vsync source invalid, use default source
	*/
	display_mode->vsync_source = OPLUS_ADFR_TE_SOURCE_TP;

	OPLUS_ADFR_TRACE_END("oplus_adfr_te_source_vsync_switch_get_modes_helper");

	ADFR_DEBUG("end\n");

	return 0;
}

int oplus_adfr_te_source_vsync_switch_pinctrl_init(void *dsi_panel)
{
	int rc = 0;
	struct dsi_panel *panel = dsi_panel;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!panel) {
		ADFR_ERR("invalid panel param\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		ADFR_INFO("no need to init te source vsync switch pinctrl for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_vsync_switch_is_enabled(p_oplus_adfr_params)
			|| (oplus_adfr_get_vsync_switch_mode(p_oplus_adfr_params) != OPLUS_ADFR_TE_SOURCE_VSYNC_SWITCH)) {
		ADFR_DEBUG("te source vsync switch is not enabled\n");
		return 0;
	}

	if (!panel->pinctrl.pinctrl) {
		ADFR_ERR("invalid pinctrl param\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_te_source_vsync_switch_pinctrl_init");

	p_oplus_adfr_params->te1_active = pinctrl_lookup_state(panel->pinctrl.pinctrl, "te1_active");
	if (IS_ERR_OR_NULL(p_oplus_adfr_params->te1_active)) {
		rc = PTR_ERR(p_oplus_adfr_params->te1_active);
		ADFR_ERR("failed to get pinctrl te1 active state, rc=%d\n", rc);
		goto error;
	}

	p_oplus_adfr_params->te1_suspend = pinctrl_lookup_state(panel->pinctrl.pinctrl, "te1_suspend");
	if (IS_ERR_OR_NULL(p_oplus_adfr_params->te1_suspend)) {
		rc = PTR_ERR(p_oplus_adfr_params->te1_suspend);
		ADFR_ERR("failed to get pinctrl te1 suspend state, rc=%d\n", rc);
		goto error;
	}

	ADFR_INFO("init te source vsync switch pinctrl successfully\n");

error:
	OPLUS_ADFR_TRACE_END("oplus_adfr_te_source_vsync_switch_pinctrl_init");

	ADFR_DEBUG("end\n");

	return rc;
}

int oplus_adfr_te_source_vsync_switch_set_pinctrl_state(void *dsi_panel, bool enable)
{
	int rc = 0;
	struct dsi_panel *panel = dsi_panel;
	struct pinctrl_state *state = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!panel) {
		ADFR_ERR("invalid panel param\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		ADFR_INFO("no need to set te source vsync switch pinctrl state for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_vsync_switch_is_enabled(p_oplus_adfr_params)
			|| (oplus_adfr_get_vsync_switch_mode(p_oplus_adfr_params) != OPLUS_ADFR_TE_SOURCE_VSYNC_SWITCH)) {
		ADFR_DEBUG("te source vsync switch is not enabled\n");
		return 0;
	}

	if (!panel->pinctrl.pinctrl) {
		ADFR_ERR("invalid pinctrl param\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_te_source_vsync_switch_set_pinctrl_state");

	if (enable) {
		state = p_oplus_adfr_params->te1_active;
	} else {
		state = p_oplus_adfr_params->te1_suspend;
	}

	rc = pinctrl_select_state(panel->pinctrl.pinctrl, state);
	if (rc) {
		ADFR_ERR("[%s] failed to set pin state, rc=%d\n", panel->name, rc);
	} else {
		ADFR_INFO("set te1 %s pinctrl state\n", (enable ? "active" : "suspend"));
		OPLUS_ADFR_TRACE_INT("oplus_adfr_te1_pinctrl_state", enable);
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_te_source_vsync_switch_set_pinctrl_state");

	ADFR_DEBUG("end\n");

	return rc;
}

int oplus_adfr_timing_te_source_vsync_switch(void *dsi_panel)
{
	int rc = 0;
	unsigned int te_source = OPLUS_ADFR_TE_SOURCE_TP;
	unsigned int h_skew = STANDARD_ADFR;
	static unsigned int last_h_active = 1080;
	struct dsi_panel *panel = dsi_panel;
	struct dsi_display *display = NULL;
	struct drm_encoder *drm_enc = NULL;
	struct sde_encoder_virt *sde_enc = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!panel || !panel->cur_mode) {
		ADFR_ERR("invalid panel params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		ADFR_INFO("no need to switch te source when timing switching for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_vsync_switch_is_enabled(p_oplus_adfr_params)
			|| (oplus_adfr_get_vsync_switch_mode(p_oplus_adfr_params) != OPLUS_ADFR_TE_SOURCE_VSYNC_SWITCH)) {
		ADFR_DEBUG("te source vsync switch is not enabled\n");
		return 0;
	}

	if (panel->power_mode == SDE_MODE_DPMS_LP1
			|| panel->power_mode == SDE_MODE_DPMS_LP2) {
		ADFR_INFO("should not switch te source in doze/doze suspend power mode\n");
		return 0;
	}

	display = to_dsi_display(panel->host);
	if (!display || !display->bridge) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

	drm_enc = display->bridge->base.encoder;
	if (!drm_enc) {
		ADFR_ERR("invalid drm_enc param\n");
		return -EINVAL;
	}

	sde_enc = to_sde_encoder_virt(drm_enc);
	if (!sde_enc) {
		ADFR_ERR("invalid sde_enc param\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_timing_te_source_vsync_switch");

	h_skew = panel->cur_mode->timing.h_skew;

	/*
	 if use tp vsync to do resolution switch, tearing will happen
	 it seems like ddic does not support mipi offset writes after resolution switching
	 te is official, so switch to te vsync after timing switch cmds are sent because mipi will be reset after that
	 if te vysnc is using now, do nothing
	*/
	if (last_h_active != panel->cur_mode->timing.h_active) {
		te_source = OPLUS_ADFR_TE_SOURCE_TE;
	} else if (h_skew == OPLUS_ADFR || h_skew == OPLUS_MFR) {
		te_source = OPLUS_ADFR_TE_SOURCE_TE;
	} else {
		te_source = OPLUS_ADFR_TE_SOURCE_TP;
	}

	if (sde_enc->te_source != te_source) {
		sde_enc->te_source = te_source;
		ADFR_INFO("oplus_adfr_te_source:%u\n", sde_enc->te_source);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_te_source", sde_enc->te_source);

		rc = sde_encoder_helper_switch_vsync(drm_enc, false);
		if (rc) {
			ADFR_ERR("failed to switch te source, rc=%d\n", rc);
		}

		if (last_h_active != panel->cur_mode->timing.h_active) {
			/* after one frame commit completed, change back to current mode vsync */
			p_oplus_adfr_params->need_switch_vsync = true;
			ADFR_INFO("oplus_adfr_need_switch_vsync:%d\n", p_oplus_adfr_params->need_switch_vsync);
			OPLUS_ADFR_TRACE_INT("oplus_adfr_need_switch_vsync", p_oplus_adfr_params->need_switch_vsync);
		}
	}

	last_h_active = panel->cur_mode->timing.h_active;

	OPLUS_ADFR_TRACE_END("oplus_adfr_timing_te_source_vsync_switch");

	ADFR_DEBUG("end\n");

	return rc;
}

int oplus_adfr_frame_done_te_source_vsync_switch(void *drm_connector)
{
	int rc = 0;
	struct drm_connector *connector = drm_connector;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct drm_encoder *drm_enc = NULL;
	struct sde_encoder_virt *sde_enc = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!connector) {
		ADFR_ERR("invalid connector param\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(connector);
	if (!c_conn) {
		ADFR_ERR("invalid c_conn param\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		ADFR_DEBUG("not in dsi mode, should not do the te source vsync switch after frame done\n");
		return 0;
	}

	display = c_conn->display;
	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to do the te source vsync switch after frame done for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_vsync_switch_is_enabled(p_oplus_adfr_params)
			|| (oplus_adfr_get_vsync_switch_mode(p_oplus_adfr_params) != OPLUS_ADFR_TE_SOURCE_VSYNC_SWITCH)) {
		ADFR_DEBUG("te source vsync switch is not enabled\n");
		return 0;
	}

	drm_enc = c_conn->encoder;
	if (!drm_enc) {
		ADFR_ERR("invalid drm_enc param\n");
		return -EINVAL;
	}

	sde_enc = to_sde_encoder_virt(drm_enc);
	if (!sde_enc) {
		ADFR_ERR("invalid sde_enc param\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_frame_done_te_source_vsync_switch");

	if (p_oplus_adfr_params->need_switch_vsync) {
		/* wait for idle */
		sde_encoder_wait_for_event(drm_enc, MSM_ENC_TX_COMPLETE);

		/* after resolution switch, change back to tp vsync */
		sde_enc->te_source = OPLUS_ADFR_TE_SOURCE_TP;
		ADFR_INFO("oplus_adfr_te_source:%u\n", sde_enc->te_source);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_te_source", sde_enc->te_source);

		rc = sde_encoder_helper_switch_vsync(drm_enc, false);
		if (rc) {
			ADFR_ERR("failed to switch te source, rc=%d\n", rc);
		}

		/* update fakeframe setting */
		rc = oplus_adfr_fakeframe_status_update(display->panel, false);
		if (rc) {
			ADFR_ERR("failed to update fakeframe status, rc=%d\n", rc);
		}

		p_oplus_adfr_params->need_switch_vsync = false;
		ADFR_INFO("oplus_adfr_need_switch_vsync:%d\n", p_oplus_adfr_params->need_switch_vsync);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_need_switch_vsync", p_oplus_adfr_params->need_switch_vsync);
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_frame_done_te_source_vsync_switch");

	ADFR_DEBUG("end\n");

	return rc;
}

/* te source vsync switch entry and exit */
int oplus_adfr_aod_fod_te_source_vsync_switch(void *dsi_display, unsigned int te_source)
{
	int rc = 0;
	unsigned int h_skew = STANDARD_ADFR;
	struct dsi_display *display = dsi_display;
	struct drm_encoder *drm_enc = NULL;
	struct sde_encoder_virt *sde_enc = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to switch te source in aod or fod mode for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_vsync_switch_is_enabled(p_oplus_adfr_params)
			|| (oplus_adfr_get_vsync_switch_mode(p_oplus_adfr_params) != OPLUS_ADFR_TE_SOURCE_VSYNC_SWITCH)) {
		ADFR_DEBUG("te source vsync switch is not enabled\n");
		return 0;
	}

	if (!display->panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return -EINVAL;
	}

	if (!display->bridge) {
		ADFR_ERR("invalid bridge param\n");
		return -EINVAL;
	}

	drm_enc = display->bridge->base.encoder;
	if (!drm_enc) {
		ADFR_ERR("invalid drm_enc param\n");
		return -EINVAL;
	}

	sde_enc = to_sde_encoder_virt(drm_enc);
	if (!sde_enc) {
		ADFR_ERR("invalid sde_enc param\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_aod_fod_te_source_vsync_switch");

	h_skew = display->panel->cur_mode->timing.h_skew;

	/* no need to switch te source in osync mode */
	if (h_skew == OPLUS_ADFR || h_skew == OPLUS_MFR) {
		te_source = OPLUS_ADFR_TE_SOURCE_TE;
	}

	/* force to switch te vsync as tp vsync will change to 15hz in aod mode */
	if (sde_enc->te_source != te_source) {
		sde_enc->te_source = te_source;
		ADFR_INFO("oplus_adfr_te_source:%u\n", sde_enc->te_source);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_te_source", sde_enc->te_source);

		if (display->panel->panel_initialized) {
			sde_encoder_wait_for_event(drm_enc, MSM_ENC_TX_COMPLETE);
			ADFR_INFO("wait for MSM_ENC_TX_COMPLETE done\n");
		}

		rc = sde_encoder_helper_switch_vsync(drm_enc, false);
		if (rc) {
			ADFR_ERR("failed to switch te source, rc=%d\n", rc);
		}
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_aod_fod_te_source_vsync_switch");

	ADFR_DEBUG("end\n");

	return rc;
}

/* ----- mux vsync switch ----- */
int oplus_adfr_gpio_release(void *dsi_panel)
{
	struct dsi_panel *panel = dsi_panel;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!panel) {
		ADFR_ERR("invalid panel param\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		ADFR_INFO("no need to release gpio for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_vsync_switch_is_enabled(p_oplus_adfr_params)
			|| (oplus_adfr_get_vsync_switch_mode(p_oplus_adfr_params) != OPLUS_ADFR_MUX_VSYNC_SWITCH)) {
		ADFR_DEBUG("mux vsync switch is not enabled\n");
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_gpio_release");

	if (gpio_is_valid(p_oplus_adfr_params->mux_vsync_switch_gpio)) {
		gpio_free(p_oplus_adfr_params->mux_vsync_switch_gpio);
	}

	ADFR_INFO("realse adfr gpio\n");

	OPLUS_ADFR_TRACE_END("oplus_adfr_gpio_release");

	ADFR_DEBUG("end\n");

	return 0;
}

/* wait te and delay some us */
static int oplus_adfr_vblank_wait(void *dsi_display, unsigned int te_count, unsigned int delay_us)
{
	unsigned int i = 0;
	struct dsi_display *display = dsi_display;
	struct sde_connector *c_conn = NULL;
	struct drm_encoder *drm_enc = NULL;

	ADFR_DEBUG("start\n");

	if (!te_count && !delay_us) {
		ADFR_ERR("invalid input params\n");
		return 0;
	}

	if (!display) {
		ADFR_ERR("invalid display param\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(display->drm_conn);
	if (!c_conn) {
		ADFR_ERR("invalid c_conn param\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		ADFR_DEBUG("not in dsi mode, should not wait vblank\n");
		return 0;
	}

	drm_enc = c_conn->encoder;
	if (!drm_enc) {
		ADFR_ERR("invalid drm_enc param\n");
		return -EINVAL;
	}

	if (sde_encoder_is_disabled(drm_enc)) {
		ADFR_ERR("sde encoder is disabled\n");
		return -EFAULT;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_vblank_wait");

	if (te_count) {
		OPLUS_ADFR_TRACE_BEGIN("sde_encoder_wait_for_event");
		for (i = 0; i < te_count; i++) {
			sde_encoder_wait_for_event(drm_enc, MSM_ENC_VBLANK);
			ADFR_INFO("wait for %u vblank event done\n", i + 1);
		}
		OPLUS_ADFR_TRACE_END("sde_encoder_wait_for_event");
	}

	if (delay_us) {
		OPLUS_ADFR_TRACE_BEGIN("usleep_range");
		usleep_range(delay_us, (delay_us + 10));
		ADFR_INFO("usleep_range %u done\n", delay_us);
		OPLUS_ADFR_TRACE_END("usleep_range");
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_vblank_wait");

	ADFR_DEBUG("end\n");

	return 0;
}

/* mux vsync switch, 0:tp vsync,1:te vsync */
static int oplus_adfr_set_mux_vsync_switch_gpio(void *dsi_panel, unsigned int level)
{
	int rc = 0;
	struct dsi_panel *panel = dsi_panel;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!panel) {
		ADFR_ERR("invalid panel param\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		ADFR_INFO("no need to set mux vsync switch gpio for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_vsync_switch_is_enabled(p_oplus_adfr_params)
			|| (oplus_adfr_get_vsync_switch_mode(p_oplus_adfr_params) != OPLUS_ADFR_MUX_VSYNC_SWITCH)) {
		ADFR_DEBUG("mux vsync switch is not enabled\n");
		return 0;
	}

	if (!gpio_is_valid(p_oplus_adfr_params->mux_vsync_switch_gpio)) {
		ADFR_ERR("mux_vsync_switch_gpio is invalid\n");
		return -EINVAL;
	}

	if (p_oplus_adfr_params->mux_vsync_switch_gpio_level == level) {
		ADFR_INFO("mux_vsync_switch_gpio is already %u\n", level);
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_set_mux_vsync_switch_gpio");

	if (level == OPLUS_ADFR_MUX_VSYNC_SWITCH_TE) {
		rc = gpio_direction_output(p_oplus_adfr_params->mux_vsync_switch_gpio, 1);
		if (rc) {
			ADFR_ERR("unable to set dir for mux_vsync_switch_gpio, rc=%d\n", rc);
			goto end;
		}
	} else if (level == OPLUS_ADFR_MUX_VSYNC_SWITCH_TP) {
		gpio_set_value(p_oplus_adfr_params->mux_vsync_switch_gpio, 0);
	}

	p_oplus_adfr_params->mux_vsync_switch_gpio_level = level;
	ADFR_INFO("oplus_adfr_mux_vsync_switch_gpio_level:%u\n", p_oplus_adfr_params->mux_vsync_switch_gpio_level);
	OPLUS_ADFR_TRACE_INT("oplus_adfr_mux_vsync_switch_gpio_level", p_oplus_adfr_params->mux_vsync_switch_gpio_level);

end:
	OPLUS_ADFR_TRACE_END("oplus_adfr_set_mux_vsync_switch_gpio");

	ADFR_DEBUG("end\n");

	return rc;
}

int oplus_adfr_timing_mux_vsync_switch(void *dsi_display)
{
	int rc = 0;
	unsigned int level = OPLUS_ADFR_MUX_VSYNC_SWITCH_TP;
	unsigned int h_skew = STANDARD_ADFR;
	struct dsi_display *display = dsi_display;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to switch mux vsync when timing switching for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_vsync_switch_is_enabled(p_oplus_adfr_params)
			|| (oplus_adfr_get_vsync_switch_mode(p_oplus_adfr_params) != OPLUS_ADFR_MUX_VSYNC_SWITCH)) {
		ADFR_DEBUG("mux vsync switch is not enabled\n");
		return 0;
	}

	/* filter out other mux vsync switch */
	if (p_oplus_adfr_params->force_te_vsync) {
		ADFR_INFO("force te vsync, filter out other mux vsync switch\n");
		return 0;
	}

	if (!display->panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_timing_mux_vsync_switch");

	h_skew = display->panel->cur_mode->timing.h_skew;

	if (h_skew == OPLUS_ADFR) {
		level = OPLUS_ADFR_MUX_VSYNC_SWITCH_TE;
	} else {
		level = OPLUS_ADFR_MUX_VSYNC_SWITCH_TP;
	}

	if (p_oplus_adfr_params->mux_vsync_switch_gpio_level != level) {
		/*
		need to send pre switch cmds to increase refresh rate when sa/oa switching
		note:sm is also use tp vysnc for iris chip
		*/
		rc = oplus_adfr_panel_cmd_set(display->panel, DSI_CMD_ADFR_PRE_SWITCH);
		if (rc) {
			ADFR_ERR("[%s] failed to send DSI_CMD_ADFR_PRE_SWITCH cmds, rc=%d\n", display->name, rc);
		}

		/*
		 the wavefroms of tp vysnc and te vsync are different, there have unpredictable waveforms if
		 switching during the high level porch period, so wait 1 vblank and then switch the mux vsync,
		 it can get the fix timing sequence waveforms to avoid the unpredictable waveforms
		 note: switch the mux gpio would cost some time so that the above switching is on low level
		*/
		rc = oplus_adfr_vblank_wait(display, 1, 0);
		if (rc) {
			ADFR_ERR("failed to wait 1 vblank\n");
		}

		rc = oplus_adfr_set_mux_vsync_switch_gpio(display->panel, level);
		if (rc) {
			ADFR_ERR("failed to set mux_vsync_switch_gpio, rc=%d\n", rc);
		}
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_timing_mux_vsync_switch");

	ADFR_DEBUG("end\n");

	return rc;
}

/*
 if use tp vsync to do resolution switch, tearing will happen
 it seems like ddic does not support mipi offset writes after resolution switching
 te is official, so switch to te vsync after timing switch cmds are sent because mipi will be reset after that
 if te vysnc is using now, do nothing
*/
int oplus_adfr_resolution_mux_vsync_switch(void *dsi_panel)
{
	int rc = 0;
	static unsigned int last_h_active = 1080;
	struct dsi_panel *panel = dsi_panel;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!panel) {
		ADFR_ERR("invalid panel param\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		ADFR_INFO("no need to switch mux vsync when resolution switching for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_vsync_switch_is_enabled(p_oplus_adfr_params)
			|| (oplus_adfr_get_vsync_switch_mode(p_oplus_adfr_params) != OPLUS_ADFR_MUX_VSYNC_SWITCH)) {
		ADFR_DEBUG("mux vsync switch is not enabled\n");
		return 0;
	}

	if (!gpio_is_valid(p_oplus_adfr_params->mux_vsync_switch_gpio)) {
		ADFR_ERR("mux_vsync_switch_gpio is invalid\n");
		return -EINVAL;
	}

	if (!panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_resolution_mux_vsync_switch");

	/* just do the mux vysnc switch when using tp vsync and resolution changing */
	if ((last_h_active != panel->cur_mode->timing.h_active)
			&& (p_oplus_adfr_params->mux_vsync_switch_gpio_level == OPLUS_ADFR_MUX_VSYNC_SWITCH_TP)) {
		rc = oplus_adfr_set_mux_vsync_switch_gpio(panel, OPLUS_ADFR_MUX_VSYNC_SWITCH_TE);
		if (rc) {
			ADFR_ERR("failed to set mux_vsync_switch_gpio, rc=%d\n", rc);
		} else {
			/* after one frame commit completed, change back to current mode vsync */
			p_oplus_adfr_params->need_switch_vsync = true;
			ADFR_INFO("oplus_adfr_need_switch_vsync:%d\n", p_oplus_adfr_params->need_switch_vsync);
			OPLUS_ADFR_TRACE_INT("oplus_adfr_need_switch_vsync", p_oplus_adfr_params->need_switch_vsync);
		}
	}

	last_h_active = panel->cur_mode->timing.h_active;

	OPLUS_ADFR_TRACE_END("oplus_adfr_resolution_mux_vsync_switch");

	ADFR_DEBUG("end\n");

	return rc;
}

int oplus_adfr_frame_done_mux_vsync_switch(void *drm_connector)
{
	int rc = 0;
	struct drm_connector *connector = drm_connector;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!connector) {
		ADFR_ERR("invalid connector param\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(connector);
	if (!c_conn) {
		ADFR_ERR("invalid c_conn param\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		ADFR_DEBUG("not in dsi mode, should not do the mux vsync switch after frame done\n");
		return 0;
	}

	display = c_conn->display;
	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to do the mux vsync switch after frame done for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_vsync_switch_is_enabled(p_oplus_adfr_params)
			|| (oplus_adfr_get_vsync_switch_mode(p_oplus_adfr_params) != OPLUS_ADFR_MUX_VSYNC_SWITCH)) {
		ADFR_DEBUG("mux vsync switch is not enabled\n");
		return 0;
	}

	if (!c_conn->encoder) {
		ADFR_ERR("invalid encoder param\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_frame_done_mux_vsync_switch");

	if (p_oplus_adfr_params->need_switch_vsync) {
		/* wait for idle */
		sde_encoder_wait_for_event(c_conn->encoder, MSM_ENC_TX_COMPLETE);

		/* after resolution switch, change back to tp vsync */
		rc = oplus_adfr_set_mux_vsync_switch_gpio(display->panel, OPLUS_ADFR_MUX_VSYNC_SWITCH_TP);
		if (rc) {
			ADFR_ERR("failed to set mux_vsync_switch_gpio, rc=%d\n", rc);
		}

		/* update fakeframe setting */
		rc = oplus_adfr_fakeframe_status_update(display->panel, false);
		if (rc) {
			ADFR_ERR("failed to update fakeframe status, rc=%d\n", rc);
		}

		p_oplus_adfr_params->need_switch_vsync = false;
		ADFR_INFO("oplus_adfr_need_switch_vsync:%d\n", p_oplus_adfr_params->need_switch_vsync);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_need_switch_vsync", p_oplus_adfr_params->need_switch_vsync);
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_frame_done_mux_vsync_switch");

	ADFR_DEBUG("end\n");

	return rc;
}

/* mux vsync switch entry and exit */
int oplus_adfr_aod_fod_mux_vsync_switch(void *dsi_panel, bool force_te_vsync)
{
	int rc = 0;
	unsigned int h_skew = STANDARD_ADFR;
	struct dsi_panel *panel = dsi_panel;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!panel) {
		ADFR_ERR("invalid panel param\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		ADFR_INFO("no need to switch mux vsync in aod or fod mode for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_vsync_switch_is_enabled(p_oplus_adfr_params)
			|| (oplus_adfr_get_vsync_switch_mode(p_oplus_adfr_params) != OPLUS_ADFR_MUX_VSYNC_SWITCH)) {
		ADFR_DEBUG("mux vsync switch is not enabled\n");
		return 0;
	}

	if (!gpio_is_valid(p_oplus_adfr_params->mux_vsync_switch_gpio)) {
		ADFR_ERR("mux_vsync_switch_gpio is invalid\n");
		return -EINVAL;
	}

	if (!panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_aod_fod_mux_vsync_switch");

	/* force to switch te vsync as tp vsync will change to 15hz in aod mode */
	if (force_te_vsync) {
		if (p_oplus_adfr_params->mux_vsync_switch_gpio_level == OPLUS_ADFR_MUX_VSYNC_SWITCH_TP) {
			rc = oplus_adfr_set_mux_vsync_switch_gpio(panel, OPLUS_ADFR_MUX_VSYNC_SWITCH_TE);
			if (rc) {
				ADFR_ERR("failed to set mux_vsync_switch_gpio, rc=%d\n", rc);
			}
		}

		p_oplus_adfr_params->force_te_vsync = force_te_vsync;
		ADFR_INFO("oplus_adfr_force_te_vsync:%d\n", p_oplus_adfr_params->force_te_vsync);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_force_te_vsync", p_oplus_adfr_params->force_te_vsync);
	} else {
		/* change back to tp vysnc since aod or fod mode is off */
		if (p_oplus_adfr_params->force_te_vsync) {
			h_skew = panel->cur_mode->timing.h_skew;
			/* maybe change to oa in aod or fod mode */
			if ((p_oplus_adfr_params->mux_vsync_switch_gpio_level == OPLUS_ADFR_MUX_VSYNC_SWITCH_TE)
					&& (h_skew != OPLUS_ADFR)) {
				rc = oplus_adfr_set_mux_vsync_switch_gpio(panel, OPLUS_ADFR_MUX_VSYNC_SWITCH_TP);
				if (rc) {
					ADFR_ERR("failed to set mux_vsync_switch_gpio, rc=%d\n", rc);
				}
			}

			p_oplus_adfr_params->force_te_vsync = false;
			ADFR_INFO("oplus_adfr_force_te_vsync:%d\n", p_oplus_adfr_params->force_te_vsync);
			OPLUS_ADFR_TRACE_INT("oplus_adfr_force_te_vsync", p_oplus_adfr_params->force_te_vsync);
		}
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_aod_fod_mux_vsync_switch");

	ADFR_DEBUG("end\n");

	return rc;
}

/* -------------------- idle mode -------------------- */
static int oplus_adfr_idle_mode_min_fps_delay(void *sde_encoder_phys_cmd)
{
	s64 delay = 0;
	s64 us_per_frame = 0;
	ktime_t last_te_timestamp = 0;
	struct sde_encoder_phys_cmd *cmd_enc = sde_encoder_phys_cmd;
	struct sde_encoder_phys_cmd_te_timestamp *te_timestamp = NULL;

	ADFR_DEBUG("start\n");

	if (!cmd_enc) {
		ADFR_ERR("invalid cmd_enc param\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_idle_mode_min_fps_delay");

	/* for tp vysnc shift issue */
	us_per_frame = 1000000 / 60;
	te_timestamp = list_last_entry(&cmd_enc->te_timestamp_list, struct sde_encoder_phys_cmd_te_timestamp, list);
	last_te_timestamp = te_timestamp->timestamp;

	delay = (us_per_frame >> 1) - (ktime_to_us(ktime_sub(ktime_get(), last_te_timestamp)) % us_per_frame);
	ADFR_DEBUG("time interval since the last rd_ptr is %lu\n", ktime_to_us(ktime_sub(ktime_get(), last_te_timestamp)));

	if (delay > 0) {
		/* make sure to send min fps in the next 8.3ms period */
		delay = delay + 1000;
		usleep_range(delay, delay + 100);
		ADFR_DEBUG("delay %lu us for idle min fps setting\n", delay);
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_idle_mode_min_fps_delay");

	ADFR_DEBUG("end\n");

	return 0;
}

/* if idle mode is enabled, the min fps will be reduced when entering mipi idle and increased when exiting mipi idle, thus power can be saved more accurately */
int oplus_adfr_idle_mode_handle(void *sde_encoder_virt, bool enter_idle)
{
	int rc = 0;
	unsigned char min_fps_mapping_table_count = 0;
	unsigned int idle_off_min_fps = 0;
	unsigned int h_skew = STANDARD_ADFR;
	unsigned int refresh_rate = 120;
	struct sde_encoder_virt *sde_enc = sde_encoder_virt;
	struct sde_encoder_phys_cmd *cmd_enc = NULL;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct dsi_display_mode_priv_info *priv_info = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!sde_enc || !sde_enc->phys_encs[0] || !sde_enc->cur_master || !sde_enc->cur_master->connector) {
		ADFR_ERR("invalid sde_enc params\n");
		return -EINVAL;
	}

	cmd_enc = to_sde_encoder_phys_cmd(sde_enc->phys_encs[0]);
	if (!cmd_enc) {
		ADFR_ERR("invalid cmd_enc param\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(sde_enc->cur_master->connector);
	if (!c_conn) {
		ADFR_ERR("invalid c_conn param\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		ADFR_DEBUG("not in dsi mode, should not handle idle mode\n");
		return 0;
	}

	display = c_conn->display;
	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to handle idle mode for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_idle_mode_is_enabled(p_oplus_adfr_params)) {
		ADFR_DEBUG("idle mode is not enabled\n");
		return 0;
	}

	if (!display->panel->cur_mode || !display->panel->cur_mode->priv_info) {
		ADFR_ERR("invalid cur_mode params\n");
		return -EINVAL;
	}

	priv_info = display->panel->cur_mode->priv_info;
	if (priv_info->oplus_adfr_min_fps_mapping_table_count) {
		min_fps_mapping_table_count = priv_info->oplus_adfr_min_fps_mapping_table_count;
	} else {
		ADFR_DEBUG("no need to handle idle mode because oplus,adfr-min-fps-mapping-table is not set\n");
		return 0;
	}
	if (priv_info->oplus_adfr_idle_off_min_fps) {
		idle_off_min_fps = priv_info->oplus_adfr_idle_off_min_fps;
	} else {
		ADFR_DEBUG("no need to handle idle mode because oplus,adfr-idle-off-min-fps is not set\n");
		return 0;
	}

	if (p_oplus_adfr_params->skip_min_fps_setting) {
		ADFR_DEBUG("skip min fps setting\n");
		return 0;
	}

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	if (oplus_ofp_is_supported() && !oplus_ofp_oled_capacitive_is_enabled()
			&& !oplus_ofp_local_hbm_is_enabled() && !oplus_ofp_ultrasonic_is_enabled()
			&& oplus_ofp_get_hbm_state()) {
		ADFR_DEBUG("should not handle idle mode when hbm state is true\n");
		return 0;
	}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

	/* idle mode are available only after power on */
	if (display->panel->power_mode != SDE_MODE_DPMS_ON) {
		ADFR_DEBUG("should not handle idle mode when power mode is %u\n", display->panel->power_mode);
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_idle_mode_handle");

	h_skew = display->panel->cur_mode->timing.h_skew;
	refresh_rate = display->panel->cur_mode->timing.refresh_rate;

	if (enter_idle) {
		if (h_skew == STANDARD_ADFR || h_skew == STANDARD_MFR) {
			/* enter idle mode if auto mode is off and min fps is less than idle_off_min_fps */
			if ((p_oplus_adfr_params->auto_mode == OPLUS_ADFR_AUTO_OFF)
					&& (p_oplus_adfr_params->sa_min_fps < idle_off_min_fps)
					&& (p_oplus_adfr_params->sa_min_fps >= priv_info->oplus_adfr_min_fps_mapping_table[min_fps_mapping_table_count - 1])) {
				p_oplus_adfr_params->idle_mode = OPLUS_ADFR_IDLE_ON;
				ADFR_DEBUG("oplus_adfr_idle_mode:%u\n", p_oplus_adfr_params->idle_mode);
				OPLUS_ADFR_TRACE_INT("oplus_adfr_idle_mode", p_oplus_adfr_params->idle_mode);

				/* for tp vysnc shift issue */
				if (refresh_rate == 60) {
					rc = oplus_adfr_idle_mode_min_fps_delay(cmd_enc);
					if (rc) {
						ADFR_ERR("failed to delay min fps when entering mipi idle, rc=%d\n", rc);
					}
				}

				/* send min fps before enter idle */
				rc = oplus_adfr_min_fps_update(display, p_oplus_adfr_params->sa_min_fps);
				if (rc) {
					ADFR_ERR("failed to update sa min fps, rc=%d\n", rc);
				}
				ADFR_DEBUG("enter idle, min fps is %u\n", p_oplus_adfr_params->sa_min_fps);

				if (gpio_is_valid(p_oplus_adfr_params->test_te.gpio)
						&& (p_oplus_adfr_params->test_te.config != OPLUS_ADFR_TEST_TE_DISABLE)
						&& (p_oplus_adfr_params->sa_min_fps <= 10)) {
					/* speed up refrsh rate updates if enter idle mode */
					hrtimer_start(&p_oplus_adfr_params->test_te.timer, ms_to_ktime(10), HRTIMER_MODE_REL);
				}
			}
		}
	} else {
		/* exit idle mode */
		if (p_oplus_adfr_params->idle_mode == OPLUS_ADFR_IDLE_ON) {
			if (p_oplus_adfr_params->sa_min_fps < idle_off_min_fps) {
				/* for tp vysnc shift issue */
				if (refresh_rate == 60) {
					rc = oplus_adfr_idle_mode_min_fps_delay(cmd_enc);
					if (rc) {
						ADFR_ERR("failed to delay min fps when exiting mipi idle, rc=%d\n", rc);
					}
				}

				/* send min fps after exit idle */
				rc = oplus_adfr_min_fps_update(display, idle_off_min_fps);
				if (rc) {
					ADFR_ERR("failed to update sa min fps, rc=%d\n", rc);
				}
				ADFR_DEBUG("exit idle, min fps is %u\n", idle_off_min_fps);
			}

			p_oplus_adfr_params->idle_mode = OPLUS_ADFR_IDLE_OFF;
			ADFR_DEBUG("oplus_adfr_idle_mode:%u\n", p_oplus_adfr_params->idle_mode);
			OPLUS_ADFR_TRACE_INT("oplus_adfr_idle_mode", p_oplus_adfr_params->idle_mode);
		}
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_idle_mode_handle");

	ADFR_DEBUG("end\n");

	return rc;
}

/* -------------------- temperature detection -------------------- */
/* the highest min fps setting is required when the temperature meets certain conditions, otherwise recovery it */
int oplus_adfr_temperature_detection_handle(void *dsi_display, int ntc_temp, int shell_temp)
{
	static bool last_skip_min_fps_setting = false;
	int rc = 0;
	unsigned int refresh_rate = 120;
	unsigned int h_skew = STANDARD_ADFR;
	struct dsi_display *display = dsi_display;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to handle temperature detection for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_temperature_detection_is_enabled(p_oplus_adfr_params)) {
		ADFR_DEBUG("temperature detection is not enabled\n");
		return 0;
	}

	if (!display->panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_temperature_detection_handle");

	refresh_rate = display->panel->cur_mode->timing.refresh_rate;
	h_skew = display->panel->cur_mode->timing.h_skew;

	if (((h_skew != OPLUS_ADFR) && (h_skew != OPLUS_MFR))
			&& ((abs(ntc_temp - shell_temp) >= 5)
				|| (ntc_temp < 0)
				|| (shell_temp < 0)
				|| (((ntc_temp > 45) || (shell_temp > 45)) && (refresh_rate == 144))
				|| (((ntc_temp > 45) || (shell_temp > 45)) && (refresh_rate == 120))
				|| (((ntc_temp > 40) || (shell_temp > 40)) && (refresh_rate == 90))
				|| (((ntc_temp > 40) || (shell_temp > 40)) && (refresh_rate == 60)))) {
		p_oplus_adfr_params->skip_min_fps_setting = true;

		if (!last_skip_min_fps_setting && p_oplus_adfr_params->skip_min_fps_setting) {
			if (p_oplus_adfr_params->sa_min_fps == refresh_rate) {
				ADFR_INFO("ntc_temp:%d,shell_temp:%d,refresh_rate:%u,already in min fps %u\n",
							ntc_temp, shell_temp, refresh_rate, p_oplus_adfr_params->sa_min_fps);
			} else {
				ADFR_INFO("ntc_temp:%d,shell_temp:%d,refresh_rate:%u,need to set min fps to %u\n",
							ntc_temp, shell_temp, refresh_rate, refresh_rate);
				rc = oplus_adfr_min_fps_update(display, refresh_rate);
				if (rc) {
					ADFR_ERR("failed to update sa min fps, rc=%d\n", rc);
				}
			}
		}
	} else {
		p_oplus_adfr_params->skip_min_fps_setting = false;

		if (((h_skew != OPLUS_ADFR) && (h_skew != OPLUS_MFR))
				&& (last_skip_min_fps_setting && !p_oplus_adfr_params->skip_min_fps_setting)) {
			if (p_oplus_adfr_params->sa_min_fps == refresh_rate) {
				p_oplus_adfr_params->sa_min_fps_updated = false;
				ADFR_INFO("ntc_temp:%d,shell_temp:%d,refresh_rate:%u,no need to update min fps %u\n",
							ntc_temp, shell_temp, refresh_rate, p_oplus_adfr_params->sa_min_fps);
			} else {
				p_oplus_adfr_params->sa_min_fps_updated = true;
				ADFR_INFO("ntc_temp:%d,shell_temp:%d,refresh_rate:%u,need to recovery min fps to %u\n",
							ntc_temp, shell_temp, refresh_rate, p_oplus_adfr_params->sa_min_fps);
			}
			ADFR_DEBUG("oplus_adfr_sa_min_fps_updated:%d\n", p_oplus_adfr_params->sa_min_fps_updated);
			OPLUS_ADFR_TRACE_INT("oplus_adfr_sa_min_fps_updated", p_oplus_adfr_params->sa_min_fps_updated);
		}
	}

	ADFR_DEBUG("oplus_adfr_skip_min_fps_setting:%u\n", p_oplus_adfr_params->skip_min_fps_setting);
	OPLUS_ADFR_TRACE_INT("oplus_adfr_skip_min_fps_setting", p_oplus_adfr_params->skip_min_fps_setting);

	last_skip_min_fps_setting = p_oplus_adfr_params->skip_min_fps_setting;

	OPLUS_ADFR_TRACE_END("oplus_adfr_temperature_detection_handle");

	ADFR_DEBUG("end\n");

	return rc;
}

/* -------------------- test te -------------------- */
/* test te timer */
enum hrtimer_restart oplus_adfr_test_te_timer_handler(struct hrtimer *timer)
{
	struct oplus_adfr_test_te_params *p_oplus_adfr_test_te_params = from_timer(p_oplus_adfr_test_te_params, timer, timer);
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!p_oplus_adfr_test_te_params) {
		ADFR_ERR("invalid p_oplus_adfr_test_te_params param\n");
		goto end;
	}

	p_oplus_adfr_params = container_of(p_oplus_adfr_test_te_params, struct oplus_adfr_params, test_te);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		goto end;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_ERR("adfr is not supported\n");
		goto end;
	}

	if (!gpio_is_valid(p_oplus_adfr_params->test_te.gpio)) {
		ADFR_ERR("test te gpio is invalid, no need to handle test te irq\n");
		goto end;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_test_te_timer_handler");

	/* speed up refrsh rate updates if enter idle mode */
	p_oplus_adfr_params->test_te.refresh_rate = p_oplus_adfr_params->sa_min_fps;
	if (p_oplus_adfr_params->test_te.config == OPLUS_ADFR_TEST_TE_ENABLE_WITCH_LOG) {
		ADFR_INFO("enter idle mode, update refresh_rate to %u\n", p_oplus_adfr_params->test_te.refresh_rate);
	}
	OPLUS_ADFR_TRACE_INT("oplus_adfr_test_te_refresh_rate", p_oplus_adfr_params->test_te.refresh_rate);

	OPLUS_ADFR_TRACE_END("oplus_adfr_test_te_timer_handler");

end:
	ADFR_DEBUG("end\n");

	return HRTIMER_NORESTART;
}

/* test te detectiton */
static irqreturn_t oplus_adfr_test_te_irq_handler(int irq, void *data)
{
	unsigned int temp_refresh_rate = 0;
	u64 current_timestamp = 0;
	struct dsi_display *display = (struct dsi_display *)data;
	struct dsi_display_mode *mode = NULL;
	struct sde_connector *c_conn = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return IRQ_HANDLED;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to handle test te irq for iris chip\n");
		return IRQ_HANDLED;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return IRQ_HANDLED;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_ERR("adfr is not supported\n");
		return IRQ_HANDLED;
	}

	if (!gpio_is_valid(p_oplus_adfr_params->test_te.gpio)) {
		ADFR_ERR("test te gpio is invalid, no need to handle test te irq\n");
		return IRQ_HANDLED;
	}

	mode = display->panel->cur_mode;
	if (!mode) {
		ADFR_ERR("invalid mode param\n");
		return IRQ_HANDLED;
	}

	c_conn = to_sde_connector(display->drm_conn);
	if (!c_conn) {
		ADFR_ERR("invalid c_conn param\n");
		return IRQ_HANDLED;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_test_te_irq_handler");

	if (p_oplus_adfr_params->test_te.config != OPLUS_ADFR_TEST_TE_DISABLE) {
		/* check the test te interval to calculate refresh rate of ddic */
		current_timestamp = (u64)ktime_to_ms(ktime_get());
		temp_refresh_rate = 1000 / (current_timestamp - p_oplus_adfr_params->test_te.last_timestamp);

		/* filtering algorithm */
		if ((mode->timing.h_skew == STANDARD_ADFR) || (mode->timing.h_skew == STANDARD_MFR)) {
			if (mode->timing.refresh_rate == 144) {
				p_oplus_adfr_params->test_te.high_refresh_rate_count = 0;
				if (temp_refresh_rate <= 90) {
					/* update refresh rate if 1 continous temp_refresh_rate are less than or equal to 90 */
					p_oplus_adfr_params->test_te.refresh_rate = 72;
				} else {
					p_oplus_adfr_params->test_te.refresh_rate = 144;
				}
			} else if ((mode->timing.refresh_rate == 120) || (mode->timing.refresh_rate == 60)) {
				if (temp_refresh_rate > 55) {
					p_oplus_adfr_params->test_te.high_refresh_rate_count++;
					/* update refresh rate if 4 continous temp_refresh_rate are greater than 55 */
					if (p_oplus_adfr_params->test_te.high_refresh_rate_count == 4) {
						p_oplus_adfr_params->test_te.refresh_rate = mode->timing.refresh_rate;
						if ((mode->timing.refresh_rate == 120) && (p_oplus_adfr_params->sw_fps == 60) && (p_oplus_adfr_params->sa_min_fps != 120)) {
							/* show the ddic refresh rate */
							p_oplus_adfr_params->test_te.refresh_rate = 60;
						}
						p_oplus_adfr_params->test_te.high_refresh_rate_count--;
					}
				} else if (temp_refresh_rate > 16 && temp_refresh_rate <= 55) {
					p_oplus_adfr_params->test_te.high_refresh_rate_count = 0;
					/* update refresh rate if 1 continous temp_refresh_rate are greater than 16 and less than or equal to 55 */
					p_oplus_adfr_params->test_te.refresh_rate = 30;
				} else {
					p_oplus_adfr_params->test_te.high_refresh_rate_count = 0;
					/* if current refresh rate of ddic is less than or equal to 16, use it directly */
					p_oplus_adfr_params->test_te.refresh_rate = temp_refresh_rate;
				}

				if (p_oplus_adfr_params->idle_mode == OPLUS_ADFR_IDLE_ON) {
					/* use sa min fps directly when enter idle mode */
					p_oplus_adfr_params->test_te.refresh_rate = p_oplus_adfr_params->sa_min_fps;
				}
			} else {
				p_oplus_adfr_params->test_te.high_refresh_rate_count = 0;
				/* fix refresh rate */
				p_oplus_adfr_params->test_te.refresh_rate = mode->timing.refresh_rate;
			}
		} else {
			p_oplus_adfr_params->test_te.high_refresh_rate_count = 0;
			/* fix refresh rate */
			p_oplus_adfr_params->test_te.refresh_rate = mode->timing.refresh_rate;
		}

		if (p_oplus_adfr_params->test_te.refresh_rate > mode->timing.refresh_rate) {
			p_oplus_adfr_params->test_te.refresh_rate = mode->timing.refresh_rate;
		}

		ADFR_DEBUG("oplus_adfr_test_te_high_refresh_rate_count:%u\n", p_oplus_adfr_params->test_te.high_refresh_rate_count);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_test_te_high_refresh_rate_count", p_oplus_adfr_params->test_te.high_refresh_rate_count);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_temp_refresh_rate", temp_refresh_rate);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_test_te_refresh_rate", p_oplus_adfr_params->test_te.refresh_rate);

		if (p_oplus_adfr_params->test_te.config == OPLUS_ADFR_TEST_TE_ENABLE_WITCH_LOG) {
			/* print key information on every test te irq handler */
			ADFR_INFO("last_timestamp:%lu,current_timestamp:%lu,temp_refresh_rate:%u,refresh_rate:%u\n",
						p_oplus_adfr_params->test_te.last_timestamp, current_timestamp,
						temp_refresh_rate, p_oplus_adfr_params->test_te.refresh_rate);

			if ((mode->timing.h_skew == STANDARD_ADFR) || (mode->timing.h_skew == STANDARD_MFR)) {
				ADFR_INFO("h_active:%u,v_active:%u,fps:%u,h_skew:%u,auto_mode:%u,sa_min_fps:%u,sw_fps:%u,fakeframe:%u,idle_mode:%u\n",
							mode->timing.h_active,
							mode->timing.v_active,
							mode->timing.refresh_rate,
							mode->timing.h_skew,
							p_oplus_adfr_params->auto_mode,
							p_oplus_adfr_params->sa_min_fps,
							p_oplus_adfr_params->sw_fps,
							p_oplus_adfr_params->fakeframe,
							p_oplus_adfr_params->idle_mode);
			} else {
				ADFR_INFO("h_active:%u,v_active:%u,fps:%u,h_skew:%u,osync_mode:%u,osync_min_fps:%u,osync_window_min_fps:%u,osync_sync_threshold_start:%u\n",
							mode->timing.h_active,
							mode->timing.v_active,
							mode->timing.refresh_rate,
							mode->timing.h_skew,
							c_conn->qsync_mode,
							p_oplus_adfr_params->osync_min_fps,
							p_oplus_adfr_params->osync_window_min_fps,
							p_oplus_adfr_params->osync_sync_threshold_start);
			}
		}

		p_oplus_adfr_params->test_te.last_timestamp = current_timestamp;
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_test_te_irq_handler");

	ADFR_DEBUG("end\n");

	return IRQ_HANDLED;
}

int oplus_adfr_register_test_te_irq(void *dsi_display)
{
	int rc = 0;
	unsigned int test_te_irq = 0;
	struct dsi_display *display = dsi_display;
	struct platform_device *pdev = NULL;
	struct device *dev = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to register test te irq for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return 0;
	}

	pdev = display->pdev;
	if (!pdev) {
		ADFR_ERR("invalid pdev param\n");
		return -EINVAL;
	}

	dev = &pdev->dev;
	if (!dev) {
		ADFR_ERR("invalid dev param\n");
		return -EINVAL;
	}

	if (display->trusted_vm_env) {
		ADFR_INFO("GPIO's are not enabled in trusted VM\n");
		return 0;
	}

	if (!gpio_is_valid(p_oplus_adfr_params->test_te.gpio)) {
		ADFR_DEBUG("test te gpio is invalid, no need to register test te irq\n");
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_register_test_te_irq");

	test_te_irq = gpio_to_irq(p_oplus_adfr_params->test_te.gpio);

	/* avoid deferred spurious irqs with disable_irq() */
	irq_set_status_flags(test_te_irq, IRQ_DISABLE_UNLAZY);

	/* detect test te rising edge */
	if (!strcmp(display->display_type, "primary")) {
		rc = devm_request_irq(dev, test_te_irq, oplus_adfr_test_te_irq_handler,
								IRQF_TRIGGER_RISING | IRQF_ONESHOT, "TEST_TE_GPIO_0", display);
	} else {
		rc = devm_request_irq(dev, test_te_irq, oplus_adfr_test_te_irq_handler,
								IRQF_TRIGGER_RISING | IRQF_ONESHOT, "TEST_TE_GPIO_1", display);
	}

	if (rc) {
		ADFR_ERR("test te request_irq failed rc:%d\n", rc);
		irq_clear_status_flags(test_te_irq, IRQ_DISABLE_UNLAZY);
	} else {
		if (p_oplus_adfr_params->test_te.gpio != display->disp_te_gpio) {
			disable_irq(test_te_irq);
		}
		ADFR_INFO("register test te irq successfully\n");
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_register_test_te_irq");

	ADFR_DEBUG("end\n");

	return rc;
}

/* -------------------- osync mode -------------------- */
int oplus_adfr_set_osync_params(void *sde_connector, unsigned int oplus_adfr_osync_params)
{
	unsigned int h_skew = OPLUS_ADFR;
	uint64_t prop_val = 0;
	struct sde_connector *c_conn = sde_connector;
	struct dsi_display *display = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!c_conn) {
		ADFR_ERR("invalid c_conn param\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		ADFR_DEBUG("not in dsi mode, should not set osync params\n");
		return 0;
	}

	display = c_conn->display;
	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to set osync params for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return 0;
	}

	if (!display->panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return -EINVAL;
	}

	h_skew = display->panel->cur_mode->timing.h_skew;

	if (h_skew != OPLUS_ADFR) {
		ADFR_DEBUG("not in oa mode, should not set osync params\n");
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_set_osync_params");

	if (oplus_adfr_osync_params == OPLUS_ADFR_OSYNC_MODE) {
		if (c_conn->qsync_mode == SDE_RM_QSYNC_DISABLED) {
			/* change osync min fps and close osync window at once when osync mode is disabled */
			p_oplus_adfr_params->osync_min_fps = 0;
			p_oplus_adfr_params->osync_window_setting_status = OPLUS_ADFR_OSYNC_WINDOW_SETTING_AT_ONCE;
			ADFR_INFO("oplus_adfr_osync_window_setting_status:OPLUS_ADFR_OSYNC_WINDOW_SETTING_AT_ONCE\n");
		} else {
			/* reset osync min fps when osync mode is enabled */
			p_oplus_adfr_params->osync_min_fps = 0;
			/* no need to change osync window when osync mode is enabled */
			p_oplus_adfr_params->osync_window_min_fps = 0;
		}
		ADFR_INFO("osync_mode:%u,osync_min_fps:%u,osync_window_min_fps:%u\n",
					c_conn->qsync_mode, p_oplus_adfr_params->osync_min_fps, p_oplus_adfr_params->osync_window_min_fps);
	} else if (oplus_adfr_osync_params == OPLUS_ADFR_OSYNC_MIN_FPS) {
		if (c_conn->qsync_mode != SDE_RM_QSYNC_DISABLED) {
			prop_val = sde_connector_get_property(c_conn->base.state, CONNECTOR_PROP_ADFR_MIN_FPS);
			if (!(prop_val & OPLUS_ADFR_SA_MAGIC)) {
				if (prop_val != p_oplus_adfr_params->osync_min_fps) {
					ADFR_INFO("updated osync min fps %u -> %u\n",
							p_oplus_adfr_params->osync_min_fps, prop_val);
					c_conn->qsync_updated = true;
					p_oplus_adfr_params->osync_min_fps = prop_val;
					if (!prop_val) {
						/* close osync window at once when osync min fps is 0 */
						p_oplus_adfr_params->osync_window_setting_status = OPLUS_ADFR_OSYNC_WINDOW_SETTING_AT_ONCE;
						ADFR_INFO("oplus_adfr_osync_window_setting_status:OPLUS_ADFR_OSYNC_WINDOW_SETTING_AT_ONCE\n");
					} else {
						p_oplus_adfr_params->osync_window_setting_status = OPLUS_ADFR_OSYNC_WINDOW_SETTING_START;
						ADFR_INFO("oplus_adfr_osync_window_setting_status:OPLUS_ADFR_OSYNC_WINDOW_SETTING_START\n");
					}
				}
			}
		}
	}

	OPLUS_ADFR_TRACE_INT("oplus_adfr_osync_min_fps", p_oplus_adfr_params->osync_min_fps);
	OPLUS_ADFR_TRACE_INT("oplus_adfr_osync_window_min_fps", p_oplus_adfr_params->osync_window_min_fps);
	OPLUS_ADFR_TRACE_INT("oplus_adfr_osync_window_setting_status", p_oplus_adfr_params->osync_window_setting_status);

	OPLUS_ADFR_TRACE_END("oplus_adfr_set_osync_params");

	ADFR_DEBUG("end\n");

	return 0;
}

int oplus_adfr_osync_min_fps_update(void *dsi_display)
{
	int rc = 0;
	unsigned int h_skew = OPLUS_ADFR;
	struct dsi_display *display = dsi_display;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to update osync min fps for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return 0;
	}

	if (!display->panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return -EINVAL;
	}

	h_skew = display->panel->cur_mode->timing.h_skew;

	if (h_skew != OPLUS_ADFR) {
		ADFR_ERR("not in oa mode, should not update osync min fps\n");
		return -EFAULT;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_osync_min_fps_update");

	rc = oplus_adfr_min_fps_update(display, p_oplus_adfr_params->osync_min_fps);
	if (rc) {
		ADFR_ERR("failed to update osync min fps, rc=%d\n", rc);
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_osync_min_fps_update");

	ADFR_DEBUG("end\n");

	return rc;
}

int oplus_adfr_get_osync_window_min_fps(void *drm_connector)
{
	unsigned int h_skew = OPLUS_ADFR;
	struct drm_connector *connector = drm_connector;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!connector) {
		ADFR_ERR("invalid connector param\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(connector);
	if (!c_conn) {
		ADFR_ERR("invalid c_conn param\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		ADFR_DEBUG("not in dsi mode, should not get osync window min fps\n");
		return -EINVAL;
	}

	display = c_conn->display;
	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to get osync window min fps for iris chip\n");
		return -EINVAL;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return -EINVAL;
	}

	if (!display->panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return -EINVAL;
	}

	h_skew = display->panel->cur_mode->timing.h_skew;

	if (h_skew != OPLUS_ADFR) {
		ADFR_DEBUG("not in oa mode, should not get osync window min fps\n");
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_get_osync_window_min_fps");
	ADFR_DEBUG("oplus_adfr_osync_window_min_fps:%d\n", p_oplus_adfr_params->osync_window_min_fps);
	OPLUS_ADFR_TRACE_END("oplus_adfr_get_osync_window_min_fps");

	ADFR_DEBUG("end\n");

	return p_oplus_adfr_params->osync_window_min_fps;
}

int oplus_adfr_osync_threshold_lines_update(void *drm_connector, unsigned int *threshold_lines, unsigned int yres)
{
	unsigned int h_skew = OPLUS_ADFR;
	struct drm_connector *connector = drm_connector;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!connector || !threshold_lines) {
		ADFR_ERR("invalid input params\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(connector);
	if (!c_conn) {
		ADFR_ERR("invalid c_conn param\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		ADFR_DEBUG("not in dsi mode, should not update osync threshold lines\n");
		return 0;
	}

	display = c_conn->display;
	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to update osync threshold lines for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return 0;
	}

	if (!display->panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return -EINVAL;
	}

	h_skew = display->panel->cur_mode->timing.h_skew;

	if (h_skew != OPLUS_ADFR) {
		ADFR_DEBUG("not in oa mode, should not update osync threshold lines\n");
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_osync_threshold_lines_update");

	/* correct the ap window setting to be less than the ddic window */
	if (yres > 3216) {
		*threshold_lines = *threshold_lines - 47 - 58;
	} else {
		*threshold_lines = *threshold_lines - 35 - 43;
	}

	ADFR_DEBUG("threshold_lines=%u\n", *threshold_lines);

	OPLUS_ADFR_TRACE_END("oplus_adfr_osync_threshold_lines_update");

	ADFR_DEBUG("end\n");

	return 0;
}

int oplus_adfr_osync_tearcheck_update(void *sde_encoder_phys)
{
	unsigned int h_skew = OPLUS_ADFR;
	struct sde_encoder_phys *phys_enc = sde_encoder_phys;
	struct sde_hw_tear_check tc_cfg = {0};
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct sde_encoder_phys_cmd *cmd_enc = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!phys_enc || !phys_enc->connector) {
		ADFR_ERR("invalid phys_enc params\n");
		return -EINVAL;
	}

	cmd_enc = to_sde_encoder_phys_cmd(phys_enc);
	if (!cmd_enc) {
		ADFR_ERR("invalid cmd_enc param\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(phys_enc->connector);
	if (!c_conn) {
		ADFR_ERR("invalid c_conn param\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		ADFR_DEBUG("not in dsi mode, should not update osync tearcheck\n");
		return 0;
	}

	display = c_conn->display;
	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to update osync tearcheck for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return -ENOTSUPP;
	}

	if (!display->panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return -EINVAL;
	}

	h_skew = display->panel->cur_mode->timing.h_skew;

	if (h_skew != OPLUS_ADFR) {
		ADFR_DEBUG("not in oa mode, should not update osync tearcheck\n");
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_osync_tearcheck_update");

	if (p_oplus_adfr_params->osync_window_setting_status == OPLUS_ADFR_OSYNC_WINDOW_SETTING_START) {
		/* osync window should be set in the next frame because osync min fps cmds take one frame to take effect */
		p_oplus_adfr_params->osync_window_setting_status = OPLUS_ADFR_OSYNC_WINDOW_SETTING_NEXT_FRAME;
		ADFR_INFO("oplus_adfr_osync_window_setting_status:OPLUS_ADFR_OSYNC_WINDOW_SETTING_NEXT_FRAME\n");
		OPLUS_ADFR_TRACE_INT("oplus_adfr_osync_window_setting_status", p_oplus_adfr_params->osync_window_setting_status);
	} else if (p_oplus_adfr_params->osync_window_setting_status == OPLUS_ADFR_OSYNC_WINDOW_SETTING_NEXT_FRAME
				/* close window at once when osync off */
				|| p_oplus_adfr_params->osync_window_setting_status == OPLUS_ADFR_OSYNC_WINDOW_SETTING_AT_ONCE) {
		OPLUS_ADFR_TRACE_BEGIN("update_tearcheck");

		p_oplus_adfr_params->osync_window_min_fps = p_oplus_adfr_params->osync_min_fps;
		ADFR_INFO("oplus_adfr_osync_window_min_fps:%d\n", p_oplus_adfr_params->osync_window_min_fps);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_osync_window_min_fps", p_oplus_adfr_params->osync_window_min_fps);

		tc_cfg.sync_threshold_start = _get_tearcheck_threshold(phys_enc);
		cmd_enc->qsync_threshold_lines = tc_cfg.sync_threshold_start;
		if (phys_enc->has_intf_te && phys_enc->hw_intf->ops.update_tearcheck) {
			phys_enc->hw_intf->ops.update_tearcheck(phys_enc->hw_intf, &tc_cfg);
		} else if (phys_enc->hw_pp->ops.update_tearcheck) {
			phys_enc->hw_pp->ops.update_tearcheck(phys_enc->hw_pp, &tc_cfg);
		}
		SDE_EVT32(DRMID(phys_enc->parent), tc_cfg.sync_threshold_start);

		/* trigger ap osync flush */
		c_conn->qsync_updated = true;

		p_oplus_adfr_params->osync_sync_threshold_start = tc_cfg.sync_threshold_start;
		ADFR_INFO("oplus_adfr_osync_sync_threshold_start:%u\n", p_oplus_adfr_params->osync_sync_threshold_start);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_osync_sync_threshold_start", p_oplus_adfr_params->osync_sync_threshold_start);

		p_oplus_adfr_params->osync_window_setting_status = OPLUS_ADFR_OSYNC_WINDOW_SETTING_END;
		ADFR_INFO("oplus_adfr_osync_window_setting_status:OPLUS_ADFR_OSYNC_WINDOW_SETTING_END\n");
		OPLUS_ADFR_TRACE_INT("oplus_adfr_osync_window_setting_status", p_oplus_adfr_params->osync_window_setting_status);

		OPLUS_ADFR_TRACE_END("update_tearcheck");
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_osync_tearcheck_update");

	ADFR_DEBUG("end\n");

	return 0;
}

/* adjust osync window still if qsync enable without qsync updated */
int oplus_adfr_adjust_osync_tearcheck(void *sde_encoder_phys)
{
	unsigned int h_skew = OPLUS_ADFR;
	struct sde_encoder_phys *phys_enc = sde_encoder_phys;
	struct sde_hw_tear_check tc_cfg = {0};
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct sde_encoder_phys_cmd *cmd_enc = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!phys_enc || !phys_enc->connector) {
		ADFR_ERR("invalid phys_enc params\n");
		return -EINVAL;
	}

	cmd_enc = to_sde_encoder_phys_cmd(phys_enc);
	if (!cmd_enc) {
		ADFR_ERR("invalid cmd_enc param\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(phys_enc->connector);
	if (!c_conn) {
		ADFR_ERR("invalid c_conn param\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		ADFR_DEBUG("not in dsi mode, should not adjust osync tearcheck\n");
		return 0;
	}

	display = c_conn->display;
	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to adjust osync tearcheck for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return 0;
	}

	if (!display->panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return -EINVAL;
	}

	h_skew = display->panel->cur_mode->timing.h_skew;

	if (h_skew != OPLUS_ADFR) {
		ADFR_DEBUG("not in oa mode, should not adjust osync tearcheck\n");
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_adjust_osync_tearcheck");

	/* use original sync_threshold_start by default */
	tc_cfg.sync_threshold_start = _get_tearcheck_threshold(phys_enc);

	if (!sde_connector_get_qsync_mode(phys_enc->connector)
			|| !p_oplus_adfr_params->osync_window_min_fps) {
		p_oplus_adfr_params->osync_sync_threshold_start = tc_cfg.sync_threshold_start;
		ADFR_DEBUG("oplus_adfr_osync_sync_threshold_start:%u\n", p_oplus_adfr_params->osync_sync_threshold_start);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_osync_sync_threshold_start", p_oplus_adfr_params->osync_sync_threshold_start);
		goto end;
	}

	/*
	 this is probably still in the last qsync window period even though this is the second frame,
	 so close osync window to avoid tearing and keep qsync enable for this frame
	*/
	if (p_oplus_adfr_params->osync_frame_status != OPLUS_ADFR_RD_PTR) {
		/* 300 is a estimated value */
		tc_cfg.sync_threshold_start = 300;
	}

	if(p_oplus_adfr_params->osync_sync_threshold_start != tc_cfg.sync_threshold_start) {
		OPLUS_ADFR_TRACE_BEGIN("update_tearcheck");

		cmd_enc->qsync_threshold_lines = tc_cfg.sync_threshold_start;
		if (phys_enc->has_intf_te && phys_enc->hw_intf->ops.update_tearcheck) {
			phys_enc->hw_intf->ops.update_tearcheck(phys_enc->hw_intf, &tc_cfg);
		} else if (phys_enc->hw_pp->ops.update_tearcheck) {
			phys_enc->hw_pp->ops.update_tearcheck(phys_enc->hw_pp, &tc_cfg);
		}
		SDE_EVT32(DRMID(phys_enc->parent), tc_cfg.sync_threshold_start);

		/* trigger ap osync flush */
		c_conn->qsync_updated = true;

		p_oplus_adfr_params->osync_sync_threshold_start = tc_cfg.sync_threshold_start;
		ADFR_INFO("oplus_adfr_osync_sync_threshold_start:%u\n", p_oplus_adfr_params->osync_sync_threshold_start);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_osync_sync_threshold_start", p_oplus_adfr_params->osync_sync_threshold_start);

		OPLUS_ADFR_TRACE_END("update_tearcheck");
	}

end:
	OPLUS_ADFR_TRACE_END("oplus_adfr_adjust_osync_tearcheck");

	ADFR_DEBUG("end\n");

	return 0;
}

/*
 if backlight cmd is set after osync window setting finished and osync mode is enabled,
 filter it, otherwise tearing issue happen
*/
bool oplus_adfr_osync_backlight_filter(void *dsi_panel, unsigned int bl_level)
{
	bool need_filter_osync_backlight_cmd = false;
	unsigned int h_skew = OPLUS_ADFR;
	struct dsi_panel *panel = dsi_panel;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!panel) {
		ADFR_ERR("invalid panel param\n");
		return false;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		ADFR_INFO("no need to filter osync backlight for iris chip\n");
		return false;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return false;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_oa_bl_mutual_exclusion_is_enabled(p_oplus_adfr_params)) {
		ADFR_DEBUG("oa bl mutual exclusion is not enabled\n");
		return false;
	}

	if (!panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return false;
	}

	h_skew = panel->cur_mode->timing.h_skew;

	if (h_skew != OPLUS_ADFR) {
		ADFR_DEBUG("not in oa mode, should not filter osync backlight\n");
		return false;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_osync_backlight_filter");

	if (p_oplus_adfr_params->need_filter_osync_backlight_cmd && bl_level) {
		need_filter_osync_backlight_cmd = true;
		ADFR_INFO("filter osync backlight cmd\n");
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_osync_backlight_filter");

	ADFR_DEBUG("end\n");

	return need_filter_osync_backlight_cmd;
}

/* get osync backlight update status to avoid tearing issue */
int oplus_adfr_osync_backlight_updated(void *sde_connector, bool osync_backlight_updated)
{
	unsigned int h_skew = OPLUS_ADFR;
	uint64_t prop_val = 0;
	struct sde_connector *c_conn = sde_connector;
	struct dsi_display *display = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!c_conn) {
		ADFR_ERR("invalid c_conn param\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		ADFR_DEBUG("not in dsi mode, should not update osync backlight status\n");
		return 0;
	}

	display = c_conn->display;
	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to update osync backlight status for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_oa_bl_mutual_exclusion_is_enabled(p_oplus_adfr_params)) {
		ADFR_DEBUG("oa bl mutual exclusion is not enabled\n");
		return 0;
	}

	if (!display->panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return -EINVAL;
	}

	h_skew = display->panel->cur_mode->timing.h_skew;

	if (h_skew != OPLUS_ADFR) {
		ADFR_DEBUG("not in oa mode, should not update osync backlight status\n");
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_osync_backlight_updated");

	p_oplus_adfr_params->osync_backlight_updated = osync_backlight_updated;
	ADFR_DEBUG("oplus_adfr_osync_backlight_updated:%d\n", p_oplus_adfr_params->osync_backlight_updated);
	OPLUS_ADFR_TRACE_INT("oplus_adfr_osync_backlight_updated", p_oplus_adfr_params->osync_backlight_updated);

	prop_val = sde_connector_get_property(c_conn->base.state, CONNECTOR_PROP_QSYNC_MODE);
	if (prop_val != SDE_RM_QSYNC_DISABLED) {
		/* reset the osync mode timer if osync mode is still enabled */
		hrtimer_start(&p_oplus_adfr_params->osync_mode_timer, ms_to_ktime(1000), HRTIMER_MODE_REL);
		ADFR_DEBUG("osync_mode_timer start\n");
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_osync_backlight_updated");

	ADFR_DEBUG("end\n");

	return 0;
}

/* restore osync mode after the backlight is no longer updated */
enum hrtimer_restart oplus_adfr_osync_mode_timer_handler(struct hrtimer *timer)
{
	struct oplus_adfr_params *p_oplus_adfr_params = from_timer(p_oplus_adfr_params, timer, osync_mode_timer);

	ADFR_DEBUG("start\n");

	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		goto end;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_oa_bl_mutual_exclusion_is_enabled(p_oplus_adfr_params)) {
		ADFR_DEBUG("oa bl mutual exclusion is not enabled\n");
		goto end;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_osync_mode_timer_handler");

	/* restore osync mode */
	p_oplus_adfr_params->need_restore_osync_mode = true;
	ADFR_INFO("oplus_adfr_need_restore_osync_mode:%d\n", p_oplus_adfr_params->need_restore_osync_mode);
	OPLUS_ADFR_TRACE_INT("oplus_adfr_need_restore_osync_mode", p_oplus_adfr_params->need_restore_osync_mode);

	OPLUS_ADFR_TRACE_END("oplus_adfr_osync_mode_timer_handler");

end:
	ADFR_DEBUG("end\n");

	return HRTIMER_NORESTART;
}

/* need restore osync mode after backlight settings stop */
bool oplus_adfr_need_restore_osync_mode(void *sde_connector)
{
	bool rc = false;
	unsigned int h_skew = OPLUS_ADFR;
	struct sde_connector *c_conn = sde_connector;
	struct dsi_display *display = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!c_conn) {
		ADFR_ERR("invalid c_conn param\n");
		return false;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		ADFR_DEBUG("not in dsi mode, no need to restore osync mode\n");
		return false;
	}

	display = c_conn->display;
	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return false;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to restore osync mode for iris chip\n");
		return false;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return false;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_oa_bl_mutual_exclusion_is_enabled(p_oplus_adfr_params)) {
		ADFR_DEBUG("oa bl mutual exclusion is not enabled\n");
		return false;
	}

	if (!display->panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return false;
	}

	h_skew = display->panel->cur_mode->timing.h_skew;

	if (h_skew != OPLUS_ADFR) {
		ADFR_DEBUG("not in oa mode, should not restore osync mode\n");
		return false;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_need_restore_osync_mode");

	if (p_oplus_adfr_params->need_restore_osync_mode) {
		rc = true;
		ADFR_DEBUG("need_restore_osync_mode is true\n");

		/* reset it once used */
		p_oplus_adfr_params->need_restore_osync_mode = false;
		ADFR_INFO("oplus_adfr_need_restore_osync_mode:%d\n", p_oplus_adfr_params->need_restore_osync_mode);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_need_restore_osync_mode", p_oplus_adfr_params->need_restore_osync_mode);
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_need_restore_osync_mode");

	ADFR_DEBUG("end\n");

	return rc;
}

/* need to force off osync window if osync mode is enabled after panel disable to avoid tearing issue */
int oplus_adfr_need_force_off_osync_mode(void *dsi_display, bool need_force_off_osync_mode)
{
	unsigned int h_skew = OPLUS_ADFR;
	struct dsi_display *display = dsi_display;
	struct sde_connector *c_conn = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to force off osync mode for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return 0;
	}

	if (!display->panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return -EINVAL;
	}

	h_skew = display->panel->cur_mode->timing.h_skew;

	if (h_skew != OPLUS_ADFR) {
		ADFR_DEBUG("not in oa mode, no need to force off osync mode\n");
		return 0;
	}

	c_conn = to_sde_connector(display->drm_conn);
	if (!c_conn) {
		ADFR_ERR("invalid c_conn param\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_need_force_off_osync_mode");

	if (c_conn->qsync_mode != SDE_RM_QSYNC_DISABLED) {
		p_oplus_adfr_params->need_force_off_osync_mode = need_force_off_osync_mode;
		ADFR_INFO("oplus_adfr_need_force_off_osync_mode:%d\n", p_oplus_adfr_params->need_force_off_osync_mode);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_need_force_off_osync_mode", p_oplus_adfr_params->need_force_off_osync_mode);
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_need_force_off_osync_mode");

	ADFR_DEBUG("end\n");

	return 0;
}

int oplus_adfr_force_off_osync_mode(void *sde_encoder_phys)
{
	int rc = 0;
	unsigned int h_skew = OPLUS_ADFR;
	struct sde_encoder_phys *phys_enc = sde_encoder_phys;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!phys_enc || !phys_enc->connector) {
		ADFR_ERR("invalid phys_enc params\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(phys_enc->connector);
	if (!c_conn) {
		ADFR_ERR("invalid c_conn param\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		ADFR_DEBUG("not in dsi mode, should not force off osync mode\n");
		return 0;
	}

	display = c_conn->display;
	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to force off osync mode for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return 0;
	}

	if (!display->panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return -EINVAL;
	}

	h_skew = display->panel->cur_mode->timing.h_skew;

	if (h_skew != OPLUS_ADFR) {
		ADFR_DEBUG("not in oa mode, should not force off osync mode\n");
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_force_off_osync_mode");

	if (p_oplus_adfr_params->need_force_off_osync_mode) {
		/* if need_force_off_osync_mode is true, close osync window immediately */
		ADFR_INFO("force off osync mode %d -> %d\n",
				c_conn->qsync_mode, SDE_RM_QSYNC_DISABLED);
		c_conn->qsync_updated = true;
		c_conn->qsync_mode = SDE_RM_QSYNC_DISABLED;
		OPLUS_ADFR_TRACE_INT("oplus_adfr_osync_mode", c_conn->qsync_mode);

		/* change osync min fps and close osync window at once when osync mode is disabled */
		p_oplus_adfr_params->osync_min_fps = 0;
		ADFR_INFO("oplus_adfr_osync_min_fps:%u\n", p_oplus_adfr_params->osync_min_fps);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_osync_min_fps", p_oplus_adfr_params->osync_min_fps);

		p_oplus_adfr_params->osync_window_setting_status = OPLUS_ADFR_OSYNC_WINDOW_SETTING_AT_ONCE;
		ADFR_INFO("oplus_adfr_osync_window_setting_status:OPLUS_ADFR_OSYNC_WINDOW_SETTING_AT_ONCE\n");
		OPLUS_ADFR_TRACE_INT("oplus_adfr_osync_window_setting_status", p_oplus_adfr_params->osync_window_setting_status);

		p_oplus_adfr_params->need_force_off_osync_mode = false;
		ADFR_INFO("oplus_adfr_need_force_off_osync_mode:%d\n", p_oplus_adfr_params->need_force_off_osync_mode);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_need_force_off_osync_mode", p_oplus_adfr_params->need_force_off_osync_mode);
	} else if (oplus_adfr_oa_bl_mutual_exclusion_is_enabled(p_oplus_adfr_params)) {
		if ((c_conn->qsync_mode != SDE_RM_QSYNC_DISABLED)
				&& (p_oplus_adfr_params->osync_window_setting_status != OPLUS_ADFR_OSYNC_WINDOW_SETTING_START)
				&& p_oplus_adfr_params->osync_backlight_updated) {
			/*
			 because refresh rate of ddic will be increased to maximum fps as backlight is updated,
			 and ap osync window does not match the ddic window, tearing issue will happen
			 so close osync mode immediately if ap osync window is set and backlight is updated to avoid tearing issue
			*/
			ADFR_INFO("force off osync mode %d -> %d\n",
					c_conn->qsync_mode, SDE_RM_QSYNC_DISABLED);
			c_conn->qsync_updated = true;
			c_conn->qsync_mode = SDE_RM_QSYNC_DISABLED;
			OPLUS_ADFR_TRACE_INT("oplus_adfr_osync_mode", c_conn->qsync_mode);

			/* change osync min fps and close osync window at once when osync mode is disabled */
			p_oplus_adfr_params->osync_min_fps = 0;
			ADFR_INFO("oplus_adfr_osync_min_fps:%u\n", p_oplus_adfr_params->osync_min_fps);
			OPLUS_ADFR_TRACE_INT("oplus_adfr_osync_min_fps", p_oplus_adfr_params->osync_min_fps);

			p_oplus_adfr_params->osync_window_setting_status = OPLUS_ADFR_OSYNC_WINDOW_SETTING_AT_ONCE;
			ADFR_INFO("oplus_adfr_osync_window_setting_status:OPLUS_ADFR_OSYNC_WINDOW_SETTING_AT_ONCE\n");
			OPLUS_ADFR_TRACE_INT("oplus_adfr_osync_window_setting_status", p_oplus_adfr_params->osync_window_setting_status);

			/* send osync off cmds */
			rc = sde_connector_prepare_commit(phys_enc->connector);
			if (rc) {
				ADFR_ERR("failed to prepare commit, rc=%d\n", rc);
			}

			/* to avoid frequent osync mode changing when backlight changing, restore osync mode after 1000ms */
			hrtimer_start(&p_oplus_adfr_params->osync_mode_timer, ms_to_ktime(1000), HRTIMER_MODE_REL);
			ADFR_DEBUG("osync_mode_timer start\n");
		} else if ((c_conn->qsync_mode != SDE_RM_QSYNC_DISABLED)
						&& (p_oplus_adfr_params->osync_window_setting_status != OPLUS_ADFR_OSYNC_WINDOW_SETTING_START)) {
			/*
			 if backlight cmd is set after osync window setting finished and osync mode is enabled,
			 filter it, otherwise tearing issue happen
			*/
			p_oplus_adfr_params->need_filter_osync_backlight_cmd = true;
			ADFR_DEBUG("oplus_adfr_need_filter_osync_backlight_cmd:%d\n", p_oplus_adfr_params->need_filter_osync_backlight_cmd);
			OPLUS_ADFR_TRACE_INT("oplus_adfr_need_filter_osync_backlight_cmd", p_oplus_adfr_params->need_filter_osync_backlight_cmd);
		}
	}

	p_oplus_adfr_params->osync_backlight_updated = false;
	ADFR_DEBUG("oplus_adfr_osync_backlight_updated:%d\n", p_oplus_adfr_params->osync_backlight_updated);
	OPLUS_ADFR_TRACE_INT("oplus_adfr_osync_backlight_updated", p_oplus_adfr_params->osync_backlight_updated);

	OPLUS_ADFR_TRACE_END("oplus_adfr_force_off_osync_mode");

	ADFR_DEBUG("end\n");

	return rc;
}

/* need to resend osync cmds if osync mode is enabled after panel enable */
int oplus_adfr_need_resend_osync_cmd(void *dsi_display, bool need_resend_osync_cmd)
{
	unsigned int h_skew = OPLUS_ADFR;
	struct dsi_display *display = dsi_display;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to resend osync cmds for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return 0;
	}

	if (!display->panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return -EINVAL;
	}

	h_skew = display->panel->cur_mode->timing.h_skew;

	if (h_skew != OPLUS_ADFR) {
		ADFR_DEBUG("not in oa mode, no need to resend osync cmds\n");
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_need_resend_osync_cmd");

	p_oplus_adfr_params->need_resend_osync_cmd = need_resend_osync_cmd;
	ADFR_DEBUG("oplus_adfr_need_resend_osync_cmd:%d\n", p_oplus_adfr_params->need_resend_osync_cmd);
	OPLUS_ADFR_TRACE_INT("oplus_adfr_need_resend_osync_cmd", p_oplus_adfr_params->need_resend_osync_cmd);

	OPLUS_ADFR_TRACE_END("oplus_adfr_need_resend_osync_cmd");

	ADFR_DEBUG("end\n");

	return 0;
}

/* resend osync cmds after display_lock unlock */
int oplus_adfr_resend_osync_cmd(void *dsi_display)
{
	int rc = 0;
	unsigned int h_skew = OPLUS_ADFR;
	struct dsi_display *display = dsi_display;
	struct sde_connector *c_conn = NULL;
	struct msm_display_conn_params params;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to resend osync cmds for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return 0;
	}

	if (!display->panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return -EINVAL;
	}

	h_skew = display->panel->cur_mode->timing.h_skew;

	if (h_skew != OPLUS_ADFR) {
		ADFR_DEBUG("not in oa mode, should not resend osync cmds\n");
		return 0;
	}

	c_conn = to_sde_connector(display->drm_conn);
	if (!c_conn) {
		ADFR_ERR("invalid c_conn param\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_resend_osync_cmd");

	if (p_oplus_adfr_params->need_resend_osync_cmd) {
		if (c_conn->qsync_mode != SDE_RM_QSYNC_DISABLED) {
			params.qsync_mode = c_conn->qsync_mode;
			params.qsync_update = true;
			rc = dsi_display_pre_commit(display, &params);
			if (rc) {
				ADFR_ERR("failed to resend osync cmds, rc=%d\n", rc);
			} else {
				ADFR_INFO("resend osync cmds,osync_mode:%u,osync_min_fps:%u\n", params.qsync_mode, p_oplus_adfr_params->osync_min_fps);
			}
		}

		p_oplus_adfr_params->need_resend_osync_cmd = false;
		ADFR_DEBUG("oplus_adfr_need_resend_osync_cmd:%d\n", p_oplus_adfr_params->need_resend_osync_cmd);
		OPLUS_ADFR_TRACE_INT("oplus_adfr_need_resend_osync_cmd", p_oplus_adfr_params->need_resend_osync_cmd);
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_resend_osync_cmd");

	ADFR_DEBUG("end\n");

	return rc;
}

/* --------------- high precision mode ---------------*/
/* prevent the wrong high precision fps setting */
static int oplus_adfr_high_precision_fps_check(void *dsi_panel, unsigned int high_precision_fps)
{
	unsigned char high_precision_fps_mapping_table_count = 0;
	unsigned int refresh_rate = 120;
	unsigned int h_skew = STANDARD_ADFR;
	struct dsi_panel *panel = dsi_panel;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!panel) {
		ADFR_ERR("invalid panel param\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		ADFR_INFO("no need to check high precision fps for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_high_precision_sa_mode_is_enabled(p_oplus_adfr_params)) {
		ADFR_DEBUG("high precision mode is not enabled\n");
		return 0;
	}

	if (!panel->cur_mode || !panel->cur_mode->priv_info) {
		ADFR_ERR("invalid cur_mode params\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_high_precision_fps_check");

	refresh_rate = panel->cur_mode->timing.refresh_rate;
	h_skew = panel->cur_mode->timing.h_skew;
	high_precision_fps_mapping_table_count = panel->cur_mode->priv_info->oplus_adfr_high_precision_fps_mapping_table_count;
	ADFR_DEBUG("refresh_rate:%u,h_skew:%u,high_precision_fps_mapping_table_count:%u\n",
					refresh_rate, h_skew, high_precision_fps_mapping_table_count);

	if (!high_precision_fps_mapping_table_count || !high_precision_fps) {
		/* fixed max high precision fps */
		high_precision_fps = refresh_rate;
	} else if ((high_precision_fps > panel->cur_mode->priv_info->oplus_adfr_high_precision_fps_mapping_table[0])
					|| (high_precision_fps < panel->cur_mode->priv_info->oplus_adfr_high_precision_fps_mapping_table[high_precision_fps_mapping_table_count - 1])) {
		/* the highest frame rate is the most stable */
		high_precision_fps = panel->cur_mode->priv_info->oplus_adfr_high_precision_fps_mapping_table[0];
	}

	ADFR_DEBUG("high precision fps is %u after check\n", high_precision_fps);

	OPLUS_ADFR_TRACE_END("oplus_adfr_high_precision_fps_check");

	ADFR_DEBUG("end\n");

	return high_precision_fps;
}

static int oplus_adfr_high_precision_update_te_shift(void *dsi_display)
{
	int rc = 0;
	struct dsi_display *display = dsi_display;
	static unsigned int last_high_precision_te_shift_status = 0;
	struct dsi_display_mode_priv_info *priv_info = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	if (!display || !display->panel || !display->panel->cur_mode) {
		ADFR_ERR("display is null\n");
		return -EINVAL;
	}

	if (display->panel->power_mode != SDE_MODE_DPMS_ON) {
		ADFR_DEBUG("ignore when power is %d\n", display->panel->power_mode);
		return 0;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to update high precision te shift for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_high_precision_oa_mode_is_enabled(p_oplus_adfr_params)) {
		ADFR_DEBUG("oa high precision mode is not enabled\n");
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_high_precision_update_te_shift");

	priv_info = display->panel->cur_mode->priv_info;
	if ((p_oplus_adfr_params->current_wr_rd_irq_interval < priv_info->oplus_adfr_sw_stabilize_frame_threshold_us)
		&& (p_oplus_adfr_params->last_wr_rd_irq_interval < priv_info->oplus_adfr_sw_stabilize_frame_threshold_us)) {
		p_oplus_adfr_params->high_precision_te_shift_status = OPLUS_ADFR_TE_SHIFT_OFF;
	} else {
		p_oplus_adfr_params->high_precision_te_shift_status = OPLUS_ADFR_TE_SHIFT_ON;
	}

	if (last_high_precision_te_shift_status != p_oplus_adfr_params->high_precision_te_shift_status) {
		ADFR_DEBUG("oplus_adfr_high_precision_shift_status is %d\n", p_oplus_adfr_params->high_precision_te_shift_status);
		if (p_oplus_adfr_params->high_precision_te_shift_status == OPLUS_ADFR_TE_SHIFT_OFF) {
			rc = oplus_adfr_display_cmd_set(display, DSI_CMD_ADFR_HIGH_PRECISION_TE_SHIFT_OFF);
		} else if (p_oplus_adfr_params->high_precision_te_shift_status == OPLUS_ADFR_TE_SHIFT_ON) {
			rc = oplus_adfr_display_cmd_set(display, DSI_CMD_ADFR_HIGH_PRECISION_TE_SHIFT_ON);
		}
	}
	last_high_precision_te_shift_status = p_oplus_adfr_params->high_precision_te_shift_status;
	OPLUS_ADFR_TRACE_END("oplus_adfr_high_precision_update_te_shift");
	return rc;
}

static int oplus_adfr_get_stabilize_frame_type(void *dsi_display)
{
	int i = 0;
	unsigned int high_precision_fps = 0;
	struct dsi_display *display = dsi_display;
	struct dsi_display_mode_priv_info *priv_info = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	if (!display || !display->panel || !display->panel->cur_mode || !display->panel->cur_mode->priv_info) {
		ADFR_ERR("invalid display or panel params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to get stabilize frame type for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_high_precision_oa_mode_is_enabled(p_oplus_adfr_params)) {
		ADFR_DEBUG("oa high precision mode is not enabled\n");
		return 0;
	}

	priv_info = display->panel->cur_mode->priv_info;

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_get_stabilize_frame_type");

	/* check high precision fps */
	high_precision_fps = oplus_adfr_high_precision_fps_check(display->panel, p_oplus_adfr_params->oa_high_precision_fps);

	if (priv_info->oplus_adfr_sw_stabilize_frame_config_table_count) {
		/* find the sw stabilize frame mapping fps */
		for (i = 0; i < priv_info->oplus_adfr_sw_stabilize_frame_config_table_count - 1; i++) {
			if ((high_precision_fps <= priv_info->oplus_adfr_sw_stabilize_frame_config_table[i])
					&& (high_precision_fps > priv_info->oplus_adfr_sw_stabilize_frame_config_table[i + 1])) {
				break;
			}
		}
		if (high_precision_fps == priv_info->oplus_adfr_sw_stabilize_frame_config_table[i]) {
			p_oplus_adfr_params->stabilize_frame_type = OPLUS_ADFR_SW_STABILIZE_FRAME;
		}
	}
	if (priv_info->oplus_adfr_hw_stabilize_frame_config_table_count) {
		/* find the hw stabilize frame mapping fps */
		for (i = 0; i < priv_info->oplus_adfr_hw_stabilize_frame_config_table_count - 1; i++) {
			if ((high_precision_fps <= priv_info->oplus_adfr_hw_stabilize_frame_config_table[i])
					&& (high_precision_fps > priv_info->oplus_adfr_hw_stabilize_frame_config_table[i + 1])) {
				break;
			}
		}
		if (high_precision_fps == priv_info->oplus_adfr_hw_stabilize_frame_config_table[i]) {
			p_oplus_adfr_params->stabilize_frame_type = OPLUS_ADFR_HW_STABILIZE_FRAME;
		}
	}

	ADFR_DEBUG("oplus_adfr_get_stabilize_frame_type is %d\n", p_oplus_adfr_params->stabilize_frame_type);
	OPLUS_ADFR_TRACE_INT("oplus_adfr_stabilize_frame_type", p_oplus_adfr_params->stabilize_frame_type);
	OPLUS_ADFR_TRACE_END("oplus_adfr_get_stabilize_frame_type");
	return p_oplus_adfr_params->stabilize_frame_type;
}

static int oplus_adfr_high_precision_fps_update(void *dsi_display, unsigned int high_precision_fps)
{
	int rc = 0;
	int i = 0;
	struct dsi_display *display = dsi_display;
	struct dsi_display_mode_priv_info *priv_info = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!display || !display->panel) {
		ADFR_ERR("invalid display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to update high precision fps for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return 0;
	}

	if (!display->panel->cur_mode || !display->panel->cur_mode->priv_info) {
		ADFR_ERR("invalid panel params\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_high_precision_fps_update");

	priv_info = display->panel->cur_mode->priv_info;

	/* check high precision fps */
	high_precision_fps = oplus_adfr_high_precision_fps_check(display->panel, high_precision_fps);
	if (high_precision_fps <= 0) {
		rc = high_precision_fps;
		ADFR_ERR("failed to check high precision fps, rc=%d\n", rc);
		goto end;
	}

	if (!priv_info->oplus_adfr_high_precision_fps_mapping_table_count) {
		ADFR_DEBUG("no support high precision fps\n");
		goto end;
	} else {
		/* find the high precision fps mapping cmd set */
		for (i = 0; i < priv_info->oplus_adfr_high_precision_fps_mapping_table_count - 1; i++) {
			if ((high_precision_fps <= priv_info->oplus_adfr_high_precision_fps_mapping_table[i])
					&& (high_precision_fps > priv_info->oplus_adfr_high_precision_fps_mapping_table[i + 1])) {
				break;
			}
		}
	}

	/* send the commands to set high precision fps */
	if (oplus_panel_pwm_turbo_switch_state(display->panel) == PWM_SWITCH_DC_STATE) {
		rc = oplus_adfr_display_cmd_set(display, DSI_CMD_ADFR_HIGH_PRECISION_FPS_0 + i);
		if (rc) {
			ADFR_ERR("[%s] failed to send DSI_CMD_ADFR_HIGH_PRECISION_FPS_%d cmds, rc=%d\n", display->name, i, rc);
		}
	} else {
		rc = oplus_adfr_display_cmd_set(display, DSI_CMD_HPWM_ADFR_HIGH_PRECISION_FPS_0 + i);
		if (rc) {
			ADFR_ERR("[%s] failed to send DSI_CMD_HPWM_ADFR_HIGH_PRECISION_FPS_%d cmds, rc=%d\n", display->name, i, rc);
		}
	}

	ADFR_DEBUG("oplus_adfr_high_precision_fps_update:%u\n", high_precision_fps);
	OPLUS_ADFR_TRACE_INT("oplus_adfr_high_precision_fps_update", high_precision_fps);

end:
	OPLUS_ADFR_TRACE_END("oplus_adfr_high_precision_fps_update");

	ADFR_DEBUG("end\n");

	return rc;
}

int oplus_adfr_high_precision_handle(void *sde_enc_v)
{
	struct sde_encoder_virt *sde_enc = (struct sde_encoder_virt *)sde_enc_v;
	struct sde_encoder_phys *phys = NULL;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct dsi_display_mode_priv_info *priv_info = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;
	int h_skew = STANDARD_ADFR;
	unsigned int refresh_rate = 120;
	unsigned int stabilize_frame_type = 0;
	int rc = 0;
	u32 propval;

	ADFR_DEBUG("start\n");

	if (!sde_enc) {
		ADFR_ERR("invalid sde_encoder_virt parameters\n");
		return 0;
	}

	phys = sde_enc->phys_encs[0];
	if (!phys || !phys->connector) {
		ADFR_ERR("invalid sde_encoder_phys parameters\n");
		return 0;
	}

	c_conn = to_sde_connector(phys->connector);
	if (!c_conn) {
		ADFR_ERR("invalid sde_connector parameters\n");
		return 0;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI)
		return 0;

	display = c_conn->display;
	if (!display || !display->panel || !display->panel->cur_mode) {
		ADFR_ERR("invalid display or panel params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_INFO("no need to handle high precision mode for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_DEBUG("adfr is not supported\n");
		return 0;
	}

	/* oa high precision fps is available only after power on */
	if ((display->panel->power_mode != SDE_MODE_DPMS_ON)
			|| (oplus_last_backlight == 0 || oplus_last_backlight == 1)) {
		ADFR_DEBUG("should not handle high precision mode when power mode is %u\n", display->panel->power_mode);
		return -EFAULT;
	}

	priv_info = display->panel->cur_mode->priv_info;
	refresh_rate = display->panel->cur_mode->timing.refresh_rate;
	h_skew = display->panel->cur_mode->timing.h_skew;

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_high_precision_handle");

	propval = sde_connector_get_property(c_conn->base.state, CONNECTOR_PROP_HIGH_PRECISION_FPS);
	ADFR_DEBUG("CONNECTOR_PROP_HIGH_PRECISION_FPS:0x%08x\n", propval);

	if (oplus_adfr_high_precision_sa_mode_is_enabled(p_oplus_adfr_params)) {
		if ((h_skew == STANDARD_ADFR) && (refresh_rate == 120)) {
			if (propval & OPLUS_ADFR_HIGH_PRECISION_SA_MODE_MAGIC) {
				if (OPLUS_ADFR_HIGH_PRECISION_SA_VALUE(propval) != p_oplus_adfr_params->sa_high_precision_fps) {
					p_oplus_adfr_params->sa_high_precision_fps_updated = true;
					p_oplus_adfr_params->sa_high_precision_fps = OPLUS_ADFR_HIGH_PRECISION_SA_VALUE(propval);
					/* latest setting */
					pr_info("sa high precision fps %d[%d]\n", p_oplus_adfr_params->sa_high_precision_fps, p_oplus_adfr_params->sa_high_precision_fps_updated);
				}
			}

			if (p_oplus_adfr_params->sa_high_precision_fps_updated) {
				/* update sa high precision fps */
				oplus_adfr_high_precision_fps_update(display, p_oplus_adfr_params->sa_high_precision_fps);
				p_oplus_adfr_params->sa_high_precision_fps_updated = false;
				ADFR_DEBUG("sa_high_precision_fps_updated:%d\n", p_oplus_adfr_params->sa_high_precision_fps_updated);
				OPLUS_ADFR_TRACE_INT("sa_high_precision_fps_updated", p_oplus_adfr_params->sa_high_precision_fps_updated);
			}
		}
	}

	if (oplus_adfr_high_precision_oa_mode_is_enabled(p_oplus_adfr_params)) {
		if ((h_skew == OPLUS_ADFR) && (refresh_rate == 120)) {
			if (propval & OPLUS_ADFR_HIGH_PRECISION_OA_MODE_MAGIC) {
				if (OPLUS_ADFR_HIGH_PRECISION_OA_VALUE(propval) != p_oplus_adfr_params->oa_high_precision_fps) {
					p_oplus_adfr_params->oa_high_precision_fps_updated = true;
					p_oplus_adfr_params->oa_high_precision_fps = OPLUS_ADFR_HIGH_PRECISION_OA_VALUE(propval);
					/* latest setting */
					pr_info("oa high precision fps %d[%d]\n", p_oplus_adfr_params->oa_high_precision_fps, p_oplus_adfr_params->oa_high_precision_fps_updated);
				}
			}

			if (p_oplus_adfr_params->oa_high_precision_fps_updated) {
				/* update oa high precision fps */
				oplus_adfr_high_precision_fps_update(display, p_oplus_adfr_params->oa_high_precision_fps);
				p_oplus_adfr_params->oa_high_precision_fps_updated = false;
			}

			stabilize_frame_type = oplus_adfr_get_stabilize_frame_type(display);
			if ((priv_info->oplus_adfr_sw_stabilize_frame_config_table_count)
				&& (stabilize_frame_type == OPLUS_ADFR_SW_STABILIZE_FRAME)
				&& (oplus_panel_pwm_turbo_switch_state(display->panel) == PWM_SWITCH_DC_STATE)
				&& p_oplus_adfr_params->osync_min_fps) {
					oplus_adfr_high_precision_update_te_shift(display);
			}
		}
	}
	OPLUS_ADFR_TRACE_END("oplus_adfr_high_precision_handle");

	ADFR_DEBUG("end\n");

	return rc;
}

int oplus_adfr_high_precision_switch_state(void *dsi_panel)
{
	int rc = 0;
	int i = 0;
	unsigned int high_precision_switch_state = 0;
	unsigned int high_precision_fps = 0;
	unsigned int h_skew = STANDARD_ADFR;
	static unsigned int last_high_precision_switch_state = 0;
	struct dsi_panel *panel = dsi_panel;
	struct dsi_display_mode_priv_info *priv_info = NULL;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!panel || !panel->cur_mode)
		return -EINVAL;

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		ADFR_INFO("no need to switch high precision pwm due to backlight change for iris chip\n");
		return 0;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_high_precision_oa_mode_is_enabled(p_oplus_adfr_params)) {
		ADFR_DEBUG("oa high precision mode is not enabled\n");
		return 0;
	}

	/* oa high precision fps is available only after power on */
	if (panel->power_mode != SDE_MODE_DPMS_ON) {
		ADFR_DEBUG("should not switch oa high precision when power mode is %u\n", panel->power_mode);
		return -EFAULT;
	}

	h_skew = panel->cur_mode->timing.h_skew;
	if (!oplus_adfr_high_precision_switch_is_enabled(p_oplus_adfr_params) || h_skew != OPLUS_ADFR) {
		ADFR_DEBUG("not in oa mode, should not switch oa high precision mode\n");
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_high_precision_switch_state");

	priv_info = panel->cur_mode->priv_info;
	high_precision_switch_state = oplus_panel_pwm_turbo_switch_state(panel);

	/* check high precision fps */
	high_precision_fps = oplus_adfr_high_precision_fps_check(panel, p_oplus_adfr_params->oa_high_precision_fps);
	if (high_precision_fps <= 0) {
		rc = high_precision_fps;
		ADFR_ERR("failed to check high precision fps, rc=%d\n", rc);
		goto end;
	}

	if (!priv_info->oplus_adfr_high_precision_fps_mapping_table_count) {
		ADFR_DEBUG("no support high precision fps\n");
		goto end;
	} else {
		/* find the high precision fps mapping cmd set */
		for (i = 0; i < priv_info->oplus_adfr_high_precision_fps_mapping_table_count - 1; i++) {
			if ((high_precision_fps <= priv_info->oplus_adfr_high_precision_fps_mapping_table[i])
					&& (high_precision_fps > priv_info->oplus_adfr_high_precision_fps_mapping_table[i + 1])) {
				break;
			}
		}
	}

	if (last_high_precision_switch_state != high_precision_switch_state) {
		if (high_precision_switch_state == PWM_SWITCH_DC_STATE) {
			rc = oplus_adfr_panel_cmd_set_nolock(panel, DSI_CMD_ADFR_HIGH_PRECISION_FPS_0 + i);
			if (rc) {
				ADFR_ERR("[%s] failed to send DSI_CMD_ADFR_HIGH_PRECISION_FPS_%d cmds, rc=%d\n", panel->name, i, rc);
			}
		} else {
			rc = oplus_adfr_panel_cmd_set_nolock(panel, DSI_CMD_HPWM_ADFR_HIGH_PRECISION_FPS_0 + i);
			if (rc) {
				ADFR_ERR("[%s] failed to send DSI_CMD_HPWM_ADFR_HIGH_PRECISION_FPS_%d cmds, rc=%d\n", panel->name, i, rc);
			}
		}
	}

	last_high_precision_switch_state = high_precision_switch_state;

	ADFR_DEBUG("oplus_high_precision_fps_cmd:%u\n", high_precision_fps);
	OPLUS_ADFR_TRACE_INT("oplus_high_precision_fps_cmd", high_precision_fps);

end:
	OPLUS_ADFR_TRACE_END("oplus_adfr_high_precision_switch_state");
	ADFR_DEBUG("end\n");
	return 0;
}
/* -------------------- node -------------------- */
/* adfr_config */
ssize_t oplus_adfr_set_config_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int config = 0;
	struct dsi_display *display = oplus_display_get_current_display();
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!buf || !display || !display->panel) {
		ADFR_ERR("invalid buf or display params\n");
		return count;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_ERR("can not set config for iris chip\n");
		return count;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return count;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_set_adfr_config_attr");

	sscanf(buf, "%x", &config);

	p_oplus_adfr_params->config = config;
	ADFR_INFO("oplus_adfr_config:0x%x\n", p_oplus_adfr_params->config);
	OPLUS_ADFR_TRACE_INT("oplus_adfr_config", p_oplus_adfr_params->config);

	OPLUS_ADFR_TRACE_END("oplus_adfr_set_adfr_config_attr");

	ADFR_DEBUG("end\n");

	return count;
}

ssize_t oplus_adfr_get_config_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	struct dsi_display *display = oplus_display_get_current_display();
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!buf || !display || !display->panel) {
		ADFR_ERR("invalid buf or display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_ERR("can not get config for iris chip\n");
		return -EFAULT;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_get_config_attr");

	OFP_INFO("oplus_adfr_config:0x%x\n", p_oplus_adfr_params->config);

	OPLUS_ADFR_TRACE_END("oplus_adfr_get_config_attr");

	ADFR_DEBUG("end\n");

	return sprintf(buf, "0x%x\n", p_oplus_adfr_params->config);
}

/* mux_vsync_switch */
ssize_t oplus_adfr_set_mux_vsync_switch_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int rc = 0;
	unsigned int mux_vsync_switch_gpio_level = 0;
	struct dsi_display *display = oplus_display_get_current_display();
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!buf || !display || !display->panel) {
		ADFR_ERR("invalid buf or display params\n");
		return count;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_ERR("can not set mux_vsync_switch_gpio for iris chip\n");
		return count;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return count;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_vsync_switch_is_enabled(p_oplus_adfr_params)
			|| (oplus_adfr_get_vsync_switch_mode(p_oplus_adfr_params) != OPLUS_ADFR_MUX_VSYNC_SWITCH)) {
		ADFR_ERR("mux vsync switch is not enabled\n");
		return count;
	}

	if (!gpio_is_valid(p_oplus_adfr_params->mux_vsync_switch_gpio)) {
		ADFR_ERR("mux_vsync_switch_gpio is invalid, should not set mux_vsync_switch_gpio\n");
		return count;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_set_mux_vsync_switch_attr");

	sscanf(buf, "%u", &mux_vsync_switch_gpio_level);
	ADFR_INFO("set mux_vsync_switch_gpio to %u\n", mux_vsync_switch_gpio_level);

	rc = oplus_adfr_set_mux_vsync_switch_gpio(display->panel, mux_vsync_switch_gpio_level);
	if (rc) {
		ADFR_ERR("failed to set mux_vsync_switch_gpio, rc=%d\n", rc);
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_set_mux_vsync_switch_attr");

	ADFR_DEBUG("end\n");

	return count;
}

ssize_t oplus_adfr_get_mux_vsync_switch_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	struct dsi_display *display = oplus_display_get_current_display();
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!buf || !display || !display->panel) {
		ADFR_ERR("invalid buf or display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_ERR("can not get mux_vsync_switch_gpio_level for iris chip\n");
		return -EFAULT;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_vsync_switch_is_enabled(p_oplus_adfr_params)
			|| (oplus_adfr_get_vsync_switch_mode(p_oplus_adfr_params) != OPLUS_ADFR_MUX_VSYNC_SWITCH)) {
		ADFR_ERR("mux vsync switch is not enabled\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(p_oplus_adfr_params->mux_vsync_switch_gpio)) {
		ADFR_ERR("mux_vsync_switch_gpio is invalid, can not get mux_vsync_switch_gpio_level\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_get_mux_vsync_switch_attr");

	ADFR_INFO("oplus_adfr_mux_vsync_switch_gpio_level:%u\n", p_oplus_adfr_params->mux_vsync_switch_gpio_level);

	OPLUS_ADFR_TRACE_END("oplus_adfr_get_mux_vsync_switch_attr");

	ADFR_DEBUG("end\n");

	return sprintf(buf, "%u\n", p_oplus_adfr_params->mux_vsync_switch_gpio_level);
}

/* test te */
int oplus_adfr_set_test_te(void *buf)
{
	unsigned int *test_te_config = buf;
	unsigned int test_te_irq = 0;
	struct dsi_display *display = oplus_display_get_current_display();
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!buf || !display || !display->panel) {
		ADFR_ERR("invalid buf or display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_ERR("can not set test te for iris chip\n");
		return -EFAULT;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_ERR("adfr is not supported\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(p_oplus_adfr_params->test_te.gpio)) {
		ADFR_ERR("test te gpio is invalid, should not set test te\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_set_test_te");

	p_oplus_adfr_params->test_te.config = *test_te_config;
	ADFR_INFO("oplus_adfr_test_te_config:%u\n", p_oplus_adfr_params->test_te.config);
	OPLUS_ADFR_TRACE_INT("oplus_adfr_test_te_config", p_oplus_adfr_params->test_te.config);

	test_te_irq = gpio_to_irq(p_oplus_adfr_params->test_te.gpio);
	if (p_oplus_adfr_params->test_te.config != OPLUS_ADFR_TEST_TE_DISABLE) {
		enable_irq(test_te_irq);
		ADFR_INFO("enable test te irq\n");
	} else {
		disable_irq(test_te_irq);
		ADFR_INFO("disable test te irq\n");
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_set_test_te");

	ADFR_DEBUG("end\n");

	return 0;
}

int oplus_adfr_get_test_te(void *buf)
{
	unsigned int *refresh_rate = buf;
	struct dsi_display *display = oplus_display_get_current_display();
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!buf || !display || !display->panel) {
		ADFR_ERR("invalid buf or display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_ERR("can not get test te refresh rate for iris chip\n");
		return -EFAULT;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_ERR("adfr is not supported\n");
		return -EINVAL;
	}

	if (!display->panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_get_test_te");

	if (!gpio_is_valid(p_oplus_adfr_params->test_te.gpio)) {
		*refresh_rate = display->panel->cur_mode->timing.refresh_rate;
		ADFR_INFO("test te gpio is invalid, use current timing refresh rate\n");
	} else {
		*refresh_rate = p_oplus_adfr_params->test_te.refresh_rate;
	}

	ADFR_DEBUG("oplus_adfr_test_te_refresh_rate:%u\n", *refresh_rate);

	OPLUS_ADFR_TRACE_END("oplus_adfr_get_test_te");

	ADFR_DEBUG("end\n");

	return 0;
}

ssize_t oplus_adfr_set_test_te_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int test_te_config = OPLUS_ADFR_TEST_TE_DISABLE;
	unsigned int test_te_irq = 0;
	struct dsi_display *display = oplus_display_get_current_display();
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!buf || !display || !display->panel) {
		ADFR_ERR("invalid buf or display params\n");
		return count;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_ERR("can not set test te for iris chip\n");
		return count;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return count;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_ERR("adfr is not supported\n");
		return count;
	}

	if (!gpio_is_valid(p_oplus_adfr_params->test_te.gpio)) {
		ADFR_ERR("test te gpio is invalid, should not set test te\n");
		return count;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_set_test_te_attr");

	sscanf(buf, "%u", &test_te_config);

	p_oplus_adfr_params->test_te.config = test_te_config;
	ADFR_INFO("oplus_adfr_test_te_config:%u\n", p_oplus_adfr_params->test_te.config);
	OPLUS_ADFR_TRACE_INT("oplus_adfr_test_te_config", p_oplus_adfr_params->test_te.config);

	test_te_irq = gpio_to_irq(p_oplus_adfr_params->test_te.gpio);
	if (p_oplus_adfr_params->test_te.config != OPLUS_ADFR_TEST_TE_DISABLE) {
		enable_irq(test_te_irq);
		ADFR_INFO("enable test te irq\n");
	} else {
		disable_irq(test_te_irq);
		ADFR_INFO("disable test te irq\n");
	}

	OPLUS_ADFR_TRACE_END("oplus_adfr_set_test_te_attr");

	ADFR_DEBUG("end\n");

	return count;
}

ssize_t oplus_adfr_get_test_te_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	unsigned int refresh_rate = 0;
	struct dsi_display *display = oplus_display_get_current_display();
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	ADFR_DEBUG("start\n");

	if (!buf || !display || !display->panel) {
		ADFR_ERR("invalid buf or display params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		ADFR_ERR("can not get test te refresh rate for iris chip\n");
		return -EFAULT;
	}
#endif /* CONFIG_PXLW_IRIS */

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params)) {
		ADFR_ERR("adfr is not supported\n");
		return -EINVAL;
	}

	if (!display->panel->cur_mode) {
		ADFR_ERR("invalid cur_mode param\n");
		return -EINVAL;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_adfr_get_test_te_attr");

	if (!gpio_is_valid(p_oplus_adfr_params->test_te.gpio)) {
		refresh_rate = display->panel->cur_mode->timing.refresh_rate;
		ADFR_INFO("test te gpio is invalid, use current timing refresh rate\n");
	} else {
		refresh_rate = p_oplus_adfr_params->test_te.refresh_rate;
	}

	ADFR_INFO("oplus_adfr_test_te_refresh_rate:%u\n", refresh_rate);

	OPLUS_ADFR_TRACE_END("oplus_adfr_get_test_te_attr");

	ADFR_DEBUG("end\n");

	return sprintf(buf, "%u\n", refresh_rate);
}
ssize_t oplus_display_set_high_precision_rscc(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	struct dsi_display *display = oplus_display_get_current_display();
	struct dsi_display_mode_priv_info *priv_info;
	struct oplus_adfr_params *p_oplus_adfr_params = NULL;

	if (!display || !display->panel || !display->panel->cur_mode) {
		ADFR_ERR("display is null\n");
		return -EINVAL;
	}
	priv_info = display->panel->cur_mode->priv_info;

	p_oplus_adfr_params = oplus_adfr_get_params(display->panel);
	if (!p_oplus_adfr_params) {
		ADFR_ERR("invalid p_oplus_adfr_params param\n");
		return -EINVAL;
	}

	if (!oplus_adfr_is_supported(p_oplus_adfr_params) || !oplus_adfr_high_precision_sa_mode_is_enabled(p_oplus_adfr_params)) {
		ADFR_DEBUG("sa high precision mode is not enabled\n");
		return 0;
	}

	OPLUS_ADFR_TRACE_BEGIN("oplus_display_set_high_precision_rscc");
	sscanf(buf, "%du", &(p_oplus_adfr_params->high_precision_rscc_state));
	if (p_oplus_adfr_params->high_precision_rscc_state == OPLUS_ADFR_RSCC_CLK_STATE)
		priv_info->disable_rsc_solver = true;
	else
		priv_info->disable_rsc_solver = false;
	ADFR_INFO("set high precision rscc %d", priv_info->disable_rsc_solver);
	OPLUS_ADFR_TRACE_INT("high_precision_rscc_state", priv_info->disable_rsc_solver);
	OPLUS_ADFR_TRACE_END("oplus_display_set_high_precision_rscc");
	return count;
}

ssize_t oplus_display_get_high_precision_rscc(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	struct dsi_display *display = oplus_display_get_current_display();
	struct dsi_display_mode_priv_info *priv_info;

	if (!display || !display->panel || !display->panel->cur_mode) {
		ADFR_ERR("display is null\n");
		return -EINVAL;
	}
	priv_info = display->panel->cur_mode->priv_info;

	ADFR_INFO("kVRR:oplus_display_get_high_precision_rscc %d\n", priv_info->disable_rsc_solver);
	return sprintf(buf, "%d\n", priv_info->disable_rsc_solver);
}

