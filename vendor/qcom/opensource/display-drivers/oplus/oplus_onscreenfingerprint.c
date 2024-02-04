/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
** File : oplus_onscreenfingerprint.c
** Description : oplus_onscreenfingerprint implement
** Version : 2.0
** Date : 2022/08/01
** Author : Display
***************************************************************/

#include "oplus_onscreenfingerprint.h"
#include "oplus_display_panel_seed.h"
#include "oplus_bl.h"
#include "oplus_display_interface.h"
#include "oplus_display_panel_common.h"
#include "sde_encoder_phys.h"
#include "sde_trace.h"
#include <linux/msm_drm_notify.h>
#include "../../../oplus/kernel/touchpanel/oplus_touchscreen_v2/touchpanel_notify/touchpanel_event_notify.h"

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
#include "oplus_adfr.h"
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
#include "oplus_display_temp_compensation.h"
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */

#if defined(CONFIG_PXLW_IRIS)
#include "dsi_iris_api.h"
#endif

/* -------------------- macro -------------------- */
/* fp type bit setting */
#define OPLUS_OFP_FP_TYPE_LCD_CAPACITIVE					(BIT(0))
#define OPLUS_OFP_FP_TYPE_OLED_CAPACITIVE					(BIT(1))
#define OPLUS_OFP_FP_TYPE_OPTICAL_OLD_SOLUTION				(BIT(2))
#define OPLUS_OFP_FP_TYPE_OPTICAL_NEW_SOLUTION				(BIT(3))
#define OPLUS_OFP_FP_TYPE_LOCAL_HBM							(BIT(4))
#define OPLUS_OFP_FP_TYPE_BRIGHTNESS_ADAPTATION				(BIT(5))
#define OPLUS_OFP_FP_TYPE_ULTRASONIC						(BIT(6))
#define OPLUS_OFP_FP_TYPE_ULTRA_LOW_POWER_AOD				(BIT(7))
/* get fp type config */
#define OPLUS_OFP_GET_LCD_CAPACITIVE_CONFIG(fp_type)		((fp_type) & OPLUS_OFP_FP_TYPE_LCD_CAPACITIVE)
#define OPLUS_OFP_GET_OLED_CAPACITIVE_CONFIG(fp_type)		((fp_type) & OPLUS_OFP_FP_TYPE_OLED_CAPACITIVE)
#define OPLUS_OFP_GET_OPTICAL_OLD_SOLUTION_CONFIG(fp_type)	((fp_type) & OPLUS_OFP_FP_TYPE_OPTICAL_OLD_SOLUTION)
#define OPLUS_OFP_GET_OPTICAL_NEW_SOLUTION_CONFIG(fp_type)	((fp_type) & OPLUS_OFP_FP_TYPE_OPTICAL_NEW_SOLUTION)
#define OPLUS_OFP_GET_LOCAL_HBM_CONFIG(fp_type)				((fp_type) & OPLUS_OFP_FP_TYPE_LOCAL_HBM)
#define OPLUS_OFP_GET_BRIGHTNESS_ADAPTATION_CONFIG(fp_type)	((fp_type) & OPLUS_OFP_FP_TYPE_BRIGHTNESS_ADAPTATION)
#define OPLUS_OFP_GET_ULTRASONIC_CONFIG(fp_type)			((fp_type) & OPLUS_OFP_FP_TYPE_ULTRASONIC)
#define OPLUS_OFP_GET_ULTRA_LOW_POWER_AOD_CONFIG(fp_type)	((fp_type) & OPLUS_OFP_FP_TYPE_ULTRA_LOW_POWER_AOD)
/* dbv level */
#define OPLUS_OFP_900NIT_DBV_LEVEL 							(0x0DBB)

#define to_dsi_display(x) container_of(x, struct dsi_display, host)

/* -------------------- parameters -------------------- */
/* log level config */
unsigned int oplus_ofp_log_level = OPLUS_OFP_LOG_LEVEL_DEBUG;
EXPORT_SYMBOL(oplus_ofp_log_level);
/* dual display id */
unsigned int oplus_ofp_display_id = OPLUS_OFP_PRIMARY_DISPLAY;
EXPORT_SYMBOL(oplus_ofp_display_id);
/* ofp global structure */
static struct oplus_ofp_params g_oplus_ofp_params[2] = {0};
/* a mutex lock used to avoid multiple concurrent invocations */
static DEFINE_MUTEX(oplus_ofp_lock);

/* -------------------- extern -------------------- */
/* extern params */
extern bool is_pvt_panel;
extern u32 oplus_last_backlight;

/* extern functions */

/* -------------------- oplus_ofp_params -------------------- */
static int oplus_ofp_set_display_id(unsigned int display_id)
{
	OFP_DEBUG("start\n");

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_set_display_id");

	oplus_ofp_display_id = display_id;
	OFP_INFO("oplus_ofp_display_id:%u\n", oplus_ofp_display_id);
	OPLUS_OFP_TRACE_INT("oplus_ofp_display_id", oplus_ofp_display_id);

	OPLUS_OFP_TRACE_END("oplus_ofp_set_display_id");

	OFP_DEBUG("end\n");

	return 0;
}

/* update display id for dual panel */
int oplus_ofp_update_display_id(void)
{
	struct dsi_display *display = oplus_display_get_current_display();

	OFP_DEBUG("start\n");

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_update_display_id");

	mutex_lock(&oplus_ofp_lock);

	if (!display) {
		OFP_ERR("failed to get current display, set default display id to 0\n");
		oplus_ofp_display_id = OPLUS_OFP_PRIMARY_DISPLAY;
	} else {
		if (!strcmp(display->display_type, "primary")) {
			oplus_ofp_display_id = OPLUS_OFP_PRIMARY_DISPLAY;
		} else if (!strcmp(display->display_type, "secondary")) {
			oplus_ofp_display_id = OPLUS_OFP_SECONDARY_DISPLAY;
		}
	}

	OFP_INFO("oplus_ofp_display_id:%u\n", oplus_ofp_display_id);
	OPLUS_OFP_TRACE_INT("oplus_ofp_display_id", oplus_ofp_display_id);

	mutex_unlock(&oplus_ofp_lock);

	OPLUS_OFP_TRACE_END("oplus_ofp_update_display_id");

	OFP_DEBUG("end\n");

	return 0;
}

static struct oplus_ofp_params *oplus_ofp_get_params(unsigned int display_id)
{
	return &g_oplus_ofp_params[display_id];
}

static int oplus_ofp_fp_type_compatible_mode_config(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_fp_type_compatible_mode_config");

	if (p_oplus_ofp_params->fp_type_compatible_mode) {
		if (!is_pvt_panel) {
			/* these panels do not support lhbm */
			p_oplus_ofp_params->fp_type = 0x88;
			OFP_INFO("fp_type:0x%x\n", p_oplus_ofp_params->fp_type);
		}
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_fp_type_compatible_mode_config");

	OFP_DEBUG("end\n");

	return 0;
}

/* get fp_type value from panel dtsi */
int oplus_ofp_init(void *dsi_panel)
{
	int rc = 0;
	unsigned int fp_type = 0;
	struct dsi_panel *panel = dsi_panel;
	struct dsi_parser_utils *utils = NULL;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!panel || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		OFP_INFO("no need to init secondary panel for iris chip\n");
		return 0;
	}
#endif

	utils = &panel->utils;
	if (!utils) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (!strcmp(panel->type, "secondary")) {
		p_oplus_ofp_params = oplus_ofp_get_params(OPLUS_OFP_SECONDARY_DISPLAY);
		if (!p_oplus_ofp_params) {
			OFP_ERR("Invalid params\n");
			return -EINVAL;
		}
		OFP_INFO("init secondary display ofp params\n");
		oplus_ofp_set_display_id(OPLUS_OFP_SECONDARY_DISPLAY);
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_init");

	mutex_init(&oplus_ofp_lock);

	rc = utils->read_u32(utils->data, "oplus,ofp-fp-type", &fp_type);
	if (rc) {
		OFP_INFO("failed to read oplus,ofp-fp-type, rc=%d\n", rc);
		/* set default value to BIT(0) */
		p_oplus_ofp_params->fp_type = BIT(0);
	} else {
		p_oplus_ofp_params->fp_type = fp_type;
	}

	OFP_INFO("fp_type:0x%x\n", p_oplus_ofp_params->fp_type);

	/* indicates whether fp type compatible mode is set or not */
	p_oplus_ofp_params->fp_type_compatible_mode = utils->read_bool(utils->data, "oplus,ofp-fp-type-compatible-mode");
	OFP_INFO("fp_type_compatible_mode:%d\n", p_oplus_ofp_params->fp_type_compatible_mode);
	oplus_ofp_fp_type_compatible_mode_config();

	if (oplus_ofp_is_supported()) {
		/* indicates whether gamut needs to be bypassed in aod/fod scenarios or not */
		p_oplus_ofp_params->need_to_bypass_gamut = utils->read_bool(utils->data, "oplus,ofp-need-to-bypass-gamut");
		OFP_INFO("need_to_bypass_gamut:%d\n", p_oplus_ofp_params->need_to_bypass_gamut);

		/* indicates whether display on cmd(29h) needs to be sent after image data write before aod on or not */
		p_oplus_ofp_params->need_to_wait_data_before_aod_on = utils->read_bool(utils->data, "oplus,ofp-need-to-wait-data-before-aod-on");
		OFP_INFO("need_to_wait_data_before_aod_on:%d\n", p_oplus_ofp_params->need_to_wait_data_before_aod_on);

		if (p_oplus_ofp_params->need_to_wait_data_before_aod_on) {
			/* add workqueue to send display on(29) cmd */
			if (!strcmp(panel->type, "primary")) {
				p_oplus_ofp_params->aod_display_on_set_wq = create_singlethread_workqueue("aod_display_on_set_0");
			} else if (!strcmp(panel->type, "secondary")) {
				p_oplus_ofp_params->aod_display_on_set_wq = create_singlethread_workqueue("aod_display_on_set_1");
			}

			INIT_WORK(&p_oplus_ofp_params->aod_display_on_set_work, oplus_ofp_aod_display_on_set_work_handler);
		}

		if (!oplus_ofp_oled_capacitive_is_enabled()) {
			/* add workqueue to send ui ready and aod off cmds */
			if (!strcmp(panel->type, "primary")) {
				p_oplus_ofp_params->uiready_event_wq = create_singlethread_workqueue("uiready_event_0");
				p_oplus_ofp_params->aod_off_set_wq = create_singlethread_workqueue("aod_off_set_0");
			} else if (!strcmp(panel->type, "secondary")) {
				p_oplus_ofp_params->uiready_event_wq = create_singlethread_workqueue("uiready_event_1");
				p_oplus_ofp_params->aod_off_set_wq = create_singlethread_workqueue("aod_off_set_1");
			}

			INIT_WORK(&p_oplus_ofp_params->uiready_event_work, oplus_ofp_uiready_event_work_handler);
			INIT_WORK(&p_oplus_ofp_params->aod_off_set_work, oplus_ofp_aod_off_set_work_handler);

			/* add for touchpanel event notifier */
			p_oplus_ofp_params->touchpanel_event_notifier.notifier_call = oplus_ofp_touchpanel_event_notifier_call;
			rc = touchpanel_event_register_notifier(&p_oplus_ofp_params->touchpanel_event_notifier);
			if (rc) {
				OFP_ERR("failed to register touchpanel event notifier, rc=%d\n", rc);
			}
		}

		if (oplus_ofp_local_hbm_is_enabled()) {
			/* indicates whether lhbm pressed icon gamma needs to be read and updated or not */
			p_oplus_ofp_params->need_to_update_lhbm_pressed_icon_gamma = utils->read_bool(utils->data, "oplus,ofp-need-to-update-lhbm-pressed-icon-gamma");
			OFP_INFO("need_to_update_lhbm_pressed_icon_gamma:%d\n", p_oplus_ofp_params->need_to_update_lhbm_pressed_icon_gamma);
		}
	}

	if (!strcmp(panel->type, "secondary")) {
		/* set default display id to primary display */
		oplus_ofp_set_display_id(OPLUS_OFP_PRIMARY_DISPLAY);
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_init");

	OFP_DEBUG("end\n");

	return rc;
}

bool oplus_ofp_is_supported(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return false;
	}

	/* global config, support oled panel */
	return (bool)(!OPLUS_OFP_GET_LCD_CAPACITIVE_CONFIG(p_oplus_ofp_params->fp_type));
}

bool oplus_ofp_oled_capacitive_is_enabled(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return false;
	}

	if (!oplus_ofp_is_supported()) {
		OFP_DEBUG("ofp is not support, oled capacitive is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_OFP_GET_OLED_CAPACITIVE_CONFIG(p_oplus_ofp_params->fp_type));
}

bool oplus_ofp_optical_new_solution_is_enabled(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return false;
	}

	if (!oplus_ofp_is_supported()) {
		OFP_DEBUG("ofp is not support, optical new solution is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_OFP_GET_OPTICAL_NEW_SOLUTION_CONFIG(p_oplus_ofp_params->fp_type));
}

bool oplus_ofp_local_hbm_is_enabled(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return false;
	}

	if (!oplus_ofp_is_supported()) {
		OFP_DEBUG("ofp is not support, local hbm is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_OFP_GET_LOCAL_HBM_CONFIG(p_oplus_ofp_params->fp_type));
}
/*
static bool oplus_ofp_brightness_adaptation_is_enabled(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return false;
	}

	if (!oplus_ofp_is_supported()) {
		OFP_DEBUG("ofp is not support, brightness adaptation is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_OFP_GET_BRIGHTNESS_ADAPTATION_CONFIG(p_oplus_ofp_params->fp_type));
}
*/
bool oplus_ofp_ultrasonic_is_enabled(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return false;
	}

	if (!oplus_ofp_is_supported()) {
		OFP_DEBUG("ofp is not support, ultrasonic is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_OFP_GET_ULTRASONIC_CONFIG(p_oplus_ofp_params->fp_type));
}

bool oplus_ofp_ultra_low_power_aod_is_enabled(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return false;
	}

	if (!oplus_ofp_is_supported()) {
		OFP_DEBUG("ofp is not support, ultra low power aod is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_OFP_GET_ULTRA_LOW_POWER_AOD_CONFIG(p_oplus_ofp_params->fp_type));
}

bool oplus_ofp_get_hbm_state(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return 0;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_get_hbm_state");
	OFP_DEBUG("oplus_ofp_get_hbm_state:%d\n", p_oplus_ofp_params->hbm_state);
	OPLUS_OFP_TRACE_END("oplus_ofp_get_hbm_state");

	OFP_DEBUG("end\n");

	return p_oplus_ofp_params->hbm_state;
}

static int oplus_ofp_set_hbm_state(bool hbm_state)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_set_hbm_state");

	p_oplus_ofp_params->hbm_state = hbm_state;
	OFP_INFO("oplus_ofp_hbm_state:%d\n", hbm_state);
	OPLUS_OFP_TRACE_INT("oplus_ofp_hbm_state", p_oplus_ofp_params->hbm_state);

	oplus_ofp_send_hbm_state_event(hbm_state);

	OPLUS_OFP_TRACE_END("oplus_ofp_set_hbm_state");

	OFP_DEBUG("end\n");

	return 0;
}

static bool oplus_ofp_get_aod_state(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return 0;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_get_aod_state");
	OFP_DEBUG("oplus_ofp_aod_state:%d\n", p_oplus_ofp_params->aod_state);
	OPLUS_OFP_TRACE_END("oplus_ofp_get_aod_state");

	OFP_DEBUG("end\n");

	return p_oplus_ofp_params->aod_state;
}

static int oplus_ofp_set_aod_state(bool aod_state)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_set_aod_state");

	p_oplus_ofp_params->aod_state = aod_state;
	OFP_INFO("oplus_ofp_aod_state:%d\n", p_oplus_ofp_params->aod_state);
	OPLUS_OFP_TRACE_INT("oplus_ofp_aod_state", p_oplus_ofp_params->aod_state);

	OPLUS_OFP_TRACE_END("oplus_ofp_set_aod_state");

	OFP_DEBUG("end\n");

	return 0;
}

bool oplus_ofp_get_ultra_low_power_aod_state(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return false;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_get_ultra_low_power_aod_state");
	OFP_DEBUG("oplus_ofp_ultra_low_power_aod_state:%d\n", p_oplus_ofp_params->ultra_low_power_aod_state);
	OPLUS_OFP_TRACE_END("oplus_ofp_get_ultra_low_power_aod_state");

	OFP_DEBUG("end\n");

	return p_oplus_ofp_params->ultra_low_power_aod_state;
}

/* aod unlocking value update */
static int oplus_ofp_aod_unlocking_update(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_aod_unlocking_update");

	if (p_oplus_ofp_params->fp_press || p_oplus_ofp_params->doze_active) {
		/* press icon layer is ready in aod mode */
		p_oplus_ofp_params->aod_unlocking = true;
		OFP_INFO("oplus_ofp_aod_unlocking:%d\n", p_oplus_ofp_params->aod_unlocking);
		OPLUS_OFP_TRACE_INT("oplus_ofp_aod_unlocking", p_oplus_ofp_params->aod_unlocking);
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_aod_unlocking_update");

	OFP_DEBUG("end\n");

	return 0;
}

/* update hbm_enable property value */
int oplus_ofp_property_update(void *sde_connector, void *sde_connector_state, int prop_id, uint64_t prop_val)
{
	struct sde_connector *c_conn = sde_connector;
	struct sde_connector_state *c_state = sde_connector_state;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!c_conn || !c_state || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		OFP_DEBUG("not in dsi mode, should not update ofp properties\n");
		return 0;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_property_update");

	switch (prop_id) {
	case CONNECTOR_PROP_HBM_ENABLE:
		if (prop_val != p_oplus_ofp_params->hbm_enable) {
			OFP_INFO("HBM_ENABLE:%lu,dim:%lu,fingerpress:%lu,icon:%lu,aod:%lu\n", prop_val, (prop_val & OPLUS_OFP_PROPERTY_DIM_LAYER),
				(prop_val & OPLUS_OFP_PROPERTY_FINGERPRESS_LAYER), (prop_val & OPLUS_OFP_PROPERTY_ICON_LAYER),
					(prop_val & OPLUS_OFP_PROPERTY_AOD_LAYER));
		}
		p_oplus_ofp_params->hbm_enable = prop_val;
		OPLUS_OFP_TRACE_INT("oplus_ofp_hbm_enable", p_oplus_ofp_params->hbm_enable);

		msm_property_set_dirty(&c_conn->property_info, &c_state->property_state, CONNECTOR_PROP_HBM_ENABLE);
		break;

	default:
		break;
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_property_update");

	OFP_DEBUG("end\n");

	return 0;
}

/* -------------------- fod -------------------- */
int oplus_ofp_parse_dtsi_config(void *dsi_display_mode, void *dsi_parser_utils)
{
	unsigned int data = 0;
	int rc = 0;
	struct dsi_display_mode *mode = dsi_display_mode;
	struct dsi_parser_utils *utils = dsi_parser_utils;
	struct dsi_display_mode_priv_info *priv_info;

	OFP_DEBUG("start\n");

	if (oplus_ofp_oled_capacitive_is_enabled() || oplus_ofp_ultrasonic_is_enabled()) {
		OFP_DEBUG("no need to parse ofp dtsi config\n");
		return 0;
	}

	if (!mode || !mode->priv_info)
		return -EINVAL;

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_parse_dtsi_config");

	priv_info = mode->priv_info;

	/* indicates whether need to separate backlight before hbm on in power on mode or not */
	priv_info->oplus_ofp_need_to_separate_backlight = utils->read_bool(utils->data, "oplus,ofp-need-to-separate-backlight");
	OFP_DEBUG("oplus_ofp_need_to_separate_backlight:%d\n", priv_info->oplus_ofp_need_to_separate_backlight);

	/*
	 indicates whether need to sync data(dim layer or fingerpress layer) in aod unlocking or not
	 ps: it will remove all the delay to speed up aod unlocking by default
	*/
	priv_info->oplus_ofp_need_to_sync_data_in_aod_unlocking = utils->read_bool(utils->data, "oplus,ofp-need-to-sync-data-in-aod-unlocking");
	OFP_DEBUG("oplus_ofp_need_to_sync_data_in_aod_unlocking:%d\n", priv_info->oplus_ofp_need_to_sync_data_in_aod_unlocking);

	/* indicates how many frames does backlight on cmd take effect */
	rc = utils->read_u32(utils->data, "oplus,ofp-backlight-on-period", &data);
	if (rc) {
		OFP_DEBUG("failed to parse oplus,ofp-backlight-on-period\n");
		priv_info->oplus_ofp_backlight_on_period = 1;
	} else {
		priv_info->oplus_ofp_backlight_on_period = data;
	}
	OFP_DEBUG("oplus_ofp_backlight_on_period:%u\n", priv_info->oplus_ofp_backlight_on_period);

	/* indicates how many frames does hbm on cmds take effect */
	rc = utils->read_u32(utils->data, "oplus,ofp-hbm-on-period", &data);
	if (rc) {
		OFP_DEBUG("failed to parse oplus,ofp-hbm-on-period\n");
		priv_info->oplus_ofp_hbm_on_period = 1;
	} else {
		priv_info->oplus_ofp_hbm_on_period = data;
	}
	OFP_DEBUG("oplus_ofp_hbm_on_period:%u\n", priv_info->oplus_ofp_hbm_on_period);

	/*
	 check how many black frames are inserted in aod off cmds flow which will affect hbm on cmds execution time,
	 then calculate delay time to separate aod off cmds and hbm on cmds to make sure ui ready is accurate
	*/
	rc = utils->read_u32(utils->data, "oplus,ofp-aod-off-insert-black-frame", &data);
	if (rc) {
		OFP_DEBUG("failed to parse oplus,ofp-aod-off-insert-black-frame\n");
		priv_info->oplus_ofp_aod_off_insert_black_frame = 0;
	} else {
		priv_info->oplus_ofp_aod_off_insert_black_frame = data;
	}
	OFP_DEBUG("oplus_ofp_aod_off_insert_black_frame:%u\n", priv_info->oplus_ofp_aod_off_insert_black_frame);

	/* check the total time of black frames by oscilloscope, which will be used to check whether hbm cmds are sent within black frames or not */
	rc = utils->read_u32(utils->data, "oplus,ofp-aod-off-black-frame-total-time", &data);
	if (rc) {
		OFP_DEBUG("failed to parse oplus,ofp-aod-off-black-frame-total-time\n");
		priv_info->oplus_ofp_aod_off_black_frame_total_time = 0;
	} else {
		priv_info->oplus_ofp_aod_off_black_frame_total_time = data;
	}
	OFP_DEBUG("oplus_ofp_aod_off_black_frame_total_time:%u\n", priv_info->oplus_ofp_aod_off_black_frame_total_time);

	OPLUS_OFP_TRACE_END("oplus_ofp_parse_dtsi_config");

	OFP_DEBUG("end\n");

	return 0;
}

/* cmd set */
static int oplus_ofp_panel_cmd_set_nolock(void *dsi_panel, enum dsi_cmd_set_type type)
{
	int rc = 0;
	int seed_mode = 0;
	unsigned int refresh_rate = 0;
	unsigned int us_per_frame = 0;
	unsigned int delay_us = 0;
	struct dsi_panel *panel = dsi_panel;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!panel || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_panel_cmd_set_nolock");

	if (!dsi_panel_initialized(panel)) {
		rc = -EINVAL;
		OFP_DEBUG("should not send cmd sets if panel is not initialized\n");
		goto error;
	}

	OPLUS_OFP_TRACE_BEGIN("dsi_panel_tx_cmd_set");
	rc = dsi_panel_tx_cmd_set(panel, type);
	OPLUS_OFP_TRACE_END("dsi_panel_tx_cmd_set");
	if (rc) {
		OFP_ERR("[%s] failed to send %s, rc=%d\n",
			panel->name, cmd_set_prop_map[type], rc);
		goto error;
	}

	/* after tx cmd set */
	switch (type) {
	case DSI_CMD_HBM_ON:
		oplus_ofp_set_hbm_state(true);
		oplus_hbm_pwm_state(panel, true);

		/* do not use loading effect compensation mode to FOD sensing, it causes the luminance drop in FOD pattern */
		OPLUS_OFP_TRACE_BEGIN("dsi_panel_seed_mode");
		rc = dsi_panel_seed_mode(panel, PANEL_LOADING_EFFECT_OFF);
		if (rc) {
			OFP_ERR("failed to set seed mode:PANEL_LOADING_EFFECT_OFF\n");
		}
		OPLUS_OFP_TRACE_END("dsi_panel_seed_mode");

#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
		if (oplus_temp_compensation_is_supported()) {
			rc = oplus_temp_compensation_cmd_set(panel, OPLUS_TEMP_COMPENSATION_FOD_ON_SETTING);
			if (rc) {
				OFP_ERR("failed to set temp compensation cmd, rc=%d\n", rc);
			}
		}
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */
		break;

	case DSI_CMD_HBM_OFF:
		oplus_ofp_set_hbm_state(false);
		oplus_hbm_pwm_state(panel, false);

		/*
		 if backlight level is in global hbm range before hbm on, reset the oplus_global_hbm_flags,
		 so that it can reenter global hbm level after hbm off
		*/
		if (oplus_display_panel_get_global_hbm_status()) {
			oplus_display_panel_set_global_hbm_status(GLOBAL_HBM_DISABLE);
		}

		/* recovery backlight level */
		OPLUS_OFP_TRACE_BEGIN("dsi_panel_set_backlight");
		rc = dsi_panel_set_backlight(panel, panel->bl_config.bl_level);
		OPLUS_OFP_TRACE_END("dsi_panel_set_backlight");
		if (rc) {
			OFP_ERR("unable to set backlight\n");
			goto error;
		}

		/*
		 the normal backlight of some panel takes effect for more than one frame,
		 so increase the corresponding delay to ensure that the dim layer can match the brightness
		*/
		if (panel->cur_mode->priv_info->oplus_ofp_backlight_on_period > 1) {
			refresh_rate = panel->cur_mode->timing.refresh_rate;
			us_per_frame = 1000000/refresh_rate;
			delay_us = (panel->cur_mode->priv_info->oplus_ofp_backlight_on_period - 1) * us_per_frame;

			OPLUS_OFP_TRACE_BEGIN("usleep_range");
			usleep_range(delay_us, (delay_us + 10));
			OFP_INFO("usleep_range %u done\n", delay_us);
			OPLUS_OFP_TRACE_END("usleep_range");
		}

		/* recovery loading effect mode */
		seed_mode = __oplus_get_seed_mode();
		OPLUS_OFP_TRACE_BEGIN("dsi_panel_seed_mode");
		rc = dsi_panel_seed_mode(panel, seed_mode);
		if (rc) {
			OFP_ERR("failed to set seed mode:%d\n", seed_mode);
		}
		OPLUS_OFP_TRACE_END("dsi_panel_seed_mode");
		break;

	case DSI_CMD_LHBM_PRESSED_ICON_ON:
		oplus_ofp_set_hbm_state(true);
		oplus_hbm_pwm_state(panel, true);

		if (p_oplus_ofp_params->need_to_update_lhbm_pressed_icon_gamma && (panel->bl_config.bl_level > OPLUS_OFP_900NIT_DBV_LEVEL)) {
			OFP_INFO("set backlight level to OPLUS_OFP_900NIT_DBV_LEVEL after pressed icon on\n");
			OPLUS_OFP_TRACE_BEGIN("dsi_panel_set_backlight");
			rc = dsi_panel_set_backlight(panel, OPLUS_OFP_900NIT_DBV_LEVEL);
			OPLUS_OFP_TRACE_END("dsi_panel_set_backlight");
			if (rc) {
				OFP_ERR("unable to set backlight\n");
			}
		}
		break;

	case DSI_CMD_LHBM_PRESSED_ICON_OFF:
		oplus_ofp_set_hbm_state(false);
		oplus_hbm_pwm_state(panel, false);

		/*
		 if backlight level is in global hbm range before hbm on, reset the oplus_global_hbm_flags,
		 so that it can reenter global hbm level after hbm off
		*/
		if (oplus_display_panel_get_global_hbm_status()) {
			oplus_display_panel_set_global_hbm_status(GLOBAL_HBM_DISABLE);
		}

		/* recovery backlight level */
		OPLUS_OFP_TRACE_BEGIN("dsi_panel_set_backlight");
		rc = dsi_panel_set_backlight(panel, panel->bl_config.bl_level);
		OPLUS_OFP_TRACE_END("dsi_panel_set_backlight");
		if (rc) {
			OFP_ERR("unable to set backlight\n");
			goto error;
		}

		/*
		 the normal backlight of some panel takes effect for more than one frame,
		 so increase the corresponding delay to ensure that the dim layer can match the brightness
		*/
		if (panel->cur_mode->priv_info->oplus_ofp_backlight_on_period > 1) {
			refresh_rate = panel->cur_mode->timing.refresh_rate;
			us_per_frame = 1000000/refresh_rate;
			delay_us = (panel->cur_mode->priv_info->oplus_ofp_backlight_on_period - 1) * us_per_frame;

			OPLUS_OFP_TRACE_BEGIN("usleep_range");
			usleep_range(delay_us, (delay_us + 10));
			OFP_INFO("usleep_range %u done\n", delay_us);
			OPLUS_OFP_TRACE_END("usleep_range");
		}
		break;

	case DSI_CMD_ULTRA_LOW_POWER_AOD_ON:
		p_oplus_ofp_params->ultra_low_power_aod_state = true;
		OFP_INFO("ultra_low_power_aod_state:%d\n", p_oplus_ofp_params->ultra_low_power_aod_state);
		OPLUS_OFP_TRACE_INT("oplus_ofp_ultra_low_power_aod_state", p_oplus_ofp_params->ultra_low_power_aod_state);
		break;

	case DSI_CMD_ULTRA_LOW_POWER_AOD_OFF:
		p_oplus_ofp_params->ultra_low_power_aod_state = false;
		OFP_INFO("ultra_low_power_aod_state:%d\n", p_oplus_ofp_params->ultra_low_power_aod_state);
		OPLUS_OFP_TRACE_INT("oplus_ofp_ultra_low_power_aod_state", p_oplus_ofp_params->ultra_low_power_aod_state);
		break;

	default:
		break;
	}

error:
	OPLUS_OFP_TRACE_END("oplus_ofp_panel_cmd_set_nolock");
	OFP_DEBUG("end\n");

	return rc;
}

static int oplus_ofp_panel_cmd_set(void *dsi_panel, enum dsi_cmd_set_type type)
{
	int rc = 0;
	struct dsi_panel *panel = dsi_panel;

	OFP_DEBUG("start\n");

	if (!panel) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_panel_cmd_set");

	mutex_lock(&panel->panel_lock);

	if (!dsi_panel_initialized(panel)) {
		rc = -EINVAL;
		OFP_DEBUG("should not send cmd sets if panel is not initialized\n");
		goto error;
	}

	rc = oplus_ofp_panel_cmd_set_nolock(panel, type);
	if (rc) {
		OFP_ERR("[%s] failed to send %s, rc=%d\n",
			panel->name, cmd_set_prop_map[type], rc);
	}

error:
	mutex_unlock(&panel->panel_lock);

	OPLUS_OFP_TRACE_END("oplus_ofp_panel_cmd_set");
	OFP_DEBUG("end\n");

	return rc;
}

/* uniform interface for cmd set */
static int oplus_ofp_display_cmd_set(void *dsi_display, enum dsi_cmd_set_type type)
{
	int rc = 0;
	struct dsi_display *display = dsi_display;

	OFP_DEBUG("start\n");

	if (!display || !display->panel) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_display_cmd_set");

	mutex_lock(&display->display_lock);

	/* enable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
			DSI_CORE_CLK, DSI_CLK_ON);
		if (rc) {
			OFP_ERR("[%s] failed to enable DSI clocks, rc=%d\n", display->name, rc);
			goto error;
		}
	}

	rc = oplus_ofp_panel_cmd_set(display->panel, type);
	if (rc) {
		OFP_ERR("[%s] failed to send %s, rc=%d\n",
			display->name, cmd_set_prop_map[type], rc);
	}

	/* disable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
			DSI_CORE_CLK, DSI_CLK_OFF);
		if (rc) {
			OFP_ERR("[%s] failed to disable DSI clocks, rc=%d\n", display->name, rc);
		}
	}

	if (type == DSI_CMD_LHBM_PRESSED_ICON_OFF)
		oplus_panel_set_lhbm_off_te_timestamp(display->panel);

error:
	mutex_unlock(&display->display_lock);

	OPLUS_OFP_TRACE_END("oplus_ofp_display_cmd_set");
	OFP_DEBUG("end\n");

	return rc;
}

/*
 due to the fact that the brightness of lhbm pressed icon may change with the backlight,
 it is necessary to readjust the lhbm pressed icon gamma to meet the requirements of fingerprint unlocking
*/
int oplus_ofp_lhbm_pressed_icon_gamma_update(void *dsi_display)
{
	static bool calibrated = false;
	unsigned char rx_buf_0[5] = {0};
	unsigned char rx_buf_1[5] = {0};
	unsigned char *tx_buf = NULL;
	static unsigned char extrapolated_value[5][6] = {0};
	int rc = 0;
	int rgb_data_0[3] = {0};
	int rgb_data_1[3] = {0};
	int rgb_data_2[3] = {0};
	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int lcm_cmd_count = 0;
	static unsigned int failure_count = 0;
	struct dsi_display *display = dsi_display;
	struct dsi_cmd_desc *cmds = NULL;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!oplus_ofp_local_hbm_is_enabled()) {
		OFP_DEBUG("local hbm is not enabled, no need to update lhbm pressed icon gamma\n");
		return 0;
	}

	if (!display || !display->panel || !display->panel->cur_mode || !display->panel->cur_mode->priv_info || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (!p_oplus_ofp_params->need_to_update_lhbm_pressed_icon_gamma) {
		OFP_DEBUG("need_to_update_lhbm_pressed_icon_gamma is not config, no need to update lhbm pressed icon gamma\n");
		return 0;
	}

	if (!dsi_panel_initialized(display->panel) || display->panel->bl_config.bl_level) {
		OFP_DEBUG("should not update lhbm pressed icon gamma if panel is not initialized or panel backlight is turned on\n");
		return 0;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_lhbm_pressed_icon_gamma_update");

	if (!calibrated && (failure_count < 100)) {
		/* read lhbm pressed icon gamma */
		rc = oplus_ofp_display_cmd_set(display, DSI_CMD_PANEL_READ_REGISTER_OPEN);
		if (rc) {
			OFP_ERR("[%s] failed to send DSI_CMD_PANEL_READ_REGISTER_OPEN cmds, rc=%d\n", display->name, rc);
			rc = oplus_ofp_display_cmd_set(display, DSI_CMD_PANEL_READ_REGISTER_CLOSE);
			if (rc) {
				OFP_ERR("[%s] failed to send DSI_CMD_PANEL_READ_REGISTER_CLOSE cmds, rc=%d\n", display->name, rc);
			}
			goto error;
		}

		rc = dsi_display_read_panel_reg(display, 0x96, rx_buf_0, 5);
		if (rc) {
			OFP_ERR("failed to read panel reg 0x96, rc=%d\n", rc);
			rc = oplus_ofp_display_cmd_set(display, DSI_CMD_PANEL_READ_REGISTER_CLOSE);
			if (rc) {
				OFP_ERR("[%s] failed to send DSI_CMD_PANEL_READ_REGISTER_CLOSE cmds, rc=%d\n", display->name, rc);
			}
			goto error;
		}

		rc = dsi_display_read_panel_reg(display, 0x97, rx_buf_1, 5);
		if (rc) {
			OFP_ERR("failed to read panel reg 0x97, rc=%d\n", rc);
			rc = oplus_ofp_display_cmd_set(display, DSI_CMD_PANEL_READ_REGISTER_CLOSE);
			if (rc) {
				OFP_ERR("[%s] failed to send DSI_CMD_PANEL_READ_REGISTER_CLOSE cmds, rc=%d\n", display->name, rc);
			}
			goto error;
		}

		rc = oplus_ofp_display_cmd_set(display, DSI_CMD_PANEL_READ_REGISTER_CLOSE);
		if (rc) {
			OFP_ERR("[%s] failed to send DSI_CMD_PANEL_READ_REGISTER_CLOSE cmds, rc=%d\n", display->name, rc);
			goto error;
		}
		for (i = 0; i < 5; i++) {
			OFP_INFO("rx_buf_0[%u] = 0x%02X, rx_buf_1[%u] = 0x%02X\n", i, rx_buf_0[i], i, rx_buf_1[i]);
		}

		/* readjust the corresponding gamma value through extrapolation */
		rgb_data_0[0] = ((rx_buf_0[0] & 0x0F) << 8) | rx_buf_0[2];
		rgb_data_0[1] = ((rx_buf_0[1] & 0xF0) << 4) | rx_buf_0[3];
		rgb_data_0[2] = ((rx_buf_0[1] & 0x0F) << 8) | rx_buf_0[4];
		rgb_data_1[0] = ((rx_buf_1[0] & 0x0F) << 8) | rx_buf_1[2];
		rgb_data_1[1] = ((rx_buf_1[1] & 0xF0) << 4) | rx_buf_1[3];
		rgb_data_1[2] = ((rx_buf_1[1] & 0x0F) << 8) | rx_buf_1[4];

		rgb_data_2[0] = (173 * 110 - 24700) * (rgb_data_1[0] - rgb_data_0[0]) / 800 + rgb_data_0[0];
		rgb_data_2[1] = (173 * 110 - 24700) * (rgb_data_1[1] - rgb_data_0[1]) / 800 + rgb_data_0[1];
		rgb_data_2[2] = (173 * 110 - 24700) * (rgb_data_1[2] - rgb_data_0[2]) / 800 + rgb_data_0[2];
		extrapolated_value[0][0] = 0x93;
		extrapolated_value[0][1] = rgb_data_2[0] >> 8;
		extrapolated_value[0][2] = ((rgb_data_2[1] & 0xF00) >> 4) | ((rgb_data_2[2] & 0xF00) >> 8);
		extrapolated_value[0][3] = rgb_data_2[0] & 0xFF;
		extrapolated_value[0][4] = rgb_data_2[1] & 0xFF;
		extrapolated_value[0][5] = rgb_data_2[2] & 0xFF;
		if (!extrapolated_value[0][1] && !extrapolated_value[0][2] && !extrapolated_value[0][3]
				&& !extrapolated_value[0][4] && !extrapolated_value[0][5]) {
			OFP_ERR("the extrapolated_value[0] are incorrect\n");
			rc = -EINVAL;
			goto error;
		}

		rgb_data_2[0] = (205 * 110 - 24700) * (rgb_data_1[0] - rgb_data_0[0]) / 800 + rgb_data_0[0];
		rgb_data_2[1] = (205 * 110 - 24700) * (rgb_data_1[1] - rgb_data_0[1]) / 800 + rgb_data_0[1];
		rgb_data_2[2] = (205 * 110 - 24700) * (rgb_data_1[2] - rgb_data_0[2]) / 800 + rgb_data_0[2];
		extrapolated_value[1][0] = 0x94;
		extrapolated_value[1][1] = rgb_data_2[0] >> 8;
		extrapolated_value[1][2] = ((rgb_data_2[1] & 0xF00) >> 4) | ((rgb_data_2[2] & 0xF00) >> 8);
		extrapolated_value[1][3] = rgb_data_2[0] & 0xFF;
		extrapolated_value[1][4] = rgb_data_2[1] & 0xFF;
		extrapolated_value[1][5] = rgb_data_2[2] & 0xFF;
		if (!extrapolated_value[1][1] && !extrapolated_value[1][2] && !extrapolated_value[1][3]
				&& !extrapolated_value[1][4] && !extrapolated_value[1][5]) {
			OFP_ERR("the extrapolated_value[1] are incorrect\n");
			rc = -EINVAL;
			goto error;
		}

		rgb_data_2[0] = (226 * 110 - 24700) * (rgb_data_1[0] - rgb_data_0[0]) / 800 + rgb_data_0[0];
		rgb_data_2[1] = (226 * 110 - 24700) * (rgb_data_1[1] - rgb_data_0[1]) / 800 + rgb_data_0[1];
		rgb_data_2[2] = (226 * 110 - 24700) * (rgb_data_1[2] - rgb_data_0[2]) / 800 + rgb_data_0[2];
		extrapolated_value[2][0] = 0x95;
		extrapolated_value[2][1] = rgb_data_2[0] >> 8;
		extrapolated_value[2][2] = ((rgb_data_2[1] & 0xF00) >> 4) | ((rgb_data_2[2] & 0xF00) >> 8);
		extrapolated_value[2][3] = rgb_data_2[0] & 0xFF;
		extrapolated_value[2][4] = rgb_data_2[1] & 0xFF;
		extrapolated_value[2][5] = rgb_data_2[2] & 0xFF;
		if (!extrapolated_value[2][1] && !extrapolated_value[2][2] && !extrapolated_value[2][3]
				&& !extrapolated_value[2][4] && !extrapolated_value[2][5]) {
			OFP_ERR("the extrapolated_value[2] are incorrect\n");
			rc = -EINVAL;
			goto error;
		}

		rgb_data_2[0] = (247 * 110 - 24700) * (rgb_data_1[0] - rgb_data_0[0]) / 800 + rgb_data_0[0];
		rgb_data_2[1] = (247 * 110 - 24700) * (rgb_data_1[1] - rgb_data_0[1]) / 800 + rgb_data_0[1];
		rgb_data_2[2] = (247 * 110 - 24700) * (rgb_data_1[2] - rgb_data_0[2]) / 800 + rgb_data_0[2];
		extrapolated_value[3][0] = 0x96;
		extrapolated_value[3][1] = rgb_data_2[0] >> 8;
		extrapolated_value[3][2] = ((rgb_data_2[1] & 0xF00) >> 4) | ((rgb_data_2[2] & 0xF00) >> 8);
		extrapolated_value[3][3] = rgb_data_2[0] & 0xFF;
		extrapolated_value[3][4] = rgb_data_2[1] & 0xFF;
		extrapolated_value[3][5] = rgb_data_2[2] & 0xFF;
		if (!extrapolated_value[3][1] && !extrapolated_value[3][2] && !extrapolated_value[3][3]
				&& !extrapolated_value[3][4] && !extrapolated_value[3][5]) {
			OFP_ERR("the extrapolated_value[3] are incorrect\n");
			rc = -EINVAL;
			goto error;
		}

		rgb_data_2[0] = (255 * 110 - 24700) * (rgb_data_1[0] - rgb_data_0[0]) / 800 + rgb_data_0[0];
		rgb_data_2[1] = (255 * 110 - 24700) * (rgb_data_1[1] - rgb_data_0[1]) / 800 + rgb_data_0[1];
		rgb_data_2[2] = (255 * 110 - 24700) * (rgb_data_1[2] - rgb_data_0[2]) / 800 + rgb_data_0[2];
		extrapolated_value[4][0] = 0x97;
		extrapolated_value[4][1] = rgb_data_2[0] >> 8;
		extrapolated_value[4][2] = ((rgb_data_2[1] & 0xF00) >> 4) | ((rgb_data_2[2] & 0xF00) >> 8);
		extrapolated_value[4][3] = rgb_data_2[0] & 0xFF;
		extrapolated_value[4][4] = rgb_data_2[1] & 0xFF;
		extrapolated_value[4][5] = rgb_data_2[2] & 0xFF;
		if (!extrapolated_value[4][1] && !extrapolated_value[4][2] && !extrapolated_value[4][3]
				&& !extrapolated_value[4][4] && !extrapolated_value[4][5]) {
			OFP_ERR("the extrapolated_value[4] are incorrect\n");
			rc = -EINVAL;
			goto error;
		}

		for (i = 0; i < 5; i++) {
			for (j = 0; j < 6; j++) {
				OFP_INFO("extrapolated_value[%u][%u] = 0x%02X\n", i, j, extrapolated_value[i][j]);
			}
		}

		calibrated = true;
		OFP_INFO("update lhbm pressed icon gamma successfully\n");
	}

	if (calibrated) {
		cmds = display->panel->cur_mode->priv_info->cmd_sets[DSI_CMD_LHBM_PRESSED_ICON_GAMMA].cmds;
		lcm_cmd_count = display->panel->cur_mode->priv_info->cmd_sets[DSI_CMD_LHBM_PRESSED_ICON_GAMMA].count;
		if (lcm_cmd_count != 8) {
			OFP_ERR("Invalid DSI_CMD_LHBM_PRESSED_ICON_GAMMA cmd sets\n");
			rc = -EINVAL;
		} else {
			for (i = 0; i < 5; i++) {
				tx_buf = (unsigned char *)cmds[2+i].msg.tx_buf;
				memcpy(tx_buf, extrapolated_value[i], 6);
			}
			/* send the adjusted gamma value */
			rc = oplus_ofp_display_cmd_set(display, DSI_CMD_LHBM_PRESSED_ICON_GAMMA);
			if (rc) {
				OFP_ERR("[%s] failed to send DSI_CMD_LHBM_PRESSED_ICON_GAMMA cmds, rc=%d\n", display->name, rc);
			}
		}
	}

error:
	/* if gamma read fails more than 100 times, no further operation will be performed */
	if (!calibrated && (failure_count < 100)) {
		failure_count++;
		OFP_ERR("failure_count:%u\n", failure_count);
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_lhbm_pressed_icon_gamma_update");

	OFP_DEBUG("end\n");

	return rc;
}

/*
 when the backlight is in the outdoor hbm state, the brightness of the lhbm pressed icon is too high, which causes the unlocking failure.
 therefore, the backlight brightness must be limited to the normal state in the fingerprint unlocking scenario
*/
int oplus_ofp_lhbm_backlight_update(void *sde_encoder_virt, void *dsi_panel, unsigned int *bl_level)
{
	bool new_icon_layer_status = false;
	static bool last_icon_layer_status = false;
	int rc = 0;
	uint64_t hbm_enable = 0;
	struct sde_encoder_virt *sde_enc = sde_encoder_virt;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct dsi_panel *panel = dsi_panel;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!oplus_ofp_local_hbm_is_enabled()) {
		OFP_DEBUG("local hbm is not enabled, no need to update backlight after icon on/off\n");
		return 0;
	}

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid p_oplus_ofp_params params\n");
		return -EINVAL;
	}

	if (!p_oplus_ofp_params->need_to_update_lhbm_pressed_icon_gamma) {
		OFP_DEBUG("need_to_update_lhbm_pressed_icon_gamma is not set, no need to update backlight after icon on/off\n");
		return 0;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_lhbm_backlight_update");

	if (!panel || !bl_level) {
		if (!sde_enc || !sde_enc->cur_master || !sde_enc->cur_master->connector) {
			OFP_ERR("Invalid sde_enc params\n");
			return -EINVAL;
		}

		c_conn = to_sde_connector(sde_enc->cur_master->connector);
		if (!c_conn) {
			OFP_ERR("Invalid c_conn params\n");
			return -EINVAL;
		}

		if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
			OFP_DEBUG("not in dsi mode, should not update backlight after icon on/off\n");
			return 0;
		}

		display = c_conn->display;

		if (!display || !display->panel) {
			OFP_ERR("Invalid display params\n");
			return -EINVAL;
		}

		hbm_enable = sde_connector_get_property(c_conn->base.state, CONNECTOR_PROP_HBM_ENABLE);
		new_icon_layer_status = hbm_enable & OPLUS_OFP_PROPERTY_ICON_LAYER;

		if (!p_oplus_ofp_params->aod_unlocking && !p_oplus_ofp_params->doze_active) {
			if (!last_icon_layer_status && new_icon_layer_status) {
				if (display->panel->bl_config.bl_level > OPLUS_OFP_900NIT_DBV_LEVEL) {
					OFP_INFO("set backlight level to OPLUS_OFP_900NIT_DBV_LEVEL after icon on\n");
					mutex_lock(&display->panel->panel_lock);
					rc = dsi_panel_set_backlight(display->panel, OPLUS_OFP_900NIT_DBV_LEVEL);
					if (rc) {
						OFP_ERR("unable to set backlight\n");
					}
					mutex_unlock(&display->panel->panel_lock);
				}
			} else if (last_icon_layer_status && !new_icon_layer_status) {
				if (display->panel->bl_config.bl_level > OPLUS_OFP_900NIT_DBV_LEVEL) {
					OFP_INFO("recovery backlight level to %u after icon off\n", display->panel->bl_config.bl_level);
					mutex_lock(&display->panel->panel_lock);
					rc = dsi_panel_set_backlight(display->panel, display->panel->bl_config.bl_level);
					if (rc) {
						OFP_ERR("unable to set backlight\n");
					}
					mutex_unlock(&display->panel->panel_lock);
				}
			}
		}

		last_icon_layer_status = new_icon_layer_status;
	} else {
		display = to_dsi_display(panel->host);
		if (!display) {
			OFP_ERR("Invalid display params\n");
			return -EINVAL;
		}

		c_conn = to_sde_connector(display->drm_conn);
		if (!c_conn) {
			OFP_ERR("Invalid c_conn params\n");
			return -EINVAL;
		}

		if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
			OFP_DEBUG("not in dsi mode, should not update backlight after icon on\n");
			return -EINVAL;
		}

		hbm_enable = sde_connector_get_property(c_conn->base.state, CONNECTOR_PROP_HBM_ENABLE);
		if (!p_oplus_ofp_params->aod_unlocking && !p_oplus_ofp_params->doze_active
				&& (hbm_enable & OPLUS_OFP_PROPERTY_ICON_LAYER) && (*bl_level > OPLUS_OFP_900NIT_DBV_LEVEL)) {
			*bl_level = OPLUS_OFP_900NIT_DBV_LEVEL;
			OFP_INFO("icon layer is on and backlight lvl is greater than OPLUS_OFP_900NIT_DBV_LEVEL, set backlight to OPLUS_OFP_900NIT_DBV_LEVEL\n");
		}
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_lhbm_backlight_update");

	OFP_DEBUG("end\n");

	return rc;
}

int oplus_ofp_send_hbm_state_event(unsigned int hbm_state)
{
	OFP_DEBUG("start\n");

	if (oplus_ofp_local_hbm_is_enabled()) {
		OFP_DEBUG("local hbm is enabled, no need to send hbm state event\n");
		return 0;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_send_hbm_state_event");

	oplus_event_data_notifier_trigger(DRM_PANEL_EVENT_HBM_STATE, hbm_state, true);
	OFP_INFO("DRM_PANEL_EVENT_HBM_STATE:%u\n", hbm_state);

	OPLUS_OFP_TRACE_END("oplus_ofp_send_hbm_state_event");

	OFP_DEBUG("end\n");

	return 0;
}

/* wait te and delay some us */
static int oplus_ofp_vblank_wait(void *sde_connector, unsigned int te_count, unsigned int delay_us)
{
	unsigned int i = 0;
	struct sde_connector *c_conn = sde_connector;
	struct drm_encoder *drm_enc = NULL;

	OFP_DEBUG("start\n");

	if (!te_count && !delay_us) {
		return 0;
	}

	if (!c_conn) {
		OFP_ERR("Invalid c_conn params\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		OFP_DEBUG("not in dsi mode, should not wait vblank\n");
		return 0;
	}

	drm_enc = c_conn->encoder;
	if (!drm_enc) {
		OFP_ERR("Invalid drm_enc params\n");
		return -EINVAL;
	}

	if (sde_encoder_is_disabled(drm_enc)) {
		OFP_ERR("sde encoder is disabled\n");
		return -EFAULT;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_vblank_wait");

	if (te_count) {
		OPLUS_OFP_TRACE_BEGIN("sde_encoder_wait_for_event");
		for (i = 0; i < te_count; i++) {
			sde_encoder_wait_for_event(drm_enc, MSM_ENC_VBLANK);
			OFP_INFO("wait for %u vblank event done\n", i + 1);
		}
		OPLUS_OFP_TRACE_END("sde_encoder_wait_for_event");
	}

	if (delay_us) {
		OPLUS_OFP_TRACE_BEGIN("usleep_range");
		usleep_range(delay_us, (delay_us + 10));
		OFP_INFO("usleep_range %u done\n", delay_us);
		OPLUS_OFP_TRACE_END("usleep_range");
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_vblank_wait");

	OFP_DEBUG("end\n");

	return 0;
}

static int oplus_ofp_hbm_wait_handle(void *sde_connector, bool hbm_en)
{
	int rc = 0;
	unsigned int refresh_rate = 0;
	unsigned int us_per_frame = 0;
	unsigned int te_count = 0;
	unsigned int delay_us = 0;
	struct sde_connector *c_conn = sde_connector;
	struct dsi_display *display = NULL;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (oplus_ofp_oled_capacitive_is_enabled() || oplus_ofp_ultrasonic_is_enabled()) {
		OFP_DEBUG("no need to handle hbm wait\n");
		return 0;
	}

	if (!c_conn || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		OFP_DEBUG("not in dsi mode, should not handle hbm wait\n");
		return 0;
	}

	display = c_conn->display;

	if (!display || !display->panel || !display->panel->cur_mode) {
		OFP_ERR("Invalid display params\n");
		return -EINVAL;
	}

	if (p_oplus_ofp_params->aod_unlocking && hbm_en && !p_oplus_ofp_params->aod_off_hbm_on_delay
			&& !display->panel->cur_mode->priv_info->oplus_ofp_need_to_sync_data_in_aod_unlocking) {
		OFP_DEBUG("no need to delay in aod unlocking\n");
		return 0;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_hbm_wait_handle");

	refresh_rate = display->panel->cur_mode->timing.refresh_rate;
	us_per_frame = 1000000/refresh_rate;

	if (hbm_en) {
		if (p_oplus_ofp_params->aod_unlocking) {
			if (display->panel->cur_mode->priv_info->oplus_ofp_aod_off_insert_black_frame) {
				bool need_aod_off_hbm_on_delay = (bool)p_oplus_ofp_params->aod_off_hbm_on_delay;

				if (display->panel->cur_mode->priv_info->oplus_ofp_aod_off_black_frame_total_time) {
					if (ktime_sub(ktime_get(), p_oplus_ofp_params->aod_off_cmd_timestamp)
							>= ms_to_ktime(display->panel->cur_mode->priv_info->oplus_ofp_aod_off_black_frame_total_time)) {
						/* as rd_ptr irq will be disabled in idle mode, so the aod_off_hbm_on_delay
						may not be accurate. then add oplus_ofp_aod_off_black_frame_total_time
						to check whether the hbm on cmds are sent in the black frames or not */
						OFP_DEBUG("no need to do some delay because hbm on cmds are not sent in the black frames\n");
						need_aod_off_hbm_on_delay = false;
					}
				}

				if (need_aod_off_hbm_on_delay) {
					/* do some frame delay to separate aod off cmds and hbm on cmds */
					te_count = p_oplus_ofp_params->aod_off_hbm_on_delay;
					delay_us = (us_per_frame >> 1) + 700;
					OFP_INFO("wait %d te and %u us to separate aod off cmds and hbm on cmds\n", te_count, delay_us);

					p_oplus_ofp_params->aod_off_hbm_on_delay = 0;
					OPLUS_OFP_TRACE_INT("oplus_ofp_aod_off_hbm_on_delay", p_oplus_ofp_params->aod_off_hbm_on_delay);
				} else if (display->panel->cur_mode->priv_info->oplus_ofp_need_to_sync_data_in_aod_unlocking) {
					/* wait 1 te ,then send hbm on cmds in the second half of the frame */
					te_count = 1;
					delay_us = (us_per_frame >> 1) + 700;
				}
			} else if (display->panel->cur_mode->priv_info->oplus_ofp_need_to_sync_data_in_aod_unlocking) {
				/* wait 1 te ,then send hbm on cmds in the second half of the frame */
				te_count = 1;
				delay_us = (us_per_frame >> 1) + 700;
			}
		} else {
			/* backlight will affect hbm on time in some panel, need to separate the 51 cmd for stable hbm on time */
			if (display->panel->cur_mode->priv_info->oplus_ofp_need_to_separate_backlight) {
				/* wait 1 te and 1 frame, then send hbm on cmds in the second half of the frame */
				te_count = 1;
				delay_us = us_per_frame + (us_per_frame >> 1) + 700;
			} else {
				/* wait 1 te ,then send hbm on cmds in the second half of the frame */
				te_count = 1;
				delay_us = (us_per_frame >> 1) + 700;
			}
		}
	} else {
		/* wait 1 te ,then send hbm off cmds in the second half of the frame */
		te_count = 1;
		delay_us = (us_per_frame >> 1) + 700;
	}

	OFP_INFO("te_count=%d, delay_us=%u\n", te_count, delay_us);
	rc = oplus_ofp_vblank_wait(c_conn, te_count, delay_us);
	if (rc) {
		OFP_ERR("oplus_ofp_vblank_wait failed\n");
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_hbm_wait_handle");

	OFP_DEBUG("end\n");

	return rc;
}

static int oplus_ofp_set_panel_hbm(void *sde_connector, bool hbm_en)
{
	int rc = 0;
	struct sde_connector *c_conn = sde_connector;
	struct dsi_display *display = NULL;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (oplus_ofp_oled_capacitive_is_enabled() || oplus_ofp_ultrasonic_is_enabled()) {
		OFP_DEBUG("no need to set panel hbm\n");
		return 0;
	}

	if (oplus_ofp_get_hbm_state() == hbm_en) {
		OFP_DEBUG("already in hbm state %d\n", hbm_en);
		return 0;
	}

	if (!c_conn || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		OFP_DEBUG("not in dsi mode, should not set panel hbm\n");
		return 0;
	}

	display = c_conn->display;

	if (!display || !display->panel) {
		OFP_ERR("Invalid display params\n");
		return -EINVAL;
	}

	if (!dsi_panel_initialized(display->panel)) {
		OFP_ERR("should not set panel hbm if panel is not initialized\n");
		return -EFAULT;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_set_panel_hbm");

	if (oplus_ofp_get_aod_state()) {
		OFP_INFO("send aod off cmds before hbm on because panel is still in aod mode\n");
		rc = oplus_ofp_aod_off_handle(display);
		if (rc) {
			OFP_ERR("failed to send aod off cmds\n");
		}
	}

	/* delay before hbm cmds */
	oplus_ofp_hbm_wait_handle(c_conn, hbm_en);

	if (oplus_ofp_local_hbm_is_enabled()) {
		/* send lhbm pressed icon cmds */
		if (hbm_en) {
			rc = oplus_ofp_display_cmd_set(display, DSI_CMD_LHBM_PRESSED_ICON_ON);
			if (rc) {
				OFP_ERR("[%s] failed to send DSI_CMD_LHBM_PRESSED_ICON_ON cmds, rc=%d\n", display->name, rc);
			}
		} else {
			rc = oplus_ofp_display_cmd_set(display, DSI_CMD_LHBM_PRESSED_ICON_OFF);
			if (rc) {
				OFP_ERR("[%s] failed to send DSI_CMD_LHBM_PRESSED_ICON_OFF cmds, rc=%d\n", display->name, rc);
			}
		}
		OFP_INFO("lhbm pressed icon cmds are flushed\n");
	} else {
		/* send hbm cmds */
		if (hbm_en) {
			rc = oplus_ofp_display_cmd_set(display, DSI_CMD_HBM_ON);
			if (rc) {
				OFP_ERR("[%s] failed to send DSI_CMD_HBM_ON cmds, rc=%d\n", display->name, rc);
			}
		} else {
			rc = oplus_ofp_display_cmd_set(display, DSI_CMD_HBM_OFF);
			if (rc) {
				OFP_ERR("[%s] failed to send DSI_CMD_HBM_OFF cmds, rc=%d\n", display->name, rc);
			}
		}
		OFP_INFO("hbm cmds are flushed\n");
	}

	if (!hbm_en && p_oplus_ofp_params->aod_unlocking) {
		p_oplus_ofp_params->aod_unlocking = false;
		OFP_INFO("oplus_ofp_aod_unlocking:%d\n", p_oplus_ofp_params->aod_unlocking);
		OPLUS_OFP_TRACE_INT("oplus_ofp_aod_unlocking", p_oplus_ofp_params->aod_unlocking);
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_set_panel_hbm");

	OFP_DEBUG("end\n");

	return rc;
}

int oplus_ofp_hbm_handle(void *sde_encoder_virt)
{
	int rc = 0;
	uint64_t hbm_enable = 0;
	unsigned int bl_level = 0;
	struct sde_encoder_virt *sde_enc = sde_encoder_virt;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (oplus_ofp_oled_capacitive_is_enabled() || oplus_ofp_ultrasonic_is_enabled()) {
		OFP_DEBUG("no need to handle hbm\n");
		return 0;
	}

	if (!sde_enc || !sde_enc->cur_master || !sde_enc->cur_master->connector || !p_oplus_ofp_params) {
		OFP_ERR("Invalid sde_enc params\n");
		return -EINVAL;
	}

	if (p_oplus_ofp_params->hbm_mode) {
		OFP_DEBUG("already in hbm mode %u\n", p_oplus_ofp_params->hbm_mode);
		return 0;
	}

	c_conn = to_sde_connector(sde_enc->cur_master->connector);
	if (!c_conn) {
		OFP_ERR("Invalid c_conn params\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		OFP_DEBUG("not in dsi mode, should not handle hbm\n");
		return 0;
	}

	display = c_conn->display;

	if (!display || !display->panel) {
		OFP_ERR("Invalid display params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_hbm_handle");

	hbm_enable = sde_connector_get_property(c_conn->base.state, CONNECTOR_PROP_HBM_ENABLE);
	bl_level = display->panel->bl_config.bl_level;
	OFP_DEBUG("hbm_enable:%lu, bl_level=%u\n", hbm_enable, bl_level);

	if ((!p_oplus_ofp_params->doze_active && (hbm_enable & OPLUS_OFP_PROPERTY_DIM_LAYER
			|| hbm_enable & OPLUS_OFP_PROPERTY_FINGERPRESS_LAYER) && bl_level)
				|| (p_oplus_ofp_params->doze_active && (hbm_enable & OPLUS_OFP_PROPERTY_FINGERPRESS_LAYER)
					&& bl_level)) {
		rc = oplus_ofp_set_panel_hbm(c_conn, true);
		if (rc) {
			OFP_ERR("failed to set panel hbm on\n");
		}
	} else if ((!(hbm_enable & OPLUS_OFP_PROPERTY_DIM_LAYER)
					&& !(hbm_enable & OPLUS_OFP_PROPERTY_FINGERPRESS_LAYER))
						|| !p_oplus_ofp_params->fp_press || !bl_level) {
		rc = oplus_ofp_set_panel_hbm(c_conn, false);
		if (rc) {
			OFP_ERR("failed to set panel hbm off\n");
		}
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_hbm_handle");

	OFP_DEBUG("end\n");

	return rc;
}

int oplus_ofp_cmd_post_wait(void *dsi_display_mode, void *dsi_cmd_desc, enum dsi_cmd_set_type type)
{
	struct dsi_display_mode *mode = dsi_display_mode;
	struct dsi_cmd_desc *cmds = dsi_cmd_desc;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!mode || !cmds || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_cmd_post_wait");

	if (cmds->post_wait_ms) {
		/*
		 hbm on cmds can be unstable and cause flicker, and wait 1 te solution will increase 1 frame delay.
		 therefore, delete the delay of hbm on cmds, then using te counting to check whether hbm on is taking
		 effect or not to shorten the ui ready time
		*/
		if (p_oplus_ofp_params->aod_unlocking && (type == DSI_CMD_HBM_ON)
				&& !mode->priv_info->oplus_ofp_need_to_sync_data_in_aod_unlocking) {
			OFP_DEBUG("no need to wait when cmds are DSI_CMD_HBM_ON in aod unlocking\n");
		} else {
				usleep_range(cmds->post_wait_ms*1000,
						((cmds->post_wait_ms*1000)+10));
		}
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_cmd_post_wait");

	OFP_DEBUG("end\n");

	return 0;
}

/* update panel hbm status */
int oplus_ofp_panel_hbm_status_update(void *sde_encoder_phys)
{
	static bool last_hbm_state = false;
	static unsigned int rd_ptr_count = 0;
	struct sde_encoder_phys *phys_enc = sde_encoder_phys;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!(oplus_ofp_optical_new_solution_is_enabled() || oplus_ofp_local_hbm_is_enabled())) {
		OFP_DEBUG("no need to update panel hbm status\n");
		return 0;
	}

	if (!phys_enc || !phys_enc->connector || !p_oplus_ofp_params) {
		OFP_ERR("Invalid phys_enc params\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(phys_enc->connector);
	if (!c_conn) {
		OFP_ERR("Invalid c_conn params\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		OFP_DEBUG("not in dsi mode, should not update panel hbm status\n");
		return 0;
	}

	display = c_conn->display;
	if (!display || !display->panel || !display->panel->cur_mode) {
		OFP_ERR("Invalid display params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_panel_hbm_status_update");

	if (p_oplus_ofp_params->aod_unlocking && oplus_ofp_get_hbm_state()
			&& !display->panel->cur_mode->priv_info->oplus_ofp_need_to_sync_data_in_aod_unlocking) {
		/*
		 since the delay of hbm on cmds is removed in aod unlocking, it is necessary to count te irq
		 to confirm whether the hbm cmds are taking effect or not
		*/
		if (!last_hbm_state && oplus_ofp_get_hbm_state()) {
			rd_ptr_count = 1;
		} else {
			rd_ptr_count++;
		}

		if (rd_ptr_count == display->panel->cur_mode->priv_info->oplus_ofp_hbm_on_period) {
			/* hbm cmds are taking effect in panel module */
			p_oplus_ofp_params->panel_hbm_status = true;
			OFP_INFO("oplus_ofp_panel_hbm_status:%d\n", p_oplus_ofp_params->panel_hbm_status);
		}
	} else {
		if (!last_hbm_state && oplus_ofp_get_hbm_state()) {
			/* hbm cmds are taking effect in panel module */
			p_oplus_ofp_params->panel_hbm_status = true;
			OFP_INFO("oplus_ofp_panel_hbm_status:%d\n", p_oplus_ofp_params->panel_hbm_status);
		} else if (last_hbm_state && !oplus_ofp_get_hbm_state()) {
			/* hbm cmds are not taking effect in panel module */
			p_oplus_ofp_params->panel_hbm_status = false;
			OFP_INFO("oplus_ofp_panel_hbm_status:%d\n", p_oplus_ofp_params->panel_hbm_status);
			rd_ptr_count = 0;
		}
	}

	last_hbm_state = oplus_ofp_get_hbm_state();
	OPLUS_OFP_TRACE_INT("oplus_ofp_panel_hbm_status", p_oplus_ofp_params->panel_hbm_status);

	OPLUS_OFP_TRACE_END("oplus_ofp_panel_hbm_status_update");

	OFP_DEBUG("end\n");

	return 0;
}

/* update pressed icon status */
int oplus_ofp_pressed_icon_status_update(void *sde_encoder_phys, unsigned int irq_type)
{
	uint64_t hbm_enable = 0;
	static uint64_t last_hbm_enable = 0;
	struct sde_encoder_phys *phys_enc = sde_encoder_phys;
	struct sde_connector *c_conn = NULL;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!oplus_ofp_optical_new_solution_is_enabled() && !oplus_ofp_ultrasonic_is_enabled() && !oplus_ofp_local_hbm_is_enabled()) {
		OFP_DEBUG("no need to update pressed icon status\n");
		return 0;
	}

	if (!phys_enc || !phys_enc->connector || !p_oplus_ofp_params) {
		OFP_ERR("Invalid phys_enc params\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(phys_enc->connector);
	if (!c_conn) {
		OFP_ERR("Invalid c_conn params\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		OFP_DEBUG("not in dsi mode, should not update pressed icon status\n");
		return 0;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_pressed_icon_status_update");

	hbm_enable = sde_connector_get_property(c_conn->base.state, CONNECTOR_PROP_HBM_ENABLE);

	if (irq_type == OPLUS_OFP_WD_PTR) {
		if ((!(last_hbm_enable & OPLUS_OFP_PROPERTY_FINGERPRESS_LAYER))
				&& (hbm_enable & OPLUS_OFP_PROPERTY_FINGERPRESS_LAYER)) {
			/* pressed icon scanning has started */
			p_oplus_ofp_params->pressed_icon_status = OPLUS_OFP_PRESSED_ICON_ON_WR_PTR;
			OFP_INFO("oplus_ofp_pressed_icon_status:OPLUS_OFP_PRESSED_ICON_ON_WR_PTR\n");
		} else if ((last_hbm_enable & OPLUS_OFP_PROPERTY_FINGERPRESS_LAYER)
						&& (!(hbm_enable & OPLUS_OFP_PROPERTY_FINGERPRESS_LAYER))) {
			/* the data scanning without pressed icon has started */
			p_oplus_ofp_params->pressed_icon_status = OPLUS_OFP_PRESSED_ICON_OFF_WR_PTR;
			OFP_INFO("oplus_ofp_pressed_icon_status:OPLUS_OFP_PRESSED_ICON_OFF_WR_PTR\n");
		}
		last_hbm_enable = hbm_enable;
	} else if (irq_type == OPLUS_OFP_PP_DONE) {
		if (p_oplus_ofp_params->pressed_icon_status == OPLUS_OFP_PRESSED_ICON_ON_WR_PTR) {
			/* pressed icon has been flush to DDIC ram */
			p_oplus_ofp_params->pressed_icon_status = OPLUS_OFP_PRESSED_ICON_ON_PP_DONE;
			OFP_INFO("oplus_ofp_pressed_icon_status:OPLUS_OFP_PRESSED_ICON_ON_PP_DONE\n");
		} else if (p_oplus_ofp_params->pressed_icon_status == OPLUS_OFP_PRESSED_ICON_OFF_WR_PTR) {
			/* the data without pressed icon has been flush to DDIC ram */
			p_oplus_ofp_params->pressed_icon_status = OPLUS_OFP_PRESSED_ICON_OFF_PP_DONE;
			OFP_INFO("oplus_ofp_pressed_icon_status:OPLUS_OFP_PRESSED_ICON_OFF_PP_DONE\n");
		}
	} else if (irq_type == OPLUS_OFP_RD_PTR) {
		if (p_oplus_ofp_params->pressed_icon_status == OPLUS_OFP_PRESSED_ICON_ON_PP_DONE) {
			/* pressed icon has been displayed in panel */
			p_oplus_ofp_params->pressed_icon_status = OPLUS_OFP_PRESSED_ICON_ON;
			OFP_INFO("oplus_ofp_pressed_icon_status:OPLUS_OFP_PRESSED_ICON_ON\n");
		} else if (p_oplus_ofp_params->pressed_icon_status == OPLUS_OFP_PRESSED_ICON_OFF_PP_DONE) {
			/* the data without pressed icon has been displayed in panel */
			p_oplus_ofp_params->pressed_icon_status = OPLUS_OFP_PRESSED_ICON_OFF;
			OFP_INFO("oplus_ofp_pressed_icon_status:OPLUS_OFP_PRESSED_ICON_OFF\n");
		}
	}
	OPLUS_OFP_TRACE_INT("oplus_ofp_pressed_icon_status", p_oplus_ofp_params->pressed_icon_status);

	OPLUS_OFP_TRACE_END("oplus_ofp_pressed_icon_status_update");

	OFP_DEBUG("end\n");

	return 0;
}

static int oplus_ofp_send_uiready_event(unsigned int ui_status)
{
	enum panel_event_notification_type notify_type = DRM_PANEL_EVENT_ONSCREENFINGERPRINT_UI_DISAPPEAR;

	OFP_DEBUG("start\n");

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_send_uiready_event");

	/* oplus notifier trigger */
	if (ui_status == OPLUS_OFP_UI_READY) {
		notify_type = DRM_PANEL_EVENT_ONSCREENFINGERPRINT_UI_READY;
	} else {
		notify_type = DRM_PANEL_EVENT_ONSCREENFINGERPRINT_UI_DISAPPEAR;
	}

	oplus_event_data_notifier_trigger(notify_type, 0, true);
	OFP_DEBUG("oplus_event_data_notifier_trigger:%u\n", notify_type);

	OPLUS_OFP_TRACE_END("oplus_ofp_send_uiready_event");

	OFP_DEBUG("end\n");

	return 0;
}

void oplus_ofp_uiready_event_work_handler(struct work_struct *work_item)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (oplus_ofp_oled_capacitive_is_enabled()) {
		OFP_DEBUG("no need to send uiready event\n");
		return;
	}

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_uiready_event_work_handler");

	/* send uiready immediately */
	OFP_INFO("send uiready:%u\n", p_oplus_ofp_params->notifier_chain_value);
	oplus_ofp_send_uiready_event(p_oplus_ofp_params->notifier_chain_value);
	OPLUS_OFP_TRACE_INT("oplus_ofp_notifier_chain_value", p_oplus_ofp_params->notifier_chain_value);

	OPLUS_OFP_TRACE_END("oplus_ofp_uiready_event_work_handler");

	OFP_DEBUG("end\n");

	return;
}

/* notify uiready */
int oplus_ofp_notify_uiready(void *sde_encoder_phys)
{
	static unsigned int last_notifier_chain_value = OPLUS_OFP_UI_DISAPPEAR;
	struct sde_encoder_phys *phys_enc = sde_encoder_phys;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (oplus_ofp_oled_capacitive_is_enabled()) {
		OFP_DEBUG("no need to notify uiready\n");
		return 0;
	}

	if (!phys_enc || !phys_enc->connector || !p_oplus_ofp_params) {
		OFP_ERR("Invalid phys_enc params\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(phys_enc->connector);
	if (!c_conn) {
		OFP_ERR("Invalid c_conn params\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		OFP_DEBUG("not in dsi mode, should not notify uiready\n");
		return 0;
	}

	display = c_conn->display;
	if (!display || !display->panel) {
		OFP_ERR("Invalid display params\n");
		return -EINVAL;
	}

	if (!display->panel->panel_initialized) {
		OFP_ERR("panel is not initialized, should not notify uiready\n");
		return -EFAULT;
	}

	if (IS_ERR_OR_NULL(p_oplus_ofp_params->uiready_event_wq)
			|| IS_ERR_OR_NULL(&p_oplus_ofp_params->uiready_event_work)) {
		OFP_ERR("uiready work queue or work handler is NULL");
		return -EFAULT;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_notify_uiready");

	if (((p_oplus_ofp_params->pressed_icon_status == OPLUS_OFP_PRESSED_ICON_ON_WR_PTR)
			|| (p_oplus_ofp_params->pressed_icon_status == OPLUS_OFP_PRESSED_ICON_ON))
				&& p_oplus_ofp_params->panel_hbm_status) {
		/* pressed icon scanning has started and hbm on cmds are also taking effect */
		p_oplus_ofp_params->notifier_chain_value = OPLUS_OFP_UI_READY;
	} else if (((p_oplus_ofp_params->pressed_icon_status == OPLUS_OFP_PRESSED_ICON_OFF_WR_PTR)
				|| (p_oplus_ofp_params->pressed_icon_status == OPLUS_OFP_PRESSED_ICON_OFF))
					|| !p_oplus_ofp_params->panel_hbm_status) {
		/* the data scanning without pressed icon has started or hbm off cmds are taking effect */
		p_oplus_ofp_params->notifier_chain_value = OPLUS_OFP_UI_DISAPPEAR;
	}


	if (last_notifier_chain_value != p_oplus_ofp_params->notifier_chain_value) {
		OFP_INFO("queue uiready event work\n");
		queue_work(p_oplus_ofp_params->uiready_event_wq, &p_oplus_ofp_params->uiready_event_work);
		last_notifier_chain_value = p_oplus_ofp_params->notifier_chain_value;
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_notify_uiready");

	OFP_DEBUG("end\n");

	return 0;
}

/*
 due to the fact that the brightness of lhbm pressed icon may change with the backlight,
 it is necessary to readjust the lhbm pressed icon grayscale to meet the requirements of fingerprint unlocking
*/
static int oplus_ofp_lhbm_pressed_icon_grayscale_update(void *dsi_panel, unsigned int bl_level)
{
	bool pwm_is_changing = false;
	bool need_to_update_grayscale = false;
	static bool last_pwm_state = false;
	unsigned char *tx_buf = NULL;
	unsigned char tx_buf_0[7] = {0xC2, 0xFE, 0xFE, 0xDD, 0x61, 0x00, 0x62};
	unsigned char tx_buf_1[7] = {0xC2, 0xF9, 0xF9, 0xDA, 0x61, 0x00, 0x62};
	unsigned char tx_buf_2[7] = {0xC2, 0xF5, 0xF5, 0xD6, 0x61, 0x00, 0x62};
	unsigned char tx_buf_3[7] = {0xC2, 0xEF, 0xEF, 0xDA, 0x61, 0x00, 0x62};
	unsigned char tx_buf_4[7] = {0xC2, 0xE9, 0xE9, 0xD4, 0x61, 0x00, 0x62};
	unsigned char tx_buf_5[7] = {0xC2, 0xFA, 0xFA, 0xD9, 0x61, 0x00, 0x62};
	unsigned char tx_buf_6[7] = {0xC2, 0xF4, 0xF4, 0xD2, 0x61, 0x00, 0x62};
	unsigned char tx_buf_7[7] = {0xC2, 0xF0, 0xF0, 0xD6, 0x61, 0x00, 0x62};
	unsigned char tx_buf_8[7] = {0xC2, 0xEA, 0xEA, 0xD2, 0x61, 0x00, 0x62};
	unsigned char tx_buf_9[7] = {0xC2, 0xE5, 0xE5, 0xCE, 0x61, 0x00, 0x62};
	int rc = 0;
	unsigned int lcm_cmd_count = 0;
	struct dsi_panel *panel = dsi_panel;
	struct dsi_cmd_desc *cmds = NULL;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!oplus_ofp_local_hbm_is_enabled()) {
		OFP_DEBUG("local hbm is not enabled, no need to update lhbm pressed icon grayscale\n");
		return 0;
	}

	if (!panel || !panel->cur_mode || !panel->cur_mode->priv_info || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (!p_oplus_ofp_params->need_to_update_lhbm_pressed_icon_gamma) {
		OFP_DEBUG("need_to_update_lhbm_pressed_icon_gamma is not config, no need to update lhbm pressed icon grayscale\n");
		return 0;
	}

	if (!dsi_panel_initialized(panel)) {
		OFP_ERR("should not update lhbm pressed icon grayscale if panel is not initialized\n");
		return -EFAULT;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_lhbm_pressed_icon_grayscale_update");

	pwm_is_changing = (last_pwm_state != oplus_panel_pwm_onepulse_is_enabled(panel));
	cmds = panel->cur_mode->priv_info->cmd_sets[DSI_CMD_LHBM_PRESSED_ICON_GRAYSCALE].cmds;
	lcm_cmd_count = panel->cur_mode->priv_info->cmd_sets[DSI_CMD_LHBM_PRESSED_ICON_GRAYSCALE].count;

	if ((lcm_cmd_count != 3) || (cmds[1].msg.tx_len != 7)) {
		OFP_ERR("Invalid DSI_CMD_LHBM_PRESSED_ICON_GRAYSCALE cmd sets\n");
		rc = -EINVAL;
	} else if (!oplus_panel_pwm_onepulse_is_enabled(panel)) {
		if (((oplus_last_backlight == 0x0000) || (oplus_last_backlight > 0x0700) || pwm_is_changing)
				&& ((bl_level > 0x0000) && (bl_level <= 0x0700))) {
			cmds[1].msg.tx_buf = (const void *)tx_buf_0;
			need_to_update_grayscale = true;
		} else if (((oplus_last_backlight <= 0x0700) || (oplus_last_backlight > 0x08F0) || pwm_is_changing)
						&& ((bl_level > 0x0700) && (bl_level <= 0x08F0))) {
			cmds[1].msg.tx_buf = (const void *)tx_buf_1;
			need_to_update_grayscale = true;
		} else if (((oplus_last_backlight <= 0x08F0) || (oplus_last_backlight > 0x0A88) || pwm_is_changing)
						&& ((bl_level > 0x08F0) && (bl_level <= 0x0A88))) {
			cmds[1].msg.tx_buf = (const void *)tx_buf_2;
			need_to_update_grayscale = true;
		} else if (((oplus_last_backlight <= 0x0A88) || (oplus_last_backlight > 0x0C00) || pwm_is_changing)
						&& ((bl_level > 0x0A88) && (bl_level <= 0x0C00))) {
			cmds[1].msg.tx_buf = (const void *)tx_buf_3;
			need_to_update_grayscale = true;
		} else if (((oplus_last_backlight <= 0x0C00) || (oplus_last_backlight > 0x0DBB) || pwm_is_changing)
						&& ((bl_level > 0x0C00) && (bl_level <= 0x0DBB))) {
			cmds[1].msg.tx_buf = (const void *)tx_buf_4;
			need_to_update_grayscale = true;
		}
	} else {
		if (((oplus_last_backlight == 0x0000) || (oplus_last_backlight > 0x0700) || pwm_is_changing)
				&& ((bl_level > 0x0000) && (bl_level <= 0x0700))) {
			cmds[1].msg.tx_buf = (const void *)tx_buf_5;
			need_to_update_grayscale = true;
		} else if (((oplus_last_backlight <= 0x0700) || (oplus_last_backlight > 0x08F0) || pwm_is_changing)
						&& ((bl_level > 0x0700) && (bl_level <= 0x08F0))) {
			cmds[1].msg.tx_buf = (const void *)tx_buf_6;
			need_to_update_grayscale = true;
		} else if (((oplus_last_backlight <= 0x08F0) || (oplus_last_backlight > 0x0A88) || pwm_is_changing)
						&& ((bl_level > 0x08F0) && (bl_level <= 0x0A88))) {
			cmds[1].msg.tx_buf = (const void *)tx_buf_7;
			need_to_update_grayscale = true;
		} else if (((oplus_last_backlight <= 0x0A88) || (oplus_last_backlight > 0x0C00) || pwm_is_changing)
						&& ((bl_level > 0x0A88) && (bl_level <= 0x0C00))) {
			cmds[1].msg.tx_buf = (const void *)tx_buf_8;
			need_to_update_grayscale = true;
		} else if (((oplus_last_backlight <= 0x0C00) || (oplus_last_backlight > 0x0DBB) || pwm_is_changing)
						&& ((bl_level > 0x0C00) && (bl_level <= 0x0DBB))) {
			cmds[1].msg.tx_buf = (const void *)tx_buf_9;
			need_to_update_grayscale = true;
		}
	}

	if (need_to_update_grayscale) {
		tx_buf = (unsigned char *)cmds[1].msg.tx_buf;
		OFP_INFO("lhbm pressed icon grayscale:0x%02X, 0x%02X, 0x%02X\n", tx_buf[1],  tx_buf[2],  tx_buf[3]);
		rc = oplus_ofp_panel_cmd_set_nolock(dsi_panel, DSI_CMD_LHBM_PRESSED_ICON_GRAYSCALE);
		if (rc) {
			OFP_ERR("[%s] failed to send DSI_CMD_LHBM_PRESSED_ICON_GRAYSCALE cmds, rc=%d\n", panel->name, rc);
		}
	}

	last_pwm_state = oplus_panel_pwm_onepulse_is_enabled(panel);

	OPLUS_OFP_TRACE_END("oplus_ofp_lhbm_pressed_icon_grayscale_update");

	OFP_DEBUG("end\n");

	return rc;
}

/* need filter backlight in hbm mode, hbm state and aod unlocking process */
bool oplus_ofp_backlight_filter(void *dsi_panel, unsigned int bl_level)
{
	bool need_filter_backlight = false;
	int rc = 0;
	uint64_t hbm_enable = 0;
	struct dsi_panel *panel = dsi_panel;
	struct dsi_display *display = NULL;
	struct sde_connector *c_conn = NULL;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!panel || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return false;
	}

	display = to_dsi_display(panel->host);
	if (!display) {
		OFP_ERR("Invalid display params\n");
		return false;
	}

	c_conn = to_sde_connector(display->drm_conn);
	if (!c_conn) {
		OFP_ERR("Invalid c_conn params\n");
		return false;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		OFP_DEBUG("not in dsi mode, should not filter backlight\n");
		return false;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_backlight_filter");

	hbm_enable = sde_connector_get_property(c_conn->base.state, CONNECTOR_PROP_HBM_ENABLE);

	if (oplus_ofp_get_hbm_state()) {
		if (!bl_level) {
			p_oplus_ofp_params->hbm_mode = 0;
			OFP_INFO("oplus_ofp_hbm_mode:%u\n", p_oplus_ofp_params->hbm_mode);
			OPLUS_OFP_TRACE_INT("oplus_ofp_hbm_mode", p_oplus_ofp_params->hbm_mode);
			if (oplus_ofp_local_hbm_is_enabled()) {
				if (oplus_ofp_get_hbm_state()) {
					rc = oplus_ofp_panel_cmd_set_nolock(dsi_panel, DSI_CMD_LHBM_PRESSED_ICON_OFF);
					if (rc) {
						OFP_ERR("[%s] failed to send DSI_CMD_LHBM_PRESSED_ICON_OFF cmds, rc=%d\n", panel->name, rc);
					}
				}
			}
			oplus_ofp_set_hbm_state(false);
			OFP_DEBUG("backlight is 0, set hbm mode and hbm state to false\n");

			if (p_oplus_ofp_params->aod_unlocking) {
				p_oplus_ofp_params->aod_unlocking = false;
				OFP_INFO("oplus_ofp_aod_unlocking:%d\n", p_oplus_ofp_params->aod_unlocking);
				OPLUS_OFP_TRACE_INT("oplus_ofp_aod_unlocking", p_oplus_ofp_params->aod_unlocking);
			}

			need_filter_backlight = false;
		} else {
			if (!oplus_ofp_local_hbm_is_enabled()) {
				OFP_INFO("hbm state is true, filter backlight %u setting\n", bl_level);
				need_filter_backlight = true;
			} else if (p_oplus_ofp_params->need_to_update_lhbm_pressed_icon_gamma && (bl_level > OPLUS_OFP_900NIT_DBV_LEVEL)) {
				OFP_INFO("hbm state is true and backlight lvl is greater than OPLUS_OFP_900NIT_DBV_LEVEL, filter backlight %u setting\n", bl_level);
				need_filter_backlight = true;
			}
		}
	} else if (p_oplus_ofp_params->aod_unlocking && p_oplus_ofp_params->fp_press && bl_level) {
		OFP_INFO("aod unlocking is true, filter backlight %u setting\n", bl_level);
		need_filter_backlight = true;
	} else if (!p_oplus_ofp_params->aod_unlocking && !p_oplus_ofp_params->doze_active
					&& (hbm_enable & OPLUS_OFP_PROPERTY_DIM_LAYER) && bl_level
					&& panel->cur_mode->priv_info->oplus_ofp_need_to_separate_backlight
					&& oplus_last_backlight) {
		/* backlight will affect hbm on time in some panel, need to separate the 51 cmd for stable hbm on time */
		OFP_INFO("dim layer exist, filter backlight %u setting in advance\n", bl_level);
		need_filter_backlight = true;
	} else if (oplus_ofp_get_aod_state()) {
		OFP_INFO("aod state is true, filter backlight %u setting\n", bl_level);
		need_filter_backlight = true;
	} else if (!oplus_ofp_get_aod_state() && (hbm_enable & OPLUS_OFP_PROPERTY_AOD_LAYER) && bl_level) {
		OFP_INFO("aod layer exist, filter backlight %u setting\n", bl_level);
		need_filter_backlight = true;
	} else if (p_oplus_ofp_params->dimlayer_hbm || hbm_enable) {
		OFP_INFO("backlight lvl:%u\n", bl_level);
	}

	if (hbm_enable != p_oplus_ofp_params->hbm_enable)
		OFP_INFO("panel name = %s, is_secondary = %d, hbm_enable = %d, hbm_enable2 = %d\n",
				panel->name, panel->is_secondary, hbm_enable, p_oplus_ofp_params->hbm_enable);

	if (panel->oplus_priv.pwm_switch_support) {
		panel->pwm_hbm_state = need_filter_backlight;
	}

	if (!need_filter_backlight) {
		oplus_ofp_lhbm_pressed_icon_grayscale_update(panel, bl_level);
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_backlight_filter");

	OFP_DEBUG("end\n");

	return need_filter_backlight;
}

static bool oplus_ofp_need_to_bypass_pq(void *s_crtc)
{
	bool need_to_bypass_pq = false;
	uint64_t hbm_enable = 0;
	struct sde_crtc *sde_crtc = s_crtc;
	struct drm_crtc *crtc = NULL;
	struct drm_encoder *drm_enc = NULL;
	struct sde_encoder_virt *sde_enc = NULL;
	struct sde_connector *c_conn = NULL;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (oplus_ofp_local_hbm_is_enabled()) {
		OFP_DEBUG("local hbm is enabled, no need to bypass pq\n");
		return false;
	}

	if (!sde_crtc || !p_oplus_ofp_params) {
		OFP_ERR("Invalid sde_crtc params\n");
		return false;
	}

	crtc = &sde_crtc->base;
	if (!crtc) {
		OFP_ERR("Invalid crtc params\n");
		return false;
	}

	list_for_each_entry(drm_enc, &crtc->dev->mode_config.encoder_list, head) {
		if (drm_enc->crtc != crtc)
			continue;

		sde_enc = to_sde_encoder_virt(drm_enc);
	}

	if (!sde_enc || !sde_enc->cur_master || !sde_enc->cur_master->connector) {
		OFP_DEBUG("Invalid sde_enc params\n");
		return false;
	}

	c_conn = to_sde_connector(sde_enc->cur_master->connector);
	if (!c_conn) {
		OFP_ERR("Invalid c_conn params\n");
		return false;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		OFP_DEBUG("not in dsi mode, should not bypass pq\n");
		return false;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_need_to_bypass_pq");

	hbm_enable = sde_connector_get_property(c_conn->base.state, CONNECTOR_PROP_HBM_ENABLE);
	/* need to bypass pq when aod/dim/fingerpress layer exist */
	if ((hbm_enable & OPLUS_OFP_PROPERTY_AOD_LAYER)
			|| (hbm_enable & OPLUS_OFP_PROPERTY_DIM_LAYER)
				|| (hbm_enable & OPLUS_OFP_PROPERTY_FINGERPRESS_LAYER)) {
		need_to_bypass_pq = true;
	}

	OFP_DEBUG("need_to_bypass_pq:%d\n", need_to_bypass_pq);

	OPLUS_OFP_TRACE_END("oplus_ofp_need_to_bypass_pq");

	OFP_DEBUG("end\n");

	return need_to_bypass_pq;
}

bool oplus_ofp_need_pcc_change(void *s_crtc)
{
	bool need_pcc_change = false;
	bool new_bypass_pq_status = false;
	static bool last_bypass_pq_status = false;
	struct sde_crtc *sde_crtc = s_crtc;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (oplus_ofp_local_hbm_is_enabled()) {
		OFP_DEBUG("local hbm is enabled, no need pcc change\n");
		return false;
	}

	if (!sde_crtc || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return false;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_need_pcc_change");

	new_bypass_pq_status = oplus_ofp_need_to_bypass_pq(sde_crtc);

	/*
	 if aod/dim/fingerpress layer exist, pcc config will be bypassed, and if aod/dim/fingerpress layer all disappear,
	 pcc config will be restored, so need to update pcc config
	*/
	if (new_bypass_pq_status != last_bypass_pq_status) {
		need_pcc_change = true;
		OFP_INFO("need pcc change\n");
	}

	last_bypass_pq_status = new_bypass_pq_status;

	OPLUS_OFP_TRACE_END("oplus_ofp_need_pcc_change");

	OFP_DEBUG("end\n");

	return need_pcc_change;
}

int oplus_ofp_set_dspp_pcc_feature(void *sde_hw_cp_cfg, void *s_crtc, bool before_setup_pcc)
{
	struct sde_hw_cp_cfg *hw_cfg = sde_hw_cp_cfg;
	static struct drm_msm_pcc *saved_pcc = NULL;
	struct sde_crtc *sde_crtc = s_crtc;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (oplus_ofp_local_hbm_is_enabled()) {
		OFP_DEBUG("local hbm is enabled, no need to set dspp pcc feature\n");
		return 0;
	}

	if (!hw_cfg || !sde_crtc || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_set_dspp_pcc_feature");

	if (before_setup_pcc) {
		saved_pcc = hw_cfg->payload;

		if (oplus_ofp_need_to_bypass_pq(sde_crtc)) {
			hw_cfg->payload = NULL;
			OFP_DEBUG("bypass dspp pcc feature\n");
		}
	} else {
		hw_cfg->payload = saved_pcc;
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_set_dspp_pcc_feature");

	OFP_DEBUG("end\n");

	return 0;
}

int oplus_ofp_bypass_dspp_gamut(void *sde_hw_cp_cfg, void *s_crtc)
{
	struct sde_hw_cp_cfg *hw_cfg = sde_hw_cp_cfg;
	struct sde_crtc *sde_crtc = s_crtc;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (oplus_ofp_local_hbm_is_enabled()) {
		OFP_DEBUG("local hbm is enabled, no need to bypass dspp gamut\n");
		return 0;
	}

	if (!hw_cfg || !sde_crtc ||!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_bypass_dspp_gamut");


	if (oplus_ofp_need_to_bypass_pq(sde_crtc) && p_oplus_ofp_params->need_to_bypass_gamut) {
		hw_cfg->payload = NULL;
		OFP_DEBUG("bypass dspp gamut\n");
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_bypass_dspp_gamut");

	OFP_DEBUG("end\n");

	return 0;
}

/* -------------------- aod -------------------- */
void oplus_ofp_aod_display_on_set_work_handler(struct work_struct *work_item)
{
	int rc = 0;
	struct dsi_display *display = oplus_display_get_current_display();

	OFP_DEBUG("start\n");

	if (!display) {
		OFP_ERR("Invalid params\n");
		return;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_aod_display_on_set_work_handler");

	OFP_INFO("send aod display on cmd\n");
	rc = oplus_ofp_display_cmd_set(display, DSI_CMD_POST_ON_BACKLIGHT);
	if (rc) {
		OFP_ERR("[%s] failed to send DSI_CMD_POST_ON_BACKLIGHT cmd, rc=%d\n", display->name, rc);
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_aod_display_on_set_work_handler");

	OFP_DEBUG("end\n");

	return;
}

/*
 as some panel has their own specific aod on sequence, if need_to_wait_data_before_aod_on is config,
 aod mode on setting should be sent firstly, then wait for the first image data of aod mode,
 after that display on(29) cmd will be sent
*/
int oplus_ofp_aod_display_on_set(void *sde_encoder_phys)
{
	struct sde_encoder_phys *phys_enc = sde_encoder_phys;
	struct sde_connector *c_conn = NULL;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!phys_enc || !phys_enc->connector || !p_oplus_ofp_params) {
		OFP_ERR("Invalid phys_enc params\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(phys_enc->connector);
	if (!c_conn) {
		OFP_ERR("Invalid c_conn params\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		OFP_DEBUG("not in dsi mode, should not set aod display on cmd\n");
		return 0;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_aod_display_on_set");

	if (oplus_ofp_get_aod_state() && p_oplus_ofp_params->wait_data_before_aod_on) {
		p_oplus_ofp_params->wait_data_before_aod_on = false;
		OFP_INFO("oplus_ofp_wait_data_before_aod_on:%d\n", p_oplus_ofp_params->wait_data_before_aod_on);
		OPLUS_OFP_TRACE_INT("oplus_ofp_wait_data_before_aod_on", p_oplus_ofp_params->wait_data_before_aod_on);

		queue_work(p_oplus_ofp_params->aod_display_on_set_wq, &p_oplus_ofp_params->aod_display_on_set_work);
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_aod_display_on_set");

	OFP_DEBUG("end\n");

	return 0;
}

/* aod off handle */
int oplus_ofp_aod_off_handle(void *dsi_display)
{
	int rc = 0;
	struct dsi_display *display = dsi_display;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!display || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_aod_off_handle");

	/* doze disable handle */
	OFP_INFO("aod off handle\n");

	/* make sure that ultra low power aod mode exit firstly */
	if (oplus_ofp_ultra_low_power_aod_is_enabled() && p_oplus_ofp_params->ultra_low_power_aod_state) {
		rc = oplus_ofp_display_cmd_set(display, DSI_CMD_ULTRA_LOW_POWER_AOD_OFF);
		if (rc) {
			OFP_ERR("[%s] failed to send DSI_CMD_ULTRA_LOW_POWER_AOD_OFF cmds, rc=%d\n", display->name, rc);
		}
	}

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	oplus_adfr_aod_fod_mux_vsync_switch(display->panel, false);
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

	rc = dsi_panel_set_nolp(display->panel);
	if (rc) {
		OFP_ERR("[%s] failed to send DSI_CMD_SET_NOLP cmds, rc=%d\n", display->name, rc);
	}
	oplus_ofp_set_aod_state(false);

	if (!oplus_ofp_oled_capacitive_is_enabled()) {
		/* update aod unlocking value */
		oplus_ofp_aod_unlocking_update();
	}

	/* aod off cmds are sent to ddic */
	p_oplus_ofp_params->aod_off_cmd_timestamp = ktime_get();
	OFP_DEBUG("aod_off_cmd_timestamp:%lu\n", ktime_to_ms(p_oplus_ofp_params->aod_off_cmd_timestamp));

	/* update backlight after exit aod mode */
	OFP_INFO("aod off set backlight\n");
	mutex_lock(&display->panel->panel_lock);
	dsi_panel_set_backlight(display->panel, display->panel->bl_config.bl_level);
	mutex_unlock(&display->panel->panel_lock);

	OPLUS_OFP_TRACE_END("oplus_ofp_aod_off_handle");

	OFP_DEBUG("end\n");

	return rc;
}

int oplus_ofp_power_mode_handle(void *dsi_display, int power_mode)
{
	int rc = 0;
	struct dsi_display *display = dsi_display;
	struct oplus_ofp_params *p_oplus_ofp_params = NULL;

	OFP_DEBUG("start\n");

	if (!display) {
		OFP_ERR("Invalid display para\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		OFP_ERR("no need to init secondary panel for iris chip\n");
		return rc;
	}
#endif
	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_power_mode_handle");

	mutex_lock(&oplus_ofp_lock);

	if (!strcmp(display->display_type, "primary")) {
		oplus_ofp_set_display_id(OPLUS_OFP_PRIMARY_DISPLAY);
		p_oplus_ofp_params = oplus_ofp_get_params(OPLUS_OFP_PRIMARY_DISPLAY);
	} else if (!strcmp(display->display_type, "secondary")) {
		oplus_ofp_set_display_id(OPLUS_OFP_SECONDARY_DISPLAY);
		p_oplus_ofp_params = oplus_ofp_get_params(OPLUS_OFP_SECONDARY_DISPLAY);
	} else {
		OFP_ERR("unknown display type:%s\n", display->display_type);
		mutex_unlock(&oplus_ofp_lock);
		OPLUS_OFP_TRACE_END("oplus_ofp_power_mode_handle");
		return -EINVAL;
	}
	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid p_oplus_ofp_params param\n");
		mutex_unlock(&oplus_ofp_lock);
		OPLUS_OFP_TRACE_END("oplus_ofp_power_mode_handle");
		return -EINVAL;
	}

	switch (power_mode) {
	case SDE_MODE_DPMS_LP1:
	case SDE_MODE_DPMS_LP2:
		if (!p_oplus_ofp_params->doze_active) {
			p_oplus_ofp_params->doze_active = true;
			OFP_INFO("oplus_ofp_doze_active:%d\n", p_oplus_ofp_params->doze_active);
			OPLUS_OFP_TRACE_INT("oplus_ofp_doze_active", p_oplus_ofp_params->doze_active);

			if (!oplus_ofp_oled_capacitive_is_enabled() && !oplus_ofp_ultrasonic_is_enabled()) {
				/* hbm mode -> normal mode -> aod mode */
				if (oplus_ofp_get_hbm_state()) {
					if (oplus_ofp_local_hbm_is_enabled()) {
						rc = oplus_ofp_display_cmd_set(display, DSI_CMD_LHBM_PRESSED_ICON_OFF);
						if (rc) {
							OFP_ERR("[%s] failed to send DSI_CMD_LHBM_PRESSED_ICON_OFF cmds, rc=%d\n", display->name, rc);
						}
					} else {
						rc = oplus_ofp_display_cmd_set(display, DSI_CMD_HBM_OFF);
						if (rc) {
							OFP_ERR("[%s] failed to send DSI_CMD_HBM_OFF cmds, rc=%d\n", display->name, rc);
						}
					}
				}
			}

			/* reset aod unlocking flag when fingerprint unlocking failed */
			if (p_oplus_ofp_params->aod_unlocking) {
				p_oplus_ofp_params->aod_unlocking = false;
				OFP_INFO("oplus_ofp_aod_unlocking:%d\n", p_oplus_ofp_params->aod_unlocking);
				OPLUS_OFP_TRACE_INT("oplus_ofp_aod_unlocking", p_oplus_ofp_params->aod_unlocking);
			}

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
			oplus_adfr_aod_fod_te_source_vsync_switch(display, OPLUS_ADFR_TE_SOURCE_TE);
			oplus_adfr_aod_fod_mux_vsync_switch(display->panel, true);
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

			oplus_ofp_set_aod_state(true);

			/* aod on */
			rc = dsi_panel_set_lp1(display->panel);
			if (rc) {
				OFP_ERR("[%s] failed to send DSI_CMD_SET_LP1 cmds, rc=%d\n", display->name, rc);
			}
			rc = dsi_panel_set_lp2(display->panel);
			if (rc) {
				OFP_ERR("[%s] failed to send DSI_CMD_SET_LP2 cmds, rc=%d\n", display->name, rc);
			}

			if (p_oplus_ofp_params->aod_light_mode) {
				rc = oplus_ofp_display_cmd_set(display, DSI_CMD_AOD_LOW_LIGHT_MODE);
				if (rc) {
					OFP_ERR("[%s] failed to send DSI_CMD_AOD_LOW_LIGHT_MODE cmds, rc=%d\n", display->name, rc);
				}
			}

			if (p_oplus_ofp_params->need_to_wait_data_before_aod_on) {
				p_oplus_ofp_params->wait_data_before_aod_on = true;
				OFP_INFO("oplus_ofp_wait_data_before_aod_on:%d\n", p_oplus_ofp_params->wait_data_before_aod_on);
				OPLUS_OFP_TRACE_INT("oplus_ofp_wait_data_before_aod_on", p_oplus_ofp_params->wait_data_before_aod_on);
			}
		}
		break;

	case SDE_MODE_DPMS_ON:
		if (p_oplus_ofp_params->doze_active) {
			p_oplus_ofp_params->doze_active = false;
			OFP_INFO("oplus_ofp_doze_active:%d\n", p_oplus_ofp_params->doze_active);
			OPLUS_OFP_TRACE_INT("oplus_ofp_doze_active", p_oplus_ofp_params->doze_active);
		}

		if (oplus_ofp_get_aod_state()) {
			rc = oplus_ofp_aod_off_handle(display);
			if (rc) {
				OFP_ERR("[%s] failed to handle aod off, rc=%d\n", display->name, rc);
			}
		}

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
		oplus_adfr_aod_fod_te_source_vsync_switch(display, OPLUS_ADFR_TE_SOURCE_TP);
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */
		break;

	case SDE_MODE_DPMS_OFF:
		if (p_oplus_ofp_params->doze_active) {
			p_oplus_ofp_params->doze_active = false;
			OFP_INFO("oplus_ofp_doze_active:%d\n", p_oplus_ofp_params->doze_active);
			OPLUS_OFP_TRACE_INT("oplus_ofp_doze_active", p_oplus_ofp_params->doze_active);
		}

		if (oplus_ofp_get_aod_state()) {
			rc = oplus_ofp_aod_off_handle(display);
			if (rc) {
				OFP_ERR("[%s] failed to handle aod off, rc=%d\n", display->name, rc);
			}
		}
		break;

	default:
		OFP_DEBUG("power_mode:%d\n", power_mode);
	}

	mutex_unlock(&oplus_ofp_lock);

	OPLUS_OFP_TRACE_END("oplus_ofp_power_mode_handle");

	OFP_DEBUG("end\n");

	return rc;
}

void oplus_ofp_aod_off_set_work_handler(struct work_struct *work_item)
{
	int rc = 0;
	struct dsi_display *display = oplus_display_get_current_display();

	OFP_DEBUG("start\n");

	if (!display) {
		OFP_ERR("Invalid params\n");
		return;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_aod_off_set_work_handler");

	OFP_INFO("send aod off cmds to speed up aod unlocking\n");
	rc = oplus_ofp_aod_off_handle(display);
	if (rc) {
		OFP_ERR("failed to send aod off cmds\n");
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_aod_off_set_work_handler");

	OFP_DEBUG("end\n");

	return;
}

static int oplus_ofp_aod_off_set(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (oplus_ofp_oled_capacitive_is_enabled()) {
		OFP_DEBUG("no need to send aod off cmds in doze mode to speed up fingerprint unlocking\n");
		return 0;
	}

	if (IS_ERR_OR_NULL(p_oplus_ofp_params)) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (oplus_ofp_get_hbm_state()) {
		OFP_DEBUG("ignore aod off setting in hbm state\n");
		return 0;
	}

	if (IS_ERR_OR_NULL(p_oplus_ofp_params->aod_off_set_wq)
			|| IS_ERR_OR_NULL(&p_oplus_ofp_params->aod_off_set_work)) {
		OFP_ERR("aod off work queue or work handler is NULL");
		return 0;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_aod_off_set");

	if (oplus_ofp_get_aod_state() && p_oplus_ofp_params->doze_active) {
		OFP_INFO("queue aod off set work\n");
		queue_work(p_oplus_ofp_params->aod_off_set_wq, &p_oplus_ofp_params->aod_off_set_work);
		oplus_ofp_set_aod_state(false);
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_aod_off_set");

	OFP_DEBUG("end\n");

	return 0;
}

/*
 touchpanel notify touchdown event when fingerprint is pressed,
 then display driver send aod off cmds immediately and vsync change to 120hz/90hz,
 so that press icon layer can sent down faster
*/
int oplus_ofp_touchpanel_event_notifier_call(struct notifier_block *nb, unsigned long action, void *data)
{
	struct touchpanel_event *tp_event = (struct touchpanel_event *)data;

	OFP_DEBUG("start\n");

	if (!oplus_ofp_is_supported() || oplus_ofp_oled_capacitive_is_enabled()) {
		OFP_DEBUG("no need to send aod off cmds in doze mode to speed up fingerprint unlocking\n");
		return NOTIFY_OK;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_touchpanel_event_notifier_call");

	if (tp_event) {
		if (action == EVENT_ACTION_FOR_FINGPRINT) {
			OFP_DEBUG("EVENT_ACTION_FOR_FINGPRINT\n");

			if (tp_event->touch_state == 1) {
				OFP_INFO("tp touchdown\n");
				/* send aod off cmds in doze mode to speed up fingerprint unlocking */
				oplus_ofp_aod_off_set();
			}
		}
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_touchpanel_event_notifier_call");

	OFP_DEBUG("end\n");

	return NOTIFY_OK;
}

/*
 as there have some black frames are inserted in aod off cmds flow which will affect hbm on cmds execution time,
 so check how many black frames have taken effect first, then calculate delay time to separate aod off cmds and hbm
 on cmds to make sure ui ready is accurate
*/
int oplus_ofp_aod_off_hbm_on_delay_check(void *sde_encoder_phys)
{
	static bool last_aod_unlocking = false;
	static unsigned int rd_ptr_count = 0;
	unsigned int aod_off_insert_black_frame = 0;
	struct sde_encoder_phys *phys_enc = sde_encoder_phys;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (oplus_ofp_oled_capacitive_is_enabled() || oplus_ofp_ultrasonic_is_enabled()) {
		OFP_DEBUG("no need to check aod off hbm on delay\n");
		return 0;
	}

	if (!phys_enc || !phys_enc->connector || !p_oplus_ofp_params) {
		OFP_ERR("Invalid phys_enc params\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(phys_enc->connector);
	if (!c_conn) {
		OFP_ERR("Invalid c_conn params\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		OFP_DEBUG("not in dsi mode, should not check aod off hbm on delay\n");
		return 0;
	}

	display = c_conn->display;
	if (!display || !display->panel || !display->panel->cur_mode) {
		OFP_ERR("Invalid display params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_aod_off_hbm_on_delay_check");

	aod_off_insert_black_frame = display->panel->cur_mode->priv_info->oplus_ofp_aod_off_insert_black_frame;

	if (aod_off_insert_black_frame) {
		if (p_oplus_ofp_params->aod_unlocking) {
			if (!last_aod_unlocking) {
				rd_ptr_count = 1;
			} else if (rd_ptr_count && rd_ptr_count < 10) {
				rd_ptr_count++;
			} else {
				/* 10 irq is enough */
				rd_ptr_count = 10;
			}
		} else {
			rd_ptr_count = 0;
			p_oplus_ofp_params->aod_off_hbm_on_delay = 0;
		}

		if (rd_ptr_count < aod_off_insert_black_frame) {
			p_oplus_ofp_params->aod_off_hbm_on_delay = aod_off_insert_black_frame - rd_ptr_count;
			OFP_DEBUG("aod_off_insert_black_frame=%d,rd_ptr_count=%d,aod_off_hbm_on_delay=%d\n",
				aod_off_insert_black_frame, rd_ptr_count, p_oplus_ofp_params->aod_off_hbm_on_delay);
		} else {
			p_oplus_ofp_params->aod_off_hbm_on_delay = 0;
		}

		last_aod_unlocking = p_oplus_ofp_params->aod_unlocking;
		OPLUS_OFP_TRACE_INT("oplus_ofp_aod_off_hbm_on_delay", p_oplus_ofp_params->aod_off_hbm_on_delay);
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_aod_off_hbm_on_delay_check");

	OFP_DEBUG("end\n");

	return 0;
}

/*
 since setting the backlight while the aod layer is exist will cause splash issue,
 the backlight will be filtered at this time and restored after the aod layer disappears
*/
int oplus_ofp_aod_off_backlight_recovery(void *sde_encoder_virt)
{
	int rc = 0;
	uint64_t hbm_enable = 0;
	static bool last_aod_layer_status = false;
	bool new_aod_layer_status = false;
	struct sde_encoder_virt *sde_enc = sde_encoder_virt;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!sde_enc || !sde_enc->cur_master || !sde_enc->cur_master->connector || !p_oplus_ofp_params) {
		OFP_ERR("Invalid sde_enc params\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(sde_enc->cur_master->connector);
	if (!c_conn) {
		OFP_ERR("Invalid c_conn params\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		OFP_DEBUG("not in dsi mode, should not recovery backlight after aod off\n");
		return 0;
	}

	display = c_conn->display;

	if (!display || !display->panel) {
		OFP_ERR("Invalid display params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_aod_off_backlight_recovery");

	hbm_enable = sde_connector_get_property(c_conn->base.state, CONNECTOR_PROP_HBM_ENABLE);
	new_aod_layer_status = hbm_enable & OPLUS_OFP_PROPERTY_AOD_LAYER;

	if (last_aod_layer_status && !new_aod_layer_status) {
		OFP_INFO("recovery backlight level after aod off\n");
		display->panel->post_power_on = true;
		mutex_lock(&display->panel->panel_lock);
		rc = dsi_panel_set_backlight(display->panel, display->panel->bl_config.bl_level);
		if (rc) {
			OFP_ERR("unable to set backlight\n");
		}
		mutex_unlock(&display->panel->panel_lock);
	}

	last_aod_layer_status = new_aod_layer_status;

	OPLUS_OFP_TRACE_END("oplus_ofp_aod_off_backlight_recovery");

	OFP_DEBUG("end\n");

	return rc;
}

int oplus_ofp_ultra_low_power_aod_update(void *sde_encoder_virt)
{
	int rc = 0;
	uint64_t hbm_enable = 0;
	struct sde_encoder_virt *sde_enc = sde_encoder_virt;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!(oplus_ofp_is_supported() && oplus_ofp_ultra_low_power_aod_is_enabled())) {
		OFP_DEBUG("ultra low power aod is not supported\n");
		return 0;
	} else if (!oplus_ofp_get_aod_state()) {
		OFP_DEBUG("not in aod mode, should not updata ultra low power aod\n");
		return 0;
	} else if (oplus_ofp_get_hbm_state()) {
		OFP_DEBUG("ignore ultra low power aod update in hbm state\n");
		return 0;
	}

	if (!sde_enc || !sde_enc->cur_master || !sde_enc->cur_master->connector || !p_oplus_ofp_params) {
		OFP_ERR("Invalid sde_enc params\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(sde_enc->cur_master->connector);
	if (!c_conn) {
		OFP_ERR("Invalid c_conn params\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		OFP_DEBUG("not in dsi mode, should not updata ultra low power aod\n");
		return 0;
	}

	display = c_conn->display;

	if (!display || !display->panel) {
		OFP_ERR("Invalid display params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_ultra_low_power_aod_update");

	hbm_enable = sde_connector_get_property(c_conn->base.state, CONNECTOR_PROP_HBM_ENABLE);

	if (!(hbm_enable & OPLUS_OFP_PROPERTY_ICON_LAYER)
		&& (hbm_enable & OPLUS_OFP_PROPERTY_AOD_LAYER)
			&& p_oplus_ofp_params->ultra_low_power_aod_mode) {
		/* when icon layer disappear, enable ultra low power aod immediately */
		if (!p_oplus_ofp_params->ultra_low_power_aod_state) {
			OFP_INFO("update ultra low power aod on\n");
			rc = oplus_ofp_display_cmd_set(display, DSI_CMD_ULTRA_LOW_POWER_AOD_ON);
			if (rc) {
				OFP_ERR("[%s] failed to send DSI_CMD_ULTRA_LOW_POWER_AOD_ON cmds, rc=%d\n", display->name, rc);
			}
		}
	} else {
		if (p_oplus_ofp_params->ultra_low_power_aod_state) {
			OFP_INFO("update ultra low power aod off\n");
			rc = oplus_ofp_display_cmd_set(display, DSI_CMD_ULTRA_LOW_POWER_AOD_OFF);
			if (rc) {
				OFP_ERR("[%s] failed to send DSI_CMD_ULTRA_LOW_POWER_AOD_OFF cmds, rc=%d\n", display->name, rc);
			}
		}
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_ultra_low_power_aod_update");

	OFP_DEBUG("end\n");

	return rc;
}

/* -------------------- node -------------------- */
/* fp_type */
int oplus_ofp_set_fp_type(void *buf)
{
	unsigned int *fp_type = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_set_fp_type");

	p_oplus_ofp_params->fp_type = *fp_type;
	OFP_INFO("fp_type:0x%x\n", p_oplus_ofp_params->fp_type);
	OPLUS_OFP_TRACE_INT("oplus_ofp_fp_type", p_oplus_ofp_params->fp_type);

	OPLUS_OFP_TRACE_END("oplus_ofp_set_fp_type");

	OFP_DEBUG("end\n");

	return 0;
}

int oplus_ofp_get_fp_type(void *buf)
{
	unsigned int *fp_type = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_get_fp_type");

	*fp_type = p_oplus_ofp_params->fp_type;
	OFP_INFO("fp_type:0x%x\n", *fp_type);

	OPLUS_OFP_TRACE_END("oplus_ofp_get_fp_type");

	OFP_DEBUG("end\n");

	return 0;
}

ssize_t oplus_ofp_set_fp_type_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int fp_type = 0;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return count;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_set_fp_type_attr");

	sscanf(buf, "%u", &fp_type);

	p_oplus_ofp_params->fp_type = fp_type;
	OFP_INFO("fp_type:0x%x\n", p_oplus_ofp_params->fp_type);
	OPLUS_OFP_TRACE_INT("oplus_ofp_fp_type", p_oplus_ofp_params->fp_type);

	OPLUS_OFP_TRACE_END("oplus_ofp_set_fp_type_attr");

	OFP_DEBUG("end\n");

	return count;
}

ssize_t oplus_ofp_get_fp_type_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_get_fp_type_attr");

	OFP_INFO("fp_type:0x%x\n", p_oplus_ofp_params->fp_type);

	OPLUS_OFP_TRACE_END("oplus_ofp_get_fp_type_attr");

	OFP_DEBUG("end\n");

	return sprintf(buf, "%u\n", p_oplus_ofp_params->fp_type);
}

/* ----- fod part ----- */
/* hbm */
int oplus_ofp_set_hbm(void *buf)
{
	int rc = 0;
	unsigned int *hbm_mode = buf;
	struct dsi_display *display = oplus_display_get_current_display();
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!buf || !display || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OFP_INFO("hbm mode:%u to %u\n", p_oplus_ofp_params->hbm_mode, *hbm_mode);

	if (!dsi_panel_initialized(display->panel)) {
		OFP_ERR("should not set hbm if panel is not initialized\n");
		return -EFAULT;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_set_hbm");

	p_oplus_ofp_params->hbm_mode = (*hbm_mode);
	OFP_INFO("oplus_ofp_hbm_mode:%u\n", p_oplus_ofp_params->hbm_mode);
	OPLUS_OFP_TRACE_INT("oplus_ofp_hbm_mode", p_oplus_ofp_params->hbm_mode);

	if (p_oplus_ofp_params->hbm_mode) {
		rc = oplus_ofp_display_cmd_set(display, DSI_CMD_HBM_ON);
		if (rc) {
			OFP_ERR("[%s] failed to send DSI_CMD_HBM_ON cmds, rc=%d\n", display->name, rc);
		}
	} else {
		rc = oplus_ofp_display_cmd_set(display, DSI_CMD_HBM_OFF);
		if (rc) {
			OFP_ERR("[%s] failed to send DSI_CMD_HBM_OFF cmds, rc=%d\n", display->name, rc);
		}
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_set_hbm");

	OFP_DEBUG("end\n");

	return rc;
}

int oplus_ofp_get_hbm(void *buf)
{
	unsigned int *hbm_mode = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_get_hbm");

	*hbm_mode = p_oplus_ofp_params->hbm_mode;
	OFP_INFO("hbm_mode:%u\n", *hbm_mode);

	OPLUS_OFP_TRACE_END("oplus_ofp_get_hbm");

	OFP_DEBUG("end\n");

	return 0;
}

ssize_t oplus_ofp_set_hbm_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int rc = 0;
	unsigned int hbm_mode = 0;
	struct dsi_display *display = oplus_display_get_current_display();
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!buf || !display || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return count;
	}

	sscanf(buf, "%u", &hbm_mode);
	OFP_INFO("hbm mode:%u to %u\n", p_oplus_ofp_params->hbm_mode, hbm_mode);

	if (!dsi_panel_initialized(display->panel)) {
		OFP_ERR("should not set hbm if panel is not initialized\n");
		return count;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_set_hbm_attr");

	p_oplus_ofp_params->hbm_mode = hbm_mode;
	OFP_INFO("oplus_ofp_hbm_mode:%u\n", p_oplus_ofp_params->hbm_mode);
	OPLUS_OFP_TRACE_INT("oplus_ofp_hbm_mode", p_oplus_ofp_params->hbm_mode);

	if (p_oplus_ofp_params->hbm_mode) {
		rc = oplus_ofp_display_cmd_set(display, DSI_CMD_HBM_ON);
		if (rc) {
			OFP_ERR("[%s] failed to send DSI_CMD_HBM_ON cmds, rc=%d\n", display->name, rc);
		}
	} else {
		rc = oplus_ofp_display_cmd_set(display, DSI_CMD_HBM_OFF);
		if (rc) {
			OFP_ERR("[%s] failed to send DSI_CMD_HBM_OFF cmds, rc=%d\n", display->name, rc);
		}
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_set_hbm_attr");

	OFP_DEBUG("end\n");

	return count;
}

ssize_t oplus_ofp_get_hbm_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_get_hbm_attr");

	OFP_INFO("hbm_mode:%u\n", p_oplus_ofp_params->hbm_mode);

	OPLUS_OFP_TRACE_END("oplus_ofp_get_hbm_attr");

	OFP_DEBUG("end\n");

	return sprintf(buf, "%u\n", p_oplus_ofp_params->hbm_mode);
}

/* aor */
ssize_t oplus_ofp_set_aor_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int rc = 0;
	unsigned int aor_mode = 0;
	struct dsi_display *display = oplus_display_get_current_display();
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!buf || !display || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return count;
	}

	sscanf(buf, "%u", &aor_mode);
	OFP_INFO("aor mode:%u to %u\n", p_oplus_ofp_params->aor_mode, aor_mode);

	if (!dsi_panel_initialized(display->panel)) {
		OFP_ERR("should not set aor if panel is not initialized\n");
		return count;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_set_aor_attr");

	p_oplus_ofp_params->aor_mode = aor_mode;
	OFP_INFO("oplus_ofp_aor_mode:%d\n", p_oplus_ofp_params->aor_mode);
	SDE_ATRACE_INT("oplus_ofp_aor_mode", p_oplus_ofp_params->aor_mode);

	if (p_oplus_ofp_params->aor_mode) {
		rc = oplus_ofp_display_cmd_set(display, DSI_CMD_AOR_ON);
		if (rc) {
			OFP_ERR("[%s] failed to send DSI_CMD_AOR_ON cmds, rc=%d\n", display->name, rc);
		}
	} else {
		rc = oplus_ofp_display_cmd_set(display, DSI_CMD_AOR_OFF);
		if (rc) {
			OFP_ERR("[%s] failed to send DSI_CMD_AOR_OFF cmds, rc=%d\n", display->name, rc);
		}
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_set_aor_attr");

	OFP_DEBUG("end\n");

	return count;
}

ssize_t oplus_ofp_get_aor_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_get_aor_attr");

	OFP_INFO("aor_mode:%d\n", p_oplus_ofp_params->aor_mode);

	OPLUS_OFP_TRACE_END("oplus_ofp_get_aor_attr");

	OFP_DEBUG("end\n");

	return sprintf(buf, "%d\n", p_oplus_ofp_params->aor_mode);
}

/* dimlayer_hbm */
int oplus_ofp_set_dimlayer_hbm(void *buf)
{
	unsigned int *dimlayer_hbm = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!oplus_ofp_is_supported() || oplus_ofp_oled_capacitive_is_enabled() || oplus_ofp_ultrasonic_is_enabled()) {
		OFP_DEBUG("no need to set dimlayer hbm\n");
		return 0;
	}

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_set_dimlayer_hbm");

	p_oplus_ofp_params->dimlayer_hbm = *dimlayer_hbm;
	OFP_INFO("dimlayer_hbm:%u\n", p_oplus_ofp_params->dimlayer_hbm);
	OPLUS_OFP_TRACE_INT("oplus_ofp_dimlayer_hbm", p_oplus_ofp_params->dimlayer_hbm);

	OPLUS_OFP_TRACE_END("oplus_ofp_set_dimlayer_hbm");

	OFP_DEBUG("end\n");

	return 0;
}

int oplus_ofp_get_dimlayer_hbm(void *buf)
{
	unsigned int *dimlayer_hbm = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_get_dimlayer_hbm");

	*dimlayer_hbm = p_oplus_ofp_params->dimlayer_hbm;
	OFP_INFO("dimlayer_hbm:%u\n", *dimlayer_hbm);

	OPLUS_OFP_TRACE_END("oplus_ofp_get_dimlayer_hbm");

	OFP_DEBUG("end\n");

	return 0;
}

ssize_t oplus_ofp_set_dimlayer_hbm_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int dimlayer_hbm = 0;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!oplus_ofp_is_supported() || oplus_ofp_oled_capacitive_is_enabled() || oplus_ofp_ultrasonic_is_enabled()) {
		OFP_DEBUG("no need to set dimlayer hbm\n");
		return count;
	}

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return count;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_set_dimlayer_hbm_attr");

	sscanf(buf, "%u", &dimlayer_hbm);

	p_oplus_ofp_params->dimlayer_hbm = dimlayer_hbm;
	OFP_INFO("dimlayer_hbm:%u\n", p_oplus_ofp_params->dimlayer_hbm);
	OPLUS_OFP_TRACE_INT("oplus_ofp_dimlayer_hbm", p_oplus_ofp_params->dimlayer_hbm);

	OPLUS_OFP_TRACE_END("oplus_ofp_set_dimlayer_hbm_attr");

	OFP_DEBUG("end\n");

	return count;
}

ssize_t oplus_ofp_get_dimlayer_hbm_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_get_dimlayer_hbm_attr");

	OFP_INFO("dimlayer_hbm:%u\n", p_oplus_ofp_params->dimlayer_hbm);

	OPLUS_OFP_TRACE_END("oplus_ofp_get_dimlayer_hbm_attr");

	OFP_DEBUG("end\n");

	return sprintf(buf, "%u\n", p_oplus_ofp_params->dimlayer_hbm);
}

/* notify_fppress */
int oplus_ofp_notify_fp_press(void *buf)
{
	unsigned int *fp_press = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!oplus_ofp_is_supported() || oplus_ofp_oled_capacitive_is_enabled()) {
		OFP_DEBUG("no need to set fp press\n");
		return 0;
	}

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_notify_fp_press");

	if (*fp_press) {
		/* finger is pressed down and pressed icon layer is ready */
		p_oplus_ofp_params->fp_press = true;
	} else {
		p_oplus_ofp_params->fp_press = false;
	}
	OFP_INFO("oplus_ofp_fp_press:%d\n", p_oplus_ofp_params->fp_press);
	OPLUS_OFP_TRACE_INT("oplus_ofp_fp_press", p_oplus_ofp_params->fp_press);

	if (p_oplus_ofp_params->fp_press) {
		/* send aod off cmds in doze mode to speed up fingerprint unlocking */
		OFP_DEBUG("fp press is true\n");
		oplus_ofp_aod_off_set();
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_notify_fp_press");

	OFP_DEBUG("end\n");

	return 0;
}

/* notify fp press for sysfs */
ssize_t oplus_ofp_notify_fp_press_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int fp_press = 0;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!oplus_ofp_is_supported() || oplus_ofp_oled_capacitive_is_enabled()) {
		OFP_DEBUG("no need to set fp press\n");
		return count;
	}

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return count;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_notify_fp_press_attr");

	sscanf(buf, "%d", &fp_press);

	if (fp_press) {
		/* finger is pressed down and pressed icon layer is ready */
		p_oplus_ofp_params->fp_press = true;
	} else {
		p_oplus_ofp_params->fp_press = false;
	}
	OFP_INFO("oplus_ofp_fp_press:%d\n", p_oplus_ofp_params->fp_press);
	OPLUS_OFP_TRACE_INT("oplus_ofp_fp_press", p_oplus_ofp_params->fp_press);

	if (p_oplus_ofp_params->fp_press) {
		/* send aod off cmds in doze mode to speed up fingerprint unlocking */
		OFP_DEBUG("fp press is true\n");
		oplus_ofp_aod_off_set();
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_notify_fp_press_attr");

	OFP_DEBUG("end\n");

	return count;
}

/* ----- aod part ----- */
/* aod_light_mode_set */
int oplus_ofp_set_aod_light_mode(void *buf)
{
	int rc = 0;
	unsigned int *aod_light_mode = buf;
	static unsigned int last_aod_light_mode = 0;
	struct dsi_display *display = oplus_display_get_current_display();
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!buf || !display || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	last_aod_light_mode = p_oplus_ofp_params->aod_light_mode;
	p_oplus_ofp_params->aod_light_mode = (*aod_light_mode);
	OFP_INFO("aod_light_mode:%u\n", p_oplus_ofp_params->aod_light_mode);
	OPLUS_OFP_TRACE_INT("oplus_ofp_aod_light_mode", p_oplus_ofp_params->aod_light_mode);

	if (!oplus_ofp_is_supported()) {
		OFP_DEBUG("aod is not supported\n");
		return 0;
	} else if (!oplus_ofp_get_aod_state()) {
		OFP_ERR("not in aod mode, should not set aod_light_mode\n");
		return 0;
	} else if (oplus_ofp_get_hbm_state()) {
		OFP_INFO("ignore aod light mode setting in hbm state\n");
		return 0;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_set_aod_light_mode");

	if (last_aod_light_mode != p_oplus_ofp_params->aod_light_mode) {
		if (p_oplus_ofp_params->aod_light_mode) {
			rc = oplus_ofp_display_cmd_set(display, DSI_CMD_AOD_LOW_LIGHT_MODE);
			if (rc) {
				OFP_ERR("[%s] failed to send DSI_CMD_AOD_LOW_LIGHT_MODE cmds, rc=%d\n", display->name, rc);
			}
		} else {
			rc = oplus_ofp_display_cmd_set(display, DSI_CMD_AOD_HIGH_LIGHT_MODE);
			if (rc) {
				OFP_ERR("[%s] failed to send DSI_CMD_AOD_HIGH_LIGHT_MODE cmds, rc=%d\n", display->name, rc);
			}
		}
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_set_aod_light_mode");

	OFP_DEBUG("end\n");

	return rc;
}

int oplus_ofp_get_aod_light_mode(void *buf)
{
	unsigned int *aod_light_mode = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_get_aod_light_mode");

	*aod_light_mode = p_oplus_ofp_params->aod_light_mode;
	OFP_INFO("aod_light_mode:%u\n", *aod_light_mode);

	OPLUS_OFP_TRACE_END("oplus_ofp_get_aod_light_mode");

	OFP_DEBUG("end\n");

	return 0;
}

ssize_t oplus_ofp_set_aod_light_mode_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int rc = 0;
	unsigned int aod_light_mode = 0;
	static unsigned int last_aod_light_mode = 0;
	struct dsi_display *display = oplus_display_get_current_display();
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!buf || !display || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return count;
	}

	sscanf(buf, "%u", &aod_light_mode);

	last_aod_light_mode = p_oplus_ofp_params->aod_light_mode;
	p_oplus_ofp_params->aod_light_mode = aod_light_mode;
	OFP_INFO("aod_light_mode:%u\n", p_oplus_ofp_params->aod_light_mode);
	OPLUS_OFP_TRACE_INT("oplus_ofp_aod_light_mode", p_oplus_ofp_params->aod_light_mode);

	if (!oplus_ofp_is_supported()) {
		OFP_DEBUG("aod is not supported\n");
		return count;
	} else if (!oplus_ofp_get_aod_state()) {
		OFP_ERR("not in aod mode, should not set aod_light_mode\n");
		return count;
	} else if (oplus_ofp_get_hbm_state()) {
		OFP_INFO("ignore aod light mode setting in hbm state\n");
		return count;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_set_aod_light_mode_attr");

	if (last_aod_light_mode != p_oplus_ofp_params->aod_light_mode) {
		if (p_oplus_ofp_params->aod_light_mode) {
			rc = oplus_ofp_display_cmd_set(display, DSI_CMD_AOD_LOW_LIGHT_MODE);
			if (rc) {
				OFP_ERR("[%s] failed to send DSI_CMD_AOD_LOW_LIGHT_MODE cmds, rc=%d\n", display->name, rc);
			}
		} else {
			rc = oplus_ofp_display_cmd_set(display, DSI_CMD_AOD_HIGH_LIGHT_MODE);
			if (rc) {
				OFP_ERR("[%s] failed to send DSI_CMD_AOD_HIGH_LIGHT_MODE cmds, rc=%d\n", display->name, rc);
			}
		}
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_set_aod_light_mode_attr");

	OFP_DEBUG("end\n");

	return count;
}

ssize_t oplus_ofp_get_aod_light_mode_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_get_aod_light_mode_attr");

	OFP_INFO("aod_light_mode:%u\n", p_oplus_ofp_params->aod_light_mode);

	OPLUS_OFP_TRACE_END("oplus_ofp_get_aod_light_mode_attr");

	OFP_DEBUG("end\n");

	return sprintf(buf, "%u\n", p_oplus_ofp_params->aod_light_mode);
}

/* ultra_low_power_aod_mode */
int oplus_ofp_set_ultra_low_power_aod_mode(void *buf)
{
	int rc = 0;
	unsigned int *ultra_low_power_aod_mode = buf;
	struct dsi_display *display = oplus_display_get_current_display();
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!buf || !display || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	p_oplus_ofp_params->ultra_low_power_aod_mode = (*ultra_low_power_aod_mode);
	OFP_INFO("ultra_low_power_aod_mode:%u\n", p_oplus_ofp_params->ultra_low_power_aod_mode);
	OPLUS_OFP_TRACE_INT("oplus_ofp_ultra_low_power_aod_mode", p_oplus_ofp_params->ultra_low_power_aod_mode);

	if (!(oplus_ofp_is_supported() && oplus_ofp_ultra_low_power_aod_is_enabled())) {
		OFP_DEBUG("ultra low power aod is not supported\n");
		return 0;
	} else if (!oplus_ofp_get_aod_state()) {
		OFP_ERR("not in aod mode, should not set ultra_low_power_aod_mode\n");
		return 0;
	} else if (oplus_ofp_get_hbm_state()) {
		OFP_INFO("ignore ultra low power aod mode setting in hbm state\n");
		return 0;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_set_ultra_low_power_aod_mode");

	/*
	 this could lead to discontinuous animation when enter or exit ultra low power aod mode frequently,
	 so ignore this cmds until icon layer disappear
	*/
	if (!(p_oplus_ofp_params->hbm_enable & OPLUS_OFP_PROPERTY_ICON_LAYER)) {
		if (*ultra_low_power_aod_mode && !p_oplus_ofp_params->ultra_low_power_aod_state) {
			rc = oplus_ofp_display_cmd_set(display, DSI_CMD_ULTRA_LOW_POWER_AOD_ON);
			if (rc) {
				OFP_ERR("[%s] failed to send DSI_CMD_ULTRA_LOW_POWER_AOD_ON cmds, rc=%d\n", display->name, rc);
			}
		} else if (!(*ultra_low_power_aod_mode) && p_oplus_ofp_params->ultra_low_power_aod_state) {
			rc = oplus_ofp_display_cmd_set(display, DSI_CMD_ULTRA_LOW_POWER_AOD_OFF);
			if (rc) {
				OFP_ERR("[%s] failed to send DSI_CMD_ULTRA_LOW_POWER_AOD_OFF cmds, rc=%d\n", display->name, rc);
			}
		}
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_set_ultra_low_power_aod_mode");

	OFP_DEBUG("end\n");

	return rc;
}

int oplus_ofp_get_ultra_low_power_aod_mode(void *buf)
{
	unsigned int *ultra_low_power_aod_mode = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_get_ultra_low_power_aod_mode");

	*ultra_low_power_aod_mode = p_oplus_ofp_params->ultra_low_power_aod_mode;
	OFP_INFO("ultra_low_power_aod_mode:%u\n", *ultra_low_power_aod_mode);

	OPLUS_OFP_TRACE_END("oplus_ofp_get_ultra_low_power_aod_mode");

	OFP_DEBUG("end\n");

	return 0;
}

ssize_t oplus_ofp_set_ultra_low_power_aod_mode_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int rc = 0;
	unsigned int ultra_low_power_aod_mode = 0;
	struct dsi_display *display = oplus_display_get_current_display();
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!buf || !display || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return count;
	}

	sscanf(buf, "%u", &ultra_low_power_aod_mode);
	p_oplus_ofp_params->ultra_low_power_aod_mode = ultra_low_power_aod_mode;
	OFP_INFO("ultra_low_power_aod_mode:%u\n", p_oplus_ofp_params->ultra_low_power_aod_mode);
	OPLUS_OFP_TRACE_INT("oplus_ofp_ultra_low_power_aod_mode", p_oplus_ofp_params->ultra_low_power_aod_mode);

	if (!(oplus_ofp_is_supported() && oplus_ofp_ultra_low_power_aod_is_enabled())) {
		OFP_DEBUG("ultra low power aod is not supported\n");
		return count;
	} else if (!oplus_ofp_get_aod_state()) {
		OFP_ERR("not in aod mode, should not set ultra_low_power_aod_mode\n");
		return count;
	} else if (oplus_ofp_get_hbm_state()) {
		OFP_INFO("ignore ultra low power aod mode setting in hbm state\n");
		return count;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_set_ultra_low_power_aod_mode_attr");

	/*
	 this could lead to discontinuous animation when enter or exit ultra low power aod mode frequently,
	 so ignore this cmds until icon layer disappear
	*/
	if (!(p_oplus_ofp_params->hbm_enable & OPLUS_OFP_PROPERTY_ICON_LAYER)) {
		if (ultra_low_power_aod_mode && !p_oplus_ofp_params->ultra_low_power_aod_state) {
			rc = oplus_ofp_display_cmd_set(display, DSI_CMD_ULTRA_LOW_POWER_AOD_ON);
			if (rc) {
				OFP_ERR("[%s] failed to send DSI_CMD_ULTRA_LOW_POWER_AOD_ON cmds, rc=%d\n", display->name, rc);
			}
		} else if (!ultra_low_power_aod_mode && p_oplus_ofp_params->ultra_low_power_aod_state) {
			rc = oplus_ofp_display_cmd_set(display, DSI_CMD_ULTRA_LOW_POWER_AOD_OFF);
			if (rc) {
				OFP_ERR("[%s] failed to send DSI_CMD_ULTRA_LOW_POWER_AOD_OFF cmds, rc=%d\n", display->name, rc);
			}
		}
	}

	OPLUS_OFP_TRACE_END("oplus_ofp_set_ultra_low_power_aod_mode_attr");

	OFP_DEBUG("end\n");

	return count;
}

ssize_t oplus_ofp_get_ultra_low_power_aod_mode_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params(oplus_ofp_display_id);

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OPLUS_OFP_TRACE_BEGIN("oplus_ofp_get_ultra_low_power_aod_mode_attr");

	OFP_INFO("ultra_low_power_aod_mode:%u\n", p_oplus_ofp_params->ultra_low_power_aod_mode);

	OPLUS_OFP_TRACE_END("oplus_ofp_get_ultra_low_power_aod_mode_attr");

	OFP_DEBUG("end\n");

	return sprintf(buf, "%u\n", p_oplus_ofp_params->ultra_low_power_aod_mode);
}
