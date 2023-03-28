/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
** File : oplus_onscreenfingerprint.h
** Description : oplus_onscreenfingerprint header
** Version : 2.0
** Date : 2022/08/01
** Author : Display
***************************************************************/

#ifndef _OPLUS_ONSCREENFINGERPRINT_H_
#define _OPLUS_ONSCREENFINGERPRINT_H_

/* please just only include linux common head file to keep me pure */
#include "oplus_display_private_api.h"

enum oplus_ofp_log_level {
	OPLUS_OFP_LOG_LEVEL_ERR = 0,
	OPLUS_OFP_LOG_LEVEL_WARN = 1,
	OPLUS_OFP_LOG_LEVEL_INFO = 2,
	OPLUS_OFP_LOG_LEVEL_DEBUG = 3,
};

enum oplus_ofp_display_id {
	OPLUS_OFP_PRIMARY_DISPLAY = 0,
	OPLUS_OFP_SECONDARY_DISPLAY = 1,
};

enum oplus_ofp_property_value {
	OPLUS_OFP_PROPERTY_NONE = 0,
	OPLUS_OFP_PROPERTY_DIM_LAYER = BIT(0),
	OPLUS_OFP_PROPERTY_FINGERPRESS_LAYER = BIT(1),
	OPLUS_OFP_PROPERTY_ICON_LAYER = BIT(2),
	OPLUS_OFP_PROPERTY_AOD_LAYER = BIT(3),
};

enum oplus_ofp_irq_type {
	OPLUS_OFP_RD_PTR = 0,
	OPLUS_OFP_WD_PTR = 1,
	OPLUS_OFP_PP_DONE = 2,
};

enum oplus_ofp_pressed_icon_status {
	OPLUS_OFP_PRESSED_ICON_OFF_WR_PTR = 0,			/* the data scanning without pressed icon has started */
	OPLUS_OFP_PRESSED_ICON_OFF_PP_DONE = 1,			/* the data without pressed icon has been flush to DDIC ram */
	OPLUS_OFP_PRESSED_ICON_OFF = 2,					/* the data without pressed icon has been displayed in panel */
	OPLUS_OFP_PRESSED_ICON_ON_WR_PTR = 3,			/* pressed icon scanning has started */
	OPLUS_OFP_PRESSED_ICON_ON_PP_DONE = 4,			/* pressed icon has been flush to DDIC ram */
	OPLUS_OFP_PRESSED_ICON_ON = 5,					/* pressed icon has been displayed in panel */
};

enum oplus_ofp_ui_status {
	OPLUS_OFP_UI_DISAPPEAR = 0,
	OPLUS_OFP_UI_READY = 1,
};

/* remember to initialize params */
struct oplus_ofp_params {
	unsigned int fp_type;							/*
													 bit(0):lcd capacitive fingerprint(aod/fod aren't supported)
													 bit(1):oled capacitive fingerprint(only support aod)
													 bit(2):optical fingerprint old solution(dim layer && pressed icon are controlled by kernel)
													 bit(3):optical fingerprint new solution(dim layer && pressed icon aren't controlled by kernel)
													 bit(4):local hbm
													 bit(5):pressed icon brightness adaptation
													 bit(6):ultrasonic fingerprint
													 bit(7):ultra low power aod
													*/
	bool need_to_bypass_gamut;						/* indicates whether gamut needs to be bypassed in aod/fod scenarios or not */
	/* fod */
	unsigned int hbm_mode;							/* a node value used for fingerprint calibration */
	unsigned int aor_mode;							/* a node value used for aor change setting */
	unsigned int dimlayer_hbm;						/* indicates whether the dimlayer and hbm should enable or not(reserved) */
	uint64_t hbm_enable;							/* HBM_ENABLE property value */
	bool hbm_state;									/* indicates whether panel is hbm state or not */
	bool panel_hbm_status;							/* indicates whether hbm cmds are taking effect in panel module or not */
	bool fp_press;									/* indicates whether pressed icon layer is ready or not */
	unsigned int pressed_icon_status;				/* indicates whether pressed icon has been displayed in panel module or not */
	unsigned int notifier_chain_value;				/* ui ready notifier chain value */
	struct workqueue_struct *uiready_event_wq;		/* a workqueue used to send uiready event */
	struct work_struct uiready_event_work;			/* a work struct used to send uiready event */
	/* aod */
	bool doze_active;								/* indicates whether the current power mode is doze/doze suspend or not */
	bool aod_state;									/* indicates whether panel is aod state or not */
	bool need_to_wait_data_before_aod_on;			/* indicates whether display on cmd(29h) needs to be sent after image data write before aod on or not */
	bool wait_data_before_aod_on;					/* indicates whether to start waiting image data before aod on or not */
	bool aod_unlocking;								/* indicates whether the fingerprint unlocking is in aod state or not */
	unsigned int aod_off_hbm_on_delay;				/* indicates that how many frames need to wait to separate aod off cmds and hbm on cmds */
	ktime_t aod_off_cmd_timestamp;					/* record aod off cmds timestamp for aod off hbm on delay judgment */
	unsigned int aod_light_mode;					/* aod brightness setting, 0:50nit, 1:10nit */
	bool ultra_low_power_aod_state;					/* indicates whether panel is ultra low power aod state or not */
	unsigned int ultra_low_power_aod_mode;			/* indicates whether ultra low power aod mode needs to be entered or not */
	struct workqueue_struct *aod_display_on_set_wq;	/* a workqueue used to send display on(29) cmd after image data write before aod on */
	struct work_struct aod_display_on_set_work;		/* a work struct used to send display on(29) cmd after image data write before aod on */
	struct workqueue_struct *aod_off_set_wq;		/* a workqueue used to send aod off cmds to speed up aod unlocking */
	struct work_struct aod_off_set_work;			/* a work struct used to send aod off cmds to speed up aod unlocking */
	struct notifier_block touchpanel_event_notifier;/* add for touchpanel event notifier */
};

/* log level config */
extern unsigned int oplus_ofp_log_level;
/* dual display id */
extern unsigned int oplus_ofp_display_id;
/* debug log switch */
extern unsigned int oplus_dsi_log_type;
/* dynamic trace enable */
extern unsigned int oplus_display_trace_enable;
/* dsi cmd set prop map */
extern const char *cmd_set_prop_map[DSI_CMD_SET_MAX];

/* debug log */
#define OFP_ERR(fmt, arg...)	\
	do {	\
		if (oplus_ofp_log_level >= OPLUS_OFP_LOG_LEVEL_ERR)	\
			pr_err("[OFP][%u][ERR][%s:%d]"pr_fmt(fmt), oplus_ofp_display_id, __func__, __LINE__, ##arg);	\
	} while (0)

#define OFP_WARN(fmt, arg...)	\
	do {	\
		if (oplus_ofp_log_level >= OPLUS_OFP_LOG_LEVEL_WARN)	\
			pr_warn("[OFP][%u][WARN][%s:%d]"pr_fmt(fmt), oplus_ofp_display_id, __func__, __LINE__, ##arg);	\
	} while (0)

#define OFP_INFO(fmt, arg...)	\
	do {	\
		if (oplus_ofp_log_level >= OPLUS_OFP_LOG_LEVEL_INFO)	\
			pr_info("[OFP][%u][INFO][%s:%d]"pr_fmt(fmt), oplus_ofp_display_id, __func__, __LINE__, ##arg);	\
	} while (0)

#define OFP_DEBUG(fmt, arg...)	\
	do {	\
		if ((oplus_ofp_log_level >= OPLUS_OFP_LOG_LEVEL_DEBUG) && (oplus_dsi_log_type & OPLUS_DEBUG_LOG_OFP))	\
			pr_info("[OFP][%u][DEBUG][%s:%d]"pr_fmt(fmt), oplus_ofp_display_id, __func__, __LINE__, ##arg);	\
	} while (0)

/* debug trace */
#define OPLUS_OFP_TRACE_BEGIN(name)	\
	do {	\
		if (oplus_display_trace_enable & OPLUS_DISPLAY_OFP_TRACE_ENABLE)	\
			SDE_ATRACE_BEGIN(name);	\
	} while (0)

#define OPLUS_OFP_TRACE_END(name)	\
	do {	\
		if (oplus_display_trace_enable & OPLUS_DISPLAY_OFP_TRACE_ENABLE)	\
			SDE_ATRACE_END(name);	\
	} while (0)

#define OPLUS_OFP_TRACE_INT(name, value)	\
	do {	\
		if (oplus_display_trace_enable & OPLUS_DISPLAY_OFP_TRACE_ENABLE)	\
			SDE_ATRACE_INT(name, value);	\
	} while (0)

/* -------------------- oplus_ofp_params -------------------- */
int oplus_ofp_update_display_id(void);
int oplus_ofp_init(void *dsi_panel);
bool oplus_ofp_is_supported(void);
bool oplus_ofp_oled_capacitive_is_enabled(void);
bool oplus_ofp_optical_new_solution_is_enabled(void);
bool oplus_ofp_local_hbm_is_enabled(void);
bool oplus_ofp_ultrasonic_is_enabled(void);
bool oplus_ofp_get_hbm_state(void);
int oplus_ofp_property_update(void *sde_connector, void *sde_connector_state, int prop_id, uint64_t prop_val);

/* -------------------- fod -------------------- */
int oplus_ofp_parse_dtsi_config(void *dsi_display_mode, void *dsi_parser_utils);
int oplus_ofp_hbm_handle(void *sde_encoder_virt);
int oplus_ofp_cmd_post_wait(void *dsi_display_mode, void *dsi_cmd_desc, enum dsi_cmd_set_type type);
int oplus_ofp_panel_hbm_status_update(void *sde_encoder_phys);
int oplus_ofp_pressed_icon_status_update(void *sde_encoder_phys, unsigned int irq_type);
void oplus_ofp_uiready_event_work_handler(struct work_struct *work_item);
int oplus_ofp_notify_uiready(void *sde_encoder_phys);
bool oplus_ofp_backlight_filter(void *dsi_panel, unsigned int bl_level);
bool oplus_ofp_need_pcc_change(void *s_crtc);
int oplus_ofp_set_dspp_pcc_feature(void *sde_hw_cp_cfg, void *s_crtc, bool before_setup_pcc);
int oplus_ofp_bypass_dspp_gamut(void *sde_hw_cp_cfg, void *s_crtc);

/* -------------------- aod -------------------- */
void oplus_ofp_aod_display_on_set_work_handler(struct work_struct *work_item);
int oplus_ofp_aod_display_on_set(void *sde_encoder_phys);
int oplus_ofp_aod_off_handle(void *dsi_display);
int oplus_ofp_power_mode_handle(void *dsi_display, int power_mode);
void oplus_ofp_aod_off_set_work_handler(struct work_struct *work_item);
int oplus_ofp_touchpanel_event_notifier_call(struct notifier_block *nb, unsigned long action, void *data);
int oplus_ofp_aod_off_hbm_on_delay_check(void *sde_encoder_phys);
int oplus_ofp_aod_off_backlight_recovery(void *sde_encoder_virt);
int oplus_ofp_ultra_low_power_aod_update(void *sde_encoder_virt);

/* -------------------- node -------------------- */
/* fp_type */
int oplus_ofp_set_fp_type(void *buf);
int oplus_ofp_get_fp_type(void *buf);
ssize_t oplus_ofp_set_fp_type_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);
ssize_t oplus_ofp_get_fp_type_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);
/* ----- fod part ----- */
/* hbm */
int oplus_ofp_set_hbm(void *buf);
int oplus_ofp_get_hbm(void *buf);
ssize_t oplus_ofp_set_hbm_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);
ssize_t oplus_ofp_get_hbm_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);
/* aor */
ssize_t oplus_ofp_set_aor_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);
ssize_t oplus_ofp_get_aor_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);
/* dimlayer_hbm */
int oplus_ofp_set_dimlayer_hbm(void *buf);
int oplus_ofp_get_dimlayer_hbm(void *buf);
ssize_t oplus_ofp_set_dimlayer_hbm_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);
ssize_t oplus_ofp_get_dimlayer_hbm_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);
/* notify_fppress */
int oplus_ofp_notify_fp_press(void *buf);
ssize_t oplus_ofp_notify_fp_press_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);
/* ----- aod part ----- */
/* aod_light_mode_set */
int oplus_ofp_set_aod_light_mode(void *buf);
int oplus_ofp_get_aod_light_mode(void *buf);
ssize_t oplus_ofp_set_aod_light_mode_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);
ssize_t oplus_ofp_get_aod_light_mode_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);
/* ultra_low_power_aod_mode */
int oplus_ofp_set_ultra_low_power_aod_mode(void *buf);
int oplus_ofp_get_ultra_low_power_aod_mode(void *buf);
ssize_t oplus_ofp_set_ultra_low_power_aod_mode_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);
ssize_t oplus_ofp_get_ultra_low_power_aod_mode_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);

#endif /*_OPLUS_ONSCREENFINGERPRINT_H_*/
