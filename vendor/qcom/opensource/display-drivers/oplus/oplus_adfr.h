/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
** File : oplus_adfr.h
** Description : ADFR kernel module
** Version : 1.0
** Date : 2020/10/23
** Author : Display
******************************************************************/

#ifndef _OPLUS_ADFR_H_
#define _OPLUS_ADFR_H_

/* please just only include linux common head file to keep me pure */
#include <linux/device.h>
#include <linux/hrtimer.h>
#include "oplus_display_private_api.h"

enum oplus_vsync_mode {
	OPLUS_DOUBLE_TE_VSYNC = 0,
	OPLUS_EXTERNAL_TE_TP_VSYNC = 8,
	OPLUS_INVALID_VSYNC,
};

enum oplus_te_source {
	OPLUS_TE_SOURCE_TP = 0,		/* TE0 */
	OPLUS_TE_SOURCE_TE = 1,		/* TE1 */
};

enum oplus_vsync_switch {
	OPLUS_VSYNC_SWITCH_TP = 0,	/* TP VSYNC */
	OPLUS_VSYNC_SWITCH_TE = 1,	/* TE VSYNC */
};

enum h_skew_type {
	SDC_ADFR = 0,				/* SA */
	SDC_MFR = 1,				/* SM */
	OPLUS_ADFR = 2,				/* OA */
	OPLUS_MFR = 3,				/* OM */
};

enum oplus_adfr_auto_mode_value {
	OPLUS_ADFR_AUTO_OFF = 0,
	OPLUS_ADFR_AUTO_ON = 1,
	OPLUS_ADFR_AUTO_IDLE = 2,	/* for sw fps */
};

enum oplus_adfr_auto_fakeframe_value {
	OPLUS_ADFR_FAKEFRAME_OFF = 0,
	OPLUS_ADFR_FAKEFRAME_ON = 1,
};

enum oplus_adfr_auto_min_fps_value {
	OPLUS_ADFR_AUTO_MIN_FPS_MAX = 0x00,
	OPLUS_ADFR_AUTO_MIN_FPS_60HZ = 0x01,
	OPLUS_ADFR_AUTO_MIN_FPS_30HZ = 0x03,
	OPLUS_ADFR_AUTO_MIN_FPS_20HZ = 0x05,
	OPLUS_ADFR_AUTO_MIN_FPS_10HZ = 0x0B,
	OPLUS_ADFR_AUTO_MIN_FPS_1HZ = 0x77,
};

enum oplus_adfr_idle_mode_value {
	OPLUS_ADFR_IDLE_OFF = 0,
	OPLUS_ADFR_IDLE_ON = 1,
};

/* fixed qsync window and panel min fps nonsynchronous issue */
enum deferred_window_status {
	DEFERRED_WINDOW_END = 0,					/* deferred min fps window end */
	DEFERRED_WINDOW_START = 1,					/* deferred min fps window start */
	DEFERRED_WINDOW_NEXT_FRAME = 2,				/* set the min fps window in next frame */
	SET_WINDOW_IMMEDIATELY = 3,					/* set the min fps window immediately */
};

enum oplus_adfr_dynamic_te_config {
	OPLUS_ADFR_DYNAMIC_TE_DISABLE = 0,			/* disable dynamic te irq detection */
	OPLUS_ADFR_DYNAMIC_TE_ENABLE = 1,			/* enable dynamic te irq detection */
	OPLUS_ADFR_DYNAMIC_TE_ENABLE_WITCH_LOG = 2,	/* enable dynamic te irq detection and log */
};

struct oplus_adfr_dynamic_te {
	u64 last_te_timestamp;
	u64 current_te_timestamp;
	struct hrtimer timer;
	int refresh_rate;
	int config;									/* 0: disable irq detect 1:enable irq detect 2: enable irq detect and log */
};

struct oplus_te_refcount {
	bool te_calculate_enable;					/* enable by hidl interface  */
	u64 te_refcount;							/* count for te  */
	ktime_t start_timeline;						/* the irq enable timeline  */
	ktime_t end_timeline;						/* the irq disable timeline */
};

/* dynamic te detect */
extern struct oplus_adfr_dynamic_te oplus_adfr_dynamic_te;

/* --------------- adfr hidl ---------------*/
int oplus_display_set_dynamic_te(void *buf);
int oplus_display_get_dynamic_te(void *buf);
/* --------------- adfr misc ---------------*/
void oplus_adfr_init(void *dsi_panel);
inline bool oplus_adfr_is_support(void);
inline bool oplus_adfr_idle_mode_is_enable(void);
ssize_t oplus_adfr_get_debug(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);
ssize_t oplus_adfr_set_debug(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);
ssize_t oplus_get_vsync_switch(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);
ssize_t oplus_set_vsync_switch(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);
ssize_t oplus_adfr_get_dynamic_te(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);
ssize_t oplus_adfr_set_dynamic_te(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);

/* --------------- msm_drv ---------------*/
int oplus_adfr_thread_create(void *msm_priv, void *msm_ddev, void *msm_dev);
void oplus_adfr_thread_destroy(void *msm_priv);

/* ------------ sde_connector ------------ */
/* for qsync minfps */
int oplus_adfr_handle_qsync_mode_minfps(u32 propval);
bool oplus_adfr_qsync_mode_minfps_is_updated(void);
u32 oplus_adfr_get_qsync_mode_minfps(void);
/* qsync mode timer */
enum hrtimer_restart oplus_adfr_qsync_mode_timer_handler(struct hrtimer *timer);
void oplus_adfr_qsync_mode_timer_start(void *sde_connector, int deferred_ms);

/* --------------- sde_crtc ---------------*/
void sde_crtc_adfr_handle_frame_event(void *crtc, void *event);

/* --------------- sde_encoder ---------------*/
/* fake frame */
/* disable fakeframe */
int oplus_adfr_fakeframe_status_update(void *panel, bool force_disable);
/* if frame start commit, cancel the second fake frame */
int sde_encoder_adfr_cancel_fakeframe(void *enc);
/* timer handler for second fake frame */
enum hrtimer_restart sde_encoder_fakeframe_timer_handler(struct hrtimer *timer);
/* the fake frame cmd send function */
void sde_encoder_fakeframe_work_handler(struct kthread_work *work);
void oplus_adfr_fakeframe_timer_start(void *enc, int deferred_ms);
int sde_encoder_adfr_trigger_fakeframe(void *enc);
/* trigger first fake frame */
void sde_encoder_adfr_prepare_commit(void *crtc, void *enc, void *conn);
/* trigger second fake frame */
void sde_encoder_adfr_kickoff(void *crtc, void *enc, void *conn);

/* --------------- sde_encoder_phys_cmd ---------------*/
/* indicates whether filter backlight cmd is need */
bool oplus_adfr_backlight_cmd_filter_set(bool enable);
bool oplus_adfr_backlight_cmd_filter_get(void);
void oplus_adfr_force_qsync_mode_off(void *drm_connector);
int oplus_adfr_adjust_tearcheck_for_dynamic_qsync(void *sde_phys_enc);

/* --------------- dsi_connector ---------------*/
int sde_connector_send_fakeframe(void *conn);

/* --------------- dsi_display ---------------*/
int dsi_display_qsync_update_min_fps(void *dsi_display, void *dsi_params);
int dsi_display_qsync_restore(void *dsi_display);
/* fake frame */
/*
 * dsi_display_send_fakeframe - send 2C/3C dcs to Panel
 * @display: Pointer to private display structure
 * Returns: Zero on success
 */
int dsi_display_send_fakeframe(void *disp);
/* vsync switch */
void dsi_display_adfr_change_te_irq_status(void *display, bool enable);
/* dynamic te detect */
enum hrtimer_restart oplus_adfr_dynamic_te_timer_handler(struct hrtimer *timer);
void oplus_adfr_register_dynamic_te_irq(void *dsi_display);

/* --------------- dsi_panel ---------------*/
int dsi_panel_parse_adfr(void *dsi_mode, void *dsi_utils);
/* qsync enhance */
int dsi_panel_send_qsync_min_fps_dcs(void *dsi_panel,
		int ctrl_idx, uint32_t min_fps);
/* fake frame */
int dsi_panel_send_fakeframe_dcs(void *dsi_panel,
		int ctrl_idx);
void dsi_panel_adfr_status_reset(void *dsi_panel);

/* --------------- vsync switch ---------------*/
void oplus_dsi_display_vsync_switch(void *disp, bool force_te_vsync);
bool oplus_adfr_vsync_switch_is_enable(void);
enum oplus_vsync_mode oplus_adfr_get_vsync_mode(void);
/* ------------- mux switch ------------ */
/* vsync switch in resolution switch and aod scene */
void sde_encoder_adfr_vsync_switch(void *enc);
void sde_kms_adfr_vsync_switch(void *m_kms, void *d_crtc);
void oplus_adfr_resolution_vsync_switch(void *dsi_panel);
void oplus_adfr_aod_fod_vsync_switch(void *dsi_panel, bool force_te_vsync);
void oplus_adfr_vsync_switch_reset(void *dsi_panel);
/* ---------- te source switch --------- */
/* double TE */
int oplus_adfr_get_vsync_source(void *dsi_panel);
int oplus_adfr_vsync_source_switch(void *dsi_panel, u8 v_source);
int oplus_adfr_vsync_source_reset(void *enc);
int oplus_adfr_timing_vsync_source_switch(void *dsi_panel);
void sde_encoder_adfr_vsync_source_switch(void *enc);
void sde_kms_adfr_vsync_source_switch(void *m_kms, void *d_crtc);
void sde_encoder_adfr_aod_fod_source_switch(void *dsi_display, int te_source);

/* --------------- auto mode ---------------*/
/* Add for auto on cmd filter */
bool oplus_adfr_auto_on_cmd_filter_set(bool enable);
bool oplus_adfr_auto_on_cmd_filter_get(void);
int oplus_adfr_handle_auto_mode(u32 propval);
int dsi_display_auto_mode_update(void *dsi_display);
bool oplus_adfr_has_auto_mode(u32 value);

/* --------------- idle mode -------------- */
/* ADFR:Add for idle mode control */
/* idle mode handle */
void oplus_adfr_handle_idle_mode(void *sde_enc_v, int enter_idle);

int oplus_enable_te_refcount(void *data);
int oplus_get_te_fps(void *data);

/* ---------------  Only for  exist X7 chip salami project -------------- */
/* ADFR:Add for GPIO control */
int iris_set_panel_vsync_switch_gpio(struct dsi_panel *panel, int level);
u32 iris_emv_get_current_extend_frame(void);
#endif /* _OPLUS_ADFR_H_ */

