/***************************************************************
** Copyright (C), 2023, OPLUS Mobile Comm Corp., Ltd
** File : oplus_adfr.h
** Description : oplus_adfr header
** Version : 2.0
** Date : 2023/05/04
** Author : Display
***************************************************************/

#ifndef _OPLUS_ADFR_H_
#define _OPLUS_ADFR_H_

/* please just only include linux common head file to keep me pure */
#include <linux/kobject.h>
#include "oplus_dsi_support.h"

enum oplus_adfr_log_level {
	OPLUS_ADFR_LOG_LEVEL_NONE = 0,
	OPLUS_ADFR_LOG_LEVEL_ERR = 1,
	OPLUS_ADFR_LOG_LEVEL_WARN = 2,
	OPLUS_ADFR_LOG_LEVEL_INFO = 3,
	OPLUS_ADFR_LOG_LEVEL_DEBUG = 4,
};

enum oplus_adfr_display_id {
	OPLUS_ADFR_PRIMARY_DISPLAY = 0,
	OPLUS_ADFR_SECONDARY_DISPLAY = 1,
};

enum oplus_adfr_irq_type {
	OPLUS_ADFR_RD_PTR = 0,
	OPLUS_ADFR_WD_PTR = 1,
	OPLUS_ADFR_PP_DONE = 2,
};

enum oplus_adfr_h_skew_mode {
	STANDARD_ADFR = 0,								/* SA */
	STANDARD_MFR = 1,								/* SM */
	OPLUS_ADFR = 2,									/* OA */
	OPLUS_MFR = 3,									/* OM */
};

enum oplus_adfr_auto_mode {
	OPLUS_ADFR_AUTO_OFF = 0,						/* manual mode */
	OPLUS_ADFR_AUTO_ON = 1,							/* auto mode */
	OPLUS_ADFR_AUTO_IDLE = 2,						/* min fps value is sw fps if auto idle is set */
};

enum oplus_adfr_fakeframe_mode {
	OPLUS_ADFR_FAKEFRAME_OFF = 0,
	OPLUS_ADFR_FAKEFRAME_ON = 1,
};

enum oplus_adfr_vsync_switch_mode {
	OPLUS_ADFR_TE_SOURCE_VSYNC_SWITCH = 0,			/* use two dsi te of ap to do vsync switch */
	OPLUS_ADFR_MUX_VSYNC_SWITCH = 8,				/* use external mux ic to do vsync switch */
	OPLUS_ADFR_INVALID_VSYNC_SWITCH,				/* invalid vsync switch */
};

enum oplus_adfr_te_source {
	OPLUS_ADFR_TE_SOURCE_TP = 0,					/* TE0 */
	OPLUS_ADFR_TE_SOURCE_TE = 1,					/* TE1 */
};

enum oplus_adfr_mux_vsync_switch_level {
	OPLUS_ADFR_MUX_VSYNC_SWITCH_TP = 0,				/* TP VSYNC */
	OPLUS_ADFR_MUX_VSYNC_SWITCH_TE = 1,				/* TE VSYNC */
};

enum oplus_adfr_idle_mode {
	OPLUS_ADFR_IDLE_OFF = 0,						/* exit mipi idle */
	OPLUS_ADFR_IDLE_ON = 1,							/* enter mipi idle */
};

/* high precision params */
enum oplus_adfr_high_precision_target_fps_value {
	OPLUS_ADFR_HIGH_PRECISION_FPS_60 = 60,				/* high precision 60HZ */
	OPLUS_ADFR_HIGH_PRECISION_FPS_72 = 72,				/* high precision 72HZ */
	OPLUS_ADFR_HIGH_PRECISION_FPS_90 = 90,				/* high precision 90HZ */
	OPLUS_ADFR_HIGH_PRECISION_FPS_120 = 120,			/* high precision 120HZ */
};

enum oplus_adfr_high_precision_stabilize_frame_type {
	OPLUS_ADFR_HW_STABILIZE_FRAME = 1,				/* hw stabilize frame type */
	OPLUS_ADFR_SW_STABILIZE_FRAME = 2,				/* sw stabilize frame type */
};

enum oplus_adfr_high_precision_te_shift_status {
	OPLUS_ADFR_TE_SHIFT_OFF = 1,					/* turn off te shift */
	OPLUS_ADFR_TE_SHIFT_ON = 2,						/* turn on te shift */
};

enum oplus_adfr_high_precision_rscc_state {
	OPLUS_ADFR_RSCC_CLK_STATE = 1,						/* change rscc state from cmd to clk */
	OPLUS_ADFR_RSCC_CMD_STATE = 2,						/* change rscc state from clk to cmd */
};

enum oplus_adfr_test_te_config {
	OPLUS_ADFR_TEST_TE_DISABLE = 0,					/* disable test te irq detection */
	OPLUS_ADFR_TEST_TE_ENABLE = 1,					/* enable test te irq detection */
	OPLUS_ADFR_TEST_TE_ENABLE_WITCH_LOG = 2,		/* enable test te irq detection and log */
};

enum oplus_adfr_osync_params {
	OPLUS_ADFR_OSYNC_MODE = 0,						/* update osync mode params */
	OPLUS_ADFR_OSYNC_MIN_FPS = 1,					/* update osync min fps params */
};

/*
 usually, osync min fps cmds should be sent to panel in the first frame,
 and then set the osync window in the next frame, because osync min fps cmds take one frame to take effect
*/
enum oplus_adfr_osync_window_setting_status {
	OPLUS_ADFR_OSYNC_WINDOW_SETTING_START = 0,		/* start to set osync window */
	OPLUS_ADFR_OSYNC_WINDOW_SETTING_NEXT_FRAME = 1,	/* set osync window in the next frame */
	OPLUS_ADFR_OSYNC_WINDOW_SETTING_END = 2,		/* end of osync window setting */
	OPLUS_ADFR_OSYNC_WINDOW_SETTING_AT_ONCE = 3,	/* set osync window at once */
};

struct oplus_adfr_test_te_params {
	int gpio;										/* a gpio used to check current refresh rate of ddic */
	unsigned int config;							/*
													 0:disable test te irq detection
													 1:enable test te irq detection
													 2:enable test te irq detection && log
													*/
	unsigned int high_refresh_rate_count;			/* a value used to indicates the count of high refresh rate */
	unsigned int refresh_rate;						/* a value used to indicates current refresh rate of ddic */
	u64 last_timestamp;								/* a value used to indicates the last test te irq timestamp */
	struct hrtimer timer;							/* a timer used to update refresh rate of ddic if enter idle mode */
};

/* remember to initialize params */
struct oplus_adfr_params {
	unsigned int config;							/*
													 bit(0):global
													 bit(1):fakeframe
													 bit(2):vsync switch
													 bit(3):vsync switch mode, 0:OPLUS_ADFR_TE_SOURCE_VSYNC_SWITCH, 1:OPLUS_ADFR_MUX_VSYNC_SWITCH
													 bit(4):idle mode
													 bit(5):temperature detection
													 bit(6):oa bl mutual exclusion
													 bit(7):sa mode restore when finished booting
													 bit(8):dry run
													*/
	unsigned int auto_mode;							/* a value used to control auto/manual frame rate mode */
	bool auto_mode_updated;							/* indicates whether auto mode is updated or not */
	bool need_filter_auto_on_cmd;					/* indicates whether auto on cmds need to be filtered if auto off cmds have been sent within one frame or not */
	unsigned int sa_min_fps;						/* the minimum self-refresh rate when no image would be sent to ddic in sa mode */
	bool sa_min_fps_updated;						/* indicates whether sa min fps is updated or not */
	bool skip_min_fps_setting;						/* indicates whether min fps setting should be skipped or not */
	unsigned int sw_fps;							/* software vsync value */
	unsigned int fakeframe;							/* indicates whether fakeframe is enabled or not */
	bool fakeframe_updated;							/* indicates whether fakeframe is updated or not */
	bool need_send_fakeframe_cmd;					/* indicates whether fakeframe cmd should be sent before te or not */
	struct pinctrl_state *te1_active;				/* a pinctrl state used to control te1 active */
	struct pinctrl_state *te1_suspend;				/* a pinctrl state used to control te1 suspend */
	int mux_vsync_switch_gpio;						/* a gpio used to switch input vsync signal by mux ic */
	unsigned int mux_vsync_switch_gpio_level;		/* a value used to indicates current mux vsync switch gpio level */
	bool force_te_vsync;							/* indicates whether te vsync is forced to set or not */
	bool need_switch_vsync;							/* indicates whether vsync signal needs to be switched or not */
	unsigned int idle_mode;							/* a value used to indicates current mipi idle status */
	struct oplus_adfr_test_te_params test_te;		/* a structure used to store current test te params */
	unsigned int osync_min_fps;						/* the minimum self-refresh rate when no image would be sent to ddic in osync mode */
	unsigned int osync_window_min_fps;				/* a value used to calculate the osync window threshold lines */
	unsigned int osync_window_setting_status;		/* a value used to indicates current osync window setting status */
	unsigned int osync_sync_threshold_start;		/* a value used to indicates current tc_cfg.sync_threshold_start value */
	unsigned int osync_frame_status;				/* a value used to indicates current osync frame status */
	bool need_filter_osync_backlight_cmd;			/* indicates whether osync backlight cmd needs to be filtered or not */
	bool osync_backlight_updated;					/* indicates whether osync backlight is updated or not */
	struct hrtimer osync_mode_timer;				/* a hrtimer used to restore osync mode after the backlight is no longer updated */
	bool need_restore_osync_mode;					/* indicates whether osync mode needs to be restored or not */
	bool need_force_off_osync_mode;					/* indicates whether osync mode needs to be forced off or not */
	bool need_resend_osync_cmd;						/* indicates whether osync cmds needs to be resent or not */
	/* high precision params */
	unsigned int high_precision_state;              /* a value used to indicates panel high precision state */
	unsigned int panel_high_precision_state;        /* indicates whether high precision cmds are taking effect in panel module or not */
	unsigned int high_precision_te_shift_status;	/* a value used to indicates high precision te shift status */
	unsigned int high_precision_rscc_state;			/* a value used to indicates high precision rscc state */
	unsigned int sa_high_precision_fps;				/* the high precision fps when no image would be sent to ddic in sa mode */
	unsigned int oa_high_precision_fps;				/* the high precision fps when no image would be sent to ddic in oa mode */
	bool sa_high_precision_fps_updated;				/* indicates whether sa high precision fps is updated or not */
	bool oa_high_precision_fps_updated;				/* indicates whether oa high precision fps is updated or not */
	unsigned int stabilize_frame_type;				/* a value used to indicates stabilize frame type */
	unsigned long current_wr_rd_irq_interval;		/* a value used to indicates current wr rd ptr interval */
	unsigned long last_wr_rd_irq_interval;			/* a value used to indicates last wr rd ptr interval */
};

/* log level config */
extern unsigned int oplus_adfr_log_level;
/* dual display id */
extern unsigned int oplus_adfr_display_id;
/* debug log switch */
extern unsigned int oplus_dsi_log_type;
/* dynamic trace enable */
extern unsigned int oplus_display_trace_enable;

/* debug log */
#define ADFR_ERR(fmt, arg...)	\
	do {	\
		if (oplus_adfr_log_level >= OPLUS_ADFR_LOG_LEVEL_ERR)	\
			pr_err("[ADFR][%u][ERR][%s:%d]"pr_fmt(fmt), oplus_adfr_display_id, __func__, __LINE__, ##arg);	\
	} while (0)

#define ADFR_WARN(fmt, arg...)	\
	do {	\
		if (oplus_adfr_log_level >= OPLUS_ADFR_LOG_LEVEL_WARN)	\
			pr_warn("[ADFR][%u][WARN][%s:%d]"pr_fmt(fmt), oplus_adfr_display_id, __func__, __LINE__, ##arg);	\
	} while (0)

#define ADFR_INFO(fmt, arg...)	\
	do {	\
		if (oplus_adfr_log_level >= OPLUS_ADFR_LOG_LEVEL_INFO)	\
			pr_info("[ADFR][%u][INFO][%s:%d]"pr_fmt(fmt), oplus_adfr_display_id, __func__, __LINE__, ##arg);	\
	} while (0)

#define ADFR_DEBUG(fmt, arg...)	\
	do {	\
		if ((oplus_adfr_log_level >= OPLUS_ADFR_LOG_LEVEL_DEBUG) && (oplus_dsi_log_type & OPLUS_DEBUG_LOG_ADFR))	\
			pr_info("[ADFR][%u][DEBUG][%s:%d]"pr_fmt(fmt), oplus_adfr_display_id, __func__, __LINE__, ##arg);	\
	} while (0)

/* debug trace */
#define OPLUS_ADFR_TRACE_BEGIN(name)	\
	do {	\
		if (oplus_display_trace_enable & OPLUS_DISPLAY_ADFR_TRACE_ENABLE)	\
			SDE_ATRACE_BEGIN(name);	\
	} while (0)

#define OPLUS_ADFR_TRACE_END(name)	\
	do {	\
		if (oplus_display_trace_enable & OPLUS_DISPLAY_ADFR_TRACE_ENABLE)	\
			SDE_ATRACE_END(name);	\
	} while (0)

#define OPLUS_ADFR_TRACE_INT(name, value)	\
	do {	\
		if (oplus_display_trace_enable & OPLUS_DISPLAY_ADFR_TRACE_ENABLE)	\
			SDE_ATRACE_INT(name, value);	\
	} while (0)

/* -------------------- oplus_adfr_params -------------------- */
int oplus_adfr_update_display_id(void);
int oplus_adfr_init(void *dsi_panel);
bool oplus_adfr_is_supported(void *oplus_adfr_params);
bool oplus_adfr_vsync_switch_is_enabled(void *oplus_adfr_params);
enum oplus_adfr_vsync_switch_mode oplus_adfr_get_vsync_switch_mode(void *oplus_adfr_params);
bool oplus_adfr_oa_bl_mutual_exclusion_is_enabled(void *oplus_adfr_params);

/* -------------------- standard adfr -------------------- */
int oplus_adfr_parse_dtsi_config(void *dsi_panel, void *dsi_display_mode, void *dsi_parser_utils);
bool oplus_adfr_h_skew_is_different(void *dsi_display, void *dsi_display_mode_0, void *dsi_display_mode_1);
int oplus_adfr_property_update(void *sde_connector, void *sde_connector_state, int prop_id, uint64_t prop_val);
int oplus_adfr_irq_handler(void *sde_encoder_phys, unsigned int irq_type);
int oplus_adfr_sa_mode_restore(void *dsi_display);
int oplus_adfr_sa_handle(void *sde_encoder_virt);
int oplus_adfr_status_reset(void *dsi_panel);

/* -------------------- high precision standard adfr -------------------- */
int oplus_adfr_high_precision_handle(void *sde_enc_v);
int oplus_adfr_high_precision_switch_state(void *dsi_panel);
int oplus_adfr_get_panel_high_precision_state(void *dsi_display);

/* -------------------- fakeframe -------------------- */
int oplus_adfr_fakeframe_check(void *sde_encoder_virt);
int oplus_adfr_fakeframe_handle(void *sde_encoder_virt);
int oplus_adfr_fakeframe_status_update(void *dsi_panel, bool force_disable);

/* -------------------- pre switch -------------------- */
int oplus_adfr_pre_switch_send(void *dsi_panel);

/* -------------------- vsync switch -------------------- */
/* ----- te source vsync switch ----- */
int oplus_adfr_te_source_vsync_switch_mode_fixup(void *dsi_display, void *dsi_display_mode_0, void *dsi_display_mode_1);
int oplus_adfr_te_source_vsync_switch_get_modes_helper(void *dsi_display, void *dsi_display_mode);
int oplus_adfr_te_source_vsync_switch_pinctrl_init(void *dsi_panel);
int oplus_adfr_te_source_vsync_switch_set_pinctrl_state(void *dsi_panel, bool enable);
int oplus_adfr_timing_te_source_vsync_switch(void *dsi_panel);
int oplus_adfr_frame_done_te_source_vsync_switch(void *drm_connector);
int oplus_adfr_aod_fod_te_source_vsync_switch(void *dsi_display, unsigned int te_source);
/* ----- mux vsync switch ----- */
int oplus_adfr_gpio_release(void *dsi_panel);
int oplus_adfr_timing_mux_vsync_switch(void *dsi_display);
int oplus_adfr_resolution_mux_vsync_switch(void *dsi_panel);
int oplus_adfr_frame_done_mux_vsync_switch(void *drm_connector);
int oplus_adfr_aod_fod_mux_vsync_switch(void *dsi_panel, bool force_te_vsync);

/* -------------------- idle mode -------------------- */
int oplus_adfr_idle_mode_handle(void *sde_encoder_virt, bool enter_idle);

/* -------------------- temperature detection -------------------- */
int oplus_adfr_temperature_detection_handle(void *dsi_display, int ntc_temp, int shell_temp);

/* -------------------- test te -------------------- */
enum hrtimer_restart oplus_adfr_test_te_timer_handler(struct hrtimer *timer);
int oplus_adfr_register_test_te_irq(void *dsi_display);

/* -------------------- osync mode -------------------- */
int oplus_adfr_set_osync_params(void *sde_connector, unsigned int oplus_adfr_osync_params);
int oplus_adfr_osync_min_fps_update(void *dsi_display);
int oplus_adfr_get_osync_window_min_fps(void *drm_connector);
int oplus_adfr_osync_threshold_lines_update(void *drm_connector, unsigned int *threshold_lines, unsigned int yres);
int oplus_adfr_osync_tearcheck_update(void *sde_encoder_phys);
int oplus_adfr_adjust_osync_tearcheck(void *sde_encoder_phys);
bool oplus_adfr_osync_backlight_filter(void *dsi_panel, unsigned int bl_level);
int oplus_adfr_osync_backlight_updated(void *sde_connector, bool osync_backlight_updated);
enum hrtimer_restart oplus_adfr_osync_mode_timer_handler(struct hrtimer *timer);
bool oplus_adfr_need_restore_osync_mode(void *sde_connector);
int oplus_adfr_need_force_off_osync_mode(void *dsi_display, bool need_force_off_osync_mode);
int oplus_adfr_force_off_osync_mode(void *sde_encoder_phys);
int oplus_adfr_need_resend_osync_cmd(void *dsi_display, bool need_resend_osync_cmd);
int oplus_adfr_resend_osync_cmd(void *dsi_display);

/* -------------------- node -------------------- */
/* adfr_config */
ssize_t oplus_adfr_set_config_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);
ssize_t oplus_adfr_get_config_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);
/* mux_vsync_switch */
ssize_t oplus_adfr_set_mux_vsync_switch_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);
ssize_t oplus_adfr_get_mux_vsync_switch_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);
/* test te */
int oplus_adfr_set_test_te(void *buf);
int oplus_adfr_get_test_te(void *buf);
ssize_t oplus_adfr_set_test_te_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);
ssize_t oplus_adfr_get_test_te_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);
/* high precision test */
ssize_t oplus_display_set_high_precision_rscc(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);
ssize_t oplus_display_get_high_precision_rscc(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);
ssize_t oplus_display_set_high_precision_fps(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);
ssize_t oplus_display_get_high_precision_fps(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);

#endif /* _OPLUS_ADFR_H_ */

