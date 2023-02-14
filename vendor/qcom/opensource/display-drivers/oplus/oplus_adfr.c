/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
** File : oplus_adfr.h
** Description : ADFR kernel module
** Version : 1.0
** Date : 2020/10/23
** Author : Display
******************************************************************/
#include "sde_trace.h"
#include "msm_drv.h"
#include "sde_kms.h"
#include "sde_connector.h"
#include "sde_crtc.h"
#include "sde_encoder_phys.h"
#include <uapi/linux/sched/types.h>

#include "dsi_display.h"
#include "dsi_panel.h"
#include "dsi_parser.h"
#include "dsi_drm.h"
#include "dsi_defs.h"

#include "oplus_adfr.h"
#include "oplus_dsi_support.h"
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "oplus_onscreenfingerprint.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
#if defined(CONFIG_PXLW_IRIS)
#include "dsi_iris_api.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_def.h"
#include "dsi_iris_pq.h"
#endif

#define OPLUS_ADFR_CONFIG_GLOBAL (1<<0)
#define OPLUS_ADFR_CONFIG_FAKEFRAME (1<<1)
#define OPLUS_ADFR_CONFIG_VSYNC_SWITCH (1<<2)
#define OPLUS_ADFR_CONFIG_VSYNC_SWITCH_MODE (1<<3)
#define OPLUS_ADFR_CONFIG_IDLE_MODE (1<<4)

#define OPLUS_ADFR_DEBUG_GLOBAL_DISABLE (1<<0)
#define OPLUS_ADFR_DEBUG_FAKEFRAME_DISABLE (1<<1)
#define OPLUS_ADFR_DEBUG_VSYNC_SWITCH_DISABLE (1<<2)
#define OPLUS_ADFR_DEBUG_IDLE_MODE_DISABLE (1<<4)

#define ADFR_GET_GLOBAL_CONFIG(config) ((config) & OPLUS_ADFR_CONFIG_GLOBAL)
#define ADFR_GET_FAKEFRAME_CONFIG(config) ((config) & OPLUS_ADFR_CONFIG_FAKEFRAME)
#define ADFR_GET_VSYNC_SWITCH_CONFIG(config) ((config) & OPLUS_ADFR_CONFIG_VSYNC_SWITCH)
#define ADFR_GET_VSYNC_SWITCH_MODE(config) ((config) & OPLUS_ADFR_CONFIG_VSYNC_SWITCH_MODE)
#define ADFR_GET_IDLE_MODE_CONFIG(config) ((config) & OPLUS_ADFR_CONFIG_IDLE_MODE)

#define OPLUS_ADFR_AUTO_MAGIC 0X00800000
#define OPLUS_ADFR_AUTO_MODE_MAGIC 0X00400000
#define OPLUS_ADFR_AUTO_MODE_VALUE(auto_value) (((auto_value)&0X003F0000)>>16)
#define OPLUS_ADFR_AUTO_FAKEFRAME_MAGIC 0X00008000
#define OPLUS_ADFR_AUTO_FAKEFRAME_VALUE(auto_value) (((auto_value)&0X00007F00)>>8)
#define OPLUS_ADFR_AUTO_MIN_FPS_MAGIC 0X00000080
#define OPLUS_ADFR_AUTO_MIN_FPS_VALUE(auto_value) ((auto_value)&0X0000007F)

#define SDC_AUTO_MIN_FPS_CMD_OFFSET 2
#define SDC_MANUAL_MIN_FPS_CMD_OFFSET 2
#define SDC_AUTO_MIN_FPS_CMD_HIGH_OFFSET 4
#define SDC_MANUAL_MIN_FPS_CMD_HIGH_OFFSET 4
#define SDC_MIN_FPS_CMD_SIZE 2

#define to_dsi_bridge(x)  container_of((x), struct dsi_bridge, base)
#define to_sde_encoder_phys_cmd(x) container_of(x, struct sde_encoder_phys_cmd, base)
#define to_dsi_display(x) container_of(x, struct dsi_display, host)

static u32 oplus_adfr_config = 0;
static u32 oplus_adfr_debug = 0;
static bool need_deferred_fakeframe = false;
/* add for adfr hardware revision compatibility */
static bool oplus_adfr_compatibility_mode = false;

/* qsync mode minfps */
bool oplus_adfr_qsync_mode_minfps_updated = false;
static u32 oplus_adfr_qsync_mode_minfps = 0;
/* disable qsync when backlight updated */
bool oplus_adfr_need_filter_backlight_cmd = false;

/* samsung auto mode */
bool oplus_adfr_auto_mode_updated = false;
static u32 oplus_adfr_auto_mode = 0;
bool oplus_adfr_auto_fakeframe_updated = false;
static u32 oplus_adfr_auto_fakeframe = 0;
bool oplus_adfr_auto_min_fps_updated = false;
static u32 oplus_adfr_auto_min_fps = 0;
static u32 oplus_adfr_auto_sw_fps = 0;
static u64 oplus_adfr_auto_update_counter = 0;
bool oplus_adfr_need_filter_auto_on_cmd = false;

/* pixelworks X7 emv */
#if defined(CONFIG_PXLW_IRIS)
u32 iris_current_extend_frame = OPLUS_ADFR_AUTO_MIN_FPS_MAX;
#endif

/* idle mode */
static u32 oplus_adfr_idle_mode = OPLUS_ADFR_IDLE_OFF;

struct oplus_te_refcount te_refcount = {0, 0, 0, 0};
/* dynamic te detect */
struct oplus_adfr_dynamic_te oplus_adfr_dynamic_te = {0};
DEFINE_MUTEX(dynamic_te_lock);

/* --------------- adfr misc ---------------*/

void oplus_adfr_init(void *panel_node)
{
	u32 config = 0;
	int rc = 0;
	struct device_node *of_node = panel_node;

	pr_info("kVRR oplus_adfr_init now.");

	if (!of_node) {
		pr_err("kVRR oplus_adfr_init: the param is null.");
		return;
	}


	rc = of_property_read_u32(of_node, "oplus,adfr-config", &config);
	if (rc == 0) {
		oplus_adfr_config = config;
	} else {
		oplus_adfr_config = 0;
	}

	if (is_factory_boot()) {
		oplus_adfr_config = 0;
		pr_info("kVRR adfr disabled in factory mode!\n");
		return;
	}

	/* add for adfr hardware revision compatibility */
	if (oplus_adfr_is_support()) {
		/* if adfr-compatibility-mode is define, should not do the vsync switch, just set to TE vsync always */
		oplus_adfr_compatibility_mode = of_property_read_bool(of_node, "oplus,adfr-compatibility-mode");
		/* add for dynamic te check */
		hrtimer_init(&oplus_adfr_dynamic_te.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		oplus_adfr_dynamic_te.timer.function = oplus_adfr_dynamic_te_timer_handler;
	}

	pr_info("kVRR adfr config = %#X, adfr compatibility mode = %d\n", oplus_adfr_config, oplus_adfr_compatibility_mode);
}

ssize_t oplus_adfr_get_debug(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	pr_err("kVRR get adfr config %#X debug %#X \n", oplus_adfr_config, oplus_adfr_debug);
	return sprintf(buf, "debug:0x%08X config:0x%08X auto_mode:0x%08X fakeframe:0x%08X auto_minfps:0x%08X auto_counter:%llu\n",
		oplus_adfr_debug, oplus_adfr_config, oplus_adfr_auto_mode, oplus_adfr_auto_fakeframe, oplus_adfr_auto_min_fps, oplus_adfr_auto_update_counter);
}

ssize_t oplus_adfr_set_debug(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%u", &oplus_adfr_debug);
	pr_err("kVRR get adfr config %#X debug %#X \n", oplus_adfr_config, oplus_adfr_debug);

	return count;
}

static inline bool oplus_adfr_fakeframe_is_enable(void)
{
	return (bool)(ADFR_GET_FAKEFRAME_CONFIG(oplus_adfr_config) &&
		!(oplus_adfr_debug & OPLUS_ADFR_DEBUG_FAKEFRAME_DISABLE) &&
		oplus_adfr_auto_fakeframe);
}

bool oplus_adfr_vsync_switch_is_enable(void)
{
	return (bool)(ADFR_GET_VSYNC_SWITCH_CONFIG(oplus_adfr_config) &&
		!(oplus_adfr_debug & OPLUS_ADFR_DEBUG_VSYNC_SWITCH_DISABLE));
}

enum oplus_vsync_mode oplus_adfr_get_vsync_mode(void)
{
	if (!oplus_adfr_vsync_switch_is_enable()) {
		return OPLUS_INVALID_VSYNC;
	}

	return (enum oplus_vsync_mode)ADFR_GET_VSYNC_SWITCH_MODE(oplus_adfr_config);
}

inline bool oplus_adfr_idle_mode_is_enable(void)
{
	return (bool)(ADFR_GET_IDLE_MODE_CONFIG(oplus_adfr_config) &&
		!(oplus_adfr_debug & OPLUS_ADFR_DEBUG_IDLE_MODE_DISABLE));
}

inline bool oplus_adfr_is_support(void)
{
	return (bool)(ADFR_GET_GLOBAL_CONFIG(oplus_adfr_config) &&
		!(oplus_adfr_debug & OPLUS_ADFR_DEBUG_GLOBAL_DISABLE));
}


/* --------------- msm_drv ---------------*/

static void oplus_adfr_thread_priority_worker(struct kthread_work *work)
{
	int ret = 0;
	struct sched_param param = { 0 };
	struct task_struct *task = current->group_leader;

	/**
	 * this priority was found during empiric testing to have appropriate
	 * realtime scheduling to process display updates and interact with
	 * other real time and normal priority task
	 */
	param.sched_priority = 16;
	ret = sched_setscheduler(task, SCHED_FIFO, &param);
	if (ret)
		pr_warn("pid:%d name:%s priority update failed: %d\n",
			current->tgid, task->comm, ret);
}

int oplus_adfr_thread_create(void *msm_priv, void *msm_ddev, void *msm_dev)
{
	struct msm_drm_private *priv;
	struct drm_device *ddev;
	struct device *dev;
	int i = 0;

	priv = msm_priv;
	ddev = msm_ddev;
	dev = msm_dev;

	for (i = 0; i < priv->num_crtcs; i++) {
		/* initialize adfr thread */
		priv->adfr_thread[i].crtc_id = priv->crtcs[i]->base.id;
		kthread_init_worker(&priv->adfr_thread[i].worker);
		priv->adfr_thread[i].dev = ddev;
		priv->adfr_thread[i].thread =
			kthread_run(kthread_worker_fn,
				&priv->adfr_thread[i].worker,
				"adfr:%d", priv->adfr_thread[i].crtc_id);
		kthread_init_work(&priv->thread_priority_work, oplus_adfr_thread_priority_worker);
		kthread_queue_work(&priv->adfr_thread[i].worker, &priv->thread_priority_work);
		kthread_flush_work(&priv->thread_priority_work);

		if (IS_ERR(priv->adfr_thread[i].thread)) {
			dev_err(dev, "kVRR failed to create adfr_commit kthread\n");
			priv->adfr_thread[i].thread = NULL;
		}

		if ((!priv->adfr_thread[i].thread)) {
			/* clean up previously created threads if any */
			for (; i >= 0; i--) {
				if (priv->adfr_thread[i].thread) {
					kthread_stop(
						priv->adfr_thread[i].thread);
					priv->adfr_thread[i].thread = NULL;
				}
			}
			return -EINVAL;
		}
	}
	pr_info("kVRR adfr thread create successfully\n");

	return 0;
}

void oplus_adfr_thread_destroy(void *msm_priv)
{
	struct msm_drm_private *priv;
	int i;

	priv = msm_priv;

	for (i = 0; i < priv->num_crtcs; i++) {
		if (priv->adfr_thread[i].thread) {
			kthread_flush_worker(&priv->adfr_thread[i].worker);
			kthread_stop(priv->adfr_thread[i].thread);
			priv->adfr_thread[i].thread = NULL;
		}
	}
}

/* ------------ sde_connector ------------ */
/* handle it early since qsync min fps dirty will disappeared with high probabilities */
int oplus_adfr_handle_qsync_mode_minfps(u32 propval)
{
	int handled = 0;
	SDE_INFO("kVRR update qsync mode minfps %u[%08X]\n", propval, propval);

	SDE_ATRACE_BEGIN("oplus_adfr_handle_qsync_mode_minfps");

	oplus_adfr_qsync_mode_minfps_updated = true;
	oplus_adfr_qsync_mode_minfps = propval;
	handled = 1;

	SDE_ATRACE_INT("oplus_adfr_qsync_mode_minfps", oplus_adfr_qsync_mode_minfps);
	SDE_ATRACE_END("oplus_adfr_handle_qsync_mode_minfps");

	SDE_INFO("kVRR qsync mode minfps %u[%d]\n",
		oplus_adfr_qsync_mode_minfps, oplus_adfr_qsync_mode_minfps_updated);

	return handled;
}

bool oplus_adfr_qsync_mode_minfps_is_updated(void) {
	bool updated = oplus_adfr_qsync_mode_minfps_updated;
	oplus_adfr_qsync_mode_minfps_updated = false;
	return updated;
}

u32 oplus_adfr_get_qsync_mode_minfps(void) {
	SDE_INFO("kVRR get qsync mode minfps %u\n", oplus_adfr_qsync_mode_minfps);
	return oplus_adfr_qsync_mode_minfps;
}

/* qsync mode timer */
enum hrtimer_restart oplus_adfr_qsync_mode_timer_handler(struct hrtimer *timer)
{
	struct sde_connector *c_conn =
			from_timer(c_conn, timer, qsync_mode_timer);

	/* qsync status reset */
	c_conn->qsync_mode_recovery = true;
	oplus_adfr_qsync_mode_minfps_updated = true;
	SDE_DEBUG("kVRR qsync_mode_timer handler\n");

	return HRTIMER_NORESTART;
}

void oplus_adfr_qsync_mode_timer_start(void *sde_connector, int deferred_ms)
{
	struct sde_connector *c_conn = sde_connector;

	SDE_DEBUG("kVRR qsync_mode_timer start\n");
	hrtimer_start(&c_conn->qsync_mode_timer, ms_to_ktime(deferred_ms), HRTIMER_MODE_REL);
}

/* --------------- sde_crtc ---------------*/

void sde_crtc_adfr_handle_frame_event(void *crt, void *event)
{
	struct drm_crtc *crtc = crt;
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);
	struct sde_crtc_frame_event *fevent = event;
	struct drm_encoder *encoder;

	/* cancel deferred adfr fakeframe timer */
	if (oplus_adfr_fakeframe_is_enable() &&
		(fevent->event & SDE_ENCODER_FRAME_EVENT_SIGNAL_RETIRE_FENCE)) {
		mutex_lock(&sde_crtc->crtc_lock);
		list_for_each_entry(encoder, &crtc->dev->mode_config.encoder_list, head) {
			if (encoder->crtc != crtc)
				continue;

			sde_encoder_adfr_cancel_fakeframe(encoder);
		}
		mutex_unlock(&sde_crtc->crtc_lock);
	}
}


/* --------------- sde_encoder ---------------*/

static inline struct dsi_display_mode_priv_info *oplus_get_current_mode_priv_info(struct drm_connector * drm_conn)
{
	struct msm_drm_private *priv;
	struct sde_kms *sde_kms;
	struct dsi_display *dsi_display;
	struct dsi_panel *panel;

	if (!drm_conn) {
		SDE_ERROR("kVRR adfr drm_conn is null.\n");
		return NULL;
	}

	priv = drm_conn->dev->dev_private;
	sde_kms = to_sde_kms(priv->kms);

	if (!sde_kms) {
		SDE_ERROR("kVRR adfr sde_kms is null.\n");
		return NULL;
	}

	if (sde_kms->dsi_display_count && sde_kms->dsi_displays) {
		/* only use primary dsi */
		dsi_display = sde_kms->dsi_displays[0];
	} else {
		SDE_ERROR("kVRR adfr sde_kms's dsi_display is null.\n");
		return NULL;
	}

	panel = dsi_display->panel;

	if (!panel || !panel->cur_mode) {
		SDE_ERROR("kVRR adfr dsi_display's panel is null.\n");
		return NULL;
	}

	return panel->cur_mode->priv_info;
}

void sde_encoder_adfr_prepare_commit(void *crt, void *enc, void *conn) {
	struct dsi_display_mode_priv_info *priv_info;
	struct drm_crtc *crtc = crt;
	struct drm_connector *drm_conn = conn;
	struct sde_connector *sde_conn = NULL;
	struct dsi_display *dsi_display = NULL;

	if (!oplus_adfr_fakeframe_is_enable()) {
		return;
	}

	/* when power on, disable deferred fakeframe
	* after power on and before first frame flush
	* if panel get a fakeframe then refresh itself (with a dirty buffer), tearing happen
	* so for power on case, set need_deferred_fakeframe false
	* this can avoid deferred fakeframe tearing issue (eg. AOD)
	* power off --> sde_encoder_virt_disable set "sde_enc->cur_master = NULL"
	* power on  --> sde_encoder_virt_enable  set "sde_enc->cur_master = XXX"
	* prepare_commit need cur_master is not null but it is before than sde_encoder_virt_enable
	* so use prepare_commit(NULL, NULL, NULL) to imply this commit is first commit after power on
	*/
	if (!crt && !enc && !conn) {
		need_deferred_fakeframe = false;
		/* SDE_ATRACE_INT("need_deferred_fakeframe", need_deferred_fakeframe); */
		return;
	}

	if (!crt || !enc || !conn) {
		SDE_ERROR("kVRR sde_encoder_adfr_prepare_commit error: %p %p %p",
			crt, enc, conn);
		return;
	}

	sde_conn = to_sde_connector(drm_conn);
	if (!sde_conn) {
		SDE_ERROR("kVRR sde_encoder_adfr_prepare_commit error: %p", sde_conn);
		return;
	}

	if (!sde_conn->display) {
		SDE_ERROR("kVRR sde_encoder_adfr_prepare_commit error: %p", sde_conn->display);
		return;
	}
	dsi_display = sde_conn->display;
	if (!dsi_display->panel) {
		SDE_ERROR("kVRR sde_encoder_adfr_prepare_commit error: %p", dsi_display->panel);
		return;
	}
	/* after power on, enable deferred fakeframe */
	/* if (get_oplus_display_power_status() == OPLUS_DISPLAY_POWER_ON) { */
	if (dsi_display->panel->power_mode == SDE_MODE_DPMS_ON) {
		need_deferred_fakeframe = true;
	} else {
		need_deferred_fakeframe = false;
		/* SDE_DEBUG("kVRR display stats: %d , skip fakeframe", dsi_display->panel->power_mode); */
		return;
	}

	priv_info = oplus_get_current_mode_priv_info(drm_conn);

	/* check 1st bit */
	if (!priv_info || !(priv_info->fakeframe_config & 0X00000001)) {
		return;
	}

	/* before commit send a fakeframe to triger the panel flush
	* but if pre-frame is pending, ignore this time
	* because pre-frame is a real frame, Not Need fakeframe
	* SDE_ATRACE_INT("frame_pending", sde_crtc_frame_pending(sde_enc->crtc));
	*/
	if ((sde_crtc_frame_pending(crtc) == 0)) {
		sde_encoder_adfr_trigger_fakeframe(enc);
		need_deferred_fakeframe = true;
	}
}

void sde_encoder_adfr_kickoff(void *crt, void *enc, void *conn) {
	struct dsi_display_mode_priv_info *priv_info;
	struct drm_connector *drm_conn = conn;
	int deferred_ms = -1;

	if (!oplus_adfr_fakeframe_is_enable()) {
		return;
	}

	/* SDE_ATRACE_INT("need_deferred_fakeframe", need_deferred_fakeframe); */
	if (!need_deferred_fakeframe) {
		/* SDE_ERROR("kVRR sde_encoder_adfr_kickoff skip, need_deferred_fakeframe is false."); */
		return;
	}

	if (!crt || !enc || !conn) {
		SDE_ERROR("kVRR sde_encoder_adfr_kickoff error:  %p %p %p",
			crt, enc, conn);
		return;
	}

	priv_info = oplus_get_current_mode_priv_info(drm_conn);

	/* check 2st bit */
	if (!priv_info || !(priv_info->fakeframe_config & 0X00000002)) {
		return;
	}
	deferred_ms = priv_info->deferred_fakeframe_time;

	oplus_adfr_fakeframe_timer_start(enc, deferred_ms);
	need_deferred_fakeframe = false;
}

/* --------------- sde_encoder_phys_cmd ---------------*/
/* Add for qsync tearing issue */
/* indicates whether filter backlight cmd is need */
bool oplus_adfr_backlight_cmd_filter_set(bool enable)
{
	oplus_adfr_need_filter_backlight_cmd = enable;
	return oplus_adfr_need_filter_backlight_cmd;
}

bool oplus_adfr_backlight_cmd_filter_get(void)
{
	return oplus_adfr_need_filter_backlight_cmd;
}

/* if force_qsync_mode_off is true, close qsync window immediately */
void oplus_adfr_force_qsync_mode_off(void *drm_connector)
{
	struct drm_connector *connector = drm_connector;
	struct sde_connector *c_conn;
	struct drm_bridge *temp_bridge;
	struct dsi_bridge *c_bridge;
	struct dsi_display *display;

	if (!connector || !connector->encoder)
		return;
	if (!drm_bridge_chain_get_first_bridge(connector->encoder))
		return;

	c_conn = to_sde_connector(connector);
	temp_bridge = drm_bridge_chain_get_first_bridge(c_conn->encoder);
	c_bridge = to_dsi_bridge(temp_bridge);
	display = c_bridge->display;

	if (!display)
		return;

	if (display->force_qsync_mode_off) {
		SDE_INFO("kVRR force qsync mode update %d -> %d\n",
				c_conn->qsync_mode, SDE_RM_QSYNC_DISABLED);
		c_conn->qsync_updated = true;
		c_conn->qsync_mode = SDE_RM_QSYNC_DISABLED;
		/* qsync disable need change min fps */
		c_conn->qsync_curr_dynamic_min_fps = 0;
		c_conn->qsync_deferred_window_status = SET_WINDOW_IMMEDIATELY;
		display->force_qsync_mode_off = false;
	} else if ((c_conn->qsync_mode != SDE_RM_QSYNC_DISABLED) &&
		(c_conn->qsync_deferred_window_status != DEFERRED_WINDOW_START) && c_conn->oplus_adfr_backlight_updated) {
		/* if qsync is enable and backlight status update, close qsync immediately */
		SDE_INFO("kVRR force qsync mode update %d -> %d\n",
				c_conn->qsync_mode, SDE_RM_QSYNC_DISABLED);
		c_conn->qsync_updated = true;
		c_conn->qsync_mode = SDE_RM_QSYNC_DISABLED;
		/* qsync disable need change min fps */
		c_conn->qsync_curr_dynamic_min_fps = 0;
		c_conn->qsync_deferred_window_status = SET_WINDOW_IMMEDIATELY;

		/* send qsync off cmd */
		sde_connector_prepare_commit(connector);
		/* start timer to delay qsync status recovery */
		oplus_adfr_qsync_mode_timer_start(c_conn, 1000);
	} else if (c_conn->qsync_mode != SDE_RM_QSYNC_DISABLED && (c_conn->qsync_deferred_window_status != DEFERRED_WINDOW_START)) {
		/* if backlight cmd is set after qsync window setting and qsync is enable, filter it
		otherwise tearing issue happen */
		oplus_adfr_backlight_cmd_filter_set(true);
	}

	c_conn->oplus_adfr_backlight_updated = false;
	return;
}

int oplus_adfr_adjust_tearcheck_for_dynamic_qsync(void *sde_phys_enc)
{
	struct sde_encoder_phys *phys_enc = sde_phys_enc;
	struct sde_hw_tear_check tc_cfg = {0};
	struct sde_connector *sde_conn = NULL;
	int ret = 0;

	if (!phys_enc || !phys_enc->connector) {
		SDE_ERROR("kVRR invalid encoder parameters\n");
		return -EINVAL;
	}

	sde_conn = to_sde_connector(phys_enc->connector);

	if (sde_connector_get_qsync_mode(phys_enc->connector) == 0 ||
		sde_connector_get_qsync_dynamic_min_fps(phys_enc->connector) == 0) {
		/* Fixed qsync window and panel min fps nonsynchronous issue */
		phys_enc->current_sync_threshold_start = phys_enc->qsync_sync_threshold_start;
		return ret;
	}

	SDE_ATRACE_BEGIN("adjust_tearcheck_for_qsync");
	SDE_ATRACE_INT("frame_state", atomic_read(&phys_enc->frame_state));
	SDE_DEBUG("frame_state = %d\n", atomic_read(&phys_enc->frame_state));

	/* this time maybe remain in qsync window, so shrink qsync window */
	/* to avoid tearing and keep qsync enable for this frame */
	if (atomic_read(&phys_enc->frame_state) != 0) {
		/* 300 is a estimated value */
		tc_cfg.sync_threshold_start = 300;
	} else {
		/* remain use original qsync window */
		tc_cfg.sync_threshold_start = phys_enc->qsync_sync_threshold_start;
	}

	if(phys_enc->current_sync_threshold_start != tc_cfg.sync_threshold_start) {
		SDE_ATRACE_BEGIN("update_qsync");

		if (phys_enc->has_intf_te &&
			phys_enc->hw_intf->ops.update_tearcheck)
			phys_enc->hw_intf->ops.update_tearcheck(
				phys_enc->hw_intf, &tc_cfg);
		else if (phys_enc->hw_pp->ops.update_tearcheck)
			phys_enc->hw_pp->ops.update_tearcheck(
				phys_enc->hw_pp, &tc_cfg);
		SDE_EVT32(DRMID(phys_enc->parent), tc_cfg.sync_threshold_start);
		phys_enc->current_sync_threshold_start = tc_cfg.sync_threshold_start;
		/* trigger AP update qsync flush */
		sde_conn->qsync_updated = true;

		SDE_ATRACE_END("update_qsync");
	}

	SDE_DEBUG("kVRR threshold_lines %d\n", phys_enc->current_sync_threshold_start);
	SDE_ATRACE_INT("threshold_lines", phys_enc->current_sync_threshold_start);
	SDE_ATRACE_END("adjust_tearcheck_for_qsync");

	return ret;
}

/* --------------- dsi_connector ---------------*/

/* fake frame */
int sde_connector_send_fakeframe(void *conn)
{
	struct drm_connector *connector = conn;
	struct sde_connector *c_conn;
	int rc;

	if (!connector) {
		SDE_ERROR("kVRR invalid argument\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(connector);
	if (!c_conn->display) {
		SDE_ERROR("kVRR invalid connector display\n");
		return -EINVAL;
	}

	rc = dsi_display_send_fakeframe(c_conn->display);

	SDE_EVT32(connector->base.id, rc);
	return rc;
}

/* --------------- dsi_display ---------------*/

/* qsync enhance */
/* update qsync min fps */
int dsi_display_qsync_update_min_fps(void *dsi_display, void *dsi_params)
{
	struct dsi_display *display = dsi_display;
	struct msm_display_conn_params *params = dsi_params;
	int i;
	int rc = 0;

	if (!params->qsync_update) {
		return 0;
	}

	/* allow qsync off but update qsync min fps only */
	SDE_ATRACE_BEGIN("dsi_display_qsync_update_min_fps");

	mutex_lock(&display->display_lock);

	display_for_each_ctrl(i, display) {
		/* send the commands to updaet qsync min fps */
		rc = dsi_panel_send_qsync_min_fps_dcs(display->panel, i, params->qsync_dynamic_min_fps);
		if (rc) {
			DSI_ERR("kVRR fail qsync UPDATE cmds rc:%d\n", rc);
			goto exit;
		}
	}

exit:
	SDE_EVT32(params->qsync_mode, params->qsync_dynamic_min_fps, rc);
	mutex_unlock(&display->display_lock);

	SDE_ATRACE_END("dsi_display_qsync_update_min_fps");

	return rc;
}

/* save qsync info, then restore qsync status after panel enable*/
int dsi_display_qsync_restore(void *dsi_display)
{
	struct msm_display_conn_params params;
	struct dsi_display *display = dsi_display;
	int rc = 0;

	if (display->need_qsync_restore) {
		display->need_qsync_restore = false;
	} else {
		return 0;
	}

	params.qsync_update = display->current_qsync_mode ||
						  display->current_qsync_dynamic_min_fps;

	if (!params.qsync_update) {
		DSI_DEBUG("kVRR %s:INFO: qsync status is clean.\n", __func__);
		return 0;
	}

	params.qsync_mode = display->current_qsync_mode;
	params.qsync_dynamic_min_fps = display->current_qsync_dynamic_min_fps;

	SDE_ATRACE_BEGIN("dsi_display_qsync_restore");

	DSI_INFO("kVRR qsync restore mode %d minfps %d \n",
			params.qsync_mode, params.qsync_dynamic_min_fps);
	rc = dsi_display_pre_commit(display, &params);
	SDE_EVT32(params.qsync_mode, params.qsync_dynamic_min_fps, rc);

	SDE_ATRACE_END("dsi_display_qsync_restore");

	return rc;
}

/* fake frame */
int dsi_display_send_fakeframe(void *disp)
{
	struct dsi_display *display = (struct dsi_display *)disp;
	int i, rc = 0;

	if (!display) {
		pr_err("kVRR Invalid params\n");
		return -EINVAL;
	}

	SDE_ATRACE_BEGIN("dsi_display_send_fakeframe");
	display_for_each_ctrl(i, display) {
		/* send the commands to simulate a frame transmission */
		rc = dsi_panel_send_fakeframe_dcs(display->panel, i);
		if (rc) {
			DSI_ERR("kVRR fail fake frame cmds rc:%d\n", rc);
			goto exit;
		}
	}

exit:
	SDE_ATRACE_END("dsi_display_send_fakeframe");
	SDE_EVT32(rc);

	return rc;
}

void oplus_adfr_set_dynamic_te_config(int config)
{
	mutex_lock(&dynamic_te_lock);
	oplus_adfr_dynamic_te.config = config;
	mutex_unlock(&dynamic_te_lock);
}

/* dynamic te timer */
enum hrtimer_restart oplus_adfr_dynamic_te_timer_handler(struct hrtimer *timer)
{
	/* update report rate if enter idle mode */
	oplus_adfr_dynamic_te.refresh_rate = 120/(oplus_adfr_auto_min_fps + 1);
	if (oplus_adfr_dynamic_te.config == OPLUS_ADFR_DYNAMIC_TE_ENABLE_WITCH_LOG) {
		DSI_INFO("kVRR dynamic te: enter idle mode, refresh_rate=%d\n", oplus_adfr_dynamic_te.refresh_rate);
	}

	return HRTIMER_NORESTART;
}

/* dynamic te detect */
irqreturn_t oplus_adfr_dynamic_te_handler(int irq, void *data)
{
	static int mid_refresh_rate_count = 0;
	static int high_refresh_rate_count = 0;
	int temp_refresh_rate = 0;
	struct dsi_display *display = (struct dsi_display *)data;
	struct dsi_mode_info timing;

	if (!display)
		return IRQ_HANDLED;

	if (oplus_adfr_dynamic_te.config != OPLUS_ADFR_DYNAMIC_TE_DISABLE) {
		timing = display->panel->cur_mode->timing;

		/* check the te interval to calculate framerate */
		oplus_adfr_dynamic_te.last_te_timestamp = oplus_adfr_dynamic_te.current_te_timestamp;
		oplus_adfr_dynamic_te.current_te_timestamp = (u64)ktime_to_ms(ktime_get());
		temp_refresh_rate = 1000/(oplus_adfr_dynamic_te.current_te_timestamp - oplus_adfr_dynamic_te.last_te_timestamp);

		/* filtering algorithm */
		if (timing.h_skew == SDC_ADFR || timing.h_skew == SDC_MFR) {
			if (timing.refresh_rate == 90) {
				mid_refresh_rate_count = 0;
				high_refresh_rate_count = 0;
				/* fix frame rate */
				oplus_adfr_dynamic_te.refresh_rate = 90;
			} else if (timing.refresh_rate == 120 || timing.refresh_rate == 60) {
				if (temp_refresh_rate > 55) {
					mid_refresh_rate_count = 0;
					high_refresh_rate_count++;
					if (timing.refresh_rate == 120) {
						/* update refresh rate if 4 continous temp_refresh_rate are greater than 55 */
						if (high_refresh_rate_count == 4) {
							if (oplus_adfr_auto_sw_fps == 60) {
								/* show the ddic refresh rate */
								if (oplus_adfr_auto_min_fps == OPLUS_ADFR_AUTO_MIN_FPS_MAX) {
									oplus_adfr_dynamic_te.refresh_rate = 120;
								} else {
									oplus_adfr_dynamic_te.refresh_rate = 60;
								}
							} else {
								oplus_adfr_dynamic_te.refresh_rate = 120;
							}
							high_refresh_rate_count--;
						}
					} else {
							/* update refresh rate if 4 continous temp_refresh_rate are greater than 55 */
							if (high_refresh_rate_count >= 4) {
								oplus_adfr_dynamic_te.refresh_rate = 60;
								high_refresh_rate_count = 3;
							}
					}
				} else if (temp_refresh_rate > 16 && temp_refresh_rate <= 55) {
					mid_refresh_rate_count++;
					high_refresh_rate_count = 0;
					/* update refresh rate if 1 continous temp_refresh_rate are greater than 16 and less than or equal to 55 */
					if (mid_refresh_rate_count == 1) {
						oplus_adfr_dynamic_te.refresh_rate = 30;
						mid_refresh_rate_count--;
					}
				} else {
					mid_refresh_rate_count = 0;
					high_refresh_rate_count = 0;
					/* update report value if refresh rate is less than or equal to 16 */
					oplus_adfr_dynamic_te.refresh_rate = temp_refresh_rate;
				}
			} else {
					mid_refresh_rate_count = 0;
					high_refresh_rate_count = 0;
					oplus_adfr_dynamic_te.refresh_rate = 0;
			}
		} else if (timing.h_skew == OPLUS_ADFR || timing.h_skew == OPLUS_MFR) {
				mid_refresh_rate_count = 0;
				high_refresh_rate_count = 0;
				oplus_adfr_dynamic_te.refresh_rate = 120;
		} else {
				mid_refresh_rate_count = 0;
				high_refresh_rate_count = 0;
				oplus_adfr_dynamic_te.refresh_rate = 0;
		}

		if (oplus_adfr_dynamic_te.config == OPLUS_ADFR_DYNAMIC_TE_ENABLE_WITCH_LOG) {
			DSI_INFO("kVRR dynamic te: temp_refresh_rate=%d\n", temp_refresh_rate);
			/* print key information every te interval */
			DSI_INFO("kVRR dynamic te: last_te_timestamp=%lu, current_te_timestamp=%lu, refresh_rate=%d\n",
				oplus_adfr_dynamic_te.last_te_timestamp, oplus_adfr_dynamic_te.current_te_timestamp,
					oplus_adfr_dynamic_te.refresh_rate);
			DSI_INFO("kVRR hactive=%d,vactive=%d,fps=%d,h_skew=%d,auto_mode=%d,auto_minfps=%d,sw_fps=%d,fakeframe=%d,idle_mode=%d,qsync_mode=%d,qsync_minfps=%d\n",
					timing.h_active, timing.v_active, timing.refresh_rate, timing.h_skew,
						oplus_adfr_auto_mode, oplus_adfr_auto_min_fps, oplus_adfr_auto_sw_fps,
							oplus_adfr_auto_fakeframe, oplus_adfr_idle_mode, display->current_qsync_mode,
								display->current_qsync_dynamic_min_fps);
		}
	}

	return IRQ_HANDLED;
}

void oplus_adfr_register_dynamic_te_irq(void *dsi_display)
{
	struct dsi_display *display = dsi_display;
	int rc = 0;
	struct platform_device *pdev;
	struct device *dev;
	unsigned int dynamic_te_irq;

	pdev = display->pdev;
	if (!pdev) {
		DSI_ERR("invalid platform device\n");
		return;
	}

	dev = &pdev->dev;
	if (!dev) {
		DSI_ERR("invalid device\n");
		return;
	}

	if (display->trusted_vm_env) {
		DSI_INFO("GPIO's are not enabled in trusted VM\n");
		return;
	}

	if (!gpio_is_valid(display->panel->dynamic_te_gpio)) {
		rc = -EINVAL;
		goto error;
	}

	dynamic_te_irq = gpio_to_irq(display->panel->dynamic_te_gpio);

	/* Avoid deferred spurious irqs with disable_irq() */
	irq_set_status_flags(dynamic_te_irq, IRQ_DISABLE_UNLAZY);

	/* detect TE rising edge */
	rc = devm_request_irq(dev, dynamic_te_irq, oplus_adfr_dynamic_te_handler,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"DYNAMIC_TE_GPIO", display);
	if (rc) {
		DSI_ERR("dynamic TE request_irq failed rc:%d\n", rc);
		irq_clear_status_flags(dynamic_te_irq, IRQ_DISABLE_UNLAZY);
		goto error;
	}

	if (display->panel->dynamic_te_gpio != display->disp_te_gpio &&
			display->panel->dynamic_te_gpio != display->disp_te_gpio_1)
		disable_irq(dynamic_te_irq);

	DSI_DEBUG("kVRR register dynamic te irq successfully\n");

	return;

error:
	DSI_WARN("Unable to register for dynamic TE IRQ\n");
}

ssize_t oplus_adfr_get_dynamic_te(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	struct dsi_display *display = oplus_display_get_current_display();
	struct dsi_mode_info timing;
	int refresh_rate;

	if (display == NULL) {
		DSI_ERR("%s error :NULL display", __func__);
		return -EINVAL;
	}

	if (display->panel == NULL) {
		DSI_ERR("%s error :NULL panel", __func__);
		return -EINVAL;
	}

	if (display->panel->cur_mode == NULL) {
		DSI_ERR("%s error :NULL cur_mode", __func__);
		return -EINVAL;
	}

	if (!gpio_is_valid(display->panel->dynamic_te_gpio)) {
		timing = display->panel->cur_mode->timing;
		refresh_rate = timing.refresh_rate;
		return sprintf(buf, "%d\n", refresh_rate);
	}

	if (!strcmp(display->display_type, "primary")) {
		refresh_rate = oplus_adfr_dynamic_te.refresh_rate;
		DSI_INFO("kVRR dynamic te refresh rate is %d\n", refresh_rate);
	}
	if (!strcmp(display->display_type, "secondary")) {
		timing = display->panel->cur_mode->timing;
		refresh_rate = timing.refresh_rate;
	}
	return sprintf(buf, "%d\n", refresh_rate);
}

ssize_t oplus_adfr_set_dynamic_te(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct dsi_display *display = oplus_display_get_current_display();
	unsigned int dynamic_te_irq;
	int config = 0;

	if (display == NULL) {
		DSI_ERR("%s error :NULL display", __func__);
		return -EINVAL;
	}

	if (display->panel == NULL) {
		DSI_ERR("%s error :NULL panel", __func__);
		return -EINVAL;
	}

	if (!gpio_is_valid(display->panel->dynamic_te_gpio)) {
		DSI_ERR("invalid dynamic te gpio\n");
		return count;
	}

	sscanf(buf, "%du", &config);
	oplus_adfr_set_dynamic_te_config(config);

	dynamic_te_irq = gpio_to_irq(display->panel->dynamic_te_gpio);

	if (oplus_adfr_dynamic_te.config != OPLUS_ADFR_DYNAMIC_TE_DISABLE) {
		enable_irq(dynamic_te_irq);
		DSI_INFO("kVRR dynamic te detect enable\n");
	} else {
		disable_irq(dynamic_te_irq);
		DSI_INFO("kVRR dynamic te detect disable\n");
	}

	return count;
}

int oplus_display_set_dynamic_te(void *buf)
{
	struct dsi_display *display = oplus_display_get_current_display();
	unsigned int dynamic_te_irq;
	uint32_t *buf_temp = buf;
	uint32_t config = *buf_temp;

	if (display == NULL) {
		DSI_ERR("%s error :NULL display", __func__);
		return -EINVAL;
	}

	if (display->panel == NULL) {
		DSI_ERR("%s error :NULL panel", __func__);
		return -EINVAL;
	}

	if (!gpio_is_valid(display->panel->dynamic_te_gpio)) {
		DSI_ERR("invalid dynamic te gpio\n");
		return -EINVAL;
	}

	oplus_adfr_set_dynamic_te_config(config);

	dynamic_te_irq = gpio_to_irq(display->panel->dynamic_te_gpio);

	if (oplus_adfr_dynamic_te.config != OPLUS_ADFR_DYNAMIC_TE_DISABLE) {
		enable_irq(dynamic_te_irq);
		DSI_INFO("kVRR dynamic te detect enable\n");
	} else {
		disable_irq(dynamic_te_irq);
		DSI_INFO("kVRR dynamic te detect disable\n");
	}

	return 0;
}

int oplus_display_get_dynamic_te(void *buf)
{
	struct dsi_display *display = oplus_display_get_current_display();
	struct dsi_mode_info timing;
	uint32_t *refresh_rate = buf;

	if (display == NULL) {
		DSI_ERR("%s error :NULL display", __func__);
		return -EINVAL;
	}

	if (display->panel == NULL) {
		DSI_ERR("%s error :NULL panel", __func__);
		return -EINVAL;
	}

	if (display->panel->cur_mode == NULL) {
		DSI_ERR("%s error :NULL cur_mode", __func__);
		return -EINVAL;
	}

	if (!gpio_is_valid(display->panel->dynamic_te_gpio)) {
		timing = display->panel->cur_mode->timing;
		*refresh_rate = timing.refresh_rate;
		return 0;
	}

	if (!strcmp(display->display_type, "primary")) {
		*refresh_rate = oplus_adfr_dynamic_te.refresh_rate;
		DSI_INFO("kVRR dynamic te refresh rate is %d\n", *refresh_rate);
	}
	if (!strcmp(display->display_type, "secondary")) {
		timing = display->panel->cur_mode->timing;
		*refresh_rate = timing.refresh_rate;
	}

	return 0;
}

/* --------------- dsi_panel ---------------*/

/* qsync enhance */
const char *qsync_min_fps_set_map[DSI_CMD_QSYNC_MIN_FPS_COUNTS] = {
	"qcom,mdss-dsi-qsync-min-fps-0",
	"qcom,mdss-dsi-qsync-min-fps-1",
	"qcom,mdss-dsi-qsync-min-fps-2",
	"qcom,mdss-dsi-qsync-min-fps-3",
	"qcom,mdss-dsi-qsync-min-fps-4",
	"qcom,mdss-dsi-qsync-min-fps-5",
	"qcom,mdss-dsi-qsync-min-fps-6",
	"qcom,mdss-dsi-qsync-min-fps-7",
	"qcom,mdss-dsi-qsync-min-fps-8",
	"qcom,mdss-dsi-qsync-min-fps-9",
};

/* qsync enhance */
int dsi_panel_send_qsync_min_fps_dcs(void *dsi_panel,
		int ctrl_idx, uint32_t min_fps)
{
	struct dsi_panel *panel = dsi_panel;
	struct dsi_display_mode_priv_info *priv_info;
	int rc = 0;
	int i = 0;

	if (!panel || !panel->cur_mode) {
		DSI_ERR("kVRR Invalid params\n");
		return -EINVAL;
	}

	priv_info = panel->cur_mode->priv_info;

	mutex_lock(&panel->panel_lock);

	/* select a best fps to fit min_fps */
	for(i = priv_info->qsync_min_fps_sets_size - 1; i >= 0; i--) {
		if(priv_info->qsync_min_fps_sets[i] <= min_fps) {
			DSI_DEBUG("kVRR ctrl:%d qsync find min fps %d\n", ctrl_idx, priv_info->qsync_min_fps_sets[i]);
			break;
		}
	}

	if(i >= 0 && i < priv_info->qsync_min_fps_sets_size) {
		DSI_INFO("kVRR ctrl:%d qsync update min fps %d use \n", ctrl_idx, min_fps);
		SDE_ATRACE_INT("oplus_adfr_qsync_mode_minfps_cmd", min_fps);
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_QSYNC_MIN_FPS_0+i);
		if (rc)
			DSI_ERR("kVRR [%s] failed to send DSI_CMD_QSYNC_MIN_FPS cmds rc=%d\n",
				panel->name, rc);
	} else {
		DSI_ERR("kVRR ctrl:%d failed to sets qsync min fps %u, %d\n", ctrl_idx, min_fps, i);
	}
	mutex_unlock(&panel->panel_lock);
	return rc;
}

/* fake frame */
int dsi_panel_send_fakeframe_dcs(void *dsi_panel,
		int ctrl_idx)
{
	struct dsi_panel *panel = dsi_panel;
	int rc = 0;

	/* SDC's auto, fakeframe and minfps are available only after power on */

	if (!panel) {
		DSI_ERR("kVRR invalid params\n");
		return -EINVAL;
	}

	/* if (get_oplus_display_power_status() != OPLUS_DISPLAY_POWER_ON) { */
	if (panel->power_mode != SDE_MODE_DPMS_ON) {
		DSI_INFO("kVRR ignore %s when power is %d", __FUNCTION__, panel->power_mode);
		return 0;
	}

	if (!oplus_adfr_fakeframe_is_enable()) {
		DSI_INFO("kVRR fakeframe canceled\n");
		return 0;
	}
	mutex_lock(&panel->panel_lock);

	DSI_DEBUG("kVRR ctrl:%d fake frame\n", ctrl_idx);
	if (get_oplus_display_power_status() == OPLUS_DISPLAY_POWER_ON) {
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_FAKEFRAME);
		if (rc)
			DSI_ERR("kVRR [%s] failed to send DSI_CMD_FAKEFRAME cmds rc=%d\n",
					panel->name, rc);
	}

	mutex_unlock(&panel->panel_lock);
	return rc;
}


/* qsync enhance */
static int dsi_panel_parse_qsync_min_fps(
		struct dsi_display_mode_priv_info *priv_info,
		struct dsi_parser_utils *utils)
{
	int rc = 0;
	u32 i;

	if (!priv_info) {
		DSI_ERR("kVRR dsi_panel_parse_qsync_min_fps err: invalid mode priv info\n");
		return -EINVAL;
	}

	priv_info->qsync_min_fps_sets_size = 0;

	for (i = 0; i < DSI_CMD_QSYNC_MIN_FPS_COUNTS; i++) {
		rc = utils->read_u32(utils->data, qsync_min_fps_set_map[i],
			&priv_info->qsync_min_fps_sets[i]);
		if (rc) {
			DSI_DEBUG("kVRR failed to parse qsync min fps set %u\n", i);
			break;
		}
		else {
			priv_info->qsync_min_fps_sets_size++;
			DSI_DEBUG("kVRR parse qsync min fps set %u = %u\n",
			priv_info->qsync_min_fps_sets_size - 1, priv_info->qsync_min_fps_sets[i]);
		}
	}

	return rc;
}

/* fake frame */
static int dsi_panel_parse_fakeframe(
		struct dsi_display_mode_priv_info *priv_info,
		struct dsi_parser_utils *utils)
{
	int rc = 0;

	if (!priv_info) {
		DSI_ERR("kVRR dsi_panel_parse_fakeframe err: invalid mode priv info\n");
		return -EINVAL;
	}

	priv_info->fakeframe_config = 0;
	priv_info->deferred_fakeframe_time = 0;

	rc = utils->read_u32(utils->data, "oplus,adfr-fakeframe-config",
			&priv_info->fakeframe_config);
	if (rc) {
		DSI_DEBUG("kVRR failed to parse fakeframe.\n");
	}

	rc = utils->read_u32(utils->data, "oplus,adfr-fakeframe-deferred-time",
			&priv_info->deferred_fakeframe_time);
	if (rc) {
		DSI_DEBUG("kVRR failed to parse deferred_fakeframe_time.\n");
	}

	DSI_DEBUG("kVRR adfr fakeframe_config: %u, deferred_fakeframe_time: %u \n",
		priv_info->fakeframe_config, priv_info->deferred_fakeframe_time);

	return rc;
}

int dsi_panel_parse_adfr(void *dsi_mode, void *dsi_utils)
{
	struct dsi_display_mode *mode = dsi_mode;
	struct dsi_parser_utils *utils = dsi_utils;
	struct dsi_display_mode_priv_info *priv_info = mode->priv_info;

	/* qsync enhance */
	if (dsi_panel_parse_qsync_min_fps(priv_info, utils)) {
		DSI_DEBUG("kVRR adfr failed to parse qsyn min fps\n");
	}
	/* fake frame */
	if (dsi_panel_parse_fakeframe(priv_info, utils)) {
		DSI_DEBUG("kVRR adfr failed to parse fakeframe\n");
	}

	return 0;
}

/* update fakeframe status according to different situation */
int oplus_adfr_fakeframe_status_update(void *dsi_panel, bool force_disable)
{
	struct dsi_panel *panel = dsi_panel;
	int refresh_rate = 120;

	if (!panel || !panel->cur_mode) {
		DSI_ERR("kVRR invalid params\n");
		return -EINVAL;
	}

	refresh_rate = panel->cur_mode->timing.refresh_rate;

	if (force_disable == true) {
		oplus_adfr_auto_fakeframe = OPLUS_ADFR_FAKEFRAME_OFF;
	} else {
		/* if fakeframe is sent after resolution switch, local garbage issue will happen in low probability */
		if (panel->cur_h_active != panel->cur_mode->timing.h_active) {
			oplus_adfr_auto_fakeframe = OPLUS_ADFR_FAKEFRAME_OFF;
		} else {
			if (refresh_rate == 120 || refresh_rate == 90) {
				if (oplus_adfr_auto_sw_fps == 60) {
					/* if oplus_adfr_auto_sw_fps is 60hz, no need to send fakeframe */
					oplus_adfr_auto_fakeframe = OPLUS_ADFR_FAKEFRAME_OFF;
				} else {
					oplus_adfr_auto_fakeframe = OPLUS_ADFR_FAKEFRAME_ON;
				}
			} else {
				oplus_adfr_auto_fakeframe = OPLUS_ADFR_FAKEFRAME_OFF;
			}
		}
	}
	DSI_INFO("kVRR sw_fps %d, fakeframe %d\n", oplus_adfr_auto_sw_fps, oplus_adfr_auto_fakeframe);
	SDE_ATRACE_INT("oplus_adfr_auto_fakeframe", oplus_adfr_auto_fakeframe);

	return 0;
}

/* Add for adfr status reset */
/* reset auto mode status as panel power on and timing switch to SM */
void dsi_panel_adfr_status_reset(void *dsi_panel)
{
	struct dsi_panel *panel = dsi_panel;
	u32 refresh_rate = 120;
	u32 h_skew = SDC_ADFR;
	u32 oplus_adfr_auto_min_fps_cmd = OPLUS_ADFR_AUTO_MIN_FPS_MAX;

	if ((panel == NULL) || (panel->cur_mode == NULL)) {
		DSI_ERR("kVRR Invalid params");
		return;
	}

	h_skew = panel->cur_mode->timing.h_skew;
	refresh_rate = panel->cur_mode->timing.refresh_rate;

	if ((h_skew == SDC_ADFR) || (h_skew == SDC_MFR)) {
		/* after auto off cmd was sent, auto on cmd filter start */
		oplus_adfr_auto_on_cmd_filter_set(true);
		oplus_adfr_auto_mode = OPLUS_ADFR_AUTO_OFF;

		oplus_adfr_fakeframe_status_update(panel, false);

		if (refresh_rate == 60) {
			oplus_adfr_auto_min_fps = OPLUS_ADFR_AUTO_MIN_FPS_60HZ;
		} else {
			/* 90hz min fps in auto mode off should be 0x08 which will be corrected before cmd sent */
			oplus_adfr_auto_min_fps = OPLUS_ADFR_AUTO_MIN_FPS_MAX;
		}

		if (refresh_rate == 90) {
			/* should +9 in auto off mode */
			oplus_adfr_auto_min_fps_cmd = OPLUS_ADFR_AUTO_MIN_FPS_MAX + 9;
		} else {
			oplus_adfr_auto_min_fps_cmd = oplus_adfr_auto_min_fps;
		}

		/* update auto mode and qsync para when timing switch or panel enable for debug */
		SDE_ATRACE_INT("oplus_adfr_auto_mode", oplus_adfr_auto_mode);
		SDE_ATRACE_INT("oplus_adfr_auto_min_fps", oplus_adfr_auto_min_fps);
		SDE_ATRACE_INT("oplus_adfr_auto_mode_cmd", oplus_adfr_auto_mode);
		SDE_ATRACE_INT("oplus_adfr_auto_min_fps_cmd", oplus_adfr_auto_min_fps_cmd);
		SDE_ATRACE_INT("oplus_adfr_qsync_mode_minfps_cmd", 0);
		DSI_INFO("kVRR auto mode reset: auto mode %d, fakeframe %d, min fps %d\n", oplus_adfr_auto_mode,
			oplus_adfr_auto_fakeframe, oplus_adfr_auto_min_fps);
	} else {
		SDE_ATRACE_INT("oplus_adfr_auto_mode_cmd", 0);
		SDE_ATRACE_INT("oplus_adfr_auto_min_fps_cmd", 0);
		SDE_ATRACE_INT("oplus_adfr_qsync_mode_minfps_cmd", refresh_rate);
		DSI_INFO("kVRR oplus_adfr_qsync_mode_minfps_cmd %d\n", refresh_rate);
	}
	SDE_ATRACE_INT("qsync_mode_cmd", 0);
	SDE_ATRACE_INT("h_skew", h_skew);

#if defined(CONFIG_PXLW_IRIS)
	iris_current_extend_frame = oplus_adfr_auto_min_fps;
#endif
	return;
}

/* --------------- vsync switch ---------------*/

/* ------------- mux switch ------------ */
static int oplus_dsi_display_enable_and_waiting_for_next_te_irq(struct dsi_display *display)
{
	int const switch_te_timeout = msecs_to_jiffies(1100);

	dsi_display_adfr_change_te_irq_status(display, true);
	DSI_INFO("kVRR Waiting for the next TE to switch\n");

	display->panel->vsync_switch_pending = true;
	reinit_completion(&display->switch_te_gate);

	if (!wait_for_completion_timeout(&display->switch_te_gate, switch_te_timeout)) {
		DSI_ERR("kVRR vsync switch TE check failed\n");
		dsi_display_adfr_change_te_irq_status(display, false);
		return -EINVAL;
	}

	return 0;
}

/*GPIO SWITCH: 0-TP Vsync    1-TE Vsync*/
static int oplus_dsi_display_vsync_switch_check_te(struct dsi_display *display, int level)
{
	int rc = 0;

	if ((display == NULL) || (display->panel == NULL)) {
		DSI_ERR("kVRR Invalid params");
		return -EINVAL;
	}

	if (level == display->panel->vsync_switch_gpio_level) {
		DSI_INFO("kVRR vsync_switch_gpio is already %d\n", level);
		return 0;
	}

	/* add for Filter out all vsync switch */
	if (display->panel->force_te_vsync == true) {
		DSI_INFO("kVRR force te vsync, filter other vsync switch\n");
		return 0;
	}

	if (!gpio_is_valid(display->panel->vsync_switch_gpio)) {
		DSI_ERR("kVRR vsync_switch_gpio is invalid\n");
		return -EINVAL;
	}

	oplus_dsi_display_enable_and_waiting_for_next_te_irq(display);

	if (oplus_adfr_compatibility_mode == false) {
		if (level) {
			rc = gpio_direction_output(display->panel->vsync_switch_gpio, 1);
			if (rc) {
				DSI_ERR("kVRR unable to set dir for vsync_switch_gpio, rc=%d\n", rc);
				dsi_display_adfr_change_te_irq_status(display, false);
				return rc;
			} else {
				DSI_INFO("kVRR set vsync_switch_gpio to 1\n");
			}
		} else {
			gpio_set_value(display->panel->vsync_switch_gpio, 0);
			DSI_INFO("kVRR set vsync_switch_gpio to 0\n");
		}
	}

	dsi_display_adfr_change_te_irq_status(display, false);

	display->panel->vsync_switch_gpio_level = level;
	SDE_ATRACE_INT("vsync_switch_gpio_level", display->panel->vsync_switch_gpio_level);

	return rc;
}

static int oplus_dsi_display_set_vsync_switch_gpio(struct dsi_display *display, int level)
{
	struct dsi_panel *panel = NULL;
	int rc = 0;

	/* only support in mux switch */
	if (oplus_adfr_get_vsync_mode() != OPLUS_EXTERNAL_TE_TP_VSYNC) {
		DSI_ERR("kVRR %s is not supported\n", __func__);
		return -EINVAL;
	}

	if ((display == NULL) || (display->panel == NULL))
		return -EINVAL;

	panel = display->panel;

	mutex_lock(&display->display_lock);

	if (!panel->panel_initialized) {
		if (gpio_is_valid(panel->vsync_switch_gpio)) {
			if (level) {
				rc = gpio_direction_output(panel->vsync_switch_gpio, 1);	/*TE Vsync */
				if (rc) {
					DSI_ERR("kVRR unable to set dir for vsync_switch_gpio gpio rc=%d\n", rc);
				} else {
					DSI_INFO("kVRR set vsync_switch_gpio to 1\n");
				}
			} else {
				gpio_set_value(panel->vsync_switch_gpio, 0);	/* TP Vsync */
				DSI_INFO("kVRR set vsync_switch_gpio to 0\n");
			}
			panel->vsync_switch_gpio_level = level;
			SDE_ATRACE_INT("vsync_switch_gpio_level", panel->vsync_switch_gpio_level);
		}
	} else {
		oplus_dsi_display_vsync_switch_check_te(display, level);
	}

	mutex_unlock(&display->display_lock);
	return rc;
}

static int oplus_dsi_display_get_vsync_switch_gpio(struct dsi_display *display)
{
	if ((display == NULL) || (display->panel == NULL))
		return -EINVAL;

	return display->panel->vsync_switch_gpio_level;
}


/*GPIO SWITCH: 0-TP Vsync    1-TE Vsync*/
ssize_t oplus_set_vsync_switch(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct dsi_display *display = get_main_display();
	int ret = 0;
	int vsync_switch_gpio = 0;

	if (display == NULL) {
		DSI_ERR("%s error :NULL display", __func__);
		return -EINVAL;
	}

	sscanf(buf, "%du", &vsync_switch_gpio);

	printk(KERN_INFO "kVRR %s oplus_set_vsync_switch = %d\n", __func__, vsync_switch_gpio);

	ret = oplus_dsi_display_set_vsync_switch_gpio(display, vsync_switch_gpio);
	if (ret)
		pr_err("kVRR oplus_dsi_display_set_vsync_switch_gpio(%d) fail\n", vsync_switch_gpio);

	return count;
}

ssize_t oplus_get_vsync_switch(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	struct dsi_display *display = get_main_display();
	int vsync_switch_gpio = OPLUS_VSYNC_SWITCH_TE;

	if (display == NULL) {
		DSI_ERR("%s error :NULL display", __func__);
		return -EINVAL;
	}

	vsync_switch_gpio = oplus_dsi_display_get_vsync_switch_gpio(display);

	return sprintf(buf, "%d\n", vsync_switch_gpio);
}

void oplus_dsi_display_vsync_switch(void *disp, bool force_te_vsync)
{
	struct dsi_display *display = disp;
	int level = OPLUS_VSYNC_SWITCH_TE;
	int h_skew = SDC_ADFR;
	int rc = 0;

	if (!oplus_adfr_vsync_switch_is_enable()) {
		SDE_EVT32(0);
		return;
	}

	if ((display == NULL) || (display->panel == NULL) || (display->panel->cur_mode == NULL)) {
		DSI_ERR("kVRR Invalid params");
		return;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported()) {
		if (!strcmp(display->display_type, "secondary")) {
			DSI_INFO("kVRR [%s] return due to iris secondary panel no need config!\n", __func__);
			return;
		}
	}
#endif
	if (force_te_vsync == true) {
		if (oplus_adfr_get_vsync_mode() == OPLUS_EXTERNAL_TE_TP_VSYNC) {
			if (display->panel->vsync_switch_gpio_level == OPLUS_VSYNC_SWITCH_TP) {
				level = OPLUS_VSYNC_SWITCH_TE;
				oplus_dsi_display_vsync_switch_check_te(display, level);

				display->panel->force_te_vsync = true;
			}
		}
	} else {
		/* disable fake frame before vsync switch */
		oplus_adfr_fakeframe_status_update(display->panel, true);

		if (!strcmp(display->panel->oplus_priv.vendor_name, "AMB670YF07_CS")
				|| !strcmp(display->panel->oplus_priv.vendor_name, "AMB670YF07_FS")) {
			if (display->panel->cur_h_active == display->panel->cur_mode->timing.h_active) {
				DSI_INFO("KVRR the same resolution no need ADFR pre-switch!\n");
			} else {
				mutex_lock(&display->panel->panel_lock);
				rc = dsi_panel_tx_cmd_set(display->panel, DSI_CMD_ADFR_PRE_SWITCH);
				mutex_unlock(&display->panel->panel_lock);
			}
		} else {
			mutex_lock(&display->panel->panel_lock);
			rc = dsi_panel_tx_cmd_set(display->panel, DSI_CMD_ADFR_PRE_SWITCH);
			mutex_unlock(&display->panel->panel_lock);
		}

		if (rc)
			DSI_ERR("kVRR [%s] failed to send DSI_CMD_ADFR_PRE_SWITCH cmds rc=%d\n", display->panel, rc);

		if (oplus_adfr_get_vsync_mode() != OPLUS_EXTERNAL_TE_TP_VSYNC) {
			DSI_DEBUG("kVRR OPLUS_EXTERNAL_TE_TP_VSYNC is not supported\n");
			return;
		}

		h_skew = display->panel->cur_mode->timing.h_skew;

		if (h_skew == OPLUS_ADFR) {
			level = OPLUS_VSYNC_SWITCH_TE;
		} else {
			level = OPLUS_VSYNC_SWITCH_TP;
		}
		oplus_dsi_display_vsync_switch_check_te(display, level);
	}
}

/* add for vsync switch in resolution switch and aod scene */
void sde_encoder_adfr_vsync_switch(void *enc) {
	struct drm_encoder *drm_enc = enc;
	struct sde_encoder_virt *sde_enc = to_sde_encoder_virt(drm_enc);
	struct drm_connector *drm_conn;
	struct drm_bridge *temp_bridge;
	struct dsi_bridge *c_bridge;
	struct dsi_display *display;
	struct dsi_panel *panel;

	if ((drm_enc == NULL) || (sde_enc->cur_master == NULL) || (sde_enc->cur_master->connector == NULL)) {
		SDE_DEBUG("kVRR : invalid drm encoder parameters\n");
		return;
	}

	drm_conn = sde_enc->cur_master->connector;
	if ((drm_conn == NULL) || (drm_conn->encoder == NULL)) {
		SDE_ERROR("kVRR : invalid drm connector parameters\n");
		return;
	}

	if (drm_bridge_chain_get_first_bridge(drm_conn->encoder) == NULL) {
		SDE_ERROR("kVRR : invalid drm connector parameters\n");
		return;
	}

	temp_bridge = drm_bridge_chain_get_first_bridge(drm_conn->encoder);
	c_bridge = to_dsi_bridge(temp_bridge);
	display = c_bridge->display;

	if ((display == NULL) || (display->panel == NULL)) {
		SDE_ERROR("kVRR : invalid dsi display parameters\n");
		return;
	}
	panel = display->panel;

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported()) {
		if (!strcmp(panel->type, "secondary")) {
			pr_info("kVRR iris is secondary panel!\n");
			return;
		}
	}
#endif

	SDE_ATRACE_BEGIN("sde_encoder_adfr_vsync_switch");

	if (panel->need_vsync_switch) {
		/* wait for idle */
		sde_encoder_wait_for_event(drm_enc, MSM_ENC_TX_COMPLETE);
		/* after resolution switch and aod off , change back to tp vsync */
		/* if oplus_adfr_compatibility_mode is true, could not switch to tp vsync because hardware is not supported */
		if (oplus_adfr_compatibility_mode == false) {
			if (gpio_is_valid(panel->vsync_switch_gpio)) {
				gpio_set_value(panel->vsync_switch_gpio, 0);
				DSI_INFO("kVRR set vsync_switch_gpio to 0\n");
				panel->vsync_switch_gpio_level = OPLUS_VSYNC_SWITCH_TP;
			}
		}
		panel->need_vsync_switch = false;
		SDE_DEBUG("kVRR : vsync switch to %d\n", panel->vsync_switch_gpio_level);
		SDE_ATRACE_INT("vsync_switch_gpio_level", panel->vsync_switch_gpio_level);

		/* update fakeframe setting */
		oplus_adfr_fakeframe_status_update(panel, false);
	}

	SDE_ATRACE_END("sde_encoder_adfr_vsync_switch");
}

void sde_kms_adfr_vsync_switch(void *m_kms,
		void *d_crtc)
{
	struct msm_kms *kms = m_kms;
	struct drm_crtc *crtc = d_crtc;
	struct drm_encoder *encoder;
	struct drm_device *dev;

	if (!kms || !crtc || !crtc->state) {
		SDE_ERROR("kVRR invalid params\n");
		return;
	}

	dev = crtc->dev;

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head) {
		if (encoder->crtc != crtc)
			continue;
		sde_encoder_adfr_vsync_switch(encoder);
	}
}

/*
 if use TP when timing switch (resolution switch), tearing happen
 it seems like DDIC does not support MIPI offset writes after resolution switching
 TE is official, so do the TE switch after timing switch because MIPI will be reset after that
 if current use TE, do nothing
*/
void oplus_adfr_resolution_vsync_switch(void *dsi_panel)
{
	int rc = 0;
	struct dsi_panel *panel = dsi_panel;

	if ((panel == NULL) || (panel->cur_mode == NULL)) {
		DSI_ERR("kVRR Invalid params");
		return;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported()) {
		if (!strcmp(panel->type, "secondary")) {
			pr_info("kVRR iris is secondary panel!\n");
			return;
		}
	}
#endif

	/* just do switch when use tp vsync and resolution change */
	if ((panel->cur_h_active != panel->cur_mode->timing.h_active) && (panel->vsync_switch_gpio_level == OPLUS_VSYNC_SWITCH_TP)) {
		if (gpio_is_valid(panel->vsync_switch_gpio)) {
			rc = gpio_direction_output(panel->vsync_switch_gpio, 1);
			if (rc) {
				DSI_ERR("kVRR unable to set dir for vsync_switch_gpio gpio rc=%d\n", rc);
			} else {
				DSI_INFO("kVRR set vsync_switch_gpio to 1\n");
			}
			panel->vsync_switch_gpio_level = OPLUS_VSYNC_SWITCH_TE;
		}

		/* after one frame commit completed, change back to current mode vsync */
		panel->need_vsync_switch = true;
		SDE_ATRACE_INT("vsync_switch_gpio_level", panel->vsync_switch_gpio_level);
	}
	panel->cur_h_active = panel->cur_mode->timing.h_active;
	panel->cur_refresh_rate = panel->cur_mode->timing.refresh_rate;
}

/* vsync switch Entry and exit */
void oplus_adfr_aod_fod_vsync_switch(void *dsi_panel, bool force_te_vsync)
{
	struct dsi_panel *panel = dsi_panel;
	int h_skew = SDC_ADFR;
	int rc = 0;

	if (!oplus_adfr_vsync_switch_is_enable()) {
		SDE_EVT32(0);
		return;
	}

	if (panel == NULL) {
		DSI_ERR("kVRR Invalid params");
		return;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported()) {
		if (!strcmp(panel->type, "secondary")) {
			pr_info("kVRR iris is secondary panel!\n");
			return;
		}
	}
#endif

	/* force switch to te vsync as tp vsync will change in aod and fod mode */
	if (force_te_vsync == true) {
		if (panel->vsync_switch_gpio_level == OPLUS_VSYNC_SWITCH_TP) {
			if (gpio_is_valid(panel->vsync_switch_gpio)) {
				rc = gpio_direction_output(panel->vsync_switch_gpio, 1);
				if (rc) {
					DSI_ERR("kVRR unable to set dir for vsync_switch_gpio gpio rc=%d\n", rc);
				} else {
					DSI_INFO("kVRR set vsync_switch_gpio to 1\n");
				}
				panel->vsync_switch_gpio_level = OPLUS_VSYNC_SWITCH_TE;
				panel->force_te_vsync = true;
				SDE_ATRACE_INT("vsync_switch_gpio_level", panel->vsync_switch_gpio_level);
			}
		}
	} else {
		/* change back to tp vysnc since aod/fod mode is off */
		if ((panel->force_te_vsync == true) && (panel->vsync_switch_gpio_level == OPLUS_VSYNC_SWITCH_TE)) {
			h_skew = panel->cur_mode->timing.h_skew;
			/* maybe change to OA in aod/fod mode */
			if (h_skew == SDC_ADFR || h_skew == SDC_MFR || h_skew == OPLUS_MFR) {
				panel->need_vsync_switch = true;
				DSI_INFO("kVRR set need_vsync_switch to true\n");
			}
			panel->force_te_vsync = false;
		}
	}
}

/* Add for vsync switch status reset */
/* switch to tp vsync since panel is no longer in aod mode after power on */
void oplus_adfr_vsync_switch_reset(void *dsi_panel)
{
	struct dsi_panel *panel = dsi_panel;
	u32 h_skew = SDC_ADFR;

	if ((panel == NULL) || (panel->cur_mode == NULL)) {
		DSI_ERR("kVRR Invalid params");
		return;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported()) {
		if (!strcmp(panel->type, "secondary")) {
			pr_info("kVRR iris is secondary panel!\n");
			return;
		}
	}
#endif

	h_skew = panel->cur_mode->timing.h_skew;

	if (oplus_adfr_get_vsync_mode() == OPLUS_EXTERNAL_TE_TP_VSYNC) {
		/* reset to tp vsync after power on */
		if (panel->panel_initialized == false) {
			if (panel->force_te_vsync == true) {
				/* maybe change to OA in aod/fod mode */
				if (h_skew == SDC_ADFR || h_skew == SDC_MFR || h_skew == OPLUS_MFR) {
					/* could not change vsync gpio if the machine is incompatible with adfr */
					if (oplus_adfr_compatibility_mode == false) {
						if (gpio_is_valid(panel->vsync_switch_gpio)) {
							gpio_set_value(panel->vsync_switch_gpio, 0);
							DSI_INFO("kVRR set vsync_switch_gpio to 0\n");
							panel->vsync_switch_gpio_level = OPLUS_VSYNC_SWITCH_TP;
						}
					}
				}
				panel->force_te_vsync = false;
				SDE_ATRACE_INT("vsync_switch_gpio_level", panel->vsync_switch_gpio_level);
			}
		}
	}
}

/* ---------- te source switch --------- */
int oplus_adfr_get_vsync_source(void *dsi_panel) {
	struct dsi_panel *panel = dsi_panel;
	u32 h_skew = SDC_ADFR;

	if (!panel) {
		DSI_ERR("invalid panel params\n");
		return -EINVAL;
	}

	h_skew = panel->cur_mode->timing.h_skew;
	DSI_INFO("kVRR %s h_skew = %d", __func__, h_skew);

	if (panel->power_mode == SDE_MODE_DPMS_LP1 ||
			panel->power_mode == SDE_MODE_DPMS_LP2) {
		return OPLUS_TE_SOURCE_TE;
	}

	if ((h_skew == OPLUS_ADFR) || (h_skew == OPLUS_MFR)) {
		return OPLUS_TE_SOURCE_TE;
	} else if ((h_skew == SDC_ADFR) || (h_skew == SDC_MFR)) {
		return OPLUS_TE_SOURCE_TP;
	} else {
		DSI_ERR("kVRR %s error value", __func__);
		return -1;
	}
}

int oplus_adfr_vsync_source_switch(void *dsi_panel, u8 v_source) {
	struct dsi_panel *panel = dsi_panel;
	struct dsi_display *d_display = NULL;
	struct drm_encoder *drm_enc = NULL;
	struct sde_encoder_virt *sde_enc = NULL;

	if (!panel) {
		DSI_ERR("invalid panel params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported()) {
		if (!strcmp(panel->type, "secondary")) {
			pr_info("kVRR iris is secondary panel!\n");
			return -EINVAL;
		}
	}
#endif

	d_display = to_dsi_display(panel->host);
	if (!d_display) {
		DSI_ERR("invalid display params\n");
		return -EINVAL;
	}

	drm_enc = d_display->bridge->base.encoder;
	if (!drm_enc) {
		DSI_ERR("invalid encoder params\n");
		return -EINVAL;
	}

	sde_enc = to_sde_encoder_virt(drm_enc);
	if (sde_enc == NULL) {
		SDE_ERROR("invalid pointer sde_enc");
		return -EFAULT;
	}

	if (sde_enc->cur_master == NULL) {
		SDE_ERROR("invalid pointer cur_master, vsync switch failed, need switch again");
		panel->need_te_source_switch = true;
		return -EFAULT;
	}

	SDE_ATRACE_BEGIN("oplus_adfr_vsync_source_switch");

	sde_enc->te_source = v_source;
	sde_encoder_helper_switch_vsync(drm_enc, false);
	SDE_ATRACE_INT("te_source", sde_enc->te_source);

	SDE_ATRACE_END("oplus_adfr_vsync_source_switch");

	return 0;
}

int oplus_adfr_vsync_source_reset(void *enc) {
	struct drm_encoder *drm_enc = enc;
	struct sde_encoder_virt *sde_enc = NULL;
	struct drm_connector *drm_conn;
	struct sde_connector *sde_conn;
	struct drm_bridge *temp_bridge;
	struct dsi_bridge *c_bridge;
	struct dsi_display *display;
	struct dsi_panel *panel;
	u8 v_source;

	if (drm_enc == NULL) {
		SDE_ERROR("kVRR : invalid drm encoder parameters\n");
		return 0;
	}

	sde_enc = to_sde_encoder_virt(drm_enc);
	if ((sde_enc->cur_master == NULL) || (sde_enc->cur_master->connector == NULL)) {
		SDE_ERROR("kVRR : invalid sde encoder parameters\n");
		return 0;
	}

	drm_conn = sde_enc->cur_master->connector;
	if ((drm_conn == NULL) || (drm_conn->encoder == NULL)) {
		SDE_ERROR("kVRR : invalid drm connector parameters\n");
		return 0;
	}

	sde_conn = to_sde_connector(drm_conn);
	if (sde_conn == NULL) {
		SDE_ERROR("kVRR : invalid sde connector parameters\n");
		return 0;
	}
	if (sde_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		SDE_DEBUG("kVRR : Only reset when display is dsi_display\n");
		return 0;
	}

	if (drm_bridge_chain_get_first_bridge(drm_conn->encoder) == NULL) {
		SDE_ERROR("kVRR : invalid drm bridge parameters\n");
		return 0;
	}

	temp_bridge = drm_bridge_chain_get_first_bridge(drm_conn->encoder);
	c_bridge = to_dsi_bridge(temp_bridge);
	display = c_bridge->display;

	if ((display == NULL) || (display->panel == NULL)) {
		SDE_ERROR("kVRR : invalid dsi display parameters\n");
		return 0;
	}
	panel = display->panel;

	if (panel->is_switching) {
		panel->is_switching = false;
		SDE_INFO("kVRR Don't reset TE when timing switch");
		return 0;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported()) {
		if (!strcmp(panel->type, "secondary")) {
			pr_info("kVRR iris is secondary panel!\n");
			return 0;
		}
	}
#endif

	if (panel->power_mode == SDE_MODE_DPMS_LP1 ||
			panel->power_mode == SDE_MODE_DPMS_LP2) {
		SDE_INFO("kVRR Don't reset TE in AOD mode");
		return 0;
	}

	SDE_ATRACE_BEGIN("oplus_adfr_vsync_source_reset");

	v_source = oplus_adfr_get_vsync_source(panel);
	SDE_INFO("kVRR : vsync source switched to %d when resume\n", v_source);
	panel->need_te_source_switch = false;
	oplus_adfr_vsync_source_switch(panel, v_source);

	SDE_ATRACE_END("oplus_adfr_vsync_source_reset");

	return 0;
}

int oplus_adfr_timing_vsync_source_switch(void *dsi_panel) {
	struct dsi_panel *panel = dsi_panel;
	struct dsi_display *d_display = NULL;
	struct drm_encoder *drm_enc = NULL;
	struct sde_encoder_virt *sde_enc = NULL;
	u8 v_source;

	if (!panel) {
		DSI_ERR("invalid panel params\n");
		return -EINVAL;
	}

	d_display = to_dsi_display(panel->host);
	if (!d_display) {
		DSI_ERR("invalid display params\n");
		return -EINVAL;
	}

	if (panel->power_mode == SDE_MODE_DPMS_LP1 ||
			panel->power_mode == SDE_MODE_DPMS_LP2) {
		return 0;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported()) {
		if (!strcmp(panel->type, "secondary")) {
			pr_info("kVRR iris sencodary!\n");
			return 0;
		}
	}
#endif

	drm_enc = d_display->bridge->base.encoder;
	if (!drm_enc) {
		DSI_ERR("invalid encoder params\n");
		return -EINVAL;
	}

	sde_enc = to_sde_encoder_virt(drm_enc);
	if (sde_enc == NULL) {
		SDE_ERROR("invalid pointer sde_enc");
		return -EFAULT;
	}

	SDE_ATRACE_BEGIN("oplus_adfr_timing_vsync_source_switch");

	if (panel->cur_h_active != panel->cur_mode->timing.h_active) {
		if (!strcmp(panel->oplus_priv.vendor_name, "S6E3HC4"))
			goto exit;
		v_source = OPLUS_TE_SOURCE_TE;
		SDE_INFO("kVRR : vsync source switched to %d before resolution switch\n", v_source);
		oplus_adfr_vsync_source_switch(panel, v_source);
		panel->need_te_source_switch = true;
		SDE_INFO("kVRR : need to switch to normal after resolution switch\n");
	} else {
		v_source = oplus_adfr_get_vsync_source(panel);
		if (sde_enc->te_source != v_source) {
			SDE_INFO("kVRR : vsync source switched to %d before game mode switch\n", v_source);
			oplus_adfr_vsync_source_switch(panel, v_source);
		}
	}

exit:
	SDE_ATRACE_END("oplus_adfr_timing_vsync_source_switch");
	return 0;
}

void sde_encoder_adfr_vsync_source_switch(void *enc) {
	struct drm_encoder *drm_enc = enc;
	struct sde_encoder_virt *sde_enc = to_sde_encoder_virt(drm_enc);
	struct drm_connector *drm_conn;
	struct sde_connector *sde_conn;
	struct drm_bridge *temp_bridge;
	struct dsi_bridge *c_bridge;
	struct dsi_display *display;
	struct dsi_panel *panel;
	u8 v_source;

	if ((drm_enc == NULL) || (sde_enc->cur_master == NULL) || (sde_enc->cur_master->connector == NULL)) {
		SDE_INFO("kVRR : invalid drm encoder parameters\n");
		return;
	}

	drm_conn = sde_enc->cur_master->connector;
	if ((drm_conn == NULL) || (drm_conn->encoder == NULL)) {
		SDE_ERROR("kVRR : invalid drm connector parameters\n");
		return;
	}

	sde_conn = to_sde_connector(drm_conn);
	if (sde_conn == NULL) {
		SDE_ERROR("kVRR : invalid sde connector parameters\n");
		return;
	}
	if (sde_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		SDE_DEBUG("kVRR : Only switch when display is dsi_display\n");
		return;
	}

	if (drm_bridge_chain_get_first_bridge(drm_conn->encoder) == NULL) {
		SDE_ERROR("kVRR : invalid drm connector parameters\n");
		return;
	}

	temp_bridge = drm_bridge_chain_get_first_bridge(drm_conn->encoder);
	c_bridge = to_dsi_bridge(temp_bridge);
	display = c_bridge->display;

	if ((display == NULL) || (display->panel == NULL)) {
		SDE_ERROR("kVRR : invalid dsi display parameters\n");
		return;
	}
	panel = display->panel;

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported()) {
		if (!strcmp(panel->type, "secondary")) {
			pr_info("kVRR iris is secondary panel!\n");
			return;
		}
	}
#endif

	SDE_ATRACE_BEGIN("sde_encoder_adfr_vsync_source_switch");

	if (panel->power_mode == SDE_MODE_DPMS_LP1 ||
			panel->power_mode == SDE_MODE_DPMS_LP2) {
		panel->need_te_source_switch = false;
	}

	if (panel->need_te_source_switch) {
		/* wait for idle */
		v_source = oplus_adfr_get_vsync_source(panel);
		sde_encoder_wait_for_event(drm_enc, MSM_ENC_TX_COMPLETE);
		SDE_INFO("kVRR : vsync source switched to %d when needed\n", v_source);
		panel->need_te_source_switch = false;
		oplus_adfr_vsync_source_switch(panel, v_source);

		/* update fakeframe setting after resolution switch */
		oplus_adfr_fakeframe_status_update(panel, false);
	}

	SDE_ATRACE_INT("te_source", sde_enc->te_source);
	SDE_ATRACE_END("sde_encoder_adfr_vsync_source_switch");
}

/* double TE */
void sde_kms_adfr_vsync_source_switch(void *m_kms,
		void *d_crtc)
{
	struct msm_kms *kms = m_kms;
	struct drm_crtc *crtc = d_crtc;
	struct drm_encoder *encoder;
	struct drm_device *dev;

	if (!kms || !crtc || !crtc->state) {
		SDE_ERROR("kVRR invalid params\n");
		return;
	}

	dev = crtc->dev;

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head) {
		if (encoder->crtc != crtc)
			continue;
		sde_encoder_adfr_vsync_source_switch(encoder);
	}
}

/*return value*/
/*1: switch to panel TE*/
/*0: don't need to switch to panel TE*/
int dsi_panel_aod_need_vsync_source_switch(void *dsi_panel)
{
	struct dsi_panel *panel = dsi_panel;
	u32 h_skew = SDC_ADFR;

	if ((panel == NULL) || (panel->cur_mode == NULL)) {
		DSI_ERR("kVRR Invalid params");
		return 0;
	}

	h_skew = panel->cur_mode->timing.h_skew;
	DSI_INFO("kVRR %s h_skew = %d", __func__, h_skew);

	if ((h_skew == OPLUS_ADFR) || (h_skew == OPLUS_MFR)) {
		return 0;
	} else if ((h_skew == SDC_ADFR) || (h_skew == SDC_MFR)) {
		return 1;
	} else {
		DSI_ERR("kVRR %s error value", __func__);
		return 0;
	}
}

/* te_source:                    */
/* OPLUS_TE_SOURCE_TP = 0,  TE0  */
/* OPLUS_TE_SOURCE_TE = 1,  TE1  */
void sde_encoder_adfr_aod_fod_source_switch(void *dsi_display, int te_source) {
	struct drm_encoder *drm_enc = NULL;
	struct sde_encoder_virt *sde_enc = NULL;
	struct dsi_display *d_display = dsi_display;

	if (!d_display || !d_display->bridge) {
		SDE_ERROR("kVRR error: %s NULL Pointer\n", __func__);
		return;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported()) {
		if (!strcmp(d_display->display_type, "secondary")) {
			pr_info("kVRR iris is secondary panel!\n");
			return;
		}
	}
#endif

	drm_enc = d_display->bridge->base.encoder;
	sde_enc = to_sde_encoder_virt(drm_enc);
	SDE_INFO("kVRR : [from %d change to %d] in fod aod scenes\n", sde_enc->te_source, te_source);
	if(!dsi_panel_aod_need_vsync_source_switch(d_display->panel)) {
		SDE_INFO("kVRR : don't need to change te\n");
		return;
	}

	SDE_ATRACE_BEGIN("sde_encoder_adfr_aod_fod_source_switch");
	if (te_source != sde_enc->te_source) {
		sde_enc->te_source = te_source;
		if (!d_display->panel->panel_initialized) {
			/* no need to wait for idle for panel not initialized*/
			SDE_INFO("kVRR : vsync source switched to %d before panel initialized\n", sde_enc->te_source);
			sde_encoder_helper_switch_vsync(drm_enc, false);
			SDE_ATRACE_INT("aod_te_source", sde_enc->te_source);
		} else {
			/* wait for idle */
			sde_encoder_wait_for_event(drm_enc, MSM_ENC_TX_COMPLETE);
			SDE_INFO("kVRR : vsync source switched to %d after wait for idle\n", sde_enc->te_source);
			sde_encoder_helper_switch_vsync(drm_enc, false);
			SDE_ATRACE_INT("aod_te_source", sde_enc->te_source);
		}
	}
	SDE_ATRACE_END("sde_encoder_adfr_aod_fod_source_switch");
}

/* --------------- auto mode ---------------*/
/* Add for auto on cmd filter */
/* if auto on command is to be sent within the same frame, filter it out */
bool oplus_adfr_auto_on_cmd_filter_set(bool enable)
{
	oplus_adfr_need_filter_auto_on_cmd = enable;
	return oplus_adfr_need_filter_auto_on_cmd;
}

bool oplus_adfr_auto_on_cmd_filter_get(void)
{
	return oplus_adfr_need_filter_auto_on_cmd;
}

bool oplus_adfr_has_auto_mode(u32 value)
{
	return (value & OPLUS_ADFR_AUTO_MAGIC);
}

int oplus_adfr_handle_auto_mode(u32 propval)
{
	int handled = 0;

	DSI_DEBUG("kVRR update auto mode %u 0x%08X \n", propval, propval);

	SDE_ATRACE_BEGIN("oplus_adfr_handle_auto_mode");

	if (!(propval & OPLUS_ADFR_AUTO_MAGIC)) {
		DSI_INFO("kVRR update auto mode skip, without auto magic %08X \n", propval);
		SDE_ATRACE_INT("auto_handled", handled);
		SDE_ATRACE_END("oplus_adfr_handle_auto_mode");
		return handled;
	}

	handled = 1;
	oplus_adfr_auto_update_counter += 1;
	DSI_DEBUG("kVRR auto update counter %llu\n", oplus_adfr_auto_update_counter);

	if (propval & OPLUS_ADFR_AUTO_MODE_MAGIC) {
		/* Add for auto on cmd filter */
		if (oplus_adfr_auto_on_cmd_filter_get() && (OPLUS_ADFR_AUTO_MODE_VALUE(propval) == OPLUS_ADFR_AUTO_ON)) {
			DSI_INFO("kVRR auto off and auto on cmd are sent on the same frame, filter it\n");
			handled |= (1<<1);
			SDE_ATRACE_INT("auto_handled", handled);
			SDE_ATRACE_END("oplus_adfr_handle_auto_mode");
			return handled;
		} else if (OPLUS_ADFR_AUTO_MODE_VALUE(propval) == OPLUS_ADFR_AUTO_IDLE) {
			/* if auto mode = 2, min fps is the sw fps */
			if (propval & OPLUS_ADFR_AUTO_MIN_FPS_MAGIC) {
				oplus_adfr_auto_sw_fps = OPLUS_ADFR_AUTO_MIN_FPS_VALUE(propval);
				DSI_DEBUG("kVRR sw fps %d\n", oplus_adfr_auto_sw_fps);
				SDE_ATRACE_INT("oplus_adfr_auto_sw_fps", oplus_adfr_auto_sw_fps);
				/* fakeframe need to be updated */
				oplus_adfr_auto_fakeframe_updated = true;
			} else {
				DSI_ERR("kVRR update sw fps skip, without auto min fps magic %08X\n", propval);
			}
			handled |= (1<<2);
			SDE_ATRACE_INT("auto_handled", handled);
			SDE_ATRACE_END("oplus_adfr_handle_auto_mode");
			return handled;
		} else if (OPLUS_ADFR_AUTO_MODE_VALUE(propval) != oplus_adfr_auto_mode) {
			oplus_adfr_auto_mode_updated = true;
			/* filter repeat auto mode setting */
			/* when auto mode changes, write the corresponding min fps again */
			oplus_adfr_auto_min_fps_updated = true;
			oplus_adfr_auto_mode = OPLUS_ADFR_AUTO_MODE_VALUE(propval);
			handled |= (1<<3);
		}
	}

	if (propval & OPLUS_ADFR_AUTO_FAKEFRAME_MAGIC) {
		if (OPLUS_ADFR_AUTO_FAKEFRAME_VALUE(propval) != oplus_adfr_auto_fakeframe) {
			/* no need to get fakeframe value
			oplus_adfr_auto_fakeframe_updated = true;
			oplus_adfr_auto_fakeframe = OPLUS_ADFR_AUTO_FAKEFRAME_VALUE(propval);
			*/
			handled |= (1<<4);
		}
	}

	if (propval & OPLUS_ADFR_AUTO_MIN_FPS_MAGIC) {
		if (OPLUS_ADFR_AUTO_MIN_FPS_VALUE(propval) != oplus_adfr_auto_min_fps) {
			oplus_adfr_auto_min_fps_updated = true;
			oplus_adfr_auto_min_fps = OPLUS_ADFR_AUTO_MIN_FPS_VALUE(propval);
			handled |= (1<<5);
		}
	}

	if (handled == 1) {
		DSI_WARN("kVRR update auto mode nothing, unknown or repetitive value %08X\n", propval);
	}

	SDE_ATRACE_INT("auto_handled", handled);
	SDE_ATRACE_INT("oplus_adfr_auto_mode", oplus_adfr_auto_mode);
	SDE_ATRACE_INT("oplus_adfr_auto_fakeframe", oplus_adfr_auto_fakeframe);
	SDE_ATRACE_INT("oplus_adfr_auto_min_fps", oplus_adfr_auto_min_fps);
	SDE_ATRACE_END("oplus_adfr_handle_auto_mode");

	/* latest setting */
	DSI_INFO("kVRR auto mode %d[%d], fakeframe %d[%d], min fps %d[%d], handled 0x%02x\n",
		oplus_adfr_auto_mode, oplus_adfr_auto_mode_updated,
		oplus_adfr_auto_fakeframe, oplus_adfr_auto_fakeframe_updated,
		oplus_adfr_auto_min_fps, oplus_adfr_auto_min_fps_updated, handled);

	return handled;
}

static int dsi_panel_send_auto_on_dcs(struct dsi_panel *panel,
		int ctrl_idx)
{
	int rc = 0;

	/* SDC's auto, fakeframe and minfps are available only after power on */

	if (!panel) {
		DSI_ERR("kVRR invalid params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported()) {
		if (!strcmp(panel->type, "secondary")) {
			pr_info("kVRR iris is secondary panel!\n");
			return 0;
		}
	}
#endif

	/* if (get_oplus_display_power_status() != OPLUS_DISPLAY_POWER_ON) { */
	if (panel->power_mode != SDE_MODE_DPMS_ON) {
		DSI_INFO("kVRR ignore %s when power is %d", __FUNCTION__, panel->power_mode);
		return 0;
	}

	mutex_lock(&panel->panel_lock);

	DSI_INFO("kVRR ctrl:%d auto on\n", ctrl_idx);
	SDE_ATRACE_INT("oplus_adfr_auto_mode_cmd", OPLUS_ADFR_AUTO_ON);
	rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SET_QSYNC_ON);
	if (rc)
		DSI_ERR("kVRR [%s] failed to send DSI_CMD_SET_AUTO_ON cmds rc=%d\n",
				panel->name, rc);

	mutex_unlock(&panel->panel_lock);
	return rc;
}

static int dsi_panel_send_auto_off_dcs(struct dsi_panel *panel,
		int ctrl_idx)
{
	int rc = 0;
	/* SDC's auto, fakeframe and minfps are available only after power on */

	if (!panel) {
		DSI_ERR("kVRR invalid params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported()) {
		if (!strcmp(panel->type, "secondary")) {
			pr_info("kVRR iris is secondary panel!\n");
			return 0;
		}
	}
#endif

	/* if (get_oplus_display_power_status() != OPLUS_DISPLAY_POWER_ON) { */
	if (panel->power_mode != SDE_MODE_DPMS_ON) {
		DSI_INFO("kVRR ignore %s when power is %d", __FUNCTION__, panel->power_mode);
		return 0;
	}

	mutex_lock(&panel->panel_lock);

	DSI_INFO("kVRR ctrl:%d auto off\n", ctrl_idx);
	SDE_ATRACE_INT("oplus_adfr_auto_mode_cmd", OPLUS_ADFR_AUTO_OFF);
	rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SET_QSYNC_OFF);
	if (rc) {
		DSI_ERR("kVRR [%s] failed to send DSI_CMD_SET_AUTO_OFF cmds rc=%d\n",
				panel->name, rc);
	} else {
		/* after auto off cmd was sent, auto on cmd filter start */
		oplus_adfr_auto_on_cmd_filter_set(true);
	}

	mutex_unlock(&panel->panel_lock);
	return rc;
}

static int dsi_display_auto_mode_enable(struct dsi_display *display, bool enable)
{
	int i;
	int rc = 0;

	mutex_lock(&display->display_lock);

	display_for_each_ctrl(i, display) {
		if (enable) {
			/* send the commands to enable auto mode */
			rc = dsi_panel_send_auto_on_dcs(display->panel, i);
			if (rc) {
				DSI_ERR("kVRR fail auto ON cmds rc:%d\n", rc);
				goto exit;
			}
		} else {
			/* send the commands to disbale auto mode */
			rc = dsi_panel_send_auto_off_dcs(display->panel, i);
			if (rc) {
				DSI_ERR("kVRR fail auto OFF cmds rc:%d\n", rc);
				goto exit;
			}
		}
	}

exit:
	SDE_EVT32(enable, rc);
	mutex_unlock(&display->display_lock);
	return rc;
}

/* prevent the wrong min fps setting */
static int dsi_panel_auto_minfps_check(struct dsi_panel *panel, u32 extend_frame)
{
	int h_skew = panel->cur_mode->timing.h_skew;
	int refresh_rate = panel->cur_mode->timing.refresh_rate;

	if (h_skew == SDC_ADFR) {
		if (oplus_adfr_auto_mode == OPLUS_ADFR_AUTO_OFF) {
			if (refresh_rate == 120) {
				if ((extend_frame < OPLUS_ADFR_AUTO_MIN_FPS_MAX) || (extend_frame > OPLUS_ADFR_AUTO_MIN_FPS_1HZ)) {
					/* The highest frame rate is the most stable */
					extend_frame = OPLUS_ADFR_AUTO_MIN_FPS_MAX;
				} else if ((oplus_adfr_idle_mode == OPLUS_ADFR_IDLE_OFF) && (extend_frame > OPLUS_ADFR_AUTO_MIN_FPS_20HZ)
					&& (extend_frame <= OPLUS_ADFR_AUTO_MIN_FPS_1HZ)) {
					/* force to 20hz if the min fps is less than 20hz when auto mode is off and idle mode is also off */
					extend_frame = OPLUS_ADFR_AUTO_MIN_FPS_20HZ;
				}
			} else if (refresh_rate == 90) {
				/* locked in 90hz */
				extend_frame = OPLUS_ADFR_AUTO_MIN_FPS_MAX + 9;
			}
		} else {
			if (refresh_rate == 120) {
				if ((extend_frame < OPLUS_ADFR_AUTO_MIN_FPS_MAX) || (extend_frame > OPLUS_ADFR_AUTO_MIN_FPS_1HZ)) {
					extend_frame = OPLUS_ADFR_AUTO_MIN_FPS_MAX;
				}
			} else if (refresh_rate == 90) {
				extend_frame = OPLUS_ADFR_AUTO_MIN_FPS_MAX;
			}
		}
	} else if (h_skew == SDC_MFR) {
		if ((extend_frame < OPLUS_ADFR_AUTO_MIN_FPS_60HZ) || (extend_frame > OPLUS_ADFR_AUTO_MIN_FPS_1HZ)) {
			extend_frame = OPLUS_ADFR_AUTO_MIN_FPS_60HZ;
		} else if ((oplus_adfr_idle_mode == OPLUS_ADFR_IDLE_OFF) && (extend_frame > OPLUS_ADFR_AUTO_MIN_FPS_20HZ) && (extend_frame <= OPLUS_ADFR_AUTO_MIN_FPS_1HZ)) {
			extend_frame = OPLUS_ADFR_AUTO_MIN_FPS_20HZ;
		}
	}

	return extend_frame;
}

static int dsi_panel_send_auto_minfps_dcs(struct dsi_panel *panel,
		int ctrl_idx, u32 extend_frame)
{
	int rc = 0;
	struct dsi_display_mode *mode;
	struct dsi_cmd_desc *cmds;
	size_t tx_len;
	u8 *tx_buf;
	u32 count;
	int k = 0;
	u8 cmd_high = 0;
	u8 cmd_low = 0;

	/* SDC's auto, fakeframe and minfps are available only after power on */

	if (!panel || !panel->cur_mode || !panel->oplus_priv.vendor_name) {
		DSI_ERR("kVRR invalid params\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported()) {
		if (!strcmp(panel->type, "secondary")) {
			pr_info("kVRR iris secondary disable send auto minfps!\n");
			return 0;
		}
	}
#endif

	/* if (get_oplus_display_power_status() != OPLUS_DISPLAY_POWER_ON) { */
	if (panel->power_mode != SDE_MODE_DPMS_ON) {
		DSI_INFO("kVRR ignore %s %u when power is %d", __FUNCTION__, extend_frame, panel->power_mode);
		return 0;
	}

	mutex_lock(&panel->panel_lock);

	mode = panel->cur_mode;

	/*check minfps*/
	extend_frame = dsi_panel_auto_minfps_check(panel, extend_frame);

	/* pixelworks X7 */
#if defined(CONFIG_PXLW_IRIS)
	iris_current_extend_frame = extend_frame;
#endif
	if (!strcmp(panel->oplus_priv.vendor_name, "S6E3HC4")) {
		if (!strcmp(panel->name, "samsung s6e3hc4 amb682cg01 dsc cmd mode panel")) {
			cmd_high = extend_frame*6/256;
			cmd_low = extend_frame*6%256;
		} else {
			cmd_high = extend_frame*3/256;
			cmd_low = extend_frame*3%256;
		}
	} else {
		cmd_low = extend_frame;
	}
	LCD_DEBUG_COMMON("kVRR extend_frame = %d, cmd:[0x%02X 0x%02X]\n",
			extend_frame, cmd_high, cmd_low);

	/*update the sdc min fps cmds*/
	if (oplus_adfr_auto_mode == OPLUS_ADFR_AUTO_OFF) {
		cmds = mode->priv_info->cmd_sets[DSI_CMD_QSYNC_MIN_FPS_0].cmds;
		count = mode->priv_info->cmd_sets[DSI_CMD_QSYNC_MIN_FPS_0].count;

		if (count == 0) {
			DSI_INFO("kVRR [%s] No commands to be sent for manual min fps.\n", panel->name);
			goto exit;
		}

		if (count <= SDC_MANUAL_MIN_FPS_CMD_OFFSET) {
			DSI_ERR("kVRR [%s] No commands to be sent for manual min fps, wrong cmds count.\n", panel->name);
			goto exit;
		}

		/*update manual min fps*/
		tx_len = cmds[SDC_MANUAL_MIN_FPS_CMD_OFFSET].msg.tx_len;
		tx_buf = (u8 *)cmds[SDC_MANUAL_MIN_FPS_CMD_OFFSET].msg.tx_buf;
		if (tx_len != SDC_MIN_FPS_CMD_SIZE) {
			DSI_ERR("kVRR [%s] No commands to be sent for manual min fps, wrong cmds size %u.\n", panel->name, tx_len);
			goto exit;
		}

		tx_buf[SDC_MIN_FPS_CMD_SIZE-1] = cmd_low;
		DSI_DEBUG("kVRR send manual min fps %u .\n", extend_frame);
		for (k = 0; k < tx_len; k++) {
			DSI_DEBUG("kVRR manual min fps %02x", tx_buf[k]);
		}

		if (!strcmp(panel->oplus_priv.vendor_name, "S6E3HC4")) {
			if (count <= SDC_MANUAL_MIN_FPS_CMD_HIGH_OFFSET) {
				DSI_ERR("kVRR [%s] No commands to be sent for manual min fps cmd_high, wrong cmds count.\n", panel->name);
				goto exit;
			}
			/*update manual min fps*/
			tx_len = cmds[SDC_MANUAL_MIN_FPS_CMD_HIGH_OFFSET].msg.tx_len;
			tx_buf = (u8 *)cmds[SDC_MANUAL_MIN_FPS_CMD_HIGH_OFFSET].msg.tx_buf;
			if (tx_len != SDC_MIN_FPS_CMD_SIZE) {
				DSI_ERR("kVRR [%s] No commands to be sent for manual min fps cmd_high, wrong cmds size %u.\n", panel->name, tx_len);
				goto exit;
			}

			tx_buf[SDC_MIN_FPS_CMD_SIZE-1] = cmd_high;
		}

		DSI_DEBUG("kVRR ctrl:%d manual min fps\n", ctrl_idx);
		SDE_ATRACE_INT("oplus_adfr_auto_min_fps_cmd", extend_frame);
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_QSYNC_MIN_FPS_0);
		if (rc)
			DSI_ERR("kVRR [%s] failed to send DSI_CMD_QSYNC_MIN_FPS_0 cmds rc=%d\n",
				panel->name, rc);
	} else {
		cmds = mode->priv_info->cmd_sets[DSI_CMD_QSYNC_MIN_FPS_1].cmds;
		count = mode->priv_info->cmd_sets[DSI_CMD_QSYNC_MIN_FPS_1].count;

		if (count == 0) {
			DSI_DEBUG("kVRR [%s] No commands to be sent for auto min fps.\n", panel->name);
			goto exit;
		}

		if (count <= SDC_AUTO_MIN_FPS_CMD_OFFSET) {
			DSI_ERR("kVRR [%s] No commands to be sent for auto min fps, wrong cmds count.\n", panel->name);
			goto exit;
		}
		/*update auto min fps*/
		tx_len = cmds[SDC_AUTO_MIN_FPS_CMD_OFFSET].msg.tx_len;
		tx_buf = (u8 *)cmds[SDC_AUTO_MIN_FPS_CMD_OFFSET].msg.tx_buf;
		if (tx_len != SDC_MIN_FPS_CMD_SIZE) {
			DSI_ERR("kVRR [%s] No commands to be sent for auto min fps, wrong cmds size %u.\n", panel->name, tx_len);
			goto exit;
		}

		tx_buf[SDC_MIN_FPS_CMD_SIZE-1] = cmd_low;
		DSI_INFO("kVRR send auto min fps %u .\n", extend_frame);
		for (k = 0; k < tx_len; k++) {
			DSI_DEBUG("kVRR auto min fps %02x", tx_buf[k]);
		}

		if (!strcmp(panel->oplus_priv.vendor_name, "S6E3HC4")) {
			if (count <= SDC_AUTO_MIN_FPS_CMD_HIGH_OFFSET) {
				DSI_ERR("kVRR [%s] No commands to be sent for auto min fps cmd_high, wrong cmds count.\n", panel->name);
				goto exit;
			}
			/*update auto min fps*/
			tx_len = cmds[SDC_AUTO_MIN_FPS_CMD_HIGH_OFFSET].msg.tx_len;
			tx_buf = (u8 *)cmds[SDC_AUTO_MIN_FPS_CMD_HIGH_OFFSET].msg.tx_buf;
			if (tx_len != SDC_MIN_FPS_CMD_SIZE) {
				DSI_ERR("kVRR [%s] No commands to be sent for auto min fps cmd_high, wrong cmds size %u.\n", panel->name, tx_len);
				goto exit;
			}

			tx_buf[SDC_MIN_FPS_CMD_SIZE-1] = cmd_high;
		}

		DSI_DEBUG("kVRR ctrl:%d auto min fps\n", ctrl_idx);
		SDE_ATRACE_INT("oplus_adfr_auto_min_fps_cmd", extend_frame);
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_QSYNC_MIN_FPS_1);
		if (rc)
			DSI_ERR("kVRR [%s] failed to send DSI_CMD_QSYNC_MIN_FPS_1 cmds rc=%d\n",
				panel->name, rc);
	}

exit:
	SDE_EVT32(extend_frame, rc);
	mutex_unlock(&panel->panel_lock);
	return rc;
}

static int dsi_display_auto_mode_min_fps(struct dsi_display *display, u32 extend_frame)
{
	int i;
	int rc = 0;

	mutex_lock(&display->display_lock);

	display_for_each_ctrl(i, display) {
		/* send the commands to set auto mode min fps */
		rc = dsi_panel_send_auto_minfps_dcs(display->panel, i, extend_frame);
		if (rc) {
			DSI_ERR("kVRR fail auto Min Fps cmds rc:%d\n", rc);
			goto exit;
		}
	}

exit:
	SDE_EVT32(extend_frame, rc);
	mutex_unlock(&display->display_lock);
	return rc;
}

int dsi_display_auto_mode_update(void *dsi_display)
{
	struct dsi_display *display = dsi_display;
	int h_skew = SDC_ADFR;
	int rc = 0;

	if (!display || !display->panel || !display->panel->cur_mode) {
		DSI_ERR("kVRR dsi_display_auto_mode_update Invalid params\n");
		return -EINVAL;
	}

	h_skew = display->panel->cur_mode->timing.h_skew;
	if ((h_skew != SDC_ADFR) && (h_skew != SDC_MFR)) {
		/* DSI_ERR("kVRR OPLUS ADFR does not support auto mode setting\n"); */
		return 0;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported()) {
		if (!strcmp(display->display_type, "secondary")) {
			pr_info("kVRR iris is secondary panel!\n");
			return 0;
		}
	}
#endif

	SDE_ATRACE_BEGIN("dsi_display_auto_mode_update");

	if (oplus_adfr_auto_mode_updated) {
		dsi_display_auto_mode_enable(display, oplus_adfr_auto_mode);
		oplus_adfr_auto_mode_updated = false;
	}

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	if (oplus_ofp_is_supported() && !oplus_ofp_oled_capacitive_is_enabled()
			&& !oplus_ofp_local_hbm_is_enabled() && !oplus_ofp_ultrasonic_is_enabled()) {
		if (oplus_adfr_auto_min_fps_updated && !oplus_ofp_get_hbm_state()) {
			dsi_display_auto_mode_min_fps(display, oplus_adfr_auto_min_fps);
			oplus_adfr_auto_min_fps_updated = false;
		}
	} else {
		if (oplus_adfr_auto_min_fps_updated) {
			dsi_display_auto_mode_min_fps(display, oplus_adfr_auto_min_fps);
			oplus_adfr_auto_min_fps_updated = false;
		}
	}
#else /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	if (oplus_adfr_auto_min_fps_updated) {
		dsi_display_auto_mode_min_fps(display, oplus_adfr_auto_min_fps);
		oplus_adfr_auto_min_fps_updated = false;
	}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

	if (oplus_adfr_auto_fakeframe_updated) {
		/* update fakeframe status after auto mode handle */
		oplus_adfr_fakeframe_status_update(display->panel, false);
		oplus_adfr_auto_fakeframe_updated = false;
	}

	SDE_ATRACE_END("dsi_display_auto_mode_update");

	return rc;
}

/* --------------- idle mode ---------------*/
void oplus_adfr_idle_mode_minfps_delay(void *sde_encoder_phys_cmd)
{
	struct sde_encoder_phys_cmd *cmd_enc = sde_encoder_phys_cmd;
	s64 us_per_frame = 0;
	struct sde_encoder_phys_cmd_te_timestamp *te_timestamp = NULL;
	ktime_t last_te_timestamp = 0;
	s64 delay = 0;

	if (!cmd_enc) {
		SDE_ERROR("kVRR : invalid sde_encoder_phys_cmd parameters\n");
		return;
	}

	SDE_ATRACE_BEGIN("oplus_adfr_idle_mode_minfps_delay");

	/* for tp vysnc shift issue */
	us_per_frame = 1000000 / 60;
	te_timestamp = list_last_entry(&cmd_enc->te_timestamp_list, struct sde_encoder_phys_cmd_te_timestamp, list);
	last_te_timestamp = te_timestamp->timestamp;

	delay = (us_per_frame >> 1) - (ktime_to_us(ktime_sub(ktime_get(), last_te_timestamp)) % us_per_frame);
	SDE_DEBUG("kVRR time interval since the last rd_ptr is %lu\n", ktime_to_us(ktime_sub(ktime_get(), last_te_timestamp)));

	if (delay > 0) {
		/* make sure to send min fps in the next 8.3ms period */
		delay = delay + 1000;
		usleep_range(delay, delay + 100);
		SDE_DEBUG("kVRR delay %lu us for idle min fps setting\n", delay);
	}

	SDE_ATRACE_END("oplus_adfr_idle_mode_minfps_delay");

	return;
}

/* if idle mode is on, the min fps will be reduced when entering MIPI idle and increased when leaving MIPI idle, thus saving power more accurately */
void oplus_adfr_handle_idle_mode(void *sde_enc_v, int enter_idle)
{
	struct sde_encoder_virt *sde_enc = (struct sde_encoder_virt *)sde_enc_v;
	struct sde_encoder_phys *phys = NULL;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct dsi_panel *panel = NULL;
	struct sde_encoder_phys_cmd *cmd_enc = NULL;
	u32 h_skew = SDC_ADFR;
	u32 refresh_rate = 120;

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	if (oplus_ofp_is_supported() && !oplus_ofp_oled_capacitive_is_enabled()
			&& !oplus_ofp_local_hbm_is_enabled() && !oplus_ofp_ultrasonic_is_enabled()
			 && oplus_ofp_get_hbm_state())
		return;
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

	if (!oplus_adfr_idle_mode_is_enable()) {
		return;
	}

	if (!sde_enc) {
		SDE_ERROR("kVRR : invalid sde_encoder_virt parameters\n");
		return;
	}

	phys = sde_enc->phys_encs[0];

	if (!phys || !phys->connector) {
		SDE_ERROR("kVRR : invalid sde_encoder_phys parameters\n");
		return;
	}

	c_conn = to_sde_connector(phys->connector);
	if (!c_conn) {
		SDE_ERROR("kVRR : invalid sde_connector parameters\n");
		return;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI)
		return;

	display = c_conn->display;
	if (!display || !display->panel || !display->panel->cur_mode) {
		SDE_ERROR("kVRR : invalid dsi_display parameters\n");
		return;
	}
	panel = display->panel;

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported()) {
		if (!strcmp(panel->type, "secondary")) {
			pr_info("kVRR iris is secondary panel!\n");
			return;
		}
	}
#endif

	cmd_enc = to_sde_encoder_phys_cmd(phys);
	if (!cmd_enc) {
		SDE_ERROR("kVRR : invalid sde_encoder_phys_cmd parameters\n");
		return;
	}

	SDE_ATRACE_BEGIN("oplus_adfr_handle_idle_mode");

	h_skew = panel->cur_mode->timing.h_skew;
	refresh_rate = panel->cur_mode->timing.refresh_rate;

	if (enter_idle) {
		if (h_skew == SDC_ADFR || h_skew == SDC_MFR) {
			if (refresh_rate == 120 || refresh_rate == 60) {
				/* enter idle mode if auto mode is off and min fps is less than 20hz */
				if ((oplus_adfr_auto_mode == OPLUS_ADFR_AUTO_OFF) && (oplus_adfr_auto_min_fps > OPLUS_ADFR_AUTO_MIN_FPS_20HZ)
					&& (oplus_adfr_auto_min_fps <= OPLUS_ADFR_AUTO_MIN_FPS_1HZ)) {
					oplus_adfr_idle_mode = OPLUS_ADFR_IDLE_ON;
					SDE_DEBUG("kVRR idle mode on");

					/* for tp vysnc shift issue */
					if (refresh_rate == 60) {
						oplus_adfr_idle_mode_minfps_delay(cmd_enc);
					}

					/* send min fps before enter idle */
					SDE_DEBUG("kVRR enter idle, min fps %d", oplus_adfr_auto_min_fps);
					dsi_display_auto_mode_min_fps(display, oplus_adfr_auto_min_fps);

					if (oplus_adfr_dynamic_te.config != OPLUS_ADFR_DYNAMIC_TE_DISABLE) {
						if (oplus_adfr_auto_min_fps >= OPLUS_ADFR_AUTO_MIN_FPS_10HZ) {
							/* update report rate if enter idle mode */
							hrtimer_start(&oplus_adfr_dynamic_te.timer, ms_to_ktime(10), HRTIMER_MODE_REL);
						}
					}
				}
			}
		}
	} else {
		/* exit idle mode */
		if (oplus_adfr_idle_mode == OPLUS_ADFR_IDLE_ON) {
			if (oplus_adfr_auto_min_fps >= OPLUS_ADFR_AUTO_MIN_FPS_20HZ) {
				/* for tp vysnc shift issue */
				if (refresh_rate == 60) {
					oplus_adfr_idle_mode_minfps_delay(cmd_enc);
				}

				/* send min fps after exit idle */
				SDE_DEBUG("kVRR exit idle, min fps %d", OPLUS_ADFR_AUTO_MIN_FPS_20HZ);
				dsi_display_auto_mode_min_fps(display, OPLUS_ADFR_AUTO_MIN_FPS_20HZ);
			}

			oplus_adfr_idle_mode = OPLUS_ADFR_IDLE_OFF;
			SDE_DEBUG("kVRR idle mode off");
		}
	}

	SDE_ATRACE_INT("oplus_adfr_idle_mode", oplus_adfr_idle_mode);
	SDE_ATRACE_END("oplus_adfr_handle_idle_mode");

	return;
}

int oplus_enable_te_refcount(void *data)
{
	unsigned int *te_enable =  (unsigned int *)data;
	struct dsi_display *display = NULL;
	DSI_INFO("%s te_enable = %d", __func__, (*te_enable));

	display = get_main_display();
	if (display == NULL) {
		pr_err("%s error :NULL display", __func__);
		return -1;
	}

	if ((*te_enable) == 1) {
		te_refcount.te_calculate_enable = true;
		te_refcount.start_timeline = ktime_get();
		te_refcount.te_refcount = 0;
	} else if ((*te_enable) == 0) {
		te_refcount.te_calculate_enable = false;
		te_refcount.end_timeline = ktime_get();
	}

	dsi_display_adfr_change_te_irq_status(display, te_refcount.te_calculate_enable);

	return 0;
}

int oplus_get_te_fps(void *data)
{
	unsigned int *te_fps =  (unsigned int *)data;

	unsigned long long end_time, start_time;

	end_time = ktime_to_ms(te_refcount.end_timeline);
	start_time = ktime_to_ms(te_refcount.start_timeline);

	if (end_time < start_time) {
		pr_err("%s error :out of time", __func__);
	}

	(*te_fps) = te_refcount.te_refcount*1000 / (end_time - start_time);

	DSI_INFO("%s te count = %d, end_time = %lld, start_time = %lld, te fps = %d",
		__func__, te_refcount.te_refcount, end_time, start_time, (*te_fps));

	return 0;
}

#if defined(CONFIG_PXLW_IRIS)
int iris_set_panel_vsync_switch_gpio(struct dsi_panel *panel, int level) {
	int ret = -1;
	struct dsi_display *primary_display = get_main_display();
	struct drm_connector *connector = primary_display->drm_conn;
	struct sde_connector *c_conn;

	if ((primary_display == NULL) || (primary_display->panel == NULL))
		return -EINVAL;

	if (primary_display->panel->vsync_switch_gpio_level == level) {
		DSI_INFO("%s kVRR current gpio level is (%d) same target and return\n", __func__, primary_display->panel->vsync_switch_gpio_level);
		return 0;
	}

	if (!connector || !connector->encoder)
		return ret;

	if (!drm_bridge_chain_get_first_bridge(connector->encoder))
		return ret;

	c_conn = to_sde_connector(connector);

	if (level == OPLUS_VSYNC_SWITCH_TE) {
		if (gpio_is_valid(primary_display->panel->vsync_switch_gpio)) {
			ret = gpio_direction_output(primary_display->panel->vsync_switch_gpio, 1);
			if (ret) {
				DSI_ERR("%s kVRR unable to set dir for vsync_switch_gpio gpio rc=%d\n", __func__, ret);
			} else {
				DSI_INFO("%s kVRR set vsync_switch_gpio to 1\n", __func__);
				SDE_ATRACE_INT("vsync_switch_gpio_level", primary_display->panel->vsync_switch_gpio_level);
			}

			primary_display->panel->vsync_switch_gpio_level = OPLUS_VSYNC_SWITCH_TE;
		}
	}

	if (level == OPLUS_VSYNC_SWITCH_TP) {
		/* after one frame commit completed, change back to current mode vsync */
		primary_display->panel->need_vsync_switch = true;
		DSI_INFO("%s kVRR set need_vsync_switch to true to switch gpio\n", __func__);
		ret = 0;
	}

	return ret;
}
u32 iris_emv_get_current_extend_frame(void) {
	struct dsi_display *primary_display = get_main_display();

	DSI_INFO("kVRR get_extend_frame is %u %u %d rate: %d\n",
			iris_current_extend_frame, oplus_adfr_auto_min_fps, oplus_adfr_auto_min_fps_updated, primary_display->panel->cur_mode->timing.refresh_rate);

	return iris_current_extend_frame;
}
#endif

