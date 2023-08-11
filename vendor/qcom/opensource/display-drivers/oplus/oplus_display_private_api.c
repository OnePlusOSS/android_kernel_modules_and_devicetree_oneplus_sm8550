/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
**
** File : oplus_display_private_api.h
** Description : oplus display private api implement
** Version : 1.1
** Date : 2022/03/22
** Author : Display
******************************************************************/
#include "oplus_display_private_api.h"
#include "oplus_ffl.h"
#include "oplus_display_panel_power.h"
#include "oplus_display_panel_seed.h"
#include "oplus/oplus_bl.h"
#include "oplus_display_interface.h"

/*
 * we will create a sysfs which called /sys/kernel/oplus_display,
 * In that directory, oplus display private api can be called
 */
#include <linux/notifier.h>
#include <linux/msm_drm_notify.h>
#include <soc/oplus/device_info.h>
#include "dsi_pwr.h"
#include "oplus_display_panel.h"
/* OPLUS_FEATURE_ADFR, oplus adfr */
#include "oplus_adfr.h"

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "oplus_onscreenfingerprint.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
#if defined(CONFIG_PXLW_IRIS)
#include "dsi_iris_api.h"
#endif

/* String length define */
#define STR_SIZE 512

int dither_enable = 1;
int spr_mode = 0;
int lcd_closebl_flag = 0;
int oplus_request_power_status = 0;
int iris_recovery_check_state = -1;

int backlight_smooth_enable = 1;

extern int oplus_underbrightness_alpha;
int oplus_dc2_alpha;
int oplus_dimlayer_bl_enable_v3 = 0;
int oplus_dimlayer_bl_enable_v2 = 0;
int oplus_dimlayer_bl_alpha_v2 = 260;
int oplus_dimlayer_bl_enable = 0;
int oplus_dimlayer_bl_alpha = 223;
int oplus_dimlayer_bl_alpha_value = 223;
int oplus_dimlayer_bl_enable_real = 0;
int oplus_dimlayer_dither_threshold = 0;
int oplus_dimlayer_dither_bitdepth = 6;
int oplus_dimlayer_bl_delay = -1;
int oplus_dimlayer_bl_delay_after = -1;
int oplus_boe_dc_min_backlight = 175;
int oplus_boe_dc_max_backlight = 3350;
int oplus_boe_max_backlight = 2050;
int oplus_dimlayer_bl_enable_v3_real;

int oplus_dimlayer_bl_enable_v2_real = 0;
bool oplus_skip_datadimming_sync = false;

extern int oplus_debug_max_brightness;
int oplus_seed_backlight = 0;

ktime_t oplus_backlight_time;
u32 oplus_last_backlight = 0;
u32 oplus_backlight_delta = 0;
unsigned int oplus_dsi_log_type = OPLUS_DEBUG_LOG_DISABLED;
unsigned int oplus_display_trace_enable = OPLUS_DISPLAY_DISABLE_TRACE;
int dsi_cmd_panel_debug = 0;
uint64_t serial_number_fir = 0x0;
uint64_t serial_number_sec = 0x0;

EXPORT_SYMBOL(oplus_dimlayer_bl_alpha);
EXPORT_SYMBOL(oplus_dimlayer_bl_enable_real);
EXPORT_SYMBOL(oplus_last_backlight);
EXPORT_SYMBOL(oplus_seed_backlight);
EXPORT_SYMBOL(lcd_closebl_flag);
EXPORT_SYMBOL(oplus_request_power_status);
EXPORT_SYMBOL(oplus_backlight_delta);
EXPORT_SYMBOL(oplus_backlight_time);
EXPORT_SYMBOL(oplus_dimlayer_bl_alpha_value);
EXPORT_SYMBOL(oplus_dimlayer_bl_enable);
EXPORT_SYMBOL(oplus_dsi_log_type);
EXPORT_SYMBOL(oplus_display_trace_enable);
EXPORT_SYMBOL(backlight_smooth_enable);

extern PANEL_VOLTAGE_BAK panel_vol_bak[PANEL_VOLTAGE_ID_MAX];
extern u32 panel_pwr_vg_base;
extern int seed_mode;
extern void dsi_display_set_cmd_tx_ctrl_flags(struct dsi_display *display, struct dsi_cmd_desc *cmd);

#define PANEL_TX_MAX_BUF 512
#define PANEL_CMD_MIN_TX_COUNT 2
#define OPLUS_ATTR(_name, _mode, _show, _store) \
struct kobj_attribute oplus_attr_##_name = __ATTR(_name, _mode, _show, _store)

DEFINE_MUTEX(oplus_spr_lock);
DEFINE_MUTEX(oplus_dither_lock);

int oplus_display_set_vendor(struct dsi_display *display)
{
	if (!display || !display->panel ||
			!display->panel->oplus_priv.vendor_name ||
			!display->panel->oplus_priv.manufacture_name) {
		LCD_ERR("failed to config lcd proc device\n");
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->display_type, "secondary"))) {
		LCD_ERR("iris secondary disable config lcd_s proc device\n");
		return -EINVAL;
	}
#endif

	if (!strcmp(display->display_type, "secondary"))
		register_device_proc("lcd_s", (char *)display->panel->oplus_priv.vendor_name,
				(char *)display->panel->oplus_priv.manufacture_name);

	register_device_proc("lcd", (char *)display->panel->oplus_priv.vendor_name,
			(char *)display->panel->oplus_priv.manufacture_name);

	return 0;
}
EXPORT_SYMBOL(oplus_display_set_vendor);

int dsi_panel_spr_mode(struct dsi_panel *panel, int mode)
{
	int rc = 0;

	if (!panel) {
		LCD_ERR("Invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);

	if (!dsi_panel_initialized(panel)) {
		rc = -EINVAL;
		goto error;
	}

	switch (mode) {
	case 0:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SPR_MODE0);
		break;
	case 1:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SPR_MODE1);
		break;
	case 2:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SPR_MODE2);
		break;
	default:
		rc = dsi_panel_tx_cmd_set(panel,
				DSI_CMD_SPR_MODE0);
		LCD_ERR("[%s] Invalid spr mode %d\n",
				panel->oplus_priv.vendor_name, mode);
		break;
	}

error:
	mutex_unlock(&panel->panel_lock);
	return rc;
}

int dsi_panel_read_panel_reg(struct dsi_display_ctrl *ctrl,
		struct dsi_panel *panel, u8 cmd, void *rbuf,  size_t len)
{
	int rc = 0;
	struct dsi_cmd_desc cmdsreq;
	 struct dsi_display *display = get_main_display();

	if (!panel || !ctrl || !ctrl->ctrl) {
		return -EINVAL;
	}

	if (!dsi_ctrl_validate_host_state(ctrl->ctrl)) {
		return 1;
	}

	/* acquire panel_lock to make sure no commands are in progress */
	mutex_lock(&panel->panel_lock);

	if (!dsi_panel_initialized(panel)) {
		rc = -EINVAL;
		goto error;
	}

	memset(&cmdsreq, 0x0, sizeof(cmdsreq));
	cmdsreq.msg.type = 0x06;
	cmdsreq.msg.tx_buf = &cmd;
	cmdsreq.msg.tx_len = 1;
	cmdsreq.msg.rx_buf = rbuf;
	cmdsreq.msg.rx_len = len;
	cmdsreq.msg.flags |= MIPI_DSI_MSG_UNICAST_COMMAND;

	cmdsreq.ctrl_flags = DSI_CTRL_CMD_READ;

	dsi_display_set_cmd_tx_ctrl_flags(display, &cmdsreq);
	rc = dsi_ctrl_transfer_prepare(ctrl->ctrl, cmdsreq.ctrl_flags);
	if (rc) {
		LCD_ERR("prepare for rx cmd transfer failed rc=%d\n", rc);
		goto error;
	}

	rc = dsi_ctrl_cmd_transfer(ctrl->ctrl, &cmdsreq);

	if (rc < 0) {
		LCD_ERR("rx cmd transfer failed, rc=%d\n", rc);
	}

	dsi_ctrl_transfer_unprepare(ctrl->ctrl, cmdsreq.ctrl_flags);

error:
	/* release panel_lock */
	mutex_unlock(&panel->panel_lock);
	return rc;
}

int dsi_display_spr_mode(struct dsi_display *display, int mode)
{
	int rc = 0;

	if (!display || !display->panel) {
		LCD_ERR("Invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&display->display_lock);

	/* enable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_ON);
	}

	rc = dsi_panel_spr_mode(display->panel, mode);

	if (rc) {
		LCD_ERR("[%s] failed to dsi_panel_spr_on, rc=%d\n",
				display->name, rc);
	}

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
					  DSI_CORE_CLK, DSI_CLK_OFF);
	}

	mutex_unlock(&display->display_lock);
	return rc;
}

int dsi_display_read_panel_reg(struct dsi_display *display, u8 cmd, void *data,
		size_t len)
{
	int rc = 0;
	struct dsi_display_ctrl *m_ctrl;

	if (!display || !display->panel || data == NULL) {
		LCD_ERR("Invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&display->display_lock);

	m_ctrl = &display->ctrl[display->cmd_master_idx];

	if (display->tx_cmd_buf == NULL) {
		rc = dsi_host_alloc_cmd_tx_buffer(display);

		if (rc) {
			LCD_ERR("failed to allocate cmd tx buffer memory\n");
			goto done;
		}
	}

	rc = dsi_display_cmd_engine_enable(display);

	if (rc) {
		LCD_ERR("cmd engine enable failed\n");
		goto done;
	}

	/* enable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_ALL_CLKS, DSI_CLK_ON);
	}

	rc = dsi_panel_read_panel_reg(m_ctrl, display->panel, cmd, data, len);

	if (rc < 0) {
		LCD_ERR("[%s] failed to read panel register, rc=%d,cmd=%d\n",
				display->name,
				rc,
				cmd);
	}

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_ALL_CLKS, DSI_CLK_OFF);
	}

	dsi_display_cmd_engine_disable(display);

done:
	mutex_unlock(&display->display_lock);
	LCD_INFO("return: %d\n", rc);
	return rc;
}

static ssize_t oplus_display_set_seed(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int temp_save = 0;
	struct dsi_display *display = get_main_display();

	if (!display || !display->panel) {
		LCD_ERR("display is null\n");
		return -EINVAL;
	}

	sscanf(buf, "%du", &temp_save);
	LCD_INFO("oplus_display_set_seed = %d\n", temp_save);

	__oplus_set_seed_mode(temp_save);

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	if (oplus_ofp_is_supported() && !oplus_ofp_oled_capacitive_is_enabled()
			&& !oplus_ofp_local_hbm_is_enabled()) {
		if (oplus_ofp_get_hbm_state()) {
			OFP_INFO("should not set seed in hbm state\n");
			return count;
		}
	}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

	/* if (__oplus_get_power_status() == OPLUS_DISPLAY_POWER_ON) { */
	if (display->panel->power_mode == SDE_MODE_DPMS_ON) {
		if (get_main_display() == NULL) {
			LCD_ERR("display is null\n");
			return count;
		}

		dsi_display_seed_mode_lock(get_main_display(), seed_mode);

	} else {
		LCD_ERR("oplus_display_set_seed = %d, but now display panel status is not on\n",
				temp_save);
	}

	return count;
}

int __oplus_display_set_spr(int mode)
{
	mutex_lock(&oplus_spr_lock);

	if (mode != spr_mode) {
		spr_mode = mode;
	}

	mutex_unlock(&oplus_spr_lock);
	return 0;
}

int __oplus_display_set_dither(int mode)
{
	mutex_lock(&oplus_dither_lock);

	if (mode != dither_enable) {
		dither_enable = mode;
	}

	mutex_unlock(&oplus_dither_lock);
	return 0;
}

int oplus_display_panel_update_spr_mode(void)
{
	struct dsi_display *display = get_main_display();
	int ret = 0;

	if (!display) {
		LCD_ERR("display is null\n");
		return -EINVAL;
	}

	ret = dsi_display_spr_mode(display, spr_mode);

	return ret;
}

static ssize_t oplus_display_set_spr(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int temp_save = 0;
	struct dsi_display *display = get_main_display();

	if (!display || !display->panel) {
		LCD_ERR("display is null\n");
		return -EINVAL;
	}

	sscanf(buf, "%du", &temp_save);
	LCD_INFO("oplus_display_set_spr = %d\n", temp_save);

	__oplus_display_set_spr(temp_save);

	/* if (__oplus_get_power_status() == OPLUS_DISPLAY_POWER_ON) { */
	if (display->panel->power_mode == SDE_MODE_DPMS_ON) {
		if (get_main_display() == NULL) {
			LCD_ERR("display is null\n");
			return count;
		}

		dsi_display_spr_mode(get_main_display(), spr_mode);

	} else {
		LCD_ERR("oplus_display_set_spr = %d, but now display panel status is not on\n",
				temp_save);
	}

	return count;
}

static ssize_t oplus_display_set_dither(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int temp_save = 0;

	sscanf(buf, "%du", &temp_save);
	LCD_INFO("oplus_display_set_dither = %d\n", temp_save);
	__oplus_display_set_dither(temp_save);
	return count;
}

bool oplus_is_support_panel_dither(const char *panel_name) {
	if (panel_name == NULL)
		return false;
	if ((!strcmp(panel_name, "AMS662ZS02")) && dither_enable) {
		return true;
	} else {
		return false;
	}
}

int oplus_display_audio_ready = 0;
static ssize_t oplus_display_set_audio_ready(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	sscanf(buf, "%du", &oplus_display_audio_ready);
	return count;
}

static ssize_t oplus_display_get_seed(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	LCD_INFO("oplus_display_get_seed = %d\n", seed_mode);
	return sprintf(buf, "%d\n", seed_mode);
}

static ssize_t oplus_display_get_spr(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	LCD_INFO("oplus_display_get_spr = %d\n", spr_mode);
	return sprintf(buf, "%d\n", spr_mode);
}

static ssize_t oplus_display_get_dither(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	LCD_INFO("oplus_display_get_dither = %d\n", dither_enable);
	return sprintf(buf, "%d\n", dither_enable);
}
static ssize_t oplus_display_get_iris_state(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", iris_recovery_check_state);
}

static ssize_t oplus_display_regulator_control(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int temp_save = 0;
	struct dsi_display *temp_display;
	sscanf(buf, "%du", &temp_save);
	LCD_INFO("oplus_display_regulator_control = %d\n", temp_save);
	if (get_main_display() == NULL) {
		LCD_ERR("display is null\n");
		return count;
	}
	temp_display = get_main_display();
	if (temp_save == 0) {
		dsi_pwr_enable_regulator(&temp_display->panel->power_info, false);
	} else if (temp_save == 1) {
		dsi_pwr_enable_regulator(&temp_display->panel->power_info, true);
	}
	return count;
}

static ssize_t oplus_display_get_panel_serial_number(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned char read[30];
	PANEL_SERIAL_INFO panel_serial_info;
	uint64_t serial_number;
	struct dsi_display *display = get_main_display();
	int i;
	int panel_id = 0;

	if (!display || !display->panel) {
		LCD_ERR("display is null\n");
		return -1;
	}

	if(display->enabled == false) {
		LCD_INFO("primary display is disable, try sec display\n");
		display = get_sec_display();
		if (!display) {
			LCD_INFO("second display is null\n");
			return -1;
		}
		if (display->enabled == false) {
			LCD_INFO("second panel is disabled\n");
			return -1;
		}
		panel_id = 1;
	}

	/* if (__oplus_get_power_status() != OPLUS_DISPLAY_POWER_ON) { */
	if (display->panel->power_mode != SDE_MODE_DPMS_ON) {
		LCD_INFO("display panel in off status\n");
		return ret;
	}

	if (!display->panel->panel_initialized) {
		LCD_ERR("panel initialized = false\n");
		return ret;
	}
	/*
	 * To fix bug id 5489022, we do not read serial number frequently.
	 * First read, then return the saved value.
	 */
	if (1 == panel_id) {
		if (serial_number_sec != 0) {
			ret = scnprintf(buf, PAGE_SIZE, "Get panel serial number: %llx\n",
							serial_number_sec);
			pr_info("%s read serial_number_sec 0x%x\n", __func__, serial_number_sec);
			return ret;
		}
	} else {
		if (serial_number_fir != 0) {
			ret = scnprintf(buf, PAGE_SIZE, "Get panel serial number: %llx\n",
							serial_number_fir);
			pr_info("%s read serial_number_fir 0x%x\n", __func__, serial_number_fir);
			return ret;
		}
	}

	/*
	 * for some unknown reason, the panel_serial_info may read dummy,
	 * retry when found panel_serial_info is abnormal.
	 */
	for (i = 0; i < 5; i++) {
		if (!strcmp(display->panel->name, "boe rm692e5 dsc cmd mode panel")) {
			mutex_lock(&display->display_lock);
			mutex_lock(&display->panel->panel_lock);
			ret = dsi_panel_tx_cmd_set(display->panel, DSI_CMD_PANEL_DATE_SWITCH);
			mutex_unlock(&display->panel->panel_lock);
			mutex_unlock(&display->display_lock);

			if (ret) {
				return -1;
			}
		} else if (!display->panel->oplus_ser.is_reg_lock) {
			/* unknow what this case want to do */
		} else {
			mutex_lock(&display->display_lock);
			mutex_lock(&display->panel->panel_lock);
			if (display->config.panel_mode == DSI_OP_CMD_MODE) {
				dsi_display_clk_ctrl(display->dsi_clk_handle, DSI_ALL_CLKS, DSI_CLK_ON);
			}
			 {
				char value[] = {0x5A, 0x5A};
				ret = mipi_dsi_dcs_write(&display->panel->mipi_device, 0xF0, value, sizeof(value));
			 }
			if (display->config.panel_mode == DSI_OP_CMD_MODE) {
				dsi_display_clk_ctrl(display->dsi_clk_handle, DSI_ALL_CLKS, DSI_CLK_OFF);
			}
			mutex_unlock(&display->panel->panel_lock);
			mutex_unlock(&display->display_lock);
			if (ret < 0) {
				ret = scnprintf(buf, PAGE_SIZE, "Get panel serial number failed, reason:%d", ret);
				msleep(20);
				continue;
			}
		}

		ret = dsi_display_read_panel_reg(display, display->panel->oplus_ser.serial_number_reg,
				read, display->panel->oplus_ser.serial_number_conut);

		if (ret < 0) {
			ret = scnprintf(buf, PAGE_SIZE,
					"Get panel serial number failed, reason:%d", ret);
			msleep(20);
			continue;
		}

		/*  0xA1               11th        12th    13th    14th    15th
		 *  HEX                0x32        0x0C    0x0B    0x29    0x37
		 *  Bit           [D7:D4][D3:D0] [D5:D0] [D5:D0] [D5:D0] [D5:D0]
		 *  exp              3      2       C       B       29      37
		 *  Yyyy,mm,dd      2014   2m      12d     11h     41min   55sec
		*/
		panel_serial_info.reg_index = display->panel->oplus_ser.serial_number_index;

		if (!strcmp(display->panel->name, "boe rm692e5 dsc cmd mode panel")) {
			read[panel_serial_info.reg_index] += 3;
			panel_serial_info.year		= (read[panel_serial_info.reg_index] & 0xF0) >> 0x4;
			panel_serial_info.year += 1;
		} else {
			panel_serial_info.year		= (read[panel_serial_info.reg_index] & 0xF0) >> 0x4;
		}
		panel_serial_info.month		= read[panel_serial_info.reg_index]	& 0x0F;
		panel_serial_info.day		= read[panel_serial_info.reg_index + 1]	& 0x1F;
		panel_serial_info.hour		= read[panel_serial_info.reg_index + 2]	& 0x1F;
		panel_serial_info.minute	= read[panel_serial_info.reg_index + 3]	& 0x3F;
		panel_serial_info.second	= read[panel_serial_info.reg_index + 4]	& 0x3F;
		panel_serial_info.reserved[0] = read[panel_serial_info.reg_index + 5];
		panel_serial_info.reserved[1] = read[panel_serial_info.reg_index + 6];

		serial_number = (panel_serial_info.year		<< 56)\
				+ (panel_serial_info.month		<< 48)\
				+ (panel_serial_info.day		<< 40)\
				+ (panel_serial_info.hour		<< 32)\
				+ (panel_serial_info.minute	<< 24)\
				+ (panel_serial_info.second	<< 16)\
				+ (panel_serial_info.reserved[0] << 8)\
				+ (panel_serial_info.reserved[1]);

		if (!panel_serial_info.year) {
			/*
			 * the panel we use always large than 2011, so
			 * force retry when year is 2011
			 */
			msleep(20);
			continue;
		}

		ret = scnprintf(buf, PAGE_SIZE, "Get panel serial number: %llx\n",
				serial_number);
		/*Save serial_number value.*/
		if (1 == panel_id) {
			serial_number_sec = serial_number;
		} else {
			serial_number_fir = serial_number;
		}
		break;
	}

	return ret;
}

static char oplus_rx_reg[PANEL_TX_MAX_BUF] = {0x0};
static char oplus_rx_len = 0;
static ssize_t oplus_display_get_panel_reg(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	struct dsi_display *display = get_main_display();
	int i, cnt = 0;

	if (1 == dsi_cmd_panel_debug)
		display = get_sec_display();

	if (!display) {
		return -EINVAL;
	}

	mutex_lock(&display->display_lock);

	for (i = 0; i < oplus_rx_len; i++)
		cnt += snprintf(buf + cnt, PANEL_TX_MAX_BUF - cnt,
				"%02x ", oplus_rx_reg[i]);

	cnt += snprintf(buf + cnt, PANEL_TX_MAX_BUF - cnt, "\n");
	mutex_unlock(&display->display_lock);

	return cnt;
}

static ssize_t oplus_display_set_panel_reg(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	char reg[PANEL_TX_MAX_BUF] = {0x0};
	char payload[PANEL_TX_MAX_BUF] = {0x0};
	u32 index = 0, value = 0, step = 0;
	int ret = 0;
	int len = 0;
	char *bufp = (char *)buf;
	struct dsi_display *display = get_main_display();
	char read;

	if (1 == dsi_cmd_panel_debug)
		display = get_sec_display();

	if (!display || !display->panel) {
		LCD_ERR("display is null\n");
		return -EFAULT;
	}

	if (sscanf(bufp, "%c%n", &read, &step) && read == 'r') {
		bufp += step;
		sscanf(bufp, "%x %d", &value, &len);

		if (len > PANEL_TX_MAX_BUF) {
			LCD_ERR("failed\n");
			return -EINVAL;
		}

		dsi_display_read_panel_reg(display, value, reg, len);

		for (index = 0; index < len; index++) {
			LCD_INFO("%x\n", reg[index]);
		}

		mutex_lock(&display->display_lock);
		memcpy(oplus_rx_reg, reg, PANEL_TX_MAX_BUF);
		oplus_rx_len = len;
		mutex_unlock(&display->display_lock);
		return count;
	}

	while (sscanf(bufp, "%x%n", &value, &step) > 0) {
		reg[len++] = value;

		if (len >= PANEL_TX_MAX_BUF) {
			LCD_ERR("wrong input reg len\n");
			return -EFAULT;
		}

		bufp += step;
	}

	for (index = 0; index < len; index ++) {
		payload[index] = reg[index + 1];
	}

	/* if (__oplus_get_power_status() == OPLUS_DISPLAY_POWER_ON) { */
	if (display->panel->power_mode != SDE_MODE_DPMS_OFF) {
		/* enable the clk vote for CMD mode panels */
		mutex_lock(&display->display_lock);
		mutex_lock(&display->panel->panel_lock);

		if (display->panel->panel_initialized) {
			if (display->config.panel_mode == DSI_OP_CMD_MODE) {
				dsi_display_clk_ctrl(display->dsi_clk_handle,
						DSI_ALL_CLKS, DSI_CLK_ON);
			}

			ret = mipi_dsi_dcs_write(&display->panel->mipi_device, reg[0],
					payload, len - 1);

			if (display->config.panel_mode == DSI_OP_CMD_MODE) {
				dsi_display_clk_ctrl(display->dsi_clk_handle,
						DSI_ALL_CLKS, DSI_CLK_OFF);
			}
		}

		mutex_unlock(&display->panel->panel_lock);
		mutex_unlock(&display->display_lock);

		if (ret < 0) {
			return ret;
		}
	}

	return count;
}

static ssize_t oplus_display_get_panel_id(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	struct dsi_display *display = get_main_display();
	int ret = 0;
	unsigned char read[30];
	char da = 0;
	char db = 0;
	char dc = 0;

	if (!display || !display->panel) {
		LCD_ERR("display is null\n");
		ret = -1;
		return ret;
	}

	/* if (__oplus_get_power_status() == OPLUS_DISPLAY_POWER_ON) { */
	if (display->panel->power_mode == SDE_MODE_DPMS_ON) {
		if (display == NULL) {
			LCD_ERR("display is null\n");
			ret = -1;
			return ret;
		}

		if (!strcmp(display->panel->oplus_priv.vendor_name, "NT37290")) {
			char value[] = {0x55, 0xAA, 0x52, 0x08, 0x03};
			ret = mipi_dsi_dcs_write(&display->panel->mipi_device, 0xF0, value, sizeof(value));
			ret = dsi_display_read_panel_reg(display, 0xDF, read, 8);
			if (ret < 0) {
				LCD_ERR("failed to read da ret=%d\n", ret);
				return -EINVAL;
			}
			da = read[4];
			db = read[5];
			dc = read[6];
		} else {
			ret = dsi_display_read_panel_reg(display, 0xda, read, 1);
			if (ret < 0) {
				LCD_ERR("failed to read da ret=%d\n", ret);
				return -EINVAL;
			}
			da = read[0];
			ret = dsi_display_read_panel_reg(display, 0xdb, read, 1);
			if (ret < 0) {
				LCD_ERR("failed to read da ret=%d\n", ret);
				return -EINVAL;
			}
			db = read[0];
			ret = dsi_display_read_panel_reg(display, 0xdc, read, 1);
			if (ret < 0) {
				LCD_ERR("failed to read da ret=%d\n", ret);
				return -EINVAL;
			}
			dc = read[0];
		}
		ret = scnprintf(buf, PAGE_SIZE, "%02x %02x %02x\n", da, db, dc);

	} else {
		LCD_ERR("display panel status is not on\n");
	}

	return ret;
}

static ssize_t oplus_display_get_panel_dsc(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned char read[30];
	struct dsi_display *display = get_main_display();

	if (!display || !display->panel) {
		LCD_ERR("display is null\n");
		return -EINVAL;
	}

	/* if (__oplus_get_power_status() == OPLUS_DISPLAY_POWER_ON) { */
	if (display->panel->power_mode == SDE_MODE_DPMS_ON) {
		if (get_main_display() == NULL) {
			LCD_ERR("display is null\n");
			ret = -1;
			return ret;
		}

		ret = dsi_display_read_panel_reg(get_main_display(), 0x03, read, 1);

		if (ret < 0) {
			ret = scnprintf(buf, PAGE_SIZE, "oplus_display_get_panel_dsc failed, reason:%d",
					ret);

		} else {
			ret = scnprintf(buf, PAGE_SIZE, "oplus_display_get_panel_dsc: 0x%x\n", read[0]);
		}

	} else {
		LCD_ERR("display panel status is not on\n");
	}

	return ret;
}

static ssize_t oplus_display_dump_info(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	int ret = 0;
	struct dsi_display *temp_display;

	temp_display = get_main_display();

	if (temp_display == NULL) {
		LCD_ERR("display is null\n");
		ret = -1;
		return ret;
	}

	if (temp_display->modes == NULL) {
		LCD_ERR("display mode is null\n");
		ret = -1;
		return ret;
	}

	ret = scnprintf(buf, PAGE_SIZE,
			"oplus_display_dump_info: height =%d,width=%d,frame_rate=%d,clk_rate=%llu\n",
			temp_display->modes->timing.h_active, temp_display->modes->timing.v_active,
			temp_display->modes->timing.refresh_rate,
			temp_display->modes->timing.clk_rate_hz);

	return ret;
}

static ssize_t oplus_display_get_power_status(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	LCD_INFO("oplus_display_get_power_status = %d, request power :%d\n",
			__oplus_get_power_status(), oplus_request_power_status);

	return sprintf(buf, "kernel power :%d   request power :%d\n",
			__oplus_get_power_status(), oplus_request_power_status);
}

static ssize_t oplus_display_set_power_status(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int temp_save = 0;

	sscanf(buf, "%du", &temp_save);
	LCD_INFO("oplus_display_set_power_status = %d\n", temp_save);

	__oplus_set_request_power_status(temp_save);

	return count;
}

static ssize_t oplus_display_get_closebl_flag(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	LCD_INFO("oplus_display_get_closebl_flag = %d\n", lcd_closebl_flag);
	return sprintf(buf, "%d\n", lcd_closebl_flag);
}

static ssize_t oplus_display_set_closebl_flag(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int closebl = 0;
	sscanf(buf, "%du", &closebl);
	LCD_ERR("lcd_closebl_flag = %d\n", closebl);

	if (1 != closebl) {
		lcd_closebl_flag = 0;
	}

	LCD_ERR("oplus_display_set_closebl_flag = %d\n", lcd_closebl_flag);
	return count;
}

static ssize_t oplus_backlight_smooth_get_debug(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	LCD_INFO("oplus_backlight_smooth_get_debug = %d\n", backlight_smooth_enable);
	return sprintf(buf, "%d\n", backlight_smooth_enable);
}

static ssize_t oplus_backlight_smooth_set_debug(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int bk_feature = 0;
	sscanf(buf, "%du", &bk_feature);
	LCD_ERR("backlight_smooth_enable = %d\n", bk_feature);

	if (1 != bk_feature) {
		backlight_smooth_enable = 0;
	} else {
		backlight_smooth_enable = 1;
	}

	LCD_INFO("oplus_backlight_smooth_set_debug = %d\n", backlight_smooth_enable);
	return count;
}

static ssize_t oplus_get_pwm_turbo_debug(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	int rc = 0;
	u32 enabled = 0;
	struct dsi_display *display = get_main_display();
	struct dsi_panel *panel = NULL;

	if (!display || !display->panel) {
		LCD_ERR("Invalid display or panel\n");
		rc = -EINVAL;
		return rc;
	}

	panel = display->panel;

	if (!panel->oplus_priv.pwm_turbo_support) {
		LCD_ERR("Falied to get pwm turbo status, because it is unsupport\n");
		rc = -EFAULT;
		return rc;
	}

	mutex_lock(&display->display_lock);
	mutex_lock(&panel->panel_lock);

	enabled = panel->oplus_priv.pwm_turbo_enabled;

	mutex_unlock(&panel->panel_lock);
	mutex_unlock(&display->display_lock);
	LCD_INFO("Get pwm turbo status: %d\n", enabled);

	return sprintf(buf, "%d\n", enabled);
}

static ssize_t oplus_set_pwm_turbo_debug(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int rc = 0;
	u32 enabled = 0;
	struct dsi_display *display = get_main_display();
	struct dsi_panel *panel = NULL;

	if (!display || !display->panel) {
		LCD_ERR("Invalid display or panel\n");
		rc = -EINVAL;
		return rc;
	}

	panel = display->panel;

	if (!panel->oplus_priv.pwm_turbo_support) {
		LCD_ERR("Falied to set pwm turbo status, because it is unsupport\n");
		rc = -EFAULT;
		return rc;
	}

	sscanf(buf, "%du", &enabled);
	LCD_INFO("Set pwm turbo status: %d\n", enabled);

	mutex_lock(&display->display_lock);
	oplus_panel_update_pwm_turbo_lock(panel, enabled);
	mutex_unlock(&display->display_lock);

	return count;
}

static ssize_t oplus_get_ffc_mode_debug(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	int rc = 0;
	u32 ffc_mode = 0;
	struct dsi_display *display = get_main_display();
	struct dsi_panel *panel = NULL;

	if (!display || !display->panel) {
		LCD_ERR("Invalid display or panel\n");
		rc = -EINVAL;
		return rc;
	}

	panel = display->panel;

	if (!panel->oplus_priv.ffc_enabled) {
		LCD_ERR("Falied to get ffc mode, because it is unsupport\n");
		rc = -EFAULT;
		return rc;
	}

	mutex_lock(&panel->oplus_ffc_lock);
	ffc_mode = panel->oplus_priv.ffc_mode_index;
	mutex_unlock(&panel->oplus_ffc_lock);
	LCD_INFO("Get ffc mode: %d\n", ffc_mode);

	return sprintf(buf, "%d\n", ffc_mode);
}

static ssize_t oplus_set_ffc_mode_debug(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int rc = 0;
	u32 ffc_mode = 0;
	struct dsi_display *display = get_main_display();
	struct dsi_panel *panel = NULL;

	if (!display || !display->panel) {
		LCD_ERR("Invalid display or panel\n");
		rc = -EINVAL;
		return rc;
	}

	panel = display->panel;

	if (!panel->oplus_priv.ffc_enabled) {
		LCD_ERR("Falied to set ffc mode, because it is unsupport\n");
		rc = -EFAULT;
		return rc;
	}

	if(display->panel->power_mode != SDE_MODE_DPMS_ON) {
		LCD_WARN("display panel is not on\n");
		rc = -EFAULT;
		return rc;
	}

	sscanf(buf, "%du", &ffc_mode);
	LCD_INFO("Set ffc mode: %d\n", ffc_mode);

	if (ffc_mode > FFC_MODE_MAX_COUNT) {
		LCD_ERR("Invalid ffc mode:[%d]\n", ffc_mode);
		rc = -EINVAL;
		return rc;
	}

	mutex_lock(&panel->oplus_ffc_lock);
	panel->oplus_priv.ffc_mode_index = ffc_mode;

	mutex_lock(&panel->panel_lock);
	rc = oplus_panel_set_ffc_mode_unlock(panel);
	mutex_unlock(&panel->panel_lock);

	mutex_unlock(&display->panel->oplus_ffc_lock);

	return count;
}

static void oplus_display_print_cmd_desc(const struct dsi_panel_cmd_set *cmd_sets)
{
	int i, j, len;
	char buf[PANEL_TX_MAX_BUF];
	struct dsi_cmd_desc *cmds;
	struct mipi_dsi_msg msg;
	char *tx_buf = NULL;

	for (i = 0; i < cmd_sets->count; i++) {
		len = 0;
		cmds = &(cmd_sets->cmds[i]);
		msg = cmds->msg;
		tx_buf = (char*)msg.tx_buf;
		memset(buf, 0, sizeof(buf));

		if (msg.tx_len >= (PANEL_TX_MAX_BUF / 2)) {
			LCD_ERR("%s Skip current cmds[%d], invalid msg.tx_len=%d\n",
					DISPLAY_TOOL_CMD_KEYWORD, i, msg.tx_len);
			break;
		}

		/* Packet Info */
		len += snprintf(buf, sizeof(buf) - len, "%02X ", msg.type);
		len += snprintf(buf + len, sizeof(buf) - len, "%02X ", 0x00);
		len += snprintf(buf + len, sizeof(buf) - len, "%02X ", msg.channel);
		/* Batch Flag */
		len += snprintf(buf + len, sizeof(buf) - len, "%02X ",
				(msg.flags & MIPI_DSI_MSG_BATCH_COMMAND) ?
				MIPI_DSI_MSG_BATCH_COMMAND : 0x00);
		/* Delay */
		len += snprintf(buf + len, sizeof(buf) - len, "%02X ", cmds->post_wait_ms);
		len += snprintf(buf + len, sizeof(buf) - len, "%02X %02X",
				msg.tx_len >> 8, msg.tx_len & 0xFF);
		/* Packet Payload */
		for (j = 0 ; j < msg.tx_len ; j++) {
			len += snprintf(buf + len, sizeof(buf) - len, " %02X", tx_buf[j]);
		}

		LCD_ERR("%s%s\n", DISPLAY_TOOL_CMD_KEYWORD, buf);
	}
}

extern const char *cmd_set_prop_map[];
static ssize_t oplus_display_get_dsi_command(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	int i, cnt;

	cnt = snprintf(buf, PAGE_SIZE,
			"read current dsi_cmd:\n"
			"    echo dump > dsi_cmd  - then you can find dsi cmd on kmsg\n"
			"set sence dsi cmd:\n"
			"  example hbm on:\n"
			"    echo qcom,mdss-dsi-hbm-on-command > dsi_cmd\n"
			"    echo [dsi cmd0] > dsi_cmd\n"
			"    echo [dsi cmd1] > dsi_cmd\n"
			"    echo [dsi cmdX] > dsi_cmd\n"
			"    echo flush > dsi_cmd\n"
			"available dsi_cmd sences:\n");

	for (i = 0; i < DSI_CMD_SET_MAX; i++)
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
				"    %s\n", cmd_set_prop_map[i]);

	return cnt;
}

static int oplus_display_dump_dsi_command(struct dsi_display *display)
{
	struct dsi_display_mode *mode;
	struct dsi_display_mode_priv_info *priv_info;
	struct dsi_panel_cmd_set *cmd_sets;
	const char *cmd_name = NULL;
	int i;

	if (!display || !display->panel || !display->panel->cur_mode) {
		LCD_ERR("failed to get main dsi display\n");
		return -EFAULT;
	}

	mode = display->panel->cur_mode;

	if (!mode || !mode->priv_info) {
		LCD_ERR("failed to get dsi display mode\n");
		return -EFAULT;
	}

	priv_info = mode->priv_info;
	cmd_sets = priv_info->cmd_sets;

	for (i = 0; i < DSI_CMD_SET_MAX; i++) {
		cmd_name = cmd_set_prop_map[i];
		if (!cmd_name || !cmd_sets[i].cmds || !cmd_sets[i].count) {
			continue;
		}
		LCD_ERR("%s%s: %s\n", DISPLAY_TOOL_CMD_KEYWORD, cmd_name,
				cmd_sets[i].state == DSI_CMD_SET_STATE_LP ?
				"dsi_lp_mode" : "dsi_hs_mode");
		oplus_display_print_cmd_desc(&cmd_sets[i]);
	}

	return 0;
}

static int oplus_dsi_panel_get_cmd_pkt_count(const char *data, u32 length,
		u32 *cnt)
{
	const u32 cmd_set_min_size = 7;
	u32 count = 0;
	u32 packet_length;
	u32 tmp;

	while (length >= cmd_set_min_size) {
		packet_length = cmd_set_min_size;
		tmp = ((data[5] << 8) | (data[6]));
		packet_length += tmp;

		if (packet_length > length) {
			LCD_ERR("format error packet_length[%d] length[%d] count[%d]\n",
					packet_length, length, count);
			return -EINVAL;
		}

		length -= packet_length;
		data += packet_length;
		count++;
	}

	*cnt = count;

	return 0;
}

static int oplus_dsi_panel_create_cmd_packets(const char *data,
		u32 length,
		u32 count,
		struct dsi_cmd_desc *cmd)
{
	int rc = 0;
	int i, j;
	u8 *payload;
	u32 size;

	for (i = 0; i < count; i++) {
		cmd[i].msg.type = data[0];
		cmd[i].msg.channel = data[2];
		cmd[i].msg.flags |= data[3];
		cmd[i].ctrl = 0;
		cmd[i].post_wait_ms = data[4];
		cmd[i].msg.tx_len = ((data[5] << 8) | (data[6]));

		if (cmd[i].msg.flags & MIPI_DSI_MSG_BATCH_COMMAND)
			cmd[i].last_command = false;
		else
			cmd[i].last_command = true;

		size = cmd[i].msg.tx_len * sizeof(u8);

		payload = kzalloc(size, GFP_KERNEL);
		if (!payload) {
			rc = -ENOMEM;
			goto error_free_payloads;
		}

		for (j = 0; j < cmd[i].msg.tx_len; j++) {
			payload[j] = data[7 + j];
		}

		cmd[i].msg.tx_buf = payload;
		data += (7 + cmd[i].msg.tx_len);
	}

	return rc;
error_free_payloads:
	for (i = i - 1; i >= 0; i--) {
		cmd--;
		kfree(cmd->msg.tx_buf);
	}

	return rc;
}

static ssize_t oplus_display_set_dsi_command(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	struct dsi_display *display = get_main_display();
	struct dsi_display_mode *mode;
	struct dsi_display_mode_priv_info *priv_info;
	struct dsi_panel_cmd_set *cmd_sets;
	char *bufp = (char *)buf;
	struct dsi_cmd_desc *cmds;
	struct dsi_panel_cmd_set *cmd;
	static char *cmd_bufs;
	static int cmd_counts;
	static u32 oplus_dsi_command = DSI_CMD_SET_MAX;
	static int oplus_dsi_state = DSI_CMD_SET_STATE_HS;
	u32 old_dsi_command = oplus_dsi_command;
	u32 packet_count = 0, size;
	int rc = count, i;
	char data[SZ_512] = "";
	bool flush = false;

	if (1 == dsi_cmd_panel_debug)
		display = get_sec_display();

	if (strlen(buf) >= SZ_512) {
		LCD_ERR("input buffer size[%d] is out of range[%d]\n",
				strlen(buf), SZ_512);
		return -ENOMEM;
	}

	if (!cmd_bufs) {
		cmd_bufs = kmalloc(SZ_4K, GFP_KERNEL);

		if (!cmd_bufs) {
			return -ENOMEM;
		}
	}

	sscanf(buf, "%s", data);

	if (!strcmp("dump", data)) {
		rc = oplus_display_dump_dsi_command(display);

		if (rc < 0) {
			return rc;
		}

		return count;

	} else if (!strcmp("flush", data)) {
		flush = true;

	} else if (!strcmp("dsi_hs_mode", data)) {
		oplus_dsi_state = DSI_CMD_SET_STATE_HS;

	} else if (!strcmp("dsi_lp_mode", data)) {
		oplus_dsi_state = DSI_CMD_SET_STATE_LP;

	} else {
		for (i = 0; i < DSI_CMD_SET_MAX; i++) {
			if (!strcmp(cmd_set_prop_map[i], data)) {
				oplus_dsi_command = i;
				flush = true;
				break;
			}
		}
	}

	if (!flush) {
		u32 value = 0, step = 0;

		while (sscanf(bufp, "%x%n", &value, &step) > 0) {
			if (value > 0xff) {
				LCD_ERR("input reg don't large than 0xff\n");
				return -EINVAL;
			}

			cmd_bufs[cmd_counts++] = value;

			if (cmd_counts >= SZ_4K) {
				LCD_ERR("wrong input reg len\n");
				cmd_counts = 0;
				return -EFAULT;
			}

			bufp += step;
		}

		return count;
	}

	if (!cmd_counts) {
		return rc;
	}

	if (old_dsi_command >= DSI_CMD_SET_MAX) {
		LCD_ERR("UnSupport dsi command set\n");
		goto error;
	}

	if (!display || !display->panel || !display->panel->cur_mode) {
		LCD_ERR("failed to get main dsi display\n");
		rc = -EFAULT;
		goto error;
	}

	mode = display->panel->cur_mode;

	if (!mode || !mode->priv_info) {
		LCD_ERR("failed to get dsi display mode\n");
		rc = -EFAULT;
		goto error;
	}

	priv_info = mode->priv_info;
	cmd_sets = priv_info->cmd_sets;

	cmd = &cmd_sets[old_dsi_command];

	rc = oplus_dsi_panel_get_cmd_pkt_count(cmd_bufs, cmd_counts,
			&packet_count);

	if (rc) {
		LCD_ERR("commands failed, rc=%d\n", rc);
		goto error;
	}

	size = packet_count * sizeof(*cmd->cmds);

	cmds = kzalloc(size, GFP_KERNEL);

	if (!cmds) {
		rc = -ENOMEM;
		goto error;
	}

	rc = oplus_dsi_panel_create_cmd_packets(cmd_bufs, cmd_counts,
			packet_count, cmds);

	if (rc) {
		LCD_ERR("failed to create cmd packets, rc=%d\n", rc);
		goto error_free_cmds;
	}

	mutex_lock(&display->panel->panel_lock);

	kfree(cmd->cmds);
	cmd->cmds = cmds;
	cmd->count = packet_count;

	if (oplus_dsi_state == DSI_CMD_SET_STATE_LP) {
		cmd->state = DSI_CMD_SET_STATE_LP;

	} else if (oplus_dsi_state == DSI_CMD_SET_STATE_HS) {
		cmd->state = DSI_CMD_SET_STATE_HS;
	}

	mutex_unlock(&display->panel->panel_lock);

	cmd_counts = 0;
	oplus_dsi_state = DSI_CMD_SET_STATE_HS;

	return count;

error_free_cmds:
	kfree(cmds);
error:
	cmd_counts = 0;
	oplus_dsi_state = DSI_CMD_SET_STATE_HS;

	return rc;
}

extern int oplus_panel_alpha;
int oplus_interpolate(int x, int xa, int xb, int ya, int yb)
{
	int bf, factor, plus;
	int sub = 0;

	bf = 2 * (yb - ya) * (x - xa) / (xb - xa);
	factor = bf / 2;
	plus = bf % 2;

	return ya + factor + plus + sub;
}
EXPORT_SYMBOL(oplus_interpolate);

static ssize_t oplus_display_get_dim_alpha(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	struct dsi_display *display = get_main_display();

	if (!display || !display->panel) {
		LCD_ERR("display is null\n");
		return -EINVAL;
	}

	if (display->panel->power_mode != SDE_MODE_DPMS_ON) {
		return sprintf(buf, "%d\n", 0);
	}

	return sprintf(buf, "%d\n", oplus_underbrightness_alpha);
}

static ssize_t oplus_display_set_dim_alpha(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	sscanf(buf, "%x", &oplus_panel_alpha);

	return count;
}

static ssize_t oplus_display_get_dc_dim_alpha(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

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

	return sprintf(buf, "%d\n", ret);
}

static ssize_t oplus_display_get_dimlayer_backlight(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d %d %d %d %d %d\n", oplus_dimlayer_bl_alpha,
			oplus_dimlayer_bl_alpha_value, oplus_dimlayer_dither_threshold,
			oplus_dimlayer_dither_bitdepth, oplus_dimlayer_bl_delay,
			oplus_dimlayer_bl_delay_after);
}

static ssize_t oplus_display_set_dimlayer_backlight(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	sscanf(buf, "%d %d %d %d %d %d", &oplus_dimlayer_bl_alpha,
			&oplus_dimlayer_bl_alpha_value, &oplus_dimlayer_dither_threshold,
			&oplus_dimlayer_dither_bitdepth, &oplus_dimlayer_bl_delay,
			&oplus_dimlayer_bl_delay_after);

	return count;
}

int oplus_dimlayer_bl_on_vblank = INT_MAX;
int oplus_dimlayer_bl_off_vblank = INT_MAX;
static int oplus_datadimming_v3_debug_value = -1;
static int oplus_datadimming_v3_debug_delay = 16000;
static ssize_t oplus_display_get_debug(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d %d %d %d %d\n", oplus_dimlayer_bl_on_vblank,
			oplus_dimlayer_bl_off_vblank, oplus_datadimming_v3_debug_value,
			oplus_datadimming_v3_debug_delay, dsi_cmd_panel_debug);
}

static ssize_t oplus_display_set_debug(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	sscanf(buf, "%d %d %d %d %d", &oplus_dimlayer_bl_on_vblank,
			&oplus_dimlayer_bl_off_vblank, &oplus_datadimming_v3_debug_value,
			&oplus_datadimming_v3_debug_delay, &dsi_cmd_panel_debug);

	return count;
}

static ssize_t oplus_display_get_dimlayer_enable(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d %d\n", oplus_dimlayer_bl_enable,
			oplus_dimlayer_bl_enable_v2);
}


int oplus_datadimming_vblank_count = 0;
atomic_t oplus_datadimming_vblank_ref = ATOMIC_INIT(0);
static int oplus_boe_data_dimming_process_unlock(int brightness, int enable)
{
	struct dsi_display *display = get_main_display();
	struct drm_connector *dsi_connector = display->drm_conn;
	struct dsi_panel *panel = display->panel;
	struct mipi_dsi_device *mipi_device;
	int rc = 0;

	if (!panel) {
		LCD_ERR("failed to find display panel\n");
		return -EINVAL;
	}

	if (!dsi_panel_initialized(panel)) {
		LCD_ERR("dsi panel is not init\n");
		return -EINVAL;
	}

	mipi_device = &panel->mipi_device;

	if (oplus_datadimming_vblank_count != 0) {
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
		if (oplus_ofp_is_supported() && !oplus_ofp_oled_capacitive_is_enabled()
				&& !oplus_ofp_ultrasonic_is_enabled()) {
			if (oplus_ofp_get_hbm_state()) {
				goto next;
			}
		}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

		drm_crtc_wait_one_vblank(dsi_connector->state->crtc);
		rc = mipi_dsi_dcs_set_display_brightness(mipi_device, brightness);
		drm_crtc_wait_one_vblank(dsi_connector->state->crtc);
	}

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
next:
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	/* enable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_ALL_CLKS, DSI_CLK_ON);
	}

	if (enable) {
		rc = dsi_panel_tx_cmd_set(display->panel, DSI_CMD_DATA_DIMMING_ON);
	} else {
		rc = dsi_panel_tx_cmd_set(display->panel, DSI_CMD_DATA_DIMMING_OFF);
	}

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_ALL_CLKS, DSI_CLK_OFF);
	}

	return rc;
}

struct oplus_brightness_alpha brightness_remapping[] = {
	{0, 0},
	{1, 1},
	{2, 155},
	{3, 160},
	{5, 165},
	{6, 170},
	{8, 175},
	{10, 180},
	{20, 190},
	{40, 230},
	{80, 270},
	{100, 300},
	{150, 400},
	{200, 600},
	{300, 800},
	{400, 1000},
	{600, 1250},
	{800, 1400},
	{1000, 1500},
	{1200, 1650},
	{1400, 1800},
	{1600, 1900},
	{1780, 2000},
	{2047, 2047},
};

int oplus_backlight_remapping(int brightness)
{
	struct oplus_brightness_alpha *lut = brightness_remapping;
	int count = ARRAY_SIZE(brightness_remapping);
	int i = 0;
	int bl_lvl = brightness;

	if (oplus_datadimming_v3_debug_value >= 0) {
		return oplus_datadimming_v3_debug_value;
	}

	for (i = 0; i < count; i++) {
		if (lut[i].brightness >= brightness) {
			break;
		}
	}

	if (i == 0) {
		bl_lvl = lut[0].alpha;
	} else if (i == count) {
		bl_lvl = lut[count - 1].alpha;
	} else {
		bl_lvl = oplus_interpolate(brightness, lut[i - 1].brightness,
				lut[i].brightness, lut[i - 1].alpha,
				lut[i].alpha);
	}

	return bl_lvl;
}

static bool dimming_enable = false;

int oplus_panel_process_dimming_v3_post(struct dsi_panel *panel, int brightness)
{
	bool enable = oplus_dimlayer_bl_enable_v3_real;
	int ret = 0;

	if (brightness <= 1) {
		enable = false;
	}

	if (enable != dimming_enable) {
		if (enable) {
			ret = oplus_boe_data_dimming_process_unlock(brightness, true);

			if (ret) {
				LCD_ERR("failed to enable data dimming\n");
				goto error;
			}

			dimming_enable = true;
			LCD_ERR("Enter DC backlight v3\n");

		} else {
			ret = oplus_boe_data_dimming_process_unlock(brightness, false);

			if (ret) {
				LCD_ERR("failed to enable data dimming\n");
				goto error;
			}

			dimming_enable = false;
			LCD_ERR("Exit DC backlight v3\n");
		}
	}

error:
	return 0;
}

static bool oplus_datadimming_v2_need_flush = false;
static bool oplus_datadimming_v2_need_sync = false;
void oplus_panel_process_dimming_v2_post(struct dsi_panel *panel,
		bool force_disable)
{
	struct dsi_display *display = get_main_display();
	struct drm_connector *dsi_connector = display->drm_conn;

	if (oplus_datadimming_v2_need_flush) {
		if (oplus_datadimming_v2_need_sync &&
				((!strcmp(panel->oplus_priv.vendor_name, "S6E3HC3")) ||
				(!strcmp(panel->oplus_priv.vendor_name, "S6E3HC4")) ||
				(!strcmp(panel->oplus_priv.vendor_name, "AMB670YF01"))) &&
				dsi_connector && dsi_connector->state && dsi_connector->state->crtc) {
			struct drm_crtc *crtc = dsi_connector->state->crtc;
			int frame_time_us, ret = 0;
			u32 current_vblank;

			frame_time_us = mult_frac(1000, 1000, panel->cur_mode->timing.refresh_rate);

			current_vblank = drm_crtc_vblank_count(crtc);
			ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
						 current_vblank != drm_crtc_vblank_count(crtc),
						 usecs_to_jiffies(frame_time_us + 1000));
			if (!ret) {
				LCD_ERR("crtc wait_event_timeout \n");
			}
		}

		if (display->config.panel_mode == DSI_OP_VIDEO_MODE) {
			panel->oplus_priv.skip_mipi_last_cmd = true;
		}

		dsi_panel_seed_mode_unlock(panel, seed_mode);

		if (display->config.panel_mode == DSI_OP_VIDEO_MODE) {
			panel->oplus_priv.skip_mipi_last_cmd = false;
		}

		oplus_datadimming_v2_need_flush = false;
	}
}
EXPORT_SYMBOL(oplus_panel_process_dimming_v2_post);

int oplus_panel_process_dimming_v2(struct dsi_panel *panel, int bl_lvl,
				  bool force_disable)
{
	struct dsi_display *display = get_main_display();
	struct drm_connector *dsi_connector = display->drm_conn;

	oplus_datadimming_v2_need_flush = false;
	oplus_datadimming_v2_need_sync = false;

	if (!force_disable && oplus_dimlayer_bl_enable_v2_real &&
			bl_lvl > 1 && bl_lvl < oplus_dimlayer_bl_alpha_v2) {
		if (!oplus_seed_backlight) {
			LCD_ERR("Enter DC backlight v2\n");

			if (!oplus_skip_datadimming_sync &&
					oplus_last_backlight != 0 &&
					oplus_last_backlight != 1) {
				oplus_datadimming_v2_need_sync = true;
			}
		}

		oplus_seed_backlight = bl_lvl;
		bl_lvl = oplus_dimlayer_bl_alpha_v2;
		oplus_datadimming_v2_need_flush = true;

	} else if (oplus_seed_backlight) {
		LCD_ERR("Exit DC backlight v2\n");
		oplus_seed_backlight = 0;
		oplus_dc2_alpha = 0;
		oplus_datadimming_v2_need_flush = true;
		oplus_datadimming_v2_need_sync = true;
	}

	if (oplus_datadimming_v2_need_flush) {
		if (oplus_datadimming_v2_need_sync &&
				((!strcmp(panel->oplus_priv.vendor_name, "S6E3HC3")) ||
				(!strcmp(panel->oplus_priv.vendor_name, "S6E3HC4")) ||
				(!strcmp(panel->oplus_priv.vendor_name, "AMB670YF01")))&&
				dsi_connector && dsi_connector->state && dsi_connector->state->crtc) {
			struct drm_crtc *crtc = dsi_connector->state->crtc;
			int frame_time_us, ret = 0;
			u32 current_vblank;

			frame_time_us = mult_frac(1000, 1000, panel->cur_mode->timing.refresh_rate);

			current_vblank = drm_crtc_vblank_count(crtc);
			ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
						 current_vblank != drm_crtc_vblank_count(crtc),
						 usecs_to_jiffies(frame_time_us + 1000));

			if (!ret) {
				LCD_ERR("crtc wait_event_timeout\n");
			}
		}
	}

	return bl_lvl;
}
EXPORT_SYMBOL(oplus_panel_process_dimming_v2);

int oplus_panel_process_dimming_v3(struct dsi_panel *panel, int brightness)
{
	bool enable = oplus_dimlayer_bl_enable_v3_real;
	int bl_lvl = brightness;

	if (enable) {
		bl_lvl = oplus_backlight_remapping(brightness);
	}

	oplus_panel_process_dimming_v3_post(panel, bl_lvl);
	return bl_lvl;
}
EXPORT_SYMBOL(oplus_panel_process_dimming_v3);

static ssize_t oplus_display_set_dimlayer_enable(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	struct dsi_display *display = get_main_display();
	struct drm_connector *dsi_connector = display->drm_conn;

	if (display && display->name) {
		int enable = 0;
		int err = 0;

		sscanf(buf, "%d", &enable);
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

		if ((!strcmp(display->panel->oplus_priv.vendor_name, "S6E3HC3")) ||
				(!strcmp(display->panel->oplus_priv.vendor_name, "S6E3HC4")) ||
				(!strcmp(display->panel->oplus_priv.vendor_name, "AMB670YF01")) ||
				!strcmp(display->panel->oplus_priv.vendor_name, "AMS643YE01")) {
			oplus_dimlayer_bl_enable_v2 = enable;

		} else {
			if (!strcmp(display->name,
					"qcom,mdss_dsi_oplus19101boe_nt37800_1080_2400_cmd")) {
				oplus_dimlayer_bl_enable_v3 = enable;

			} else {
				oplus_dimlayer_bl_enable = enable;
			}
		}

		mutex_unlock(&display->display_lock);
	}

	return count;
}

static ssize_t oplus_display_get_esd_status(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	struct dsi_display *display = get_main_display();
	int rc = 0;

	if (!display) {
		return -ENODEV;
	}

	mutex_lock(&display->display_lock);

	if (!display->panel) {
		rc = -EINVAL;
		goto error;
	}

	rc = sprintf(buf, "%d\n", display->panel->esd_config.esd_enabled);

error:
	mutex_unlock(&display->display_lock);
	return rc;
}

static ssize_t oplus_display_set_esd_status(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	struct dsi_display *display = get_main_display();
	int enable = 0;

	sscanf(buf, "%du", &enable);

	LCD_ERR("debug for oplus_display_set_esd_status, the enable value = %d\n",
			enable);

	if (!display) {
		return -ENODEV;
	}

	if (!display->panel || !display->drm_conn) {
		return -EINVAL;
	}

	if (!enable) {
		if (display->panel->esd_config.esd_enabled) {
			sde_connector_schedule_status_work(display->drm_conn, false);
			display->panel->esd_config.esd_enabled = false;
			LCD_ERR("disable esd work\n");
		}
	} else {
		if (!display->panel->esd_config.esd_enabled) {
			sde_connector_schedule_status_work(display->drm_conn, true);
			display->panel->esd_config.esd_enabled = true;
			LCD_ERR("enabled esd work\n");
		}
	}

	return count;
}

static ssize_t oplus_display_notify_panel_blank(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	struct dsi_display *display = get_main_display();
	int temp_save = 0;

	if (display == NULL) {
		LCD_ERR("display is null\n");
		return count;
	}

	sscanf(buf, "%du", &temp_save);
	LCD_INFO("oplus_display_notify_panel_blank = %d\n", temp_save);

	if (temp_save == 1) {
		oplus_event_data_notifier_trigger(DRM_PANEL_EVENT_UNBLANK, 0, true);
	} else if (temp_save == 0) {
		oplus_event_data_notifier_trigger(DRM_PANEL_EVENT_BLANK, 0, true);
	}

	return count;
}

extern int is_ffl_enable;
static ssize_t oplus_get_ffl_setting(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", is_ffl_enable);
}

static ssize_t oplus_set_ffl_setting(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int enable = 0;

	sscanf(buf, "%du", &enable);
	LCD_INFO("oplus_set_ffl_setting = %d\n", enable);

	oplus_ffl_set(enable);

	return count;
}

static ssize_t oplus_display_get_roundcorner(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	struct dsi_display *display = get_main_display();
	bool roundcorner = true;

	if (display && display->name &&
			!strcmp(display->name, "qcom,mdss_dsi_oplus19101boe_nt37800_1080_2400_cmd")) {
		roundcorner = false;
	}

	return sprintf(buf, "%d\n", roundcorner);
}

DEFINE_MUTEX(dynamic_osc_clock_lock);
int dynamic_osc_clock = 139600;

int dsi_update_dynamic_osc_clock(void)
{
	struct dsi_display *display = get_main_display();
	int rc = 0;
	int osc_clock_rate = dynamic_osc_clock;

	if (!display||!display->panel) {
		LCD_ERR("display is null\n");
		return -EINVAL;
	}

	if (!display->panel->oplus_priv.is_osc_support) {
		LCD_ERR("not support osc\n");
		return 0;
	}

	mutex_lock(&display->display_lock);
	mutex_lock(&display->panel->panel_lock);

	if (!dsi_panel_initialized(display->panel)) {
		rc = -EINVAL;
		goto unlock;
	}

	/* enable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_ON);
	}

	if (osc_clock_rate) {
		if (osc_clock_rate == display->panel->oplus_priv.osc_clk_mode0_rate) {
			rc = dsi_panel_tx_cmd_set(display->panel, DSI_CMD_OSC_CLK_MODEO0);
		} else if (osc_clock_rate == display->panel->oplus_priv.osc_clk_mode1_rate) {
			rc = dsi_panel_tx_cmd_set(display->panel, DSI_CMD_OSC_CLK_MODEO1);
		} else {
			LCD_ERR("unsupport osc clk rate=%d\n", osc_clock_rate);
		}
		if (rc)
			LCD_ERR("Failed to configure osc dynamic clk\n");
	} else {
		LCD_INFO("osc clk rate is 0, not config\n");
	}

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_OFF);
	}

unlock:
	mutex_unlock(&display->panel->panel_lock);
	mutex_unlock(&display->display_lock);

	return rc;
}

static ssize_t oplus_display_get_dynamic_osc_clock(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	struct dsi_display *display = get_main_display();
	int rc = 0;

	if (!display) {
		LCD_ERR("display is null\n");
		return -EINVAL;
	}

	mutex_lock(&display->display_lock);

	rc = snprintf(buf, PAGE_SIZE, "%d\n", dynamic_osc_clock);
	LCD_INFO("read dsi clk rate %d\n", dynamic_osc_clock);

	mutex_unlock(&display->display_lock);

	return rc;
}

static ssize_t oplus_display_set_dynamic_osc_clock(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	struct dsi_display *display = get_main_display();
	int osc_clk = 0;
	int rc = 0;

	if (!display || !display->panel) {
		LCD_ERR("display is null\n");
		return -EINVAL;
	}

	if(display->panel->oplus_priv.is_osc_support) {
		sscanf(buf, "%du", &osc_clk);
		dynamic_osc_clock = osc_clk;
		if (display->panel->panel_mode != DSI_OP_CMD_MODE) {
			LCD_ERR("only supported for command mode\n");
			return -EFAULT;
		}

		LCD_INFO("osc clk param value: '%d'\n", osc_clk);
		dsi_update_dynamic_osc_clock();
		return count;
	}

	/* if (__oplus_get_power_status() != OPLUS_DISPLAY_POWER_ON) { */
	if (display->panel->power_mode != SDE_MODE_DPMS_ON) {
		LCD_INFO("display panel in off status\n");
		return -EFAULT;
	}

	sscanf(buf, "%du", &osc_clk);

	if (display->panel->panel_mode != DSI_OP_CMD_MODE) {
		LCD_ERR("only supported for command mode\n");
		return -EFAULT;
	}

	LCD_INFO("osc clk param value: '%d'\n", osc_clk);

	mutex_lock(&display->display_lock);
	mutex_lock(&display->panel->panel_lock);

	if (!dsi_panel_initialized(display->panel)) {
		rc = -EINVAL;
		goto unlock;
	}

	/* enable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_ON);
	}

	if (osc_clk == 139600) {
		rc = dsi_panel_tx_cmd_set(display->panel, DSI_CMD_OSC_CLK_MODEO0);

	} else {
		rc = dsi_panel_tx_cmd_set(display->panel, DSI_CMD_OSC_CLK_MODEO1);
	}

	if (rc) {
		LCD_ERR("Failed to configure osc dynamic clk\n");

	} else {
		rc = count;
	}

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_OFF);
	}

	dynamic_osc_clock = osc_clk;

unlock:
	mutex_unlock(&display->panel->panel_lock);
	mutex_unlock(&display->display_lock);

	return count;
}

static ssize_t oplus_display_get_max_brightness_show(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	struct dsi_display *display = get_main_display();

	if (!display) {
		LCD_ERR("display is null\n");
		return -EINVAL;
	}

	if(oplus_debug_max_brightness == 0) {
		return sprintf(buf, "%d\n", display->panel->bl_config.brightness_normal_max_level);
	} else {
		return sprintf(buf, "%d\n", oplus_debug_max_brightness);
	}
}

static ssize_t oplus_display_set_max_brightness_store(struct kobject *obj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%du", &oplus_debug_max_brightness);
	return count;
}

static ssize_t oplus_display_get_ccd_check(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	int ccd_check = 0;
	return sprintf(buf, "%d\n", ccd_check);
}

int oplus_display_set_power(struct drm_connector *connector,
		int power_mode, void *disp)
{
	struct dsi_display *display = disp;
	int rc = 0;
	bool notify_off = false;

	if (!display || !display->panel) {
		LCD_ERR("display is null\n");
		return -EINVAL;
	}

	if (power_mode == SDE_MODE_DPMS_OFF)
		atomic_set(&display->panel->esd_pending, 1);

	switch (power_mode) {
	case SDE_MODE_DPMS_LP1:
	case SDE_MODE_DPMS_LP2:
		if (power_mode == SDE_MODE_DPMS_LP1 &&
				display->panel->power_mode == SDE_MODE_DPMS_ON) {
			notify_off = true;
		}

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
		if (oplus_ofp_is_supported()) {
			oplus_ofp_power_mode_handle(display, power_mode);
		}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

		if (notify_off) {
			oplus_panel_event_data_notifier_trigger(display->panel,
					DRM_PANEL_EVENT_BLANK, 0, true);
		}

		__oplus_set_power_status(OPLUS_DISPLAY_POWER_DOZE_SUSPEND);
		break;

	case SDE_MODE_DPMS_ON:
		LCD_INFO("SDE_MODE_DPMS_ON\n");

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
		if (oplus_ofp_is_supported()) {
			oplus_ofp_power_mode_handle(display, SDE_MODE_DPMS_ON);
		}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

		__oplus_set_power_status(OPLUS_DISPLAY_POWER_ON);
		oplus_panel_event_data_notifier_trigger(display->panel,
				DRM_PANEL_EVENT_UNBLANK, 0, true);
		break;

	case SDE_MODE_DPMS_OFF:
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
		if (oplus_ofp_is_supported()) {
			oplus_ofp_power_mode_handle(display, SDE_MODE_DPMS_OFF);
		}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

		LCD_INFO("SDE_MODE_DPMS_OFF\n");
		break;
	default:
		return rc;
	}

	LCD_DEBUG("Power mode transition from %d to %d %s\n",
		display->panel->power_mode, power_mode,
		rc ? "failed" : "successful");

	if (!rc) {
		display->panel->power_mode = power_mode;
	}

	return rc;
}
EXPORT_SYMBOL(oplus_display_set_power);

static int oplus_display_find_vreg_by_name(const char *name)
{
	int count = 0, i = 0;
	struct dsi_vreg *vreg = NULL;
	struct dsi_regulator_info *dsi_reg = NULL;
	struct dsi_display *display = get_main_display();

	if (!display) {
		return -ENODEV;
	}

	if (!display->panel) {
		return -EINVAL;
	}

	dsi_reg = &display->panel->power_info;
	count = dsi_reg->count;

	for (i = 0; i < count; i++) {
		vreg = &dsi_reg->vregs[i];
		LCD_INFO("finding vreg: %s\n", vreg->vreg_name);

		if (!strcmp(vreg->vreg_name, name)) {
			LCD_INFO("find the vreg: %s\n", name);
			return i;

		} else {
			continue;
		}
	}

	LCD_ERR("dose not find the vreg: %s\n", name);

	return -EINVAL;
}

static u32 update_current_voltage(u32 id)
{
	int vol_current = 0, pwr_id = 0;
	struct dsi_vreg *dsi_reg = NULL;
	struct dsi_regulator_info *dsi_reg_info = NULL;
	struct dsi_display *display = get_main_display();

	if (!display) {
		return -ENODEV;
	}

	if (!display->panel || !display->drm_conn) {
		return -EINVAL;
	}

	dsi_reg_info = &display->panel->power_info;
	pwr_id = oplus_display_find_vreg_by_name(panel_vol_bak[id].pwr_name);

	if (pwr_id < 0) {
		LCD_ERR("can't find the pwr_id, please check the vreg name\n");
		return pwr_id;
	}

	dsi_reg = &dsi_reg_info->vregs[pwr_id];

	if (!dsi_reg) {
		return -EINVAL;
	}

	vol_current = regulator_get_voltage(dsi_reg->vreg);

	return vol_current;
}

static ssize_t oplus_display_get_panel_pwr(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	u32 ret = 0;
	u32 i = 0;

	for (i = 0; i < (PANEL_VOLTAGE_ID_MAX - 1); i++) {
		ret = update_current_voltage(panel_vol_bak[i].voltage_id);

		if (ret < 0) {
			LCD_ERR("update_current_voltage error = %d\n", ret);

		} else {
			panel_vol_bak[i].voltage_current = ret;
		}
	}

	return sprintf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d\n",
			panel_vol_bak[0].voltage_id, panel_vol_bak[0].voltage_min,
			panel_vol_bak[0].voltage_current, panel_vol_bak[0].voltage_max,
			panel_vol_bak[1].voltage_id, panel_vol_bak[1].voltage_min,
			panel_vol_bak[1].voltage_current, panel_vol_bak[1].voltage_max,
			panel_vol_bak[2].voltage_id, panel_vol_bak[2].voltage_min,
			panel_vol_bak[2].voltage_current, panel_vol_bak[2].voltage_max);
}

static ssize_t oplus_display_set_panel_pwr(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	u32 panel_vol_value = 0, rc = 0, panel_vol_id = 0, pwr_id = 0;
	struct dsi_vreg *dsi_reg = NULL;
	struct dsi_regulator_info *dsi_reg_info = NULL;
	struct dsi_display *display = get_main_display();

	sscanf(buf, "%d %d", &panel_vol_id, &panel_vol_value);
	panel_vol_id = panel_vol_id & 0x0F;

	LCD_INFO("buf = [%s], id = %d value = %d, count = %d\n",
			buf, panel_vol_id, panel_vol_value, count);

	if (panel_vol_id < 0 || panel_vol_id > PANEL_VOLTAGE_ID_MAX) {
		return -EINVAL;
	}

	if (panel_vol_value < panel_vol_bak[panel_vol_id].voltage_min ||
			panel_vol_id > panel_vol_bak[panel_vol_id].voltage_max) {
		return -EINVAL;
	}

	if (!display) {
		return -ENODEV;
	}

	if (!display->panel || !display->drm_conn) {
		return -EINVAL;
	}

	if (panel_vol_id == PANEL_VOLTAGE_ID_VG_BASE) {
		LCD_INFO("set the VGH_L pwr = %d \n", panel_vol_value);
		panel_pwr_vg_base = panel_vol_value;
		return count;
	}

	dsi_reg_info = &display->panel->power_info;

	pwr_id = oplus_display_find_vreg_by_name(panel_vol_bak[panel_vol_id].pwr_name);

	if (pwr_id < 0) {
		LCD_ERR("can't find the vreg name, please re-check vreg name: %s \n",
				panel_vol_bak[panel_vol_id].pwr_name);
		return pwr_id;
	}

	dsi_reg = &dsi_reg_info->vregs[pwr_id];

	rc = regulator_set_voltage(dsi_reg->vreg, panel_vol_value, panel_vol_value);

	if (rc) {
		LCD_ERR("Set voltage(%s) fail, rc=%d\n",
				dsi_reg->vreg_name, rc);
		return -EINVAL;
	}

	return count;
}

static ssize_t oplus_display_get_dsi_log_switch(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "\
		dynamic conctrl debug log, 0x0 --> disable all debug log\n \
		1 -> enable  0-> disable\n \
		BIT(0) --> dump register log\n \
		BIT(1) --> backlight log\n \
		BIT(2) --> common log\n \
		BIT(3) --> ofp log\n \
		BIT(4) --> vrr log\n \
		BIT(5) --> lcd log\n \
		current value: 0x%X\n", oplus_dsi_log_type);
}

static ssize_t oplus_display_set_dsi_log_switch(struct kobject *obj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%x", &oplus_dsi_log_type);
	LCD_INFO("buf = [%s], oplus_dsi_log_type = 0x%X , count = %d\n",
			buf, oplus_dsi_log_type, count);

	return count;
}

static ssize_t oplus_display_get_trace_enable_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	if (!buf) {
		LCD_ERR("Invalid params\n");
		return -EINVAL;
	}

	return sprintf(buf, "dynamic trace enable\n \
		0x0 --> disable all trace\n \
		BIT(0) --> enable ofp trace\n \
		BIT(1) --> enable vrr trace\n \
		BIT(2) --> enable lcd trace\n \
		current value: 0x%X\n", oplus_display_trace_enable);
}

static ssize_t oplus_display_set_trace_enable_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	if (!buf) {
		LCD_ERR("Invalid params\n");
		return count;
	}

	sscanf(buf, "%x", &oplus_display_trace_enable);
	LCD_INFO("buf = [%s], oplus_display_trace_enable = 0x%X , count = %d\n",
			buf, oplus_display_trace_enable, count);

	return count;
}
static struct kobject *oplus_display_kobj;

static OPLUS_ATTR(audio_ready, S_IRUGO | S_IWUSR, NULL,
		oplus_display_set_audio_ready);
static OPLUS_ATTR(seed, S_IRUGO | S_IWUSR, oplus_display_get_seed,
		oplus_display_set_seed);
static OPLUS_ATTR(panel_serial_number, S_IRUGO | S_IWUSR,
		oplus_display_get_panel_serial_number, NULL);
static OPLUS_ATTR(dump_info, S_IRUGO | S_IWUSR, oplus_display_dump_info, NULL);
static OPLUS_ATTR(panel_dsc, S_IRUGO | S_IWUSR, oplus_display_get_panel_dsc,
		NULL);
static OPLUS_ATTR(power_status, S_IRUGO | S_IWUSR,
		oplus_display_get_power_status, oplus_display_set_power_status);
static OPLUS_ATTR(display_regulator_control, S_IRUGO | S_IWUSR, NULL,
		oplus_display_regulator_control);
static OPLUS_ATTR(panel_id, S_IRUGO | S_IWUSR, oplus_display_get_panel_id,
		NULL);
static OPLUS_ATTR(sau_closebl_node, S_IRUGO | S_IWUSR,
		oplus_display_get_closebl_flag, oplus_display_set_closebl_flag);
static OPLUS_ATTR(write_panel_reg, S_IRUGO | S_IWUSR,
		oplus_display_get_panel_reg, oplus_display_set_panel_reg);
static OPLUS_ATTR(dsi_cmd, S_IRUGO | S_IWUSR, oplus_display_get_dsi_command,
		oplus_display_set_dsi_command);
static OPLUS_ATTR(dim_alpha, S_IRUGO | S_IWUSR, oplus_display_get_dim_alpha,
		oplus_display_set_dim_alpha);
static OPLUS_ATTR(dim_dc_alpha, S_IRUGO | S_IWUSR,
		oplus_display_get_dc_dim_alpha, oplus_display_set_dim_alpha);
static OPLUS_ATTR(dimlayer_bl_en, S_IRUGO | S_IWUSR,
		oplus_display_get_dimlayer_enable, oplus_display_set_dimlayer_enable);
static OPLUS_ATTR(dimlayer_set_bl, S_IRUGO | S_IWUSR,
		oplus_display_get_dimlayer_backlight, oplus_display_set_dimlayer_backlight);
static OPLUS_ATTR(debug, S_IRUGO | S_IWUSR, oplus_display_get_debug,
		oplus_display_set_debug);
static OPLUS_ATTR(esd_status, S_IRUGO | S_IWUSR, oplus_display_get_esd_status,
		oplus_display_set_esd_status);
static OPLUS_ATTR(notify_panel_blank, S_IRUGO | S_IWUSR, NULL,
		oplus_display_notify_panel_blank);
static OPLUS_ATTR(ffl_set, S_IRUGO | S_IWUSR, oplus_get_ffl_setting,
		oplus_set_ffl_setting);
static OPLUS_ATTR(spr, S_IRUGO | S_IWUSR, oplus_display_get_spr,
		oplus_display_set_spr);
static OPLUS_ATTR(dither, S_IRUGO | S_IWUSR, oplus_display_get_dither,
		oplus_display_set_dither);
static OPLUS_ATTR(roundcorner, S_IRUGO | S_IRUSR, oplus_display_get_roundcorner,
		NULL);
static OPLUS_ATTR(dynamic_osc_clock, S_IRUGO | S_IWUSR,
		oplus_display_get_dynamic_osc_clock, oplus_display_set_dynamic_osc_clock);
static OPLUS_ATTR(max_brightness, S_IRUGO | S_IWUSR,
		oplus_display_get_max_brightness_show, oplus_display_set_max_brightness_store);
static OPLUS_ATTR(ccd_check, S_IRUGO | S_IRUSR, oplus_display_get_ccd_check,
		NULL);
static OPLUS_ATTR(iris_rm_check, S_IRUGO | S_IWUSR,
		oplus_display_get_iris_state, NULL);
static OPLUS_ATTR(panel_pwr, S_IRUGO | S_IWUSR, oplus_display_get_panel_pwr,
		oplus_display_set_panel_pwr);
static OPLUS_ATTR(dsi_log_switch, S_IRUGO | S_IWUSR, oplus_display_get_dsi_log_switch,
		oplus_display_set_dsi_log_switch);
static OPLUS_ATTR(trace_enable, S_IRUGO | S_IWUSR, oplus_display_get_trace_enable_attr, oplus_display_set_trace_enable_attr);
/* OPLUS_FEATURE_ADFR, oplus adfr */
static OPLUS_ATTR(adfr_debug, S_IRUGO|S_IWUSR, oplus_adfr_get_debug, oplus_adfr_set_debug);
static OPLUS_ATTR(vsync_switch, S_IRUGO|S_IWUSR, oplus_get_vsync_switch, oplus_set_vsync_switch);
/* dynamic te detect */
static OPLUS_ATTR(dynamic_te, S_IRUGO|S_IWUSR, oplus_adfr_get_dynamic_te, oplus_adfr_set_dynamic_te);

static OPLUS_ATTR(backlight_smooth, S_IRUGO|S_IWUSR, oplus_backlight_smooth_get_debug,
		oplus_backlight_smooth_set_debug);
static OPLUS_ATTR(pwm_turbo, S_IRUGO|S_IWUSR, oplus_get_pwm_turbo_debug,
		oplus_set_pwm_turbo_debug);
static OPLUS_ATTR(ffc_mode, S_IRUGO|S_IWUSR, oplus_get_ffc_mode_debug,
		oplus_set_ffc_mode_debug);
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
static OPLUS_ATTR(fp_type, S_IRUGO | S_IWUSR, oplus_ofp_get_fp_type_attr, oplus_ofp_set_fp_type_attr);
static OPLUS_ATTR(hbm, S_IRUGO | S_IWUSR, oplus_ofp_get_hbm_attr, oplus_ofp_set_hbm_attr);
static OPLUS_ATTR(aor, S_IRUGO | S_IWUSR, oplus_ofp_get_aor_attr, oplus_ofp_set_aor_attr);
static OPLUS_ATTR(dimlayer_hbm, S_IRUGO | S_IWUSR, oplus_ofp_get_dimlayer_hbm_attr, oplus_ofp_set_dimlayer_hbm_attr);
static OPLUS_ATTR(notify_fppress, S_IRUGO | S_IWUSR, NULL, oplus_ofp_notify_fp_press_attr);
static OPLUS_ATTR(aod_light_mode_set, S_IRUGO | S_IWUSR, oplus_ofp_get_aod_light_mode_attr, oplus_ofp_set_aod_light_mode_attr);
static OPLUS_ATTR(ultra_low_power_aod_mode, S_IRUGO | S_IWUSR, oplus_ofp_get_ultra_low_power_aod_mode_attr, oplus_ofp_set_ultra_low_power_aod_mode_attr);
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

/*
 * Create a group of attributes so that we can create and destroy them all
 * at once.
 */
static struct attribute *oplus_display_attrs[] = {
	&oplus_attr_audio_ready.attr,
	&oplus_attr_seed.attr,
	&oplus_attr_panel_serial_number.attr,
	&oplus_attr_dump_info.attr,
	&oplus_attr_panel_dsc.attr,
	&oplus_attr_power_status.attr,
	&oplus_attr_display_regulator_control.attr,
	&oplus_attr_panel_id.attr,
	&oplus_attr_sau_closebl_node.attr,
	&oplus_attr_write_panel_reg.attr,
	&oplus_attr_dsi_cmd.attr,
	&oplus_attr_dim_alpha.attr,
	&oplus_attr_dim_dc_alpha.attr,
	&oplus_attr_dimlayer_set_bl.attr,
	&oplus_attr_dimlayer_bl_en.attr,
	&oplus_attr_debug.attr,
	&oplus_attr_esd_status.attr,
	&oplus_attr_notify_panel_blank.attr,
	&oplus_attr_ffl_set.attr,
	&oplus_attr_spr.attr,
	&oplus_attr_roundcorner.attr,
	&oplus_attr_dynamic_osc_clock.attr,
	&oplus_attr_max_brightness.attr,
	&oplus_attr_ccd_check.attr,
	&oplus_attr_iris_rm_check.attr,
	&oplus_attr_panel_pwr.attr,
	/* OPLUS_FEATURE_ADFR, oplus adfr */
	&oplus_attr_adfr_debug.attr,
	&oplus_attr_vsync_switch.attr,
	/* OPLUS_FEATURE_ADFR, dynamic te detect */
	&oplus_attr_dynamic_te.attr,
	&oplus_attr_backlight_smooth.attr,
	&oplus_attr_dsi_log_switch.attr,
	&oplus_attr_dither.attr,
	&oplus_attr_trace_enable.attr,
	&oplus_attr_pwm_turbo.attr,
	&oplus_attr_ffc_mode.attr,
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	&oplus_attr_fp_type.attr,
	&oplus_attr_hbm.attr,
	&oplus_attr_aor.attr,
	&oplus_attr_dimlayer_hbm.attr,
	&oplus_attr_notify_fppress.attr,
	&oplus_attr_aod_light_mode_set.attr,
	&oplus_attr_ultra_low_power_aod_mode.attr,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group oplus_display_attr_group = {
	.attrs = oplus_display_attrs,
};

/*
 * Create a new API to get display resolution
 */
int oplus_display_get_resolution(unsigned int *xres, unsigned int *yres)
{
	*xres = *yres = 0;

	if (get_main_display() && get_main_display()->modes) {
		*xres = get_main_display()->modes->timing.v_active;
		*yres = get_main_display()->modes->timing.h_active;
	}

	return 0;
}
EXPORT_SYMBOL(oplus_display_get_resolution);

int oplus_display_private_api_init(void)
{
	struct dsi_display *display = get_main_display();
	int retval;

	if (!display) {
		return -EPROBE_DEFER;
	}

	oplus_display_kobj = kobject_create_and_add("oplus_display", kernel_kobj);

	if (!oplus_display_kobj) {
		return -ENOMEM;
	}

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(oplus_display_kobj, &oplus_display_attr_group);

	if (retval) {
		goto error_remove_kobj;
	}

	retval = sysfs_create_link(oplus_display_kobj,
			&display->pdev->dev.kobj, "panel");

	if (retval) {
		goto error_remove_sysfs_group;
	}

	return 0;

error_remove_sysfs_group:
	sysfs_remove_group(oplus_display_kobj, &oplus_display_attr_group);
error_remove_kobj:
	kobject_put(oplus_display_kobj);
	oplus_display_kobj = NULL;

	return retval;
}

void  oplus_display_private_api_exit(void)
{
	sysfs_remove_link(oplus_display_kobj, "panel");
	sysfs_remove_group(oplus_display_kobj, &oplus_display_attr_group);
	kobject_put(oplus_display_kobj);
}
