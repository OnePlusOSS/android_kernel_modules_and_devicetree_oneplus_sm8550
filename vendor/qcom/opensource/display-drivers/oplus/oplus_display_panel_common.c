/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
** File : oplus_display_panel_common.c
** Description : oplus display panel common feature
** Version : 1.0
** Date : 2020/06/13
** Author : Display
******************************************************************/
#include "oplus_display_panel_common.h"
#include "oplus_display_panel.h"
#include "oplus_display_panel_seed.h"
#include <linux/notifier.h>
#include <linux/msm_drm_notify.h>
#include <linux/soc/qcom/panel_event_notifier.h>
#include "oplus_display_private_api.h"
#include "oplus_display_interface.h"

#if defined(CONFIG_PXLW_IRIS)
#include "../msm/dsi/iris/dsi_iris_loop_back.h"
#endif

#if defined(CONFIG_PXLW_IRIS)
#include "dsi_iris_api.h"
#endif

#define DSI_PANEL_OPLUS_DUMMY_VENDOR_NAME  "PanelVendorDummy"
#define DSI_PANEL_OPLUS_DUMMY_MANUFACTURE_NAME  "dummy1024"

int oplus_debug_max_brightness = 0;
int oplus_dither_enable = 0;
int oplus_dre_status = 0;
int oplus_cabc_status = OPLUS_DISPLAY_CABC_UI;
extern int lcd_closebl_flag;
extern int oplus_display_audio_ready;
char oplus_rx_reg[PANEL_TX_MAX_BUF] = {0x0};
char oplus_rx_len = 0;
extern int spr_mode;
extern int dynamic_osc_clock;
extern int oplus_hw_partial_round;
int mca_mode = 1;
bool apollo_backlight_enable = false;
extern int dither_enable;
bool oplus_enhance_mipi_strength = false;
EXPORT_SYMBOL(oplus_enhance_mipi_strength);
EXPORT_SYMBOL(oplus_debug_max_brightness);
EXPORT_SYMBOL(oplus_dither_enable);

extern int dsi_display_read_panel_reg(struct dsi_display *display, u8 cmd,
		void *data, size_t len);
extern int __oplus_display_set_spr(int mode);
extern int dsi_display_spr_mode(struct dsi_display *display, int mode);
extern int dsi_panel_spr_mode(struct dsi_panel *panel, int mode);
extern int __oplus_display_set_dither(int mode);
enum {
	REG_WRITE = 0,
	REG_READ,
	REG_X,
};

int oplus_display_panel_get_id(void *buf)
{
	struct dsi_display *display = get_main_display();
	int ret = 0;
	unsigned char read[30];
	struct panel_id *panel_rid = buf;

	if (!display || !display->panel) {
		printk(KERN_INFO "oplus_display_get_panel_id and main display is null");
		ret = -1;
		return ret;
	}
	/* if (get_oplus_display_power_status() == OPLUS_DISPLAY_POWER_ON) { */
	if (display->panel->power_mode == SDE_MODE_DPMS_ON) {
		if (!strcmp(display->panel->oplus_priv.vendor_name, "NT37290")) {
			char value[] = {0x55, 0xAA, 0x52, 0x08, 0x03};
			ret = mipi_dsi_dcs_write(&display->panel->mipi_device, 0xF0, value, sizeof(value));
			ret = dsi_display_read_panel_reg(display, 0xDF, read, 8);
			if (ret < 0) {
				pr_err("failed to read DA ret=%d\n", ret);
				return -EINVAL;
			}
			panel_rid->DA = (uint32_t)read[4];
			panel_rid->DB = (uint32_t)read[5];
			panel_rid->DC = (uint32_t)read[6];
		} else {
			ret = dsi_display_read_panel_reg(display, 0xDA, read, 1);

			if (ret < 0) {
				pr_err("failed to read DA ret=%d\n", ret);
				return -EINVAL;
			}

			panel_rid->DA = (uint32_t)read[0];

			ret = dsi_display_read_panel_reg(display, 0xDB, read, 1);

			if (ret < 0) {
				pr_err("failed to read DB ret=%d\n", ret);
				return -EINVAL;
			}

			panel_rid->DB = (uint32_t)read[0];

			ret = dsi_display_read_panel_reg(display, 0xDC, read, 1);

			if (ret < 0) {
				pr_err("failed to read DC ret=%d\n", ret);
				return -EINVAL;
			}

			panel_rid->DC = (uint32_t)read[0];
		}
	} else {
		printk(KERN_ERR
				"%s oplus_display_get_panel_id, but now display panel status is not on\n",
				__func__);
		return -EINVAL;
	}

	return ret;
}

int oplus_display_panel_get_oplus_max_brightness(void *buf)
{
	uint32_t *max_brightness = buf;
	int panel_id = (*max_brightness >> 12);
	struct dsi_display *display = get_main_display();
	if (panel_id == 1)
		display = get_sec_display();

	(*max_brightness) = display->panel->bl_config.bl_normal_max_level;

	return 0;
}

int oplus_display_panel_get_max_brightness(void *buf)
{
	uint32_t *max_brightness = buf;
	int panel_id = (*max_brightness >> 12);
	struct dsi_display *display = get_main_display();
	if (panel_id == 1)
		display = get_sec_display();

	if (oplus_debug_max_brightness == 0) {
		(*max_brightness) = display->panel->bl_config.bl_normal_max_level;
	} else {
		(*max_brightness) = oplus_debug_max_brightness;
	}

	return 0;
}

int oplus_display_panel_set_max_brightness(void *buf)
{
	uint32_t *max_brightness = buf;

	oplus_debug_max_brightness = (*max_brightness);

	return 0;
}

int oplus_display_panel_get_brightness(void *buf)
{
	uint32_t *brightness = buf;
	int panel_id = (*brightness >> 12);
	struct dsi_display *display = get_main_display();
	if (panel_id == 1)
		display = get_sec_display();

	if(!strcmp(display->panel->oplus_priv.vendor_name, "AMS643YE01")) {
		(*brightness) = display->panel->bl_config.oplus_raw_bl;
	} else {
		(*brightness) = display->panel->bl_config.bl_level;
	}
	return 0;
}

int oplus_display_panel_get_vendor(void *buf)
{
	struct panel_info *p_info = buf;
	struct dsi_display *display = NULL;
	char *vendor = NULL;
	char *manu_name = NULL;
	int panel_id = p_info->version[0];

	display = get_main_display();
	if (1 == panel_id)
		display = get_sec_display();

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(display->panel->type, "secondary"))) {
		pr_info("%s: iris secondary panel no need config!\n", __func__);
		return 0;
	}
#endif

	if (!display || !display->panel ||
			!display->panel->oplus_priv.vendor_name ||
			!display->panel->oplus_priv.manufacture_name) {
		pr_err("failed to config lcd proc device");
		return -EINVAL;
	}

	vendor = (char *)display->panel->oplus_priv.vendor_name;
	manu_name = (char *)display->panel->oplus_priv.manufacture_name;

	memcpy(p_info->version, vendor,
			strlen(vendor) > 31 ? 31 : (strlen(vendor) + 1));
	memcpy(p_info->manufacture, manu_name,
			strlen(manu_name) > 31 ? 31 : (strlen(manu_name) + 1));

	return 0;
}

int oplus_display_panel_get_ccd_check(void *buf)
{
	struct dsi_display *display = get_main_display();
	struct mipi_dsi_device *mipi_device;
	int rc = 0;
	unsigned int *ccd_check = buf;
	char value[] = { 0x5A, 0x5A };
	char value1[] = { 0x44, 0x50 };
	char value2[] = { 0x03 };
	char value3[] = { 0x5A, 0x5A };
	char value4[] = { 0x02 };
	char value5[] = { 0x44, 0x50 };
	char value6[] = { 0x05 };
	char value7[] = { 0xA5, 0xA5 };
	unsigned char read[10];
	unsigned char read1[10];

	if (!display || !display->panel) {
		pr_err("failed for: %s %d\n", __func__, __LINE__);
		return -EINVAL;
	}

	/* if (get_oplus_display_power_status() != OPLUS_DISPLAY_POWER_ON) { */
	if (display->panel->power_mode != SDE_MODE_DPMS_ON) {
		printk(KERN_ERR"%s display panel in off status\n", __func__);
		return -EFAULT;
	}

	if (display->panel->panel_mode != DSI_OP_CMD_MODE) {
		pr_err("only supported for command mode\n");
		return -EFAULT;
	}

	if (!(display && display->panel->oplus_priv.vendor_name) ||
			!strcmp(display->panel->oplus_priv.vendor_name, "NT37800")) {
		(*ccd_check) = 0;
		goto end;
	}

	mipi_device = &display->panel->mipi_device;

	mutex_lock(&display->display_lock);
	mutex_lock(&display->panel->panel_lock);

	if (!dsi_panel_initialized(display->panel)) {
		rc = -EINVAL;
		goto unlock;
	}

	rc = dsi_display_cmd_engine_enable(display);

	if (rc) {
		pr_err("%s, cmd engine enable failed\n", __func__);
		goto unlock;
	}

	/* enable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_ON);
	}

	if (!strcmp(display->panel->oplus_priv.vendor_name, "AMB655UV01")) {
		rc = mipi_dsi_dcs_write(mipi_device, 0xF0, value, sizeof(value));
		rc = mipi_dsi_dcs_write(mipi_device, 0xE7, value1, sizeof(value1));
		usleep_range(1000, 1100);
		rc = mipi_dsi_dcs_write(mipi_device, 0xB0, value2, sizeof(value2));
	} else {
		rc = mipi_dsi_dcs_write(mipi_device, 0xF0, value3, sizeof(value3));
		rc = mipi_dsi_dcs_write(mipi_device, 0xB0, value4, sizeof(value4));
		rc = mipi_dsi_dcs_write(mipi_device, 0xCC, value5, sizeof(value5));
		usleep_range(1000, 1100);
		rc = mipi_dsi_dcs_write(mipi_device, 0xB0, value6, sizeof(value6));
	}

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_OFF);
	}

	dsi_display_cmd_engine_disable(display);

	mutex_unlock(&display->panel->panel_lock);
	mutex_unlock(&display->display_lock);

	if (!strcmp(display->panel->oplus_priv.vendor_name, "AMB655UV01")) {
			rc = dsi_display_read_panel_reg(display, 0xE1, read, 1);
			pr_err("read ccd_check value = 0x%x rc=%d\n", read[0], rc);
			(*ccd_check) = read[0];
	} else {
			rc = dsi_display_read_panel_reg(display, 0xCC, read1, 1);
			pr_err("read ccd_check value = 0x%x rc=%d\n", read1[0], rc);
			(*ccd_check) = read1[0];
	}

	mutex_lock(&display->display_lock);
	mutex_lock(&display->panel->panel_lock);

	if (!dsi_panel_initialized(display->panel)) {
		rc = -EINVAL;
		goto unlock;
	}

	rc = dsi_display_cmd_engine_enable(display);

	if (rc) {
		pr_err("%s, cmd engine enable failed\n", __func__);
		goto unlock;
	}

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_ON);
	}

	rc = mipi_dsi_dcs_write(mipi_device, 0xF0, value7, sizeof(value7));

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_OFF);
	}

	dsi_display_cmd_engine_disable(display);
unlock:

	mutex_unlock(&display->panel->panel_lock);
	mutex_unlock(&display->display_lock);
end:
	pr_err("[%s] ccd_check = %d\n",  display->panel->oplus_priv.vendor_name,
			(*ccd_check));
	return 0;
}

int oplus_display_panel_get_serial_number(void *buf)
{
	int ret = 0, i;
	unsigned char read[30] = {0};
	PANEL_SERIAL_INFO panel_serial_info;
	uint64_t serial_number;
	struct panel_serial_number *panel_rnum = buf;
	struct dsi_display *display = get_main_display();
	int panel_id = panel_rnum->serial_number[0];

	if (!display || !display->panel) {
		printk(KERN_INFO
				"oplus_display_get_panel_serial_number and main display is null");
		return -1;
	}

	if (0 == panel_id && display->enabled == false) {
		pr_err("%s main panel is disabled", __func__);
		return -1;
	}

	if (1 == panel_id) {
		display = get_sec_display();
		if (!display) {
			printk(KERN_INFO "oplus_display_get_panel_serial_number and main display is null");
			return -1;
		}
		if (display->enabled == false) {
			pr_err("%s second panel is disabled", __func__);
			return -1;
		}
	}

	/* if (get_oplus_display_power_status() != OPLUS_DISPLAY_POWER_ON) { */
	if (display->panel->power_mode != SDE_MODE_DPMS_ON) {
		printk(KERN_ERR"%s display panel in off status\n", __func__);
		return ret;
	}

	if (!display->panel->panel_initialized) {
		printk(KERN_ERR"%s  panel initialized = false\n", __func__);
		return ret;
	}

	if (!display->panel->oplus_ser.serial_number_support) {
		printk(KERN_ERR"%s display panel serial number not support\n", __func__);
		return ret;
	}

	/*
	 * for some unknown reason, the panel_serial_info may read dummy,
	 * retry when found panel_serial_info is abnormal.
	 */
	for (i = 0; i < 5; i++) {
		if (!strcmp(display->panel->name, "boe rm692e5 dsc cmd mode panel")) {
			ret = dsi_panel_tx_cmd_set(display->panel, DSI_CMD_PANEL_DATE_SWITCH);
			if (ret) {
				printk(KERN_ERR"%s Failed to set DSI_CMD_PANEL_DATE_SWITCH !!\n", __func__);
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

		if (!strcmp(display->panel->oplus_priv.vendor_name, "NT37290")) {
			panel_serial_info.year -= 1;
		}

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

		ret = scnprintf(panel_rnum->serial_number, PAGE_SIZE, "Get panel serial number: %llx\n",
				serial_number);
		break;
	}

	return ret;
}

extern unsigned int oplus_dsi_log_type;
int oplus_display_set_qcom_loglevel(void *data)
{
	struct kernel_loglevel *k_loginfo = data;
	if (k_loginfo == NULL) {
		DSI_ERR("k_loginfo is null pointer\n");
		return -EINVAL;
	}

	if (k_loginfo->enable) {
		oplus_dsi_log_type |= OPLUS_DEBUG_LOG_BACKLIGHT;
		oplus_dsi_log_type |= OPLUS_DEBUG_LOG_COMMON;
	} else {
		oplus_dsi_log_type &= ~OPLUS_DEBUG_LOG_BACKLIGHT;
		oplus_dsi_log_type &= ~OPLUS_DEBUG_LOG_COMMON;
	}

	DSI_INFO("Set qcom kernel log, enable:0x%X, level:0x%X, current:0x%X\n",
			k_loginfo->enable,
			k_loginfo->log_level,
			oplus_dsi_log_type);
	return 0;
}

int oplus_big_endian_copy(void *dest, void *src, int count)
{
	int index = 0, knum = 0, rc = 0;
	uint32_t *u_dest = (uint32_t*) dest;
	char *u_src = (char*) src;

	if (dest == NULL || src == NULL) {
		printk("%s null pointer\n", __func__);
		return -EINVAL;
	}

	if (dest == src) {
		return rc;
	}

	while (count > 0) {
		u_dest[index] = ((u_src[knum] << 24) | (u_src[knum+1] << 16) | (u_src[knum+2] << 8) | u_src[knum+3]);
		index += 1;
		knum += 4;
		count = count - 1;
	}

	return rc;
}

int oplus_display_get_softiris_color_status(void *data)
{
	struct softiris_color *iris_color_status = data;
	bool color_vivid_status = false;
	bool color_srgb_status = false;
	bool color_softiris_status = false;
	bool color_dual_panel_status = false;
	bool color_dual_brightness_status = false;
	bool color_oplus_calibrate_status = false;
	struct dsi_parser_utils *utils = NULL;
	struct dsi_panel *panel = NULL;

	struct dsi_display *display = get_main_display();
	if (!display) {
		pr_err("failed for: %s %d\n", __func__, __LINE__);
		return -EINVAL;
	}

	panel = display->panel;
	if (!panel) {
		pr_err("failed for: %s %d\n", __func__, __LINE__);
		return -EINVAL;
	}

	utils = &panel->utils;
	if (!utils) {
		pr_err("failed for: %s %d\n", __func__, __LINE__);
		return -EINVAL;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		pr_info("%s: iris secondary panel no need config!\n", __func__);
		return 0;
	}
#endif

	color_vivid_status = utils->read_bool(utils->data, "oplus,color_vivid_status");
	DSI_INFO("oplus,color_vivid_status: %s", color_vivid_status ? "true" : "false");

	color_srgb_status = utils->read_bool(utils->data, "oplus,color_srgb_status");
	DSI_INFO("oplus,color_srgb_status: %s", color_srgb_status ? "true" : "false");

	color_softiris_status = utils->read_bool(utils->data, "oplus,color_softiris_status");
	DSI_INFO("oplus,color_softiris_status: %s", color_softiris_status ? "true" : "false");

	color_dual_panel_status = utils->read_bool(utils->data, "oplus,color_dual_panel_status");
	DSI_INFO("oplus,color_dual_panel_status: %s", color_dual_panel_status ? "true" : "false");

	color_dual_brightness_status = utils->read_bool(utils->data, "oplus,color_dual_brightness_status");
	DSI_INFO("oplus,color_dual_brightness_status: %s", color_dual_brightness_status ? "true" : "false");

	color_oplus_calibrate_status = utils->read_bool(utils->data, "oplus,color_oplus_calibrate_status");
	DSI_INFO("oplus,color_oplus_calibrate_status: %s", color_oplus_calibrate_status ? "true" : "false");

	iris_color_status->color_vivid_status = (uint32_t)color_vivid_status;
	iris_color_status->color_srgb_status = (uint32_t)color_srgb_status;
	iris_color_status->color_softiris_status = (uint32_t)color_softiris_status;
	iris_color_status->color_dual_panel_status = (uint32_t)color_dual_panel_status;
	iris_color_status->color_dual_brightness_status = (uint32_t)color_dual_brightness_status;
	iris_color_status->color_oplus_calibrate_status = (uint32_t)color_oplus_calibrate_status;

	return 0;
}

int oplus_display_panel_get_id2(void)
{
	struct dsi_display *display = get_main_display();
	int ret = 0;
	unsigned char read[30];
	if(!display || !display->panel) {
		printk(KERN_INFO "oplus_display_get_panel_id and main display is null");
		return 0;
	}

	/* if(get_oplus_display_power_status() == OPLUS_DISPLAY_POWER_ON) { */
	if (display->panel->power_mode == SDE_MODE_DPMS_ON) {
		if(display == NULL) {
			printk(KERN_INFO "oplus_display_get_panel_id and main display is null");
			return 0;
		}

		if ((!strcmp(display->panel->oplus_priv.vendor_name, "S6E3HC3")) ||
			(!strcmp(display->panel->oplus_priv.vendor_name, "S6E3HC4")) ||
			(!strcmp(display->panel->oplus_priv.vendor_name, "AMB670YF01"))) {
			ret = dsi_display_read_panel_reg(display, 0xDB, read, 1);
			if (ret < 0) {
				pr_err("failed to read DB ret=%d\n", ret);
				return -EINVAL;
			}
			ret = (int)read[0];
		}
	} else {
		printk(KERN_ERR	 "%s oplus_display_get_panel_id, but now display panel status is not on\n", __func__);
		return 0;
	}

	return ret;
}

int oplus_display_panel_hbm_lightspot_check(void)
{
	int rc = 0;
	char value[] = { 0xE0 };
	char value1[] = { 0x0F, 0xFF };
	struct dsi_display *display = get_main_display();
	struct mipi_dsi_device *mipi_device;

	if (!display || !display->panel) {
		pr_err("failed for: %s %d\n", __func__, __LINE__);
		return -EINVAL;
	}

	/* if (get_oplus_display_power_status() != OPLUS_DISPLAY_POWER_ON) { */
	if (display->panel->power_mode != SDE_MODE_DPMS_ON) {
		printk(KERN_ERR"%s display panel in off status\n", __func__);
		return -EFAULT;
	}

	mipi_device = &display->panel->mipi_device;

	mutex_lock(&display->display_lock);
	mutex_lock(&display->panel->panel_lock);

	if (!dsi_panel_initialized(display->panel)) {
		pr_err("%s, dsi_panel_initialized failed\n", __func__);
		rc = -EINVAL;
		goto unlock;
	}

	rc = dsi_display_cmd_engine_enable(display);

	if (rc) {
		pr_err("%s, cmd engine enable failed\n", __func__);
		rc = -EINVAL;
		goto unlock;
	}

	/* enable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_ON);
	}

	rc = mipi_dsi_dcs_write(mipi_device, 0x53, value, sizeof(value));
	usleep_range(1000, 1100);
	rc = mipi_dsi_dcs_write(mipi_device, 0x51, value1, sizeof(value1));
	usleep_range(1000, 1100);
	pr_err("[%s] hbm_lightspot_check successfully\n",  display->panel->oplus_priv.vendor_name);

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_OFF);
	}

	dsi_display_cmd_engine_disable(display);

unlock:

	mutex_unlock(&display->panel->panel_lock);
	mutex_unlock(&display->display_lock);
	return 0;
}

int oplus_display_get_dp_support(void *buf)
{
	struct dsi_display *display = NULL;
	struct dsi_panel *d_panel = NULL;
	uint32_t *dp_support = buf;

	if (!dp_support) {
		pr_err("oplus_display_get_dp_support error dp_support is null\n");
		return -EINVAL;
	}

	display = get_main_display();
	if (!display) {
		pr_err("oplus_display_get_dp_support error get main display is null\n");
		return -EINVAL;
	}

	d_panel = display->panel;
	if (!d_panel) {
		pr_err("oplus_display_get_dp_support error get main panel is null\n");
		return -EINVAL;
	}

	*dp_support = d_panel->oplus_priv.dp_support;

	return 0;
}

int oplus_display_panel_set_audio_ready(void *data) {
	uint32_t *audio_ready = data;

	oplus_display_audio_ready = (*audio_ready);
	printk("%s oplus_display_audio_ready = %d\n", __func__, oplus_display_audio_ready);

	return 0;
}

int oplus_display_panel_dump_info(void *data) {
	int ret = 0;
	struct dsi_display * temp_display;
	struct display_timing_info *timing_info = data;

	temp_display = get_main_display();

	if (temp_display == NULL) {
		printk(KERN_INFO "oplus_display_dump_info and main display is null");
		ret = -1;
		return ret;
	}

	if(temp_display->modes == NULL) {
		printk(KERN_INFO "oplus_display_dump_info and display modes is null");
		ret = -1;
		return ret;
	}

	timing_info->h_active = temp_display->modes->timing.h_active;
	timing_info->v_active = temp_display->modes->timing.v_active;
	timing_info->refresh_rate = temp_display->modes->timing.refresh_rate;
	timing_info->clk_rate_hz_l32 = (uint32_t)(temp_display->modes->timing.clk_rate_hz & 0x00000000FFFFFFFF);
	timing_info->clk_rate_hz_h32 = (uint32_t)(temp_display->modes->timing.clk_rate_hz >> 32);

	return 0;
}

int oplus_display_panel_get_dsc(void *data) {
	int ret = 0;
	uint32_t *reg_read = data;
	unsigned char read[30];
	struct dsi_display *display = get_main_display();

	if (!display || !display->panel) {
		pr_err("failed for: %s %d\n", __func__, __LINE__);
		return -EINVAL;
	}

	/* if (get_oplus_display_power_status() == OPLUS_DISPLAY_POWER_ON) { */
	if (display->panel->power_mode == SDE_MODE_DPMS_ON) {
		ret = dsi_display_read_panel_reg(get_main_display(), 0x03, read, 1);
		if (ret < 0) {
			printk(KERN_ERR  "%s read panel dsc reg error = %d!\n", __func__, ret);
			ret = -1;
		} else {
			(*reg_read) = read[0];
			ret = 0;
		}
	} else {
		printk(KERN_ERR	 "%s but now display panel status is not on\n", __func__);
		ret = -1;
	}

	return ret;
}

int oplus_display_panel_get_closebl_flag(void *data)
{
	uint32_t *closebl_flag = data;

	(*closebl_flag) = lcd_closebl_flag;
	printk(KERN_INFO "oplus_display_get_closebl_flag = %d\n", lcd_closebl_flag);

	return 0;
}

int oplus_display_panel_set_closebl_flag(void *data)
{
	uint32_t *closebl = data;

	pr_err("lcd_closebl_flag = %d\n", (*closebl));
	if (1 != (*closebl))
		lcd_closebl_flag = 0;
	pr_err("oplus_display_set_closebl_flag = %d\n", lcd_closebl_flag);

	return 0;
}

int oplus_display_panel_get_reg(void *data)
{
	struct dsi_display *display = get_main_display();
	struct panel_reg_get *panel_reg = data;
	uint32_t u32_bytes = sizeof(uint32_t)/sizeof(char);

	if (!display) {
		return -EINVAL;
	}

	mutex_lock(&display->display_lock);

	u32_bytes = oplus_rx_len%u32_bytes ? (oplus_rx_len/u32_bytes + 1) : oplus_rx_len/u32_bytes;
	oplus_big_endian_copy(panel_reg->reg_rw, oplus_rx_reg, u32_bytes);
	panel_reg->lens = oplus_rx_len;

	mutex_unlock(&display->display_lock);

	return 0;
}

int oplus_display_panel_set_reg(void *data)
{
	char reg[PANEL_TX_MAX_BUF] = {0x0};
	char payload[PANEL_TX_MAX_BUF] = {0x0};
	u32 index = 0, value = 0;
	int ret = 0;
	int len = 0;
	struct dsi_display *display = get_main_display();
	struct panel_reg_rw *reg_rw = data;

	if (!display || !display->panel) {
		pr_err("debug for: %s %d\n", __func__, __LINE__);
		return -EFAULT;
	}

	if (reg_rw->lens > PANEL_REG_MAX_LENS) {
		pr_err("error: wrong input reg len\n");
		return -EINVAL;
	}

	if (reg_rw->rw_flags == REG_READ) {
		value = reg_rw->cmd;
		len = reg_rw->lens;
		dsi_display_read_panel_reg(get_main_display(), value, reg, len);

		for (index=0; index < len; index++) {
			printk("reg[%d] = %x ", index, reg[index]);
		}
		mutex_lock(&display->display_lock);
		memcpy(oplus_rx_reg, reg, PANEL_TX_MAX_BUF);
		oplus_rx_len = len;
		mutex_unlock(&display->display_lock);
		return 0;
	}

	if (reg_rw->rw_flags == REG_WRITE) {
		memcpy(payload, reg_rw->value, reg_rw->lens);
		reg[0] = reg_rw->cmd;
		len = reg_rw->lens;
		for (index=0; index < len; index++) {
			reg[index + 1] = payload[index];
		}

		/* if(get_oplus_display_power_status() == OPLUS_DISPLAY_POWER_ON) { */
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
						payload, len);
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
		return 0;
	}
	printk("%s error: please check the args!\n", __func__);
	return -1;
}

int oplus_display_panel_notify_blank(void *data)
{
	struct msm_drm_notifier notifier_data;
	int blank;
	uint32_t *temp_save_user = data;
	int temp_save = (*temp_save_user);
	struct dsi_display *display = get_main_display();

	printk(KERN_INFO "%s oplus_display_notify_panel_blank = %d\n", __func__, temp_save);

	if(temp_save == 1) {
		blank = MSM_DRM_BLANK_UNBLANK;
		notifier_data.data = &blank;
		notifier_data.id = 0;
		msm_drm_notifier_call_chain(MSM_DRM_EARLY_EVENT_BLANK,
				&notifier_data);
		msm_drm_notifier_call_chain(MSM_DRM_EVENT_BLANK,
				&notifier_data);
		oplus_panel_event_notification_trigger(display, DRM_PANEL_EVENT_UNBLANK);
	} else if (temp_save == 0) {
		blank = MSM_DRM_BLANK_POWERDOWN;
		notifier_data.data = &blank;
		notifier_data.id = 0;
		msm_drm_notifier_call_chain(MSM_DRM_EARLY_EVENT_BLANK,
				&notifier_data);
		oplus_panel_event_notification_trigger(display, DRM_PANEL_EVENT_BLANK);
	}
	return 0;
}

int oplus_display_panel_get_spr(void *data)
{
	uint32_t *spr_mode_user = data;

	printk(KERN_INFO "oplus_display_get_spr = %d\n", spr_mode);
	*spr_mode_user = spr_mode;

	return 0;
}

int oplus_display_panel_set_spr(void *data)
{
	uint32_t *temp_save_user = data;
	int temp_save = (*temp_save_user);
	struct dsi_display *display = get_main_display();

	if (!display || !display->panel) {
		pr_err("failed for: %s %d\n", __func__, __LINE__);
		return -EINVAL;
	}

	printk(KERN_INFO "%s oplus_display_set_spr = %d\n", __func__, temp_save);

	__oplus_display_set_spr(temp_save);
	/* if(get_oplus_display_power_status() == OPLUS_DISPLAY_POWER_ON) { */
	if (display->panel->power_mode == SDE_MODE_DPMS_ON) {
		if(get_main_display() == NULL) {
			printk(KERN_INFO "oplus_display_set_spr and main display is null");
			return 0;
		}

		dsi_display_spr_mode(get_main_display(), spr_mode);
	} else {
		printk(KERN_ERR	 "%s oplus_display_set_spr = %d, but now display panel status is not on\n", __func__, temp_save);
	}
	return 0;
}

int oplus_display_panel_get_dither(void *data)
{
	uint32_t *dither_mode_user = data;
	printk(KERN_ERR "oplus_display_get_dither = %d\n", dither_enable);
	*dither_mode_user = dither_enable;
	return 0;
}

int oplus_display_panel_set_dither(void *data)
{
	uint32_t *temp_save_user = data;
	int temp_save = (*temp_save_user);
	printk(KERN_ERR "%s oplus_display_set_dither = %d\n", __func__, temp_save);
	__oplus_display_set_dither(temp_save);
	return 0;
}

int oplus_display_panel_get_roundcorner(void *data)
{
	uint32_t *round_corner = data;
	struct dsi_display *display = get_main_display();
	bool roundcorner = true;

	if (display && display->name &&
			!strcmp(display->name, "qcom,mdss_dsi_oplus19101boe_nt37800_1080_2400_cmd"))
		roundcorner = false;

	*round_corner = roundcorner;

	return 0;
}

int oplus_display_panel_get_dynamic_osc_clock(void *data)
{
	int rc = 0;
	struct dsi_display *display = get_main_display();
	uint32_t *osc_rate = data;

	if (!display || !display->panel) {
		DSI_ERR("Invalid display or panel\n");
		rc = -EINVAL;
		return rc;
	}

	if (!display->panel->oplus_priv.ffc_enabled) {
		DSI_WARN("[%s] FFC is disabled, failed to get osc rate\n",
				__func__);
		rc = -EFAULT;
		return rc;
	}

	mutex_lock(&display->display_lock);

	*osc_rate = display->panel->oplus_priv.osc_rate_cur;
	DSI_INFO("Read osc rate=%d\n", display->panel->oplus_priv.osc_rate_cur);

	mutex_unlock(&display->display_lock);

	return rc;
}

int oplus_display_panel_set_dynamic_osc_clock(void *data)
{
	int rc = 0;
	struct dsi_display *display = get_main_display();
	uint32_t *osc_rate = data;

	if (!display || !display->panel) {
		DSI_ERR("Invalid display or panel\n");
		rc = -EINVAL;
		return rc;
	}

	if (!display->panel->oplus_priv.ffc_enabled) {
		DSI_WARN("[%s] FFC is disabled, failed to set osc rate\n",
				__func__);
		rc = -EFAULT;
		return rc;
	}

	if(display->panel->power_mode != SDE_MODE_DPMS_ON) {
		DSI_WARN("[%s] display panel is not on\n", __func__);
		rc = -EFAULT;
		return rc;
	}

	DSI_INFO("Set osc rate=%d\n", *osc_rate);

	mutex_lock(&display->display_lock);

	rc = oplus_display_update_osc_ffc(display, *osc_rate);
	if (!rc) {
		mutex_lock(&display->panel->panel_lock);
		rc = oplus_panel_set_ffc_mode_unlock(display->panel);
		mutex_unlock(&display->panel->panel_lock);
	}

	mutex_unlock(&display->display_lock);

	return rc;
}

int oplus_display_get_cabc_status(void *buf)
{
	uint32_t *cabc_status = buf;
	struct dsi_display *display = NULL;
	struct dsi_panel *panel = NULL;

	display = get_main_display();
	if (!display) {
		DSI_ERR("No display device\n");
		return -ENODEV;
	}

	panel = display->panel;
	if (!panel) {
		DSI_ERR("No panel device\n");
		return -ENODEV;
	}

	if(panel->oplus_priv.cabc_enabled) {
		*cabc_status = oplus_cabc_status;
	} else {
		*cabc_status = OPLUS_DISPLAY_CABC_OFF;
	}
	return 0;
}

int oplus_display_set_cabc_status(void *buf)
{
	int rc = 0;
	uint32_t *cabc_status = buf;
	struct dsi_display *display = NULL;
	struct dsi_panel *panel = NULL;

	display = get_main_display();
	if (!display) {
		DSI_ERR("No display device\n");
		return -ENODEV;
	}

	panel = display->panel;
	if (!panel) {
		DSI_ERR("No panel device\n");
		return -ENODEV;
	}

	if (!panel->oplus_priv.cabc_enabled) {
		DSI_WARN("This project don't support cabc\n");
		return -EFAULT;
	}

	if (*cabc_status >= OPLUS_DISPLAY_CABC_UNKNOW) {
		DSI_ERR("Unknow cabc status = [%d]\n", *cabc_status);
		return -EINVAL;
	}

	if(get_oplus_display_power_status() == OPLUS_DISPLAY_POWER_ON) {
			if (*cabc_status == OPLUS_DISPLAY_CABC_OFF) {
				rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_CABC_OFF);
				if (rc) {
					DSI_ERR("[%s] failed to send DSI_CMD_CABC_OFF cmds, rc=%d\n",
							panel->name, rc);
				}
			} else if (*cabc_status == OPLUS_DISPLAY_CABC_UI) {
				rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_CABC_UI);
				if (rc) {
					DSI_ERR("[%s] failed to send DSI_CMD_CABC_UI cmds, rc=%d\n",
							panel->name, rc);
				}
			} else if (*cabc_status == OPLUS_DISPLAY_CABC_IMAGE) {
				rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_CABC_IMAGE);
				if (rc) {
					DSI_ERR("[%s] failed to send DSI_CMD_CABC_IMAGE cmds, rc=%d\n",
							panel->name, rc);
				}
			}  else if (*cabc_status == OPLUS_DISPLAY_CABC_VIDEO) {
				rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_CABC_VIDEO);
				if (rc) {
					DSI_ERR("[%s] failed to send DSI_CMD_CABC_VIDEO cmds, rc=%d\n",
							panel->name, rc);
				}
			}
		oplus_cabc_status = *cabc_status;
		pr_err("debug for %s, buf = [%s], oplus_cabc_status = %d\n",
				__func__, buf, oplus_cabc_status);
	} else {
		pr_err("debug for %s, buf = [%s], but display panel status is not on!\n",
				__func__, *cabc_status);
	}
	return rc;
}

int oplus_display_get_dre_status(void *buf)
{
	uint32_t *dre_status = buf;
	struct dsi_display *display = NULL;
	struct dsi_panel *panel = NULL;

	display = get_main_display();
	if (!display) {
		DSI_ERR("No display device\n");
		return -ENODEV;
	}

	panel = display->panel;
	if (!panel) {
		DSI_ERR("No panel device\n");
		return -ENODEV;
	}

	if(panel->oplus_priv.dre_enabled) {
		*dre_status = oplus_dre_status;
	} else {
		*dre_status = OPLUS_DISPLAY_DRE_OFF;
	}
	return 0;
}

int oplus_display_set_dre_status(void *buf)
{
	int rc = 0;
	uint32_t *dre_status = buf;
	struct dsi_display *display = NULL;
	struct dsi_panel *panel = NULL;

	display = get_main_display();
	if (!display) {
		DSI_ERR("No display device\n");
		return -ENODEV;
	}

	panel = display->panel;
	if (!panel) {
		DSI_ERR("No panel device\n");
		return -ENODEV;
	}

	if(!panel->oplus_priv.dre_enabled) {
		DSI_ERR("This project don't support dre\n");
		return -EFAULT;
	}

	if (*dre_status >= OPLUS_DISPLAY_DRE_UNKNOW) {
		DSI_ERR("Unknow DRE status = [%d]\n", *dre_status);
		return -EINVAL;
	}

	if(get_oplus_display_power_status() == OPLUS_DISPLAY_POWER_ON) {
		if (*dre_status == OPLUS_DISPLAY_DRE_ON) {
			/* if(mtk)  */
			/*	disp_aal_set_dre_en(0);   MTK AAL api */
		} else {
			/* if(mtk) */
			/*	disp_aal_set_dre_en(1);  MTK AAL api */
		}
		oplus_dre_status = *dre_status;
		pr_err("debug for %s, buf = [%s], oplus_dre_status = %d\n",
				__func__, buf, oplus_dre_status);
	} else {
		pr_err("debug for %s, buf = [%s], but display panel status is not on!\n",
				__func__, *dre_status);
	}
	return rc;
}

int oplus_display_get_dither_status(void *buf)
{
	uint32_t *dither_enable = buf;
	*dither_enable = oplus_dither_enable;

	return 0;
}

int oplus_display_set_dither_status(void *buf)
{
	uint32_t *dither_enable = buf;
	oplus_dither_enable = *dither_enable;
	pr_err("debug for %s, buf = [%s], oplus_dither_enable = %d\n",
			__func__, buf, oplus_dither_enable);

	return 0;
}

int oplus_panel_set_ffc_mode_unlock(struct dsi_panel *panel)
{
	int rc = 0;
	u32 cmd_index = 0;

	if (panel->oplus_priv.ffc_mode_index >= FFC_MODE_MAX_COUNT) {
		DSI_ERR("Invalid ffc_mode_index=%d\n",
				panel->oplus_priv.ffc_mode_index);
		rc = -EINVAL;
		return rc;
	}

	cmd_index = DSI_CMD_FFC_MODE0 + panel->oplus_priv.ffc_mode_index;
	rc = dsi_panel_tx_cmd_set(panel, cmd_index);
	if (rc) {
		DSI_ERR("Failed to send DSI_CMD_FFC_MODE%d, rc=%d\n",
				panel->oplus_priv.ffc_mode_index,
				rc);
	}

	return rc;
}

int oplus_panel_set_ffc_kickoff_lock(struct dsi_panel *panel)
{
	int rc = 0;

	mutex_lock(&panel->oplus_ffc_lock);
	panel->oplus_priv.ffc_delay_frames--;
	if (panel->oplus_priv.ffc_delay_frames) {
		mutex_unlock(&panel->oplus_ffc_lock);
		return rc;
	}

	mutex_lock(&panel->panel_lock);
	rc = oplus_panel_set_ffc_mode_unlock(panel);
	mutex_unlock(&panel->panel_lock);

	mutex_unlock(&panel->oplus_ffc_lock);

	return rc;
}

int oplus_panel_check_ffc_config(struct dsi_panel *panel,
		struct oplus_clk_osc *clk_osc_pending)
{
	int rc = 0;
	int index;
	struct oplus_clk_osc *seq = panel->oplus_priv.clk_osc_seq;
	u32 count = panel->oplus_priv.ffc_mode_count;
	u32 last_index = panel->oplus_priv.ffc_mode_index;

	if (!seq || !count) {
		DSI_ERR("Invalid clk_osc_seq or ffc_mode_count\n");
		rc = -EINVAL;
		return rc;
	}

	for (index = 0; index < count; index++) {
		if (seq->clk_rate == clk_osc_pending->clk_rate &&
				seq->osc_rate == clk_osc_pending->osc_rate) {
			break;
		}
		seq++;
	}

	if (index < count) {
		DSI_INFO("Update ffc config: index:[%d -> %d], clk=%d, osc=%d\n",
				last_index,
				index,
				clk_osc_pending->clk_rate,
				clk_osc_pending->osc_rate);

		panel->oplus_priv.ffc_mode_index = index;
		panel->oplus_priv.clk_rate_cur = clk_osc_pending->clk_rate;
		panel->oplus_priv.osc_rate_cur = clk_osc_pending->osc_rate;
	} else {
		rc = -EINVAL;
	}

	return rc;
}

int oplus_display_update_clk_ffc(struct dsi_display *display,
		struct dsi_display_mode *cur_mode, struct dsi_display_mode *adj_mode)
{
	int rc = 0;
	struct dsi_panel *panel = display->panel;
	struct oplus_clk_osc clk_osc_pending;

	DSI_MM_INFO("DisplayDriverID@@426$$Switching ffc mode, clk:[%d -> %d]",
			display->cached_clk_rate,
			display->dyn_bit_clk);

	if (display->cached_clk_rate == display->dyn_bit_clk) {
		DSI_MM_WARN("DisplayDriverID@@427$$Ignore duplicated clk ffc setting, clk=%d",
				display->dyn_bit_clk);
		return rc;
	}

	mutex_lock(&panel->oplus_ffc_lock);

	clk_osc_pending.clk_rate = display->dyn_bit_clk;
	clk_osc_pending.osc_rate = panel->oplus_priv.osc_rate_cur;

	rc = oplus_panel_check_ffc_config(panel, &clk_osc_pending);
	if (!rc) {
		panel->oplus_priv.ffc_delay_frames = FFC_DELAY_MAX_FRAMES;
	} else {
		DSI_MM_ERR("DisplayDriverID@@427$$Failed to find ffc mode index, clk=%d, osc=%d",
				clk_osc_pending.clk_rate,
				clk_osc_pending.osc_rate);
	}

	mutex_unlock(&panel->oplus_ffc_lock);

	return rc;
}

int oplus_display_update_osc_ffc(struct dsi_display *display,
		u32 osc_rate)
{
	int rc = 0;
	struct dsi_panel *panel = display->panel;
	struct oplus_clk_osc clk_osc_pending;

	DSI_MM_INFO("DisplayDriverID@@428$$Switching ffc mode, osc:[%d -> %d]",
			panel->oplus_priv.osc_rate_cur,
			osc_rate);

	if (osc_rate == panel->oplus_priv.osc_rate_cur) {
		DSI_MM_WARN("DisplayDriverID@@429$$Ignore duplicated osc ffc setting, osc=%d",
				panel->oplus_priv.osc_rate_cur);
		return rc;
	}

	mutex_lock(&panel->oplus_ffc_lock);

	clk_osc_pending.clk_rate = panel->oplus_priv.clk_rate_cur;
	clk_osc_pending.osc_rate = osc_rate;
	rc = oplus_panel_check_ffc_config(panel, &clk_osc_pending);
	if (rc) {
		DSI_MM_ERR("DisplayDriverID@@429$$Failed to find ffc mode index, clk=%d, osc=%d",
				clk_osc_pending.clk_rate,
				clk_osc_pending.osc_rate);
	}

	mutex_unlock(&panel->oplus_ffc_lock);

	return rc;
}

int oplus_panel_parse_ffc_config(struct dsi_panel *panel)
{
	int rc = 0;
	int i;
	u32 length = 0;
	u32 count = 0;
	u32 size = 0;
	u32 *arr_32 = NULL;
	const u32 *arr;
	struct dsi_parser_utils *utils = &panel->utils;
	struct oplus_clk_osc *seq;

	if (panel->host_config.ext_bridge_mode)
		return 0;

	panel->oplus_priv.ffc_enabled = utils->read_bool(utils->data,
			"oplus,ffc-enabled");
	if (!panel->oplus_priv.ffc_enabled) {
		rc = -EFAULT;
		goto error;
	}

	arr = utils->get_property(utils->data,
			"oplus,clk-osc-sequence", &length);
	if (!arr) {
		DSI_ERR("[%s] oplus,clk-osc-sequence not found\n", panel->name);
		rc = -EINVAL;
		goto error;
	}
	if (length & 0x1) {
		DSI_ERR("[%s] syntax error for oplus,clk-osc-sequence\n",
				panel->name);
		rc = -EINVAL;
		goto error;
	}

	length = length / sizeof(u32);
	DSI_INFO("[%s] oplus,clk-osc-sequence length=%d\n",
			panel->name, length);

	size = length * sizeof(u32);
	arr_32 = kzalloc(size, GFP_KERNEL);
	if (!arr_32) {
		rc = -ENOMEM;
		goto error;
	}

	rc = utils->read_u32_array(utils->data, "oplus,clk-osc-sequence",
					arr_32, length);
	if (rc) {
		DSI_ERR("[%s] cannot read oplus,clk-osc-sequence\n", panel->name);
		goto error_free_arr_32;
	}

	count = length / 2;
	if (count > FFC_MODE_MAX_COUNT) {
		DSI_ERR("[%s] invalid ffc mode count:%d, greater than maximum:%d\n",
				panel->name, count, FFC_MODE_MAX_COUNT);
		rc = -EINVAL;
		goto error_free_arr_32;
	}

	size = count * sizeof(*seq);
	seq = kzalloc(size, GFP_KERNEL);
	if (!seq) {
		rc = -ENOMEM;
		goto error_free_arr_32;
	}

	panel->oplus_priv.clk_osc_seq = seq;
	panel->oplus_priv.ffc_delay_frames = 0;
	panel->oplus_priv.ffc_mode_count = count;
	panel->oplus_priv.ffc_mode_index = 0;
	panel->oplus_priv.clk_rate_cur = arr_32[0];
	panel->oplus_priv.osc_rate_cur = arr_32[1];

	for (i = 0; i < length; i += 2) {
		DSI_INFO("[%s] clk osc seq: index=%d <%d %d>\n",
				panel->name, i / 2, arr_32[i], arr_32[i+1]);
		seq->clk_rate = arr_32[i];
		seq->osc_rate = arr_32[i + 1];
		seq++;
	}

error_free_arr_32:
	kfree(arr_32);
error:
	if (rc) {
		panel->oplus_priv.ffc_enabled = false;
	}

	DSI_INFO("[%s] oplus,ffc-enabled: %s",
			panel->name,
			panel->oplus_priv.ffc_enabled ? "true" : "false");

	return rc;
}

int dsi_panel_parse_oplus_config(struct dsi_panel *panel)
{
	struct dsi_parser_utils *utils = &panel->utils;
	int ret = 0;

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		pr_info("%s: iris secondary panel no need config!\n", __func__);
		return 0;
	}
#endif

	/* Add for parse ffc config */
	oplus_panel_parse_ffc_config(panel);

	panel->oplus_priv.vendor_name = utils->get_property(utils->data,
			"oplus,mdss-dsi-vendor-name", NULL);

	if (!panel->oplus_priv.vendor_name) {
		pr_err("Failed to found panel name, using dumming name\n");
		panel->oplus_priv.vendor_name = DSI_PANEL_OPLUS_DUMMY_VENDOR_NAME;
	}

	panel->oplus_priv.manufacture_name = utils->get_property(utils->data,
			"oplus,mdss-dsi-manufacture", NULL);

	if (!panel->oplus_priv.manufacture_name) {
		pr_err("Failed to found panel name, using dumming name\n");
		panel->oplus_priv.manufacture_name = DSI_PANEL_OPLUS_DUMMY_MANUFACTURE_NAME;
	}

	panel->oplus_priv.gpio_pre_enabled = utils->read_bool(utils->data,
			"oplus,gpio-pre-enabled");
	DSI_INFO("oplus,gpio-pre-enabled: %s",
		panel->oplus_priv.gpio_pre_enabled ? "true" : "false");

	panel->oplus_priv.is_pxlw_iris5 = utils->read_bool(utils->data,
			"oplus,is_pxlw_iris5");
	DSI_INFO("is_pxlw_iris5: %s",
			panel->oplus_priv.is_pxlw_iris5 ? "true" : "false");

	panel->oplus_priv.iris_pw_enable = utils->read_bool(utils->data,
			"oplus,iris-pw-enabled");
	DSI_INFO("iris-pw-enabled: %s", panel->oplus_priv.iris_pw_enable ? "true" : "false");
	if (panel->oplus_priv.iris_pw_enable) {
		panel->oplus_priv.iris_pw_rst_gpio = utils->get_named_gpio(utils->data, "oplus,iris-pw-rst-gpio", 0);
		if (!gpio_is_valid(panel->oplus_priv.iris_pw_rst_gpio)) {
			DSI_ERR("failed get pw rst gpio\n");
		}
		panel->oplus_priv.iris_pw_0p9_en_gpio = utils->get_named_gpio(utils->data, "oplus,iris-pw-0p9-gpio", 0);
		if (!gpio_is_valid(panel->oplus_priv.iris_pw_0p9_en_gpio)) {
			DSI_ERR("failed get pw 0p9 en gpio\n");
		}
	}

	panel->oplus_priv.is_osc_support = utils->read_bool(utils->data, "oplus,osc-support");
	pr_info("[%s]osc mode support: %s", __func__, panel->oplus_priv.is_osc_support ? "Yes" : "Not");

	if (panel->oplus_priv.is_osc_support) {
		ret = utils->read_u32(utils->data, "oplus,mdss-dsi-osc-clk-mode0-rate",
				&panel->oplus_priv.osc_clk_mode0_rate);
		if (ret) {
			pr_err("[%s]failed get panel parameter: oplus,mdss-dsi-osc-clk-mode0-rate\n", __func__);
			panel->oplus_priv.osc_clk_mode0_rate = 0;
		}
		dynamic_osc_clock = panel->oplus_priv.osc_clk_mode0_rate;

		ret = utils->read_u32(utils->data, "oplus,mdss-dsi-osc-clk-mode1-rate",
				&panel->oplus_priv.osc_clk_mode1_rate);
		if (ret) {
			pr_err("[%s]failed get panel parameter: oplus,mdss-dsi-osc-clk-mode1-rate\n", __func__);
			panel->oplus_priv.osc_clk_mode1_rate = 0;
		}
	}

	/* Add for apollo */
	panel->oplus_priv.is_apollo_support = utils->read_bool(utils->data, "oplus,apollo_backlight_enable");
	apollo_backlight_enable = panel->oplus_priv.is_apollo_support;
	DSI_INFO("apollo_backlight_enable: %s", panel->oplus_priv.is_apollo_support ? "true" : "false");

	if (panel->oplus_priv.is_apollo_support) {
		ret = utils->read_u32(utils->data, "oplus,apollo-sync-brightness-level",
				&panel->oplus_priv.sync_brightness_level);

		if (ret) {
			pr_info("[%s] failed to get panel parameter: oplus,apollo-sync-brightness-level\n", __func__);
			/* Default sync brightness level is set to 200 */
			panel->oplus_priv.sync_brightness_level = 200;
		}
		panel->oplus_priv.dc_apollo_sync_enable = utils->read_bool(utils->data, "oplus,dc_apollo_sync_enable");
		if (panel->oplus_priv.dc_apollo_sync_enable) {
			ret = utils->read_u32(utils->data, "oplus,dc-apollo-backlight-sync-level",
					&panel->oplus_priv.dc_apollo_sync_brightness_level);
			if (ret) {
				pr_info("[%s] failed to get panel parameter: oplus,dc-apollo-backlight-sync-level\n", __func__);
				panel->oplus_priv.dc_apollo_sync_brightness_level = 397;
			}
			ret = utils->read_u32(utils->data, "oplus,dc-apollo-backlight-sync-level-pcc-max",
					&panel->oplus_priv.dc_apollo_sync_brightness_level_pcc);
			if (ret) {
				pr_info("[%s] failed to get panel parameter: oplus,dc-apollo-backlight-sync-level-pcc-max\n", __func__);
				panel->oplus_priv.dc_apollo_sync_brightness_level_pcc = 30000;
			}
			ret = utils->read_u32(utils->data, "oplus,dc-apollo-backlight-sync-level-pcc-min",
					&panel->oplus_priv.dc_apollo_sync_brightness_level_pcc_min);
			if (ret) {
				pr_info("[%s] failed to get panel parameter: oplus,dc-apollo-backlight-sync-level-pcc-min\n", __func__);
				panel->oplus_priv.dc_apollo_sync_brightness_level_pcc_min = 29608;
			}
			pr_info("dc apollo sync enable(%d,%d,%d)\n", panel->oplus_priv.dc_apollo_sync_brightness_level,
					panel->oplus_priv.dc_apollo_sync_brightness_level_pcc, panel->oplus_priv.dc_apollo_sync_brightness_level_pcc_min);
		}
	}

	if (oplus_adfr_is_support()) {
		if (oplus_adfr_get_vsync_mode() == OPLUS_EXTERNAL_TE_TP_VSYNC) {
			/* power on with vsync_switch_gpio high bacause default timing is fhd OA 60hz */
			panel->vsync_switch_gpio_level = 0;
		}
	}

	panel->oplus_priv.enhance_mipi_strength = utils->read_bool(utils->data, "oplus,enhance_mipi_strength");
	oplus_enhance_mipi_strength = panel->oplus_priv.enhance_mipi_strength;
	pr_info("lcm enhance_mipi_strength: %s", panel->oplus_priv.enhance_mipi_strength ? "true" : "false");

	return 0;
}
EXPORT_SYMBOL(dsi_panel_parse_oplus_config);

int oplus_display_get_iris_loopback_status(void *buf)
{
#if defined(CONFIG_PXLW_IRIS)
	uint32_t *status = buf;

	*status = iris_loop_back_validate();
#endif
	return 0;
}

