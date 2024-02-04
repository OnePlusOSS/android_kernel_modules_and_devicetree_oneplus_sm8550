/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
**
** File : oplus_dsi_support.h
** Description : display driver private management
** Version : 1.0
** Date : 2018/03/17
** Author : Display
******************************************************************/
#ifndef _OPLUS_DSI_SUPPORT_H_
#define _OPLUS_DSI_SUPPORT_H_

#include <linux/err.h>
#include <linux/string.h>
#include <linux/notifier.h>

/* A hardware display blank change occurred */
#define OPLUS_DISPLAY_EVENT_BLANK			0x01

/* A hardware display blank early change occurred */
#define OPLUS_DISPLAY_EARLY_EVENT_BLANK		0x02

/* log level config */
extern unsigned int oplus_lcd_log_level;
/* debug log config */
extern unsigned int oplus_dsi_log_type;
extern unsigned int oplus_bl_print_window;
/* dual display id */
extern unsigned int oplus_ofp_display_id;
extern unsigned int oplus_display_trace_enable;

static inline int str_equal(const char *a, const char *b)
{
	return !strcmp(a, b);
}

/* lcd debug log */
#define LCD_ERR(fmt, arg...)	\
	do {	\
		if (oplus_lcd_log_level >= OPLUS_LOG_LEVEL_ERR)	\
			pr_err("[LCD][%u][ERR][%s:%d] " pr_fmt(fmt), oplus_ofp_display_id, __func__, __LINE__, ##arg);	\
	} while (0)

#define LCD_WARN(fmt, arg...)	\
	do {	\
		if (oplus_lcd_log_level >= OPLUS_LOG_LEVEL_WARN)	\
			pr_warn("[LCD][%u][WARN][%s:%d] " pr_fmt(fmt), oplus_ofp_display_id, __func__, __LINE__, ##arg);	\
	} while (0)

#define LCD_INFO(fmt, arg...)	\
	do {	\
		if (oplus_lcd_log_level >= OPLUS_LOG_LEVEL_INFO)	\
			pr_info("[LCD][%u][INFO][%s:%d] " pr_fmt(fmt), oplus_ofp_display_id, __func__, __LINE__, ##arg);	\
	} while (0)

#define LCD_INFO_ONCE(fmt, arg...)	\
	do {	\
		pr_info_once("[LCD][%u][INFO_ONCE][%s:%d] " pr_fmt(fmt), oplus_ofp_display_id, __func__, __LINE__, ##arg);	\
	} while (0)

#define LCD_DEBUG(fmt, arg...)	\
	do {	\
		if ((oplus_lcd_log_level >= OPLUS_LOG_LEVEL_DEBUG) && (oplus_dsi_log_type & OPLUS_DEBUG_LOG_LCD))	\
			pr_info("[LCD][%u][DEBUG][%s:%d] " pr_fmt(fmt), oplus_ofp_display_id, __func__, __LINE__, ##arg);	\
		else	\
			pr_debug("[LCD][%u][DEBUG][%s:%d] " pr_fmt(fmt), oplus_ofp_display_id, __func__, __LINE__, ##arg);	\
	} while (0)

#define LCD_DEBUG_CMD(fmt, arg...)	\
	do {	\
		if ((oplus_lcd_log_level >= OPLUS_LOG_LEVEL_DEBUG) && (oplus_dsi_log_type & OPLUS_DEBUG_LOG_CMD))	\
			pr_info("[LCD][%u][CMD][%s:%d] " pr_fmt(fmt), oplus_ofp_display_id, __func__, __LINE__, ##arg);	\
		else	\
			pr_debug("[LCD][%u][CMD][%s:%d] " pr_fmt(fmt), oplus_ofp_display_id, __func__, __LINE__, ##arg);	\
	} while (0)

#define LCD_DEBUG_BACKLIGHT(fmt, arg...)	\
	do {	\
		if ((oplus_lcd_log_level >= OPLUS_LOG_LEVEL_DEBUG) && ((oplus_dsi_log_type & OPLUS_DEBUG_LOG_BACKLIGHT) || (oplus_bl_print_window > 0)))	\
			pr_info("[LCD][%u][BACKLIGHT][%s:%d] " pr_fmt(fmt), oplus_ofp_display_id, __func__, __LINE__, ##arg);	\
		else	\
			pr_debug("[LCD][%u][BACKLIGHT][%s:%d] " pr_fmt(fmt), oplus_ofp_display_id, __func__, __LINE__, ##arg);	\
	} while (0)

#define LCD_DEBUG_COMMON(fmt, arg...)	\
	do {	\
		if ((oplus_lcd_log_level >= OPLUS_LOG_LEVEL_DEBUG) && (oplus_dsi_log_type & OPLUS_DEBUG_LOG_COMMON))	\
			pr_info("[LCD][%u][COMMON][%s:%d] " pr_fmt(fmt), oplus_ofp_display_id, __func__, __LINE__, ##arg);	\
		else	\
			pr_debug("[LCD][%u][COMMON][%s:%d] " pr_fmt(fmt), oplus_ofp_display_id, __func__, __LINE__, ##arg);	\
	} while (0)

/* lcd debug trace */
#define OPLUS_LCD_TRACE_BEGIN(name)	\
	do {	\
		if (oplus_display_trace_enable & OPLUS_DISPLAY_LCD_TRACE_ENABLE)	\
			SDE_ATRACE_BEGIN(name);	\
	} while (0)

#define OPLUS_LCD_TRACE_END(name)	\
	do {	\
		if (oplus_display_trace_enable & OPLUS_DISPLAY_LCD_TRACE_ENABLE)	\
			SDE_ATRACE_END(name);	\
	} while (0)

#define OPLUS_LCD_TRACE_INT(name, value)	\
	do {	\
		if (oplus_display_trace_enable & OPLUS_DISPLAY_LCD_TRACE_ENABLE)	\
			SDE_ATRACE_INT(name, value);	\
	} while (0)

enum oplus_log_level {
	OPLUS_LOG_LEVEL_NONE = 0,
	OPLUS_LOG_LEVEL_ERR,
	OPLUS_LOG_LEVEL_WARN,
	OPLUS_LOG_LEVEL_INFO,
	OPLUS_LOG_LEVEL_DEBUG,
};

/**
 * enum oplus_debug_log --       flags to control debug log; 1->enbale  0->disable
 * @OPLUS_DEBUG_LOG_DISABLED:    disable all debug log
 * @OPLUS_DEBUG_LOG_CMD:         dump register log
 * @OPLUS_DEBUG_LOG_BACKLIGHT:   backlight log
 * @OPLUS_DEBUG_LOG_COMMON:      common log
 * @OPLUS_DEBUG_LOG_OFP:         OFP log
 * @OPLUS_DEBUG_LOG_ADFR:        ADFR log
 * @OPLUS_DEBUG_LOG_LCD:         lcd log
 * @OPLUS_DEBUG_LOG_TEMP_COMPENSATION:temp compensation log
 * @OPLUS_DEBUG_LOG_ALL:         enable all debug log
 */
enum oplus_debug_log {
	OPLUS_DEBUG_LOG_DISABLED = 0,
	OPLUS_DEBUG_LOG_CMD = BIT(0),
	OPLUS_DEBUG_LOG_BACKLIGHT = BIT(1),
	OPLUS_DEBUG_LOG_COMMON = BIT(2),
	OPLUS_DEBUG_LOG_OFP = BIT(3),
	OPLUS_DEBUG_LOG_ADFR = BIT(4),
	OPLUS_DEBUG_LOG_LCD = BIT(5),
	OPLUS_DEBUG_LOG_TEMP_COMPENSATION = BIT(6),
	OPLUS_DEBUG_LOG_ALL = 0xFFFF,
};

enum oplus_display_trace_enable {
	OPLUS_DISPLAY_DISABLE_TRACE = 0,
	OPLUS_DISPLAY_OFP_TRACE_ENABLE = BIT(0),
	OPLUS_DISPLAY_ADFR_TRACE_ENABLE = BIT(1),
	OPLUS_DISPLAY_LCD_TRACE_ENABLE = BIT(2),
	OPLUS_DISPLAY_TEMP_COMPENSATION_TRACE_ENABLE = BIT(3),
};

enum oplus_display_support_list {
	OPLUS_SAMSUNG_ANA6706_DISPLAY_FHD_DSC_CMD_PANEL = 0,
	OPLUS_SAMSUNG_ONEPLUS_DISPLAY_FHD_DSC_CMD_PANEL,
	OPLUS_DISPLAY_UNKNOW,
};

enum oplus_display_power_status {
	OPLUS_DISPLAY_POWER_OFF = 0,
	OPLUS_DISPLAY_POWER_DOZE,
	OPLUS_DISPLAY_POWER_ON,
	OPLUS_DISPLAY_POWER_DOZE_SUSPEND,
	OPLUS_DISPLAY_POWER_ON_UNKNOW,
};

enum oplus_display_feature {
	OPLUS_DISPLAY_HDR = 0,
	OPLUS_DISPLAY_SEED,
	OPLUS_DISPLAY_HBM,
	OPLUS_DISPLAY_LBR,
	OPLUS_DISPLAY_AOD,
	OPLUS_DISPLAY_ULPS,
	OPLUS_DISPLAY_ESD_CHECK,
	OPLUS_DISPLAY_DYNAMIC_MIPI,
	OPLUS_DISPLAY_PARTIAL_UPDATE,
	OPLUS_DISPLAY_FEATURE_MAX,
};

enum oplus_display_cabc_status {
	OPLUS_DISPLAY_CABC_OFF = 0,
	OPLUS_DISPLAY_CABC_UI,
	OPLUS_DISPLAY_CABC_IMAGE,
	OPLUS_DISPLAY_CABC_VIDEO,
	OPLUS_DISPLAY_CABC_UNKNOW,
};

enum oplus_display_dre_status {
	OPLUS_DISPLAY_DRE_OFF = 0,
	OPLUS_DISPLAY_DRE_ON,
	OPLUS_DISPLAY_DRE_UNKNOW,
};

typedef struct panel_serial_info {
	int reg_index;
	uint64_t year;
	uint64_t month;
	uint64_t day;
	uint64_t hour;
	uint64_t minute;
	uint64_t second;
	uint64_t reserved[2];
} PANEL_SERIAL_INFO;


typedef struct oplus_display_notifier_event {
	enum oplus_display_power_status status;
	void *data;
} OPLUS_DISPLAY_NOTIFIER_EVENT;

int oplus_display_register_client(struct notifier_block *nb);
int oplus_display_unregister_client(struct notifier_block *nb);
void oplus_display_notifier_early_status(enum oplus_display_power_status
		power_status);
void oplus_display_notifier_status(enum oplus_display_power_status power_status);
bool oplus_is_correct_display(enum oplus_display_support_list lcd_name);
bool oplus_is_silence_reboot(void);
bool oplus_is_factory_boot(void);
enum oplus_display_power_status __oplus_get_power_status(void);
void __oplus_set_power_status(enum oplus_display_power_status power_status);
bool oplus_display_is_support_feature(enum oplus_display_feature feature_name);
int oplus_display_get_resolution(unsigned int *xres, unsigned int *yres);

/* add for dual panel */
void oplus_display_set_current_display(void *dsi_display);
void oplus_display_update_current_display(void);
struct dsi_display *oplus_display_get_current_display(void);

#endif /* _OPLUS_DSI_SUPPORT_H_ */

