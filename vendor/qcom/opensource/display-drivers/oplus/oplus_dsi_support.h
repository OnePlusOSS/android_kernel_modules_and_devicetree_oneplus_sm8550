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

/* debug log config */
extern unsigned int oplus_bl_print_window;


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
void notifier_oplus_display_early_status(enum oplus_display_power_status
		power_status);
void notifier_oplus_display_status(enum oplus_display_power_status power_status);
bool is_oplus_correct_display(enum oplus_display_support_list lcd_name);
bool is_silence_reboot(void);
bool is_factory_boot(void);
void set_oplus_display_power_status(enum oplus_display_power_status power_status);
enum oplus_display_power_status get_oplus_display_power_status(void);
bool is_oplus_display_support_feature(enum oplus_display_feature feature_name);
int oplus_display_get_resolution(unsigned int *xres, unsigned int *yres);

/* add for dual panel */
void oplus_display_set_current_display(void *dsi_display);
void oplus_display_update_current_display(void);
struct dsi_display *oplus_display_get_current_display(void);

#endif /* _OPLUS_DSI_SUPPORT_H_ */

