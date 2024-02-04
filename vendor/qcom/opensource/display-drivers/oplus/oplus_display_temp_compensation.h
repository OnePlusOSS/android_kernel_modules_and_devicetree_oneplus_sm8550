/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
** File : oplus_display_temp_compensation.h
** Description : oplus_display_temp_compensation header
** Version : 1.0
** Date : 2022/11/20
** Author : Display
***************************************************************/

#ifndef _OPLUS_DISPLAY_TEMP_COMPENSATION_H_
#define _OPLUS_DISPLAY_TEMP_COMPENSATION_H_

/* please just only include linux common head file to keep me pure */
#include "oplus_display_private_api.h"

enum oplus_temp_compensation_log_level {
	OPLUS_TEMP_COMPENSATION_LOG_LEVEL_NONE = 0,
	OPLUS_TEMP_COMPENSATION_LOG_LEVEL_ERR = 1,
	OPLUS_TEMP_COMPENSATION_LOG_LEVEL_WARN = 2,
	OPLUS_TEMP_COMPENSATION_LOG_LEVEL_INFO = 3,
	OPLUS_TEMP_COMPENSATION_LOG_LEVEL_DEBUG = 4,
};

enum oplus_temp_compensation_dbv_index {
	OPLUS_TEMP_COMPENSATION_GREATER_THAN_3515_DBV_INDEX = 0,	/* dbv > 3515 */
	OPLUS_TEMP_COMPENSATION_1604_3515_DBV_INDEX = 1,			/* 1604 <= dbv <= 3515 */
	OPLUS_TEMP_COMPENSATION_1511_1604_DBV_INDEX = 2,			/* 1511 <= dbv < 1604 */
	OPLUS_TEMP_COMPENSATION_1419_1511_DBV_INDEX = 3,			/* 1419 <= dbv < 1511 */
	OPLUS_TEMP_COMPENSATION_1328_1419_DBV_INDEX = 4,			/* 1328 <= dbv < 1419 */
	OPLUS_TEMP_COMPENSATION_1212_1328_DBV_INDEX = 5,			/* 1212 <= dbv < 1328 */
	OPLUS_TEMP_COMPENSATION_1096_1212_DBV_INDEX = 6,			/* 1096 <= dbv < 1212 */
	OPLUS_TEMP_COMPENSATION_950_1096_DBV_INDEX = 7,				/* 950 <= dbv < 1096 */
	OPLUS_TEMP_COMPENSATION_761_950_DBV_INDEX = 8,				/* 761 <= dbv < 950 */
	OPLUS_TEMP_COMPENSATION_544_761_DBV_INDEX = 9,				/* 544 <= dbv < 761 */
	OPLUS_TEMP_COMPENSATION_LESS_THAN_544_DBV_INDEX = 10,		/* dbv < 544 */
};

enum oplus_temp_compensation_temp_index {
	OPLUS_TEMP_COMPENSATION_LESS_THAN_MINUS10_TEMP_INDEX = 0,	/* -20 ~ -10 */
	OPLUS_TEMP_COMPENSATION_MINUS10_0_TEMP_INDEX = 1,			/* -10 ~ 0 */
	OPLUS_TEMP_COMPENSATION_0_10_TEMP_INDEX = 2,				/* 0 ~ 10 */
	OPLUS_TEMP_COMPENSATION_10_20_TEMP_INDEX = 3,				/* 10 ~ 20 */
	OPLUS_TEMP_COMPENSAITON_20_25_TEMP_INDEX = 4,				/* 20 ~ 25 */
	OPLUS_TEMP_COMPENSATION_25_30_TEMP_INDEX = 5,				/* 25 ~ 30 */
	OPLUS_TEMP_COMPENSAITON_30_35_TEMP_INDEX = 6,				/* 30 ~ 35 */
	OPLUS_TEMP_COMPENSATION_35_40_TEMP_INDEX = 7,				/* 35 ~ 40 */
	OPLUS_TEMP_COMPENSATION_40_45_TEMP_INDEX = 8,				/* 40 ~ 45 */
	OPLUS_TEMP_COMPENSATION_45_50_TEMP_INDEX = 9,				/* 45 ~ 50 */
	OPLUS_TEMP_COMPENSATION_GREATER_THAN_50_TEMP_INDEX = 10,	/* > 50 */
};

enum oplus_temp_compensation_setting_mode {
	OPLUS_TEMP_COMPENSATION_BACKLIGHT_SETTING = 0,				/* backlight compensation setting */
	OPLUS_TEMP_COMPENSATION_TEMPERATURE_SETTING = 1,			/* temperature compensation setting */
	OPLUS_TEMP_COMPENSATION_FOD_ON_SETTING = 2,					/* set hbm compensation setting */
	OPLUS_TEMP_COMPENSATION_FOD_OFF_SETTING = 3,				/* recover to normal backlight compensation setting */
	OPLUS_TEMP_COMPENSATION_ESD_SETTING = 4,					/* force to set compensation setting after esd recovery */
	OPLUS_TEMP_COMPENSATION_FIRST_HALF_FRAME_SETTING = 5,		/* force to set compensation setting in the first half frame */
};

/* remember to initialize params */
struct oplus_temp_compensation_params {
	unsigned int config;										/*
																 bit(0):enable temp compensation function
																 bit(1):enable temp compensation funciton but filter the sending of ddic cmds
																 bit(2):enable temp compensation data updating
																*/
	unsigned int reg_repeat;									/*
																 a value used to indicates how many times should the specific temp compensation datas be copied to the specific registers
																 both defalut value && max value are 4
																*/
	unsigned char ***data;										/* the compensation data which are parsed from dtsi */
	unsigned int *dbv_group;									/* the dbv gruop parsed from dtsi which decide how to group the compensation data by backlight */
	int *temp_group;											/* the temp gruop parsed from dtsi which decide how to group the compensation data by temperature */
	unsigned char dbv_group_count;								/* the count of dbv group(sizeof(dbv_group)+1) */
	unsigned char temp_group_count;								/* the count of temp group(sizeof(temp_group)+1) */
	unsigned char voltage_group_count;							/* the count of voltage group */
	struct iio_channel *ntc_temp_chan;							/* a channel used to get ntc temp */
	int ntc_temp;												/* current ntc temp value */
	int shell_temp;												/* current shell temp value */
	bool fake_ntc_temp;											/* indicates whether fake ntc temp is set or not */
	bool fake_shell_temp;										/* indicates whether fake shell temp is set or not */
	bool need_to_set_in_first_half_frame;						/* indicates whether temp compensation params should be set in first half frame or not */
	struct workqueue_struct *first_half_frame_cmd_set_wq;		/* a workqueue used to send temp compensation cmd in the first half frame */
	struct work_struct first_half_frame_cmd_set_work;			/* a work struct used to send temp compensation cmd in the first half frame */
};

/* log level config */
extern unsigned int oplus_temp_compensation_log_level;
/* debug log switch */
extern unsigned int oplus_dsi_log_type;
/* dynamic trace enable */
extern unsigned int oplus_display_trace_enable;

/* debug log */
#define TEMP_COMPENSATION_ERR(fmt, arg...)	\
	do {	\
		if (oplus_temp_compensation_log_level >= OPLUS_TEMP_COMPENSATION_LOG_LEVEL_ERR)	\
			pr_err("[TEMP_COMPENSATION][ERR][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

#define TEMP_COMPENSATION_WARN(fmt, arg...)	\
	do {	\
		if (oplus_temp_compensation_log_level >= OPLUS_TEMP_COMPENSATION_LOG_LEVEL_WARN)	\
			pr_warn("[TEMP_COMPENSATION][WARN][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

#define TEMP_COMPENSATION_INFO(fmt, arg...)	\
	do {	\
		if (oplus_temp_compensation_log_level >= OPLUS_TEMP_COMPENSATION_LOG_LEVEL_INFO)	\
			pr_info("[TEMP_COMPENSATION][INFO][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

#define TEMP_COMPENSATION_DEBUG(fmt, arg...)	\
	do {	\
		if ((oplus_temp_compensation_log_level >= OPLUS_TEMP_COMPENSATION_LOG_LEVEL_DEBUG)	\
				&& (oplus_dsi_log_type & OPLUS_DEBUG_LOG_TEMP_COMPENSATION))	\
			pr_info("[TEMP_COMPENSATION][DEBUG][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

/* debug trace */
#define OPLUS_TEMP_COMPENSAITON_TRACE_BEGIN(name)	\
	do {	\
		if (oplus_display_trace_enable & OPLUS_DISPLAY_TEMP_COMPENSATION_TRACE_ENABLE)	\
			SDE_ATRACE_BEGIN(name);	\
	} while (0)

#define OPLUS_TEMP_COMPENSAITON_TRACE_END(name)	\
	do {	\
		if (oplus_display_trace_enable & OPLUS_DISPLAY_TEMP_COMPENSATION_TRACE_ENABLE)	\
			SDE_ATRACE_END(name);	\
	} while (0)

#define OPLUS_TEMP_COMPENSAITON_TRACE_INT(name, value)	\
	do {	\
		if (oplus_display_trace_enable & OPLUS_DISPLAY_TEMP_COMPENSATION_TRACE_ENABLE)	\
			SDE_ATRACE_INT(name, value);	\
	} while (0)

/* -------------------- function implementation -------------------- */
bool oplus_temp_compensation_is_supported(void);
int oplus_temp_compensation_init(void *dsi_panel);
int oplus_temp_compensation_register_ntc_channel(void *dsi_display);
int oplus_temp_compensation_data_update(void *dsi_display);
int oplus_temp_compensation_cmd_set(void *dsi_panel, unsigned int setting_mode);
void oplus_temp_compensation_first_half_frame_cmd_set_work_handler(struct work_struct *work_item);
int oplus_temp_compensation_first_half_frame_cmd_set(void *dsi_panel);
int oplus_temp_compensation_temp_check(void *dsi_display);
int oplus_temp_compensation_get_ntc_temp(void);

/* -------------------- node -------------------- */
/* config */
ssize_t oplus_temp_compensation_set_config_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);
ssize_t oplus_temp_compensation_get_config_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);
/* ntc temp */
ssize_t oplus_temp_compensation_set_ntc_temp_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);
ssize_t oplus_temp_compensation_get_ntc_temp_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);
/* shell temp */
ssize_t oplus_temp_compensation_set_shell_temp_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count);
ssize_t oplus_temp_compensation_get_shell_temp_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);

#endif /* _OPLUS_DISPLAY_TEMP_COMPENSATION_H_ */
