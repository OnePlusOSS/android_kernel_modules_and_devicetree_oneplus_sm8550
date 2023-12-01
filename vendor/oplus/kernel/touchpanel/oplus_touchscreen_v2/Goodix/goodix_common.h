/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef _TOUCHPANEL_COMMON_GOODIX_H_
#define _TOUCHPANEL_COMMON_GOODIX_H_

#include <linux/uaccess.h>
#include "../touchpanel_common.h"
#include "../touch_comon_api/touch_comon_api.h"
#include "../touchpanel_autotest/touchpanel_autotest.h"
#include "../touchpanel_healthinfo/touchpanel_healthinfo.h"
#include "../touch_pen/touch_pen_core.h"

#include <linux/firmware.h>
#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/proc_fs.h>

#define IS_NUM_OR_CHAR(x) (((x) >= 'A' && (x) <= 'Z') || ((x) >= '0' && (x) <= '9'))

/****************************PART1:auto test define*************************************/
#define Limit_MagicNum1     0x494D494C
#define Limit_MagicNum2     0x474D4954
#define Limit_MagicItem     0x4F50504F

/****************************PART2:FW Update define*************************************/
#define FW_HEAD_SIZE                         128
#define FW_HEAD_SUBSYSTEM_INFO_SIZE          8
#define FW_HEAD_OFFSET_SUBSYSTEM_INFO_BASE   32
#define PACK_SIZE                            256
#define UPDATE_STATUS_IDLE                   0
#define UPDATE_STATUS_RUNNING                1
#define UPDATE_STATUS_ABORT                  2
#define FW_SECTION_TYPE_SS51_PATCH           0x02

#define CMD_STR(name) #name

typedef enum {
	GTP_RAWDATA,
	GTP_DIFFDATA,
	GTP_BASEDATA,
	GTP_FW_STATUS,
	GTP_NORMAL_LIZE,
} debug_type;

struct goodix_proc_operations {
	void (*goodix_config_info_read)(struct seq_file *s, void *chip_data);
};

struct fw_update_info {
	u8              *buffer;
	u8              *firmware_file_data;
	u32             fw_length;
	int             status;
	int             progress;
	int             max_progress;
	struct fw_info  *firmware;
};

typedef enum {
	BASE_DC_COMPONENT = 0X01,
	BASE_SYS_UPDATE = 0X02,
	BASE_NEGATIVE_FINGER = 0x03,
	BASE_MONITOR_UPDATE = 0x04,
	BASE_CONSISTENCE = 0x06,
	BASE_FORCE_UPDATE = 0x07,
} BASELINE_ERR;

typedef enum {
	RST_MAIN_REG = 0x01,
	RST_OVERLAY_ERROR = 0x02,
	RST_LOAD_OVERLAY = 0x03,
	RST_CHECK_PID = 0x04,
	RST_CHECK_RAM = 0x06,
	RST_CHECK_RAWDATA = 0x07,
} RESET_REASON;

typedef enum {
	ESD_RAM_ERROR = 0x01,
	ESD_RAWDATA_ERROR = 0x07,
} ESD_REASON;

typedef enum {
	HSYNC_NO_INPUT = 0x01,
	HSYNC_FREQ_HOPPING = 0x02,
} HSYNC_ERR;

struct goodix_health_info {
	uint8_t   shield_water: 1;
	uint8_t   shield_freq: 1;
	uint8_t   baseline_refresh: 1;
	uint8_t   fw_rst: 1;
	uint8_t   shield_palm: 1;
	uint8_t   reserve_bit: 3;
	uint8_t   reserve;
	uint8_t   freq_before;
	uint8_t   freq_after;
	uint8_t   baseline_refresh_type;
	uint8_t   reset_reason;
	uint8_t   reserve1;
	uint8_t   reserve2;
	uint8_t   reserve3;
	uint8_t   reserve4;
	uint8_t   reserve5;
	uint8_t   reserve6;
	uint8_t   reserve7;
	uint8_t   reserve8;
	uint8_t   reserve9;
	uint8_t   reserve10;
	uint8_t   reserve11;
	uint8_t   reserve12;
	uint8_t   reserve13;
	uint8_t   reserve14;
	uint8_t   checksum;
};


struct goodix_health_info_v2 {
	uint8_t   shield_water: 1;
	uint8_t   shield_freq: 1;
	uint8_t   baseline_refresh: 1;
	uint8_t   esd_rst: 1;
	uint8_t   shield_palm: 1;
	uint8_t   charger_mode: 1;
	uint8_t   low_temperature: 1;
	uint8_t   broken_compensated: 1;
	uint8_t   sync_error: 1;
	uint8_t   reserve_bit: 7;
	uint8_t   shield_water_state;
	uint8_t   freq_before_low;
	uint8_t   freq_before_high;
	uint8_t   freq_after_low;
	uint8_t   freq_after_high;
	uint8_t   baseline_refresh_type;
	uint8_t   esd_reason; /* 0x01:RAM error; 0x07:Rawdata error */
	uint8_t   noise_level_before;
	uint8_t   noise_level_after;
	uint8_t   reserve11;
	uint8_t   hsync_error; /* 0x01:no Hsync; 0x02:Hsync freq hop */
	uint8_t   hsync_freq_before_byte0;
	uint8_t   hsync_freq_before_byte1;
	uint8_t   hsync_freq_before_byte2;
	uint8_t   hsync_freq_before_byte3;
	uint8_t   hsync_freq_after_byte0;
	uint8_t   hsync_freq_after_byte1;
	uint8_t   hsync_freq_after_byte2;
	uint8_t   hsync_freq_after_byte3;
	uint8_t   reserve21;
	uint8_t   reserve22;
	uint8_t   reserve23;
	uint8_t   reserve24;
	uint8_t   reserve25;
	uint8_t   reserve26;
	uint8_t   reserve27;
	uint8_t   reserve28;
	uint8_t   reserve29;
	uint8_t   checksum_low;
	uint8_t   checksum_high;
};

/*test item*/
enum {
	TYPE_ERROR                   = 0x00,
	TYPE_TEST1                   = 0x01,
	TYPE_TEST2                   = 0x02,
	TYPE_TEST3                   = 0x03,
	TYPE_TEST4                   = 0x04,
	TYPE_TEST5                   = 0x05,
	TYPE_TEST6					 = 0x06,
	TYPE_MAX                     = 0xFF,
};

/* return value : <0 error;*/
struct goodix_auto_test_operations {
	int (*test1)(struct seq_file *s, void *chip_data,
		     struct auto_testdata *goodix_testdata,
		     struct test_item_info *p_test_item_info);
	int (*test2)(struct seq_file *s, void *chip_data,
		     struct auto_testdata *goodix_testdata,
		     struct test_item_info *p_test_item_info);
	int (*test3)(struct seq_file *s, void *chip_data,
		     struct auto_testdata *goodix_testdata,
		     struct test_item_info *p_test_item_info);
	int (*test4)(struct seq_file *s, void *chip_data,
		     struct auto_testdata *goodix_testdata,
		     struct test_item_info *p_test_item_info);
	int (*test5)(struct seq_file *s, void *chip_data,
		     struct auto_testdata *goodix_testdata,
		     struct test_item_info *p_test_item_info);
	int (*test6)(struct seq_file *s, void *chip_data,
		     struct auto_testdata *goodix_testdata,
		     struct test_item_info *p_test_item_info);
	int (*auto_test_preoperation)(struct seq_file *s, void *chip_data,
				      struct auto_testdata *goodix_testdata,
				      struct test_item_info *p_test_item_info);
	int (*auto_test_endoperation)(struct seq_file *s, void *chip_data,
				      struct auto_testdata *goodix_testdata,
				      struct test_item_info *p_test_item_info);
};

/****************************PART3:FUNCTION*************************************/
int goodix_create_proc(struct touchpanel_data *ts,
		       struct goodix_proc_operations *goodix_ops);
int goodix_remove_proc(struct touchpanel_data *ts);
int goodix_auto_test(struct seq_file *s,  struct touchpanel_data *ts);
void goodix_print_differ(s16 *diff_buf, u32 buf_size, int tx, int rx);

#endif  /*_TOUCHPANEL_COMMON_GOODIX_H_*/

