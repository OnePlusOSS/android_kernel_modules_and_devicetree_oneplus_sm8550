/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef __FT3681_CORE_H__
#define __FT3681_CORE_H__

/*********PART1:Head files**********************/
#include "../focal_common.h"

/*********PART2:Define Area**********************/

#define RESET_TO_NORMAL_TIME                    200        /*Sleep time after reset*/
#define POWEWRUP_TO_RESET_TIME                  10

#define INTERVAL_READ_REG                       200  /* unit:ms */
#define TIMEOUT_READ_REG                        1000 /* unit:ms */

#define FTS_VAL_CHIP_ID                         0x56
#define FTS_VAL_CHIP_ID2                        0x62
#define FTS_VAL_BT_ID                           0x56
#define FTS_VAL_BT_ID2                          0x62
#define FTS_VAL_PB_ID                           0x56
#define FTS_VAL_PB_ID2                          0xE2

#define FTS_REG_POINTS                          0x01
#define FTS_REG_POINTS_N                        0x10
#define FTS_REG_POINTS_LB                       0x3E

#define FTS_REG_SMOOTH_LEVEL                    0x85
#define FTS_REG_REPORT_RATE                     0x88/*0x12:180hz, 0x0C:120hz*/
#define FTS_REG_HIGH_FRAME_TIME                 0x8A
#define FTS_REG_SENSITIVE_LEVEL                 0x90
#define FTS_REG_STABLE_DISTANCE_AFTER_N         0xB9
#define FTS_REG_STABLE_DISTANCE                 0xBA
#define FTS_REG_FOD_EN                          0xCF
#define FTS_REG_FOD_INFO                        0xE1
#define FTS_REG_FOD_INFO_LEN                    9
#define FTS_REG_AOD_INFO                        0xD3
#define FTS_REG_AOD_INFO_LEN                    6


#define FTS_REG_GAME_MODE_EN                    0xC3
#define FTS_REG_CHARGER_MODE_EN                 0x8B
#define FTS_REG_EDGE_LIMIT                      0x8C
#define FTS_REG_HEADSET_MODE_EN                 0xC4

#define FTS_REG_CTRL                            0x8B
#define FTS_REG_CHARGER_MODE_EN_BIT             0x00
#define FTS_REG_GAME_MODE_EN_BIT                0x02
#define FTS_REG_EDGE_LIMIT_BIT                  0x04
#define FTS_REG_HEADSET_MODE_EN_BIT             0x06

#define FTS_REG_INT_CNT                         0x8F
#define FTS_REG_FLOW_WORK_CNT                   0x91
#define FTS_REG_CHIP_ID                         0xA3
#define FTS_REG_CHIP_ID                         0xA3
#define FTS_REG_CHIP_ID2                        0x9F
#define FTS_REG_POWER_MODE                      0xA5
#define FTS_REG_FW_VER                          0xA6
#define FTS_REG_VENDOR_ID                       0xA8
#define FTS_REG_FREQUENCE_WATER_MODE		0xBF
#define FTS_REG_GESTURE_EN                      0xD0
#define FTS_REG_GESTURE_CONFIG1                 0xD1
#define FTS_REG_GESTURE_CONFIG2                 0xD2
#define FTS_REG_GESTURE_CONFIG3                 0xD5
#define FTS_REG_GESTURE_CONFIG4                 0xD6
#define FTS_REG_GESTURE_CONFIG5                 0xD7
#define FTS_REG_GESTURE_CONFIG6                 0xD8
#define FTS_REG_GESTURE_OUTPUT_ADDRESS          0xD3
#define FTS_REG_MODULE_ID                       0xE3
#define FTS_REG_LIC_VER                         0xE4
#define FTS_REG_AUTOCLB_ADDR                    0xEE
#define FTS_REG_SAMSUNG_SPECIFAL                0xFA
#define FTS_REG_HEALTH_1                        0xFD
#define FTS_REG_HEALTH_2                        0xFE

#define FTS_REG_WORK_MODE                       0x9E
#define FTS_REG_WORK_MODE_SNR_MODE              0x81
#define FTS_REG_WORK_MODE_FINAL_DIFF_MODE       0x01
#define FTS_REG_WORK_MODE_NORMAL_MODE           0x00
#define FTS_FW_INFO                             0x96
#define FTS_REG_TEMPERATURE                     0x97

#define FTS_REG_PALM_TO_SLEEP_STATUS            0x9B


#define FTS_MAX_POINTS_SUPPORT                  10
#define FTS_MAX_ID                              0x0A

#define FTS_MAX_POINTS_LENGTH                   102 /* 2+6*10+4*10 */
#define FTS_MAX_POINTS_SNR_LENGTH               1505 /* 2+6*10+4*10 + 2 + 2*tx*rx + (tx+rx)*2*2 + 1 + 40 */
#define FTS_DIFF_BUF_LENGTH                     576 /* tx*rx */
#define FTS_SC_BUF_LENGTH                       52 /* tx+rx */

#define FTS_GESTURE_DATA_LEN                    28

#define BYTES_PER_TIME                          (128)  /* max:128 */

/*
 * factory test registers
 */
#define ENTER_WORK_FACTORY_RETRIES              5
#define DEVIDE_MODE_ADDR                        0x00
#define FTS_FACTORY_MODE_VALUE                  0x40
#define FTS_WORK_MODE_VALUE                     0x00
#define FACTORY_TEST_RETRY                      50
#define FACTORY_TEST_DELAY                      18
#define FACTORY_TEST_RETRY_DELAY                100

/* mc_sc */
#define FACTORY_REG_LINE_ADDR                   0x01
#define FACTORY_REG_CHX_NUM                     0x02
#define FACTORY_REG_CHY_NUM                     0x03
#define FACTORY_REG_CLB                         0x04
#define FACTORY_REG_DATA_SELECT                 0x06
#define FACTORY_REG_FRE_LIST                    0x0A
#define FACTORY_REG_DATA_TYPE                   0x5B
#define FACTORY_REG_TOUCH_THR                   0x0D
#define FACTORY_REG_NORMALIZE                   0x16
#define FACTORY_REG_MAX_DIFF                    0x1B
#define FACTORY_REG_FRAME_NUM                   0x1C
#define FACTORY_REG_GCB                         0xBD

#define FACTORY_REG_RAWDATA_ADDR_MC_SC          0x36
#define FACTORY_REG_FIR                         0xFB
#define FACTORY_REG_WC_SEL                      0x09
#define FACTORY_REG_MC_SC_MODE                  0x44
#define FACTORY_REG_HC_SEL                      0x0F
#define FACTORY_REG_MC_SC_CB_H_ADDR_OFF         0x49
#define FACTORY_REG_MC_SC_CB_ADDR_OFF           0x45
#define FACTORY_REG_MC_SC_CB_ADDR               0x4E
#define FACTROY_REG_SHORT_TEST_EN               0x07
#define FACTROY_REG_SHORT_CA                    0x01
#define FACTROY_REG_SHORT_CC                    0x02
#define FACTROY_REG_SHORT_CG                    0x03
#define FACTROY_REG_SHORT_OFFSET                0x04
#define FACTROY_REG_SHORT_AB_CH                 0x58
#define FACTROY_REG_SHORT_DELAY                 0x5A
#define FACTORY_REG_SHORT_ADDR_MC               0xF4

#define FACTROY_REG_SCAP_CFG                    0x58
#define FACTROY_REG_SCAP_GCB_TX                 0xBC
#define FACTROY_REG_SCAP_GCB_RX                 0xBE
#define FACTROY_REG_CB_BUF_SEL                  0xBF

#define FACTROY_REG_SHORT2_TEST_EN              0xC0
#define FACTROY_REG_SHORT2_CA                   0x01
#define FACTROY_REG_SHORT2_CC                   0x02
#define FACTROY_REG_SHORT2_CG                   0x03
#define FACTROY_REG_SHORT2_OFFSET               0x04
#define FACTROY_REG_SHORT2_RES_LEVEL            0xC1
#define FACTROY_REG_SHORT2_DEALY                0xC2
#define FACTROY_REG_SHORT2_TEST_STATE           0xC3
#define FACTORY_REG_SHORT2_ADDR_MC              0xC4
#define FACTROY_REG_SHORT2_AB_CH                0xC6

#define SC_NUM_MAX                              256


#define FACTORY_REG_PARAM_UPDATE_STATE_TOUCH    0xB5

#define FTS_MAX_COMMMAND_LENGTH                 16

#define TEST_RETVAL_00                          0x00
#define TEST_RETVAL_AA                          0xAA

#define FTS_EVENT_FOD                           0x26

#define MAX_PACKET_SIZE                         128

#define FTS_120HZ_REPORT_RATE                   0x0C
#define FTS_180HZ_REPORT_RATE                   0x12
#define FTS_240HZ_REPORT_RATE                   0x18
#define FTS_360HZ_REPORT_RATE                   0x24
#define FTS_720HZ_REPORT_RATE                   0x24            /*not support*/

#define FTS_NOT_GAME_MODE                       0x00
#define FTS_240HZ_GAME_MODE                     0x01
#define FTS_360HZ_GAME_MODE                     0x02
#define FTS_720HZ_GAME_MODE                     0x03

#define FTS_GET_RATE_120                        120
#define FTS_GET_RATE_240                        10
#define FTS_GET_RATE_300                        300
#define FTS_GET_RATE_600                        600

#define FTS_WRITE_RATE_120                      120
#define FTS_WRITE_RATE_180                      180
#define FTS_WRITE_RATE_240                      240
#define FTS_WRITE_RATE_360                      360
#define FTS_WRITE_RATE_720                      720

#define SPI_RETRY_NUMBER          				3
#define CS_HIGH_DELAY             				150 /* unit: us */
#define SPI_BUF_LENGTH          			    4096

#define DATA_CRC_EN               				0x20
#define WRITE_CMD                 				0x00
#define READ_CMD                    			(0x80 | DATA_CRC_EN)

#define SPI_DUMMY_BYTE             				3
#define SPI_HEADER_LENGTH           			6   /*CRC*/

#define PITCH_X_WIDTH 540
#define PITCH_Y_WIDTH 536
#define PITCH_PHY_WIDTH_MM 4

#define GET_LEN_BY_WIDTH_MAJOR(width_major, len)\
({\
	if (width_major > 10 && width_major < 14)\
		*len = 5;\
	if (width_major > 12 && width_major < 16)\
		*len = 5;\
	if (width_major > 16 && width_major < 20)\
		*len = 7;\
	if (width_major > 18 && width_major < 22)\
		*len = 7;\
})

static u8 soc_membist_test_cmd[28][11] = {\
	{0x70, 0x55, 0xAA}, \
	{0x70, 0x07, 0xF8, 0x00, 0x04, 0x00, 0x00, 0xAA, 0x55, 0x00, 0x00}, \
	{0x70, 0x07, 0xF8, 0x00, 0x02, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00}, \
	{0x70, 0x07, 0xF8, 0x01, 0x02, 0x00, 0x00, 0x77, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x01, 0x00, 0x00, 0x00, 0x01, 0xcd, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x01, 0x00, 0x00, 0x00, 0x7f, 0x2f, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x01, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x01, 0x00, 0x00, 0x00, 0x08, 0xcf, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x01, 0x00, 0x00, 0x00, 0xc5, 0x6a, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x01, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x01, 0x00, 0x00, 0x00, 0x05, 0xcf, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x01, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x01, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x01, 0x00, 0x00, 0x00, 0x41, 0xd0, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x01, 0x00, 0x00, 0x00, 0x82, 0xc1, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x01, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x01, 0x00, 0x00, 0x00, 0x08, 0x4f, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xF8, 0x00, 0x4B, 0x00, 0x00, 0xFA, 0x07, 0x00, 0x00}, \
	{0x70, 0x07, 0xF8, 0x00, 0x4B, 0x00, 0x00, 0xFA, 0x00, 0x00, 0x00}, \
	{0x70, 0x06, 0xF9, 0x00, 0x4B, 0x00, 0x00}, \
	{0x70, 0x06, 0xF9, 0x00, 0x4F, 0x00, 0x00}, \
	{0x70, 0x06, 0xF9, 0x00, 0x4E, 0x00, 0x00}, \
};

static u8 afe_membist_test_cmd[38][11] = { \
	{0x70, 0x07, 0xf8, 0x80, 0x49, 0x00, 0x00, 0x00, 0x50, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x02, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x00, 0x00, 0x00, 0x01, 0x8d, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x00, 0x00, 0x00, 0x05, 0x8d, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x00, 0x00, 0x00, 0x59, 0x91, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x00, 0x00, 0x00, 0x5b, 0x91, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x00, 0x00, 0x00, 0x7e, 0x90, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x00, 0x00, 0x00, 0x7f, 0x90, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x00, 0x00, 0x00, 0x83, 0x90, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x00, 0x00, 0x00, 0x84, 0x90, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x00, 0x00, 0x00, 0x85, 0x90, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
};

static u8 afe_membist_test_read[5][11] = { \
	{0x70, 0x07, 0xf8, 0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x00, 0x00, 0x00, 0x80, 0x10, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
	{0x70, 0x06, 0xf9, 0x81, 0x01, 0x00, 0x00}, \
	{0x70, 0x07, 0xf8, 0x81, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, \
};

struct fts_autotest_offset {
	int32_t *fts_raw_data_P;
	int32_t *fts_raw_data_N;
	int32_t *fts_panel_differ_data_P;
	int32_t *fts_panel_differ_data_N;
	int32_t *fts_noise_data_P;
	int32_t *fts_noise_data_N;
	int32_t *fts_uniformity_data_P;
	int32_t *fts_uniformity_data_N;
	int32_t *fts_scap_cb_data_P;
	int32_t *fts_scap_cb_data_N;
	int32_t *fts_scap_cb_data_waterproof_P;
	int32_t *fts_scap_cb_data_waterproof_N;
	int32_t *fts_scap_raw_data_P;
	int32_t *fts_scap_raw_data_N;
	int32_t *fts_scap_raw_waterproof_data_P;
	int32_t *fts_scap_raw_waterproof_data_N;
};

enum FW_STATUS {
	FTS_RUN_IN_ERROR,
	FTS_RUN_IN_APP,
	FTS_RUN_IN_ROM,
	FTS_RUN_IN_PRAM,
	FTS_RUN_IN_BOOTLOADER,
};

struct fts_fod_info {
	u8 fp_id;
	u8 event_type;
	u8 fp_area_rate;
	u8 tp_area;
	u16 fp_x;
	u16 fp_y;
	u8 fp_down;
	u8 fp_down_report;
};

typedef enum {
	TYPE_NO_FOD_TRIGGER = 0,
	TYPE_SMALL_FOD_TRIGGER,
	TYPE_FOD_TRIGGER,
} fod_trigger_type;

struct fts_aod_info {
	u8 gesture_id;
	u8 point_num;
	u16 aod_x;
	u16 aod_y;
};

struct ftxxxx_proc {
	struct proc_dir_entry *proc_entry;
	u8 opmode;
	u8 cmd_len;
	u8 cmd[FTS_MAX_COMMMAND_LENGTH];
};

struct chip_data_ft3681 {
	bool esd_check_need_stop;   /*true:esd check do nothing*/
	bool esd_check_enabled;
	bool use_panelfactory_limit;
	bool is_power_down;
	bool is_ic_sleep;    /*ic sleep status*/
	u8 touch_buf[FTS_MAX_POINTS_SNR_LENGTH];
	u8 irq_type;
	u8 touch_direction;
	u8 fp_en;
	u8 fp_down;
	u8 fwver;
	u32 touch_size;

	u8 snr_buf[FTS_MAX_POINTS_SNR_LENGTH];
	int diff_buf[FTS_DIFF_BUF_LENGTH];
	int sc_water[FTS_SC_BUF_LENGTH];
	int sc_nomal[FTS_SC_BUF_LENGTH];

	int rl_cnt;
	int scb_cnt;
	int srawdata_cnt;
	int last_mode;
	int csv_fd;
	int irq_num;
	int probe_done;
	int *noise_rawdata;
	int *rawdata;
	int *panel_differ;
	int *scap_cb;
	int *scap_rawdata;
	int *rawdata_linearity;
	int tp_index;
	u8 fre_num;
	u8 ctrl_reg_state;
	u8 snr_count;
	u8 differ_mode;
	int tp_temperature;
	int freq_point;

	int scap_gcb_rx;
	int scap_gcb_tx;

	char *test_limit_name;
	char *fw_name;

	unsigned char *bus_tx_buf;
	unsigned char *bus_rx_buf;

	tp_dev tp_type;             /*tp_devices.h*/

	struct spi_device *ft_spi;
	struct device *dev;
	struct hw_resource *hw_res;
	struct fts_proc_operations *syna_ops;
	struct fts_fod_info fod_info;
	struct fts_aod_info aod_info;
	struct seq_file *s;
	struct fts_autotest_offset *fts_autotest_offset;
	struct touchpanel_data *ts;
	struct monitor_data *monitor_data;
	struct mutex bus_lock;
	struct ftxxxx_proc proc;

	int *node_valid;
	int *node_valid_sc;
	int gesture_state;
	bool black_gesture_indep;
	bool switch_game_rate_support;
	bool high_resolution_support;
	bool high_resolution_support_x8;
	bool high_resolution_support_x16;
	bool snr_is_reading;
	bool snr_read_support;
	bool snr_data_is_ready;
	bool differ_read_every_frame;
	bool tp_data_record_support;

	bool is_in_water;
	fod_trigger_type fod_trigger;

	bool charger_connected;
	u32 spi_speed;
};


extern struct chip_data_ft3681 *g_fts3681_data;

int focal_create_apk_debug_channel(struct touchpanel_data *ts);
int fts3681_test_entry(struct chip_data_ft3681 *ts_data,
		   struct auto_testdata *focal_testdata);
int ft3681_auto_preoperation(struct seq_file *s, void *chip_data,
			     struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);
int ft3681_noise_autotest(struct seq_file *s, void *chip_data,
			  struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);
int ft3681_rawdata_autotest(struct seq_file *s, void *chip_data,
			    struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);
int ft3681_uniformity_autotest(struct seq_file *s, void *chip_data,
			       struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);
int ft3681_scap_cb_autotest(struct seq_file *s, void *chip_data,
			    struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);
int ft3681_scap_rawdata_autotest(struct seq_file *s, void *chip_data,
				 struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);
int ft3681_short_test(struct seq_file *s, void *chip_data,
		      struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);
int ft3681_panel_differ_test(struct seq_file *s, void *chip_data,
                             struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);
int ft3681_auto_endoperation(struct seq_file *s, void *chip_data,
			     struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);

int ft3681_membist_test(struct seq_file *s, void *chip_data,
								struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);
int ft3681_cal_test(struct seq_file *s, void *chip_data,
							struct auto_testdata *focal_testdata, struct test_item_info *p_test_item_info);
int ft3681_fts_hw_reset(struct chip_data_ft3681 *ts_data, u32 delayms);

int ft3681_fts_read(u8 *cmd, u32 cmdlen, u8 *data, u32 datalen);
int ft3681_fts_write(u8 *writebuf, u32 writelen);
int ft3681_fts_write_reg(u8 addr, u8 value);
int ft3681_fts_read_reg(u8 addr, u8 *value);

int fts_spi_write_direct(u8 *writebuf, u32 writelen);
int fts_spi_read_direct(u8 *writebuf, u32 writelen, u8 *readbuf, u32 readlen);
int fts_ft3681_write_cal_pramboot(void *chip_data);
int fts_set_spi_max_speed(u32 speed, u8 mode);

#endif /*__FT3681_CORE_H__*/
