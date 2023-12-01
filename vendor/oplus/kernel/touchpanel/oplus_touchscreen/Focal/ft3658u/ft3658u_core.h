/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef __FT3658U_CORE_H__
#define __FT3658U_CORE_H__

/*********PART1:Head files**********************/
#include "../focal_common.h"

/*********PART2:Define Area**********************/

#define RESET_TO_NORMAL_TIME                    200        /*Sleep time after reset*/
#define POWEWRUP_TO_RESET_TIME                  10

#define INTERVAL_READ_REG                       200  /* unit:ms */
#define TIMEOUT_READ_REG                        1000 /* unit:ms */

#define FT3658U_VAL_CHIP_ID                         0x56
#define FT3658U_VAL_CHIP_ID2                        0x52
#define FT3658U_VAL_BT_ID                           0x56
#define FT3658U_VAL_BT_ID2                          0xB2

#define FT3658U_REG_POINTS                          0x01
#define FT3658U_REG_POINTS_N                        0x10
#define FT3658U_REG_POINTS_LB                       0x3E

#define FT3658U_REG_SMOOTH_LEVEL                    0x85
#define FT3658U_REG_REPORT_RATE                     0x88/*0x12:180hz, 0x0C:120hz*/
#define FT3658U_REG_HIGH_FRAME_TIME                 0x8A
#define FT3658U_REG_SENSITIVE_LEVEL                 0x90
#define FT3658U_REG_STABLE_DISTANCE_AFTER_N         0xB9
#define FT3658U_REG_STABLE_DISTANCE                 0xBA
#define FT3658U_REG_FOD_EN                          0xCF
#define FT3658U_REG_FOD_INFO                        0xE1
#define FT3658U_REG_FOD_INFO_LEN                    9


#define FT3658U_REG_GAME_MODE_EN                    0xC3
#define FT3658U_REG_CHARGER_MODE_EN                 0x8B
#define FT3658U_REG_EDGE_LIMIT                      0x8C
#define FT3658U_REG_HEADSET_MODE_EN                 0xC4

#define FT3658U_REG_CTRL                            0x8B
#define FT3658U_REG_CHARGER_MODE_EN_BIT             0x00
#define FT3658U_REG_GAME_MODE_EN_BIT                0x02
#define FT3658U_REG_EDGE_LIMIT_BIT                  0x04
#define FT3658U_REG_HEADSET_MODE_EN_BIT             0x06

#define FT3658U_REG_INT_CNT                         0x8F
#define FT3658U_REG_FLOW_WORK_CNT                   0x91
#define FT3658U_REG_CHIP_ID                         0xA3
#define FT3658U_REG_CHIP_ID                         0xA3
#define FT3658U_REG_CHIP_ID2                        0x9F
#define FT3658U_REG_POWER_MODE                      0xA5
#define FT3658U_REG_FW_VER                          0xA6
#define FT3658U_REG_VENDOR_ID                       0xA8
#define FT3658U_REG_GESTURE_EN                      0xD0
#define FT3658U_REG_GESTURE_CONFIG1                 0xD1
#define FT3658U_REG_GESTURE_CONFIG2                 0xD2
#define FT3658U_REG_GESTURE_CONFIG3                 0xD5
#define FT3658U_REG_GESTURE_CONFIG4                 0xD6
#define FT3658U_REG_GESTURE_CONFIG5                 0xD7
#define FT3658U_REG_GESTURE_CONFIG6                 0xD8
#define FT3658U_REG_GESTURE_OUTPUT_ADDRESS          0xD3
#define FT3658U_REG_MODULE_ID                       0xE3
#define FT3658U_REG_LIC_VER                         0xE4
#define FT3658U_REG_AUTOCLB_ADDR                    0xEE
#define FT3658U_REG_SAMSUNG_SPECIFAL                0xFA
#define FT3658U_REG_HEALTH_1                        0xFD
#define FT3658U_REG_HEALTH_2                        0xFE
#define FT3658U_REG_RESOLUTION_INFORMATION          0x96
#define FT3658U_REG_TEMPERATURE                     0x97
#define FT3658U_REG_REPORT_RATE_2K                     0x99

#define FT3658U_REG_PALM_TO_SLEEP_SWITCH            0x9A
#define FT3658U_REG_PALM_TO_SLEEP_STATUS            0x9B

#define FT3658U_REG_WORK_MODE                       0x9E
#define FT3658U_REG_WORK_MODE_SNR_MODE              0x81
#define FT3658U_REG_WORK_MODE_FINAL_DIFF_MODE       0x01
#define FT3658U_REG_WORK_MODE_NORMAL_MODE           0x00


#define FT3658U_MAX_POINTS_SUPPORT                  10
#define FT3658U_MAX_ID                              0x0A

#define FT3658U_MAX_POINTS_LENGTH                   102 /* 2+6*10+4*10 */
#define FT3658U_MAX_POINTS_SNR_LENGTH               1505 /* 2+6*10+4*10 + 2 + 2*tx*rx + (tx+rx)*2*2 + 1 + 40 */
#define FT3658U_DIFF_BUF_LENGTH                     576 /* tx*rx */
#define FT3658U_SC_BUF_LENGTH                       52 /* tx+rx */

#define FT3658U_GESTURE_DATA_LEN                    28

#define BYTES_PER_TIME                          (128)  /* max:128 */

/*
 * factory test registers
 */
#define ENTER_WORK_FACTORY_RETRIES              5
#define DEVIDE_MODE_ADDR                        0x00
#define FT3658U_FACTORY_MODE_VALUE                  0x40
#define FT3658U_WORK_MODE_VALUE                     0x00
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

#define FT3658U_MAX_COMMMAND_LENGTH                 16

#define TEST_RETVAL_00                          0x00
#define TEST_RETVAL_AA                          0xAA

#define FT3658U_EVENT_FOD                           0x26

#define MAX_PACKET_SIZE                         128

#define FT3658U_120HZ_REPORT_RATE                   0x0C
#define FT3658U_180HZ_REPORT_RATE                   0x12
#define FT3658U_240HZ_REPORT_RATE                   0x18

#define FTS_120HZ_REPORT_RATE                   0x00
#define FTS_180HZ_REPORT_RATE                   0x01
#define FTS_240HZ_REPORT_RATE                   0x02
#define FTS_360HZ_REPORT_RATE                   0x03


#define FTS_GET_RATE_0                       0
#define FTS_GET_RATE_180                        180
#define FTS_GET_RATE_300                        300
#define FTS_GET_RATE_600                        600

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


struct mc_sc_threshold {
	int noise_coefficient;
	int short_cg;
	int short_cc;
	int *node_valid;
	int *node_valid_sc;
	int *rawdata_h_min;
	int *rawdata_h_max;
	int *tx_linearity_max;
	int *tx_linearity_min;
	int *rx_linearity_max;
	int *rx_linearity_min;
	int *scap_cb_off_min;
	int *scap_cb_off_max;
	int *scap_cb_on_min;
	int *scap_cb_on_max;
	int *scap_rawdata_off_min;
	int *scap_rawdata_off_max;
	int *scap_rawdata_on_min;
	int *scap_rawdata_on_max;
	int *panel_differ_min;
	int *panel_differ_max;
};

struct ft3658u_test {
	struct mc_sc_threshold thr;
};

struct ft3658u_autotest_offset {
	int32_t *ft3658u_raw_data_P;
	int32_t *ft3658u_raw_data_N;
	int32_t *ft3658u_panel_differ_data_P;
	int32_t *ft3658u_panel_differ_data_N;
	int32_t *ft3658u_noise_data_P;
	int32_t *ft3658u_noise_data_N;
	int32_t *ft3658u_uniformity_data_P;
	int32_t *ft3658u_uniformity_data_N;
	int32_t *ft3658u_scap_cb_data_P;
	int32_t *ft3658u_scap_cb_data_N;
	int32_t *ft3658u_scap_cb_data_waterproof_P;
	int32_t *ft3658u_scap_cb_data_waterproof_N;
	int32_t *ft3658u_scap_raw_data_P;
	int32_t *ft3658u_scap_raw_data_N;
	int32_t *ft3658u_scap_raw_waterproof_data_P;
	int32_t *ft3658u_scap_raw_waterproof_data_N;
};

enum FW_STATUS {
	FT3658U_RUN_IN_ERROR,
	FT3658U_RUN_IN_APP,
	FT3658U_RUN_IN_ROM,
	FT3658U_RUN_IN_PRAM,
	FT3658U_RUN_IN_BOOTLOADER,
};

struct ft3658u_fod_info {
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

struct ftxxxx_proc {
	struct proc_dir_entry *proc_entry;
	u8 opmode;
	u8 cmd_len;
	u8 cmd[FT3658U_MAX_COMMMAND_LENGTH];
};

struct chip_data_ft3658u {
	bool esd_check_need_stop;   /*true:esd check do nothing*/
	bool esd_check_enabled;
	bool use_panelfactory_limit;
	bool is_power_down;
	bool is_ic_sleep;    /*ic sleep status*/
	u8 touch_buf[FT3658U_MAX_POINTS_SNR_LENGTH];
	u8 irq_type;
	u8 touch_direction;
	u8 fp_en;
	u8 fp_down;
	u8 fwver;
	u32 touch_size;

	int diff_buf[FT3658U_DIFF_BUF_LENGTH];
	int sc_water[FT3658U_SC_BUF_LENGTH];
	int sc_nomal[FT3658U_SC_BUF_LENGTH];
	int snr_count;

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
	int null_noise_value;
	int tp_index;
	u8 fre_num;
	u8 ctrl_reg_state;
	u8 work_mode;

	int scap_gcb_rx_wateron;
	int scap_gcb_tx_wateron;

	int scap_gcb_rx;
	int scap_gcb_tx;

	char *test_limit_name;
	char *fw_name;

	u8 *h_fw_file;
	u32 h_fw_size;

	unsigned char *bus_tx_buf;
	unsigned char *bus_rx_buf;

	tp_dev tp_type;             /*tp_devices.h*/

	struct spi_device *ft_spi;
	struct device *dev;
	struct hw_resource *hw_res;
	struct fts_proc_operations *syna_ops;
	struct ft3658u_fod_info fod_info;
	struct seq_file *s;
	struct ft3658u_test mpt;
	struct ft3658u_autotest_offset *ft3658u_autotest_offset;
	struct touchpanel_data *ts;
	struct mutex bus_lock;
	struct ftxxxx_proc proc;
	struct monitor_data_v2 *monitor_data_v2;
	struct firmware_headfile       *p_firmware_headfile;
	int *node_valid;
	int *node_valid_sc;
	int gesture_state;
	bool black_gesture_indep;
	bool switch_game_rate_support;
	bool high_resolution_support;
	bool high_resolution_support_x8;
	int x_resolution;
	int y_resolution;
	int coordinate_scale;
	int tp_temperature;

	bool is_in_water;
	fod_trigger_type fod_trigger;

	bool differ_is_reading;
	bool differ_read_every_frame;

	bool charger_connected;
	u32 spi_speed;
};


extern struct chip_data_ft3658u *g_ft3658u_data;

int focal_create_apk_debug_channel(struct touchpanel_data *ts);
int ft3658u_test_entry(struct chip_data_ft3658u *ts_data);

int ft3658u_hw_reset(struct chip_data_ft3658u *ts_data, u32 delayms);

int ft3658u_read(u8 *cmd, u32 cmdlen, u8 *data, u32 datalen);
int ft3658u_write(u8 *writebuf, u32 writelen);
int ft3658u_write_reg(u8 addr, u8 value);
int ft3658u_read_reg(u8 addr, u8 *value);

int ft3658u_rstpin_reset(void *chip_data);

int ft3658u_spi_write_direct(u8 *writebuf, u32 writelen);
int ft3658u_spi_read_direct(u8 *writebuf, u32 writelen, u8 *readbuf, u32 readlen);
int ft3658u_set_spi_max_speed(u32 speed, u8 mode);

#endif /*__FT3658U_CORE_H__*/
