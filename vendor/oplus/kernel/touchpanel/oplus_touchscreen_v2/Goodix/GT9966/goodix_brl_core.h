/**************************************************************
 * Copyright (c)  2008- 2030  Oplus Mobile communication Corp.ltd.
 * File       : goodix_drivers_gt9886.h
 * Description: header file for Goodix GT9886 driver
 * Version   : 1.0
 * Date        : 2019-08-27
 * TAG         : BSP.TP.Init
 * ---------------- Revision History: --------------------------
 *   <version>    <date>          < author >                            <desc>
 ****************************************************************/

#ifndef __GOODIX_BRL_CORE_H__
#define __GOODIX_BRL_CORE_H__

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/iio/consumer.h>

#include "../goodix_common.h"
#include "../gtx8_tools.h"

#ifdef TPD_DEVICE
#undef TPD_DEVICE
#define TPD_DEVICE "goodix-gt9966"
#else
#define TPD_DEVICE "goodix-gt9966"
#endif

#define GOODIX_READ_VERSION_RETRY   5
#define FW_VERSION_INFO_ADDR        0x10014
#define GOODIX_NORMAL_PID           "9966"


#define GOODIX_DRIVER_VERSION			"v1.0.3"

/****************************Start of define declare****************************/

#define set_reg_bit(reg, pos, val)        ((reg) = ((reg) & (~(1 << (pos)))) | (!!(val) << (pos)))

#define BIT_SET(bit)      (1 << (bit))
#define BIT_CLR(bit)      ~(1 << (bit))
#define BIT_CHK(src, bit) (src & BIT_SET(bit))

#define MAX_POINT_NUM               10       /*max touch point number this ic support*/
#define MAX_GT_IRQ_DATA_LENGTH      90       /*irq data(points,key,checksum) size read from irq*/
#define MAX_GT_EDGE_DATA_LENGTH     50       /*irq edge data read from irq*/

#define MAX_GESTURE_POINT_NUM       128      /*max point number of black gesture*/

/* gesture type */
#define GOODIX_LEFT2RIGHT_SWIP           0xAA
#define GOODIX_RIGHT2LEFT_SWIP           0xBB
#define GOODIX_UP2DOWN_SWIP              0xAB
#define GOODIX_DOWN2UP_SWIP              0xBA
#define GOODIX_DOU_TAP                   0xCC
#define GOODIX_DOU_SWIP                  0x48
#define GOODIX_SINGLE_TAP                0x4C
#define GOODIX_PENDETECT                 0xDD
#define GOODIX_UP_VEE                    0x76
#define GOODIX_DOWN_VEE                  0x5E
#define GOODIX_LEFT_VEE                  0x3E
#define GOODIX_RIGHT_VEE                 0x63
#define GOODIX_CIRCLE_GESTURE            0x6F
#define GOODIX_M_GESTRUE                 0x6D
#define GOODIX_W_GESTURE                 0x77

#define GTP_SENSOR_ID_DEFAULT            255
#define GTP_SENSOR_ID_ERR                0

#define GTP_GESTURE_DOU_TAP              7   /* bit7 */
#define GTP_GESTURE_UP_VEE               16
#define GTP_GESTURE_DOWN_VEE             17
#define GTP_GESTURE_LEFT_VEE             18
#define GTP_GESTURE_RIGHT_VEE            19
#define GTP_GESTURE_CIRCLE               4
#define GTP_GESTURE_DOU_SWIP             20
#define GTP_GESTURE_L2R_SWIP             10
#define GTP_GESTURE_R2L_SWIP             11
#define GTP_GESTURE_U2D_SWIP             24
#define GTP_GESTURE_D2U_SWIP             25
#define GTP_GESTURE_M                    3
#define GTP_GESTURE_W                    5
#define GTP_GESTURE_SIN_TAP              12
#define GTP_NEW_PANEL_NAME               3

#define GTP_DRIVER_SEND_CFG         1        /* send config to TP while initializing (for no config built in TP's flash)*/

/*Request type define*/
#define GTP_RQST_RESPONDED          0x00
#define GTP_RQST_CONFIG             0x01
#define GTP_RQST_FRE                0x02
#define GTP_RQST_RESET              0x03
#define GTP_RQST_CLOCK              0x04
#define GTP_RQST_TIMING             0x05
#define GTP_RQST_HOP_FREQ           0x06
#define GTP_RQST_PEN_SIZE           0x07
#define GTP_RQST_IDLE               0xFF

/*triger event*/
#define GOODIX_TOUCH_EVENT			0x80
#define GOODIX_REQUEST_EVENT		0x40
#define GOODIX_GESTURE_EVENT		0x20
#define GOODIX_FP_EVENT				0x08
#define POINT_TYPE_STYLUS_HOVER		0x01
#define POINT_TYPE_STYLUS			0x03
/* TODO need confirm those event value*/
#define GOODIX_FINGER_PRINT_EVENT   0x08
#define GOODIX_FINGER_STATUS_EVENT  0x02
#define GOODIX_FINGER_IDLE_EVENT    0x04

/* need get grip coor */
#define GRIP_COOR_SUPPORT_FLAG      0x01
#define EDGE_INPUT_COORD            0x102F6
#define EDGE_INPUT_OFFSET           4

#define GTP_TEST_NORMALIZE          0x1A280

/*config define*/
#define FINGER_CHECK                0x0F
#define MAX_FINGER_ID               0xa0
#define IRQ_EVENT_HEAD_LEN          8
#define BYTES_PER_POINT             8
#define COOR_DATA_CHECKSUM_SIZE     2
#define GESTURE_DATA_ADDR_OFFECT    2
#define GESTURE_DATA_ADDR_SIZE      4
#define POINT_NUM_OFFSET            2
#define IRQ_EVENT_TYPE_OFFSET       0
#define REQUEST_EVENT_TYPE_OFFSET   2
#define TOUCH_ALL_OFFECT           10
#define TOUCH_POINT_OFFECT          6
#define TOUCH_CHECKSUM_OFFECT       1

/* pen */
#define GTP_PEN_START_REG_ADDR      0x1027C
#define GTP_PEN_DATA_LEN_OFFSET     5
#define GTP_PEN_SET_CMD_ID          0x6A
#define GTP_PEN_SET_CMD_CON_STA     0xA4
#define GTP_PEN_SET_CMD_SIZE        0x6B
#define GTP_PEN_SET_CMD_REQ_CFG     0x6C
#define GTP_PEN_SET_CMD_CFG_ACK     0x6D
#define GTP_PEN_SET_CMD_PRESS       0xB9
#define GTP_PEN_SET_CMD_SPEED_ON    0x6E
#define GTP_PEN_DOWN_CMD_FRQ        0x6F

#define GTP_SET_CMD_STATUS_OFFSET   0
#define GTP_SET_CMD_ACK_OFFSET      1
#define GTP_SET_CMD_LEN_OFFSET      2
#define GTP_SET_CMD_CMD_OFFSET      3
#define GTP_SET_CMD_DATA0_OFFSET    4

/* test addr */
#define DATA_SYNC_ALGOLIB_ADDR      0x1036c
#define DATA_DEGBU_FW_TX_OFFECT     10
#define BYTES_PER_EDGE                      4
#define TS_MAX_SENSORID                     9
#define TS_CFG_MAX_LEN                      4096
#define TS_CFG_HEAD_LEN                     4
#define TS_CFG_BAG_NUM_INDEX                2
#define TS_CFG_BAG_START_INDEX              4
#define GOODIX_CFG_MAX_SIZE                 4096
#define TOUCH_DATA_OFFECT_2                 2

/*command define*/
#define GTP_CMD_NORMAL                  0x00
#define GTP_CMD_RAWDATA                 0x01
#define GTP_CMD_SLEEP                   0x84
#define GTP_CMD_CHARGER_ON              0x10
#define GTP_CMD_CHARGER_OFF             0x11
#define GTP_CMD_GESTURE_ON              0x12
#define GTP_CMD_GESTURE_OFF             0x13

#define GTP_CMD_DELAY                   10

#define GTP_CMD_FW_STATUS               0x90
#define GTP_CMD_FW_STATUS_ON            0x85
#define GTP_CMD_FW_STATUS_OFF           0x86
#define GTP_CMD_QUIT_IDLE               0x9F

#define GTP_CMD_SPECIAL_LEN             4
#define GTP_CMD_NORMAL__LEN             5

#define GTP_CMD_SHORT_TEST              0
#define GTM_CMD_EDGE_LIMIT_LANDSCAPE    17
#define GTM_CMD_EDGE_LIMIT_VERTICAL     18
#define GTP_CMD_ENTER_DOZE_TIME         0
#define GTP_CMD_DEFULT_DOZE_TIME        0
#define GTP_CMD_FACE_DETECT_ON          0
#define GTP_CMD_FACE_DETECT_OFF         0
#define GTP_CMD_FOD_FINGER_PRINT        0
#define GTP_CMD_GESTURE_ENTER_IDLE      0

#define GTP_CMD_FINGER_PRINT_AREA       0 /*change the finger print detect area*/
#define GTP_CMD_FILTER                  0 /*changer filter*/
#define GTP_CMD_DEBUG                   0x42
#define GTP_CMD_DOWN_DELTA              0x43
#define GTP_CMD_GESTURE_DEBUG           0
#define GTP_CMD_GAME_MODE               0xC2
#define GTP_CMD_GESTURE_MASK            0

#define GTP_CMD_SIXTY_CMD               0xC5

#define GTP_CMD_TEMP_CMD                0x60

#define GTP_MOTOR_POSITON_MASK          0xE0
#define GTP_PEN_ENABLE_MASK             0xA4
#define GTP_GAME_HIGH_FRAME             0xC3
#define GTP_PLAM_MASK                   0xC4
#define GTP_PLAM_ADDR_FLAG              0x10
#define GTP_SMOOTH_CMD                  0x48
#define GTP_SENSITIVE_CMD               0x4A

#define GOODIX_CMD_REG                  0x10174

#define GTP_MASK_ENABLE                 0x01
#define GTP_MASK_DISABLE                0x00
/****************************Start of auto test ********************/
#define GOODIX_RETRY_NUM_3               3
#define GOODIX_CONFIG_REFRESH_DATA       0x01

#define FLOAT_AMPLIFIER                1000
#define MAX_U16_VALUE                  65535
#define RAWDATA_TEST_TIMES             10

#define MAX_TEST_ITEMS                 10

/* MOTOR COORD DEFINE*/
#define MOTOR_MAX_MAX                 370
#define MOTOR_MAX_MIN                 300
#define MOTOR_MIN_MAX                  70
#define MOTOR_MIN_MIN                   0
#define MOTOR_COORD_PARA                3
#define MOTOR_DYNA_LIMIT                3
#define MAX_SIZE                       25

#define RESET_MODE_ENABLE                   2

/* pencil healthinfo */
#define PENCIL_DATA_MATCH 6
#define PENCIL_DATA_ADDR1 "AA3002"
#define PENCIL_DATA_ADDR2 "AA3110"
#define PENCIL_DATA_ADDR3 "AA3210"
#define PENCIL_DATA_PATH  "pencil_healthinfo"
#define PENCIL_DATA_LAST  '\0'
#define PENCIL_DATA_CHMOD 0666

#define PENCIL_RETYR_TIME 50
#define PENCIL_RETYR_CNT  5

enum PENCIL_DATA_ADDR {
	P_ADDR1,
	P_ADDR2,
	P_ADDR3,
};

enum PENCIL_DATA_DEFINE {
	CODING,
	ADC,
	BASE,
	PRESS,
};

#define MAX_PENCIL_DATA       128
#define PENCIL_CHECK_COUNT    40
#define PENCIL_STR_TO_HEX     16
#define PENCIL_DATA_OFFECT    4
#define PENCIL_DATA_GROUP     4
#define PENCIL_DATA_MAX       10
#define PENCIL_LIMIT_MIN      1500
#define PENCIL_LIMIT_MAX      6999
#define EXT_OSC_FREQ          16000000
#define INTER_CLK_FREQ        144000000

#define GET_NEW_TRX           15
#define GET_NEW_CLK           25
#define CLK_NAME              "_CLK"

enum PENCIL_OSC_FREQ {
	DEF_OSC_FREQ_16K = 16,
	PEN_OSC_FREQ_64K = 64,
	EXT_OSC_FREQ_16K = 16000000,
	EXT_OSC_FREQ_64K = 64000000,
};

enum TRX_DIRECTION {
	TX_LEFT_RIGHT = 0,
	TX_DOWN_UP    = 1,
};

/* SOME STATE */

/* setp 1 ap */
#define GAME_MODE_ENABLE                    1
#define SMOOTH_MODE_ENABLE                  2
#define SENSITIVE_MODE_ENABLE               3
#define SELF_TEST_ENABLE                    4
#define RESET_MODE_ENABLE                   2

/*
	TYPE_TEST1: noise_test
	TYPE_TEST2: rawdata_test
	TYPE_TEST3: delta_test
	TYPE_TEST4: self_rawdata_test
	TYPE_TEST5: short_test
*/
static char *test_item_name[MAX_TEST_ITEMS] = {
	"",
	"NOISE_TEST",
	"RAWDATA_TEST",
	"DELTA_TEST",
	"SELF_RAWDATA_TEST",
	"SHORT_TEST",
	"",
	"",
	"",
	""
};

#define GTP_TEST_OK                1
#define GTP_TEST_NG                0

/* error code */
#define NO_DATA                    0

#define CNT_MIN                    0
#define CNT_MAX                    2
#define CNT_CLR                    0

#define NO_ERR                     0
#define RESULT_ERR                -1
#define RAWDATA_SIZE_LIMIT        -2

/****************************Start of Firmware update info********************/

/*mtk max i2c size is 4k*/
#define ISP_MAX_BUFFERSIZE          (1024 * 4)

#define I2C_DATA_MAX_BUFFERSIZE     (1024 * 3)

/* cfg parse from bin */
#define CFG_BIN_SIZE_MIN            279
#define BIN_CFG_START_LOCAL         6
#define MODULE_NUM                  22
#define CFG_NUM                     23
#define CFG_INFO_BLOCK_BYTES        8
#define CFG_HEAD_BYTES              32

#define GOODIX_EXIST                1
#define GOODIX_NOT_EXIST            0

#define GOODIX_MAX_RATE_NUM         2
#define GOODIX_MAX_TEMP             2

enum PANEL_TYPE {
	DEFAULT_PANEL,
	SAMSUNG_PANEL,
};

#define GOODIX_CHECK_HRTIMER_NS        150000000
#define GOODIX_CHECK_HRTIMER_S         0
#define GOODIX_PALM_IN_PEN_HRTIMER_S   5
/* pen control cmd */
#define PEN_CTL_FEEDBACK        0xffff
/* step 1 vibrator*/
#define PEN_CTL_VIBRATOR          1
#define PEN_CTL_VIBRATOR_ENABLE   0
#define PEN_OPP_CMD_VIBRATOR      0x6E
#define PEN_CMD_VIBRATOR          0xF1

/* setp 2 palm in pen mode*/
#define PEN_CTL_SMALL_PALM        3
#define PEN_CTL_SMALL_PALM_ENABLE 2
#define PEN_CTL_SMALL_PALM_CLOSE  8
#define PEN_CMD_SMALL_PALM        0x4B

/* GTX8 cfg name */
#define GOODIX_NORMAL_CONFIG              "normal_config"             /*config_type: 0x01*/
#define GOODIX_TEST_CONFIG                "tptest_config"             /*config_type: 0x00, test config*/
#define GOODIX_NORMAL_NOISE_CONFIG        "normal_noise_config"       /*config_type: 0x02,normal sensitivity, use for charging*/
#define GOODIX_GLOVE_CONFIG               "glove_config"              /*config_type: 0x03,high sensitivity*/
#define GOODIX_GLOVE_NOISE_CONFIG         "glove_noise_config"        /*config_type: 0x04,high sensitivity, use for charging*/
#define GOODIX_HOLSTER_CONFIG             "holster_config"            /*config_type: 0x05,holster*/
#define GOODIX_HOLSTER_NOISE_CONFIG       "holster_noise_config"      /*config_type: 0x06,holster ,use for charging*/
#define GOODIX_NOISE_TEST_CONFIG          "tpnoise_test_config"       /*config_type: 0x07,noise test config*/

#define getU32(a) ((u32)getUint((u8 *)(a), 4))
#define getU16(a) ((u16)getUint((u8 *)(a), 2))


/*#define DATA_CHANGE(grip_data) (grip_data == 255 ? 0 : (grip_data == 0 ? 0 :12))*/
#define DATA_CHANGE(grip_data) (grip_data == 255 ? 0 : grip_data)

/****************************End of Firmware update info********************/

/****************************Start of config data****************************/
#define GTP_CONFIG_MIN_LENGTH    186
#define GTP_CONFIG_MAX_LENGTH    10000

enum goodix_ic_bus_type {
	GOODIX_BUS_TYPE_I2C,
	GOODIX_BUS_TYPE_SPI,
	GOODIX_BUS_TYPE_I3C,
};
/*
struct goodix_bus_interface {
	int bus_type;
	struct device *dev;
	int (*read)(struct device *dev, unsigned int addr,
			 unsigned char *data, unsigned int len);
	int (*write)(struct device *dev, unsigned int addr,
			unsigned char *data, unsigned int len);
};
*/
struct goodix_register {
	uint16_t GTP_REG_FW_CHK_MAINSYS;        /*mainsys reg used to check fw status*/
	uint16_t GTP_REG_FW_CHK_SUBSYS;         /*subsys reg used to check fw status*/
	uint16_t GTP_REG_CONFIG_DATA;           /*configure firmware*/
	uint16_t GTP_REG_READ_COOR;             /*touch state and info*/
	uint16_t GTP_REG_PRODUCT_VER;           /*product id & version*/
	uint16_t GTP_REG_WAKEUP_GESTURE;        /*gesture type*/
	uint16_t GTP_REG_GESTURE_COOR;          /*gesture point data*/
	uint16_t GTP_REG_CMD;                   /*recevice cmd from host*/
	uint16_t GTP_REG_RQST;                  /*request from ic*/
	uint16_t GTP_REG_NOISE_DETECT;          /*noise state*/
	uint16_t GTP_REG_ESD_WRITE;             /*esd state write*/
	uint16_t GTP_REG_ESD_READ;              /*esd state read*/
	uint16_t GTP_REG_DEBUG;                 /*debug log*/
	uint16_t GTP_REG_DOWN_DIFFDATA;         /*down diff data log*/
	uint16_t GTP_REG_EDGE_INFO;             /*edge points' info:ewx/ewy/xer/yer*/

	uint16_t GTP_REG_RAWDATA;
	uint16_t GTP_REG_DIFFDATA;
	uint16_t GTP_REG_BASEDATA;
};

struct goodix_fp_coor {
	u16 fp_x_coor;
	u16 fp_y_coor;
	u16 fp_area;
};

#define MAX_SCAN_FREQ_NUM            5
#define MAX_SCAN_RATE_NUM            5
#define MAX_FREQ_NUM_STYLUS          8
#define MAX_STYLUS_SCAN_FREQ_NUM     6
#pragma pack(1)
struct goodix_fw_version {
	u8 rom_pid[6];               /* rom PID */
	u8 rom_vid[3];               /* Mask VID */
	u8 rom_vid_reserved;
	u8 patch_pid[8];              /* Patch PID */
	u8 patch_vid[4];              /* Patch VID */
	u8 patch_vid_reserved;
	u8 sensor_id;
	u8 reserved[2];
	u16 checksum;
};

struct goodix_ic_info_version {
	u8 info_customer_id;
	u8 info_version_id;
	u8 ic_die_id;
	u8 ic_version_id;
	u32 config_id;
	u8 config_version;
	u8 frame_data_customer_id;
	u8 frame_data_version_id;
	u8 touch_data_customer_id;
	u8 touch_data_version_id;
	u8 reserved[3];
};

struct goodix_ic_info_feature { /* feature info*/
	u16 freqhop_feature;
	u16 calibration_feature;
	u16 gesture_feature;
	u16 side_touch_feature;
	u16 stylus_feature;
};

struct goodix_ic_info_param { /* param */
	u8 drv_num;
	u8 sen_num;
	u8 button_num;
	u8 force_num;
	u8 active_scan_rate_num;
	u16 active_scan_rate[MAX_SCAN_RATE_NUM];
	u8 mutual_freq_num;
	u16 mutual_freq[MAX_SCAN_FREQ_NUM];
	u8 self_tx_freq_num;
	u16 self_tx_freq[MAX_SCAN_FREQ_NUM];
	u8 self_rx_freq_num;
	u16 self_rx_freq[MAX_SCAN_FREQ_NUM];
	u8 stylus_freq_num;
	u16 stylus_freq[MAX_FREQ_NUM_STYLUS];
};

struct goodix_ic_info_misc { /* other data */
	u32 cmd_addr;
	u16 cmd_max_len;
	u32 cmd_reply_addr;
	u16 cmd_reply_len;
	u32 fw_state_addr;
	u16 fw_state_len;
	u32 fw_buffer_addr;
	u16 fw_buffer_max_len;
	u32 frame_data_addr;
	u16 frame_data_head_len;
	u16 fw_attr_len;
	u16 fw_log_len;
	u8 pack_max_num;
	u8 pack_compress_version;
	u16 stylus_struct_len;
	u16 mutual_struct_len;
	u16 self_struct_len;
	u16 noise_struct_len;
	u32 touch_data_addr;
	u16 touch_data_head_len;
	u16 point_struct_len;
	u16 reserved1;
	u16 reserved2;
	u32 mutual_rawdata_addr;
	u32 mutual_diffdata_addr;
	u32 mutual_refdata_addr;
	u32 self_rawdata_addr;
	u32 self_diffdata_addr;
	u32 self_refdata_addr;
	u32 iq_rawdata_addr;
	u32 iq_refdata_addr;
	u32 im_rawdata_addr;
	u16 im_readata_len;
	u32 noise_rawdata_addr;
	u16 noise_rawdata_len;
	u32 stylus_rawdata_addr;
	u16 stylus_rawdata_len;
	u32 noise_data_addr;
	u32 esd_addr;
};

struct goodix_ic_info {
	u16 length;
	struct goodix_ic_info_version version;
	struct goodix_ic_info_feature feature;
	struct goodix_ic_info_param parm;
	struct goodix_ic_info_misc misc;
};
#pragma pack()

#define MAX_CMD_DATA_LEN 10
#define MAX_CMD_BUF_LEN  16
#pragma pack(1)
struct goodix_ts_cmd {
	union {
		struct {
			u8 state;
			u8 ack;
			u8 len;
			u8 cmd;
			u8 data[MAX_CMD_DATA_LEN];
		};
		u8 buf[MAX_CMD_BUF_LEN];
	};
};

struct clk_test_parm {
	union {
		struct {
			u8 gio;
			u8 div:2;           /* 0:no div 1:8 div */
			u8 gio_set:1;       /* 0:GIO10 1:EXT_CLK */
			u8 en:1;
			u8 osc_en_io:4;
			u8 trigger_mode;    /* 0:rising 1:high 2:falling 3:low */
			u16 clk_in_num;     /* collect clk num (1-1022) */
			u16 checksum;
		};
		u8 buf[7];
	};
};
#pragma pack()

struct config_info {
	u8      goodix_int_type;
	u32     goodix_abs_x_max;
	u32     goodix_abs_y_max;
};
#define MAX_STR_LEN                 32

enum CHECKSUM_MODE {
	CHECKSUM_MODE_U8_LE,
	CHECKSUM_MODE_U16_LE,
};

/*
 * struct goodix_ts_config - chip config data
 * @initialized: whether intialized
 * @name: name of this config
 * @lock: mutex
 * @reg_base: register base of config data
 * @length: bytes of the config
 * @delay: delay time after sending config
 * @data: config data buffer
 */
struct goodix_ts_config {
	unsigned int length;
	char name[MAX_STR_LEN + 1];
	unsigned char data[GOODIX_CFG_MAX_SIZE];
};

/**
 * struct ts_test_params - test parameters
 * drv_num: touch panel tx(driver) number
 * sen_num: touch panel tx(sensor) number
 * max_limits: max limits of rawdata
 * min_limits: min limits of rawdata
 * deviation_limits: channel deviation limits
 * short_threshold: short resistance threshold
 * r_drv_drv_threshold: resistance threshold between drv and drv
 * r_drv_sen_threshold: resistance threshold between drv and sen
 * r_sen_sen_threshold: resistance threshold between sen and sen
 * r_drv_gnd_threshold: resistance threshold between drv and gnd
 * r_sen_gnd_threshold: resistance threshold between sen and gnd
 * avdd_value: avdd voltage value
 */
struct ts_test_params {
	u16 rawdata_addr;
	u16 noisedata_addr;
	u16 self_rawdata_addr;
	u16 self_noisedata_addr;

	u16 basedata_addr;
	u32 max_drv_num;
	u32 max_sen_num;
	u32 drv_num;
	u32 sen_num;
	u8 *drv_map;
	u8 *sen_map;

	u32 *max_limits;
	u32 *min_limits;

	u32 *deviation_limits;
	u32 *self_max_limits;
	u32 *self_min_limits;

	u32 noise_threshold;
	u32 self_noise_threshold;
	u32 short_threshold;
	u32 r_drv_drv_threshold;
	u32 r_drv_sen_threshold;
	u32 r_sen_sen_threshold;
	u32 r_drv_gnd_threshold;
	u32 r_sen_gnd_threshold;
	u32 avdd_value;
};

/**
 * struct ts_test_rawdata - rawdata structure
 * data: rawdata buffer
 * size: rawdata size
 */
struct ts_test_rawdata {
	s16 *data;
	u32 size;
};

struct ts_test_self_rawdata {
	s16 *data;
	u32 size;
};

/**
 * struct goodix_ts_test - main data structrue
 * ts: goodix touch screen data
 * test_config: test mode config data
 * orig_config: original config data
 * noise_config: noise config data
 * test_param: test parameters from limit img
 * rawdata: raw data structure from ic data
 * noisedata: noise data structure from ic data
 * self_rawdata: self raw data structure from ic data
 * self_noisedata: self noise data structure from ic data
 * test_result: test result string
 */
struct goodix_ts_test {
	void *ts;
	struct goodix_ts_config test_config;
	struct goodix_ts_config orig_config;
	struct goodix_ts_config noise_config;
	struct ts_test_params test_params;
	bool   is_item_support[MAX_TEST_ITEMS];
	struct ts_test_rawdata rawdata;
	struct ts_test_rawdata noisedata;
	struct ts_test_rawdata deltadata;
	struct ts_test_self_rawdata self_rawdata;
	struct ts_test_self_rawdata self_noisedata;

	struct goodix_testdata *p_testdata;
	struct seq_file *p_seq_file;
	/*[0][0][0][0][0]..  0 without test; 1 pass, 2 panel failed; 3 software failed */
	char test_result[MAX_TEST_ITEMS];
	int error_count;
	uint64_t      device_tp_fw;
};

struct short_record {
	u32 master;
	u32 slave;
	u16 short_code;
	u8 group1;
	u8 group2;
};

typedef enum panel_fps {
	GOODIX_60_FPS      = 60,
	GOODIX_DEFAULT_FPS = 120,
} p_fps;

enum pen_and_touch_state {
	ALL_TOUCH_UP = 0,
	ALL_PEN_UP   = 0,
	TOUCH_UP     = 0,
	NO_TOUCH     = 0,
	TOUCH_DOWN   = 1,
	PEN_UP       = 2,
	PEN_DOWN     = 3,
};

enum check_hrtimer_state {
	NOT_CHECK        = 0,
	NO_CHECK_TIME    = 0,
	TIME_CHECK_OVER  = 0,
	OFF              = 0,
	ON               = 1,
	TIME_CHECK_START = 1,
};

typedef enum {
	PEN__CHECK_HRTIMER,
} choice_one_hrtimer;

struct chip_data_brl {
	bool                                halt_status;                    /*1: need ic reset*/
	u8                                  *touch_data;
	u8                                  *edge_data;
	tp_dev                              tp_type;
	u16                                 *spuri_fp_touch_raw_data;
	int                                 irq;
	struct i2c_client                   *client;
	struct spi_device                   *s_client;
	struct goodix_fw_version            ver_info;
	struct goodix_ic_info               ic_info;
	struct config_info                  config_info;
	struct goodix_register              reg_info;
	struct fw_update_info               update_info;
	struct hw_resource                  *hw_res;
	struct goodix_proc_operations       *goodix_ops;                    /*goodix func provide for debug*/
	struct goodix_fp_coor               fp_coor_report;
	struct goodix_health_info_v2        health_info;
	/*struct goodix_bus_interface         *bus;*/

	struct goodix_ts_config             normal_cfg;
	struct goodix_ts_config             normal_noise_cfg;
	struct goodix_ts_config             test_cfg;
	struct goodix_ts_config             noise_test_cfg;

	struct hrtimer                      check_hrtimer;
	ktime_t                             check_time;
	int                                 check_hrtimer_over;
	int                                 check_start;
	int                                 check_id;
	bool                                check_palm_flag;

	u16                                 checkdata[PENCIL_DATA_GROUP];
	u16                                 pen_err_coding[PENCIL_DATA_MAX];
	int                                 pen_err_coding_cnt;
	u16                                 pen_err_adc[PENCIL_DATA_MAX];
	int                                 pen_err_adc_cnt;
	u16                                 pen_err_base[PENCIL_DATA_MAX];
	int                                 pen_err_base_cnt;
	u16                                 pen_err_press;
	bool                                pen_healthinfo_node;
	bool                                pen_enable;
	bool                                pen_support;
	bool                                pen_support_opp;
	int                                 pen_input_state;
	u8                                  pen_num;
	u8                                  point_type;
	u16                                 pen_x;
	u16                                 pen_y;
	u32                                 pen_min_x;
	u32                                 pen_max_x;
	u32                                 pen_min_y;
	u32                                 pen_max_y;

	int                                 rawdiff_mode;
	bool                                esd_check_enabled;
	bool                                fp_down_flag;
	bool                                single_tap_flag;
	uint8_t                             touch_direction;
	char                                *p_tp_fw;
	bool                                kernel_grip_support;
	bool                                get_grip_coor;
	bool                                abnormal_grip_coor;
	bool                                motor_coord_support;
	bool                                motor_max_limit;
	bool                                snr_read_support;

	unsigned int                        pen_osc_frequency;
	unsigned int                        hardware_trx_direction;
	unsigned int                        support_gesture_type;
	unsigned int                        display_refresh_rate;
	int                                 rate_num;
	unsigned int                        rate_array[GOODIX_MAX_RATE_NUM];
	unsigned int                        rate_num_support;

	unsigned int                        gt_temperature[GOODIX_MAX_TEMP];
	unsigned int                        temp_recorder_cnt;
	bool                                gt_temp_enable;

	int                                 get_new_trx[2];
	char                                get_new_trx_name[GET_NEW_TRX];

	int                                 get_new_clk[2];
	char                                get_new_clk_name[GET_NEW_CLK];

	int                                 tp_index;
	u32                                 gesture_type;

	unsigned int                        touch_state;
	unsigned int                        pen_state;
	unsigned int                        check_now_state;
	unsigned int                        check_old_state;
	unsigned int                        check_chg_state;

	/*add for healthinfo*/
	struct goodix_ts_test               *brl_test;
	unsigned int                        esd_err_count;
	unsigned int                        send_cmd_err_count;

	/* motor Coordinate para */
	unsigned int                       motor_max_max;
	unsigned int                       motor_max_min;
	unsigned int                       motor_min_max;
	unsigned int                       motor_min_min;
	unsigned int                       motor_get_coord;
	unsigned int                       virtual_origin;
	unsigned int                       dynamic_k_value;
	unsigned int                       motor_runing;
	unsigned int                       runing_x_coord;
	unsigned int                       runing_x_offect;
	unsigned int                       motor_prevent;
	unsigned int                       motor_offect;
	u32 motor_coord_para[MOTOR_COORD_PARA];
	u32 motor_dynamic_limit[MOTOR_DYNA_LIMIT];
	unsigned int                       max_x;
	unsigned int                       max_y;
	unsigned int                       resolution_ratio;
	/* pen control data */
	unsigned int                       pen_ctl_para;
	u16                                pen_last_press;
	unsigned int                       pen_press;
	unsigned int                       pen_frq_val;
	/* ic state*/
	bool                               game_enable;
	bool                               gesture_enable;
	bool                               sleep_enable;
	bool                               default_rate_set;
	struct mutex                       debug_lock;
	unsigned int                       panel_type;
	struct work_struct                 check_pendown_work;
	struct touchpanel_data *ts;
	/*diff mode*/
	bool enable_differ_mode;
	u8 *diff_rw_buf;
	s16 *diff_buf;
	u32 diff_size;
};

/****************************End of struct declare***************************/
int goodix_bus_init(void);
void goodix_bus_exit(void);

static inline u8 checksum_u8(u8 *data, u32 size)
{
	u8 checksum = 0;
	u32 i = 0;
	for (i = 0; i < size; i++) {
		checksum += data[i];
	}
	return checksum;
}
int goodix_send_cmd_simple(struct chip_data_brl *chip_info, u8 cmd_type, u8 cmd_data);
void goodix_clear_status(struct chip_data_brl *chip_info);
int brl_send_cmd(struct chip_data_brl *chip_info, struct goodix_ts_cmd *cmd);
int goodix_reg_read(struct chip_data_brl *chip_info, u32 addr, u8 *data, u32 len);
void start_time(struct chip_data_brl *chip_info, choice_one_hrtimer choice_one_hrtimer);
int checksum_cmp(const u8 *data, int size, int mode);

void pen_checkdown_work(struct work_struct *work);
void goodix_get_pen_health_info(struct chip_data_brl *chip_info, struct monitor_data *mon_data);
int goodix_pen_uplink_data(void *chip_data, u32 buf_len, u8 *buf, u32 *out_len);
int goodix_pen_downlink_data(void *chip_data, u32 cmd, u32 buf_len, u8 *buf);
int goodix_enable_pen_mode(struct chip_data_brl *chip_info, bool enable);
int goodix_pen_control(struct chip_data_brl *chip_info, int ctl_cmd);
int goodix_pen_control_palm_inpen(struct chip_data_brl *chip_info, int ctl_cmd);
void goodix_get_pen_points(void *chip_data, struct pen_info *pen_info);
void pen_init_debug_node(struct chip_data_brl *chip_info);

#endif /*__GOODIX_BRL_CORE_H__*/
