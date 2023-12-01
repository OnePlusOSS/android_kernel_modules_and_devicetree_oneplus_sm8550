/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef HX83102J_H
#define HX83102J_H

/*********PART1:Head files**********************/
#include <linux/i2c.h>


#include <linux/platform_data/spi-mt65xx.h>

#include "../himax_common.h"
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#include "oplus_spi.h"
#else
#include "oplus_spi.h"
#endif

#ifndef REMOVE_OPLUS_FUNCTION
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM

#else
#include <soc/oplus/system/boot_mode.h>
#endif
#endif

#define HIMAX_DRIVER_VER "OPLUS_V2_#5213_SONIC_10"

#define HX_RST_PIN_FUNC
#define HX_ZERO_FLASH
#define HX_0F_DEBUG
#define HX_OPV2_CRITERIA
/*#define HX_CODE_OVERLAY*/
/*#define HX_CSV_CRITERIA*/
#define HX_ENTER_ALGORITHM_NUMBER		/*Support calculation enter algorithm number*/
#define HX_ALG_OVERLAY
#define HX_OPT_HW_CRC
/*handwrite pen battery low*/
/*#define PEN_BATTERY_LOW_NOTIFIER*/

/*#define HX_BOOT_UPGRADE*/
/*********PART2:Define Area**********************/
#define STYLUS_INFO_SZ 12
#define DATA_LEN_4				4
#define FW_PAGE_SZ               (128)
#define HARDWARE_MAX_ITEM_LONGTH (64)
#define ENABLE_UNICODE    0x40
#define ENABLE_VEE        0x20
#define ENABLE_CIRCLE     0x08
#define ENABLE_SWIPE      0x02
#define ENABLE_DTAP       0x01

#define UNICODE_DETECT    0x0b
#define VEE_DETECT        0x0a
#define CIRCLE_DETECT     0x08
#define SWIPE_DETECT      0x07
#define DTAP_DETECT       0x03

#define RESET_TO_NORMAL_TIME         5			 /*Sleep time after reset*/

#define SPURIOUS_FP_LIMIT            100
#define SPURIOUS_FP_RX_NUM           8
#define SPURIOUS_FP_TX_NUM           9
#define SPURIOUS_FP_BASE_DATA_RETRY  10

#define I2C_ERROR_MAX_TIME           5

#define HX_32K_SZ 0x8000
#define FOUR_BYTE_ADDR_SZ     4

#define EXTEND_EE_SHORT_RESET_DUR    60
#define EXTEND_EE_SHORT_INT_DUR      150
#define EXTEND_EE_SHORT_TX_ON_COUNT  146
#define EXTEND_EE_SHORT_RX_ON_COUNT  146
#define EXTEND_EE_SHORT_TEST_LIMIT_PART1    160
#define EXTEND_EE_SHORT_TEST_LIMIT_PART2    80         /* ( unit = ratio )*/

/* tddi f54 test reporting - */
#define ELEC_OPEN_TEST_TX_ON_COUNT      2
#define ELEC_OPEN_TEST_RX_ON_COUNT      2
#define ELEC_OPEN_INT_DUR_ONE           15
#define ELEC_OPEN_INT_DUR_TWO           25
#define ELEC_OPEN_TEST_LIMIT_ONE        500
#define ELEC_OPEN_TEST_LIMIT_TWO        50

#define COMMAND_FORCE_UPDATE            4

/*#define UPDATE_DISPLAY_CONFIG*/

/*Self Capacitance key test limite*/
#define MENU_LOW_LIMITE        1630
#define MENU_HIGH_LIMITE       3803

#define BACK_LOW_LIMITE        3016
#define BACK_HIGH_LIMITE       7039

#define TEST_FAIL    1
#define TEST_PASS    0

#define LIMIT_DOZE_LOW     50
#define LIMIT_DOZE_HIGH    975
#define firmware_update_space	261120	/*256*1024*/

/*gmq-himax*/

#define HX64K         0x10000
#define HX128K         0x20000
#define HX1K		  0x400
#define HX106K         0x1a800
/* New Version 1K*/
enum bin_desc_map_table {
	TP_CONFIG_TABLE = 0x00000A00,
	FW_CID = 0x10000000,
	FW_VER = 0x10000100,
	CFG_VER = 0x10000600,
};

/*Old Version 1K
 *enum bin_desc_map_table {
 *TP_CONFIG_TABLE = 0x0000000A,
 *FW_CID = 0x10000000,
 *FW_VER = 0x10000001,
 *CFG_VER = 0x10000005,
 *};
 **/


#define FW_BIN_16K_SZ 0x4000
#define FW_BIN_64K_SZ 0x10000

#define HX_KEY_MAX_COUNT             4
#define DEFAULT_RETRY_CNT            3
#define HIMAX_REG_RETRY_TIMES        10

#define HX_83112A_SERIES_PWON        16
#define HX_83112B_SERIES_PWON        17
#define HX_83112F_SERIES_PWON        18
#define HX_83102D_SERIES_PWON        19
#define HX_83102E_SERIES_PWON        20
#define HX_83102J_SERIES_PWON        21

#define HX_TP_BIN_CHECKSUM_SW        1
#define HX_TP_BIN_CHECKSUM_HW        2
#define HX_TP_BIN_CHECKSUM_CRC       3

#define SHIFTBITS 5

#define  FW_SIZE_64K          65536
#define  FW_SIZE_128K         131072

#define NO_ERR                0
#define READY_TO_SERVE        1
#define WORK_OUT              2
#define I2C_FAIL              -1
#define MEM_ALLOC_FAIL        -2
#define CHECKSUM_FAIL         -3
#define GESTURE_DETECT_FAIL   -4
#define INPUT_REGISTER_FAIL   -5
#define FW_NOT_READY          -6
#define LENGTH_FAIL           -7
#define OPEN_FILE_FAIL        -8
#define ERR_WORK_OUT          -10
#define HX_INIT_FAIL          -11
#define HX_FINGER_ON          1
#define HX_FINGER_LEAVE       2

#define HX_REPORT_COORD       1
#define HX_REPORT_SMWP_EVENT  2

/*gesture head information*/
#define GEST_PTLG_ID_LEN     (4)
#define GEST_PTLG_HDR_LEN    (4)
#define GEST_PTLG_HDR_ID1    (0xCC)
#define GEST_PTLG_HDR_ID2    (0x44)
#define GEST_PT_MAX_NUM      (128)
#define HX_GESUTRE_SZ		27
#define HX_IDX_DOUBLE_CLICK 	0
#define HX_IDX_UP           	1
#define HX_IDX_DOWN         	2
#define HX_IDX_LEFT         	3
#define HX_IDX_RIGHT        	4
#define HX_IDX_C            	5
#define HX_IDX_Z            	6
#define HX_IDX_M            	7
#define HX_IDX_O            	8
#define HX_IDX_S            	9
#define HX_IDX_V            	10
#define HX_IDX_W            	11
#define HX_IDX_E            	12
#define HX_IDX_LC_M         	13
#define HX_IDX_AT           	14
#define HX_IDX_RESERVE      	15
#define HX_IDX_FINGER_GEST  	16
#define HX_IDX_V_DOWN       	17
#define HX_IDX_V_LEFT       	18
#define HX_IDX_V_RIGHT      	19
#define HX_IDX_F_RIGHT      	20
#define HX_IDX_F_LEFT       	21
#define HX_IDX_DF_UP        	22
#define HX_IDX_DF_DOWN      	23
#define HX_IDX_DF_LEFT      	24
#define HX_IDX_DF_RIGHT     	25
#define HX_IDX_SINGLE       	26
int hx_common_gesture_id[HX_GESUTRE_SZ] = {
	0x80, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
	0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x81, 0x1D, 0x2D, 0x3D, 0x1F, 0x2F, 0x51, 0x52, 0x53, 0x54, 0xFF};
int hx_oplus_gesture_id[HX_GESUTRE_SZ] = {
	0x01, 0x0B, 0x0A, 0x09, 0x08, 0xFF, 0xFF, 0x0C, 0x06, 0x0E, 0x02,
	0x0D, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x04, 0x05, 0xFF, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x0F};

#define BS_RAWDATANOISE      10
#define    BS_OPENSHORT      0

#define    SKIPRXNUM         31
/**COMMON USE*********END***/

/** FOR DEBUG USE*****START***/
typedef enum {
	DEBUG_DATA_BASELINE = 0x08,
	DEBUG_DATA_DELTA    = 0x09,
	DEBUG_DATA_RAW      = 0x0A,
	DEBUG_DATA_DOWN     = 0x0B,
} DEBUG_DATA_TYPE;

/*self test use*/

/* ASCII format */
#define ASCII_LF    (0x0A)
#define ASCII_CR    (0x0D)
#define ASCII_COMMA (0x2C)
#define ASCII_ZERO  (0x30)
#define CHAR_EL     '\0'
#define CHAR_NL     '\n'
#define ACSII_SPACE (0x20)
typedef enum {
	HX_AUTO_TEST = 1,
	HX_BS_TEST = 2,
} HX_TEST_TYPE;

typedef enum {
	HIMAX_INSPECTION_OPEN,
	HIMAX_INSPECTION_MICRO_OPEN,
	HIMAX_INSPECTION_SHORT,
	HIMAX_INSPECTION_RAWDATA,
	HIMAX_INSPECTION_NOISE,
	HIMAX_INSPECTION_SORTING,
	HIMAX_INSPECTION_BACK_NORMAL,
	HIMAX_INSPECTION_LPWUG_RAWDATA,
	HIMAX_INSPECTION_LPWUG_NOISE,
	HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA,
	HIMAX_INSPECTION_LPWUG_IDLE_NOISE,
} THP_INSPECTION_ENUM;


typedef enum {
	OPLUS_TEST_IDX_OPEN = 0x0001,
	OPLUS_TEST_IDX_MICRO_OPEN = 0x0002,
	OPLUS_TEST_IDX_SHORT = 0x0003,
	OPLUS_TEST_IDX_RAWDATA = 0x0004,
	OPLUS_TEST_IDX_NOISE = 0x0005,
	OPLUS_TEST_IDX_LPWUG_RAWDATA = 0x0006,
	OPLUS_TEST_IDX_LPWUG_NOISE = 0x0007,
	OPLUS_TEST_IDX_LPWUG_IDLE_RAWDATA = 0x0008,
	OPLUS_TEST_IDX_LPWUG_IDLE_NOISE = 0x0009,
} OPLUS_TEST_ENUM;

/* Error code of AFE Inspection */
#define HX_RSLT_OUT_PATH_OK "/sdcard/TpTestReport/screenOn/OK/"
#define HX_RSLT_OUT_PATH_NG "/sdcard/TpTestReport/screenOn/NG/"
#define HX_GES_RSLT_OUT_PATH_OK "/sdcard/TpTestReport/screenOff/OK/"
#define HX_GES_RSLT_OUT_PATH_NG "/sdcard/TpTestReport/screenOff/NG/"

/*#define HX_RSLT_OUT_FILE_OK "tp_testlimit_OK_"*/
/*#define HX_RSLT_OUT_FILE_NG "tp_testlimitst_NG_"*/

typedef enum {
	RESULT_OK = 0,
	RESULT_ERR,
	RESULT_RETRY,
} RETURN_RESLUT;

typedef enum {
	SKIPTXNUM_START = 6,
	SKIPTXNUM_6 = SKIPTXNUM_START,
	SKIPTXNUM_7,
	SKIPTXNUM_8,
	SKIPTXNUM_9,
	SKIPTXNUM_END = SKIPTXNUM_9,
} SKIPTXNUMINDEX;

char *g_himax_inspection_mode[] = {
	"HIMAX_INSPECTION_OPEN",
	"HIMAX_INSPECTION_MICRO_OPEN",
	"HIMAX_INSPECTION_SHORT",
	"HIMAX_INSPECTION_RAWDATA",
	"HIMAX_INSPECTION_NOISE",
	"HIMAX_INSPECTION_SORTING",
	"HIMAX_INSPECTION_BACK_NORMAL",
	"HIMAX_INSPECTION_LPWUG_RAWDATA",
	"HIMAX_INSPECTION_LPWUG_NOISE",
	"HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA",
	"HIMAX_INSPECTION_LPWUG_IDLE_NOISE",
	NULL
};
char *g_op_hx_inspection_mode[] = {
	"HX_OP_INSPECTION_OPEN",
	"HX_OP_INSPECTION_MICRO_OPEN",
	"HX_OP_INSPECTION_SHORT",
	"HX_OP_INSPECTION_RAWDATA",
	"HX_OP_INSPECTION_NOISE",
	"HX_OP_INSPECTION_LPWUG_RAWDATA",
	"HX_OP_INSPECTION_LPWUG_NOISE",
	"HX_OP_INSPECTION_LPWUG_IDLE_RAWDATA",
	"HX_OP_INSPECTION_LPWUG_IDLE_NOISE",
};

/* for criteria */
char *g_hx_inspt_crtra_name[] = {
	"CRITERIA_OPEN_MAX",
	"CRITERIA_OPEN_MIN",
	"CRITERIA_MICRO_OPEN_MAX",
	"CRITERIA_MICRO_OPEN_MIN",
	"CRITERIA_SHORT_MAX",
	"CRITERIA_SHORT_MIN",
	"CRITERIA_RAWDATA_MAX",  /* CRITERIA_RAW_MAX */
	"CRITERIA_RAWDATA_MIN",   /* CRITERIA_RAW_MIN */
	"CRITERIA_NOISE_MAX",
	"CRITERIA_NOISE_MIN",
	"LPWUG_RAWDATA_MAX",
	"LPWUG_RAWDATA_MIN",
	"LPWUG_NOISE_MAX",
	"LPWUG_NOISE_MIN",
	"LPWUG_IDLE_RAWDATA_MAX",
	"LPWUG_IDLE_RAWDATA_MIN",
	"LPWUG_IDLE_NOISE_MAX",
	"LPWUG_IDLE_NOISE_MIN",
	NULL
};

enum HX_CRITERIA_ENUM {
	IDX_OPENMAX = 0,
	IDX_OPENMIN,
	IDX_M_OPENMAX,
	IDX_M_OPENMIN,
	IDX_SHORTMAX,
	IDX_SHORTMIN,
	IDX_RAWDATA_MAX,
	IDX_RAWDATA_MIN,
	IDX_NOISEMAX,
	IDX_NOISEMIN,
	IDX_LPWUG_RAWDATA_MAX,
	IDX_LPWUG_RAWDATA_MIN,
	IDX_LPWUG_NOISE_MAX,
	IDX_LPWUG_NOISE_MIN,
	IDX_LPWUG_IDLE_RAWDATA_MAX,
	IDX_LPWUG_IDLE_RAWDATA_MIN,
	IDX_LPWUG_IDLE_NOISE_MAX,
	IDX_LPWUG_IDLE_NOISE_MIN,
};

/* Error code of AFE Inspection */
typedef enum {
	THP_AFE_INSPECT_OK      = 0,               /* OK */
	THP_AFE_INSPECT_ESPI    = (1 << 0),        /* SPI communication error */
	THP_AFE_INSPECT_ERAW    = (1 << 1),        /* Raw data error */
	THP_AFE_INSPECT_ENOISE  = (1 << 2),        /* Noise error */
	THP_AFE_INSPECT_EOPEN   = (1 << 3),        /* Sensor open error */
	THP_AFE_INSPECT_EMOPEN  = (1 << 4),        /* Sensor micro open error */
	THP_AFE_INSPECT_ESHORT  = (1 << 5),        /* Sensor short error */
	THP_AFE_INSPECT_ERC     = (1 << 6),        /* Sensor RC error */
	THP_AFE_INSPECT_EPIN    = (1 << 7),        /* Errors of TSVD!FTSHD!FTRCST!FTRCRQ and other PINs
												  when Report Rate Switching between 60 Hz and 120 Hz */
	THP_AFE_INSPECT_EOTHER  = (1 << 8),         /* All other errors */
	THP_AFE_INSPECT_EFILE  = (1 << 9)         /* Get Criteria file error */
} THP_AFE_INSPECT_ERR_ENUM;


/*Himax MP Limit*/
/*#define RAWMIN         3547*0.05
#define RAWMAX         5352*0.9*/

/* Define Criteria*/
#define RAWMIN         500
#define RAWMAX         23000
#define SHORTMIN       0
#define SHORTMAX       150
#define OPENMIN        50
#define OPENMAX        500
#define M_OPENMIN      0
#define M_OPENMAX      150

#define RAWMIN_BD12    895;
#define RAWMAX_BD12    16117;
#define LPWUG_RAWDATA_MAX_BD12        20000;
#define LPWUG_IDLE_RAWDATA_MAX_BD12   20000;

#define NOISEFRAME     BS_RAWDATANOISE+40
#define NOISE_P        256 /*gmqtest*/
#define UNIFMAX        500

/*Himax MP Password*/
#define PWD_OPEN_START      0x77
#define PWD_OPEN_END        0x88
#define PWD_SHORT_START     0x11
#define PWD_SHORT_END       0x33
#define PWD_RAWDATA_START   0x00
#define PWD_RAWDATA_END     0x99
#define PWD_NOISE_START     0x00
#define PWD_NOISE_END       0x99
#define PWD_SORTING_START   0xAA
#define PWD_SORTING_END     0xCC

/*Himax DataType*/
#define DATA_OPEN           0x0B
#define DATA_MICRO_OPEN     0x0C
#define DATA_SHORT          0x0A
#define DATA_RAWDATA        0x0A
#define DATA_NOISE          0x0F
#define DATA_BACK_NORMAL    0x00
#define DATA_LPWUG_RAWDATA  0x0C
#define DATA_LPWUG_NOISE    0x0F
/*#define DATA_DOZE_RAWDATA 0x0A
#define DATA_DOZE_NOISE 0x0F */
#define DATA_LPWUG_IDLE_RAWDATA    0x0A
#define DATA_LPWUG_IDLE_NOISE      0x0F

/*Himax Data Ready Password*/
#define Data_PWD0    0xA5
#define Data_PWD1    0x5A

static uint16_t g_noisemax;
static uint16_t g_lpwug_noisemax;

#define BS_LPWUG           1
/*#define BS_DOZE            1*/
#define BS_LPWUG_IDLE      1

/* Define Criteria*/
#define LPWUG_NOISE_MAX          600
#define LPWUG_NOISE_MIN          -300
#define LPWUG_RAWDATA_MAX        20000
#define LPWUG_RAWDATA_MIN        200
#define DOZE_NOISE_MAX           100
#define DOZE_NOISE_MIN           0
#define DOZE_RAWDATA_MAX         9999
#define DOZE_RAWDATA_MIN         0
#define LPWUG_IDLE_NOISE_MAX     600
#define LPWUG_IDLE_NOISE_MIN     -300
#define LPWUG_IDLE_RAWDATA_MAX   20000
#define LPWUG_IDLE_RAWDATA_MIN   200

#define PWD_NONE                 0x00
#define PWD_LPWUG_START          0x55
#define PWD_LPWUG_END            0x66
/*#define PWD_DOZE_START 0x22
#define PWD_DOZE_END 0x44 */
#define PWD_LPWUG_IDLE_START     0x50
#define PWD_LPWUG_IDLE_END       0x60

#define SKIP_NOTCH_START         5
#define SKIP_NOTCH_END           10
#define SKIP_DUMMY_START         23    /*TX+SKIP_NOTCH_START*/
#define SKIP_DUMMY_END           28 	/*TX+SKIP_NOTCH_END*/

#if defined(HX_CODE_OVERLAY)
	#define OVL_SECTION_NUM      3
	#define OVL_GESTURE_REQUEST  0x11
	#define OVL_GESTURE_REPLY    0x22
	#define OVL_BORDER_REQUEST   0x55
	#define OVL_BORDER_REPLY     0x66
	#define OVL_SORTING_REQUEST  0x99
	#define OVL_SORTING_REPLY    0xAA
	#define OVL_FAULT            0xFF

	uint8_t *ovl_idx = NULL;
#endif

/** FOR DEBUG USE ****END****/

struct himax_report_data {
	int touch_all_size;
	int raw_cnt_max;
	int raw_cnt_rmd;
	int touch_info_size;
	uint8_t finger_num;
	uint8_t finger_on;
	uint8_t *hx_coord_buf;
	uint8_t hx_state_info[5];

	int event_size;
	uint8_t *hx_event_buf;
	int hx_event_buf_oplus[128];

	int rawdata_size;
	uint8_t diag_cmd;
	uint8_t *hx_rawdata_buf;
	/*uint32_t *diag_mutual;*/
	int32_t *diag_mutual;
	uint8_t rawdata_frame_size;
};

/*********PART3:Struct Area**********************/
struct himax_fw_debug_info {
	u16 recal0 : 1;
	u16 recal1 : 1;
	u16 paseline : 1;
	u16 palm : 1;
	u16 idle : 1;
	u16 water : 1;
	u16 hopping : 1;
	u16 noise : 1;
	u16 glove : 1;
	u16 border : 1;
	u16 vr : 1;
	u16 big_small : 1;
	u16 one_block : 1;
	u16 blewing : 1;
	u16 thumb_flying : 1;
	u16 border_extend : 1;
};
#ifdef PEN_BATTERY_LOW_NOTIFIER
struct handwrite_pen_battery {
	dev_t dev_num;
	struct device *pen_battery_low_dev;
	struct cdev pen_cdev;
	bool pen_batlow_flag;
	struct class *penbattery_notifier_class;
};
#endif /*end of PEN_BATTERY_LOW_NOTIFIER*/

struct chip_data_hx83102j;

struct himax_core_fp {
	int (*fp_reload_disable)(struct chip_data_hx83102j *chip_info, int disable);
	bool (*fp_sys_reset)(struct chip_data_hx83102j *chip_info);
	void (*fp_clean_sram_0f)(struct chip_data_hx83102j *chip_info, uint8_t *addr, int write_len, int type);
	void (*fp_write_sram_0f)(struct chip_data_hx83102j *chip_info, const struct firmware *fw_entry, uint8_t *addr, int start_index, uint32_t write_len);
	void (*fp_firmware_update_0f)(struct chip_data_hx83102j *chip_info, const struct firmware *fw_entry);
	int (*fp_0f_operation_dirly)(struct chip_data_hx83102j *chip_info);
	int (*fp_0f_op_file_dirly)(struct chip_data_hx83102j *chip_info, char *file_name);
	void (*fp_0f_operation)(struct work_struct *work);
#ifdef HX_0F_DEBUG
	void (*fp_read_sram_0f)(struct chip_data_hx83102j *chip_info, const struct firmware *fw_entry, uint8_t *addr, int start_index, int read_len);
	void (*fp_read_all_sram)(struct chip_data_hx83102j *chip_info, uint8_t *addr, int read_len);
	void (*fp_firmware_read_0f)(struct chip_data_hx83102j *chip_info, const struct firmware *fw_entry, int type);
	void (*fp_0f_operation_check)(struct chip_data_hx83102j *chip_info, int type);
#endif
};

struct chip_data_hx83102j {
	bool is_auto_test;
	struct himax_report_data *hx_touch_data;
	uint32_t *p_tp_fw;
	tp_dev tp_type;
	char   *test_limit_name;
	struct himax_proc_operations *syna_ops; /*hx83102j func provide to hx83102j common driver*/

	struct hw_resource *hw_res;
	int16_t *spuri_fp_data;
	struct spurious_fp_touch *p_spuri_fp_touch;
	/********SPI bus*******************************/
	struct spi_device    *hx_spi;
	int                  hx_irq;

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
	struct mtk_chip_config hx_spi_mcc;
#else
	struct mt_chip_conf    hx_spi_mcc;
#endif
	/********SPI bus*******************************/
#ifdef HX_ZERO_FLASH
	struct mutex             spi_lock;
	struct mutex 			 fw_update_lock;
	struct workqueue_struct  *himax_0f_update_wq;
	struct delayed_work      work_0f_update;
	struct firmware_headfile *p_firmware_headfile;
	uint8_t *tmp_data;
#endif
	uint8_t     touch_direction;    /*show touchpanel current direction*/
	bool        using_headfile;
	bool        first_download_finished;

	uint32_t    vendor_proj_info;
	uint16_t    fw_cid_ver;
	uint32_t	touch_ver;
	uint32_t	vendor_panel_ver;
	uint32_t	vendor_fw_ver;

	u8 *g_fw_buf;
	size_t g_fw_len;
	bool g_fw_sta;

#ifdef CONFIG_OPLUS_TP_APK
	bool lock_point_status;
	bool plug_status;
	bool debug_mode_sta;
	bool debug_gesture_sta;
	bool earphone_sta;
	bool charger_sta;
	bool noise_sta;
#endif /*end of CONFIG_OPLUS_TP_APK*/
#ifdef PEN_BATTERY_LOW_NOTIFIER
	struct handwrite_pen_battery pen_battery;
#endif /*end of PEN_BATTERY_LOW_NOTIFIER*/
	int hx_hw_reset_activate;
	int hx_touch_info_point_cnt;
	int g_lcd_vendor;
	int irq_en_cnt;

	int g_1kind_raw_size;
	uint32_t g_rslt_data_len;
	char *g_rslt_data;
	int **hx83102j_nf_inspection_criteria;
	int *hx83102j_nf_inspt_crtra_flag;
	int hx_criteria_item;
	int hx_criteria_size;
	char *g_file_path_ok;
	char *g_file_path_ng;
	bool isea006proj;
	bool isbd12proj;
	bool isread_csv;
	int hx83102j_nf_fail_write_count;

#ifdef HX_ZERO_FLASH
	int g_f_0f_updat;
	/* 128k+ */
	int hx83102j_nf_cfg_crc;
	int hx83102j_nf_cfg_sz;
	uint8_t hx83102j_nf_sram_min[4];
	unsigned char *hx83102j_nf_fw_buf;
/* 128k- */
	struct zf_operation *pzf_op;
	bool g_auto_update_flag;
	int g_poweronof;
	int hx83102j_freq_point;
/* 128k- */
#endif

#ifdef HX_ALG_OVERLAY
	uint8_t g_alg_idx_t;
	bool g_has_alg_overlay;
	uint8_t hx_pen_num;
	uint8_t g_pen_num;
#endif
	int check_point_format;
	unsigned char switch_algo;
	uint8_t hx_proc_send_flag;
	uint8_t in_self_test;

	int gflagautotest;

	struct himax_core_fp g_core_fp;

/**COMMON USE   ***START***/
	unsigned long fw_ver_maj_flash_addr;
	unsigned long fw_ver_min_flash_addr;
	unsigned long cfg_ver_maj_flash_addr;
	unsigned long cfg_ver_min_flash_addr;
	unsigned long cid_ver_maj_flash_addr;
	unsigned long cid_ver_min_flash_addr;
	unsigned long fw_ver_maj_flash_leng;
	unsigned long fw_ver_min_flash_leng;
	unsigned long cfg_ver_maj_flash_leng;
	unsigned long cfg_ver_min_flash_leng;
	unsigned long cid_ver_maj_flash_leng;
	unsigned long cid_ver_min_flash_leng;
	unsigned long fw_cfg_ver_flash_addr;
	uint32_t cfg_table_flash_addr;

	struct proc_dir_entry *himax_proc_register_file;
	uint8_t byte_length;
	uint8_t register_command[4];
	bool cfg_flag;

	/*#ifdef HX_ESD_RECOVERY*/
	u8 hx_esd_reset_activate;
	int hx_eb_event_flag;
	int hx_ec_event_flag;
	int hx_ed_event_flag;
	int hx_ee_event_flag;
	int g_zero_event_count;
	int wdt_event_count;
/*#endif*/

	bool hx_reset_state;

	unsigned char ic_type;
	unsigned char ic_checksum;
	bool dsram_flag;
	int g_diag_command;
	uint8_t diag_coor[128];
	int32_t diag_self[100];

	int g_max_mutual;
	int g_min_mutual;
	int g_max_self;
	int g_min_self;

	int mutual_set_flag;
	uint8_t cmd_set[8];

	/**GESTURE_TRACK*/
	int gest_pt_cnt;
	int gest_pt_x[10];
	int gest_pt_y[10];

	const struct firmware *g_fw_entry;
	struct zf_info *g_zf_info_arr;
};

/*********PART4:ZERO FLASH**********************/
#if defined(HX_ZERO_FLASH)

#define MAX_TRANS_SZ    4080     /*Some MTK:240_WRITE_limit*/
#define MAX_RECVS_SZ    56      /*MTK:56_READ_limit*/

#define ZF_ADDR_DIS_FLASH_RELOAD              0x10007f00
#define ZF_DATA_DIS_FLASH_RELOAD              0x00009AA9
#define ZF_ADDR_SYSTEM_RESET                 0x90000018
#define ZF_DATA_SYSTEM_RESET                 0x00000055
/*#define zf_data_sram_start_addr              0x08000000*/
#define ZF_DATA_SRAM_START_ADDR              0x20000000

#define ZF_DATA_SRAM_CLEAN             0x10000000
#define ZF_DATA_CFG_INFO             0x10007000
#define ZF_DATA_FW_CFG_P1             0x10007084
#define ZF_DATA_FW_CFG_P2             0x10007264
#define ZF_DATA_FW_CFG_P3             0x10007300
#define ZF_DATA_ADC_CFG_1             0x10006A00
#define ZF_DATA_ADC_CFG_2             0x10007B28
#define ZF_DATA_ADC_CFG_3             0x10007AF0
#define ZF_DATA_MAP_TABLE             0x10007500
#define ZF_DATA_MODE_SWITCH             0x10007294

/* CORE_DRIVER*/
#define DRIVER_ADDR_FW_DEFINE_2ND_FLASH_RELOAD          0x100072c0
/* CORE_FW */
#define FW_ADDR_RAW_OUT_SEL                 0x800204b4
#define FW_DATA_CLEAR                       0x00000000
#define FW_ADDR_SET_FRAME_ADDR              0x10007294
#define FW_ADDR_SORTING_MODE_EN             0x10007f04
/* CORE FLASH*/
#define FLASH_ADDR_CTRL_BASE           0x80000000
#define FLASH_ADDR_SPI200_TRANS_FMT    (FLASH_ADDR_CTRL_BASE + 0x10)
#define FLASH_ADDR_SPI200_TRANS_CTRL   (FLASH_ADDR_CTRL_BASE + 0x20)
#define FLASH_ADDR_SPI200_CMD          (FLASH_ADDR_CTRL_BASE + 0x24)
#define FLASH_ADDR_SPI200_ADDR         (FLASH_ADDR_CTRL_BASE + 0x28)
#define FLASH_ADDR_SPI200_DATA         (FLASH_ADDR_CTRL_BASE + 0x2c)
/* CORE IC*/
#define IC_ADR_AHB_ADDR_BYTE_0           0x00
#define IC_ADR_AHB_RDATA_BYTE_0          0x08
#define IC_ADR_AHB_ACCESS_DIRECTION      0x0c
#define IC_CMD_AHB_ACCESS_DIRECTION_READ     0x00


#define PWD_TURN_ON_MPAP_OVL    0x107380
#define ADDR_CTRL_MPAP_OVL      0x100073EC
#define IC_CMD_RST                       0x00000000

#define OVL_ALG_REQUEST  0x11
#define OVL_ALG_REPLY    0x22
#define OVL_ALG_FAULT    0xFF

struct zf_operation {
	uint8_t addr_dis_flash_reload[4];
	uint8_t data_dis_flash_reload[4];
	uint8_t addr_system_reset[4];
	uint8_t data_system_reset[4];
	uint8_t data_sram_start_addr[4];
	uint8_t data_sram_clean[4];
	uint8_t data_cfg_info[4];
	uint8_t data_fw_cfg[4];
	uint8_t data_fw_cfg_p1[4];
	uint8_t data_fw_cfg_p2[4];
	uint8_t data_fw_cfg_p3[4];
	uint8_t data_adc_cfg_1[4];
	uint8_t data_adc_cfg_2[4];
	uint8_t data_adc_cfg_3[4];
	uint8_t data_map_table[4];
	uint8_t data_mode_switch[4];
#ifdef HX_ALG_OVERLAY
	uint8_t addr_set_frame_addr[4];
	uint8_t addr_raw_out_sel[4];
	uint8_t data_clear[4];
	uint8_t addr_fw_define_2nd_flash_reload[4];
	uint8_t addr_sorting_mode_en[4];
	uint8_t addr_spi200_data[4];
	uint8_t addr_ahb_addr_byte_0[1];
	uint8_t addr_ahb_access_direction[1];
	uint8_t data_ahb_access_direction_read[1];
	uint8_t addr_ahb_rdata_byte_0[1];
#endif
};
struct zf_info {
	uint8_t sram_addr[4];
	int write_size;
	uint32_t fw_addr;
	uint32_t cfg_addr;
};



#endif

#endif  /*HX83112B_H*/
