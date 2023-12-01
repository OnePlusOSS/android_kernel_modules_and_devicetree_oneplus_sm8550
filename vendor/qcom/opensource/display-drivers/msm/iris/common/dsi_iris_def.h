/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IRIS_DEF_H_
#define _DSI_IRIS_DEF_H_


// Use Iris Analog bypass mode to light up panel
// Note: input timing should be same with output timing
//#define IRIS_ABYP_LIGHTUP
//#define IRIS_MIPI_TEST


#define IRIS_FIRMWARE_NAME_I7P	"iris7p.fw"
#define IRIS_CCF1_FIRMWARE_NAME_I7P "iris7p_ccf1.fw"
#define IRIS_CCF2_FIRMWARE_NAME_I7P "iris7p_ccf2.fw"
#define IRIS_CCF3_FIRMWARE_NAME_I7P "iris7p_ccf3.fw"
#define IRIS_CCF1_CALIBRATED_FIRMWARE_NAME_I7P "iris7p_ccf1b.fw"
#define IRIS_CCF2_CALIBRATED_FIRMWARE_NAME_I7P "iris7p_ccf2b.fw"
#define IRIS_CCF3_CALIBRATED_FIRMWARE_NAME_I7P "iris7p_ccf3b.fw"

#define IRIS_FIRMWARE_NAME_I7	"iris7.fw"
#define IRIS_CCF1_FIRMWARE_NAME_I7 "iris7_ccf1.fw"
#define IRIS_CCF2_FIRMWARE_NAME_I7 "iris7_ccf2.fw"
#define IRIS_CCF3_FIRMWARE_NAME_I7 "iris7_ccf3.fw"
#define IRIS_CCF4_FIRMWARE_NAME_I7 "iris7_ccf4.fw"
#define IRIS_CCF1_CALIBRATED_FIRMWARE_NAME_I7 "iris7_ccf1b.fw"
#define IRIS_CCF2_CALIBRATED_FIRMWARE_NAME_I7 "iris7_ccf2b.fw"
#define IRIS_CCF3_CALIBRATED_FIRMWARE_NAME_I7 "iris7_ccf3b.fw"
#define IRIS_CCF4_CALIBRATED_FIRMWARE_NAME_I7 "iris7_ccf4b.fw"

#define IRIS3_CHIP_VERSION	0x6933
#define IRIS5_CHIP_VERSION	0x6935
#define IRIS7_CHIP_VERSION      0x4777

#define DIRECT_BUS_HEADER_SIZE 8

#define CEILING(x, y) (((x)+((y)-1))/(y))

// bit mask
#define	BIT_MSK(bit)			(1 << (bit))

// set 1 bit to 1
#define BIT_SET(val, offset)	((val) | BIT_MSK(offset))

// set 1 bit to 0
#define BIT_CLR(val, offset)	((val) & (~BIT_MSK(offset)))

// bits mask
#define	BITS_MSK(bits)			(BIT_MSK(bits) - 1)

// left shift bits mask for offset bits
#define	BITS_SHFT(bits, offset)		(BITS_MSK(bits) << (offset))

// clear bits which from offeset for val
#define	BITS_CLR(val, bits, offset)	((val) & ~(BITS_SHFT(bits, offset)))

// get bits value which from offset for val
#define	BITS_GET(val, bits, offset)	\
	(((val) & BITS_SHFT(bits, offset)) >> (offset))

// set bits value which from offset by bitval
#define	BITS_SET(val, bits, offset, bitsval)	\
	(BITS_CLR(val, bits, offset) | (((bitsval) & BITS_MSK(bits)) << (offset)))

#define LUT_LEN 256
#define DPP_3DLUT_GROUP 3 // table 0,3,6 should store at the same address in iris
#define SCALER1D_LUT_NUMBER 64
#define SDR2HDR_LUT_BLOCK_SIZE (128*4)
#define SDR2HDR_LUT2_BLOCK_NUMBER (6)
#define SDR2HDR_LUTUVY_BLOCK_NUMBER (12)
#define SDR2HDR_LUT2_ADDRESS 0x3000
#define SDR2HDR_LUTUVY_ADDRESS 0x6000
#define SDR2HDR_LUT_BLOCK_ADDRESS_INC 0x400
#define SDR2HDR_LUT2_BLOCK_CNT (6)  //for ambient light lut
#define SDR2HDR_LUTUVY_BLOCK_CNT (12)  // for maxcll lut

#define PANEL_BL_MAX_RATIO 10000
#define IRIS_MODE_RFB                   0x0
#define IRIS_MODE_FRC_PREPARE           0x1
#define IRIS_MODE_FRC_PREPARE_DONE      0x2
#define IRIS_MODE_FRC                   0x3
#define IRIS_MODE_FRC_CANCEL            0x4
#define IRIS_MODE_FRC_PREPARE_RFB       0x5
#define IRIS_MODE_FRC_PREPARE_TIMEOUT   0x6
#define IRIS_MODE_RFB2FRC               0x7
#define IRIS_MODE_RFB_PREPARE           0x8
#define IRIS_MODE_RFB_PREPARE_DONE      0x9
#define IRIS_MODE_RFB_PREPARE_TIMEOUT   0xa
#define IRIS_MODE_FRC2RFB               0xb
#define IRIS_MODE_PT_PREPARE            0xc
#define IRIS_MODE_PT_PREPARE_DONE       0xd
#define IRIS_MODE_PT_PREPARE_TIMEOUT    0xe
#define IRIS_MODE_RFB2PT                0xf
#define IRIS_MODE_PT2RFB                0x10
#define IRIS_MODE_PT                    0x11
#define IRIS_MODE_KICKOFF60_ENABLE      0x12
#define IRIS_MODE_KICKOFF60_DISABLE     0x13
#define IRIS_MODE_PT2BYPASS             0x14
#define IRIS_MODE_BYPASS                0x15
#define IRIS_MODE_BYPASS2PT             0x16
#define IRIS_MODE_PTLOW_PREPARE         0x17
#define IRIS_MODE_DSI_SWITCH_2PT        0x18    // dsi mode switch during RFB->PT
#define IRIS_MODE_DSI_SWITCH_2RFB       0x19    // dsi mode switch during PT->RFB
#define IRIS_MODE_FRC_POST              0x1a    // for set parameters after FRC
#define IRIS_MODE_RFB_PREPARE_DELAY     0x1b    // for set parameters before RFB
#define IRIS_MODE_RFB_POST              0x1c    // for set parameters after RFB
#define IRIS_MODE_INITING               0xff
#define IRIS_MODE_OFF                   0xf0
#define IRIS_MODE_HDR_EN                0x20

#define IRIS_EMV_MIN			0x40
#define IRIS_EMV_ON_PREPARE		0x40
#define IRIS_EMV_ON_SWAP		0x41
#define IRIS_EMV_ON_CONFIGURE		0x42
#define IRIS_EMV_ON_FRC			0x43
#define IRIS_EMV_OFF_PREPARE		0x44
#define IRIS_EMV_CLOSE_THE_SET		0x45
#define IRIS_EMV_OFF_CONFIGURE		0x46
#define IRIS_EMV_OFF_PT		0x47
#define IRIS_EMV_OFF_FINAL		0x48
#define IRIS_EMV_CLOSE_PIPE_0		0x49
#define IRIS_EMV_OPEN_PIPE_0		0x4a
#define IRIS_EMV_CLOSE_PIPE_1		0x4b
#define IRIS_EMV_OPEN_PIPE_1		0x4c
#define IRIS_EMV_OFF_PRECONFIG		0x4d
#define IRIS_EMV_DUMP			0x4e
#define IRIS_EMV_ON_FRC_FINAL		0x4f
#define IRIS_EMV_HEALTH_DOWN		0x50
#define IRIS_EMV_HEALTH_UP		0x51
#define IRIS_EMV_ON_PRECONFIG		0x52
#define IRIS_EMV_HELPER_A		0x53
#define IRIS_EMV_ON_FRC_POST		0x54
#define IRIS_EMV_MAX			0x5f


enum SDR2HDR_LEVEL {
	SDR2HDR_LEVEL0 = 0,
	SDR2HDR_LEVEL1,
	SDR2HDR_LEVEL2,
	SDR2HDR_LEVEL3,
	SDR2HDR_LEVEL4,
	SDR2HDR_LEVEL5,
	SDR2HDR_LEVEL_CNT
};

enum SDR2HDR_TABLE_TYPE {
	SDR2HDR_LUT0 = 0,
	SDR2HDR_LUT1,
	SDR2HDR_LUT2,
	SDR2HDR_LUT3,
	SDR2HDR_UVY0,
	SDR2HDR_UVY1,
	SDR2HDR_UVY2,
	SDR2HDR_INV_UV0,
	SDR2HDR_INV_UV1,
};

enum FRC_PHASE_TYPE {
	FRC_PHASE_V1_60TE = 0,
	FRC_PHASE_V1_90TE,
	FRC_PHASE_V1_120TE,
	FRC_PHASE_TYPE_CNT
};

enum {
	IRIS_CONT_SPLASH_LK = 1,
	IRIS_CONT_SPLASH_KERNEL,
	IRIS_CONT_SPLASH_NONE,
	IRIS_CONT_SPLASH_BYPASS_PRELOAD,
};

enum {
	IRIS_DTSI_PIP_IDX_START = 0,
	IRIS_DTSI0_PIP_IDX = IRIS_DTSI_PIP_IDX_START,
	IRIS_DTSI1_PIP_IDX,
	IRIS_DTSI2_PIP_IDX,
	IRIS_DTSI3_PIP_IDX,
	IRIS_DTSI4_PIP_IDX,
	IRIS_DTSI5_PIP_IDX,
	IRIS_DTSI_PIP_IDX_CNT,
	IRIS_LUT_PIP_IDX = IRIS_DTSI_PIP_IDX_CNT,
	IRIS_PIP_IDX_CNT,

	IRIS_DTSI_NONE = 0xFF,
};

enum {
	IRIS_IP_START = 0x00,
	IRIS_IP_SYS = IRIS_IP_START,
	IRIS_IP_RX = 0x01,
	IRIS_IP_TX = 0x02,
	IRIS_IP_PWIL = 0x03,
	IRIS_IP_DPORT = 0x04,
	IRIS_IP_DTG = 0x05,
	IRIS_IP_DSC_DEN = 0x07,
	IRIS_IP_DSC_ENC = 0x08,
	IRIS_IP_SDR2HDR = 0x09,
	IRIS_IP_SDR2HDR_2 = 0x0a,
	IRIS_IP_IOINC1D = 0x0b,
	IRIS_IP_DPP = 0x0e,
	IRIS_IP_EXT = 0x10,
	IRIS_IP_DMA = 0x11,
	IRIS_IP_AI = 0x12,

	IRIS_IP_RX_2 = 0x021,
	IRIS_IP_SRAM = 0x022,
	IRIS_IP_PWIL_2 = 0x023,
	IRIS_IP_DSC_ENC_2 = 0x24,
	IRIS_IP_DSC_DEN_2 = 0x25,
	IRIS_IP_DSC_DEN_3 = 0x26,
	IRIS_IP_PBSEL_2 = 0x27,
	IRIS_IP_PBSEL_3 = 0x28,
	IRIS_IP_PBSEL_4 = 0x29,
	IRIS_IP_OSD_COMP = 0x2a,
	IRIS_IP_OSD_DECOMP = 0x2b,
	IRIS_IP_OSD_BW = 0x2c,
	IRIS_IP_PSR_MIF = 0x2d,
	IRIS_IP_BLEND = 0x2e,
	IRIS_IP_IOINC1D_2 = 0x2f,
	IRIS_IP_FRC_MIF = 0x30,
	IRIS_IP_FRC_DS = 0x31,
	IRIS_IP_GMD = 0x32,
	IRIS_IP_FBD = 0x33,
	IRIS_IP_CADDET = 0x34,
	IRIS_IP_MVC = 0x35,
	IRIS_IP_FI = 0x36,
	IRIS_IP_DSC_DEC_AUX = 0x37,
	IRIS_IP_DSC_ENC_TNR = 0x38,
	IRIS_IP_SR = 0x39,
	IRIS_IP_END,
	IRIS_IP_CNT = IRIS_IP_END
};

enum LUT_TYPE {
	LUT_IP_START = 128, /*0x80*/
	DBC_LUT = LUT_IP_START,
	DPP_3DLUT,
	SDR2HDR_LUT,
	IOINC1D_LUT,
	AMBINET_HDR_GAIN, /*HDR case*/
	AMBINET_SDR2HDR_LUT, /*SDR2HDR case;*/  //0x85
	GAMMA_LUT,
	FRC_PHASE_LUT,
	APP_CODE_LUT,
	DPP_DITHER_LUT,
	IOINC1D_PP_LUT,  //0x8a
	DTG_PHASE_LUT,
	APP_VERSION_LUT,
	DPP_DEMURA_LUT,
	IOINC1D_LUT_SHARP,  //0x8e
	IOINC1D_PP_LUT_SHARP,
	IOINC1D_LUT_9TAP,  //0x90
	IOINC1D_LUT_SHARP_9TAP,
	DPP_PRE_LUT, //0x92
	BLENDING_LUT,
	SR_LUT,       //0x94
	MISC_INFO_LUT, //0x95
	DPP_DLV_LUT,
	LUT_IP_END
};

enum FIRMWARE_STATUS {
	FIRMWARE_LOAD_FAIL,
	FIRMWARE_LOAD_SUCCESS,
	FIRMWARE_IN_USING,
};

enum result {
	IRIS_FAILED = -1,
	IRIS_SUCCESS = 0,
};

enum PANEL_TYPE {
	PANEL_LCD_SRGB = 0,
	PANEL_LCD_P3,
	PANEL_OLED,
};

enum LUT_MODE {
	INTERPOLATION_MODE = 0,
	SINGLE_MODE,
};

enum SCALER_IP_TYPE {
	SCALER_INPUT = 0,
	SCALER_PP,
	SCALER_INPUT_SHARP,
	SCALER_PP_SHARP,
	SCALER_INPUT_9TAP,
	SCALER_INPUT_SHARP_9TAP,
};

enum IRIS_MEMC_MODE {
	MEMC_DISABLE = 0,
	MEMC_SINGLE_VIDEO_ENABLE,
	MEMC_DUAL_VIDEO_ENABLE,
	MEMC_SINGLE_GAME_ENABLE,
	MEMC_DUAL_EXTMV_ENABLE,
	MEMC_DUAL_GAME_ENABLE,
	MEMC_SINGLE_EXTMV_ENABLE,
};

struct iris_pq_setting {
	u32 cmcolortempmode:2;
	u32 cmcolorgamut:8;
	u32 alenable:1;
	u32 demomode:3;
	u32 sdr2hdr:4;
	u32 readingmode:4;
	u32 edr_ratio:2;   // 0x1 2times, 0x2 2.5times 0x3 3 times
	u32 reserved:8;
};

enum IRIS_PERF_KT {
	kt_perf_start = 0,
	kt_frcen_set,
	kt_metaset_init,
	kt_iris2nd_up_start,
	kt_iris2nd_holdon,
	kt_iris2nd_open,
	kt_iris2nd_close,
	kt_iris2nd_up_ready,
	kt_iris2nd_down,
	kt_ap2nd_down,
	kt_ap2nd_up,
	kt_dual_open,
	kt_flush_osd,
	kt_flush_video,
	kt_flush_none,
	kt_dualon_start,
	kt_dualon_swap,
	kt_dualon_ready,
	kt_autorefresh_on,
	kt_autorefresh_off,
	kt_memcinfo_set,
	kt_frcdsc_start,
	kt_frcdsc_changed,
	kt_frc_prep_start,
	kt_frc_prep_end,
	kt_pt_frcon_start,
	kt_pt_frcon_end,
	kt_pt_frc_ready,
	kt_pt_prep_start,
	kt_pt_prep_end,
	kt_frc_pt_start,
	kt_frc_pt_end,
	kt_pt_ready,
	kt_frcsw_timeout,
	kt_off_prep,
	kt_close_set,
	kt_dual_close,
	kt_metaset_final,
	KT_PERF_MAX,
};

struct extmv_clockinout {
	ktime_t kt[KT_PERF_MAX];
	bool valid;
};

struct extmv_frc_meta {
	u32 mode;
	u32 gamePixelFormat;
	u32 gameWidthSrc;  /*H-resolution*/
	u32 gameHeightSrc; /*V-resolution*/
	u32 gameLeftSrc;   /*start position X*/
	u32 gameTopSrc;    /*start position Y*/
	u32 mvd0PixelFormat;
	u32 mvd0Width;  /*H-resolution*/
	u32 mvd0Height; /*V-resolution*/
	u32 mvd0Left;   /*start position X*/
	u32 mvd0Top;    /*start position Y*/
	u32 mvd1PixelFormat;
	u32 mvd1Width;  /*H-resolution*/
	u32 mvd1Height; /*V-resolution*/
	u32 mvd1Left;   /*start position X*/
	u32 mvd1Top;    /*start position Y*/
	u32 valid;
	u32 bmvSize;
	u32 orientation;
	u32 gmvdPixelFormat;
	u32 gmvdWidthSrc;  /*H-resolution*/
	u32 gmvdHeightSrc; /*V-resolution*/
	u32 gmvdLeft;   /*start position X*/
	u32 gmvdTop;    /*start position Y*/
	u32 containerWidth;
	u32 containerHeight;
	u32 gameWidth;
	u32 gameHeight;
	u32 gameLeft;
	u32 gameTop;
	u32 overflow;
	u32 gmvdWidthSrcLast;
	u32 gmvdHeightSrcLast;
	u32 gameWidthSrcLast;
	u32 gameHeightSrcLast;
};

struct iris_panel_timing_info {
	u32 flag;  //1--power on, 0---off
	u32 width;
	u32 height;
	u32 fps;
	u32 dsc;
};

struct iris_memc_info {
	u8 bit_mask;
	u8 memc_mode;
	u8 memc_level;
	u32 memc_app;
	u8 video_fps;
	u8 panel_fps;
	u8 vfr_en;
	u8 tnr_en;
	u8 low_latency_mode;
	u8 n2m_mode;
	u8 native_frc_en;
	u8 osd_window_en;
	u8 emv_game_mode;
	u16 capt_left;
	u16 capt_top;
	u16 capt_hres;
	u16 capt_vres;
	u32 osd_window[4];
	u32 mv_hres;
	u32 mv_vres;
	u32 latencyValue[64];
	u32 OSDProtection[64];
};

enum low_latency_mode {
	NO_LT_MODE,
	NORMAL_LT,
	LT_MODE,
	ULTRA_LT_MODE,
	LT_INVALID,
};

struct iris_switch_dump {
	bool trigger;
	u32 sw_pwr_ctrl;
	u32 rx_frame_cnt0;
	u32 rx_frame_cnt1;
	u32 rx_video_meta;
	u32 pwil_cur_meta0;
	u32 pwil_status;
	u32 pwil_disp_ctrl0;
	u32 pwil_int;
	u32 dport_int;
	u32 fi_debugbus[3];
};

struct quality_setting {
	struct iris_pq_setting pq_setting;
	u32 cctvalue;
	u32 colortempvalue;
	u32 luxvalue;
	u32 maxcll;
	u32 source_switch;
	u32 al_bl_ratio;
	u32 system_brightness;
	u32 min_colortempvalue;
	u32 max_colortempvalue;
	u32 dspp_dirty;
	u32 sdr2hdr_lce;
	u32 sdr2hdr_de;
	u32 sdr2hdr_tf_coef;
	u32 sdr2hdr_ftc;
	u32 sdr2hdr_csc;
	u32 sdr2hdr_de_ftc;
	u32 sdr2hdr_scurve;
	u32 sdr2hdr_graphic_det;
	u32 sdr2hdr_ai_tm;
	u32 sdr2hdr_ai_lce;
	u32 sdr2hdr_ai_de;
	u32 sdr2hdr_ai_graphic;
	u32 ai_ambient;
	u32 ai_backlight;
	u32 ai_auto_en;
	u32 pwil_dport_disable;
	u32 dpp_demura_en;
	u32 scurvelevel;
};

struct iris_setting_info {
	struct quality_setting quality_cur;
	struct quality_setting quality_def;
};
struct ocp_header {
	u32 header;
	u32 address;
};

struct iris_update_ipopt {
	uint8_t ip;
	uint8_t opt_old;
	uint8_t opt_new;
	uint8_t chain;
};

struct iris_update_regval {
	uint8_t ip;
	uint8_t opt_id;
	uint16_t reserved;
	uint32_t mask;
	//uint32_t addr;
	uint32_t value;
};

struct iris_lp_ctrl {
	bool dynamic_power;
	bool ulps_lp;
	bool dbp_mode;	  // 0: pt mode; 1: dbp mode
	bool flfp_enable;	// first line first pixel enable or not
	bool te_swap; 	// indicated using te_swap and mask 3 TE for every 4 TEs from panel.
	u8 abyp_lp;
	// bit [0]: iris esd check [1]: panel esd check    [2]: recovery enable
	//     [3]: print more     [4]: force trigger once [5]: regdump disable
	int esd_ctrl;
	uint32_t esd_cnt_iris;
	uint32_t esd_cnt_panel;
};

struct iris_abyp_ctrl {
	bool abyp_disable;
	bool abyp_failed;
	bool lightup_sys_powerdown; // power down sys directly in light up
	bool preloaded;
	uint8_t abypass_mode;
	uint16_t pending_mode;	// pending_mode is accessed by SDEEncoder and HWBinder
	struct mutex abypass_mutex;
};

struct iris_frc_setting {
	u8 mv_buf_num;
	u8 vid_buf_num;
	u8 rgme_buf_num;
	u8 layer_c_en;
	u16 disp_hres;
	u16 disp_vres;
	bool disp_dsc;
	u16 input_vtotal;
	u16 disp_htotal;
	u16 disp_vtotal;
	u32 mv_hres;
	u32 mv_vres;
	u16 hres_2nd;
	u16 vres_2nd;
	u16 refresh_rate_2nd;
	bool dsc_2nd;
	u32 emv_hres;
	u32 emv_vres;
	u32 init_video_baseaddr;
	u32 init_dual_video_baseaddr;
	u32 init_single_mv_hres;
	u32 init_single_mv_vres;
	u32 init_dual_mv_hres;
	u32 init_dual_mv_vres;
	u32 mv_coef;
	u8 pps_table_sel;
	u32 dsc_enc_ctrl0;
	u32 dsc_enc_ctrl1;
	u32 video_baseaddr;
	u32 mv_baseaddr;
	u8 sr_sel;
	u8 frc_vfr_disp;
	u8 frc_dynen;
	u8 force_repeat;
	u8 sr_en;
	u32 iris_osd0_tl;
	u32 iris_osd1_tl;
	u32 iris_osd2_tl;
	u32 iris_osd3_tl;
	u32 iris_osd4_tl;
	u32 iris_osd0_br;
	u32 iris_osd1_br;
	u32 iris_osd2_br;
	u32 iris_osd3_br;
	u32 iris_osd4_br;
	u32 iris_osd_window_ctrl;
	u32 iris_osd_win_dynCompensate;
};

struct iris_mspwil_setting {
	u8 memc_mode;
	u8 memc_lt_mode;
	u8 memc_n2m_mode;
	u8 tnr_mode;
	u8 input_scale_level;
	u8 pp_scale_level;
	u8 dsc_para_indx;
	u32 mv_hres;
	u32 mv_vres;
	u32 panel_te;
	u32 disp_hres;
	u32 disp_vres;
};

enum pwil_mode {
	PT_MODE,
	RFB_MODE,
	FRC_MODE,
};

enum iris_config_type {
	IRIS_MEMC_LEVEL = 5,
	USER_DEMO_WND = 17,
	IRIS_CHIP_VERSION = 33,      // 0x0 : IRIS2, 0x1 : IRIS2-plus, 0x2 : IRIS3-lite
	IRIS_LUX_VALUE = 34,
	IRIS_CCT_VALUE = 35,
	IRIS_READING_MODE = 36,

	IRIS_CM_COLOR_TEMP_MODE = 39,
	IRIS_CM_COLOR_GAMUT = 40,
	IRIS_AL_ENABLE = 44,			//AL means ambient light
	IRIS_DEMO_MODE = 46,
	IRIS_SDR2HDR = 47,
	IRIS_COLOR_TEMP_VALUE = 48,
	IRIS_HDR_MAXCLL = 49,
	//IRIS_PP_DATA_PATH = 53,
	IRIS_DYNAMIC_POWER_CTRL = 54,
	IRIS_DBP_MODE = 55,
	IRIS_ANALOG_BYPASS_MODE = 56,
	IRIS_PANEL_TYPE = 57,
	IRIS_DPP_ONLY = 59,
	IRIS_HDR_PANEL_NITES_SET = 60,
	IRIS_SCALER_FILTER_LEVEL = 70,
	IRIS_CCF1_UPDATE = 71,
	IRIS_CCF2_UPDATE = 72,
	IRIS_FW_UPDATE = 73,
	IRIS_HUE_SAT_ADJ = 74,
	IRIS_SCALER_PP_FILTER_LEVEL = 76,
	IRIS_CSC_MATRIX = 75,
	IRIS_LOOP_BACK_MODE = 77,
	IRIS_CONTRAST_DIMMING = 80,
	IRIS_S_CURVE = 81,
	IRIS_DC_DIMMING = 87,
	IRIS_CLEAR_TRIGGER = 88,
	IRIS_BRIGHTNESS_CHIP = 82,
	IRIS_HDR_PREPARE = 90,
	IRIS_HDR_COMPLETE = 91,
	IRIS_MCF_DATA = 92,
	IRIS_BLENDING_CSR_CTRL = 93,
	IRIS_SET_DPP_APL_ABS = 94,
	IRIS_SET_DPP_APL_RES = 95,
	IRIS_GET_DPP_MCU_RES = 96,
	IRIS_ENABLE_DPP_APL = 97,
	IRIS_GET_DPP_APL_RES = 98,
	IRIS_PANEL_NITS = 99,
	IRIS_DUMP_APL_PER_FRAME = 100,

	IRIS_DBG_TARGET_REGADDR_VALUE_GET = 103,
	IRIS_DBG_TARGET_REG_DUMP = 104,
	IRIS_DBG_TARGET_REGADDR_VALUE_SET = 105,
	IRIS_DBG_KERNEL_LOG_LEVEL = 106,
	IRIS_DBG_SEND_PACKAGE = 107,
	IRIS_DBG_TIMING_SWITCH_LEVEL = 110,
	IRIS_DBG_TARGET_REGADDR_VALUE_SET2 = 112,
	IRIS_DEBUG_CAP = 113,
	IRIS_MIPI_RX_VALIDATE = 114,
	IRIS_CLEAR_FRC_MIF_INT = 116,
	IRIS_GET_FRC_MIF_INTRAW = 117,
	IRIS_GET_MEMC_REG_STATUS = 118,
	IRIS_MODE_SET = 120,
	IRIS_VIDEO_FRAME_RATE_SET = 121,
	IRIS_OUT_FRAME_RATE_SET = 122,	// debug only
	IRIS_OSD_ENABLE = 123,
	IRIS_OSD_AUTOREFRESH = 124,
	IRIS_OSD_OVERFLOW_ST = 125,
	// [23-16]: pwil mode, [15-8]: tx mode, [7-0]: rx mode
	IRIS_WORK_MODE = 126,
	IRIS_FRC_LOW_LATENCY = 127,
	IRIS_PANEL_TE = 128,
	IRIS_AP_TE = 129,
	IRIS_N2M_ENABLE = 130,
	IRIS_WAIT_VSYNC = 132,
	IRIS_MIPI2RX_PWRST = 133,
	IRIS_DUAL2SINGLE_ST = 134,
	IRIS_MEMC_OSD = 135,
	IRIS_MEMC_OSD_PROTECT = 136,
	IRIS_CM_PARA_SET = 138,

	IRIS_DPP_DEMO_WINDOW = 139,
	IRIS_FINGER_DISPLAY = 140,
	IRIS_DPP_FADE_INOUT = 141,
	IRIS_DPP_PATH_MUX = 142,
	IRIS_GAMMA_MODE = 143,

	IRIS_PARAM_VALID = 144,
	IRIS_SDR2HDR_AI_ENALE = 145,
	IRIS_SDR2HDR_AI_INPUT_AMBIENTLIGHT = 146,
	IRIS_SDR2HDR_AI_INPUT_BACKLIGHT = 148,
	IRIS_SDR2HDR_LCE = 155,
	IRIS_SDR2HDR_DE = 156,
	IRIS_SDR2HDR_TF_COEF = 157,
	IRIS_SDR2HDR_FTC = 158,
	IRIS_HDR10PLUS = 161,
	IRIS_DPP_3DLUT_GAIN = 163,
	IRIS_SET_DSI_MODE_INFO = 167,
	IRIS_OSD_LAYER_EMPTY = 168,
	IRIS_SET_METADATA = 169,
	IRIS_SET_METADATA_LOCK = 170,
	IRIS_GET_METADATA = 171,
	IRIS_DUAL_CH_CTRL = 172,
	IRIS_MEMC_CTRL = 173,
	IRIS_MEMC_INFO_SET = 174,
	IRIS_DEBUG_SET = 175,
	IRIS_DEBUG_GET = 176,
	IRIS_KERNEL_STATUS_GET = 177,
	IRIS_SET_MVD_META = 178,
	IRIS_SDR2HDR_CSC_SWITCH = 179,
	IRIS_TNR_MODE = 180,
	IRIS_VFR_MODE = 181,
	IRIS_SDR2HDR_DE_FTC = 182,
	IRIS_SDR2HDR_SCURVE = 183,
	IRIS_SDR2HDR_GRAPHIC_DET = 184,
	IRIS_SDR2HDR_AI_TM = 185,
	IRIS_SDR2HDR_AI_LCE = 186,
	IRIS_SDR2HDR_AI_DE = 187,
	IRIS_SDR2HDR_AI_GRAPHIC = 188,
	IRIS_PWIL_DPORT_DISABLE = 189,
	IRIS_HDR_DATA_PATH = 190,
	IRIS_DPP_CSC_SET = 191,
	IRIS_PT_SR_SET = 192,
	IRIS_DEMURA_LUT_SET = 193,
	IRIS_DEMURA_ENABLE = 194,
	IRIS_DEMURA_XY_LUT_SET = 195,
	IRIS_FRC_PQ_LEVEL = 196,
	IRIS_SDR2HDR_UPDATE = 197,
	IRIS_SCL_CONFIG = 198,
	IRIS_SCL_MODEL  = 199,
	IRIS_MEMC_DSC_CONFIG = 200,
	IRIS_CONFIG_TYPE_MAX
};

enum SDR2HDR_CASE {
	SDR2HDR_Bypass = 0,
	HLG_HDR,
	HDR10,
	SDR2HDR_1,
	SDR2HDR_2,
	SDR2HDR_3,
	SDR2HDR_4,
	SDR2HDR_5,
	SDR2HDR_6,
	SDR2HDR_7,
	SDR2HDR_8,
	SDR2HDR_9,
	SDR2HDR_10,
	SDR2HDR_11,
	SDR2HDR_12,
	SDR2HDR_13,
};

enum HDR_POWER {
	HDR_POWER_OFF = 0,
	HDR_POWER_ON,
};

enum VC_ST {
	VC_PT = 0,
	VC_FRC,
	VC_ST_MAX
};

enum SDR2HDR_LUT_GAMMA_INDEX {
	SDR2HDR_LUT_GAMMA_120 = 0,
	SDR2HDR_LUT_GAMMA_106 = 1,
	SDR2HDR_LUT_GAMMA_102 = 2,
	SDR2HDR_LUT_GAMMA_100 = 3,
	SDR2HDR_LUT_GAMMA_MAX
};

enum iris_legacy_enabled {
	IRIS2_VER = 0,
	IRIS2PLUS_VER,
	IRIS3LITE_VER,
	IRIS5_VER,
	IRIS6_VER,
	IRISSOFT_VER,
	IRIS5DUAL_VER,
	UNKNOWN_VER
};

enum iris_feature_enable_bit {
	SUPPORT_HDR10 = 16,
	SUPPORT_SDR2HDR,
	SUPPORT_MEMC,
	SUPPORT_DUAL_MEMC,
	SUPPORT_EMV,
	SUPPORT_SR,
	SUPPORT_SOFT_IRIS,
	UNDEFINED
};

union iris_chip_caps {
	u32 val;
	struct {
		u32 legacy_enabled : 7;
		u32 version_is_new : 1;
		u32 version_number : 8;
		u32 feature_enabled: 12;
		u32 reserved       : 2;
		u32 asic_type      : 2;
	};
};

enum iris_abypass_status {
	PASS_THROUGH_MODE = 0,
	ANALOG_BYPASS_MODE,
	ABP2PT_SWITCHING,
};

enum iris_abyp_status {
	IRIS_PT_MODE = 0,
	IRIS_ABYP_MODE,
	IRIS_PT_TO_ABYP_MODE,
	IRIS_ABYP_TO_PT_MODE,
	MAX_MODE = 255,
};

struct msmfb_iris_tm_points_info {
	void *lut_lutx_payload;
	void *lut_luty_payload;
	void *lut_luttm_payload;
	void *lut_lutcsc_payload;
	void *lut_lutratio_payload;
};

struct msmfb_iris_demura_info {
	void *lut_swpayload;
};

struct msmfb_iris_demura_xy {
	void *lut_xypayload;
};

struct iris_vc_ctrl {
	bool vc_enable;
	uint8_t vc_arr[VC_ST_MAX];
	uint8_t to_iris_vc_id;
	uint8_t to_panel_hs_vc_id;
	uint8_t to_panel_lp_vc_id;
};

enum AI_CASE {
	AI_AMBIENT_BACKLIGHT_DISABLE = 0,
	AI_AMBIENT_ENABLE,
	AI_BACKLIGHT_ENABLE,
	AI_AMBIENT_BACKLIGHT_ENABLE,
};

enum MSG_FLAG {
	READ_FLAG,
	LAST_FLAG,
	BATCH_FLAG,
};

enum dts_ctx_id {
	DTS_CTX_FROM_IMG = 1,
	DTS_CTX_FROM_FW,
};

struct iris_dts_ops {
	enum dts_ctx_id id;

	const void *(*get_property)(const struct device_node *np, const char *name,
		int *lenp);
	bool (*read_bool)(const struct device_node *np,
		const char *propname);
	int (*read_u8)(const struct device_node *np,
		const char *propname, u8 *out_value);
	int (*count_u8_elems)(const struct device_node *np,
		const char *propname);
	int (*read_u8_array)(const struct device_node *np,
		const char *propname, u8 *out_values, size_t sz);
	int (*read_u32)(const struct device_node *np,
		const char *propname, u32 *out_value);
	int (*read_u32_array)(const struct device_node *np,
		const char *propname, u32 *out_values, size_t sz);
};

enum iris_chip_type {
	CHIP_UNKNOWN = 0,
	CHIP_IRIS5,
	CHIP_IRIS6,
	CHIP_IRIS7,
	CHIP_IRIS7P,
};

#endif // _DSI_IRIS_DEF_H_
