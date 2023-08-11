// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include <video/mipi_display.h>
#include <sde_encoder_phys.h>
#include "dsi_iris_api.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_lightup_ocp.h"
#include "dsi_iris_lp.h"
#include "dsi_iris_pq.h"
#include "dsi_iris_lut.h"
#include "dsi_iris_ioctl.h"
#include "dsi_iris_timing_switch.h"
#include "dsi_iris_log.h"
#include "dsi_iris_memc.h"
#include "dsi_iris_reg.h"
#include "dsi_iris_dts_fw.h"

extern uint8_t iris_pq_update_path;

static bool iris_HDR10;
static bool iris_HDR10_YCoCg;
static bool shadow_iris_HDR10;
static bool shadow_iris_HDR10_YCoCg;
static bool iris_yuv_datapath;
static bool iris_capture_ctrl_en;
static bool iris_debug_cap;
static u8 iris_sdr2hdr_mode;
static struct iris_setting_info iris_setting;
static bool iris_skip_dma;
static u32 iris_min_color_temp;
static u32 iris_max_color_temp;
static u32 iris_min_x_value;
static u32 iris_max_x_value;
static u8 iris_sdr2hdr_lut_index;
static u32 iris_sdr2hdr_current_level;
static u32 hdr_img_size[4]; // {width, height, width_2, height_2}
// {LCE_TF_COEF, DLV_TF_COEF, TM_TF_COEF, NOISE_TF_COEF, FLESH_TF_COEF}
static u32 hdr_tf_coef[] = {40, 40, 40, 40, 40};
static u32 sdr2hdr_level_orig;
static u32 last_filter_level;
static bool iris_ai_lce_disable;
static uint32_t lce_sel_restore;

static bool dpp_precsc_enable;

#ifndef MIN
#define  MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif

#define IRIS_CCT_MIN_VALUE		2500
#define IRIS_CCT_MAX_VALUE		7500
#define IRIS_CCT_STEP			25
#define IRIS_X_6500K			3128
#define IRIS_X_7500K			2991
#define IRIS_X_7700K			2969
#define IRIS_X_2500K			4637
#define IRIS_LAST_BIT_CTRL	1

static uint32_t lut_y[15] = {};
static uint32_t lut_x[15] = {};
static uint32_t lut_tm[5] = {};
static uint32_t lut_csc[25] = {};
static uint32_t lut_ratio[3] = {};
static uint32_t lce_sel;
static uint32_t lut_demura_sw[1683] = {};
static uint32_t lut_demura_xy[2048] = {};

static struct msmfb_iris_tm_points_info iris_tm_points_lut;

static struct msmfb_iris_demura_info iris_demura_lut;

static struct msmfb_iris_demura_xy iris_demura_xy;

static u32 iris_lce_level[][5] = {
	{40, 80, 120, 160, 200},		//LCE_GAMMAGAIN_DARK
	{512, 512, 512, 512, 512},		//LCE_GAMMAUPPER_DARK
	{30, 60, 90, 120, 150},			//LCE_GAMMAGAIN_BRIGHT
	{512, 512, 512, 512, 512},		//LCE_GAMMAUPPER_BRIGHT

	{20, 20, 20, 20, 20},			//LCE_HC_DARK_EDGE_L_GAIN
	{20, 20, 20, 20, 20},			//LCE_HC_DARK_EDGE_H_GAIN
	{30, 60, 90, 120, 150},			//LCE_HC_DARK_UPPER

	{15, 15, 15, 15, 15},			//LCE_HC_BRIGHT_EDGE_L_GAIN
	{20, 20, 20, 20, 20},			//LCE_HC_BRIGHT_EDGE_H_GAIN
	{30, 50, 80, 100, 120},			//LCE_HC_BRIGHT_UPPER

	{40, 40, 40, 30, 20},			//LCE_HC_MID_EDGE_L_GAIN
	{40, 40, 40, 30, 20},			//LCE_HC_MID_EDGE_H_GAIN
	{40, 80, 100, 180, 240},		//LCE_HC_MID_UPPER
};

static u32 iris_de_level[][4] = {
	{128, 160, 196, 80},				//DETAIL_GAINP0
	{200, 250, 300, 80},				//DETAIL_GAINP1
	{250, 300, 400, 80},				//DETAIL_GAINP2
	{300, 380, 450, 80},				//DETAIL_GAINP3
	{200, 300, 350, 80},				//DETAIL_GAINP4
};

static u32 iris_ftc_level[6] = {
	0, 32, 37, 48, 56, 72		//FLESH_DET_RATIO
};

static u32 iris_de_ftc_level[4] = {
	100, 128, 200, 250		//DETAIL_FLESHSMOOTH_GAIN
};

u32 iris_de_default[30] = {0};

static u32 iris_de_disable[2] = {0x1a201e1, 0x5800};

static u32 iris_sdr2hdr_csc[][3] = {
// sRGB2sRGB, sRGB2P3, P32P3
{0x00004000, 0x00004000, 0x00004000},
{0x00000000, 0x00000000, 0x00000000},
{0x000064ca, 0x000064ca, 0x000062b1},
{0x00004000, 0x00004000, 0x00004000},
{0x0000f404, 0x0000f404, 0x0000f27f},
{0x0000e20b, 0x0000e20b, 0x0000df56},
{0x00004000, 0x00004000, 0x00004000},
{0x000076c2, 0x000076c2, 0x000075da},
{0x00000000, 0x00000000, 0x00000000},
{0x00000000, 0x00000000, 0x00000000},
{0x00000000, 0x00000000, 0x00000000},
{0x00000000, 0x00000000, 0x00000000},
{0x00040000, 0x00034a33, 0x00040000},
{0x00000000, 0x0000b5cd, 0x00000000},
{0x00000000, 0x00000000, 0x00000000},
{0x00000000, 0x000021fe, 0x00000000},
{0x00040000, 0x0003de02, 0x00040000},
{0x00000000, 0x00000000, 0x00000000},
{0x00000000, 0x0000117e, 0x00000000},
{0x00000000, 0x00004a23, 0x00000000},
{0x00040000, 0x0003a45f, 0x00040000},
{0x00000000, 0x00000000, 0x00000000},
{0x00000000, 0x00000000, 0x00000000},
{0x00000000, 0x00000000, 0x00000000},
{0x0001a649, 0x0001a649, 0x0001f240},
{0x00016e2a, 0x00016e2a, 0x0001100b},
{0x0000b8cf, 0x0000b8cf, 0x0000caf9},
{0x0000d9be, 0x0000d9be, 0x0000ea78},
{0x0002dc55, 0x0002dc55, 0x0002c457},
{0x000049ec, 0x000049ec, 0x00005131},
{0x000013cb, 0x000013cb, 0x00000000},
{0x00007a0e, 0x00007a0e, 0x00002e32},
{0x0003cd58, 0x0003cd58, 0x00042d00},
{0x00000000, 0x00000000, 0x00000000},
{0x00000000, 0x00000000, 0x00000000},
{0x00000000, 0x00000000, 0x00000000},
{0x00004000, 0x00004000, 0x00004000},
{0x00000000, 0x00000000, 0x00000000},
{0x000064ca, 0x000062b1, 0x000062b1},
{0x00004000, 0x00004000, 0x00004000},
{0x0000f404, 0x0000f27f, 0x0000f27f},
{0x0000e20b, 0x0000df56, 0x0000df56},
{0x00004000, 0x00004000, 0x00004000},
{0x000076c2, 0x000075da, 0x000075da},
{0x00000000, 0x00000000, 0x00000000},
{0x00000000, 0x00000000, 0x00000000},
{0x00000000, 0x00000000, 0x00000000},
{0x00000000, 0x00000000, 0x00000000},
{0x00001b36, 0x00001d4e, 0x00001d4e},
{0x00005b8c, 0x0000588a, 0x0000588a},
{0x0000093e, 0x00000a26, 0x00000a26},
{0x0000f157, 0x0000f016, 0x0000f016},
{0x0000ceab, 0x0000cfeb, 0x0000cfeb},
{0x00004000, 0x00004000, 0x00004000},
{0x00004000, 0x00004000, 0x00004000},
{0x0000c5df, 0x0000c695, 0x0000c695},
{0x0000fa23, 0x0000f96c, 0x0000f96c},
};

/*range is 2500~11000*/
static u32 iris_color_x_buf[] = {
	4637, 4626, 4615, 4603,
	4591, 4578, 4565, 4552,
	4538, 4524, 4510, 4496,
	4481, 4467, 4452, 4437,
	4422, 4407, 4392, 4377,
	4362, 4347, 4332, 4317,
	4302, 4287, 4272, 4257,
	4243, 4228, 4213, 4199,
	4184, 4170, 4156, 4141,
	4127, 4113, 4099, 4086,
	4072, 4058, 4045, 4032,
	4018, 4005, 3992, 3980,
	3967, 3954, 3942, 3929,
	3917, 3905, 3893, 3881,
	3869, 3858, 3846, 3835,
	3823, 3812, 3801, 3790,
	3779, 3769, 3758, 3748,
	3737, 3727, 3717, 3707,
	3697, 3687, 3677, 3668,
	3658, 3649, 3639, 3630,
	3621, 3612, 3603, 3594,
	3585, 3577, 3568, 3560,
	3551, 3543, 3535, 3527,
	3519, 3511, 3503, 3495,
	3487, 3480, 3472, 3465,
	3457, 3450, 3443, 3436,
	3429, 3422, 3415, 3408,
	3401, 3394, 3388, 3381,
	3375, 3368, 3362, 3356,
	3349, 3343, 3337, 3331,
	3325, 3319, 3313, 3307,
	3302, 3296, 3290, 3285,
	3279, 3274, 3268, 3263,
	3258, 3252, 3247, 3242,
	3237, 3232, 3227, 3222,
	3217, 3212, 3207, 3202,
	3198, 3193, 3188, 3184,
	3179, 3175, 3170, 3166,
	3161, 3157, 3153, 3149,
	3144, 3140, 3136, 3132,
	3128, 3124, 3120, 3116,
	3112, 3108, 3104, 3100,
	3097, 3093, 3089, 3085,
	3082, 3078, 3074, 3071,
	3067, 3064, 3060, 3057,
	3054, 3050, 3047, 3043,
	3040, 3037, 3034, 3030,
	3027, 3024, 3021, 3018,
	3015, 3012, 3009, 3006,
	3003, 3000, 2997, 2994,
	2991, 2988, 2985, 2982,
	2980, 2977, 2974, 2971,
	2969, 2966, 2963, 2961,
	2958, 2955, 2953, 2950,
	2948, 2945, 2943, 2940,
	2938, 2935, 2933, 2930,
	2928, 2926, 2923, 2921,
	2919, 2916, 2914, 2912,
	2910, 2907, 2905, 2903,
	2901, 2899, 2896, 2894,
	2892, 2890, 2888, 2886,
	2884, 2882, 2880, 2878,
	2876, 2874, 2872, 2870,
	2868, 2866, 2864, 2862,
	2860, 2858, 2856, 2854,
	2853, 2851, 2849, 2847,
	2845, 2844, 2842, 2840,
	2838, 2837, 2835, 2833,
	2831, 2830, 2828, 2826,
	2825, 2823, 2821, 2820,
	2818, 2817, 2815, 2813,
	2812, 2810, 2809, 2807,
	2806, 2804, 2803, 2801,
	2800, 2798, 2797, 2795,
	2794, 2792, 2791, 2789,
	2788, 2787, 2785, 2784,
	2782, 2781, 2780, 2778,
	2777, 2776, 2774, 2773,
	2772, 2770, 2769, 2768,
	2766, 2765, 2764, 2763,
	2761, 2760, 2759, 2758,
	2756, 2755, 2754, 2753,
	2751, 2750, 2749, 2748,
	2747, 2745, 2744, 2743,
	2742, 2741, 2740, 2739,
	2737,
};

static long nCSCCoffValue[18] = {
	0x000, 0x800, 0x000,
	0x000, 0x000, 0x800,
	0x800, 0x000, 0x000,
	0x000, 0x800, 0x000,
	0x000, 0x000, 0x800,
	0x800, 0x000, 0x000,
};
static u16 *iris_crstk_coef_buf;
static u32 dwCSCCoffBuffer[8] = {
	0x08000000, 0x00000000, 0x08000000, 0x00000800,
	0x00000000, 0x00000000, 0x00000000, 0x00000000
};
static u32 dwCSC2CoffBuffer[12] = {
	0x0000, 0x0800, 0x0000,
	0x0000, 0x0000, 0x0800,
	0x0800, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000,
};
static u32 dwCSC2CoffValue[12];

typedef enum {
	eScalerDownNom = 0,
	eScalerDown1_6,	//shrink ratio >=1.6;
	eScalerDown1_8,	//shrink ratio >=1.8;
	eScalerDown2_0,	//shrink ratio >=2;
	eScalerDown2_2,
	eScalerDown2_4,
	eScalerDown2_6,
	eScalerDown2_8,
	eScaleDownInvalid,
} eScalerDownSoftRatio;

struct iris_setting_info *iris_get_setting(void)
{
	return &iris_setting;
}

void iris_set_HDR10_YCoCg(bool val)
{
	shadow_iris_HDR10_YCoCg = val;
}

void iris_set_sdr2hdr_mode(u8 val)
{
	iris_sdr2hdr_mode = val;
}

void iris_set_ai_lce_disable(bool val)
{
	iris_ai_lce_disable = val;
}

bool iris_get_ai_lce_disable(void)
{
	return iris_ai_lce_disable;
}

int iris_get_hdr_enable(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->valid < PARAM_PARSED)
		return 0;
	else if (iris_HDR10_YCoCg)
		return 2;
	else if (iris_HDR10)
		return 1;
	else if (iris_setting.quality_cur.pq_setting.sdr2hdr != SDR2HDR_Bypass)
		return 100;
	else
		return 0;
}

bool iris_dspp_dirty(void)
{
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;

	if (pqlt_cur_setting->dspp_dirty > 0) {
		IRIS_LOGI("DSPP is dirty");
		pqlt_cur_setting->dspp_dirty--;
		return true;
	}

	return false;
}

void iris_quality_setting_off(void)
{
	iris_setting.quality_cur.al_bl_ratio = 0;
	if (iris_setting.quality_cur.pq_setting.sdr2hdr != SDR2HDR_Bypass) {
		iris_setting.quality_cur.pq_setting.sdr2hdr = SDR2HDR_Bypass;
		iris_sdr2hdr_level_set(SDR2HDR_Bypass);
		iris_hdr_power_set();
		sdr2hdr_level_orig = SDR2HDR_Bypass;
	} else {
		iris_cm_color_gamut_pre_clear();
	}
	iris_csc_para_reset();
	iris_csc2_para_reset();
	iris_setting.quality_cur.pq_setting.cmcolorgamut = 0;
	iris_cm_color_gamut_set(
			iris_setting.quality_cur.pq_setting.cmcolorgamut, true);

	iris_capture_ctrl_en = false;
	iris_skip_dma = false;
	iris_sdr2hdr_mode = 0;
	iris_ai_lce_disable = false;
	last_filter_level = 0;
	dpp_precsc_enable = false;
}

bool iris_get_debug_cap(void)
{
	return iris_debug_cap;
}

void iris_set_debug_cap(bool val)
{
	iris_debug_cap = val;
}

void iris_set_skip_dma(bool skip)
{
	IRIS_LOGD("skip_dma=%d", skip);
	iris_skip_dma = skip;
}

static void iris_end_dpp(bool bcommit)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!iris_skip_dma) {
		if (!iris_dynamic_power_get()) {
			if (pcfg->frc_enabled || pcfg->pwil_mode == FRC_MODE)
				iris_dma_gen_ctrl(0x1, 7, !bcommit);
			else
				iris_dma_gen_ctrl(0x1, 4, !bcommit);
		} else
			//iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe9, 0xe9, 0);
			iris_dma_trig(DMA_CH10, !bcommit);
		iris_update_pq_opt(iris_pq_update_path, bcommit);
	}

}

static void iris_end_hdr(bool bcommit)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!iris_skip_dma) {
		if (!iris_dynamic_power_get()) {
			if (pcfg->frc_enabled || pcfg->pwil_mode == FRC_MODE)
				iris_dma_gen_ctrl(0x2, 7, 0);
			else
				iris_dma_gen_ctrl(0x2, 4, 0);
		} else
			iris_dma_trig(DMA_CH11, 0);
		iris_update_pq_opt(iris_pq_update_path, bcommit);
	}
}

static void iris_end_pwil(bool bcommit)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!iris_skip_dma) {
		if (!iris_dynamic_power_get()) {
			if (pcfg->frc_enabled || pcfg->pwil_mode == FRC_MODE)
				iris_dma_gen_ctrl(0x4, 7, 0);
			else
				iris_dma_gen_ctrl(0x4, 4, 0);
		} else
			iris_dma_trig(DMA_CH12, 0);
		iris_update_pq_opt(iris_pq_update_path, bcommit);
	}
}

static void iris_end_blending(bool bcommit)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!iris_skip_dma) {
		if (!iris_dynamic_power_get()) {
			if (pcfg->frc_enabled || pcfg->pwil_mode == FRC_MODE)
				iris_dma_gen_ctrl(0x8, 7, 0);
			else
				iris_dma_gen_ctrl(0x8, 4, 0);
		} else
			iris_dma_trig(DMA_CH13, 0);
		iris_update_pq_opt(iris_pq_update_path, bcommit);
	}
}

static void iris_end_hdr_blending(bool bcommit)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!iris_skip_dma) {
		if (!iris_dynamic_power_get()) {
			if (pcfg->frc_enabled || pcfg->pwil_mode == FRC_MODE)
				iris_dma_gen_ctrl(0xa, 7, 0);
			else
				iris_dma_gen_ctrl(0xa, 4, 0);
		} else
			iris_dma_trig(DMA_CH11 | DMA_CH13, 0);
		iris_update_pq_opt(iris_pq_update_path, bcommit);
	}
}

static void iris_end_pq(bool bcommit)
{
	if (!iris_skip_dma) {
		if (!iris_dynamic_power_get())
			iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe0, 0xe0, 0);
		iris_update_pq_opt(iris_pq_update_path, bcommit);
	}
}

void iris_init_ipopt_t(void)
{
	int i = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	for (i = 0; i < pcfg->ip_opt_cnt; i++)
		pcfg->pq_update_cmd.update_ipopt_array[i].ip = 0xff;

	pcfg->pq_update_cmd.array_index = 0;
}

static u32 iris_color_temp_x_get(u32 index)
{
	return iris_color_x_buf[index];
}

void iris_ioinc_filter_ratio_send(void)
{
	//u32 dwRatioDiff = 0;
	u32 i = 0;
	u32 len = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	//eScalerDownSoftRatio dwScaleDownRatio = eScalerDownNom;
	bool video_mode = false;
	u32 level = 0x10;

	struct iris_ctrl_opt arr_input[] = {
		{IOINC1D_LUT, 0x00, 0x01},
		{IOINC1D_LUT, 0x40, 0x01},
		{IOINC1D_LUT, 0x80, 0x01},
		{IOINC1D_LUT, 0xc0, 0x01},
		{IOINC1D_LUT_SHARP, 0x00, 0x01},
		{IOINC1D_LUT_SHARP, 0x40, 0x01},
		{IOINC1D_LUT_SHARP, 0x80, 0x01},
		{IOINC1D_LUT_SHARP, 0xc0, 0x00},
	};

	struct iris_ctrl_opt arr_pp[] = {
		{IOINC1D_PP_LUT, 0x00, 0x01},
		{IOINC1D_PP_LUT, 0x40, 0x01},
		{IOINC1D_PP_LUT, 0x80, 0x01},
		{IOINC1D_PP_LUT, 0xc0, 0x01},
		{IOINC1D_PP_LUT_SHARP, 0x00, 0x01},
		{IOINC1D_PP_LUT_SHARP, 0x40, 0x01},
		{IOINC1D_PP_LUT_SHARP, 0x80, 0x01},
		{IOINC1D_PP_LUT_SHARP, 0xc0, 0x00},
	};


	/*
	dwRatioDiff = pcfg->frc_setting.disp_hres * 10 / pcfg->frc_setting.memc_hres;

	if ((dwRatioDiff < 20) && (dwRatioDiff >= 18))
		dwScaleDownRatio = eScalerDown1_8;
	else if ((dwRatioDiff < 22) && (dwRatioDiff >= 20))
		dwScaleDownRatio = eScalerDown2_0;
	else if ((dwRatioDiff < 24) && (dwRatioDiff >= 22))
		dwScaleDownRatio = eScalerDown2_2;
	else if ((dwRatioDiff < 26) && (dwRatioDiff >= 24))
		dwScaleDownRatio = eScalerDown2_4;
	else if ((dwRatioDiff < 28) && (dwRatioDiff >= 26))
		dwScaleDownRatio = eScalerDown2_6;
	else if (dwRatioDiff >= 28)
		dwScaleDownRatio = eScalerDown2_8;
	*/

	if (pcfg->memc_info.memc_mode == MEMC_SINGLE_VIDEO_ENABLE ||
		pcfg->memc_info.memc_mode == MEMC_DUAL_VIDEO_ENABLE)
		video_mode = true;
	if (video_mode == true)
		level = 0x28;
	if (last_filter_level != 0 && level == last_filter_level) {
		IRIS_LOGI("%s, ioinc ratio=%d no need to update 1d filter.", __func__, level);
		return;
	}
	len = sizeof(arr_input)/sizeof(struct iris_ctrl_opt);
	for (i = 0; i < len; i++) {
		arr_input[i].opt_id += level;
		IRIS_LOGD("%s[%d], %02x %02x %02x", __func__, __LINE__,
			arr_input[i].ip, arr_input[i].opt_id, arr_input[i].chain);
	}

	if (video_mode) {
		iris_send_assembled_pkt(arr_input, len);
	} else {
		iris_send_assembled_pkt(arr_input, len);
	}
	last_filter_level = level;
	IRIS_LOGI("%s, ioinc ratio=%d last_filter_level=%d", __func__, level, last_filter_level);

	level = 0xb;
	len = sizeof(arr_pp)/sizeof(struct iris_ctrl_opt);
	for (i = 0; i < len; i++) {
		arr_pp[i].opt_id += level;
		IRIS_LOGD("%s[%d], %02x %02x %02x", __func__, __LINE__,
			arr_pp[i].ip, arr_pp[i].opt_id, arr_pp[i].chain);
	}
	if (video_mode) {
		iris_send_assembled_pkt(arr_pp, len);
	} else {
		iris_send_assembled_pkt(arr_pp, len);
	}
	IRIS_LOGI("%s, ioinc ratio=%d", __func__, level);

}

void iris_crst_coef_check(const u8 *fw_data, size_t fw_size)
{
	u32 len = 0;

	if (fw_size < ((CRSTK_COEF_SIZE + CCT_VALUE_SIZE) * CRSTK_COEF_GROUP))
		IRIS_LOGE("fw_size should be = %d bytes", (CRSTK_COEF_SIZE + CCT_VALUE_SIZE) * CRSTK_COEF_GROUP);
	else
		len = fw_size;

	if (iris_crstk_coef_buf == NULL) {
		if (len == 0)
			len = (CRSTK_COEF_SIZE + CCT_VALUE_SIZE) * CRSTK_COEF_GROUP;

		iris_crstk_coef_buf = kzalloc(len, GFP_KERNEL);
	}
	if (!iris_crstk_coef_buf) {
		IRIS_LOGE("%s:failed to alloc mem iris_crstk_coef_buf:%p",
			__func__, iris_crstk_coef_buf);
		return;
	}
	IRIS_LOGI("crs_fw size: %d", len);
	//memcpy(&iris_crstk_coef_buf[0], (fw_data), ((CRSTK_COEF_SIZE + CCT_VALUE_SIZE) * CRSTK_COEF_GROUP));
	memcpy(&iris_crstk_coef_buf[0], (fw_data), len);

	IRIS_LOGI("csc2 or precsc: 0x%x,\n", iris_crstk_coef_buf[len/2 - 1]);

}

void iris_pq_parameter_init(void)
{
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 index;
	uint32_t *payload = NULL;
	int i;
	static bool iris_first_boot = true;

	if (pqlt_cur_setting->pq_setting.sdr2hdr
			== SDR2HDR_Bypass)
		iris_yuv_datapath = false;
	else
		iris_yuv_datapath = true;

	/* no pxlw node */
	if (pcfg->valid <= PARAM_EMPTY) {
		IRIS_LOGW("no pxlw node");
		return;
	}

	if (pcfg->panel->panel_mode == DSI_OP_VIDEO_MODE)
		iris_debug_cap = true;

	iris_min_color_temp = pcfg->min_color_temp;
	iris_max_color_temp = pcfg->max_color_temp;

	index = (iris_min_color_temp-IRIS_CCT_MIN_VALUE)/IRIS_CCT_STEP;
	iris_min_x_value = iris_color_temp_x_get(index);

	index = (iris_max_color_temp-IRIS_CCT_MIN_VALUE)/IRIS_CCT_STEP;
	iris_max_x_value = iris_color_temp_x_get(index);

	pqlt_cur_setting->colortempvalue = 0;	// Use default color temperature from PCS.
	pqlt_cur_setting->ai_auto_en = AI_BACKLIGHT_ENABLE;

	iris_ai_lce_disable = false;

	if (iris_first_boot) {
		iris_first_boot = false;
		for (i = 0; i < 14; i++) {
			payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, 0xc0 + i, 2);
			iris_de_default[2 * i] = payload[0];
			iris_de_default[2 * i + 1] = payload[9];
		}
	}

	IRIS_LOGI("%s, iris_min_x_value=%d, iris_max_x_value = %d", __func__, iris_min_x_value, iris_max_x_value);
}

void iris_cm_ratio_set(void)
{
	u32 index;
	u32 index_default;
	u32 xvalue;
	u32 xvalue_default;
	u32 ratio;
	u32 value = 0;
	u32 value_default;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	struct iris_ip_opt *psopt;
	uint32_t *data = NULL;
	int i, k;
	//struct iris_cfg *pcfg = iris_get_cfg();
	uint16_t coefBuff_start = 0;
	uint32_t coefBuffIndex = pqlt_cur_setting->pq_setting.cmcolorgamut;
	uint32_t  *payload = NULL;

	//csc coef has 54 values + 27 precsc values + cct has 3 values.
	if ((pqlt_cur_setting->pq_setting.cmcolorgamut >= 7 && pqlt_cur_setting->pq_setting.cmcolorgamut <= 9) ||
		(pqlt_cur_setting->pq_setting.cmcolorgamut >= 12 && pqlt_cur_setting->pq_setting.cmcolorgamut <= 15))
		coefBuffIndex = 2;
	else if (pqlt_cur_setting->pq_setting.cmcolorgamut >= 10 &&  pqlt_cur_setting->pq_setting.cmcolorgamut <= 11)
		coefBuffIndex -= 3; //10-->7. 11-->8;

	if (coefBuffIndex < 0) {
		IRIS_LOGE("invalid coefBuffIndex %d", coefBuffIndex);
		return;
	}
	if (!iris_crstk_coef_buf) {
		IRIS_LOGE("iris_crstk_coef_buf is NULL");
		return;
	}
	coefBuff_start = coefBuffIndex * (CRSTK_COEF_SIZE/2 + CCT_VALUE_SIZE/2);
	iris_min_color_temp = iris_crstk_coef_buf[coefBuff_start+CRSTK_COEF_SIZE/2];
	value_default = iris_crstk_coef_buf[coefBuff_start+CRSTK_COEF_SIZE/2 + 1];
	iris_max_color_temp = iris_crstk_coef_buf[coefBuff_start+CRSTK_COEF_SIZE/2 + 2];

	if (iris_min_color_temp == 0)
		iris_min_color_temp = 2500;
	if (iris_max_color_temp == 0)
		iris_max_color_temp = 11000;
	if (value_default == 0)
		value_default = 6500;

	index = (iris_min_color_temp-IRIS_CCT_MIN_VALUE)/IRIS_CCT_STEP;
	iris_min_x_value = iris_color_temp_x_get(index);
	index = (iris_max_color_temp-IRIS_CCT_MIN_VALUE)/IRIS_CCT_STEP;
	iris_max_x_value = iris_color_temp_x_get(index);

	if (pqlt_cur_setting->pq_setting.cmcolortempmode == IRIS_COLOR_TEMP_MANUL) {
		if (pqlt_cur_setting->colortempvalue == 0)
			value = value_default;
		else
			value = pqlt_cur_setting->colortempvalue;
	} else if (pqlt_cur_setting->pq_setting.cmcolortempmode == IRIS_COLOR_TEMP_AUTO)
		value = pqlt_cur_setting->cctvalue;
	else
		value = value_default;

	if (value > iris_max_color_temp)
		value = iris_max_color_temp;
	else if (value < iris_min_color_temp)
		value = iris_min_color_temp;
	index = (value - IRIS_CCT_MIN_VALUE)/25;
	xvalue = iris_color_temp_x_get(index);

	if (value_default > iris_max_color_temp)
		value_default = iris_max_color_temp;
	else if (value_default < iris_min_color_temp)
		value_default = iris_min_color_temp;
	index_default = (value_default - IRIS_CCT_MIN_VALUE)/25;
	xvalue_default = iris_color_temp_x_get(index_default);

	IRIS_LOGD("cm color temperature default CCT=%d, xvalue_default = %d\n", value_default, xvalue_default);
	IRIS_LOGD("min_cct = %d, max_cct = %d\n", iris_min_color_temp, iris_max_color_temp);
	IRIS_LOGD("value = %d, index = %d, xvalue = %d\n", value, index, xvalue);
	IRIS_LOGD("cmcolorgamut=%d, coefBuff_start=%d, iris_max_x_value=%d, iris_min_x_value=%d\n",
		pqlt_cur_setting->pq_setting.cmcolorgamut, coefBuff_start, iris_max_x_value, iris_min_x_value);

	if (1 /*pcfg->lut_mode == SINGLE_MODE*/) {
		psopt = iris_find_ip_opt(IRIS_IP_DPP, 0x32);  //csc2 crstk coef
		if (!psopt) {
			IRIS_LOGE("can not find i_p=%x id=0x32", IRIS_IP_DPP);
			return;
		}
		data = (uint32_t *)psopt->cmd[0].msg.tx_buf;
		if ((xvalue >= iris_max_x_value) && (xvalue < xvalue_default)) {
			ratio = ((xvalue - iris_max_x_value)*10000)/(xvalue_default - iris_max_x_value);

			IRIS_LOGD("ratio:%d, xvalue: %d, iris_max_x_value: %d", ratio, xvalue, iris_max_x_value);
			for (i = 0; i < 18; i++) {
				nCSCCoffValue[i] = (iris_crstk_coef_buf[coefBuff_start+18+i] * ratio +
							(10000 - ratio)*iris_crstk_coef_buf[coefBuff_start+i])/10000;
			}

		} else if ((xvalue <=  iris_min_x_value) && (xvalue >= xvalue_default)) {
			ratio = ((xvalue - xvalue_default)*10000)/(iris_min_x_value - xvalue_default);

			IRIS_LOGD("ratio:%d, xvalue: %d, iris_min_x_value: %d", ratio, xvalue, iris_min_x_value);
			for (i = 0; i < 18; i++) {
				nCSCCoffValue[i] = (iris_crstk_coef_buf[coefBuff_start+36+i]*ratio +
							iris_crstk_coef_buf[coefBuff_start+18+i]*(10000-ratio))/10000;
			}
		}

		data[2] = nCSCCoffValue[1] << 16 | nCSCCoffValue[0];
		data[3] = nCSCCoffValue[3] << 16 | nCSCCoffValue[2];
		data[4] = nCSCCoffValue[5] << 16 | nCSCCoffValue[4];
		data[5] = nCSCCoffValue[7] << 16 | nCSCCoffValue[6];
		data[6] = 0x0000 << 16 | nCSCCoffValue[8];
		data[7] = nCSCCoffValue[10] << 16 | nCSCCoffValue[9];
		data[8] = nCSCCoffValue[12] << 16 | nCSCCoffValue[11];
		data[9] = nCSCCoffValue[14] << 16 | nCSCCoffValue[13];
		data[10] = nCSCCoffValue[16] << 16 | nCSCCoffValue[15];
		data[11] = 0x0000 << 16 | nCSCCoffValue[17];
		iris_init_update_ipopt_t(IRIS_IP_DPP, 0x32, 0x32, 0x01);

		psopt = iris_find_ip_opt(IRIS_IP_DPP, 0x33);  //csc2' coef
		if (!psopt) {
			IRIS_LOGE("could not find i_p=%d opt_id=0x33", IRIS_IP_DPP);
			return;
		}

		for (i = 0; i < 9; i++) {
			if (i/3 == 0)
				k = 1;
			else if (i/3 == 1)
				k = 5;
			else
				k = 6;
			if (dwCSC2CoffBuffer[i] > 0x4000) {
				dwCSC2CoffValue[i] = 0x8000 - dwCSC2CoffBuffer[i];
				dwCSC2CoffValue[i] = (dwCSC2CoffValue[i] * nCSCCoffValue[k]) / 0x800;
				dwCSC2CoffValue[i] = 0x8000 - dwCSC2CoffValue[i];
			} else {
				dwCSC2CoffValue[i] = (dwCSC2CoffBuffer[i] * nCSCCoffValue[k]) / 0x800;
			}
		}
		for (i = 9; i < 12; i++) {
			if (i == 9)
				k = 6;
			else if (i == 10)
				k = 1;
			else
				k = 5;
			if (dwCSC2CoffBuffer[i] > 0x80000) {
				dwCSC2CoffValue[i] = 0x100000 - dwCSC2CoffBuffer[i];
				dwCSC2CoffValue[i] = (dwCSC2CoffValue[i] * nCSCCoffValue[k]) / 0x800;
				if (dwCSC2CoffValue[i] >= 0x80000)
					dwCSC2CoffValue[i] = 0x7ffff;
				else
					dwCSC2CoffValue[i] = 0x100000 - dwCSC2CoffValue[i];
			} else {
				dwCSC2CoffValue[i] = (dwCSC2CoffBuffer[i] * nCSCCoffValue[k]) / 0x800;
				if (dwCSC2CoffValue[i] >= 0x80000)
					dwCSC2CoffValue[i] = 0x7ffff;
			}
		}

		if (dpp_precsc_enable == false) {
			data = (uint32_t *)psopt->cmd[0].msg.tx_buf;
			data[2] = dwCSC2CoffValue[1] << 16 | dwCSC2CoffValue[0];
			data[3] = dwCSC2CoffValue[3] << 16 | dwCSC2CoffValue[2];
			data[4] = dwCSC2CoffValue[5] << 16 | dwCSC2CoffValue[4];
			data[5] = dwCSC2CoffValue[7] << 16 | dwCSC2CoffValue[6];
			data[6] = 0x0000 << 16 | dwCSC2CoffValue[8];
			iris_init_update_ipopt_t(IRIS_IP_DPP, 0x33, 0x33, 0x01);

			payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0xc0, 2);
			for (i = 0; i < 3; i++)
				payload[i] = dwCSC2CoffValue[i + 9];
			iris_init_update_ipopt_t(IRIS_IP_DPP, 0xc0, 0xc0, 0x01);
		} else {
			data = (uint32_t *)psopt->cmd[0].msg.tx_buf;
			data[2] = nCSCCoffValue[1] << 16 | nCSCCoffValue[0];
			data[3] = nCSCCoffValue[3] << 16 | nCSCCoffValue[2];
			data[4] = nCSCCoffValue[5] << 16 | nCSCCoffValue[4];
			data[5] = nCSCCoffValue[7] << 16 | nCSCCoffValue[6];
			data[6] = 0x0000 << 16 | nCSCCoffValue[8];
			iris_init_update_ipopt_t(IRIS_IP_DPP, 0x33, 0x33, 0x01);

			payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0xc0, 2);
			for (i = 0; i < 3; i++)
				payload[i] = 0;
			iris_init_update_ipopt_t(IRIS_IP_DPP, 0xc0, 0xc0, 0x01);
		}
	}

	for (i = 0; i < 18; i++)
		IRIS_LOGD("nCSCCoffValue[%d] = 0x%04x", i, nCSCCoffValue[i]);

	IRIS_LOGD("cm color temperature value=%d", value);
}

u32 iris_cm_ratio_set_for_iic(void)
{
	u32 tablesel;
	u32 index;
	u32 xvalue;
	u32 ratio;
	u32 value;
	u32 regvalue = 0;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;

	value = pqlt_cur_setting->colortempvalue;

	if (value > iris_max_color_temp)
		value = iris_max_color_temp;
	else if (value < iris_min_color_temp)
		value = iris_min_color_temp;
	index = (value - IRIS_CCT_MIN_VALUE) / 25;
	xvalue = iris_color_temp_x_get(index);

	if (xvalue == iris_min_x_value) {
		tablesel = 0;
		regvalue = tablesel | 0x02;
	} else if ((xvalue < iris_min_x_value) && (xvalue >= IRIS_X_7700K)) {
		tablesel = 0;
		ratio = ((xvalue - IRIS_X_7700K) * 16383) / (iris_min_x_value - IRIS_X_7700K);
		regvalue = tablesel | (ratio << 16);
	} else if ((xvalue >= iris_max_x_value) && (xvalue < IRIS_X_7700K)) {
		tablesel = 1;
		ratio = ((xvalue - iris_max_x_value) * 16383) / (IRIS_X_7700K - iris_max_x_value);
		regvalue = tablesel | (ratio << 16);
	}

	IRIS_LOGI("cm color temperature value=%d", value);

	return regvalue;
}

void iris_cm_colortemp_mode_set(u32 mode, bool bcommit)
{
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;

	if (mode == 0) {
		iris_dpp_precsc_enable(0, false);
		iris_cm_color_gamut_set(0, false);
	} else {
		iris_dpp_precsc_enable(dpp_precsc_enable, false);
		iris_cm_color_gamut_set(pqlt_cur_setting->pq_setting.cmcolorgamut, false);
	}
	iris_cm_ratio_set();

	if (bcommit)
		iris_end_dpp(true);
	IRIS_LOGD("cm color temperature mode=%d", mode);
}

void iris_cm_color_temp_set(void)
{
	/*struct quality_setting *pqlt_cur_setting = & iris_setting.quality_cur;*/

	/*if(pqlt_cur_setting->pq_setting.cmcolorgamut == 0) {*/

	iris_cm_ratio_set();
	iris_end_dpp(true);

	/*}*/
	IRIS_LOGD("%s",  __func__);
}

void iris_cm_color_gamut_pre_set(u32 source_switch)
{
	struct iris_update_regval regval;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	/*add protection for source and scene switch at the same time*/
	if (source_switch == 3)
		source_switch = 1;
	pqlt_cur_setting->source_switch = source_switch;

	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0x50;
	regval.mask = 0x00000302;
	regval.value = 0x0000000;

	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x50, 0x50, 0x01);

	iris_end_dpp(true);

	IRIS_LOGD("source switch = %d", source_switch);
}

void iris_dpp_apl_enable(bool enable, uint8_t chain)
{
	uint32_t  *payload = NULL;
	//struct iris_update_ipopt popt[IP_OPT_MAX];
	uint32_t apl_ctrl = 0;
	uint32_t csc_coef_ctrl = 0;
	//enable/disable apl_ctrl
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x21, 2);
	apl_ctrl = payload[0] & 0xffffffcc;
	if (enable)
		apl_ctrl = apl_ctrl | 0x3 << 4 | 0x3;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x21, 2, apl_ctrl);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x21, 0x21, chain);

	//csc_coef_ctrl, CSC2_COEF_UPDATE_EN = 1 to enable
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x31, 2);
	csc_coef_ctrl = payload[0] & 0xfffffffe;
	//if (enable)
	//	csc_coef_ctrl = csc_coef_ctrl | 0x1;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x31, 2, csc_coef_ctrl);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x31, 0x31, chain);

}

void iris_dpp_3dlut_gain(u32 count, u32 *values, bool bcommit)
{
	uint32_t  *payload = NULL;
	int len;
	uint32_t lut3d_interp3;

	if (count == 1) {
		payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x52, 2);
		lut3d_interp3 = payload[0] & 0xffff0000;
		lut3d_interp3 = lut3d_interp3 | values[0];
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x52, 2, lut3d_interp3);
	} else if (count == 2) {
		payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x52, 2);
		lut3d_interp3 = (values[1] << 16) | values[0];
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x52, 2, lut3d_interp3);
	} else if (count == 3) {
		payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x52, 3);
		lut3d_interp3 = (values[1] << 16) | values[0];
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x52, 2, lut3d_interp3);
		lut3d_interp3 = payload[0] & 0xffff0000;
		lut3d_interp3 = lut3d_interp3 | values[2];
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x52, 3, lut3d_interp3);
	} else if (count == 4) {
		payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x52, 2);
		lut3d_interp3 = (values[1] << 16) | values[0];
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x52, 2, lut3d_interp3);
		lut3d_interp3 = (values[3] << 16) | values[2];
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x52, 3, lut3d_interp3);
	}
	len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x52, 0x52, 0x01);
	if (bcommit)
		iris_end_dpp(true);
	IRIS_LOGD("3dlut interpolation, gain=0x%x, len=%d", values[0], len);
}

void iris_dpp_3dlut_send(u8 lut_optid, uint8_t chain)
{
	u8 lut_type = DPP_3DLUT;

	iris_init_update_ipopt_t(lut_type, lut_optid, lut_optid, chain);

	if (chain == 0)
		iris_end_dpp(true);

	IRIS_LOGD("3dlut send optid: 0x%x", lut_optid);
}

void iris_cm_color_gamut_set(u32 level, bool bcommit)
{
	struct iris_update_regval regval;
	u32 gammalevel;
	uint32_t  *payload = NULL;
	uint32_t gammactrl = 0;
	uint32_t gammamode = 0;
	uint32_t currentmode;
	bool apl = 0;
	uint32_t lut3d_interp1, lut3d_interp2;
	uint32_t interp1_src, interp1_src2;
	uint32_t interp2_src, interp2_src2;
	uint32_t interp1_dst, interp2_dst;
	uint32_t interp3_src, interp3_src2, interp3_dst;
	u16 aplstatus_value = iris_get_firmware_aplstatus_value();

	//iris_dport_disable(0x1, 0x1);
	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0x50;

	if (level == 0) {  //3dlut bypass
		regval.mask = 0x00000023;
		regval.value = 0x0000020;
	} else {
		regval.mask = 0x00000023;
		//regval.value = 0x0 | ((level) << 10);
		regval.value = 0x0000003;
	}

	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x50, 0x50, 0x01);

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x51, 2);

#ifdef TABLE_ITSELF  //interpolation with itself
	if (level > 0 && level < 6) {
		interp1_src = level + 1;
		lut3d_interp1 = ((interp1_src << 10) | (interp1_src << 6) | 1);
	} else if (level >= 6 && level < 11) {
		interp1_src = level + 3;
		lut3d_interp1 = ((interp1_src << 10) | (interp1_src << 6) | 1);
	} else {
		lut3d_interp1 = 0;
	}
	lut3d_interp1 = (payload[0] & 0xffff8000) | lut3d_interp1;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x51, 2, lut3d_interp1);
#else
	switch (level) {
	case 1:  //Native          3d-lut:0x9
	case 2:  //Vivid No CAM    3d-lut:0xa
	case 3:  //VIvid High CAM  3d-lut:0xb
		interp1_src = level + 8;
		lut3d_interp1 = ((interp1_src << 10) | (interp1_src << 6) | 1);
		lut3d_interp1 = (payload[0] & 0xc0000000) | lut3d_interp1;
		lut3d_interp2 = 0;
		lut3d_interp2 = (payload[1] & 0xffff8000) | lut3d_interp2;
	break;
	case 4:  //sRGB            3d-lut:0x4
	case 5:  //P3              3d-lut:0x5
	case 6:  //BT2020,	   3d-lut 0x6
		interp1_src = level;
		lut3d_interp1 = ((interp1_src << 10) | (interp1_src << 6) | 1);
		lut3d_interp1 = (payload[0] & 0xc0000000) | lut3d_interp1;
		lut3d_interp2 = 0;
		lut3d_interp2 = (payload[1] & 0xffff8000) | lut3d_interp2;
	break;
	case 7: //Vivid No CAM + Vivid High CAM
		interp1_src = 0xa; interp1_src2 = 0xb;
		lut3d_interp1 = ((interp1_src << 10) | (interp1_src2 << 6) | 1);
		lut3d_interp1 = (payload[0] & 0xc0000000) | lut3d_interp1;
		lut3d_interp2 = 0;
		lut3d_interp2 = (payload[1] & 0xffff8000) | lut3d_interp2;
	break;
	case 8: //Vivid No CAM + Vivid High CAM->0x2, Native+Native->0x03
		interp1_src = 0xa; interp1_src2 = 0xb; interp1_dst = 0x2;
		lut3d_interp1 =  ((interp1_src << 10) | (interp1_src2 << 6) | (interp1_dst << 2) | 1);
		interp2_src = 0x9; interp2_src2 = 0x9; interp2_dst = 0x3;
		lut3d_interp1 |= ((interp2_src << 25) | (interp2_src2 << 21) | (interp2_dst << 17) | (1 << 15));
		interp3_src = 0x3; interp3_src2 = 0x2; interp3_dst = 0;
		lut3d_interp2 = ((interp3_src << 10) | (interp3_src2 << 6) | 1);
		lut3d_interp1 = (payload[0] & 0xc0000000) | lut3d_interp1;
		lut3d_interp2 = (payload[1] & 0xffff8000) | lut3d_interp2;
	break;
	case 9:
		interp1_src = 0xa; interp1_src2 = 0xb; interp1_dst = 0x2;
		lut3d_interp1 =  ((interp1_src << 10) | (interp1_src2 << 6) | (interp1_dst << 2) | 1);
		interp2_src = 0xc; interp2_src2 = 0xd; interp2_dst = 0x3;
		lut3d_interp1 |= ((interp2_src << 25) | (interp2_src2 << 21) | (interp2_dst << 17) | (1 << 15));
		interp3_src = 0x3; interp3_src2 = 0x2; interp3_dst = 0;
		lut3d_interp2 = ((interp3_src << 10) | (interp3_src2 << 6) | 1);
		lut3d_interp1 = (payload[0] & 0xc0000000) | lut3d_interp1;
		lut3d_interp2 = (payload[1] & 0xffff8000) | lut3d_interp2;
	break;
	case 10: //Vivid Low Brightness No CAM   3d-lut:0xc
	case 11: //Vidi Low Brightness High CAM  3d-lut:0xd
		interp1_src = level + 0x2;
		lut3d_interp1 = ((interp1_src << 10) | (interp1_src << 6) | 1);
		lut3d_interp1 = (payload[0] & 0xc0000000) | lut3d_interp1;
		lut3d_interp2 = 0;
		lut3d_interp2 = (payload[1] & 0xffff8000) | lut3d_interp2;
	break;
	case 12: //Vivid No CAM + Vivid High CAM->0x2, Native+Native->0x03
		interp1_src = 0xa; interp1_src2 = 0xc; interp1_dst = 0x2;
		lut3d_interp1 =  ((interp1_src << 10) | (interp1_src2 << 6) | (interp1_dst << 2) | 1);
		interp2_src = 0x9; interp2_src2 = 0x9; interp2_dst = 0x3;
		lut3d_interp1 |= ((interp2_src << 25) | (interp2_src2 << 21) | (interp2_dst << 17) | (1 << 15));
		interp3_src = 0x3; interp3_src2 = 0x2; interp3_dst = 0;
		lut3d_interp2 = ((interp3_src << 10) | (interp3_src2 << 6) | 1);
		lut3d_interp1 = (payload[0] & 0xc0000000) | lut3d_interp1;
		lut3d_interp2 = (payload[1] & 0xffff8000) | lut3d_interp2;
	break;
	case 13:
		interp1_src = 0x4; interp1_src2 = 0x5; interp1_dst = 0x2;
		lut3d_interp1 =  ((interp1_src << 10) | (interp1_src2 << 6) | (interp1_dst << 2) | 1);
		interp2_src = 0x9; interp2_src2 = 0x9; interp2_dst = 0x3;
		lut3d_interp1 |= ((interp2_src << 25) | (interp2_src2 << 21) | (interp2_dst << 17) | (1 << 15));
		interp3_src = 0x3; interp3_src2 = 0x2; interp3_dst = 0;
		lut3d_interp2 = ((interp3_src << 10) | (interp3_src2 << 6) | 1);
		lut3d_interp1 = (payload[0] & 0xc0000000) | lut3d_interp1;
		lut3d_interp2 = (payload[1] & 0xffff8000) | lut3d_interp2;
	break;
	case 14:
		interp1_src = 0xb; interp1_src2 = 0xd; interp1_dst = 0x2;
		lut3d_interp1 =  ((interp1_src << 10) | (interp1_src2 << 6) | (interp1_dst << 2) | 1);
		interp2_src = 0x9; interp2_src2 = 0x9; interp2_dst = 0x3;
		lut3d_interp1 |= ((interp2_src << 25) | (interp2_src2 << 21) | (interp2_dst << 17) | (1 << 15));
		interp3_src = 0x3; interp3_src2 = 0x2; interp3_dst = 0;
		lut3d_interp2 = ((interp3_src << 10) | (interp3_src2 << 6) | 1);
		lut3d_interp1 = (payload[0] & 0xc0000000) | lut3d_interp1;
		lut3d_interp2 = (payload[1] & 0xffff8000) | lut3d_interp2;
	break;
	case 15:
		interp1_src = 0xc; interp1_src2 = 0xd; interp1_dst = 0x2;
		lut3d_interp1 =  ((interp1_src << 10) | (interp1_src2 << 6) | (interp1_dst << 2) | 1);
		interp2_src = 0x9; interp2_src2 = 0x9; interp2_dst = 0x3;
		lut3d_interp1 |= ((interp2_src << 25) | (interp2_src2 << 21) | (interp2_dst << 17) | (1 << 15));
		interp3_src = 0x3; interp3_src2 = 0x2; interp3_dst = 0;
		lut3d_interp2 = ((interp3_src << 10) | (interp3_src2 << 6) | 1);
		lut3d_interp1 = (payload[0] & 0xc0000000) | lut3d_interp1;
		lut3d_interp2 = (payload[1] & 0xffff8000) | lut3d_interp2;
	break;

	default:
		lut3d_interp1 = 0;
		lut3d_interp2 = 0;
		lut3d_interp1 = (payload[0] & 0xc0000000) | lut3d_interp1;
		lut3d_interp2 = (payload[1] & 0xffff8000) | lut3d_interp2;
	break;
	}
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x51, 2, lut3d_interp1);
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x51, 3, lut3d_interp2);
#endif
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x51, 0x51, 0x01);

	apl = (aplstatus_value & (0x1 << level)) ? 1 : 0;
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x20, 2);
	currentmode = payload[0] & 0x7;
	if (apl == 0) {
		gammactrl = payload[0] & 0xff0; //65bin gamma
		gammalevel = 0x00 + level;

//#if 1
//level 7/8/9: -->interpolation <vivid no cam, vivid high cam, native, use vivid's gamma now.>
//level 10/11: vivid low brightness <also use vivid's gamma now>
//may be changed later again.
		if (level >= 7)
			gammalevel = 0x00 + 2;
//#endif
	} else {
		gammamode = 2; //17bin gamma
		gammactrl = ((payload[0] & 0xff0) | gammamode | (0x1 << 3));
		//gammalevel = 0x20+level;
		gammalevel = 0xa0 + level;
	}

	IRIS_LOGD("aplstauts: 0x%x, gammamctrl: %d, gammalevel: 0x%x", aplstatus_value, gammactrl, gammalevel);
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x20, 2, gammactrl);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x20, 0x20, 0x01);
	iris_dpp_apl_enable(apl, 0x01);
	iris_update_ip_opt(GAMMA_LUT, gammalevel, 0x01);

	iris_cm_ratio_set();

	if ((level >= 7 && level <= 9) || (level >= 12 && level <= 15))
		iris_update_ip_opt(DPP_PRE_LUT, 0x2, 0x1);
	else
		iris_update_ip_opt(DPP_PRE_LUT, level, 0x1);
	if (bcommit)
		iris_end_dpp(true);
	IRIS_LOGI("cm color gamut=%d", level);
}

#if 0
void iris_cm_color_gamut_set(u32 level)
{
	struct iris_update_regval regval;
	bool chain = 0;
	int len;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	u32 gammalevel;
	uint8_t path = iris_pq_update_path;

	iris_capture_disable_pq(popt, &chain);

	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0xfc;
	regval.mask = 0x00000011;
	regval.value = 0x00000011;

	/*use liner gamma if cm lut disable*/
	if (pqlt_cur_setting->pq_setting.cmcolortempmode ==
			IRIS_COLOR_TEMP_OFF)
		gammalevel = 0;
	else
		gammalevel = level + 1;


	iris_update_ip_opt(GAMMA_LUT, gammalevel, 0x01);
	iris_update_ip_opt(DPP_3DLUT, level * 3 + 0, 0x01);
	iris_update_ip_opt(DPP_3DLUT, level * 3 + 1, 0x01);
	len = iris_update_ip_opt(DPP_3DLUT,
			level*3 + 2, (pqlt_cur_setting->source_switch == 0) ? 0x01 : chain);

	/*do not generate lut table for source switch.*/
	if (pqlt_cur_setting->source_switch == 0) {
		iris_update_bitmask_regval_nonread(&regval, false);
		len = iris_init_update_ipopt_t(IRIS_IP_DPP,
				0xfc, 0xfc, skiplast);
	}
	len = iris_capture_enable_pq(popt, len);
	iris_update_pq_opt(path, true);
	IRIS_LOGI("cm color gamut=%d, len=%d", level, len);
}
#endif
void iris_cm_color_gamut_pre_clear(void)
{
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;

	if (pqlt_cur_setting->source_switch != 0) {
		pqlt_cur_setting->source_switch = 0;
		iris_cm_color_gamut_set(
				iris_setting.quality_cur.pq_setting.cmcolorgamut, false);
		iris_cm_colortemp_mode_set(
				iris_setting.quality_cur.pq_setting.cmcolortempmode, true);
	}
}

void iris_dpp_gamma_set(void)
{
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	u32 gammalevel;

	if (pqlt_cur_setting->pq_setting.cmcolortempmode
			== IRIS_COLOR_TEMP_OFF)
		gammalevel = 0; /*use liner gamma if cm lut disable*/
	else
		gammalevel = pqlt_cur_setting->pq_setting.cmcolorgamut + 1;

	iris_update_ip_opt(GAMMA_LUT,
			gammalevel, 0x01);

	iris_end_dpp(true);
}

void iris_dpp_precsc_enable(u32 enable, bool bcommit)
{
	uint32_t  *payload = NULL;
	uint32_t csc_ctrl = 0;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x30, 2);
	csc_ctrl = payload[0] & 0xdeff;
	if (enable)
		csc_ctrl = csc_ctrl | 0x1 << 8;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x30, 2, csc_ctrl);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x30, 0x30, 0x01);
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x30, 2);

	if (bcommit)
		iris_end_dpp(true);

	IRIS_LOGD("dpp precsc enable =%d", enable);
}

void iris_dpp_precsc_set(bool enable)
{
	dpp_precsc_enable = enable;
}

void iris_dpp_prelut_rgb_set(u32 lut_optid, bool bcommit)
{
	u8 lut_type = DPP_PRE_LUT;

	iris_init_update_ipopt_t(lut_type, lut_optid, lut_optid, 0x1);

	iris_end_dpp(bcommit);

	IRIS_LOGI("dpp pre lut rgb switch =%d", lut_optid);
}

void iris_dpp_gammamode_set(u32 gammamode, u32 gammaIndex)
{
//struct iris_update_regval regval;
	uint32_t  *payload = NULL;
	uint32_t currentmode = 0;
	uint32_t gammactrl = 0;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x20, 2);
	currentmode = payload[0] & 0x7;

	if ((gammamode == 0) && (currentmode != gammamode)) {
		//regval.value = 0;
		gammactrl = payload[0] & 0xff8;
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x20, 2, gammactrl);
		iris_init_update_ipopt_t(IRIS_IP_DPP, 0x20, 0x20, 0x01);

		iris_update_ip_opt(GAMMA_LUT, 0x00, 0x01);
	} else if (gammamode == 1) {
		gammactrl = ((payload[0]&0xfc0) | gammamode | (gammaIndex << 4));
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x20, 2, gammactrl);
		iris_init_update_ipopt_t(IRIS_IP_DPP, 0x20, 0x20, 0x01);

		if (currentmode != gammamode) {
			iris_update_ip_opt(GAMMA_LUT, 0x10, 0x01);
		}
	} else if (gammamode == 2) {
		gammactrl = ((payload[0]&0xfc0) | gammamode | (gammaIndex << 4));
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x20, 2, gammactrl);
		iris_init_update_ipopt_t(IRIS_IP_DPP, 0x20, 0x20, 0x01);

		if (currentmode != gammamode) {
			iris_update_ip_opt(GAMMA_LUT, 0x20, 0x01);
		}
	}

	iris_end_dpp(true);
}

/*enable: 1, demo window; 2: enable gamma_en*/
void iris_dpp_demo_window_set(u8 enable, u8 owAndGamma, u32 xWindow, u32 yWindow)
{
	uint32_t  *payload = NULL;
	uint32_t regvalue;

	IRIS_LOGD("%s, X:0x%x Y:0x%x, enable,out[%d, %d]", __func__, xWindow, yWindow, enable, owAndGamma);

	//len = iris_start_demo();
	// RESERVED
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x64, 4);
	IRIS_LOGD("DPP< 0xE0, pos 4, 0x%x", payload[0]);
	// payload[0]: WND_CTRL
	//bit0: WND_EN,  bit1: out_en; bit 2: gamma_en;
	if (enable == 2)
		regvalue = 0x5;
	else
		regvalue = (enable != 0 ? 1:0);
	regvalue |= (owAndGamma << 1);  //owAndGamma: 0x1: out_en, 0x2: gamma_en; 0x3: out_en&gamm_en
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x64, 2, regvalue);
	//iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, 0xD0, 2, regvalue);
	regvalue = xWindow;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x64, 3, regvalue);
	//iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, 0xD0, 3, regvalue);
	regvalue = yWindow;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x64, 4, regvalue);
	//iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, 0xD0, 4, regvalue);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x64, 0x64, 0x01);
	//len = iris_update_ip_opt(popt, IP_OPT_MAX, IRIS_IP_SDR2HDR, 0xD0,  0x01);

	iris_end_dpp(true);

	IRIS_LOGI("%s, X[0x%x], Y[0x%x], enable: %d", __func__, xWindow, yWindow, enable);

}

//shapeInFill: bit2~5, 0x0~0xf. [5:2]->[DIM_FIRST_EN, FILL_RPLACE_EN, FILL_INSIDE, FILL_SHAPE]
void iris_dpp_fingerDisplay_set(u8 enable, u8 shapeInFill, u32 radius_a, u32 radius_b,
								u32 radius_c, u32 radius_d, u32 position, u32 fillcolor)
{
	uint32_t  *payload = NULL;
	struct iris_update_regval regval;
	uint32_t regvalue = 0;
	uint32_t circle_div = 0xffffffff;
	uint32_t fade_width = 1;

	IRIS_LOGI("%s enable: %d, shape: %d, radius:%d %d %d %d, position:0x%x, fillcolor:0x%x",
				__func__, enable, shapeInFill, radius_a, radius_b, radius_c, radius_d, position, fillcolor);

	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0x60;    //DIM_CTRL
	regval.mask = 0x06;
	regvalue = ((enable?1:0) << 1 | (shapeInFill << 2));
	regval.value = regvalue;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x60, 0x60, 0x1);

	// RESERVED
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x62, 2);
	// payload[0]: eclipse A/B, [1]: eclipse C/D, [2]: fill center, [3]: fillcolor
	IRIS_LOGD("dpp, 0x62, radius0: 0x%x", payload[0]);
	regvalue = ((radius_a & 0x3ff) | ((radius_b & 0x3ff) << 16));
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x62, 2, regvalue);
	regvalue = ((radius_c & 0x3ff) | ((radius_d & 0x3ff) << 16));
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x62, 3, regvalue);
	regvalue = position;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x62, 4, regvalue);
	regvalue = fillcolor;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x62, 5, regvalue);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x62, 0x62, 0x01);

	if (shapeInFill) {
		if (radius_a/radius_b != radius_c/radius_d)
			IRIS_LOGI("Eclips rule is not obey!");
		circle_div = 68719476736 / (radius_a * radius_a - radius_c * radius_c) / (radius_b * radius_b);
	} else {
		if ((radius_a - radius_b) != (radius_c - radius_d))
			IRIS_LOGI("Rectangle rule is not obey!");
		fade_width = 4096/(radius_a - radius_c);
	}

	regvalue = circle_div;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x63, 2, regvalue);
	regvalue = (0xf | (fade_width << 8));
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x63, 3, regvalue);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x63, 0x63, 0x01);

	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x80, 0x80, 0x01);

	iris_end_dpp(true);
}

void iris_dpp_fadeinout_enable(u8 enable)
{
	struct iris_update_regval regval;

	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0x54;    //FADE_CTRL
	regval.mask = 0x00000001;
	regval.value = enable?1:0;

	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x54, 0x54, 0x01);

	iris_end_dpp(true);
}

void iris_dpp_fadeinout_step(u8 enable, u32 fadestep)
{
	struct iris_update_regval regval;
	uint32_t regvalue = 0;

	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0x54;    //FADE_CTRL
	regval.mask = 0x0000ffff;
	regvalue = ((fadestep<<4) | (enable?1:0));
	regval.value = regvalue;

	IRIS_LOGI("fadestep: %d, eanble: %d, regvalue: 0x%x", fadestep, enable, regvalue);
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x54, 0x54, 0x01);

	iris_end_dpp(true);
}

void iris_reading_mode_set(u32 level)
{
	IRIS_LOGI("reading mode=%d", level);
}

/*
void iris_maxcll_lut_set(u32 lutpos)
{
	int len;
	struct iris_update_regval regval;
	bool chain = 0;
	uint8_t path = iris_pq_update_path;

	iris_capture_disable_pq(popt, &chain);

	if (!(iris_sdr2hdr_lutyctl & 0xFFFF0000)) {
		len = iris_update_ip_opt(popt, IP_OPT_MAX, AMBINET_HDR_GAIN, 0x0, 1);

		regval.ip = IRIS_IP_SDR2HDR;
		regval.opt_id = 0x92;
		regval.mask = 0x0000FFFF;
		regval.value = ((lutpos << 6) & 0xc0) | (iris_sdr2hdr_lutyctl & 0xFF3F);
		iris_update_bitmask_regval_nonread(&regval, false);

		iris_sdr2hdr_lutyctl_set(lutpos);
		len += iris_init_update_ipopt_t(IRIS_IP_SDR2HDR,
				0x92, 0x92, chain);
	} else
		len = iris_update_ip_opt(popt, IP_OPT_MAX, AMBINET_HDR_GAIN, 0x0, chain);

	len = iris_capture_enable_pq(popt, len);
	iris_update_pq_opt(path, true);
	IRIS_LOGD("%s, len=%d", __func__, len);
}
*/
static bool iris_sdr2hdr_valid(u32 level)
{
	bool isValid = false;

	if (level < SDR2HDR_13)
		isValid = true;

	return isValid;
}

static void iris_hdr_datapath_set(void)
{
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	u32 power_level = HDR_POWER_OFF;
	uint32_t  *payload = NULL;

	if (pqlt_cur_setting->pq_setting.sdr2hdr > 0)
		power_level = HDR_POWER_ON;

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xf0, 4);
	payload[0] = ((payload[0] & ~0x00000008) | (power_level << 3));
	iris_set_ipopt_payload_data(IRIS_IP_PWIL, 0xf0, 4, payload[0]);
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xf0, 0xf0, 0x01);
}

void iris_hdr_power_set(void)
{
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	u32 power_level = HDR_POWER_OFF;

	if (pqlt_cur_setting->pq_setting.sdr2hdr > 0)
		power_level = HDR_POWER_ON;

	iris_pmu_hdr_set((power_level == HDR_POWER_ON ? 1:0), 0);
}

void iris_lux_set(u32 level, bool update)
{
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR_2, 0x50, 2, level >> 1);
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR_2, 0x50, 0x50, 1);

	if (update)
		iris_end_hdr(true);
	IRIS_LOGW("lux value =%d", level);
}

void iris_aux_channel_lux_set(u32 level)
{
	uint32_t  *payload = NULL;

	level = level >> 1;
	payload = iris_get_ipopt_payload_data(IRIS_IP_BLEND, 0x90, 9);
	payload[0] = (payload[0] & ~0x0ffff000) | (level << 12);

	iris_update_ip_opt(IRIS_IP_BLEND, 0x90, 1);

	iris_end_blending(true);
	IRIS_LOGW("aux lux value =%d", level);
}

void iris_al_enable(bool enable)
{
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	uint32_t  *payload = NULL;
	uint32_t  *payload2 = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, pqlt_cur_setting->pq_setting.sdr2hdr, 2);
	payload2 = iris_get_ipopt_payload_data(IRIS_IP_BLEND, 0x90, 2);

	payload2[0] &= ~0x00000001;
	if (enable == true) {
		if (pqlt_cur_setting->pq_setting.sdr2hdr >= 9)
			payload[0] = ((payload[0] & ~0x00006000) | (0x00000002 << 13));
		else
			payload[0] = ((payload[0] & ~0x00006000) | (0x00000001 << 13));
		payload2[0] |= 0x00000001;
	} else {
		payload[0] = payload[0] & ~0x00006000;
	}
	iris_update_ip_opt(IRIS_IP_SDR2HDR, pqlt_cur_setting->pq_setting.sdr2hdr, 1);
	iris_update_ip_opt(IRIS_IP_BLEND, 0x90, 1);

	iris_end_hdr_blending(true);
	IRIS_LOGW("al enable =%d", enable);
}

void iris_sdr2hdr_lut_set(u32 level)
{
	u32 lut_level;
	uint32_t  *payload = NULL;
	u32 mask = 0x00000008;
	bool lut_switch = true;

	if (level == SDR2HDR_Bypass) {
		iris_sdr2hdr_lut_index = 0;
		iris_sdr2hdr_current_level = level;
		lut_switch = false;
	}

	if (level == iris_sdr2hdr_current_level)
		lut_switch = false;

	if ((level <= SDR2HDR_13) && (level >= SDR2HDR_1)) {
		if ((iris_sdr2hdr_current_level <= SDR2HDR_13)
			&& (iris_sdr2hdr_current_level >= SDR2HDR_1)) {
			lut_switch = false;
		}
	}
	iris_sdr2hdr_current_level = level;

	switch (level) {
	case HDR10:
		lut_level = 0x02;
		break;
	case HLG_HDR:
		lut_level = 0x05;
		break;
	case SDR2HDR_1:
	case SDR2HDR_2:
	case SDR2HDR_3:
	case SDR2HDR_4:
	case SDR2HDR_5:
	case SDR2HDR_6:
	case SDR2HDR_7:
	case SDR2HDR_8:
	case SDR2HDR_9:
	case SDR2HDR_10:
	case SDR2HDR_11:
	case SDR2HDR_12:
	case SDR2HDR_13:
		lut_level = 0x01;
		break;
	default:
		lut_level = 0x01;
		break;
	}

	if (lut_switch == true) {
		iris_sdr2hdr_lut_index ^= 1;
		if (iris_sdr2hdr_lut_index == 1)
			lut_level += 0x10;
		iris_update_ip_opt(SDR2HDR_LUT, lut_level, 0x01);
	}
	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, level, 3);
	if (iris_sdr2hdr_lut_index == 0)
		payload[0] &= ~mask;
	else
		payload[0] |= mask;
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, level, 3, payload[0]);
}

void iris_sdr2hdr_ai_enable(bool enable, bool update)
{
	uint32_t  *payload = NULL;
	u32 last = 1;

	if (update)
		last = 0;
	payload = iris_get_ipopt_payload_data(IRIS_IP_AI, 0x10, 2);
	payload[0] &= ~0x40000000;
	if (enable == true)
		payload[0] |= 0x40000000;
	iris_init_update_ipopt_t(IRIS_IP_AI, 0x10, 0x10, last);
	if (update == true)
		iris_update_pq_opt(iris_pq_update_path, true);
}

void iris_sdr2hdr_allow(bool enable)
{
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;

	if (pqlt_cur_setting->pq_setting.sdr2hdr == SDR2HDR_Bypass)
		return;

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, pqlt_cur_setting->pq_setting.sdr2hdr, 2);
	payload[0] &= ~0x00000001;
	if (enable == false)
		payload[0] |= 0x00000001;
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, pqlt_cur_setting->pq_setting.sdr2hdr,
		pqlt_cur_setting->pq_setting.sdr2hdr, 0x01);
	IRIS_LOGE("%s, payload = 0x%x", __func__, payload[0]);
	iris_end_hdr(true);
}

static void iris_blending_lut_enable(bool enable)
{
	uint32_t  *payload = NULL;
	uint32_t  *payload2 = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_BLEND, 0x60, 2);
	payload2 = iris_get_ipopt_payload_data(IRIS_IP_BLEND, 0x50, 2);
	payload[0] &= ~0x00000001;
	payload[1] &= ~0x00000001;
	payload2[0] &= ~0x00000001;
	if (enable == true) {
		payload[0] |= 0x00000001;
		payload[1] |= 0x00000001;
		payload2[0] |= 0x00000001;
	}
}

void iris_pwil_dport_disable(bool enable, u32 value)
{
	u32 cmd[4];

	cmd[0] = IRIS_PWIL_ADDR + PWIL_DISP_CTRL1;
	cmd[1] = 0x00000000;
	cmd[1] |= (value << 27);
	if (enable)
		cmd[1] |= 0x04000000;
	cmd[2] = IRIS_PWIL_ADDR + PWIL_REG_UPDATE;
	cmd[3] = 0x00000100;

	iris_ocp_write_mult_vals(4, cmd);
}

void iris_sdr2hdr_hdr_en(void)
{
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	bool update = true;

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, pqlt_cur_setting->pq_setting.sdr2hdr, 2);

	if (pqlt_cur_setting->pq_setting.sdr2hdr >= 9 && sdr2hdr_level_orig < 9) {
		payload[0] |= 0x00000200;
		iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, pqlt_cur_setting->pq_setting.sdr2hdr,
			pqlt_cur_setting->pq_setting.sdr2hdr, 0x01);
	}
	if (pqlt_cur_setting->pq_setting.sdr2hdr == HDR10) {
		payload[0] |= 0x00000100;
		iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, pqlt_cur_setting->pq_setting.sdr2hdr,
			pqlt_cur_setting->pq_setting.sdr2hdr, 0x01);
		if (sdr2hdr_level_orig >= 9)
			iris_sdr2hdr_ai_enable(false, false);
		iris_end_hdr(true);
	} else if (pqlt_cur_setting->pq_setting.sdr2hdr > HDR10 && lce_sel > 0) {
		payload[0] |= lce_sel;
		iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, pqlt_cur_setting->pq_setting.sdr2hdr,
			pqlt_cur_setting->pq_setting.sdr2hdr, 0x01);
		if (sdr2hdr_level_orig >= 9)
			iris_sdr2hdr_ai_enable(false, false);
		iris_end_hdr(true);
		lce_sel = 0;
		update = false;
	} else if (pqlt_cur_setting->pq_setting.sdr2hdr == SDR2HDR_Bypass) {
		if (sdr2hdr_level_orig >= 9)
			iris_sdr2hdr_ai_enable(false, false);
		iris_hdr_datapath_set();
		iris_end_pwil(true);
		iris_hdr_power_set();
	} else if (pqlt_cur_setting->pq_setting.sdr2hdr < 9 &&
		sdr2hdr_level_orig >= 9) {
		iris_sdr2hdr_ai_enable(false, true);
	}
	if (update == true)
		iris_end_hdr(true);

	sdr2hdr_level_orig = pqlt_cur_setting->pq_setting.sdr2hdr;

	if (pqlt_cur_setting->pq_setting.sdr2hdr != SDR2HDR_Bypass) {
		payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, pqlt_cur_setting->pq_setting.sdr2hdr, 2);
		payload[2] = 0x00000800;
		iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, pqlt_cur_setting->pq_setting.sdr2hdr,
			pqlt_cur_setting->pq_setting.sdr2hdr, 0);
		iris_end_hdr(true);
	}
	if (iris_ai_lce_disable == true) {
		payload[0] |= lce_sel_restore;
		lce_sel_restore = 0;
	}
}

void iris_sdr2hdr_hdr_update(void)
{
	iris_end_hdr(true);
}

void iris_sdr2hdr_level_set(u32 level)
{
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	bool dma_sent = false;
	//struct iris_update_regval regval;
	u32 pwil_csc = 0x40;
	u32 blending_csc = 0x40;
	uint32_t  *payload = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!iris_sdr2hdr_valid(level))
		return;

	if (level >= 9) {
		iris_sdr2hdr_ai_enable(true, false);
		/* mcu */
		iris_mcu_state_set(MCU_START);
	} else if (!(pcfg->frc_enabled || pcfg->pwil_mode == FRC_MODE) &&
				iris_mcu_mode_get() == MCU_START) {
		/* mcu */
		iris_mcu_state_set(MCU_INT_DISABLE);

		iris_mcu_state_set(MCU_STOP);
	}

	if ((level <= HDR10) && (level >= HLG_HDR)) {
		shadow_iris_HDR10 = true;
	} else {
		shadow_iris_HDR10 = false;
		shadow_iris_HDR10_YCoCg = false;
	}

	if ((level <= HDR10) && (level >= HLG_HDR)) {
		pwil_csc = 0x41;   //2020
		blending_csc = 0x42;   //P3
	} else {
		pwil_csc = 0x40;   //709
		blending_csc = 0x40;   // 709
	}

	iris_blending_lut_enable(shadow_iris_HDR10);
	iris_sdr2hdr_lut_set(level);

	iris_update_ip_opt(IRIS_IP_PWIL, pwil_csc, 0x01);

	iris_update_ip_opt(IRIS_IP_BLEND, blending_csc, 0x01);

	iris_update_ip_opt(IRIS_IP_BLEND, 0x60, 0x01);

	iris_update_ip_opt(IRIS_IP_BLEND, 0x50, 0x01);

	if (level != SDR2HDR_Bypass) {
		payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, 0xc0 + level, 2);
		if (pcfg->pt_sr_enable == false) {
			payload[0] = iris_de_default[level * 2];
			payload[9] = iris_de_default[level * 2 + 1];
		} else {
			payload[0] = iris_de_disable[0];
			payload[9] = iris_de_disable[1];
		}

		iris_update_ip_opt(IRIS_IP_SDR2HDR, 0x10 + level, 0x01);

		iris_init_update_ipopt_t(IRIS_IP_SDR2HDR,
				0x30, 0x30, 0x01);

		iris_update_ip_opt(IRIS_IP_SDR2HDR, 0x40 + level, 0x01);

		iris_update_ip_opt(IRIS_IP_SDR2HDR, 0x50 + level, 0x01);

		iris_update_ip_opt(IRIS_IP_SDR2HDR, 0x60 + level, 0x01);

		iris_update_ip_opt(IRIS_IP_SDR2HDR, 0x70 + level, 0x01);

		iris_update_ip_opt(IRIS_IP_SDR2HDR, 0x90 + level, 0x01);

		iris_update_ip_opt(IRIS_IP_SDR2HDR, 0xa0 + level, 0x01);

		iris_update_ip_opt(IRIS_IP_SDR2HDR, 0xb0 + level, 0x01);

		iris_update_ip_opt(IRIS_IP_SDR2HDR, 0xc0 + level, 0x01);

		iris_update_ip_opt(IRIS_IP_SDR2HDR, 0xd0 + level, 0x01);

		iris_update_ip_opt(IRIS_IP_SDR2HDR_2, level, 0x01);

		iris_update_ip_opt(IRIS_IP_SDR2HDR_2, 0x10 + level, 0x01);

		iris_update_ip_opt(IRIS_IP_SDR2HDR_2, 0x20 + level, 0x01);

		iris_ai_bl_al_enable(pqlt_cur_setting->ai_auto_en, false);

		payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, level, 2);
		payload[2] = 0xFFFFFBFF;
		if (iris_ai_lce_disable == true) {
			lce_sel_restore = payload[0] & 0x00001800;
			payload[0] &= ~0x00001800;
		}
		if (pqlt_cur_setting->pq_setting.alenable == true) {
			if (level >= 9)
				payload[0] = ((payload[0] & ~0x00006000) | (0x00000002 << 13));
			else
				payload[0] = ((payload[0] & ~0x00006000) | (0x00000001 << 13));
		} else {
			payload[0] = payload[0] & ~0x00006000;
		}
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, level, 2, payload[0]);

		if (level == HDR10 && pcfg->tx_mode == 0) {
			payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, level, 2);
			payload[0] &= ~0x00000300;
		} else if (level > HDR10 && sdr2hdr_level_orig <= HDR10) {
			payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, level, 2);
			lce_sel = payload[0] & 0x00001820;
			payload[0] &= ~0x00001820;
		}

		if (level >= 9 && sdr2hdr_level_orig < 9) {
			payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, level, 2);
			payload[0] &= ~0x00000300;
		}
	} else {
		payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, sdr2hdr_level_orig, 2);
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, level, 2, (payload[0] | 0x00000001));
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, level, 4, 0xFFFFFBFF);
		iris_update_ip_opt(IRIS_IP_SDR2HDR, 0xd0 + level, 0x01);
	}

	iris_update_ip_opt(IRIS_IP_SDR2HDR, level, 0x01);

	if (!iris_skip_dma) {
		iris_hdr_datapath_set();
		if (!iris_dynamic_power_get()) {
			// HDR + BLENDING + PWIL
			if (sdr2hdr_level_orig == 0 &&
				pqlt_cur_setting->pq_setting.sdr2hdr > 0){
				if (pcfg->frc_enabled || pcfg->pwil_mode == FRC_MODE)
					iris_dma_gen_ctrl(0xc, 7, 1);
				else
					iris_dma_gen_ctrl(0xc, 5, 1);
				iris_hdr_power_set();
			} else {
				if (pcfg->frc_enabled || pcfg->pwil_mode == FRC_MODE)
					iris_dma_gen_ctrl(0xe, 7, 0);
				else
					iris_dma_gen_ctrl(0xe, 4, 0);
				iris_update_pq_opt(iris_pq_update_path, true);
			}
		} else {
			//PWIL
			if (sdr2hdr_level_orig == 0 &&
				pqlt_cur_setting->pq_setting.sdr2hdr > 0) {
				iris_dma_trig(DMA_CH12, 1);
				iris_hdr_power_set();
			} else if (sdr2hdr_level_orig > 0 &&
				pqlt_cur_setting->pq_setting.sdr2hdr == 0) {
				iris_dma_trig(DMA_CH12, 0);
				iris_update_pq_opt(iris_pq_update_path, true);
			} else {
				iris_dma_trig(DMA_CH12 | DMA_CH11, 0);
				iris_update_pq_opt(iris_pq_update_path, true);
			}
		}
		dma_sent = true;
	}

	if (dma_sent) {
		IRIS_LOGI("AP csc prepare.");
		iris_HDR10 = shadow_iris_HDR10;
		iris_HDR10_YCoCg = shadow_iris_HDR10_YCoCg;
		pqlt_cur_setting->dspp_dirty = 1;
	}

	IRIS_LOGI("sdr2hdr level =%d", level);
}

static void _iris_sdr2hdr_update_demo_win2(u32 win_x, u32 win_y)
{
	uint8_t cur_level = iris_sdr2hdr_current_level;
	uint32_t *payload = NULL;
	uint32_t x1 = 0;
	uint32_t x2 = 0;
	uint32_t y1 = 0;
	uint32_t y2 = 0;
	uint32_t x1_2 = 0;
	uint32_t x2_2 = 0;
	uint32_t y1_2 = 0;
	uint32_t y2_2 = 0;

	if (hdr_img_size[0] == 0 || hdr_img_size[1] == 0 ||
			hdr_img_size[2] == 0 || hdr_img_size[3] == 0)
		return;

	x1 = BITS_GET(win_x, 12, 0);
	x2 = BITS_GET(win_x, 12, 16);
	y1 = BITS_GET(win_y, 12, 0);
	y2 = BITS_GET(win_y, 12, 16);
	x1_2 = x1 * hdr_img_size[2] / hdr_img_size[0];
	x2_2 = x2 * hdr_img_size[2] / hdr_img_size[0];
	y1_2 = y1 * hdr_img_size[3] / hdr_img_size[1];
	y2_2 = y2 * hdr_img_size[3] / hdr_img_size[1];

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR_2, cur_level + 0x40, 2);

	IRIS_LOGI("%s(), current WND_X_2 0x%08X, WND_Y_2 0x%08X", __func__, payload[0], payload[1]);
	payload[0] = BITS_SET(x1_2 & BITS_MSK(12), 12, 16, x2_2);
	payload[1] = BITS_SET(y1_2 & BITS_MSK(12), 12, 16, y2_2);
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR_2, cur_level + 0x40, 2, payload[0]);
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR_2, cur_level + 0x40, 3, payload[1]);

	IRIS_LOGI("%s(), set WND_X_2 0x%08X, WND_Y_2 0x%08X", __func__, payload[0], payload[1]);
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR_2,
			cur_level + 0x40, cur_level + 0x40, 1);
}

void iris_sdr2hdr_set_demo_win(u8 demo_en, u8 win_out, u32 win_x, u32 win_y)
{
	uint8_t cur_level = iris_sdr2hdr_current_level;
	uint32_t *payload = NULL;
	uint32_t reg_val = 0;


	IRIS_LOGI("%s(), demo en: %s, out win: %s, win x: 0x%08X[%d, %d], win y: 0x%08X[%d, %d]",
			__func__,
			demo_en == 1 ? "true" : "false",
			win_out == 1 ? "true" : "false",
			win_x, BITS_GET(win_x, 12, 0), BITS_GET(win_x, 12, 16),
			win_y, BITS_GET(win_y, 12, 0), BITS_GET(win_y, 12, 16));

	/* set DEMO_EN(bit5) and WND_OUT_EN(bit28) */
	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, cur_level + 0x00, 2);
	reg_val = payload[0];

	IRIS_LOGI("%s(), current HDR_CTRL 0x%08X", __func__, reg_val);
	reg_val = BITS_SET(reg_val, 1, 21, demo_en & BITS_MSK(1));
	reg_val = BITS_SET(reg_val, 1, 28, win_out & BITS_MSK(1));

	IRIS_LOGI("%s(), set HDR_CTRL 0x%08X", __func__, reg_val);
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, cur_level + 0x00, 2, reg_val);
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR,
			cur_level + 0x00, cur_level + 0x00, 1);

	/* set WND_X and WND_Y */
	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR_2, cur_level + 0x30, 2);

	IRIS_LOGI("%s(), current WND_X 0x%08X, WND_Y 0x%08X", __func__, payload[0], payload[1]);
	payload[0] = win_x;
	payload[1] = win_y;
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR_2, cur_level + 0x30, 2, payload[0]);
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR_2, cur_level + 0x30, 3, payload[1]);

	/* set WND_X_2 and WND_Y_2 */
	_iris_sdr2hdr_update_demo_win2(win_x, win_y);

	IRIS_LOGI("%s(), set WND_X 0x%08X, WND_Y 0x%08X", __func__, payload[0], payload[1]);
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR_2,
			cur_level + 0x30, cur_level + 0x30, 0x01);
	iris_end_hdr(true);
}

void iris_sdr2hdr_set_img_size(uint32_t img_width, uint32_t img_height)
{
	IRIS_LOGI("%s(), set size: %u x %u", __func__, img_width, img_height);

	hdr_img_size[0] = img_width;
	hdr_img_size[1] = img_height;
}

static void _iris_sdr2hdr_save_size2(uint32_t img_width, uint32_t img_height)
{
	IRIS_LOGD("%s(), set size2: %u x %u", __func__, img_width, img_height);

	hdr_img_size[2] = img_width;
	hdr_img_size[3] = img_height;
}

void iris_sdr2hdr_update_size2(uint32_t img_width, uint32_t img_height)
{
	uint32_t *payload = NULL;
	uint32_t reg_val = 0;
	uint32_t fmt_width = img_width - 1;
	uint32_t fmt_height = img_height - 1;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;

	IRIS_LOGI("%s(), set image size2 %u x %u", __func__, img_width, img_height);
	_iris_sdr2hdr_save_size2(img_width, img_height);

	/* for TOP win, HDR_IMG_SIZE_2, FILM_ROW_2, FILM_COL_2, TM_VAL_NUM_RATIO_2 */
	{
		payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, 0x30, 2);

		reg_val = img_width & BITS_MSK(14);
		payload[0] = BITS_SET(reg_val, 14, 16, img_height);

		payload[1] = BITS_SET(0, 14, 16, fmt_height);
		payload[2] = BITS_SET(0, 14, 16, fmt_width);

		reg_val = (1 << 30) / (img_width * img_height);
		payload[9] = BITS_SET(0, 30, 0, reg_val);

		IRIS_LOGD("%s(), HDR_IMG_SIZE_2: 0x%08X, FILM_ROW_2: 0x%08X, ",
				"FILM_COL_2: 0x%08X, TM_VAL_NUM_RATIO_2: 0x%08X",
				__func__, payload[0], payload[1], payload[2], payload[9]);
	}

	/* for NOISE win, NOISE_WIN_COL_2, NOISE_WIN_ROW_2, NOISE_REC_BLK_NUM_2 */
	{
		const uint32_t ne_blk_sz = 8;
		uint32_t blk_dim_sz = 0;

		payload[5] = BITS_SET(0, 14, 16, fmt_width);
		payload[6] = BITS_SET(0, 14, 16, fmt_height);

		blk_dim_sz = img_width / ne_blk_sz;
		blk_dim_sz *= img_height / ne_blk_sz;
		reg_val = ((1 << 28) + blk_dim_sz / 2) / blk_dim_sz;
		payload[4] = reg_val;

		IRIS_LOGD("%s(), NOISE_REC_BLK_NUM_2: 0x%08X, ",
				"NOISE_WIN_COL_2: 0x%08X, NOISE_WIN_ROW_2: 0x%08X",
				__func__, payload[4], payload[5], payload[6]);
	}

	/* for DE win, DETAIL_WIN_H_2, DETAIL_WIN_V_2 */
	{
		payload[10] = BITS_SET(0, 12, 16, fmt_width);
		payload[11] = BITS_SET(0, 12, 16, fmt_height);

		IRIS_LOGD("%s(), DETAIL_WIN_H_2: 0x%08X, DETAIL_WIN_V_2: 0x%08X",
				__func__, payload[10], payload[11]);
	}

	/* for FLESH DET win, JUDGE_FLESH_12, JUDGE_FLESH_13, JUDGE_FLESH_14, JUDGE_FLESH_15 */
	{
		const uint32_t img_sz_bit = 30;

		payload[12] = BITS_SET(0, 12, 12, fmt_width);
		payload[13] = BITS_SET(0, 12, 12, fmt_height);

		reg_val = (1 << img_sz_bit) / (img_width * img_height);
		payload[14] = reg_val;

		reg_val = (1024 * 1024 * 1024) / (img_width * img_height);
		payload[15] = reg_val;

		IRIS_LOGD("%s(), JUDGE_FLESH_12: 0x%08X, JUDGE_FLESH_13: 0x%08X, ",
				"JUDGE_FLESH_14: 0x%08X, JUDGE_FLESH_15: 0x%08X",
				__func__, payload[12], payload[13], payload[14], payload[15]);
	}

	/* for LCE WGT win, AHE_SIZE_2, WGT_NORM_DIV_H_2, WGT_NORM_DIV_V_2 */
	{
		const uint32_t lce_h_blk = 3;
		const uint32_t lce_v_blk = 5;
		uint32_t blk_h_sz = 0;
		uint32_t blk_v_sz = 0;
		uint32_t blk_param = 32 << (25 - 10);

		blk_h_sz = (fmt_width + lce_h_blk) / lce_h_blk;
		if (blk_h_sz % 2 != 0)
			blk_h_sz += 1;
		blk_v_sz = (fmt_height + lce_v_blk) / lce_v_blk;

		reg_val = blk_h_sz & BITS_MSK(12);
		payload[3] = BITS_SET(reg_val, 12, 12, blk_v_sz);

		payload[7] = blk_param / blk_h_sz;
		payload[8] = blk_param / blk_v_sz;

		IRIS_LOGD("%s(), AHE_SIZE_2: 0x%08X, WGT_NORM_DIV_H_2: 0x%08X, WGT_NORM_DIV_V_2: 0x%08X",
				__func__, payload[3], payload[7], payload[8]);
	}

	/* set WND_X_2 and WND_Y_2 */
	//payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR_2, cur_level + 0x30, 2);
	//if (payload[2] != 0 || payload[3] != 0)
	//	_iris_sdr2hdr_update_demo_win2(payload[2], payload[3]);

	if (pqlt_cur_setting->pq_setting.sdr2hdr > 0) {
		iris_init_update_ipopt_t(IRIS_IP_SDR2HDR,
				0x30, 0x30, 0x01);

		iris_end_hdr(true);
	}
}

void iris_sdr2hdr_set_lce(u32 value)
{
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	u32 option = 0x50 + pqlt_cur_setting->pq_setting.sdr2hdr;

	if (!iris_sdr2hdr_valid(pqlt_cur_setting->pq_setting.sdr2hdr))
		return;

	if (value > 0) {
		payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 3);
		payload[6] = (payload[0] & 0xff00ffff) | (iris_lce_level[0][value-1] << 16);

		payload[17] = iris_lce_level[1][value-1];

		payload[7] = (payload[0] & 0xff00ffff) | (iris_lce_level[2][value-1] << 16);

		payload[16] = iris_lce_level[3][value-1];

		payload[3] = iris_lce_level[4][value-1] + (iris_lce_level[5][value-1] << 8)
			+ (iris_lce_level[7][value-1] << 16) + (iris_lce_level[8][value-1] << 24);

		payload[0] = (payload[0] & 0x003fffff) | (iris_lce_level[6][value-1] << 22);

		payload[1] = (payload[0] & 0x000fffff) | (iris_lce_level[9][value-1] << 20);

		payload[4] = (payload[0] & 0x000fffff) | (iris_lce_level[10][value-1] << 20);

		payload[5] = (payload[0] & 0x000fffff) | (iris_lce_level[11][value-1] << 20);

		payload[2] = (payload[0] & 0x000fffff) | (iris_lce_level[12][value-1] << 20);

		iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, option, option, 0x01);
	}
	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, pqlt_cur_setting->pq_setting.sdr2hdr, 2);
	if (value == 0)
		payload[0] &= 0xffffe7ff;
	else
		payload[0] |= (payload[0] & 0xffffe7ff) | 0x00000800;
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, pqlt_cur_setting->pq_setting.sdr2hdr,
		pqlt_cur_setting->pq_setting.sdr2hdr, 0x01);

	iris_end_hdr(true);

	IRIS_LOGI("%s(), value: %u", __func__, value);
}

void iris_sdr2hdr_set_de(u32 value)
{
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	u32 option = 0xC0 + pqlt_cur_setting->pq_setting.sdr2hdr;

	if (!iris_sdr2hdr_valid(pqlt_cur_setting->pq_setting.sdr2hdr))
		return;

	if (value > 0) {
		payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 3);
		payload[0] = iris_de_level[0][value-1] + (iris_de_level[1][value-1] << 10)
			+ (iris_de_level[2][value-1] << 20);
		payload[1] = (payload[1] & 0xfff00000) + iris_de_level[3][value-1]
			+ (iris_de_level[4][value-1] << 10);
		iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, option, option, 0x01);
	}

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, pqlt_cur_setting->pq_setting.sdr2hdr, 2);
	if (value == 0)
		payload[0] &= 0xffffffdf;
	else
		payload[0] |= (payload[0] & 0xffffffdf) | 0x00000020;
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, pqlt_cur_setting->pq_setting.sdr2hdr,
		pqlt_cur_setting->pq_setting.sdr2hdr, 0x01);

	iris_end_hdr(true);

	IRIS_LOGI("%s(), value: %u", __func__, value);
}

void iris_sdr2hdr_set_tf_coef(u32 value)
{
	uint8_t cur_level = iris_sdr2hdr_current_level;
	uint32_t *payload = NULL;
	uint32_t reg_val = 0;
	uint32_t tf_coef_arry[] = {1024, 1024, 1024, 1024, 1024};
	uint32_t *tf_coef = hdr_tf_coef;
	bool save_coef = false;

	IRIS_LOGI("%s(), value: %u, current HDR level: %u",
			__func__, value, cur_level);

	if (cur_level == SDR2HDR_Bypass)
		return;

	if (value == 1) {
		tf_coef = tf_coef_arry;
		save_coef = true;
	}

	/* for LCE_TF_COEF */
	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, cur_level + 0x50, 2);
	reg_val = payload[13];
	if (save_coef)
		hdr_tf_coef[0] = BITS_GET(reg_val, 11, 15);
	payload[13] = BITS_SET(reg_val, 11, 15, tf_coef[0]);
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR,
			cur_level + 0x50, cur_level + 0x50, 1);

	/* for DLV_TF_COEF */
	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, cur_level + 0x60, 2);
	if (save_coef)
		hdr_tf_coef[1] = payload[4] & BITS_MSK(11);
	payload[4] = tf_coef[1] & BITS_MSK(11);
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR,
			cur_level + 0x60, cur_level + 0x60, 1);

	/* for TM_TF_COEF */
	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR_2, cur_level + 0x10, 2);
	if (save_coef)
		hdr_tf_coef[2] = payload[31] & BITS_MSK(11);
	payload[31] = tf_coef[2] & BITS_MSK(11);
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR_2,
			cur_level + 0x10, cur_level + 0x10, 1);

	/* for NOISE_TF_COEF and FLESH_TF_COEF  */
	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR_2, cur_level + 0x20, 2);
	reg_val = payload[1];
	if (save_coef) {
		hdr_tf_coef[3] = BITS_GET(payload[1], 11, 19);
		hdr_tf_coef[4] = payload[9] & BITS_MSK(11);
	}
	payload[1] = BITS_SET(reg_val, 11, 19, tf_coef[3]);
	payload[9] = tf_coef[4] & BITS_MSK(11);
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR_2,
			cur_level + 0x20, cur_level + 0x20, 0x01);

	iris_end_hdr(true);
}

void iris_sdr2hdr_set_ftc(u32 value)
{
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	u32 option = 0x20 + pqlt_cur_setting->pq_setting.sdr2hdr;

	if (!iris_sdr2hdr_valid(pqlt_cur_setting->pq_setting.sdr2hdr))
		return;

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR_2, option, 4);
	payload[0] = (payload[0] & 0x00ffffff) + (iris_ftc_level[value] << 24);
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR_2, option, option, 0x01);

	iris_end_hdr(true);

	IRIS_LOGI("%s(), value: %u", __func__, value);
}

void iris_sdr2hdr_set_degain(void)
{
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	u32 option = 0xC0 + pqlt_cur_setting->pq_setting.sdr2hdr;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!iris_sdr2hdr_valid(pqlt_cur_setting->pq_setting.sdr2hdr))
		return;


	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 2);
	if (pcfg->pt_sr_enable == false) {
		payload[0] = iris_de_default[pqlt_cur_setting->pq_setting.sdr2hdr * 2];
		payload[9] = iris_de_default[pqlt_cur_setting->pq_setting.sdr2hdr * 2 + 1];
	} else {
		payload[0] = iris_de_disable[0];
		payload[9] = iris_de_disable[1];
	}
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, option, option, 0x01);

	iris_end_hdr(true);

	IRIS_LOGD("%s()", __func__);
}

void iris_sdr2hdr_graphic_set(u32 value)
{
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	u32 option = pqlt_cur_setting->pq_setting.sdr2hdr;

	if (!iris_sdr2hdr_valid(pqlt_cur_setting->pq_setting.sdr2hdr))
		return;

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 2);
	payload[0] &= ~0x02000000;
	if (value != 0)
		payload[0] |= 0x02000000;
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, option, option, 0x01);

	iris_end_hdr(true);

	IRIS_LOGI("%s(), value: %u", __func__, value);
}

void iris_sdr2hdr_scurve_set(u32 value)
{
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	u32 option = pqlt_cur_setting->pq_setting.sdr2hdr;

	if (!iris_sdr2hdr_valid(pqlt_cur_setting->pq_setting.sdr2hdr))
		return;

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 2);
	payload[0] &= ~0x00000400;
	if (value != 0)
		payload[0] |= 0x00000400;
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, option, option, 0x01);

	iris_end_hdr(true);

	IRIS_LOGI("%s(), value: %u", __func__, value);
}

void iris_sdr2hdr_set_de_ftc(u32 value)
{
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	u32 option = 0xC0 + pqlt_cur_setting->pq_setting.sdr2hdr;

	if (!iris_sdr2hdr_valid(pqlt_cur_setting->pq_setting.sdr2hdr))
		return;

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 10);
	if (value > 0) {
		payload[0] = (payload[0] & 0x007fffff) + (iris_de_ftc_level[value - 1] << 23);
		payload[0] = (payload[0] & 0xffbfffff) + 0x00400000;
	} else {
		payload[0] &= 0xffbfffff;
	}
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, option, option, 0x01);

	iris_end_hdr(true);

	IRIS_LOGI("%s(), value: %u", __func__, value);
}

void iris_sdr2hdr_csc_switch(u32 value)
{
	uint32_t  *payload = NULL;
	int i;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	struct iris_cfg *pcfg = iris_get_cfg();
	uint32_t level;
	u32 pwil_csc = 0x40;
	u32 blending_csc = 0x40;

	if (pqlt_cur_setting->pq_setting.sdr2hdr < SDR2HDR_1)
		return;

	if (value == 1) {
		pwil_csc = 0x40;   //709
		blending_csc = 0x42;   //P3
	} else if (value == 2) {
		pwil_csc = 0x42;   //P3
		blending_csc = 0x42;   //P3
	} else {
		pwil_csc = 0x40;   //709
		blending_csc = 0x40;    //709
	}

	iris_update_ip_opt(IRIS_IP_PWIL, pwil_csc, 0x01);

	iris_update_ip_opt(IRIS_IP_BLEND, blending_csc, 0x01);

	level = 0x70 + pqlt_cur_setting->pq_setting.sdr2hdr;
	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, level, 7);
	for (i = 0; i < 12; i++) {
		payload[i] = iris_sdr2hdr_csc[i][value];
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, level, 7+i, payload[i]);
	}
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR,
			level, level, 1);

	level = 0x90 + pqlt_cur_setting->pq_setting.sdr2hdr;
	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, level, 2);
	for (i = 0; i < 24; i++) {
		payload[i] = iris_sdr2hdr_csc[12+i][value];
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, level, 2+i, payload[i]);
	}
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, level, level, 1);

	level = 0xA0 + pqlt_cur_setting->pq_setting.sdr2hdr;
	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, level, 2);
	for (i = 0; i < 21; i++) {
		payload[i] = iris_sdr2hdr_csc[36+i][value];
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, level, 2+i, payload[i]);
	}
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR,
			level, level, 0x01);

	if (!iris_skip_dma) {
		if (!iris_dynamic_power_get()) {
			// HDR + BLENDING + PWIL
			if (pcfg->frc_enabled || pcfg->pwil_mode == FRC_MODE)
				iris_dma_gen_ctrl(0xe, 7, 0);
			else
				iris_dma_gen_ctrl(0xe, 4, 0);
		} else
			// PWIL
			//iris_dma_gen_ctrl(0x4, 0, 0);
			iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe6, 0xe6, 0);
		iris_update_pq_opt(iris_pq_update_path, true);
	}

	IRIS_LOGI("%s(), value: %u", __func__, value);
}

void iris_sdr2hdr_ai_tm(u32 value)
{
	uint32_t  *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_AI, 0x20, 2);
	payload[0] = (payload[0] & 0xffff0000) | value;
	iris_init_update_ipopt_t(IRIS_IP_AI, 0x20, 0x20, 0x01);
	iris_init_update_ipopt_t(IRIS_IP_AI, 0x80, 0x80, iris_skip_dma);

	if (!iris_skip_dma)
		iris_update_pq_opt(iris_pq_update_path, true);

	IRIS_LOGI("%s(), value: %u", __func__, value);
}

void iris_sdr2hdr_ai_lce(u32 value)
{
	uint32_t  *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_AI, 0x20, 2);
	payload[0] = (payload[0] & 0x0000ffff) | (value << 16);
	iris_init_update_ipopt_t(IRIS_IP_AI, 0x20, 0x20, 0x01);
	iris_init_update_ipopt_t(IRIS_IP_AI, 0x80, 0x80, iris_skip_dma);

	if (!iris_skip_dma)
		iris_update_pq_opt(iris_pq_update_path, true);

	IRIS_LOGI("%s(), value: %u", __func__, value);
}

void iris_sdr2hdr_ai_de(u32 value)
{
	uint32_t  *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_AI, 0x20, 3);
	payload[0] = (payload[0] & 0xffff0000) | value;
	iris_init_update_ipopt_t(IRIS_IP_AI, 0x20, 0x20, 0x01);
	iris_init_update_ipopt_t(IRIS_IP_AI, 0x80, 0x80, iris_skip_dma);

	if (!iris_skip_dma)
		iris_update_pq_opt(iris_pq_update_path, true);

	IRIS_LOGI("%s(), value: %u", __func__, value);
}

void iris_sdr2hdr_ai_graphic(u32 value)
{
	uint32_t  *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_AI, 0x20, 3);
	payload[0] = (payload[0] & 0x0000ffff) | (value << 16);
	iris_init_update_ipopt_t(IRIS_IP_AI, 0x20, 0x20, 0x01);
	iris_init_update_ipopt_t(IRIS_IP_AI, 0x80, 0x80, iris_skip_dma);

	if (!iris_skip_dma)
		iris_update_pq_opt(iris_pq_update_path, true);

	IRIS_LOGI("%s(), value: %u", __func__, value);
}

void iris_hdr_ai_input_bl(u32 bl_value, bool update)
{
	uint32_t  *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR_2, 0x50, 4);
	payload[0] = bl_value;

	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR_2, 0x50, 0x50, 1);

	if (update)
		iris_end_hdr(true);
}

void iris_ai_bl_al_enable(u32 value, bool update)
{
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;

	switch (value) {
	case AI_AMBIENT_BACKLIGHT_DISABLE:
		if (pqlt_cur_setting->pq_setting.sdr2hdr >= 9)
			iris_lux_set(500, update);
		else
			iris_lux_set(pqlt_cur_setting->luxvalue, update);
		iris_hdr_ai_input_bl(250, update);
		break;
	case AI_AMBIENT_ENABLE:
		iris_lux_set(pqlt_cur_setting->luxvalue, update);
		iris_hdr_ai_input_bl(250, update);
		break;
	case AI_BACKLIGHT_ENABLE:
		if (pqlt_cur_setting->pq_setting.sdr2hdr >= 9)
			iris_lux_set(500, update);
		else
			iris_lux_set(pqlt_cur_setting->luxvalue, update);
		iris_hdr_ai_input_bl(pqlt_cur_setting->ai_backlight, update);
		break;
	case AI_AMBIENT_BACKLIGHT_ENABLE:
		iris_lux_set(pqlt_cur_setting->luxvalue, update);
		iris_hdr_ai_input_bl(pqlt_cur_setting->ai_backlight, update);
		break;
	default:
		break;
	}
}

void iris_panel_nits_set(u32 bl_ratio, bool bSystemRestore, int level)
{
#if 0
	char led_pwm1[3] = {MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0x0, 0x0};
	char hbm_data[2] = {MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x0};
	struct dsi_cmd_desc backlight_cmd = {
		{0, MIPI_DSI_DCS_LONG_WRITE, 0, 0, 0, sizeof(led_pwm1), led_pwm1, 0, NULL}, 1, 1};
	struct dsi_cmd_desc hbm_cmd = {
		{0, MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, 0, 0, sizeof(hbm_data), hbm_data, 0, NULL}, 1, 1};

	struct dsi_panel_cmd_set cmdset = {
		.state = DSI_CMD_SET_STATE_HS,
		.count = 1,
		.cmds = &backlight_cmd,
	};
	u32 bl_lvl;
	struct iris_cfg *pcfg = iris_get_cfg();

	/* Don't control panel's brightness when sdr2hdr mode is 3 */
	if (iris_sdr2hdr_mode == 3)
		return;

	if (bSystemRestore)
		bl_lvl = iris_setting.quality_cur.system_brightness;
	else
		bl_lvl = bl_ratio * pcfg->panel_dimming_brightness / PANEL_BL_MAX_RATIO;

	if (pcfg->panel->bl_config.bl_max_level > 255) {
		led_pwm1[1] = (unsigned char)(bl_lvl >> 8);
		led_pwm1[2] = (unsigned char)(bl_lvl & 0xff);
	} else {
		led_pwm1[1] = (unsigned char)(bl_lvl & 0xff);
		backlight_cmd.msg.tx_len = 2;
	}
	iris_pt_send_panel_cmd(pcfg->panel, &cmdset);

	// Support HBM for different panels.
	hbm_data[1] = (bSystemRestore) ? pcfg->panel_hbm[0] : pcfg->panel_hbm[1];
	cmdset.cmds = &hbm_cmd;
	if (pcfg->panel_hbm[0] != pcfg->panel_hbm[1])
		iris_pt_send_panel_cmd(pcfg->panel, &cmdset);
	IRIS_LOGD("panel_nits: bl_lvl=0x%x, hbm=0x%x, restore=%d", bl_lvl, hbm_data[1], bSystemRestore);
#endif
}


void iris_scaler_filter_update(u8 scaler_type, u32 level)
{
	uint8_t ip = IOINC1D_LUT;

	if (scaler_type == SCALER_INPUT)
		ip = IOINC1D_LUT;
	else if (scaler_type == SCALER_PP)
		ip = IOINC1D_PP_LUT;
	else if (scaler_type == SCALER_INPUT_SHARP)
		ip = IOINC1D_LUT_SHARP;
	else if (scaler_type == SCALER_PP_SHARP)
		ip = IOINC1D_PP_LUT_SHARP;
	else if (scaler_type == SCALER_INPUT_9TAP)
		ip = IOINC1D_LUT_9TAP;
	else if (scaler_type == SCALER_INPUT_SHARP_9TAP)
		ip = IOINC1D_LUT_SHARP_9TAP;

	iris_init_update_ipopt_t(ip, level, level, 0);

	iris_update_pq_opt(iris_pq_update_path, true);

	IRIS_LOGI("scaler filter scaler_type=%d, level=%d", scaler_type, level);
}

void iris_scaler_gamma_enable(u32 level)
{
	uint32_t  *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x00, 2);

	payload[0] &= ~0x00000040;
	payload[0] |= (level << 6);

	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x00, 0x00, 0x01);

	iris_end_dpp(true);
	IRIS_LOGI("gamma enable=%d", level);
}

/*
void iris_hdr_csc_prepare(void)
{
	int len;
	bool chain = 0;
	uint8_t path = iris_pq_update_path;

	if (iris_capture_ctrl_en == false) {
		IRIS_LOGD("iris csc prepare.");
		iris_capture_ctrl_en = true;
		if (!iris_dynamic_power_get() && !iris_skip_dma)
			chain = 1;

		len = iris_init_update_ipopt_t(IRIS_IP_PWIL,
				0x52, 0x52, chain);
		if (!iris_dynamic_power_get() && !iris_skip_dma)
			len = iris_init_update_ipopt_t(IRIS_IP_DMA,
					0xe2, 0xe2, 0);

		iris_update_pq_opt(path, true);
	}
}
*/

int iris_kickoff(void *phys_enc)
{
	struct sde_encoder_phys *phys_encoder = phys_enc;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct iris_cfg *pcfg;

	if (phys_encoder == NULL)
		return -EFAULT;
	if (phys_encoder->connector == NULL)
		return -EFAULT;

	c_conn = to_sde_connector(phys_encoder->connector);
	if (c_conn == NULL)
		return -EFAULT;

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI)
		return 0;

	display = c_conn->display;
	if (display == NULL)
		return -EFAULT;

	pcfg = iris_get_cfg();
	if (pcfg->fod_pending)
		iris_post_fod(pcfg->panel);
	if (pcfg->memc_info.vfr_en)
		iris_memc_vfr_video_update_monitor(pcfg, display);
	if (iris_virtual_display(display) || pcfg->valid < PARAM_PARSED)
		return 0;

	complete(&pcfg->frame_ready_completion);
	return 0;
}

void iris_hdr_csc_complete(int step)
{
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (step == 0 || step == 1 || step == 3 || step == 4) {
		IRIS_LOGD("AP csc prepare.");
		iris_HDR10 = shadow_iris_HDR10;
		iris_HDR10_YCoCg = shadow_iris_HDR10_YCoCg;
		iris_setting.quality_cur.dspp_dirty = 1;
		if (step == 4)
			return;
	} else if (step == 5 || step == 6) {
		struct iris_cfg *pcfg = iris_get_cfg();

		IRIS_LOGD("Wait frame ready.");
		reinit_completion(&pcfg->frame_ready_completion);
		if (!wait_for_completion_timeout(&pcfg->frame_ready_completion,
					msecs_to_jiffies(50)))
			IRIS_LOGE("%s: timeout waiting for frame ready", __func__);
	}

	IRIS_LOGD("AP csc complete.");
	if (pqlt_cur_setting->pq_setting.sdr2hdr > 0)
		iris_hdr_datapath_set();
	if (!iris_dynamic_power_get()) {
		// HDR + BLENDING + PWIL + DPP
		if (sdr2hdr_level_orig == 0 &&
			pqlt_cur_setting->pq_setting.sdr2hdr > 0) {
			if (pcfg->frc_enabled || pcfg->pwil_mode == FRC_MODE)
				iris_dma_gen_ctrl(0xd, 7, 1);
			else
				iris_dma_gen_ctrl(0xd, 5, 1);

			iris_hdr_power_set();
		} else {
			if (pcfg->frc_enabled || pcfg->pwil_mode == FRC_MODE)
				iris_dma_gen_ctrl(0xf, 7, 0);
			else
				iris_dma_gen_ctrl(0xf, 4, 0);
			iris_update_pq_opt(iris_pq_update_path, true);
		}
	} else {
		//PWIL + DPP
		//iris_dma_gen_ctrl(0x5, 0, 0, 0);
		//DPP
		//iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe9, 0xe9, 1);
		//PWIL
		//iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe6, 0xe6, 0);
		if (sdr2hdr_level_orig == 0 &&
			pqlt_cur_setting->pq_setting.sdr2hdr > 0) {
			iris_dma_trig(DMA_CH10 | DMA_CH12, 1);
			iris_hdr_power_set();
		} else {
			iris_dma_trig(DMA_CH10 | DMA_CH12 | DMA_CH11, 0);
			iris_update_pq_opt(iris_pq_update_path, true);
		}
	}

	IRIS_LOGD("iris csc complete.");
}

int32_t  iris_update_ip_opt(
		uint8_t ip, uint8_t opt_id, uint8_t chain)
{
	int i = 0;
	uint8_t old_opt;
	int32_t cnt = 0;

	struct iris_pq_ipopt_val *pq_ipopt_val = iris_get_cur_ipopt_val(ip);

	if (pq_ipopt_val == NULL) {
		IRIS_LOGI("can not get pq ipot val i_p = %02x, opt_id = %02x",
				ip, opt_id);
		return 1;
	}

	if (ip == DPP_3DLUT) {
		for (i = 0; i < pq_ipopt_val->opt_cnt; i++) {
			if ((opt_id % DPP_3DLUT_GROUP)
					== (pq_ipopt_val->popt[i]
						% DPP_3DLUT_GROUP)) {
				old_opt = pq_ipopt_val->popt[i];
				pq_ipopt_val->popt[i] = opt_id;
				cnt = iris_init_update_ipopt_t(ip, old_opt, opt_id, chain);
				return cnt;
			}
		}
	}

	if (ip == GAMMA_LUT) {
		old_opt  = pq_ipopt_val->popt[i];
		pq_ipopt_val->popt[i] = opt_id;
		cnt = iris_init_update_ipopt_t(ip, old_opt, opt_id, chain);
		return cnt;
	}

	for (i = 0; i < pq_ipopt_val->opt_cnt; i++) {
		if ((opt_id & 0xf0) == ((pq_ipopt_val->popt[i]) & 0xf0)) {
			old_opt  = pq_ipopt_val->popt[i];
			pq_ipopt_val->popt[i] = opt_id;
			cnt = iris_init_update_ipopt_t(ip, old_opt, opt_id, chain);
			return cnt;
		}
	}
	return 1;
}

void iris_psf_mif_efifo_set(u8 mode, bool osd_enable)
{
	bool enable = false;
	bool chain = 0;

	/* disable efifo */
	if (true)
		return;

	if (!iris_dynamic_power_get())
		chain = 1;
	if ((mode == PT_MODE) && (osd_enable == true))
		enable = true;
}

void iris_rx_meta_dma_list_send(u32 meta, bool commit)
{
	uint32_t *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_RX, 0x02, 3);
	payload[0] = meta;
	iris_init_update_ipopt_t(IRIS_IP_RX, 0x02, 0x02, 0);
	iris_update_pq_opt(iris_pq_update_path, commit);
	IRIS_LOGI("%s(), set pwil mode according to dma mode: %x", __func__, payload[0]);

}

void iris_dtg_frame_rate_set(void)
{
	iris_update_ip_opt(IRIS_IP_DTG, 0x0, 0x1);

	/* sync TE selection type, for First Line First Pixel on FRC */
	iris_update_ip_opt(IRIS_IP_DTG, ID_DTG_TE_SEL, 0x1);

	/* use SW update to avoid PI_PRELOAD mismatch when switching TE on FRC */
	//iris_update_ip_opt(IRIS_IP_DTG, 0xF0, 0x0);
	iris_update_ip_opt(IRIS_IP_DTG, 0xF9, 0x0);
	iris_update_pq_opt(iris_pq_update_path, true);
}

static ssize_t iris_pq_config_write(struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	unsigned long val;
	struct iris_cfg_log {
		uint8_t type;
		char *str;
	};

	struct iris_cfg_log arr[] = {
		{IRIS_DEMO_MODE, "IRIS_DEMO_MODE"},
		{IRIS_SDR2HDR, "IRIS_SDR2HDR"},
		{IRIS_DYNAMIC_POWER_CTRL, "IRIS_DYNAMIC_POWER_CTRL"},
		{IRIS_HDR_MAXCLL, "IRIS_HDR_MAXCLL"},
		{IRIS_ANALOG_BYPASS_MODE, "IRIS_ANALOG_BYPASS_MODE"},
		{IRIS_CM_COLOR_TEMP_MODE, "IRIS_CM_COLOR_TEMP_MODE"},
		{IRIS_CM_COLOR_GAMUT, "IRIS_CM_COLOR_GAMUT"},
		{IRIS_CM_COLOR_GAMUT_PRE, "IRIS_CM_COLOR_GAMUT_PRE"},
		{IRIS_CCT_VALUE, "IRIS_CCT_VALUE"},
		{IRIS_COLOR_TEMP_VALUE, "IRIS_COLOR_TEMP_VALUE"},
		{IRIS_AL_ENABLE, "IRIS_AL_ENABLE"},
		{IRIS_LUX_VALUE, "IRIS_LUX_VALUE"},
		{IRIS_READING_MODE, "IRIS_READING_MODE"},
		{IRIS_CHIP_VERSION, "IRIS_CHIP_VERSION"},
		{IRIS_PANEL_TYPE, "IRIS_PANEL_TYPE"},
	};
	u32 type;
	u32 value;
	int i = 0;
	uint32_t cfg_val = 0;
	int len = (sizeof(arr))/(sizeof(arr[0]));

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	type = (val & 0xffff0000) >> 16;
	value = (val & 0xffff);
	iris_configure(DSI_PRIMARY, type, value);

	for (i = 0; i < len; i++) {
		iris_configure_get(DSI_PRIMARY, arr[i].type, 1, &cfg_val);
		IRIS_LOGI("%s: %d", arr[i].str, cfg_val);
	}

	return count;
}

static const struct file_operations iris_pq_config_fops = {
	.open = simple_open,
	.write = iris_pq_config_write,
};


int iris_dbgfs_pq_init(struct dsi_display *display)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir("iris", NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("debugfs_create_dir for iris_debug failed, error %ld",
					PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}
	if (debugfs_create_file("iris_pq_config", 0644, pcfg->dbg_root, display,
				&iris_pq_config_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}
	return 0;
}

int iris_update_backlight(u32 bl_lvl)
{
	int rc = 0;
	struct iris_cfg *pcfg = NULL;
	struct dsi_panel *panel;
	char led_pwm1[3] = {MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0x0, 0x0};
	struct dsi_cmd_desc_pxlw backlight_cmd_pxlw = {
		{0, MIPI_DSI_DCS_LONG_WRITE, 0, 0, 0, sizeof(led_pwm1), led_pwm1, 0, NULL}, 1, 0};

	struct dsi_cmd_desc backlight_cmd;
	struct dsi_panel_cmd_set cmdset = {
		.state = DSI_CMD_SET_STATE_HS,
		.count = 1,
		.cmds = &backlight_cmd,
	};
	remap_to_qcom_style(&backlight_cmd, &backlight_cmd_pxlw, 1);

	pcfg = iris_get_cfg();
	panel = pcfg->panel;

	iris_setting.quality_cur.system_brightness = bl_lvl;

	if (panel->bl_config.bl_max_level > 255) {
		if (pcfg->switch_bl_endian) {
			led_pwm1[1] = (unsigned char)(bl_lvl >> 8);
			led_pwm1[2] = (unsigned char)(bl_lvl & 0xff);
		} else {
			led_pwm1[1] = (unsigned char)(bl_lvl & 0xff);
			led_pwm1[2] = (unsigned char)(bl_lvl >> 8);
		}
	} else {
		led_pwm1[1] = (unsigned char)(bl_lvl & 0xff);
		backlight_cmd.msg.tx_len = 2;
	}

	if (pcfg->abyp_ctrl.abypass_mode == PASS_THROUGH_MODE)
		rc = iris_pt_send_panel_cmd(panel, &cmdset);
	else
		rc = iris_abyp_send_panel_cmd(panel, &cmdset);

	if (!pcfg->switch_bl_endian)
		bl_lvl = (((bl_lvl & 0xff) << 8) | (bl_lvl >> 8));

	iris_setting.quality_cur.ai_backlight = ((u32)pcfg->panel_nits*bl_lvl)/panel->bl_config.bl_max_level;
	if (iris_setting.quality_cur.ai_auto_en == AI_BACKLIGHT_ENABLE
		|| iris_setting.quality_cur.ai_auto_en == AI_AMBIENT_BACKLIGHT_ENABLE)
		iris_hdr_ai_input_bl(iris_setting.quality_cur.ai_backlight, true);

	return rc;
}

int iris_update_backlight_value(u32 bl_lvl)
{
	int rc = 0;
	struct iris_cfg *pcfg = NULL;
	struct dsi_panel *panel;

	pcfg = iris_get_cfg();
	panel = pcfg->panel;

	if (!pcfg->switch_bl_endian)
		bl_lvl = (((bl_lvl & 0xff) << 8) | (bl_lvl >> 8));

	iris_setting.quality_cur.ai_backlight = ((u32)pcfg->panel_nits*bl_lvl)/panel->bl_config.bl_max_level;

	return rc;
}

void iris_dom_set(int mode)
{
	uint32_t  *payload = NULL;
	uint32_t dport_ctrl0;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->tx_mode == DSI_OP_VIDEO_MODE)
		return;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPORT, 0xF0, 2);
	dport_ctrl0 = payload[0];
	dport_ctrl0 &= ~0xc000;
	dport_ctrl0 |= (mode & 0x3) << 14;
	iris_set_ipopt_payload_data(IRIS_IP_DPORT, 0xF0, 2, dport_ctrl0);
	iris_update_ip_opt(IRIS_IP_DPORT, 0xF0, 0x01);

	iris_end_pq(true);
}

static void iris_brightness_para_set(uint32_t *values)
{
	struct iris_update_regval regval;
	uint32_t dimmingGain = 0;
	uint32_t  *payload = NULL;
	uint32_t dimmingEnable = values[4];

	if (values == NULL) {
		IRIS_LOGE("brightness value is empty");
		return;
	}

	IRIS_LOGD("dimming set: enable %d, values: 0x%x\n", dimmingEnable, values[1]);
	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0x60;    //DIM_CTRL/FD
	regval.mask = 0x00000001;
	regval.value = dimmingEnable?0x1:0x0;

	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x60, 0x60, 0x1);

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x61, 2);
	dimmingGain = ((values[1] << 16) | values[0]);
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x61, 2, dimmingGain);
	dimmingGain = values[2];
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x61, 3, dimmingGain);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x61, 0x61, 0x01);

}

void iris_csc2_para_set(uint32_t *values)
{
	uint32_t  *payload = NULL;
	int i, k;

	if (values == NULL) {
		IRIS_LOGE("brightness value is empty");
		return;
	}

	//pre csc
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x34, 2);
	for (i = 0; i < 5; i++)
		payload[i] = values[i];
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x34, 0x34, 0x01);
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0xd0, 2);
	for (i = 0; i < 3; i++)
		payload[i] = values[i + 5];
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0xd0, 0xd0, 0x01);

	for (i = 0; i < 9; i++) {
		if (i%2 == 0)
			dwCSC2CoffBuffer[i] = values[i/2] & 0xffff;
		else
			dwCSC2CoffBuffer[i] = (values[i/2] & 0xffff0000) >> 16;
		if (i/3 == 0)
			k = 1;
		else if (i/3 == 1)
			k = 5;
		else
			k = 6;
		if (dwCSC2CoffBuffer[i] > 0x4000) {
			dwCSC2CoffValue[i] = 0x8000 - dwCSC2CoffBuffer[i];
			dwCSC2CoffValue[i] = (dwCSC2CoffValue[i] * nCSCCoffValue[k]) / 0x800;
			dwCSC2CoffValue[i] = 0x8000 - dwCSC2CoffValue[i];
		} else {
			dwCSC2CoffValue[i] = (dwCSC2CoffBuffer[i] * nCSCCoffValue[k]) / 0x800;
		}
	}
	for (i = 9; i < 12; i++) {
		dwCSC2CoffBuffer[i] = values[i-4];
		if (i == 9)
			k = 6;
		else if (i == 10)
			k = 1;
		else
			k = 5;
		if (dwCSC2CoffBuffer[i] > 0x80000) {
			dwCSC2CoffValue[i] = 0x100000 - dwCSC2CoffBuffer[i];
			dwCSC2CoffValue[i] = (dwCSC2CoffValue[i] * nCSCCoffValue[k]) / 0x800;
			if (dwCSC2CoffValue[i] >= 0x80000)
				dwCSC2CoffValue[i] = 0x7ffff;
			else
				dwCSC2CoffValue[i] = 0x100000 - dwCSC2CoffValue[i];
		} else {
			dwCSC2CoffValue[i] = (dwCSC2CoffBuffer[i] * nCSCCoffValue[k]) / 0x800;
			if (dwCSC2CoffValue[i] >= 0x80000)
				dwCSC2CoffValue[i] = 0x7ffff;
		}
	}
	if (dpp_precsc_enable == false) {
		values[0] = dwCSC2CoffValue[1] << 16 | dwCSC2CoffValue[0];
		values[1] = dwCSC2CoffValue[3] << 16 | dwCSC2CoffValue[2];
		values[2] = dwCSC2CoffValue[5] << 16 | dwCSC2CoffValue[4];
		values[3] = dwCSC2CoffValue[7] << 16 | dwCSC2CoffValue[6];
		values[4] = 0x0000 << 16 | dwCSC2CoffValue[8];

		payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x33, 2);
		for (i = 0; i < 5; i++)
			payload[i] = values[i];
		iris_init_update_ipopt_t(IRIS_IP_DPP, 0x33, 0x33, 0x01);
		payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0xc0, 2);
		for (i = 0; i < 3; i++)
			payload[i] = dwCSC2CoffValue[i + 9];
		iris_init_update_ipopt_t(IRIS_IP_DPP, 0xc0, 0xc0, 0x01);
	}
	iris_end_dpp(true);
}

void iris_csc2_para_reset(void)
{
	uint32_t  *payload = NULL;
	int i;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x33, 2);
	for (i = 0; i < 5; i++)
		payload[i] = dwCSCCoffBuffer[i];
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x33, 0x33, 0x01);
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0xc0, 2);
	for (i = 0; i < 3; i++)
		payload[i] = dwCSCCoffBuffer[i + 5];
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0xc0, 0xc0, 0x01);

	//pre csc
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x34, 2);
	for (i = 0; i < 5; i++)
		payload[i] = dwCSCCoffBuffer[i];
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x34, 0x34, 0x01);
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0xd0, 2);
	for (i = 0; i < 3; i++)
		payload[i] = dwCSCCoffBuffer[i + 5];
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0xd0, 0xd0, 0x01);
	iris_end_dpp(true);
}

void iris_csc_para_set(uint32_t *values)
{
	uint32_t  *payload = NULL;
	int i;

	if (values == NULL) {
		IRIS_LOGE("brightness value is empty");
		return;
	}
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x40, 3);
	for (i = 0; i < 8; i++)
		payload[i] = values[i];
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x40, 0x40, 0x01);
	iris_end_dpp(true);
}

void iris_csc_para_reset(void)
{
	uint32_t  *payload = NULL;
	int i;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x40, 3);
	for (i = 0; i < 8; i++)
		payload[i] = dwCSCCoffBuffer[i];
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x40, 0x40, 0x01);
	iris_end_dpp(true);
}

void iris_brightness_level_set(u32 *value)
{
	iris_brightness_para_set(value);
	iris_end_dpp(true);
}

int32_t iris_parse_color_temp_range(struct device_node *np, struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return -EINVAL;

	/* 2500K~7500K for default */
	pcfg->min_color_temp = 2500;
	pcfg->max_color_temp = 7500;

	rc = p_dts_ops->read_u32(np, "pxlw,min-color-temp", &(pcfg->min_color_temp));
	if (rc) {
		IRIS_LOGE("can not get property: pxlw,min-color-temp");
		return rc;
	}
	IRIS_LOGI("pxlw,min-color-temp: %d", pcfg->min_color_temp);

	rc = p_dts_ops->read_u32(np, "pxlw,max-color-temp", &(pcfg->max_color_temp));
	if (rc) {
		IRIS_LOGE("can not get property:pxlw,max-color-temp");
		return rc;
	}
	IRIS_LOGI("pxlw,max-color-temp: %d", pcfg->max_color_temp);

	return rc;
}

static int32_t _iris_count_ip(const uint8_t *data, int32_t len, int32_t *pval)
{
	int tmp = 0;
	int i = 0;
	int j = 0;

	if (data == NULL || len == 0 || pval == NULL) {
		IRIS_LOGE("%s(), invalid data or pval or len", __func__);
		return -EINVAL;
	}

	tmp = data[0];
	len = len >> 1;

	for (i = 0; i < len; i++) {
		if (tmp == data[2 * i]) {
			pval[j]++;
		} else {
			tmp = data[2 * i];
			j++;
			pval[j]++;
		}
	}

	/*j begin from 0*/
	return j + 1;
}

static int32_t _iris_alloc_pq_init_space(struct iris_cfg *pcfg,
		const uint8_t *pdata, int32_t item_cnt)
{
	int32_t i = 0;
	int32_t size = 0;
	int32_t ip_cnt = 0;
	int32_t rc = 0;
	int32_t *ptr = NULL;
	struct iris_pq_init_val *pinit_val = &pcfg->pq_init_val;

	if (pdata == NULL || item_cnt == 0) {
		IRIS_LOGE("%s(), invalid input, data: %p, size: %d", pdata, item_cnt);
		return -EINVAL;
	}

	size = sizeof(*ptr) * (item_cnt >> 1);
	ptr = vmalloc(size);
	if (ptr == NULL) {
		IRIS_LOGE("can not malloc space for ptr");
		return -EINVAL;
	}
	memset(ptr, 0x00, size);

	ip_cnt = _iris_count_ip(pdata, item_cnt, ptr);
	if (ip_cnt <= 0) {
		IRIS_LOGE("can not static i_p option");
		rc = -EINVAL;
		goto EXIT_FREE;
	}

	pinit_val->ip_cnt = ip_cnt;
	size = sizeof(struct iris_pq_ipopt_val) * ip_cnt;
	pinit_val->val = vmalloc(size);
	if (pinit_val->val == NULL) {
		IRIS_LOGE("can not malloc pinit_val->val");
		rc = -EINVAL;
		goto EXIT_FREE;
	}

	for (i = 0; i < ip_cnt; i++) {
		pinit_val->val[i].opt_cnt = ptr[i];
		size = sizeof(uint8_t) * ptr[i];
		pinit_val->val[i].popt = vmalloc(size);
	}

EXIT_FREE:
	vfree(ptr);
	ptr = NULL;

	return rc;
}
int32_t iris_parse_default_pq_param(struct device_node *np,
		struct iris_cfg *pcfg)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t k = 0;
	int32_t item_cnt = 0;
	int32_t rc = 0;
	const uint8_t *key = "pxlw,iris-pq-default-val";
	const uint8_t *pdata = NULL;
	struct iris_pq_init_val *pinit_val = &pcfg->pq_init_val;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return -EINVAL;

	pdata = p_dts_ops->get_property(np, key, &item_cnt);
	if (!pdata) {
		IRIS_LOGE("%s pxlw,iris-pq-default-val fail", __func__);
		return -EINVAL;
	}

	rc = _iris_alloc_pq_init_space(pcfg, pdata, item_cnt);
	if (rc) {
		IRIS_LOGE("malloc error");
		return rc;
	}

	for (i = 0; i < pinit_val->ip_cnt; i++) {
		struct iris_pq_ipopt_val *pval = &(pinit_val->val[i]);

		pval->ip = pdata[k++];
		for (j = 0; j < pval->opt_cnt; j++) {
			pval->popt[j] = pdata[k];
			k += 2;
		}
		/*need to skip one*/
		k -= 1;
	}

	if (IRIS_IF_LOGV()) {
		IRIS_LOGE("ip_cnt = %0x", pinit_val->ip_cnt);
		for (i = 0; i < pinit_val->ip_cnt; i++) {
			char ptr[256];
			int32_t len = 0;
			int32_t sum = 256;
			struct iris_pq_ipopt_val *pval = &(pinit_val->val[i]);

			snprintf(ptr, sum, "i_p is %0x opt is ", pval->ip);
			for (j = 0; j < pval->opt_cnt; j++) {
				len = strlen(ptr);
				sum -= len;
				snprintf(ptr + len, sum, "%0x ", pval->popt[j]);
			}
			IRIS_LOGE("%s", ptr);
		}
	}

	return rc;
}

void iris_init_tm_points_lut(void)
{
	iris_tm_points_lut.lut_lutx_payload = &lut_x;
	iris_tm_points_lut.lut_luty_payload = &lut_y;
	iris_tm_points_lut.lut_luttm_payload = &lut_tm;
	iris_tm_points_lut.lut_lutcsc_payload = &lut_csc;
	iris_tm_points_lut.lut_lutratio_payload = &lut_ratio;

	iris_demura_lut.lut_swpayload = &lut_demura_sw;
	iris_demura_xy.lut_xypayload = &lut_demura_xy;
}

void iris_update_tm_lut(void)
{
	uint32_t  *payload = NULL;
	u32 option = 0x02;
	u32 option2 = 0x12;
	u32 option3 = 0x92;
	u32 option4 = 0xb2;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	int i;

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR_2, option, 2);
	for (i = 0; i < 5; i++)
		payload[1+i] = lut_tm[i];
	for (i = 0; i < 7; i++)
		payload[6+i] = lut_x[2*i] + (lut_x[2*i+1] << 16);
	payload[13] = lut_x[14];
	for (i = 0; i < 7; i++)
		payload[14+i] = lut_y[2*i] + (lut_y[2*i+1] << 16);
	payload[21] = lut_y[14];

	for (i = 0; i < 22; i++)
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR_2, option, 2+i, payload[i]);

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, option2, 2);
	for (i = 0; i < 2; i++)
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, option2, 2+i, lut_ratio[i]);

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, option3, 2);
	for (i = 0; i < 24; i++)
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, option3, 2+i, lut_csc[i]);

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, option4, 2);
	payload[0] = (payload[0] & ~0x00300000) | (lut_csc[24] << 20);
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, option4, 2, payload[0]);

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 7);
	payload[0] = (payload[0] & 0xffffe000) | lut_ratio[2];

	if (pqlt_cur_setting->pq_setting.sdr2hdr != HDR10)
		return;


	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR_2, option, option, 0x01);
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, option2, option2, 0x01);
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, option3, option3, 0x01);
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, option4, option4, 0x01);
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, option, option, 0x01);

	iris_end_hdr(true);
	IRIS_LOGW("%s", __func__);
}

struct msmfb_iris_tm_points_info *iris_get_tm_points_info(void)
{
	return &iris_tm_points_lut;
}

struct msmfb_iris_demura_info *iris_get_demura_info(void)
{
	return &iris_demura_lut;
}

struct msmfb_iris_demura_xy *iris_get_demura_xy(void)
{
	return &iris_demura_xy;
}

void iris_update_demura_lut(void)
{
	uint32_t  *payload = NULL;
	u32 lut_index;
	struct iris_cfg *pcfg = iris_get_cfg();
	int i, j;
	u32 payload_size;
	u32 payload_num;
	u32 payload_last;

	payload_size = pcfg->split_pkt_size / 4;
	payload_num = 1683 / payload_size;
	payload_last = 1683 - payload_num * payload_size;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x90, 2);
	if ((payload[0] & 0x4000) == 0x4000) {
		payload[0] &= ~0x4000;
		lut_index = 0x20;
	} else {
		payload[0] |= 0x4000;
		lut_index = 0x30;
	}


	for (i = 0; i < payload_num; i++) {
		payload = iris_get_ipopt_payload_data(DPP_DEMURA_LUT, lut_index, payload_size * i + 2);
		for (j = 0; j < payload_size; j++)
			payload[j] = lut_demura_sw[payload_size * i + j];
	}
	if (payload_last > 0) {
		payload = iris_get_ipopt_payload_data(DPP_DEMURA_LUT, lut_index, payload_size * payload_num + 2);
		for (j = 0; j < payload_last; j++)
			payload[j] = lut_demura_sw[payload_size * payload_num + j];
	}

	iris_update_ip_opt(DPP_DEMURA_LUT, lut_index, 0x01);
	iris_update_ip_opt(IRIS_IP_DPP, 0x90, 0x01);

	iris_end_dpp(true);

	IRIS_LOGW("%s", __func__);
}

void iris_update_demura_xy_lut(void)
{
	uint32_t  *payload = NULL;
	u32 lut_index = 0x10;
	struct iris_cfg *pcfg = iris_get_cfg();
	int i, j;
	u32 payload_size;
	u32 payload_num;
	u32 payload_last;

	payload_size = pcfg->split_pkt_size / 4;
	payload_num = 2048 / payload_size;
	payload_last = 2048 - payload_num * payload_size;

	for (i = 0; i < payload_num; i++) {
		payload = iris_get_ipopt_payload_data(DPP_DEMURA_LUT, lut_index, payload_size * i + 2);
		for (j = 0; j < payload_size; j++)
			payload[j] = lut_demura_xy[payload_size * i + j];
	}
	if (payload_last > 0) {
		payload = iris_get_ipopt_payload_data(DPP_DEMURA_LUT, lut_index, payload_size * payload_num + 2);
		for (j = 0; j < payload_last; j++)
			payload[j] = lut_demura_xy[payload_size * payload_num + j];
	}

	iris_update_ip_opt(DPP_DEMURA_LUT, lut_index, 0x01);

	iris_end_dpp(true);

	IRIS_LOGW("%s", __func__);
}

void iris_demura_enable(bool enable)
{
	uint32_t  *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x90, 2);
	if (enable)
		payload[0] |= 0x01;
	else
		payload[0] &= ~0x01;

	iris_update_ip_opt(IRIS_IP_DPP, 0x90, 0x01);

	iris_end_dpp(true);

	IRIS_LOGW("%s", __func__);
}

void iris_pwil_dpp_en(bool dpp_en)
{
	uint32_t  *payload = NULL;
	u32 cmd[4];

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xF0, 4);
	if (dpp_en)
		payload[0] |= 0x10;
	else
		payload[0] &= ~0x10;
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xF0, 0xF0, 0x01);

	if (iris_dynamic_power_get()) {
		cmd[0] = IRIS_PWIL_ADDR + DATA_PATH_CTRL0;
		cmd[1] = payload[0];
		cmd[2] = IRIS_PWIL_ADDR + PWIL_REG_UPDATE;
		cmd[3] = 0x00000100;

		iris_ocp_write_mult_vals(4, cmd);
	}
	iris_dma_trig(DMA_CH12, 0);
	iris_update_pq_opt(iris_pq_update_path, true);

	IRIS_LOGD("%s, dpp_en = %d", __func__, dpp_en);
}

void iris_pwil_dport_mode(bool dport_mode)
{
	iris_pwil_dport_disable(dport_mode, 2);

	IRIS_LOGD("%s, dport_mode = %d", __func__, dport_mode);
}

void iris_sr_level_set(u32 mode, u32 guided_level, u32 dejaggy_level, u32 peaking_level, u32 DLTI_level)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (mode == FRC_MODE) {
		if (pcfg->memc_info.memc_mode == MEMC_SINGLE_GAME_ENABLE ||
			pcfg->memc_info.memc_mode == MEMC_DUAL_EXTMV_ENABLE) {
			pcfg->frcgame_pq_guided_level = guided_level;
			pcfg->frcgame_pq_dejaggy_level = dejaggy_level;
			pcfg->frcgame_pq_peaking_level = peaking_level;
			pcfg->frcgame_pq_DLTI_level = DLTI_level;
		} else {
			pcfg->frc_pq_guided_level = guided_level;
			pcfg->frc_pq_dejaggy_level = dejaggy_level;
			pcfg->frc_pq_peaking_level = peaking_level;
			pcfg->frc_pq_DLTI_level = DLTI_level;
		}
	} else {
		pcfg->pt_sr_guided_level = guided_level;
		pcfg->pt_sr_dejaggy_level = dejaggy_level;
		pcfg->pt_sr_peaking_level = peaking_level;
		pcfg->pt_sr_DLTI_level = DLTI_level;
	}
	dejaggy_level |= 0x10;
	peaking_level |= 0x20;
	DLTI_level |= 0x30;
	iris_init_update_ipopt_t(IRIS_IP_SR, guided_level, guided_level, 0x01);
	iris_init_update_ipopt_t(IRIS_IP_SR, dejaggy_level, dejaggy_level, 0x01);
	iris_init_update_ipopt_t(IRIS_IP_SR, peaking_level, peaking_level, 0x01);
	iris_init_update_ipopt_t(IRIS_IP_SR, DLTI_level, DLTI_level, 0x01);
	iris_dma_trig(DMA_CH2, 0);
	iris_update_pq_opt(iris_pq_update_path, true);

	IRIS_LOGI("%s, guided = %d, dejaggy = %d, peaking = %d, DLTI = %d",
		__func__, guided_level, dejaggy_level & 0x0f,
		peaking_level & 0x0f, DLTI_level & 0x0f);
}
