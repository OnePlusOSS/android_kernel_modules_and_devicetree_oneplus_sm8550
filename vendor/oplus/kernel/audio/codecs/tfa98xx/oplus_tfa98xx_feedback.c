/******************************************************************************
** File: - oplus_tfa98xx_feedback.cpp
**
** Copyright (C), 2022-2024, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**     Implementation of tfa98xx reg error or speaker r0 or f0 error feedback.
**
** Version: 1.0
** --------------------------- Revision History: ------------------------------
**      <author>                                       <date>                  <desc>
*******************************************************************************/

#define pr_fmt(fmt) "%s(): " fmt, __func__

#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "config.h"
#include "tfa98xx.h"
#include "tfa.h"
#include "tfa_internal.h"
#include "tfa98xx_ext.h"
#if IS_ENABLED(CONFIG_SND_SOC_MTK_AUDIO_DSP)
#include <mtk-dsp-common.h>
#endif

#include <soc/oplus/system/oplus_mm_kevent_fb.h>

#define CONFIG_MTK_PLATFORM

#define OPLUS_AUDIO_EVENTID_SMARTPA_ERR    10041
#define OPLUS_AUDIO_EVENTID_SPK_ERR        10042
#define OPLUS_AUDIO_EVENTID_LOW_PRESSURE   20005
#define ERROR_INFO_MAX_LEN                 32

#define REG_BITS  16
#define TFA9874_STATUS_NORMAL_VALUE    ((0x850F << REG_BITS) + 0x16)/*reg 0x13 high 16 bits and 0x10 low 16 bits*/
#define TFA9874_STATUS_CHECK_MASK      ((0x300 << REG_BITS) + 0x9C)/*reg 0x10 mask bit2~4, bit7, reg 0x13 mask bit8 , bit9 */
#define TFA9873_STATUS_NORMAL_VALUE    ((0x850F << REG_BITS) + 0x56) /*reg 0x13 high 16 bits and 0x10 low 16 bits*/
#define TFA9873_STATUS_CHECK_MASK      ((0x300 << REG_BITS) + 0x15C)/*reg 0x10 mask bit2~4, bit6, bit8, reg 0x13 mask bit8 , bit9*/

#define CHECK_SPK_DELAY_TIME    (6)/* seconds */
#define TFA_SET_CMD_READY_DELAY (1)/* seconds */

/* this enum order must consistent with ready command data, such as: tfaCmdStereoReady_LP */
enum {
	POS_R0 = 0,
	POS_TE,
	POS_F0,
	POS_AT,
	POS_NUM
};

enum {
	MONO = 0,
	LEFT = MONO,
	RIGHT,
	MAX_SPK_NUM
};

#define IS_DEV_COUNT_VALID(cnt) (((cnt) > MONO) && ((cnt) <= MAX_SPK_NUM))

/* SB35-->8.28*/
#define TFA_LIB_VER_SB35        0x81c0000
/* macro for parsing data which read from adsp */
#define TFA_CMD_LEN             3
#define TFA_DATA_BYTES          3
#define TFA_OFFSET_BASE         3
#define TFA_MAX_PARAM_NUM       (MAX_SPK_NUM * POS_NUM)
/* read result data len */
#define TFA_DATA_LEN            (TFA_OFFSET_BASE * 2 + TFA_MAX_PARAM_NUM * TFA_DATA_BYTES)

/*threshold for default min f0*/
#define TFA_SPK_F0_DEFAULT_MIN  (450)
/*threshold for atmospheric, 0.85bar*/
#define TFA_ATMOSPHERIC_MIN     (85)

/*bit0:check mono/left regs; bit1:check right regs; bit16:check speaker status*/
#define CHECK_MASK_MONO_REGS    (0x1)
#define CHECK_MASK_STEREO_REGS  (0x3)
#define CHECK_SPEAKER_MASKS     (0x100)

#define SetFdBuf(buf, arg, ...) \
	do { \
		int len = strlen(buf); \
		scnprintf(buf + len, sizeof(buf) - len - 1, arg, ##__VA_ARGS__); \
	} while (0)

#define PARAM_OFFSET(pos, dev_cnt, id) \
		(((pos) * (dev_cnt) + (id)) * TFA_DATA_BYTES + TFA_OFFSET_BASE)

#define GET_VALUE(pdata, offset)  ((pdata[offset] << 16) + (pdata[offset+1] << 8) + pdata[offset+2])

#define GET_R0_TE_F0(r0_array, te_array, f0_array, pdata, cnt, id) \
	do { \
		r0_array[id] = GET_VALUE(pdata, PARAM_OFFSET(POS_R0, cnt, id)) * 1000 / 0x10000; \
		te_array[id] = GET_VALUE(pdata, PARAM_OFFSET(POS_TE, cnt, id)) / 0x4000; \
		f0_array[id] = GET_VALUE(pdata, PARAM_OFFSET(POS_F0, cnt, id)); \
	}while (0)

#define SetErrStr(fd_buf, id, r0_array, te_array, f0_array, r0_min, r0_max, f0_min, flag) \
	do { \
		SetFdBuf(fd_buf, "TFA98xx SPK%u:r0=%u, f0=%u, te=%u;", id+1, r0_array[id], f0_array[id], te_array[id]); \
		if ((r0_array[id] < r0_min[id]) || (r0_array[id] > r0_max[id])) { \
			SetFdBuf(fd_buf, "R0 out of range(%u, %u);", r0_min[id], r0_max[id]); \
			flag = 1; \
		} \
		if (f0_array[id] < f0_min[id]) { \
			SetFdBuf(fd_buf, "F0 smaller than %u;", f0_min[id]); \
			flag = 1; \
		} \
	} while (0)

struct tfa_cmd_ready {
	int dev_cnt;
	uint32_t low_pr;
	uint32_t lib_new;
	uint32_t len;
	int8_t *pcmd;
};

struct check_status_err {
	int bit;
	uint32_t err_val;
	char info[ERROR_INFO_MAX_LEN];
};

struct oplus_tfa98xx_feedback {
	uint32_t chk_flag;
	int pa_cnt;
	int cmd_step;
	uint32_t low_pr;/*low pressure protection flag*/
	uint32_t lib_new;
	uint32_t r0_min[MAX_SPK_NUM];
	uint32_t r0_max[MAX_SPK_NUM];
	uint32_t f0_min[MAX_SPK_NUM];
	struct mutex *lock;
	ktime_t last_fb;
};

static struct oplus_tfa98xx_feedback tfa_fb = {
	.chk_flag = 0,
	.pa_cnt = 0,
	.cmd_step = 0,
	.low_pr = 0,
	.lib_new = 0xff,
	.r0_min = {0},
	.r0_max = {0},
	.f0_min = {0},
	.lock = NULL,
	.last_fb = 0
};

static char const *tfa98xx_check_feedback_text[] = {"Off", "On"};
static const struct soc_enum tfa98xx_check_feedback_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tfa98xx_check_feedback_text), tfa98xx_check_feedback_text);

static const struct check_status_err check_err_tfa9874[] = {
	/*register 0x10 check bits*/
	{2,             0, "OverTemperature"},
	{3,             1, "CurrentHigh"},
	{4,             0, "VbatLow"},
	{7,             1, "NoClock"},
	/*register 0x13 check bits*/
	{8 + REG_BITS,  0, "VbatHigh"},
	{9 + REG_BITS,  1, "Clipping"},
};

static const struct check_status_err check_err_tfa9873[] = {
	/*register 0x10 check bits*/
	{2,             0, "OverTemperature"},
	{3,             1, "CurrentHigh"},
	{4,             0, "VbatLow"},
	{6,             0, "UnstableClk"},
	{8,             1, "NoClock"},
	/*register 0x13 check bits*/
	{8 + REG_BITS,  0, "VbatHigh"},
	{9 + REG_BITS,  1, "Clipping"},
};

static const unsigned char fb_regs[] = {0x00, 0x01, 0x02, 0x04, 0x05, 0x11, 0x14, 0x15, 0x16};

static int8_t tfaCmdLibVer[] = {
	0x00, 0x80, 0xfe
};

static int8_t tfaCmdReady_1[] = {
	0x00, 0x80, 0x0b, 0x00, 0x00, 0x03,
	0x22, 0x00, 0x00,/*for r0*/
	0x22, 0x00, 0x02,/*for te*/
	0x22, 0x00, 0x13/*for f0*/
};

static int8_t tfaCmdReady_1_LP[] = {
	0x00, 0x80, 0x0b, 0x00, 0x00, 0x04,
	0x22, 0x00, 0x00,/*for r0*/
	0x22, 0x00, 0x02,/*for te*/
	0x22, 0x00, 0x13,/*for f0*/
	0x22, 0x00, 0x04/*for atmospheric pressure*/
};

static int8_t tfaCmdReady_2[] = {
	0x00, 0x80, 0x0b, 0x00, 0x00, 0x06,
	0x22, 0x00, 0x00, 0x22, 0x00, 0x01,/*for r0*/
	0x22, 0x00, 0x02, 0x22, 0x00, 0x03,/*for te*/
	0x22, 0x00, 0x13, 0x22, 0x00, 0x14/*for f0*/
};

static int8_t tfaCmdReady_2_LP[] = {
	0x00, 0x80, 0x0b, 0x00, 0x00, 0x08,
	0x22, 0x00, 0x00, 0x22, 0x00, 0x01,/*for r0*/
	0x22, 0x00, 0x02, 0x22, 0x00, 0x03,/*for te*/
	0x22, 0x00, 0x13, 0x22, 0x00, 0x14,/*for f0*/
	0x22, 0x00, 0x04, 0x22, 0x00, 0x05/*for atmospheric pressure*/
};

static int8_t tfaCmdReady_1_SB40[] = {
	0x00, 0x80, 0x0b, 0x00, 0x00, 0x03,
	0x22, 0x00, 0x00,/*for r0*/
	0x22, 0x00, 0x02,/*for te*/
	0x22, 0x00, 0x1b/*for f0*/
};

static int8_t tfaCmdReady_1_LP_SB40[] = {
	0x00, 0x80, 0x0b, 0x00, 0x00, 0x04,
	0x22, 0x00, 0x00,/*for r0*/
	0x22, 0x00, 0x02,/*for te*/
	0x22, 0x00, 0x1b,/*for f0*/
	0x22, 0x00, 0x04/*for atmospheric pressure*/
};

static int8_t tfaCmdReady_2_SB40[] = {
	0x00, 0x80, 0x0b, 0x00, 0x00, 0x06,
	0x22, 0x00, 0x00, 0x22, 0x00, 0x01,/*for r0*/
	0x22, 0x00, 0x02, 0x22, 0x00, 0x03,/*for te*/
	0x22, 0x00, 0x1b, 0x22, 0x00, 0x1c/*for f0*/
};

static int8_t tfaCmdReady_2_LP_SB40[] = {
	0x00, 0x80, 0x0b, 0x00, 0x00, 0x08,
	0x22, 0x00, 0x00, 0x22, 0x00, 0x01,/*for r0*/
	0x22, 0x00, 0x02, 0x22, 0x00, 0x03,/*for te*/
	0x22, 0x00, 0x1b, 0x22, 0x00, 0x1c,/*for f0*/
	0x22, 0x00, 0x04, 0x22, 0x00, 0x05/*for atmospheric pressure*/
};

/*SB3.5 --> lib_ver=0x81c0000, SB4.0 --> lib_ver=0x9000301*/
static const struct tfa_cmd_ready gcmd_ready_table[] = {
	{1,  0,  0, sizeof(tfaCmdReady_1),           tfaCmdReady_1},
	{1,  1,  0, sizeof(tfaCmdReady_1_LP),        tfaCmdReady_1_LP},
	{2,  0,  0, sizeof(tfaCmdReady_2),           tfaCmdReady_2},
	{2,  1,  0, sizeof(tfaCmdReady_2_LP),        tfaCmdReady_2_LP},
	{1,  0,  1, sizeof(tfaCmdReady_1_SB40),     tfaCmdReady_1_SB40},
	{1,  1,  1, sizeof(tfaCmdReady_1_LP_SB40),  tfaCmdReady_1_LP_SB40},
	{2,  0,  1, sizeof(tfaCmdReady_2_SB40),     tfaCmdReady_2_SB40},
	{2,  1,  1, sizeof(tfaCmdReady_2_LP_SB40),  tfaCmdReady_2_LP_SB40},
};

static int8_t tfaCmdGet[] = {
	0x00, 0x80, 0x8b, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#ifdef CONFIG_MTK_PLATFORM
extern int tfa98xx_send_data_to_dsp(int8_t *buffer, int16_t DataLength);
extern int tfa98xx_receive_data_from_dsp(int8_t *buffer, int16_t size, uint32_t *DataLength);
#endif

inline bool is_param_valid(struct tfa98xx *tfa98xx)
{
	if ((tfa98xx == NULL) || (tfa98xx->tfa == NULL) || (tfa98xx->tfa98xx_wq == NULL)) {
		pr_err("input parameter is not available\n");
		return false;
	}

	if ((tfa98xx->pa_type != PA_TFA9874) && (tfa98xx->pa_type != PA_TFA9873)) {
		return false;
	}

	if ((tfa98xx->tfa->channel >= MAX_SPK_NUM) && (tfa98xx->tfa->channel != 0xff)) {
		pr_err("channel = %d error\n", tfa98xx->tfa->channel);
		return false;
	}

	return true;
}

static int tfa98xx_set_check_feedback(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	int need_chk = ucontrol->value.integer.value[0];

	/* only record true status, tfa_fb.chk_flag will set false in tfa98xx_mute*/
	if (need_chk) {
		if (tfa_fb.pa_cnt > 1) {
			tfa_fb.chk_flag = CHECK_SPEAKER_MASKS + CHECK_MASK_STEREO_REGS;
		} else {
			tfa_fb.chk_flag = CHECK_SPEAKER_MASKS + CHECK_MASK_MONO_REGS;
		}
	}
	pr_info("need_chk = %d, tfa_fb.chk_flag = 0x%x\n", need_chk, tfa_fb.chk_flag);

	return 1;
}

static int tfa98xx_get_check_feedback(struct snd_kcontrol *kcontrol,
						struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tfa_fb.chk_flag;
	pr_info("tfa_fb.chk_flag = 0x%x\n", tfa_fb.chk_flag);

	return 0;
}

const struct snd_kcontrol_new tfa98xx_check_feedback[] = {
	SOC_ENUM_EXT("TFA_CHECK_FEEDBACK", tfa98xx_check_feedback_enum,
		       tfa98xx_get_check_feedback, tfa98xx_set_check_feedback),
};

static int tfa98xx_check_status_reg(struct tfa98xx *tfa98xx)
{
	uint32_t reg_val;
	uint16_t reg10 = 0;
	uint16_t reg13 = 0;
	uint16_t reg_tmp = 0;
	int flag = 0;
	char fd_buf[MM_KEVENT_MAX_PAYLOAD_SIZE] = {0};
	char info[MM_KEVENT_MAX_PAYLOAD_SIZE] = {0};
	int offset = 0;
	enum Tfa98xx_Error err;
	int i;

	mutex_lock(tfa_fb.lock);
	/* check status register 0x10 value */
	err = tfa98xx_read_register16(tfa98xx->tfa, 0x10, &reg10);
	if (Tfa98xx_Error_Ok == err) {
		err = tfa98xx_read_register16(tfa98xx->tfa, 0x13, &reg13);
	}
	pr_info("read SPK%d status regs ret=%d, reg[0x10]=0x%x, reg[0x13]=0x%x", \
			tfa98xx->tfa->channel, err, reg10, reg13);

	if (Tfa98xx_Error_Ok == err) {
		reg_val = (reg13 << REG_BITS) + reg10;
		flag = 0;
		if ((tfa98xx->pa_type == PA_TFA9874) &&
				((TFA9874_STATUS_NORMAL_VALUE&TFA9874_STATUS_CHECK_MASK) != (reg_val&TFA9874_STATUS_CHECK_MASK))) {
			SetFdBuf(info, "TFA9874 SPK%d:reg[0x10]=0x%x,reg[0x13]=0x%x,", tfa98xx->tfa->channel, reg10, reg13);
			for (i = 0; i < ARRAY_SIZE(check_err_tfa9874); i++) {
				if (check_err_tfa9874[i].err_val == (1 & (reg_val >> check_err_tfa9874[i].bit))) {
					SetFdBuf(info, "%s,", check_err_tfa9874[i].info);
				}
			}
			flag = 1;
		} else if ((tfa98xx->pa_type == PA_TFA9873) &&
				((TFA9873_STATUS_NORMAL_VALUE&TFA9873_STATUS_CHECK_MASK) != (reg_val&TFA9873_STATUS_CHECK_MASK))) {
			SetFdBuf(info, "TFA9873 SPK%d:reg[0x10]=0x%x,reg[0x13]=0x%x,", tfa98xx->tfa->channel, reg10, reg13);
			for (i = 0; i < ARRAY_SIZE(check_err_tfa9873); i++) {
				if (check_err_tfa9873[i].err_val == (1 & (reg_val >> check_err_tfa9873[i].bit))) {
					SetFdBuf(info, "%s,", check_err_tfa9873[i].info);
				}
			}
			flag = 1;
		}

		/* read other registers */
		if (flag == 1) {
			SetFdBuf(info, "dump regs(");
			for (i = 0; i < sizeof(fb_regs); i++) {
				err = tfa98xx_read_register16(tfa98xx->tfa, fb_regs[i], &reg_tmp);
				if (Tfa98xx_Error_Ok == err) {
					SetFdBuf(info, "%x=%x,", fb_regs[i], reg_tmp);
				} else {
					break;
				}
			}
			SetFdBuf(info, "),");
		}
	} else {
		SetFdBuf(info, "%s SPK%d: failed to read regs 0x10 and 0x13, error=%d,", \
				(tfa98xx->pa_type == PA_TFA9873) ? "TFA9873" : "TFA9874", tfa98xx->tfa->channel, err);
		tfa_fb.last_fb = ktime_get();
	}
	mutex_unlock(tfa_fb.lock);

	/* feedback the check error */
	offset = strlen(info);
	if ((offset > 0) && (offset < MM_KEVENT_MAX_PAYLOAD_SIZE)) {
		scnprintf(fd_buf, sizeof(fd_buf) - 1, "payload@@%s", info);
		pr_err("fd_buf=%s\n", fd_buf);
		mm_fb_audio_kevent_named(OPLUS_AUDIO_EVENTID_SMARTPA_ERR,
				MM_FB_KEY_RATELIMIT_5MIN, fd_buf);
	}

	return 0;
}

static int tfa98xx_set_cmd_for_speaker_info(struct tfa98xx *tfa98xx, int8_t *pbuf, int16_t size)
{
	int err = -1;

	if ((CHECK_SPEAKER_MASKS & tfa_fb.chk_flag) == 0) {
		return -1;
	}

	if ((pbuf == NULL) || (size <= 0)) {
		pr_err("input parameter is not available\n");
		return -ENODEV;
	}

	if (tfa98xx->tfa->is_probus_device) {
		mutex_lock(&tfa98xx->dsp_lock);
#ifdef CONFIG_MTK_PLATFORM
		err = tfa98xx_send_data_to_dsp(pbuf, size);
#else
		err = send_tfa_cal_apr(pbuf, size, false);
#endif
		mdelay(5);
		mutex_unlock(&tfa98xx->dsp_lock);
	}

	if (err != 0) {
		pr_err("send data to adsp error, ret = %d\n", err);
		tfa_fb.last_fb = ktime_get();
	}

	return err;
}


static int tfa98xx_get_lib_version(struct tfa98xx *tfa98xx)
{
	int err = -1;
	unsigned char result[TFA_DATA_LEN] = {0};
	unsigned char lib_ver[4]= {0};
	unsigned int lib_num = 0;
#ifdef CONFIG_MTK_PLATFORM
	uint32_t DataLength = 0;
#endif

	if (tfa98xx->tfa->is_probus_device) {
		mutex_lock(&tfa98xx->dsp_lock);
#ifdef CONFIG_MTK_PLATFORM
		err = tfa98xx_send_data_to_dsp(tfaCmdLibVer, sizeof(tfaCmdLibVer));
		if (err == 0) {
			mdelay(5);
			err = tfa98xx_receive_data_from_dsp(result, sizeof(result), &DataLength);
		}
#else
		err = send_tfa_cal_apr(tfaCmdLibVer, sizeof(tfaCmdLibVer), false);
		if (err != 0) {
			mdelay(5);
			err = send_tfa_cal_apr(result, sizeof(result), true);
		}
#endif
		mutex_unlock(&tfa98xx->dsp_lock);
	}

	if (err != 0) {
		pr_err("send data to adsp error, ret = %d\n", err);
		tfa_fb.last_fb = ktime_get();
		return err;
	}

	/* Split 3rd byte into two seperate ITF version fields (3rd field and 4th field) */
	lib_ver[0] = (result[0]);
	lib_ver[1] = (result[1]);
	if ((lib_ver[0] != 2) && (lib_ver[1] >= 33)) {
		lib_ver[3] = (result[2]) & 0x07;
		lib_ver[2] = (result[2] >> 3) & 0x1F;
	} else {
		lib_ver[3] = (result[2]) & 0x3f;
		lib_ver[2] = (result[2] >> 6) & 0x03;
	}
	pr_info("tfa lib version is %d.%d.%d.%d", lib_ver[0], lib_ver[1], lib_ver[2], lib_ver[3]);

	lib_num = (lib_ver[0] << 24) + (lib_ver[1] << 16) + (lib_ver[2] << 8) + lib_ver[3];
	tfa_fb.lib_new = (lib_num > TFA_LIB_VER_SB35) ? 1 : 0;

	return err;
}

static int tfa98xx_check_speaker_status(struct tfa98xx *tfa98xx)
{
	char fd_buf[MM_KEVENT_MAX_PAYLOAD_SIZE] = {0};
	char lp_buf[MM_KEVENT_MAX_PAYLOAD_SIZE] = {0};
	unsigned char result[TFA_DATA_LEN] = {0};
	uint32_t r0_array[MAX_SPK_NUM];
	uint32_t te_array[MAX_SPK_NUM];
	uint32_t f0_array[MAX_SPK_NUM];
	uint32_t at_array[MAX_SPK_NUM];
#ifdef CONFIG_MTK_PLATFORM
	uint32_t DataLength = 0;
#endif
	int spk_err = 0;
	int low_pressure = 0;
	int err = Tfa98xx_Error_Ok;
	int lp = 0;

	if ((CHECK_SPEAKER_MASKS & tfa_fb.chk_flag) == 0) {
		return 0;
	}
	if (!IS_DEV_COUNT_VALID(tfa_fb.pa_cnt)) {
		return -1;
	}

	if (tfa98xx->tfa->is_probus_device) {
		mutex_lock(&tfa98xx->dsp_lock);
#ifdef CONFIG_MTK_PLATFORM
		err = tfa98xx_receive_data_from_dsp(
			result, sizeof(result), &DataLength);
#else
		err = send_tfa_cal_apr(result, sizeof(result), true);
#endif
		mutex_unlock(&tfa98xx->dsp_lock);
	}

	/* pr_info("ret=%d, get value=%d\n", err, result[2]); */

	if (err != Tfa98xx_Error_Ok) {
		pr_err("read data from adsp error, ret = %d\n", err);
		tfa_fb.last_fb = ktime_get();
		return err;
	}

	/* check result value, feedback info if out of range*/
	SetFdBuf(fd_buf, "payload@@");
	for (lp = 0; lp < tfa_fb.pa_cnt; lp++) {
		/*get r0&te&f0 result value and convert to real value*/
		GET_R0_TE_F0(r0_array, te_array, f0_array, result, tfa_fb.pa_cnt, lp);
		SetErrStr(fd_buf, lp, r0_array, te_array, f0_array, \
				tfa_fb.r0_min, tfa_fb.r0_max, tfa_fb.f0_min, spk_err);
		pr_debug("SPK%d: r0 = %u, te = %u, f0 = %u", \
				lp, r0_array[lp], te_array[lp], f0_array[lp]);

		/*get pressure value and multiply by 100*/
		if ((tfa_fb.low_pr == 1) && (low_pressure != 1)) {
			at_array[lp] = GET_VALUE(result, PARAM_OFFSET(POS_AT, tfa_fb.pa_cnt, lp)) * 100 / 0x400000;
			if (at_array[lp] < TFA_ATMOSPHERIC_MIN) {
				SetFdBuf(lp_buf, "pressure@@%u", at_array[lp]);
				low_pressure = 1;
			}
			pr_debug("SPK%d: atmospheric pressure is %u/100", lp, at_array[lp]);
		}
	}

	if ((spk_err == 1) && (CHECK_SPEAKER_MASKS & tfa_fb.chk_flag) != 0) {
		mm_fb_audio_kevent_named(OPLUS_AUDIO_EVENTID_SMARTPA_ERR,
				MM_FB_KEY_RATELIMIT_5MIN, fd_buf);
	}

	if ((low_pressure == 1) && (CHECK_SPEAKER_MASKS & tfa_fb.chk_flag) != 0) {
		mm_fb_audio_kevent_named(OPLUS_AUDIO_EVENTID_LOW_PRESSURE,
				MM_FB_KEY_RATELIMIT_30MIN, lp_buf);
	}

	return err;
}

static void tfa98xx_check_work(struct work_struct *work)
{
	struct tfa98xx *tfa98xx = container_of(work, struct tfa98xx, check_work.work);
	int lp = 0;
	int ret = -1;

	if ((CHECK_SPEAKER_MASKS & tfa_fb.chk_flag) == 0) {
		return;
	}

	if ((tfa_fb.last_fb != 0)  && ktime_before(ktime_get(), ktime_add_ms(tfa_fb.last_fb, MM_FB_KEY_RATELIMIT_5MIN))) {
		return;
	}

	if (!is_param_valid(tfa98xx)) {
		pr_err("parameter is not available\n");
		return;
	}

	pr_info("chk_flag = 0x%x, cmd_step = %d, pa_cnt = %d, low_pr = %d, lib_new = %d\n", \
			tfa_fb.chk_flag, tfa_fb.cmd_step, tfa_fb.pa_cnt, tfa_fb.low_pr, tfa_fb.lib_new);

	switch (tfa_fb.cmd_step) {
	case 0:
		if ((tfa_fb.lib_new != 0) && (tfa_fb.lib_new != 1)) {
			tfa98xx_get_lib_version(tfa98xx);
		}

		for (lp = 0; lp < sizeof(gcmd_ready_table) / sizeof(struct tfa_cmd_ready); lp++) {
			if ((gcmd_ready_table[lp].dev_cnt == tfa_fb.pa_cnt) && \
					(gcmd_ready_table[lp].low_pr == tfa_fb.low_pr) && \
					(gcmd_ready_table[lp].lib_new == tfa_fb.lib_new)) {
				ret = tfa98xx_set_cmd_for_speaker_info(tfa98xx, \
						gcmd_ready_table[lp].pcmd , gcmd_ready_table[lp].len);
				break;
			}
		}
		if (ret == 0) {
			tfa_fb.cmd_step = 1;
			queue_delayed_work(tfa98xx->tfa98xx_wq, &tfa98xx->check_work, \
					TFA_SET_CMD_READY_DELAY * HZ);
		} else {
			tfa_fb.chk_flag = (~CHECK_SPEAKER_MASKS) & tfa_fb.chk_flag;
			tfa_fb.cmd_step = 0;
		}
		break;
	case 1:
		ret = tfa98xx_set_cmd_for_speaker_info(tfa98xx, tfaCmdGet, sizeof(tfaCmdGet));
		if (ret == 0) {
			tfa98xx_check_speaker_status(tfa98xx);
		}
		tfa_fb.chk_flag = (~CHECK_SPEAKER_MASKS) & tfa_fb.chk_flag;
		tfa_fb.cmd_step = 0;
		break;
	default:
		break;
	}
}

void oplus_tfa98xx_check_reg(struct tfa98xx *tfa98xx)
{
	uint32_t channel = 0;

	if (tfa_fb.chk_flag == 0) {
		return;
	}

	if (!is_param_valid(tfa98xx)) {
		pr_err("parameter is not available\n");
		return;
	}

	if (NULL == tfa_fb.lock) {
		return;
	}

	channel = tfa98xx->tfa->channel;
	if (((1 << channel) & tfa_fb.chk_flag) == 0) {
		return;
	}

	if ((tfa_fb.last_fb != 0)  && ktime_before(ktime_get(), ktime_add_ms(tfa_fb.last_fb, MM_FB_KEY_RATELIMIT_5MIN))) {
		return;
	}

	tfa98xx_check_status_reg(tfa98xx);
	tfa_fb.chk_flag = (~(1 << channel)) & tfa_fb.chk_flag;

	if (tfa98xx->check_work.wq) {
		cancel_delayed_work_sync(&tfa98xx->check_work);
		tfa_fb.chk_flag = (~CHECK_SPEAKER_MASKS) & tfa_fb.chk_flag;
		tfa_fb.cmd_step = 0;
	}
}

void oplus_tfa98xx_queue_check_work(struct tfa98xx *tfa98xx)
{
	if ((tfa_fb.chk_flag & CHECK_SPEAKER_MASKS) != 0) {
		if (!is_param_valid(tfa98xx)) {
			pr_err("parameter is not available\n");
			return;
		}

		if ((tfa_fb.last_fb != 0)  && ktime_before(ktime_get(), ktime_add_ms(tfa_fb.last_fb, MM_FB_KEY_RATELIMIT_5MIN))) {
			return;
		}

		if (tfa98xx && tfa98xx->tfa98xx_wq && tfa98xx->check_work.work.func) {
			queue_delayed_work(tfa98xx->tfa98xx_wq, &tfa98xx->check_work, CHECK_SPK_DELAY_TIME * HZ);
			tfa_fb.cmd_step = 0;
		}
	}
}

void oplus_tfa98xx_exit_check(struct tfa98xx *tfa98xx)
{
	pr_info("chk_flag = 0x%x, cmd_step = %d\n", tfa_fb.chk_flag, tfa_fb.cmd_step);

	if (tfa_fb.chk_flag) {
		tfa_fb.chk_flag = 0;

		if (tfa98xx && tfa98xx->check_work.wq) {
			cancel_delayed_work_sync(&tfa98xx->check_work);
			tfa_fb.chk_flag = (~CHECK_SPEAKER_MASKS) & tfa_fb.chk_flag;
			tfa_fb.cmd_step = 0;
		}
	}
}

void oplus_tfa98xx_get_dt(struct tfa98xx *tfa98xx, struct device_node *np)
{
	int ret = 0;
	uint32_t channel = 0;

	if (tfa98xx && tfa98xx->tfa && np) {
		if ((tfa98xx->tfa->channel < MAX_SPK_NUM) || (tfa98xx->tfa->channel == 0xff)) {
			channel = (tfa98xx->tfa->channel == 0xff) ? 0 : tfa98xx->tfa->channel;
			tfa_fb.r0_min[channel] = tfa98xx->tfa->min_mohms;
			tfa_fb.r0_max[channel] = tfa98xx->tfa->max_mohms;

			ret = of_property_read_u32(np, "tfa_min_f0", &tfa_fb.f0_min[channel]);
			if (ret) {
				pr_info("Failed to parse tfa_min_f0 node\n");
				tfa_fb.f0_min[channel] = TFA_SPK_F0_DEFAULT_MIN;
			}

			pr_info("spk%u r0 range (%u, %u), f0 min=%u\n", \
					channel, tfa_fb.r0_min[channel], tfa_fb.r0_max[channel], tfa_fb.f0_min[channel]);
		}

		if (tfa_fb.low_pr == 0) {
			ret = of_property_read_u32(np, "tfa_low_pressure", &tfa_fb.low_pr);
			if (ret) {
				pr_info("Failed to parse tfa_min_f0 node\n");
				tfa_fb.low_pr = 0;
			} else {
				pr_info("get dt tfa_pressure:%u\n", tfa_fb.low_pr);
			}
		}
	}
}

void oplus_tfa98xx_feedback_init(struct tfa98xx *tfa98xx, struct mutex *lock, int count)
{
	if (tfa98xx && tfa98xx->component && tfa98xx->tfa && lock) {
		if ((tfa98xx->tfa->channel == 0) || \
			    ((count == 1) && (tfa98xx->tfa->channel == 0xff))) {
			snd_soc_add_component_controls(tfa98xx->component,
					   tfa98xx_check_feedback,
					   ARRAY_SIZE(tfa98xx_check_feedback));

			INIT_DELAYED_WORK(&tfa98xx->check_work, tfa98xx_check_work);

			tfa_fb.lock = lock;
			if ((count > 0) && (count <= MAX_SPK_NUM)) {
				tfa_fb.pa_cnt = count;
			} else {
				tfa_fb.pa_cnt = 1;
			}
			pr_info("%s: success, pa_cnt = %d\n", tfa_fb.pa_cnt);
		} else {
			tfa98xx->check_work.work.func = NULL;
			tfa98xx->check_work.wq = NULL;
		}
	}
}

