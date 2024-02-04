/**
 * @brief		OIS system calibration parameters
 *
 * @author		(C) 2020 ON Semiconductor.
 *
 * @file		Calibration_Wide.h
 * @date		svn:$Date:: 2020-06-18 13:32:58 +0900#$
 * @version 	svn:$Revision: 6 $
 * @attention
 **/

//#define SEL_CLOSED_AF
#define	SEL_BALL_TYPE

//********************************************************************************
// defines
//********************************************************************************
#define XY_BIAS				(0x55)
#define X_OFSTO				(0xDF) 
#define Y_OFSTO				(0xB0) 
#ifdef SEL_CLOSED_AF
 #define AF_BIAS			(0x80000000 )
 #define AF_OFSTO			(0x80000000 ) 
 #define AF_OFSTI			(0x52000000 ) 
#endif

#define MARGIN				(0x0300 )		// Margin

/************ for linearity measurement step ***********/
#define SLT_OFFSET_X		(0x0FA0)
#define SLT_OFFSET_Y		(0x0FA0)
#define SLT_DIR_X			(+1)			// NVC move direction X
#define SLT_DIR_Y			(-1)			// NVC move direction Y

/************ Target amplitude ***********/
#define BIAS_ADJ_RANGE_X	(0x8CCC)		// 55%
#define BIAS_ADJ_RANGE_Y	(0x8CCC)		// 55%
#ifdef SEL_CLOSED_AF
 #define BIAS_ADJ_RANGE_Z	(0x9999)		//!< 60%
 #define HIGH_MARGIN_Z		(0x0000)		//!< 20d in BIAS_ADJ_RANGE_Z(INF) from 0mA
 #define LOW_MARGIN_Z		(0x051E)		//!< 5% in BIAS_ADJ_RANGE_Z(MACRO) from mecha edge
#endif

/* for OIS calibration */
#define SINE_OFFSET_OIS		0x001230B7		// Freq Setting = Freq * 80000000h / Fs(18k)	: 10Hz
#define SINE_GAIN_OIS		0x7FFFFFFF		// Set Sine Wave Gain
#define SINE_NUM_OIS		1801			// Fs / Freq

/* for Ball Type */
#define	MAX_CURRENT			0x7E000000
#define	MIN_CURRENT			0x82000000
#define	CURRENT_STEP		0x00800000
#define	STEP_WAIT_TIME		1
#define	MEASURE_NUM			128

/* for AF calibration */
#ifdef SEL_CLOSED_AF
 #define SINE_OFFSET_AF		0x000746AE		// Freq Setting = Freq * 80000000h / Fs(18k)	: 4Hz
//#define SINE_GAINP_AF		0x66666666		// Set Sine Wave plus Gain about 120mA
//#define SINE_GAINM_AF		0x26666666		// Set Sine Wave minus Gain about 45mA
 #define SINE_GAINP_AF		0x26666666		// Set Sine Wave plus Gain about 45mA
 #define SINE_GAINM_AF		0x66666666		// Set Sine Wave minus Gain about 120mA
 #define SINE_NUM_AF		4504			// Fs / Freq
#endif

#define DECRE_CAL			(0x0100)		// decrease value

/*********** Default loop gain ***********/
#define SXGAIN_LOP			(0x3FFFFFFF )	// 0.5
#define SYGAIN_LOP			(0x3FFFFFFF )	// 0.5
#ifdef SEL_CLOSED_AF
 #define SZGAIN_LOP			(0x30000000 )	// 0.375000
#endif

/* Loop gain adjustment frequency parameter (X) */
#define LOOP_NUM_HX			2402			// 18.014kHz/0.120kHz*16times
#define LOOP_FREQ_HX		0x00DA4897		// 120Hz  = Freq * 80000000h / Fs(18k)
#define LOOP_GAIN_HX		0x0337184F		// -32dB
#define GAIN_GAP_HX			(1000)			// 20*log(1000/1000)=0dB

/* Loop gain adjustment frequency parameter (Y) */
#define LOOP_NUM_HY			2620			// 18.014kHz/0.110kHz*16times
#define LOOP_FREQ_HY		0x00C817B4		// 110Hz  = Freq * 80000000h / Fs(18k)
#define LOOP_GAIN_HY		0x0337184F		// -32dB
#define GAIN_GAP_HY			(1000)			// 20*log(1000/1000)=0dB

/* Loop gain adjustment frequency parameter (Z) */
#ifdef SEL_CLOSED_AF
 #define LOOP_NUM_HZ		1601			// 18.014kHz/0.180kHz*16times	DVT
 #define LOOP_FREQ_HZ		0x01476CE2		// 180Hz  = Freq * 80000000h / Fs(18k)
 #define LOOP_GAIN_HZ		0x02492492		// -35dB
 #define GAIN_GAP_HZ		(1000)			// 20*log(1000/1000)=0dB
#endif

/* Loop gain pass max min range */
#define LOOP_MAX_X			(SXGAIN_LOP << 1)	// x2
#define LOOP_MIN_X			(SXGAIN_LOP >> 1)	// x0.5
#define LOOP_MAX_Y			(SYGAIN_LOP << 1)	// x2
#define LOOP_MIN_Y			(SYGAIN_LOP >> 1)	// x0.5
#ifdef SEL_CLOSED_AF
 #define LOOP_MAX_Z			(0x70000000)		// x2.5
 #define LOOP_MIN_Z			(0x10000000)		// x0.35
#endif

//********************************************************************************
// temperature compensation parameters : R3W11166
//********************************************************************************
#define TEMP_RCODEX			0x00000000
#define TEMP_RCODEY			0x00000000
#define TEMP_RCODEZ			0x7FFFFFFF
#define TEMP_SHAG			0x02C80000
#define TEMP_SHBG			0x037A0000
#define TEMP_SHCG			0x00000000
#define TEMP_SHOUTAG		0x00000000
#define TEMP_SHOUTBG		0x00000000
#define TEMP_SHAB			0x00003A90
#define TEMP_SHAC			0x7FFF8AE0
#define TEMP_SHAA			0x00003A90
#define TEMP_SHBB			0x0000012B
#define TEMP_SHBC			0x7FFFFDAA
#define TEMP_SHBA			0x0000012B
#define TEMP_SHCB			0x00000000
#define TEMP_SHCC			0x00000000
#define TEMP_SHCA			0x00000000
#define TEMP_TAB			0x00000CE5
#define TEMP_TAC			0x7FFFE635
#define TEMP_TAA			0x00000CE5
#define TEMP_TBB			0x0000520D
#define TEMP_TBC			0x7FFF5BE1
#define TEMP_TBA			0x0000520D
#define TEMP_TEMPOFF		0x25000000
#define TEMP_TAG			0x6BD00000
#define TEMP_TBG			0xD1200000
#define TEMP_SHIFTG			0x3FFFFFFF
#define TEMP_SHOUTAG1		0x00000000
#define TEMP_SHOUTBG1		0x00000000
#define TEMP_TCX			0x08
#define TEMP_TBX			0x00
#define TEMP_TAX			0x00

//********************************************************************************
// structure for calibration
//********************************************************************************
#ifdef __OISCMD__
const ADJ_HALL_LS LS_HallCalParameter[] = {
{
/****** P1 ACT=01 ******/
/* BiasInit */		XY_BIAS,
/* XOffsetInit */	X_OFSTO,
/* YOffsetInit */	Y_OFSTO,
/* Margin */		MARGIN,
/* XTargetRange */	BIAS_ADJ_RANGE_X,
/* XTargetMax */	(BIAS_ADJ_RANGE_X + MARGIN),
/* XTargetMin */	(BIAS_ADJ_RANGE_X - MARGIN),
/* YTargetRange */	BIAS_ADJ_RANGE_Y,
/* YTargetMax */	(BIAS_ADJ_RANGE_Y + MARGIN),
/* YTargetMin */	(BIAS_ADJ_RANGE_Y - MARGIN),
#ifdef SEL_BALL_TYPE
/* Max_Current */	MAX_CURRENT,
/* Min_Current */	MIN_CURRENT,
/* Current_Step */	CURRENT_STEP,
/* Step_Wait_Time */STEP_WAIT_TIME,
/* MeasureNum */	MEASURE_NUM,
#else
/* OisSinNum */		SINE_NUM_OIS,
/* OisSinFreq */	SINE_OFFSET_OIS,
/* OisSinGain */	SINE_GAIN_OIS,
#endif
#ifdef SEL_CLOSED_AF
/* AfSinNum */		SINE_NUM_AF,
/* AfSinFreq */		SINE_OFFSET_AF,
/* AfSinGainP */	SINE_GAINP_AF,
/* AfSinGainM */	SINE_GAINM_AF,
#endif
/* DecrementStep */ DECRE_CAL,
#ifdef SEL_CLOSED_AF
/* ZBiasInit */		AF_BIAS,
/* ZOffsetInit */	AF_OFSTO,
/* ZOffsetInitIn */	AF_OFSTI,
/* ZTargetRange */	BIAS_ADJ_RANGE_Z,
/* ZTargetMax */	(BIAS_ADJ_RANGE_Z + MARGIN),
/* ZTargetMin */	(BIAS_ADJ_RANGE_Z - MARGIN),
/* ZHighMargin */	HIGH_MARGIN_Z,
/* ZLowMargin */	LOW_MARGIN_Z,
#endif
} };

const ADJ_LOPGAN LS_LoopGainParameter[] = { 
{
/****** P1 ACT=01 ******/
/* Hxgain */		SXGAIN_LOP,
/* Hygain */		SYGAIN_LOP,
/* XNoiseNum */		LOOP_NUM_HX,
/* XNoiseFreq */	LOOP_FREQ_HX,
/* XNoiseGain */	LOOP_GAIN_HX, 
/* XGap  */			GAIN_GAP_HX,
/* YNoiseNum */		LOOP_NUM_HY,
/* YNoiseFreq */	LOOP_FREQ_HY,
/* YNoiseGain */	LOOP_GAIN_HY, 
/* YGap  */			GAIN_GAP_HY,
/* XJudgeHigh */ 	LOOP_MAX_X,
/* XJudgeLow  */ 	LOOP_MIN_X,
/* YJudgeHigh */ 	LOOP_MAX_Y,
/* YJudgeLow  */ 	LOOP_MIN_Y,
#ifdef SEL_CLOSED_AF
/* Hzgain */		SZGAIN_LOP,
/* ZNoiseNum */		LOOP_NUM_HZ,
/* ZNoiseFreq */	LOOP_FREQ_HZ,
/* ZNoiseGain */	LOOP_GAIN_HZ,
/* ZGap  */			GAIN_GAP_HZ,
/* ZJudgeHigh */ 	LOOP_MAX_Z,
/* ZJudgeLow  */ 	LOOP_MIN_Z,
#endif
} };

const ADJ_LINEARITY_MIXING LS_LinMixParameter[] = {
{
/****** P1 ACT=01 ******/
/* SltOffsetX */	SLT_OFFSET_X,
/* SltOffsetY */	SLT_OFFSET_Y,
/* SltDirX */		SLT_DIR_X,
/* SltDirY */		SLT_DIR_Y
} };

#else	// __OISCMD__
extern const ADJ_HALL_LS LS_HallCalParameter[];
extern const ADJ_LOPGAN LS_LoopGainParameter[];
extern const ADJ_LINEARITY_MIXING LS_LinMixParameter[];
#endif	// __OISCMD__

#undef SLT_OFFSET_X
#undef SLT_OFFSET_Y
#undef SLT_DIR_X
#undef SLT_DIR_Y

#undef XY_BIAS
#undef X_OFSTO
#undef X_OFSTI
#undef Y_OFSTO
#undef Y_OFSTI
#undef MARGIN
#undef BIAS_ADJ_RANGE_X
#undef BIAS_ADJ_RANGE_Y
#undef SINE_OFFSET
#undef SINE_GAIN
#undef SINE_NUM
#undef SINE_OFFSET_OIS
#undef SINE_GAIN_OIS
#undef SINE_NUM_OIS
#undef DECRE_CAL
#undef SXGAIN_LOP
#undef SYGAIN_LOP
#undef LOOP_NUM_HX
#undef LOOP_FREQ_HX
#undef LOOP_GAIN_HX
#undef GAIN_GAP_HX
#undef LOOP_NUM_HY
#undef LOOP_FREQ_HY
#undef LOOP_GAIN_HY
#undef GAIN_GAP_HY
#undef LOOP_MAX_X
#undef LOOP_MIN_X
#undef LOOP_MAX_Y
#undef LOOP_MIN_Y
#undef	MAX_CURRENT
#undef	MIN_CURRENT
#undef	CURRENT_STEP
#undef	STEP_WAIT_TIME
#undef	MEASURE_NUM

#ifdef SEL_CLOSED_AF
 #undef AF_BIAS
 #undef AF_OFSTO
 #undef AF_OFSTI
 #undef BIAS_ADJ_RANGE_Z
 #undef SZGAIN_LOP
 #undef SINE_OFFSET_AF
 #undef SINE_GAINP_AF
 #undef SINE_GAINM_AF
 #undef SINE_NUM_AF	
 #undef LOOP_NUM_HZ
 #undef LOOP_FREQ_HZ
 #undef LOOP_GAIN_HZ
 #undef GAIN_GAP_HZ
 #undef LOOP_MAX_Z
 #undef LOOP_MIN_Z
 #undef SEL_CLOSED_AF
 #undef HIGH_MARGIN_Z
 #undef LOW_MARGIN_Z
#endif

#undef TEMP_RCODEX
#undef TEMP_RCODEY
#undef TEMP_RCODEZ
#undef TEMP_SHAG
#undef TEMP_SHBG
#undef TEMP_SHCG
#undef TEMP_SHOUTAG
#undef TEMP_SHOUTBG
#undef TEMP_SHAB
#undef TEMP_SHAC
#undef TEMP_SHAA
#undef TEMP_SHBB
#undef TEMP_SHBC
#undef TEMP_SHBA
#undef TEMP_SHCB
#undef TEMP_SHCC
#undef TEMP_SHCA
#undef TEMP_TAB
#undef TEMP_TAC
#undef TEMP_TAA
#undef TEMP_TBB
#undef TEMP_TBC
#undef TEMP_TBA
#undef TEMP_TEMPOFF
#undef TEMP_TAG
#undef TEMP_TBG
#undef TEMP_SHIFTG
#undef TEMP_TCX
#undef TEMP_TBX
#undef TEMP_TAX
