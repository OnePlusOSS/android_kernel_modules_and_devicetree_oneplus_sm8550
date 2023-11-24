/**
 * @brief		OIS system calibration parameters
 *
 * @author		(C) 2020 ON Semiconductor.
 *
 * @file		Calibration_TAH1120.h
 * @date		svn:$Date:: 2021-01-29 20:36:10 +0900#$
 * @version 	svn:$Revision: 10 $
 * @attention
 **/



//********************************************************************************
// defines
//********************************************************************************
#define	XY_BIAS			(0x30000000 )
#define	X_OFSTO			(0x80000000 ) 
#define	X_OFSTI			(0x80000000 ) 
#define	Y_OFSTO			(0x80000000 ) 
#define	Y_OFSTI			(0x80000000 ) 

#define	Z_BIAS			(0x30000000 )
#define	Z_OFSTO			(0x80000000 ) 
#define	Z_OFSTI			(0x80000000 ) 

#define	MARGIN			(0x0300 )				// Margin

/************ for linearity measurement step ***********/
#define	SLT_OFFSET_X		(0x1200)			// NVC move step Y
#define	SLT_OFFSET_Y		(0x1200)			// NVC move step X
#define	SLT_DIR_X			(-1)				// NVC move direction X
#define	SLT_DIR_Y			(+1)				// NVC move direction Y

/************ Target amplitude ***********/
#if 1
#define	BIAS_ADJ_RANGE_X	(0xCCCC)			//!< 80%
#define	BIAS_ADJ_RANGE_Y	(0xCCCC)			//!< 80%
#define	BIAS_ADJ_RANGE_Z	(0xCCCC)			//!< 80%
#else
#define	BIAS_ADJ_RANGE_X	(0x7FFF)			//!< 50%
#define	BIAS_ADJ_RANGE_Y	(0x7FFF)			//!< 50%
#define	BIAS_ADJ_RANGE_Z	(0x7FFF)			//!< 50%
#endif

/* for OIS calibration */
#define	SINE_OFFSET_OIS		0x00000000			// unused
#define	SINE_GAIN_OIS		0x00000000			// unused
#define	SINE_NUM_OIS		1536				// 100ms (1/15kHz * 1536)

#define 	DECRE_CAL			(0x0100)		// decrease value

#define		ACT_MAX_DRIVE_X		0x3FFFFFFF		// 100mA /200=0.5
#define		ACT_MAX_DRIVE_Y		0x3FFFFFFF		// 100mA /200=0.5
#define		ACT_MIN_DRIVE_X		0xC0000000
#define		ACT_MIN_DRIVE_Y		0xC0000000


#define		ACT_X_STEP_NUM				0x1F
#define		ACT_X_STEP					(ACT_MAX_DRIVE_X/(ACT_X_STEP_NUM+1))
#define		ACT_X_STEP_TIME				2

#define		ACT_Y_STEP_NUM				0x1F
#define		ACT_Y_STEP					(ACT_MAX_DRIVE_Y/(ACT_Y_STEP_NUM+1))
#define		ACT_Y_STEP_TIME				2

//#define		MEASURE_WAIT				80
#define		MEASURE_WAIT				100


#define		SXGAIN_LOP		(0x38000000 )		// 0.625
#define		SYGAIN_LOP		(0x38000000 )		// 0.625
#define		SZGAIN_LOP		(0x38000000 )		// 0.625

#define 	LOOP_NUM_HX		(2508		)		// unused 18.0288kHz/0.130kHz*16times
#define 	LOOP_FREQ_HX	(0x00D10422	)		// unused Freq * 80000000h / Fs
#define 	LOOP_GAIN_HX	(0x09249249	)		// unused -22.92dB
#define		GAIN_GAP_HX		(1000)				// unused 20*log(1000/1000)=0dB

#define 	LOOP_NUM_HY		(2508		)		// unused 18.0288kHz/0.130kHz*16times
#define 	LOOP_FREQ_HY	(0x00D10422	)		// unused Freq * 80000000h / Fs
#define 	LOOP_GAIN_HY	(0x09249249	)		// unused -22.92dB
#define		GAIN_GAP_HY		(1000)				// unused 20*log(1000/1000)=0dB

#define 	LOOP_NUM_HZ		(2508		)		// unused 18.0288kHz/0.130kHz*16times
#define 	LOOP_FREQ_HZ	(0x00D10422	)		// unused Freq * 80000000h / Fs
#define 	LOOP_GAIN_HZ	(0x09249249	)		// unused -22.92dB
#define		GAIN_GAP_HZ		(1000)				// unused 20*log(1000/1000)=0dB


#define 	LOOP_MAX_X		(SXGAIN_LOP << 1)	// x2
#define 	LOOP_MIN_X		(SXGAIN_LOP >> 1)	// x0.5
#define 	LOOP_MAX_Y		(SYGAIN_LOP << 1)	// x2
#define 	LOOP_MIN_Y		(SYGAIN_LOP >> 1)	// x0.5
#define 	LOOP_MAX_Z		(SYGAIN_LOP << 1)	// x2
#define 	LOOP_MIN_Z		(SYGAIN_LOP >> 1)	// x0.5



//********************************************************************************
// structure for calibration
//********************************************************************************
const ADJ_HALL SMA_HallCalParameter = {
/* BiasInit */		XY_BIAS,
/* XOffsetInit */		X_OFSTO,
/* XOffsetInitIn */		X_OFSTI,
/* YOffsetInit */		Y_OFSTO,
/* YOffsetInitIn */		Y_OFSTI,
/* BiasInit */		Z_BIAS,
/* ZOffsetInit */		Z_OFSTO,
/* ZOffsetInitIn */		Z_OFSTI,

/* Margin */		MARGIN,
/* XTargetRange */	BIAS_ADJ_RANGE_X,
/* XTargetMax */		(BIAS_ADJ_RANGE_X + MARGIN),
/* XTargetMin */		(BIAS_ADJ_RANGE_X - MARGIN),
/* YTargetRange */	BIAS_ADJ_RANGE_Y,
/* YTargetMax */		(BIAS_ADJ_RANGE_Y + MARGIN),
/* YTargetMin */		(BIAS_ADJ_RANGE_Y - MARGIN),

/* ZTargetRange */	BIAS_ADJ_RANGE_Z,
/* ZTargetMax */		(BIAS_ADJ_RANGE_Z + MARGIN),
/* ZTargetMin */		(BIAS_ADJ_RANGE_Z - MARGIN),

/* OisSinNum */		SINE_NUM_OIS,
/* OisSinFreq */		SINE_OFFSET_OIS,
/* OisSinGain */		SINE_GAIN_OIS,
/* DecrementStep */ 	DECRE_CAL,
	
/* ActMaxDrive_X 	*/	ACT_MAX_DRIVE_X,
/* ActMaxDrive_Y	*/	ACT_MAX_DRIVE_Y,
/* ActMinDrive_X	*/	ACT_MIN_DRIVE_X,
/* ActMinDrive_Y	*/	ACT_MIN_DRIVE_Y,
/* ActStep_X	*/  ACT_X_STEP,
/* ActStep_X_Num*/  ACT_X_STEP_NUM,
/* ActStep_X_time*/ ACT_X_STEP_TIME,
/* ActStep_Y 	*/  ACT_Y_STEP,
/* ActStep_Y_Num*/  ACT_Y_STEP_NUM,
/* ActStep_Y_time*/ ACT_Y_STEP_TIME,
/* WaitTime*/  		MEASURE_WAIT,  	

};

const ADJ_LOPGAN SMA_LoopGainParameter = { 

/* Hxgain */		SXGAIN_LOP,
/* Hygain */		SYGAIN_LOP,
/* XNoiseNum */		LOOP_NUM_HX,
/* XNoiseFreq */		LOOP_FREQ_HX,
/* XNoiseGain */		LOOP_GAIN_HX, 
/* XGap  */			GAIN_GAP_HX,
/* YNoiseNum */		LOOP_NUM_HY,
/* YNoiseFreq */		LOOP_FREQ_HY,
/* YNoiseGain */		LOOP_GAIN_HY, 
/* YGap  */			GAIN_GAP_HY,
/* XJudgeHigh */ 		LOOP_MAX_X,
/* XJudgeLow  */ 		LOOP_MIN_X,
/* YJudgeHigh */ 		LOOP_MAX_Y,
/* YJudgeLow  */ 		LOOP_MIN_Y,

/* Hzgain */		SZGAIN_LOP,
/* ZNoiseNum */		LOOP_NUM_HZ,
/* ZNoiseFreq */		LOOP_FREQ_HZ,
/* ZNoiseGain */		LOOP_GAIN_HZ, 
/* ZGap  */			GAIN_GAP_HZ,
/* ZJudgeHigh */ 		LOOP_MAX_Z,
/* ZJudgeLow  */ 		LOOP_MIN_Z,

}; //   

const ADJ_LINEARITY_MIXING SMA_LinMixParameter = {
/****** TAT-9400 3-hall ACT=04 ******/
/* SltOffsetX */		SLT_OFFSET_X,
/* SltOffsetY */		SLT_OFFSET_Y,
/* SltDirX */		SLT_DIR_X,
/* SltDirY */		SLT_DIR_Y
};

#undef	SLT_OFFSET_X
#undef	SLT_OFFSET_Y
#undef	SLT_DIR_X
#undef	SLT_DIR_Y

#undef	XY_BIAS
#undef	X_OFSTO
#undef	X_OFSTI
#undef	Y_OFSTO
#undef	Y_OFSTI
#undef 	MARGIN
#undef 	BIAS_ADJ_RANGE_X
#undef 	BIAS_ADJ_RANGE_Y
#undef	SINE_OFFSET
#undef	SINE_GAIN
#undef	SINE_NUM
#undef	SINE_OFFSET_OIS
#undef	SINE_GAIN_OIS
#undef	SINE_NUM_OIS
#undef 	DECRE_CAL
#undef	SXGAIN_LOP
#undef	SYGAIN_LOP
#undef 	LOOP_NUM_HX
#undef 	LOOP_FREQ_HX
#undef 	LOOP_GAIN_HX
#undef	GAIN_GAP_HX
#undef 	LOOP_NUM_HY
#undef 	LOOP_FREQ_HY
#undef 	LOOP_GAIN_HY
#undef	GAIN_GAP_HY
#undef 	LOOP_MAX_X
#undef 	LOOP_MIN_X
#undef 	LOOP_MAX_Y
#undef 	LOOP_MIN_Y

#undef	ACT_MAX_DRIVE_X
#undef	ACT_MAX_DRIVE_Y
#undef	ACT_MIN_DRIVE_X
#undef	ACT_MIN_DRIVE_Y



