/**
 * @brief		OIS system common header for LC898129
 * 				Defines, Structures, function prototypes
 *
 * @author		(C) 2020 ON Semiconductor.
 *
 * @file		Ois.h
 * @date		svn:$Date:: 2021-02-01 19:30:40 +0900#$
 * @version		svn:$Revision: 11 $
 * @attention
 **/
#ifndef OIS_H_
#define OIS_H_

typedef	signed char          INT_8;
typedef	short                INT_16;
typedef	long                 INT_32;
typedef	long long            INT_64;
typedef	unsigned char        UINT_8;
typedef	unsigned short       UINT_16;
typedef	unsigned long        UINT_32;
typedef	unsigned long long   UINT_64;

//#define	NO_CNTWR						// define if you can't use cntwrt and cntrd

//#define		_BIG_ENDIAN_
#define		USE_FRA			// uncomment if use FRA function
#define		USE_AF 0
#define 	DEBUG 0
#define 	SLT_XY_SWAP 0
#include	"OisAPI.h"
#include	"OisLc898129.h"
#include	"OisLc898129SMA.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"

#if DEBUG
#include <AT91SAM7S.h>
#include <us.h>
 #ifndef	_CMD_H_
 extern void dbg_printf(const char *, ...);
 extern void dbg_Dump(const char *, int);
 #endif
 #define TRACE(fmt, ...)               dbg_printf(fmt, ## __VA_ARGS__)
 #define TRACE_DUMP(x,y)		dbg_Dump(x,y)
#else
 #define TRACE(...)
 #define TRACE_DUMP(x,y)
#endif

/**************** FW version *****************/
#define	MDL_VER			0x00
#define	FW_VER			0x03

/**************** Select Mode **************/
//#define		NEUTRAL_CENTER			//!< Upper Position Current 0mA Measurement
//#define		NEUTRAL_CENTER_FINE	//!< Optimize natural center current
#define		SEL_SHIFT_COR			//!< Shift correction
//#define		ADJ_OFFIN_OIS			//!< OIS side hall input offset adjustment
//#define		ADJ_OFFIN_AF			//!< AF  side hall input offset adjustment
/**************** Filter sampling **************/
#define	FS_FREQ			15027.32F

// Command Status
#define	EXE_END			0x00000002L		//!< Execute End (Adjust OK)
#define	EXE_HXADJ		0x00000006L		//!< Adjust NG : X Hall NG (Gain or Offset)
#define	EXE_HYADJ		0x0000000AL		//!< Adjust NG : Y Hall NG (Gain or Offset)
#define	EXE_LXADJ		0x00000012L		//!< Adjust NG : X Loop NG (Gain)
#define	EXE_LYADJ		0x00000022L		//!< Adjust NG : Y Loop NG (Gain)
#define	EXE_GXADJ		0x00000042L		//!< Adjust NG : X Gyro NG (offset)
#define	EXE_GYADJ		0x00000082L		//!< Adjust NG : Y Gyro NG (offset)
#define	EXE_ERR			0x00000099L		//!< Execute Error End
#define	EXE_HZADJ		0x00100002L		//!< Adjust NG : AF Hall NG (Gain or Offset)
#define	EXE_LZADJ		0x00200002L		//!< Adjust NG : AF Loop NG (Gain)
#define	EXE_LNZADJ		0x00800000L		//!< Adjust NG : AF Hall Linearity adjust
#define EXE_GZADJ		0x00400002L		//!< Adjust NG : Z Gyro NG (offset)
#define EXE_AZADJ		0x00200002L		//!< Adjust NG : Z ACCL NG (offset)
#define EXE_AYADJ		0x00100002L		//!< Adjust NG : Y ACCL NG (offset)
#define EXE_AXADJ		0x00080002L		//!< Adjust NG : X ACCL NG (offset)

#define	DEFAULT_SLAVE_ADR	0x48
#define	BALL_TYPE

// Common Define
#define	SUCCESS			0x00			//!< Success
#define	FAILURE			0x01			//!< Failure

#ifndef ON
 #define	ON				0x01		//!< ON
 #define	OFF				0x00		//!< OFF
#endif
 #define	SPC				0x02		//!< Special Mode

#define	X_DIR			0x00			//!< X Direction
#define	Y_DIR			0x01			//!< Y Direction
#define	Z_DIR			0x02			//!< Z Direction

#define	X_402			0xE4
#define	Y_402			0xE8
#define	DEFAULT_SLV_ADDRESS_402		0xE4
#define	MAGIC_CODE_SLV_ADDRESS_402	0xE6

#define	WPB_OFF			0x01			//!< Write protect OFF
#define WPB_ON			0x00			//!< Write protect ON

struct STFILREG {						//!< Register data table
	UINT_16	UsRegAdd ;
	UINT_8	UcRegDat ;
} ;

struct STFILRAM {						//!< Filter coefficient table
	UINT_16	UsRamAdd ;
	UINT_32	UlRamDat ;
} ;

// Calibration.h *******************************************************************
#define	HLXBO				0x00000001			//!< D/A Converter Channel Select OIS X BIAS
#define	HLYBO				0x00000002			//!< D/A Converter Channel Select OIS Y BIAS
#define	HLZBO				0x00000004			//!< D/A Converter Channel Select AF BIAS
//@#define	HLYBO				0x00000004			//!< D/A Converter Channel Select OIS Y BIAS
//@#define	HLZBO				0x00000002			//!< D/A Converter Channel Select AF BIAS

#define	HLXOI				0x00000008			//!< D/A Converter Channel Select OIS X OFFSET (input side)
#define	HLYOI				0x00000010			//!< D/A Converter Channel Select OIS Y OFFSET (input side)
#define	HLZOI				0x00000020			//!< D/A Converter Channel Select AF OFFSET    (input side)
//@#define	HLYOI				0x00000020			//!< D/A Converter Channel Select OIS Y OFFSET (input side)
//@#define	HLZOI				0x00000010			//!< D/A Converter Channel Select AF OFFSET    (input side)

#define HLSMAI				0x00000040
#define	HLXOO				0x00000080			//!< D/A Converter Channel Select OIS X OFFSET (output side)
#define	HLYOO				0x00000100			//!< D/A Converter Channel Select OIS Y OFFSET (output side)
#define	HLZOO				0x00000200			//!< D/A Converter Channel Select AF OFFSET    (output side)
//@#define	HLYOO				0x00000200			//!< D/A Converter Channel Select OIS Y OFFSET (output side)
//@#define	HLZOO				0x00000100			//!< D/A Converter Channel Select AF OFFSET    (output side)

// MeasureFilter.h *******************************************************************
typedef struct {
	INT_32				SiSampleNum ;			//!< Measure Sample Number
	INT_32				SiSampleMax ;			//!< Measure Sample Number Max

	struct {
		INT_32			SiMax1 ;				//!< Max Measure Result
		INT_32			SiMin1 ;				//!< Min Measure Result
		UINT_32			UiAmp1 ;				//!< Amplitude Measure Result
		INT_64			LLiIntegral1 ;			//!< Integration Measure Result
		INT_64			LLiAbsInteg1 ;			//!< Absolute Integration Measure Result
		INT_32			PiMeasureRam1 ;			//!< Measure Delay RAM Address

		INT_32			SiPolarity1 ;
		INT_32			SiThresh1 ;
		UINT_32			UiLengthA1 ;
		UINT_32			UiLengthB1 ;
		INT_64			LLiLengthA1 ;
		INT_64			LLiLengthB1 ;
	} MeasureFilterA ;

	struct {
		INT_32			SiMax2 ;				//!< Max Measure Result
		INT_32			SiMin2 ;				//!< Min Measure Result
		UINT_32			UiAmp2 ;				//!< Amplitude Measure Result
		INT_64			LLiIntegral2 ;			//!< Integration Measure Result
		INT_64			LLiAbsInteg2 ;			//!< Absolute Integration Measure Result
		INT_32			PiMeasureRam2 ;			//!< Measure Delay RAM Address

		INT_32			SiPolarity2 ;
		INT_32			SiThresh2 ;
		UINT_32			UiLengthA2 ;
		UINT_32			UiLengthB2 ;
		INT_64			LLiLengthA2 ;
		INT_64			LLiLengthB2 ;
	} MeasureFilterB ;
} MeasureFunction_Type ;


/*** caution [little-endian] ***/

#ifdef _BIG_ENDIAN_
// Big endian
// Word Data Union
union	WRDVAL{
	INT_16	SsWrdVal ;
	UINT_16	UsWrdVal ;
	UINT_8	UcWrkVal[ 2 ] ;
	INT_8	ScWrkVal[ 2 ] ;
	struct {
		UINT_8	UcHigVal ;	// [15:8]
		UINT_8	UcLowVal ;	// [7:0]
	} StWrdVal ;
} ;


union	DWDVAL {
	UINT_32	UlDwdVal ;
	UINT_16	UsDwdVal[ 2 ] ;
	struct {
		UINT_16	UsHigVal ;	// [31:16]
		UINT_16	UsLowVal ;	// [15:0]
	} StDwdVal ;
	struct {
		UINT_8	UcRamVa3 ;	// [31:24]
		UINT_8	UcRamVa2 ;	// [23:16]
		UINT_8	UcRamVa1 ;	// [15:8]
		UINT_8	UcRamVa0 ;	// [7:0]
	} StCdwVal ;
} ;

union	ULLNVAL {
	UINT_64	UllnValue ;
	UINT_32	UlnValue[ 2 ] ;
	struct {
		UINT_32	UlHigVal ;	// [63:32]
		UINT_32	UlLowVal ;	// [31:0]
	} StUllnVal ;
} ;


// Float Data Union
union	FLTVAL {
	float	SfFltVal ;
	UINT_32	UlLngVal ;
	UINT_16	UsDwdVal[ 2 ] ;
	struct {
		UINT_16	UsHigVal ;	// [31:16]
		UINT_16	UsLowVal ;	// [15:0]
	} StFltVal ;
} ;

#else	// BIG_ENDDIAN
// Little endian
// Word Data Union
union	WRDVAL{
	UINT_16	UsWrdVal ;
	UINT_8	UcWrkVal[ 2 ] ;
	struct {
		UINT_8	UcLowVal ;	// [15:0]
		UINT_8	UcHigVal ;	// [31:16]
	} StWrdVal ;
} ;

union	DWDVAL {
	UINT_32	UlDwdVal ;
	UINT_16	UsDwdVal[ 2 ] ;
	struct {
		UINT_16	UsLowVal ;	// [15:0]
		UINT_16	UsHigVal ;	// [31:16]
	} StDwdVal ;
	struct {
		UINT_8	UcRamVa0 ;	// [7:0]
		UINT_8	UcRamVa1 ;	// [15:8]
		UINT_8	UcRamVa2 ;	// [23:16]
		UINT_8	UcRamVa3 ;	// [31:24]
	} StCdwVal ;
} ;

union	ULLNVAL {
	UINT_64	UllnValue ;
	UINT_32	UlnValue[ 2 ] ;
	struct {
		UINT_32	UlLowVal ;	// [31:0]
		UINT_32	UlHigVal ;	// [63:32]
	} StUllnVal ;
} ;

// Float Data Union
union	FLTVAL {
	float	SfFltVal ;
	UINT_32	UlLngVal ;
	UINT_16	UsDwdVal[ 2 ] ;
	struct {
		UINT_16	UsLowVal ;	// [15:0]
		UINT_16	UsHigVal ;	// [31:16]
	} StFltVal ;
} ;
#endif	// _BIG_ENDIAN_

typedef union WRDVAL	UnWrdVal ;
typedef union DWDVAL	UnDwdVal;
typedef union ULLNVAL	UnllnVal;
typedef union FLTVAL	UnFltVal ;

typedef struct {
	UINT_32 BiasInit;
	UINT_32 XOffsetInit;
	UINT_32 XOffsetInitIn;
	UINT_32 YOffsetInit;
	UINT_32 YOffsetInitIn;
	UINT_32 ZBiasInit;
	UINT_32 ZOffsetInit;
	UINT_32 ZOffsetInitIn;

	UINT_32 OffsetMargin;
	UINT_32 XTargetRange;
	UINT_32 XTargetMax;
	UINT_32 XTargetMin;
	UINT_32 YTargetRange;
	UINT_32 YTargetMax;
	UINT_32 YTargetMin;
	UINT_32 ZTargetRange;
	UINT_32 ZTargetMax;
	UINT_32 ZTargetMin;

	UINT_32 OisSinNum;
	UINT_32 OisSinFreq;
	UINT_32 OisSinGain;
	UINT_32 DecrementStep;
	
	UINT_32 ActMaxDrive_X;
	UINT_32 ActMaxDrive_Y;
	UINT_32 ActMinDrive_X;
	UINT_32 ActMinDrive_Y;
	UINT_32 ActStep_X;
	UINT_32 ActStep_X_Num;
	UINT_32 ActStep_X_time;
	UINT_32 ActStep_Y;
	UINT_32 ActStep_Y_Num;
	UINT_32 ActStep_Y_time;
	UINT_32 WaitTime;
} ADJ_HALL;

typedef struct {
	UINT_8 BiasInit;
	UINT_8 XOffsetInit;
	UINT_8 YOffsetInit;
	UINT_32 OffsetMargin;
	UINT_32 XTargetRange;
	UINT_32 XTargetMax;
	UINT_32 XTargetMin;
	UINT_32 YTargetRange;
	UINT_32 YTargetMax;
	UINT_32 YTargetMin;
#ifdef BALL_TYPE
	UINT_32	Max_Current;
	UINT_32	Min_Current;
	UINT_32	Current_Step;
	UINT_32	Step_Wait_Time;
	UINT_16	MeasureNum;
#else
	UINT_32 OisSinNum;
	UINT_32 OisSinFreq;
	UINT_32 OisSinGain;
#endif
#if ( USE_AF == 1 )
	UINT_32 AfSinNum;
	UINT_32 AfSinFreq;
	UINT_32 AfSinGainP;
	UINT_32 AfSinGainM;
#endif
	UINT_16 DecrementStep;
#if ( USE_AF == 1 )
	UINT_32 ZBiasInit;
	UINT_32 ZOffsetInit;
	UINT_32 ZOffsetInitIn;
	UINT_32 ZTargetRange;
	UINT_32 ZTargetMax;
	UINT_32 ZTargetMin;
	UINT_32 ZHighMargin;
	UINT_32 ZLowMargin;
#endif
} ADJ_HALL_LS;

typedef struct {
	UINT_32 Hxgain;
	UINT_32 Hygain;
	UINT_32 XNoiseNum;
	UINT_32 XNoiseFreq;
	UINT_32 XNoiseGain;
	UINT_32 XGap;
	UINT_32 YNoiseNum;
	UINT_32 YNoiseFreq;
	UINT_32 YNoiseGain;
	UINT_32 YGap;
	UINT_32 XJudgeHigh;
	UINT_32 XJudgeLow;
	UINT_32 YJudgeHigh;
	UINT_32 YJudgeLow;
	UINT_32 Hzgain;
	UINT_32 ZNoiseNum;
	UINT_32 ZNoiseFreq;
	UINT_32 ZNoiseGain;
	UINT_32 ZGap;
	UINT_32 ZJudgeHigh;
	UINT_32 ZJudgeLow;
} ADJ_LOPGAN;

typedef struct {
	INT_16	SltOffsetX;
	INT_16	SltOffsetY;
	INT_16	SltDirX;
	INT_16	SltDirY;
} ADJ_LINEARITY_MIXING;

typedef struct STADJPAR {
	struct {
		UINT_32	UlAdjPhs ;				//!< Hall Adjust Phase

		UINT_16	UsHlxCna ;				//!< Hall Center Value after Hall Adjust
		UINT_16	UsHlxMax ;				//!< Hall Max Value
		UINT_16	UsHlxMxa ;				//!< Hall Max Value after Hall Adjust
		UINT_16	UsHlxMin ;				//!< Hall Min Value
		UINT_16	UsHlxMna ;				//!< Hall Min Value after Hall Adjust
		UINT_16	UsHlxGan ;				//!< Hall Gain Value
		UINT_16	UsHlxOffI ;				//!< Hall Offset Value
		UINT_16	UsHlxOffO ;				//!< Hall Offset Value
		UINT_16	UsAdxOff ;				//!< Hall A/D Offset Value
		UINT_16	UsHlxCen ;				//!< Hall Center Value
		UINT_16	UsHigxGap ;
		UINT_16	UsLowxGap ;
		
		UINT_16	UsHlyCna ;				//!< Hall Center Value after Hall Adjust
		UINT_16	UsHlyMax ;				//!< Hall Max Value
		UINT_16	UsHlyMxa ;				//!< Hall Max Value after Hall Adjust
		UINT_16	UsHlyMin ;				//!< Hall Min Value
		UINT_16	UsHlyMna ;				//!< Hall Min Value after Hall Adjust
		UINT_16	UsHlyGan ;				//!< Hall Gain Value
		UINT_16	UsHlyOffI ;				//!< Hall Offset Value
		UINT_16	UsHlyOffO ;				//!< Hall Offset Value
		UINT_16	UsAdyOff ;				//!< Hall A/D Offset Value
		UINT_16	UsHlyCen ;				//!< Hall Center Value
		UINT_16	UsHigyGap ;
		UINT_16	UsLowyGap ;
		
		UINT_16	UsHlzCna ;				//!< Z Hall Center Value after Hall Adjust
		UINT_16	UsHlzMax ;				//!< Z Hall Max Value
		UINT_16	UsHlzMxa ;				//!< Z Hall Max Value after Hall Adjust
		UINT_16	UsHlzMin ;				//!< Z Hall Min Value
		UINT_16	UsHlzMna ;				//!< Z Hall Min Value after Hall Adjust
		UINT_16	UsHlzGan ;				//!< Z Hall Gain Value
		UINT_16	UsHlzOffI ;				//!< Z Hall Offset Value
		UINT_16	UsHlzOffO ;				//!< Z Hall Offset Value
		UINT_16	UsAdzOff ;				//!< Z Hall A/D Offset Value
		UINT_16	UsHlzCen ;				//!< Z Hall Center Value
		UINT_16	UsHlzAmp ;				//!< Z Hall Amp Magnification
		UINT_16	UsHigzGap ;
		UINT_16	UsLowzGap ;

		UINT_16 UsTemp ;				//!< Ambient temperature
	} StHalAdj ;

	struct {
		UINT_32	UlLxgVal ;				//!< Loop Gain X
		UINT_32	UlLygVal ;				//!< Loop Gain Y
		UINT_32	UlLzgVal ;				//!< Loop Gain Z
	} StLopGan ;

	struct {
		UINT_16	UsGxoVal ;				//!< Gyro A/D Offset X
		UINT_16	UsGyoVal ;				//!< Gyro A/D Offset Y
		UINT_16	UsGzoVal ;				//!< Gyro A/D Offset Z
		UINT_16	UsGxoSts ;				//!< Gyro Offset X Status
		UINT_16	UsGyoSts ;				//!< Gyro Offset Y Status
		UINT_16	UsGzoSts ;				//!< Gyro Offset Z Status
	} StGvcOff ;
} stAdjPar ;

__OIS_CMD_HEADER__	stAdjPar	StAdjPar ;		//!< Calibration data

typedef struct STHALLINEAR {
	UINT_16	XCoefA[6] ;
	UINT_16	XCoefB[6] ;
	UINT_16	XZone[5] ;
	UINT_16	YCoefA[6] ;
	UINT_16	YCoefB[6] ;
	UINT_16	YZone[5] ;
} stHalLinear ;

typedef struct STPOSOFF {
	struct {
		INT_32	Pos[6][3];
	} StPos;
	UINT_32		UlAclOfSt ;				//!< accel offset status

} stPosOff ;

__OIS_CMD_HEADER__	stPosOff	StPosOff ;				//!< Execute Command Parameter

typedef struct STACLVAL {
	struct {
		INT_32	SlOffsetX ;
		INT_32	SlOffsetY ;
		INT_32	SlOffsetZ ;
	} StAccel ;

	INT_32	SlInvMatrix[9] ;

} stAclVal ;

__OIS_CMD_HEADER__	stAclVal	StAclVal ;				//!< Execute Command Parameter

__OIS_CMD_HEADER__	stMesRam	StHAT_X ;				//!< Accuracy data X
__OIS_CMD_HEADER__	stMesRam	StHAT_Y ;				//!< Accuracy data Y
__OIS_CMD_HEADER__	stMesRam	StHAT_Z ;				//!< Accuracy data Z
__OIS_CMD_HEADER__	UINT_8		LS_SS_Sel ;

typedef	enum tFILTERMODE {
 	HALL_ADJ,
 	LOOPGAIN,
 	THROUGH,
 	NOISE,
	OSCCHK,
	GAINCURV,
	SELFTEST
}	eFilterMode;

#pragma pack( 1 )
typedef struct	STSCTDAT {
	UINT_8		UcFF;
	UnDwdVal	stData;
} stSctDat;
#pragma pack()

//	for RtnCen
#define		BOTH_ON			0x00
#define		XONLY_ON		0x01
#define		YONLY_ON		0x02
#define		BOTH_OFF		0x03
#define		ZONLY_OFF		0x04
#define		ZONLY_ON		0x05
//	for SetSinWavePara
#define		SINEWAVE		0
#define		XHALWAVE		1
#define		YHALWAVE		2
#define		ZHALWAVE		3
#define		XACTTEST		10
#define		YACTTEST		11
#define		CIRCWAVE		255
//	for TnePtp
#define		HALL_H_VAL		0x3F800000		//!< 1.0
//	for TneCen
#define		OFFDAC_8BIT		0				//!< 8bit Offset DAC select
#define		OFFDAC_3BIT		1				//!< 3bit Offset DAC select
#define		PTP_BEFORE		0
#define		PTP_AFTER		1
#define		PTP_ACCEPT		2

// for FRA measurement
#ifdef		USE_FRA
 #include	"OisFRA.h"
#endif

// for Accelerometer offset measurement
/* accelerometer uses 2g setting */
/**************MeasGyAcOffset************/
#define 	MESOF_NUM		2048						// 2048times
#define 	GYROFFSET_H		( 0x06D6 << 16 )			// 
//#define		GSENS			( 4096 << 16 )				// LSB/g 4g
#define		GSENS			( 16393 << 16 )				// LSM6DSM 2g

#define		GSENS_MARG		(GSENS / 4)					// 1/4 g
#define		POSTURETH		(GSENS - GSENS_MARG)		// LSB/g
//#define		ZG_MRGN			(409 << 16)					// G tolerance  100mG
#define		ZG_MRGN		(1310 << 16)

/**************MeasGyAcOffset************/

#endif /* #ifndef OIS_H_ */
