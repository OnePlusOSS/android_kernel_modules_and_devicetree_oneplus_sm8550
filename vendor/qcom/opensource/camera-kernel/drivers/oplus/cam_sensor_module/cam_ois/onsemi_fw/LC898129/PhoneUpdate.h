/**
 * @brief		LC898129 Global declaration & prototype declaration
 *
 * @author		(C) 2019 ON Semiconductor.
 * @file		PhoneUpdate.h
 * @date		svn:$Date:: 2021-05-19 17:20:17 +0900#$
 * @version		svn:$Revision: 46 $
 * @attention
 *
 **/
#ifndef PHONEUPDATE_H_
#define PHONEUPDATE_H_

#include <linux/types.h>
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "cam_ois_dev.h"

//==============================================================================
//
//==============================================================================
typedef	signed char			 INT_8;
typedef	short				 INT_16;
typedef	long                 INT_32;
typedef	long long            INT_64;
typedef	unsigned char       UINT_8;
typedef	unsigned short      UINT_16;
typedef	unsigned long       UINT_32;
typedef	unsigned long long	UINT_64;

//****************************************************
//	STRUCTURE DEFINE
//****************************************************
typedef struct {
	UINT_16				Index;
	const UINT_8*		UpdataCode;
	UINT_32				SizeUpdataCode;
	UINT_64				SizeUpdataCodeCksm;
	UINT_8*		        FromCode;
	UINT_32				SizeFromCode;
	unsigned int		SizeFromCodeCksm;
	unsigned int		SizeFromCodeValid;
}	CODE_TBL_EXT;

typedef struct STRECALIB {
	INT_16	SsFctryOffX ;
	INT_16	SsFctryOffY ;
	INT_16	SsRecalOffX ;
	INT_16	SsRecalOffY ;
	INT_16	SsDiffX ;
	INT_16	SsDiffY ;
} stReCalib ;


typedef struct STMESRAM {
	INT_32	SlMeasureMaxValue ;
	INT_32	SlMeasureMinValue ;
	INT_32	SlMeasureAmpValue ;
	INT_32	SlMeasureAveValue ;
} stMesRam ;

typedef struct STGYROOFFSETTBL {
	struct {
		INT_32	SlOffsetX ;
		INT_32	SlOffsetY ;
		INT_32	SlOffsetZ ;
	} StAngle ;

	struct {
		INT_32	SlOffsetX ;
		INT_32	SlOffsetY ;
		INT_32	SlOffsetZ ;
	} StAccel ;
} stGyroOffsetTbl ;

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

/*** caution [little-endian] ***/
#ifdef _BIG_ENDIAN_
typedef union	DWDVAL {
	UINT_32	UlDwdVal ;
	UINT_16	UsDwdVal[ 2 ] ;
	struct {
		UINT_16	UsHigVal ;
		UINT_16	UsLowVal ;
	} StDwdVal ;
	struct {
		UINT_8	UcRamVa3 ;
		UINT_8	UcRamVa2 ;
		UINT_8	UcRamVa1 ;
		UINT_8	UcRamVa0 ;
	} StCdwVal ;
}	UnDwdVal ;
typedef union	ULLNVAL {
	UINT_64	UllnValue ;
	UINT_32	UlnValue[ 2 ] ;
	struct {
		UINT_32	UlHigVal ;	// [63:32]
		UINT_32	UlLowVal ;	// [31:0]
	} StUllnVal ;
}	UnllnVal ;

#else	// BIG_ENDDIAN
typedef union	DWDVAL {
	UINT_32	UlDwdVal ;
	UINT_16	UsDwdVal[ 2 ] ;
	struct {
		UINT_16	UsLowVal ;
		UINT_16	UsHigVal ;
	} StDwdVal ;
	struct {
		UINT_8	UcRamVa0 ;
		UINT_8	UcRamVa1 ;
		UINT_8	UcRamVa2 ;
		UINT_8	UcRamVa3 ;
	} StCdwVal ;
}	UnDwdVal ;
typedef union	ULLNVAL {
	UINT_64	UllnValue ;
	UINT_32	UlnValue[ 2 ] ;
	struct {
		UINT_32	UlLowVal ;	// [31:0]
		UINT_32	UlHigVal ;	// [63:32]
	} StUllnVal ;
}	UnllnVal ;

#endif	// _BIG_ENDIAN_

#define	SUCCESS			0x00
#define	FAILURE			0x01

#define	X_DIR			0x00
#define	Y_DIR			0x01
#define	Z_DIR			0x02

//==============================================================================
//
//==============================================================================
#define		CMD_IO_ADR_ACCESS				0xC000
#define		CMD_IO_DAT_ACCESS				0xD000
#define 	SYSDSP_DSPDIV					0xD00014
#define 	SYSDSP_SOFTRES					0xD0006C
#define 	SYSDSP_REMAP					0xD000AC
#define 	SYSDSP_CVER						0xD00100
#define		ROMINFO							0xE050D4
#define FLASHROM_129						0xE07000
#define 		FLASHROM_FLA_RDAT			(FLASHROM_129 + 0x00)
#define 		FLASHROM_FLA_WDAT			(FLASHROM_129 + 0x04)
#define 		FLASHROM_ACSCNT				(FLASHROM_129 + 0x08)
#define 		FLASHROM_FLA_ADR			(FLASHROM_129 + 0x0C)
	#define			USER_MAT				0
	#define			INF_MAT0				1
	#define			INF_MAT1				2
	#define			INF_MAT2				4
	#define			TRIM_MAT				16

#define 		FLASHROM_CMD				(FLASHROM_129 + 0x10)
#define 		FLASHROM_FLAWP				(FLASHROM_129 + 0x14)
#define 		FLASHROM_FLAINT				(FLASHROM_129 + 0x18)
#define 		FLASHROM_FLAMODE			(FLASHROM_129 + 0x1C)
#define 		FLASHROM_TPECPW				(FLASHROM_129 + 0x20)
#define 		FLASHROM_TACC				(FLASHROM_129 + 0x24)

#define 		FLASHROM_ERR_FLA			(FLASHROM_129 + 0x98)
#define 		FLASHROM_RSTB_FLA			(FLASHROM_129 + 0x4CC)
#define 		FLASHROM_UNLK_CODE1			(FLASHROM_129 + 0x554)
#define 		FLASHROM_CLK_FLAON			(FLASHROM_129 + 0x664)
#define 		FLASHROM_UNLK_CODE2			(FLASHROM_129 + 0xAA8)
#define 		FLASHROM_UNLK_CODE3			(FLASHROM_129 + 0xCCC)

#define		READ_STATUS_INI					0x01000000



#define		HallFilterD_HXDAZ1				0x0138
#define		HallFilterD_HYDAZ1				0x0188
#define		HALL_RAM_HXOUT0					0x01D0
#define		HALL_RAM_HYOUT0					0x0220
#define		HALL_RAM_HXOFF1					0x01CC
#define		HALL_RAM_HYOFF1					0x021C

#define		SinWaveC						0x0570
#define		SinWaveC_Pt						0x0570
#define		SinWaveC_Regsiter				0x0574

#define		SinWave_Offset					0x057C
#define		SinWave_Phase					0x0580
#define		SinWave_Gain					0x0584
#define		SinWave_Output					0x0588
#define		SinWave_OutAddr					0x058C

#define		StMeasureFunc_MFA				0x04C0
#define		StMeasFunc_MFA_SiMax1			0x04C0
#define		StMeasFunc_MFA_SiMin1			0x04C4
#define		StMeasFunc_MFA_UiAmp1			0x04C8
#define		StMeasFunc_MFA_LLiIntegral1		0x04D0
#define		StMeasFunc_MFA_LLiAbsInteg1		0x04D8
#define		StMeasFunc_MFA_PiMeasureRam1	0x04E0

#define		GYRO_RAM_GX_ADIDAT				0x0460
#define		GYRO_RAM_GY_ADIDAT				0x0464

#define		GYRO_RAM_GXOFFZ					0x0480
#define		GYRO_RAM_GYOFFZ					0x0484

#define		GYRO_ZRAM_GZ_ADIDAT				0x061C
#define		GYRO_ZRAM_GZOFFZ				0x0628

#define		AcclRAM_X						0x06D0
#define			ACCLRAM_X_AC_ADIDAT			0x06D0
#define			ACCLRAM_X_AC_OFFSET			0x06D4

#define		AcclRAM_Y						0x06FC
#define			ACCLRAM_Y_AC_ADIDAT			0x06FC
#define			ACCLRAM_Y_AC_OFFSET			0x0700

#define		AcclRAM_Z						0x0728
#define			ACCLRAM_Z_AC_ADIDAT			0x0728
#define			ACCLRAM_Z_AC_OFFSET			0x072C

#define		RAM_X_OUT						0x0EB8
#define		RAM_Y_OUT						0x0EBC
#define		RAM_Z_OUT						0x0EC0

#define		HallFilterCoeffX_hxgain0		0x8070
#define		HallFilterCoeffY_hygain0		0x810C

#define		GyroFilterTableX_gxzoom			0x8410
#define		GyroFilterTableY_gyzoom			0x8470
#define		GyroFilterTableZ_gzzoom			0x8768

#define		GyroFilterTableX_LS_gxzoom		0x87D8
#define		GyroFilterTableY_LS_gyzoom		0x8810

#define		INF0_DATA						0x1A00
#define		INF1_DATA						0x1B00
#define		INF2_DATA						0x1C00


#define	EXE_END		0x00000002L		//!< Execute End (Adjust OK)
#define	EXE_HXADJ	0x00000006L		//!< Adjust NG : X Hall NG (Gain or Offset)
#define	EXE_HYADJ	0x0000000AL		//!< Adjust NG : Y Hall NG (Gain or Offset)
#define	EXE_LXADJ	0x00000012L		//!< Adjust NG : X Loop NG (Gain)
#define	EXE_LYADJ	0x00000022L		//!< Adjust NG : Y Loop NG (Gain)
#define	EXE_GXADJ	0x00000042L		//!< Adjust NG : X Gyro NG (offset)
#define	EXE_GYADJ	0x00000082L		//!< Adjust NG : Y Gyro NG (offset)
#define	EXE_ERR		0x00000099L		//!< Execute Error End
#define	EXE_HZADJ	0x00100002L		//!< Adjust NG : AF Hall NG (Gain or Offset)
#define	EXE_LZADJ	0x00200002L		//!< Adjust NG : AF Loop NG (Gain)
#define	EXE_LNZADJ	0x00800000L		//!< Adjust NG : AF Hall Linearity adjust
#define	EXE_GZADJ	0x00400002L		//!< Adjust NG : Z Gyro NG (offset)
#define	EXE_AZADJ	0x00200002L		// Adjust NG : Z ACCL NG (offset)
#define	EXE_AYADJ	0x00100002L		// Adjust NG : Y ACCL NG (offset)
#define	EXE_AXADJ	0x00080002L		// Adjust NG : X ACCL NG (offset)

//==============================================================================
// Prototype
//==============================================================================
extern void BootMode( void ) ;
extern UINT_8 PmemDownload129( UINT_8, UINT_8 ) ;
extern UINT_8 FlashProgram129( UINT_8, UINT_8,struct cam_ois_ctrl_t * ) ;
extern UINT_8 RdFlashMulti( UINT_8, UINT_32, UINT_32 *, UINT_8 ) ;
extern UINT_8 RdFlashSingle( UINT_8, UINT_32, UINT_32 * ) ;
extern UINT_8 RdStatus( UINT_8 ) ;
extern UINT_8 WrGyAcOffsetData( UINT_8 ) ;
extern UINT_8 WrGyroGainData( UINT_8 ) ;
extern UINT_8 WrGyroGainData_LS( UINT_8 ) ;
extern UINT_32 MeasGyAcOffset( void ) ;
extern void GetGyroOffset( UINT_16 *, UINT_16 *, UINT_16 * ) ;
extern void GetAcclOffset( UINT_16 *, UINT_16 *, UINT_16 * ) ;
extern void SetSinWavePara( UINT_8,  UINT_8 ) ;

#endif /* #ifndef PHONEUPDATE_H_ */
