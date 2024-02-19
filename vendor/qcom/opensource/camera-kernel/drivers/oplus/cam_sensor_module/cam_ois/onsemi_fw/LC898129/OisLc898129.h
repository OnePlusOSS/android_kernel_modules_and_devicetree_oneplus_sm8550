/**
 * @brief		LC898129 Global declaration
 *
 * @author		(C) 2019 ON Semiconductor.
 *
 * @file		OisLC898129.h
 * @date		svn:$Date:: 2021-01-20 16:15:44 +0900#$
 * @version	svn:$Revision: 2 $
 * @attention
 **/


/************************************************/
/*	Command										*/
/************************************************/
#define		CMD_IO_ADR_ACCESS				0xC000				//!< IO Write Access
#define		CMD_IO_DAT_ACCESS				0xD000				//!< IO Read Access
#define		CMD_FOR_LS						0x80000000
#define		CMD_RETURN_TO_CENTER			0xF010				//!< Center Servo ON/OFF choose axis
	#define		ALL_SRV_OFF						0x00070000			//!< All    Servo OFF
	#define		ALL_SRV_ON						0x00000007			//!< All    Servo ON
	#define		BOTH_SRV_OFF					0x00030000			//!< Both   Servo OFF
	#define		BOTH_SRV_ON						0x00000003			//!< Both   Servo ON
	#define		XAXS_SRV_OFF					0x00010000			//!< X axis Servo OFF
	#define		XAXS_SRV_ON						0x00000001			//!< X axis Servo ON
	#define		YAXS_SRV_OFF					0x00020000			//!< Y axis Servo OFF
	#define		YAXS_SRV_ON						0x00000002			//!< Y axis Servo ON
	#define		ZAXS_SRV_OFF					0x00040000			//!< Z axis Servo OFF
	#define		ZAXS_SRV_ON						0x00000004			//!< Z axis Servo ON
#define		CMD_PAN_TILT					0xF011				//!< Pan Tilt Enable/Disable
	#define		PAN_TILT_OFF					0x00000000			//!< Pan/Tilt OFF
	#define		PAN_TILT_ON						0x00000001			//!< Pan/Tilt ON
#define		CMD_OIS_ENABLE					0xF012				//!< Ois Enable/Disable
	#define		OIS_DISABLE						0x00000000			//!< OIS Disable
	#define		OIS_ENABLE						0x00000001			//!< OIS Enable
	#define		OIS_ENA_NCL						0x00000002			//!< OIS Enable ( none Delay clear )
	#define		OIS_ENA_DOF						0x00000004			//!< OIS Enable ( Drift offset exec )
#define		CMD_MOVE_STILL_MODE				0xF013				//!< Select mode
	#define		MOVIE_MODE						0x00000000			//!< Movie mode
	#define		STILL_MODE						0x00000001			//!< Still mode
	#define		MOVIE_MODE1						0x00000002			//!< Movie Preview mode 1
	#define		STILL_MODE1						0x00000003			//!< Still Preview mode 1
	#define		MOVIE_MODE2						0x00000004			//!< Movie Preview mode 2
	#define		STILL_MODE2						0x00000005			//!< Still Preview mode 2
	#define		MOVIE_MODE3						0x00000006			//!< Movie Preview mode 3
	#define		STILL_MODE3						0x00000007			//!< Still Preview mode 3
#define		CMD_CALIBRATION					0xF014				//!< Gyro offset re-calibration
#define		CMD_AF_TARGET					0xF01A				//!< Closed AF target code
#define		CMD_SSC_ENABLE					0xF01C				//!< Select mode
	#define		SSC_DISABLE						0x00000000			//!< Ssc Disable
	#define		SSC_ENABLE						0x00000001			//!< Ssc Enable
#define		CMD_402_TX_BUFFER_READ_WRITE	0xF800
#define		CMD_402_RX_BUFFER_READ_WRITE	0xF801
#define		CMD_402_BYTE_READ				0xF802
#define		CMD_402_BYTE_WRITE				0xF803
#define		CMD_402_SHORT_READ				0xF804
#define		CMD_402_SHORT_WRITE				0xF805


	// Calibration flags
	#define		HALL_CALB_BIT					0x00FF00FF			//!< Caribration bit mask
	#define		HALL_CALB_FLG					0x00008000			//!< Hall calibration done bit
	#define		GYRO_GAIN_FLG					0x00004000			//!< Gyro gain adjustment done bit
//	#define		ANGL_CORR_FLG					0x00002000			//!< Angle correction adjustment done bit
	#define		AF_OIS_XTLK_FLG					0x00002000			//!< AF OIS cross talk correction adjustment done bit
	#define		CLAF_LIN_FLG					0x00001000			//!< CLAF Linearity calibration done
	#define		OPAF_FST_FLG					0x00001000			//!< OPAF FST calibration done bit
	#define		CLAF_CALB_FLG					0x00000800			//!< CLAF Hall calibration done bit
	#define		HLLN_CALB_FLG					0x00000400			//!< Hall linear calibration done bit
	#define		CROS_TALK_FLG					0x00000200			//!< Cross talk calibration
	#define		ACCL_OFST_FLG					0x00000100			//!< Accel offset calibration

#define		CMD_READ_STATUS					0xF100				//!< Status Read
	#define		READ_STATUS_INI					0x01000000

#define		STBOSCPLL						0x00D00074			//!< STB OSC
	#define		OSC_STB							0x00000002			//!< OSC standby

//==============================================================================
// Calibration Data Memory Map
//==============================================================================
// Calibration Status
//==============================================================================
#define	CALIBRATION_STATUS		(  0 )
// Loop gain / lens offset
#define	LOOPGAIN_LENS_OFFSET_X	(  1 )	/* [31:16]X Lens offset	[15:0]X loop gain */
#define	LOOPGAIN_LENS_OFFSET_Y	(  2 )	/* [31:16]Y Lens offset	[15:0]Y loop gain */
// Hall amplitude Calibration
#define	HALL_AMPLITUDE_X		(  3 )	/* [31:16]X hall max	[15:0]X hall min */
#define	HALL_AMPLITUDE_Y		(  4 )	/* [31:16]Y hall max	[15:0]Y hall min */
// Accl/gyro offset
#define	ACCL_GYRO_OFFSET_X		(  5 )	/* [31:16]X acc offset	[15:0]X gyro offset */
#define	ACCL_GYRO_OFFSET_Y		(  6 )	/* [31:16]Y acc offset	[15:0]Y gyro offset */
#define	ACCL_GYRO_OFFSET_Z		(  7 )	/* [31:16]Z acc offset	[15:0]Z gyro offset */
// Gyro gain
#define	GYRO_GAIN_X				(  8 )
#define	GYRO_GAIN_Y				(  9 )
// Liniearity correction
#define LN_POS1					( 10 )	/* [31:16]Y  [15:0]X */
#define LN_POS2					( 11 )  /* [31:16]Y  [15:0]X */
#define LN_POS3					( 12 )  /* [31:16]Y  [15:8]X */
#define LN_POS4					( 13 )  /* [31:16]Y  [15:0]X */
#define LN_POS5					( 14 )  /* [31:16]Y  [15:0]X */
#define LN_POS6					( 15 )  /* [31:16]Y  [15:0]X */
#define LN_POS7					( 16 )  /* [31:16]Y  [15:8]X */
#define LN_STEP					( 17 )  /* [31:16]Y  [15:0]X */
// Gyro mixing correction
#define MIXING_X				( 18 )	/* [31:16]HX45X [15:0]HX45X */
#define MIXING_Y				( 19 )	/* [31:16]HY45Y [15:0]HY45Y */
#define MIXING_SFT				( 20 )	/* [31:16]2ndA  [15:8]YSFT [7:0]:XSHT   */
#define MIXING_2ND				( 21 )	/* [31:16]2ndC  [15:0]2ndB  */
// Factory Gyro offset
#define	GYRO_FCTRY_OFST			( 22 )	/* [31:16] Y    [15:0] X */
#define	GYRO_FCTRY_OFSTZ		( 23 )	/* [31:16] -    [15:0] Z */

#define MAT0_CKSM				( 63 )

//==============================================================================
// ES1 INF2 Memory Map
//==============================================================================
#define	FROMCALIB1				( 33 )
#define	FROMCALIB2				( 34 )
#define	FROMCALIB3				( 35 )
#define	FROMCALIB4				( 36 )
#define	FROMCALIB5				( 37 )
#define	FROMCALIB6				( 38 )
#define	FROMCALIB1_USER			( 60 )
#define	FROMCALIB2_USER			( 61 )

//==============================================================================
//DMA
//==============================================================================
#define		HallFilterD_HXDAZ1				0x0138
#define		HallFilterD_HYDAZ1				0x0188

#define		HALL_RAM_X_COMMON				0x01C8
#define			HALL_RAM_HXOFF					0x01C8
#define			HALL_RAM_HXOFF1					0x01CC
#define			HALL_RAM_HXOUT0					0x01D0
#define			HALL_RAM_HXOUT1					0x01D4
#define			HALL_RAM_SINDX0					0x01D8
#define			HALL_RAM_HXLOP					0x01DC
#define			HALL_RAM_SINDX1					0x01E0
#define			HALL_RAM_HALL_X_OUT				0x01E4
#define		HALL_RAM_HALL_SwitchX			0x0214

#define		HALL_RAM_Y_COMMON				0x0218
#define			HALL_RAM_HYOFF					0x0218
#define			HALL_RAM_HYOFF1					0x021C
#define			HALL_RAM_HYOUT0					0x0220
#define			HALL_RAM_HYOUT1					0x0224
#define			HALL_RAM_SINDY0					0x0228
#define			HALL_RAM_HYLOP					0x022C
#define			HALL_RAM_SINDY1					0x0230
#define			HALL_RAM_HALL_Y_OUT				0x0234
#define		HALL_RAM_HALL_SwitchY			0x0264


#define		HALL_RAM_COMMON					0x0268
				//  HallFilterDelay.h HALL_RAM_COMMON_t
#define			HALL_RAM_HXIDAT					0x0268
#define			HALL_RAM_HYIDAT					0x026C

#define			HALL_RAM_GYROX_OUT				0x0270
#define			HALL_RAM_GYROY_OUT				0x0274

#define		GyroFilterDelayX_delay3_2		0x03D8
#define		GyroFilterDelayX_GXH1Z2				0x03DC
#define		GyroFilterDelayY_delay3_2		0x0400
#define		GyroFilterDelayY_GYH1Z2				0x0404

#define		GYRO_RAM_X						0x0418
				// GyroFilterDelay.h GYRO_RAM_t
#define			GYRO_RAM_GYROX_OFFSET			0x0418
#define			GYRO_RAM_GX2X4XF_IN				0x041C
#define			GYRO_RAM_GX2X4XF_OUT			0x0420
#define			GYRO_RAM_GXFAST					0x0424
#define			GYRO_RAM_GXSLOW					0x0428
#define			GYRO_RAM_GYROX_G1OUT			0x042C
#define			GYRO_RAM_GYROX_G2OUT			0x0430
#define			GYRO_RAM_GYROX_G3OUT			0x0434
#define			GYRO_RAM_GYROX_OUT				0x0438
#define		GYRO_RAM_Y						0x043C
				// GyroFilterDelay.h GYRO_RAM_t
#define			GYRO_RAM_GYROY_OFFSET			0x043C
#define			GYRO_RAM_GY2X4XF_IN				0x0440
#define			GYRO_RAM_GY2X4XF_OUT			0x0444
#define			GYRO_RAM_GYFAST					0x0448
#define			GYRO_RAM_GYSLOW					0x044C
#define			GYRO_RAM_GYROY_G1OUT			0x0450
#define			GYRO_RAM_GYROY_G2OUT			0x0454
#define			GYRO_RAM_GYROY_G3OUT			0x0458
#define			GYRO_RAM_GYROY_OUT				0x045C
#define		GYRO_RAM_COMMON					0x0460
				// GyroFilterDelay.h GYRO_RAM_COMMON_t
#define			GYRO_RAM_GX_ADIDAT				0x0460
#define			GYRO_RAM_GY_ADIDAT				0x0464
#define			GYRO_RAM_SINDX					0x0468
#define			GYRO_RAM_SINDY					0x046C
#define			GYRO_RAM_GXLENSZ				0x0470
#define			GYRO_RAM_GYLENSZ				0x0474
#define			GYRO_RAM_GXOX_OUT				0x0478
#define			GYRO_RAM_GYOX_OUT				0x047C
#define			GYRO_RAM_GXOFFZ					0x0480
#define			GYRO_RAM_GYOFFZ					0x0484
#define			GYRO_RAM_LIMITX					0x0488
#define			GYRO_RAM_LIMITY					0x048C
#define			GYRO_RAM_GYROX_AFCnt			0x0490
#define			GYRO_RAM_GYROY_AFCnt			0x0494
#define			GYRO_RAM_GYRO_Switch			0x0498			// 1Byte
#define			GYRO_RAM_GYRO_AF_Switch			0x0499		// 1Byte

#define		StMeasureFunc					0x04B8
				// MeasureFilter.h	MeasureFunction_Type
#define			StMeasFunc_SiSampleNum			0x04B8
#define			StMeasFunc_SiSampleMax			0x04BC

#define		StMeasureFunc_MFA				0x04C0
#define			StMeasFunc_MFA_SiMax1			0x04C0
#define			StMeasFunc_MFA_SiMin1			0x04C4
#define			StMeasFunc_MFA_UiAmp1			0x04C8
#define			StMeasFunc_MFA_LLiIntegral1		0x04D0
#define			StMeasFunc_MFA_LLiAbsInteg1		0x04D8
#define			StMeasFunc_MFA_PiMeasureRam1	0x04E0

#define			StMeasFunc_MFA_SiPolarity1		0x04E4
#define			StMeasFunc_MFA_SiThresh1		0x04E8
#define			StMeasFunc_MFA_UiLengthA1		0x04EC
#define			StMeasFunc_MFA_UiLengthB1		0x04F0
#define			StMeasFunc_MFA_LLiLengthA1		0x04F8
#define			StMeasFunc_MFA_LLiLengthB1		0x0500


#define		StMeasureFunc_MFB				0x0508
#define			StMeasFunc_MFB_SiMax2			0x0508
#define			StMeasFunc_MFB_SiMin2			0x050C
#define			StMeasFunc_MFB_UiAmp2			0x0510
#define			StMeasFunc_MFB_LLiIntegral2		0x0518
#define			StMeasFunc_MFB_LLiAbsInteg2		0x0520
#define			StMeasFunc_MFB_PiMeasureRam2	0x0528

#define			StMeasFunc_MFB_SiPolarity2		0x052C
#define			StMeasFunc_MFB_SiThresh2		0x0530
#define			StMeasFunc_MFB_UiLengthA2		0x0534
#define			StMeasFunc_MFB_UiLengthB2		0x0538
#define			StMeasFunc_MFB_LLiLengthA2		0x0540
#define			StMeasFunc_MFB_LLiLengthB2		0x0548


#define		MeasureFilterA_Delay			0x0550
				// MeasureFilter.h	MeasureFilter_Delay_Type
#define			MeasureFilterA_Delay_z11		0x0550
#define			MeasureFilterA_Delay_z12		0x0554
#define			MeasureFilterA_Delay_z21		0x0558
#define			MeasureFilterA_Delay_z22		0x055C

#define		MeasureFilterB_Delay			0x0560
				// MeasureFilter.h	MeasureFilter_Delay_Type
#define			MeasureFilterB_Delay_z11		0x0560
#define			MeasureFilterB_Delay_z12		0x0564
#define			MeasureFilterB_Delay_z21		0x0568
#define			MeasureFilterB_Delay_z22		0x056C

#define		SinWaveC						0x0570
#define			SinWaveC_Pt						0x0570
#define			SinWaveC_Regsiter				0x0574
//#define			SinWaveC_SignFlag				0x0578

#define		SinWave							0x057C
				// SinGenerator.h SinWave_t
#define			SinWave_Offset					0x057C
#define			SinWave_Phase					0x0580
#define			SinWave_Gain					0x0584
#define			SinWave_Output					0x0588
#define			SinWave_OutAddr					0x058C
#define		CosWave							0x0590
				// SinGenerator.h SinWave_t
#define			CosWave_Offset					0x0590
#define			CosWave_Phase					0x0594
#define			CosWave_Gain					0x0598
#define			CosWave_Output					0x059C
#define			CosWave_OutAddr					0x05A0

#define		WaitTimerData					0x05A4
				// CommonLibrary.h  WaitTimer_Type
#define			WaitTimerData_UiWaitCounter		0x05A4
#define			WaitTimerData_UiTargetCount		0x05A8

#define		PanTilt_DMA						0x05B8
#define			PanTilt_DMA_ScTpdSts			0x05C4

//#ifdef	SEL_SHIFT_COR
#define			GyroRAM_Z_GYRO_OFFSET		0x05F8 

#define			GYRO_ZRAM_GZ_ADIDAT			0x061C
#define			GYRO_ZRAM_GZOFFZ			0x0628

#define		AcclFilDly_X					0x0640
#define		AcclFilDly_Y					0x0670
#define		AcclFilDly_Z					0x06A0

#define		AcclRAM_X						0x06D0
#define			ACCLRAM_X_AC_ADIDAT			0x06D0
#define			ACCLRAM_X_AC_OFFSET			0x06D4

#define		AcclRAM_Y						0x06FC
#define			ACCLRAM_Y_AC_ADIDAT			0x06FC
#define			ACCLRAM_Y_AC_OFFSET			0x0700

#define		AcclRAM_Z						0x0728
#define			ACCLRAM_Z_AC_ADIDAT			0x0728
#define			ACCLRAM_Z_AC_OFFSET			0x072C
//#endif	//SEL_SHIFT_COR



#define		SinWaveEx					0x0754
#define			SinWaveEx_Enable			0x0754
#define			SinWaveEx_GainM				0x0758
#define		CosWaveEx					0x075C
#define			CosWaveEx_Enable			0x075C
#define			CosWaveEx_GainM				0x0760


#define		FRA_DMA							0x0798
#define			FRA_DMA_Control				0x079C
//#define			FRA_DMA_DeciCount		0x07A0
#define			FRA_DMA_DeciShift			0x07A8
#define			FRA_DMA_InputData			0x07B0
#define			FRA_DMA_OutputData			0x07B4

#define			FRA_DMA_Gain				0x0808
#define			FRA_DMA_Phase				0x080C

#define			GYRO_RAM_GYRO_Switch_LS		0x0960			// 1Byte

#define		INF0_DATA						0x1A00
#define		INF1_DATA						0x1B00
#define		INF2_DATA						0x1C00

//==============================================================================
//DMB
//==============================================================================
#define		SiVerNum						0x8000
#define		SiCalID							0x8004
#define		SiActInf						0x8008

#define		StCalibrationData				0x00AC
				// Calibration.h  CalibrationData_Type
#define			StCaliData_UsCalibrationStatus	0x00AC
#define			StCaliData_UiHallBias_X			0x00B0
#define			StCaliData_UiHallOffsetI_X		0x00B4
#define			StCaliData_UiHallOffsetO_X		0x00B8
#define			StCaliData_UiHallBias_Y			0x00BC
#define			StCaliData_UiHallOffsetI_Y		0x00C0
#define			StCaliData_UiHallOffsetO_Y		0x00C4
#define			StCaliData_UiHallBias_Z			0x00C8
#define			StCaliData_UiHallOffsetI_Z		0x00CC
#define			StCaliData_UiHallOffsetO_Z		0x00D0
#define			StCaliData_SiHallMax_X			0x00D4
#define			StCaliData_SiHallMin_X			0x00D8
#define			StCaliData_SiHallMax_Y			0x00DC
#define			StCaliData_SiHallMin_Y			0x00E0
#define			StCaliData_SiHallMax_Z			0x00E4
#define			StCaliData_SiHallMin_Z			0x00E8
#define			StCaliData_SiLoopGain_X			0x00EC
#define			StCaliData_SiLoopGain_Y			0x00F0
#define			StCaliData_SiLoopGain_Z			0x00F4
#define			StCaliData_SiLens_Offset_X		0x00F8
#define			StCaliData_SiLens_Offset_Y		0x00FC
#define			StCaliData_SiLens_Offset_Z		0x0100
#define			StCaliData_SiOptCenter_Offset_X	0x0104
#define			StCaliData_SiOptCenter_Offset_Y	0x0108
#define			StCaliData_SiGyroOffset_X		0x010C
#define			StCaliData_SiGyroOffset_Y		0x0110
#define			StCaliData_SiGyroOffset_Z		0x0114
#define			StCaliData_SiGyroGain_X			0x0118
#define			StCaliData_SiGyroGain_Y			0x011C

#define		HallFilterCoeffX				0x8010
				// HallFilterCoeff.h  DM_HFC_t
#define			HallFilterCoeffX_HXIGAIN		0x8010
#define			HallFilterCoeffX_GYROXOUTGAIN	0x8014
#define			HallFilterCoeffX_HXOFFGAIN		0x8018

#define			HallFilterCoeffX_hxiab			0x801C
#define			HallFilterCoeffX_hxiac			0x8020
#define			HallFilterCoeffX_hxiaa			0x8024
#define			HallFilterCoeffX_hxibb			0x8028
#define			HallFilterCoeffX_hxibc			0x802C
#define			HallFilterCoeffX_hxiba			0x8030
#define			HallFilterCoeffX_hxdab			0x8034
#define			HallFilterCoeffX_hxdac			0x8038
#define			HallFilterCoeffX_hxdaa			0x803C
#define			HallFilterCoeffX_hxdbb			0x8040
#define			HallFilterCoeffX_hxdbc			0x8044
#define			HallFilterCoeffX_hxdba			0x8048
#define			HallFilterCoeffX_hxdcc			0x804C
#define			HallFilterCoeffX_hxdcb			0x8050
#define			HallFilterCoeffX_hxdca			0x8054
#define			HallFilterCoeffX_hxpgain0		0x8058
#define			HallFilterCoeffX_hxigain0		0x805C
#define			HallFilterCoeffX_hxdgain0		0x8060
#define			HallFilterCoeffX_hxpgain1		0x8064
#define			HallFilterCoeffX_hxigain1		0x8068
#define			HallFilterCoeffX_hxdgain1		0x806C
#define			HallFilterCoeffX_hxgain0		0x8070
#define			HallFilterCoeffX_hxgain1		0x8074

#define			HallFilterCoeffX_hxsb			0x8078
#define			HallFilterCoeffX_hxsc			0x807C
#define			HallFilterCoeffX_hxsa			0x8080

#define			HallFilterCoeffX_hxob			0x8084
#define			HallFilterCoeffX_hxoc			0x8088
#define			HallFilterCoeffX_hxod			0x808C
#define			HallFilterCoeffX_hxoe			0x8090
#define			HallFilterCoeffX_hxoa			0x8094
#define			HallFilterCoeffX_hxpb			0x8098
#define			HallFilterCoeffX_hxpc			0x809C
#define			HallFilterCoeffX_hxpd			0x80A0
#define			HallFilterCoeffX_hxpe			0x80A4
#define			HallFilterCoeffX_hxpa			0x80A8

#define		HallFilterCoeffY				0x80AC
				// HallFilterCoeff.h  DM_HFC_t
#define			HallFilterCoeffY_HYIGAIN		0x80AC
#define			HallFilterCoeffY_GYROYOUTGAIN	0x80B0
#define			HallFilterCoeffY_HYOFFGAIN		0x80B4
                                                
#define			HallFilterCoeffY_hyiab			0x80B8
#define			HallFilterCoeffY_hyiac			0x80BC
#define			HallFilterCoeffY_hyiaa			0x80C0
#define			HallFilterCoeffY_hyibb			0x80C4
#define			HallFilterCoeffY_hyibc			0x80C8
#define			HallFilterCoeffY_hyiba			0x80CC
#define			HallFilterCoeffY_hydab			0x80D0
#define			HallFilterCoeffY_hydac			0x80D4
#define			HallFilterCoeffY_hydaa			0x80D8
#define			HallFilterCoeffY_hydbb			0x80DC
#define			HallFilterCoeffY_hydbc			0x80E0
#define			HallFilterCoeffY_hydba			0x80E4
#define			HallFilterCoeffY_hydcc			0x80E8
#define			HallFilterCoeffY_hydcb			0x80EC
#define			HallFilterCoeffY_hydca			0x80F0
#define			HallFilterCoeffY_hypgain0		0x80F4
#define			HallFilterCoeffY_hyigain0		0x80F8
#define			HallFilterCoeffY_hydgain0		0x80FC
#define			HallFilterCoeffY_hypgain1		0x8100
#define			HallFilterCoeffY_hyigain1		0x8104
#define			HallFilterCoeffY_hydgain1		0x8108
#define			HallFilterCoeffY_hygain0		0x810C
#define			HallFilterCoeffY_hygain1		0x8110
                                                
#define			HallFilterCoeffY_hysb			0x8114
#define			HallFilterCoeffY_hysc			0x8118
#define			HallFilterCoeffY_hysa			0x811C
                                                
#define			HallFilterCoeffY_hyob			0x8120
#define			HallFilterCoeffY_hyoc			0x8124
#define			HallFilterCoeffY_hyod			0x8128
#define			HallFilterCoeffY_hyoe			0x812C
#define			HallFilterCoeffY_hyoa			0x8130
#define			HallFilterCoeffY_hypb			0x8134
#define			HallFilterCoeffY_hypc			0x8138
#define			HallFilterCoeffY_hypd			0x813C
#define			HallFilterCoeffY_hype			0x8140
#define			HallFilterCoeffY_hypa			0x8144

#define		HallFilterLimitX				0x8148
#define		HallFilterLimitY				0x8160
#define		HallFilterShiftX				0x8178
#define		HallFilterShiftY				0x817E

#define		HallCurrentLimitX				0x818C
#define		HallCurrentLimitY				0x8190

#define		HF_MIXING						0x8194
#define			HF_hx45x						0x8194			// HallMixingCoeff.hx45x
#define			HF_hx45y						0x8198			// HallMixingCoeff.hx45y
#define			HF_hy45y						0x819C			// HallMixingCoeff.hy45y
#define			HF_hy45x						0x81A0			// HallMixingCoeff.hy45x
#define			HF_ShiftX						0x81A4

#define		HAL_LN_CORRECT					0x81A8
#define			HAL_LN_COEFAX					0x81A8		// HallLinearCorrAX.zone_coef[6]
#define			HAL_LN_COEFBX					0x81B4		// HallLinearCorrBX.zone_coef[6]
#define			HAL_LN_ZONEX					0x81C0		// HallLinearZoneX.zone_area[5]
#define			HAL_LN_COEFAY					0x81CA		// HallLinearCorrAY.zone_coef[6]
#define			HAL_LN_COEFBY					0x81D6		// HallLinearCorrBY.zone_coef[6]
#define			HAL_LN_ZONEY					0x81E2		// HallLinearZoneY.zone_area[5]

#define		GyroFilterTableX				0x83C8
				// GyroFilterCoeff.h  DM_GFC_t
#define			GyroFilterTableX_gx45x			0x83C8
#define			GyroFilterTableX_gx45y			0x83CC
#define			GyroFilterTableX_gxgyro			0x83D0
#define			GyroFilterTableX_gxsengen		0x83D4

#define			GyroFilterTableX_gxl1b			0x83D8
#define			GyroFilterTableX_gxl1c			0x83DC
#define			GyroFilterTableX_gxl1a			0x83E0
#define			GyroFilterTableX_gxl2b			0x83E4
#define			GyroFilterTableX_gxl2c			0x83E8
#define			GyroFilterTableX_gxl2a			0x83EC
#define			GyroFilterTableX_gxigain		0x83F0

#define			GyroFilterTableX_gxh1b			0x83F4
#define			GyroFilterTableX_gxh1c			0x83F8
#define			GyroFilterTableX_gxh1a			0x83FC
#define			GyroFilterTableX_gxk1b			0x8400
#define			GyroFilterTableX_gxk1c			0x8404
#define			GyroFilterTableX_gxk1a			0x8408
#define			GyroFilterTableX_gxgain			0x840C

#define			GyroFilterTableX_gxzoom			0x8410
#define			GyroFilterTableX_gxlenz			0x8414
#define			GyroFilterTableX_gxt2b			0x8418
#define			GyroFilterTableX_gxt2c			0x841C
#define			GyroFilterTableX_gxt2a			0x8420

#define			GyroFilterTableX_afzoom			0x8424

#define		GyroFilterTableY				0x8428
				// GyroFilterCoeff.h  DM_GFC_t
#define			GyroFilterTableY_gy45y			0x8428
#define			GyroFilterTableY_gy45x			0x842C
#define			GyroFilterTableY_gygyro			0x8430
#define			GyroFilterTableY_gysengen		0x8434

#define			GyroFilterTableY_gyl1b			0x8438
#define			GyroFilterTableY_gyl1c			0x843C
#define			GyroFilterTableY_gyl1a			0x8440
#define			GyroFilterTableY_gyl2b			0x8444
#define			GyroFilterTableY_gyl2c			0x8448
#define			GyroFilterTableY_gyl2a			0x844C
#define			GyroFilterTableY_gyigain		0x8450

#define			GyroFilterTableY_gyh1b			0x8454
#define			GyroFilterTableY_gyh1c			0x8458
#define			GyroFilterTableY_gyh1a			0x845C
#define			GyroFilterTableY_gyk1b			0x8460
#define			GyroFilterTableY_gyk1c			0x8464
#define			GyroFilterTableY_gyk1a			0x8468
#define			GyroFilterTableY_gygain			0x846C

#define			GyroFilterTableY_gyzoom			0x8470
#define			GyroFilterTableY_gylenz			0x8474
#define			GyroFilterTableY_gyt2b			0x8478
#define			GyroFilterTableY_gyt2c			0x847C
#define			GyroFilterTableY_gyt2a			0x8480
                                                
#define			GyroFilterTableY_afzoom			0x8484

#define		GyroFilterTableZ				0x87E0
				// GyroFilterCoeff.h  DM_GFC_t
#define			GyroFilterTableZ_gy45y			0x87E0
#define			GyroFilterTableZ_gy45x			0x87E4
#define			GyroFilterTableZ_gzgyro			0x87E8
#define			GyroFilterTableZ_gzsengen		0x87EC

#define			GyroFilterTableZ_gzl1b			0x87F0
#define			GyroFilterTableZ_gzl1c			0x87F4
#define			GyroFilterTableZ_gzl1a			0x87F8
#define			GyroFilterTableZ_gzl2b			0x87FC
#define			GyroFilterTableZ_gzl2c			0x8800
#define			GyroFilterTableZ_gzl2a			0x8804
#define			GyroFilterTableZ_gzigain		0x8808

#define			GyroFilterTableZ_gzh1b			0x880C
#define			GyroFilterTableZ_gzh1c			0x8810
#define			GyroFilterTableZ_gzh1a			0x8814
#define			GyroFilterTableZ_gzk1b			0x8818
#define			GyroFilterTableZ_gzk1c			0x881C
#define			GyroFilterTableZ_gzk1a			0x8820
#define			GyroFilterTableZ_gzgain			0x8824

#define			GyroFilterTableZ_gzzoom			0x8828
#define			GyroFilterTableZ_gzlenz			0x882C
#define			GyroFilterTableZ_gzt2b			0x8830
#define			GyroFilterTableZ_gzt2c			0x8834
#define			GyroFilterTableZ_gzt2a			0x8838
                                                
#define			GyroFilterTableZ_afzoom			0x883C

#define			GyroFilterTableX_LS_gxzoom		0x88E0
#define			GyroFilterTableY_LS_gyzoom		0x8918

#define		GyroFilterShiftX				0x8490
				// GyroFilterCoeff.h  GF_Shift_t
#define			RG_GX2X4XF						0x8490
#define			RG_GX2X4XB						0x8491
#define			RG_GXOX							0x8492
#define			RG_GXAFZ						0x8493

#define		GyroFilterShiftY				0x8494
				// GyroFilterCoeff.h  GF_Shift_t
#define			RG_GY2X4XF						0x8494
#define			RG_GY2X4XB						0x8495
#define			RG_GYOX							0x8496
#define			RG_GYAFZ						0x8497

#define		GyroFilterShiftZ				0x8898
				// GyroFilterCoeff.h  GF_Shift_t
#define			RG_GZ2X4XF						0x8898
#define			RG_GZ2X4XB						0x8899
#define			RG_GZOX							0x889A
#define			RG_GZAFZ						0x889B

#define		MeasureFilterA_Coeff			0x84D8
				// MeasureFilter.h  MeasureFilter_Type
#define			MeasureFilterA_Coeff_b1			0x84D8
#define			MeasureFilterA_Coeff_c1			0x84DC
#define			MeasureFilterA_Coeff_a1			0x84E0
#define			MeasureFilterA_Coeff_b2			0x84E4
#define			MeasureFilterA_Coeff_c2			0x84E8
#define			MeasureFilterA_Coeff_a2			0x84EC

#define		MeasureFilterB_Coeff			0x84F0
				// MeasureFilter.h  MeasureFilter_Type
#define			MeasureFilterB_Coeff_b1			0x84F0
#define			MeasureFilterB_Coeff_c1			0x84F4
#define			MeasureFilterB_Coeff_a1			0x84F8
#define			MeasureFilterB_Coeff_b2			0x84FC
#define			MeasureFilterB_Coeff_c2			0x8500
#define			MeasureFilterB_Coeff_a2			0x8504


#define		GF_LimitX						0x8734
				// GyroFilterCoeff.h  GF_Limit_t
#define			GF_LimitX_H2LMT					0x8734
#define			GF_LimitX_JLIMT					0x8738
#define			GF_LimitX_HLIMT					0x873C

#define		GF_LimitY						0x8740
				// GyroFilterCoeff.h  GF_Limit_t
#define			GF_LimitY_H2LMT					0x8740
#define			GF_LimitY_JLIMT					0x8744
#define			GF_LimitY_HLIMT					0x8748

//#ifdef	SEL_SHIFT_COR
#define		AcclMatTab_X					0x8850
#define		AcclMatTab_Y					0x885C
#define		AcclMatTab_Z					0x8868
//#endif	//SEL_SHIFT_COR

#define		Accl45Filter					0x87C0
#define			Accl45Filter_XAmain				0x87C0
#define			Accl45Filter_XAsub				0x87C4
#define			Accl45Filter_YAmain				0x87C8
#define			Accl45Filter_YAsub				0x87CC

#define		MotionSensor_Sel				0x8B20
#define			MS_SEL_GX0						0x8B20
#define			MS_SEL_GX1						0x8B24
#define			MS_SEL_GY0						0x8B28
#define			MS_SEL_GY1						0x8B2C
#define			MS_SEL_GZ						0x8B30
#define			MS_SEL_AX0						0x8B34
#define			MS_SEL_AX1						0x8B38
#define			MS_SEL_AY0						0x8B3C
#define			MS_SEL_AY1						0x8B40
#define			MS_SEL_AZ						0x8B44

#define			FRA_DMB_C0 					0x8B9C
#define			FRA_DMB_S0 					0x8BA0
#define			FRA_DMB_CN 					0x8BA4
#define			FRA_DMB_SN 					0x8BA8


//==============================================================================
//IO
//==============================================================================
// System Control配置アドレス
#define 		SYSDSP_DSPDIV					0xD00014
#define 		SYSDSP_SOFTRES					0xD0006C
#define 		OSCRSEL							0xD00090	// OSC Frequency 1
#define 		OSCCURSEL						0xD00094	// OSC Frequency 2
#define 		SYSDSP_REMAP					0xD000AC
#define 		SYSDSP_CVER						0xD00100
//#define 		IOPLEVR							0xD00104	// IO port level read
#define			VGAVREF							0xD00280
// A/D D/A interface
#define 		ADDA_FSCNT							0xD01004
#define 		ADDA_FSCTRL							0xD01008
#define 		ADDA_ADDAINT						0xD0100C
#define 		ADDA_ADE							0xD01010
#define 		ADDA_ADAV							0xD01014
#define 		ADDA_ADORDER						0xD01018
#define 		ADDA_EXTEND							0xD0101C
#define 		ADDA_AD0O							0xD01020
#define 		ADDA_AD1O							0xD01024
#define 		ADDA_AD2O							0xD01028
#define 		ADDA_AD3O							0xD0102C

#define 		ADDA_DASELW							0xD01040
#define 		ADDA_DASU							0xD01044
#define 		ADDA_DAHD							0xD01048
#define 		ADDA_DASWAP							0xD0104C
#define 		ADDA_DASEL							0xD01050
#define 		ADDA_DAO							0xD01054

#define 		PERICLKON							0xD00000
#define			OSCCNT								0xD000D4
#define 		FRQTRM								0xD00098
#define 		OSCCKCNT							0xD00108

#define			ROMINFO								0xE050D4

// PWM I/F配置アドレス
#define 		OISDRVFC5							0xD02100
#define 		OISDRVFC6							0xD02104
#define 		OISDRVFC7							0xD02108
#define 		OISDRVFC8							0xD0210C

#define			DRVCH1SEL							0xD02128
#define			DRVCH2SEL							0xD0212C

#define 		OISGAINAM							0xD02190
#define 		OISOFSTAM							0xD02194
#define 		OISGAINBM							0xD02198
#define 		OISOFSTBM							0xD0219C

#define 		AFDRVFC5							0xD02200
#define 		AFDRVFC6							0xD02204

#define 		AFGAINM								0xD02290
#define 		AFSOFSTM							0xD02294

#define			SADR								0xE05080
#define			SADRWPB								0xE05084

/************************************************************************/
/*        Flash access													*/
/************************************************************************/
#define FLASHROM_129		0xE07000	// Flash Memory I/F配置アドレス
#define 		FLASHROM_FLA_RDAT					(FLASHROM_129 + 0x00)
#define 		FLASHROM_FLA_WDAT					(FLASHROM_129 + 0x04)
#define 		FLASHROM_ACSCNT						(FLASHROM_129 + 0x08)
#define 		FLASHROM_FLA_ADR					(FLASHROM_129 + 0x0C)
	#define			USER_MAT				0
	#define			INF_MAT0				1
	#define			INF_MAT1				2
	#define			INF_MAT2				4
	#define			TRIM_MAT				16

#define 		FLASHROM_CMD						(FLASHROM_129 + 0x10)
#define 		FLASHROM_FLAWP						(FLASHROM_129 + 0x14)
#define 		FLASHROM_FLAINT						(FLASHROM_129 + 0x18)
#define 		FLASHROM_FLAMODE					(FLASHROM_129 + 0x1C)
#define 		FLASHROM_TPECPW						(FLASHROM_129 + 0x20)
#define 		FLASHROM_TACC						(FLASHROM_129 + 0x24)

#define 		FLASHROM_ERR_FLA					(FLASHROM_129 + 0x98)
#define 		FLASHROM_RSTB_FLA					(FLASHROM_129 + 0x4CC)
#define 		FLASHROM_UNLK_CODE1					(FLASHROM_129 + 0x554)
#define 		FLASHROM_CLK_FLAON					(FLASHROM_129 + 0x664)
#define 		FLASHROM_UNLK_CODE2					(FLASHROM_129 + 0xAA8)
#define 		FLASHROM_UNLK_CODE3					(FLASHROM_129 + 0xCCC)


#define			AREA_ALL	0	// 1,2,4,8 ALL
#define			AREA_HALL	1	// HALL,GYRO OFFSET,ACCL OFFSET
#define			AREA_GYRO	2	// GYRO GAIN
#define			AREA_CRS	4	// CROSS TALK
#define			AREA_LIN	8	// LINEARITY

#define			CALIB_STATUS

/************************************************************************/
/*        LC898402														*/
/************************************************************************/
#define			EP_IDSEL			0x47
	#define			SLV_E8				0x80
	#define			SLV_E4_REVERSE		0xC1
#define			EP_DAHLO			0x48
#define			EP_DAHLB			0x49
#define			EP_LINEAR_C1		0x54
#define			EP_HGHLMT			0x5E
#define			EP_LOWLMT			0x5F
#define			EP_ANLG				0x61
	#define			STBB_402			0x80
	#define			GSEL_3_9			0x10		// 3.9mV/mT
	#define			GSEL_6_3			0x30		// 6.3mV/mT
	#define			GSEL_10_1			0x00		// 10.1mV/mT
	#define			GSEL_16_2			0x20		// 16.2mV/mT
	#define			GSEL_24_8			0x40		// 24.8mV/mT
	#define			GSEL_39_8			0x60		// 39.9mV/mT
	#define			HLINV				0x08
	#define			DRVINV				0x04
#define			EP_FHLO				0x62
#define			EP_FREE_AREA_START	0x63
#define			EP_HALL_MAX			EP_FREE_AREA_START
#define			EP_HALL_MIN			( EP_FREE_AREA_START + 2 )
#define			EP_AD_OFFSET		( EP_FREE_AREA_START + 4 )
#define			EP_LOOP_GAIN		( EP_FREE_AREA_START + 6 )

#define			ANLG		0x81
	#define			STBB_402		0x80
#define			DAHLO_402	0x8E
#define			DAHLB_402	0x8F
#define			STANDBY_402	0x96
#define			E2WPMS1		0x98
	#define			PMSCODE1		0xE2
#define			E2WPMS2		0x99
	#define			PMSCODE2		0xA0
	#define			SEL_ID			0x08
	#define			SEL_UP			0x04
	#define			SEL_DN			0x02
#define			ADWLCTL		0xE0
#define			FNUM		0xF0
	#define			FNUM_DATA		0xA5
#define			CVER_402	0xF1
	#define			CVER_ES1		0x21
	#define			CVER_ES2		0x22

