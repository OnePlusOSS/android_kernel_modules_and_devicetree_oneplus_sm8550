/**
 * @brief		LC898129 Global declaration for SMA
 *
 * @author		(C) 2020 ON Semiconductor.
 *
 * @file		OisLC898129SMA.h
 * @date		svn:$Date:: 2021-01-29 20:36:10 +0900#$
 * @version	svn:$Revision: 10 $
 * @attention
 **/

//==============================================================================
// Calibration Data Memory Map
//==============================================================================
// Calibration Status for SMA
#define SMA_CALIBRATION_STATUS		( 32 )
// Hall Bias/Offset
#define SMA_HALL_BIAS_OFFSET_X		( 33 )
#define SMA_HALL_BIAS_OFFSET_Y		( 34 )
#define SMA_HALL_BIAS_OFFSET_Z		( 35 )
// Loop gain / lens offset
#define SMA_LOOPGAIN_LENS_OFFSET_X	( 36 )	/* [31:16]X Lens offset	[15:0]X loop gain */
#define SMA_LOOPGAIN_LENS_OFFSET_Y	( 37 )	/* [31:16]Y Lens offset	[15:0]Y loop gain */
#define SMA_LOOPGAIN_LENS_OFFSET_Z	( 38 )	/* [31:16]Z Lens offset	[15:0]Z loop gain */
// Hall amplitude Calibration
#define SMA_HALL_AMPLITUDE_X		( 39 )	/* [31:16]X hall max	[15:0]X hall min */
#define SMA_HALL_AMPLITUDE_Y		( 40 )	/* [31:16]Y hall max	[15:0]Y hall min */
#define SMA_HALL_AMPLITUDE_Z		( 41 )	/* [31:16]Z hall max	[15:0]Z hall min */
// Gyro gain
#define SMA_GYRO_GAIN_Z				( 44 )
#define SMA_GYRO_GAIN_X				( 45 )
#define SMA_GYRO_GAIN_Y				( 46 )
// Liniearity correction
#define SMA_LN_POS1					( 47 )	/* [31:16]Y  [15:0]X */
#define SMA_LN_POS2					( 48 )  /* [31:16]Y  [15:0]X */
#define SMA_LN_POS3					( 49 )  /* [31:16]Y  [15:8]X */
#define SMA_LN_POS4					( 50 )  /* [31:16]Y  [15:0]X */
#define SMA_LN_POS5					( 51 )  /* [31:16]Y  [15:0]X */
#define SMA_LN_POS6					( 52 )  /* [31:16]Y  [15:0]X */
#define SMA_LN_POS7					( 53 )  /* [31:16]Y  [15:8]X */
#define SMA_LN_STEP					( 54 )  /* [31:16]Y  [15:0]X */
// Gyro mixing correction
#define SMA_MIXING_X				( 55 )	/* [31:16]HX45X [15:0]HX45X */
#define SMA_MIXING_Y				( 56 )	/* [31:16]HY45Y [15:0]HY45Y */
#define SMA_MIXING_SFT				( 57 )	/* [31:16]2ndA  [15:8]YSFT [7:0]:XSHT   */
#define SMA_MIXING_2ND				( 58 )	/* [31:16]2ndC  [15:0]2ndB  */
// Factory Gyro offset
#define SMA_GYRO_FCTRY_OFST			( 59 )	/* [31:16] Y    [15:0] X */


#define SMA_BIAS0					( 60 )	/* [31:16]BIAS1 [15:0]BIAS0 */
#define SMA_BIAS2					( 61 )	/* [31:16]BIAS3 [15:0]BIAS2 */
#define SMA_TEMP					( 62 )	/* Ambient temperature */

//==============================================================================
//DMA
//==============================================================================
#define		SmaHallFilterD_HXDAZ1			0x0288
#define		SmaHallFilterD_HYDAZ1			0x02D8
#define		SmaHallFilterD_HZDAZ1			0x0B20

#define		SMAHALL_RAM_X_COMMON			0x0318
#define			SMAHALL_RAM_HXOFF				0x0318
#define			SMAHALL_RAM_HXOFF1				0x031C
#define			SMAHALL_RAM_HXOUT0				0x0320
#define			SMAHALL_RAM_HXOUT1				0x0324
#define			SMAHALL_RAM_SINDX0				0x0328
#define			SMAHALL_RAM_HXLOP				0x032C
#define			SMAHALL_RAM_SINDX1				0x0330
#define			SMAHALL_RAM_HALL_X_OUT			0x0334
#define		SMAHALL_RAM_HALL_SwitchX		0x0364

#define		SMAHALL_RAM_Y_COMMON			0x0368
#define			SMAHALL_RAM_HYOFF				0x0368
#define			SMAHALL_RAM_HYOFF1				0x036C
#define			SMAHALL_RAM_HYOUT0				0x0370
#define			SMAHALL_RAM_HYOUT1				0x0374
#define			SMAHALL_RAM_SINDY0				0x0378
#define			SMAHALL_RAM_HYLOP				0x037C
#define			SMAHALL_RAM_SINDY1				0x0380
#define			SMAHALL_RAM_HALL_Y_OUT			0x0384
#define		SMAHALL_RAM_HALL_SwitchY		0x03B4


#define		SMAHALL_RAM_COMMON				0x03B8
				//  SmaHallFilterDelay.h SmaHALL_RAM_COMMON_t
#define			SMAHALL_RAM_HXIDAT				0x03B8
#define			SMAHALL_RAM_HYIDAT				0x03BC
#define			SMAHALL_RAM_HZIDAT				0x03C0

#define			SMAHALL_RAM_GYROX_OUT			0x0BB0
#define			SMAHALL_RAM_GYROY_OUT			0x0BB4
#define			SMAHALL_RAM_GYROZ_OUT			0x0BB8

#define		SMAHALL_RAM_Z_COMMON			0x0B60
#define			SMAHALL_RAM_HZOFF				0x0B60
#define			SMAHALL_RAM_HZOFF1				0x0B64
#define			SMAHALL_RAM_HZOUT0				0x0B68
#define			SMAHALL_RAM_HZOUT1				0x0B6C
#define			SMAHALL_RAM_SINDZ0				0x0B70
#define			SMAHALL_RAM_HZLOP				0x0B74
#define			SMAHALL_RAM_SINDZ1				0x0B78
#define			SMAHALL_RAM_HALL_Z_OUT			0x0B7C
#define		SMAHALL_RAM_HALL_SwitchZ		0x0BAC

#define		SMA_RAM						0x0820
#define			SMA_RAM_MONI_OUT0			0x0860
#define			SMA_RAM_MONI_OUT1			0x0864
#define			SMA_RAM_MONI_OUT2			0x0868
#define			SMA_RAM_MONI_OUT3			0x086C

#define			SMA_KELVIN					0x0870
#define			SMA_RAM_SW					0x087C
#define			SMA_BIAS					0x0880

#define 		SMA_RAM_OUTBIAS0			0x088C
#define 		SMA_RAM_OUTBIAS1			0x0890
#define 		SMA_RAM_OUTBIAS2			0x0894
#define 		SMA_RAM_OUTBIAS3			0x0898
#define 		SMA_RAM_GAP0				0x089C
#define 		SMA_RAM_GAP1				0x08A0
#define 		SMA_RAM_GAP2				0x08A4
#define 		SMA_RAM_GAP3				0x08A8
#define 		SMA_RAM_ADIDAT0				0x08AC
#define 		SMA_RAM_ADIDAT1				0x08B0
#define 		SMA_RAM_ADIDAT2				0x08B4
#define 		SMA_RAM_ADIDAT3				0x08B8
#define			SMA_RAM_LPF0				0x08BC
#define			SMA_RAM_LPF1				0x08C0
#define			SMA_RAM_LPF2				0x08C4
#define			SMA_RAM_LPF3				0x08C8
#define			SMA_RAM_LPF4				0x08CC

//==============================================================================
//DMB
//==============================================================================
#define		StSmaCalibrationData			0x0034
				// Calibration.h  CalibrationData_Type
#define			StSmaCaliData_UsCalibrationStatus	0x0034
#define			StSmaCaliData_UiHallBias_X			0x0038
#define			StSmaCaliData_UiHallOffsetI_X		0x003C
#define			StSmaCaliData_UiHallOffsetO_X		0x0040
#define			StSmaCaliData_UiHallBias_Y			0x0044
#define			StSmaCaliData_UiHallOffsetI_Y		0x0048
#define			StSmaCaliData_UiHallOffsetO_Y		0x004C
#define			StSmaCaliData_UiHallBias_Z			0x0050
#define			StSmaCaliData_UiHallOffsetI_Z		0x0054
#define			StSmaCaliData_UiHallOffsetO_Z		0x0058
#define			StSmaCaliData_SiHallMax_X			0x005C
#define			StSmaCaliData_SiHallMin_X			0x0060
#define			StSmaCaliData_SiHallMax_Y			0x0064
#define			StSmaCaliData_SiHallMin_Y			0x0068
#define			StSmaCaliData_SiHallMax_Z			0x006C
#define			StSmaCaliData_SiHallMin_Z			0x0070
#define			StSmaCaliData_SiLoopGain_X			0x0074
#define			StSmaCaliData_SiLoopGain_Y			0x0078
#define			StSmaCaliData_SiLoopGain_Z			0x007C
#define			StSmaCaliData_SiLens_Offset_X		0x0080
#define			StSmaCaliData_SiLens_Offset_Y		0x0084
#define			StSmaCaliData_SiLens_Offset_Z		0x0088
#define			StSmaCaliData_SiOptCenter_Offset_X	0x008C
#define			StSmaCaliData_SiOptCenter_Offset_Y	0x0090
#define			StSmaCaliData_SiGyroOffset_X		0x0094
#define			StSmaCaliData_SiGyroOffset_Y		0x0098
#define			StSmaCaliData_SiGyroOffset_Z		0x009C
#define			StSmaCaliData_SiGyroGain_X			0x00A0
#define			StSmaCaliData_SiGyroGain_Y			0x00A4

#define		SmaHallFilterCoeffX				0x81F0
				// SmaHallFilterCoeff.h  SmaDM_HFC_t
#define			SmaHallFilterCoeffX_HXIGAIN			0x81F0
#define			SmaHallFilterCoeffX_GYROXOUTGAIN	0x81F4
#define			SmaHallFilterCoeffX_HXOFFGAIN		0x81F8

#define			SmaHallFilterCoeffX_hxiab		0x81FC
#define			SmaHallFilterCoeffX_hxiac		0x8200
#define			SmaHallFilterCoeffX_hxiaa		0x8204
#define			SmaHallFilterCoeffX_hxibb		0x8208
#define			SmaHallFilterCoeffX_hxibc		0x820C
#define			SmaHallFilterCoeffX_hxiba		0x8210
#define			SmaHallFilterCoeffX_hxdab		0x8214
#define			SmaHallFilterCoeffX_hxdac		0x8218
#define			SmaHallFilterCoeffX_hxdaa		0x821C
#define			SmaHallFilterCoeffX_hxdbb		0x8220
#define			SmaHallFilterCoeffX_hxdbc		0x8224
#define			SmaHallFilterCoeffX_hxdba		0x8228
#define			SmaHallFilterCoeffX_hxdcc		0x822C
#define			SmaHallFilterCoeffX_hxdcb		0x8230
#define			SmaHallFilterCoeffX_hxdca		0x8234
#define			SmaHallFilterCoeffX_hxpgain0	0x8238
#define			SmaHallFilterCoeffX_hxigain0	0x823C
#define			SmaHallFilterCoeffX_hxdgain0	0x8240
#define			SmaHallFilterCoeffX_hxpgain1	0x8244
#define			SmaHallFilterCoeffX_hxigain1	0x8248
#define			SmaHallFilterCoeffX_hxdgain1	0x824C
#define			SmaHallFilterCoeffX_hxgain0		0x8250
#define			SmaHallFilterCoeffX_hxgain1		0x8254

#define			SmaHallFilterCoeffX_hxsb		0x8258
#define			SmaHallFilterCoeffX_hxsc		0x825C
#define			SmaHallFilterCoeffX_hxsa		0x8260

#define			SmaHallFilterCoeffX_hxob		0x8264
#define			SmaHallFilterCoeffX_hxoc		0x8268
#define			SmaHallFilterCoeffX_hxod		0x826C
#define			SmaHallFilterCoeffX_hxoe		0x8270
#define			SmaHallFilterCoeffX_hxoa		0x8274
#define			SmaHallFilterCoeffX_hxpb		0x8278
#define			SmaHallFilterCoeffX_hxpc		0x827C
#define			SmaHallFilterCoeffX_hxpd		0x8280
#define			SmaHallFilterCoeffX_hxpe		0x8284
#define			SmaHallFilterCoeffX_hxpa		0x8288

#define		SmaHallFilterCoeffY				0x828C
				// SmaHallFilterCoeff.h  SmaDM_HFC_t
#define			SmaHallFilterCoeffY_HYIGAIN			0x828C
#define			SmaHallFilterCoeffY_GYROYOUTGAIN	0x8290
#define			SmaHallFilterCoeffY_HYOFFGAIN		0x8294

#define			SmaHallFilterCoeffY_hyiab		0x8298
#define			SmaHallFilterCoeffY_hyiac		0x829C
#define			SmaHallFilterCoeffY_hyiaa		0x82A0
#define			SmaHallFilterCoeffY_hyibb		0x82A4
#define			SmaHallFilterCoeffY_hyibc		0x82A8
#define			SmaHallFilterCoeffY_hyiba		0x82AC
#define			SmaHallFilterCoeffY_hydab		0x82B0
#define			SmaHallFilterCoeffY_hydac		0x82B4
#define			SmaHallFilterCoeffY_hydaa		0x82B8
#define			SmaHallFilterCoeffY_hydbb		0x82BC
#define			SmaHallFilterCoeffY_hydbc		0x82C0
#define			SmaHallFilterCoeffY_hydba		0x82C4
#define			SmaHallFilterCoeffY_hydcc		0x82C8
#define			SmaHallFilterCoeffY_hydcb		0x82CC
#define			SmaHallFilterCoeffY_hydca		0x82D0
#define			SmaHallFilterCoeffY_hypgain0	0x82D4
#define			SmaHallFilterCoeffY_hyigain0	0x82D8
#define			SmaHallFilterCoeffY_hydgain0	0x82DC
#define			SmaHallFilterCoeffY_hypgain1	0x82E0
#define			SmaHallFilterCoeffY_hyigain1	0x82E4
#define			SmaHallFilterCoeffY_hydgain1	0x82E8
#define			SmaHallFilterCoeffY_hygain0		0x82EC
#define			SmaHallFilterCoeffY_hygain1		0x82F0

#define			SmaHallFilterCoeffY_hysb		0x82F4
#define			SmaHallFilterCoeffY_hysc		0x82F8
#define			SmaHallFilterCoeffY_hysa		0x82FC

#define			SmaHallFilterCoeffY_hyob		0x8300
#define			SmaHallFilterCoeffY_hyoc		0x8304
#define			SmaHallFilterCoeffY_hyod		0x8308
#define			SmaHallFilterCoeffY_hyoe		0x830C
#define			SmaHallFilterCoeffY_hyoa		0x8310
#define			SmaHallFilterCoeffY_hypb		0x8314
#define			SmaHallFilterCoeffY_hypc		0x8318
#define			SmaHallFilterCoeffY_hypd		0x831C
#define			SmaHallFilterCoeffY_hype		0x8320
#define			SmaHallFilterCoeffY_hypa		0x8324

#define		SmaHallFilterLimitX				0x8328
#define		SmaHallFilterLimitY				0x8340
#define		SmaHallFilterShiftX				0x8358
#define		SmaHallFilterShiftY				0x835E

#define		SmaHF_MIXING					0x836C
#define			SmaHF_hx45x						0x836C			// SmaHallMixingCoeff.hx45x
#define			SmaHF_hx45y						0x8370			// SmaHallMixingCoeff.hx45y
#define			SmaHF_hy45y						0x8374			// SmaHallMixingCoeff.hy45y
#define			SmaHF_hy45x						0x8378			// SmaHallMixingCoeff.hy45x
#define			SmaHF_ShiftX					0x837C

#define		SmaHAL_LN_CORRECT				0x8380
#define			SmaHAL_LN_COEFAX				0x8380	// SmaHallLinearCorrAX.zone_coef[6]
#define			SmaHAL_LN_COEFBX				0x838C	// SmaHallLinearCorrBX.zone_coef[6]
#define			SmaHAL_LN_ZONEX					0x8398		// SmaHallLinearZoneX.zone_area[5]
#define			SmaHAL_LN_COEFAY				0x83A2	// SmaHallLinearCorrAY.zone_coef[6]
#define			SmaHAL_LN_COEFBY				0x83AE	// SmaHallLinearCorrBY.zone_coef[6]
#define			SmaHAL_LN_ZONEY					0x83BA		// SmaHallLinearZoneY.zone_area[5]

#define		SMA_COEFF						0x8BAC
#define			SMA_COEFF_TGAIN0BIAS			0x8C80
#define			SMA_COEFF_TGAIN1BIAS			0x8C84
#define			SMA_COEFF_TGAIN2BIAS			0x8C88
#define			SMA_COEFF_TGAIN3BIAS			0x8C8C

