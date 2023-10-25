/**
 * @brief		FRA measurement command for LC898129
 *
 * @author		(C) 2019 ON Semiconductor.
 *
 * @file		OisFRA.c
 * @date		svn:$Date:: 2021-01-27 10:02:50 +0900#$
 * @version	svn:$Revision: 8 $
 * @attention
 **/

//**************************
//	Include Header File
//**************************
#define		__OISFRA__

//#include	<math.h>
#include	"Ois.h"
#include	"OisFRA.h"
#include    <linux/kernel.h>

#define		VOLT_VREF	(1400.0f)

//****************************************************
//	CUSTOMER NECESSARY CREATING LIST
//****************************************************
/* for I2C communication */
extern	UINT_8 RamWrite32A( UINT_16, UINT_32 );
extern 	UINT_8 RamRead32A( UINT_16, void * );
/* for Wait timer [Need to adjust for your system] */
extern	void WitTim( UINT_16 );

//**************************
//	External Function Prototype
//**************************
extern void	SetSineWave(   UINT_8 , UINT_8 );
extern void	SetSinWavGenInt( void );
//extern void	SetTransDataAdr( UINT_16, UINT_32  ) ;
extern void	MeasureWait( void ) ;
extern void	ClrMesFil( void ) ;
extern void	SetWaitTime( UINT_16 ) ;

#ifdef ACT_THROUGH_CLOSE
UINT_32 BackupParameter[30];
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/* function name    : SetThroughParameter 		                                         */
/* input parameter  :                                                                    */
/* output parameter :                                                                    */
/* comment          : Setup throgh filter		                                         */
/*                                                                            2018.01.18 */
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
void SetThroughParameter(UINT_8	UcDirSel )
{
	if( UcDirSel == X_DIR ) {
		BackupParameter[29] = X_DIR;
		RamRead32A( HallFilterCoeffX_hxdgain0, &BackupParameter[0]);
		RamRead32A( HallFilterCoeffX_hxpgain0, &BackupParameter[1]);
		RamRead32A( HallFilterCoeffX_hxpgain1, &BackupParameter[2]);
		RamRead32A( HallFilterCoeffX_hxigain0, &BackupParameter[3]);
		RamRead32A( HallFilterCoeffX_hxgain0, &BackupParameter[4]);	
		RamRead32A( HallFilterShiftX, &BackupParameter[5]);	
		RamRead32A( (HallFilterShiftX+4), &BackupParameter[6]);	
		RamRead32A( HallFilterCoeffX_hxsa, &BackupParameter[7]);
		RamRead32A( HallFilterCoeffX_hxsb, &BackupParameter[8]);
		RamRead32A( HallFilterCoeffX_hxsc, &BackupParameter[9]);
		RamRead32A( HallFilterCoeffX_hxoa, &BackupParameter[10]);
		RamRead32A( HallFilterCoeffX_hxob, &BackupParameter[11]);
		RamRead32A( HallFilterCoeffX_hxoc, &BackupParameter[12]);
		RamRead32A( HallFilterCoeffX_hxod, &BackupParameter[13]);
		RamRead32A( HallFilterCoeffX_hxoe, &BackupParameter[14]);
		RamRead32A( HallFilterCoeffX_hxpa, &BackupParameter[15]);
		RamRead32A( HallFilterCoeffX_hxpb, &BackupParameter[16]);
		RamRead32A( HallFilterCoeffX_hxpc, &BackupParameter[17]);
		RamRead32A( HallFilterCoeffX_hxpd, &BackupParameter[18]);
		RamRead32A( HallFilterCoeffX_hxpe, &BackupParameter[19]);

		RamWrite32A( HallFilterCoeffX_hxdgain0, 0x00000000);	//RAMW32	80EC	00000000
		RamWrite32A( HallFilterCoeffX_hxpgain0, 0x7fffffff);	//RAMW32	80D8	7fffffff
		RamWrite32A( HallFilterCoeffX_hxpgain1, 0x7fffffff);	//RAMW32	80E4	7fffffff
		RamWrite32A( HallFilterCoeffX_hxigain0, 0x00000000);	//RAMW32	80E8	00000000
		RamWrite32A( HallFilterCoeffX_hxgain0, 0x7fffffff);		//RAMW32	80F0	7fffffff
		RamWrite32A( HallFilterShiftX, 0x00000000);				//RAMW32	81F8	00000000
		RamWrite32A( (HallFilterShiftX+4), 0x00000000);			//RAMW32	81FC	00000000
		RamWrite32A( HallFilterCoeffX_hxsa, 0x7fffffff);		//RAMW32	8100	7fffffff
		RamWrite32A( HallFilterCoeffX_hxsb, 0x00000000);		//RAMW32	80F8	00000000
		RamWrite32A( HallFilterCoeffX_hxsc, 0x00000000);		//RAMW32	80FC	00000000
		RamWrite32A( HallFilterCoeffX_hxoa, 0x7fffffff);		//RAMW32	8114	7fffffff
		RamWrite32A( HallFilterCoeffX_hxob, 0x00000000);		//RAMW32	8104	00000000
		RamWrite32A( HallFilterCoeffX_hxoc, 0x00000000);		//RAMW32	8108	00000000
		RamWrite32A( HallFilterCoeffX_hxod, 0x00000000);		//RAMW32	810C	00000000
		RamWrite32A( HallFilterCoeffX_hxoe, 0x00000000);		//RAMW32	8110	00000000
		RamWrite32A( HallFilterCoeffX_hxpa, 0x7fffffff);		//RAMW32	8128	7fffffff
		RamWrite32A( HallFilterCoeffX_hxpb, 0x00000000);		//RAMW32	8118	00000000
		RamWrite32A( HallFilterCoeffX_hxpc, 0x00000000);		//RAMW32	811C	00000000
		RamWrite32A( HallFilterCoeffX_hxpd, 0x00000000);		//RAMW32	8120	00000000
		RamWrite32A( HallFilterCoeffX_hxpe, 0x00000000);		//RAMW32	8124	00000000
	}else if( UcDirSel == Y_DIR ){
		BackupParameter[29] = Y_DIR;
		RamRead32A( HallFilterCoeffY_hydgain0, &BackupParameter[0]);
		RamRead32A( HallFilterCoeffY_hypgain0, &BackupParameter[1]);
		RamRead32A( HallFilterCoeffY_hypgain1, &BackupParameter[2]);
		RamRead32A( HallFilterCoeffY_hyigain0, &BackupParameter[3]);
		RamRead32A( HallFilterCoeffY_hygain0, &BackupParameter[4]);
		RamRead32A( HallFilterShiftY, &BackupParameter[5]);	
		RamRead32A( HallFilterCoeffY_hysa, &BackupParameter[6]);
		RamRead32A( HallFilterCoeffY_hysb, &BackupParameter[7]);
		RamRead32A( HallFilterCoeffY_hysc, &BackupParameter[8]);
		RamRead32A( HallFilterCoeffY_hyoa, &BackupParameter[9]);
		RamRead32A( HallFilterCoeffY_hyob, &BackupParameter[10]);
		RamRead32A( HallFilterCoeffY_hyoc, &BackupParameter[11]);
		RamRead32A( HallFilterCoeffY_hyod, &BackupParameter[12]);
		RamRead32A( HallFilterCoeffY_hyoe, &BackupParameter[13]);
		RamRead32A( HallFilterCoeffY_hypa, &BackupParameter[14]);
		RamRead32A( HallFilterCoeffY_hypb, &BackupParameter[15]);
		RamRead32A( HallFilterCoeffY_hypc, &BackupParameter[16]);
		RamRead32A( HallFilterCoeffY_hypd, &BackupParameter[17]);
		RamRead32A( HallFilterCoeffY_hype, &BackupParameter[18]);
		
		RamWrite32A( HallFilterCoeffY_hydgain0, 0x00000000);	//RAMW32	8188	00000000
		RamWrite32A( HallFilterCoeffY_hypgain0, 0x7fffffff);	//RAMW32	8174	7fffffff
		RamWrite32A( HallFilterCoeffY_hypgain1, 0x7fffffff);	//RAMW32	8180	7fffffff
		RamWrite32A( HallFilterCoeffY_hyigain0, 0x00000000);	//RAMW32	8184	00000000
		RamWrite32A( HallFilterCoeffY_hygain0, 0x7fffffff);		//RAMW32	818C	7fffffff
		RamWrite32A( HallFilterShiftY, 0x00000000);				//RAMW32	8200	00000000
		RamWrite32A( HallFilterCoeffY_hysa, 0x7fffffff);		//RAMW32	819C	7fffffff
		RamWrite32A( HallFilterCoeffY_hysb, 0x00000000);		//RAMW32	8194	00000000
		RamWrite32A( HallFilterCoeffY_hysc, 0x00000000);		//RAMW32	8198	00000000
		RamWrite32A( HallFilterCoeffY_hyoa, 0x7fffffff);		//RAMW32	81B0	7fffffff
		RamWrite32A( HallFilterCoeffY_hyob, 0x00000000);		//RAMW32	81A0	00000000
		RamWrite32A( HallFilterCoeffY_hyoc, 0x00000000);		//RAMW32	81A4	00000000
		RamWrite32A( HallFilterCoeffY_hyod, 0x00000000);		//RAMW32	81A8	00000000
		RamWrite32A( HallFilterCoeffY_hyoe, 0x00000000);		//RAMW32	81AC	00000000
		RamWrite32A( HallFilterCoeffY_hypa, 0x7fffffff);		//RAMW32	81C4	7fffffff
		RamWrite32A( HallFilterCoeffY_hypb, 0x00000000);		//RAMW32	81B4	00000000
		RamWrite32A( HallFilterCoeffY_hypc, 0x00000000);		//RAMW32	81B8	00000000
		RamWrite32A( HallFilterCoeffY_hypd, 0x00000000);		//RAMW32	81BC	00000000
		RamWrite32A( HallFilterCoeffY_hype, 0x00000000);		//RAMW32	81C0	00000000
	}else if( UcDirSel == Z_DIR ){
		}
	}

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/* function name    : ResetThroughParameter 		                                     */
/* input parameter  :                                                                    */
/* output parameter :                                                                    */
/* comment          : DFTの係数発生    			                                         */
/*                                                                            2018.01.18 */
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
void ResetThroughParameter(void)
{
	if( BackupParameter[29] == X_DIR ) {
		RamWrite32A( HallFilterCoeffX_hxdgain0, BackupParameter[0]);
		RamWrite32A( HallFilterCoeffX_hxpgain0, BackupParameter[1]);
		RamWrite32A( HallFilterCoeffX_hxpgain1, BackupParameter[2]);
		RamWrite32A( HallFilterCoeffX_hxigain0, BackupParameter[3]);
		RamWrite32A( HallFilterCoeffX_hxgain0, BackupParameter[4]);	
		RamWrite32A( HallFilterShiftX, BackupParameter[5]);	
		RamWrite32A( (HallFilterShiftX+4), BackupParameter[6]);	
		RamWrite32A( HallFilterCoeffX_hxsa, BackupParameter[7]);
		RamWrite32A( HallFilterCoeffX_hxsb, BackupParameter[8]);
		RamWrite32A( HallFilterCoeffX_hxsc, BackupParameter[9]);
		RamWrite32A( HallFilterCoeffX_hxoa, BackupParameter[10]);
		RamWrite32A( HallFilterCoeffX_hxob, BackupParameter[11]);
		RamWrite32A( HallFilterCoeffX_hxoc, BackupParameter[12]);
		RamWrite32A( HallFilterCoeffX_hxod, BackupParameter[13]);
		RamWrite32A( HallFilterCoeffX_hxoe, BackupParameter[14]);
		RamWrite32A( HallFilterCoeffX_hxpa, BackupParameter[15]);
		RamWrite32A( HallFilterCoeffX_hxpb, BackupParameter[16]);
		RamWrite32A( HallFilterCoeffX_hxpc, BackupParameter[17]);
		RamWrite32A( HallFilterCoeffX_hxpd, BackupParameter[18]);
		RamWrite32A( HallFilterCoeffX_hxpe, BackupParameter[19]);
	}else if( BackupParameter[29] == Y_DIR ){
		RamWrite32A( HallFilterCoeffY_hydgain0, BackupParameter[0]);
		RamWrite32A( HallFilterCoeffY_hypgain0, BackupParameter[1]);
		RamWrite32A( HallFilterCoeffY_hypgain1, BackupParameter[2]);
		RamWrite32A( HallFilterCoeffY_hyigain0, BackupParameter[3]);
		RamWrite32A( HallFilterCoeffY_hygain0, BackupParameter[4]);
		RamWrite32A( HallFilterShiftY, BackupParameter[5]);	
		RamWrite32A( HallFilterCoeffY_hysa, BackupParameter[6]);
		RamWrite32A( HallFilterCoeffY_hysb, BackupParameter[7]);
		RamWrite32A( HallFilterCoeffY_hysc, BackupParameter[8]);
		RamWrite32A( HallFilterCoeffY_hyoa, BackupParameter[9]);
		RamWrite32A( HallFilterCoeffY_hyob, BackupParameter[10]);
		RamWrite32A( HallFilterCoeffY_hyoc, BackupParameter[11]);
		RamWrite32A( HallFilterCoeffY_hyod, BackupParameter[12]);
		RamWrite32A( HallFilterCoeffY_hyoe, BackupParameter[13]);
		RamWrite32A( HallFilterCoeffY_hypa, BackupParameter[14]);
		RamWrite32A( HallFilterCoeffY_hypb, BackupParameter[15]);
		RamWrite32A( HallFilterCoeffY_hypc, BackupParameter[16]);
		RamWrite32A( HallFilterCoeffY_hypd, BackupParameter[17]);
		RamWrite32A( HallFilterCoeffY_hype, BackupParameter[18]);
	}else if( BackupParameter[29] == Z_DIR ){
	}
}
#endif
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/* function name    : CoeffGenerate  		                                             */
/* input parameter  :                                                                    */
/* output parameter :                                                                    */
/* comment          : DFTの係数発生    			                                         */
/*                                                                            2018.01.18 */
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#define	Q31 	( 0x7FFFFFFF )
#define	Q23 	( 0x007FFFFF )
#define	Q21 	( 0x001FFFFF )
#define	PAI 	( 3.14159265358979323846 )
#define N 		( 2048 )
int     nDivision;

void CoeffGenerate( double fc )
{
	double  df, fs; 
	int     point; //, C0, S0, CN, SN;
	double  theta;			// theta = 2*Pi*f/Fs
#if 1 //2.5kHzだとGUIがついてこれない。
	if 		( fc > 40 ){ nDivision = 0; fs = (FS_FREQ    ); }
	else if ( fc > 20 ){ nDivision = 1; fs = (FS_FREQ / 2); }
	else if ( fc > 10 ){ nDivision = 2; fs = (FS_FREQ / 4); }
	else if ( fc >  5 ){ nDivision = 3; fs = (FS_FREQ / 8); }
	else 			   { nDivision = 4; fs = (FS_FREQ /16); }
#else
	if 		( fc > 40 ){ nDivision = 0; fs = (FS_FREQ    ); }
	else if ( fc > 5 ) { nDivision = 1; fs = (FS_FREQ / 2); }
	else 			   { nDivision = 2; fs = (FS_FREQ / 4); }
#endif	
	
	//***** 取得した周波数テーブルから判定ポイントと判定thetaの算出 *****
	df = fs / (double)N;									// FFTの1ポイント当たりの周波数
	point = (int)(fc / df + 0.5);							// 判定ポイントの算出
	theta = 2.0 * PAI * (double)point * df / fs;			// 判定ポイントでの位相の算出

//	C0 = (int)((double)Q31 * cos(theta) + 0.5);
//	S0 = (int)((double)Q31 * sin(theta) + 0.5);
//	CN = (int)((double)Q31 * cos(((double)N - 1.0) * theta) + 0.5);
//	SN = (int)((double)Q31 * sin(((double)N - 1.0) * theta) + 0.5);

	//RamWrite32A( FRA_DMA_DeciShift, nDivision );	
	//RamWrite32A( FRA_DMB_C0, C0 ) ;
	//RamWrite32A( FRA_DMB_S0, S0 ) ;
	//RamWrite32A( FRA_DMB_CN, CN ) ;
	//RamWrite32A( FRA_DMB_SN, SN ) ;

//CAM_ERR(CAM_OIS, "freq=%f, 0x%08X, 0x%08X, 0x%08X, 0x%08X,\n", fc, C0, S0, CN, SN);
}



//********************************************************************************
// Function Name 	: Freq_Convert
// Retun Value		: Phase Step Value
// Argment Value	: Frequency
// Explanation		: Convert Frequency
// History			: First edition
//********************************************************************************
UINT_32	Freq_Convert( float SfFreq )
{
	UINT_32	UlPhsStep ;

	UlPhsStep	= ( UINT_32 )( ( SfFreq * ( float )0x100000000 / FS_FREQ + 0.5F ) / 2.0F ) ;

	return( UlPhsStep ) ;
}



//********************************************************************************
// Function Name 	: MesStart_FRA_Single
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Measure start setting Function
// History			: First edition
//********************************************************************************
void	MesStart_FRA_Single( UINT_8	UcDirSel )
{
	float			SfTmp ;
	INT_32	GainQ23, PhaseQ21 ;
	float	ScaledAmp = 1.0 ;
	UINT_32	UlReadVal ;	

	SetSinWavGenInt() ;
	// Change Frequency 
	RamWrite32A( SinWave_Offset,	Freq_Convert( StFRAParam.StHostCom.SfFrqCom.SfFltVal ) ) ;		// Freq Setting = Freq * 80000000h / Fs

	SfTmp	= StFRAParam.StHostCom.SfAmpCom.SfFltVal / VOLT_VREF * ScaledAmp ;						// AVDD 2800mV / 2 = 1400mV
	RamWrite32A( SinWave_Gain,		( UINT_32 )( ( float )0x7FFFFFFF * SfTmp ) ) ;					// Set Sine Wave Gain
	CAM_ERR(CAM_OIS, " SfTmp = %f\n", SfTmp ) ;

	if( !LS_SS_Sel ) {
		if ( StFRAParam.StHostCom.UcAvgCycl == 10) 	{  		// Actuator Through
			if( UcDirSel == X_DIR ) {
				RtnCenBit( XAXS_SRV_OFF ) ;
				RamWrite32A( SinWave_OutAddr	,	(UINT_32)SMAHALL_RAM_SINDX1 ) ;								// Set Sine Wave Input RAM
				// Set parameter and input/output address
				RamWrite32A( FRA_DMA_InputData, SMAHALL_RAM_SINDX1 ) ;
				RamWrite32A( FRA_DMA_OutputData, SMAHALL_RAM_HXIDAT ) ;
			}else if( UcDirSel == Y_DIR ){
				RtnCenBit( YAXS_SRV_OFF ) ;
				RamWrite32A( SinWave_OutAddr	,	(UINT_32)SMAHALL_RAM_SINDY1 ) ;								// Set Sine Wave Input RAM
				// Set parameter and input/output address
				RamWrite32A( FRA_DMA_InputData, SMAHALL_RAM_SINDY1 ) ;
				RamWrite32A( FRA_DMA_OutputData, SMAHALL_RAM_HYIDAT ) ;
			}else if( UcDirSel == Z_DIR ){
				RtnCenBit( ZAXS_SRV_OFF ) ;
				RamWrite32A( SinWave_OutAddr	,	(UINT_32)SMAHALL_RAM_SINDZ1 ) ;								// Set Sine Wave Input RAM
				// Set parameter and input/output address
				RamWrite32A( FRA_DMA_InputData, SMAHALL_RAM_SINDZ1 ) ;
				RamWrite32A( FRA_DMA_OutputData, SMAHALL_RAM_HZIDAT ) ;
			}
		} else {
			if( UcDirSel == X_DIR ) {
				RamWrite32A( SinWave_OutAddr	,	(UINT_32)SMAHALL_RAM_HXOFF1 ) ;								// Set Sine Wave Input RAM
				// Set parameter and input/output address
#ifdef CLOSED_RESPONSE
				RamWrite32A( FRA_DMA_InputData, SMAHALL_RAM_HXOFF1 ) ;
#else
				RamWrite32A( FRA_DMA_InputData, SmaHallFilterD_HXDAZ1 ) ;
#endif
				RamWrite32A( FRA_DMA_OutputData, SMAHALL_RAM_HXOUT0 ) ;
			} else if( UcDirSel == Y_DIR ) {
				RamWrite32A( SinWave_OutAddr	,	(UINT_32)SMAHALL_RAM_HYOFF1 ) ;								// Set Sine Wave Input RAM
				// Set parameter and input/output address
#ifdef CLOSED_RESPONSE
				RamWrite32A( FRA_DMA_InputData, SMAHALL_RAM_HYOFF1  ) ;
#else
				RamWrite32A( FRA_DMA_InputData, SmaHallFilterD_HYDAZ1 ) ;
#endif			
				RamWrite32A( FRA_DMA_OutputData, SMAHALL_RAM_HYOUT0 ) ;

			}else if( UcDirSel == Z_DIR ){
				RamWrite32A( SinWave_OutAddr	,	(UINT_32)SMAHALL_RAM_HZOFF1 ) ;								// Set Sine Wave Input RAM
				// Set parameter and input/output address
#ifdef CLOSED_RESPONSE
				RamWrite32A( FRA_DMA_InputData, SMAHALL_RAM_HZOFF1  ) ;
#else
				RamWrite32A( FRA_DMA_InputData, SmaHallFilterD_HZDAZ1 ) ;
#endif
				RamWrite32A( FRA_DMA_OutputData, SMAHALL_RAM_HZOUT0 ) ;
			}
		}
	} else {
		if ( StFRAParam.StHostCom.UcAvgCycl == 10) 	{  		// Actuator Through
			if( UcDirSel == X_DIR ) {
				RtnCen_LS( YONLY_ON ) ;
				RamWrite32A( SinWave_OutAddr	,	(UINT_32)HALL_RAM_SINDX1 ) ;								// Set Sine Wave Input RAM
				// Set parameter and input/output address
				RamWrite32A( FRA_DMA_InputData, HALL_RAM_SINDX1 ) ;
				RamWrite32A( FRA_DMA_OutputData, HALL_RAM_HXIDAT ) ;
			}else if( UcDirSel == Y_DIR ){
				RtnCen_LS( XONLY_ON ) ;
				RamWrite32A( SinWave_OutAddr	,	(UINT_32)HALL_RAM_SINDY1 ) ;								// Set Sine Wave Input RAM
				// Set parameter and input/output address
				RamWrite32A( FRA_DMA_InputData, HALL_RAM_SINDY1 ) ;
				RamWrite32A( FRA_DMA_OutputData, HALL_RAM_HYIDAT ) ;
			}
		} else {
			if( UcDirSel == X_DIR ) {
				RamWrite32A( SinWave_OutAddr	,	(UINT_32)HALL_RAM_HXOFF1 ) ;								// Set Sine Wave Input RAM
				// Set parameter and input/output address
#ifdef CLOSED_RESPONSE
				RamWrite32A( FRA_DMA_InputData, HALL_RAM_HXOFF1 ) ;
#else
				RamWrite32A( FRA_DMA_InputData, HallFilterD_HXDAZ1 ) ;
#endif
				RamWrite32A( FRA_DMA_OutputData, HALL_RAM_HXOUT0 ) ;
			} else if( UcDirSel == Y_DIR ) {
				RamWrite32A( SinWave_OutAddr	,	(UINT_32)HALL_RAM_HYOFF1 ) ;								// Set Sine Wave Input RAM
				// Set parameter and input/output address
#ifdef CLOSED_RESPONSE
				RamWrite32A( FRA_DMA_InputData, HALL_RAM_HYOFF1  ) ;
#else
				RamWrite32A( FRA_DMA_InputData, HallFilterD_HYDAZ1 ) ;
#endif			
				RamWrite32A( FRA_DMA_OutputData, HALL_RAM_HYOUT0 ) ;
			}
		}
	}
	RamWrite32A( SinWaveC_Regsiter	,	0x00000001 ) ;												// Sine Wave Start

	CoeffGenerate( StFRAParam.StHostCom.SfFrqCom.SfFltVal );
//	WitTim(10);
	// Start to measure
	RamWrite32A( FRA_DMA_Control,1 ) ;

	if (nDivision == 0)	WitTim(100);
	if (nDivision == 1)	WitTim(200);
	if (nDivision == 2)	WitTim(400);
	if (nDivision == 3)	WitTim(800);
	if (nDivision == 4)	WitTim(1600);
	do{
		WitTim(10);
		RamRead32A( FRA_DMA_Control	, &UlReadVal ) ;	
	}while (UlReadVal == 1);
	// Read answer
	RamRead32A( FRA_DMA_Gain	, &GainQ23 ) ;		// Gain
	RamRead32A( FRA_DMA_Phase	, &PhaseQ21 ) ;		// Phase
	StFRAParam.StMesRslt.SfGainAvg = (float)GainQ23 / Q23; //0x007FFFFF;
	StFRAParam.StMesRslt.SfPhaseAvg = (float)PhaseQ21 / Q21; //0x001FFFFF;	

	CAM_ERR(CAM_OIS, "Phase %f deg : Gain %f dB\n", StFRAParam.StMesRslt.SfPhaseAvg, StFRAParam.StMesRslt.SfGainAvg );
}



//********************************************************************************
// Function Name 	: MesStart_FRA_Continue
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Continue Measurement Function
// History			: First edition
//********************************************************************************
void	MesStart_FRA_Continue( void )
{
	INT_32	GainQ23, PhaseQ21 ;
	UINT_32	UlReadVal ;	

	// Change Frequency 
	RamWrite32A( SinWave_Offset,	Freq_Convert( StFRAParam.StHostCom.SfFrqCom.SfFltVal ) ) ;
	// Set parameter
	CoeffGenerate( StFRAParam.StHostCom.SfFrqCom.SfFltVal );
//	WitTim(10)
	// Start to measure
	RamWrite32A( FRA_DMA_Control,1 ) ;									// Integral Value Clear
	if (nDivision == 0)	WitTim(100);
	if (nDivision == 1)	WitTim(200);
	if (nDivision == 2)	WitTim(400);
	if (nDivision == 3)	WitTim(800);
	if (nDivision == 4)	WitTim(1600);
	do{
		WitTim(10);	
		RamRead32A( FRA_DMA_Control	, &UlReadVal ) ;	
	}while (UlReadVal == 1);
	// Read answer
	RamRead32A( FRA_DMA_Gain	, &GainQ23 ) ;		// Gain
	RamRead32A( FRA_DMA_Phase	, &PhaseQ21 ) ;		// Phase
	StFRAParam.StMesRslt.SfGainAvg = (float)GainQ23 / Q23;
	StFRAParam.StMesRslt.SfPhaseAvg = (float)PhaseQ21 / Q21;	

	CAM_ERR(CAM_OIS, "Phase %f deg : Gain %f dB\n", StFRAParam.StMesRslt.SfPhaseAvg, StFRAParam.StMesRslt.SfGainAvg );
}



//********************************************************************************
// Function Name 	: MesEnd_FRA_Sweep
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Stop Measurement Function
// History			: First edition
//********************************************************************************
void	MesEnd_FRA_Sweep( void )
{
	// Stop Sine Wave
	RamWrite32A( SinWaveC_Regsiter,		0x00000000 ) ;												// Sine Wave Stop
	RamWrite32A( SinWave_OutAddr,	( UINT_32 )0x00000000 ) ;									// Set Sine Wave Input RAM

	if ( StFRAParam.StHostCom.UcAvgCycl == 10) 	{  		// Actuator Through
#ifdef ACT_THROUGH_CLOSE
		ResetThroughParameter( );
#else
		if( !LS_SS_Sel ) {
			RtnCenBit( ALL_SRV_ON ) ;
		} else {
			RtnCen_LS( 0 ) ;
		}
#endif
	}

	if( !LS_SS_Sel ) {
		RamWrite32A( SMAHALL_RAM_SINDX0,	0x00000000 ) ;		// DelayRam Clear
		RamWrite32A( SMAHALL_RAM_SINDY0,	0x00000000 ) ;		// DelayRam Clear
		RamWrite32A( SMAHALL_RAM_SINDZ0,	0x00000000 ) ;		// DelayRam Clear

		RamWrite32A( SMAHALL_RAM_SINDX1,	0x00000000 ) ;		// DelayRam Clear
		RamWrite32A( SMAHALL_RAM_SINDY1,	0x00000000 ) ;		// DelayRam Clear
		RamWrite32A( SMAHALL_RAM_SINDZ1,	0x00000000 ) ;		// DelayRam Clear
	} else {
		RamWrite32A( HALL_RAM_SINDX0,	0x00000000 ) ;			// DelayRam Clear
		RamWrite32A( HALL_RAM_SINDY0,	0x00000000 ) ;			// DelayRam Clear

		RamWrite32A( HALL_RAM_SINDX1,	0x00000000 ) ;			// DelayRam Clear
		RamWrite32A( HALL_RAM_SINDY1,	0x00000000 ) ;			// DelayRam Clear
	}
}
