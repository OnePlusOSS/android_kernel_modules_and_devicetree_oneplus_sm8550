/**
 * @brief		OIS system command for LC898129
 *
 * @author		(C) 2020 ON Semiconductor.
 *
 * @file		OisCmd.c
 * @date		svn:$Date:: 2021-01-28 22:03:14 +0900#$
 * @version		svn:$Revision: 9 $
 * @attention
 **/

//**************************
//	Include Header File
//**************************
#define		__OISCMD__

//#include	<stdlib.h>	/* use for abs() */
//#include	<math.h>	/* use for sqrt() */
#include <linux/kernel.h>
#include	"Ois.h"

/* Actuator calibration parameters */
#include 	"Calibration_TAH1120.h"		// Hall Monitor
#include	"Calibration_LS.h"

//****************************************************
//	CUSTOMER NECESSARY CREATING LIST
//****************************************************
/* for I2C communication */
extern	UINT_8 RamWrite32A( UINT_16 addr, UINT_32 data);
extern	UINT_8 RamRead32A( UINT_16 addr, void * data );
extern	UINT_8 IOWrite32A( UINT_32 IOadrs, UINT_32 IOdata );
/* for Wait timer [Need to adjust for your system] */
extern	void WitTim( UINT_16 );
/* for Set I2C Slave address  [Need to adjust for your system] */
extern	void setI2cSlvAddr( UINT_8, UINT_8 );

//**************************
//	Global Variable
//**************************

//**************************
//	extern  Function LIST
//**************************

//**************************
//	define
//**************************
#define 	MES_XG1			0						//!< LXG1 Measure Mode
#define 	MES_XG2			1						//!< LXG2 Measure Mode

#define 	TNE 			80						//!< Waiting Time For Movement

/******* Hall calibration Type 1 *******/

#define 	OFFSET_DIV		2						//!< Divide Difference For Offset Step
#define 	TIME_OUT		40						//!< Time Out Count

#define		BIAS_HLMT		(UINT_32)0xBF000000
#define		BIAS_LLMT		(UINT_32)0x20000000
#define		BIAS_HLMT_LS	0xBF
#define		BIAS_LLMT_LS	0x20

// Threshold of osciration amplitude
#define		ULTHDVAL	0x01000000					// Threshold of the hale value

//**************************
//	Local Function Prototype
//**************************
void	IniCmd( void ) ;							//!< Command Execute Process Initial
void	IniPtAve( void ) ;							//!< Average setting
void	MeasureStart( INT_32 , INT_32 , INT_32 ) ;	//!< Measure Start Function
void	MeasureStart2( INT_32 , INT_32 , INT_32 , UINT_16 );	//!< Measure Start 2 Function
void	MeasureWait( void ) ;						//!< Measure Wait
void	MemoryClear( UINT_16 , UINT_16 ) ;			//!< Memory Cloear
void	SetWaitTime( UINT_16 ) ; 					//!< Set Wait Timer

UINT_32	LopGan( UINT_8, ADJ_LOPGAN * );
UINT_8	TneHvc( void );

void	TnePtp_TRIPLE( UINT_8, UINT_8, ADJ_HALL * );
UINT_32	TneCen_TRIPLE( UINT_8, ADJ_HALL *, UINT_8 ) ;		//!< Hall Bias/Offset Tuning
UINT_32	HallAdj_TRIPLE( ADJ_HALL* Ptr );
UINT_32 TneOffIn_TRIPLE ( UINT_8, ADJ_HALL* );
void	TneOff_TRIPLE( UnDwdVal, UINT_8 ) ;					//!< Hall Offset Tuning
void	TneBia_TRIPLE( UnDwdVal, UINT_8, UINT_16 ) ;		//!< Hall Bias Tuning
UINT_32 HallAdj( ADJ_HALL_LS *Ptr, ADJ_LOPGAN *LpPtr ) ;
UINT_32	TneCen( UINT_8, ADJ_HALL_LS *, UINT_8 );				//!< Hall bias/offset tuning
UINT_32	TnePtp ( UINT_8, UINT_8, ADJ_HALL_LS * );
void	TneOff( UnDwdVal, UINT_8 );					//!< Hall offset tuning
void	TneBia( UnDwdVal, UINT_8, UINT_16 );		//!< Hall bias tuning
UINT_32	LopGan_LS( UINT_8, ADJ_LOPGAN * );



void	DacControl( UINT_8, UINT_32, UINT_32 );

#ifdef	NEUTRAL_CENTER_FINE
void	TneFin( void ) ;							//!< Fine tune for natural center offset
void	TneFinX( void ) ;							//!< Fine tune for natural center offset
void	TneFinY( void ) ;							//!< Fine tune for natural center offset
#endif	// NEUTRAL_CENTER_FINE

void	RdHallCalData( void ) ;
void	SetSinWavePara( UINT_8 , UINT_8 ) ;			//!< Sin wave test function
void	SetSineWave( UINT_8 , UINT_8 );
void	SetSinWavGenInt( void );
void	SetTransDataAdr( UINT_16  , UINT_32  ) ;	//!< Hall VC Offset Adjust

void	ClrMesFil( void );
void	MesFil( eFilterMode );

//**************************
//	Const
//**************************

//********************************************************************************
// Function Name 	: MemClr
// Retun Value		: void
// Argment Value	: Clear Target PoINT_32er, Clear Byte Number
// Explanation		: Memory Clear Function
// History			: First edition
//********************************************************************************
void	MemClr( UINT_8	*NcTgtPtr, UINT_16	UsClrSiz )
{
	UINT_16	UsClrIdx ;

	for ( UsClrIdx = 0 ; UsClrIdx < UsClrSiz ; UsClrIdx++ )
	{
		*NcTgtPtr	= 0 ;
		NcTgtPtr++ ;
	}
}

//********************************************************************************
// Function Name 	: HallAdj_TRIPLE
// Retun Value		: Hall Tuning SUCCESS or FAILURE
// Argment Value	: NON
// Explanation		: Hall system auto adjustment function
// History			: First edition 						
//********************************************************************************
UINT_32 HallAdj_TRIPLE( ADJ_HALL* Ptr  )
{
	UINT_32	UlHlxSts, UlHlySts;
	UINT_32	UlHoffINxSts, UlHoffINySts, UlHoffINzSts;
	UINT_32	UlReadVal ;
	
	RtnCen( BOTH_OFF ) ;		// Both OFF

	RamRead32A( SMA_RAM_SW, &UlReadVal );
	UlReadVal &= 0x0000000F;
	RamWrite32A( SMA_RAM_SW, UlReadVal ) ;
	WitTim( TNE ) ;

	DacControl( 0 , HLXBO , Ptr->BiasInit ) ;
	RamWrite32A( StSmaCaliData_UiHallBias_X , Ptr->BiasInit ) ;
	DacControl( 0 , HLXOO , Ptr->XOffsetInit ) ;
	RamWrite32A( StSmaCaliData_UiHallOffsetO_X , Ptr->XOffsetInit ) ;
	DacControl( 0 , HLXOI , Ptr->XOffsetInitIn ) ;
	RamWrite32A( StSmaCaliData_UiHallOffsetI_X , Ptr->XOffsetInitIn ) ;

	DacControl( 0 , HLYBO , Ptr->BiasInit ) ;
	RamWrite32A( StSmaCaliData_UiHallBias_Y , Ptr->BiasInit ) ;
	DacControl( 0 , HLYOO , Ptr->YOffsetInit ) ;
	RamWrite32A( StSmaCaliData_UiHallOffsetO_Y , Ptr->YOffsetInit ) ;
	DacControl( 0 , HLYOI , Ptr->YOffsetInitIn ) ;
	RamWrite32A( StSmaCaliData_UiHallOffsetI_Y , Ptr->YOffsetInitIn ) ;

	DacControl( 0 , HLZBO , Ptr->ZBiasInit ) ;
	RamWrite32A( StSmaCaliData_UiHallBias_Z , Ptr->ZBiasInit) ;
	DacControl( 0 , HLZOO , Ptr->ZOffsetInit ) ;
	RamWrite32A( StSmaCaliData_UiHallOffsetO_Z , Ptr->ZOffsetInit ) ;
	DacControl( 0 , HLZOI , Ptr->ZOffsetInitIn ) ;
	RamWrite32A( StSmaCaliData_UiHallOffsetI_Z , Ptr->ZOffsetInitIn ) ;

	//--------------------------------------
	// Adjustment hall input offset
	//--------------------------------------
	UlHoffINxSts = TneOffIn_TRIPLE( X_DIR, Ptr ) ;
	UlHoffINySts = TneOffIn_TRIPLE( Y_DIR, Ptr ) ;
	UlHoffINzSts = TneOffIn_TRIPLE( Z_DIR, Ptr ) ;
	if( (UlHoffINxSts != 0x02) || (UlHoffINySts != 0x02) || (UlHoffINzSts != 0x02 )){
		return( UlHoffINxSts | UlHoffINySts | UlHoffINzSts );
	}

	//--------------------------------------
	// Adjustment axis before Y
	//--------------------------------------
	UlHlySts = TneCen_TRIPLE( Y_DIR, Ptr , PTP_BEFORE ) ;
	// Y1
	StAdjPar.StHalAdj.UsAdyOff = StAdjPar.StHalAdj.UsHlyCen ;
	RamWrite32A( SMAHALL_RAM_HYOFF,  (UINT_32)((StAdjPar.StHalAdj.UsAdyOff << 16 ) & 0xFFFF0000 )) ;
	// Y2
	StAdjPar.StHalAdj.UsAdzOff = StAdjPar.StHalAdj.UsHlzCen ;
	RamWrite32A( SMAHALL_RAM_HZOFF,  (UINT_32)((StAdjPar.StHalAdj.UsAdzOff << 16 ) & 0xFFFF0000 )) ;
	
//	RtnCen( YONLY_ON ) ;		// Y Servo ON	
CAM_ERR(CAM_OIS, "		( AXIS = %02x , UlHlySts = %08xh ) , \n", Y_DIR , (unsigned int)UlHlySts ) ;

	//--------------------------------------
	// Adjustment axis before X
	//--------------------------------------
	UlHlxSts = TneCen_TRIPLE( X_DIR, Ptr , PTP_BEFORE ) ;
	// X
	StAdjPar.StHalAdj.UsAdxOff = StAdjPar.StHalAdj.UsHlxCen  ;
	RamWrite32A( SMAHALL_RAM_HXOFF,  (UINT_32)((StAdjPar.StHalAdj.UsAdxOff << 16 ) & 0xFFFF0000 )) ;
//	RtnCen( XONLY_ON ) ;		// X Servo ON
CAM_ERR(CAM_OIS, "		( AXIS = %02x , UlHlxSts = %08xh ) , \n", X_DIR , (unsigned int)UlHlxSts ) ;
	
	// Skip second calibration when error occurred
	if( (UlHlxSts != EXE_HXADJ) && (UlHlySts != EXE_HYADJ) ){

		//--------------------------------------
		// Adjustment axis after Y
		//--------------------------------------
		UlHlySts = TneCen_TRIPLE( Y_DIR, Ptr , PTP_AFTER ) ;
		// Y1
		StAdjPar.StHalAdj.UsAdyOff = StAdjPar.StHalAdj.UsHlyCna  ;
		RamWrite32A( SMAHALL_RAM_HYOFF,  (UINT_32)((StAdjPar.StHalAdj.UsAdyOff << 16 ) & 0xFFFF0000 )) ;
		RamWrite32A( StSmaCaliData_SiLens_Offset_Y,  (UINT_32)((StAdjPar.StHalAdj.UsAdyOff << 16 ) & 0xFFFF0000 )) ;		
CAM_ERR(CAM_OIS, "***StAdjPar.StHalAdj.UsAdyOff = %04X\n", StAdjPar.StHalAdj.UsAdyOff);
		// Y2
		StAdjPar.StHalAdj.UsAdzOff = StAdjPar.StHalAdj.UsHlzCna ;
		RamWrite32A( SMAHALL_RAM_HZOFF,  (UINT_32)((StAdjPar.StHalAdj.UsAdzOff << 16 ) & 0xFFFF0000 )) ;		
		RamWrite32A( StSmaCaliData_SiLens_Offset_Z,  (UINT_32)((StAdjPar.StHalAdj.UsAdzOff << 16 ) & 0xFFFF0000 )) ;		
CAM_ERR(CAM_OIS, "***StAdjPar.StHalAdj.UsAdzOff = %04X\n", StAdjPar.StHalAdj.UsAdzOff);

//		RtnCen( YONLY_ON ) ;		// Y Servo ON		
	
		//--------------------------------------
		// Adjustment axis after X
		//--------------------------------------
		UlHlxSts = TneCen_TRIPLE( X_DIR, Ptr , PTP_AFTER ) ;
		// X
		StAdjPar.StHalAdj.UsAdxOff = StAdjPar.StHalAdj.UsHlxCna  ;
		RamWrite32A( SMAHALL_RAM_HXOFF,  (UINT_32)((StAdjPar.StHalAdj.UsAdxOff << 16 ) & 0xFFFF0000 )) ;
		RamWrite32A( StSmaCaliData_SiLens_Offset_X,  (UINT_32)((StAdjPar.StHalAdj.UsAdxOff << 16 ) & 0xFFFF0000 )) ;		
CAM_ERR(CAM_OIS, "***StAdjPar.StHalAdj.UsAdxOff = %04X\n", StAdjPar.StHalAdj.UsAdxOff);
		
		if( ( UlHlySts | UlHlxSts ) == EXE_END ){
//			RtnCen( BOTH_ON ) ;		// Both Servo ON
		}else{
			// Failure after step
			RtnCen( BOTH_OFF ) ;	// Both OFF
		}

	}else{
		// Failure before step
		RtnCen( BOTH_OFF ) ;	// Both OFF		
	}

	//---------------------------------------------------
	// Copy Hall Bias/Offset data to temporary variable
	//---------------------------------------------------
	RamRead32A( StSmaCaliData_UiHallOffsetO_X , &UlReadVal ) ;
	StAdjPar.StHalAdj.UsHlxOffO = (UINT_16)( UlReadVal >> 16 ) ;
CAM_ERR(CAM_OIS, "***StAdjPar.StHalAdj.UsHlxOffO = %04X\n", StAdjPar.StHalAdj.UsHlxOffO);

	RamRead32A( StSmaCaliData_UiHallOffsetI_X , &UlReadVal ) ;
	StAdjPar.StHalAdj.UsHlxOffI = (UINT_16)( UlReadVal >> 16 ) ;
CAM_ERR(CAM_OIS, "***StAdjPar.StHalAdj.UsHlxOffI = %04X\n", StAdjPar.StHalAdj.UsHlxOffI);

	RamRead32A( StSmaCaliData_UiHallBias_X , &UlReadVal ) ;
	StAdjPar.StHalAdj.UsHlxGan = (UINT_16)( UlReadVal >> 16 ) ;
CAM_ERR(CAM_OIS, "***StAdjPar.StHalAdj.UsHlxGan = %04X\n", StAdjPar.StHalAdj.UsHlxGan);

	RamRead32A( StSmaCaliData_UiHallOffsetO_Y , &UlReadVal ) ;
	StAdjPar.StHalAdj.UsHlyOffO = (UINT_16)( UlReadVal >> 16 ) ;
CAM_ERR(CAM_OIS, "***StAdjPar.StHalAdj.UsHlyOffO = %04X\n", StAdjPar.StHalAdj.UsHlyOffO);

	RamRead32A( StSmaCaliData_UiHallOffsetI_Y , &UlReadVal ) ;
	StAdjPar.StHalAdj.UsHlyOffI = (UINT_16)( UlReadVal >> 16 ) ;
CAM_ERR(CAM_OIS, "***StAdjPar.StHalAdj.UsHlyOffI = %04X\n", StAdjPar.StHalAdj.UsHlyOffI);

	RamRead32A( StSmaCaliData_UiHallBias_Y , &UlReadVal ) ;
	StAdjPar.StHalAdj.UsHlyGan = (UINT_16)( UlReadVal >> 16 ) ;
CAM_ERR(CAM_OIS, "***StAdjPar.StHalAdj.UsHlyGan = %04X\n", StAdjPar.StHalAdj.UsHlyGan);
	
	RamRead32A( StSmaCaliData_UiHallOffsetO_Z , &UlReadVal ) ;
	StAdjPar.StHalAdj.UsHlzOffO = (UINT_16)( UlReadVal >> 16 ) ;
CAM_ERR(CAM_OIS, "***StAdjPar.StHalAdj.UsHlzOffO = %04X\n", StAdjPar.StHalAdj.UsHlzOffO);

	RamRead32A( StSmaCaliData_UiHallOffsetI_Z , &UlReadVal ) ;
	StAdjPar.StHalAdj.UsHlzOffI = (UINT_16)( UlReadVal >> 16 ) ;
CAM_ERR(CAM_OIS, "***StAdjPar.StHalAdj.UsHlzOffI = %04X\n", StAdjPar.StHalAdj.UsHlzOffI);
	
	RamRead32A( StSmaCaliData_UiHallBias_Z , &UlReadVal ) ;
	StAdjPar.StHalAdj.UsHlzGan = (UINT_16)( UlReadVal >> 16 ) ;
CAM_ERR(CAM_OIS, "***StAdjPar.StHalAdj.UsHlzGan = %04X\n", StAdjPar.StHalAdj.UsHlzGan);
	
	return ( UlHlySts | UlHlxSts );
}

//********************************************************************************
// Function Name 	: HallAdj
// Retun Value		: Hall Tuning SUCCESS or FAILURE
// Argment Value	: NON
// Explanation		: Hall system auto adjustment function
// History			: First edition 						
//********************************************************************************
UINT_32 HallAdj( ADJ_HALL_LS *Ptr, ADJ_LOPGAN *LpPtr )
{
	UINT_32	UlHlxSts, UlHlySts;

	RtnCen_LS( BOTH_OFF );												// Both OFF
	WitTim( TNE );

	RamWrite32A( HALL_RAM_HXOFF, 0x00000000 ) ;							//< X Offset Clr
	RamWrite32A( HALL_RAM_HYOFF, 0x00000000 ) ;							//< Y Offset Clr
	RamWrite32A( HallFilterCoeffX_hxgain0, LpPtr->Hxgain ) ;
	RamWrite32A( HallFilterCoeffY_hygain0, LpPtr->Hygain ) ;

	RegWrite402( X_402, DAHLB_402, Ptr->BiasInit ) ;
	RegWrite402( X_402, DAHLO_402, Ptr->XOffsetInit ) ;

	RegWrite402( Y_402, DAHLB_402, Ptr->BiasInit ) ;
	RegWrite402( Y_402, DAHLO_402, Ptr->YOffsetInit ) ;

	UlHlySts = TneCen( Y_DIR, Ptr, PTP_BEFORE );
	CAM_ERR(CAM_OIS, "		( AXIS = %02x , UlHlySts = %08xh ) , \n", Y_DIR , (UINT_32)UlHlySts );

	StAdjPar.StHalAdj.UsAdyOff = StAdjPar.StHalAdj.UsHlyCen ;
	RamWrite32A( HALL_RAM_HYOFF,  (UINT_32)((StAdjPar.StHalAdj.UsAdyOff << 16 ) & 0xFFFF0000 )) ;
	RtnCen_LS( YONLY_ON ) ;		// Y Servo ON

	UlHlxSts = TneCen( X_DIR, Ptr, PTP_BEFORE );
	CAM_ERR(CAM_OIS, "		( AXIS = %02x , UlHlxSts = %08xh ) , \n", X_DIR , (UINT_32)UlHlxSts );
	
	StAdjPar.StHalAdj.UsAdxOff = StAdjPar.StHalAdj.UsHlxCen ;
	RamWrite32A( HALL_RAM_HXOFF,  (UINT_32)((StAdjPar.StHalAdj.UsAdxOff << 16 ) & 0xFFFF0000 )) ;
	RtnCen_LS( XONLY_ON ) ;		// X Servo ON

	// Skip second calibration when error occurred
	if( (UlHlxSts != EXE_HXADJ) && (UlHlySts != EXE_HYADJ) ){
		UlHlxSts = TneCen( Y_DIR, Ptr, PTP_AFTER );

		StAdjPar.StHalAdj.UsAdyOff = StAdjPar.StHalAdj.UsHlyCna ;
		RamWrite32A( HALL_RAM_HYOFF,  (UINT_32)((StAdjPar.StHalAdj.UsAdyOff << 16 ) & 0xFFFF0000 )) ;
		RtnCen_LS( YONLY_ON ) ;		// Y Servo ON

		UlHlySts = TneCen( X_DIR, Ptr, PTP_AFTER );
	
		StAdjPar.StHalAdj.UsAdxOff = StAdjPar.StHalAdj.UsHlxCna ;
		RamWrite32A( HALL_RAM_HXOFF,  (UINT_32)((StAdjPar.StHalAdj.UsAdxOff << 16 ) & 0xFFFF0000 )) ;
	}

	return ( UlHlySts | UlHlxSts );
}

//********************************************************************************
// Function Name 	: TneRun
// Retun Value		: Hall Tuning SUCCESS or FAILURE
// Argment Value	: NON
// Explanation		: Hall System Auto Adjustment Function
// History			: First edition 									2015.08.12
//********************************************************************************
UINT_32	TneRun( void )
{
	UINT_32	UlAtxSts=EXE_END, UlAtySts=EXE_END ; 
	UINT_32	UlFinSts=EXE_END ;
	ADJ_HALL* HallPtr;
	ADJ_LOPGAN* LopgainPtr;
	DSPVER Info;
	UINT_32 UlReadVal;

	LS_SS_Sel	= 0 ;
//--------------------------------------
// Initialize Calibration data
//--------------------------------------
	// Select parameter
	if( GetInfomation( &Info ) != 0){
		return( EXE_ERR );
	}

	HallPtr = (ADJ_HALL*)&SMA_HallCalParameter;
	LopgainPtr = (ADJ_LOPGAN *)&SMA_LoopGainParameter;

CAM_ERR(CAM_OIS, "   Hall Calibration \n" );
CAM_ERR(CAM_OIS, " 	BiasInit      : %08X \n" , HallPtr->BiasInit      );
CAM_ERR(CAM_OIS, " 	BiasInitZ     : %08X \n" , HallPtr->ZBiasInit     );
CAM_ERR(CAM_OIS, " 	XOffsetInit   : %08X \n" , HallPtr->XOffsetInit   );
CAM_ERR(CAM_OIS, " 	YOffsetInit   : %08X \n" , HallPtr->YOffsetInit   );
CAM_ERR(CAM_OIS, " 	ZOffsetInit   : %08X \n" , HallPtr->ZOffsetInit   );
CAM_ERR(CAM_OIS, " 	OffsetMargin  : %04X \n" , HallPtr->OffsetMargin  );
CAM_ERR(CAM_OIS, " 	XTargetRange  : %04X \n" , HallPtr->XTargetRange  );
CAM_ERR(CAM_OIS, " 	XTargetMax    : %04X \n" , HallPtr->XTargetMax    );
CAM_ERR(CAM_OIS, " 	XTargetMin    : %04X \n" , HallPtr->XTargetMin    );
CAM_ERR(CAM_OIS, " 	YTargetRange  : %04X \n" , HallPtr->YTargetRange  );
CAM_ERR(CAM_OIS, " 	YTargetMax    : %04X \n" , HallPtr->YTargetMax    );
CAM_ERR(CAM_OIS, " 	YTargetMin    : %04X \n" , HallPtr->YTargetMin    );
CAM_ERR(CAM_OIS, " 	ZTargetRange  : %04X \n" , HallPtr->ZTargetRange  );
CAM_ERR(CAM_OIS, " 	ZTargetMax    : %04X \n" , HallPtr->ZTargetMax    );
CAM_ERR(CAM_OIS, " 	ZTargetMin    : %04X \n" , HallPtr->ZTargetMin    );

CAM_ERR(CAM_OIS, " 	OisSinNum     : %08X \n" , HallPtr->OisSinNum     );
CAM_ERR(CAM_OIS, " 	OisSinFreq    : %08X \n" , HallPtr->OisSinFreq    );
CAM_ERR(CAM_OIS, " 	OisSinGain    : %08X \n" , HallPtr->OisSinGain    );
CAM_ERR(CAM_OIS, " 	DecrementStep : %04X \n" , HallPtr->DecrementStep );
CAM_ERR(CAM_OIS, "   Loop Gain \n" );
CAM_ERR(CAM_OIS, " 	Hxgain        : %08X \n" , LopgainPtr->Hxgain     );
CAM_ERR(CAM_OIS, " 	Hygain        : %08X \n" , LopgainPtr->Hygain     );
CAM_ERR(CAM_OIS, " 	XNoiseNum     : %08X \n" , LopgainPtr->XNoiseNum  );
CAM_ERR(CAM_OIS, " 	XNoiseFreq    : %08X \n" , LopgainPtr->XNoiseFreq );
CAM_ERR(CAM_OIS, " 	XNoiseGain    : %08X \n" , LopgainPtr->XNoiseGain );
CAM_ERR(CAM_OIS, " 	XGap          : %08X \n" , LopgainPtr->XGap       );
CAM_ERR(CAM_OIS, " 	YNoiseNum     : %08X \n" , LopgainPtr->YNoiseNum  );
CAM_ERR(CAM_OIS, " 	YNoiseFreq    : %08X \n" , LopgainPtr->YNoiseFreq );
CAM_ERR(CAM_OIS, " 	YNoiseGain    : %08X \n" , LopgainPtr->YNoiseGain );
CAM_ERR(CAM_OIS, " 	YGap          : %08X \n" , LopgainPtr->YGap       );
CAM_ERR(CAM_OIS, " 	XJudgeHigh    : %08X \n" , LopgainPtr->XJudgeHigh );
CAM_ERR(CAM_OIS, " 	XJudgeLow     : %08X \n" , LopgainPtr->XJudgeLow  );
CAM_ERR(CAM_OIS, " 	YJudgeHigh    : %08X \n" , LopgainPtr->YJudgeHigh );
CAM_ERR(CAM_OIS, " 	YJudgeLow     : %08X \n" , LopgainPtr->YJudgeLow  );

	RtnCen( BOTH_OFF ) ;

	if( Info.ActVersion == 06 ) {

CAM_ERR(CAM_OIS, " Select Act : TAH1316 \n" );
		RamWrite32A( SMAHALL_RAM_HXOFF,  0x00000000 ) ;			//< X Offset Clr
		RamWrite32A( SMAHALL_RAM_HYOFF,  0x00000000 ) ;			//< Y Offset Clr
		RamWrite32A( SMAHALL_RAM_HZOFF,  0x00000000 ) ;			//< Y Offset Clr	

		RamWrite32A( SMA_COEFF_TGAIN0BIAS, 0x40400000 ) ;		// 30mA fixed
		RamWrite32A( SMA_COEFF_TGAIN1BIAS, 0x40400000 ) ;
		RamWrite32A( SMA_COEFF_TGAIN2BIAS, 0x40400000 ) ;
		RamWrite32A( SMA_COEFF_TGAIN3BIAS, 0x40400000 ) ;

//--------------------------------------
//		 Wire bias calibration
//--------------------------------------	
		RamRead32A( SMA_RAM_SW, &UlReadVal );
		UlReadVal &= ~0x00000010;			// disable bit4
		RamWrite32A( SMA_RAM_SW, UlReadVal );

		RamWrite32A( SMA_BIAS, 0x26400000 );// 60mA

//--------------------------------------
//		 Hall Calibration
//--------------------------------------	
		UlFinSts |= HallAdj_TRIPLE( HallPtr );
		if( ((UlFinSts & EXE_HXADJ) == EXE_HXADJ) || ((UlFinSts & EXE_HYADJ) == EXE_HYADJ) ) return ( UlFinSts );

	} else if( Info.ActVersion == 07 ) {

CAM_ERR(CAM_OIS, " Select Act : TAH1120 \n" );
		RamWrite32A( SMAHALL_RAM_HXOFF,  0x00000000 ) ;			//< X Offset Clr
		RamWrite32A( SMAHALL_RAM_HYOFF,  0x00000000 ) ;			//< Y Offset Clr
		RamWrite32A( SMAHALL_RAM_HZOFF,  0x00000000 ) ;			//< Y Offset Clr	

		RamWrite32A( SMA_COEFF_TGAIN0BIAS, 0x40400000 ) ;		// 30mA fixed
		RamWrite32A( SMA_COEFF_TGAIN1BIAS, 0x40400000 ) ;
		RamWrite32A( SMA_COEFF_TGAIN2BIAS, 0x40400000 ) ;
		RamWrite32A( SMA_COEFF_TGAIN3BIAS, 0x40400000 ) ;

//--------------------------------------
//		 Wire bias calibration
//--------------------------------------	
		RamRead32A( SMA_RAM_SW, &UlReadVal );
		UlReadVal &= ~0x00000010;			// disable bit4
		RamWrite32A( SMA_RAM_SW, UlReadVal );

		RamWrite32A( SMA_BIAS, 0x26400000 );// 60mA

//--------------------------------------
//		 Hall Calibration
//--------------------------------------	
		UlFinSts |= HallAdj_TRIPLE( HallPtr );
		if( ((UlFinSts & EXE_HXADJ) == EXE_HXADJ) || ((UlFinSts & EXE_HYADJ) == EXE_HYADJ) ) return ( UlFinSts );

	} else {
		return( EXE_ERR );
	}	
	
#ifdef NEUTRAL_CENTER
	//---------------------------------------------------
	// Measurement neutral center
	//---------------------------------------------------
	RamWrite32A( SMA_BIAS, 0x00000000 );// 0mA
	WitTim( 500 );
	RamWrite32A( SMA_BIAS, 0x26400000 );// 60mA
CAM_ERR(CAM_OIS, "Mecha   center X=%04X, Y=%04X, Z=%04X\n", StAdjPar.StHalAdj.UsHlxCna, StAdjPar.StHalAdj.UsHlyCna, StAdjPar.StHalAdj.UsHlzCna );
	UlFinSts |= TneHvc( );
	StAdjPar.StHalAdj.UsAdxOff = StAdjPar.StHalAdj.UsHlxCna;
	StAdjPar.StHalAdj.UsAdyOff = StAdjPar.StHalAdj.UsHlyCna;
	StAdjPar.StHalAdj.UsAdzOff = StAdjPar.StHalAdj.UsHlzCna;

CAM_ERR(CAM_OIS, "Neutral center X=%04X, Y=%04X, Z=%04X\n", StAdjPar.StHalAdj.UsHlxCna, StAdjPar.StHalAdj.UsHlyCna, StAdjPar.StHalAdj.UsHlzCna );

	RamWrite32A( SMAHALL_RAM_HXOFF,  (UINT_32)((StAdjPar.StHalAdj.UsAdxOff << 16 ) & 0xFFFF0000 )) ;
	RamWrite32A( StSmaCaliData_SiLens_Offset_X,  (UINT_32)((StAdjPar.StHalAdj.UsAdxOff << 16 ) & 0xFFFF0000 )) ;		
	RamWrite32A( SMAHALL_RAM_HYOFF,  (UINT_32)((StAdjPar.StHalAdj.UsAdyOff << 16 ) & 0xFFFF0000 )) ;
	RamWrite32A( StSmaCaliData_SiLens_Offset_Y,  (UINT_32)((StAdjPar.StHalAdj.UsAdyOff << 16 ) & 0xFFFF0000 )) ;		
	RamWrite32A( SMAHALL_RAM_HZOFF,  (UINT_32)((StAdjPar.StHalAdj.UsAdzOff << 16 ) & 0xFFFF0000 )) ;		
	RamWrite32A( StSmaCaliData_SiLens_Offset_Z,  (UINT_32)((StAdjPar.StHalAdj.UsAdzOff << 16 ) & 0xFFFF0000 )) ;		

#endif	// NEUTRAL_CENTER

//--------------------------------------
// Adjust loop gain
//--------------------------------------
////	RtnCen( BOTH_ON ) ;					// Y ON / X ON
	WitTim( TNE ) ;

//	UlAtxSts	= LopGan( X_DIR, LopgainPtr ) ;		// X Loop Gain Adjust
//	UlAtySts	= LopGan( Y_DIR, LopgainPtr ) ;		// Y Loop Gain Adjust
	RamWrite32A( SmaHallFilterCoeffX_hxgain0 , LopgainPtr->Hxgain ) ;
	StAdjPar.StLopGan.UlLxgVal = LopgainPtr->Hxgain ;

	RamWrite32A( SmaHallFilterCoeffY_hygain0 , LopgainPtr->Hygain ) ;
	StAdjPar.StLopGan.UlLygVal = LopgainPtr->Hygain ;
	StAdjPar.StLopGan.UlLzgVal = LopgainPtr->Hygain ;
	UlAtxSts = UlAtySts = EXE_END;

	UlFinSts	|= UlAtxSts | UlAtySts ;
CAM_ERR(CAM_OIS, "UlFinSts :  %08X\n",UlFinSts );
	

//--------------------------------------
// Error check
//--------------------------------------
	if( UlFinSts != EXE_END ) {
		RtnCen( BOTH_OFF ) ;				// Both OFF
		StAdjPar.StHalAdj.UlAdjPhs = UlFinSts ;

		return( UlFinSts ) ;
	}

	UlReadVal |= 0x00000010;			// enable bit4
	RamWrite32A( SMA_RAM_SW, UlReadVal );

//--------------------------------------
// Calculation adjust status
//--------------------------------------
	UlFinSts	= UlFinSts | UlAtxSts | UlAtySts | EXE_END ;
	StAdjPar.StHalAdj.UlAdjPhs = UlFinSts ;
CAM_ERR(CAM_OIS, "UlFinSts :  %08X\n",UlFinSts );
	return( UlFinSts ) ;
}

//********************************************************************************
// Function Name 	: TneOffIn_TRIPLE
// Retun Value		: NONE
// Argment Value	: UcDirSel, *p
// Explanation		: Hall input offset Adjustment Function
// History			: First edition
//********************************************************************************
UINT_32 TneOffIn_TRIPLE( UINT_8 UcDirSel, ADJ_HALL* p )
{
	UINT_32		DAC_ADDR;
	INT_32		OFFSET_ADDR;
	UINT_16		*pHigGap, *pLowGap;
	UINT_16		*pCENTER_ADDR;

	INT_32		A, B, C;
	INT_32		SiOff1, SiOff2;
	INT_32		SiCenter1, SiCenter2;
	UINT_32 	UlDac;
	INT_32		offset_step = 0x01000000,  offset_gap = 0x04000000, detect_margin = 0x00004000, cnt;

CAM_ERR(CAM_OIS, "\nTneOffIn\n");
	//********************************************************************************
	if( UcDirSel == X_DIR ) {								// X axis
		DAC_ADDR = HLXOI;
		SiOff1 = p->XOffsetInitIn;	// Init offset
		OFFSET_ADDR = StSmaCaliData_UiHallOffsetI_X;
		pCENTER_ADDR = &StAdjPar.StHalAdj.UsHlxCen;
		pHigGap = &StAdjPar.StHalAdj.UsHigxGap;
		pLowGap = &StAdjPar.StHalAdj.UsLowxGap;
		offset_step *= +1;

	//********************************************************************************
	} else if( UcDirSel == Y_DIR ) {						// Y axis
		DAC_ADDR = HLYOI;
		SiOff1 = p->YOffsetInitIn;	// Init offset
		OFFSET_ADDR = StSmaCaliData_UiHallOffsetI_Y;
		pCENTER_ADDR = &StAdjPar.StHalAdj.UsHlyCen;
		pHigGap = &StAdjPar.StHalAdj.UsHigyGap;
		pLowGap = &StAdjPar.StHalAdj.UsLowyGap;
		offset_step *= +1;

	//********************************************************************************
	} else {												// Z axis
		DAC_ADDR = HLZOI;
		SiOff1 = p->ZOffsetInitIn;	// Init offset
		OFFSET_ADDR = StSmaCaliData_UiHallOffsetI_Z;
		pCENTER_ADDR = &StAdjPar.StHalAdj.UsHlzCen;
		pHigGap = &StAdjPar.StHalAdj.UsHigzGap;
		pLowGap = &StAdjPar.StHalAdj.UsLowzGap;
		offset_step *= -1;
	}

	cnt=0;
	do {
		DacControl( 0 , DAC_ADDR , SiOff1 ) ;
		RamWrite32A( OFFSET_ADDR , SiOff1 ) ;
		WitTim( 10 );

		TnePtp_TRIPLE( UcDirSel, PTP_BEFORE, p );
		SiCenter1 = (INT_16)*pCENTER_ADDR;
CAM_ERR(CAM_OIS, "SiOff1 = %08X SiCenter1 = %08X  cnt = %d: \n",SiOff1, SiCenter1, cnt );
		if( (SiCenter1 > ((INT_32)0xFFFF8000 + detect_margin)) && ( SiCenter1 < ((INT_32)0x00007FFF - detect_margin))
			&&	(*pHigGap > 0x0300) && (*pLowGap > 0x0300) ){
CAM_ERR(CAM_OIS, "PASS\n");
			break;
		}else{
			if( SiCenter1 > 0 )	SiOff1 -= offset_step ;
			else				SiOff1 += offset_step ;
				
			if( cnt >= 60 )	return( 0x12 );
		}
	} while( ++cnt < 64 );

	if( SiCenter1 > 0 )		SiOff2 = SiOff1 - offset_gap ;	// second offset
	else					SiOff2 = SiOff1 + offset_gap ;	// second offset

	cnt=0;
	do {
		DacControl( 0 , DAC_ADDR , SiOff2 ) ;
		RamWrite32A( OFFSET_ADDR , SiOff2 ) ;
		WitTim( 10 );

		TnePtp_TRIPLE( UcDirSel, PTP_BEFORE, p );
		SiCenter2 = (INT_16)*pCENTER_ADDR;
CAM_ERR(CAM_OIS, "SiOff2 = %08X SiCenter2 = %08X  cnt = %d: \n",SiOff2, SiCenter2, cnt );
		if( (SiCenter2 > ((INT_32)0xFFFF8000 + detect_margin)) && ( SiCenter2 < ((INT_32)0x00007FFF - detect_margin))
			&&	(*pHigGap > 0x0300) && (*pLowGap > 0x0300) ){
CAM_ERR(CAM_OIS, "PASS\n");
			break;
		}else{
			if( SiCenter2 > 0 )	SiOff2 -= offset_step ;
			else				SiOff2 += offset_step ;
			if( cnt >= 10 )	return( 0x22 );
		}
	} while ( ++cnt < 16 );


	A = (INT_32)((UINT_32)SiOff2 >> 24) * SiCenter1;
	B = (INT_32)((UINT_32)SiOff1 >> 24) * SiCenter2;
	C = SiCenter1 - SiCenter2;
	UlDac = (A - B) / C;
CAM_ERR(CAM_OIS, "A=%d, B=%d, C=%d, DAC=%X\n",A, B, C, UlDac );
	UlDac <<= 24;

	DacControl( 0 , DAC_ADDR , UlDac ) ;
	RamWrite32A( OFFSET_ADDR , UlDac ) ;

CAM_ERR(CAM_OIS, "Center1 = %04X, DAC1 = %08X\n", SiCenter1, SiOff1 );
CAM_ERR(CAM_OIS, "Center2 = %04X, DAC2 = %08X\n", SiCenter2, SiOff2 );
CAM_ERR(CAM_OIS, "Target DAC = %08X\n", UlDac );
	return( 0x02 );
}


//********************************************************************************
// Function Name 	: TnePtp_TRIPLE
// Retun Value		: Hall Top & Bottom Gaps
// Argment Value	: X,Y Direction, Adjust Before After Parameter
// Explanation		: Measuring Hall Paek To Peak
// History			: First edition
//********************************************************************************
void	TnePtp_TRIPLE ( UINT_8	UcDirSel, UINT_8	UcBfrAft, ADJ_HALL* p )
{
	INT_32	SlMeasureParameterA , SlMeasureParameterB ;
	INT_32	SlMeasureMaxValueA, SlMeasureMinValueA, SlMeasureMaxValueB, SlMeasureMinValueB ;
	UINT_16	UsSinAdr ;
	INT_32	sl_act_min_drv, sl_act_max_drv;	
	INT_32	sl_act_num, sl_act_step, sl_act_wait, i ;	
	
CAM_ERR(CAM_OIS, "TnePtp_TRIPLE ") ;

	if( UcDirSel == X_DIR ) {								// X axis
CAM_ERR(CAM_OIS, "Axis-X\n") ;
		SlMeasureParameterA		=	SMAHALL_RAM_HXIDAT ;	// Set Measure RAM Address
		SlMeasureParameterB		=	SMAHALL_RAM_HYIDAT ;	// Set Measure RAM Address
		sl_act_max_drv		= (INT_32)(p->ActMaxDrive_X) ;
		sl_act_min_drv		= (INT_32)(p->ActMinDrive_X) ;	
		sl_act_num			= (INT_32)(p->ActStep_X_Num);
		sl_act_step			= (INT_32)((p->ActStep_X));
		sl_act_wait			= (INT_32)(p->ActStep_X_time);
		UsSinAdr = SMAHALL_RAM_SINDX1;
#ifdef DEBUG
		{
			UINT_32	ulBias, ulOffset;
			RamRead32A( StSmaCaliData_UiHallBias_X, &ulBias );
			RamRead32A( StSmaCaliData_UiHallOffsetO_X, &ulOffset );
CAM_ERR(CAM_OIS, "  X BIAS=%08X , OFFSETO=%08X\n", ulBias, ulOffset ) ;

		}
#endif
	} else {												// Y/Z axis
CAM_ERR(CAM_OIS, "Axis-Y/Z\n") ;
		SlMeasureParameterA		=	SMAHALL_RAM_HYIDAT ;	// Set Measure RAM Address
		SlMeasureParameterB		=	SMAHALL_RAM_HZIDAT ;	// Set Measure RAM Address
		sl_act_max_drv		= (INT_32)(p->ActMaxDrive_Y) ;
		sl_act_min_drv		= (INT_32)(p->ActMinDrive_Y) ;
		sl_act_num			= (INT_32)(p->ActStep_Y_Num);
		sl_act_step			= (INT_32)((p->ActStep_Y));		
		sl_act_wait			= (INT_32)(p->ActStep_Y_time);			
		UsSinAdr = SMAHALL_RAM_SINDY1;
#ifdef DEBUG
		{
			UINT_32	ulBias, ulOffset;
			RamRead32A( StSmaCaliData_UiHallBias_Y, &ulBias );
			RamRead32A( StSmaCaliData_UiHallOffsetO_Y, &ulOffset );
CAM_ERR(CAM_OIS, "  Y BIAS=%08X , OFFSETO=%08X\n", ulBias, ulOffset ) ;
			RamRead32A( StSmaCaliData_UiHallBias_Z, &ulBias );
			RamRead32A( StSmaCaliData_UiHallOffsetO_Z, &ulOffset );
CAM_ERR(CAM_OIS, "  Z BIAS=%08X , OFFSETO=%08X\n", ulBias, ulOffset ) ;

		}
#endif
	}
	
	MesFil( THROUGH ) ;					// Filter setting for measurement

	// move to positive
	for( i=0 ; i <= sl_act_num; i++ ){
		RamWrite32A( UsSinAdr, (sl_act_max_drv - ((sl_act_step) * (sl_act_num -i))) );
		WitTim(sl_act_wait);
//CAM_ERR(CAM_OIS, "DrvMax:%08x\n",  (sl_act_max_drv - ((sl_act_step) * (sl_act_num -i))) );
	} 
CAM_ERR(CAM_OIS, "DrvMax:%08x -",  sl_act_max_drv );

	WitTim( p->WaitTime );
	MeasureStart( p->OisSinNum , SlMeasureParameterA , SlMeasureParameterB ) ;					// Start measure
	MeasureWait() ;						// Wait complete of measurement
	RamRead32A( StMeasFunc_MFA_SiMin1 , ( UINT_32 * )&SlMeasureMinValueA ) ;	// Min value
	RamRead32A( StMeasFunc_MFB_SiMin2 , ( UINT_32 * )&SlMeasureMinValueB ) ;	// Min value	

	RamWrite32A( UsSinAdr		,	0x00000000 ) ;				// DelayRam Clear
CAM_ERR(CAM_OIS, "  Min A:0x%08X", SlMeasureMinValueA);	
CAM_ERR(CAM_OIS, "  Min B:0x%08X\n", SlMeasureMinValueB);	

	// move to negative
	for( i=0 ; i <= sl_act_num; i++ ){
		RamWrite32A( UsSinAdr, (sl_act_min_drv + ((sl_act_step) * (sl_act_num - i))) );
		WitTim(sl_act_wait);
//CAM_ERR(CAM_OIS, "DrvMin:%08x\n", (sl_act_min_drv + ((sl_act_step) * (sl_act_num - i))) );
	} 
CAM_ERR(CAM_OIS, "DrvMin:%08x -", sl_act_min_drv );

	WitTim( p->WaitTime );
	MeasureStart( p->OisSinNum , SlMeasureParameterA , SlMeasureParameterB ) ;					// Start measure
	MeasureWait() ;						// Wait complete of measurement
	RamRead32A( StMeasFunc_MFA_SiMin1 , ( UINT_32 * )&SlMeasureMaxValueA ) ;	// Max value
	RamRead32A( StMeasFunc_MFB_SiMin2 , ( UINT_32 * )&SlMeasureMaxValueB ) ;	// Max value
	
	RamWrite32A( UsSinAdr		,	0x00000000 ) ;				// DelayRam Clear
CAM_ERR(CAM_OIS, "  Max A:0x%08X", SlMeasureMaxValueA);	
CAM_ERR(CAM_OIS, "  Max B:0x%08X\n", SlMeasureMaxValueB);	

	
	if( UcBfrAft == PTP_BEFORE ) {
		if( UcDirSel == X_DIR ) {	// X
			StAdjPar.StHalAdj.UsHlxMax	= ( UINT_16 )((SlMeasureMaxValueA >> 16) & 0x0000FFFF ) ;
			StAdjPar.StHalAdj.UsHlxMin	= ( UINT_16 )((SlMeasureMinValueA >> 16) & 0x0000FFFF ) ;
			
			StAdjPar.StHalAdj.UsHlxCen	= ( ( INT_16 )StAdjPar.StHalAdj.UsHlxMax + ( INT_16 )StAdjPar.StHalAdj.UsHlxMin )/ 2 ;
			StAdjPar.StHalAdj.UsHigxGap	= 0x7fff - StAdjPar.StHalAdj.UsHlxMax ;
			StAdjPar.StHalAdj.UsLowxGap	= StAdjPar.StHalAdj.UsHlxMin - 0x8000 ;

		} else {	// Y1 and Y2
			StAdjPar.StHalAdj.UsHlyMax	= ( UINT_16 )((SlMeasureMaxValueA >> 16) & 0x0000FFFF ) ;
			StAdjPar.StHalAdj.UsHlyMin	= ( UINT_16 )((SlMeasureMinValueA >> 16) & 0x0000FFFF ) ;
			
			StAdjPar.StHalAdj.UsHlyCen	= ( ( INT_16 )StAdjPar.StHalAdj.UsHlyMax + ( INT_16 )StAdjPar.StHalAdj.UsHlyMin )/ 2 ;
			StAdjPar.StHalAdj.UsHigyGap	= 0x7fff - StAdjPar.StHalAdj.UsHlyMax ;
			StAdjPar.StHalAdj.UsLowyGap	= StAdjPar.StHalAdj.UsHlyMin - 0x8000 ;
			
			StAdjPar.StHalAdj.UsHlzMax	= ( UINT_16 )((SlMeasureMaxValueB >> 16) & 0x0000FFFF ) ;
			StAdjPar.StHalAdj.UsHlzMin	= ( UINT_16 )((SlMeasureMinValueB >> 16) & 0x0000FFFF ) ;

			StAdjPar.StHalAdj.UsHlzCen	= ( ( INT_16 )StAdjPar.StHalAdj.UsHlzMax + ( INT_16 )StAdjPar.StHalAdj.UsHlzMin)/ 2 ;
			StAdjPar.StHalAdj.UsHigzGap	= 0x7fff - StAdjPar.StHalAdj.UsHlzMax ;
			StAdjPar.StHalAdj.UsLowzGap	= StAdjPar.StHalAdj.UsHlzMin - 0x8000 ;
		
		}
	} else {
		if( UcDirSel == X_DIR ){	// X
			StAdjPar.StHalAdj.UsHlxMxa	= ( UINT_16 )((SlMeasureMaxValueA >> 16) & 0x0000FFFF ) ;
			StAdjPar.StHalAdj.UsHlxMna	= ( UINT_16 )((SlMeasureMinValueA >> 16) & 0x0000FFFF ) ;
			
			StAdjPar.StHalAdj.UsHlxCna	= ( ( INT_16 )StAdjPar.StHalAdj.UsHlxMxa + ( INT_16 )StAdjPar.StHalAdj.UsHlxMna )/ 2 ;
			StAdjPar.StHalAdj.UsHigxGap	= 0x7fff - StAdjPar.StHalAdj.UsHlxMxa ;
			StAdjPar.StHalAdj.UsLowxGap	= StAdjPar.StHalAdj.UsHlxMna - 0x8000 ;
			RamWrite32A( StSmaCaliData_SiHallMax_X , SlMeasureMaxValueA ) ;
			RamWrite32A( StSmaCaliData_SiHallMin_X , SlMeasureMinValueA ) ;
CAM_ERR(CAM_OIS, "		ADJ( X ) MAX = %04x, MIN = %04x, CNT = %04x\n ",  StAdjPar.StHalAdj.UsHlxMxa, StAdjPar.StHalAdj.UsHlxMna, StAdjPar.StHalAdj.UsHlxCna) ;
CAM_ERR(CAM_OIS, "			GapH = %04x, GapL = %04x\n", StAdjPar.StHalAdj.UsHigxGap, StAdjPar.StHalAdj.UsLowxGap ) ;		

		} else {	// Y1 and Y2
			StAdjPar.StHalAdj.UsHlyMxa	= ( UINT_16 )((SlMeasureMaxValueA >> 16) & 0x0000FFFF ) ;
			StAdjPar.StHalAdj.UsHlyMna	= ( UINT_16 )((SlMeasureMinValueA >> 16) & 0x0000FFFF ) ;
			
			StAdjPar.StHalAdj.UsHlyCna	= ( ( INT_16 )StAdjPar.StHalAdj.UsHlyMxa + ( INT_16 )StAdjPar.StHalAdj.UsHlyMna )/ 2 ;
			StAdjPar.StHalAdj.UsHigyGap	= 0x7fff - StAdjPar.StHalAdj.UsHlyMxa ;
			StAdjPar.StHalAdj.UsLowyGap	= StAdjPar.StHalAdj.UsHlyMna - 0x8000 ;
			RamWrite32A( StSmaCaliData_SiHallMax_Y , SlMeasureMaxValueA ) ;
			RamWrite32A( StSmaCaliData_SiHallMin_Y , SlMeasureMinValueA ) ;	
CAM_ERR(CAM_OIS, "		ADJ( Y ) MAX = %04x, MIN = %04x, CNT = %04x\n ",  StAdjPar.StHalAdj.UsHlyMxa, StAdjPar.StHalAdj.UsHlyMna, StAdjPar.StHalAdj.UsHlyCna) ;
CAM_ERR(CAM_OIS, "			GapH = %04x, GapL = %04x\n", StAdjPar.StHalAdj.UsHigyGap, StAdjPar.StHalAdj.UsLowyGap ) ;	

			StAdjPar.StHalAdj.UsHlzMxa	= ( UINT_16 )((SlMeasureMaxValueB >> 16) & 0x0000FFFF ) ;
			StAdjPar.StHalAdj.UsHlzMna	= ( UINT_16 )((SlMeasureMinValueB >> 16) & 0x0000FFFF ) ;
			StAdjPar.StHalAdj.UsHlzCna	= ( ( INT_16 )StAdjPar.StHalAdj.UsHlzMxa + ( INT_16 )StAdjPar.StHalAdj.UsHlzMna )/ 2 ;
			StAdjPar.StHalAdj.UsHigzGap	= 0x7fff - StAdjPar.StHalAdj.UsHlzMxa ;
			StAdjPar.StHalAdj.UsLowzGap	= StAdjPar.StHalAdj.UsHlzMna - 0x8000 ;
			RamWrite32A( StSmaCaliData_SiHallMax_Z , SlMeasureMaxValueB ) ;
			RamWrite32A( StSmaCaliData_SiHallMin_Z , SlMeasureMinValueB ) ;
CAM_ERR(CAM_OIS, "		ADJ( Z ) MAX = %04x, MIN = %04x, CNT = %04x\n ",  StAdjPar.StHalAdj.UsHlzMxa, StAdjPar.StHalAdj.UsHlzMna, StAdjPar.StHalAdj.UsHlzCna) ;
CAM_ERR(CAM_OIS, "			GapH = %04x, GapL = %04x\n", StAdjPar.StHalAdj.UsHigzGap, StAdjPar.StHalAdj.UsLowzGap ) ;			
		}
	}
}

//********************************************************************************
// Function Name 	: TneCen_TRIPLE
// Retun Value		: Hall Center Tuning Result
// Argment Value	: X,Y Direction, Hall Top & Bottom Gaps
// Explanation		: Hall Center Tuning Function
// History			: First edition 									2015.08.12
//********************************************************************************
UINT_32	TneCen_TRIPLE( UINT_8 UcTneAxs, ADJ_HALL* ptr , UINT_8 UcBfrAft )
{
	UnDwdVal	StTneVal ;
	UINT_8		UcTmeOut =1, UcTofRst = SUCCESS;//, UcTofRstB = SUCCESS ;
	UINT_16		UsBiasVal ;
	UINT_32		UlTneRstA = FAILURE, UlTneRstB = FAILURE, UlBiasVal , UlValNow, ans ;
	UINT_16		UsValBef=0,UsValNow =0 ;
	UINT_32		UlBiaBef,UlBiaNow ;
	UINT_16		UsTargetRange;
	
	// Measurement amplitude
	TnePtp_TRIPLE( UcTneAxs , UcBfrAft , ptr ) ;
	
	if(UcTneAxs == X_DIR){
		// X
		StTneVal.StDwdVal.UsHigVal = StAdjPar.StHalAdj.UsHigxGap;
		StTneVal.StDwdVal.UsLowVal = StAdjPar.StHalAdj.UsLowxGap;
		TneOff_TRIPLE( StTneVal, X_DIR ) ;
		UlTneRstB = SUCCESS;				

	}else{
		// Y1, Y2
		StTneVal.StDwdVal.UsHigVal = StAdjPar.StHalAdj.UsHigyGap;
		StTneVal.StDwdVal.UsLowVal = StAdjPar.StHalAdj.UsLowyGap;
		TneOff_TRIPLE( StTneVal, Y_DIR ) ;

		StTneVal.StDwdVal.UsHigVal = StAdjPar.StHalAdj.UsHigzGap;
		StTneVal.StDwdVal.UsLowVal = StAdjPar.StHalAdj.UsLowzGap;
		TneOff_TRIPLE( StTneVal, Z_DIR ) ;
	}

	while ( (UlTneRstA || UlTneRstB ) && UcTmeOut )
	{
		if( UcTofRst == FAILURE ) {
CAM_ERR(CAM_OIS, " --Tne_OFF--\n" ) ;
			if( UcTneAxs == X_DIR ){
				// X
				StTneVal.StDwdVal.UsHigVal = StAdjPar.StHalAdj.UsHigxGap;
				StTneVal.StDwdVal.UsLowVal = StAdjPar.StHalAdj.UsLowxGap;
				TneOff_TRIPLE( StTneVal, X_DIR ) ;		

			}else{
				// Y1
				StTneVal.StDwdVal.UsHigVal = StAdjPar.StHalAdj.UsHigyGap;
				StTneVal.StDwdVal.UsLowVal = StAdjPar.StHalAdj.UsLowyGap;
				TneOff_TRIPLE( StTneVal, Y_DIR ) ;		
				// Y2
				StTneVal.StDwdVal.UsHigVal = StAdjPar.StHalAdj.UsHigzGap;
				StTneVal.StDwdVal.UsLowVal = StAdjPar.StHalAdj.UsLowzGap;
				TneOff_TRIPLE( StTneVal, Z_DIR ) ;
			}	
			TnePtp_TRIPLE( UcTneAxs , PTP_AFTER, ptr ) ;

		} else {
CAM_ERR(CAM_OIS, " ---TneBias---\n" ) ;
			if( UcTneAxs == X_DIR ) {
				if( UlTneRstA == FAILURE ){
					RamRead32A( StSmaCaliData_UiHallBias_X , &UlBiaBef ) ;
					UsTargetRange = (UINT_16)ptr->XTargetRange;
					StTneVal.StDwdVal.UsHigVal = StAdjPar.StHalAdj.UsHigxGap;
					StTneVal.StDwdVal.UsLowVal = StAdjPar.StHalAdj.UsLowxGap;
					TneBia_TRIPLE( StTneVal, X_DIR, UsTargetRange ) ;
					RamRead32A( StSmaCaliData_UiHallBias_X , &UlBiaNow ) ;
					UcTofRst	= FAILURE ;
				}
			} else if( UcTneAxs == Y_DIR ) {
				if( UlTneRstA == FAILURE ){
					RamRead32A( StSmaCaliData_UiHallBias_Y , &UlBiaBef ) ;
					UsTargetRange = (UINT_16)ptr->YTargetRange;
					StTneVal.StDwdVal.UsHigVal = StAdjPar.StHalAdj.UsHigyGap;
					StTneVal.StDwdVal.UsLowVal = StAdjPar.StHalAdj.UsLowyGap;
					TneBia_TRIPLE( StTneVal, Y_DIR, UsTargetRange ) ;	
					RamRead32A( StSmaCaliData_UiHallBias_Y , &UlBiaNow ) ;		
					UcTofRst	= FAILURE ;
				}
				if( UlTneRstB == FAILURE ){
					RamRead32A( StSmaCaliData_UiHallBias_Z , &UlBiaBef ) ;	
					UsTargetRange = (UINT_16)ptr->ZTargetRange;			
					StTneVal.StDwdVal.UsHigVal = StAdjPar.StHalAdj.UsHigzGap;
					StTneVal.StDwdVal.UsLowVal = StAdjPar.StHalAdj.UsLowzGap;
					TneBia_TRIPLE( StTneVal, Z_DIR, UsTargetRange ) ;				
					RamRead32A( StSmaCaliData_UiHallBias_Z , &UlBiaNow ) ;	
					UcTofRst	= FAILURE ;
				}
			}	
			if((( UlBiaBef == BIAS_HLMT ) && ( UlBiaNow == BIAS_HLMT ))
			|| (( UlBiaBef == BIAS_LLMT ) && ( UlBiaNow == BIAS_LLMT ))){
				UcTmeOut += 10;
			}
			TnePtp_TRIPLE( UcTneAxs , PTP_AFTER, ptr ) ;
		}

// Conditional Judgement
		if(UcTneAxs == X_DIR){
			if( (StAdjPar.StHalAdj.UsHigxGap > ptr->OffsetMargin ) && (StAdjPar.StHalAdj.UsLowxGap > ptr->OffsetMargin ) )	/* position check */
			{
				UcTofRst	= SUCCESS ;
				UsValBef = UsValNow = 0x0000 ;
			}else if( (StAdjPar.StHalAdj.UsHigxGap <= ptr->OffsetMargin ) && (StAdjPar.StHalAdj.UsLowxGap <= ptr->OffsetMargin ) ){
				UcTofRst	= SUCCESS ;
//				UlTneRstA	= (UINT_32)FAILURE ;
			}else{
				UcTofRst	= FAILURE ;			
				UsValBef = UsValNow ;

				RamRead32A( StSmaCaliData_UiHallOffsetO_X , &UlValNow ) ;
				UsValNow = (UINT_16)( UlValNow >> 16 ) ;
				if( ((( UsValBef & 0xFF00 ) == 0x1000 ) && ( UsValNow & 0xFF00 ) == 0x1000 ) || ((( UsValBef & 0xFF00 ) == 0xEF00 ) && ( UsValNow & 0xFF00 ) == 0xEF00 ) ) {
					UcTmeOut += 10;
					RamRead32A( StSmaCaliData_UiHallBias_X , &UlBiasVal ) ;
					UsBiasVal = (UINT_16)( UlBiasVal >> 16 ) ;
					if( UsBiasVal > ptr->DecrementStep )	UsBiasVal -= ptr->DecrementStep ;
					UlBiasVal = ( UINT_32 )( UsBiasVal << 16 ) ;
					DacControl( 0, HLXBO , UlBiasVal ) ;
CAM_ERR(CAM_OIS, "	Saturation!!	HLXBO = %08x\n ",  UlBiasVal ) ;
					RamWrite32A( StSmaCaliData_UiHallBias_X , UlBiasVal ) ;
					StAdjPar.StHalAdj.UsHlxGan = (UINT_16)( UlBiasVal >> 16 ) ;		
				}
			}
			if((( (UINT_16)0xFFFF - ( StAdjPar.StHalAdj.UsHigxGap + StAdjPar.StHalAdj.UsLowxGap )) < (UINT_16)ptr->XTargetMax )
			&& (( (UINT_16)0xFFFF - ( StAdjPar.StHalAdj.UsHigxGap + StAdjPar.StHalAdj.UsLowxGap )) > (UINT_16)ptr->XTargetMin ) ) {
				if(UcTofRst	== SUCCESS)
				{
					UlTneRstA	= SUCCESS;
					break ;					
				}
			}

		}else{
			// Y1
			if( (StAdjPar.StHalAdj.UsHigyGap > ptr->OffsetMargin ) && (StAdjPar.StHalAdj.UsLowyGap > ptr->OffsetMargin ) )	/* position check */
			{
				UcTofRst	= SUCCESS ;
				UsValBef = UsValNow = 0x0000 ;
			}else if( (StAdjPar.StHalAdj.UsHigyGap <= ptr->OffsetMargin ) && (StAdjPar.StHalAdj.UsLowyGap <= ptr->OffsetMargin ) ){
				UcTofRst	= SUCCESS ;
//				UlTneRstA	= (UINT_32)FAILURE ;
			}else{
				UcTofRst	= FAILURE ;
				UsValBef = UsValNow ;

				RamRead32A( StSmaCaliData_UiHallOffsetO_Y , &UlValNow ) ;
				UsValNow = (UINT_16)( UlValNow ) ;
				if( ((( UsValBef & 0xFF00 ) == 0x1000 ) && ( UsValNow & 0xFF00 ) == 0x1000 ) || ((( UsValBef & 0xFF00 ) == 0xEF00 ) && ( UsValNow & 0xFF00 ) == 0xEF00 ) ) {
					UcTmeOut += 10;
					RamRead32A( StSmaCaliData_UiHallBias_Y , &UlBiasVal ) ;
					UsBiasVal = (UINT_16)( UlBiasVal >> 16 ) ;
					if( UsBiasVal > ptr->DecrementStep )	UsBiasVal -= ptr->DecrementStep ;
					UlBiasVal = ( UINT_32 )( UsBiasVal << 16 ) ;
					DacControl( 0, HLYBO , UlBiasVal ) ;
CAM_ERR(CAM_OIS, "^Saturation!!	HLYBO = %08x\n ",  UlBiasVal ) ;
					RamWrite32A( StSmaCaliData_UiHallBias_Y , UlBiasVal ) ;
					StAdjPar.StHalAdj.UsHlyGan = (UINT_16)( UlBiasVal >> 16 ) ;				 	
				}
			}
			if((( (UINT_16)0xFFFF - ( StAdjPar.StHalAdj.UsHigyGap + StAdjPar.StHalAdj.UsLowyGap )) < (UINT_16)ptr->YTargetMax )
			&& (( (UINT_16)0xFFFF - ( StAdjPar.StHalAdj.UsHigyGap + StAdjPar.StHalAdj.UsLowyGap )) > (UINT_16)ptr->YTargetMin ) ) {
				if(UcTofRst	== SUCCESS)
				{
					UlTneRstA = SUCCESS;							
				}
			}
		
			// Y2
			if( (StAdjPar.StHalAdj.UsHigzGap > ptr->OffsetMargin ) && (StAdjPar.StHalAdj.UsLowzGap > ptr->OffsetMargin ) )	/* position check */
			{
				UcTofRst	= SUCCESS ;
				UsValBef = UsValNow = 0x0000 ;
			}else if( (StAdjPar.StHalAdj.UsHigzGap <= ptr->OffsetMargin ) && (StAdjPar.StHalAdj.UsLowzGap <= ptr->OffsetMargin ) ){
				UcTofRst	= SUCCESS ;
//				UlTneRstB	= (UINT_32)FAILURE ;				
			}else{
				UcTofRst	= FAILURE ;
				UsValBef = UsValNow ;

				RamRead32A( StSmaCaliData_UiHallOffsetO_Z , &UlValNow ) ;
				UsValNow = (UINT_16)( UlValNow ) ;
				if( ((( UsValBef & 0xFF00 ) == 0x1000 ) && ( UsValNow & 0xFF00 ) == 0x1000 ) || ((( UsValBef & 0xFF00 ) == 0xEF00 ) && ( UsValNow & 0xFF00 ) == 0xEF00 ) ) {
					UcTmeOut += 20;
					RamRead32A( StSmaCaliData_UiHallBias_Z , &UlBiasVal ) ;
					UsBiasVal = (UINT_16)( UlBiasVal >> 16 ) ;
					if( UsBiasVal > ptr->DecrementStep )		UsBiasVal -= ptr->DecrementStep ;

					UlBiasVal = ( UINT_32 )( UsBiasVal << 16 ) ;
					DacControl( 0, HLZBO , UlBiasVal ) ;
CAM_ERR(CAM_OIS, "	Saturation!!	HLYAFO = %08x\n ",  UlBiasVal ) ;					
					RamWrite32A( StSmaCaliData_UiHallBias_Z , UlBiasVal ) ;
					StAdjPar.StHalAdj.UsHlzGan = (UINT_16)( UlBiasVal >> 16 ) ;								 	
				}
			}
			if((( (UINT_16)0xFFFF - ( StAdjPar.StHalAdj.UsHigzGap + StAdjPar.StHalAdj.UsLowzGap )) < (UINT_16)ptr->ZTargetMax )
			&& (( (UINT_16)0xFFFF - ( StAdjPar.StHalAdj.UsHigzGap + StAdjPar.StHalAdj.UsLowzGap )) > (UINT_16)ptr->ZTargetMin) ){
				if(UcTofRst	== SUCCESS)
				{
					UlTneRstB	= SUCCESS ;
				}
			}
			
			if( (UlTneRstA == SUCCESS) && (UlTneRstB == SUCCESS)){
				break;
			}	
		}
		if ( ++UcTmeOut >= TIME_OUT ) {
			UcTmeOut	= 0 ;
CAM_ERR(CAM_OIS, "  Tne = TimeOut\n" ) ;
		}		 																							// Set Time Out Count
	}	

					
//	SetSinWavGenInt() ;		// 

	ans	= EXE_END ;	
	if( UcTneAxs == X_DIR ) {
		if( UlTneRstA == FAILURE ) 	ans |= EXE_HXADJ ;
	}else if( UcTneAxs == Y_DIR ) {
		if( UlTneRstB == FAILURE ) 	ans |= EXE_HYADJ ;
		if( UlTneRstA == FAILURE ) 	ans |= EXE_HZADJ ;
	}
	return( ans ) ;

}

//********************************************************************************
// Function Name 	: TneBia_TRIPLE
// Retun Value		: None
// Argment Value	: Hall Top & Bottom Gaps , X,Y Direction
// Explanation		: Hall Bias Tuning Function
// History			: First edition
//********************************************************************************
void	TneBia_TRIPLE( UnDwdVal	StTneVal, UINT_8	UcTneAxs , UINT_16	UsHalAdjRange )
{
	UINT_32			UlSetBia ;

CAM_ERR(CAM_OIS, "TneBia\n ") ;
	if( UcTneAxs == X_DIR ) {
		RamRead32A( StSmaCaliData_UiHallBias_X , &UlSetBia ) ;
	} else if( UcTneAxs == Y_DIR ) {
		RamRead32A( StSmaCaliData_UiHallBias_Y , &UlSetBia ) ;
	} else {
		RamRead32A( StSmaCaliData_UiHallBias_Z , &UlSetBia ) ;
	}

	if( UlSetBia == 0x00000000 )	UlSetBia = 0x01000000 ;

	UlSetBia = (( UlSetBia >> 16 ) & (UINT_32)0x0000FF00 ) ;
	UlSetBia *= (UINT_32)UsHalAdjRange ;
	if( (UINT_32)( StTneVal.StDwdVal.UsHigVal + StTneVal.StDwdVal.UsLowVal ) >= (UINT_32)0xFFFF )
	{
		UlSetBia = BIAS_HLMT ;
	}else{
		UlSetBia /= (UINT_32)( 0xFFFF - ( StTneVal.StDwdVal.UsHigVal + StTneVal.StDwdVal.UsLowVal )) ;
		if( UlSetBia > (UINT_32)0x0000FFFF )		UlSetBia = 0x0000FFFF ;
		UlSetBia = ( UlSetBia << 16 ) ;
		if( UlSetBia > BIAS_HLMT ) {
CAM_ERR(CAM_OIS, "( AXIS = %02x , BIAS CLIP %08xh -> %08xh ) , \n", UcTneAxs , (INT_32)UlSetBia, (INT_32)BIAS_HLMT ) ;
			UlSetBia = BIAS_HLMT ;
		}
		if( UlSetBia < BIAS_LLMT ) {
			UlSetBia = BIAS_LLMT ;
CAM_ERR(CAM_OIS, "( AXIS = %02x , BIAS CLIP %08xh -> %08xh ) , \n", UcTneAxs , (INT_32)UlSetBia, (INT_32)BIAS_LLMT ) ;
		}
	}

CAM_ERR(CAM_OIS, "( AXIS = %02x , BIAS = %08xh ) , \n", UcTneAxs , (INT_32)UlSetBia ) ;
	if( UcTneAxs == X_DIR ) {
		DacControl( 0 , HLXBO , UlSetBia ) ;
		RamWrite32A( StSmaCaliData_UiHallBias_X , UlSetBia) ;
		StAdjPar.StHalAdj.UsHlxGan = (UINT_16)( UlSetBia >> 16 ) ;		
	} else if( UcTneAxs == Y_DIR ){
		DacControl( 0 , HLYBO , UlSetBia ) ;
		RamWrite32A( StSmaCaliData_UiHallBias_Y , UlSetBia) ;
		StAdjPar.StHalAdj.UsHlyGan = (UINT_16)( UlSetBia >> 16 ) ;		
	} else {
		DacControl( 0 , HLZBO , UlSetBia ) ;
		RamWrite32A( StSmaCaliData_UiHallBias_Z , UlSetBia) ;
		StAdjPar.StHalAdj.UsHlzGan = (UINT_16)( UlSetBia >> 16 ) ;					
	}
}

//********************************************************************************
// Function Name 	: TneOff_TRIPLE
// Retun Value		: None
// Argment Value	: Hall Top & Bottom Gaps , X,Y Direction
// Explanation		: Hall Offset Tuning Function
// History			: First edition 						
//********************************************************************************
void TneOff_TRIPLE( UnDwdVal StTneVal, UINT_8 UcTneAxs )
{
	UINT_32	UlSetOff = 0;
	UINT_32	UlSetVal ;
	INT_32	SiSign = 1;
	
CAM_ERR(CAM_OIS, "TneOff\n ") ;
	if( UcTneAxs == X_DIR ) {
		RamRead32A( StSmaCaliData_UiHallOffsetO_X , &UlSetOff ) ;
		SiSign = +1;
	} else if( UcTneAxs == Y_DIR ){
		RamRead32A( StSmaCaliData_UiHallOffsetO_Y , &UlSetOff ) ;
		SiSign = +1;
	} else if( UcTneAxs == Z_DIR ){
		RamRead32A( StSmaCaliData_UiHallOffsetO_Z , &UlSetOff ) ;
		SiSign = -1;
	}
	UlSetOff 	= ( UlSetOff >> 16 ) ;

	if ( StTneVal.StDwdVal.UsHigVal > StTneVal.StDwdVal.UsLowVal ) {
		UlSetVal	= ( UINT_32 )(( StTneVal.StDwdVal.UsHigVal - StTneVal.StDwdVal.UsLowVal ) / OFFSET_DIV ) ;	// Calculating Value For Increase Step
//@		UlSetOff	+= UlSetVal ;	// Calculating Value For Increase Step
		UlSetOff	+= (INT_32)UlSetVal * SiSign ;	// Calculating Value For Increase Step
		if( UlSetOff > 0x0000FFFF )		UlSetOff = 0x0000FFFF ;
	} else {
		UlSetVal	= ( UINT_32 )(( StTneVal.StDwdVal.UsLowVal - StTneVal.StDwdVal.UsHigVal ) / OFFSET_DIV ) ;	// Calculating Value For Decrease Step
		if( UlSetOff < UlSetVal ){
			UlSetOff	= 0x00000000 ;
		}else{
//@			UlSetOff	-= UlSetVal ;	// Calculating Value For Decrease Step
			UlSetOff	-= (INT_32)UlSetVal * SiSign ;	// Calculating Value For Decrease Step
		}
	}

CAM_ERR(CAM_OIS, "		UlSetOff = %08x\n ",  (unsigned int)UlSetOff ) ;
	if( UlSetOff > ( INT_32 )0x0000EFFF ) {
		UlSetOff	= 0x0000EFFF ;
	} else if( UlSetOff < ( INT_32 )0x00001000 ) {
		UlSetOff	= 0x00001000 ;
	}

	UlSetOff = ( UlSetOff << 16 ) ;
	
	if( UcTneAxs == X_DIR ) {
		DacControl( 0, HLXOO, UlSetOff ) ;
CAM_ERR(CAM_OIS, "		HLXOO = %08x\n ",  (unsigned int)UlSetOff ) ;
		RamWrite32A( StSmaCaliData_UiHallOffsetO_X , UlSetOff ) ;
	} else if( UcTneAxs == Y_DIR ){
		DacControl( 0, HLYOO, UlSetOff ) ;
CAM_ERR(CAM_OIS, "		HLYOO = %08x\n ",  (unsigned int)UlSetOff ) ;
		RamWrite32A( StSmaCaliData_UiHallOffsetO_Y , UlSetOff ) ;
	} else if( UcTneAxs == Z_DIR ){
		DacControl( 0, HLZOO, UlSetOff ) ;
CAM_ERR(CAM_OIS, "		HLZOO = %08x\n ",  (unsigned int)UlSetOff ) ;
		RamWrite32A( StSmaCaliData_UiHallOffsetO_Z , UlSetOff ) ;
	}
CAM_ERR(CAM_OIS, "		( AXIS = %02x , OFST = %08xh ) , \n", UcTneAxs , (unsigned int)UlSetOff ) ;

}


//********************************************************************************
// Function Name 	: TneRun_LS
// Retun Value		: Hall Tuning SUCCESS or FAILURE
// Argment Value	: NON
// Explanation		: Hall System Auto Adjustment Function
// History			: First edition 									2015.08.12
//********************************************************************************
UINT_32	TneRun_LS( void )
{
	UINT_32	UlAtxSts, UlAtySts; 
	UINT_32	UlFinSts ;
	UINT_8	uc_read_data ;
	UINT_32	UlCurLmtX, UlCurLmtY;
	ADJ_HALL_LS *HallPtr;
	ADJ_LOPGAN *LopgainPtr;
	DSPVER Info;

	LS_SS_Sel	= 1 ;
	//--------------------------------------
	// Initialize calibration data
	//--------------------------------------
	// Select parameter
	if( GetInfomation( &Info ) != 0) {
		return( EXE_ERR );
	}

	if( Info.ActVersion == 0x07 ) {			// TAH1120
		HallPtr = (ADJ_HALL_LS*)&LS_HallCalParameter[ 0 ];
		LopgainPtr = (ADJ_LOPGAN *)&LS_LoopGainParameter[ 0 ];
CAM_ERR(CAM_OIS, " Select Act : TAH1120 LS \n" );

	} else {
		return( FAILURE );
	}

CAM_ERR(CAM_OIS, "   Hall Calibration \n" );
CAM_ERR(CAM_OIS, " 	BiasInit      : %08X \n" , HallPtr->BiasInit      );
CAM_ERR(CAM_OIS, " 	XOffsetInit   : %08X \n" , HallPtr->XOffsetInit   );
CAM_ERR(CAM_OIS, " 	YOffsetInit   : %08X \n" , HallPtr->YOffsetInit   );
CAM_ERR(CAM_OIS, " 	OffsetMargin  : %04X \n" , HallPtr->OffsetMargin  );
CAM_ERR(CAM_OIS, " 	XTargetRange  : %04X \n" , HallPtr->XTargetRange  );
CAM_ERR(CAM_OIS, " 	XTargetMax    : %04X \n" , HallPtr->XTargetMax    );
CAM_ERR(CAM_OIS, " 	XTargetMin    : %04X \n" , HallPtr->XTargetMin    );
CAM_ERR(CAM_OIS, " 	YTargetRange  : %04X \n" , HallPtr->YTargetRange  );
CAM_ERR(CAM_OIS, " 	YTargetMax    : %04X \n" , HallPtr->YTargetMax    );
CAM_ERR(CAM_OIS, " 	YTargetMin    : %04X \n" , HallPtr->YTargetMin    );
#ifndef BALL_TYPE
CAM_ERR(CAM_OIS, " 	OisSinNum     : %08X \n" , HallPtr->OisSinNum     );
CAM_ERR(CAM_OIS, " 	OisSinFreq    : %08X \n" , HallPtr->OisSinFreq    );
CAM_ERR(CAM_OIS, " 	OisSinGain    : %08X \n" , HallPtr->OisSinGain    );
#else
CAM_ERR(CAM_OIS, " 	Max Current   : %08X \n" , HallPtr->Max_Current   );
CAM_ERR(CAM_OIS, " 	Min Current   : %08X \n" , HallPtr->Min_Current   );
CAM_ERR(CAM_OIS, " 	Current Step  : %08X \n" , HallPtr->Current_Step  );
CAM_ERR(CAM_OIS, " 	Step Wait Time: %08X \n" , HallPtr->Step_Wait_Time);
CAM_ERR(CAM_OIS, " 	Measure Number: %08X \n" , HallPtr->MeasureNum    );
#endif
CAM_ERR(CAM_OIS, " 	DecrementStep : %04X \n" , HallPtr->DecrementStep );
CAM_ERR(CAM_OIS, "   Loop Gain \n" );
CAM_ERR(CAM_OIS, " 	Hxgain        : %08X \n" , LopgainPtr->Hxgain     );
CAM_ERR(CAM_OIS, " 	Hygain        : %08X \n" , LopgainPtr->Hygain     );
CAM_ERR(CAM_OIS, " 	XNoiseNum     : %08X \n" , LopgainPtr->XNoiseNum  );
CAM_ERR(CAM_OIS, " 	XNoiseFreq    : %08X \n" , LopgainPtr->XNoiseFreq );
CAM_ERR(CAM_OIS, " 	XNoiseGain    : %08X \n" , LopgainPtr->XNoiseGain );
CAM_ERR(CAM_OIS, " 	XGap          : %08X \n" , LopgainPtr->XGap       );
CAM_ERR(CAM_OIS, " 	YNoiseNum     : %08X \n" , LopgainPtr->YNoiseNum  );
CAM_ERR(CAM_OIS, " 	YNoiseFreq    : %08X \n" , LopgainPtr->YNoiseFreq );
CAM_ERR(CAM_OIS, " 	YNoiseGain    : %08X \n" , LopgainPtr->YNoiseGain );
CAM_ERR(CAM_OIS, " 	YGap          : %08X \n" , LopgainPtr->YGap       );
CAM_ERR(CAM_OIS, " 	XJudgeHigh    : %08X \n" , LopgainPtr->XJudgeHigh );
CAM_ERR(CAM_OIS, " 	XJudgeLow     : %08X \n" , LopgainPtr->XJudgeLow  );
CAM_ERR(CAM_OIS, " 	YJudgeHigh    : %08X \n" , LopgainPtr->YJudgeHigh );
CAM_ERR(CAM_OIS, " 	YJudgeLow     : %08X \n" , LopgainPtr->YJudgeLow  );

#if ( USE_AF == 1 )
	/* Disable AF corrections */
	RamRead32A( CLAF_RAMA_AFCNT, &UlReadVal );
	UlReadVal &= ~0x00003000;	// bit12, bit13 off
	RamWrite32A( CLAF_RAMA_AFCNT, UlReadVal );
#endif

	/* OIS hall current limiter backup */
	RamRead32A( HallCurrentLimitX, &UlCurLmtX );
	RamRead32A( HallCurrentLimitY, &UlCurLmtY );
	/* OIS hall current limiter relese */
	RamWrite32A( HallCurrentLimitX, 0x7FFFFFFF );
	RamWrite32A( HallCurrentLimitY, 0x7FFFFFFF );

	/* Hall Adjustment */
	UlFinSts = HallAdj( HallPtr, LopgainPtr );

	/* OIS hall current limiter set */
	RamWrite32A( HallCurrentLimitX, UlCurLmtX );
	RamWrite32A( HallCurrentLimitY, UlCurLmtY );

	if( ((UlFinSts & EXE_HXADJ) == EXE_HXADJ) || ((UlFinSts & EXE_HYADJ) == EXE_HYADJ) ) return ( UlFinSts );

	//--------------------------------------
	// Calibration mecha/neutral center
	//--------------------------------------
#ifdef	NEUTRAL_CENTER
	TneHvc();
#endif	//NEUTRAL_CENTER

	WitTim( TNE );

	StAdjPar.StHalAdj.UsAdxOff = StAdjPar.StHalAdj.UsHlxCna;
	StAdjPar.StHalAdj.UsAdyOff = StAdjPar.StHalAdj.UsHlyCna;

	RamWrite32A( HALL_RAM_HXOFF, (UINT_32)((StAdjPar.StHalAdj.UsAdxOff << 16 ) & 0xFFFF0000 ) );
	RamWrite32A( HALL_RAM_HYOFF, (UINT_32)((StAdjPar.StHalAdj.UsAdyOff << 16 ) & 0xFFFF0000 ) );

	//---------------------------------------------------
	// Copy Hall Bias/Offset data to temporary variable
	//---------------------------------------------------
	RegRead402( Y_402, DAHLO_402, &uc_read_data );
	StAdjPar.StHalAdj.UsHlxOffO	= ( UINT_16 )uc_read_data ;
CAM_ERR(CAM_OIS, "***StAdjPar.StHalAdj.UsHlxOffO = %04X\n", StAdjPar.StHalAdj.UsHlxOffO);

	RegRead402( Y_402, DAHLB_402, &uc_read_data );
	StAdjPar.StHalAdj.UsHlxGan	= ( UINT_16 )uc_read_data ;
CAM_ERR(CAM_OIS, "***StAdjPar.StHalAdj.UsHlxGan = %04X\n", StAdjPar.StHalAdj.UsHlxGan);

	RegRead402( X_402, DAHLO_402, &uc_read_data );
	StAdjPar.StHalAdj.UsHlyOffO	= ( UINT_16 )uc_read_data ;
CAM_ERR(CAM_OIS, "***StAdjPar.StHalAdj.UsHlyOffO = %04X\n", StAdjPar.StHalAdj.UsHlyOffO);

	RegRead402( X_402, DAHLB_402, &uc_read_data );
	StAdjPar.StHalAdj.UsHlyGan = ( UINT_16 )uc_read_data ;
CAM_ERR(CAM_OIS, "***StAdjPar.StHalAdj.UsHlyGan = %04X\n", StAdjPar.StHalAdj.UsHlyGan);


	//--------------------------------------
	// Fine calibration neutral center
	//--------------------------------------
#ifdef	NEUTRAL_CENTER_FINE
	TneFin();

CAM_ERR(CAM_OIS, "    XadofFin = %04xh \n", StAdjPar.StHalAdj.UsAdxOff ) ;
CAM_ERR(CAM_OIS, "    YadofFin = %04xh \n", StAdjPar.StHalAdj.UsAdyOff ) ;
		RamWrite32A( HALL_RAM_HXOFF,  (UINT_32)((StAdjPar.StHalAdj.UsAdxOff << 16 ) & 0xFFFF0000 )) ;
		RamWrite32A( HALL_RAM_HYOFF,  (UINT_32)((StAdjPar.StHalAdj.UsAdyOff << 16 ) & 0xFFFF0000 )) ;
#endif	//NEUTRAL_CENTER_FINE

	//--------------------------------------
	// Adjust loop gain
	//--------------------------------------
	RtnCen_LS( BOTH_ON );								// Y ON / X ON
	WitTim( TNE );

	UlAtxSts = LopGan_LS( X_DIR, LopgainPtr );				// X Loop Gain Adjust
	UlAtySts = LopGan_LS( Y_DIR, LopgainPtr );				// Y Loop Gain Adjust

	//--------------------------------------
	// Error check
	//--------------------------------------
	UlFinSts |= UlAtxSts | UlAtySts;
	if( UlFinSts != EXE_END ) {
CAM_ERR(CAM_OIS, "***Loop gain adj fail = %04X:%04X[%04X]\n", UlAtxSts, UlAtySts, UlFinSts );
		RtnCen_LS( BOTH_OFF );								// Both OFF
		StAdjPar.StHalAdj.UlAdjPhs = UlFinSts;
		return( UlFinSts );
	}

	//--------------------------------------
	// Calculation adjust status
	//--------------------------------------
	UlFinSts = UlFinSts | UlAtxSts | UlAtySts | EXE_END;
	StAdjPar.StHalAdj.UlAdjPhs = UlFinSts;

	return( UlFinSts ) ;
}

//********************************************************************************
// Function Name 	: TneCen
// Retun Value		: Hall Center Tuning Result
// Argment Value	: X,Y Direction, Hall Top & Bottom Gaps
// Explanation		: Hall Center Tuning Function
// History			: First edition 									2015.08.12
//********************************************************************************
UINT_32	TneCen( UINT_8 UcTneAxs, ADJ_HALL_LS* ptr , UINT_8 UcBfrAft )
{
	UnDwdVal StTneVal;
	UINT_16 UsValBef = 0, UsValNow = 0;
	UINT_8 	UcTmeOut, UcTofRst;
	UINT_8	UcValBefB, UcValNowB;
	UINT_8	UcBiasVal, UcValNow;
	UINT_16	UsBiasVal ;
	UINT_32	UlTneRst;
	UINT_16	UsTargetMax, UsTargetMin;
	UINT_16	UsTargetRange;
	UINT_32 headroom;

#if ( USE_AF == 1 )
	if (UcTneAxs == Z_DIR)
		headroom = 0x2666;	//15% headroom in each end
	else
#endif
		headroom = ptr->OffsetMargin;

	UcTmeOut = 1;
	UlTneRst = FAILURE;
	UcTofRst = FAILURE;

	// Measurement amplitude
	StTneVal.UlDwdVal = TnePtp( UcTneAxs , UcBfrAft, ptr );
	
	if(UcTneAxs == X_DIR) {
		RegRead402( Y_402, DAHLB_402, &UcValNowB );
	}else if(UcTneAxs == Y_DIR) {
		RegRead402( X_402, DAHLB_402, &UcValNowB );
	}
#if ( USE_AF == 1 )
	else{
		RamRead32A( StCaliData_UiHallBias_Z, &UlValNowB );
	}
#endif

	TneOff( StTneVal, UcTneAxs );
	UcTofRst = SUCCESS;													// Set succsess tentative

	while ( UlTneRst && (UINT_32)UcTmeOut ) {
		UcValBefB = UcValNowB;
		
		if( UcTofRst == FAILURE ) {
			CAM_ERR(CAM_OIS, " UcTofRst == FAILURE\n" ) ;
			TneOff( StTneVal, UcTneAxs );

		} else {
			if(UcTneAxs == X_DIR) {
				UsTargetRange = (UINT_16)ptr->XTargetRange;
			} else {
				UsTargetRange = (UINT_16)ptr->YTargetRange;
			}
#if ( USE_AF == 1 )
			else {
				UsTargetRange = (UINT_16)ptr->ZTargetRange;
			}
#endif
			TneBia( StTneVal, UcTneAxs, UsTargetRange );				// Hall Bias change

			UcTofRst = FAILURE;
		}

		// Measurement amplitude
		StTneVal.UlDwdVal = TnePtp( UcTneAxs, PTP_AFTER, ptr );
		
		if(UcTneAxs == X_DIR) {
			RegRead402( Y_402, DAHLB_402, &UcValNowB );
		} else if(UcTneAxs == Y_DIR) {
			RegRead402( X_402, DAHLB_402, &UcValNowB );
		}
#if ( USE_AF == 1 )
		else {
			RamRead32A( StCaliData_UiHallBias_Z, &UlValNowB );
		}
#endif
		CAM_ERR(CAM_OIS, " CMP BiasVal = %02X , %02X\n", UcValBefB , UcValNowB);
		
		if( (( UcValBefB == BIAS_HLMT ) && ( UcValNowB == BIAS_HLMT )) ||
			(( UcValBefB == BIAS_LLMT ) && ( UcValNowB == BIAS_LLMT )) ) {
			UcTmeOut += 2;
		}

		//if( (StTneVal.StDwdVal.UsHigVal > ptr->OffsetMargin ) && (StTneVal.StDwdVal.UsLowVal > ptr->OffsetMargin ) ) {
		if ((StTneVal.StDwdVal.UsHigVal > headroom) && (StTneVal.StDwdVal.UsLowVal > headroom)) {
			UcTofRst = SUCCESS;
			UsValBef = UsValNow = 0x0000;
		//} else if( (StTneVal.StDwdVal.UsHigVal <= ptr->OffsetMargin ) && (StTneVal.StDwdVal.UsLowVal <= ptr->OffsetMargin ) ) {
		} else if( (StTneVal.StDwdVal.UsHigVal <= headroom) && (StTneVal.StDwdVal.UsLowVal <= headroom) ) {
			UcTofRst = SUCCESS;
			UlTneRst = (UINT_32)FAILURE;
		} else {
			UcTofRst = FAILURE;
			UsValBef = UsValNow;

			if( UcTneAxs == X_DIR ) {
				RegRead402( Y_402, DAHLO_402, &UcValNow );
				UsValNow = ( ( UINT_16 )UcValNow << 8 );
				CAM_ERR(CAM_OIS, "X_UsValNow = %08X\n", (UINT_32)UsValNow);
			} else if( UcTneAxs == Y_DIR ) {
				RegRead402( X_402, DAHLO_402, &UcValNow );
				UsValNow = ( ( UINT_16 )UcValNow << 8 );
				CAM_ERR(CAM_OIS, "Y_UsValNow = %08X\n", (UINT_32)UsValNow);
			}
#if ( USE_AF == 1 )
			else {
				RamRead32A( StCaliData_UiHallOffsetO_Z, &UlValNow );
				UsValNow = (UINT_16)( UlValNow >> 16 );
				CAM_ERR(CAM_OIS, "AF_UsValNow = %08X\n",(UINT_32)UsValNow);
			}
#endif

			if( ((( UsValBef & 0xFF00 ) == 0x1000 ) && ( UsValNow & 0xFF00 ) == 0x1000 ) ||
				((( UsValBef & 0xFF00 ) == 0xDF00 ) && ( UsValNow & 0xFF00 ) == 0xDF00 ) ) {
				UcTmeOut += 2;
				if( UcTneAxs == X_DIR ) {
					RegRead402( Y_402, DAHLB_402, &UcBiasVal );
					UsBiasVal = ( ( UINT_16 )UcBiasVal << 8 );
				} else {
					RegRead402( X_402, DAHLB_402, &UcBiasVal );
					UsBiasVal = ( ( UINT_16 )UcBiasVal << 8 );
				}
#if ( USE_AF == 1 )
				else {
					RamRead32A( StCaliData_UiHallBias_Z, &UlBiasVal );
					UsBiasVal = (UINT_16)( UlBiasVal >> 16 );
				}
#endif

				if( UsBiasVal > ptr->DecrementStep ) {
					UsBiasVal -= ptr->DecrementStep;
				}

				if( UcTneAxs == X_DIR ) {
					UcBiasVal = (UINT_8)( UsBiasVal >> 8 );
					RegWrite402( Y_402, DAHLB_402, UcBiasVal );
				} else if( UcTneAxs == Y_DIR ) {
					UcBiasVal = (UINT_8)( UsBiasVal >> 8 );
					RegWrite402( X_402, DAHLB_402, UcBiasVal );
				}
#if ( USE_AF == 1 )
				else {
					UlBiasVal = (UINT_32)( UsBiasVal << 16 );
					DacControl( HLAFBO, UlBiasVal );
					RamWrite32A( StCaliData_UiHallBias_Z, UlBiasVal );
				}
#endif
			}
		}

		if(UcTneAxs == X_DIR) {
			UsTargetMax = (UINT_16)ptr->XTargetMax;
			UsTargetMin = (UINT_16)ptr->XTargetMin;
		} else {
			UsTargetMax = (UINT_16)ptr->YTargetMax;
			UsTargetMin = (UINT_16)ptr->YTargetMin;
		}
#if ( USE_AF == 1 )
		else {
			UsTargetMax = (UINT_16)ptr->ZTargetMax;
			UsTargetMin = (UINT_16)ptr->ZTargetMin;
		}
#endif

		if( (( (UINT_16)0xFFFF - ( StTneVal.StDwdVal.UsHigVal + StTneVal.StDwdVal.UsLowVal )) < UsTargetMax ) &&
			(( (UINT_16)0xFFFF - ( StTneVal.StDwdVal.UsHigVal + StTneVal.StDwdVal.UsLowVal )) > UsTargetMin ) ) {
			if(UcTofRst	== SUCCESS) {
				UlTneRst = (UINT_32)SUCCESS;
				break;
			}
		}
		UlTneRst = (UINT_32)FAILURE;
		UcTmeOut++;

		if( UcTmeOut >= TIME_OUT ) {									// Set Timeout count
			UcTmeOut = 0;
		}
	}

	SetSinWavGenInt();

	if( UlTneRst == (UINT_32)FAILURE ) {
		if( UcTneAxs == X_DIR ) {
			UlTneRst = EXE_HXADJ;
//			StAdjPar.StHalAdj.UsHlxGan	= 0xFFFF;
//			StAdjPar.StHalAdj.UsHlxOffO	= 0xFFFF;
		} else if( UcTneAxs == Y_DIR ) {
			UlTneRst = EXE_HYADJ;
//			StAdjPar.StHalAdj.UsHlyGan	= 0xFFFF;
//			StAdjPar.StHalAdj.UsHlyOffO	= 0xFFFF;
		}
#if ( USE_AF == 1 )
		else {
			UlTneRst = EXE_HZADJ;
//			StAdjPar.StHalAdj.UsHlzGan	= 0xFFFF;
//			StAdjPar.StHalAdj.UsHlzOffO	= 0xFFFF;
		}
#endif
	} else {
		UlTneRst = EXE_END;
	}

	return( UlTneRst );
}

//********************************************************************************
// Function Name 	: TnePtp
// Retun Value		: Hall Top & Bottom Gaps
// Argment Value	: X,Y Direction, Adjust Before After Parameter
// Explanation		: Measuring Hall Paek To Peak
// History			: First edition
//********************************************************************************
UINT_32	TnePtp( UINT_8 UcDirSel, UINT_8	UcBfrAft, ADJ_HALL_LS *p )
{
	UnDwdVal StTneVal;
	INT_32 SlMeasureParameterA, SlMeasureParameterB;
	INT_32 SlMeasureMaxValue, SlMeasureMinValue;

#ifdef BALL_TYPE
	UINT_16	us_current_address ;
	INT_32	sl_current_step ;
	UnllnVal	StMeasValueA ;

	if( UcDirSel == X_DIR ) {
		SlMeasureParameterA =			HALL_RAM_HXIDAT;				// Set measure RAM address
		SlMeasureParameterB =			HALL_RAM_HYIDAT;				// Set measure RAM address
		us_current_address	=			HALL_RAM_SINDX1;				// Set control RAM address
	} else {
		SlMeasureParameterA =			HALL_RAM_HYIDAT;				// Set measure RAM address
		SlMeasureParameterB =			HALL_RAM_HXIDAT;				// Set measure RAM address
		us_current_address	=			HALL_RAM_SINDY1;				// Set control RAM address
	}

	MesFil( THROUGH );													// Select filter setting for measurement

	sl_current_step	= 0 ;

	do {
		RamWrite32A( us_current_address, ( UINT_32 )sl_current_step ) ;
		WitTim( p->Step_Wait_Time ) ;
		sl_current_step	+= p->Current_Step ;
	} while( sl_current_step <= ( INT_32 )p->Max_Current ) ;

	if( sl_current_step > ( INT_32 )p->Max_Current ) {
		sl_current_step	= p->Max_Current ;
		RamWrite32A( us_current_address, ( UINT_32 )sl_current_step ) ;
	}

	MeasureStart( p->MeasureNum, SlMeasureParameterA, SlMeasureParameterB );		// Start measure
	MeasureWait();																// Wait for complete of measurement
	RamRead32A( StMeasFunc_MFA_LLiIntegral1,		&StMeasValueA.StUllnVal.UlLowVal );
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4,	&StMeasValueA.StUllnVal.UlHigVal );

	SlMeasureMinValue	= StMeasValueA.UllnValue / p->MeasureNum ;

	do {
		RamWrite32A( us_current_address, ( UINT_32 )sl_current_step ) ;
		WitTim( p->Step_Wait_Time ) ;
		sl_current_step	-= p->Current_Step ;
	} while( sl_current_step >= ( INT_32 )p->Min_Current ) ;

	if( sl_current_step < ( INT_32 )p->Min_Current ) {
		sl_current_step	= p->Min_Current ;
		RamWrite32A( us_current_address, ( UINT_32 )sl_current_step ) ;
	}
	
	MeasureStart( p->MeasureNum, SlMeasureParameterA, SlMeasureParameterB );		// Start measure
	MeasureWait();																// Wait for complete of measurement
	RamRead32A( StMeasFunc_MFA_LLiIntegral1,		&StMeasValueA.StUllnVal.UlLowVal );
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4,	&StMeasValueA.StUllnVal.UlHigVal );

	SlMeasureMaxValue	= StMeasValueA.UllnValue / p->MeasureNum ;

	RamWrite32A( us_current_address, ( UINT_32 )0 ) ;
#else

	SetSinWavGenInt();

	if( UcDirSel == X_DIR ) {											// X axis
		SlMeasureParameterA =			HALL_RAM_HXIDAT;				// Set measure RAM address
		SlMeasureParameterB =			HALL_RAM_HYIDAT;				// Set measure RAM address
		RamWrite32A( SinWave_Offset,	p->OisSinFreq );				// Freq setting = Freq * 80000000h / Fs	: 10Hz
		RamWrite32A( SinWave_Gain,		p->OisSinGain );				// Set sine wave gain
		RamWrite32A( SinWaveEx_Enable,	0x00000000 );					// Disable SinWaveEx
		RamWrite32A( SinWaveEx_GainM,	p->OisSinGain );				// Set sine wave minus gain
	} else if( UcDirSel == Y_DIR ) {									// Y axis
		SlMeasureParameterA =			HALL_RAM_HYIDAT;				// Set measure RAM address
		SlMeasureParameterB =			HALL_RAM_HXIDAT;				// Set measure RAM address
		RamWrite32A( SinWave_Offset,	p->OisSinFreq );				// Freq setting = Freq * 80000000h / Fs	: 10Hz
		RamWrite32A( SinWave_Gain,		p->OisSinGain );				// Set sine wave gain
		RamWrite32A( SinWaveEx_Enable,	0x00000000 );					// Disable SinWaveEx
		RamWrite32A( SinWaveEx_GainM,	p->OisSinGain );				// Set sine wave minus gain
	}
#if ( USE_AF == 1 )
	else {															// Z axis
		SlMeasureParameterA =			CLAF_RAMA_AFADIN;				// Set measure RAM address
		SlMeasureParameterB =			CLAF_RAMA_AFADIN;				// Set measure RAM address
		RamWrite32A( SinWave_Offset,	p->AfSinFreq );					// Freq setting = Freq * 80000000h / Fs	: 4Hz
		RamWrite32A( SinWave_Gain,		p->AfSinGainP );				// Set sine wave gain
		RamWrite32A( SinWaveEx_Enable,	0x00000001 );					// Enable SinWaveEx
		RamWrite32A( SinWaveEx_GainM,	p->AfSinGainM );				// Set sine wave minus gain
	}
#endif

	RamWrite32A( SinWaveC_Regsiter,		0x00000001 );					// Start sine wave
	if( UcDirSel == X_DIR ) {
		SetTransDataAdr( SinWave_OutAddr, (UINT_32)HALL_RAM_SINDX1 );	// Set sine wave input RAM
	}else if( UcDirSel == Y_DIR ) {
		SetTransDataAdr( SinWave_OutAddr, (UINT_32)HALL_RAM_SINDY1 );	// Set sine wave input RAM
	}
#if ( USE_AF == 1 )
	else{
		SetTransDataAdr( SinWave_OutAddr, (UINT_32)CLAF_RAMA_AFOUT );	// Set sine wave input RAM
	}
#endif

	MesFil( THROUGH );													// Select filter setting for measurement

#if ( USE_AF == 1 )
	if( UcDirSel != Z_DIR ) {											// X or Y axis
		MeasureStart2( p->OisSinNum, SlMeasureParameterA, SlMeasureParameterB, 1 );	// Start measure
	} else {
		MeasureStart2( p->AfSinNum, SlMeasureParameterA, SlMeasureParameterB, 1 );	// Start measure
	}
#else
	MeasureStart2( p->OisSinNum, SlMeasureParameterA, SlMeasureParameterB, 1 );		// Start measure
#endif
	MeasureWait();														// Wait for complete of measurement

	RamWrite32A( SinWaveC_Regsiter,	0x00000000 );						// Stop sine wave
	RamWrite32A( SinWaveEx_Enable,	0x00000000 );						// Disable SinWaveEx

	if( UcDirSel == X_DIR ) {
		SetTransDataAdr( SinWave_OutAddr,	(UINT_32)0x00000000 );		// Reset sine wave input RAM
		RamWrite32A( HALL_RAM_SINDX1,		(UINT_32)0x00000000 );		// Clear delay ram
	}else if( UcDirSel == Y_DIR ) {
		SetTransDataAdr( SinWave_OutAddr,	(UINT_32)0x00000000 );		// Reset sine wave input RAM
		RamWrite32A( HALL_RAM_SINDY1,		(UINT_32)0x00000000 );		// Clear delay ram
	}
#if ( USE_AF == 1 )
	else{
		SetTransDataAdr( SinWave_OutAddr,	(UINT_32)0x00000000 );		// Reset sine wave input RAM
		RamWrite32A( CLAF_RAMA_AFOUT,		(UINT_32)0x00000000 );		// Clear delay ram
	}
#endif
	RamRead32A( StMeasFunc_MFA_SiMax1, (UINT_32 *)&SlMeasureMaxValue );	// Get max value
	RamRead32A( StMeasFunc_MFA_SiMin1, (UINT_32 *)&SlMeasureMinValue );	// Get min value
#endif	// BALL_TYPE

	StTneVal.StDwdVal.UsHigVal = (UINT_16)((SlMeasureMaxValue >> 16) & 0x0000FFFF );
	StTneVal.StDwdVal.UsLowVal = (UINT_16)((SlMeasureMinValue >> 16) & 0x0000FFFF );

CAM_ERR(CAM_OIS, "\nPTP topbtm H = %04xh , L = %04xh , AXIS = %02x \n", StTneVal.StDwdVal.UsHigVal,StTneVal.StDwdVal.UsLowVal, UcDirSel );

	if( UcBfrAft == 0 ) {
		if( UcDirSel == X_DIR ) {
			StAdjPar.StHalAdj.UsHlxCen = ( ( INT_16 )StTneVal.StDwdVal.UsHigVal + ( INT_16 )StTneVal.StDwdVal.UsLowVal ) / 2;
			StAdjPar.StHalAdj.UsHlxMax = StTneVal.StDwdVal.UsHigVal;
			StAdjPar.StHalAdj.UsHlxMin = StTneVal.StDwdVal.UsLowVal;
		} else if( UcDirSel == Y_DIR ) {
			StAdjPar.StHalAdj.UsHlyCen = ( ( INT_16 )StTneVal.StDwdVal.UsHigVal + ( INT_16 )StTneVal.StDwdVal.UsLowVal ) / 2;
			StAdjPar.StHalAdj.UsHlyMax = StTneVal.StDwdVal.UsHigVal;
			StAdjPar.StHalAdj.UsHlyMin = StTneVal.StDwdVal.UsLowVal;
		}
#if ( USE_AF == 1 )
		else {
			StAdjPar.StHalAdj.UsHlzCen = ( ( INT_16 )StTneVal.StDwdVal.UsHigVal + ( INT_16 )StTneVal.StDwdVal.UsLowVal ) / 2;
			StAdjPar.StHalAdj.UsHlzMax = StTneVal.StDwdVal.UsHigVal;
			StAdjPar.StHalAdj.UsHlzMin = StTneVal.StDwdVal.UsLowVal;
		}
#endif
	} else {
		if( UcDirSel == X_DIR ){
			StAdjPar.StHalAdj.UsHlxCna = ( ( INT_16 )StTneVal.StDwdVal.UsHigVal + ( INT_16 )StTneVal.StDwdVal.UsLowVal ) / 2;
			StAdjPar.StHalAdj.UsHlxMxa = StTneVal.StDwdVal.UsHigVal;
			StAdjPar.StHalAdj.UsHlxMna = StTneVal.StDwdVal.UsLowVal;
		} else if( UcDirSel == Y_DIR ) {
			StAdjPar.StHalAdj.UsHlyCna = ( ( INT_16 )StTneVal.StDwdVal.UsHigVal + ( INT_16 )StTneVal.StDwdVal.UsLowVal ) / 2;
			StAdjPar.StHalAdj.UsHlyMxa = StTneVal.StDwdVal.UsHigVal;
			StAdjPar.StHalAdj.UsHlyMna = StTneVal.StDwdVal.UsLowVal;
		}
#if ( USE_AF == 1 )
		else {
			StAdjPar.StHalAdj.UsHlzCna = ( ( INT_16 )StTneVal.StDwdVal.UsHigVal + ( INT_16 )StTneVal.StDwdVal.UsLowVal ) / 2;
			StAdjPar.StHalAdj.UsHlzMxa = StTneVal.StDwdVal.UsHigVal;
			StAdjPar.StHalAdj.UsHlzMna = StTneVal.StDwdVal.UsLowVal;
		}
#endif
	}

	StTneVal.StDwdVal.UsHigVal = 0x7fff - StTneVal.StDwdVal.UsHigVal;	// Maximum Gap = Maximum - Hall Peak Top
	StTneVal.StDwdVal.UsLowVal = StTneVal.StDwdVal.UsLowVal - 0x8000; 	// Minimum Gap = Hall Peak Bottom - Minimum

CAM_ERR(CAM_OIS, "PTP margin H = %04xh , L = %04xh , AXIS = %02x \n", StTneVal.StDwdVal.UsHigVal,StTneVal.StDwdVal.UsLowVal, UcDirSel );

	return( StTneVal.UlDwdVal );
}

//********************************************************************************
// Function Name 	: TneOff
// Retun Value		: None
// Argment Value	: Hall Top & Bottom Gaps , X,Y Direction
// Explanation		: Hall Offset Tuning Function
// History			: First edition 						
//********************************************************************************
void TneOff( UnDwdVal StTneVal, UINT_8 UcTneAxs )
{
	UINT_32	UlSetOff;
	UINT_32	UlSetVal;
	UINT_8	uc_current_dac = 0, uc_updated_dac = 0 ;
	
	CAM_ERR(CAM_OIS, "TneOff\n ");
	if( UcTneAxs == X_DIR ) {
		RegRead402( Y_402, DAHLO_402, &uc_current_dac );
	} else if( UcTneAxs == Y_DIR ) {
		RegRead402( X_402, DAHLO_402, &uc_current_dac );
	}
#if ( USE_AF == 1 )
	else if( UcTneAxs == Z_DIR ) {
		RamRead32A( StCaliData_UiHallOffsetO_Z, &UlSetOff );
	}
#endif
	UlSetOff = ( ( UINT_32 )uc_current_dac << 8 );

	CAM_ERR(CAM_OIS, "		UlSetOff(Before) = %08X\n ", (UINT_32)UlSetOff );
	CAM_ERR(CAM_OIS, "		UsHigVal(Gap) = %04X\n ", StTneVal.StDwdVal.UsHigVal );
	CAM_ERR(CAM_OIS, "		UsLowVal(Gap) = %04X\n ", StTneVal.StDwdVal.UsLowVal );

#if ( USE_AF == 1 )
	if( UcTneAxs == Z_DIR ) {
		if( StTneVal.StDwdVal.UsHigVal > StTneVal.StDwdVal.UsLowVal ) {
			UlSetVal = (UINT_32)( ( StTneVal.StDwdVal.UsHigVal - StTneVal.StDwdVal.UsLowVal ) / OFFSET_DIV );		// Calculating value for decrease step
			if( UlSetOff < UlSetVal ) {
				UlSetOff = 0x00000000;
			} else {
				UlSetOff -= UlSetVal;
			}
		} else {
			UlSetVal = (UINT_32)( ( StTneVal.StDwdVal.UsLowVal - StTneVal.StDwdVal.UsHigVal ) / OFFSET_DIV );		// Calculating value for increase step
			UlSetOff += UlSetVal;
			if( UlSetOff > 0x0000FFFF )	UlSetOff = 0x0000FFFF;
		}
	} else {
#endif
		if( StTneVal.StDwdVal.UsHigVal > StTneVal.StDwdVal.UsLowVal ) {
			UlSetVal = (UINT_32)(( StTneVal.StDwdVal.UsHigVal - StTneVal.StDwdVal.UsLowVal ) / OFFSET_DIV );		// Calculating value for increase step
			UlSetOff += UlSetVal;
			if( UlSetOff > 0x0000FFFF )	UlSetOff = 0x0000FFFF;
		} else {
			UlSetVal = (UINT_32)( ( StTneVal.StDwdVal.UsLowVal - StTneVal.StDwdVal.UsHigVal ) / OFFSET_DIV );		// Calculating value for decrease step
			if( UlSetOff < UlSetVal ) {
				UlSetOff = 0x00000000;
			} else {
				UlSetOff -= UlSetVal;
			}
		}
#if ( USE_AF == 1 )
	}
#endif

	CAM_ERR(CAM_OIS, "		UlSetOff = %08X\n ",  (UINT_32)UlSetOff );
	if( UlSetOff > (INT_32)0x0000DFFF ) {
		UlSetOff = 0x0000DFFF;
	} else if( UlSetOff < (INT_32)0x00002000 ) {
		UlSetOff = 0x00002000;
	}

	uc_updated_dac	= ( UINT_8 )( UlSetOff >> 8 );
	
	if( UcTneAxs == X_DIR ) {
		RegWrite402( Y_402, DAHLO_402, uc_updated_dac );
		CAM_ERR(CAM_OIS, "		HLXOO = %02X\n ",  uc_updated_dac );
	} else if( UcTneAxs == Y_DIR ) {
		RegWrite402( X_402, DAHLO_402, uc_updated_dac );
		CAM_ERR(CAM_OIS, "		HLYOO = %02X\n ",  uc_updated_dac );
	}
#if ( USE_AF == 1 )
	else if( UcTneAxs == Z_DIR ) {
		DacControl( HLAFOO, UlSetOff );
		CAM_ERR(CAM_OIS, "		HLZOO = %08x\n ",  (UINT_32)UlSetOff );
		RamWrite32A( StCaliData_UiHallOffsetO_Z , UlSetOff );
	}
#endif
	CAM_ERR(CAM_OIS, "		( AXIS = %02X , OFST = %02Xh ) , \n", UcTneAxs , uc_updated_dac );
}


//********************************************************************************
// Function Name 	: TneBia
// Retun Value		: None
// Argment Value	: Hall Top & Bottom Gaps , X,Y Direction
// Explanation		: Hall Bias Tuning Function
// History			: First edition
//********************************************************************************
#define	BIAS_HLMT2	0xBF000000
#define	BIAS_LLMT2	0x20000000
void TneBia( UnDwdVal StTneVal, UINT_8 UcTneAxs, UINT_16 UsHalAdjRange )
{
	UINT_32 UlSetBia;
	UINT_8	uc_current_dac	= 0 , uc_updated_dac	= 0 ;

	CAM_ERR(CAM_OIS, "TneBia\n ");
	if( UcTneAxs == X_DIR ) {
		RegRead402( Y_402, DAHLB_402, &uc_current_dac );
	} else if( UcTneAxs == Y_DIR ) {
		RegRead402( X_402, DAHLB_402, &uc_current_dac );
	}
#if ( USE_AF == 1 )
	else {
		RamRead32A( StCaliData_UiHallBias_Z, &UlSetBia );
	}
#endif

	UlSetBia	= ( ( UINT_32 )uc_current_dac << 24 ) ;

	if(( StTneVal.StDwdVal.UsHigVal >= 0x0000) && ( StTneVal.StDwdVal.UsHigVal <= 0x0100) ){
//		UlSetBia -= ( UlSetBia >> 2 );
		UlSetBia -= 0x01000000;
		if( UlSetBia < BIAS_LLMT2 )		UlSetBia = BIAS_LLMT2 ;
	}else if(( StTneVal.StDwdVal.UsHigVal + StTneVal.StDwdVal.UsLowVal ) == 0xFFFF ){
		UlSetBia = BIAS_HLMT2 ;
	}else{
		if( UlSetBia == 0x00000000 )	UlSetBia = 0x01000000;
		UlSetBia = (( UlSetBia >> 16 ) & (UINT_32)0x0000FF00 );
		UlSetBia *= (UINT_32)UsHalAdjRange;
		UlSetBia /= (UINT_32)( 0xFFFF - ( StTneVal.StDwdVal.UsHigVal + StTneVal.StDwdVal.UsLowVal ) );
		
		if( UlSetBia > (UINT_32)0x0000FFFF )	UlSetBia = 0x0000FFFF;
		uc_updated_dac = ( UINT_8 )( UlSetBia >> 8 );
		if( uc_updated_dac > BIAS_HLMT_LS )	uc_updated_dac = BIAS_HLMT_LS;
		if( uc_updated_dac < BIAS_LLMT_LS )	uc_updated_dac = BIAS_LLMT_LS;
	}

	CAM_ERR(CAM_OIS, "( AXIS = %02x , BIAS = %02xh ) , \n", UcTneAxs , uc_updated_dac );
	if( UcTneAxs == X_DIR ) {
		RegWrite402( Y_402, DAHLB_402, uc_updated_dac );
	} else if( UcTneAxs == Y_DIR ){
		RegWrite402( X_402, DAHLB_402, uc_updated_dac );
	}
#if ( USE_AF == 1 )
	else {
		DacControl( HLAFBO, UlSetBia );
		RamWrite32A( StCaliData_UiHallBias_Z, UlSetBia );
	}
#endif
}

//********************************************************************************
// Function Name 	: LopGan_LS
// Retun Value		: Execute Result
// Argment Value	: X,Y Direction
// Explanation		: Loop Gain Adjust Function
// History			: First edition
//********************************************************************************
UINT_32	LopGan_LS( UINT_8 UcDirSel, ADJ_LOPGAN *ptr )
{
	UINT_32		UlReturnState = EXE_END;

#ifdef	BALL_TYPE
	if( UcDirSel == X_DIR ) {							// X axis
		RamWrite32A( HallFilterCoeffX_hxgain0 , ptr->Hxgain ) ;
		StAdjPar.StLopGan.UlLxgVal = ptr->Hxgain ;
		UlReturnState = EXE_END ;
	} else if( UcDirSel == Y_DIR ){						// Y axis
		RamWrite32A( HallFilterCoeffY_hygain0 , ptr->Hygain ) ;
		StAdjPar.StLopGan.UlLygVal = ptr->Hygain ;
		UlReturnState = EXE_END ;
	}
#else
	UnllnVal	StMeasValueA, StMeasValueB;
	INT_32		SlMeasureParameterA, SlMeasureParameterB;
	UINT_64		UllCalculateVal;
	UINT_16		UsSinAdr;
	UINT_32		UlFreq, UlGain;
	INT_32		SlNum;
	UINT_32		UlSwitchBk;

	if( UcDirSel == X_DIR ) {											// X axis
		SlMeasureParameterA = HALL_RAM_HXOUT1;							// Set measure RAM address
		SlMeasureParameterB = HALL_RAM_HXLOP;							// Set measure RAM address
		RamWrite32A( HallFilterCoeffX_hxgain0, ptr->Hxgain );
		UsSinAdr = HALL_RAM_SINDX0;
		UlFreq = ptr->XNoiseFreq;
		UlGain = ptr->XNoiseGain;
		SlNum  = ptr->XNoiseNum;

	} else if( UcDirSel == Y_DIR ) {									// Y axis
		SlMeasureParameterA = HALL_RAM_HYOUT1;							// Set measure RAM address
		SlMeasureParameterB = HALL_RAM_HYLOP;							// Set measure RAM address
		RamWrite32A( HallFilterCoeffY_hygain0, ptr->Hygain );
		UsSinAdr = HALL_RAM_SINDY0;
		UlFreq = ptr->YNoiseFreq;
		UlGain = ptr->YNoiseGain;
		SlNum  = ptr->YNoiseNum;

	}
#if ( USE_AF == 1 )
	else {															// AF axis
		SlMeasureParameterA = CLAF_RAMA_AFLOP2;							// Set measure RAM address
		SlMeasureParameterB = CLAF_DELAY_AFPZ0;							// Set measure RAM address
		RamWrite32A( CLAF_Gain_afloop2,  ptr->Hzgain );
		UsSinAdr = CLAF_RAMA_AFCNTO;
		UlFreq = ptr->ZNoiseFreq;
		UlGain = ptr->ZNoiseGain;
		SlNum  = ptr->ZNoiseNum;
		RamRead32A( CLAF_RAMA_AFCNT, &UlSwitchBk );						// FST control OFF
		RamWrite32A( CLAF_RAMA_AFCNT, UlSwitchBk & 0xffffff0f );
	}
#endif

	SetSinWavGenInt();

	RamWrite32A( SinWave_Offset,	UlFreq );							// Freq setting
	RamWrite32A( SinWave_Gain,		UlGain );							// Set sine wave gain

	CAM_ERR(CAM_OIS, "		Loop Gain %d , Freq %08xh, Gain %08xh , Num %08xh \n", UcDirSel, UlFreq, UlGain, SlNum );
	RamWrite32A( SinWaveC_Regsiter,	0x00000001 );						// Sine Wave Start

	SetTransDataAdr( SinWave_OutAddr, (UINT_32)UsSinAdr );				// Set sine wave input RAM

	MesFil( LOOPGAIN );													// Filter setting for measurement

	MeasureStart( SlNum , SlMeasureParameterA , SlMeasureParameterB );	// Start measure

	MeasureWait();														// Wait for complete of measurement

	RamRead32A( StMeasFunc_MFA_LLiAbsInteg1,		&StMeasValueA.StUllnVal.UlLowVal );	// X axis
	RamRead32A( StMeasFunc_MFA_LLiAbsInteg1 + 4,	&StMeasValueA.StUllnVal.UlHigVal );
	RamRead32A( StMeasFunc_MFB_LLiAbsInteg2,		&StMeasValueB.StUllnVal.UlLowVal );	// Y axis
	RamRead32A( StMeasFunc_MFB_LLiAbsInteg2 + 4,	&StMeasValueB.StUllnVal.UlHigVal );

	SetSinWavGenInt();													// Sine wave stop

	SetTransDataAdr( SinWave_OutAddr, (UINT_32)0x00000000 );			// Set sine wave input RAM
	RamWrite32A( UsSinAdr, 0x00000000 );

	if( UcDirSel == X_DIR ) {											// X axis
		if( StMeasValueA.UllnValue == 0x00000000 ) {
			UllCalculateVal = (UINT_64)0x000000007FFFFFFF;
		} else {
			UllCalculateVal = ( StMeasValueB.UllnValue * 1000 / StMeasValueA.UllnValue ) * ptr->Hxgain / ptr->XGap;
		}
		if( UllCalculateVal > (UINT_64)0x000000007FFFFFFF )	UllCalculateVal = (UINT_64)0x000000007FFFFFFF;
		StAdjPar.StLopGan.UlLxgVal = (UINT_32)UllCalculateVal;
		RamWrite32A( HallFilterCoeffX_hxgain0 , StAdjPar.StLopGan.UlLxgVal );

		if( UllCalculateVal > ptr->XJudgeHigh ) {
			UlReturnState = EXE_LXADJ;
		} else if( UllCalculateVal < ptr->XJudgeLow ) {
			UlReturnState = EXE_LXADJ;
		} else {
			UlReturnState = EXE_END;
		}

	} else if( UcDirSel == Y_DIR ) {									// Y axis
		if( StMeasValueA.UllnValue == 0x00000000 ) {
			UllCalculateVal = (UINT_64)0x000000007FFFFFFF;
		} else {
			UllCalculateVal = ( StMeasValueB.UllnValue * 1000 / StMeasValueA.UllnValue ) * ptr->Hygain / ptr->YGap;
		}
		if( UllCalculateVal > (UINT_64)0x000000007FFFFFFF )	UllCalculateVal = (UINT_64)0x000000007FFFFFFF;
		StAdjPar.StLopGan.UlLygVal = (UINT_32)UllCalculateVal;
		RamWrite32A( HallFilterCoeffY_hygain0 , StAdjPar.StLopGan.UlLygVal );

		if( UllCalculateVal > ptr->YJudgeHigh ) {
			UlReturnState = EXE_LYADJ;
		} else if( UllCalculateVal < ptr->YJudgeLow ) {
			UlReturnState = EXE_LYADJ;
		} else {
			UlReturnState = EXE_END;
		}
	}
#if ( USE_AF == 1 )
	else {															// Z axis
		if( StMeasValueA.UllnValue == 0x00000000 ) {
			UllCalculateVal = (UINT_64)0x000000007FFFFFFF;
		} else {
			UllCalculateVal = ( StMeasValueB.UllnValue * 1000 / StMeasValueA.UllnValue ) * ptr->Hzgain / ptr->ZGap;
		}
		CAM_ERR(CAM_OIS, "	StMeasValueB.UllnValue %08xh %08xh \n", (UINT_32)(StMeasValueB.UllnValue >> 32), (UINT_32)(StMeasValueB.UllnValue) );
		CAM_ERR(CAM_OIS, "	StMeasValueA.UllnValue %08xh %08xh \n", (UINT_32)(StMeasValueA.UllnValue >> 32), (UINT_32)(StMeasValueA.UllnValue) );
		CAM_ERR(CAM_OIS, "	Hzgain %08xh \n", (UINT_32)(ptr->Hzgain) );
		CAM_ERR(CAM_OIS, "	ZGap %08xh \n", (UINT_32)(ptr->ZGap) );
		CAM_ERR(CAM_OIS, "	UllCalculateVal %08xh %08xh \n", (UINT_32)(UllCalculateVal >> 32), (UINT_32)(UllCalculateVal) );

		if( UllCalculateVal > (UINT_64)0x000000007FFFFFFF )	UllCalculateVal = (UINT_64)0x000000007FFFFFFF;
		StAdjPar.StLopGan.UlLzgVal = (UINT_32)UllCalculateVal;
		RamWrite32A( CLAF_Gain_afloop2 , StAdjPar.StLopGan.UlLzgVal );

		if( UllCalculateVal > ptr->ZJudgeHigh ) {
			UlReturnState = EXE_LZADJ;
		} else if( UllCalculateVal < ptr->ZJudgeLow ) {
			UlReturnState = EXE_LZADJ;
		} else {
			UlReturnState = EXE_END;
		}
		RamWrite32A( CLAF_RAMA_AFCNT, UlSwitchBk );						// Resume FST control
	}
#endif	// USE_AF
#endif	// BALL_TYPE
	return( UlReturnState ) ;
}



//********************************************************************************
// Function Name 	: MeasAddressSelection
// Retun Value		: NONE
// Argment Value	: mode, *measadr_a, *measadr_b
// Explanation		: Select address of each gyro axis
//********************************************************************************
void MeasAddressSelection( UINT_8 mode , INT_32 * measadr_a , INT_32 * measadr_b )
{
	if( mode == 0 ){
		*measadr_a		=	GYRO_RAM_GX_ADIDAT ;		// Set Measure RAM Address
		*measadr_b		=	GYRO_RAM_GY_ADIDAT ;		// Set Measure RAM Address
	}else if( mode == 1 ){
		*measadr_a		=	GYRO_ZRAM_GZ_ADIDAT ;		// Set Measure RAM Address
		*measadr_b		=	ACCLRAM_Z_AC_ADIDAT ;		// Set Measure RAM Address
	}else{
		*measadr_a		=	ACCLRAM_X_AC_ADIDAT ;			// Set Measure RAM Address
		*measadr_b		=	ACCLRAM_Y_AC_ADIDAT ;			// Set Measure RAM Address
	}
}
	
//********************************************************************************
// Function Name 	: MeasGyAcOffset
// Retun Value		: result status
// Argment Value	: NONE
// Explanation		: Measurement offset of all axis
//********************************************************************************
UINT_32	MeasGyAcOffset(  void  )
{
	UINT_32	UlRsltSts;
	INT_32			SlMeasureParameterA , SlMeasureParameterB ;
	INT_32			SlMeasureParameterNum ;
	UnllnVal		StMeasValueA , StMeasValueB ;
	INT_32			SlMeasureAveValueA[3] , SlMeasureAveValueB[3] ;
	UINT_8			i, j ;
	INT_32			SlMeasureAZ = 0;
	
CAM_ERR(CAM_OIS, "***MeasGyAcOffset\n" );
	MesFil( THROUGH ) ;					// Set Measure Filter

	SlMeasureParameterNum	=	MESOF_NUM ;					// Measurement times
	
	for( i=0 ; i<3 ; i++ )
	{
		MeasAddressSelection( i, &SlMeasureParameterA , &SlMeasureParameterB );
	
		MeasureStart( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB ) ;					// Start measure
		
		MeasureWait() ;					// Wait complete of measurement
	
CAM_ERR(CAM_OIS, "Read Adr = %04x, %04xh \n",StMeasFunc_MFA_LLiIntegral1 + 4 , StMeasFunc_MFA_LLiIntegral1) ;
		RamRead32A( StMeasFunc_MFA_LLiIntegral1 		, &StMeasValueA.StUllnVal.UlLowVal ) ;	// X axis
		RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4		, &StMeasValueA.StUllnVal.UlHigVal ) ;
		RamRead32A( StMeasFunc_MFB_LLiIntegral2 		, &StMeasValueB.StUllnVal.UlLowVal ) ;	// Y axis
		RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4		, &StMeasValueB.StUllnVal.UlHigVal ) ;
	
CAM_ERR(CAM_OIS, "(%d) AOFT = %08x, %08xh \n",i,(unsigned int)StMeasValueA.StUllnVal.UlHigVal,(unsigned int)StMeasValueA.StUllnVal.UlLowVal) ;
CAM_ERR(CAM_OIS, "(%d) BOFT = %08x, %08xh \n",i,(unsigned int)StMeasValueB.StUllnVal.UlHigVal,(unsigned int)StMeasValueB.StUllnVal.UlLowVal) ;
		SlMeasureAveValueA[i] = (INT_32)( (INT_64)StMeasValueA.UllnValue / SlMeasureParameterNum ) ;
		SlMeasureAveValueB[i] = (INT_32)( (INT_64)StMeasValueB.UllnValue / SlMeasureParameterNum ) ;
CAM_ERR(CAM_OIS, "AVEOFT = %08xh \n",(unsigned int)SlMeasureAveValueA[i]) ;
CAM_ERR(CAM_OIS, "AVEOFT = %08xh \n",(unsigned int)SlMeasureAveValueB[i]) ;
	
	}
	
	UlRsltSts = EXE_END ;
	
	// Check accel Z
	if( (SlMeasureAveValueB[1]) >= POSTURETH ){
		SlMeasureAZ = SlMeasureAveValueB[1] - (INT_32)GSENS;
	}else if( (SlMeasureAveValueB[1]) <= -POSTURETH ){
		SlMeasureAZ = SlMeasureAveValueB[1] + (INT_32)GSENS;
	}else{
		UlRsltSts |= EXE_AZADJ ;
	}
CAM_ERR(CAM_OIS, "AZOFF = %08xh \n",(unsigned int)SlMeasureAZ) ;
	
	if( abs(SlMeasureAveValueA[0]) > GYROFFSET_H )					UlRsltSts |= EXE_GXADJ ;
	if( abs(SlMeasureAveValueB[0]) > GYROFFSET_H ) 					UlRsltSts |= EXE_GYADJ ;
	if( abs(SlMeasureAveValueA[1]) > GYROFFSET_H ) 					UlRsltSts |= EXE_GZADJ ;
//	if(    (SlMeasureAveValueB[1]) < POSTURETH )					UlRsltSts |= EXE_AZADJ ;
	if( abs(SlMeasureAveValueA[2]) > ZG_MRGN )						UlRsltSts |= EXE_AXADJ ;
	if( abs(SlMeasureAveValueB[2]) > ZG_MRGN )						UlRsltSts |= EXE_AYADJ ;
	if( abs( SlMeasureAZ) > ZG_MRGN )								UlRsltSts |= EXE_AZADJ ;


	if( UlRsltSts == EXE_END ){
		StAdjPar.StGvcOff.UsGxoVal = SlMeasureAveValueA[0] >> 16 ;
		StAdjPar.StGvcOff.UsGyoVal = SlMeasureAveValueB[0] >> 16 ;
		StAdjPar.StGvcOff.UsGzoVal = SlMeasureAveValueA[1] >> 16 ;

		StAclVal.StAccel.SlOffsetX = SlMeasureAveValueA[2] ;
		StAclVal.StAccel.SlOffsetY = SlMeasureAveValueB[2] ;
		StAclVal.StAccel.SlOffsetZ = SlMeasureAZ ;

		RamWrite32A( GYRO_RAM_GXOFFZ ,		SlMeasureAveValueA[0] ) ;							// X axis Gyro offset
		RamWrite32A( GYRO_RAM_GYOFFZ ,		SlMeasureAveValueB[0] ) ;							// Y axis Gyro offset
		RamWrite32A( GYRO_ZRAM_GZOFFZ ,		SlMeasureAveValueA[1] ) ;							// Z axis Gyro offset
		RamWrite32A( ACCLRAM_X_AC_OFFSET ,	SlMeasureAveValueA[2] ) ;							// X axis Accel offset
		RamWrite32A( ACCLRAM_Y_AC_OFFSET ,	SlMeasureAveValueB[2] ) ;							// Y axis Accel offset
		RamWrite32A( ACCLRAM_Z_AC_OFFSET , 	SlMeasureAZ ) ;										// Z axis Accel offset

		RamWrite32A( GYRO_RAM_GYROX_OFFSET , 0x00000000 ) ;			// X axis Drift Gyro offset
		RamWrite32A( GYRO_RAM_GYROY_OFFSET , 0x00000000 ) ;			// Y axis Drift Gyro offset
		RamWrite32A( GyroRAM_Z_GYRO_OFFSET , 0x00000000 ) ;			// Z axis Drift Gyro offset
		RamWrite32A( GyroFilterDelayX_GXH1Z2 , 0x00000000 ) ;		// X axis H1Z2 Clear
		RamWrite32A( GyroFilterDelayY_GYH1Z2 , 0x00000000 ) ;		// Y axis H1Z2 Clear
		for( i = 2; i < 6; i++ ) {
			j = 4 * i;
			RamWrite32A( AcclFilDly_X + j, 0x00000000 );			// X axis Accl LPF Clear
			RamWrite32A( AcclFilDly_Y + j, 0x00000000 );			// Y axis Accl LPF Clear
			RamWrite32A( AcclFilDly_Z + j, 0x00000000 );			// Z axis Accl LPF Clear
		}
	}
	return( UlRsltSts );
}

 #ifdef	NEUTRAL_CENTER
//********************************************************************************
// Function Name 	: TneHvc
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Tunes the Hall VC offset
// History			: First edition
//********************************************************************************
UINT_8	TneHvc( void )
{
	UINT_8	UcRsltSts;
	INT_32			SlMeasureParameterA , SlMeasureParameterB ;
	INT_32			SlMeasureParameterNum ;
	UnllnVal		StMeasValueA , StMeasValueB ;
	INT_32			SlMeasureAveValueA , SlMeasureAveValueB ;

	RtnCen( BOTH_OFF ) ;		// Both OFF

	WitTim( 500 ) ;

	//平均値測定
	MesFil( NOISE ) ;						// Set Measure Filter

	SlMeasureParameterNum	=	(UINT_32)(100.0 / (1000.0 / (float)FS_FREQ)) ;		// 100ms
	SlMeasureParameterA		=	(UINT_32)SMAHALL_RAM_HXIDAT ;		// Set Measure RAM Address
	SlMeasureParameterB		=	(UINT_32)SMAHALL_RAM_HYIDAT ;		// Set Measure RAM Address

	ClrMesFil();							// Clear Delay RAM
	SetWaitTime(50) ;
	MeasureStart( SlMeasureParameterNum, SlMeasureParameterA, SlMeasureParameterB ) ;					// Start measure

	MeasureWait() ;							// Wait complete of measurement

	RamRead32A( StMeasFunc_MFA_LLiIntegral1 	, &StMeasValueA.StUllnVal.UlLowVal ) ;	// X axis
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4 , &StMeasValueA.StUllnVal.UlHigVal ) ;
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 	, &StMeasValueB.StUllnVal.UlLowVal ) ;	// Y axis
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4	, &StMeasValueB.StUllnVal.UlHigVal ) ;

	SlMeasureAveValueA = (INT_32)( (INT_64)StMeasValueA.UllnValue / SlMeasureParameterNum ) ;
	SlMeasureAveValueB = (INT_32)( (INT_64)StMeasValueB.UllnValue / SlMeasureParameterNum ) ;

	StAdjPar.StHalAdj.UsHlxCna = ( UINT_16 )(( SlMeasureAveValueA >> 16 ) & 0x0000FFFF );				//Measure Result Store
	StAdjPar.StHalAdj.UsHlxCen = StAdjPar.StHalAdj.UsHlxCna;											//Measure Result Store

	StAdjPar.StHalAdj.UsHlyCna = ( UINT_16 )(( SlMeasureAveValueB >> 16 ) & 0x0000FFFF );				//Measure Result Store
	StAdjPar.StHalAdj.UsHlyCen = StAdjPar.StHalAdj.UsHlyCna;											//Measure Result Store

	// hall Z
	SlMeasureParameterA		=	(UINT_32)SMAHALL_RAM_HYIDAT ;		// Set Measure RAM Address
	SlMeasureParameterB		=	(UINT_32)SMAHALL_RAM_HZIDAT ;		// Set Measure RAM Address

	ClrMesFil();							// Clear Delay RAM
	SetWaitTime(50) ;
	MeasureStart( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB ) ;					// Start measure

	MeasureWait() ;							// Wait complete of measurement

	RamRead32A( StMeasFunc_MFB_LLiIntegral2 	, &StMeasValueB.StUllnVal.UlLowVal ) ;	// Z axis
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4	, &StMeasValueB.StUllnVal.UlHigVal ) ;

	SlMeasureAveValueB = (INT_32)( (INT_64)StMeasValueB.UllnValue / SlMeasureParameterNum ) ;

	StAdjPar.StHalAdj.UsHlzCna = ( UINT_16 )(( SlMeasureAveValueB >> 16 ) & 0x0000FFFF );				//Measure Result Store
	StAdjPar.StHalAdj.UsHlzCen = StAdjPar.StHalAdj.UsHlzCna;											//Measure Result Store

	UcRsltSts = EXE_END ;				// Clear Status

	return( UcRsltSts );
}


 #endif	//NEUTRAL_CENTER

 #ifdef	NEUTRAL_CENTER_FINE
//********************************************************************************
// Function Name 	: TneFin
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Tunes the Hall VC offset current optimize
// History			: First edition
//********************************************************************************
void	TneFin( void )
{
	UINT_32	UlReadVal ;
	UINT_16	UsAdxOff, UsAdyOff ;
	INT_32			SlMeasureParameterNum ;
	INT_32			SlMeasureAveValueA , SlMeasureAveValueB ;
	UnllnVal		StMeasValueA , StMeasValueB ;
	UINT_32	UlMinimumValueA, UlMinimumValueB ;
	UINT_16	UsAdxMin, UsAdyMin ;
	UINT_8	UcFin ;

	// Get natural center offset
	RamRead32A( SMAHALL_RAM_HXOFF,  &UlReadVal ) ;
	UsAdxOff = UsAdxMin = (UINT_16)( UlReadVal >> 16 ) ;

	RamRead32A( SMAHALL_RAM_HYOFF,  &UlReadVal ) ;
	UsAdyOff = UsAdyMin = (UINT_16)( UlReadVal >> 16 ) ;
CAM_ERR(CAM_OIS, "*****************************************************\n" );
CAM_ERR(CAM_OIS, "TneFin: Before Adx=%04X, Ady=%04X\n", UsAdxOff, UsAdyOff );

	// Servo ON
	RtnCen( BOTH_ON ) ;
	WitTim( TNE ) ;

	MesFil( THROUGH ) ;					// Filter setting for measurement

	SlMeasureParameterNum = 2000 ;

	MeasureStart( SlMeasureParameterNum , SMAHALL_RAM_HALL_X_OUT , SMAHALL_RAM_HALL_Y_OUT ) ;					// Start measure

	MeasureWait() ;						// Wait complete of measurement

//	RamRead32A( StMeasFunc_MFA_SiMax1 , ( UINT_32 * )&SlMeasureMaxValueA ) ;		// Max value
//	RamRead32A( StMeasFunc_MFA_SiMin1 , ( UINT_32 * )&SlMeasureMinValueA ) ;		// Min value
//	RamRead32A( StMeasFunc_MFA_UiAmp1 , ( UINT_32 * )&SlMeasureAmpValueA ) ;		// Amp value
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 	, &StMeasValueA.StUllnVal.UlLowVal ) ;	// X axis
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4 , &StMeasValueA.StUllnVal.UlHigVal ) ;
	SlMeasureAveValueA = (INT_32)((( (INT_64)StMeasValueA.UllnValue * 100 ) / SlMeasureParameterNum ) / 100 ) ;

//	RamRead32A( StMeasFunc_MFB_SiMax2 , ( UINT_32 * )&SlMeasureMaxValueB ) ;	// Max value
//	RamRead32A( StMeasFunc_MFB_SiMin2 , ( UINT_32 * )&SlMeasureMinValueB ) ;	// Min value
//	RamRead32A( StMeasFunc_MFB_UiAmp2 , ( UINT_32 * )&SlMeasureAmpValueB ) ;		// Amp value
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 	, &StMeasValueB.StUllnVal.UlLowVal ) ;	// Y axis
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4	, &StMeasValueB.StUllnVal.UlHigVal ) ;
	SlMeasureAveValueB = (INT_32)((( (INT_64)StMeasValueB.UllnValue * 100 ) / SlMeasureParameterNum ) / 100 ) ;


//CAM_ERR(CAM_OIS, "TneFin: MaxA=%08X\n", SlMeasureMaxValueA );
//CAM_ERR(CAM_OIS, "        MinA=%08X\n", SlMeasureMinValueA );
//CAM_ERR(CAM_OIS, "        AmpA=%08X\n", SlMeasureAmpValueA );
CAM_ERR(CAM_OIS, "TneFin: AveA=%08X\n", SlMeasureAveValueA );

//CAM_ERR(CAM_OIS, "        MaxB=%08X\n", SlMeasureMaxValueB );
//CAM_ERR(CAM_OIS, "        MinB=%08X\n", SlMeasureMinValueB );
//CAM_ERR(CAM_OIS, "        AmpB=%08X\n", SlMeasureAmpValueB );
CAM_ERR(CAM_OIS, "        AveB=%08X\n", SlMeasureAveValueB );

	UlMinimumValueA = abs(SlMeasureAveValueA) ;
	UlMinimumValueB = abs(SlMeasureAveValueB) ;
	UcFin = 0x11 ;

	while( UcFin ) {
		if( UcFin & 0x01 ) {
			if( UlMinimumValueA >= abs(SlMeasureAveValueA) ) {
				UlMinimumValueA = abs(SlMeasureAveValueA) ;
				UsAdxMin = UsAdxOff ;
				// 収束を早めるために、出力値に比例させる
				if( SlMeasureAveValueA > 0 )
					UsAdxOff = (INT_16)UsAdxOff + (SlMeasureAveValueA >> 17) + 1 ;
				else
					UsAdxOff = (INT_16)UsAdxOff + (SlMeasureAveValueA >> 17) - 1 ;

				RamWrite32A( SMAHALL_RAM_HXOFF,  (UINT_32)((UsAdxOff << 16 ) & 0xFFFF0000 )) ;
			} else {
CAM_ERR(CAM_OIS, "X fine\n");
				UcFin &= 0xFE ;		// clear exec flag X
			}
		}

		if( UcFin & 0x10 ) {
			if( UlMinimumValueB >= abs(SlMeasureAveValueB) ) {
				UlMinimumValueB = abs(SlMeasureAveValueB) ;
				UsAdyMin = UsAdyOff ;
				// 収束を早めるために、出力値に比例させる
				if( SlMeasureAveValueB > 0 )
					UsAdyOff = (INT_16)UsAdyOff + (SlMeasureAveValueB >> 17) + 1 ;
				else
					UsAdyOff = (INT_16)UsAdyOff + (SlMeasureAveValueB >> 17) - 1 ;

				RamWrite32A( SMAHALL_RAM_HYOFF,  (UINT_32)((UsAdyOff << 16 ) & 0xFFFF0000 )) ;
			} else {
CAM_ERR(CAM_OIS, "Y fine\n");
				UcFin &= 0xEF ;		// clear exec flag Y
			}
		}

		MeasureStart( SlMeasureParameterNum , SMAHALL_RAM_HALL_X_OUT , SMAHALL_RAM_HALL_Y_OUT ) ;					// Start measure
		MeasureWait() ;						// Wait complete of measurement

//		RamRead32A( StMeasFunc_MFA_SiMax1 , ( UINT_32 * )&SlMeasureMaxValueA ) ;		// Max value
//		RamRead32A( StMeasFunc_MFA_SiMin1 , ( UINT_32 * )&SlMeasureMinValueA ) ;		// Min value
//		RamRead32A( StMeasFunc_MFA_UiAmp1 , ( UINT_32 * )&SlMeasureAmpValueA ) ;		// Amp value
		RamRead32A( StMeasFunc_MFA_LLiIntegral1 	, &StMeasValueA.StUllnVal.UlLowVal ) ;	// X axis
		RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4 , &StMeasValueA.StUllnVal.UlHigVal ) ;
		SlMeasureAveValueA = (INT_32)((( (INT_64)StMeasValueA.UllnValue * 100 ) / SlMeasureParameterNum ) / 100 ) ;

//		RamRead32A( StMeasFunc_MFB_SiMax2 , ( UINT_32 * )&SlMeasureMaxValueB ) ;	// Max value
//		RamRead32A( StMeasFunc_MFB_SiMin2 , ( UINT_32 * )&SlMeasureMinValueB ) ;	// Min value
//		RamRead32A( StMeasFunc_MFB_UiAmp2 , ( UINT_32 * )&SlMeasureAmpValueB ) ;		// Amp value
		RamRead32A( StMeasFunc_MFB_LLiIntegral2 	, &StMeasValueB.StUllnVal.UlLowVal ) ;	// Y axis
		RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4	, &StMeasValueB.StUllnVal.UlHigVal ) ;
		SlMeasureAveValueB = (INT_32)((( (INT_64)StMeasValueB.UllnValue * 100 ) / SlMeasureParameterNum ) / 100 ) ;

CAM_ERR(CAM_OIS, "-->Adx %04X, Ady %04X\n", UsAdxOff, UsAdyOff );

//CAM_ERR(CAM_OIS, "TneFin: MaxA=%08X\n", SlMeasureMaxValueA );
//CAM_ERR(CAM_OIS, "        MinA=%08X\n", SlMeasureMinValueA );
//CAM_ERR(CAM_OIS, "        AmpA=%08X\n", SlMeasureAmpValueA );
CAM_ERR(CAM_OIS, "TneFin: AveA=%08X\n", SlMeasureAveValueA );

//CAM_ERR(CAM_OIS, "        MaxB=%08X\n", SlMeasureMaxValueB );
//CAM_ERR(CAM_OIS, "        MinB=%08X\n", SlMeasureMinValueB );
//CAM_ERR(CAM_OIS, "        AmpB=%08X\n", SlMeasureAmpValueB );
CAM_ERR(CAM_OIS, "        AveB=%08X\n", SlMeasureAveValueB );
	}	// while


CAM_ERR(CAM_OIS, "TneFin: After Adx=%04X, Ady=%04X\n", UsAdxMin, UsAdyMin );


	StAdjPar.StHalAdj.UsHlxCna = UsAdxMin;								//Measure Result Store
	StAdjPar.StHalAdj.UsHlxCen = StAdjPar.StHalAdj.UsHlxCna;		//Measure Result Store

	StAdjPar.StHalAdj.UsHlyCna = UsAdyMin;								//Measure Result Store
	StAdjPar.StHalAdj.UsHlyCen = StAdjPar.StHalAdj.UsHlyCna;		//Measure Result Store

	StAdjPar.StHalAdj.UsAdxOff = StAdjPar.StHalAdj.UsHlxCna  ;
	StAdjPar.StHalAdj.UsAdyOff = StAdjPar.StHalAdj.UsHlyCna  ;

	// Servo OFF
	RtnCen( BOTH_OFF ) ;		// Both OFF

}
 #endif	//NEUTRAL_CENTER_FINE


//********************************************************************************
// Function Name 	: TneSltPos
// Retun Value		: SUCCESS / FAILURE
// Argment Value	: Position number(1, 2, 3, 4, 5, 6, 7, 0:reset)
//					:          reverse(9, 10, 11, 12, 13, 14, 15)
// Explanation		: Move measurement position function
//********************************************************************************
UINT_8	TneSltPos( UINT_8 UcPos )
{
	INT_16 SsOffX = 0x0000 ;
	INT_16 SsOffY = 0x0000 ;
	INT_32	SlX, SlY;
	INT_16 SsSign = (UcPos > 8) ? -1 : 1;
	ADJ_LINEARITY_MIXING * LinMixPtr;

	LinMixPtr = (ADJ_LINEARITY_MIXING*)&SMA_LinMixParameter;

	UcPos &= 0x07 ;
/*
	if ( UcPos ) {
		SsOffX = (INT_16)((float)(LinMixPtr->SltOffsetX * (UcPos - 4)) / sqrt(2.0));
		SsOffY = (INT_16)((float)(LinMixPtr->SltOffsetY * (UcPos - 4)) / sqrt(2.0));
	}
*/
	if ( UcPos ) {
		SsOffX = (INT_16)((float)(LinMixPtr->SltOffsetX * (UcPos - 4)) / 1.414);
		SsOffY = (INT_16)((float)(LinMixPtr->SltOffsetY * (UcPos - 4)) / 1.414);
	}

	SlX = (INT_32)((SsOffX * LinMixPtr->SltDirX * SsSign) << 16);
	SlY = (INT_32)((SsOffY * LinMixPtr->SltDirY) << 16);

	RamWrite32A( SMAHALL_RAM_GYROX_OUT, SlX ) ;
	RamWrite32A( SMAHALL_RAM_GYROY_OUT, SlY ) ;

	return( SUCCESS );
}

//********************************************************************************
// Function Name 	: TneHrzPos
// Retun Value		: SUCCESS / FAILURE
// Argment Value	: Position number(1, 2, 3, 4, 5, 6, 7, 0:reset)
// Explanation		: Move measurement position function
//********************************************************************************
UINT_8	TneHrzPos( UINT_8 UcPos )
{
	INT_16 SsOff = 0x0000 ;
	INT_32	SlX, SlY;
	ADJ_LINEARITY_MIXING * LinMixPtr;

	LinMixPtr = (ADJ_LINEARITY_MIXING*)&SMA_LinMixParameter;

	UcPos &= 0x07 ;	// max 7 point

	if ( UcPos ) {
		SsOff = LinMixPtr->SltOffsetY * (UcPos - 4);
	}

	SlX = 0x00000000;
	SlY = (INT_32)((SsOff * LinMixPtr->SltDirY) << 16);

	RamWrite32A( SMAHALL_RAM_GYROX_OUT, SlX ) ;
	RamWrite32A( SMAHALL_RAM_GYROY_OUT, SlY ) ;

	return( SUCCESS );
}

//********************************************************************************
// Function Name 	: TneVrtPos
// Retun Value		: SUCCESS / FAILURE
// Argment Value	: Position number(1, 2, 3, 4, 5, 6, 7, 0:reset)
// Explanation		: Move measurement position function
//********************************************************************************
UINT_8	TneVrtPos( UINT_8 UcPos )
{
	INT_16 SsOff = 0x0000 ;
	INT_32	SlX, SlY;
	ADJ_LINEARITY_MIXING * LinMixPtr;

	LinMixPtr = (ADJ_LINEARITY_MIXING*)&SMA_LinMixParameter;

	UcPos &= 0x07 ;	// max 7 point
		
	if ( UcPos ) {
		SsOff = LinMixPtr->SltOffsetX * (UcPos - 4);
	}

	SlX = (INT_32)((SsOff * LinMixPtr->SltDirX) << 16);
	SlY = 0x00000000;

	RamWrite32A( SMAHALL_RAM_GYROX_OUT, SlX ) ;
	RamWrite32A( SMAHALL_RAM_GYROY_OUT, SlY ) ;

	return( SUCCESS );
}


//********************************************************************************
// Function Name 	: TneHrzPos_LS
// Retun Value		: SUCCESS / FAILURE
// Argment Value	: Position number(1, 2, 3, 4, 5, 6, 7, 0:reset)
// Explanation		: Move measurement position function
//********************************************************************************
UINT_8	TneHrzPos_LS( UINT_8 UcPos )
{
	INT_16 SsOff = 0x0000 ;
	INT_32	SlX, SlY;
	ADJ_LINEARITY_MIXING * LinMixPtr;

	LinMixPtr = (ADJ_LINEARITY_MIXING*)&LS_LinMixParameter;

	UcPos &= 0x07 ;	// max 7 point
		
	if ( UcPos ) {
		SsOff = LinMixPtr->SltOffsetX * (UcPos - 4);
	}

	SlX = (INT_32)((SsOff * LinMixPtr->SltDirX) << 16);
	SlY = 0x00000000;

	RamWrite32A( HALL_RAM_GYROX_OUT, SlX ) ;
	RamWrite32A( HALL_RAM_GYROY_OUT, SlY ) ;

	return( SUCCESS );
}

//********************************************************************************
// Function Name 	: TneVrtPos_LS
// Retun Value		: SUCCESS / FAILURE
// Argment Value	: Position number(1, 2, 3, 4, 5, 6, 7, 0:reset)
// Explanation		: Move measurement position function
//********************************************************************************
UINT_8	TneVrtPos_LS( UINT_8 UcPos )
{
	INT_16 SsOff = 0x0000 ;
	INT_32	SlX, SlY;
	ADJ_LINEARITY_MIXING * LinMixPtr;

	LinMixPtr = (ADJ_LINEARITY_MIXING*)&LS_LinMixParameter;

	UcPos &= 0x07 ;	// max 7 point

	if ( UcPos ) {
		SsOff = LinMixPtr->SltOffsetY * (UcPos - 4);
	}

	SlX = 0x00000000;
	SlY = (INT_32)((SsOff * LinMixPtr->SltDirY) << 16);

	RamWrite32A( HALL_RAM_GYROX_OUT, SlX ) ;
	RamWrite32A( HALL_RAM_GYROY_OUT, SlY ) ;

	return( SUCCESS );
}


//********************************************************************************
// Function Name 	: getTneHrzPos
// Retun Value		: SUCCESS / FAILURE
// Argment Value	: Position table pointer
// Explanation		: Get positions of TneHrzPos function
//********************************************************************************
UINT_8 getTneHrzPos( INT_32 *pPos )
{
	int i;
	ADJ_LINEARITY_MIXING * LinMixPtr;

	LinMixPtr = (ADJ_LINEARITY_MIXING*)&SMA_LinMixParameter;

	for( i = 1; i <= 7; i++ ) {
		pPos[ i - 1 ] = (INT_32)LinMixPtr->SltOffsetY * LinMixPtr->SltDirY * ( i - 4 ) << 16;
	}
	return( SUCCESS );
}

//********************************************************************************
// Function Name 	: getTneVrtPos
// Retun Value		: SUCCESS / FAILURE
// Argment Value	: Position table pointer
// Explanation		: Get positions of TneVrtPos function
//********************************************************************************
UINT_8 getTneVrtPos( INT_32 *pPos )
{
	int i;
	ADJ_LINEARITY_MIXING * LinMixPtr;

	LinMixPtr = (ADJ_LINEARITY_MIXING*)&SMA_LinMixParameter;

	for( i = 1; i <= 7; i++ ) {
		pPos[ i - 1 ] = (INT_32)LinMixPtr->SltOffsetX * LinMixPtr->SltDirX * ( i - 4 ) << 16;
	}
	return( SUCCESS );
}

//********************************************************************************
// Function Name 	: getTneHrzPos_LS
// Retun Value		: SUCCESS / FAILURE
// Argment Value	: Position table pointer
// Explanation		: Get positions of TneHrzPos function
//********************************************************************************
UINT_8 getTneHrzPos_LS( INT_32 *pPos )
{
	int i;
	ADJ_LINEARITY_MIXING * LinMixPtr;

	LinMixPtr = (ADJ_LINEARITY_MIXING*)&LS_LinMixParameter;

	for( i = 1; i <= 7; i++ ) {
		pPos[ i - 1 ] = (INT_32)LinMixPtr->SltOffsetY * LinMixPtr->SltDirY * ( i - 4 ) << 16;
	}
	return( SUCCESS );
}

//********************************************************************************
// Function Name 	: getTneVrtPos_LS
// Retun Value		: SUCCESS / FAILURE
// Argment Value	: Position table pointer
// Explanation		: Get positions of TneVrtPos function
//********************************************************************************
UINT_8 getTneVrtPos_LS( INT_32 *pPos )
{
	int i;
	ADJ_LINEARITY_MIXING * LinMixPtr;

	LinMixPtr = (ADJ_LINEARITY_MIXING*)&LS_LinMixParameter;

	for( i = 1; i <= 7; i++ ) {
		pPos[ i - 1 ] = (INT_32)LinMixPtr->SltOffsetX * LinMixPtr->SltDirX * ( i - 4 ) << 16;
	}
	return( SUCCESS );
}

//********************************************************************************
// Function Name 	: MovCirPos12
// Retun Value		: SUCCESS / FAILURE
// Argment Value	: Position number(1 - 12, 0:reset)
// Explanation		: Move measurement position function
//********************************************************************************
UINT_8	MovCirPos12( UINT_8 UcPos )
{
	INT_32	SlX, SlY;

	switch(UcPos) {
		case 0:		SlX = 0x00000000;	SlY = 0x00000000;	break;
		case 1:		SlX = 0x39000000;	SlY = 0x00000000;	break;
		case 2:		SlX = 0x315D0000;	SlY = 0x1C7F0000;	break;
		case 3:		SlX = 0x1C800000;	SlY = 0x315D0000;	break;
		case 4:		SlX = 0x00000000;	SlY = 0x39000000;	break;
		case 5:		SlX = 0xE3810000;	SlY = 0x315D0000;	break;
		case 6:		SlX = 0xCEA30000;	SlY = 0x1C7F0000;	break;
		case 7:		SlX = 0xC7000000;	SlY = 0x00000000;	break;
		case 8:		SlX = 0xCEA30000;	SlY = 0xE3800000;	break;
		case 9:		SlX = 0xE3800000;	SlY = 0xCEA30000;	break;
		case 10:	SlX = 0x00000000;	SlY = 0xC7000000;	break;
		case 11:	SlX = 0x1C800000;	SlY = 0xCEA30000;	break;
		case 12:	SlX = 0x315D0000;	SlY = 0xE3800000;	break;
		default:	SlX = 0x00000000;	SlY = 0x00000000;	break;
	}
	
	RamWrite32A( SMAHALL_RAM_GYROX_OUT, SlX ) ;
	RamWrite32A( SMAHALL_RAM_GYROY_OUT, SlY ) ;
	return( SUCCESS );
}

//********************************************************************************
// Function Name 	: MemoryClear
// Retun Value		: NON
// Argment Value	: Top poINT_32er , Size
// Explanation		: Memory Clear Function
// History			: First edition
//********************************************************************************
void	MemoryClear( UINT_16 UsSourceAddress, UINT_16 UsClearSize )
{
	UINT_16	UsLoopIndex ;

	for ( UsLoopIndex = 0 ; UsLoopIndex < UsClearSize ;  ) {
		RamWrite32A( UsSourceAddress	, 	0x00000000 ) ;				// 4Byte
		UsSourceAddress += 4;
		UsLoopIndex += 4 ;
	}
}


//********************************************************************************
// Function Name 	: MesFil
// Retun Value		: NON
// Argment Value	: Measure Filter Mode
// Explanation		: Measure Filter Setting Function
// History			: First edition
//********************************************************************************
void	MesFil( eFilterMode	eMesMod )		// 18.014kHz
{
	UINT_32	UlMeasFilaA , UlMeasFilaB , UlMeasFilaC ;
	UINT_32	UlMeasFilbA , UlMeasFilbB , UlMeasFilbC ;

	switch( eMesMod ) {
	case HALL_ADJ:			// Hall Bias&Offset Adjust
		UlMeasFilaA	=	0x03E45273 ;	// LPF150Hz @ FS15kHz
		UlMeasFilaB	=	0x03E45273 ;
		UlMeasFilaC	=	0x78375B1B ;
		UlMeasFilbA	=	0x7FFFFFFF ;	// Through
		UlMeasFilbB	=	0x00000000 ;
		UlMeasFilbC	=	0x00000000 ;
		break;

	case LOOPGAIN:			// Loop Gain Adjust
		UlMeasFilaA	=	0x0C1D10E5 ;	// LPF500Hz @ FS15kHz
		UlMeasFilaB	=	0x0C1D10E5 ;
		UlMeasFilaC	=	0x67C5DE37 ;
		UlMeasFilbA	=	0x7F33C48D ;	// HPF30Hz @ FS15kHz
		UlMeasFilbB	=	0x80CC3B73 ;
		UlMeasFilbC	=	0x7E67891B ;
		break;

	case THROUGH:			// for Through
		UlMeasFilaA	=	0x7FFFFFFF ;	// Through
		UlMeasFilaB	=	0x00000000 ;
		UlMeasFilaC	=	0x00000000 ;
		UlMeasFilbA	=	0x7FFFFFFF ;	// Through
		UlMeasFilbB	=	0x00000000 ;
		UlMeasFilbC	=	0x00000000 ;
		break;

	case NOISE:				// SINE WAVE TEST for NOISE
		UlMeasFilaA	=	0x03E45273 ;	// LPF150Hz @ FS15kHz
		UlMeasFilaB	=	0x03E45273 ;
		UlMeasFilaC	=	0x78375B1B ;
		UlMeasFilbA	=	0x03E45273 ;	// LPF150Hz @ FS15kHz
		UlMeasFilbB	=	0x03E45273 ;
		UlMeasFilbC	=	0x78375B1B ;
		break;

	case OSCCHK:			// OSC check
		UlMeasFilaA	=	0x078DD84D ;	// LPF300Hz @ FS15kHz
		UlMeasFilaB	=	0x078DD84D ;
		UlMeasFilaC	=	0x70E44F65 ;
		UlMeasFilbA	=	0x078DD84D ;	// LPF300Hz @ FS15kHz
		UlMeasFilbB	=	0x078DD84D ;
		UlMeasFilbC	=	0x70E44F65 ;
		break;

	case SELFTEST:			// GYRO SELF TEST
		UlMeasFilaA	=	0x105723F1 ;	// LPF700Hz @ FS15kHz
		UlMeasFilaB	=	0x105723F1 ;
		UlMeasFilaC	=	0x5F51B81D ;
		UlMeasFilbA	=	0x7FFFFFFF ;	// Through
		UlMeasFilbB	=	0x00000000 ;
		UlMeasFilbC	=	0x00000000 ;
		break;

	default:
		UlMeasFilaA	=	0x7FFFFFFF ;	// Through
		UlMeasFilaB	=	0x00000000 ;
		UlMeasFilaC	=	0x00000000 ;
		UlMeasFilbA	=	0x7FFFFFFF ;	// Through
		UlMeasFilbB	=	0x00000000 ;
		UlMeasFilbC	=	0x00000000 ;
	}

	RamWrite32A ( MeasureFilterA_Coeff_a1	, UlMeasFilaA ) ;
	RamWrite32A ( MeasureFilterA_Coeff_b1	, UlMeasFilaB ) ;
	RamWrite32A ( MeasureFilterA_Coeff_c1	, UlMeasFilaC ) ;

	RamWrite32A ( MeasureFilterA_Coeff_a2	, UlMeasFilbA ) ;
	RamWrite32A ( MeasureFilterA_Coeff_b2	, UlMeasFilbB ) ;
	RamWrite32A ( MeasureFilterA_Coeff_c2	, UlMeasFilbC ) ;

	RamWrite32A ( MeasureFilterB_Coeff_a1	, UlMeasFilaA ) ;
	RamWrite32A ( MeasureFilterB_Coeff_b1	, UlMeasFilaB ) ;
	RamWrite32A ( MeasureFilterB_Coeff_c1	, UlMeasFilaC ) ;

	RamWrite32A ( MeasureFilterB_Coeff_a2	, UlMeasFilbA ) ;
	RamWrite32A ( MeasureFilterB_Coeff_b2	, UlMeasFilbB ) ;
	RamWrite32A ( MeasureFilterB_Coeff_c2	, UlMeasFilbC ) ;
}

//********************************************************************************
// Function Name 	: ClrMesFil
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Clear Measure Filter Function
// History			: First edition
//********************************************************************************
void	ClrMesFil( void )
{
	RamWrite32A ( MeasureFilterA_Delay_z11	, 0 ) ;
	RamWrite32A ( MeasureFilterA_Delay_z12	, 0 ) ;

	RamWrite32A ( MeasureFilterA_Delay_z21	, 0 ) ;
	RamWrite32A ( MeasureFilterA_Delay_z22	, 0 ) ;

	RamWrite32A ( MeasureFilterB_Delay_z11	, 0 ) ;
	RamWrite32A ( MeasureFilterB_Delay_z12	, 0 ) ;

	RamWrite32A ( MeasureFilterB_Delay_z21	, 0 ) ;
	RamWrite32A ( MeasureFilterB_Delay_z22	, 0 ) ;
}

//********************************************************************************
// Function Name 	: MeasureStart
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Measure start setting Function
// History			: First edition
//********************************************************************************
void	MeasureStart( INT_32 SlMeasureParameterNum , INT_32 SlMeasureParameterA , INT_32 SlMeasureParameterB )
{
	MemoryClear( StMeasFunc_SiSampleNum , sizeof( MeasureFunction_Type ) ) ;
	RamWrite32A( StMeasFunc_MFA_SiMax1	 , 0x80000000 ) ;					// Set Min
	RamWrite32A( StMeasFunc_MFB_SiMax2	 , 0x80000000 ) ;					// Set Min
	RamWrite32A( StMeasFunc_MFA_SiMin1	 , 0x7FFFFFFF ) ;					// Set Max
	RamWrite32A( StMeasFunc_MFB_SiMin2	 , 0x7FFFFFFF ) ;					// Set Max

	RamWrite32A( StMeasFunc_MFA_PiMeasureRam1	, ( UINT_32 )SlMeasureParameterA ) ;		// Set Measure Filter A Ram Address
	RamWrite32A( StMeasFunc_MFB_PiMeasureRam2	, ( UINT_32 )SlMeasureParameterB ) ;		// Set Measure Filter B Ram Address

	RamWrite32A( StMeasFunc_SiSampleNum				, 0 ) ;									// Clear Measure Counter
	ClrMesFil() ;																			// Clear Delay Ram
	SetWaitTime(50) ;
	RamWrite32A( StMeasFunc_SiSampleMax				, SlMeasureParameterNum ) ;				// Set Measure Max Number
}


//********************************************************************************
// Function Name 	: MeasureStart2
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Measure start setting Function
// History			: First edition 						
//********************************************************************************
void	MeasureStart2( INT_32 SlMeasureParameterNum , INT_32 SlMeasureParameterA , INT_32 SlMeasureParameterB , UINT_16 UsTime )
{
	MemoryClear( StMeasFunc_SiSampleNum , sizeof( MeasureFunction_Type ) ) ;
	RamWrite32A( StMeasFunc_MFA_SiMax1	 , 0x80000001 ) ;					// Set Min 
	RamWrite32A( StMeasFunc_MFB_SiMax2	 , 0x80000001 ) ;					// Set Min 
	RamWrite32A( StMeasFunc_MFA_SiMin1	 , 0x7FFFFFFF ) ;					// Set Max 
	RamWrite32A( StMeasFunc_MFB_SiMin2	 , 0x7FFFFFFF ) ;					// Set Max 
	
	RamWrite32A( StMeasFunc_MFA_PiMeasureRam1	 , ( UINT_32 )SlMeasureParameterA ) ;		// Set Measure Filter A Ram Address
	RamWrite32A( StMeasFunc_MFB_PiMeasureRam2	 , ( UINT_32 )SlMeasureParameterB ) ;		// Set Measure Filter B Ram Address

	RamWrite32A( StMeasFunc_SiSampleNum				, 0 ) ;									// Clear Measure Counter
	ClrMesFil() ;																			// Clear Delay Ram
	SetWaitTime(UsTime) ;
	RamWrite32A( StMeasFunc_SiSampleMax				, SlMeasureParameterNum ) ;				// Set Measure Max Number
}

//********************************************************************************
// Function Name 	: MeasureWait
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Wait complete of Measure Function
// History			: First edition
//********************************************************************************
void	MeasureWait( void )
{
	UINT_32	SlWaitTimerSt;
	UINT_16	UsTimeOut = 5000;
	do {
		RamRead32A( StMeasFunc_SiSampleMax, &SlWaitTimerSt ) ;
		UsTimeOut--;
	} while ( SlWaitTimerSt && UsTimeOut );
//CAM_ERR(CAM_OIS, "MeasureWait EXIT NumA=%d, NumB=%d, Time=%d\n", SlWaitTimerStA, SlWaitTimerStB, UsTimeOut );
}


//********************************************************************************
// Function Name 	: SetWaitTime
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Set Timer wait Function
// History			: First edition
//********************************************************************************
#define 	ONE_MSEC_COUNT	3			// 2.5kHz * 3 ≒ 1ms
void	SetWaitTime( UINT_16 UsWaitTime )
{
	RamWrite32A( WaitTimerData_UiWaitCounter	, 0 ) ;
	RamWrite32A( WaitTimerData_UiTargetCount	, (UINT_32)(ONE_MSEC_COUNT * UsWaitTime)) ;
}


//********************************************************************************
// Function Name 	: LopGan
// Retun Value		: Execute Result
// Argment Value	: X,Y Direction
// Explanation		: Loop Gain Adjust Function
// History			: First edition
//********************************************************************************
UINT_32	LopGan( UINT_8 UcDirSel, ADJ_LOPGAN* ptr )
{
#if 1
	UINT_32			UlReturnState = EXE_END ;
	
	if( UcDirSel == X_DIR ) {							// X axis
		RamWrite32A( SmaHallFilterCoeffX_hxgain0 , ptr->Hxgain ) ;
		StAdjPar.StLopGan.UlLxgVal = ptr->Hxgain ;
		UlReturnState = EXE_END ;
	} else if( UcDirSel == Y_DIR ){						// Y axis
		RamWrite32A( SmaHallFilterCoeffY_hygain0 , ptr->Hygain ) ;
		StAdjPar.StLopGan.UlLygVal = ptr->Hygain ;
		StAdjPar.StLopGan.UlLzgVal = ptr->Hygain ;
		UlReturnState = EXE_END ;
	}
	
	return( UlReturnState ) ;
#else	
	
	UnllnVal		StMeasValueA , StMeasValueB ;
	INT_32			SlMeasureParameterA , SlMeasureParameterB ;
	UINT_64			UllCalculateVal ;
	UINT_32			UlReturnState ;
	UINT_16			UsSinAdr ;
	UINT_32			UlFreq, UlGain;
	INT_32			SlNum ;

//	UINT_32	UlSwitchBk ;
	RamWrite32A( HallFilterCoeffX_hxgain0 , LopgainPtr->Hxgain ) ;
	RamWrite32A( HallFilterCoeffY_hygain0 , LopgainPtr->Hygain ) ;
		
	if( UcDirSel == X_DIR ) {		// X axis
		SlMeasureParameterA		=	SMAHALL_RAM_HXOUT1 ;		// Set Measure RAM Address
		SlMeasureParameterB		=	SMAHALL_RAM_HXLOP ;		// Set Measure RAM Address
		RamWrite32A( HallFilterCoeffX_hxgain0 , ptr->Hxgain ) ;
		UsSinAdr = SMAHALL_RAM_SINDX0;
		UlFreq = ptr->XNoiseFreq;
		UlGain = ptr->XNoiseGain;
		SlNum  = ptr->XNoiseNum;
//	} else if( UcDirSel == Y_DIR ){						// Y axis
	} else {			
		SlMeasureParameterA		=	SMAHALL_RAM_HYOUT1 ;		// Set Measure RAM Address
		SlMeasureParameterB		=	SMAHALL_RAM_HYLOP ;		// Set Measure RAM Address
		RamWrite32A( HallFilterCoeffY_hygain0 , ptr->Hygain ) ;
		UsSinAdr = SMAHALL_RAM_SINDY0;
		UlFreq = ptr->YNoiseFreq;
		UlGain = ptr->YNoiseGain;
		SlNum  = ptr->YNoiseNum;
#if 0
	} else {						// AF axis
		SlMeasureParameterA		=	CLAF_RAMA_AFLOP2 ;		// Set Measure RAM Address
		SlMeasureParameterB		=	CLAF_DELAY_AFPZ0 ;		// Set Measure RAM Address
		RamWrite32A( CLAF_Gain_afloop2 ,  ptr->Hzgain ) ;
		RamRead32A( CLAF_RAMA_AFCNT , &UlSwitchBk ) ;		// FST control OFF
		RamWrite32A( CLAF_RAMA_AFCNT , UlSwitchBk & 0xffffff0f ) ;
		UsSinAdr = CLAF_RAMA_AFCNTO;
		UlFreq = ptr->ZNoiseFreq;
		UlGain = ptr->ZNoiseGain;
		SlNum  = ptr->ZNoiseNum;
#endif
	}

	SetSinWavGenInt();

	RamWrite32A( SinWave_Offset		,	UlFreq ) ;								// Freq Setting
	RamWrite32A( SinWave_Gain		,	UlGain ) ;								// Set Sine Wave Gain

CAM_ERR(CAM_OIS, "		Loop Gain %d , Freq %08xh, Gain %08xh , Num %08xh \n", UcDirSel , UlFreq , UlGain , SlNum ) ;
	RamWrite32A( SinWaveC_Regsiter	,	0x00000001 ) ;								// Sine Wave Start

	RamWrite32A( SinWave_OutAddr	,	( UINT_32 )UsSinAdr ) ;	// Set Sine Wave Input RAM

	MesFil( LOOPGAIN ) ;					// Filter setting for measurement

	MeasureStart( SlNum , SlMeasureParameterA , SlMeasureParameterB ) ;					// Start measure

	MeasureWait() ;						// Wait complete of measurement

	RamRead32A( StMeasFunc_MFA_LLiAbsInteg1 		, &StMeasValueA.StUllnVal.UlLowVal ) ;	// X axis
	RamRead32A( StMeasFunc_MFA_LLiAbsInteg1 + 4 	, &StMeasValueA.StUllnVal.UlHigVal ) ;
	RamRead32A( StMeasFunc_MFB_LLiAbsInteg2 		, &StMeasValueB.StUllnVal.UlLowVal ) ;	// Y axis
	RamRead32A( StMeasFunc_MFB_LLiAbsInteg2 + 4		, &StMeasValueB.StUllnVal.UlHigVal ) ;

	SetSinWavGenInt();		// Sine wave stop

	RamWrite32A( SinWave_OutAddr	,	(UINT_32)0x00000000 ) ;	// Set Sine Wave Input RAM
	RamWrite32A( UsSinAdr		,	0x00000000 ) ;				// DelayRam Clear

	if( UcDirSel == X_DIR ) {		// X axis
		if( StMeasValueA.UllnValue == 0x00000000 ){
			UllCalculateVal = (UINT_64)0x000000007FFFFFFF ;
		}
		else{
			UllCalculateVal = ( StMeasValueB.UllnValue * 1000 / StMeasValueA.UllnValue ) * ptr->Hxgain / ptr->XGap ;
		}
		if( UllCalculateVal > (UINT_64)0x000000007FFFFFFF )		UllCalculateVal = (UINT_64)0x000000007FFFFFFF ;
		StAdjPar.StLopGan.UlLxgVal = (UINT_32)UllCalculateVal ;
		RamWrite32A( HallFilterCoeffX_hxgain0 , StAdjPar.StLopGan.UlLxgVal ) ;
		if( UllCalculateVal > ptr->XJudgeHigh ){
			UlReturnState = EXE_LXADJ ;
		}else if( UllCalculateVal < ptr->XJudgeLow ){
			UlReturnState = EXE_LXADJ ;
		}else{
			UlReturnState = EXE_END ;
		}

//	}else if( UcDirSel == Y_DIR ){							// Y axis
	}else{	
		if( StMeasValueA.UllnValue == 0x00000000 ){
			UllCalculateVal = (UINT_64)0x000000007FFFFFFF ;
		}
		else{
			UllCalculateVal = ( StMeasValueB.UllnValue * 1000 / StMeasValueA.UllnValue ) * ptr->Hygain / ptr->YGap ;
		}
		if( UllCalculateVal > (UINT_64)0x000000007FFFFFFF )		UllCalculateVal = (UINT_64)0x000000007FFFFFFF ;
		StAdjPar.StLopGan.UlLygVal = (UINT_32)UllCalculateVal ;
		RamWrite32A( HallFilterCoeffY_hygain0 , StAdjPar.StLopGan.UlLygVal ) ;
		if( UllCalculateVal > ptr->YJudgeHigh ){
			UlReturnState = EXE_LYADJ ;
		}else if( UllCalculateVal < ptr->YJudgeLow ){
			UlReturnState = EXE_LYADJ ;
		}else{
			UlReturnState = EXE_END ;
		}
#if 0
	}else{							// Z axis
		if( StMeasValueA.UllnValue == 0x00000000 ){
			UllCalculateVal = (UINT_64)0x000000007FFFFFFF ;
		}
		else{
			UllCalculateVal = ( StMeasValueB.UllnValue * 1000 / StMeasValueA.UllnValue ) * ptr->Hzgain / ptr->ZGap ;
		}
CAM_ERR(CAM_OIS, "	StMeasValueB.UllnValue %08xh %08xh \n", (UINT_32)(StMeasValueB.UllnValue >> 32), (UINT_32)(StMeasValueB.UllnValue) ) ;
CAM_ERR(CAM_OIS, "	StMeasValueA.UllnValue %08xh %08xh \n", (UINT_32)(StMeasValueA.UllnValue >> 32), (UINT_32)(StMeasValueA.UllnValue) ) ;
CAM_ERR(CAM_OIS, "	Hzgain %08xh \n", (UINT_32)(ptr->Hzgain) ) ;
CAM_ERR(CAM_OIS, "	ZGap %08xh \n", (UINT_32)(ptr->ZGap) ) ;
CAM_ERR(CAM_OIS, "	UllCalculateVal %08xh %08xh \n", (UINT_32)(UllCalculateVal >> 32), (UINT_32)(UllCalculateVal) ) ;

		if( UllCalculateVal > (UINT_64)0x000000007FFFFFFF )		UllCalculateVal = (UINT_64)0x000000007FFFFFFF ;
		StAdjPar.StLopGan.UlLzgVal = (UINT_32)UllCalculateVal ;
		RamWrite32A( CLAF_Gain_afloop2 , StAdjPar.StLopGan.UlLzgVal ) ;
		if( UllCalculateVal > ptr->ZJudgeHigh ){
			UlReturnState = EXE_LZADJ ;
		}else if( UllCalculateVal < ptr->ZJudgeLow ){
			UlReturnState = EXE_LZADJ ;
		}else{
			UlReturnState = EXE_END ;
		}
		RamWrite32A( CLAF_RAMA_AFCNT , UlSwitchBk ) ;
#endif
	}

	return( UlReturnState ) ;
#endif
}




//********************************************************************************
// Function Name 	: RtnCen
// Retun Value		: Command Status
// Argment Value	: Command Parameter
// Explanation		: Return to center Command Function
// History			: First edition
//********************************************************************************
UINT_8	RtnCen( UINT_8	UcCmdPar )
{
	UINT_8	UcSndDat = FAILURE;
	UINT_32	UlTimeout = 50000;

	if( !UcCmdPar ){								// X,Y centering
		RamWrite32A( CMD_RETURN_TO_CENTER , BOTH_SRV_ON ) ;

	}else if( UcCmdPar == XONLY_ON ){				// only X centering
		RamWrite32A( CMD_RETURN_TO_CENTER , XAXS_SRV_ON | YAXS_SRV_OFF ) ;

	}else if( UcCmdPar == YONLY_ON ){				// only Y centering
		RamWrite32A( CMD_RETURN_TO_CENTER , YAXS_SRV_ON | XAXS_SRV_OFF ) ;

	}else if( UcCmdPar == ZONLY_OFF ){				// only Z centering off
		RamWrite32A( CMD_RETURN_TO_CENTER , ZAXS_SRV_OFF ) ;

	}else if( UcCmdPar == ZONLY_ON ){				// only Z centering
		RamWrite32A( CMD_RETURN_TO_CENTER , ZAXS_SRV_ON ) ;

	}else{											// Both off
		RamWrite32A( CMD_RETURN_TO_CENTER , BOTH_SRV_OFF ) ;
	}

	do {
		UcSndDat = RdStatus(1);
		UlTimeout--;
	} while( UcSndDat == FAILURE && UlTimeout != 0 );
#if 0
	if( UcCmdPar == ZONLY_OFF ){				// only Z centering off
		RamWrite32A( CLAF_RAMA_AFOUT		,	0x00000000 ) ;				// driver output clear
	}
#endif
CAM_ERR(CAM_OIS, "RtnCen() = %02x\n", UcSndDat ) ;
	return( UcSndDat );
}

//********************************************************************************
// Function Name 	: RtnCen_LS
// Retun Value		: Command Status
// Argment Value	: Command Parameter
// Explanation		: Return to center Command Function
// History			: First edition
//********************************************************************************
UINT_8	RtnCen_LS( UINT_8	UcCmdPar )
{
	UINT_8	UcSndDat = FAILURE;
	UINT_32	UlTimeout = 50000;

	if( !UcCmdPar ){								// X,Y centering
		RamWrite32A( CMD_RETURN_TO_CENTER , CMD_FOR_LS | BOTH_SRV_ON ) ;

	}else if( UcCmdPar == XONLY_ON ){				// only X centering
		RamWrite32A( CMD_RETURN_TO_CENTER , CMD_FOR_LS | XAXS_SRV_ON | YAXS_SRV_OFF ) ;

	}else if( UcCmdPar == YONLY_ON ){				// only Y centering
		RamWrite32A( CMD_RETURN_TO_CENTER , CMD_FOR_LS | YAXS_SRV_ON | XAXS_SRV_OFF ) ;

	}else if( UcCmdPar == ZONLY_OFF ){				// only Z centering off
		RamWrite32A( CMD_RETURN_TO_CENTER , CMD_FOR_LS | ZAXS_SRV_OFF ) ;

	}else if( UcCmdPar == ZONLY_ON ){				// only Z centering
		RamWrite32A( CMD_RETURN_TO_CENTER , CMD_FOR_LS | ZAXS_SRV_ON ) ;

	}else{											// Both off
		RamWrite32A( CMD_RETURN_TO_CENTER , CMD_FOR_LS | BOTH_SRV_OFF ) ;
	}

	do {
		UcSndDat = RdStatus(1);
		UlTimeout--;
	} while( UcSndDat == FAILURE && UlTimeout != 0 );
#if 0
	if( UcCmdPar == ZONLY_OFF ){				// only Z centering off
		RamWrite32A( CLAF_RAMA_AFOUT		,	0x00000000 ) ;				// driver output clear
	}
#endif
CAM_ERR(CAM_OIS, "RtnCen() = %02x\n", UcSndDat ) ;
	return( UcSndDat );
}

//********************************************************************************
// Function Name 	: RtnCenBit
// Retun Value		: Command Status
// Argment Value	: Command Parameter
// Explanation		: Return to center Command Function
// History			: First edition
//********************************************************************************
UINT_8	RtnCenBit( UINT_32	UlCmdPar )
{
	UINT_8	UcSndDat = FAILURE;
	UINT_32	UlTimeout = 50000;

	RamWrite32A( CMD_RETURN_TO_CENTER , UlCmdPar ) ;

	do {
		UcSndDat = RdStatus(1);
		UlTimeout--;
	} while( UcSndDat == FAILURE && UlTimeout != 0 );

	return( UcSndDat );
}

//********************************************************************************
// Function Name 	: OisEna
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Enable Control Function
// History			: First edition
//********************************************************************************
void	OisEna( void )
{
	UINT_8	UcStRd = 1;

	RamWrite32A( CMD_OIS_ENABLE , OIS_ENABLE ) ;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
CAM_ERR(CAM_OIS, " OisEna( Status) = %02x\n", UcStRd ) ;
}

//********************************************************************************
// Function Name 	: OisEna_LS
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Enable Control Function
// History			: First edition
//********************************************************************************
void	OisEna_LS( void )
{
	UINT_8	UcStRd = 1;

	RamWrite32A( CMD_OIS_ENABLE , CMD_FOR_LS | OIS_ENABLE ) ;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
CAM_ERR(CAM_OIS, " OisEna( Status) = %02x\n", UcStRd ) ;
}

//********************************************************************************
// Function Name 	: OisEnaNCL
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Enable Control Function w/o delay clear
// History			: First edition
//********************************************************************************
void	OisEnaNCL( void )
{
	UINT_8	UcStRd = 1;

	RamWrite32A( CMD_OIS_ENABLE , OIS_ENA_NCL | OIS_ENABLE ) ;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
CAM_ERR(CAM_OIS, " OisEnaNCL( Status) = %02x\n", UcStRd ) ;
}

//********************************************************************************
// Function Name 	: OisEnaNCL_LS
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Enable Control Function w/o delay clear
// History			: First edition
//********************************************************************************
void	OisEnaNCL_LS( void )
{
	UINT_8	UcStRd = 1;

	RamWrite32A( CMD_OIS_ENABLE , CMD_FOR_LS | OIS_ENA_NCL | OIS_ENABLE ) ;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
CAM_ERR(CAM_OIS, " OisEnaNCL( Status) = %02x\n", UcStRd ) ;
}

//********************************************************************************
// Function Name 	: OisEnaDrCl
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Enable Control Function force drift cancel
// History			: First edition
//********************************************************************************
void	OisEnaDrCl( void )
{
	UINT_8	UcStRd = 1;

	RamWrite32A( CMD_OIS_ENABLE , OIS_ENA_DOF | OIS_ENABLE ) ;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
CAM_ERR(CAM_OIS, " OisEnaDrCl( Status) = %02x\n", UcStRd ) ;
}

//********************************************************************************
// Function Name 	: OisEnaDrNcl
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Enable Control Function w/o delay clear and force drift cancel
// History			: First edition
//********************************************************************************
void	OisEnaDrNcl( void )
{
	UINT_8	UcStRd = 1;

	RamWrite32A( CMD_OIS_ENABLE , OIS_ENA_DOF | OIS_ENA_NCL | OIS_ENABLE ) ;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
CAM_ERR(CAM_OIS, " OisEnaDrCl( Status) = %02x\n", UcStRd ) ;
}

//********************************************************************************
// Function Name 	: OisDis
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Disable Control Function
// History			: First edition
//********************************************************************************
void	OisDis( void )
{
	UINT_8	UcStRd = 1;

	RamWrite32A( CMD_OIS_ENABLE , OIS_DISABLE ) ;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
CAM_ERR(CAM_OIS, " OisDis( Status) = %02x\n", UcStRd ) ;
}

//********************************************************************************
// Function Name 	: OisDis_LS
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Disable Control Function
// History			: First edition
//********************************************************************************
void	OisDis_LS( void )
{
	UINT_8	UcStRd = 1;

	RamWrite32A( CMD_OIS_ENABLE , CMD_FOR_LS | OIS_DISABLE ) ;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
CAM_ERR(CAM_OIS, " OisDis( Status) = %02x\n", UcStRd ) ;
}

//********************************************************************************
// Function Name 	: SscEna
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Ssc Enable Control Function
// History			: First edition
//********************************************************************************
void	SscEna( void )
{
	UINT_8	UcStRd = 1;

	RamWrite32A( CMD_SSC_ENABLE , SSC_ENABLE ) ;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
CAM_ERR(CAM_OIS, " SscEna( Status) = %02x\n", UcStRd ) ;
}

//********************************************************************************
// Function Name 	: SscDis
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Ssc Disable Control Function
// History			: First edition
//********************************************************************************
void	SscDis( void )
{
	UINT_8	UcStRd = 1;

	RamWrite32A( CMD_SSC_ENABLE , SSC_DISABLE ) ;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
CAM_ERR(CAM_OIS, " SscDis( Status) = %02x\n", UcStRd ) ;
}


//********************************************************************************
// Function Name 	: SetRec
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Rec Mode Enable Function
// History			: First edition
//********************************************************************************
void	SetRec( void )
{
	UINT_8	UcStRd = 1;

	RamWrite32A( CMD_MOVE_STILL_MODE ,	MOVIE_MODE ) ;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
CAM_ERR(CAM_OIS, " SetRec( Status) = %02x\n", UcStRd ) ;
}


//********************************************************************************
// Function Name 	: SetRec_LS
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Rec Mode Enable Function
// History			: First edition
//********************************************************************************
void	SetRec_LS( void )
{
	UINT_8	UcStRd = 1;

	RamWrite32A( CMD_MOVE_STILL_MODE ,	CMD_FOR_LS | MOVIE_MODE ) ;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
CAM_ERR(CAM_OIS, " SetRec( Status) = %02x\n", UcStRd ) ;
}


//********************************************************************************
// Function Name 	: SetStill
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Set Still Mode Enable Function
// History			: First edition
//********************************************************************************
void	SetStill( void )
{
	UINT_8	UcStRd = 1;

	RamWrite32A( CMD_MOVE_STILL_MODE ,	STILL_MODE ) ;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
CAM_ERR(CAM_OIS, " SetRec( Status) = %02x\n", UcStRd ) ;
}

//********************************************************************************
// Function Name 	: SetStill_LS
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Set Still Mode Enable Function
// History			: First edition
//********************************************************************************
void	SetStill_LS( void )
{
	UINT_8	UcStRd = 1;

	RamWrite32A( CMD_MOVE_STILL_MODE ,	CMD_FOR_LS | STILL_MODE ) ;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
CAM_ERR(CAM_OIS, " SetRec( Status) = %02x\n", UcStRd ) ;
}

//********************************************************************************
// Function Name 	: SetRecPreview
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Rec Preview Mode Enable Function
// History			: First edition
//********************************************************************************
void	SetRecPreview( UINT_8 mode )
{
	UINT_8	UcStRd = 1;

	switch( mode ){
	case 0:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	MOVIE_MODE ) ;
		break;
	case 1:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	MOVIE_MODE1 ) ;
		break;
	case 2:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	MOVIE_MODE2 ) ;
		break;
	case 3:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	MOVIE_MODE3 ) ;
		break;
	}
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
//CAM_ERR(CAM_OIS, " SetRec( %02x ) = %02x\n", mode , UcStRd ) ;
}


//********************************************************************************
// Function Name 	: SetStillPreview
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Set Still Preview Mode Enable Function
// History			: First edition
//********************************************************************************
void	SetStillPreview( UINT_8 mode )
{
	UINT_8	UcStRd = 1;

	switch( mode ){
	case 0:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	STILL_MODE ) ;
		break;
	case 1:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	STILL_MODE1 ) ;
		break;
	case 2:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	STILL_MODE2 ) ;
		break;
	case 3:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	STILL_MODE3 ) ;
		break;
	}
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
//CAM_ERR(CAM_OIS, " SetRec( %02x ) = %02x\n", mode , UcStRd ) ;
}


//********************************************************************************
// Function Name 	: SetSinWavePara
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Sine wave Test Function
// History			: First edition
//********************************************************************************
// 	RamWrite32A( SinWave.Gain , 0x00000000 ) ;			// Gainはそれぞれ設定すること
// 	RamWrite32A( CosWave.Gain , 0x00000000 ) ;			// Gainはそれぞれ設定すること
void	SetSinWavePara( UINT_8 UcFreq ,  UINT_8 UcMethodVal )
{
	UINT_32	UlFreqDat ;

	if(UcFreq > 0x10 )
		UcFreq = 0x10 ;			/* Limit */

	if( UcFreq == 0 )
		UlFreqDat = 0xFFFFFFFF;
	else
		UlFreqDat = (UINT_32)((float)UcFreq / (float)FS_FREQ * (float)0x7FFFFFFF);

	if( UcMethodVal == CIRCWAVE) {
		RamWrite32A( SinWave_Phase	,	0x00000000 ) ;		// 正弦波の位相量
		RamWrite32A( CosWave_Phase 	,	0x20000000 );		// 正弦波の位相量 +90 deg
	} else if( UcMethodVal == YACTTEST) {
		RamWrite32A( SinWave_Phase	,	0x00000000 ) ;		// 正弦波の位相量
		RamWrite32A( CosWave_Phase 	,	0x40000000 );		// 正弦波の位相量 +180 deg
	} else {
		RamWrite32A( SinWave_Phase	,	0x00000000 ) ;		// 正弦波の位相量
		RamWrite32A( CosWave_Phase 	,	0x00000000 );		// 正弦波の位相量
	}


	if( UlFreqDat == 0xFFFFFFFF )			/* Sine波中止 */
	{
		RamWrite32A( SinWave_Offset		,	0x00000000 ) ;									// 発生周波数のオフセットを設定
		RamWrite32A( SinWave_Phase		,	0x00000000 ) ;									// 正弦波の位相量
//		RamWrite32A( SinWave_Gain		,	0x00000000 ) ;									// 発生周波数のアッテネータ(初期値は0[dB])
//		RamWrite32A( SinWave_OutAddr	,	 (UINT_32)SinWave_Output );			// 出力先アドレス

		RamWrite32A( CosWave_Offset		,	0x00000000 );									// 発生周波数のオフセットを設定
		RamWrite32A( CosWave_Phase 		,	0x00000000 );									// 正弦波の位相量
//		RamWrite32A( CosWave_Gain 		,	0x00000000 );									// 発生周波数のアッテネータ(初期値はCut)
//		RamWrite32A( CosWave_OutAddr	,	 (UINT_32)CosWave_Output );			// 出力先アドレス

		RamWrite32A( SinWaveC_Regsiter	,	0x00000000 ) ;									// Sine Wave Stop
		RamWrite32A( SinWave_OutAddr	,	0x00000000 ) ;		// 出力先アドレス
		RamWrite32A( CosWave_OutAddr	,	0x00000000 );		// 出力先アドレス
		RamWrite32A( SMAHALL_RAM_HXOFF1		,	0x00000000 ) ;				// DelayRam Clear
		RamWrite32A( SMAHALL_RAM_HYOFF1		,	0x00000000 ) ;				// DelayRam Clear
	}
	else
	{
		RamWrite32A( SinWave_Offset		,	UlFreqDat ) ;									// 発生周波数のオフセットを設定
		RamWrite32A( CosWave_Offset		,	UlFreqDat );									// 発生周波数のオフセットを設定

		RamWrite32A( SinWaveC_Regsiter	,	0x00000001 ) ;									// Sine Wave Start
		RamWrite32A( SinWave_OutAddr	,	(UINT_32)SMAHALL_RAM_HXOFF1 ) ;		// 出力先アドレス
		RamWrite32A( CosWave_OutAddr	,	(UINT_32)SMAHALL_RAM_HYOFF1 );		// 出力先アドレス

	}


}




//********************************************************************************
// Function Name 	: SetPanTiltMode
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Pan-Tilt Enable/Disable
// History			: First edition
//********************************************************************************
void	SetPanTiltMode( UINT_8 UcPnTmod )
{
	UINT_8	UcStRd = 1;

	switch ( UcPnTmod ) {
		case OFF :
			RamWrite32A( CMD_PAN_TILT ,	PAN_TILT_OFF ) ;
			break ;
		case ON :
			RamWrite32A( CMD_PAN_TILT ,	PAN_TILT_ON ) ;
			break ;
	}

	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
CAM_ERR(CAM_OIS, " PanTilt( Status) = %02x , %02x \n", UcStRd , UcPnTmod ) ;
}

//********************************************************************************
// Function Name 	: SetPanTiltMode_LS
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Pan-Tilt Enable/Disable
// History			: First edition
//********************************************************************************
void	SetPanTiltMode_LS( UINT_8 UcPnTmod )
{
	UINT_8	UcStRd = 1;

	switch ( UcPnTmod ) {
		case OFF :
			RamWrite32A( CMD_PAN_TILT ,	CMD_FOR_LS | PAN_TILT_OFF ) ;
			break ;
		case ON :
			RamWrite32A( CMD_PAN_TILT ,	CMD_FOR_LS | PAN_TILT_ON ) ;
			break ;
	}

	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
CAM_ERR(CAM_OIS, " PanTilt( Status) = %02x , %02x \n", UcStRd , UcPnTmod ) ;
}

//********************************************************************************
// Function Name 	: SetSinWavGenInt
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Sine wave generator initial Function
// History			: First edition
//********************************************************************************
void	SetSinWavGenInt( void )
{

	RamWrite32A( SinWave_Offset		,	0x00000000 ) ;		// 発生周波数のオフセットを設定
	RamWrite32A( SinWave_Phase		,	0x60000000 ) ;		// 正弦波の位相量
	RamWrite32A( SinWave_Gain		,	0x00000000 ) ;		// 発生周波数のアッテネータ(初期値は0[dB])
//	RamWrite32A( SinWave_Gain		,	0x7FFFFFFF ) ;		// 発生周波数のアッテネータ(初期値はCut)
//	RamWrite32A( SinWave_OutAddr	,	(UINT_32)SinWave_Output ) ;		// 初期値の出力先アドレスは、自分のメンバ

	RamWrite32A( CosWave_Offset		,	0x00000000 );		// 発生周波数のオフセットを設定
	RamWrite32A( CosWave_Phase 		,	0x00000000 );		// 正弦波の位相量
	RamWrite32A( CosWave_Gain 		,	0x00000000 );		// 発生周波数のアッテネータ(初期値はCut)
//	RamWrite32A( CosWave_Gain 		,	0x7FFFFFFF );		// 発生周波数のアッテネータ(初期値は0[dB])
//	RamWrite32A( CosWave_OutAddr	,	(UINT_32)CosWave_Output );		// 初期値の出力先アドレスは、自分のメンバ

	RamWrite32A( SinWaveC_Regsiter	,	0x00000000 ) ;								// Sine Wave Stop

}


//********************************************************************************
// Function Name 	: SetTransDataAdr
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Trans Address for Data Function
// History			: First edition
//********************************************************************************
void	SetTransDataAdr( UINT_16 UsLowAddress , UINT_32 UlLowAdrBeforeTrans )
{
	UnDwdVal	StTrsVal ;

	if( UlLowAdrBeforeTrans < 0x00009000 ){
		StTrsVal.StDwdVal.UsHigVal = (UINT_16)(( UlLowAdrBeforeTrans & 0x0000F000 ) >> 8 ) ;
		StTrsVal.StDwdVal.UsLowVal = (UINT_16)( UlLowAdrBeforeTrans & 0x00000FFF ) ;
	}else{
		StTrsVal.UlDwdVal = UlLowAdrBeforeTrans ;
	}
//CAM_ERR(CAM_OIS, " TRANS  ADR = %04xh , DAT = %08xh \n",UsLowAddress , StTrsVal.UlDwdVal ) ;
	RamWrite32A( UsLowAddress	,	StTrsVal.UlDwdVal );

}


//********************************************************************************
// Function Name 	: RdStatus
// Retun Value		: 0:success 1:FAILURE
// Argment Value	: bit check  0:ALL  1:bit24
// Explanation		: High level status check Function
// History			: First edition
//********************************************************************************
UINT_8	RdStatus( UINT_8 UcStBitChk )
{
	UINT_32	UlReadVal ;

	RamRead32A( CMD_READ_STATUS , &UlReadVal );
CAM_ERR(CAM_OIS, " (Rd St) = %08x\n", (UINT_32)UlReadVal ) ;
	if( UcStBitChk ){
		UlReadVal &= READ_STATUS_INI ;
	}
	if( !UlReadVal ){
		return( SUCCESS );
	}else{
		return( FAILURE );
	}
}


//********************************************************************************
// Function Name 	: DacControl
// Argument			: 0:Write, 1:Read
// Retun Value		: Firmware version
// Argment Value	: NON
// Explanation		: Dac Control Function
// History			: First edition
//********************************************************************************
void	DacControl( UINT_8 UcMode, UINT_32 UiChannel, UINT_32 PuiData )
{
	UINT_32	UlAddaInt ;
	if( !UcMode ) {	/* Write */
		RamWrite32A( CMD_IO_ADR_ACCESS , ADDA_DASEL ) ;
		RamWrite32A( CMD_IO_DAT_ACCESS , UiChannel ) ;
		RamWrite32A( CMD_IO_ADR_ACCESS , ADDA_DAO ) ;
		RamWrite32A( CMD_IO_DAT_ACCESS , PuiData ) ;
		;
		;
		UlAddaInt = 0x00000040 ;
		while ( (UlAddaInt & 0x00000040) != 0 ) {
			RamWrite32A( CMD_IO_ADR_ACCESS , ADDA_ADDAINT ) ;
			RamRead32A(  CMD_IO_DAT_ACCESS , &UlAddaInt ) ;
			;
		}
	} else {	/* Read */
		RamWrite32A( CMD_IO_ADR_ACCESS , ADDA_DASEL ) ;
		RamWrite32A( CMD_IO_DAT_ACCESS , UiChannel ) ;
		RamWrite32A( CMD_IO_ADR_ACCESS , ADDA_DAO ) ;
		RamRead32A(  CMD_IO_DAT_ACCESS , &PuiData ) ;
		;
		;
		UlAddaInt = 0x00000040 ;
		while ( (UlAddaInt & 0x00000040) != 0 ) {
			RamWrite32A( CMD_IO_ADR_ACCESS , ADDA_ADDAINT ) ;
			RamRead32A(  CMD_IO_DAT_ACCESS , &UlAddaInt ) ;
			;
		}
	}

	return ;
}

//********************************************************************************
// Function Name 	: RdGyroOffsetTbl
// Retun Value		: none
// Argment Value	: NON
// Explanation		: Read Gyro offset accel offset from filter ram Function
// History			: First edition
//********************************************************************************
void	RdGyroOffsetTbl( stGyroOffsetTbl *pTbl )
{
//------------------------------------------------------------------------------------------------
// Get Gyro offset data
//------------------------------------------------------------------------------------------------
	RamRead32A( GYRO_RAM_GXOFFZ, &pTbl->StAngle.SlOffsetX ) ;
	RamRead32A( GYRO_RAM_GYOFFZ, &pTbl->StAngle.SlOffsetY ) ;
	RamRead32A( GYRO_ZRAM_GZOFFZ, &pTbl->StAngle.SlOffsetZ ) ;
	RamRead32A( ACCLRAM_X_AC_OFFSET, &pTbl->StAccel.SlOffsetX ) ;
	RamRead32A( ACCLRAM_Y_AC_OFFSET, &pTbl->StAccel.SlOffsetY ) ;
	RamRead32A( ACCLRAM_Z_AC_OFFSET, &pTbl->StAccel.SlOffsetZ ) ;
}

//********************************************************************************
// Function Name 	: RdHallCalData
// Retun Value		: Read calibration data
// Argment Value	: NON
// Explanation		: Read calibration Data Function
// History			: First edition
//********************************************************************************
void	RdHallCalData( void )
{
	UnDwdVal		StReadVal ;

	RamRead32A(  StSmaCaliData_UsCalibrationStatus, &StAdjPar.StHalAdj.UlAdjPhs ) ;

	RamRead32A( StSmaCaliData_SiHallMax_X,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsHlxMax = StAdjPar.StHalAdj.UsHlxMxa = StReadVal.StDwdVal.UsHigVal ;
	RamRead32A( StSmaCaliData_SiHallMin_X,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsHlxMna = StAdjPar.StHalAdj.UsHlxMin = StReadVal.StDwdVal.UsHigVal ;
	RamRead32A( StSmaCaliData_SiHallMax_Y,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsHlyMxa = StAdjPar.StHalAdj.UsHlyMax = StReadVal.StDwdVal.UsHigVal ;
	RamRead32A( StSmaCaliData_SiHallMin_Y,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsHlyMna = StAdjPar.StHalAdj.UsHlyMin = StReadVal.StDwdVal.UsHigVal ;
	
	RamRead32A( StSmaCaliData_UiHallBias_X,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsHlxGan = StReadVal.StDwdVal.UsHigVal ;
	RamRead32A( StSmaCaliData_UiHallOffsetI_X,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsHlxOffI = StReadVal.StDwdVal.UsHigVal ;
	RamRead32A( StSmaCaliData_UiHallOffsetO_X,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsHlxOffO = StReadVal.StDwdVal.UsHigVal ;
	
	RamRead32A( StSmaCaliData_UiHallBias_Y,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsHlyGan = StReadVal.StDwdVal.UsHigVal ;
	RamRead32A( StSmaCaliData_UiHallOffsetI_Y,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsHlyOffI = StReadVal.StDwdVal.UsHigVal ;
	RamRead32A( StSmaCaliData_UiHallOffsetO_Y,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsHlyOffO = StReadVal.StDwdVal.UsHigVal ;

	RamRead32A( StSmaCaliData_SiLoopGain_X,	&StAdjPar.StLopGan.UlLxgVal ) ;
	RamRead32A( StSmaCaliData_SiLoopGain_Y,	&StAdjPar.StLopGan.UlLygVal ) ;

	RamRead32A( StSmaCaliData_SiLens_Offset_X,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsAdxOff = StReadVal.StDwdVal.UsHigVal ;
	RamRead32A( StSmaCaliData_SiLens_Offset_Y,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsAdyOff = StReadVal.StDwdVal.UsHigVal ;

	RamRead32A( StSmaCaliData_SiGyroOffset_X,		&StReadVal.UlDwdVal ) ;
	StAdjPar.StGvcOff.UsGxoVal = StReadVal.StDwdVal.UsHigVal ;
	RamRead32A( StSmaCaliData_SiGyroOffset_Y,		&StReadVal.UlDwdVal ) ;
	StAdjPar.StGvcOff.UsGyoVal = StReadVal.StDwdVal.UsHigVal ;

}

//********************************************************************************
// Function Name 	: OscStb
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Osc Standby Function
// History			: First edition
//********************************************************************************
void	OscStb( void )
{
	RamWrite32A( CMD_IO_ADR_ACCESS , STBOSCPLL ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS , OSC_STB ) ;
}

//********************************************************************************
// Function Name 	: GyrWhoAmIRead
// Retun Value		: Gyro Who am I Read
// Argment Value	: NON
// Explanation		: Gyro Who am I Read Function
// History			: First edition 									2016.11.01
//********************************************************************************
UINT_8	GyrWhoAmIRead( void )
{
	UINT_8		UcRtnVal ;
	UINT_32		UlReadVal;

	// Setup for self test
	RamWrite32A( 0xF01D, 0x75000000 );	// Read who am I
	while( RdStatus( 0 ) ) ;

	RamRead32A ( 0xF01D, &UlReadVal );
	
	UcRtnVal = UlReadVal >> 24;
	
CAM_ERR(CAM_OIS, "WHO AM I read %02X\n", UcRtnVal );
	
	return(UcRtnVal);
}

//********************************************************************************
// Function Name 	: GyrWhoAmICheck
// Retun Value		: Gyro Who am I Check
// Argment Value	: NON
// Explanation		: Gyro Who am I Chek Function
// History			: First edition 									2016.11.01
//********************************************************************************
UINT_8	GyrWhoAmICheck( void )
{
	UINT_8		UcReadVal ;
	
	UcReadVal = GyrWhoAmIRead();
	
	if( UcReadVal == 0x20 ){		// ICM-20690
CAM_ERR(CAM_OIS, "WHO AM I read success\n");
		return	(SUCCESS);
	}
	else{
CAM_ERR(CAM_OIS, "WHO AM I read failure\n");
		return	(FAILURE);
	}
}

//********************************************************************************
// Function Name 	: GyrIdRead
// Retun Value		: Gyro ID Read
// Argment Value	: NON
// Explanation		: Gyro ID Read Function
// History			: First edition 									2016.11.07
//********************************************************************************
UINT_8	GyrIdRead( UINT_8 *UcGyroID )
{
	UINT_8		i ;
	UINT_32		UlReadVal;

	for( i=0; i<7 ; i++ ){
		
		// bank_sel
		RamWrite32A( 0xF01E, 0x6D000000 );
		while( RdStatus( 0 ) ) ;
		
		// start_addr
		RamWrite32A( 0xF01E, 0x6E000000 | (i << 16) );
		while( RdStatus( 0 ) ) ;
		
		// mem_r_w
		RamWrite32A( 0xF01D, 0x6F000000 );
		while( RdStatus( 0 ) ) ;
		
		// ID0[7:0] / ID1[7:0] ... ID6[7:0]
		RamRead32A ( 0xF01D, &UlReadVal );
		UcGyroID[i] = UlReadVal >> 24;
	}
	
CAM_ERR(CAM_OIS, "UcGyroID %02X %02X %02X %02X %02X %02X %02X \n", UcGyroID[0], UcGyroID[1], UcGyroID[2], UcGyroID[3], UcGyroID[4], UcGyroID[5], UcGyroID[6] );
	
	return(SUCCESS);
}

#if 0
//********************************************************************************
// Function Name 	: GyroReCalib
// Retun Value		: Command Status
// Argment Value	: Offset information data pointer
// Explanation		: Re calibration Command Function
// History			: First edition
//********************************************************************************
UINT_8	GyroReCalib( stReCalib * pReCalib )
{
	UINT_8	UcSndDat ;
	UINT_32	UlRcvDat ;
	UINT_32	UlGofX, UlGofY ;
	UnDwdVal UnFactoryOffset, UnGyroOffsetX, UnGyroOffsetY;

//------------------------------------------------------------------------------------------------
// Backup ALL Calibration data
//------------------------------------------------------------------------------------------------
	RamRead32A( INF0_DATA + (ACCL_GYRO_OFFSET_X*4), (UINT_32 *)&UnGyroOffsetX   );
	RamRead32A( INF0_DATA + (ACCL_GYRO_OFFSET_Y*4), (UINT_32 *)&UnGyroOffsetY   );
	RamRead32A( INF0_DATA + (SMA_GYRO_FCTRY_OFST*4),    (UINT_32 *)&UnFactoryOffset );

//CAM_ERR(CAM_OIS, "UnGyroOffsetX   = %08X \n",	UnGyroOffsetX   );
//CAM_ERR(CAM_OIS, "UnGyroOffsetY   = %08X \n",	UnGyroOffsetY   );
//CAM_ERR(CAM_OIS, "UnFactoryOffset = %08X \n",	UnFactoryOffset );

	// HighLevelコマンド
	RamWrite32A( CMD_CALIBRATION , 0x00000000 ) ;

	do {
		UcSndDat = RdStatus(1);
	} while (UcSndDat != 0);

	RamRead32A( CMD_CALIBRATION , &UlRcvDat ) ;
	UcSndDat = (unsigned char)(UlRcvDat >> 24);								// 終了ステータス

	// 戻り値を編集
	if( UnFactoryOffset.StDwdVal.UsLowVal == 0xFFFF )
		pReCalib->SsFctryOffX = (INT_16)UnGyroOffsetX.StDwdVal.UsLowVal ;
	else
		pReCalib->SsFctryOffX = (INT_16)UnFactoryOffset.StDwdVal.UsLowVal ;

	if( UnFactoryOffset.StDwdVal.UsHigVal == 0xFFFF )
		pReCalib->SsFctryOffY = (INT_16)UnGyroOffsetY.StDwdVal.UsHigVal ;
	else
		pReCalib->SsFctryOffY = (INT_16)UnFactoryOffset.StDwdVal.UsHigVal ;

	// キャリブレーション後の値を取得
	RamRead32A(  GYRO_RAM_GXOFFZ , &UlGofX ) ;
	RamRead32A(  GYRO_RAM_GYOFFZ , &UlGofY ) ;

	pReCalib->SsRecalOffX = (UlGofX >> 16) ;
	pReCalib->SsRecalOffY = (UlGofY >> 16) ;
	pReCalib->SsDiffX = pReCalib->SsFctryOffX - pReCalib->SsRecalOffX ;
	pReCalib->SsDiffY = pReCalib->SsFctryOffY - pReCalib->SsRecalOffY ;

CAM_ERR(CAM_OIS, "GyroReCalib() = %02x\n", UcSndDat ) ;
CAM_ERR(CAM_OIS, "Factory X = %04X, Y = %04X\n", pReCalib->SsFctryOffX, pReCalib->SsFctryOffY );
CAM_ERR(CAM_OIS, "Recalib X = %04X, Y = %04X\n", pReCalib->SsRecalOffX, pReCalib->SsRecalOffY );
CAM_ERR(CAM_OIS, "Diff    X = %04X, Y = %04X\n", pReCalib->SsDiffX, pReCalib->SsDiffY );

	return( UcSndDat );
}
#endif

//********************************************************************************
// Function Name 	: ReadCalibID
// Retun Value		: Calibraion ID
// Argment Value	: NONE
// Explanation		: Read calibraion ID Function
// History			: First edition
//********************************************************************************
UINT_32	ReadCalibID( void )
{
	UINT_32	UlCalibId;

	// Read calibration data
	RamRead32A( SiCalID, &UlCalibId );

	return( UlCalibId );
}


//********************************************************************************
// Function Name 	: FrqDet
// Retun Value		: 0:PASS, 1:OIS X NG, 2:OIS Y NG, 4:CLAF NG
// Argment Value	: NON
// Explanation		: Module Check
// History			: First edition
//********************************************************************************
UINT_8 FrqDet( void )
{
	INT_32 SlMeasureParameterA , SlMeasureParameterB ;
	INT_32 SlMeasureParameterNum ;
	UINT_32 UlXasP_P , UlYasP_P ;
//	UINT_32 UlAasP_P ;
	UnllnVal StMeasValue ;

	UINT_8 UcRtnVal;

	// Select parameter
	DSPVER Info;
	GetInfomation( &Info ) ;

	UcRtnVal = 0;

	//Measurement Setup
	MesFil( OSCCHK ) ;											// Set Measure Filter

	SlMeasureParameterNum	=	1000 ;								// 1000times( 50ms )

	SlMeasureParameterA		=	(UINT_32)SMAHALL_RAM_HXIDAT ;				// Set Measure RAM Address
	SlMeasureParameterB		=	(UINT_32)SMAHALL_RAM_HYIDAT ;				// Set Measure RAM Address

	// Start measure
	MeasureStart2( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB, 50 ) ;
	MeasureWait() ;														// Wait complete of measurement

	RamRead32A( StMeasFunc_MFA_UiAmp1, &UlXasP_P ) ;					// X Axis Peak to Peak
	RamRead32A( StMeasFunc_MFB_UiAmp2, &UlYasP_P ) ;					// Y Axis Peak to Peak
CAM_ERR(CAM_OIS, "UlXasP_P = %08X\n", (int)UlXasP_P ) ;
CAM_ERR(CAM_OIS, "UlYasP_P = %08X\n", (int)UlYasP_P ) ;

	// Amplitude value check X
	if(  UlXasP_P > ULTHDVAL ){
		UcRtnVal = 1;
	}
	// Amplitude value check Y
	if(  UlYasP_P > ULTHDVAL ){
		UcRtnVal |= 2;
	}

	// Get detail data
	RamRead32A( StMeasFunc_MFA_SiMax1,				&StHAT_X.SlMeasureMaxValue ) ;		// Max value
	RamRead32A( StMeasFunc_MFA_SiMin1,				&StHAT_X.SlMeasureMinValue ) ;		// Min value
	RamRead32A( StMeasFunc_MFA_UiAmp1,				&StHAT_X.SlMeasureAmpValue ) ;		// Amp value
	RamRead32A( StMeasFunc_MFA_LLiIntegral1,		&StMeasValue.StUllnVal.UlLowVal ) ;	// Integration Lo
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4,	&StMeasValue.StUllnVal.UlHigVal ) ;	// Integration Hi
	StHAT_X.SlMeasureAveValue =
				(INT_32)( (INT_64)StMeasValue.UllnValue / SlMeasureParameterNum ) ;		// Ave value
	
	RamRead32A( StMeasFunc_MFB_SiMax2,				&StHAT_Y.SlMeasureMaxValue ) ;		// Max value
	RamRead32A( StMeasFunc_MFB_SiMin2,				&StHAT_Y.SlMeasureMinValue ) ;		// Min value
	RamRead32A( StMeasFunc_MFB_UiAmp2,				&StHAT_Y.SlMeasureAmpValue ) ;		// Amp value
	RamRead32A( StMeasFunc_MFB_LLiIntegral2,		&StMeasValue.StUllnVal.UlLowVal ) ;	// Integration Lo
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4,	&StMeasValue.StUllnVal.UlHigVal ) ;	// Integration Hi
	StHAT_Y.SlMeasureAveValue = 
				(INT_32)( (INT_64)StMeasValue.UllnValue / SlMeasureParameterNum ) ;		// Ave value

CAM_ERR(CAM_OIS, "X Max = %08X\n", (int)StHAT_X.SlMeasureMaxValue ) ;
CAM_ERR(CAM_OIS, "X Min = %08X\n", (int)StHAT_X.SlMeasureMinValue ) ;
CAM_ERR(CAM_OIS, "X Amp = %08X\n", (int)StHAT_X.SlMeasureAmpValue ) ;
CAM_ERR(CAM_OIS, "X Ave = %08X\n", (int)StHAT_X.SlMeasureAveValue ) ;
CAM_ERR(CAM_OIS, "Y Max = %08X\n", (int)StHAT_Y.SlMeasureMaxValue ) ;
CAM_ERR(CAM_OIS, "Y Min = %08X\n", (int)StHAT_Y.SlMeasureMinValue ) ;
CAM_ERR(CAM_OIS, "Y Amp = %08X\n", (int)StHAT_Y.SlMeasureAmpValue ) ;
CAM_ERR(CAM_OIS, "Y Ave = %08X\n", (int)StHAT_Y.SlMeasureAveValue ) ;

	return(UcRtnVal);													// Retun Status value
}

//********************************************************************************
// Function Name 	: FrqDetReTry
// Retun Value		: 0:PASS, 1:OIS X NG, 2:OIS Y NG
// Argment Value	: NON
// Explanation		: Module Check
// History			: First edition
//********************************************************************************
UINT_8 FrqDetReTry( void )
{
	UINT_8 UcRtnVal;
	UINT_8 UcCntVal;
	
	UcRtnVal = 0;
	
	for( UcCntVal=0; UcCntVal<3; UcCntVal++ ){
		UcRtnVal = FrqDet();
		if( UcRtnVal == 0 ){
			break;
		}
		WitTim( 500 ) ;
	}
	
	return(UcRtnVal);													// Retun Status value
}

//********************************************************************************
// Function Name 	: FrqDetWGyr
// Retun Value		: 0:PASS, 1:OIS X NG, 2:OIS Y NG
// Argment Value	: NON
// Explanation		: Module Check
// History			: First edition
//********************************************************************************
UINT_8 FrqDetWGyr( UINT_8 ucAxis )
{
	INT_32 SlMeasureParameterA , SlMeasureParameterB ;
	INT_32 SlMeasureParameterNum ;
	UINT_32 UlHasP_P , UlGasP_P ;

	UINT_8 UcRtnVal;

	UcRtnVal = 0;

	//Measurement Setup
	MesFil( OSCCHK ) ;													// Set Measure Filter

////////// ////////// ////////// ////////// ////////// ////////// ////////// //////////

	SlMeasureParameterNum	=	1000 ;									// 1000times( 50ms )

	if( ucAxis == X_DIR ){
		SlMeasureParameterA		=	(UINT_32)SMAHALL_RAM_HXOUT0 ;			// Set Measure RAM Address
		SlMeasureParameterB		=	(UINT_32)GYRO_RAM_GX_ADIDAT ;		// Set Measure RAM Address
	}
	else{
		SlMeasureParameterA		=	(UINT_32)SMAHALL_RAM_HYOUT0 ;			// Set Measure RAM Address
		SlMeasureParameterB		=	(UINT_32)GYRO_RAM_GY_ADIDAT ;		// Set Measure RAM Address
	}

	ClrMesFil() ;														// Clear Delay Ram

	// Start measure
	MeasureStart( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB ) ;
	SetWaitTime(1) ;
	MeasureWait() ;														// Wait complete of measurement
	RamRead32A( StMeasFunc_MFA_UiAmp1, &UlHasP_P ) ;					// H Peak to Peak
	RamRead32A( StMeasFunc_MFB_UiAmp2, &UlGasP_P ) ;					// G Peak to Peak

CAM_ERR(CAM_OIS, "UlHasP_P = 0x%08X\r\n", (UINT_32)UlHasP_P ) ;
CAM_ERR(CAM_OIS, "UlGasP_P = 0x%08X\r\n", (UINT_32)UlGasP_P ) ;

	// Amplitude value check X
	if(  UlHasP_P > ULTHDVAL ){
		if( ucAxis == X_DIR ){
			UcRtnVal = 1;
		}
		else{
			UcRtnVal |= 2;
		}
	}
	
	return(UcRtnVal);													// Retun Status value
}

//********************************************************************************
// Function Name 	: MesRam
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Measure 
// History			: First edition 						2015.07.06 
//********************************************************************************
UINT_8	 MesRam( INT_32 SlMeasureParameterA, INT_32 SlMeasureParameterB, INT_32 SlMeasureParameterNum, stMesRam* pStMesRamA, stMesRam* pStMesRamB )
{
	UnllnVal	StMeasValueA , StMeasValueB ;
	
	MesFil( THROUGH ) ;							// Set Measure Filter
	
	MeasureStart2( SlMeasureParameterNum,  SlMeasureParameterA, SlMeasureParameterB, 1 ) ;		// Start measure
	
	MeasureWait() ;								// Wait complete of measurement
	
	// A : X axis
	RamRead32A( StMeasFunc_MFA_SiMax1 , &(pStMesRamA->SlMeasureMaxValue) ) ;			// Max value
	RamRead32A( StMeasFunc_MFA_SiMin1 , &(pStMesRamA->SlMeasureMinValue) ) ;			// Min value
	RamRead32A( StMeasFunc_MFA_UiAmp1 , &(pStMesRamA->SlMeasureAmpValue) ) ;			// Amp value
	RamRead32A( StMeasFunc_MFA_LLiIntegral1,	 &(StMeasValueA.StUllnVal.UlLowVal) ) ;	// Integration Low
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4, &(StMeasValueA.StUllnVal.UlHigVal) ) ;	// Integration Hig
	pStMesRamA->SlMeasureAveValue = 
				(INT_32)( (INT_64)StMeasValueA.UllnValue / SlMeasureParameterNum ) ;	// Ave value
	
	// B : Y axis
	RamRead32A( StMeasFunc_MFB_SiMax2 , &(pStMesRamB->SlMeasureMaxValue) ) ;			// Max value
	RamRead32A( StMeasFunc_MFB_SiMin2 , &(pStMesRamB->SlMeasureMinValue) ) ;			// Min value
	RamRead32A( StMeasFunc_MFB_UiAmp2 , &(pStMesRamB->SlMeasureAmpValue) ) ;			// Amp value
	RamRead32A( StMeasFunc_MFB_LLiIntegral2,	 &(StMeasValueB.StUllnVal.UlLowVal) ) ;	// Integration Low
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4, &(StMeasValueB.StUllnVal.UlHigVal) ) ;	// Integration Hig
	pStMesRamB->SlMeasureAveValue = 
				(INT_32)( (INT_64)StMeasValueB.UllnValue / SlMeasureParameterNum ) ;	// Ave value
	
	return( 0 );
}

//********************************************************************************
// Function Name 	: GetInfomation
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Information Data
// History			: First edition
//********************************************************************************
UINT_8 GetInfomation( DSPVER* Info )
{
	UnDwdVal	Data;

	RamRead32A( (SiVerNum + 0), &Data.UlDwdVal );
	Info->Vendor 		= Data.StCdwVal.UcRamVa3;	// [31:24]
	Info->User 			= Data.StCdwVal.UcRamVa2;	// [23:16]
	Info->Model 		= Data.StCdwVal.UcRamVa1;	// [15:08]
	Info->Version 		= Data.StCdwVal.UcRamVa0;	// [07:00]

	RamRead32A( (SiVerNum + 4), &Data.UlDwdVal );
	Info->Reserve0 		= Data.StCdwVal.UcRamVa3;	// [31:24]
	Info->Reserve1		= Data.StCdwVal.UcRamVa2;	// [23:16]
	Info->ActVersion	= Data.StCdwVal.UcRamVa1;	// [15:08]
	Info->GyroType		= Data.StCdwVal.UcRamVa0;	// [07:00]


CAM_ERR(CAM_OIS, " ( Vender , User   , Model  , Version ) = ( %02X,%02X,%02X,%02X ) \n", Info->Vendor, Info->User, Info->Model,Info->Version );
CAM_ERR(CAM_OIS, " (        ,        , ActVer , GyroType  ) = (   ,  ,%02X,%02X ) \n", Info->ActVersion, Info->GyroType );
	return( 0 );
}

//********************************************************************************
// Function Name 	: GyrRegWriteA
// Retun Value		: NON
// Argment Value	: Address, Write Data
// Explanation		: Gyro Register Write
// History			: First edition
//********************************************************************************
void GyrRegWriteA(UINT_8 GyrAddr, UINT_8 GyrData)
{
	UINT_32	UlWriteVal;
	
	UlWriteVal	= (unsigned long)((GyrAddr << 24) | (GyrData<< 16));
	
	RamWrite32A( 0xF01E, UlWriteVal );
	
//CAM_ERR(CAM_OIS, "GyrRegWriteA = %08X\n", UlWriteVal );
	
	while( RdStatus( 0 ) ) ;
}

//********************************************************************************
// Function Name 	: GyrRegReadA
// Retun Value		: NON
// Argment Value	: Address, Read Data Pointer
// Explanation		: Gyro Register Read
// History			: First edition
//********************************************************************************
void GyrRegReadA(UINT_8 GyrAddr, UINT_8* GyrData)
{
	INT_32	UlReadVal, UlWriteVal;
	
	UlWriteVal	= (unsigned long)(GyrAddr << 24);
	
	RamWrite32A( 0xF01D, UlWriteVal );
	while( RdStatus( 0 ) ) ;
	
	RamRead32A( 0xF01D, &UlReadVal );
	
	*GyrData	= (unsigned char)(UlReadVal >> 24);
	
//CAM_ERR(CAM_OIS, "GyrRegReadA = %08X\n", UlReadVal );
//CAM_ERR(CAM_OIS, "GyrRegReadA = %02X char\n", *GyrData );
}

//********************************************************************************
// Function Name 	: SetSlaveAddr
// Retun Value		: NON
// Argment Value	: I2C Slave Address
// Explanation		: 
// History			: First edition
//********************************************************************************
void SetSlaveAddr(UINT_8 SlaveAddr)
{
	UINT_32		UlReadVal ;
	
	RamWrite32A( CMD_IO_ADR_ACCESS , SADRWPB ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS , 0x00000001 ) ;
	
	RamWrite32A( CMD_IO_ADR_ACCESS , SADR ) ;
	if(SlaveAddr == 0x48){
		RamWrite32A( CMD_IO_DAT_ACCESS , SlaveAddr | 0x01 ) ;
	}
	else{
		RamWrite32A( CMD_IO_DAT_ACCESS , SlaveAddr ) ;
	}
	
	setI2cSlvAddr( SlaveAddr, 2 );
	RamWrite32A( CMD_IO_ADR_ACCESS , SADRWPB ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS , 0x00000000 ) ;
	
	RamWrite32A( CMD_IO_ADR_ACCESS , SADR ) ;
	RamRead32A ( CMD_IO_DAT_ACCESS, &UlReadVal ) ;
CAM_ERR(CAM_OIS, "SADR[%8X] \n", UlReadVal);
}

//********************************************************************************
// Function Name 	: SetSlaveAddr
// Retun Value		: NON
// Argment Value	: I2C Slave Address
// Explanation		: 
// History			: First edition
//********************************************************************************
void	SetGyroCoef( UINT_8 UcCnvF );
void	SetAccelCoef( UINT_8 UcCnvF );
	
	UINT_8 GetSlaveAddr(void)
{
	UINT_32 UlReadVal;
	UINT_8 UcReadVal;
	
	RamWrite32A( CMD_IO_ADR_ACCESS , SADR ) ;
	RamRead32A(  CMD_IO_DAT_ACCESS , &UlReadVal ) ;
	
	UcReadVal = (UINT_8)UlReadVal;
	
	return(UcReadVal);
}

//********************************************************************************
// Function Name 	: SetAngleCorrection
// Retun Value		: True/Fail
// Argment Value	: 
// Explanation		: Angle Correction
// History			: First edition
//********************************************************************************
/*  bit7  	HX GYR			Hall X  と同方向のGyro信号がGX?               0:GX  1:GY  */
/*  bit6  	HX GYR pol		Hall X+ と同方向のGyro信号がX+とG+で同方向?   0:NEG 1:POS */
/*  bit5  	HY GYR pol		Hall Y+ と同方向のGyro信号がY+とG+で同方向?   0:NEG 1:POS */
/*  bit4  	GZ pol			基本極性に対してGyroZ信号が同方向?            0:NEG 1:POS */
/*  bit3  	HX ACL			Hall X  と同方向のAccl信号がAX?               0:AX  1:AY  */
/*  bit2  	HX ACL pol		Hall X+ と同方向のAccl信号がX+とA+で同方向?   0:NEG 1:POS */
/*  bit1  	HY ACL pol		Hall Y+ と同方向のAccl信号がY+とA+で同方向?   0:NEG 1:POS */
/*  bit0  	AZ pol			基本極性に対してAcclZ信号が同方向?            0:NEG 1:POS */
                      //   top0°btm0°//
const UINT_8 PACT0Tbl[] = { 0xFF, 0xFF };	/* Dummy table */
const UINT_8 PACT1Tbl[] = { 0x20, 0xDF };	/* [ACT_02][ACT_01][ACT_03] */
const UINT_8 PACT2Tbl[] = { 0x46, 0xB9 };	/* [---] */


UINT_8 SetAngleCorrection( float DegreeGap, UINT_8 SelectAct, UINT_8 Arrangement )
{
	double OffsetAngle = 0.0f;
	INT_32 Slgx45x = 0, Slgx45y = 0;
	INT_32 Slgy45y = 0, Slgy45x = 0;
	
	UINT_8	UcCnvF = 0;

	if( ( DegreeGap > 180.0f) || ( DegreeGap < -180.0f ) ) return ( 1 );
	if( Arrangement >= 2 ) return ( 1 );

/************************************************************************/
/*      	Gyro angle correction										*/
/************************************************************************/
	switch(SelectAct) {
//		case 0x00 :
//		case 0x01 :
//		case 0x02 :
//		case 0x03 :
//		case 0x05 :
//		case 0x06 :
//		case 0x07 :
//		case 0x08 :
		default :
			OffsetAngle = (double)( DegreeGap ) * 3.141592653589793238 / 180.0f ;
			UcCnvF = PACT1Tbl[ Arrangement ];
			break;
//		default :
//			break;
	}
	
	SetGyroCoef( UcCnvF );
	SetAccelCoef( UcCnvF );

	//***********************************************//
	// Gyro & Accel rotation correction
	//***********************************************//
	//Slgx45x = (INT_32)( cos( OffsetAngle )*2147483647.0);
	//Slgx45y = (INT_32)(-sin( OffsetAngle )*2147483647.0);
	//Slgy45y = (INT_32)( cos( OffsetAngle )*2147483647.0);
	//Slgy45x = (INT_32)( sin( OffsetAngle )*2147483647.0);
	
	RamWrite32A( GyroFilterTableX_gx45x , 			(UINT_32)Slgx45x );
	RamWrite32A( GyroFilterTableX_gx45y , 			(UINT_32)Slgx45y );
	RamWrite32A( GyroFilterTableY_gy45y , 			(UINT_32)Slgy45y );
	RamWrite32A( GyroFilterTableY_gy45x , 			(UINT_32)Slgy45x );
	RamWrite32A( Accl45Filter_XAmain , 				(UINT_32)Slgx45x );
	RamWrite32A( Accl45Filter_XAsub  , 				(UINT_32)Slgx45y );
	RamWrite32A( Accl45Filter_YAmain , 				(UINT_32)Slgy45y );
	RamWrite32A( Accl45Filter_YAsub  , 				(UINT_32)Slgy45x );
	

	return ( 0 );
}

void	SetGyroCoef( UINT_8 UcCnvF )
{
	INT_32 Slgxx = 0, Slgxy = 0;
	INT_32 Slgyy = 0, Slgyx = 0;
	INT_32 Slgzp = 0;
	/************************************************/
	/*  signal convet								*/
	/************************************************/
	switch( UcCnvF & 0xE0 ){
		/* HX <== GX , HY <== GY */
	case 0x00:
		Slgxx = 0x7FFFFFFF ;	Slgxy = 0x00000000 ;	Slgyy = 0x7FFFFFFF ;	Slgyx = 0x00000000 ;	break;	//HX<==GX(NEG), HY<==GY(NEG)
	case 0x20:
		Slgxx = 0x7FFFFFFF ;	Slgxy = 0x00000000 ;	Slgyy = 0x80000001 ;	Slgyx = 0x00000000 ;	break;	//HX<==GX(NEG), HY<==GY(POS)
	case 0x40:
		Slgxx = 0x80000001 ;	Slgxy = 0x00000000 ;	Slgyy = 0x7FFFFFFF ;	Slgyx = 0x00000000 ;	break;	//HX<==GX(POS), HY<==GY(NEG)
	case 0x60:
		Slgxx = 0x80000001 ;	Slgxy = 0x00000000 ;	Slgyy = 0x80000001 ;	Slgyx = 0x00000000 ;	break;	//HX<==GX(POS), HY<==GY(POS)
		/* HX <== GY , HY <== GX */
	case 0x80:
		Slgxx = 0x00000000 ;	Slgxy = 0x7FFFFFFF ;	Slgyy = 0x00000000 ;	Slgyx = 0x7FFFFFFF ;	break;	//HX<==GY(NEG), HY<==GX(NEG)
	case 0xA0:
		Slgxx = 0x00000000 ;	Slgxy = 0x7FFFFFFF ;	Slgyy = 0x00000000 ;	Slgyx = 0x80000001 ;	break;	//HX<==GY(NEG), HY<==GX(POS)
	case 0xC0:
		Slgxx = 0x00000000 ;	Slgxy = 0x80000001 ;	Slgyy = 0x00000000 ;	Slgyx = 0x7FFFFFFF ;	break;	//HX<==GY(POS), HY<==GX(NEG)
	case 0xE0:
		Slgxx = 0x00000000 ;	Slgxy = 0x80000001 ;	Slgyy = 0x00000000 ;	Slgyx = 0x80000001 ;	break;	//HX<==GY(NEG), HY<==GX(NEG)
	}
	switch( UcCnvF & 0x10 ){
	case 0x00:
		Slgzp = 0x7FFFFFFF ;	break;																			//GZ(POS)
	case 0x10:
		Slgzp = 0x80000001 ;	break;																			//GZ(NEG)
	}
	RamWrite32A( MS_SEL_GX0 , (UINT_32)Slgxx );
	RamWrite32A( MS_SEL_GX1 , (UINT_32)Slgxy );
	RamWrite32A( MS_SEL_GY0 , (UINT_32)Slgyy );
	RamWrite32A( MS_SEL_GY1 , (UINT_32)Slgyx );
	RamWrite32A( MS_SEL_GZ , (UINT_32)Slgzp );
}

void	SetAccelCoef( UINT_8 UcCnvF )
{
	INT_32 Slaxx = 0, Slaxy = 0;
	INT_32 Slayy = 0, Slayx = 0;
	INT_32 Slazp = 0;
	
	switch( UcCnvF & 0x0E ){
		/* HX <== AX , HY <== AY */
	case 0x00:
		Slaxx = 0x7FFFFFFF ;	Slaxy = 0x00000000 ;	Slayy = 0x7FFFFFFF ;	Slayx = 0x00000000 ;	break;	//HX<==AX(NEG), HY<==AY(NEG)
	case 0x02:
		Slaxx = 0x7FFFFFFF ;	Slaxy = 0x00000000 ;	Slayy = 0x80000001 ;	Slayx = 0x00000000 ;	break;	//HX<==AX(NEG), HY<==AY(POS)
	case 0x04:
		Slaxx = 0x80000001 ;	Slaxy = 0x00000000 ;	Slayy = 0x7FFFFFFF ;	Slayx = 0x00000000 ;	break;	//HX<==AX(POS), HY<==AY(NEG)
	case 0x06:
		Slaxx = 0x80000001 ;	Slaxy = 0x00000000 ;	Slayy = 0x80000001 ;	Slayx = 0x00000000 ;	break;	//HX<==AX(POS), HY<==AY(POS)
		/* HX <== AY , HY <== AX */
	case 0x08:
		Slaxx = 0x00000000 ;	Slaxy = 0x7FFFFFFF ;	Slayy = 0x00000000 ;	Slayx = 0x7FFFFFFF ;	break;	//HX<==AY(NEG), HY<==AX(NEG)
	case 0x0A:
		Slaxx = 0x00000000 ;	Slaxy = 0x7FFFFFFF ;	Slayy = 0x00000000 ;	Slayx = 0x80000001 ;	break;	//HX<==AY(NEG), HY<==AX(POS)
	case 0x0C:
		Slaxx = 0x00000000 ;	Slaxy = 0x80000001 ;	Slayy = 0x00000000 ;	Slayx = 0x7FFFFFFF ;	break;	//HX<==AY(POS), HY<==AX(NEG)
	case 0x0E:
		Slaxx = 0x00000000 ;	Slaxy = 0x80000001 ;	Slayy = 0x00000000 ;	Slayx = 0x80000001 ;	break;	//HX<==AY(NEG), HY<==AX(NEG)
	}
	switch( UcCnvF & 0x01 ){
	case 0x00:
		Slazp = 0x7FFFFFFF ;	break;																			//AZ(POS)
	case 0x01:
		Slazp = 0x80000001 ;	break;																			//AZ(NEG)
	}
	RamWrite32A( MS_SEL_AX0 , (UINT_32)Slaxx );
	RamWrite32A( MS_SEL_AX1 , (UINT_32)Slaxy );
	RamWrite32A( MS_SEL_AY0 , (UINT_32)Slayy );
	RamWrite32A( MS_SEL_AY1 , (UINT_32)Slayx );
	RamWrite32A( MS_SEL_AZ , (UINT_32)Slazp );
}


//********************************************************************************
// Function Name 	: SetGyroOffset
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: set the gyro offset data. before do this before Remapmain.
// History			: First edition 						
//********************************************************************************
void	SetGyroOffset( UINT_16 GyroOffsetX, UINT_16 GyroOffsetY, UINT_16 GyroOffsetZ )
{
	RamWrite32A( GYRO_RAM_GXOFFZ , (( GyroOffsetX << 16 ) & 0xFFFF0000 ) ) ;		// X axis Gyro offset
	RamWrite32A( GYRO_RAM_GYOFFZ , (( GyroOffsetY << 16 ) & 0xFFFF0000 ) ) ;		// Y axis Gyro offset
	RamWrite32A( GYRO_ZRAM_GZOFFZ , (( GyroOffsetZ << 16 ) & 0xFFFF0000 ) ) ;		// Z axis Gyro offset
}


//********************************************************************************
// Function Name 	: SetAcclOffset
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: set the accl offset data. before do this before Remapmain.
// History			: First edition 						
//********************************************************************************
void	SetAcclOffset( UINT_16 AcclOffsetX, UINT_16 AcclOffsetY, UINT_16 AcclOffsetZ )
{
	RamWrite32A( ACCLRAM_X_AC_OFFSET , ( ( AcclOffsetX << 16 ) & 0xFFFF0000 ) ) ;		// X axis Accl offset
	RamWrite32A( ACCLRAM_Y_AC_OFFSET , ( ( AcclOffsetY << 16 ) & 0xFFFF0000 ) ) ;		// Y axis Accl offset
	RamWrite32A( ACCLRAM_Z_AC_OFFSET , ( ( AcclOffsetZ << 16 ) & 0xFFFF0000 ) ) ;		// Z axis Accl offset
}

void	GetGyroOffset( UINT_16* GyroOffsetX, UINT_16* GyroOffsetY, UINT_16* GyroOffsetZ )
{
	UINT_32	ReadValX, ReadValY, ReadValZ;
	RamRead32A( GYRO_RAM_GXOFFZ , &ReadValX );	
	RamRead32A( GYRO_RAM_GYOFFZ , &ReadValY );	
	RamRead32A( GYRO_ZRAM_GZOFFZ , &ReadValZ );	
	*GyroOffsetX = ( UINT_16 )(( ReadValX >> 16) & 0x0000FFFF );
	*GyroOffsetY = ( UINT_16 )(( ReadValY >> 16) & 0x0000FFFF );
	*GyroOffsetZ = ( UINT_16 )(( ReadValZ >> 16) & 0x0000FFFF );	
}
	
void	GetAcclOffset( UINT_16* AcclOffsetX, UINT_16* AcclOffsetY, UINT_16* AcclOffsetZ )
{
	UINT_32	ReadValX, ReadValY, ReadValZ;
	RamRead32A( ACCLRAM_X_AC_OFFSET , &ReadValX );	
	RamRead32A( ACCLRAM_Y_AC_OFFSET , &ReadValY );	
	RamRead32A( ACCLRAM_Z_AC_OFFSET , &ReadValZ );	
	*AcclOffsetX = ( UINT_16 )(( ReadValX >> 16) & 0x0000FFFF );
	*AcclOffsetY = ( UINT_16 )(( ReadValY >> 16) & 0x0000FFFF );
	*AcclOffsetZ = ( UINT_16 )(( ReadValZ >> 16) & 0x0000FFFF );	
}

//********************************************************************************
// Function Name 	: TurnOffOISLinearity
// Retun Value	: SUCCESS or FAILURE
// Argment Value	: NON
// Explanation	: make coefficient function
// History		: 
//********************************************************************************
void TurnOffOISLinearity (void)
{
	int ReadVal;
		
	RamWrite32A( SmaHF_hx45x, 0x7FFFFFFF );			
	RamWrite32A( SmaHF_hx45y, 0x00000000 );			
	RamWrite32A( SmaHF_hy45y, 0x7FFFFFFF );				
	RamWrite32A( SmaHF_hy45x, 0x00000000 );	
	RamWrite32A( SmaHF_ShiftX, 0x00000000 );	

	RamWrite32A( SmaHallFilterCoeffX_GYROXOUTGAIN, 0x7FFFFFFF );
	RamWrite32A( SmaHallFilterCoeffY_GYROYOUTGAIN, 0x7FFFFFFF );

	// OISLinearity OFF
	RamRead32A( GYRO_RAM_GYRO_Switch, &ReadVal );
	RamWrite32A( GYRO_RAM_GYRO_Switch, (ReadVal & ~0x00000008) );		
}



//********************************************************************************
// Function Name 	: TurnOffOISLinearity_LS
// Retun Value		: SUCCESS or FAILURE
// Argment Value	: NON
// Explanation		: make coefficient function
// History			: 
//********************************************************************************
void TurnOffOISLinearity_LS (void)
{
	int ReadVal;
		
	RamWrite32A( HF_hx45x, 0x7FFFFFFF );			
	RamWrite32A( HF_hx45y, 0x00000000 );			
	RamWrite32A( HF_hy45y, 0x7FFFFFFF );				
	RamWrite32A( HF_hy45x, 0x00000000 );	
	RamWrite32A( HF_ShiftX, 0x00000000 );	

	RamWrite32A( HallFilterCoeffX_GYROXOUTGAIN, 0x7FFFFFFF );
	RamWrite32A( HallFilterCoeffY_GYROYOUTGAIN, 0x7FFFFFFF );

	// OISLinearity OFF
	RamRead32A( GYRO_RAM_GYRO_Switch_LS, &ReadVal );
	RamWrite32A( GYRO_RAM_GYRO_Switch_LS, (ReadVal & ~0x00000008) );		
}



//********************************************************************************
// Function Name	: RegWrite402
// Retun Value		: Success or Failure
// Argment Value	: Slave Address, Address, Data
// Explanation		: LC898402 Register Write Function
// History			: 
//********************************************************************************
UINT_8	RegWrite402( UINT_8 uc_slv_address, UINT_8 uc_address, UINT_8 uc_data )
{
	UINT_32	ul_data ;
	UINT_8	uc_time_out	= 0 ;

	ul_data	= ( ( UINT_32 )uc_slv_address << 24 ) | ( ( UINT_32 )uc_address << 16 ) | ( ( UINT_32 )uc_data << 8 ) ;

	RamWrite32A( CMD_402_BYTE_WRITE, ul_data ) ;

	do {
		RamRead32A( CMD_402_BYTE_WRITE, &ul_data ) ;
		uc_time_out++ ;
	} while( ( uc_time_out < 100 ) && ( ul_data & 0x000000FF ) ) ;

	if( uc_time_out >= 100 ) {
		return( FAILURE ) ;
	} else {
		return( SUCCESS ) ;
	}
}



//********************************************************************************
// Function Name	: RegRead402
// Retun Value		: Success or Failure
// Argment Value	: Slave Address, Address, Data Pointer
// Explanation		: LC898402 Register Read Function
// History			: 
//********************************************************************************
UINT_8	RegRead402( UINT_8 uc_slv_address, UINT_8 uc_address, UINT_8 *puc_data )
{
	UINT_32	ul_data ;
	UINT_8	uc_time_out	= 0 ;

	ul_data	= ( ( UINT_32 )uc_slv_address << 24 ) | ( ( UINT_32 )uc_address << 16 ) ;

	RamWrite32A( CMD_402_BYTE_READ, ul_data ) ;

	do {
		RamRead32A( CMD_402_BYTE_READ, &ul_data ) ;
		uc_time_out++ ;
	} while( ( uc_time_out < 100 ) && ( ul_data & 0x000000FF ) ) ;

	RamRead32A( CMD_402_RX_BUFFER_READ_WRITE, &ul_data ) ;

	*puc_data	= ( UINT_8 )( ( ul_data >> 24 ) & 0x000000FF ) ;

	if( uc_time_out >= 100 ) {
		return( FAILURE ) ;
	} else {
		return( SUCCESS ) ;
	}
}



//********************************************************************************
// Function Name	: RamWrite402
// Retun Value		: Success or Failure
// Argment Value	: Slave Address, Address, Data
// Explanation		: LC898402 RAM Write Function
// History			: 
//********************************************************************************
UINT_8	RamWrite402( UINT_8 uc_slv_address, UINT_8 uc_address, UINT_16 us_data )
{
	UINT_32	ul_data ;
	UINT_8	uc_time_out	= 0 ;

	ul_data	= ( ( UINT_32 )uc_slv_address << 24 ) | ( ( UINT_32 )uc_address << 16 ) | ( UINT_32 )us_data ;

	RamWrite32A( CMD_402_SHORT_WRITE, ul_data ) ;

	do {
		RamRead32A( CMD_402_SHORT_WRITE, &ul_data ) ;
		uc_time_out++ ;
	} while( ( uc_time_out < 100 ) && ( ul_data & 0x000000FF ) ) ;

	if( uc_time_out >= 100 ) {
		return( FAILURE ) ;
	} else {
		return( SUCCESS ) ;
	}
}



//********************************************************************************
// Function Name	: RamRead402
// Retun Value		: Success or Failure
// Argment Value	: Slave Address, Address, Data Pointer
// Explanation		: LC898402 RAM Read Function
// History			: 
//********************************************************************************
UINT_8	RamRead402( UINT_8 uc_slv_address, UINT_8 uc_address, UINT_16 *pus_data )
{
	UINT_32	ul_data ;
	UINT_8	uc_time_out	= 0 ;

	ul_data	= ( ( UINT_32 )uc_slv_address << 24 ) | ( ( UINT_32 )uc_address << 16 ) ;

	RamWrite32A( CMD_402_SHORT_READ, ul_data ) ;

	do {
		RamRead32A( CMD_402_SHORT_READ, &ul_data ) ;
		uc_time_out++ ;
	} while( ( uc_time_out < 100 ) && ( ul_data & 0x000000FF ) ) ;

	RamRead32A( CMD_402_RX_BUFFER_READ_WRITE, &ul_data ) ;

	*pus_data	= ( UINT_16 )( ( ul_data >> 16 ) & 0x0000FFFF ) ;

	if( uc_time_out >= 100 ) {
		return( FAILURE ) ;
	} else {
		return( SUCCESS ) ;
	}
}



//********************************************************************************
// Function Name	: EEPRead402
// Retun Value		: Success or Failure
// Argment Value	: Slave Address, Address, Data
// Explanation		: LC898402 EEPROM Read Function
// History			: 
//********************************************************************************
UINT_8	EEPRead402( UINT_8 uc_slv_address, UINT_8 uc_address, UINT_8 *puc_data )
{
	UINT_32	ul_data ;
	UINT_8	uc_time_out	= 0 ;

	ul_data	= ( ( UINT_32 )uc_slv_address << 24 ) | ( ( UINT_32 )uc_address << 16 ) ;

	RamWrite32A( CMD_402_BYTE_READ, ul_data ) ;

	do {
		RamRead32A( CMD_402_BYTE_READ, &ul_data ) ;
		uc_time_out++ ;
	} while( ( uc_time_out < 100 ) && ( ul_data & 0x000000FF ) ) ;

	RamRead32A( CMD_402_RX_BUFFER_READ_WRITE, &ul_data ) ;

	*puc_data	= ( UINT_8 )( ( ul_data >> 24 ) & 0x000000FF ) ;

	if( uc_time_out >= 100 ) {
		return( FAILURE ) ;
	} else {
		return( SUCCESS ) ;
	}
}



//********************************************************************************
// Function Name	: EEP_WritePermission402
// Retun Value		: Success or Failure
// Argment Value	: Slave Address, Write Permission Selection, Release or Protection
// Explanation		: LC898402 EEPROM Write Function
// History			: 
//********************************************************************************
UINT_8	EEP_WritePermission402( UINT_8 uc_slv_address, UINT_8 uc_sel, UINT_8 uc_mode )
{
	UINT_32	ul_data ;
	UINT_8	uc_time_out	= 0, uc_result	= SUCCESS ;

	if( !uc_mode ) {
		// Write Permission 1
		ul_data	= ( ( UINT_32 )uc_slv_address << 24 ) | ( ( UINT_32 )E2WPMS1 << 16 ) | ( ( UINT_32 )PMSCODE1 << 8 ) ;

		RamWrite32A( CMD_402_BYTE_WRITE, ul_data ) ;

		uc_time_out	= 0 ;

		do {
			RamRead32A( CMD_402_BYTE_WRITE, &ul_data ) ;
			uc_time_out++ ;
		} while( ( uc_time_out < 100 ) && ( ul_data & 0x000000FF ) ) ;

		if( uc_time_out >= 100 ) {
			uc_result	= FAILURE ;
		}

		// Write Permission 2
		ul_data	= ( ( UINT_32 )uc_slv_address << 24 ) | ( ( UINT_32 )E2WPMS2 << 16 ) | ( ( UINT_32 )( PMSCODE2 | uc_sel ) << 8 ) ;

		RamWrite32A( CMD_402_BYTE_WRITE, ul_data ) ;

		uc_time_out	= 0 ;

		do {
			RamRead32A( CMD_402_BYTE_WRITE, &ul_data ) ;
			uc_time_out++ ;
		} while( ( uc_time_out < 100 ) && ( ul_data & 0x000000FF ) ) ;

		if( uc_time_out >= 100 ) {
			uc_result	= FAILURE ;
		}

		WitTim( 1 ) ;

	} else {
		// Write Permission 1 Clear
		ul_data	= ( ( UINT_32 )uc_slv_address << 24 ) | ( ( UINT_32 )E2WPMS1 << 16 ) | ( ( UINT_32 )0x00 << 8 ) ;

		RamWrite32A( CMD_402_BYTE_WRITE, ul_data ) ;

		uc_time_out	= 0 ;

		do {
			RamRead32A( CMD_402_BYTE_WRITE, &ul_data ) ;
			uc_time_out++ ;
		} while( ( uc_time_out < 100 ) && ( ul_data & 0x000000FF ) ) ;

		if( uc_time_out >= 100 ) {
			uc_result	= FAILURE ;
		}

		// Write Permission 2
		ul_data	= ( ( UINT_32 )uc_slv_address << 24 ) | ( ( UINT_32 )E2WPMS2 << 16 ) | ( ( UINT_32 )0x00 << 8 ) ;

		RamWrite32A( CMD_402_BYTE_WRITE, ul_data ) ;

		uc_time_out	= 0 ;

		do {
			RamRead32A( CMD_402_BYTE_WRITE, &ul_data ) ;
			uc_time_out++ ;
		} while( ( uc_time_out < 100 ) && ( ul_data & 0x000000FF ) ) ;

		if( uc_time_out >= 100 ) {
			uc_result	= FAILURE ;
		}
	}

	return( uc_result ) ;
}



//********************************************************************************
// Function Name	: EEPWrite402
// Retun Value		: NON
// Argment Value	: Slave Address, Address, Data, Write Permission Selection
// Explanation		: LC898402 EEPROM Write Function
// History			: 
//********************************************************************************
void	EEPWrite402( UINT_8 uc_slv_address, UINT_8 uc_address, UINT_8 uc_data )
{
	UINT_32	ul_data ;

	// EEPROM Write
	ul_data	= ( ( UINT_32 )uc_slv_address << 24 ) | ( ( UINT_32 )uc_address << 16 ) | ( ( UINT_32 )uc_data << 8 ) ;

	RamWrite32A( CMD_402_BYTE_WRITE, ul_data ) ;

	WitTim( 20 ) ;

	return ;
}



//********************************************************************************
// Function Name	: Byte_EEPWrite402
// Retun Value		: 
// Argment Value	: Slave Address, Address, Data, Write Permission Selection
// Explanation		: LC898402 EEPROM Write Function
// History			: 
//********************************************************************************
UINT_8	Byte_EEPWrite402( UINT_8 uc_slv_address, UINT_8 uc_address, UINT_8 uc_data, UINT_8 uc_sel )
{
	UINT_8	uc_result, uc_stbb_backup, uc_read_data ;

	// ANLG Backup
	uc_result	= RegRead402( uc_slv_address, ANLG, &uc_stbb_backup ) ;
	if( uc_result != SUCCESS ) {
		return( 0x01 ) ;										// Register Read Error
	}

	// Standby Setting
	uc_result	= RegWrite402( uc_slv_address, ANLG, ( uc_stbb_backup & ~STBB_402 ) ) ;
	if( uc_result != SUCCESS ) {
		return( 0x02 ) ;										// Register Write Error
	}

	WitTim( 100 ) ;

	// Write Permission
	uc_result	= EEP_WritePermission402( uc_slv_address, uc_sel, 0 ) ;
	if( uc_result != SUCCESS ) {
		return( 0x03 ) ;										// EEPROM Write Permission Error
	}

	// EEPROM Write
	EEPWrite402( uc_slv_address, uc_address, uc_data ) ;
	if( uc_result != SUCCESS ) {
		return( 0x04 ) ;										// EEPROM Write Error
	}

	// Write Protection
	uc_result	= EEP_WritePermission402( uc_slv_address, uc_sel, 1 ) ;
	if( uc_result != SUCCESS ) {
		return( 0x05 ) ;										// EEPROM Write Protection Error
	}

	// Standby Release
	uc_result	= RegWrite402( uc_slv_address, ANLG, uc_stbb_backup ) ;
	if( uc_result != SUCCESS ) {
		return( 0x06 ) ;										// Register Write Error
	}

	uc_result	= EEPRead402( uc_slv_address, uc_address, &uc_read_data ) ;
	if( uc_result != SUCCESS ) {
		return( 0x07 ) ;										// Register Read Error
	}

	if( uc_read_data == uc_data ) {
		uc_result	= SUCCESS ;
	} else {
		uc_result	= 0x08 ;									// Verify Error
	}

	return( uc_result ) ;
}



//********************************************************************************
// Function Name	: Cont_EEPRead402
// Retun Value		: Success or Failure
// Argment Value	: Slave Address, Address, Data, Data Pointer, Length
// Explanation		: LC898402 Continuous EEPROM Read Function
// History			: 
//********************************************************************************
UINT_8	Cont_EEPRead402( UINT_8 uc_slv_address, UINT_8 uc_address, UINT_8 *puc_data, UINT_8 uc_length )
{
	UINT_32	ul_data ;
	UINT_8	uc_time_out	= 0, i ;

	for( i = 0 ; i < uc_length ; i++ ) {
		ul_data	= ( ( UINT_32 )uc_slv_address << 24 ) | ( ( UINT_32 )( uc_address + i ) << 16 ) ;

		RamWrite32A( CMD_402_BYTE_READ, ul_data ) ;

		uc_time_out	= 0 ;

		do {
			RamRead32A( CMD_402_BYTE_READ, &ul_data ) ;
			uc_time_out++ ;
		} while( ( uc_time_out < 100 ) && ( ul_data & 0x000000FF ) ) ;

		RamRead32A( CMD_402_RX_BUFFER_READ_WRITE, &ul_data ) ;

		*puc_data++	= ( UINT_8 )( ( ul_data >> 24 ) & 0x000000FF ) ;

		if( uc_time_out >= 100 ) {
			return( FAILURE ) ;
		}
	}

	return( SUCCESS ) ;
}



//********************************************************************************
// Function Name	: Cont_EEPWrite402
// Retun Value		: 
// Argment Value	: Slave Address, Address, Data, Write Permission Selection, Length
// Explanation		: LC898402 Continuous EEPROM Write Function
// History			: 
//********************************************************************************
UINT_8	Cont_EEPWrite402( UINT_8 uc_slv_address, UINT_8 uc_address, UINT_8 *puc_data, UINT_8 uc_sel, UINT_8 uc_length )
{
	UINT_8	uc_result, uc_stbb_backup, i ;

	// ANLG Backup
	uc_result	= RegRead402( uc_slv_address, ANLG, &uc_stbb_backup ) ;
	if( uc_result != SUCCESS ) {
		return( 0x01 ) ;										// Register Read Error
	}

	// Standby Setting
	uc_result	= RegWrite402( uc_slv_address, ANLG, ( uc_stbb_backup & ~STBB_402 ) ) ;
	if( uc_result != SUCCESS ) {
		return( 0x02 ) ;										// Register Write Error
	}

	WitTim( 100 ) ;

	// Write Permission
	uc_result	= EEP_WritePermission402( uc_slv_address, uc_sel, 0 ) ;
	if( uc_result != SUCCESS ) {
		return( 0x03 ) ;										// EEPROM Write Permission Error
	}

	for( i = 0 ; i < uc_length ; i++ ) {
		// EEPROM Write
		EEPWrite402( uc_slv_address, uc_address + i, *puc_data++ ) ;
		if( uc_result != SUCCESS ) {
			return( 0x04 ) ;									// EEPROM Write Error
		}
	}

	// Write Protection
	uc_result	= EEP_WritePermission402( uc_slv_address, uc_sel, 1 ) ;
	if( uc_result != SUCCESS ) {
		return( 0x05 ) ;										// EEPROM Write Protection Error
	}

	// Standby Release
	uc_result	= RegWrite402( uc_slv_address, ANLG, uc_stbb_backup ) ;
	if( uc_result != SUCCESS ) {
		return( 0x06 ) ;										// Register Write Error
	}

	uc_result	= SUCCESS ;

	return( uc_result ) ;
}



//********************************************************************************
// Function Name	: EEPROM_Clear402
// Retun Value		: Success or Failure
// Argment Value	: Slave Address
// Explanation		: LC898402 EEPROM Clear Function
// History			: 
//********************************************************************************
UINT_8	EEPROM_Clear402( UINT_8 uc_slv_address )
{
	UINT_8	uc_status, i ;

	uc_status	= Byte_EEPWrite402( uc_slv_address, EP_IDSEL, 0xC0, SEL_ID ) ;
	if( uc_status != SUCCESS ) {
		return( uc_status ) ;
	}

	for( i = EP_DAHLO ; i < EP_FREE_AREA_START ; i++ ) {
		uc_status	= Byte_EEPWrite402( uc_slv_address, i, 0x00, SEL_UP ) ;
		if( uc_status != SUCCESS ) {
			return( uc_status ) ;
		}
	}

	uc_status	= Byte_EEPWrite402( uc_slv_address, EP_DAHLO, 0x80, SEL_UP ) ;
	if( uc_status != SUCCESS ) {
		return( uc_status ) ;
	}

	uc_status	= Byte_EEPWrite402( uc_slv_address, EP_DAHLB, 0x80, SEL_UP ) ;
	if( uc_status != SUCCESS ) {
		return( uc_status ) ;
	}

	uc_status	= Byte_EEPWrite402( uc_slv_address, EP_LINEAR_C1, 0x20, SEL_UP ) ;
	if( uc_status != SUCCESS ) {
		return( uc_status ) ;
	}

	uc_status	= Byte_EEPWrite402( uc_slv_address, EP_HGHLMT, 0x7F, SEL_UP ) ;
	if( uc_status != SUCCESS ) {
		return( uc_status ) ;
	}

	uc_status	= Byte_EEPWrite402( uc_slv_address, EP_LOWLMT, 0x80, SEL_UP ) ;
	if( uc_status != SUCCESS ) {
		return( uc_status ) ;
	}

	return( uc_status ) ;
}



//********************************************************************************
// Function Name	: SlaveAddress_Separate402
// Retun Value		: Success or Failure
// Argment Value	: Slave Address
// Explanation		: LC898402 EEPROM Clear Function
// History			: 
//********************************************************************************
UINT_8	SlaveAddress_Separate402( void )
{
	UINT_8	uc_status, i, uc_read_data, uc_processed_status	= 0 ;

	// Check Processed Status
	uc_status	= RegRead402( X_402, FNUM, &uc_read_data ) ;
	if( uc_status != SUCCESS ) {
		return( 0x01 ) ;									// I2C Error
	} else {
		if( uc_read_data == FNUM_DATA ) {
			uc_processed_status	= 0x01 ;
		}
	}

	uc_status	= RegRead402( Y_402, FNUM, &uc_read_data ) ;
	if( uc_status != SUCCESS ) {
		return( 0x01 ) ;									// I2C Error
	} else {
		if( uc_read_data == FNUM_DATA ) {
			uc_processed_status	|= 0x02 ;
		}
	}

	if( uc_processed_status == 0x03 ) {
		return( SUCCESS ) ;
	}

	// X Axis Slave Address Changing
	for( i = 0 ; i < 3 ; i++ ) {
		// Slave Address Check
		uc_status	= RegRead402( DEFAULT_SLV_ADDRESS_402, FNUM, &uc_read_data ) ;
		if( uc_status != SUCCESS ) {
			return( 0x01 ) ;									// I2C Error
		} else {
			if( uc_read_data != FNUM_DATA ) {
				return( 0x02 ) ;								// Slave Address Error
			}
		}

		WitTim( 10 ) ;

		// Change Slave Address of X Axis
		uc_status	= Byte_EEPWrite402( DEFAULT_SLV_ADDRESS_402, EP_IDSEL, SLV_E8, SEL_ID ) ;
		if( uc_status != SUCCESS ) {
			return( 0x03 ) ;									// EEPROM Write Error
		}

		// Change ANLG of X Axis
		uc_status	= Byte_EEPWrite402( DEFAULT_SLV_ADDRESS_402, EP_ANLG, ( STBB_402 | GSEL_24_8 | DRVINV ), SEL_UP ) ;
		if( uc_status != SUCCESS ) {
			return( 0x03 ) ;									// EEPROM Write Error
		}

		// Change FHLO of X Axis
		uc_status	= Byte_EEPWrite402( DEFAULT_SLV_ADDRESS_402, EP_FHLO, 0x18, SEL_UP ) ;
		if( uc_status != SUCCESS ) {
			return( 0x03 ) ;									// EEPROM Write Error
		}

		// EEPROM Download
		uc_status	= RegWrite402( DEFAULT_SLV_ADDRESS_402, ADWLCTL, 0x01 ) ;
		if( uc_status != SUCCESS ) {
			return( 0x04 ) ;									// Register Write Error
		}

		WitTim( 10 ) ;

		// Slave Address of Y Axis Check
		uc_status	= RegRead402( Y_402, FNUM, &uc_read_data ) ;
		if( uc_status != SUCCESS ) {
			return( 0x05 ) ;									// Register Read Error
		} else {
			if( uc_read_data == FNUM_DATA ) {
				break ;
			}
		}
	}

	if( i == 3 ) {
		return( 0x06 ) ;										// X Axis Slave Address Changing Error
	}

	// Change Temporary I2C Reverse Connection Mode
	for( i = 0 ; i < 8 ; i++ ) {
		RegWrite402( MAGIC_CODE_SLV_ADDRESS_402, 0xFF, 0x00 ) ;
	}

	// Dummy Read
	for( i = 0 ; i < 10 ; i++ ) {
		RegRead402( DEFAULT_SLV_ADDRESS_402, FNUM, &uc_read_data ) ;
		if( uc_read_data == FNUM_DATA ) {
			break ;
		}
	}

	// Y Axis Slave Address Changing
	for( i = 0 ; i < 3 ; i++ ) {
		// Slave Address Check
		uc_status	= RegRead402( DEFAULT_SLV_ADDRESS_402, FNUM, &uc_read_data ) ;
		if( uc_status != SUCCESS ) {
			return( 0x07 ) ;									// I2C Error
		} else {
			if( uc_read_data != FNUM_DATA ) {
				return( 0x08 ) ;								// Slave Address Error
			}
		}

		WitTim( 10 ) ;

		// Change I2C Reverse Connection Mode of Y Axis
		uc_status	= Byte_EEPWrite402( DEFAULT_SLV_ADDRESS_402, EP_IDSEL, SLV_E4_REVERSE, SEL_ID ) ;
		if( uc_status != SUCCESS ) {
			return( 0x09 ) ;									// EEPROM Write Error
		}

		// Change ANLG of Y Axis
		uc_status	= Byte_EEPWrite402( DEFAULT_SLV_ADDRESS_402, EP_ANLG, ( STBB_402 | GSEL_24_8 | DRVINV ), SEL_UP ) ;
		if( uc_status != SUCCESS ) {
			return( 0x03 ) ;									// EEPROM Write Error
		}

		// Change FHLO of Y Axis
		uc_status	= Byte_EEPWrite402( DEFAULT_SLV_ADDRESS_402, EP_FHLO, 0x18, SEL_UP ) ;
		if( uc_status != SUCCESS ) {
			return( 0x03 ) ;									// EEPROM Write Error
		}

		// EEPROM Download
		uc_status	= RegWrite402( DEFAULT_SLV_ADDRESS_402, ADWLCTL, 0x01 ) ;
		if( uc_status != SUCCESS ) {
			return( 0x0A ) ;									// Register Write Error
		}

		WitTim( 10 ) ;

		// Slave Address of X Axis Check
		uc_status	= RegRead402( X_402, FNUM, &uc_read_data ) ;
		if( uc_status != SUCCESS ) {
			return( 0x0B ) ;									// Register Read Error
		} else {
			if( uc_read_data == FNUM_DATA ) {
				break ;
			}
		}
	}

	if( i == 3 ) {
		return( 0x0C ) ;										// Y Axis Slave Address Changing Error
	}

	return( SUCCESS ) ;
}



//********************************************************************************
// Function Name	: Standby_402
// Retun Value		: Success or Failure
// Argment Value	: Standby or Reset
// Explanation		: LC898402 Continuous EEPROM Read Function
// History			: 
//********************************************************************************
UINT_8	Standby_402( UINT_8 uc_mode )
{
	UINT_8	uc_time_out	= 0, uc_read_data ;

	if( !uc_mode ) {										// Standby Setting
		RegWrite402( X_402, STANDBY_402, 0x80 ) ;			// X Standby

		do {
			WitTim( 1 ) ;

			RegRead402( X_402, ANLG, &uc_read_data ) ;
			uc_time_out++ ;
		} while( ( uc_read_data & STBB_402 ) && ( uc_time_out < 10 ) ) ;

		if( uc_time_out >= 10 ) {
			return( FAILURE ) ;
		}

		uc_time_out	= 0 ;
		RegWrite402( Y_402, STANDBY_402, 0x80 ) ;			// Y Standby

		do {
			WitTim( 1 ) ;

			RegRead402( Y_402, ANLG, &uc_read_data ) ;
			uc_time_out++ ;
		} while( ( uc_read_data & STBB_402 ) && ( uc_time_out < 10 ) ) ;

		if( uc_time_out >= 10 ) {
			return( FAILURE ) ;
		}
	} else {												// All Reset
		RegWrite402( X_402, STANDBY_402, 0x01 ) ;

		WitTim( 5 ) ;

		RegWrite402( X_402, ADWLCTL, 0x01 ) ;

		do {
			WitTim( 1 ) ;

			RegRead402( X_402, ADWLCTL, &uc_read_data ) ;
			uc_time_out++ ;
		} while( ( uc_read_data & 0x01 ) && ( uc_time_out < 10 ) ) ;

		if( uc_time_out >= 10 ) {
			return( FAILURE ) ;
		}

		uc_time_out	= 0 ;
		RegWrite402( Y_402, STANDBY_402, 0x01 ) ;

		WitTim( 5 ) ;

		RegWrite402( Y_402, ADWLCTL, 0x01 ) ;

		do {
			WitTim( 1 ) ;

			RegRead402( Y_402, ADWLCTL, &uc_read_data ) ;
			uc_time_out++ ;
		} while( ( uc_read_data & 0x01 ) && ( uc_time_out < 10 ) ) ;

		if( uc_time_out >= 10 ) {
			return( FAILURE ) ;
		}
	}

	return( SUCCESS ) ;
}


