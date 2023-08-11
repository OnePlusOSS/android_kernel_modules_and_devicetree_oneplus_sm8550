/**
 * @brief		OIS system header for LC898129
 * 				API list for customers
 *
 * @author		(C) 2019 ON Semiconductor.
 *
 * @file		OisAPI.h
 * @date		svn:$Date:: 2021-01-28 22:03:14 +0900#$
 * @version		svn:$Revision: 9 $
 * @attention
 **/
#ifndef OISAPI_H_
#define OISAPI_H_
#include	"MeasurementLibrary.h"

#include	"Ois.h"

//****************************************************
//	extern selector for API
//****************************************************
#ifdef	__OISCMD__
	#define	__OIS_CMD_HEADER__
#else
	#define	__OIS_CMD_HEADER__		extern
#endif

#ifdef	__OISFLSH__
	#define	__OIS_FLSH_HEADER__
#else
	#define	__OIS_FLSH_HEADER__		extern
#endif

#ifdef	__FLSPGM__
	#define	__OIS_FLPG_HEADER__
#else
	#define	__OIS_FLPG_HEADER__		extern
#endif

//****************************************************
//	MODE SELECTORS (Compile Switches)
//****************************************************
#define			__OIS_MODULE_CALIBRATION__		//!< for module maker to done the calibration.
//#define 		__CRC_VERIFY__					//!< select CRC16 for upload verify, if this comment out, MD5 is selected.
//#define		__OIS_BIG_ENDIAN__				//!< endian of MPU

//****************************************************
//	STRUCTURE DEFINE
//****************************************************
typedef struct {
	UINT_16				Index;
	const UINT_8*		UpdataCode;
	UINT_32				SizeUpdataCode;
	UINT_64				SizeUpdataCodeCksm;
	const UINT_8*		FromCode;
	UINT_32				SizeFromCode;
	UINT_64				SizeFromCodeCksm;
	UINT_32				SizeFromCodeValid;
}	CODE_TBL_EXT;

typedef struct {
	UINT_8 Vendor;		// 0x8000
	UINT_8 User;
	UINT_8 Model;
	UINT_8 Version;

	UINT_8 Reserve0;	// 0x8004
	UINT_8 Reserve1;
	UINT_8 ActVersion;
	UINT_8 GyroType;

} DSPVER;

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
} stMesRam ;									// Struct Measure Ram

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

typedef struct STAFLINEARITY {
	INT_16	SsNum ;
	double *pDblArrayDistance ;
	INT_32	SlCoefA ;
	INT_32	SlCoefB ;
	INT_32	SlCoefC ;
	INT_32	SlCoefD ;
} stAfLinearity ;

typedef struct STAF_OIS_CROSSTALK {
	short	SsNum ;
	long *	pSlArrayOisHallX ;
	long *	pSlArrayOisHallY ;
//	double *pDblArrayDistanceX ;
//	double *pDblArrayDistanceY ;
	double *pDblArrayDistanceXY ;
//	long	SlCoefXA ;
//	long	SlCoefXB ;
//	long	SlCoefXC ;
//	long	SlCoefYA ;
//	long	SlCoefYB ;
//	long	SlCoefYC ;
	long	SlCoefA ;
	long	SlCoefB ;
	long	SlCoefC ;
	long	SlCoefD ;
	long	SlCoefE ;
} stAfOisCrosstalk ;

typedef struct {
	double XonXmove[7];
	double YonXmove[7];
	double XonYmove[7];
	double YonYmove[7];
} stPixelCoordinate;

//****************************************************
//	API LIST
//****************************************************
/* Status Read and OIS enable [mandatory] */
__OIS_CMD_HEADER__	UINT_8	RdStatus( UINT_8 ) ;						//!< Status Read whether initialization finish or not.
__OIS_CMD_HEADER__	void	OisEna( void ) ;						//!< OIS Enable function
__OIS_CMD_HEADER__	void	OisEna_LS( void ) ;						//!< OIS Enable function
__OIS_CMD_HEADER__	void	OisDis( void ) ;						//!< OIS Disable function
__OIS_CMD_HEADER__	void	OisDis_LS( void ) ;						//!< OIS Disable function

/* Others [option] */
__OIS_CMD_HEADER__	UINT_8	RtnCen( UINT_8 ) ;						//!< Return to center function. Hall servo on/off
__OIS_CMD_HEADER__	UINT_8	RtnCen_LS( UINT_8 ) ;					//!< Return to center function. Hall servo on/off
__OIS_CMD_HEADER__	UINT_8	RtnCenBit( UINT_32 ) ;					//!< Return to center function. Hall servo on/off by bitmap
__OIS_CMD_HEADER__	void	OisEnaNCL( void ) ;						//!< OIS Enable function w/o delay clear
__OIS_CMD_HEADER__	void	OisEnaNCL_LS( void ) ;					//!< OIS Enable function w/o delay clear
__OIS_CMD_HEADER__	void	OisEnaDrCl( void ) ;					//!< OIS Enable function force drift cancel
__OIS_CMD_HEADER__	void	OisEnaDrNcl( void ) ;					//!< OIS Enable function w/o delay clear and force drift cancel
__OIS_CMD_HEADER__	void	SetRec( void ) ;						//!< Change to recording mode function
__OIS_CMD_HEADER__	void	SetRec_LS( void ) ;						//!< Change to recording mode function
__OIS_CMD_HEADER__	void	SetStill( void ) ;						//!< Change to still mode function
__OIS_CMD_HEADER__	void	SetStill_LS( void ) ;					//!< Change to still mode function

__OIS_CMD_HEADER__	void	SetPanTiltMode( UINT_8 ) ;				//!< Pan/Tilt control (default ON)
__OIS_CMD_HEADER__	void	SetPanTiltMode_LS( UINT_8 ) ;			//!< Pan/Tilt control (default ON)
//__OIS_CMD_HEADER__	void	RdHallCalData( void ) ;					//!< Read Hall Calibration Data in Data Ram

__OIS_CMD_HEADER__	void	OscStb( void );							//!< Standby the oscillator
//__OIS_CMD_HEADER__	UINT_8	GyroReCalib( stReCalib * ) ;			//!< Gyro offset re-calibration
__OIS_CMD_HEADER__	UINT_32	ReadCalibID( void ) ;					//!< Read calibration ID
//__OIS_CMD_HEADER__	UINT_16	GyrSlf( void ) ;						//!< Gyro self test

__OIS_CMD_HEADER__	UINT_8	GyrWhoAmIRead( void ) ;					//!< Gyro Who am I Read
__OIS_CMD_HEADER__	UINT_8	GyrWhoAmICheck( void ) ;				//!< Gyro Who am I Check
__OIS_CMD_HEADER__	UINT_8	GyrIdRead( UINT_8 * ) ;					//!< Gyro ID Read

__OIS_CMD_HEADER__	UINT_8	MesRam( INT_32 , INT_32 , INT_32 , stMesRam* , stMesRam* );

__OIS_CMD_HEADER__	void	GyrRegWriteA(UINT_8, UINT_8);
__OIS_CMD_HEADER__	void	GyrRegReadA(UINT_8, UINT_8 *);

#ifdef	__OIS_MODULE_CALIBRATION__

 /* Calibration Main [mandatory] */
 __OIS_CMD_HEADER__	UINT_32	TneRun( void );							//!< calibration
 __OIS_CMD_HEADER__	UINT_32	TneRun_LS( void );						//!< calibration

 __OIS_CMD_HEADER__	UINT_8	TneSltPos( UINT_8 ) ;					//!< for NVC
 __OIS_CMD_HEADER__	UINT_8	TneVrtPos( UINT_8 ) ;					//!< for CROSS TALK
 __OIS_CMD_HEADER__ UINT_8	TneHrzPos( UINT_8 ) ;					//!< for CROSS TALK
 __OIS_CMD_HEADER__	UINT_8	TneVrtPos_LS( UINT_8 ) ;					//!< for CROSS TALK
 __OIS_CMD_HEADER__ UINT_8	TneHrzPos_LS( UINT_8 ) ;					//!< for CROSS TALK
 __OIS_CMD_HEADER__ UINT_32	MeasGyAcOffset( void ) ;				//!< calibration gyro offset
 __OIS_CMD_HEADER__ UINT_8	getTneHrzPos( INT_32 * ) ;				//!< for CROSS TALK
 __OIS_CMD_HEADER__ UINT_8	getTneVrtPos( INT_32 * ) ;				//!< for CROSS TALK
 __OIS_CMD_HEADER__ UINT_8	getTneHrzPos_LS( INT_32 * ) ;				//!< for CROSS TALK
 __OIS_CMD_HEADER__ UINT_8	getTneVrtPos_LS( INT_32 * ) ;				//!< for CROSS TALK
 __OIS_CMD_HEADER__ void	TurnOffOISLinearity( void ) ;
 __OIS_CMD_HEADER__ void	TurnOffOISLinearity_LS( void ) ;

 __OIS_CMD_HEADER__	UINT_8	FrqDet( void ) ;						//!< oscillation detect
 __OIS_CMD_HEADER__	UINT_8	FrqDetReTry( void ) ;					//!< oscillation detect
 __OIS_CMD_HEADER__	UINT_8	FrqDetWGyr( UINT_8 ) ;					//!< oscillation detect

 __OIS_CMD_HEADER__ void	RdGyroOffsetTbl( stGyroOffsetTbl * ) ;
 __OIS_CMD_HEADER__	UINT_8	GetInfomation( DSPVER * ) ;

 __OIS_CMD_HEADER__ void SetSlaveAddr(UINT_8 SlaveAddr);
 __OIS_CMD_HEADER__ UINT_8 GetSlaveAddr(void);
 __OIS_CMD_HEADER__ UINT_8 DrvOffAdj( UINT_8 ucSlaveAdr );
 __OIS_CMD_HEADER__ UINT_8	RegWrite402( UINT_8 uc_slv_address, UINT_8 uc_address, UINT_8 uc_data ) ;
 __OIS_CMD_HEADER__ UINT_8	RegRead402( UINT_8 uc_slv_address, UINT_8 uc_address, UINT_8 *puc_data ) ;
 __OIS_CMD_HEADER__ UINT_8	RamWrite402( UINT_8 uc_slv_address, UINT_8 uc_address, UINT_16 us_data ) ;
 __OIS_CMD_HEADER__ UINT_8	RamRead402( UINT_8 uc_slv_address, UINT_8 uc_address, UINT_16 *pus_data ) ;
 __OIS_CMD_HEADER__ UINT_8	EEPRead402( UINT_8 uc_slv_address, UINT_8 uc_address, UINT_8 *puc_data ) ;
 __OIS_CMD_HEADER__ UINT_8	Byte_EEPWrite402( UINT_8 uc_slv_address, UINT_8 uc_address, UINT_8 uc_data, UINT_8 uc_sel ) ;
 __OIS_CMD_HEADER__ UINT_8	Cont_EEPRead402( UINT_8 uc_slv_address, UINT_8 uc_address, UINT_8 *puc_data, UINT_8 uc_length ) ;
 __OIS_CMD_HEADER__ UINT_8	Cont_EEPWrite402( UINT_8 uc_slv_address, UINT_8 uc_address, UINT_8 *puc_data, UINT_8 uc_sel, UINT_8 uc_length ) ;
 __OIS_CMD_HEADER__ UINT_8	EEPROM_Clear402( UINT_8 uc_slv_address ) ;
 __OIS_CMD_HEADER__ UINT_8	SlaveAddress_Separate402( void ) ;
 __OIS_CMD_HEADER__ UINT_8	Standby_402( UINT_8 uc_mode ) ;

 /* Flash Update */
 __OIS_FLPG_HEADER__	UINT_8	FlashProgram129( UINT_8, UINT_8, UINT_8 );
 __OIS_FLPG_HEADER__	UINT_8	PmemDownload129( UINT_8, UINT_8 );
 __OIS_FLPG_HEADER__	UINT_8	LoadUserAreaToPM( void  );

 __OIS_FLSH_HEADER__	UINT_8	FlashBlockErase( UINT_8, UINT_32 ) ;
 __OIS_FLSH_HEADER__	UINT_8	FlashSingleRead( UINT_8, UINT_32, UINT_32 * ) ;
 __OIS_FLSH_HEADER__	UINT_8	FlashMultiRead( UINT_8, UINT_32, UINT_32 *, UINT_8 ) ;
 __OIS_FLSH_HEADER__	UINT_8	FlashPageWrite( UINT_8, UINT_32, UINT_32 * ) ;
 __OIS_FLSH_HEADER__	UINT_8	EraseInfoMat129( UINT_8 ) ;

 __OIS_FLSH_HEADER__	void	BootMode( void ) ;						//!< Change to boot mode

 __OIS_FLSH_HEADER__	UINT_8	WrHallCalData( UINT_8 ) ;				//!< upload the calibration data except gyro gain to Flash
 __OIS_FLSH_HEADER__	UINT_8	WrHallCalData_LS( void ) ;				//!< upload the calibration data except gyro gain to Flash

 __OIS_FLSH_HEADER__	UINT_8	WrGyroGainData( UINT_8 ) ;				//!< upload the gyro gain to Flash
 __OIS_FLSH_HEADER__	UINT_8	WrGyroGainData_LS( UINT_8 ) ;			//!< upload the gyro gain to Flash
 __OIS_FLSH_HEADER__	UINT_8	WrGyAcOffsetData( UINT_8 ) ;
// __OIS_FLSH_HEADER__	UINT_8	WrOptCenerData( UINT_8 ) ;
 __OIS_FLSH_HEADER__	UINT_8	WrGyroOffsetTbl( stGyroOffsetTbl * ) ;
 __OIS_FLSH_HEADER__	UINT_8	WrWireBias( UINT_8 ) ;					//!< wire bias to Flash

 __OIS_FLSH_HEADER__	UINT_8	WrLinCalData( UINT_8, mlLinearityValue * ) ;
 __OIS_FLSH_HEADER__	UINT_8	WrLinMixCalData( UINT_8, mlMixingValue *, mlLinearityValue * ) ;
 __OIS_FLSH_HEADER__	UINT_8	WrLinMixCalData_LS( UINT_8, mlMixingValue *, mlLinearityValue * ) ;
 __OIS_FLSH_HEADER__	UINT_8	ErCalData( UINT_16 ) ;

 __OIS_FLSH_HEADER__	UINT_8	CalcSetLinMixData( UINT_8, stPixelCoordinate *, UINT_8 ) ;

 __OIS_FLSH_HEADER__	UINT_8	LoadUareaToPM( CODE_TBL_EXT *, UINT_8 ) ;
 __OIS_FLSH_HEADER__	UINT_8	RdBurstUareaFromPM( UINT_32, UINT_8 *, UINT_16, UINT_8 ) ;
 __OIS_FLSH_HEADER__	UINT_8	RdSingleUareaFromPM( UINT_32, UINT_8 *, UINT_8, UINT_8 ) ;
 __OIS_FLSH_HEADER__	UINT_8	WrUareaToPm( UINT_32, UINT_8 *, UINT_8, UINT_8 ) ;
 __OIS_FLSH_HEADER__	UINT_8	WrUareaToPmInt( UINT_32, UINT_32 *, UINT_8, UINT_8 ) ;
 __OIS_FLSH_HEADER__	UINT_8	WrUareaToFlash( void ) ;

 extern void TimPro( void );

#endif	// __OIS_MODULE_CALIBRATION__

#endif /* #ifndef OISAPI_H_ */
