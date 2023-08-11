/**\
 * @brief		OIS system flash access code for LC898129
 *
 * @author		(C) 2019 ON Semiconductor.
 *
 * @file		FlsCmd.c
 * @date		svn:$Date:: 2021-02-01 19:30:40 +0900#$
 * @version		svn:$Revision: 11 $
 * @attention
 **/
#define		__OISFLSH__
//**************************
//	Include Header File
//**************************
//#include 	<stdlib.h>
//#include	<math.h>
#include "Ois.h"
#include <linux/kernel.h>
#include <linux/slab.h>

//#define deg_to_rad(deg) (((deg)/360.0)*2.0*M_PI)
//#define rad_to_deg(rad) (((rad)/2.0/M_PI)*360.0)

/* Actuator calibration parameters */
extern ADJ_LINEARITY_MIXING SMA_LinMixParameter;
extern ADJ_LINEARITY_MIXING LS_LinMixParameter;

#define	FLASH_BLOCKS			14
#define	USER_RESERVE			2		// Reserved for customer data blocks
#define	ERASE_BLOCKS			(FLASH_BLOCKS - USER_RESERVE)

/* Burst Length for updating to PMEM Max:256*/
#define BURST_LENGTH_UC 		( 120 )		// 120 Total:122Byte
/* Burst Length for updating to Flash */
#define BURST_LENGTH_FC 		( 64 )	 	// 64 Total: 66~67Byte
/* Pmem initialized flag */
#define	PMEM_INITIALIZED		(0x706D656D)

//****************************************************
//	CUSTOMER NECESSARY CREATING FUNCTION LIST
//****************************************************
/* for I2C communication */
extern	UINT_8 RamWrite32A( UINT_16, UINT_32 );
extern 	UINT_8 RamRead32A( UINT_16, void * );
/* for I2C Multi Translation : Burst Mode*/
extern 	UINT_8 CntWrt( void *, UINT_16) ;
extern	UINT_8 CntRd( UINT_16, void *, UINT_16 ) ;

/* for Wait timer [Need to adjust for your system] */
extern void	WitTim( UINT_16 );
/* for Set I2C slave address */
extern void setI2cSlvAddr( UINT_8, UINT_8 );

//**************************
//	Local Function Prototype
//**************************

//********************************************************************************
// Function Name 	: IOWrite32A
// Retun Value		: None
// Argment Value	: IOadrs, IOdata
// Explanation		: Write data to IO area Command
//********************************************************************************
UINT_8 IORead32A( UINT_32 IOadrs, UINT_32 *IOdata )
{
	UINT_8 retval ;

	retval = RamWrite32A( CMD_IO_ADR_ACCESS, IOadrs ) ;
	if( !retval )
		retval = RamRead32A ( CMD_IO_DAT_ACCESS, IOdata ) ;
	return( retval );
}

//********************************************************************************
// Function Name 	: IOWrite32A
// Retun Value		: None
// Argment Value	: IOadrs, IOdata
// Explanation		: Write data to IO area Command
//********************************************************************************
UINT_8 IOWrite32A( UINT_32 IOadrs, UINT_32 IOdata )
{
	UINT_8 retval ;

	retval = RamWrite32A( CMD_IO_ADR_ACCESS, IOadrs ) ;
	if( !retval )
		retval = RamWrite32A( CMD_IO_DAT_ACCESS, IOdata ) ;
	return( retval );
}

//********************************************************************************
// Function Name 	: BootMode
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: 
//********************************************************************************
void BootMode( void )
{
	UINT_32	ReadVal;

	IORead32A( SYSDSP_REMAP, &ReadVal ) ;
	ReadVal = (ReadVal & 0x1) | 0x00001400;
	IOWrite32A( SYSDSP_REMAP, ReadVal ) ;							// CORE_RST[12], MC_IGNORE2[10] = 1
	WitTim( 15 );													// Wait 15ms

	CAM_ERR(CAM_OIS, "Enter boot mode \n");
}

//********************************************************************************
// Function Name 	: UnlockCodeSet
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: <Flash Memory> Unlock Code Set
// History			: First edition
//********************************************************************************
UINT_8 UnlockCodeSet( void )
{
	UINT_32 UlReadVal, UlCnt=0;

	do {
		IOWrite32A( 0xE07554, 0xAAAAAAAA );							// UNLK_CODE1(E0_7554h) = AAAA_AAAAh
		IOWrite32A( 0xE07AA8, 0x55555555 );							// UNLK_CODE2(E0_7AA8h) = 5555_5555h
		IORead32A( 0xE07014, &UlReadVal );
		if( (UlReadVal & 0x00000080) != 0 )	return ( 0 ) ;			// Check UNLOCK(E0_7014h[7]) ?
		WitTim( 1 );
	} while( UlCnt++ < 10 );
	return ( 1 );
}

//********************************************************************************
// Function Name 	: UnlockCodeClear
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: <Flash Memory> Clear Unlock Code
// History			: First edition
//********************************************************************************
UINT_8 UnlockCodeClear(void)
{
	UINT_32 UlDataVal, UlCnt=0;

	do {
		IOWrite32A( 0xE07014, 0x00000010 );							// UNLK_CODE3(E0_7014h[4]) = 1
		IORead32A( 0xE07014, &UlDataVal );
		if( (UlDataVal & 0x00000080) == 0 )	return ( 0 ) ;			// Check UNLOCK(E0_7014h[7]) ?
		WitTim( 1 );
	} while( UlCnt++ < 10 );
	return ( 3 );
}


//********************************************************************************
// Function Name 	: WritePermission
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: LC898129 Command
// History			: First edition 						2018.05.15
//********************************************************************************
void WritePermission( void )
{
	IOWrite32A( 0xE074CC, 0x00000001 );								// RSTB_FLA_WR(E0_74CCh[0])=1
	IOWrite32A( 0xE07664, 0x00000010 );								// FLA_WR_ON(E0_7664h[4])=1
}

//********************************************************************************
// Function Name 	: AdditionalUnlockCodeSet
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: LC898129 Command
// History			: First edition 						2018.05.15
//********************************************************************************
void AdditionalUnlockCodeSet( void )
{
	IOWrite32A( 0xE07CCC, 0x0000ACD5 );								// UNLK_CODE3(E0_7CCCh) = 0000_ACD5h
}

//********************************************************************************
// Function Name 	: PmemUpdate129
// Retun Value		: 0:Non error 
// Argment Value	: dist, ptr
// Explanation		: Program code Update to PMEM directly
// History			: First edition
//********************************************************************************
UINT_8 PmemUpdate129( UINT_8 dist, CODE_TBL_EXT * ptr )
{
	UINT_8	data[BURST_LENGTH_UC +2 ];
	UINT_16	Remainder;
	UINT_8	ReadData[8];
	const UINT_8 *NcDataVal;
	UINT_16 SizeofCode;
	UINT_16 SizeofCheck;
	long long CheckSumCode;
	UINT_8 *p;
	UINT_32 i, j;
	UINT_32	UlReadVal, UlCnt , UlNum ;

	if( dist != 0 ) {
		// From code for SRAM
		NcDataVal = ptr->FromCode + 32;
		SizeofCode = (UINT_16)ptr->FromCode[9] << 8 | (UINT_16)ptr->FromCode[8];

		CheckSumCode = (long long)ptr->FromCode[19] << 56 | (long long)ptr->FromCode[18] << 48 | (long long)ptr->FromCode[17] << 40 | (long long)ptr->FromCode[16] << 32
					 | (long long)ptr->FromCode[15] << 24 | (long long)ptr->FromCode[14] << 16 | (long long)ptr->FromCode[13] << 8 | (long long)ptr->FromCode[12];

		SizeofCheck = SizeofCode;
	} else {
		// Update code
		NcDataVal = ptr->UpdataCode;
		SizeofCode = ptr->SizeUpdataCode;
		CheckSumCode = ptr->SizeUpdataCodeCksm;
		SizeofCheck = SizeofCode;
	}
	p = (UINT_8 *)&CheckSumCode;
	
//--------------------------------------------------------------------------------
// 1. Write update code to Pmem 
//--------------------------------------------------------------------------------
	RamWrite32A( 0x3000, 0x00080000 );		// Pmem address set

	// Pmem data burst write
	data[0] = 0x40; // CmdH
	data[1] = 0x00; // CmdL

	// Transfer each BURST_LENGTH_UC bytes
	Remainder = ( (SizeofCode * 5) / BURST_LENGTH_UC ); 
	for(i=0 ; i< Remainder ; i++)
	{
		UlNum = 2;
		for(j=0 ; j < BURST_LENGTH_UC; j++){
			data[UlNum] =  *NcDataVal++;
			UlNum++;
		}
		
		CntWrt( data, BURST_LENGTH_UC + 2 );  // Cmd 2Byte.
	}
	Remainder = ( (SizeofCode * 5) % BURST_LENGTH_UC); 
	if (Remainder != 0 )
	{
		UlNum = 2;
		for(j=0 ; j < Remainder; j++){
			data[UlNum++] = *NcDataVal++;
		}
		CntWrt( data, Remainder + 2 );  		// Cmd 2Byte
	}

//--------------------------------------------------------------------------------
// 2. Verify
//--------------------------------------------------------------------------------

	// Checksum command
	data[0] = 0xF0;											//CmdID
	data[1] = 0x0E;											//CmdID
	data[2] = (unsigned char)((SizeofCheck >> 8) & 0x000000FF);
	data[3] = (unsigned char)(SizeofCheck & 0x000000FF);
	data[4] = 0x00;
	data[5] = 0x00;

	CntWrt( data, 6 ) ;

	// Judgment of end
	UlCnt = 0;
	do{
		WitTim( 1 );
		if( UlCnt++ > 10 ) {
			return (0x21) ;									// timeout error
		}
		RamRead32A( 0x0088, &UlReadVal );
	}while ( UlReadVal != 0 );

	CntRd( 0xF00E, ReadData , 8 );
	
	CAM_ERR(CAM_OIS, "[2] define Checksum %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",p[7],p[6],p[5],p[4],p[3],p[2],p[1],p[0]);

	// Judgment checksum value
	for( i=0; i<8; i++) {
		if(ReadData[7-i] != *p++ ) {
CAM_ERR(CAM_OIS, "[2] PMEM verify NG\n");
CAM_ERR(CAM_OIS, "[2] PMEM verify Error %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",ReadData[0],ReadData[1],ReadData[2],ReadData[3],ReadData[4],ReadData[5],ReadData[6],ReadData[7]);
			return (0x22) ;					// verify ng
		}
	}
CAM_ERR(CAM_OIS, "[2] PMEM verify End\n");
CAM_ERR(CAM_OIS, "[2] PMEM verify Success %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",ReadData[0],ReadData[1],ReadData[2],ReadData[3],ReadData[4],ReadData[5],ReadData[6],ReadData[7]);
	// Remap
	if( dist != 0 ){
		RamWrite32A( 0xF001, 0 );
	}

	return( 0 );
}


//********************************************************************************
// Function Name 	: EraseInfoMat129
// Retun Value		: NON
// Argment Value	: SelMat
// Explanation		: Info Mat All Erase
//********************************************************************************
UINT_8 EraseInfoMat129( UINT_8 SelMat )
{
	UINT_8 ans = 0;

	// fail safe
	// reject illegal mat
	if( SelMat != INF_MAT0 && SelMat != INF_MAT1 && SelMat != INF_MAT2  )	return 10;
	// reject command if over range

	IOWrite32A( 0xE0701C , 0x00000000);
	ans = FlashBlockErase( SelMat, 0 ) ;
	IOWrite32A( 0xE0701C , 0x00000002);
	return( ans );
}

//********************************************************************************
// Function Name 	: EraseUserMat129
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: User Mat All Erase
//********************************************************************************
UINT_8 EraseUserMat129( UINT_8 StartBlock, UINT_8 EndBlock )
{
	UINT_32 i;
	UINT_32	UlReadVal, UlCnt ;

	IOWrite32A( 0xE0701C , 0x00000000);
	RamWrite32A( 0xF007, 0x00000000 );					// FlashAccess Setup

	//***** Erase block of user mat *****
	for( i=StartBlock ; i<EndBlock ; i++) {
		RamWrite32A( 0xF00A, ( i << 10 ) );
		RamWrite32A( 0xF00C, 0x00000020 );

		// Judgment erase
		WitTim( 5 );
		UlCnt = 0;
		do{
			WitTim( 5 );
			if( UlCnt++ > 100 ){
				IOWrite32A( 0xE0701C , 0x00000002);
				return (0x31) ;				// block erase timeout error
			}
			RamRead32A( 0xF00C, &UlReadVal );
		}while ( UlReadVal != 0 );
	}
	IOWrite32A( 0xE0701C , 0x00000002);
	return(0);

}

//********************************************************************************
// Function Name 	: ProgramFlash129_Standard
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: User Mat All Erase
//********************************************************************************
UINT_8 ProgramFlash129_Standard( CODE_TBL_EXT* ptr )
{
	UINT_32	UlReadVal, UlCnt , UlNum ;
	UINT_8	data[(BURST_LENGTH_FC + 3)];
	UINT_32 i, j;

	const UINT_8 *NcFromVal = ptr->FromCode + 64;
	const UINT_8 *NcFromVal1st = ptr->FromCode;
	UINT_8 UcOddEvn = 0;

	IOWrite32A( 0xE0701C , 0x00000000);
	RamWrite32A( 0xF007, 0x00000000 );
	RamWrite32A( 0xF00A, 0x00000010 );
	data[0] = 0xF0;						// CmdID
	data[1] = 0x08;						// CmdID
	data[2] = 0x00;

	for(i=1 ; i< ( ptr->SizeFromCode / 64 ) ; i++)
	{
		if( ++UcOddEvn >1 )  	UcOddEvn = 0;	// flag of even/odd
		if (UcOddEvn == 0) data[1] = 0x08;
		else 			   data[1] = 0x09;		
CAM_ERR(CAM_OIS, "[%d]UcOddEvn= %d , data[1]= %d \n", i, data[1], NcFromVal );

#if (BURST_LENGTH_FC == 32)
		data[2] = 0x00;
		UlNum = 3;
		for(j=0 ; j < BURST_LENGTH_FC; j++){
			data[UlNum++] = *NcFromVal++;
		}
		CntWrt( data, BURST_LENGTH_FC+3 );  // Cmd 3Byte.
	  	data[2] = 0x20;		//+32Byte
		UlNum = 3;
		for(j=0 ; j < BURST_LENGTH_FC; j++){
			data[UlNum++] = *NcFromVal++;
		}
		CntWrt( data, BURST_LENGTH_FC+3 );  // Cmd 3Byte.
#elif (BURST_LENGTH_FC == 64)
		UlNum = 3;
		for(j=0 ; j < BURST_LENGTH_FC; j++){
			data[UlNum++] = *NcFromVal++;
		}
		CntWrt( data, BURST_LENGTH_FC+3 );  // Cmd 3Byte.
#endif

		RamWrite32A( 0xF00B, 0x00000010 );
		UlCnt = 0;
		if (UcOddEvn == 0){
			do{
				WitTim( 1 );
				RamRead32A( 0xF00C, &UlReadVal );
				if( UlCnt++ > 250 ) {
					IOWrite32A( 0xE0701C , 0x00000002);
					return (0x41) ;				// write timeout error
				}
			}while ( UlReadVal != 0 );
		 	RamWrite32A( 0xF00C, 0x00000004 );
		}else{
			do{
				WitTim( 1 );
				RamRead32A( 0xF00C, &UlReadVal );
				if( UlCnt++ > 250 ) {
					IOWrite32A( 0xE0701C , 0x00000002);
					return (0x41) ;				// write timeout error
				}
			}while ( UlReadVal != 0 );
			RamWrite32A( 0xF00C, 0x00000008 );
		}
	}

	UlCnt = 0;
	do{
		WitTim( 1 );
		RamRead32A( 0xF00C, &UlReadVal );
		if( UlCnt++ > 250 ) {
			IOWrite32A( 0xE0701C , 0x00000002);
			return (0x41) ;				// write timeout error
		}
	}while ( (UlReadVal & 0x0000000C) != 0 );

	{	/* write magic code */
		RamWrite32A( 0xF00A, 0x00000000 );
		data[1] = 0x08;
CAM_ERR(CAM_OIS, "[%d]UcOddEvn= %d , data[1]= %d \n", 0, data[1], NcFromVal1st );

#if (BURST_LENGTH_FC == 32)
		data[2] = 0x00;
		UlNum = 3;
		for(j=0 ; j < BURST_LENGTH_FC; j++){
			data[UlNum++] = *NcFromVal1st++;
		}
		CntWrt( data, BURST_LENGTH_FC+3 );  // Cmd 3Byte.
	  	data[2] = 0x20;		//+32Byte
		UlNum = 3;
		for(j=0 ; j < BURST_LENGTH_FC; j++){
			data[UlNum++] = *NcFromVal1st++;
		}
		CntWrt( data, BURST_LENGTH_FC+3 );  // Cmd 3Byte.
#elif (BURST_LENGTH_FC == 64)
		data[2] = 0x00;
		UlNum = 3;
		for(j=0 ; j < BURST_LENGTH_FC; j++){
			data[UlNum++] = *NcFromVal1st++;
		}
		CntWrt( data, BURST_LENGTH_FC+3 );  // Cmd 3Byte.
#endif

		RamWrite32A( 0xF00B, 0x00000010 );
		UlCnt = 0;
		do{
			WitTim( 1 );
			RamRead32A( 0xF00C, &UlReadVal );
			if( UlCnt++ > 250 ) {
				IOWrite32A( 0xE0701C , 0x00000002);
				return (0x41) ;				// write timeout error
			}
		}while ( UlReadVal != 0 );
	 	RamWrite32A( 0xF00C, 0x00000004 );
	}

	UlCnt = 0;
	do{
		WitTim( 1 );
		RamRead32A( 0xF00C, &UlReadVal );
		if( UlCnt++ > 250 ) {
			IOWrite32A( 0xE0701C , 0x00000002);
			return (0x41) ;				// write timeout error
		}
	}while ( (UlReadVal & 0x0000000C) != 0 );

	IOWrite32A( 0xE0701C , 0x00000002);
	return( 0 );
}


//********************************************************************************
// Function Name 	: CheckDrvOffAdj
// Retun Value		: Driver Offset Re-adjustment
// Argment Value	: NON
// Explanation		: Driver Offset Re-adjustment
// History			: First edition
//********************************************************************************
UINT_32 CheckDrvOffAdj( void )
{

	UINT_32 UlReadDrvOffxy,  UlReadDrvOffaf;

	IOWrite32A( FLASHROM_ACSCNT, 1 ); 					// 1 + 1 = 2word
	IOWrite32A( FLASHROM_FLA_ADR, ((UINT_32)INF_MAT2 << 16) | 0x3C );	// UDRVOFST X/Y/AF

	IOWrite32A( FLASHROM_FLAMODE , 0x00000000);
	IOWrite32A( FLASHROM_CMD, 0x00000001 );

	IORead32A( FLASHROM_FLA_RDAT, &UlReadDrvOffxy ) ;  	// #12
	IORead32A( FLASHROM_FLA_RDAT, &UlReadDrvOffaf ) ;  	// #13
CAM_ERR(CAM_OIS, "[%08x]UlReadDrvOffxy \n", UlReadDrvOffxy );
CAM_ERR(CAM_OIS, "[%08x]UlReadDrvOffaf \n", UlReadDrvOffaf );

	IOWrite32A( FLASHROM_FLAMODE , 0x00000002);

	if( ((UlReadDrvOffxy & 0x80808080) == 0)			// 1b default / 0b update
	 && ((UlReadDrvOffaf & 0xFE00FE00) == 0) ){			// 1b default / 0b update
		return( 0 ); 	// 0 : adjustment done.
	}

	return( 1 );		// 1 : adjustment not yet.
}

//********************************************************************************
// Function Name 	: DrvOffAdj
// Retun Value		: Driver Offset Re-adjustment
// Argment Value	: NON
// Explanation		: Driver Offset Re-adjustment
// History			: First edition
//********************************************************************************
#include	"PmemCode129.h"
//extern UINT_8	OscAdjMain( void );
	#define	MAKER_CODE	0x7777	// ONSEMI


const CODE_TBL_EXT Adj_CdTbl[] = {
	{0xEEEE, LC898129_PM, LC898129_PmemCodeSize,  LC898129_PmemCodeCheckSum, (void*)0, 0, 0, 0}
};


UINT_8 DrvOffAdj( UINT_8 ucSlaveAdr )
{
	UINT_8 ans=0;
	UINT_8 cnt=0;
	UINT_32 UlReadVal;
CAM_ERR(CAM_OIS, "DrvOffAdj \n");
#if 0
	// OSC adjustment if no adjustment
	ans = OscAdjMain();
	if( ans != 0 && ans != 1 ){		// Not OK & Not Done
		return(ans);				// error
	}
#endif
	// Driver offset adjustment if no adjustment
	ans = CheckDrvOffAdj();
CAM_ERR(CAM_OIS, "[%02x]CheckDrvOffAdj \n", ans );

	if( ans == 1 ){

		BootMode();

		IOWrite32A( 0xE07058, (UINT_32)MAKER_CODE << 16 );
	 	ans = PmemUpdate129( 0, (CODE_TBL_EXT *)Adj_CdTbl );	// download the special program for driver adjustment.
CAM_ERR(CAM_OIS, "[%02x]PmemUpdate129 \n", ans );
		if(ans != 0) return( ans );

		// Set slave address
		SetSlaveAddr( ucSlaveAdr );

		RamWrite32A( 0xF001,  0x00000000 ) ;					// Remap
		WitTim( 1 ) ;
		IOWrite32A( 0xE07CCC, 0x0000C5AD );						// additional unlock for INFO
		IOWrite32A( 0xE07CCC, 0x0000ACD5 );						// UNLK_CODE3(E0_7CCCh) = 0000_ACD5h

		WitTim( 15 );											// Flash Information Mat 2 Erase & Page 3 Program
		WitTim( 4 );											//

		cnt = 0;
		do {
			IORead32A( SYSDSP_REMAP, (UINT_32 *)&UlReadVal ) ;	// Conform adjust success
			if( UlReadVal == 0x0004A0){
				ans = 0;
				break;
			} else {
				ans = 0x91;										// adjust error
			}

			WitTim( 1 );
			cnt++;
CAM_ERR(CAM_OIS, "[%02x] %08x  \n", cnt, UlReadVal );			
		} while( cnt < 10 ) ;
CAM_ERR(CAM_OIS, "[%02x]SYSDSP_REMAP \n", ans );

	
	}
	return(ans);
}

//********************************************************************************
// Function Name 	: FlashUpdate129
// Retun Value		: NON
// Argment Value	: chiperase, ptr
// Explanation		: Flash Update for LC898129
// History			: First edition
//********************************************************************************
UINT_8 FlashUpdate129( UINT_8 chiperase, CODE_TBL_EXT* ptr )
{
	UINT_8 ans=0;
	UINT_32	UlReadVal, UlCnt ;

//--------------------------------------------------------------------------------
// 0. <Info Mat1> Driver Offset
//--------------------------------------------------------------------------------
	ans =  DrvOffAdj( DEFAULT_SLAVE_ADR	);
	
CAM_ERR(CAM_OIS, "[%02x]DrvOffAdj  \n", ans );
 	if(ans != 0) return( ans );
	
	BootMode();

 	ans = PmemUpdate129( 0, ptr );		// download the special program for updating the flash memory.
CAM_ERR(CAM_OIS, "[%02x]PmemUpdate129  \n", ans );
	if(ans != 0) return( ans );

//--------------------------------------------------------------------------------
// 1. <User Mat> Erase
//--------------------------------------------------------------------------------
	if( UnlockCodeSet() != 0 ) 		return (0x33) ;		// Unlock Code set error
	WritePermission();									// Write Permission
	AdditionalUnlockCodeSet();							// Additional Unlock Code Set

	if( chiperase != 0 )
	 	ans = EraseUserMat129(0, FLASH_BLOCKS);			// Erase all block.
	else
		ans = EraseUserMat129(0, ERASE_BLOCKS);			// Erase block exclude user data area.
CAM_ERR(CAM_OIS, "[%02x]EraseUserMat129  \n", ans );
	if(ans != 0){
		if( UnlockCodeClear() != 0 ) 	return (0x32) ;	// unlock code clear error
		else					 		return( ans );
	}

//--------------------------------------------------------------------------------
// 2. <User Mat> Write
//--------------------------------------------------------------------------------
	ans = ProgramFlash129_Standard( ptr );

	if(ans != 0){
		if( UnlockCodeClear() != 0 ) 	return (0x43) ;			// unlock code clear error
		else					 		return( ans );
	}

	CAM_ERR(CAM_OIS, "[%d]ProgramFlash end \n", ans );

	if( UnlockCodeClear() != 0 ) 	return (0x43) ;				// unlock code clear error

//--------------------------------------------------------------------------------
// 3. <User Mat> Verify
//--------------------------------------------------------------------------------
	IOWrite32A( 0xE0701C , 0x00000000);
	RamWrite32A( 0xF00A, 0x00000000 );
	RamWrite32A( 0xF00D, ptr->SizeFromCodeValid );

	RamWrite32A( 0xF00C, 0x00000100 );
	WitTim( 6 );
	UlCnt = 0;
	do{
		RamRead32A( 0xF00C, &UlReadVal );
		if( UlCnt++ > 10 ) {
			IOWrite32A( 0xE0701C , 0x00000002);
			return (0x51) ;				// check sum excute error
		}
		WitTim( 1 );
	}while ( UlReadVal != 0 );

	RamRead32A( 0xF00D, &UlReadVal );

	if( UlReadVal != ptr->SizeFromCodeCksm ) {
		IOWrite32A( 0xE0701C , 0x00000002);
		return( 0x52 );
	}

CAM_ERR(CAM_OIS, "[]UserMat Verify OK \n" );

//	CoreReset
	IOWrite32A( SYSDSP_REMAP,				0x00001001 ) ;
	WitTim( 15 ) ;
	IORead32A( ROMINFO,				(UINT_32 *)&UlReadVal ) ;
	if( UlReadVal != 0x0A)		return( 0x53 );

CAM_ERR(CAM_OIS, "[]Remap OK \n" );


	return ( 0 );
}


//********************************************************************************
// Function Name 	: FlashBlockErase
// Retun Value		: 0 : Success, 1 : Unlock Error, 2 : timeout Error
// Argment Value	: Use Mat , Flash Address
// Explanation		: <Flash Memory> Block Erase
// Unit of erase	: informaton mat  256 Byte
//					: user mat         4k Byte
//********************************************************************************
UINT_8	FlashBlockErase( UINT_8 SelMat , UINT_32 SetAddress )
{
	UINT_32	UlReadVal, UlCnt;
	UINT_8	ans	= 0 ;

	// fail safe
	// reject illegal mat
	if( SelMat != USER_MAT && SelMat != INF_MAT0 && SelMat != INF_MAT1 && SelMat != INF_MAT2  )	return 10;
	// reject command if over range
	if( SetAddress > 0x00003CFF )											return 9;

	// Flash write
	ans	= UnlockCodeSet();
	if( ans != 0 )	return( ans ) ;							// Unlock Code Set

	WritePermission();										// Write permission
	if( SelMat == TRIM_MAT ){
		IOWrite32A( 0xE07CCC, 0x00005B29 );					// additional unlock for TRIMMING
	} else if (SelMat != USER_MAT ){
		IOWrite32A( 0xE07CCC, 0x0000C5AD );					// additional unlock for INFO
	}
	AdditionalUnlockCodeSet();								// common additional unlock code set

	IOWrite32A( FLASHROM_FLA_ADR, ((UINT_32)SelMat << 16) | ( SetAddress & 0x00003C00 )) ;
	// Sector Erase Start
	IOWrite32A( FLASHROM_CMD, 4 ) ;

	WitTim( 5 ) ;

	UlCnt	= 0 ;

	do {
		if( UlCnt++ > 100 ){	ans = 2;	break;	} ;

		IORead32A( FLASHROM_FLAINT, &UlReadVal ) ;
	} while( ( UlReadVal & 0x00000080 ) != 0 ) ;

	ans	= UnlockCodeClear();								// Unlock Code Clear
	if( ans != 0 )	return( ans ) ;							// Unlock Code Set

	return( ans ) ;
}

//********************************************************************************
// Function Name 	: FlashSingleRead
// Retun Value		: 0 : Success
// Argment Value	: SelMat, UlAddress, *PulData
// Explanation		: <Flash Memory> Flash Single Read( 4 Byte read )
//********************************************************************************
UINT_8	FlashSingleRead( UINT_8 SelMat, UINT_32 UlAddress, UINT_32 *PulData )
{

	// fail safe
	// reject illegal mat
	if( SelMat != USER_MAT && SelMat != INF_MAT0 && SelMat != INF_MAT1 && SelMat != INF_MAT2  )	return 10;
	// reject command if address inner NVR3
	if( UlAddress > 0x00003FFF )											return 9;

	IOWrite32A( FLASHROM_ACSCNT, 0x00000000 );
	IOWrite32A( FLASHROM_FLA_ADR, ((UINT_32)SelMat << 16) | ( UlAddress & 0x00003FFF ) );

	IOWrite32A( FLASHROM_CMD, 0x00000001 );

	IORead32A( FLASHROM_FLA_RDAT, PulData ) ;

	return( 0 ) ;
}

//********************************************************************************
// Function Name 	: FlashMultiRead
// Retun Value		: 0 : Success
// Argment Value	: SelMat, UlAddress, *PulData, UcLength
// Explanation		: <Flash Memory> Flash Multi Read( 4 Byte * length  max read : 256byte)
//********************************************************************************
UINT_8	FlashMultiRead( UINT_8 SelMat, UINT_32 UlAddress, UINT_32 *PulData , UINT_8 UcLength )
{
	UINT_8	i	 ;

	// fail safe
	// reject illegal mat
	if( SelMat != USER_MAT && SelMat != INF_MAT0 && SelMat != INF_MAT1 && SelMat != INF_MAT2  )	return 10;
	// reject command if address inner
	if( UlAddress > 0x00003FFF )											return 9;

	IOWrite32A( FLASHROM_ACSCNT, 0x00000000 | (UINT_32)(UcLength-1) );
	IOWrite32A( FLASHROM_FLA_ADR, ((UINT_32)SelMat << 16) | ( UlAddress & 0x00003FFF ) );

	IOWrite32A( FLASHROM_CMD, 0x00000001 );
	for( i=0 ; i < UcLength ; i++ ){
		IORead32A( FLASHROM_FLA_RDAT, &PulData[i] ) ;
CAM_ERR(CAM_OIS, "Read Data[%02x] = %08x\n", i , PulData[i] );
	}
	TRACE_DUMP((const char *)PulData, UcLength * 4 );

	return( 0 ) ;
}

//********************************************************************************
// Function Name 	: FlashPageWrite
// Retun Value		: 0 : Success, 1 : Unlock Error, 2 : timeout Error
// Argment Value	: Info Mat , Flash Address
// Explanation		: <Flash Memory> Page write
// Unit of erase	: informaton mat   64 Byte
//					: user mat         64 Byte
//********************************************************************************
UINT_8	FlashPageWrite( UINT_8 SelMat , UINT_32 SetAddress , UINT_32 *PulData)
{
	UINT_32	UlReadVal, UlCnt;
	UINT_8	ans	= 0 ;
	UINT_8	i	 ;

	// fail safe
	// reject illegal mat
	if( SelMat != USER_MAT && SelMat != INF_MAT0 && SelMat != INF_MAT1 && SelMat != INF_MAT2  )	return 10;

	if( SetAddress > 0x00003FFF )							return 9;

	// Flash write
	ans	= UnlockCodeSet();
	if( ans != 0 )	return( ans ) ;							// Unlock Code Set

	WritePermission();										// Write permission
	if( SelMat != USER_MAT ){
		IOWrite32A( 0xE07CCC, 0x0000C5AD );					// additional unlock for INFO
	}
	AdditionalUnlockCodeSet();								// common additional unlock code set

	IOWrite32A( FLASHROM_FLA_ADR, ((UINT_32)SelMat << 16) | ( SetAddress & 0x00003FF0 )) ;// address is page unit
	// page write Start
	IOWrite32A( FLASHROM_CMD, 2 ) ;

//	WitTim( 5 ) ;

	UlCnt	= 0 ;

	for( i=0 ; i< 16 ; i++ ){
		IOWrite32A( FLASHROM_FLA_WDAT, PulData[i]  );	// Write data
CAM_ERR(CAM_OIS, "Write Data[%d] = %08x\n", i , PulData[i] );
	}
	do {
		if( UlCnt++ > 100 ){	ans = 2;	break;	} ;

		IORead32A( FLASHROM_FLAINT, &UlReadVal ) ;
	} while( ( UlReadVal & 0x00000080 ) != 0 ) ;

	// page program
	IOWrite32A( FLASHROM_CMD, 8  );

	do {
		if( UlCnt++ > 100 ){	ans = 2;	break;	} ;

		IORead32A( FLASHROM_FLAINT, &UlReadVal ) ;
	} while( ( UlReadVal & 0x00000080 ) != 0 ) ;

	ans	= UnlockCodeClear();								// Unlock Code Clear
	return( ans ) ;

}

//********************************************************************************
// Function Name 	: RdErInfMAT
// Retun Value		: 0 : Success, other : Error
// Argment Value	: SelMat, *InfMat, Length
// Explanation		: Read InfMAT data to buffer and erase InfMAT
// Unit of length	: UINT_32
//********************************************************************************
UINT_8	RdErInfMAT( UINT_8 SelMat, UINT_32 *InfMat, UINT_16 Length )
{
	UINT_8	ans	= 0 ;

CAM_ERR(CAM_OIS, "InfMAT%d read\n", SelMat );
	ans =FlashMultiRead( SelMat, 0, InfMat, Length );

	if( ans == 0 ) {
CAM_ERR(CAM_OIS, "done\n");
CAM_ERR(CAM_OIS, "InfMAT%d erase ", SelMat );
		ans = FlashBlockErase( SelMat , 0 );
		if( ans != 0 ) {
CAM_ERR(CAM_OIS, "error");
			ans = 2 ;
		}
	} else {
CAM_ERR(CAM_OIS, "error");
		ans = 1 ;
	}
CAM_ERR(CAM_OIS, "\n");
	return( ans ) ;
}

//********************************************************************************
// Function Name 	: WrInfMAT
// Retun Value		: 0 : Success, other : Error
// Argment Value	: SelMat, *InfMat, Length
// Explanation		: Write InfMAT data from buffer
// Unit of length	: UINT_32
//********************************************************************************
UINT_8	WrInfMAT( UINT_8 SelMat, UINT_32 *InfMat, UINT_16 Length )
{
	UINT_8	ans	= 0 ;
	UINT_32	address ;

	for( address = 0; ( address < Length ) && ( ans == 0 ); address += 16 ) {
		ans = FlashPageWrite( SelMat , address , &InfMat[address] );
	}
	return( ans ) ;
}

//********************************************************************************
// Function Name 	: MkInfMATsum
// Retun Value		: Checksum value
// Argment Value	: *InfMAT
// Explanation		: Make checksum from buffer
//********************************************************************************
UINT_16	MkInfMATsum( UINT_32 *InfMAT )
{
	UINT_16 UsCkVal = 0 ;
	UINT_16 i ;

	for( i = 0; i < 63; i++ ) {
		UsCkVal +=  (UINT_8)(InfMAT[ i ] >> 0);
		UsCkVal +=  (UINT_8)(InfMAT[ i ] >> 8);
		UsCkVal +=  (UINT_8)(InfMAT[ i ] >> 16);
		UsCkVal +=  (UINT_8)(InfMAT[ i ] >> 24);
	}
	// Remainder Index=63
	UsCkVal +=  (UINT_8)(InfMAT[ i ] >> 16);
	UsCkVal +=  (UINT_8)(InfMAT[ i ] >> 24);

CAM_ERR(CAM_OIS, "InfMAT checksum %04X\n", UsCkVal ) ;
	return ( UsCkVal ) ;
}

//********************************************************************************
// Function Name 	: WrHallCalData
// Retun Value		: 0:OK, 1:NG
// Argment Value	: UcMode
// Explanation		: Flash write hall calibration data
//********************************************************************************
UINT_8 WrHallCalData( UINT_8 UcMode )
{
	UINT_32	UlMAT0[64];
	UINT_32 UlT0, UlT1, UlT2, UlT3, UlTEMP;
	UINT_8 ans = 0;
	UINT_16	UsCkVal,UsCkVal_Bk ;

	IOWrite32A( 0xE0701C , 0x00000000 ) ;

CAM_ERR(CAM_OIS,  "WrHallCalData\n" );
	/* Back up ******************************************************/
	/* Erase   ******************************************************/
	ans = RdErInfMAT( INF_MAT0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( ans );
	}

#if DEBUG
	UINT_8	i;
	for(i=0;i<64;i++){
CAM_ERR(CAM_OIS,  "InfMAT[%02x] = %08x\n", i, UlMAT0[i] );
	}
#endif //DEBUG == 1
	/* modify   *****************************************************/
	if ( UcMode ) {	/* write */
		RamRead32A(  SMA_COEFF_TGAIN0BIAS , &UlT0 ) ;
		RamRead32A(  SMA_COEFF_TGAIN1BIAS , &UlT1 ) ;
		RamRead32A(  SMA_COEFF_TGAIN2BIAS , &UlT2 ) ;
		RamRead32A(  SMA_COEFF_TGAIN3BIAS , &UlT3 ) ;
		RamRead32A(  SMA_KELVIN , &UlTEMP ) ;

		UlMAT0[SMA_CALIBRATION_STATUS] &= ~( HALL_CALB_FLG | CLAF_CALB_FLG | HALL_CALB_BIT );
		UlMAT0[SMA_CALIBRATION_STATUS] |= StAdjPar.StHalAdj.UlAdjPhs ;							// Calibration Status

		UlMAT0[SMA_HALL_BIAS_OFFSET_X]		= (UINT_32)(((UINT_32)(StAdjPar.StHalAdj.UsHlxOffO&0xFF00) << 8) | ((UINT_32)(StAdjPar.StHalAdj.UsHlxOffI & 0xFF00)>>0) | ((UINT_32)(StAdjPar.StHalAdj.UsHlxGan & 0xFF00) >> 8));
		UlMAT0[SMA_HALL_BIAS_OFFSET_Y]		= (UINT_32)(((UINT_32)(StAdjPar.StHalAdj.UsHlyOffO&0xFF00) << 8) | ((UINT_32)(StAdjPar.StHalAdj.UsHlyOffI & 0xFF00)>>0) | ((UINT_32)(StAdjPar.StHalAdj.UsHlyGan & 0xFF00) >> 8));
		UlMAT0[SMA_HALL_BIAS_OFFSET_Z]		= (UINT_32)(((UINT_32)(StAdjPar.StHalAdj.UsHlzOffO&0xFF00) << 8) | ((UINT_32)(StAdjPar.StHalAdj.UsHlzOffI & 0xFF00)>>0) | ((UINT_32)(StAdjPar.StHalAdj.UsHlzGan & 0xFF00) >> 8));
		UlMAT0[SMA_LOOPGAIN_LENS_OFFSET_X]	= (((UINT_32)(StAdjPar.StLopGan.UlLxgVal & 0xFFFF0000) >> 16) | ((UINT_32)StAdjPar.StHalAdj.UsAdxOff << 16 ));
		UlMAT0[SMA_LOOPGAIN_LENS_OFFSET_Y]	= (((UINT_32)(StAdjPar.StLopGan.UlLygVal & 0xFFFF0000) >> 16) | ((UINT_32)StAdjPar.StHalAdj.UsAdyOff << 16 ));
		UlMAT0[SMA_LOOPGAIN_LENS_OFFSET_Z]	= (((UINT_32)(StAdjPar.StLopGan.UlLzgVal & 0xFFFF0000) >> 16) | ((UINT_32)StAdjPar.StHalAdj.UsAdzOff << 16 ));

		UlMAT0[SMA_HALL_AMPLITUDE_X]		= (UINT_32)((UINT_32)(StAdjPar.StHalAdj.UsHlxMxa << 16) | (UINT_32)StAdjPar.StHalAdj.UsHlxMna);
		UlMAT0[SMA_HALL_AMPLITUDE_Y]		= (UINT_32)((UINT_32)(StAdjPar.StHalAdj.UsHlyMxa << 16) | (UINT_32)StAdjPar.StHalAdj.UsHlyMna);
		UlMAT0[SMA_HALL_AMPLITUDE_Z]		= (UINT_32)((UINT_32)(StAdjPar.StHalAdj.UsHlzMxa << 16) | (UINT_32)StAdjPar.StHalAdj.UsHlzMna);

		UlMAT0[SMA_GYRO_GAIN_X]				= 0x3FFFFFFF;
		UlMAT0[SMA_GYRO_GAIN_Y]				= 0x3FFFFFFF;
		UlMAT0[SMA_GYRO_GAIN_Z]				= 0x3FFFFFFF;

		UlMAT0[SMA_BIAS0]					= (UlT1 & 0xFFFF0000) | (UlT0 >> 16) ;	// BIAS1[31:16] / BIAS0[15:0]
		UlMAT0[SMA_BIAS2]					= (UlT3 & 0xFFFF0000) | (UlT2 >> 16) ;	// BIAS3[31:16] / BIAS2[15:0]
		UlMAT0[SMA_TEMP ]					= UlTEMP ;
	} else {
		UlMAT0[SMA_CALIBRATION_STATUS] |= HALL_CALB_FLG | CLAF_CALB_FLG | HALL_CALB_BIT ;

		UlMAT0[SMA_HALL_BIAS_OFFSET_X]		= 0x00000000;
		UlMAT0[SMA_HALL_BIAS_OFFSET_Y]		= 0x00000000;
		UlMAT0[SMA_HALL_BIAS_OFFSET_Z]		= 0x00000000;
		UlMAT0[SMA_LOOPGAIN_LENS_OFFSET_X]	= 0x00000000;
		UlMAT0[SMA_LOOPGAIN_LENS_OFFSET_Y]	= 0x00000000;
		UlMAT0[SMA_LOOPGAIN_LENS_OFFSET_Z]	= 0x00000000;

		UlMAT0[SMA_HALL_AMPLITUDE_X]		= 0x00000000;
		UlMAT0[SMA_HALL_AMPLITUDE_Y]		= 0x00000000;
		UlMAT0[SMA_HALL_AMPLITUDE_Z]		= 0x00000000;

		UlMAT0[SMA_GYRO_GAIN_X]				= 0x3FFFFFFF;
		UlMAT0[SMA_GYRO_GAIN_Y]				= 0x3FFFFFFF;
		UlMAT0[SMA_GYRO_GAIN_Z]				= 0x3FFFFFFF;

		UlMAT0[SMA_BIAS0]					= 0xFFFFFFFF;
		UlMAT0[SMA_BIAS2]					= 0xFFFFFFFF;
		UlMAT0[SMA_TEMP ]					= 0xFFFFFFFF;
	}
	/* calcurate check sum ******************************************/
	UsCkVal = MkInfMATsum( UlMAT0 ) ;
	UlMAT0[MAT0_CKSM] &= (UINT_32)0xFFFF0000 ;
	UlMAT0[MAT0_CKSM] |= (UINT_32)UsCkVal ;

	/* update ******************************************************/
	ans = WrInfMAT( INF_MAT0, UlMAT0, 64 ) ;
	if( ans != 0 ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 3 ) ;							// 0 - 63
	}
	/* Verify ******************************************************/
	UsCkVal_Bk = UsCkVal;
	ans =FlashMultiRead( INF_MAT0, 0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 4 );
	}
	UsCkVal = MkInfMATsum( UlMAT0 ) ;

CAM_ERR(CAM_OIS,  "[RVAL]:[BVal]=[%04x]:[%04x]\n",UsCkVal, UsCkVal_Bk );
	if( UsCkVal != UsCkVal_Bk ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return(5);
	}
CAM_ERR(CAM_OIS,  "WrHallCalData____COMPLETE\n" );


	IOWrite32A( 0xE0701C , 0x00000002 ) ;
	return(0);

}

//********************************************************************************
// Function Name 	: WrHallCalData_LS
// Retun Value		: 0:OK, 1:NG
// Argment Value	: UcMode
// Explanation		: Flash write hall calibration data
//********************************************************************************
UINT_8 WrHallCalData_LS( void )
{
	UINT_32	UlMAT0[64];
	UINT_8	ans = 0, uc_402_data[ 8 ] ;
	UINT_16	UsCkVal, UsCkVal_Bk;

	IOWrite32A( 0xE0701C, 0x00000000 );

	CAM_ERR(CAM_OIS,  "WrHallCalData\n" );
	/* Back up ******************************************************/
	/* Erase   ******************************************************/
	ans = RdErInfMAT( INF_MAT0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C, 0x00000002 );
		return( ans );
	}

#if DEBUG
	UINT_8	i;
	for( i = 0; i < 64; i++ ) {
		CAM_ERR(CAM_OIS,  "InfMAT[%02x] = %08x\n", i, UlMAT0[i] );
	}
#endif //DEBUG == 1
	/* modify   *****************************************************/
	UlMAT0[CALIBRATION_STATUS] &= ~( HALL_CALB_FLG | CLAF_CALB_FLG | HALL_CALB_BIT );
	UlMAT0[CALIBRATION_STATUS] |= StAdjPar.StHalAdj.UlAdjPhs;			// Calibration Status

//	UlMAT0[LOOPGAIN_LENS_OFFSET_X]	= (((UINT_32)(StAdjPar.StLopGan.UlLxgVal & 0xFFFF0000) >> 16) | ((UINT_32)StAdjPar.StHalAdj.UsAdxOff << 16));
//	UlMAT0[LOOPGAIN_LENS_OFFSET_Y]	= (((UINT_32)(StAdjPar.StLopGan.UlLygVal & 0xFFFF0000) >> 16) | ((UINT_32)StAdjPar.StHalAdj.UsAdyOff << 16));

//	UlMAT0[HALL_AMPLITUDE_X]		= (UINT_32)((UINT_32)(StAdjPar.StHalAdj.UsHlxMxa << 16) | (UINT_32)StAdjPar.StHalAdj.UsHlxMna);
//	UlMAT0[HALL_AMPLITUDE_Y]		= (UINT_32)((UINT_32)(StAdjPar.StHalAdj.UsHlyMxa << 16) | (UINT_32)StAdjPar.StHalAdj.UsHlyMna);

	UlMAT0[ACCL_GYRO_OFFSET_X]		= (UINT_32)((UINT_32)(StAclVal.StAccel.SlOffsetX & 0xFFFF0000) | (UINT_32)StAdjPar.StGvcOff.UsGxoVal);
	UlMAT0[ACCL_GYRO_OFFSET_Y]		= (UINT_32)((UINT_32)(StAclVal.StAccel.SlOffsetY & 0xFFFF0000) | (UINT_32)StAdjPar.StGvcOff.UsGyoVal);
	UlMAT0[ACCL_GYRO_OFFSET_Z]		= (UINT_32)((UINT_32)(StAclVal.StAccel.SlOffsetZ & 0xFFFF0000) | (UINT_32)StAdjPar.StGvcOff.UsGzoVal);

	UlMAT0[GYRO_GAIN_X]				= 0x3FFFFFFF;
	UlMAT0[GYRO_GAIN_Y]				= 0x3FFFFFFF;

	/* calcurate check sum ******************************************/
	UsCkVal = MkInfMATsum( UlMAT0 );
	UlMAT0[MAT0_CKSM] &= (UINT_32)0xFFFF0000;
	UlMAT0[MAT0_CKSM] |= (UINT_32)UsCkVal;

	/* update ******************************************************/
	ans = WrInfMAT( INF_MAT0, UlMAT0, 64 );
	if( ans != 0 ) {
		IOWrite32A( 0xE0701C, 0x00000002 );
		return( 3 );							// 0 - 63
	}
	/* Verify ******************************************************/
	UsCkVal_Bk = UsCkVal;
	ans =FlashMultiRead( INF_MAT0, 0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C, 0x00000002 );
		return( 4 );
	}
	UsCkVal = MkInfMATsum( UlMAT0 );

	CAM_ERR(CAM_OIS,  "[RVAL]:[BVal]=[%04x]:[%04x]\n",UsCkVal, UsCkVal_Bk );
	if( UsCkVal != UsCkVal_Bk ) {
		IOWrite32A( 0xE0701C, 0x00000002 );
		return( 5 );
	}
	CAM_ERR(CAM_OIS,  "WrHallCalData____COMPLETE\n" );

	IOWrite32A( 0xE0701C, 0x00000002 );

	uc_402_data[ 0 ]	= ( UINT_8 )StAdjPar.StHalAdj.UsHlxOffO ;
	uc_402_data[ 1 ]	= ( UINT_8 )StAdjPar.StHalAdj.UsHlxGan ;
	ans	= Cont_EEPWrite402( Y_402, EP_DAHLO, uc_402_data, SEL_UP, 2 ) ;
	if( ans != SUCCESS ) {
		return( 6 ) ;
	}

	uc_402_data[ 0 ]	= ( UINT_8 )( StAdjPar.StHalAdj.UsHlxMxa >> 8 ) ;
	uc_402_data[ 1 ]	= ( UINT_8 )( StAdjPar.StHalAdj.UsHlxMxa & 0x00FF ) ;
	uc_402_data[ 2 ]	= ( UINT_8 )( StAdjPar.StHalAdj.UsHlxMna >> 8 ) ;
	uc_402_data[ 3 ]	= ( UINT_8 )( StAdjPar.StHalAdj.UsHlxMna & 0x00FF ) ;
	uc_402_data[ 4 ]	= ( UINT_8 )( StAdjPar.StHalAdj.UsAdxOff >> 8 ) ;
	uc_402_data[ 5 ]	= ( UINT_8 )( StAdjPar.StHalAdj.UsAdxOff & 0x00FF ) ;
	uc_402_data[ 6 ]	= ( UINT_8 )( StAdjPar.StLopGan.UlLxgVal >> 24 ) ;
	uc_402_data[ 7 ]	= ( UINT_8 )( ( StAdjPar.StLopGan.UlLxgVal >> 16 ) & 0x000000FF ) ;
	ans	= Cont_EEPWrite402( X_402, EP_HALL_MAX, uc_402_data, SEL_DN, 8 ) ;
	if( ans != SUCCESS ) {
		return( 6 ) ;
	}

	uc_402_data[ 0 ]	= ( UINT_8 )StAdjPar.StHalAdj.UsHlyOffO ;
	uc_402_data[ 1 ]	= ( UINT_8 )StAdjPar.StHalAdj.UsHlyGan ;
	ans	= Cont_EEPWrite402( X_402, EP_DAHLO, uc_402_data, SEL_UP, 2 ) ;
	if( ans != SUCCESS ) {
		return( 6 ) ;
	}

	uc_402_data[ 0 ]	= ( UINT_8 )( StAdjPar.StHalAdj.UsHlyMxa >> 8 ) ;
	uc_402_data[ 1 ]	= ( UINT_8 )( StAdjPar.StHalAdj.UsHlyMxa & 0x00FF ) ;
	uc_402_data[ 2 ]	= ( UINT_8 )( StAdjPar.StHalAdj.UsHlyMna >> 8 ) ;
	uc_402_data[ 3 ]	= ( UINT_8 )( StAdjPar.StHalAdj.UsHlyMna & 0x00FF ) ;
	uc_402_data[ 4 ]	= ( UINT_8 )( StAdjPar.StHalAdj.UsAdyOff >> 8 ) ;
	uc_402_data[ 5 ]	= ( UINT_8 )( StAdjPar.StHalAdj.UsAdyOff & 0x00FF ) ;
	uc_402_data[ 6 ]	= ( UINT_8 )( StAdjPar.StLopGan.UlLygVal >> 24 ) ;
	uc_402_data[ 7 ]	= ( UINT_8 )( ( StAdjPar.StLopGan.UlLygVal >> 16 ) & 0x000000FF ) ;
	ans	= Cont_EEPWrite402( Y_402, EP_HALL_MAX, uc_402_data, SEL_DN, 8 ) ;
	if( ans != SUCCESS ) {
		return( 6 ) ;
	}

	return( 0 );
}

//********************************************************************************
// Function Name 	: WrGyAcOffsetData
// Retun Value		: 0:OK, 1:NG
// Argment Value	: UcMode
// Explanation		: Flash Write Gyro offset Data Function
//********************************************************************************
UINT_8	WrGyAcOffsetData( UINT_8 UcMode )
{
	UINT_32	UlMAT0[64];
	UINT_32	UlReadGx, UlReadGy, UlReadGz, UlReadAx, UlReadAy, UlReadAz;
	UINT_8 ans = 0;
	UINT_16	UsCkVal,UsCkVal_Bk ;

	IOWrite32A( 0xE0701C , 0x00000000 ) ;

CAM_ERR(CAM_OIS,  "WrGyAcOffsetData \n" );
	/* Back up ******************************************************/
	/* Erase   ******************************************************/
	ans = RdErInfMAT( INF_MAT0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( ans );
	}
#if DEBUG
	UINT_8	i;
	for(i=0;i<64;i++){
CAM_ERR(CAM_OIS,  "[ %d ] = %08x\n", i, UlMAT0[i] );
	}
#endif //DEBUG == 1
	/* modify   *****************************************************/
	if( UcMode ){	// write
		RamRead32A(  GYRO_RAM_GXOFFZ , &UlReadGx ) ;
		RamRead32A(  GYRO_RAM_GYOFFZ , &UlReadGy ) ;
		RamRead32A(  GYRO_ZRAM_GZOFFZ , &UlReadGz ) ;
		RamRead32A(  ACCLRAM_X_AC_OFFSET , &UlReadAx ) ;
		RamRead32A(  ACCLRAM_Y_AC_OFFSET , &UlReadAy ) ;
		RamRead32A(  ACCLRAM_Z_AC_OFFSET , &UlReadAz ) ;		
		
		UlMAT0[CALIBRATION_STATUS] &= ~( ACCL_OFST_FLG );		
		UlMAT0[ACCL_GYRO_OFFSET_X]		= (UINT_32)((UlReadAx & 0xFFFF0000) | ((UlReadGx >> 16 ) & 0x0000FFFF));
		UlMAT0[ACCL_GYRO_OFFSET_Y]		= (UINT_32)((UlReadAy & 0xFFFF0000) | ((UlReadGy >> 16 ) & 0x0000FFFF));
		UlMAT0[ACCL_GYRO_OFFSET_Z]		= (UINT_32)((UlReadAz & 0xFFFF0000) | ((UlReadGz >> 16 ) & 0x0000FFFF));

	}else{
		UlMAT0[CALIBRATION_STATUS] |= ACCL_OFST_FLG;		
		UlMAT0[ACCL_GYRO_OFFSET_X]		= 0x00000000;
		UlMAT0[ACCL_GYRO_OFFSET_Y]		= 0x00000000;
		UlMAT0[ACCL_GYRO_OFFSET_Z]		= 0x00000000;
	}
	
	/* calcurate check sum ******************************************/
	UsCkVal = MkInfMATsum( UlMAT0 ) ;
	UlMAT0[MAT0_CKSM] &= (UINT_32)0xFFFF0000 ;
	UlMAT0[MAT0_CKSM] |= (UINT_32)UsCkVal ;

	/* update ******************************************************/
	ans = WrInfMAT( INF_MAT0, UlMAT0, 64 ) ;
	if( ans != 0 ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 3 ) ;							// 0 - 63
	}
	/* Verify ******************************************************/
	UsCkVal_Bk = UsCkVal;
	ans =FlashMultiRead( INF_MAT0, 0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 4 );
	}
	UsCkVal = MkInfMATsum( UlMAT0 ) ;

CAM_ERR(CAM_OIS,  "[RVAL]:[BVal]=[%04x]:[%04x]\n",UsCkVal, UsCkVal_Bk );
	if( UsCkVal != UsCkVal_Bk ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return(5);
	}
CAM_ERR(CAM_OIS,  "WrGyAcOffsetData____COMPLETE\n" );
	IOWrite32A( 0xE0701C , 0x00000002 ) ;
	return(0);
}

//********************************************************************************
// Function Name 	: WrGyroOffsetTbl
// Retun Value		: 0:OK, 1:NG
// Argment Value	: stGyroOffsetTbl *
// Explanation		: Flash Write Gyro offset accel offset Function
//********************************************************************************
UINT_8	WrGyroOffsetTbl( stGyroOffsetTbl *pTbl )
{
	UINT_32	UlMAT0[64];
	UINT_8 ans = 0;
	UINT_16	UsCkVal,UsCkVal_Bk ;

	IOWrite32A( 0xE0701C , 0x00000000 ) ;

CAM_ERR(CAM_OIS,  "WrGyroOffsetTbl \n" );
	/* Back up ******************************************************/
	/* Erase   ******************************************************/
	ans = RdErInfMAT( INF_MAT0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( ans );
	}
#if DEBUG
	UINT_8	i;
	for(i=0;i<64;i++){
CAM_ERR(CAM_OIS,  "[ %d ] = %08x\n", i, UlMAT0[i] );
	}
#endif //DEBUG == 1
	/* modify   *****************************************************/
	UlMAT0[ACCL_GYRO_OFFSET_X]		= (UINT_32)((UINT_32)(pTbl->StAccel.SlOffsetX & 0xFFFF0000) | (UINT_32)(pTbl->StAngle.SlOffsetX >> 16));
	UlMAT0[ACCL_GYRO_OFFSET_Y]		= (UINT_32)((UINT_32)(pTbl->StAccel.SlOffsetY & 0xFFFF0000) | (UINT_32)(pTbl->StAngle.SlOffsetY >> 16));
	UlMAT0[ACCL_GYRO_OFFSET_Z]		= (UINT_32)((UINT_32)(pTbl->StAccel.SlOffsetZ & 0xFFFF0000) | (UINT_32)(pTbl->StAngle.SlOffsetZ >> 16));

	/* calcurate check sum ******************************************/
	UsCkVal = MkInfMATsum( UlMAT0 ) ;
	UlMAT0[MAT0_CKSM] &= (UINT_32)0xFFFF0000 ;
	UlMAT0[MAT0_CKSM] |= (UINT_32)UsCkVal ;

	/* update ******************************************************/
	ans = WrInfMAT( INF_MAT0, UlMAT0, 64 ) ;
	if( ans != 0 ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 3 ) ;							// 0 - 63
	}
	/* Verify ******************************************************/
	UsCkVal_Bk = UsCkVal;
	ans =FlashMultiRead( INF_MAT0, 0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 4 );
	}
	UsCkVal = MkInfMATsum( UlMAT0 ) ;

CAM_ERR(CAM_OIS,  "[RVAL]:[BVal]=[%04x]:[%04x]\n",UsCkVal, UsCkVal_Bk );
	if( UsCkVal != UsCkVal_Bk ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return(5);
	}
CAM_ERR(CAM_OIS,  "WrGyroOffsetTbl____COMPLETE\n" );
	IOWrite32A( 0xE0701C , 0x00000002 ) ;
	return(0);
}


//********************************************************************************
// Function Name 	: WrWireBias
// Retun Value		: 0:OK, 1:NG
// Argment Value	: UcMode
// Explanation		: Flash Write Wire Bias Data Function
//********************************************************************************
UINT_8	WrWireBias( UINT_8 UcMode )
{
	UINT_32	UlMAT0[64];
	UINT_32 UlT0, UlT1, UlT2, UlT3, UlTEMP;
	UINT_8 ans = 0;
	UINT_16	UsCkVal,UsCkVal_Bk ;

	IOWrite32A( 0xE0701C , 0x00000000 ) ;

CAM_ERR(CAM_OIS,  "WrWireBias\n" );
	/* Back up ******************************************************/
	/* Erase   ******************************************************/
	ans = RdErInfMAT( INF_MAT0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( ans );
	}
#if DEBUG
	UINT_8	i;
	for(i=0;i<64;i++){
CAM_ERR(CAM_OIS,  "[ %d ] = %08x\n", i, UlMAT0[i] );
	}
#endif //DEBUG == 1
	/* modify   *****************************************************/
	if (UcMode) {	/* write */
		RamRead32A(  SMA_COEFF_TGAIN0BIAS , &UlT0 ) ;
		RamRead32A(  SMA_COEFF_TGAIN1BIAS , &UlT1 ) ;
		RamRead32A(  SMA_COEFF_TGAIN2BIAS , &UlT2 ) ;
		RamRead32A(  SMA_COEFF_TGAIN3BIAS , &UlT3 ) ;
		RamRead32A(  SMA_KELVIN , &UlTEMP ) ;

		UlMAT0[ SMA_BIAS0 ] = (UlT1 & 0xFFFF0000) | (UlT0 >> 16) ;	// BIAS1[31:16] / BIAS0[15:0]
		UlMAT0[ SMA_BIAS2 ] = (UlT3 & 0xFFFF0000) | (UlT2 >> 16) ;	// BIAS3[31:16] / BIAS2[15:0]
		UlMAT0[ SMA_TEMP  ] = UlTEMP ;
	} else {
		UlMAT0[ SMA_BIAS0 ] = 0xFFFFFFFF ;
		UlMAT0[ SMA_BIAS2 ] = 0xFFFFFFFF ;
		UlMAT0[ SMA_TEMP  ] = 0xFFFFFFFF ;
	}
	/* calcurate check sum ******************************************/
	UsCkVal = MkInfMATsum( UlMAT0 ) ;
	UlMAT0[MAT0_CKSM] &= (UINT_32)0xFFFF0000 ;
	UlMAT0[MAT0_CKSM] |= (UINT_32)UsCkVal ;

	/* update ******************************************************/
	ans = WrInfMAT( INF_MAT0, UlMAT0, 64 ) ;
	if( ans != 0 ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 3 ) ;							// 0 - 63
	}
	/* Verify ******************************************************/
	UsCkVal_Bk = UsCkVal;
	ans =FlashMultiRead( INF_MAT0, 0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 4 );
	}
	UsCkVal = MkInfMATsum( UlMAT0 ) ;

CAM_ERR(CAM_OIS,  "[RVAL]:[BVal]=[%04x]:[%04x]\n",UsCkVal, UsCkVal_Bk );
	if( UsCkVal != UsCkVal_Bk ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return(5);
	}
CAM_ERR(CAM_OIS,  "WrWireBias____COMPLETE\n" );
	IOWrite32A( 0xE0701C , 0x00000002 ) ;
	return(0);
}

//********************************************************************************
// Function Name 	: ErCalData
// Retun Value		: 0:OK, 1:NG
// Argment Value	: UsFlag	HALL_CALB_FLG | GYRO_GAIN_FLG | AF_OIS_XTLK_FLG | FOCL_GAIN_FLG
//					:			CLAF_CALB_FLG | HLLN_CALB_FLG | CROS_TALK_FLG
// Explanation		: Erase each calibration data function
// History			: First edition
//********************************************************************************
UINT_8	ErCalData( UINT_16 UsFlag )
{
	UINT_32	UlMAT0[64];
	UINT_8 ans = 0;
	UINT_16	UsCkVal,UsCkVal_Bk ;

	IOWrite32A( 0xE0701C , 0x00000000 ) ;

CAM_ERR(CAM_OIS,  "ErCalData \n" );
	/* Back up ******************************************************/
	/* Erase   ******************************************************/
	ans = RdErInfMAT( INF_MAT0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( ans );
	}
#if DEBUG
	UINT_8	i;
	for(i=0;i<64;i++){
CAM_ERR(CAM_OIS,  "[ %d ] = %08x\n", i, UlMAT0[i] );
	}
#endif //DEBUG == 1
	/* modify   *****************************************************/
	UlMAT0[ SMA_CALIBRATION_STATUS ] |= (UINT_32)UsFlag ;
	// Erase hall calibration data
	if ( UsFlag & HALL_CALB_FLG ) {
		UlMAT0[ SMA_HALL_BIAS_OFFSET_X		 ]	= 0xFFFFFFFF ;
		UlMAT0[ SMA_HALL_BIAS_OFFSET_Y		 ]	= 0xFFFFFFFF ;
		UlMAT0[ SMA_HALL_BIAS_OFFSET_Z		 ]	= 0xFFFFFFFF ;
		UlMAT0[ SMA_LOOPGAIN_LENS_OFFSET_X	 ]	= 0xFFFFFFFF ;
		UlMAT0[ SMA_LOOPGAIN_LENS_OFFSET_Y	 ]	= 0xFFFFFFFF ;
		UlMAT0[ SMA_LOOPGAIN_LENS_OFFSET_Z	 ]	= 0xFFFFFFFF ;
		UlMAT0[ SMA_HALL_AMPLITUDE_X		 ]	= 0xFFFFFFFF ;
		UlMAT0[ SMA_HALL_AMPLITUDE_Y		 ]	= 0xFFFFFFFF ;
		UlMAT0[ SMA_HALL_AMPLITUDE_Z		 ]	= 0xFFFFFFFF ;
		UlMAT0[ ACCL_GYRO_OFFSET_X			 ]	= 0xFFFFFFFF ;
		UlMAT0[ ACCL_GYRO_OFFSET_Y			 ]	= 0xFFFFFFFF ;
		UlMAT0[ ACCL_GYRO_OFFSET_Z			 ]	= 0xFFFFFFFF ;
		UlMAT0[ SMA_BIAS0					 ]	= 0xFFFFFFFF ;
		UlMAT0[ SMA_BIAS2					 ]	= 0xFFFFFFFF ;
	}

	// Erase gyro gain calibration data
	if ( UsFlag & GYRO_GAIN_FLG ) {
		UlMAT0[ SMA_GYRO_GAIN_X		 ]	= 0xFFFFFFFF ;
		UlMAT0[ SMA_GYRO_GAIN_Y		 ]	= 0xFFFFFFFF ;
	}

	// Erase close AF calibration data
	if ( UsFlag & CLAF_CALB_FLG ) {
		UlMAT0[ SMA_HALL_BIAS_OFFSET_Z	 ]	= 0xFFFFFFFF ;
		UlMAT0[ SMA_LOOPGAIN_LENS_OFFSET_Z	 ]	= 0xFFFFFFFF ;
		UlMAT0[ SMA_HALL_AMPLITUDE_Z	 ]	= 0xFFFFFFFF ;
	}

	// Erase linearity calibration data
	if ( UsFlag & HLLN_CALB_FLG ) {
		UlMAT0[ SMA_LN_POS1			 ]	= 0xFFFFFFFF ;
		UlMAT0[ SMA_LN_POS2			 ]	= 0xFFFFFFFF ;
		UlMAT0[ SMA_LN_POS3			 ]	= 0xFFFFFFFF ;
		UlMAT0[ SMA_LN_POS4			 ]	= 0xFFFFFFFF ;
		UlMAT0[ SMA_LN_POS5			 ]	= 0xFFFFFFFF ;
		UlMAT0[ SMA_LN_POS6			 ]	= 0xFFFFFFFF ;
		UlMAT0[ SMA_LN_POS7			 ]	= 0xFFFFFFFF ;
		UlMAT0[ SMA_LN_STEP			 ]	= 0xFFFFFFFF ;
	}

	// Erase gyro cross talk calibration data
	if ( UsFlag & CROS_TALK_FLG ) {
		UlMAT0[ SMA_MIXING_X		 ] 	= 0xFFFFFFFF ;
		UlMAT0[ SMA_MIXING_Y		 ] 	= 0xFFFFFFFF ;
		UlMAT0[ SMA_MIXING_SFT		 ] 	= 0xFFFFFFFF ;
		UlMAT0[ SMA_MIXING_2ND		 ] 	= 0xFFFFFFFF ;
	}

#ifdef	SEL_SHIFT_COR
		// Erase accel offset calibration data
		if ( UsFlag & ACCL_OFST_FLG ) {
			UlMAT0[ ACCL_GYRO_OFFSET_X		 ] 	|= 0xFFFF0000 ;
			UlMAT0[ ACCL_GYRO_OFFSET_Y		 ] 	|= 0xFFFF0000 ;
			UlMAT0[ ACCL_GYRO_OFFSET_Z		 ] 	|= 0xFFFF0000 ;
		}
#endif
	/* calcurate check sum ******************************************/
	UsCkVal = MkInfMATsum( UlMAT0 ) ;
	UlMAT0[MAT0_CKSM] &= (UINT_32)0xFFFF0000 ;
	UlMAT0[MAT0_CKSM] |= (UINT_32)UsCkVal ;

	/* update ******************************************************/
	ans = WrInfMAT( INF_MAT0, UlMAT0, 64 ) ;
	if( ans != 0 ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 3 ) ;							// 0 - 63
	}
	/* Verify ******************************************************/
	UsCkVal_Bk = UsCkVal;
	ans =FlashMultiRead( INF_MAT0, 0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 4 );
	}
	UsCkVal = MkInfMATsum( UlMAT0 ) ;

CAM_ERR(CAM_OIS,  "[RVAL]:[BVal]=[%04x]:[%04x]\n",UsCkVal, UsCkVal_Bk );
	if( UsCkVal != UsCkVal_Bk )	{
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return(5);
	}
CAM_ERR(CAM_OIS,  "ErCalData____COMPLETE\n" );
	IOWrite32A( 0xE0701C , 0x00000002 ) ;
	return(0);
}


//********************************************************************************
// Function Name 	: WrLinCalData
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Flash write Hall Linearity data
//********************************************************************************
UINT_8 WrLinCalData( UINT_8 UcMode, mlLinearityValue *linval )
{
	UINT_32	UlMAT0[64];
	UINT_8 ans = 0;
	UINT_16	UsCkVal,UsCkVal_Bk ;
	double		*pPosX, *pPosY;
	UINT_32	PosDifX, PosDifY;

	IOWrite32A( 0xE0701C , 0x00000000 ) ;

CAM_ERR(CAM_OIS,  "WrLinCalData : Mode = %d\n", UcMode);
	/* Back up ******************************************************/
	/* Erase   ******************************************************/
	ans = RdErInfMAT( INF_MAT0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( ans );
	}
#if DEBUG
	UINT_8	i;
	for(i=0;i<64;i++){
CAM_ERR(CAM_OIS,  "[ %d ] = %08x\n", i, UlMAT0[i] );
	}
#endif //DEBUG == 1
	/* modify   *****************************************************/
	if( UcMode ){	// write
#if (SLT_XY_SWAP ==1)
		pPosX = linval->positionY;
		pPosY = linval->positionX;
#else
		pPosX = linval->positionX;
		pPosY = linval->positionY;
#endif
		UlMAT0[SMA_CALIBRATION_STATUS] &= ~( HLLN_CALB_FLG );
		UlMAT0[SMA_LN_POS1] = (UINT_32)((UINT_32)(*pPosY++ * 10)<<16) | (UINT_32)(*pPosX++ * 10);
		UlMAT0[SMA_LN_POS2] = (UINT_32)((UINT_32)(*pPosY++ * 10)<<16) | (UINT_32)(*pPosX++ * 10);
		UlMAT0[SMA_LN_POS3] = (UINT_32)((UINT_32)(*pPosY++ * 10)<<16) | (UINT_32)(*pPosX++ * 10);
		UlMAT0[SMA_LN_POS4] = (UINT_32)((UINT_32)(*pPosY++ * 10)<<16) | (UINT_32)(*pPosX++ * 10);
		UlMAT0[SMA_LN_POS5] = (UINT_32)((UINT_32)(*pPosY++ * 10)<<16) | (UINT_32)(*pPosX++ * 10);
		UlMAT0[SMA_LN_POS6] = (UINT_32)((UINT_32)(*pPosY++ * 10)<<16) | (UINT_32)(*pPosX++ * 10);
		UlMAT0[SMA_LN_POS7] = (UINT_32)((UINT_32)(*pPosY++ * 10)<<16) | (UINT_32)(*pPosX++ * 10);
#if (SLT_XY_SWAP ==1)
		PosDifX = (linval->dacY[4]>>16);
		PosDifY = (linval->dacX[4]>>16);
#else
		PosDifX = (linval->dacX[4]>>16);
		PosDifY = (linval->dacY[4]>>16);
#endif

		UlMAT0[SMA_LN_STEP] = (UINT_32)(((UINT_32)PosDifY << 16) | (UINT_32)(PosDifX & 0xFFFF) );
	}else{
		UlMAT0[SMA_CALIBRATION_STATUS] |= HLLN_CALB_FLG;
		UlMAT0[SMA_LN_POS1] = 0xFFFFFFFF;
		UlMAT0[SMA_LN_POS2] = 0xFFFFFFFF;
		UlMAT0[SMA_LN_POS3] = 0xFFFFFFFF;
		UlMAT0[SMA_LN_POS4] = 0xFFFFFFFF;
		UlMAT0[SMA_LN_POS5] = 0xFFFFFFFF;
		UlMAT0[SMA_LN_POS6] = 0xFFFFFFFF;
		UlMAT0[SMA_LN_POS7] = 0xFFFFFFFF;
		UlMAT0[SMA_LN_STEP] = 0xFFFFFFFF;
	}
	/* calcurate check sum ******************************************/
	UsCkVal = MkInfMATsum( UlMAT0 ) ;
	UlMAT0[MAT0_CKSM] &= (UINT_32)0xFFFF0000 ;
	UlMAT0[MAT0_CKSM] |= (UINT_32)UsCkVal ;

	/* update ******************************************************/
	ans = WrInfMAT( INF_MAT0, UlMAT0, 64 ) ;
	if( ans != 0 ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 3 ) ;							// 0 - 63
	}
	/* Verify ******************************************************/
	UsCkVal_Bk = UsCkVal;
	ans =FlashMultiRead( INF_MAT0, 0, UlMAT0, 32 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 4 );
	}
	UsCkVal = MkInfMATsum( UlMAT0 ) ;

CAM_ERR(CAM_OIS,  "[RVAL]:[BVal]=[%04x]:[%04x]\n",UsCkVal, UsCkVal_Bk );
	if( UsCkVal != UsCkVal_Bk ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return(5);
	}
CAM_ERR(CAM_OIS,  "WrLinCalData____COMPLETE\n" );
	IOWrite32A( 0xE0701C , 0x00000002 ) ;
	return(0);
}

//********************************************************************************
// Function Name 	: WrLinMixCalData
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Flash write cross talk & Linearity data
//********************************************************************************
UINT_8 WrLinMixCalData( UINT_8 UcMode, mlMixingValue *mixval , mlLinearityValue *linval  )
{
	UINT_32	UlMAT0[64];
	UINT_8 ans = 0;
	UINT_16	UsCkVal,UsCkVal_Bk ;
	double		*pPosX, *pPosY;
	UINT_32	PosDifX, PosDifY;
	DSPVER Info;
	INT_32	hx45x, hx45y, hy45y, hy45x;

	// Select parameter
	if( GetInfomation( &Info ) != 0){
		return( FAILURE );
	}
	
	IOWrite32A( 0xE0701C , 0x00000000 ) ;

CAM_ERR(CAM_OIS,  "WrLinMixCalData : Mode = %d\n", UcMode);
	/* Back up ******************************************************/
	/* Erase   ******************************************************/
	ans = RdErInfMAT( INF_MAT0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( ans );
	}
#if DEBUG
	UINT_8	i;
	for(i=0;i<32;i++){
CAM_ERR(CAM_OIS,  "[ %d ] = %08x\n", i, UlMAT0[i] );
	}
#endif //DEBUG == 1
	/* modify   *****************************************************/
	if( UcMode ){	// write
#if (SLT_XY_SWAP ==1)
		pPosX = linval->positionY;
		pPosY = linval->positionX;
#else
		pPosX = linval->positionX;
		pPosY = linval->positionY;
#endif
		UlMAT0[SMA_CALIBRATION_STATUS] &= ~( HLLN_CALB_FLG | CROS_TALK_FLG );
		UlMAT0[SMA_LN_POS1] = (UINT_32)((UINT_32)(*pPosY++ * 10)<<16) | (UINT_32)(*pPosX++ * 10);
		UlMAT0[SMA_LN_POS2] = (UINT_32)((UINT_32)(*pPosY++ * 10)<<16) | (UINT_32)(*pPosX++ * 10);
		UlMAT0[SMA_LN_POS3] = (UINT_32)((UINT_32)(*pPosY++ * 10)<<16) | (UINT_32)(*pPosX++ * 10);
		UlMAT0[SMA_LN_POS4] = (UINT_32)((UINT_32)(*pPosY++ * 10)<<16) | (UINT_32)(*pPosX++ * 10);
		UlMAT0[SMA_LN_POS5] = (UINT_32)((UINT_32)(*pPosY++ * 10)<<16) | (UINT_32)(*pPosX++ * 10);
		UlMAT0[SMA_LN_POS6] = (UINT_32)((UINT_32)(*pPosY++ * 10)<<16) | (UINT_32)(*pPosX++ * 10);
		UlMAT0[SMA_LN_POS7] = (UINT_32)((UINT_32)(*pPosY++ * 10)<<16) | (UINT_32)(*pPosX++ * 10);
#if (SLT_XY_SWAP ==1)
		PosDifX = (linval->dacY[4]>>16);
		PosDifY = (linval->dacX[4]>>16);
#else
		PosDifX = (linval->dacX[4]>>16);
		PosDifY = (linval->dacY[4]>>16);
#endif
		UlMAT0[SMA_LN_STEP] = (UINT_32)(((UINT_32)PosDifY << 16) | (UINT_32)(PosDifX & 0xFFFF));


		hx45x = mixval->hx45xL ;
		hx45y = mixval->hx45yL * (-1) * (SMA_LinMixParameter.SltDirX * SMA_LinMixParameter.SltDirY) ;
		hy45y = mixval->hy45yL ;
		hy45x = mixval->hy45xL * (-1) * (SMA_LinMixParameter.SltDirX * SMA_LinMixParameter.SltDirY) ;

	//****************************************************//

		UlMAT0[SMA_MIXING_X] = (UINT_32)((hx45y & 0xFFFF0000) | (( hx45x >> 16) & 0x0000FFFF));
		UlMAT0[SMA_MIXING_Y] = (UINT_32)((hy45x & 0xFFFF0000) | (( hy45y >> 16) & 0x0000FFFF));
		UlMAT0[SMA_MIXING_SFT] = (UINT_32)((((UINT_32)mixval->hysx << 8) & 0x0000FF00) | ((UINT_32)mixval->hxsx & 0x000000FF ));

	}else{
		UlMAT0[SMA_CALIBRATION_STATUS] |= ( HLLN_CALB_FLG | CROS_TALK_FLG );
		UlMAT0[SMA_LN_POS1] = 0xFFFFFFFF;
		UlMAT0[SMA_LN_POS2] = 0xFFFFFFFF;
		UlMAT0[SMA_LN_POS3] = 0xFFFFFFFF;
		UlMAT0[SMA_LN_POS4] = 0xFFFFFFFF;
		UlMAT0[SMA_LN_POS5] = 0xFFFFFFFF;
		UlMAT0[SMA_LN_POS6] = 0xFFFFFFFF;
		UlMAT0[SMA_LN_POS7] = 0xFFFFFFFF;
		UlMAT0[SMA_LN_STEP] = 0xFFFFFFFF;
		UlMAT0[SMA_MIXING_X] = 0xFFFFFFFF;
		UlMAT0[SMA_MIXING_Y] = 0xFFFFFFFF;
		UlMAT0[SMA_MIXING_SFT] = 0xFFFFFFFF;
	}
	/* calcurate check sum ******************************************/
	UsCkVal = MkInfMATsum( UlMAT0 ) ;
	UlMAT0[MAT0_CKSM] &= (UINT_32)0xFFFF0000 ;
	UlMAT0[MAT0_CKSM] |= (UINT_32)UsCkVal ;

	/* update ******************************************************/
	ans = WrInfMAT( INF_MAT0, UlMAT0, 64 ) ;
	if( ans != 0 ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 3 ) ;							// 0 - 63
	}
	/* Verify ******************************************************/
	UsCkVal_Bk = UsCkVal;
	ans =FlashMultiRead( INF_MAT0, 0, UlMAT0, 32 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 4 );
	}
	UsCkVal = MkInfMATsum( UlMAT0 ) ;

CAM_ERR(CAM_OIS,  "[RVAL]:[BVal]=[%04x]:[%04x]\n",UsCkVal, UsCkVal_Bk );
	if( UsCkVal != UsCkVal_Bk ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return(5);
	}
CAM_ERR(CAM_OIS,  "WrLinMixCalData____COMPLETE\n" );
	IOWrite32A( 0xE0701C , 0x00000002 ) ;
	return(0);
}

//********************************************************************************
// Function Name 	: WrLinMixCalData_LS
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Flash write cross talk & Linearity data
//********************************************************************************
UINT_8 WrLinMixCalData_LS( UINT_8 UcMode, mlMixingValue *mixval , mlLinearityValue *linval  )
{
	UINT_32	UlMAT0[64];
	UINT_8 ans = 0;
	UINT_16	UsCkVal,UsCkVal_Bk ;
	double		*pPosX, *pPosY;
	UINT_32	PosDifX, PosDifY;
	DSPVER Info;
	INT_32	hx45x, hx45y, hy45y, hy45x;

	// Select parameter
	if( GetInfomation( &Info ) != 0){
		return( FAILURE );
	}
	
	IOWrite32A( 0xE0701C , 0x00000000 ) ;

CAM_ERR(CAM_OIS,  "WrLinMixCalData for LS : Mode = %d\n", UcMode);
	/* Back up ******************************************************/
	/* Erase   ******************************************************/
	ans = RdErInfMAT( INF_MAT0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( ans );
	}
#if DEBUG
	UINT_8	i;
	for(i=0;i<32;i++){
CAM_ERR(CAM_OIS,  "[ %d ] = %08x\n", i, UlMAT0[i] );
	}
#endif //DEBUG == 1
	/* modify   *****************************************************/
	if( UcMode ){	// write
#if (SLT_XY_SWAP ==1)
		pPosX = linval->positionY;
		pPosY = linval->positionX;
#else
		pPosX = linval->positionX;
		pPosY = linval->positionY;
#endif
		UlMAT0[CALIBRATION_STATUS] &= ~( HLLN_CALB_FLG | CROS_TALK_FLG );
		UlMAT0[LN_POS1] = (UINT_32)((UINT_32)(*pPosY++ * 10)<<16) | (UINT_32)(*pPosX++ * 10);
		UlMAT0[LN_POS2] = (UINT_32)((UINT_32)(*pPosY++ * 10)<<16) | (UINT_32)(*pPosX++ * 10);
		UlMAT0[LN_POS3] = (UINT_32)((UINT_32)(*pPosY++ * 10)<<16) | (UINT_32)(*pPosX++ * 10);
		UlMAT0[LN_POS4] = (UINT_32)((UINT_32)(*pPosY++ * 10)<<16) | (UINT_32)(*pPosX++ * 10);
		UlMAT0[LN_POS5] = (UINT_32)((UINT_32)(*pPosY++ * 10)<<16) | (UINT_32)(*pPosX++ * 10);
		UlMAT0[LN_POS6] = (UINT_32)((UINT_32)(*pPosY++ * 10)<<16) | (UINT_32)(*pPosX++ * 10);
		UlMAT0[LN_POS7] = (UINT_32)((UINT_32)(*pPosY++ * 10)<<16) | (UINT_32)(*pPosX++ * 10);
#if (SLT_XY_SWAP ==1)
		PosDifX = (linval->dacY[4]>>16);
		PosDifY = (linval->dacX[4]>>16);
#else
		PosDifX = (linval->dacX[4]>>16);
		PosDifY = (linval->dacY[4]>>16);
#endif
		UlMAT0[LN_STEP] = (UINT_32)(((UINT_32)PosDifY << 16) | (UINT_32)(PosDifX & 0xFFFF));


		hx45x = mixval->hx45xL ;
		hx45y = mixval->hx45yL * (-1) * (SMA_LinMixParameter.SltDirX * SMA_LinMixParameter.SltDirY) ;
		hy45y = mixval->hy45yL ;
		hy45x = mixval->hy45xL * (-1) * (SMA_LinMixParameter.SltDirX * SMA_LinMixParameter.SltDirY) ;

	//****************************************************//

		UlMAT0[MIXING_X] = (UINT_32)((hx45y & 0xFFFF0000) | (( hx45x >> 16) & 0x0000FFFF));
		UlMAT0[MIXING_Y] = (UINT_32)((hy45x & 0xFFFF0000) | (( hy45y >> 16) & 0x0000FFFF));
		UlMAT0[MIXING_SFT] = (UINT_32)((((UINT_32)mixval->hysx << 8) & 0x0000FF00) | ((UINT_32)mixval->hxsx & 0x000000FF ));

	}else{
		UlMAT0[CALIBRATION_STATUS] |= ( HLLN_CALB_FLG | CROS_TALK_FLG );
		UlMAT0[LN_POS1] = 0xFFFFFFFF;
		UlMAT0[LN_POS2] = 0xFFFFFFFF;
		UlMAT0[LN_POS3] = 0xFFFFFFFF;
		UlMAT0[LN_POS4] = 0xFFFFFFFF;
		UlMAT0[LN_POS5] = 0xFFFFFFFF;
		UlMAT0[LN_POS6] = 0xFFFFFFFF;
		UlMAT0[LN_POS7] = 0xFFFFFFFF;
		UlMAT0[LN_STEP] = 0xFFFFFFFF;
		UlMAT0[MIXING_X] = 0xFFFFFFFF;
		UlMAT0[MIXING_Y] = 0xFFFFFFFF;
		UlMAT0[MIXING_SFT] = 0xFFFFFFFF;
	}
	/* calcurate check sum ******************************************/
	UsCkVal = MkInfMATsum( UlMAT0 ) ;
	UlMAT0[MAT0_CKSM] &= (UINT_32)0xFFFF0000 ;
	UlMAT0[MAT0_CKSM] |= (UINT_32)UsCkVal ;

	/* update ******************************************************/
	ans = WrInfMAT( INF_MAT0, UlMAT0, 64 ) ;
	if( ans != 0 ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 3 ) ;							// 0 - 63
	}
	/* Verify ******************************************************/
	UsCkVal_Bk = UsCkVal;
	ans =FlashMultiRead( INF_MAT0, 0, UlMAT0, 32 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 4 );
	}
	UsCkVal = MkInfMATsum( UlMAT0 ) ;

CAM_ERR(CAM_OIS,  "[RVAL]:[BVal]=[%04x]:[%04x]\n",UsCkVal, UsCkVal_Bk );
	if( UsCkVal != UsCkVal_Bk ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return(5);
	}
CAM_ERR(CAM_OIS,  "WrLinMixCalData____COMPLETE\n" );
	IOWrite32A( 0xE0701C , 0x00000002 ) ;
	return(0);
}

//********************************************************************************
// Function Name 	: WrGyroGainData
// Retun Value		: 0:OK, 1:NG
// Argment Value	: UcMode
// Explanation		: Flash write gyro gain data
//********************************************************************************
UINT_8 WrGyroGainData( UINT_8 UcMode )
{
	UINT_32	UlMAT0[64];
	UINT_32	UlReadGxzoom , UlReadGyzoom , UlReadGzzoom;
	UINT_8 ans = 0;
	UINT_16	UsCkVal,UsCkVal_Bk ;

	IOWrite32A( 0xE0701C , 0x00000000 ) ;

CAM_ERR(CAM_OIS,  "WrGyroGainData\n" );
	/* Back up ******************************************************/
	/* Erase   ******************************************************/
	ans = RdErInfMAT( INF_MAT0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( ans );
	}
#if DEBUG
	UINT_8	i;
	for(i=0;i<64;i++){
CAM_ERR(CAM_OIS,  "[ %d ] = %08x\n", i, UlMAT0[i] );
	}
#endif //DEBUG == 1
	/* modify   *****************************************************/
	if ( UcMode ) {	/* write */
		RamRead32A(  GyroFilterTableX_gxzoom , &UlReadGxzoom ) ;
		RamRead32A(  GyroFilterTableY_gyzoom , &UlReadGyzoom ) ;
		RamRead32A(  GyroFilterTableZ_gzzoom , &UlReadGzzoom ) ;

		UlMAT0[SMA_CALIBRATION_STATUS] &= ~( GYRO_GAIN_FLG );
		UlMAT0[SMA_GYRO_GAIN_Z] = UlReadGzzoom;
		UlMAT0[SMA_GYRO_GAIN_X] = UlReadGxzoom;
		UlMAT0[SMA_GYRO_GAIN_Y] = UlReadGyzoom;
	} else {
		UlMAT0[SMA_CALIBRATION_STATUS] |= GYRO_GAIN_FLG;
		UlMAT0[SMA_GYRO_GAIN_Z] = 0x3FFFFFFF;
		UlMAT0[SMA_GYRO_GAIN_X] = 0x3FFFFFFF;
		UlMAT0[SMA_GYRO_GAIN_Y] = 0x3FFFFFFF;
	}
	/* calcurate check sum ******************************************/
	UsCkVal = MkInfMATsum( UlMAT0 ) ;
	UlMAT0[MAT0_CKSM] &= (UINT_32)0xFFFF0000 ;
	UlMAT0[MAT0_CKSM] |= (UINT_32)UsCkVal ;

	/* update ******************************************************/
	ans = WrInfMAT( INF_MAT0, UlMAT0, 64 ) ;
	if( ans != 0 ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 3 ) ;							// 0 - 63
	}
	/* Verify ******************************************************/
	UsCkVal_Bk = UsCkVal;
	ans =FlashMultiRead( INF_MAT0, 0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 4 );
	}
	UsCkVal = MkInfMATsum( UlMAT0 ) ;

CAM_ERR(CAM_OIS,  "[RVAL]:[BVal]=[%04x]:[%04x]\n",UsCkVal, UsCkVal_Bk );
	if( UsCkVal != UsCkVal_Bk )	{
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return(5);
	}
CAM_ERR(CAM_OIS,  "WrGyroGainData____COMPLETE\n" );
	IOWrite32A( 0xE0701C , 0x00000002 ) ;
	return(0);
}

//********************************************************************************
// Function Name 	: WrGyroGainData_LS
// Retun Value		: 0:OK, 1:NG
// Argment Value	: UcMode
// Explanation		: Flash write gyro gain data
//********************************************************************************
UINT_8 WrGyroGainData_LS( UINT_8 UcMode )
{
	UINT_32	UlMAT0[64];
	UINT_32	UlReadGxzoom , UlReadGyzoom ;
	UINT_8 ans = 0;
	UINT_16	UsCkVal,UsCkVal_Bk ;

	IOWrite32A( 0xE0701C , 0x00000000 ) ;

CAM_ERR(CAM_OIS,  "WrGyroGainData for LS\n" );
	/* Back up ******************************************************/
	/* Erase   ******************************************************/
	ans = RdErInfMAT( INF_MAT0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( ans );
	}
#if DEBUG
	UINT_8	i;
	for(i=0;i<64;i++){
CAM_ERR(CAM_OIS,  "[ %d ] = %08x\n", i, UlMAT0[i] );
	}
#endif //DEBUG == 1
	/* modify   *****************************************************/
	if ( UcMode ) {	/* write */
		RamRead32A(  GyroFilterTableX_LS_gxzoom , &UlReadGxzoom ) ;
		RamRead32A(  GyroFilterTableY_LS_gyzoom , &UlReadGyzoom ) ;

		UlMAT0[CALIBRATION_STATUS] &= ~( GYRO_GAIN_FLG );
		UlMAT0[GYRO_GAIN_X] = UlReadGxzoom;
		UlMAT0[GYRO_GAIN_Y] = UlReadGyzoom;
	} else {
		UlMAT0[CALIBRATION_STATUS] |= GYRO_GAIN_FLG;
		UlMAT0[GYRO_GAIN_X] = 0x3FFFFFFF;
		UlMAT0[GYRO_GAIN_Y] = 0x3FFFFFFF;
	}
	/* calcurate check sum ******************************************/
	UsCkVal = MkInfMATsum( UlMAT0 ) ;
	UlMAT0[MAT0_CKSM] &= (UINT_32)0xFFFF0000 ;
	UlMAT0[MAT0_CKSM] |= (UINT_32)UsCkVal ;

	/* update ******************************************************/
	ans = WrInfMAT( INF_MAT0, UlMAT0, 64 ) ;
	if( ans != 0 ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 3 ) ;							// 0 - 63
	}
	/* Verify ******************************************************/
	UsCkVal_Bk = UsCkVal;
	ans =FlashMultiRead( INF_MAT0, 0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 4 );
	}
	UsCkVal = MkInfMATsum( UlMAT0 ) ;

CAM_ERR(CAM_OIS,  "[RVAL]:[BVal]=[%04x]:[%04x]\n",UsCkVal, UsCkVal_Bk );
	if( UsCkVal != UsCkVal_Bk )	{
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return(5);
	}
CAM_ERR(CAM_OIS,  "WrGyroGainData____COMPLETE\n" );
	IOWrite32A( 0xE0701C , 0x00000002 ) ;
	return(0);
}

#if 0
//********************************************************************************
// Function Name 	: WrOptCenerData
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Flash write gyro gain data
//********************************************************************************
UINT_8 WrOptCenerData( UINT_8 UcMode )
{
	UINT_32	UlMAT0[64];
	UINT_32	UlReadLensxoffset , UlReadLensyoffset;
	UINT_8 ans = 0;
	UINT_16	UsCkVal,UsCkVal_Bk ;

	IOWrite32A( 0xE0701C , 0x00000000 ) ;

CAM_ERR(CAM_OIS,  "WrOptCenerData\n" );
	/* Back up ******************************************************/
	/* Erase   ******************************************************/
	ans = RdErInfMAT( INF_MAT0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( ans );
	}
#if DEBUG
	UINT_8	i;
	for(i=0;i<64;i++){
CAM_ERR(CAM_OIS,  "[ %d ] = %08x\n", i, UlMAT0[i] );
	}
#endif //DEBUG == 1
	/* modify   *****************************************************/
	if( UcMode ){	// write
		RamRead32A(  OpticalOffset_X , &UlReadLensxoffset ) ;
		RamRead32A(  OpticalOffset_Y , &UlReadLensyoffset ) ;
		UlMAT0[SMA_LOOPGAIN_LENS_OFFSET_X] = (UINT_32)((UlReadLensxoffset & 0xFFFF0000) | (UlMAT0[SMA_LOOPGAIN_LENS_OFFSET_X] & 0x0000FFFF));
		UlMAT0[SMA_LOOPGAIN_LENS_OFFSET_Y] = (UINT_32)((UlReadLensyoffset & 0xFFFF0000) | (UlMAT0[SMA_LOOPGAIN_LENS_OFFSET_Y] & 0x0000FFFF));
	}else{
		UlMAT0[SMA_LOOPGAIN_LENS_OFFSET_X] = (UINT_32)(UlMAT0[SMA_LOOPGAIN_LENS_OFFSET_X] & 0x0000FFFF);
		UlMAT0[SMA_LOOPGAIN_LENS_OFFSET_Y] = (UINT_32)(UlMAT0[SMA_LOOPGAIN_LENS_OFFSET_Y] & 0x0000FFFF);
	}
	/* calcurate check sum ******************************************/
	UsCkVal = MkInfMATsum( UlMAT0 ) ;
	UlMAT0[MAT0_CKSM] &= (UINT_32)0xFFFF0000 ;
	UlMAT0[MAT0_CKSM] |= (UINT_32)UsCkVal ;

	/* update ******************************************************/
	ans = WrInfMAT( INF_MAT0, UlMAT0, 64 ) ;
	if( ans != 0 ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 3 ) ;							// 0 - 63
	}
	/* Verify ******************************************************/
	UsCkVal_Bk = UsCkVal;
	ans =FlashMultiRead( INF_MAT0, 0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 4 );
	}
	UsCkVal = MkInfMATsum( UlMAT0 ) ;

CAM_ERR(CAM_OIS,  "[RVAL]:[BVal]=[%04x]:[%04x]\n",UsCkVal, UsCkVal_Bk );
	if( UsCkVal != UsCkVal_Bk )	{
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return(5);
	}
CAM_ERR(CAM_OIS,  "WrOptCenerData____COMPLETE\n" );
	IOWrite32A( 0xE0701C , 0x00000002 ) ;
	return(0);
}
#endif

//********************************************************************************
// Function Name 	: LoadUareaToPM
// Retun Value		: NON
// Argment Value	: ptr, mode
// Explanation		: load user area data from Flash memory to PM
//********************************************************************************
UINT_8 LoadUareaToPM( CODE_TBL_EXT* ptr , UINT_8 mode )
{
	UINT_8 ans=0;
	UINT_32	UlReadVal=0;
	UINT_32	UlCnt=0;

	UINT_32	UlCver=0;
	UINT_8 ucSlaveAdr = 0x48;

	RamRead32A( 0x500F, (UINT_32 *)&UlReadVal );						// Pmem initialized check

	if( UlReadVal != PMEM_INITIALIZED ){

		IORead32A( SYSDSP_CVER , &UlCver );
//CAM_ERR(CAM_OIS, "1:SYSDSP_CVER[%08x]  \n", UlCver );

		if( UlCver == 0x0141  ){				// LC898129 ES1
			ucSlaveAdr = GetSlaveAddr();
		}

		BootMode();

		if( UlCver == 0x0141  ){				// LC898129 ES1
			// Set default slave address
			setI2cSlvAddr( DEFAULT_SLAVE_ADR, 2 );
			SetSlaveAddr( ucSlaveAdr );
		}

//IORead32A( SYSDSP_CVER , &UlCver );
//CAM_ERR(CAM_OIS, "2:SYSDSP_CVER[%08x]  \n", UlCver );

	 	ans = PmemUpdate129( 0, ptr );		// download the special program for updating the flash memory.
		if(ans != 0){
			return( ans );
		}
		RamWrite32A( 0xF007, 0x00000000 );		// boot command Access Setup
	}

	IOWrite32A( 0xE0701C , 0x00000000);
	RamWrite32A( 0x5004 , 0x00000000 );		// Transfer user data from flash memory to pm

	do{
		WitTim( 1 );
		if( UlCnt++ > 10 ) {
			IOWrite32A( 0xE0701C , 0x00000002);
			return (0x10) ;									// transfer error
		}
		RamRead32A( 0x5004, &UlReadVal );
	}while ( UlReadVal != 0 );
	IOWrite32A( 0xE0701C , 0x00000002);

	return( 0 );
}

//********************************************************************************
// Function Name 	: RdBurstUareaFromPM
// Retun Value		: NON
// Argment Value	: UlAddress, PucData, UcLength, mode
// Explanation		: Read user area data from program memory
//********************************************************************************
UINT_8	RdBurstUareaFromPM( UINT_32 UlAddress, UINT_8 *PucData , UINT_16 UsLength , UINT_8 mode )
{
	if( !UsLength )	return(0xff);
	if( !mode ){
		RamWrite32A( 0x5000 , UlAddress );
	}
	RamWrite32A( 0x5002 , (UINT_32)UsLength - 1 );
	WitTim( 1 ) ;													// 1ms Wait for prepare trans data
	CntRd( 0x5002 , PucData , (UINT_16)UsLength );

	return( 0 );
}
UINT_8	RdSingleUareaFromPM( UINT_32 UlAddress, UINT_8 *PucData , UINT_8 UcLength , UINT_8 mode )
{
	UINT_32	ReadData;
	UINT_8 i=0 ;
	if( !UcLength )	return(0xff);
	if( !mode ){
		RamWrite32A( 0x5000 , UlAddress );
	}
	while(i < UcLength)
	{
		RamRead32A( 0x5001 , &ReadData );
//CAM_ERR(CAM_OIS, "[%02x]%08x\n",i,ReadData);
		PucData[i++] = (UINT_8)(ReadData >> 24);
		PucData[i++] = (UINT_8)(ReadData >> 16);
		PucData[i++] = (UINT_8)(ReadData >> 8);
		PucData[i++] = (UINT_8)(ReadData >> 0);
	}

	return( 0 );
}

//********************************************************************************
// Function Name 	: WrUareaToPm
// Retun Value		: 0 : Success
// Argment Value	: UlAddress, PucData, UcLength, mode
// Explanation		: Write user area data to PM
//********************************************************************************
UINT_8	WrUareaToPm( UINT_32 UlAddress, UINT_8 *PucData , UINT_8 UcLength , UINT_8 mode )
{
	UINT_8	PucBuf[256];	// address2Byte + data254Byte
	UINT_8 i=0;
	if(!UcLength || UcLength > 254)	return(0xff);

	PucBuf[0] = 0x50;
	PucBuf[1] = 0x01;

	for(i=0 ; i<UcLength ; i++)
	{
		PucBuf[i+2] = PucData[i];
	}

	if( !mode ){	// start address set if mode = 0
		RamWrite32A( 0x5000 , UlAddress );
	}
	CntWrt( PucBuf , (UINT_16)UcLength + 2 );

	return( 0 );
}


//********************************************************************************
// Function Name 	: WrUareaToPmInt
// Retun Value		: 0 : Success
// Argment Value	: UlAddress, PuiData, UcLength, mode
// Explanation		: Write user area data to PM
//********************************************************************************
UINT_8	WrUareaToPmInt( UINT_32 UlAddress, UINT_32 *PulData , UINT_8 UcLength , UINT_8 mode )
{
	UINT_8	PucBuf[256];	// address2Byte + data254Byte
	UINT_8 i = 0;

	if(!UcLength || UcLength > 63)	return(0xff);

	PucBuf[0] = 0x50;
	PucBuf[1] = 0x01;

	for(i = 0; i < UcLength; i++ ) {
		PucBuf[i*4 + 2] = (UINT_8)(PulData[i] >> 0);
		PucBuf[i*4 + 3] = (UINT_8)(PulData[i] >> 8);
		PucBuf[i*4 + 4] = (UINT_8)(PulData[i] >> 16);
		PucBuf[i*4 + 5] = (UINT_8)(PulData[i] >> 24);
	}

	if( !mode ){	// start address set if mode = 0
		RamWrite32A( 0x5000 , UlAddress );
	}
	CntWrt( PucBuf , (UINT_16)UcLength * 4 + 2 );

	return( 0 );
}

//********************************************************************************
// Function Name 	: WrUareaToFlash
// Retun Value		: NON
// Argment Value	: chiperase
// Explanation		: Update user area data from PM to Flash memory
//********************************************************************************
UINT_8	WrUareaToFlash( void )
{
	UINT_32	UlReadVal;
	UINT_32	UlCntE=0;
	UINT_32	UlCntW=0;
	UINT_8	ans=0;

	ans	= UnlockCodeSet();
	if( ans != 0 )	return( ans ) ;							// Unlock Code Set
	WritePermission();									// Write Permission
	AdditionalUnlockCodeSet();							// Additional Unlock Code Set

	IOWrite32A( 0xE0701C , 0x00000000);		//
//	RamWrite32A( 0xF007, 0x00000000 );						// FlashAccess Setup

	RamWrite32A( 0x5005 , 0x00000000 );		// erase User area data on flash memory

	WitTim( 10 );
	do{
		WitTim( 1 );
		if( UlCntE++ > 20 ) {
			IOWrite32A( 0xE0701C , 0x00000002);
			if( UnlockCodeClear() != 0 ) 	return (0x11) ;				// unlock code clear error
			return (0x10) ;									// erase error
		}
		RamRead32A( 0x5005, &UlReadVal );
	}while ( UlReadVal != 0 );

	RamWrite32A( 0x5006 , 0x00000000 );			// write user area data from PM to flash memory

	WitTim( 300 );
	do{
		WitTim( 1 );
		if( UlCntW++ > 300 ) {
			IOWrite32A( 0xE0701C , 0x00000002);
			if( UnlockCodeClear() != 0 ) 	return (0x21) ;				// unlock code clear error
			return (0x20) ;									// write error
		}
		RamRead32A( 0x5006, &UlReadVal );
	}while ( UlReadVal != 0 );
	IOWrite32A( 0xE0701C , 0x00000002);
	if( UnlockCodeClear() != 0 ) 	return (0x31) ;				// unlock code clear error

CAM_ERR(CAM_OIS, "[E-CNT: %d] [W-CNT: %d]\n",UlCntE,UlCntW);
	return( 0 );
}


//********************************************************************************
// Function Name 	: CalcSetLinMixData   
// Retun Value		: 0:OK, 1:NG
// Argment Value	: UcMode	0:disable	1:enable
//					: mlMixingValue *mixval
//					: mlLinearityValue *linval
//					: uc_act_sel	0:SS	1:LS
// Explanation		: Flash write linearity correction data function
// History			: First edition
//********************************************************************************
UINT_8	CalcSetLinMixData( UINT_8 UcMode, stPixelCoordinate *pix, UINT_8 uc_act_sel )
{
	mlMixingValue		mixVal;
	mlLinearityValue	linVal;
	unsigned long	dacX[7];
	unsigned long	dacY[7];
	double	positionX[7];
	double	positionY[7];
	ADJ_LINEARITY_MIXING * LinMixPtr;
	int		i;
	signed long j;
	DSPVER Info;
	UINT_8 ans;

	const double N = 7;

#if 1	// Disable crosstalk
	double Xa = 0, Ya = 0;
  	double Xsum_xy = 0, Xsum_x = 0, Xsum_y = 0, Xsum_x2 = 0;
  	double Ysum_xy = 0, Ysum_x = 0, Ysum_y = 0, Ysum_x2 = 0;
#endif	// Disable crosstalk

	// Select parameter
	if( GetInfomation( &Info ) != 0){
		return( FAILURE );
	} else if( Info.ActVersion == 0x06 ) {
		LinMixPtr = (ADJ_LINEARITY_MIXING*)&SMA_LinMixParameter;
	} else if( Info.ActVersion == 0x07 ) {
		if( !uc_act_sel ) {
			LinMixPtr = (ADJ_LINEARITY_MIXING*)&SMA_LinMixParameter;
		} else {
			LinMixPtr = (ADJ_LINEARITY_MIXING*)&LS_LinMixParameter;
		}
	} else {
		return( FAILURE );	
	}
	
	
	j = (-3L);
	for( i = 0 ; i < N ; i++ ){
		dacX[ i ] = (signed long )( j * (LinMixPtr->SltDirX * LinMixPtr->SltOffsetX) ) <<16;
		dacY[ i ] = (signed long )( j * (LinMixPtr->SltDirY * LinMixPtr->SltOffsetY) ) <<16;
CAM_ERR(CAM_OIS,  " dac[%d] = [ %08x ] :[ %08x ]\n", i , dacX[ i ], dacY[ i ] );
		j++;
	}

#if 1	// Disable crosstalk
	// **************************************************
	// ç≈è¨2èÊñ@
	// **************************************************
	for (i=0; i<N; i++) {
		Xsum_xy += i * pix->XonXmove[i];
		Xsum_x  += i;
		Xsum_y  += pix->XonXmove[i];
		
		Ysum_xy += i * pix->YonXmove[i];
		Ysum_y  += pix->YonXmove[i];
	}
 	Xa = ((N * Ysum_xy) - (Xsum_x * Ysum_y)) / ((N * Xsum_xy) - (Xsum_x * Xsum_y));

  	Xsum_xy = Xsum_x = Xsum_y = Xsum_x2 = 0;
  	Ysum_xy = Ysum_x = Ysum_y = Ysum_x2 = 0;

	for (i=0; i<N; i++) {
		Xsum_xy += i * pix->XonYmove[i];
		Xsum_x  += i;
		Xsum_y  += pix->XonYmove[i];

		Ysum_xy += i * pix->YonYmove[i];
		Ysum_y  += pix->YonYmove[i];
	}
 	Ya = ((N * Xsum_xy) - (Xsum_x * Xsum_y)) / ((N * Ysum_xy) - (Xsum_x * Ysum_y));

CAM_ERR(CAM_OIS, "Xa = %f\n", Xa);
CAM_ERR(CAM_OIS, "Ya = %f\n", Ya);

	// **************************************************
	// MIXINGåWêîåvéZ
	// **************************************************
//CAM_ERR(CAM_OIS, "degreeX  = %f\n", rad_to_deg(-atan(Xa)) );
//CAM_ERR(CAM_OIS, "degreeY  = %f\n", rad_to_deg(+atan(Ya)) );

/*
	mixVal.radianX = -atan(Xa);
	mixVal.radianY = +atan(Ya);

	// 45degree rotation
	mixVal.radianX += deg_to_rad(-45.0);
	mixVal.radianY += deg_to_rad(-45.0);

CAM_ERR(CAM_OIS, "degreeX45= %f\n", rad_to_deg(mixVal.radianX) );
CAM_ERR(CAM_OIS, "degreeY45= %f\n", rad_to_deg(mixVal.radianY) );

	mixVal.hx45x = +(cos(mixVal.radianY) / cos(mixVal.radianX - mixVal.radianY));
    mixVal.hx45y = +(sin(mixVal.radianY) / cos(mixVal.radianX - mixVal.radianY));
	
    mixVal.hy45y = +(cos(mixVal.radianX) / cos(mixVal.radianX - mixVal.radianY));
    mixVal.hy45x = -(sin(mixVal.radianX) / cos(mixVal.radianX - mixVal.radianY));

	mixVal.hxsx = (unsigned char)abs(mixVal.hx45x);
	mixVal.hysx = (unsigned char)abs(mixVal.hy45y); 
    mixVal.hx45x = mixVal.hx45x / pow(2, (double)mixVal.hxsx);
    mixVal.hy45y = mixVal.hy45y / pow(2, (double)mixVal.hysx);
*/
CAM_ERR(CAM_OIS, "hx45x  = %f\n", mixVal.hx45x);
CAM_ERR(CAM_OIS, "hx45y  = %f\n", mixVal.hx45y);
CAM_ERR(CAM_OIS, "hy45y  = %f\n", mixVal.hy45y);
CAM_ERR(CAM_OIS, "hy45x  = %f\n", mixVal.hy45x);
CAM_ERR(CAM_OIS, "hxsx  = %02X\n", mixVal.hxsx);
CAM_ERR(CAM_OIS, "hysx  = %02X\n", mixVal.hysx);

	mixVal.hx45xL = (long)(mixVal.hx45x * 0x7FFFFFFF);
	mixVal.hx45yL = (long)(mixVal.hx45y * 0x7FFFFFFF);
	mixVal.hy45yL = (long)(mixVal.hy45y * 0x7FFFFFFF);
	mixVal.hy45xL = (long)(mixVal.hy45x * 0x7FFFFFFF);

CAM_ERR(CAM_OIS, "hx45xL  = %08X\n", mixVal.hx45xL);
CAM_ERR(CAM_OIS, "hx45yL  = %08X\n", mixVal.hx45yL);
CAM_ERR(CAM_OIS, "hy45yL  = %08X\n", mixVal.hy45yL);
CAM_ERR(CAM_OIS, "hy45xL  = %08X\n", mixVal.hy45xL);
#else	// Disable crosstalk
	mixVal.hx45xL = (long)0x7FFFFFFF;
	mixVal.hx45yL = (long)0x00000000;
	mixVal.hy45yL = (long)0x7FFFFFFF;
	mixVal.hy45xL = (long)0x00000000;
	mixVal.hxsx = 0;
	mixVal.hysx = 0; 
#endif	// Disable crosstalk

	// setup parameters
	linVal.measurecount = N;

	linVal.dacX = (unsigned long *)&dacX;
	linVal.dacY = (unsigned long *)&dacY;
	linVal.positionX = (double *)&positionX;
	linVal.positionY = (double *)&positionY;

	if( !uc_act_sel ) {
		for( i = 0; i < N; i++ ) {
			// exchange axis data
			positionY[i] = pix->XonXmove[i];
			positionX[i] = pix->YonYmove[i];
		}

		ans = WrLinMixCalData( UcMode, &mixVal , &linVal );
	} else {
		for( i = 0; i < N; i++ ) {
			positionX[i] = pix->XonXmove[i];
			positionY[i] = pix->YonYmove[i];
		}
		ans = WrLinMixCalData_LS( UcMode, &mixVal , &linVal );
	}
	
//	Reboot
	RamWrite32A( CMD_IO_ADR_ACCESS , SYSDSP_REMAP ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS , 0x00001001 ) ;
	WitTim( 25 ) ;	
	
	while( RdStatus( 0 ) ){ 
		if(++i > 0x100) break;
	};

	return( ans );
}
