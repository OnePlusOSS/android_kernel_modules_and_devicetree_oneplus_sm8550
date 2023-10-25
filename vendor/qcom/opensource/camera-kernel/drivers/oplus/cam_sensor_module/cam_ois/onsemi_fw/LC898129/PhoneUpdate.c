/**
 * @brief		LC898129 Flash update
 *
 * @author		(C) 2019 ON Semiconductor.
 * @file		PhoneUpdate.c
 * @date		svn:$Date:: 2021-05-19 17:20:17 +0900#$
 * @version		svn:$Revision: 46 $
 * @attention
 *
 **/
//**************************
//	Include Header File
//**************************
#include	"PhoneUpdate.h"
#include	"UpdataCode129.h"
//#include	<stdlib.h>
//#include	<math.h>
#include <linux/kernel.h>
#include <linux/slab.h>

//#include	"FromCode_07_01.h"

/* Actuator calibration parameters */
//#include 	"Calibration_TAH1120.h"

#define	FLASH_BLOCKS			14
#define	USER_RESERVE			2
#define	ERASE_BLOCKS			(FLASH_BLOCKS - USER_RESERVE)

#define BURST_LENGTH_UC 		( 120 )

#define BURST_LENGTH_FC 		( 64 )

#define	PMEM_INITIALIZED		(0x706D656D)

#define 	DEBUG 0
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
//	extern  Function LIST
//**************************

//**************************
//	Table of download file
//**************************
CODE_TBL_EXT CdTbl[] = {
	{0x0007, CcUpdataCode129, UpDataCodeSize,  UpDataCodeCheckSum, (void*)0,				  0, 0, 0 },
	{0xFFFF, (void*)0,		  0,			   0,				   (void*)0,				  0, 0, 0 }
};

//**************************
//	Local Function Prototype
//**************************
UINT_8	FlashBlockErase( UINT_8 SelMat , UINT_32 SetAddress );

//********************************************************************************
// Function Name 	: IOWrite32A
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
//********************************************************************************
void BootMode( void )
{
	UINT_32	ReadVal;

	IORead32A( SYSDSP_REMAP, &ReadVal ) ;

	if( ReadVal == 0x000000A1 ) {
		RamWrite32A( 0xF019, 0x00000000 ) ;// LC898402 Communication Stop
		WitTim( 1 );// Wait 1ms
	}
	ReadVal = (ReadVal & 0x1) | 0x00001400 ;
	IOWrite32A( SYSDSP_REMAP, ReadVal ) ;
	WitTim( 15 ) ;
}

//********************************************************************************
// Function Name 	: UnlockCodeSet
//********************************************************************************
UINT_8 UnlockCodeSet( void )
{
	UINT_32 UlReadVal, UlCnt=0 ;

	do {
		IOWrite32A( 0xE07554, 0xAAAAAAAA ) ;
		IOWrite32A( 0xE07AA8, 0x55555555 ) ;
		IORead32A( 0xE07014, &UlReadVal ) ;
		if( (UlReadVal & 0x00000080) != 0 )	return ( 0 ) ;
		WitTim( 1 ) ;
	} while( UlCnt++ < 10 ) ;
	return ( 1 );
}

//********************************************************************************
// Function Name 	: UnlockCodeClear
//********************************************************************************
UINT_8 UnlockCodeClear(void)
{
	UINT_32 UlDataVal, UlCnt=0 ;

	do {
		IOWrite32A( 0xE07014, 0x00000010 ) ;
		IORead32A( 0xE07014, &UlDataVal ) ;
		if( (UlDataVal & 0x00000080) == 0 )	return ( 0 ) ;
		WitTim( 1 ) ;
	} while( UlCnt++ < 10 ) ;
	return ( 3 ) ;
}

//********************************************************************************
// Function Name 	: WritePermission
//********************************************************************************
void WritePermission( void )
{
	IOWrite32A( 0xE074CC, 0x00000001 ) ;
	IOWrite32A( 0xE07664, 0x00000010 ) ;
}

//********************************************************************************
// Function Name 	: AdditionalUnlockCodeSet
//********************************************************************************
void AdditionalUnlockCodeSet( void )
{
	IOWrite32A( 0xE07CCC, 0x0000ACD5 ) ;
}

//********************************************************************************
// Function Name 	: PmemUpdate129
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
		NcDataVal = ptr->FromCode + 32;
		SizeofCode = (UINT_16)ptr->FromCode[9] << 8 | (UINT_16)ptr->FromCode[8];

		CheckSumCode = (long long)ptr->FromCode[19] << 56 | (long long)ptr->FromCode[18] << 48 | (long long)ptr->FromCode[17] << 40 | (long long)ptr->FromCode[16] << 32
					 | (long long)ptr->FromCode[15] << 24 | (long long)ptr->FromCode[14] << 16 | (long long)ptr->FromCode[13] << 8 | (long long)ptr->FromCode[12];

		SizeofCheck = SizeofCode;
	} else {
		NcDataVal = ptr->UpdataCode;
		SizeofCode = ptr->SizeUpdataCode;
		CheckSumCode = ptr->SizeUpdataCodeCksm;
		SizeofCheck = SizeofCode;
	}
	p = (UINT_8 *)&CheckSumCode;

//--------------------------------------------------------------------------------
// 1.
//--------------------------------------------------------------------------------
	RamWrite32A( 0x3000, 0x00080000 );

	data[0] = 0x40;
	data[1] = 0x00;

	Remainder = ( (SizeofCode * 5) / BURST_LENGTH_UC );
	for(i=0 ; i< Remainder ; i++)
	{
		UlNum = 2;
		for(j=0 ; j < BURST_LENGTH_UC; j++){
			data[UlNum] =  *NcDataVal++;
			UlNum++;
		}

		CntWrt( data, BURST_LENGTH_UC + 2 );
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
// 2.
//--------------------------------------------------------------------------------
	data[0] = 0xF0;
	data[1] = 0x0E;
	data[2] = (unsigned char)((SizeofCheck >> 8) & 0x000000FF);
	data[3] = (unsigned char)(SizeofCheck & 0x000000FF);
	data[4] = 0x00;
	data[5] = 0x00;

	CntWrt( data, 6 ) ;

	UlCnt = 0;
	do{
		WitTim( 1 );
		if( UlCnt++ > 10 ) {
			return (0x21) ;
		}
		RamRead32A( 0x0088, &UlReadVal );
	}while ( UlReadVal != 0 );

	CntRd( 0xF00E, ReadData , 8 );

	for( i=0; i<8; i++) {
		if(ReadData[7-i] != *p++ ) {
			return (0x22) ;
		}
	}
	if( dist != 0 ){
		RamWrite32A( 0xF001, 0 );
	}

	return( 0 );
}

//********************************************************************************
// Function Name 	: EraseInfoMat129
//********************************************************************************
UINT_8 EraseInfoMat129( UINT_8 SelMat )
{
	UINT_8 ans = 0;

	if( SelMat != INF_MAT0 && SelMat != INF_MAT1 && SelMat != INF_MAT2  )	return 10;

	IOWrite32A( 0xE0701C , 0x00000000);
	ans = FlashBlockErase( SelMat, 0 ) ;
	IOWrite32A( 0xE0701C , 0x00000002);
	return( ans );
}

//********************************************************************************
// Function Name 	: EraseUserMat129
//********************************************************************************
UINT_8 EraseUserMat129( UINT_8 StartBlock, UINT_8 EndBlock )
{
	UINT_32 i ;
	UINT_32	UlReadVal, UlCnt ;

	IOWrite32A( 0xE0701C , 0x00000000 ) ;
	RamWrite32A( 0xF007, 0x00000000 ) ;

	for( i = StartBlock; i < EndBlock; i++ ) {
		RamWrite32A( 0xF00A, ( i << 10 ) ) ;
		RamWrite32A( 0xF00C, 0x00000020 ) ;

		WitTim( 5 ) ;
		UlCnt = 0 ;
		do {
			WitTim( 5 ) ;
			if( UlCnt++ > 100 ) {
				IOWrite32A( 0xE0701C , 0x00000002 ) ;
				return ( 0x31 ) ;
			}
			RamRead32A( 0xF00C, &UlReadVal ) ;
		}while ( UlReadVal != 0 ) ;
	}
	IOWrite32A( 0xE0701C , 0x00000002 ) ;
	return(0);
}

//********************************************************************************
// Function Name 	: ProgramFlash129_Standard
//********************************************************************************
UINT_8 ProgramFlash129_Standard( CODE_TBL_EXT *ptr )
{
	UINT_32	UlReadVal, UlCnt , UlNum ;
	UINT_8	data[ ( BURST_LENGTH_FC + 3 ) ] ;
	UINT_32 i, j ;

	const UINT_8 *NcFromVal = ptr->FromCode + 64 ;
	const UINT_8 *NcFromVal1st = ptr->FromCode ;
	UINT_8 UcOddEvn = 0;

	IOWrite32A( 0xE0701C, 0x00000000 );
	RamWrite32A( 0xF007, 0x00000000 );
	RamWrite32A( 0xF00A, 0x00000010 );
	data[ 0 ] = 0xF0;
	data[ 1 ] = 0x08;
	data[ 2 ] = 0x00;

	for( i = 1; i < ( ptr->SizeFromCode / 64 ); i++ ) {
		if( ++UcOddEvn > 1 )	UcOddEvn = 0 ;
		if( UcOddEvn == 0 )		data[ 1 ] = 0x08 ;
		else					data[ 1 ] = 0x09 ;

#if (BURST_LENGTH_FC == 32)
		data[ 2 ] = 0x00 ;
		UlNum = 3 ;
		for( j = 0; j < BURST_LENGTH_FC; j++ ) {
			data[ UlNum++ ] = *NcFromVal++ ;
		}
		CntWrt( data, BURST_LENGTH_FC + 3 ) ;

	  	data[ 2 ] = 0x20 ;
		UlNum = 3 ;
		for( j = 0; j < BURST_LENGTH_FC; j++ ) {
			data[ UlNum++ ] = *NcFromVal++ ;
		}
		CntWrt( data, BURST_LENGTH_FC + 3 ) ;

#elif (BURST_LENGTH_FC == 64)
		UlNum = 3 ;
		for( j = 0; j < BURST_LENGTH_FC; j++ ) {
			data[ UlNum++ ] = *NcFromVal++ ;
		}
		CntWrt( data, BURST_LENGTH_FC + 3 ) ;
#endif

		RamWrite32A( 0xF00B, 0x00000010 ) ;
		UlCnt = 0 ;
		if( UcOddEvn == 0 ) {
			do {
				WitTim( 1 );
				RamRead32A( 0xF00C, &UlReadVal ) ;
				if( UlCnt++ > 250 ) {
					IOWrite32A( 0xE0701C, 0x00000002 ) ;
					return ( 0x41 ) ;
				}
			} while ( UlReadVal != 0 ) ;
		 	RamWrite32A( 0xF00C, 0x00000004 ) ;
		} else {
			do {
				WitTim( 1 );
				RamRead32A( 0xF00C, &UlReadVal ) ;
				if( UlCnt++ > 250 ) {
					IOWrite32A( 0xE0701C, 0x00000002 ) ;
					return ( 0x41 ) ;
				}
			} while ( UlReadVal != 0 ) ;
			RamWrite32A( 0xF00C, 0x00000008 ) ;
		}
	}

	UlCnt = 0 ;
	do {
		WitTim( 1 ) ;
		RamRead32A( 0xF00C, &UlReadVal ) ;
		if( UlCnt++ > 250 ) {
			IOWrite32A( 0xE0701C, 0x00000002 ) ;
			return ( 0x41 ) ;
		}
	}while ( (UlReadVal & 0x0000000C) != 0 );

	RamWrite32A( 0xF00A, 0x00000000 ) ;
	data[ 1 ] = 0x08 ;

#if (BURST_LENGTH_FC == 32)
	data[ 2 ] = 0x00 ;
	UlNum = 3 ;
	for( j = 0; j < BURST_LENGTH_FC; j++ ) {
		data[ UlNum++ ] = *NcFromVal1st++ ;
	}
	CntWrt( data, BURST_LENGTH_FC + 3 ) ;

  	data[ 2 ] = 0x20 ;
	UlNum = 3 ;
	for( j = 0; j < BURST_LENGTH_FC; j++ ) {
		data[ UlNum++ ] = *NcFromVal1st++ ;
	}
	CntWrt( data, BURST_LENGTH_FC + 3 ) ;
#elif (BURST_LENGTH_FC == 64)
	data[ 2 ] = 0x00 ;
	UlNum = 3 ;
	for( j = 0; j < BURST_LENGTH_FC; j++ ) {
		data[ UlNum++ ] = *NcFromVal1st++ ;
	}
	CntWrt( data, BURST_LENGTH_FC + 3 ) ;
#endif

	RamWrite32A( 0xF00B, 0x00000010 ) ;
	UlCnt = 0 ;
	do {
		WitTim( 1 );
		RamRead32A( 0xF00C, &UlReadVal ) ;
		if( UlCnt++ > 250 ) {
			IOWrite32A( 0xE0701C, 0x00000002 ) ;
			return ( 0x41 ) ;
		}
	} while ( UlReadVal != 0 ) ;
 	RamWrite32A( 0xF00C, 0x00000004 ) ;

	UlCnt = 0 ;
	do {
		WitTim( 1 ) ;
		RamRead32A( 0xF00C, &UlReadVal ) ;
		if( UlCnt++ > 250 ) {
			IOWrite32A( 0xE0701C, 0x00000002 ) ;
			return ( 0x41 ) ;
		}
	} while ( (UlReadVal & 0x0000000C) != 0 ) ;

	IOWrite32A( 0xE0701C, 0x00000002 ) ;
	return( 0 );
}

//********************************************************************************
// Function Name 	: FlashUpdate129
//********************************************************************************
UINT_8 FlashUpdate129( CODE_TBL_EXT* ptr )
{
	UINT_8 ans=0 ;
	UINT_32	UlReadVal, UlCnt ;

//--------------------------------------------------------------------------------
// 1.
//--------------------------------------------------------------------------------
 	ans = PmemUpdate129( 0, ptr ) ;
	if(ans != 0) return ( ans ) ;

//--------------------------------------------------------------------------------
// 2.
//--------------------------------------------------------------------------------
	if( UnlockCodeSet() != 0 ) 		return ( 0x33 ) ;
	WritePermission() ;
	AdditionalUnlockCodeSet() ;

//	if( chiperase != 0 )
	 	ans = EraseUserMat129( 0, FLASH_BLOCKS ) ;
//	else
//		ans = EraseUserMat129( 0, ERASE_BLOCKS ) ;

	if(ans != 0){
		if( UnlockCodeClear() != 0 ) 	return ( 0x32 ) ;
		else					 		return ( ans ) ;
	}

//--------------------------------------------------------------------------------
// 3.
//--------------------------------------------------------------------------------
	ans = ProgramFlash129_Standard( ptr ) ;

	if(ans != 0){
		if( UnlockCodeClear() != 0 ) 	return ( 0x43 ) ;
		else					 		return ( ans ) ;
	}

	if( UnlockCodeClear() != 0 ) 	return ( 0x43 ) ;

//--------------------------------------------------------------------------------
// 4.
//--------------------------------------------------------------------------------
	IOWrite32A( 0xE0701C, 0x00000000 ) ;
	RamWrite32A( 0xF00A, 0x00000000 ) ;
	RamWrite32A( 0xF00D, ptr->SizeFromCodeValid ) ;

	RamWrite32A( 0xF00C, 0x00000100 ) ;
	WitTim( 6 ) ;
	UlCnt = 0 ;
	do {
		RamRead32A( 0xF00C, &UlReadVal ) ;
		if( UlCnt++ > 100 ) {
			IOWrite32A( 0xE0701C , 0x00000002 ) ;
			return ( 0x51 ) ;
		}
		WitTim( 1 ) ;
	} while ( UlReadVal != 0 ) ;

	RamRead32A( 0xF00D, &UlReadVal );

	if( UlReadVal != ptr->SizeFromCodeCksm ) {
		IOWrite32A( 0xE0701C , 0x00000002 );
		return( 0x52 );
	}

	IOWrite32A( SYSDSP_REMAP, 0x00001001 ) ;
	WitTim( 15 ) ;
	IORead32A( ROMINFO,	(UINT_32 *)&UlReadVal ) ;
	if( UlReadVal != 0x0A)		return( 0x53 );

	return ( 0 );
}

//********************************************************************************
// Function Name 	: FlashBlockErase
//********************************************************************************
UINT_8	FlashBlockErase( UINT_8 SelMat , UINT_32 SetAddress )
{
	UINT_32	UlReadVal, UlCnt ;
	UINT_8	ans	= 0 ;

	if( SelMat != USER_MAT && SelMat != INF_MAT0 && SelMat != INF_MAT1 && SelMat != INF_MAT2 )
		return 10 ;
	if( SetAddress > 0x00003CFF )
		return 9 ;

	ans	= UnlockCodeSet() ;
	if( ans != 0 )	return( ans ) ;

	WritePermission() ;
	//if( SelMat == TRIM_MAT ){
	//	IOWrite32A( 0xE07CCC, 0x00005B29 ) ;
	//} else
	if (SelMat != USER_MAT ){
		IOWrite32A( 0xE07CCC, 0x0000C5AD ) ;
	}
	AdditionalUnlockCodeSet() ;

	IOWrite32A( FLASHROM_FLA_ADR, ((UINT_32)SelMat << 16) | ( SetAddress & 0x00003C00 )) ;
	IOWrite32A( FLASHROM_CMD, 4 ) ;

	WitTim( 5 ) ;

	UlCnt = 0 ;

	do {
		if( UlCnt++ > 100 ){	ans = 2;	break;	} ;

		IORead32A( FLASHROM_FLAINT, &UlReadVal ) ;
	} while( ( UlReadVal & 0x00000080 ) != 0 ) ;

	ans	= UnlockCodeClear() ;
	if( ans != 0 )	return( ans ) ;

	return( ans ) ;
}

//********************************************************************************
// Function Name 	: FlashSingleRead
//********************************************************************************
UINT_8	FlashSingleRead( UINT_8 SelMat, UINT_32 UlAddress, UINT_32 *PulData )
{
	if( SelMat != USER_MAT && SelMat != INF_MAT0 && SelMat != INF_MAT1 && SelMat != INF_MAT2  )
		return 10 ;
	if( UlAddress > 0x00003FFF )
		return 9 ;

	IOWrite32A( FLASHROM_ACSCNT, 0x00000000 ) ;
	IOWrite32A( FLASHROM_FLA_ADR, ((UINT_32)SelMat << 16) | ( UlAddress & 0x00003FFF ) ) ;

	IOWrite32A( FLASHROM_CMD, 0x00000001 ) ;

	IORead32A( FLASHROM_FLA_RDAT, PulData ) ;

	return( 0 ) ;
}

//********************************************************************************
// Function Name 	: FlashMultiRead
//********************************************************************************
UINT_8	FlashMultiRead( UINT_8 SelMat, UINT_32 UlAddress, UINT_32 *PulData , UINT_8 UcLength )
{
	UINT_8	i ;

	if( SelMat != USER_MAT && SelMat != INF_MAT0 && SelMat != INF_MAT1 && SelMat != INF_MAT2  )
		return 10 ;
	if( UlAddress > 0x00003FFF )
		return 9;

	IOWrite32A( FLASHROM_ACSCNT, 0x00000000 | (UINT_32)(UcLength-1) );
	IOWrite32A( FLASHROM_FLA_ADR, ((UINT_32)SelMat << 16) | ( UlAddress & 0x00003FFF ) );

	IOWrite32A( FLASHROM_CMD, 0x00000001 );
	for( i=0 ; i < UcLength ; i++ ){
		IORead32A( FLASHROM_FLA_RDAT, &PulData[i] ) ;
		CAM_ERR(CAM_OIS, "FlashMultiRead: Read Data[%02x] = %08x\n", i , PulData[i] );
	}

	return( 0 ) ;
}

//********************************************************************************
// Function Name 	: FlashPageWrite
//********************************************************************************
UINT_8	FlashPageWrite( UINT_8 SelMat , UINT_32 SetAddress , UINT_32 *PulData)
{
	UINT_32	UlReadVal, UlCnt;
	UINT_8	ans	= 0 ;
	UINT_8	i	 ;

	if( SelMat != USER_MAT && SelMat != INF_MAT0 && SelMat != INF_MAT1 && SelMat != INF_MAT2  )
		return 10;

	if( SetAddress > 0x00003FFF )
		return 9;

	ans	= UnlockCodeSet();
	if( ans != 0 )	return( ans ) ;

	WritePermission();
	if( SelMat != USER_MAT ){
		IOWrite32A( 0xE07CCC, 0x0000C5AD ) ;
	}
	AdditionalUnlockCodeSet() ;

	IOWrite32A( FLASHROM_FLA_ADR, ((UINT_32)SelMat << 16) | ( SetAddress & 0x00003FF0 )) ;
	IOWrite32A( FLASHROM_CMD, 2 ) ;

	UlCnt	= 0 ;

	for( i=0 ; i< 16 ; i++ ){
		IOWrite32A( FLASHROM_FLA_WDAT, PulData[i]  ) ;
	}
	do {
		if( UlCnt++ > 100 ){	ans = 2;	break;	} ;

		IORead32A( FLASHROM_FLAINT, &UlReadVal ) ;
	} while( ( UlReadVal & 0x00000080 ) != 0 ) ;

	IOWrite32A( FLASHROM_CMD, 8  );

	do {
		if( UlCnt++ > 100 ){	ans = 2;	break;	} ;

		IORead32A( FLASHROM_FLAINT, &UlReadVal ) ;
	} while( ( UlReadVal & 0x00000080 ) != 0 ) ;

	ans	= UnlockCodeClear() ;
	return( ans ) ;

}

//********************************************************************************
// Function Name 	: RdErInfMAT
//********************************************************************************
UINT_8	RdErInfMAT( UINT_8 SelMat, UINT_32 *InfMat, UINT_16 Length )
{
	UINT_8	ans	= 0 ;

	ans =FlashMultiRead( SelMat, 0, InfMat, Length ) ;

	if( ans == 0 ) {
		ans = FlashBlockErase( SelMat , 0 ) ;
		if( ans != 0 ) {
			ans = 2 ;
		}
	} else {
		ans = 1 ;
	}
	return( ans ) ;
}

//********************************************************************************
// Function Name 	: WrInfMAT
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
	UsCkVal +=  (UINT_8)(InfMAT[ i ] >> 16);
	UsCkVal +=  (UINT_8)(InfMAT[ i ] >> 24);

	return ( UsCkVal ) ;
}



//********************************************************************************
// Function Name 	: FlashProgram129
//********************************************************************************
UINT_8 FlashProgram129( UINT_8 ModuleVendor, UINT_8 ActVer,struct cam_ois_ctrl_t *o_ctrl )
{
	CODE_TBL_EXT* ptr ;
	UINT_32 ois_version = 0;
	UINT_64 fw_version = 0;
	UINT_32 oem_version = 0;
	int32_t rc = 0;
	struct cam_hw_soc_info *soc_info = &o_ctrl->soc_info;
	struct device_node *of_node = NULL;
	void *vaddr = NULL;

	of_node = soc_info->dev->of_node;

	ptr = ( CODE_TBL_EXT * )CdTbl ;

	ptr->SizeFromCode = of_property_count_u8_elems(of_node, "fw_data");
	if (ptr->SizeFromCode <= 0) {
		CAM_ERR(CAM_OIS, "ptr->SizeFromCode <= 0 s:%d",of_property_count_u8_elems(of_node, "fw_data"));
		return 0;
	}
	CAM_INFO(CAM_OIS,"allocating fw_data_size: %d", ptr->SizeFromCode);

	vaddr = vmalloc(sizeof(uint8_t) * ptr->SizeFromCode);
	 if (!vaddr) {
		CAM_ERR(CAM_OIS,
			"Failed in allocating fw_data_size: %u", ptr->SizeFromCode);
		return 0;
	 }

	ptr->FromCode = (uint8_t*) (vaddr);
	rc = of_property_read_u8_array(of_node, "fw_data", ptr->FromCode, ptr->SizeFromCode);
	if ( rc < 0) {
			CAM_ERR(CAM_OIS, "Invalid fw_data params");
	}

	rc = of_property_read_u32(of_node, "check_sum_size", &ptr->SizeFromCodeValid);
	if ( rc < 0) {
		CAM_ERR(CAM_OIS, "Invalid check_sum_size params");
	}
	CAM_INFO(CAM_OIS, "check_sum_size params :0x%x", ptr->SizeFromCodeValid);

	rc = of_property_read_u32(of_node, "check_sum", &ptr->SizeFromCodeCksm);
	if ( rc < 0) {
		CAM_ERR(CAM_OIS, "Invalid check_sum params");
	}
	CAM_INFO(CAM_OIS, "check_sum params :0x%x", ptr->SizeFromCodeCksm);


	do {
		if( ptr->Index == ( ((UINT_16)ModuleVendor << 8) + ActVer) ) {
			if((!RamRead32A(0x8000, & ois_version)) && (!RamRead32A(0x8004, & oem_version)))
			{
				fw_version = ((UINT_32)ptr->FromCode[158]) << 24 |
					((UINT_32)ptr->FromCode[159]) << 16 |
					((UINT_32)ptr->FromCode[160]) << 8  |
					(UINT_32)ptr->FromCode[161];
				CAM_INFO(CAM_OIS, "ois_version:0x%x  fw_version:0x%x oem_version:0x%x" ,ois_version, fw_version, oem_version);
				if(ois_version >= fw_version && (oem_version == 0x000107ff || oem_version == 0x00010700)){
					CAM_INFO(CAM_OIS, "ois_version >= fw_version no need to update");
					vfree(vaddr);
					vaddr = NULL;
					return 0;
				}
				BootMode() ;
				rc = FlashUpdate129( ptr );
			}else{
				CAM_ERR(CAM_OIS, "read ois current version 0x8000 failed ");
			}
			break;
		}else{
			CAM_ERR(CAM_OIS, "ptr->Index:0x%x Vendor:0x%x", ptr->Index,( ((UINT_16)ModuleVendor << 8) + ActVer));
		}
		ptr++ ;
	} while (ptr->Index != 0xFFFF ) ;

	vfree(vaddr);
	vaddr = NULL;

	if(rc != 0){
		return ( 0xF0 ) ;
	}else{
		return rc;
	}

}

//********************************************************************************
// Function Name 	: GetGyroOffset
//********************************************************************************
void	GetGyroOffset( UINT_16* GyroOffsetX, UINT_16* GyroOffsetY, UINT_16* GyroOffsetZ )
{
	UINT_32 ReadValX, ReadValY, ReadValZ;
	RamRead32A( GYRO_RAM_GXOFFZ , &ReadValX );
	RamRead32A( GYRO_RAM_GYOFFZ , &ReadValY );
	RamRead32A( GYRO_ZRAM_GZOFFZ , &ReadValZ );
	*GyroOffsetX = ( UINT_16 )(( ReadValX >> 16) & 0x0000FFFF );
	*GyroOffsetY = ( UINT_16 )(( ReadValY >> 16) & 0x0000FFFF );
	*GyroOffsetZ = ( UINT_16 )(( ReadValZ >> 16) & 0x0000FFFF );
}

//********************************************************************************
// Function Name 	: GetAcclOffset
//********************************************************************************
void	GetAcclOffset( UINT_16* AcclOffsetX, UINT_16* AcclOffsetY, UINT_16* AcclOffsetZ )
{
	UINT_32 ReadValX, ReadValY, ReadValZ;
	RamRead32A( ACCLRAM_X_AC_OFFSET , &ReadValX );
	RamRead32A( ACCLRAM_Y_AC_OFFSET , &ReadValY );
	RamRead32A( ACCLRAM_Z_AC_OFFSET , &ReadValZ );
	*AcclOffsetX = ( UINT_16 )(( ReadValX >> 16) & 0x0000FFFF );
	*AcclOffsetY = ( UINT_16 )(( ReadValY >> 16) & 0x0000FFFF );
	*AcclOffsetZ = ( UINT_16 )(( ReadValZ >> 16) & 0x0000FFFF );
}

//********************************************************************************
// Function Name 	: MesFil
//********************************************************************************
void	MesFil( void )
{
	UINT_32	UlMeasFilaA , UlMeasFilaB , UlMeasFilaC ;
	UINT_32	UlMeasFilbA , UlMeasFilbB , UlMeasFilbC ;

	UlMeasFilaA	=	0x7FFFFFFF ;
	UlMeasFilaB	=	0x00000000 ;
	UlMeasFilaC	=	0x00000000 ;
	UlMeasFilbA	=	0x7FFFFFFF ;
	UlMeasFilbB	=	0x00000000 ;
	UlMeasFilbC	=	0x00000000 ;


	RamWrite32A ( 0x84E0	, UlMeasFilaA ) ;
	RamWrite32A ( 0x84D8	, UlMeasFilaB ) ;
	RamWrite32A ( 0x84DC	, UlMeasFilaC ) ;

	RamWrite32A ( 0x84EC	, UlMeasFilbA ) ;
	RamWrite32A ( 0x84E4	, UlMeasFilbB ) ;
	RamWrite32A ( 0x84E8	, UlMeasFilbC ) ;

	RamWrite32A ( 0x84F8	, UlMeasFilaA ) ;
	RamWrite32A ( 0x84F0	, UlMeasFilaB ) ;
	RamWrite32A ( 0x84F4	, UlMeasFilaC ) ;

	RamWrite32A ( 0x8504	, UlMeasFilbA ) ;
	RamWrite32A ( 0x84FC	, UlMeasFilbB ) ;
	RamWrite32A ( 0x8500	, UlMeasFilbC ) ;
}


//********************************************************************************
// Function Name 	: MemoryClear
//********************************************************************************
void	MemoryClear( UINT_16 UsSourceAddress, UINT_16 UsClearSize )
{
	UINT_16	UsLoopIndex ;

	for ( UsLoopIndex = 0 ; UsLoopIndex < UsClearSize ;  ) {
		RamWrite32A( UsSourceAddress	, 	0x00000000 ) ;	
		UsSourceAddress += 4;
		UsLoopIndex += 4 ;
	}
}

//********************************************************************************
// Function Name 	: ClrMesFil
//********************************************************************************
void	ClrMesFil( void )
{
	RamWrite32A ( 0x0550	, 0 ) ;
	RamWrite32A ( 0x0554	, 0 ) ;

	RamWrite32A ( 0x0558	, 0 ) ;
	RamWrite32A ( 0x055C	, 0 ) ;

	RamWrite32A ( 0x0560	, 0 ) ;
	RamWrite32A ( 0x0564	, 0 ) ;

	RamWrite32A ( 0x0568	, 0 ) ;
	RamWrite32A ( 0x056C	, 0 ) ;
}

//********************************************************************************
// Function Name 	: SetWaitTime
//********************************************************************************
#define 	ONE_MSEC_COUNT	3	
void	SetWaitTime( UINT_16 UsWaitTime )
{
	RamWrite32A( 0x05A4	, 0 ) ;
	RamWrite32A( 0x05A8	, (UINT_32)(ONE_MSEC_COUNT * UsWaitTime)) ;
}

//********************************************************************************
// Function Name 	: MeasureStart
//********************************************************************************
void	MeasureStart( INT_32 SlMeasureParameterNum , INT_32 SlMeasureParameterA , INT_32 SlMeasureParameterB )
{
	MemoryClear( 0x04B8 , 38*4 ) ;
	RamWrite32A( 0x04C0	 , 0x80000000 ) ;
	RamWrite32A( 0x0508	 , 0x80000000 ) ;
	RamWrite32A( 0x04C4	 , 0x7FFFFFFF ) ;
	RamWrite32A( 0x050C	 , 0x7FFFFFFF ) ;

	RamWrite32A( 0x04E0	, ( UINT_32 )SlMeasureParameterA ) ;
	RamWrite32A( 0x0528	, ( UINT_32 )SlMeasureParameterB ) ;

	RamWrite32A( 0x04B8	, 0 ) ;
	ClrMesFil() ;	
	SetWaitTime(50) ;
	RamWrite32A( 0x04BC	, SlMeasureParameterNum ) ;
}

//********************************************************************************
// Function Name 	: MeasureWait
//********************************************************************************
void	MeasureWait( void )
{
	UINT_32	SlWaitTimerSt;
	UINT_16	UsTimeOut = 5000;
	do {
		RamRead32A( 0x04BC, &SlWaitTimerSt ) ;
		UsTimeOut--;
	} while ( SlWaitTimerSt && UsTimeOut );
}

//********************************************************************************
// Function Name 	: MeasAddressSelection
//********************************************************************************
void MeasAddressSelection( UINT_8 mode , INT_32 * measadr_a , INT_32 * measadr_b )
{
	if( mode == 0 ){
		*measadr_a		=	GYRO_RAM_GX_ADIDAT ;
		*measadr_b		=	GYRO_RAM_GY_ADIDAT ;
	}else if( mode == 1 ){
		*measadr_a		=	GYRO_ZRAM_GZ_ADIDAT ;
		*measadr_b		=	ACCLRAM_Z_AC_ADIDAT ;
	}else{
		*measadr_a		=	ACCLRAM_X_AC_ADIDAT ;
		*measadr_b		=	ACCLRAM_Y_AC_ADIDAT ;
	}
}	
		
//********************************************************************************
// Function Name 	: MeasAddreMeasGyAcOffset
//********************************************************************************
#define 	MESOF_NUM	2048
#define 	GYROFFSET_H	( 0x06D6 << 16 )
#define		GSENS		( 16393 << 16 )
#define		GSENS_MARG	(GSENS / 4)
#define		POSTURETH	(GSENS - GSENS_MARG)
#define		ZG_MRGN		(1310 << 16)

UINT_32	MeasGyAcOffset(  void  )
{
	UINT_32			UlRsltSts;
	INT_32			SlMeasureParameterA , SlMeasureParameterB ;
	INT_32			SlMeasureParameterNum ;
	UnllnVal		StMeasValueA , StMeasValueB ;
	INT_32			SlMeasureAveValueA[3] , SlMeasureAveValueB[3] ;
	UINT_8			i ;
	INT_32			SlMeasureAZ = 0;

	CAM_ERR(CAM_OIS, "***MeasGyAcOffset" );
	MesFil() ;

	SlMeasureParameterNum	=	MESOF_NUM ;					// Measurement times

	for( i=0 ; i<3 ; i++ )
	{
		MeasAddressSelection( i, &SlMeasureParameterA , &SlMeasureParameterB );

		MeasureStart( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB ) ;
		MeasureWait() ;					// Wait complete of measurement

		RamRead32A( 0x04D0 		, &StMeasValueA.StUllnVal.UlLowVal ) ;	// X axis
		RamRead32A( 0x04D0 + 4	, &StMeasValueA.StUllnVal.UlHigVal ) ;
		RamRead32A( 0x0518 		, &StMeasValueB.StUllnVal.UlLowVal ) ;	// Y axis
		RamRead32A( 0x0518 + 4	, &StMeasValueB.StUllnVal.UlHigVal ) ;


		CAM_ERR(CAM_OIS, "(%d) AOFT = %08x, %08xh ",i,(unsigned int)StMeasValueA.StUllnVal.UlHigVal,(unsigned int)StMeasValueA.StUllnVal.UlLowVal) ;
		CAM_ERR(CAM_OIS, "(%d) BOFT = %08x, %08xh ",i,(unsigned int)StMeasValueB.StUllnVal.UlHigVal,(unsigned int)StMeasValueB.StUllnVal.UlLowVal) ;
		SlMeasureAveValueA[i] = (INT_32)( (INT_64)StMeasValueA.UllnValue / SlMeasureParameterNum ) ;
		SlMeasureAveValueB[i] = (INT_32)( (INT_64)StMeasValueB.UllnValue / SlMeasureParameterNum ) ;
		CAM_ERR(CAM_OIS, "AVEOFT = %08xh ",(unsigned int)SlMeasureAveValueA[i]) ;
		CAM_ERR(CAM_OIS, "AVEOFT = %08xh ",(unsigned int)SlMeasureAveValueB[i]) ;
	}

	UlRsltSts = EXE_END ;

	if( (SlMeasureAveValueB[1]) >= POSTURETH ){
		SlMeasureAZ = SlMeasureAveValueB[1] - (INT_32)GSENS;
	}else if( (SlMeasureAveValueB[1]) <= -POSTURETH ){
		SlMeasureAZ = SlMeasureAveValueB[1] + (INT_32)GSENS;
	}else{
		//UlRsltSts |= EXE_AZADJ ;
		CAM_ERR(CAM_OIS, "SlMeasureAveValueB[1]:0x%x POSTURETH:0x%x UlRsltSts:0x%x EXE_AZADJ",SlMeasureAveValueB[1], POSTURETH, UlRsltSts) ;
	}

	CAM_ERR(CAM_OIS, "AZOFF = %08xh UlRsltSts:0x%x",(unsigned int)SlMeasureAZ, UlRsltSts) ;

	if( abs(SlMeasureAveValueA[0]) > GYROFFSET_H )					UlRsltSts |= EXE_GXADJ ;
	if( abs(SlMeasureAveValueB[0]) > GYROFFSET_H ) 					UlRsltSts |= EXE_GYADJ ;
	if( abs(SlMeasureAveValueA[1]) > GYROFFSET_H ) 					UlRsltSts |= EXE_GZADJ ;
	CAM_ERR(CAM_OIS, "UlRsltSts:0x%x",UlRsltSts) ;

	if( abs(SlMeasureAveValueA[2]) > ZG_MRGN )						UlRsltSts |= EXE_AXADJ ;
	if( abs(SlMeasureAveValueB[2]) > ZG_MRGN )						UlRsltSts |= EXE_AYADJ ;
	if( abs( SlMeasureAZ) > ZG_MRGN )								UlRsltSts |= EXE_AZADJ ;

	if( UlRsltSts == EXE_END ){
		RamWrite32A( GYRO_RAM_GXOFFZ ,		SlMeasureAveValueA[0] ) ;							// X axis Gyro offset
		RamWrite32A( GYRO_RAM_GYOFFZ ,		SlMeasureAveValueB[0] ) ;							// Y axis Gyro offset
		RamWrite32A( GYRO_ZRAM_GZOFFZ ,		SlMeasureAveValueA[1] ) ;							// Z axis Gyro offset
		RamWrite32A( ACCLRAM_X_AC_OFFSET ,	SlMeasureAveValueA[2] ) ;							// X axis Accel offset
		RamWrite32A( ACCLRAM_Y_AC_OFFSET ,	SlMeasureAveValueB[2] ) ;							// Y axis Accel offset
		RamWrite32A( ACCLRAM_Z_AC_OFFSET , 	SlMeasureAZ ) ;										// Z axis Accel offset

		RamWrite32A( 0x0418 , 0x00000000 ) ;		// X axis Drift Gyro offset
		RamWrite32A( 0x043C , 0x00000000 ) ;		// Y axis Drift Gyro offset
		RamWrite32A( 0x05F8 , 0x00000000 ) ;		// Z axis Drift Gyro offset
		RamWrite32A( 0x03DC , 0x00000000 ) ;		// X axis H1Z2 Clear
		RamWrite32A( 0x0404 , 0x00000000 ) ;		// Y axis H1Z2 Clear
		RamWrite32A( 0x05DC , 0x00000000 ) ;		// Z axis H1Z2 Clear
		RamWrite32A( 0x0640 + 8 , 0x00000000 ) ;	// X axis Accl LPF Clear
		RamWrite32A( 0x0670 + 8 , 0x00000000 ) ;	// Y axis Accl LPF Clear
		RamWrite32A( 0x06A0 + 8 , 0x00000000 ) ;	// Z axis Accl LPF Clear
		RamWrite32A( 0x0640 + 12 , 0x00000000 ) ;	// X axis Accl LPF Clear
		RamWrite32A( 0x0670 + 12 , 0x00000000 ) ;	// Y axis Accl LPF Clear
		RamWrite32A( 0x06A0 + 12 , 0x00000000 ) ;	// Z axis Accl LPF Clear
		RamWrite32A( 0x0640 + 16 , 0x00000000 ) ;	// X axis Accl LPF Clear
		RamWrite32A( 0x0670 + 16 , 0x00000000 ) ;	// Y axis Accl LPF Clear
		RamWrite32A( 0x06A0 + 16 , 0x00000000 ) ;	// Z axis Accl LPF Clear
		RamWrite32A( 0x0640 + 20 , 0x00000000 ) ;	// X axis Accl LPF Clear
		RamWrite32A( 0x0670 + 20 , 0x00000000 ) ;	// Y axis Accl LPF Clear
		RamWrite32A( 0x06A0 + 20 , 0x00000000 ) ;	// Z axis Accl LPF Clear
	}

	return( UlRsltSts );
}

//********************************************************************************
// Function Name 	: WrGyAcOffsetData
//********************************************************************************
UINT_8	WrGyAcOffsetData( UINT_8 UcMode )
{
	UINT_32	UlMAT0[64];
	UINT_32	UlReadGx, UlReadGy, UlReadGz, UlReadAx, UlReadAy, UlReadAz;
	UINT_8 ans = 0;
	UINT_16	UsCkVal,UsCkVal_Bk ;

	IOWrite32A( 0xE0701C , 0x00000000 ) ;

	ans = RdErInfMAT( INF_MAT0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( ans );
	}

	if( UcMode ){
		RamRead32A(  GYRO_RAM_GXOFFZ , &UlReadGx ) ;
		RamRead32A(  GYRO_RAM_GYOFFZ , &UlReadGy ) ;
		RamRead32A(  GYRO_ZRAM_GZOFFZ , &UlReadGz ) ;
		RamRead32A(  ACCLRAM_X_AC_OFFSET , &UlReadAx ) ;
		RamRead32A(  ACCLRAM_Y_AC_OFFSET , &UlReadAy ) ;
		RamRead32A(  ACCLRAM_Z_AC_OFFSET , &UlReadAz ) ;
		CAM_ERR(CAM_OIS,  "WrGyAcOffsetData: UlReadGx:0x%x UlReadGy:0x%x" ,UlReadGx, UlReadGy);

		UlMAT0[0] &= ~( 0x00000100 );
		UlMAT0[5]	= (UINT_32)((UlReadAx & 0xFFFF0000) | ((UlReadGx >> 16 ) & 0x0000FFFF));
		UlMAT0[6]	= (UINT_32)((UlReadAy & 0xFFFF0000) | ((UlReadGy >> 16 ) & 0x0000FFFF));
		UlMAT0[7]	= (UINT_32)((UlReadAz & 0xFFFF0000) | ((UlReadGz >> 16 ) & 0x0000FFFF));
	}else{
		UlMAT0[0] |= 0x00000100;
		UlMAT0[5]	= 0x00000000;
		UlMAT0[6]	= 0x00000000;
		UlMAT0[7]	= 0x00000000;
	}

	UsCkVal = MkInfMATsum( UlMAT0 ) ;
	UlMAT0[63] &= (UINT_32)0xFFFF0000 ;
	UlMAT0[63] |= (UINT_32)UsCkVal ;


	ans = WrInfMAT( INF_MAT0, UlMAT0, 64 ) ;
	if( ans != 0 ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 3 ) ;							// 0 - 63
	}

	UsCkVal_Bk = UsCkVal;
	ans =FlashMultiRead( INF_MAT0, 0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( 4 );
	}
	UsCkVal = MkInfMATsum( UlMAT0 ) ;

	CAM_ERR(CAM_OIS,  "WrGyAcOffsetData: [RVAL]:[BVal]=[%04x]:[%04x]",UsCkVal, UsCkVal_Bk );
	if( UsCkVal != UsCkVal_Bk ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return(5);
	}

	CAM_ERR(CAM_OIS,  "WrGyAcOffsetData: WrGyAcOffsetData____COMPLETE" );
	IOWrite32A( 0xE0701C , 0x00000002 ) ;
	return(0);
}

//********************************************************************************
// Function Name 	: WrGyroGainData
//********************************************************************************
UINT_8 WrGyroGainData( UINT_8 UcMode )
{
	UINT_32	UlMAT0[64];
	UINT_32	UlReadGxzoom , UlReadGyzoom , UlReadGzzoom;
	UINT_8 ans = 0;
	UINT_16	UsCkVal,UsCkVal_Bk ;

	IOWrite32A( 0xE0701C , 0x00000000 ) ;

	ans = RdErInfMAT( INF_MAT0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( ans );
	}

	if ( UcMode ) {	/* write */
		RamRead32A(  GyroFilterTableX_gxzoom , &UlReadGxzoom ) ;
		RamRead32A(  GyroFilterTableY_gyzoom , &UlReadGyzoom ) ;
		RamRead32A(  GyroFilterTableZ_gzzoom , &UlReadGzzoom ) ;

		UlMAT0[32] &= ~( 0x00004000 );
		UlMAT0[44] = UlReadGzzoom;
		UlMAT0[45] = UlReadGxzoom;
		UlMAT0[46] = UlReadGyzoom;
	} else {
		UlMAT0[32] |= 0x00004000;
		UlMAT0[44] = 0x3FFFFFFF;
		UlMAT0[45] = 0x3FFFFFFF;
		UlMAT0[46] = 0x3FFFFFFF;
	}
	/* calcurate check sum ******************************************/
	UsCkVal = MkInfMATsum( UlMAT0 ) ;
	UlMAT0[63] &= (UINT_32)0xFFFF0000 ;
	UlMAT0[63] |= (UINT_32)UsCkVal ;

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

	if( UsCkVal != UsCkVal_Bk )	{
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return(5);
	}
	IOWrite32A( 0xE0701C , 0x00000002 ) ;
	return(0);
}

//********************************************************************************
// Function Name 	: WrGyroGainData_LS
//********************************************************************************
UINT_8 WrGyroGainData_LS( UINT_8 UcMode )
{
	UINT_32	UlMAT0[64];
	UINT_32	UlReadGxzoom , UlReadGyzoom ;
	UINT_8 ans = 0;
	UINT_16	UsCkVal,UsCkVal_Bk ;

	IOWrite32A( 0xE0701C , 0x00000000 ) ;

	/* Back up ******************************************************/
	/* Erase   ******************************************************/
	ans = RdErInfMAT( INF_MAT0, UlMAT0, 64 );
	if( ans ) {
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return( ans );
	}

	/* modify   *****************************************************/
	if ( UcMode ) {	/* write */
		RamRead32A(  GyroFilterTableX_LS_gxzoom , &UlReadGxzoom ) ;
		RamRead32A(  GyroFilterTableY_LS_gyzoom , &UlReadGyzoom ) ;

		UlMAT0[0] &= ~( 0x00004000 );
		UlMAT0[8] = UlReadGxzoom;
		UlMAT0[9] = UlReadGyzoom;
	} else {
		UlMAT0[ 0] |= 0x00004000;
		UlMAT0[8] = 0x3FFFFFFF;
		UlMAT0[9] = 0x3FFFFFFF;
	}
	/* calcurate check sum ******************************************/
	UsCkVal = MkInfMATsum( UlMAT0 ) ;
	UlMAT0[63] &= (UINT_32)0xFFFF0000 ;
	UlMAT0[63] |= (UINT_32)UsCkVal ;

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

	if( UsCkVal != UsCkVal_Bk )	{
		IOWrite32A( 0xE0701C , 0x00000002 ) ;
		return(5);
	}

	IOWrite32A( 0xE0701C , 0x00000002 ) ;
	return(0);
}

