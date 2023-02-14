/**
 * @brief		OIS system flash program code for LC898129
 *
 * @author		(C) 2019 ON Semiconductor.
 *
 * @file		FlsPgm.c
 * @date		svn:$Date:: 2021-01-21 14:36:05 +0900#$
 * @version		svn:$Revision: 5 $
 * @attention
 **/
#define	__FLSPGM__
//**************************
//	Include Header File
//**************************
#include	"Ois.h"

#include	"OisAPI.h"
#include	"UpdataCode129.h"

#include	"FromCode_07.h"  // TAH1120

//****************************************************
//	CUSTOMER NECESSARY CREATING FUNCTION LIST
//****************************************************
extern UINT_8 FlashUpdate129( UINT_8, CODE_TBL_EXT * );
extern UINT_8 PmemUpdate129( UINT_8, CODE_TBL_EXT * );
extern void setI2cSlvAddr( unsigned char, unsigned char );

//********************************************************************************
// Function Name 	: FlashProgram129
// Retun Value		: NON
// Argment Value	: chiperase, ModuleVendor, ActVer
// Explanation		: Flash Update for LC898129
//********************************************************************************
const CODE_TBL_EXT CdTbl[] = {
	{0x0007, CcUpdataCode129, UpDataCodeSize,  UpDataCodeCheckSum, CcFromCode129_07, sizeof(CcFromCode129_07), FromCheckSum_07, FromCheckSumSize_07 },
	{0xFFFF, (void*)0,        0,               0,                  (void*)0,                  0,                                 0,                        0}
};

UINT_8 FlashProgram129( UINT_8 chiperase, UINT_8 ModuleVendor, UINT_8 ActVer )
{
	CODE_TBL_EXT* ptr ;

	ptr = ( CODE_TBL_EXT * )CdTbl ;
	do {
		if( ptr->Index == ( ((UINT_16)ModuleVendor << 8) + ActVer) ) {
			return FlashUpdate129( chiperase, ptr );
		}
		ptr++ ;
	} while (ptr->Index != 0xFFFF ) ;

	return 0xF0 ;
}

UINT_8 PmemDownload129( UINT_8 ModuleVendor, UINT_8 ActVer )
{
	UINT_8 ans = 0;
	CODE_TBL_EXT* ptr ;

	ptr = ( CODE_TBL_EXT * )CdTbl ;
	do {
		if( ptr->Index == ( ((UINT_16)ModuleVendor << 8) + ActVer) ) {

			ans =  DrvOffAdj(DEFAULT_SLAVE_ADR);
			
			CAM_ERR(CAM_OIS, "[%02x]DrvOffAdj  \n", ans );
		 	if(ans != 0){
				return( ans );	
			}

			BootMode();
			
			// Set default slave address 0x48
			setI2cSlvAddr( DEFAULT_SLAVE_ADR, 2 );

			// return PmemUpdate129( 1, ptr );
			ans = PmemUpdate129( 1, ptr );
		 	if(ans == 0){
				// スレーブアドレス
				setI2cSlvAddr( DEFAULT_SLAVE_ADR, 2 );
			}
			return ans;
		}
		ptr++ ;
	} while (ptr->Index != 0xFFFF ) ;

	return 0xF0 ;
}

//********************************************************************************
// Function Name 	: LoadUserAreaToPM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Prepare user area data for update
//********************************************************************************
UINT_8 LoadUserAreaToPM( void )
{
	CODE_TBL_EXT* ptr ;
	
	ptr = ( CODE_TBL_EXT * )CdTbl ;

	return LoadUareaToPM( ptr , 0 );
}
