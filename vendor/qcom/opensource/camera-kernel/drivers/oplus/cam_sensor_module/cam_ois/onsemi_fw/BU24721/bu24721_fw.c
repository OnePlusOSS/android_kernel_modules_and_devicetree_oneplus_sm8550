/////////////////////////////////////////////////////////////////////////////
// File Name    : bu24721_fw.c
// Function        : Various function for OIS control
//
// Copyright(c)    Rohm Co.,Ltd. All rights reserved
//
/***** ROHM Confidential ***************************************************/
//#define    _USE_MATH_DEFINES                            //

#ifndef BU24721_FW_C
#define BU24721_FW_C
#endif
#include <linux/types.h>
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "fw_download_interface.h"

#include "bu24721_fw.h"
#include "FLASH_DATA_BIN.h"


struct 		_FACT_ADJ	FADJ_CAL = { 0x0000, 0x0000};
struct 		_FACT_ADJ	FADJ_PL = { 0x0000,0x0000};
//static ois_dev_t *g_rohm24721_dev;



//  *****************************************************
//  **** Change 2'c Hex value to the decimal data
//  **** ------------------------------------------------
//    **** IN
//  ****     OIS_UWORD    inpdat  (16bit)
//    ****        Change below
//  ****             0x8000---0xFFFF,0x00000--- 0x7FFF
//    ****         to
//  ****            -32868---    -1,0      --- +32768
//    **** OUT
//  ****     OIS_WORD    decimal data
//  *****************************************************
OIS_WORD H2D( OIS_UWORD u16_inpdat )
{

    OIS_WORD s16_temp;
    s16_temp = u16_inpdat;
    if( u16_inpdat > 32767 ) {
        s16_temp = (OIS_WORD)(u16_inpdat - 65536L);
    }

    return s16_temp;
}

//  *****************************************************
//  **** Change the decimal data to the 2'c Hex
//  **** ------------------------------------------------
//    **** IN
//  ****     OIS_WORD    inpdat  (16bit)
//    ****        Change below
//  ****            -32868---    -1,0      --- +32768
//    ****         to
//  ****             0x8000---0xFFFF,0x00000--- 0x7FFF
//    **** OUT
//  ****     OIS_UWORD    2'c data
//  *****************************************************
OIS_UWORD D2H( OIS_WORD s16_inpdat)
{

    OIS_UWORD u16_temp;

    if( s16_inpdat < 0 ) {
        u16_temp = (OIS_UWORD)(s16_inpdat + 65536L);
    } else {
        u16_temp = s16_inpdat;
    }
    return u16_temp;
}

OIS_UWORD cnv_fm_addr(OIS_UWORD u16_dat1)
{
    OIS_UWORD u16_dat2;
    OIS_UWORD u16_dat3;
    u16_dat2 = u16_dat1>>2;
    u16_dat3 = (u16_dat2>>8)+(u16_dat2<<8);
    return u16_dat3;
}

OIS_UBYTE F024_Polling(void)
{
    OIS_LONG	u16_i;
    OIS_UBYTE	u16_dat;
    for( u16_i = 1; u16_i <= Poling_times; u16_i ++ ) {
        u16_dat = I2C_OIS_8bit__read(OIS_status);
        if((u16_dat&0x01) == 1) {
            break;
        }
        CAM_INFO(CAM_OIS, "polling %d\n",u16_dat);
        Wait(1000);
    }
    return u16_dat;
}

void Erase_flash(void)
{
    OIS_UWORD subaddr_FM_era;
    /* Erase : 0x0000-0x9FFF (40kB) */
    subaddr_FM_era = ((0x0000 >> 2) | 0x8000); ///8000
    I2C_FM_8bit_write(subaddr_FM_era, 0xE5);
    Wait(40000);//wait 40ms

    /* Erase : 0xA000-0xA7FF (2kB) */
    subaddr_FM_era = ((0xA000 >> 2) | 0x8000); //A8
    I2C_FM_8bit_write(subaddr_FM_era, 0xE9);
    Wait(2000);//wait 2ms

    /* Erase : 0xA800-0xAFFF (2kB) */
    subaddr_FM_era = ((0xA800 >> 2) | 0x8000);//AA
    I2C_FM_8bit_write(subaddr_FM_era, 0xE9);
    Wait(2000);//wait 2ms

    /* Erase : 0xB000-0xB7FF (2kB) */
    subaddr_FM_era = ((0xB000 >> 2) | 0x8000);//AC
    I2C_FM_8bit_write(subaddr_FM_era, 0xE9);
    Wait(2000);//wait 2ms

    /* Erase : 0xB800-0xBFFF (2kB) */
    subaddr_FM_era = ((0xB800 >> 2) | 0x8000);//AE
    I2C_FM_8bit_write(subaddr_FM_era, 0xE9);
    Wait(2000);//wait 2ms

    /* Erase : 0xC000-0xC7FF (2kB) */
    subaddr_FM_era = ((0xC000 >> 2) | 0x8000);//B0
    I2C_FM_8bit_write(subaddr_FM_era, 0xE9);
    Wait(2000);//wait 2ms

    /* Erase : 0xC800-0xCFFF (2kB) */
    subaddr_FM_era = ((0xC800 >> 2) | 0x8000);//B2
    I2C_FM_8bit_write(subaddr_FM_era, 0xE9);
    Wait(2000);//wait 2ms

    /* Erase : 0xD000-0xD7FF (2kB) */
    subaddr_FM_era = ((0xD000 >> 2) | 0x8000);//B4
    I2C_FM_8bit_write(subaddr_FM_era, 0xE9);
    Wait(2000);//wait 2ms

    /* Erase : 0xD800-0xDFFF (2kB) */
    subaddr_FM_era = ((0xD800 >> 2) | 0x8000);//B6
    I2C_FM_8bit_write(subaddr_FM_era, 0xE9);
    Wait(2000);//wait 2ms

    /* Erase : 0xE000-0xE7FF (2kB) */
    subaddr_FM_era = ((0xE000 >> 2) | 0x8000);//B8
    I2C_FM_8bit_write(subaddr_FM_era, 0xE9);
    Wait(2000);//wait 2ms

    /* Erase : 0xE800-0xEFFF (2kB) */
    subaddr_FM_era = ((0xE800 >> 2) | 0x8000);//BA
    I2C_FM_8bit_write(subaddr_FM_era, 0xE9);
    Wait(2000);//wait 2ms

    /* Erase : 0xF000-0xF1FF (512B) */
    subaddr_FM_era = ((0xF000 >> 2) | 0x8000);//BC00
    I2C_FM_8bit_write(subaddr_FM_era, 0xE0);
    Wait(2000);//wait 2ms

    /* Erase : 0xF200-0xF3FF (512B) */
    subaddr_FM_era = ((0xF200 >> 2) | 0x8000);//BC80
    I2C_FM_8bit_write(subaddr_FM_era, 0xE0);
    Wait(2000);//wait 2ms


    /* Erase : 0x4000-0xF5FF (512B) */
    subaddr_FM_era = ((0xF400 >> 2) | 0x8000);//BD00
    I2C_FM_8bit_write(subaddr_FM_era, 0xE0);
    Wait(2000);//wait 2ms
    /*
        !!KEEP!! : 0xF600-0xF7FF as Calibration data(Gyro offset&gain) area (512B) is NOT erased

    */
    /* !!KEEP!! : 0xF800-0xF9FF (512B) area for Jahwa
    subaddr_FM_era = ((0xF800 >> 2) | 0x8000);
    I2C_FM_8bit_write(subaddr_FM_era, 0xE0);
    Wait(2000);//wait 2ms
    */
    /* !!KEEP!! : 0xFA00-0xFBFF (512B) area for Jahwa
    //subaddr_FM_era = ((0xFA00 >> 2) | 0x8000);//BE80
       //I2C_FM_8bit_write(subaddr_FM_era, 0xE0);
    //Wait(2000);//wait 2ms

        !!KEEP!! : 0xFC00-0xFDFF as Calibration data area (512B) is NOT erased
    */
    /* Erase : 0xFE00-0xFFFF (512B) */
    subaddr_FM_era = ((0xFE00 >> 2) | 0x8000);//BF80
    I2C_FM_8bit_write(subaddr_FM_era, 0xE0);
    Wait(2000);//wait 2ms
}

void Prog_flash(void)
{
    OIS_UBYTE out[10];
    OIS_ULONG wdata;
    OIS_UWORD addr;
    //*** Program FLASH ***//
    addr = cnv_fm_addr(0xFE00);
    out[0] = addr&0xff;
    out[1] = (addr>>8)&0xff;
    out[2] = 0x00;
    out[3] = 0x00;
    out[4] = 0x00;
    out[5] = 0x15;
    I2C_FM_block_write(out,6);
    addr = cnv_fm_addr(0xFE04);
    out[0] = addr&0xff;
    out[1] = (addr>>8)&0xff;
    out[2] = 0x42;
    out[3] = 0x44;
    out[4] = 0x58;
    out[5] = 0x1C;
    I2C_FM_block_write(out,6);
    addr = cnv_fm_addr(0xFE08);
    out[0] = addr&0xff;
    out[1] = (addr>>8)&0xff;
    out[2] = 0xA5;
    out[3] = 0x00;
    out[4] = 0x00;
    out[5] = 0xDB;
    I2C_FM_block_write(out,6);
    addr = cnv_fm_addr(0xFE0C);
    out[0] = addr&0xff;
    out[1] = (addr>>8)&0xff;
    out[2] = 0x00;
    out[3] = 0x00;
    out[4] = 0x63;
    out[5] = 0xE1;
    I2C_FM_block_write(out,6);
    addr = cnv_fm_addr(0xFE10);
    out[0] = addr&0xff;
    out[1] = (addr>>8)&0xff;
    out[2] = 0x03;
    out[3] = 0x03;
    out[4] = 0x00;
    out[5] = 0x00;
    I2C_FM_block_write(out,6);
    addr = cnv_fm_addr(0xFFEC);
    out[0] = addr&0xff;
    out[1] = (addr>>8)&0xff;
    wdata = SUM_D;
    out[2] = (wdata >> 24) & 0xFF;
    out[3] = (wdata >> 16) & 0xFF;
    out[4] = (wdata >>	8) & 0xFF;
    out[5] = (wdata >>	0) & 0xFF;
    I2C_FM_block_write(out,6);
    addr = cnv_fm_addr(0xFFF0);
    out[0] = addr&0xff;
    out[1] = (addr>>8)&0xff;
    wdata = (LEN_6 << 16) | (LEN_5 << 0);
    out[2] = (wdata >> 24) & 0xFF;
    out[3] = (wdata >> 16) & 0xFF;
    out[4] = (wdata >>	8) & 0xFF;
    out[5] = (wdata >>	0) & 0xFF;
    I2C_FM_block_write(out,6);
    addr = cnv_fm_addr(0xFFF4);
    out[0] = addr&0xff;
    out[1] = (addr>>8)&0xff;
    wdata = (LEN_3 << 16) | (LEN_1 << 0);
    out[2] = (wdata >> 24) & 0xFF;
    out[3] = (wdata >> 16) & 0xFF;
    out[4] = (wdata >>	8) & 0xFF;
    out[5] = (wdata >>	0) & 0xFF;
    I2C_FM_block_write(out,6);
    addr = cnv_fm_addr(0xFFF8);
    out[0] = addr&0xff;
    out[1] = (addr>>8)&0xff;
    wdata = (LEN_4 << 16) | (LEN_2 << 0);
    out[2] = (wdata >> 24) & 0xFF;
    out[3] = (wdata >> 16) & 0xFF;
    out[4] = (wdata >>	8) & 0xFF;
    out[5] = (wdata >>	0) & 0xFF;
    I2C_FM_block_write(out,6);
    addr = cnv_fm_addr(0xFFFC);
    out[0] = addr&0xff;
    out[1] = (addr>>8)&0xff;
    wdata = (SUMSEL << 31) | (AB_TYPE << 29) | (D_COPY << 27) | (SUM_C << 0);
    out[2] = (wdata >> 24) & 0xFF;
    out[3] = (wdata >> 16) & 0xFF;
    out[4] = (wdata >>	8) & 0xFF;
    out[5] = (wdata >>	0) & 0xFF;
    I2C_FM_block_write(out,6);
}
//  *****************************************************
//	Change FLASH Slave Address (Just after power on RESET)
//  *****************************************************
void Flash_sla_change (void)
{
    I2C_OIS_8bit_write(Fla_strl,(FLASH_SLVADR<<1));
    F024_Polling();
}
//  *****************************************************
//  Write SPI Param, Set FW, DSP data
//  *****************************************************
OIS_LONG bu24_write_flash_fw_data(void)
{
    OIS_ULONG		    sts = 1;
    OIS_UBYTE		    buf[Dat_len + 2];
    const OIS_UBYTE 	*param2;
    OIS_UWORD 		subaddr_FLASH_DATA_BIN_1;
    OIS_UWORD 		subaddr_FLASH_DATA_BIN_5;
    OIS_UWORD 		subaddr_FLASH_DATA_BIN_6;
#if    0
    OIS_UWORD 		subaddr_FLASH_DATA_BIN_7;
#endif
    OIS_ULONG		i, len, mod;
    //*** Access Mode ***//
    I2C_FM_8bit_write(FM_acc_mode, 0x01);
    //*** Disable Write protect ***//
    I2C_FM_8bit_write(FM_wri_pro, 0xA5);
#if    1
    //*** Erase exept for Calibration data area ***//
    Erase_flash();
#else
    // Erase All
    I2C_FM_8bit_write(FM_era, 0xEA);
    Wait(8000);//wait 8ms
#endif
    //*** Program Flash ***//
    Prog_flash();
    //*** Download FW ***//
    param2 = FLASH_DATA_BIN_1;
    len = FLASH_DATA_BIN_1_LEN / Dat_len;
    mod = FLASH_DATA_BIN_1_LEN % Dat_len;
    subaddr_FLASH_DATA_BIN_1 = FLASH_DATA_BIN_1_START;
    for (i = 0; i < len; i++) {
        buf[0] = (subaddr_FLASH_DATA_BIN_1 >> 8);
        buf[1] = (subaddr_FLASH_DATA_BIN_1 & 0xFF);
        memcpy(&buf[2],param2, Dat_len);  ///change all
        I2C_FM_block_write(buf,Dat_len + 2);
        subaddr_FLASH_DATA_BIN_1 += (Dat_len >> 2);
        param2  += Dat_len;
    }
    if(mod>0) {
        buf[0] = (subaddr_FLASH_DATA_BIN_1 >> 8);
        buf[1] = (subaddr_FLASH_DATA_BIN_1 & 0xFF);
        memcpy(&buf[2],param2, mod);  ///change all
        I2C_FM_block_write(buf,mod + 2);
    }else {
        CAM_INFO(CAM_OIS, "mod = %u",mod);
    }

    param2 = FLASH_DATA_BIN_5;
    len = FLASH_DATA_BIN_5_LEN / Dat_len;
    mod = FLASH_DATA_BIN_5_LEN % Dat_len;
    subaddr_FLASH_DATA_BIN_5 = FLASH_DATA_BIN_5_START;
    for (i = 0; i < len; i++) {
        buf[0] = (subaddr_FLASH_DATA_BIN_5 >> 8);
        buf[1] = (subaddr_FLASH_DATA_BIN_5 & 0xFF);
        memcpy(&buf[2], param2, Dat_len);
        I2C_FM_block_write(buf,Dat_len + 2);
        subaddr_FLASH_DATA_BIN_5 += (Dat_len >> 2);
        param2  += Dat_len;
    }
    if(mod>0) {
        buf[0] = (subaddr_FLASH_DATA_BIN_5 >> 8);
        buf[1] = (subaddr_FLASH_DATA_BIN_5 & 0xFF);
        memcpy(&buf[2],param2, mod);  ///change all
        I2C_FM_block_write(buf,mod + 2);
    }else {
        CAM_INFO(CAM_OIS, "mod = %u",mod);
    }

    param2 = FLASH_DATA_BIN_6;
    len = FLASH_DATA_BIN_6_LEN / Dat_len;
    mod = FLASH_DATA_BIN_6_LEN % Dat_len;
    subaddr_FLASH_DATA_BIN_6 = FLASH_DATA_BIN_6_START;
    for (i = 0; i < len; i++) {
        buf[0] = (subaddr_FLASH_DATA_BIN_6 >> 8);
        buf[1] = (subaddr_FLASH_DATA_BIN_6 & 0xFF);
        memcpy(&buf[2], param2, Dat_len);
        I2C_FM_block_write(buf,Dat_len + 2);
        subaddr_FLASH_DATA_BIN_6 += (Dat_len >> 2);
        param2  += Dat_len;
    }
    if(mod>0) {
        buf[0] = (subaddr_FLASH_DATA_BIN_6 >> 8);
        buf[1] = (subaddr_FLASH_DATA_BIN_6 & 0xFF);
        memcpy(&buf[2],param2, mod);  ///change all
        I2C_FM_block_write(buf,mod + 2);
    }else {
        CAM_INFO(CAM_OIS, "mod = %u",mod);
    }
#if    0
    /*
        Calibration Data region is involved in FLASH_DATA_BIN_7

        Therefore writing FLASH_DATA_BIN_7 is invalidated
        so that existing Calibration Data written by LuxVision cannot be overwritten.
    */
    param2 = FLASH_DATA_BIN_7;
    len = FLASH_DATA_BIN_7_LEN / Dat_len;
    mod = FLASH_DATA_BIN_7_LEN % Dat_len;
    subaddr_FLASH_DATA_BIN_7 = FLASH_DATA_BIN_7_START;

    for (i = 0; i < len; i++) {
        buf[0] = (subaddr_FLASH_DATA_BIN_7 >> 8);
        buf[1] = (subaddr_FLASH_DATA_BIN_7 & 0xFF);
        memcpy_s(&buf[2], param2, Dat_len);
        I2C_FM_block_write(buf,Dat_len + 2);
        subaddr_FLASH_DATA_BIN_7 += (Dat_len >> 2);
        param2  += Dat_len;
    }
    if(mod>0) {
        buf[0] = (subaddr_FLASH_DATA_BIN_7 >> 8);
        buf[1] = (subaddr_FLASH_DATA_BIN_7 & 0xFF);
        memcpy(&buf[2],param2, mod);  ///change all
        I2C_FM_block_write(buf,mod + 2);
    }
#endif
    return sts;
}

void WriteGyroGainToFlash(void)
{
    OIS_UWORD X_ori, Y_ori;
    X_ori = I2C_OIS_16bit__read(Gyro_gain_x);//read 0xF07A
    Y_ori = I2C_OIS_16bit__read(Gyro_gain_y);//read 0xF07C
    CAM_INFO(CAM_OIS, "[WriteGyroGainToFlash] will write  gl_GX_OFS= 0x%x  gl_GY_OFS= 0x%x",X_ori, Y_ori);

    Update_GyroCalib_to_flash(0, 0, X_ori, Y_ori);

    X_ori = I2C_OIS_16bit__read(Gyro_gain_x);//read 0xF07A
    Y_ori = I2C_OIS_16bit__read(Gyro_gain_y);//read 0xF07C
    CAM_INFO(CAM_OIS, "[WriteGyroGainToFlash] after write  gl_GX_OFS= 0x%x  gl_GY_OFS= 0x%x",X_ori, Y_ori);

}


void Gyro_gain_set(OIS_UWORD X_gain, OIS_UWORD Y_gain)
{
    OIS_UWORD X_ori, Y_ori;
    //X_ori = X_gain*I2C_OIS_16bit__read(Gyro_gain_x);//read 0xF07A
    //Y_ori = Y_gain*I2C_OIS_16bit__read(Gyro_gain_y);//read 0xF07C
    //I2C_OIS_16bit_write(Gyro_gain_x, X_ori);
    //I2C_OIS_16bit_write(Gyro_gain_y, Y_ori);
    X_ori = I2C_OIS_16bit__read(Gyro_gain_x);//read 0xF07A
    Y_ori = I2C_OIS_16bit__read(Gyro_gain_y);//read 0xF07C
    CAM_INFO(CAM_OIS, "[Gyro_gain_set] oldGyorGain  gl_GX_OFS= 0x%x  gl_GY_OFS= 0x%x",X_ori, Y_ori);
    CAM_INFO(CAM_OIS, "[Gyro_gain_set] newGyorGain  X_gain= 0x%x  Y_gain= 0x%x",X_gain, Y_gain);
    I2C_OIS_16bit_write(Gyro_gain_x, X_gain);
    I2C_OIS_16bit_write(Gyro_gain_y, Y_gain);

    X_ori = I2C_OIS_16bit__read(Gyro_gain_x);//read 0xF07A
    Y_ori = I2C_OIS_16bit__read(Gyro_gain_y);//read 0xF07C
    CAM_INFO(CAM_OIS, "[Gyro_gain_set] after write  gl_GX_OFS= 0x%x  gl_GY_OFS= 0x%x",X_ori, Y_ori);

}

struct _FACT_ADJ Gyro_offset_cal(void) {
    struct _FACT_ADJ resultGyorOffset = { 0x0000, 0x0000};
    OIS_ULONG diff = 0;

    //X offset
    I2C_OIS_8bit_write(Gyro_offset_lat, 0x00); //0xF088,0x00 select X axis
    Wait(70000);
    F024_Polling();
    resultGyorOffset.gl_GX_OFS = I2C_OIS_16bit__read(Gyro_offset_val);//0xF08A
    //Y offset
    I2C_OIS_8bit_write(Gyro_offset_lat, 0x01); //0xF088,0x01 select Y axis
    Wait(70000);
    F024_Polling();
    resultGyorOffset.gl_GY_OFS = I2C_OIS_16bit__read(Gyro_offset_val);//0xF08A


    CAM_INFO(CAM_OIS, "[Gyro_offset_cal] resultGyorOffset  gl_GX_OFS= 0x%x  gl_GY_OFS= 0x%x",resultGyorOffset.gl_GX_OFS, resultGyorOffset.gl_GY_OFS);

    //update to register
    I2C_OIS_8bit_write(Gyro_offset_req,0x00);//0xF09C,0x00 select X axis to update
    I2C_OIS_16bit_write(Gyro_offset_diff,resultGyorOffset.gl_GX_OFS);//write the X offset value to 0xF09D
    I2C_OIS_8bit_write(Gyro_offset_req,0x01);//0xF09C,0x01 select Y axis to update
    I2C_OIS_16bit_write(Gyro_offset_diff,resultGyorOffset.gl_GY_OFS);//write the Y offset value to 0xF09D

    //verify gyro offset
    I2C_OIS_8bit_write(Gyro_offset_req,0x02);//0xF09C,0x02 to output X diff value
    F024_Polling();
    diff = I2C_OIS_16bit__read(Gyro_offset_diff);//read 0xF09D to get the X diff value
    CAM_INFO(CAM_OIS, "[Gyro_offset_cal] Gyro_offset_diff X = 0x%x",diff);
    I2C_OIS_8bit_write(Gyro_offset_req,0x03);//0xF09C,0x02 to output Y diff value
    F024_Polling();
    diff = I2C_OIS_16bit__read(Gyro_offset_diff);//read 0xF09D to get the X diff value
    CAM_INFO(CAM_OIS, "[Gyro_offset_cal] Gyro_offset_diff Y = 0x%x",diff);


    Update_GyroCalib_to_flash(resultGyorOffset.gl_GX_OFS, resultGyorOffset.gl_GY_OFS,0 ,0);


    return resultGyorOffset;
}

void Update_GyroCalib_to_flash(OIS_UWORD gyro_X_offset, OIS_UWORD gyro_Y_offset, OIS_UWORD gyro_X_gain, OIS_UWORD gyro_Y_gain)
{

    OIS_UBYTE 	buf[0x200 + 2];
    OIS_ULONG 	rdata;
    OIS_ULONG 	i, j;
    OIS_UBYTE 	buf_34[0x20+2];

    I2C_FM_8bit_write(FM_acc_mode, 0x01);//0xF000,0x01
    for (i = 0; i < (0x200 / 4); i++) {
        // READ 512byte from Start address of Gyro Calibration data on FLASH before ERASE
        rdata = I2C_FM_32bit__read(FLASH_ADDR_GYRO_CALIB + i);

        buf[((i * 4) + 0) + 2] = (rdata >> 24);        // Store rdata[31:24]
        buf[((i * 4) + 1) + 2] = (rdata >> 16);        // Store rdata[23:16]
        buf[((i * 4) + 2) + 2] = (rdata >>  8);        // Store rdata[15: 8]
        buf[((i * 4) + 3) + 2] = (rdata >>  0);        // Store rdata[ 7: 0]
    }
    for(i = 0; i<=15; i++) {
        CAM_INFO(CAM_OIS, "[Gyro_offset_cal] before update :buf[%d] = 0x%x", i , buf[i]);
    }

    // Disable Write protect
    I2C_FM_8bit_write(FM_wri_pro, 0xA5);//0xFF0F

    // ERASE 512byte from Start address of Gyro Calibration data on FLASH
    I2C_FM_8bit_write(FLASH_ADDR_GYRO_CALIB | 0x8000, 0xE0);//0xF800 >> 2
    Wait(2000);//wait 2ms

    /* OverWrite Gyro Calibration data and PROGRAM to FLASH */
    buf[0] = (FLASH_ADDR_GYRO_CALIB >> 8);
    buf[1] = (FLASH_ADDR_GYRO_CALIB & 0xFF);
    if ((gyro_X_offset != 0)|(gyro_Y_offset != 0)) {
        buf[2] = (gyro_X_offset >> 8);
        buf[3] = (gyro_X_offset & 0xFF);
        buf[4] = (gyro_Y_offset >> 8);
        buf[5] = (gyro_Y_offset & 0xFF);//3D80 gyro offset
    }
    if ((gyro_X_gain != 0)&(gyro_Y_gain != 0)) {
        buf[6] = (gyro_X_gain >> 8);
        buf[7] = (gyro_X_gain & 0xFF);
        buf[8] = (gyro_Y_gain >> 8);
        buf[9] = (gyro_Y_gain & 0xFF);//3D81 gyro gain
        //3D82 NONE
        buf[14] = 0xCC;
        buf[15] = 0x01; //mark address for update
    }
    for (i = 0; i < (0x200 / 32); i++) {
        buf_34[0] = (FLASH_ADDR_GYRO_CALIB + i*8) >> 8;
        buf_34[1] = ((FLASH_ADDR_GYRO_CALIB + i*8) & 0xFF);
        for (j = 0; j < 0x20; j++) {
            buf_34[2 + j] = buf[i*32 + 2 + j];
        }
        I2C_FM_block_write(buf_34, 0x20 + 2);
    }

    for (i = 0; i < (0x200 / 4); i++) {
        // READ 512byte from Start address of Gyro Calibration data on FLASH before ERASE
        rdata = I2C_FM_32bit__read(FLASH_ADDR_GYRO_CALIB + i);

        buf[((i * 4) + 0) + 2] = (rdata >> 24);        // Store rdata[31:24]
        buf[((i * 4) + 1) + 2] = (rdata >> 16);        // Store rdata[23:16]
        buf[((i * 4) + 2) + 2] = (rdata >>  8);        // Store rdata[15: 8]
        buf[((i * 4) + 3) + 2] = (rdata >>  0);        // Store rdata[ 7: 0]
    }

    for(i = 0; i<=15; i++) {
        CAM_INFO(CAM_OIS, "[Gyro_offset_cal] after update :buf[%d] = 0x%x", i , buf[i]);
    }

    Wait(1000);
}

void OIS_mode_set(OIS_UWORD mode)
{
    if(mode == 0) {
        F024_Polling();
        I2C_OIS_8bit_write(OIS_control, OIS_standby);
    } else if(mode == 1) {
        F024_Polling();
        I2C_OIS_8bit_write(OIS_control, Servo_on);
    } else if(mode == 2) {
        F024_Polling();
        I2C_OIS_8bit_write(OIS_control, OIS_on);
    } else if(mode == 4) {
        F024_Polling();
        I2C_OIS_8bit_write(OIS_control, Servo_off);
    }
}
void OIS_manual_set(OIS_UWORD X_pos, OIS_UWORD Y_pos)
{
    I2C_OIS_8bit_write(OIS_control, Servo_on);
    F024_Polling();
    I2C_OIS_16bit_write(Reg_x_pos, X_pos);
    I2C_OIS_16bit_write(Reg_y_pos, Y_pos);
    I2C_OIS_8bit_write(Manu_en, 0x03);
    Wait(50000); //*** wait 50ms for big step ***//
    F024_Polling();
}
void Update_Gyro_offset_gain_cal_from_flash(void)
{
    //*** Just After FLASH boot is successful ***//
    OIS_UWORD en_gyrogain_update;
/*
    OIS_UBYTE 	buf[0x200 + 2];
    OIS_ULONG 	rdata;
    OIS_ULONG 	i;
    for (i = 0; i < (0x200 / 4); i++) {
        // READ 512byte from Start address of Gyro Calibration data on FLASH before ERASE
        rdata = I2C_FM_32bit__read(FLASH_ADDR_GYRO_CALIB + i);

        buf[((i * 4) + 0) + 2] = (rdata >> 24);        // Store rdata[31:24]
        buf[((i * 4) + 1) + 2] = (rdata >> 16);        // Store rdata[23:16]
        buf[((i * 4) + 2) + 2] = (rdata >>  8);        // Store rdata[15: 8]
        buf[((i * 4) + 3) + 2] = (rdata >>  0);        // Store rdata[ 7: 0]
    }

    for(i = 0; i<=15; i++) {
        CAM_DBG(CAM_OIS, "[Update_Gyro_offset_gain_cal_from_flash] after update :buf[%d] = 0x%x", i , buf[i]);
    }
*/

    en_gyrogain_update = I2C_FM_16bit__read(FLASH_ADDR_GYRO_CALIB + 3); //3D83 check flag data
    CAM_DBG(CAM_OIS, "[Update_Gyro_offset_gain_cal_from_flash] en_gyrogain_update = 0x%x", en_gyrogain_update);
    //*** Notice the Start address of Gyro Calibration data on FLASH ***//
    I2C_OIS_16bit_write(0xF1E2, FLASH_ADDR_GYRO_CALIB);

    if (en_gyrogain_update == 0xCC01) {
        //*** Gyro Gain Update from FLASH is performed ***//
        I2C_OIS_8bit_write(0xF1E4, 1);
    } else {
        //*** Gyro Gain Update from FLASH is not performed ***//
        I2C_OIS_8bit_write(0xF1E4, 0);
    }
}
#define ACCURACY 0x17  //?��8um accuracy
void Circle_test(void)
{

    OIS_UWORD Error = 0;
    OIS_UWORD Error_x, Error_y, Target_x, Target_y;
    I2C_OIS_8bit_write(OIS_control, Servo_on);
    Wait(100);
    F024_Polling();
    I2C_OIS_8bit_write(CIR_thr, ACCURACY);
    F024_Polling();
    I2C_OIS_8bit_write(CIR_amp_x, 0x40);  //*** radius 200um ***//
    F024_Polling();
    I2C_OIS_8bit_write(CIR_amp_y, 0x40);
    F024_Polling();
    I2C_OIS_8bit_write(CIR_sta, 0x00);
    Wait(1000000);
    Wait(1000000);
    Wait(1000000);
    Wait(1000000);
    Wait(1000000);
    F024_Polling();
    Error = I2C_OIS_8bit__read(CIR_stu);
    if (Error == 4) {
        //printf("CIR test OK");
    } else {
        Target_x = I2C_OIS_16bit__read(CIR_tar_x);
        Target_y = I2C_OIS_16bit__read(CIR_tar_y);
        Error_x = I2C_OIS_16bit__read(CIR_err_x);
        Error_y = I2C_OIS_16bit__read(CIR_err_y);
        //printf("Target_x = %04X Error_x = %04X\n Target_y = %04X Error_y = %04X\n", Target_x, Error_x, Target_y, Error_y);
        //rdata_x = I2C_OIS_read_burst(0xE200, 0x200)
        //rdata_y = I2C_OIS_read_burst(0xE400, 0x200)
    }

}
void OIS_mag_check(OIS_UWORD *srv_x, OIS_UWORD *srv_y)
{

    OIS_mode_set(1);
    Wait(20000);
    F024_Polling();
    OIS_mode_set(4);
    F024_Polling();
    I2C_OIS_8bit_write(ADC_seq, ADC_x);
    F024_Polling();
    *srv_x = I2C_OIS_16bit__read(ADC_val);
    I2C_OIS_8bit_write(ADC_seq, ADC_y);
    F024_Polling();
    *srv_y = I2C_OIS_16bit__read(ADC_val);
    OIS_mode_set(1);
}

void OIS_cal_check(void)
{
    //OIS_UBYTE	cal_dat[128];
    //SHT3x_full_init(FLASH_SLVADR);//change IIC slave address to Flash
    //I2C_read_status(0x3F70,cal_dat,128);
    //SHT3x_full_init(SLV_OIS_24721);
}

void OIS_soft_reset(OIS_ULONG Prog_ID)
{
    if(Prog_ID!=BU24721_FW_VERSION) {
        I2C_OIS_8bit_write(OIS_reset_1,0x00);
        Wait(100);
        F024_Polling();
        I2C_OIS_8bit_write(OIS_reset_2,0x00);
        Wait(6000);//wait 6ms
        F024_Polling();
        I2C_OIS_8bit_write(OIS_release_Standby,0x00);
        Wait(100);
        F024_Polling();
    }
}

void Gyro_select(OIS_UBYTE Gyro_sel, bool isTeleOisUseMonitor)
{
    //OIS_UBYTE gyro_check;
    OIS_ULONG SPI_Mode_check;
    switch(Gyro_sel) {
        //*** INVENSENSE ICM42631 ***//
    case 0:
        I2C_OIS_8bit_write(0xf12b, 0x0);

        //I2C_OIS_8bit_write(GYRO_sel_adr, Q_ICM42631);//Enable setting of Qualcomm:ICM42631
        F024_Polling();
        if (isTeleOisUseMonitor) {
            I2C_OIS_8bit_write(OIS_SPI, SPI_Monitor);
        } else {
            I2C_OIS_8bit_write(OIS_SPI, SPI_Master);//Gyro Master/Monitor Switch
        }
        F024_Polling();
        SPI_Mode_check = I2C_OIS_8bit__read(OIS_SPI);
        CAM_INFO(CAM_OIS, "SPI_Mode_check:%d ", SPI_Mode_check);
        F024_Polling();
        //SPI Master setting//
        if(SPI_Mode_check == 0) {
            I2C_OIS_8bit_write(OIS_gyro_mode, OIS_gyro_con);
            F024_Polling();
            I2C_OIS_8bit_write(OIS_gyro_adr, 0x76);
            I2C_OIS_8bit_write(OIS_gyro_dat, 0x02);
            I2C_OIS_8bit_write(OIS_gyro_adr, 0x45);
            I2C_OIS_8bit_write(OIS_gyro_dat, 0x5B);
            I2C_OIS_8bit_write(OIS_gyro_adr, 0x44);
            I2C_OIS_8bit_write(OIS_gyro_dat, 0x03);
            //Wait(45000);	//*** wait 45ms according to gyro spec ***//
            //I2C_OIS_8bit_write(OIS_SPI, SPI_Master);
            //F024_Polling();
            //I2C_OIS_8bit_write(OIS_gyro_mode, OIS_gyro_on);
            F024_Polling();
            //printf("Select ICM42631 for SPI Master\n");
        }
        //SPI Monitor setting//
        else {
            I2C_OIS_8bit_write(OIS_gyro_mode, OIS_gyro_on);
            F024_Polling();
            //printf("Select ICM42631 for SPI Monitor\n");
        }
        //gyro_check = I2C_OIS_8bit__read(GYRO_check_adr);
        break;
        //*** ST LSM6DSOQ ***//
    case 1:
        I2C_OIS_8bit_write(GYRO_sel_adr, M_BMI260);//Enable setting of MTK:BMI260
        F024_Polling();
        I2C_OIS_8bit_write(OIS_SPI, SPI_Monitor);//0xF02A,0x04
        F024_Polling();
        I2C_OIS_8bit_write(OIS_gyro_mode, OIS_gyro_on);//0xF023,0x00
        F024_Polling();
        //printf("Select BMI260 for SPI Monitor\n");
        //gyro_check = I2C_OIS_8bit__read(GYRO_check_adr);//0xF12D
        break;
        //*** BOSCH BMI260 ***//
    case 2:
        I2C_OIS_8bit_write(GYRO_sel_adr, M_BMI260);//Enable setting of MTK:ICM42631
        F024_Polling();
        I2C_OIS_8bit_write(OIS_SPI, SPI_Monitor);//0xF02A,0x04
        F024_Polling();
        I2C_OIS_8bit_write(OIS_gyro_mode, OIS_gyro_on);//0xF023,0x00
        F024_Polling();
        //printf("Select ICM42631 for SPI Monitor\n");
        //gyro_check = I2C_OIS_8bit__read(GYRO_check_adr);//0xF12D
        break;
    default:
        break;
    }

}
void Gyro_data(OIS_UBYTE count, OIS_UBYTE Gyro_sel)
{
    OIS_UBYTE i;
    OIS_UWORD gyro_x, gyro_y;
    switch(Gyro_sel) {
        //*** ST LSM6DSOQ ***//
    case 1:
        for(i = 0; i < count; i++) {
            gyro_x = I2C_OIS_16bit__read(0xF0E0);
            gyro_y = I2C_OIS_16bit__read(0xF0E6);
            Wait(10000);
        }
        break;
    case 2:
        for(i = 0; i < count; i++) {
            gyro_x = I2C_OIS_16bit__read(0xF0E6);
            gyro_y = I2C_OIS_16bit__read(0xF0E0);
            Wait(10000);
        }
        break;
    case 3:
        for(i = 0; i < count; i++) {
            gyro_x = I2C_OIS_16bit__read(0xF0E2);
            gyro_y = I2C_OIS_16bit__read(0xF0E4);
            Wait(10000);
            //printf("gyro_x %d gyro_y %d\n", H2D(revert_b(gyro_x)), H2D(revert_b(gyro_y)));
        }
        break;
    default:
        break;
    }
}

void Boot_err_sla_change (void)
{
    OIS_UBYTE out[10];
    I2C_OIS_8bit_write(OIS_boot_mode,0x55);
    F024_Polling();

    OIS_soft_reset(0);

    I2C_OIS_8bit_write(OIS_boot_mode,0x00);
    F024_Polling();
    out[0] = 0xF0;
    out[1] = 0x80;
    out[2] = 0xC4;
    out[3] = 0x00;
    out[4] = 0x44;
    out[5] = 0xE1;
    I2C_OIS_block_write(out,6);
    F024_Polling();

    out[0] = 0xF0;
    out[1] = 0x84;
    out[2] = 0x00;
    out[3] = 0x00;
    out[4] = 0x52;
    out[5] = FLASH_SLVADR;
    I2C_OIS_block_write(out,6);
    F024_Polling();
}


int Rohm_bu24721_fw_download()
{
    OIS_ULONG	Prog_ID;
    int ret = 0;

    I2C_OIS_8bit_write(OIS_release_Standby, 0x00);  //release standby
    ret = F024_Polling();
    CAM_INFO(CAM_OIS, "polling ret  = %d",ret);

    ret = I2C_OIS_32bit__read(FW_ID, &Prog_ID);//check 0xF01C program ID
    if (ret < 0) {
        CAM_ERR(CAM_OIS, "read Prog_ID failed! exit bu24721 fw download");
        return ret;
    } else {
        CAM_INFO(CAM_OIS, "Prog_ID = %08x FW_Version = %08x",Prog_ID,BU24721_FW_VERSION);
    }

    if(Prog_ID < BU24721_FW_VERSION) {
        if(Prog_ID == 0) {
            Boot_err_sla_change();
            CAM_ERR(CAM_OIS, "boot err change flash to %x",FLASH_SLVADR<<1);
        } else {
            Flash_sla_change(); //*** change Flash salve address to 0x70 ***//
        }
        bu24_write_flash_fw_data();

        OIS_soft_reset(Prog_ID);

        ret = I2C_OIS_32bit__read(FW_ID, &Prog_ID);//check 0xF01C program ID
        CAM_INFO(CAM_OIS, "After DL Prog_ID = %08x FW ID:%8x",Prog_ID,BU24721_FW_VERSION);
        if(BU24721_FW_VERSION == Prog_ID) {
            CAM_INFO(CAM_OIS, "Tele DL sucessful");
            ret = 0;
        } else {
            CAM_ERR(CAM_OIS, "Tele DL Failed");
            ret = -1;
        }

    } else {
        CAM_DBG(CAM_OIS, "FW is New no need DL FW");
        ret = 0;
    }

    I2C_OIS_8bit_write(0xf020, 0x01);
    F024_Polling();

    return ret;
}

int bu24721_do_push_center()
{
    int rc = 0;

    rc = I2C_OIS_8bit_write(OIS_control, 0x01);
    CAM_INFO(CAM_OIS, "I2C_OIS_8bit_write OIS_control(%x), rc  = %d", OIS_control, rc);
    if (0 == rc)
    {
        Wait(100);
        rc = I2C_OIS_8bit_write(OIS_status, 0x01);
        CAM_INFO(CAM_OIS, "I2C_OIS_8bit_write OIS_status(%x), rc  = %d", OIS_status, rc);
    }

    if (0 == rc)
    {
        Wait(70000);
        F024_Polling();
    }

    return rc;
}



