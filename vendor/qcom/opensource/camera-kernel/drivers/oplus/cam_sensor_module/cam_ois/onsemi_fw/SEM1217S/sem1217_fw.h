/////////////////////////////////////////////////////////////////////////////
// File Name	: BU24721_FW_H
// Function		: Header file

//
// Copyright(c)	Rohm Co.,Ltd. All rights reserved
//
/***** ROHM Confidential ***************************************************/
#ifndef SEM1217_FW_H
#define SEM1217_FW_H

#define SEM1217S_TX_SIZE_32_BYTE  32
#define SEM1217S_TX_SIZE_64_BYTE  64
#define SEM1217S_TX_SIZE_128_BYTE  128
#define SEM1217S_TX_SIZE_256_BYTE  256
#define SEM1217S_TX_BUFFER_SIZE  SEM1217S_TX_SIZE_256_BYTE
#define SEM1217S_RX_BUFFER_SIZE  4


#define SEM1217S_REG_OIS_CTRL  0x0000
#define SEM1217S_REG_OIS_STS  0x0001
#define SEM1217S_REG_AF_CTRL  0x0200
#define SEM1217S_REG_AF_STS   0x0201
#define SEM1217S_REG_FWUP_CTRL  0x1000
#define SEM1217S_REG_FWUP_ERR  0x1001
#define SEM1217S_REG_FWUP_CHKSUM  0x1002
#define SEM1217S_REG_APP_VER  0x1008
#define SEM1217S_REG_DATA_BUF  0x1100

#define SEM1217S_OIS_OFF   0x00
#define SEM1217S_AF_OFF   0x00
#define SEM1217S_STATE_READY  0x01
#define SEM1217S_STATE_INIT  0x00
#define SEM1217S_RESET_REQ   0x80
#define SEM1217S_FWUP_CTRL_32_SET  0x01
#define SEM1217S_FWUP_CTRL_64_SET  0x03
#define SEM1217S_FWUP_CTRL_128_SET  0x05
#define SEM1217S_FWUP_CTRL_256_SET  0x07
#define SEM1217S_APP_FW_SIZE  (48 * 1024)

#define SEM1217S_REG_GCAL_CTRL 0x0600
#define SEM1217S_REG_GX_OFFSET 0x0604
#define SEM1217S_REG_GY_OFFSET 0x0606
#define SEM1217S_REG_OIS_ERR 0x0004
#define SEM1217S_G_OFFSET_EN 0x01
#define SEM1217S_ERR_GCALX 0x0100
#define SEM1217S_ERR_GCALY 0x0200
#define SEM1217S_NO_ERROR 0x0000

#define SEM1217S_REGINFO_BLK_UP_CTRL 0x0300
#define SEM1217S_OIS_INFO_EN 0x01
#define SEM1217S_ERR_ODI 0x0040

#define SEM1217S_HALL_DATA_START  0x1100

#define SEM1217S_REG_OIS_MODE  0x0002
#define SEM1217S_REG_GX_GAIN 0x0608
#define SEM1217S_REG_GY_GAIN 0x060C
#define SEM1217S_OIS_ON   0x01
#define SEM1217S_STILL_MODE   0x00
#define SEM1217S_GX_GAIN_EN 0x02
#define SEM1217S_GY_GAIN_EN 0x04


struct SEM1217S_FACT_ADJ{
	uint16_t gl_GX_OFS;
	uint16_t gl_GY_OFS;
};

struct SEM1217S_FACT_ADJ SEM1217S_Gyro_offset_cal(void);
int sem1217s_fw_download (struct cam_ois_ctrl_t *o_ctrl);

void SEM1217S_WriteGyroGainToFlash(void);
void SEM1217S_Gyro_gain_set(uint32_t X_gain, uint32_t Y_gain);
int sem1217_ois_lock(struct cam_ois_ctrl_t *o_ctrl);
void dump_ois_sem1217s_hall_data(struct cam_ois_ctrl_t *o_ctrl);

#endif  // BU24721_FW_H
