/*
 * Copyright (c) 2021 ZEKU Technology(Shanghai) Corp.,Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * Basecode Created :        2020/09/28 Author: wangman@zeku.com
 *
 */

#ifndef _MMC_CORE_SD_ZK_H
#define _MMC_CORE_SD_ZK_H

#include <linux/types.h>
#include <linux/mmc/sdio.h>

/*******************************************************************/
/*  General Registers of function 0                                */
/*******************************************************************/
#define ZK_SDIO_CCCR_CCCR	(SDIO_CCCR_CCCR)
#define ZK_SDIO_CCCR_SD		(SDIO_CCCR_SD)
#define ZK_SDIO_CCCR_IOEx	(SDIO_CCCR_IOEx)
#define ZK_SDIO_CCCR_IORx	(SDIO_CCCR_IORx)
#define ZK_SDIO_CCCR_IENx	(SDIO_CCCR_IENx)
#define ZK_SDIO_CCCR_INTx	(SDIO_CCCR_INTx)
#define ZK_SDIO_CCCR_ABORT	(SDIO_CCCR_ABORT)
#define ZK_SDIO_CCCR_IF		(SDIO_CCCR_IF)
#define ZK_SDIO_CCCR_CAPS	(SDIO_CCCR_CAPS)
#define ZK_SDIO_CCCR_CIS	(SDIO_CCCR_CIS)
#define ZK_SDIO_CCCR_CIS0	(SDIO_CCCR_CIS)
#define ZK_SDIO_CCCR_CIS1	(SDIO_CCCR_CIS+1)
#define ZK_SDIO_CCCR_CIS2	(SDIO_CCCR_CIS+2)

//#define ZK_SDIO_CCCR_SUSPEND	(SDIO_CCCR_SUSPEND)	/*standard sdio register, not supported in zk sdio*/
//#define ZK_SDIO_CCCR_SELx		(SDIO_CCCR_SELx)	/*standard sdio register, not supported in zk sdio*/
//#define ZK_SDIO_CCCR_EXECx	(SDIO_CCCR_EXECx)	/*standard sdio register, not supported in zk sdio*/
//#define ZK_SDIO_CCCR_READYx	(SDIO_CCCR_READYx)	/*standard sdio register, not supported in zk sdio*/
#define ZK_SDIO_CCCR_BLKSIZE	(SDIO_CCCR_BLKSIZE)
#define ZK_SDIO_CCCR_BLKSIZE0	(SDIO_CCCR_BLKSIZE)
#define ZK_SDIO_CCCR_BLKSIZE1	(SDIO_CCCR_BLKSIZE+1)
#define ZK_SDIO_CCCR_POWER		(SDIO_CCCR_POWER)
#define ZK_SDIO_CCCR_SPEED		(SDIO_CCCR_SPEED)
#define ZK_SDIO_CCCR_UHS		(SDIO_CCCR_UHS)
//#define ZK_SDIO_CCCR_DRIVE_STRENGTH	(SDIO_CCCR_DRIVE_STRENGTH)	/*standard sdio register, not supported in zk sdio*/

#define ZK_SDIO_CCCR_FUNC_INTx		(0xf0)	/*interupt pending for func0-7 on card side*/
#define ZK_SDIO_CCCR_FUNC_INTMSKx	(0xf1)	/*interupt mask for corresponding function on card side*/
#define ZK_SDIO_CCCR_BACKDOOR1		(0xf2)
#define ZK_SDIO_CCCR_BACKDOOR2		(0xf3)

#define	ZK_SDIO_FBR_STD_IF_FUNCx(f)	(SDIO_FBR_BASE(f)+SDIO_FBR_STD_IF)
#define	ZK_SDIO_FBR_POWER_FUNCx(f)	(SDIO_FBR_BASE(f)+SDIO_FBR_POWER)
#define	ZK_SDIO_FBR_CIS_FUNCx(f)	(SDIO_FBR_BASE(f)+SDIO_FBR_CIS)	/* CIS pointer (3 bytes) */
#define	ZK_SDIO_FBR_CIS0_FUNCx(f)	(SDIO_FBR_BASE(f)+SDIO_FBR_CIS)
#define	ZK_SDIO_FBR_CIS1_FUNCx(f)	(SDIO_FBR_BASE(f)+SDIO_FBR_CIS+1)
#define	ZK_SDIO_FBR_CIS2_FUNCx(f)	(SDIO_FBR_BASE(f)+SDIO_FBR_CIS+2)

#define	ZK_SDIO_FBR_CSA_FUNCx(f)	(SDIO_FBR_BASE(f)+SDIO_FBR_CSA) /* CSA pointer (3 bytes) */
#define	ZK_SDIO_FBR_CSA0_FUNCx(f)	(SDIO_FBR_BASE(f)+SDIO_FBR_CSA)
#define	ZK_SDIO_FBR_CSA1_FUNCx(f)	(SDIO_FBR_BASE(f)+SDIO_FBR_CSA+1)
#define	ZK_SDIO_FBR_CSA2_FUNCx(f)	(SDIO_FBR_BASE(f)+SDIO_FBR_CSA+2)
//#define ZK_SDIO_FBR_CSA_DATA_FUNCx(f)	(SDIO_FBR_BASE(f)+SDIO_FBR_CSA_DATA)	/*standard sdio register, not supported in zk sdio*/
#define	ZK_SDIO_FBR_BLKSIZE_FUNCx(f)	(SDIO_FBR_BASE(f)+SDIO_FBR_BLKSIZE) /* block size (2 bytes) */
#define	ZK_SDIO_FBR_BLKSIZE0_FUNCx(f)	(SDIO_FBR_BASE(f)+SDIO_FBR_BLKSIZE)
#define	ZK_SDIO_FBR_BLKSIZE1_FUNCx(f)	(SDIO_FBR_BASE(f)+SDIO_FBR_BLKSIZE+1)


/*******************************************************************/
/*  General Registers of function 1                                */
/*******************************************************************/
#define SDIO_FN1R_H2C_INT_EVENT		(0x00)
#define SDIO_FN1R_HOST_INT_CLR_SEL	(0x01)	/*host interrupt clear select register*/
#define SDIO_FN1R_HOST_INT_STSMSK	(0x02)	/*host interrupt status mask*/
#define SDIO_FN1R_HOST_INT_STS		(0x03)	/*host interrupt status*/
/* irq status shift */
#define SDIO_IRQ_UPLD_HOST_SHIFT	(0x0U)
#define SDIO_IRQ_DPLD_HOST_SHIFT	(0x1U)
#define SDIO_IRQ_UNDERFLOW_HOST_SHIFT	(0x2U)
#define SDIO_IRQ_OVERFLOW_HOST_SHIFT	(0x3U)
#define SDIO_IRQ_MISC1_HOST_SHIFT	(0x4U)
#define SDIO_IRQ_MISC2_HOST_SHIFT	(0x5U)
#define SDIO_IRQ_INT0_HOST_SHIFT	(0x6U)
#define SDIO_IRQ_INT1_HOST_SHIFT	(0x7U)

#define SDIO_IRQ_UPLD_HOST_MASK		(0x1U << SDIO_IRQ_UPLD_HOST_SHIFT)
#define SDIO_IRQ_DPLD_HOST_MASK		(0x1U << SDIO_IRQ_DPLD_HOST_SHIFT)
#define SDIO_IRQ_UNDERFLOW_HOST_MASK	(0x1U << SDIO_IRQ_UNDERFLOW_HOST_SHIFT)
#define SDIO_IRQ_OVERFLOW_HOST_MASK	(0x1U << SDIO_IRQ_OVERFLOW_HOST_SHIFT)
#define SDIO_IRQ_MISC1_HOST_MASK	(0x1U << SDIO_IRQ_MISC1_HOST_SHIFT)
#define SDIO_IRQ_MISC2_HOST_MASK	(0x1U << SDIO_IRQ_MISC2_HOST_SHIFT)
#define SDIO_IRQ_INTC0_HOST_MASK	(0x1U << SDIO_IRQ_INT0_HOST_SHIFT)
#define SDIO_IRQ_INTC1_HOST_MASK	(0x1U << SDIO_IRQ_INT1_HOST_SHIFT)



#define SDIO_FN1R_HOST_TRANSFER_STS	(0x28)	/*host transfer status*/
#define SDIO_FN1R_C2H_INT_EVENT		(0x30)
#define SDIO_FN1R_IO_READY		(0x1 << 3)
#define SDIO_FN1R_CARD_INT_STSMSK	(0x34)
#define SDIO_FN1R_CARD_INT_STS		(0x38)
#define SDIO_FN1R_CARD_INT_CLR_SEL	(0x3c)	/*card interrupt clear select register*/

#define SDIO_FN1R_DMA_RD_BASE_ADDR	(0x40)	/*4 bytes, DMA read start base address*/
#define SDIO_FN1R_DMA_RD_BASE_ADDR0	(0x40)
#define SDIO_FN1R_DMA_RD_BASE_ADDR1	(0x41)
#define SDIO_FN1R_DMA_RD_BASE_ADDR2	(0x42)
#define SDIO_FN1R_DMA_RD_BASE_ADDR3	(0x43)

#define SDIO_FN1R_DMA_WR_BASE_ADDR	(0x44)	/*4 bytes, DMA write start base address*/
#define SDIO_FN1R_DMA_WR_BASE_ADDR0	(0x44)
#define SDIO_FN1R_DMA_WR_BASE_ADDR1	(0x45)
#define SDIO_FN1R_DMA_WR_BASE_ADDR2	(0x46)
#define SDIO_FN1R_DMA_WR_BASE_ADDR3	(0x47)

#define SDIO_FN1R_CHIP_REV			(0x5c)	/*1 bytes*/
#define SDIO_FN1R_SDU_IP_REV		(0x5e)	/*2 bytes*/
#define SDIO_FN1R_SDU_IP_REV_MINOR	(0x5e)
#define SDIO_FN1R_SDU_IP_REV_MAJOR	(0x5f)

#define SDIO_FN1R_OCR	(0x68)	/*3 bytes*/
#define SDIO_FN1R_OCR0	(0x68)
#define SDIO_FN1R_OCR1	(0x69)
#define SDIO_FN1R_OCR2	(0x6a)

#define SDIO_FN1R_CONFIG	(0x6b)
#define SDIO_FN1R_MISC_CONFIG	(0x6c)		/*4 bytes*/
#define SDIO_FN1R_MISC_CONFIG0	(0x6c)
#define SDIO_FN1R_MISC_CONFIG1	(0x6d)
#define SDIO_FN1R_MISC_CONFIG2	(0x6e)
#define SDIO_FN1R_MISC_CONFIG3	(0x6f)

#define SDIO_FN1R_CARD_TESTBUS0	(0x70)
#define SDIO_FN1R_CARD_TESTBUS1	(0x71)
#define SDIO_FN1R_CARD_TESTBUS2	(0x72)
#define SDIO_FN1R_CARD_TESTBUS3	(0x73)

#define SDIO_FN1R_DMA_ADDR	(0x74)		/*4 bytes*/
#define SDIO_FN1R_DMA_ADDR0	(0x74)
#define SDIO_FN1R_DMA_ADDR1	(0x75)
#define SDIO_FN1R_DMA_ADDR2	(0x76)
#define SDIO_FN1R_DMA_ADDR3	(0x77)

#define SDIO_FN1R_CARD_ACTINTMSK	(0x74)
#define SDIO_FN1R_CARD_INTSTS		(0x75)
#define SDIO_FN1R_HOST_ACTINTMSK	(0x76)
#define SDIO_FN1R_HOST_INTSTS		(0x77)

#define SDIO_FN1R_SD_IOADDR		(0x78)	/*3 bytes*/
#define SDIO_FN1R_SD_IOADDR0	(0x78)
#define SDIO_FN1R_SD_IOADDR1	(0x79)
#define SDIO_FN1R_SD_IOADDR2	(0x7a)

#define SDIO_FN1R_SOC_CTRL_REG0	(0x800)	/*4 bytes*/
#define SDIO_FN1R_SOC_CTRL_REG1	(0x804)	/*4 bytes*/
#define SDIO_FN1R_SOC_CTRL_REG2	(0x808)	/*4 bytes*/
#define SDIO_FN1R_SOC_CTRL_REG3	(0x80C)	/*4 bytes*/

#define SDIO_FN1R_SOC_CTRL_REG0B0	(0x800)
#define RAMDUMP_EN_SDU			(1 << 6)
#define SDIO_FN1R_SOC_CTRL_REG0B1	(0x801)
#define SDIO_FN1R_SOC_CTRL_REG0B2	(0x802)
#define SDIO_FN1R_SOC_CTRL_REG0B3	(0x803)

#define SDIO_FN1R_SOC_CTRL_REG1B0	(0x804)
#define SDIO_FN1R_SOC_CTRL_REG1B1	(0x805)
#define SDIO_FN1R_SOC_CTRL_REG1B2	(0x806)
#define SDIO_FN1R_SOC_CTRL_REG1B3	(0x807)
#define RAMDUMP_TYPE_SDU		(1 << 4)

#define SDIO_FN1R_SOC_CTRL_REG2B0	(0x808)
#define SDIO_FN1R_SOC_CTRL_REG2B1	(0x809)
#define SDIO_FN1R_SOC_CTRL_REG2B2	(0x80a)
#define SDIO_FN1R_SOC_CTRL_REG2B3	(0x80b)

#define SDIO_FN1R_SOC_CTRL_REG3B0	(0x80c)
#define SDIO_FN1R_SOC_CTRL_REG3B1	(0x80d)
#define SDIO_FN1R_SOC_CTRL_REG3B2	(0x80e)
#define SDIO_FN1R_SOC_CTRL_REG3B3	(0x80f)


#define SDIO_FN1R_SOC_STS_REG0	(0x810)	/*4 bytes*/
#define SDIO_FN1R_SOC_STS_REG1	(0x814)	/*4 bytes*/
#define SDIO_FN1R_SOC_STS_REG2	(0x818)	/*4 bytes*/
#define SDIO_FN1R_SOC_STS_REG3	(0x81C)	/*4 bytes*/

#define SDIO_FN1R_SOC_STS_REG0B0	(0x810)
#define SDIO_FN1R_SOC_STS_REG0B1	(0x811)
#define SDIO_FN1R_SOC_STS_REG0B2	(0x812)
#define SDIO_FN1R_SOC_STS_REG0B3	(0x813)

#define SDIO_FN1R_SOC_STS_REG1B0	(0x814)
#define SDIO_FN1R_SOC_STS_REG1B1	(0x815)
#define SDIO_FN1R_SOC_STS_REG1B2	(0x816)
#define MPU_INT_ST_RSP_ERR_MASK		(0x80)
#define MPU_INT_ST_ILLEGAL_READ_MASK	(0x40)
#define MPU_INT_ST_ILLEGAL_WRITE_MASK	(0x20)
#define INT_SE_HOST1_IRQ_MASK		(0x10)
#define INT_WDT_MASK			(0x08)
#define INT_MAILBOX_MASK		(0x04)
#define INT_CHIP_SLEEP_MASK		(0x02)
#define INT_CPU_LOCKUP_MASK		(0x01)

#define SDIO_FN1R_SOC_STS_REG1B3	(0x817)
#define INT_SYS_MIPI_RX_FS_MASK		(0x80)
#define INT1_ISP_TO_AP_MASK		(0x40)
#define INT0_ISP_TO_AP_MASK		(0x20)
#define INT_CHIP_WAKEUP_MASK		(0x10)
#define INT_CC_RESET_DONE_MASK		(0x08)
#define INT_PVT_IRQ_MASK		(0x04)
#define INT_SDU_HOST_MASK		(0x02)
#define INT_BUS_MONITOR_MASK		(0x01)



#define SDIO_FN1R_SOC_STS_REG2B0	(0x818)
#define SDIO_FN1R_SOC_STS_REG2B1	(0x819)
#define SDIO_FN1R_SOC_STS_REG2B2	(0x81a)
#define SDIO_FN1R_SOC_STS_REG2B3	(0x81b)

#define SDIO_FN1R_SOC_STS_REG3B0	(0x81c)
#define SDIO_FN1R_SOC_STS_REG3B1	(0x81d)
#define SDIO_FN1R_SOC_STS_REG3B2	(0x81e)
#define SDIO_FN1R_SOC_STS_REG3B3	(0x81f)

/*address aligned in 4-bytes*/
#define SDIO_FN1R_ATU_MATCH_ADDR_BASE	(0x900)
#define SDIO_FN1R_ATU_MATCH_ADDR(x)	(SDIO_FN1R_ATU_MATCH_ADDR_BASE+x*4)

/*address aligned in 4-bytes*/
#define SDIO_FN1R_ATU_REMAP_ADDR_BASE	(0xa00)
#define SDIO_FN1R_ATU_REMAP_ADDR(x)	(SDIO_FN1R_ATU_REMAP_ADDR_BASE+x*4)

/*address aligned in 1-byte*/
#define SDIO_FN1R_ATU_MATCH_ADDRB(x)	(SDIO_FN1R_ATU_MATCH_ADDR_BASE+x)
#define SDIO_FN1R_ATU_REMAP_ADDRB(x)	(SDIO_FN1R_ATU_REMAP_ADDR_BASE+x)

#endif /*end of define MMC_CORE_SD_ZK_H*/

