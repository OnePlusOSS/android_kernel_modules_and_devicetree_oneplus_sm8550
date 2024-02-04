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
 * Basecode Created :        2020/09/23 Author: zf@zeku.com
 *
 */

#ifndef _EXPLORER_SPI_H
#define _EXPLORER_SPI_H

#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/spi/spi.h>

/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK	(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY | SPI_TX_DUAL \
				| SPI_TX_QUAD | SPI_RX_DUAL | SPI_RX_QUAD)

struct cspi_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	/* TX/RX buffers are NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned		users;
	u8			*tx_buffer;
	u8			*rx_buffer;
	u32			speed_hz;
};

#define CSPI_FIFO_STATUS_REG			(0x00000030)
#define CSPI_TX_FIFO_REG			(0x00000804)

#define	CSPI_READ_EXT			(0x00)
#define	CSPI_READ_INT			(0x40)
#define	CSPI_WRITE_EXT			(0x80)
#define	CSPI_WRITE_INT			(0xC0)
#define CSPI_WORD_LENGTH			(0x4)
#define CSPI_PACKET_HEADER_LEN			(0x8)
#define CSPI_PAYLOAD_MAX_LEN			(0x10000) /* 64KB */

#define	SPI_XFER_BUF_LEN			(CSPI_PACKET_HEADER_LEN+CSPI_PAYLOAD_MAX_LEN)

#define	CSPI_ERROR_DATA			(0xdeadbeaf)

int explorer_cspi_init(void);
void explorer_cspi_exit(void);
ssize_t explorer_cspi_sync_write(struct cspi_data *cspi, u32 cc_addr, u32 *ap_addr, u32 len);
ssize_t explorer_cspi_sync_read(struct cspi_data *cspi, u32 cc_addr, u32 *ap_addr, u32 len);

#endif /* _EXPLORER_SPI_H */

