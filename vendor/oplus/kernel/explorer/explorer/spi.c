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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include "include/main.h"
#include "include/spi.h"

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static ssize_t explorer_cspi_sync(struct cspi_data *cspi, struct spi_message *message)
{
	int status;
	struct spi_device *spi;

	spin_lock_irq(&cspi->spi_lock);
	spi = cspi->spi;
	spin_unlock_irq(&cspi->spi_lock);

	if (spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_sync(spi, message);

	if (status == 0)
		status = message->actual_length;

	return status;
}

ssize_t explorer_cspi_sync_write(struct cspi_data *cspi, u32 cc_addr, u32 *ap_buffer, u32 len)
{
	u8 *p_buffer;
	u32 i = 0;
	u32 *p_u32 = NULL;

	struct spi_transfer	t = {
			.tx_buf		= NULL,
			.tx_nbits	= SPI_NBITS_SINGLE,
			.len		= CSPI_WORD_LENGTH*len+8,
			.bits_per_word = 8,
			.speed_hz	= 0,
		};
	struct spi_message	m;

	if (cspi == NULL) {
		pr_err("%s, null pointer.\n", __func__);
		return -1;
	}
	t.tx_buf = cspi->tx_buffer;
	t.speed_hz = cspi->speed_hz;

	if ((CSPI_WORD_LENGTH*len+CSPI_PACKET_HEADER_LEN)>SPI_XFER_BUF_LEN) {
		pr_err("%s, data length is invalid.\n", __func__);
		return -EINVAL;
	}

	pr_info("%s, do cspi write comm.\n", __func__);

	p_buffer = (u8 *)cspi->tx_buffer;

	/* write external with 1 symbol */
	p_buffer[0] = CSPI_WRITE_EXT;
	p_buffer[1] = 0x0;
	p_buffer[2] = (len&0xFF00)>>8;		/* data length */
	p_buffer[3] = len&0x00FF;			/* data length */

	/* add addr */
	p_u32 = (u32 *)&p_buffer[4];
	*p_u32++ = __builtin_bswap32(cc_addr);

	/* add data */
	for (i=0; i<len; i++)
		*p_u32++ = __builtin_bswap32(*ap_buffer++);

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return explorer_cspi_sync(cspi, &m);
}

static ssize_t explorer_cspi_sync_read_internal(struct cspi_data *cspi, u32 cc_addr, u32 *ap_buffer, u32 len)
{
	u8 *p_buffer;
	u32 *p_u32 = NULL;

	struct spi_transfer	read_cmd_xfer = {
			.tx_buf		= NULL,
			.tx_nbits	= SPI_NBITS_SINGLE,
			.len		= 8,
			.bits_per_word = 8,
			.speed_hz	= 0,
		};
	struct spi_message	read_msg;

	struct spi_transfer	read_data_xfer = {
			.rx_buf		= ap_buffer,
			.rx_nbits	= SPI_NBITS_SINGLE,
			.len		= CSPI_WORD_LENGTH*len,
			.bits_per_word = 8,
			.speed_hz	= 0,
		};

	if (cspi == NULL) {
		pr_err("%s, null pointer.\n", __func__);
		return -1;
	}
	read_cmd_xfer.tx_buf = cspi->tx_buffer;
	read_cmd_xfer.speed_hz = cspi->speed_hz;
	read_data_xfer.speed_hz = cspi->speed_hz;

	if ((CSPI_WORD_LENGTH*len+CSPI_PACKET_HEADER_LEN)>SPI_XFER_BUF_LEN) {
		pr_err("%s, date length is invalid.\n", __func__);
		return -EINVAL;
	}

	p_buffer = (u8 *)cspi->tx_buffer;

	/* send read internal message */
	p_buffer[0] = CSPI_READ_INT;
	p_buffer[1] = 0x0;
	p_buffer[2] = (len&0xFF00)>>8;		/* data length */
	p_buffer[3] = len&0x00FF;			/* data length */
	/* add addr */
	p_u32 = (u32 *)&p_buffer[4];
	*p_u32++ = __builtin_bswap32(cc_addr);
	spi_message_init(&read_msg);
	spi_message_add_tail(&read_cmd_xfer, &read_msg);
	explorer_cspi_sync(cspi, &read_msg);

	/* read internal data */
	spi_message_add_tail(&read_data_xfer, &read_msg);
	return explorer_cspi_sync(cspi, &read_msg);
}

ssize_t explorer_cspi_sync_read(struct cspi_data *cspi, u32 cc_addr, u32 *ap_buffer, u32 len)
{
	int ret = 0;
	u8 *p_buffer;
	u32 i = 0, *p_u32 = NULL;
	u32 slv_tx_fifo_status = 0x000000A1, poll_retry_times = 4;
	u32 *ap_rx_buffer = ap_buffer;

	struct spi_transfer	t = {
			.tx_buf		= NULL,
			.tx_nbits	= SPI_NBITS_SINGLE,
			.len		= CSPI_WORD_LENGTH*len+8,
			.bits_per_word = 8,
			.speed_hz	= 0,
		};
	struct spi_message	m;

	if (cspi == NULL) {
		pr_err("%s, null pointer.\n", __func__);
		return -1;
	}
	t.tx_buf = cspi->tx_buffer;
	t.speed_hz = cspi->speed_hz;

	if ((CSPI_WORD_LENGTH*len+CSPI_PACKET_HEADER_LEN)>SPI_XFER_BUF_LEN) {
		pr_err("%s, date length is invalid.\n", __func__);
		return -EINVAL;
	}

	p_buffer = (u8 *)cspi->tx_buffer;

	/* write external with 1 symbol */
	p_buffer[0] = CSPI_READ_EXT;
	p_buffer[1] = 0x0;
	p_buffer[2] = (len&0xFF00)>>8;		/* data length */
	p_buffer[3] = len&0x00FF;			/* data length */

	/* add addr */
	p_u32 = (u32 *)&p_buffer[4];
	*p_u32++ = __builtin_bswap32(cc_addr);

	/* add data */
	for (i=0; i<len; i++)
		*p_u32++ = __builtin_bswap32(*ap_buffer++);

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	explorer_cspi_sync(cspi, &m);

	/* polling cspi internal tx fifo status regisger */
	while ((slv_tx_fifo_status & 0x00000001) && (poll_retry_times > 0)) {
		msleep(1);
		ret = explorer_cspi_sync_read_internal(cspi, CSPI_FIFO_STATUS_REG, &slv_tx_fifo_status, 1);
		poll_retry_times--;
		/* TODO:watch out!!! endianess of read value */
		pr_info("%s, slave tx fifo status: 0x%x.\n", __func__, slv_tx_fifo_status);
	}
	pr_info("%s, slave tx fifo status: 0x%x.\n", __func__, slv_tx_fifo_status);

	/* read cspi internal fifo */
	ret = explorer_cspi_sync_read_internal(cspi, CSPI_TX_FIFO_REG, ap_rx_buffer, 1);

	/* adjust read byte order */
	*ap_rx_buffer = __builtin_bswap32(*ap_rx_buffer);

	//TODO:watch out!!! endianess of read value
	pr_info("%s, read external value: 0x%x, at cc addr 0x%x.\n", __func__, *ap_rx_buffer, cc_addr);

	return ret;
}

/*-------------------------------------------------------------------------*/
static const struct of_device_id cspi_dt_ids[] = {
	{ .compatible = "explorer_spi" },
	{},
};
MODULE_DEVICE_TABLE(of, cspi_dt_ids);

/*-------------------------------------------------------------------------*/
static int explorer_cspi_probe(struct spi_device *spi)
{
	struct cspi_data *cspi;
	int	status = 0, retval = 0;
	u32 tmp = 0;
	struct device *explorer_device = NULL;
	struct explorer_plat_data *epd = NULL;

	pr_info("%s, begin.\n", __func__);

	/* Allocate driver data */
	cspi = kzalloc(sizeof(*cspi), GFP_KERNEL);
	if (!cspi)
		return -ENOMEM;

	/* Initialize the driver data */
	cspi->spi = spi;
	spin_lock_init(&cspi->spi_lock);
	mutex_init(&cspi->buf_lock);

	INIT_LIST_HEAD(&cspi->device_entry);

	spi->max_speed_hz = 2000000;
	cspi->speed_hz = spi->max_speed_hz;

	/* setup spi mode */
	tmp = (spi->mode & (~SPI_MODE_MASK));
	spi->mode = tmp | SPI_CPHA;
	pr_info("%s, before spi_setup, spi->mode = 0x%x.\n", __func__, spi->mode);
	retval = spi_setup(spi);
	if (retval < 0) {
		pr_err("%s, spi_setup failed, retval = %d.\n", __func__, retval);
		goto err_setup_spi;
	}
    else
		pr_info("%s, after spi_setup, spi->mode = 0x%x.\n", __func__, spi->mode);

	/* allocate xfer memory */
    if (!cspi->tx_buffer) {
            cspi->tx_buffer = kmalloc(SPI_XFER_BUF_LEN, GFP_KERNEL);
            if (!cspi->tx_buffer) {
                    dev_dbg(&cspi->spi->dev, "%s, out of memory.\n", __func__);
                    status = -ENOMEM;
                    goto err_setup_spi;
            }
    }
	memset(cspi->tx_buffer, 0x00, SPI_XFER_BUF_LEN);
    if (!cspi->rx_buffer) {
            cspi->rx_buffer = kmalloc(SPI_XFER_BUF_LEN, GFP_KERNEL);
            if (!cspi->rx_buffer) {
                    dev_dbg(&cspi->spi->dev, "%s, out of memory.\n", __func__);
                    status = -ENOMEM;
                    goto err_rx_buffer_alloc;
            }
    }
	memset(cspi->rx_buffer, 0x00, SPI_XFER_BUF_LEN);

	if (status == 0)
		spi_set_drvdata(spi, cspi);

	/* find explorer platform device and set its drvdata */
	explorer_device = bus_find_device_by_name(&platform_bus_type, NULL, "soc:zeku,explorer@0");
	if (!explorer_device) {
		pr_err("%s, explorer platform device not found.\n", __func__);
		goto err_find_plt_err;
	}
	epd = dev_get_drvdata(explorer_device);
	epd->cspi = cspi;

	pr_info("%s, done. status = %d.\n", __func__, status);
	return status;

err_find_plt_err:
	kfree(cspi->rx_buffer);
err_rx_buffer_alloc:
	kfree(cspi->tx_buffer);
err_setup_spi:
	kfree(cspi);
	pr_info("%s, failed. status = %d.\n", __func__, status);
	return status;
}

static int explorer_cspi_remove(struct spi_device *spi)
{
	struct cspi_data	*cspi = spi_get_drvdata(spi);

	/* prevent new opens */
	mutex_lock(&device_list_lock);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&cspi->spi_lock);
	cspi->spi = NULL;
	spin_unlock_irq(&cspi->spi_lock);

	if (cspi->users == 0)
		kfree(cspi);
	mutex_unlock(&device_list_lock);

	pr_info("%s, done.\n", __func__);

	return 0;
}

static struct spi_driver cspi_spi_driver = {
	.driver = {
		.name =		"explorer_spi",
		.of_match_table = of_match_ptr(cspi_dt_ids),
	},
	.probe =	explorer_cspi_probe,
	.remove =	explorer_cspi_remove,

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

/*-------------------------------------------------------------------------*/
int explorer_cspi_init(void)
{
	int status;

	status = spi_register_driver(&cspi_spi_driver);

	pr_info("%s, done.\n", __func__);

	return status;
}

void explorer_cspi_exit(void)
{
	spi_unregister_driver(&cspi_spi_driver);

	pr_info("%s, done.\n", __func__);
}

