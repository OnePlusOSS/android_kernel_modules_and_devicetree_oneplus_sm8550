// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include "gtx8_tools.h"

/****************************PART1:Log TAG****************************/
#ifdef TPD_DEVICE
#undef TPD_DEVICE
#define TPD_DEVICE "Goodix-TOOL"
#else
#define TPD_DEVICE "Goodix-TOOL"
#endif

#define CLASS_NAME "gtp_tools"
#define GOODIX_TOOLS_NAME        "gtp_tools"
#define GOODIX_TS_IOC_MAGIC      'G'
#define NEGLECT_SIZE_MASK        (~(_IOC_SIZEMASK << _IOC_SIZESHIFT))

#define GTP_IRQ_ENABLE    _IO(GOODIX_TS_IOC_MAGIC, 0)
#define GTP_DEV_RESET     _IO(GOODIX_TS_IOC_MAGIC, 1)
#define GTP_SEND_COMMAND  (_IOW(GOODIX_TS_IOC_MAGIC, 2, u8) & NEGLECT_SIZE_MASK)
#define GTP_SEND_CONFIG   (_IOW(GOODIX_TS_IOC_MAGIC, 3, u8) & NEGLECT_SIZE_MASK)
#define GTP_ASYNC_READ    (_IOR(GOODIX_TS_IOC_MAGIC, 4, u8) & NEGLECT_SIZE_MASK)
#define GTP_SYNC_READ     (_IOR(GOODIX_TS_IOC_MAGIC, 5, u8) & NEGLECT_SIZE_MASK)
#define GTP_ASYNC_WRITE   (_IOW(GOODIX_TS_IOC_MAGIC, 6, u8) & NEGLECT_SIZE_MASK)
#define GTP_READ_CONFIG   (_IOW(GOODIX_TS_IOC_MAGIC, 7, u8) & NEGLECT_SIZE_MASK)
#define GTP_ESD_ENABLE    _IO(GOODIX_TS_IOC_MAGIC, 8)
#define GTP_CLEAR_RAWDATA_FLAG    _IO(GOODIX_TS_IOC_MAGIC, 9)
#define GTP_TOOLS_CTRL_SYNC (_IOW(GOODIX_TS_IOC_MAGIC, 10, u8) & NEGLECT_SIZE_MASK)

#define GOODIX_TS_IOC_MAXNR            10
#define GOODIX_TOOLS_MAX_DATA_LEN      4096
#define I2C_MSG_HEAD_LEN               20

#define GTX8_TOOLS_CLOSE  0
#define GTX8_TOOLS_OPEN  1

#define SPI_TRANS_PREFIX_LEN    1
#define REGISTER_WIDTH          4
#define SPI_READ_DUMMY_LEN      3
#define SPI_READ_PREFIX_LEN  	(SPI_TRANS_PREFIX_LEN + REGISTER_WIDTH + SPI_READ_DUMMY_LEN)
#define SPI_WRITE_PREFIX_LEN 	(SPI_TRANS_PREFIX_LEN + REGISTER_WIDTH)
#define MAX_BUF_LENGTH			(16 * 1024)

#define SPI_WRITE_FLAG          0xF0
#define SPI_READ_FLAG           0xF1

static struct gtx8_tool_info *g_gtx8_tool_info[TP_SUPPORT_MAX] = {NULL};

static int brl_spi_read(struct spi_device *spi, unsigned int addr,
		    unsigned char *data, unsigned int len)
{
	u8 *rx_buf = NULL;
	u8 *tx_buf = NULL;
	struct spi_transfer xfers;
	struct spi_message spi_msg;
	int ret = 0;

	rx_buf = kzalloc(SPI_READ_PREFIX_LEN + len, GFP_KERNEL);
	if (!rx_buf) {
		TPD_INFO("rx_buf kzalloc error\n");
		ret = -ENOMEM;
		return ret;
	}
	tx_buf = kzalloc(SPI_READ_PREFIX_LEN + len, GFP_KERNEL);
	if (!tx_buf) {
		TPD_INFO("tx_buf kzalloc error\n");
		ret = -ENOMEM;
		goto err_tx_buf_alloc;
	}

	spi_message_init(&spi_msg);
	memset(&xfers, 0, sizeof(xfers));

	/*spi_read tx_buf format: 0xF1 + addr(4bytes) + data*/
	tx_buf[0] = SPI_READ_FLAG;
	tx_buf[1] = (addr >> 24) & 0xFF;
	tx_buf[2] = (addr >> 16) & 0xFF;
	tx_buf[3] = (addr >> 8) & 0xFF;
	tx_buf[4] = addr & 0xFF;
	tx_buf[5] = 0xFF;
	tx_buf[6] = 0xFF;
	tx_buf[7] = 0xFF;

	xfers.tx_buf = tx_buf;
	xfers.rx_buf = rx_buf;
	xfers.len = SPI_READ_PREFIX_LEN + len;
	xfers.cs_change = 0;
	spi_message_add_tail(&xfers, &spi_msg);
	ret = spi_sync(spi, &spi_msg);
	if (ret < 0) {
		TPD_INFO("spi transfer error:%d\n", ret);
		goto exit;
	}
	memcpy(data, &rx_buf[SPI_READ_PREFIX_LEN], len);

exit:
	kfree(tx_buf);
err_tx_buf_alloc:
	kfree(rx_buf);

	return ret;
}

static int brl_spi_write(struct spi_device *spi, unsigned int addr,
		     unsigned char *data, unsigned int len)
{
	u8 *tx_buf = NULL;
	struct spi_transfer xfers;
	struct spi_message spi_msg;
	int ret = 0;

	tx_buf = kzalloc(SPI_WRITE_PREFIX_LEN + len, GFP_KERNEL);
	if (!tx_buf) {
		TPD_INFO("alloc tx_buf failed, size:%d\n",
			 SPI_WRITE_PREFIX_LEN + len);
		return -ENOMEM;
	}

	spi_message_init(&spi_msg);
	memset(&xfers, 0, sizeof(xfers));

	tx_buf[0] = SPI_WRITE_FLAG;
	tx_buf[1] = (addr >> 24) & 0xFF;
	tx_buf[2] = (addr >> 16) & 0xFF;
	tx_buf[3] = (addr >> 8) & 0xFF;
	tx_buf[4] = addr & 0xFF;
	memcpy(&tx_buf[SPI_WRITE_PREFIX_LEN], data, len);
	xfers.tx_buf = tx_buf;
	xfers.len = SPI_WRITE_PREFIX_LEN + len;
	xfers.cs_change = 0;
	spi_message_add_tail(&xfers, &spi_msg);
	ret = spi_sync(spi, &spi_msg);
	if (ret < 0) {
		TPD_INFO("spi transfer error:%d\n", ret);
	}

	kfree(tx_buf);
	return ret;
}

static int brl_async_read(struct gtx8_tool_info *p_goodix_tool_info, void __user *arg)
{
	u8 *databuf = NULL;
	int ret = 0;
	u32 reg_addr, length;
	u8 i2c_msg_head[I2C_MSG_HEAD_LEN];
	struct spi_device *spi = p_goodix_tool_info->spi;

	ret = copy_from_user(&i2c_msg_head, arg, I2C_MSG_HEAD_LEN);
	if (ret)
		return -EFAULT;

	reg_addr = i2c_msg_head[0] + (i2c_msg_head[1] << 8)
			+ (i2c_msg_head[2] << 16) + (i2c_msg_head[3] << 24);
	length = i2c_msg_head[4] + (i2c_msg_head[5] << 8)
			+ (i2c_msg_head[6] << 16) + (i2c_msg_head[7] << 24);
	if (length > MAX_BUF_LENGTH) {
		TPD_INFO("buffer too long:%d > %d\n", length, MAX_BUF_LENGTH);
		return -EINVAL;
	}
	databuf = kzalloc(length, GFP_KERNEL);
	if (!databuf) {
		return -ENOMEM;
	}

	if (brl_spi_read(spi, reg_addr, databuf, length)) {
		ret = -EBUSY;
		goto err_out;
	}
	ret = copy_to_user((u8 *)arg + I2C_MSG_HEAD_LEN, databuf, length);
	if (ret) {
		ret = -EFAULT;
		goto err_out;
	}
	ret = length;
err_out:
	kfree(databuf);
	return ret;
}

static int brl_async_write(struct gtx8_tool_info *p_goodix_tool_info, void __user *arg)
{
	u8 *databuf;
	int ret = 0;
	u32 reg_addr, length;
	u8 i2c_msg_head[I2C_MSG_HEAD_LEN];
	struct spi_device *spi = p_goodix_tool_info->spi;

	ret = copy_from_user(&i2c_msg_head, arg, I2C_MSG_HEAD_LEN);
	if (ret) {
		return -EFAULT;
	}
	reg_addr = i2c_msg_head[0] + (i2c_msg_head[1] << 8)
			+ (i2c_msg_head[2] << 16) + (i2c_msg_head[3] << 24);
	length = i2c_msg_head[4] + (i2c_msg_head[5] << 8)
			+ (i2c_msg_head[6] << 16) + (i2c_msg_head[7] << 24);
	if (length > MAX_BUF_LENGTH) {
		TPD_INFO("buffer too long:%d > %d\n", length, MAX_BUF_LENGTH);
		return -EINVAL;
	}

	databuf = kzalloc(length, GFP_KERNEL);
	if (!databuf) {
		return -ENOMEM;
	}
	ret = copy_from_user(databuf, (u8 *)arg + I2C_MSG_HEAD_LEN, length);
	if (ret) {
		ret = -EFAULT;
		goto err_out;
	}

	if (brl_spi_write(spi, reg_addr, databuf, length)) {
		ret = -EBUSY;
	} else {
		ret = length;
	}

err_out:
	kfree(databuf);
	return ret;
}

/* read data from i2c asynchronous,
** success return bytes read, else return <= 0
*/
static int __maybe_unused gtx8_async_read(void __user *arg,
			   struct gtx8_tool_info *p_goodix_tool_info)
{
	u8 *databuf = NULL;
	int ret = 0;
	u32 reg_addr = 0;
	u32 length = 0;
	u8 i2c_msg_head[I2C_MSG_HEAD_LEN];

	ret = copy_from_user(&i2c_msg_head, arg, I2C_MSG_HEAD_LEN);

	if (ret) {
		ret = -EFAULT;
		return ret;
	}

	reg_addr = i2c_msg_head[0] + (i2c_msg_head[1] << 8)
		   + i2c_msg_head[2] + (i2c_msg_head[3] << 8);
	length = i2c_msg_head[4] + (i2c_msg_head[5] << 8)
		 + (i2c_msg_head[6] << 16) + (i2c_msg_head[7] << 24);

	if (length > GOODIX_TOOLS_MAX_DATA_LEN) {
		TPD_INFO("%s: Invalied data length:%d\n", __func__, length);
		return -EFAULT;
	}

	databuf = kzalloc(length, GFP_KERNEL);

	if (!databuf) {
		TPD_INFO("Alloc memory failed\n");
		return -ENOMEM;
	}

	if (touch_i2c_read_block(p_goodix_tool_info->client, (u16)reg_addr, length,
				 databuf) >= 0) {
		if (copy_to_user((u8 *)arg + I2C_MSG_HEAD_LEN, databuf, length)) {
			ret = -EFAULT;
			TPD_INFO("Copy_to_user failed\n");

		} else {
			ret = length;
		}

	} else {
		ret = -EBUSY;
		TPD_INFO("Read i2c failed\n");
	}

	if (databuf) {
		kfree(databuf);
		databuf = NULL;
	}

	return ret;
}

/* write data to i2c asynchronous,
** success return bytes write, else return <= 0
*/
static int __maybe_unused gtx8_async_write(void __user *arg,
			    struct gtx8_tool_info *p_goodix_tool_info)
{
	u8 *databuf = NULL;
	int ret = 0;
	u32 reg_addr = 0;
	u32 length = 0;
	u8 i2c_msg_head[I2C_MSG_HEAD_LEN];

	ret = copy_from_user(&i2c_msg_head, arg, I2C_MSG_HEAD_LEN);

	if (ret) {
		TPD_INFO("Copy data from user failed\n");
		return -EFAULT;
	}

	reg_addr = i2c_msg_head[0] + (i2c_msg_head[1] << 8)
		   + i2c_msg_head[2] + (i2c_msg_head[3] << 8);
	length = i2c_msg_head[4] + (i2c_msg_head[5] << 8)
		 + (i2c_msg_head[6] << 16) + (i2c_msg_head[7] << 24);

	if (length > GOODIX_TOOLS_MAX_DATA_LEN) {
		TPD_INFO("%s: Invalied data length:%d\n", __func__, length);
		return -EFAULT;
	}

	databuf = kzalloc(length, GFP_KERNEL);

	if (!databuf) {
		TPD_INFO("Alloc memory failed\n");
		return -ENOMEM;
	}

	ret = copy_from_user(databuf, (u8 *)arg + I2C_MSG_HEAD_LEN, length);

	if (ret) {
		ret = -EFAULT;
		TPD_INFO("Copy data from user failed\n");
		goto err_out;
	}

	if (touch_i2c_write_block(p_goodix_tool_info->client, (u16)reg_addr, length,
				  databuf) < 0) {
		ret = -EBUSY;
		TPD_INFO("Write data to device failed\n");

	} else {
		ret = length;
	}

err_out:

	if (databuf) {
		kfree(databuf);
		databuf = NULL;
	}

	return ret;
}

static int gtx8_tools_open(struct inode *inode, struct file *filp)
{
	struct gtx8_tool_info *p_goodix_tool_info =
		container_of(inode->i_cdev, struct gtx8_tool_info, main_dev);

	TPD_INFO("tools open\n");

	if (!p_goodix_tool_info) {
		TPD_INFO("p_goodix_tool_info is null\n");
		return -1;
	}

	filp->private_data = p_goodix_tool_info;

	if (p_goodix_tool_info->devicecount > 0) {
		return -ERESTARTSYS;
		TPD_INFO("tools open failed!");
	}

	p_goodix_tool_info->devicecount++;

	return 0;
}

static int gtx8_tools_release(struct inode *inode, struct file *filp)
{
	struct gtx8_tool_info *p_goodix_tool_info =
		container_of(inode->i_cdev, struct gtx8_tool_info, main_dev);

	TPD_INFO("tools release\n");
	p_goodix_tool_info->devicecount--;

	return 0;
}

/**
 * gtx8_tools_ioctl - ioctl implementation
 *
 * @filp: Pointer to file opened
 * @cmd: Ioctl opertion command
 * @arg: Command data
 * Returns >=0 - succeed, else failed
 */
static long gtx8_tools_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	int ret = 0;
	/* struct i2c_client *client = NULL; */
	struct gtx8_tool_info *p_goodix_tool_info = filp->private_data;

	if (!p_goodix_tool_info) {
		TPD_INFO("p_goodix_tool_info is null\n");
		return -1;
	}

	/* client = p_goodix_tool_info->client; */

	if (_IOC_TYPE(cmd) != GOODIX_TS_IOC_MAGIC) {
		TPD_INFO("Bad magic num:%c\n", _IOC_TYPE(cmd));
		return -ENOTTY;
	}

	if (_IOC_NR(cmd) > GOODIX_TS_IOC_MAXNR) {
		TPD_INFO("Bad cmd num:%d > %d",
			 _IOC_NR(cmd), GOODIX_TS_IOC_MAXNR);
		return -ENOTTY;
	}

	switch (cmd & NEGLECT_SIZE_MASK) {
	case GTP_IRQ_ENABLE:
		if (arg) {
			enable_irq(p_goodix_tool_info->irq);
		} else {
			disable_irq_nosync(p_goodix_tool_info->irq);
		}
		TPD_INFO("set irq mode: %s\n", arg ? "enable" : "disable");
		ret = 0;
		break;
	case GTP_ESD_ENABLE:
		ret = 0;
		if (!p_goodix_tool_info->esd_handle_support) {
			TPD_DEBUG("Unsupport esd operation\n");
		} else {
			esd_handle_switch(p_goodix_tool_info->esd_info, !!arg);
			TPD_DEBUG("set esd mode: %s\n", arg ? "enable" : "disable");
		}
		break;
	case GTP_DEV_RESET:
		p_goodix_tool_info->reset(p_goodix_tool_info->chip_data);
		break;
	case GTP_ASYNC_READ:
		/* ret = gtx8_async_read((void __user *)arg,  p_goodix_tool_info); */
		ret = brl_async_read(p_goodix_tool_info, (void __user *)arg);
		if (ret < 0) {
			TPD_INFO("Async data read failed\n");
		}
		break;
	case GTP_SYNC_READ:
		/* *p_goodix_tool_info->p_gt8x_rawdiff_mode = 1; */
		TPD_INFO("not support sync read\n");
		break;
	case GTP_ASYNC_WRITE:
		/* ret = gtx8_async_write((void __user *)arg, p_goodix_tool_info); */
		ret = brl_async_write(p_goodix_tool_info, (void __user *)arg);
		if (ret < 0) {
			TPD_INFO("Async data write failed\n");
		}
		break;
	case GTP_CLEAR_RAWDATA_FLAG:
		*p_goodix_tool_info->p_gt8x_rawdiff_mode = 0;
		ret = 0;
		break;
	case GTP_TOOLS_CTRL_SYNC:
		*p_goodix_tool_info->p_gt8x_rawdiff_mode = !!arg;
		TPD_INFO("set tools ctrl sync %d\n", *p_goodix_tool_info->p_gt8x_rawdiff_mode);
		break;
	default:
		TPD_INFO("Invalid cmd\n");
		ret = -ENOTTY;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long gtx8_tools_compat_ioctl(struct file *file, unsigned int cmd,
				    unsigned long arg)
{
	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl) {
		return -1;
	}

	return file->f_op->unlocked_ioctl(file, cmd, (unsigned long)arg32);
}
#endif

static const struct file_operations gtx8_tools_fops = {
	.owner          = THIS_MODULE,
	.open           = gtx8_tools_open,
	.release        = gtx8_tools_release,
	.unlocked_ioctl = gtx8_tools_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = gtx8_tools_compat_ioctl,
#endif
};

#define BUF_MAX 25
int gtx8_init_tool_node(struct touchpanel_data *ts,  int *p)
{
	int ret = NO_ERR;
	char buf[BUF_MAX] = {0};
	struct gtx8_tool_info *p_gtx8_tool_info = NULL;

	TP_INFO(ts->tp_index, "Goodix tools miscdev init\n");
	/* 1. Alloc p_gtx8_tool_info */
	p_gtx8_tool_info = kzalloc(sizeof(struct gtx8_tool_info), GFP_KERNEL);

	if (p_gtx8_tool_info == NULL) {
		TP_INFO(ts->tp_index, "p_gtx8_tool_info kzalloc error\n");
		ret = -ENOMEM;
		return ret;
	}

	snprintf(buf, BUF_MAX, "%s%d", GOODIX_TOOLS_NAME, ts->tp_index);
	TP_INFO(ts->tp_index, "now set goodix tool name:%s\n", buf);

	/* 2. Alloc cdev */
	ret = alloc_chrdev_region(&p_gtx8_tool_info->dev_no, 0, ts->tp_index + 1, buf);

	if (ret  < 0) {
		TP_INFO(ts->tp_index, "alloc_chrdev_region failed ret = %d\n", ret);
		goto error_get_dev_num;
	}

	p_gtx8_tool_info->rmidev_major_num = MAJOR(p_gtx8_tool_info->dev_no);
	/* 3. Alloc class */
	p_gtx8_tool_info->class = class_create(THIS_MODULE, buf);

	if (IS_ERR(p_gtx8_tool_info->class)) {
		ret = PTR_ERR(p_gtx8_tool_info->class);
		TP_INFO(ts->tp_index, "couldn't create class rc = %d\n", ret);
		goto error_class_create;
	}

	/* 4. Alloc device */
	p_gtx8_tool_info->device = device_create(p_gtx8_tool_info->class, NULL,
			   p_gtx8_tool_info->dev_no, NULL, buf);


	if (IS_ERR(p_gtx8_tool_info->device)) {
		ret = PTR_ERR(p_gtx8_tool_info->device);
		TP_INFO(ts->tp_index, "device_create failed %d\n", ret);
		goto error_class_device_create;
	}

	cdev_init(&p_gtx8_tool_info->main_dev, &gtx8_tools_fops);
	ret = cdev_add(&p_gtx8_tool_info->main_dev,
		       MKDEV(MAJOR(p_gtx8_tool_info->dev_no), 0), 1);

	if (ret < 0) {
		TP_INFO(ts->tp_index, "cdev_add error,ret:%d\n", ret);
		goto error_cdev_add;
	}

	/* 5. init p_gtx8_tool_info */
	p_gtx8_tool_info->devicecount = 0;
	p_gtx8_tool_info->is_suspended = &ts->is_suspended;
	p_gtx8_tool_info->esd_handle_support = ts->esd_handle_support;
	p_gtx8_tool_info->esd_info = &ts->esd_info;
	p_gtx8_tool_info->client = ts->client;
	p_gtx8_tool_info->spi = ts->s_client;
	p_gtx8_tool_info->chip_data = ts->chip_data;
	p_gtx8_tool_info->irq = ts->irq;
	p_gtx8_tool_info->hw_res = &(ts->hw_res);
	p_gtx8_tool_info->reset = ts->ts_ops->reset;
	p_gtx8_tool_info->tp_index = ts->tp_index;
	p_gtx8_tool_info->p_gt8x_rawdiff_mode = (int *)p;
	/* 6. add for more goodix ic */
	g_gtx8_tool_info[ts->tp_index] = p_gtx8_tool_info;

	TP_INFO(ts->tp_index, "%s: gtp tools add dev pass\n", __func__);

	return 0;

error_cdev_add:
	device_destroy(p_gtx8_tool_info->class, p_gtx8_tool_info->dev_no);
error_class_device_create:
	class_destroy(p_gtx8_tool_info->class);
error_class_create:
	unregister_chrdev_region(p_gtx8_tool_info->dev_no, 1);
error_get_dev_num:
	kfree(p_gtx8_tool_info);
	return ret;
}
EXPORT_SYMBOL(gtx8_init_tool_node);

void gtx8_deinit_tool_node(struct touchpanel_data *ts)
{
	if (ts) {
		if (g_gtx8_tool_info[ts->tp_index]) {
			unregister_chrdev_region(g_gtx8_tool_info[ts->tp_index]->dev_no, 1);
		}
		TP_INFO(ts->tp_index, "Goodix tools miscdev exit\n");
		kfree(g_gtx8_tool_info[ts->tp_index]);
	}
}
EXPORT_SYMBOL(gtx8_deinit_tool_node);
