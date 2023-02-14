/* SPDX-License-Identifier: GPL-2.0-only  */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "<ssc_interactive>" fmt

#include <linux/init.h>
#include <linux/module.h>

#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/atomic.h>
#include "oplus_ssc_interact.h"
#include <linux/of.h>


#define FIFO_SIZE 32

extern int register_lcdinfo_notifier(struct notifier_block *nb);
extern int unregister_lcdinfo_notifier(struct notifier_block *nb);

static struct ssc_interactive *g_ssc_cxt = NULL;

static void ssc_interactive_set_fifo(uint8_t type, uint16_t data)
{
	struct fifo_frame fifo_fm;
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;
	int ret = 0;
	/* pr_info("type= %u, data=%d\n", type, data); */
	memset(&fifo_fm, 0, sizeof(struct fifo_frame));
	fifo_fm.type = type;
	fifo_fm.data = data;
	ret = kfifo_in_spinlocked(&ssc_cxt->fifo, &fifo_fm, 1, &ssc_cxt->fifo_lock);
	if(ret != 1) {
		pr_err("kfifo is full\n");
	}
	wake_up_interruptible(&ssc_cxt->wq);
}

static void ssc_interactive_dc_mode_to_hal(uint16_t dc_mode)
{
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;
	spin_lock(&ssc_cxt->rw_lock);
	if (dc_mode == ssc_cxt->a_info.dc_mode) {
		spin_unlock(&ssc_cxt->rw_lock);
		return;
	}
	/* pr_info("start dc_mode=%d\n", dc_mode);*/
	ssc_cxt->a_info.dc_mode = dc_mode;
	spin_unlock(&ssc_cxt->rw_lock);

	ssc_interactive_set_fifo(LCM_DC_MODE_TYPE, dc_mode);
}

static void ssc_interactive_brightness_to_hal(uint16_t brigtness)
{
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;

	spin_lock(&ssc_cxt->rw_lock);

	/* pr_info("new brigtness=%d, brightness=%d\n", brigtness, ssc_cxt->a_info.brightness); */
	if (brigtness == ssc_cxt->a_info.brightness) {
		spin_unlock(&ssc_cxt->rw_lock);
		return;
	}
	ssc_cxt->a_info.brightness = brigtness;
	spin_unlock(&ssc_cxt->rw_lock);

	ssc_interactive_set_fifo(LCM_BRIGHTNESS_TYPE, brigtness);
}

static ssize_t ssc_interactive_write(struct file *file, const char __user * buf,
                size_t count, loff_t * ppos)
{
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;

	if (count > *ppos) {
		count -= *ppos;
	} else
		count = 0;

	*ppos += count;
	wake_up_interruptible(&ssc_cxt->wq);
	return count;
}

static unsigned int ssc_interactive_poll(struct file *file, struct poll_table_struct *pt)
{
	unsigned int ptr = 0;
	int count = 0;
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;

	poll_wait(file, &ssc_cxt->wq, pt);
	spin_lock(&ssc_cxt->fifo_lock);
	count = kfifo_len(&ssc_cxt->fifo);
	spin_unlock(&ssc_cxt->fifo_lock);
	if (count > 0) {
		ptr |= POLLIN | POLLRDNORM;
	}
	return ptr;
}

static ssize_t ssc_interactive_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	size_t read = 0;
	int fifo_count = 0;
	int ret;
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;

	if (count !=0 && count < sizeof(struct fifo_frame)) {
		pr_err("err count %lu\n", count);
		return -EINVAL;
	}
	while ((read + sizeof(struct fifo_frame)) <= count) {
		struct fifo_frame fifo_fm;
		spin_lock(&ssc_cxt->fifo_lock);
		fifo_count = kfifo_len(&ssc_cxt->fifo);
		spin_unlock(&ssc_cxt->fifo_lock);

		if (fifo_count <= 0) {
			break;
		}
		ret = kfifo_out(&ssc_cxt->fifo, &fifo_fm, 1);
		if (copy_to_user(buf+read, &fifo_fm, sizeof(struct fifo_frame))) {
			pr_err("copy_to_user failed \n");
			return -EFAULT;
		}
		read += sizeof(struct fifo_frame);
	}
	*ppos += read;
	return read;
}

static int ssc_interactive_release (struct inode *inode, struct file *file)
{
	pr_info("%s\n", __func__);
	return 0;
}

static const struct file_operations under_mdevice_fops = {
	.owner   = THIS_MODULE,
	.read    = ssc_interactive_read,
	.write   = ssc_interactive_write,
	.poll    = ssc_interactive_poll,
	.llseek  = generic_file_llseek,
	.release = ssc_interactive_release,
};

static int lcdinfo_callback(struct notifier_block *nb, unsigned long event,
        void *data)
{
	int val = 0;
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;

	if (!data) {
		pr_err("data is NULL\n");
		return 0;
	}
	val = *(int*)data;

	switch(event) {
	case LCM_DC_MODE_TYPE:
		if (ssc_cxt->report_brightness) {
			ssc_interactive_dc_mode_to_hal(val);
		}
		/* ssc_interactive_dc_mode_to_scp(val); */
		break;
	case LCM_BRIGHTNESS_TYPE:
		if (ssc_cxt->report_brightness) {
			ssc_interactive_brightness_to_hal(val);
		}
		/* ssc_interactive_brightness_to_scp(val); */
		break;
	default:
		break;
	}

	return 0;
}

static int ssc_interactive_parse_dts()
{
	int ret = 0;
	int report_brightness = 0;
	struct device_node *node = NULL;
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;

	node = of_find_node_by_name(NULL, "ssc_interactive");
	if (!node) {
		pr_err("find ssc_interactive fail\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32(node, "report_brightness", &report_brightness);
	if (ret != 0) {
		pr_err("read report_brightness fail\n");
		return -ENOMEM;
	}

	if (report_brightness == 1) {
		ssc_cxt->report_brightness = true;
		pr_info("support brightness report\n");
	}
	return 0;
}

static int __init ssc_interactive_init(void)
{
	int err = 0;
	struct ssc_interactive *ssc_cxt = kzalloc(sizeof(*ssc_cxt), GFP_KERNEL);

	if (ssc_cxt == NULL) {
		pr_err("kzalloc ssc_cxt failed\n");
		err = -ENOMEM;
		goto alloc_ssc_cxt_failed;
	}

	g_ssc_cxt = ssc_cxt;

	err = ssc_interactive_parse_dts();
	if (err != 0) {
		pr_err("parse dts fail\n");
		goto parse_dts_failed;
	}

	err = kfifo_alloc(&ssc_cxt->fifo, FIFO_SIZE, GFP_KERNEL);
	if (err) {
		pr_err("kzalloc kfifo failed\n");
		err = -ENOMEM;
		goto alloc_fifo_failed;
	}

	ssc_cxt->a_info.brightness = 10000;
	ssc_cxt->a_info.dc_mode = 0;

	spin_lock_init(&ssc_cxt->fifo_lock);
	spin_lock_init(&ssc_cxt->rw_lock);

	init_waitqueue_head(&ssc_cxt->wq);

	memset(&ssc_cxt->mdev, 0 , sizeof(struct miscdevice));
	ssc_cxt->mdev.minor = MISC_DYNAMIC_MINOR;
	ssc_cxt->mdev.name = "ssc_interactive";
	ssc_cxt->mdev.fops = &under_mdevice_fops;
	if (misc_register(&ssc_cxt->mdev) != 0) {
		pr_err("misc_register  mdev failed\n");
		err = -ENODEV;
		goto register_mdevice_failed;
	}

	ssc_cxt->nb.notifier_call = lcdinfo_callback;
	register_lcdinfo_notifier(&ssc_cxt->nb);
	pr_info("ssc_interactive_init success!\n");

	return 0;

register_mdevice_failed:
	kfifo_free(&ssc_cxt->fifo);
parse_dts_failed:
alloc_fifo_failed:
	kfree(ssc_cxt);
alloc_ssc_cxt_failed:
	return err;
}

static void __exit ssc_interactive_exit(void)
{
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;
	unregister_lcdinfo_notifier(&ssc_cxt->nb);
	misc_deregister(&ssc_cxt->mdev);
	kfifo_free(&ssc_cxt->fifo);
	kfree(ssc_cxt);
	ssc_cxt = NULL;
	g_ssc_cxt = NULL;
}

module_init(ssc_interactive_init);
module_exit(ssc_interactive_exit);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("JiangHua.Tang");
