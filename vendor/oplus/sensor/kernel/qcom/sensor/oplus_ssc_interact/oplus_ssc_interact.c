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

#if IS_ENABLED(CONFIG_OPLUS_SENSOR_DRM_PANEL_NOTIFY)
#include <linux/soc/qcom/panel_event_notifier.h>
#include <drm/drm_panel.h>
#include <linux/of.h>
#include <linux/kernel.h>
#include <linux/task_work.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/delay.h>
#endif

#define FIFO_SIZE 32

/*static DECLARE_KFIFO_PTR(test, struct fifo_frame);*/

#if IS_ENABLED(CONFIG_OPLUS_SENSOR_DRM_PANEL_NOTIFY)
/* to do */
#else
extern int register_lcdinfo_notifier(struct notifier_block *nb);
extern int unregister_lcdinfo_notifier(struct notifier_block *nb);
#endif

static struct ssc_interactive *g_ssc_cxt = NULL;

static void ssc_interactive_set_fifo(uint8_t type, uint16_t data)
{
	struct fifo_frame fifo_fm;
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;
	int ret = 0;
	/*pr_info("type= %u, data=%d\n", type, data);*/
	memset(&fifo_fm, 0, sizeof(struct fifo_frame));
	fifo_fm.type = type;
	fifo_fm.data = data;
	ret = kfifo_in_spinlocked(&ssc_cxt->fifo, &fifo_fm, 1, &ssc_cxt->fifo_lock);
	if(ret != 1) {
		pr_err("kfifo is full\n");
	}
	wake_up_interruptible(&ssc_cxt->wq);
}


static void ssc_interactive_set_dc_mode(uint16_t dc_mode)
{
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;
	spin_lock(&ssc_cxt->rw_lock);
	if (dc_mode == ssc_cxt->a_info.dc_mode) {
		spin_unlock(&ssc_cxt->rw_lock);
		return;
	} else {
		pr_info("dc_mode change to %d\n", dc_mode);
	}
	ssc_cxt->a_info.dc_mode = dc_mode;
	spin_unlock(&ssc_cxt->rw_lock);

	ssc_interactive_set_fifo(LCM_DC_MODE_TYPE, dc_mode);
}

static void ssc_interactive_set_blank_mode(uint16_t blank_mode)
{
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;
	uint16_t brigtness = 0;

	spin_lock(&ssc_cxt->rw_lock);
	if (blank_mode == ssc_cxt->a_info.blank_mode) {
		/*pr_info("dc_mode=%d is the same\n", dc_mode);*/
		spin_unlock(&ssc_cxt->rw_lock);
		return;
	}
	ssc_cxt->a_info.blank_mode = blank_mode;
	brigtness = ssc_cxt->last_primary_bri;
	spin_unlock(&ssc_cxt->rw_lock);
	pr_info("set blank_mode=%d, re-send last bri=%d\n", (int)blank_mode, (int)brigtness);
	ssc_interactive_set_fifo(LCM_BLANK_MODE_TYPE, blank_mode);
	ssc_interactive_set_fifo(LCM_BRIGHTNESS_TYPE, brigtness);
}

static void ssc_interactive_set_pwm_turbo_mode(int on)
{
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;
	uint16_t brigtness = 0;
	spin_lock(&ssc_cxt->rw_lock);
	if (ssc_cxt->pwm_turbo_on == on) {
		spin_unlock(&ssc_cxt->rw_lock);
		return;
	} else {
		pr_info("pwm_mode change to %d, bri %d\n", on, ssc_cxt->last_primary_bri);
	}
	ssc_cxt->pwm_turbo_on = on;
	brigtness = ssc_cxt->last_primary_bri;
	spin_unlock(&ssc_cxt->rw_lock);

	ssc_interactive_set_fifo(LCM_PWM_TURBO_TYPE, on);
	if (!g_ssc_cxt->is_fold_dev) {
		ssc_interactive_set_fifo(LCM_BRIGHTNESS_TYPE, brigtness);
	}
}

static void ssc_interactive_set_hbm_mode(int on)
{
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;
	uint16_t brigtness = 0;
	spin_lock(&ssc_cxt->rw_lock);
	if (ssc_cxt->hbm_on == on) {
		spin_unlock(&ssc_cxt->rw_lock);
		return;
	} else {
		pr_info("hbm_mode change to %d, bri %d\n", on, ssc_cxt->last_primary_bri);
	}
	ssc_cxt->hbm_on = on;
	brigtness = ssc_cxt->last_primary_bri;
	spin_unlock(&ssc_cxt->rw_lock);

	if (ssc_cxt->sup_hbm_mode == HBM_MODE_LONG_INTE) {
		ssc_interactive_set_fifo(LCM_HBM_LONG_INTE_TYPE, on);
		ssc_interactive_set_fifo(LCM_BRIGHTNESS_TYPE, brigtness);
	} else if (ssc_cxt->sup_hbm_mode == HBM_MODE_SHORT_INTE) {
		ssc_interactive_set_fifo(LCM_HBM_SHORT_INTE_TYPE, on);
	} else {
		/* do nothing */
	}
}

#if IS_ENABLED(CONFIG_OPLUS_SENSOR_DRM_PANEL_ADFR_MIN_FPS)
static void ssc_interactive_set_flash_freq(enum panel_event_notifier_tag panel_tag, uint16_t freq)
{
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;
	spin_lock(&ssc_cxt->rw_lock);
	ssc_cxt->last_freq = freq;
	spin_unlock(&ssc_cxt->rw_lock);
	pr_info("set_flash_freq %d\n", freq);
	ssc_interactive_set_fifo(LCM_ADFR_MIN_FPS, freq);
}
#endif

static void ssc_interactive_set_brightness(enum panel_event_notifier_tag panel_tag, uint16_t brigtness)
{
	uint16_t cnt = 0;
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;

	if ((panel_tag <= PANEL_EVENT_NOTIFICATION_NONE) ||
		(panel_tag >= PANEL_EVENT_NOTIFICATION_MAX)) {
		pr_err("brightness_store panel_type not match  = %d\n", panel_tag);
		return;
	}

	spin_lock(&ssc_cxt->rw_lock);

	if (PANEL_EVENT_NOTIFICATION_PRIMARY == panel_tag) {
		ssc_cxt->last_primary_bri = brigtness;
		if (!ssc_cxt->brl_info.pri_brl_num) {
			pr_err("brl dts info not configured yet or pri_brl_num(%d) is invalid\n",
				ssc_cxt->brl_info.pri_brl_num);
			spin_unlock(&ssc_cxt->rw_lock);
			return;
		}

		/* set brightness according to each segment */
		for (cnt = 0; cnt < ssc_cxt->brl_info.pri_brl_num; cnt++) {
			/* turn on pwm turbo*/
			if (g_ssc_cxt->pwm_turbo_on) {
				/* do nothing
				 * CWB or SF, real-time transmission of all bri level
				 */
				break;
			}

			if (g_ssc_cxt->hbm_on && g_ssc_cxt->sup_hbm_mode == HBM_MODE_LONG_INTE) {
				/* do nothing
				 * CWB or SF, real-time transmission of all bri level
				 */
				break;
			}

			/* low to high brl: pri_brl_l2h_thrd ~ 2047 */
			if ((brigtness > ssc_cxt->brl_info.pri_brl_l2h_thrd) &&
				(ssc_cxt->brl_info.pri_brl_l2h_thrd != 0)) {
				/* do nothing
				 * CWB or SF, real-time transmission of highlights
				 */
				break;
			}

			if ((0 == cnt) && (brigtness <= ssc_cxt->brl_info.pri_brl_thrd[cnt])) {
				/* brl: 0 ~ pri_brl_thrd[cnt] */
				brigtness = ssc_cxt->brl_info.pri_brl_thrd[cnt] - 1;
			} else if (cnt == (ssc_cxt->brl_info.pri_brl_num - 1) && (brigtness > ssc_cxt->brl_info.pri_brl_thrd[cnt])) {
				/* brl: pri_brl_thrd[max] ~ 2047 */
				brigtness = ssc_cxt->brl_info.pri_brl_thrd[cnt] + 1;
			} else if ((brigtness <= ssc_cxt->brl_info.pri_brl_thrd[cnt]) &&
				(brigtness > ssc_cxt->brl_info.pri_brl_thrd[cnt - 1])) {
				/* brl: pri_brl_thrd[cnt-1] ~ pri_brl_thrd[cnt] */
				brigtness = ssc_cxt->brl_info.pri_brl_thrd[cnt] - 1;
			}
		}
	}
	/* Fold feature for secondary screen brightness level */
	else if (g_ssc_cxt->is_fold_dev &&
			PANEL_EVENT_NOTIFICATION_SECONDARY == panel_tag) {
		if (!ssc_cxt->brl_info.secd_brl_num) {
			pr_err("brl dts info not configured yet or secd_brl_num(%d) is invalid\n",
				ssc_cxt->brl_info.secd_brl_num);
			spin_unlock(&ssc_cxt->rw_lock);
			return;
		}

		/* set brightness according to each segment */
		for (cnt = 0; cnt < ssc_cxt->brl_info.secd_brl_num; cnt++) {
			/* low to high brl: pri_brl_l2h_thrd ~ 2047 */
			if ((brigtness > ssc_cxt->brl_info.secd_brl_l2h_thrd) &&
				(ssc_cxt->brl_info.secd_brl_l2h_thrd != 0)) {
				/* do nothing
				 * CWB or SF, real-time transmission of highlights
				 */
				break;
			}

			if ((0 == cnt) && (brigtness <= ssc_cxt->brl_info.secd_brl_thrd[cnt])) {
				/* brl: 0 ~ pri_brl_thrd[cnt] */
				brigtness = ssc_cxt->brl_info.secd_brl_thrd[cnt] - 1;
			} else if (cnt == (ssc_cxt->brl_info.secd_brl_num - 1) && (brigtness > ssc_cxt->brl_info.secd_brl_thrd[cnt])) {
				/* brl: pri_brl_thrd[max] ~ 2047 */
				brigtness = ssc_cxt->brl_info.secd_brl_thrd[cnt] + 1;
			} else if ((brigtness <= ssc_cxt->brl_info.secd_brl_thrd[cnt]) &&
				(brigtness > ssc_cxt->brl_info.secd_brl_thrd[cnt - 1])) {
				/* brl: pri_brl_thrd[cnt-1] ~ pri_brl_thrd[cnt] */
				brigtness = ssc_cxt->brl_info.secd_brl_thrd[cnt] - 1;
			}
		}
	}

	if (brigtness == ssc_cxt->a_info.brightness) {
		/* pr_info("brigtness=%d is the same\n", brigtness); */
		spin_unlock(&ssc_cxt->rw_lock);
		return;
	}
	/*pr_info("new brigtness=%d, brightness=%d\n", brigtness, ssc_cxt->a_info.brightness);*/

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
	.owner  = THIS_MODULE,
	.read   = ssc_interactive_read,
	.write        = ssc_interactive_write,
	.poll         = ssc_interactive_poll,
	.llseek = generic_file_llseek,
	.release = ssc_interactive_release,
};

static ssize_t brightness_store(struct device *dev,
        struct device_attribute *attr, const char *buf,
        size_t count)
{
	int err = 0;
	uint8_t panel = 0;
	uint8_t br_type = 0;
	uint16_t data = 0;
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;

	err = sscanf(buf, "%hhu %hhu %hu", &panel, &br_type, &data);
	if (err < 0) {
		pr_err("brightness_store error: err = %d\n", err);
		return err;
	}

	if (br_type != LCM_BRIGHTNESS_TYPE) {
		pr_err("brightness_store type not match = %d\n", br_type);
		return count;
	}

	if ((panel <= PANEL_EVENT_NOTIFICATION_NONE) ||
		(panel >= PANEL_EVENT_NOTIFICATION_MAX)) {
		pr_err("brightness_store panel_type not match = %d\n", panel);
		return count;
	} else if (!ssc_cxt->is_fold_dev &&
				panel == PANEL_EVENT_NOTIFICATION_SECONDARY) {
		pr_err("brightness_store, is not the folding phone, panel_type must be 1\n");
		return count;
	}

	ssc_interactive_set_brightness(panel, data);

	pr_info("brightness_store = %s, brightness =%d\n", buf, data);

	return count;
}

static ssize_t brightness_show(struct device *dev,
                   struct device_attribute *attr, char *buf)
{
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;
	uint16_t brightness = 0;

	spin_lock(&ssc_cxt->rw_lock);
	brightness = ssc_cxt->a_info.brightness;
	spin_unlock(&ssc_cxt->rw_lock);

	pr_info("brightness_show brightness=  %d\n", brightness);

	return sprintf(buf, "%d\n", brightness);
}


static ssize_t dc_mode_store(struct device *dev,
        struct device_attribute *attr, const char *buf,
        size_t count)
{
	uint8_t type = 0;
	uint16_t data = 0;
	int err = 0;

	err = sscanf(buf, "%hhu %hu", &type, &data);
	if (err < 0) {
		pr_err("dc_mode_store error: err = %d\n", err);
		return err;
	}
	if (type != LCM_DC_MODE_TYPE) {
		pr_err("dc_mode_store type not match  = %d\n", type);
		return count;
	}
	ssc_interactive_set_dc_mode(data);
	return count;
}

static ssize_t dc_mode_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;
	uint16_t dc_mode = 0;

	spin_lock(&ssc_cxt->rw_lock);
	dc_mode = ssc_cxt->a_info.dc_mode;
	spin_unlock(&ssc_cxt->rw_lock);

	pr_info("dc_mode_show dc_mode= %u\n", dc_mode);

	return snprintf(buf, PAGE_SIZE, "%d\n", dc_mode);
}


DEVICE_ATTR(brightness, 0644, brightness_show, brightness_store);
DEVICE_ATTR(dc_mode, 0644, dc_mode_show, dc_mode_store);


static struct attribute *ssc_interactive_attributes[] = {
	&dev_attr_brightness.attr,
	&dev_attr_dc_mode.attr,
	NULL
};


static struct attribute_group ssc_interactive_attribute_group = {
	.attrs = ssc_interactive_attributes
};

#if IS_ENABLED(CONFIG_OPLUS_SENSOR_DRM_PANEL_NOTIFY)
static void lcdinfo_callback(enum panel_event_notifier_tag panel_tag,
        struct panel_event_notification *notification, void *client_data)
{
	if (!notification) {
		pr_err("Invalid notification\n");
		return;
	}

	/* pr_err("Notification panel_type:%d, type:%d, data:%d",
			panel_tag, notification->notif_type,
			notification->notif_data.data); */

	switch (notification->notif_type) {
	case DRM_PANEL_EVENT_BACKLIGHT:
		if (g_ssc_cxt->need_lb_algo) {
			ssc_interactive_set_brightness(panel_tag, notification->notif_data.data);
		}
		break;
	case DRM_PANEL_EVENT_DC_MODE:
		if (g_ssc_cxt->need_lb_algo) {
			ssc_interactive_set_dc_mode(notification->notif_data.data);
		}
		break;
	case DRM_PANEL_EVENT_PWM_TURBO:
		if (g_ssc_cxt->need_lb_algo) {
			ssc_interactive_set_pwm_turbo_mode(notification->notif_data.data);
		}
		break;
#if IS_ENABLED(CONFIG_OPLUS_SENSOR_DRM_PANEL_ADFR_MIN_FPS)
	case DRM_PANEL_EVENT_ADFR_MIN_FPS:
		ssc_interactive_set_flash_freq(panel_tag, notification->notif_data.data);
		break;
#endif
	case DRM_PANEL_EVENT_HBM_STATE:
		if (g_ssc_cxt->need_lb_algo) {
			ssc_interactive_set_hbm_mode(notification->notif_data.data);
		}
		break;
	case DRM_PANEL_EVENT_UNBLANK:
#if IS_ENABLED(CONFIG_OPLUS_SENSOR_FB_QC)
		if (g_ssc_cxt->sup_power_fb) {
			ssc_fb_set_screen_status(SCREEN_ON);
		}
#endif
		if (g_ssc_cxt->report_blank_mode) {
			ssc_interactive_set_blank_mode(SCREEN_ON);
		}
		break;
	case DRM_PANEL_EVENT_BLANK:
#if IS_ENABLED(CONFIG_OPLUS_SENSOR_FB_QC)
		if (g_ssc_cxt->sup_power_fb) {
			ssc_fb_set_screen_status(SCREEN_OFF);
		}
#endif
		if (g_ssc_cxt->report_blank_mode) {
			ssc_interactive_set_blank_mode(SCREEN_OFF);
		}
		break;
	default:
		break;
	}

	return;
}
#else
static int lcdinfo_callback(struct notifier_block *nb, unsigned long event,
        void *data)
{
	int val = 0;

	if (!data) {
		return 0;
	}
	val = *(int*)data;

	switch(event) {
	case LCM_DC_MODE_TYPE:
		ssc_interactive_set_dc_mode(val);
		break;
	case LCM_BRIGHTNESS_TYPE:
		ssc_interactive_set_brightness(1, val);
		break;
	default:
		break;
	}

	return 0;
}
#endif

#if IS_ENABLED(CONFIG_OPLUS_SENSOR_DRM_PANEL_NOTIFY)
static void ssc_regiseter_lcd_notify_work(struct work_struct *work)
{
	int rc = 0;
	int i;
	int count;
	int count_sec;
	struct device_node *np = NULL;
	struct device_node *node = NULL;
	struct drm_panel *panel = NULL;
	struct device_node *node_sec = NULL;
	struct drm_panel *panel_sec = NULL;
	void *cookie = NULL;
	void *data = NULL;

	np = of_find_node_by_name(NULL, "oplus,dsi-display-dev");
	if (!np) {
		pr_err("display-dev dts info missing.\n");
		return;
	} else {
		pr_err("display-dev dts info found.\n");
	}

	/* for furture mliti-panel extend, need to add code for other panel parse and register */
	count = of_count_phandle_with_args(np, "oplus,dsi-panel-primary", NULL);
	if (count <= 0) {
		pr_err("oplus,dsi-panel-primary not found\n");
		return;
	}

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "oplus,dsi-panel-primary", i);
		panel = of_drm_find_panel(node);
		of_node_put(node);
		if (!IS_ERR(panel)) {
			g_ssc_cxt->active_panel = panel;
			pr_err("Found active panel.\n");
		}
	}

	if (g_ssc_cxt->active_panel) {
		cookie = panel_event_notifier_register(
			PANEL_EVENT_NOTIFICATION_PRIMARY,
			PANEL_EVENT_NOTIFIER_CLIENT_PRIMARY_BACKLIGHT,
			g_ssc_cxt->active_panel, &lcdinfo_callback,
			data);

		if (!cookie) {
			pr_err("Unable to register sensor_primary_panel_notifier\n");
		} else {
			pr_err("success register sensor_primary_panel_notifier\n");
			g_ssc_cxt->notify_work_regiseted = true;
			g_ssc_cxt->notifier_cookie = cookie;
		}
	} else {
		pr_err("can't find active primary panel, rc=%d\n", rc);
	}

	/* for furture folding phone*/
	if (g_ssc_cxt->is_fold_dev) {
		count_sec = of_count_phandle_with_args(np, "oplus,dsi-panel-secondary", NULL);

		if (count_sec <= 0) {
			pr_err("oplus,dsi-panel-secondary not found\n");
			return;
		}

		for (i = 0; i < count_sec; i++) {
			node_sec = of_parse_phandle(np, "oplus,dsi-panel-secondary", i);
			panel_sec = of_drm_find_panel(node_sec);
			of_node_put(node_sec);
			if (!IS_ERR(panel_sec)) {
				g_ssc_cxt->active_panel_second = panel_sec;
				pr_err("Found active secondary panel.\n");
			}
		}

		if (g_ssc_cxt->active_panel_second) {
			cookie = panel_event_notifier_register(
				PANEL_EVENT_NOTIFICATION_SECONDARY,
				PANEL_EVENT_NOTIFIER_CLIENT_SECONDARY_BACKLIGHT,
				g_ssc_cxt->active_panel_second, &lcdinfo_callback,
				data);

			if (!cookie) {
				pr_err("Unable to register sensor_sec_panel_notifier\n");
			} else {
				pr_err("success register sensor_sec_panel_notifier\n");
				g_ssc_cxt->notify_work_regiseted_second = true;
				g_ssc_cxt->notifier_cookie_second = cookie;
			}
		} else {
			pr_err("can't find active panel, rc=%d\n", rc);
		}
	}

	if ((!g_ssc_cxt->notify_work_regiseted || !g_ssc_cxt->notify_work_regiseted_second)
		&& g_ssc_cxt->notify_work_retry > 0) {
		g_ssc_cxt->notify_work_retry--;
		schedule_delayed_work(&g_ssc_cxt->regiseter_lcd_notify_work, msecs_to_jiffies(1000));
	}
	return;
}
#endif

static void parse_br_level_info_dts(struct ssc_interactive *ssc_cxt,
	struct device_node *ch_node)
{
	int rc = 0;
	uint32_t value = 0;
	uint32_t brl_num = 0;

	if (0 == strncmp(ch_node->name, "primary_lb_brl_info", 19)) {
		rc = of_property_read_u32(ch_node, "brl_thrd_num", &value);
		if (!rc) {
			brl_num = value;
			ssc_cxt->brl_info.pri_brl_num = brl_num;
		} else {
			ssc_cxt->brl_info.pri_brl_num = 0;
		}

		if (0 == ssc_cxt->brl_info.pri_brl_num) {
			pr_err("ERROR! can not parse the pri_brl_thrd_num(%u)\n", ssc_cxt->brl_info.pri_brl_num);
			return;
		}

		if (brl_num > 0 && brl_num <= BRL_MAX_LEN) {
			rc = of_property_read_u32_array(ch_node, "brl_thrd",
					&ssc_cxt->brl_info.pri_brl_thrd[0], brl_num);

			if (!rc) {
				/*uint32_t cnt = 0;
				for (cnt = 0; cnt < brl_num; cnt++) {
					pr_info("primary pri_brl_%u: %u\n", cnt, ssc_cxt->brl_info.pri_brl_thrd[cnt]);
				}*/
			}
		}

		value = 0;
		rc = of_property_read_u32(ch_node, "brl_l2h_thrd", &value);
		if (!rc) {
			ssc_cxt->brl_info.pri_brl_l2h_thrd = value;

			if (0 != value && (brl_num <= BRL_MAX_LEN) &&
				ssc_cxt->brl_info.pri_brl_l2h_thrd != ssc_cxt->brl_info.pri_brl_thrd[brl_num - 1]) {
				pr_err("pri_panel brl_l2h(%u) is not match the pri_brl_max(%u)\n",
					ssc_cxt->brl_info.pri_brl_l2h_thrd, ssc_cxt->brl_info.pri_brl_thrd[brl_num - 1]);

				ssc_cxt->brl_info.pri_brl_l2h_thrd = ssc_cxt->brl_info.pri_brl_thrd[brl_num - 1];
			}
		}
	} else if ((ssc_cxt->is_fold_dev) &&
			(0 == strncmp(ch_node->name, "secondary_lb_brl_info", 21))) {
		/* parse the secondary brl info if support folding device */
		rc = of_property_read_u32(ch_node, "brl_thrd_num", &value);
		if (!rc) {
			brl_num = value;
			ssc_cxt->brl_info.secd_brl_num = brl_num;
		} else {
			ssc_cxt->brl_info.secd_brl_num = 0;
		}

		if (0 == ssc_cxt->brl_info.secd_brl_num) {
			pr_err("ERROR! can not parse the secd_brl_thrd_num(%u)\n", ssc_cxt->brl_info.secd_brl_num);
			return;
		}

		if (brl_num > 0 && brl_num <= BRL_MAX_LEN) {
			rc = of_property_read_u32_array(ch_node, "brl_thrd",
					&ssc_cxt->brl_info.secd_brl_thrd[0], brl_num);

			if (!rc) {
				/*uint32_t cnt = 0;
				for (cnt = 0; cnt < brl_num; cnt++) {
					pr_info("secd_brl_%u: %u\n", cnt, ssc_cxt->brl_info.secd_brl_thrd[cnt]);
				}*/
			}
		}

		value = 0;
		rc = of_property_read_u32(ch_node, "brl_l2h_thrd", &value);
		if (!rc) {
			ssc_cxt->brl_info.secd_brl_l2h_thrd = value;
			if (0 != value && (brl_num <= BRL_MAX_LEN) &&
				ssc_cxt->brl_info.secd_brl_l2h_thrd != ssc_cxt->brl_info.secd_brl_thrd[brl_num-1]) {
				pr_err("secd_panel brl_l2h(%u) is not match the secd_brl_max(%u)\n",
					ssc_cxt->brl_info.secd_brl_l2h_thrd, ssc_cxt->brl_info.secd_brl_thrd[brl_num-1]);
				ssc_cxt->brl_info.secd_brl_l2h_thrd = ssc_cxt->brl_info.secd_brl_thrd[brl_num-1];
			}
		}
	}
}

static int __init ssc_interactive_init(void)
{
	int err = 0;
	int lb_value = 0;
	int hbm_mode = 0;
	struct device_node *node = NULL;
	struct ssc_interactive *ssc_cxt = kzalloc(sizeof(*ssc_cxt), GFP_KERNEL);

	g_ssc_cxt = ssc_cxt;

	node = of_find_node_by_name(NULL, "ssc_interactive");
	if (!node) {
		pr_err("ssc_interactive dts info missing.\n");
	} else {
		if (of_property_read_bool(node, "is-folding-device")) {
			ssc_cxt->is_fold_dev = true;
		} else {
			ssc_cxt->is_fold_dev = false;
		}

		if (of_property_read_bool(node, "sup-power-fb")) {
			ssc_cxt->sup_power_fb = true;
			pr_err("sup power_fb!");
		} else {
			ssc_cxt->sup_power_fb = false;
			pr_err("not sup power_fb!");
		}

		if (of_property_read_bool(node, "report_blank_mode")) {
			ssc_cxt->report_blank_mode = true;
			pr_err("sup report_blank_mode!");
		} else {
			ssc_cxt->report_blank_mode = false;
			pr_err("not sup report_blank_mode!");
		}

		err = of_property_read_u32(node, "sup-hbm-mode", &hbm_mode);
		if (!err) {
			ssc_cxt->sup_hbm_mode = hbm_mode;
			pr_info("get hbm_mode:%d \n", hbm_mode);
		}

		err = of_property_read_u32(node, "need_lb_algo", &lb_value);
		if (!err && lb_value) {
			ssc_cxt->need_lb_algo = true;
			pr_info("get the lb_value:%d \n", lb_value);
		} else {
			int rc = 0;
			struct device_node *lb_check = NULL;
			lb_check = of_find_node_with_property(NULL, "use_lb_algo");
			if (NULL != lb_check) {
				rc = of_property_read_u32(lb_check, "use_lb_algo", &lb_value);
				if (0 == rc && 1 == lb_value) {
					ssc_cxt->need_lb_algo = false;
				} else {
					pr_info("parse lb algo flag failed!\n");
				}
			}
		}

		if (ssc_cxt->need_lb_algo) {
			struct device_node *ch_node = NULL;
			for_each_child_of_node(node, ch_node) {
				parse_br_level_info_dts(ssc_cxt, ch_node);
			}
		}
	}

	if (ssc_cxt == NULL) {
		pr_err("kzalloc ssc_cxt failed\n");
		err = -ENOMEM;
		goto alloc_ssc_cxt_failed;
	}
	err = kfifo_alloc(&ssc_cxt->fifo, FIFO_SIZE, GFP_KERNEL);
	if (err) {
		pr_err("kzalloc kfifo failed\n");
		err = -ENOMEM;
		goto alloc_fifo_failed;
	}

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

	if (ssc_cxt->need_lb_algo || ssc_cxt->sup_power_fb) {
#if IS_ENABLED(CONFIG_OPLUS_SENSOR_DRM_PANEL_NOTIFY)
		ssc_cxt->notify_work_retry = 10;
		ssc_cxt->notify_work_regiseted = false;
		g_ssc_cxt->notify_work_regiseted_second = false;
		INIT_DELAYED_WORK(&ssc_cxt->regiseter_lcd_notify_work, ssc_regiseter_lcd_notify_work);
		schedule_delayed_work(&ssc_cxt->regiseter_lcd_notify_work, msecs_to_jiffies(5000));
#else
		ssc_cxt->nb.notifier_call = lcdinfo_callback;
		register_lcdinfo_notifier(&ssc_cxt->nb);
#endif
	}

	err = sysfs_create_group(&ssc_cxt->mdev.this_device->kobj, &ssc_interactive_attribute_group);
	if (err < 0) {
		pr_err("unable to create ssc_interactive_attribute_group file err=%d\n", err);
		goto sysfs_create_failed;
	}

	pr_info("ssc_interactive_init success!\n");

	return 0;
sysfs_create_failed:
	misc_deregister(&ssc_cxt->mdev);
register_mdevice_failed:
	kfifo_free(&ssc_cxt->fifo);
alloc_fifo_failed:
	kfree(ssc_cxt);
alloc_ssc_cxt_failed:
	return err;
}

static void __exit ssc_interactive_exit(void)
{
	struct ssc_interactive *ssc_cxt = g_ssc_cxt;
	sysfs_remove_group(&ssc_cxt->mdev.this_device->kobj, &ssc_interactive_attribute_group);
#if IS_ENABLED(CONFIG_OPLUS_SENSOR_DRM_PANEL_NOTIFY)
	if (ssc_cxt->active_panel && ssc_cxt->notifier_cookie) {
		panel_event_notifier_unregister(ssc_cxt->notifier_cookie);
	}
#else
	unregister_lcdinfo_notifier(&ssc_cxt->nb);
#endif
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
