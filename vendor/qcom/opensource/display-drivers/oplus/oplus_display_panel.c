/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
**
** File : oplus_display_panel.c
** Description : oplus display panel char dev  /dev/oplus_panel
** Version : 1.0
** Date : 2020/06/13
** Author : Display
******************************************************************/
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/dma-buf.h>
#include <linux/fs.h>
#include <linux/mm_types.h>
#include "oplus_display_panel.h"
#include "oplus_display_private_api.h"
/* OPLUS_FEATURE_ADFR, include header file*/
#include "oplus_adfr.h"

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "oplus_onscreenfingerprint.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

struct oplus_apollo_backlight_list *p_apollo_backlight = NULL;
static int oplus_display_set_apollo_backlight_value(void *data);

#define PANEL_IOCTL_DEF(ioctl, _func) \
	[PANEL_IOCTL_NR(ioctl)] = {		\
		.cmd = ioctl,			\
		.func = _func,			\
		.name = #ioctl,			\
	}

static const struct panel_ioctl_desc panel_ioctls[] = {
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_POWER, oplus_display_panel_set_pwr),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_POWER, oplus_display_panel_get_pwr),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_SEED, oplus_display_panel_set_seed),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_SEED, oplus_display_panel_get_seed),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_PANELID, oplus_display_panel_get_id),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_FFL, oplus_display_panel_set_ffl),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_FFL, oplus_display_panel_get_ffl),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_AOD, oplus_ofp_set_aod_light_mode),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_AOD, oplus_ofp_get_aod_light_mode),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_MAX_BRIGHTNESS, oplus_display_panel_set_max_brightness),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_MAX_BRIGHTNESS, oplus_display_panel_get_max_brightness),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_PANELINFO, oplus_display_panel_get_vendor),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_CCD, oplus_display_panel_get_ccd_check),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_SERIAL_NUMBER, oplus_display_panel_get_serial_number),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_HBM, oplus_ofp_set_hbm),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_HBM, oplus_ofp_get_hbm),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_DIM_ALPHA, oplus_display_panel_set_dim_alpha),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_DIM_ALPHA, oplus_display_panel_get_dim_alpha),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_DIM_DC_ALPHA, oplus_display_panel_set_dim_alpha),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_DIM_DC_ALPHA, oplus_display_panel_get_dim_dc_alpha),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_AUDIO_READY, oplus_display_panel_set_audio_ready),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_DISPLAY_TIMING_INFO, oplus_display_panel_dump_info),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_PANEL_DSC, oplus_display_panel_get_dsc),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_POWER_STATUS, oplus_display_panel_set_power_status),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_POWER_STATUS, oplus_display_panel_get_power_status),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_REGULATOR_CONTROL, oplus_display_panel_regulator_control),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_CLOSEBL_FLAG, oplus_display_panel_set_closebl_flag),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_CLOSEBL_FLAG, oplus_display_panel_get_closebl_flag),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_PANEL_REG, oplus_display_panel_set_reg),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_PANEL_REG, oplus_display_panel_get_reg),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_DIMLAYER_HBM, oplus_ofp_set_dimlayer_hbm),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_DIMLAYER_HBM, oplus_ofp_get_dimlayer_hbm),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_DIMLAYER_BL_EN, oplus_display_panel_set_dimlayer_enable),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_DIMLAYER_BL_EN, oplus_display_panel_get_dimlayer_enable),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_PANEL_BLANK, oplus_display_panel_notify_blank),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_SPR, oplus_display_panel_set_spr),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_SPR, oplus_display_panel_get_spr),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_ROUNDCORNER, oplus_display_panel_get_roundcorner),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_DYNAMIC_OSC_CLOCK, oplus_display_panel_set_dynamic_osc_clock),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_DYNAMIC_OSC_CLOCK, oplus_display_panel_get_dynamic_osc_clock),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_FP_PRESS, oplus_ofp_notify_fp_press),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_OPLUS_BRIGHTNESS, oplus_display_panel_set_brightness),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_OPLUS_BRIGHTNESS, oplus_display_panel_get_brightness),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_OPLUS_MAXBRIGHTNESS, oplus_display_panel_get_oplus_max_brightness),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_QCOM_LOG_LEVEL, oplus_display_panel_set_qcom_loglevel),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_ULTRA_LOW_POWER_AOD, oplus_ofp_set_ultra_low_power_aod_mode),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_ULTRA_LOW_POWER_AOD, oplus_ofp_get_ultra_low_power_aod_mode),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_LCD_MAX_BRIGHTNESS, oplus_display_panel_get_lcd_max_brightness),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_APOLLO_BACKLIGHT, oplus_display_set_apollo_backlight_value),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_SOFTIRIS_COLOR, oplus_display_panel_get_softiris_color_status),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_DITHER_STATUS, oplus_display_panel_set_dither),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_DITHER_STATUS, oplus_display_panel_get_dither),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_TE_REFCOUNT_ENABLE, oplus_enable_te_refcount),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_TE_REFCOUNT_ENABLE, oplus_get_te_fps),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_DP_SUPPORT, oplus_display_panel_get_dp_support),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_CABC_STATUS, oplus_display_panel_set_cabc_status),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_CABC_STATUS, oplus_display_panel_get_cabc_status),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_DRE_STATUS, oplus_display_panel_set_dre_status),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_DRE_STATUS, oplus_display_panel_get_dre_status),
	/* OPLUS_FEATURE_ADFR, dynamic te detect */
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_DYNAMIC_TE, oplus_display_set_dynamic_te),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_DYNAMIC_TE, oplus_display_get_dynamic_te),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_IRIS_LOOP_STATUS, oplus_display_panel_get_iris_loopback_status),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_FP_TYPE, oplus_ofp_set_fp_type),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_FP_TYPE, oplus_ofp_get_fp_type),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_PWM_TURBO, oplus_display_panel_set_pwm_turbo),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_PWM_TURBO, oplus_display_panel_get_pwm_turbo),
};

int oplus_display_fix_apollo_level(void)
{
	unsigned int *apollo_id = (unsigned int *) p_apollo_backlight->vaddr;

	if (apollo_id == NULL) {
		LCD_ERR("error ptr\n");
		return -1;
	}

	if (!p_apollo_backlight->bl_fix) {
		if (apollo_id[0] == APOLLO_BL_4096) {
			p_apollo_backlight->bl_id_lens = APOLLO_BL_4096;
			p_apollo_backlight->apollo_bl_list += sizeof(unsigned int)/sizeof(unsigned short);
			p_apollo_backlight->panel_bl_list = p_apollo_backlight->apollo_bl_list + APOLLO_BL_4096;
			p_apollo_backlight->bl_fix = true;
		} else if (apollo_id[0] == APOLLO_BL_8192) {
			p_apollo_backlight->bl_id_lens = APOLLO_BL_8192;
			p_apollo_backlight->apollo_bl_list += sizeof(unsigned int)/sizeof(unsigned short);
			p_apollo_backlight->panel_bl_list = p_apollo_backlight->apollo_bl_list + APOLLO_BL_8192;
			p_apollo_backlight->bl_fix = true;
		} else {
			p_apollo_backlight->bl_id_lens = -1;
			p_apollo_backlight->bl_fix = false;
		}
	}

	LCD_INFO("apollo_id = [%d], id_value = [%d]\n", p_apollo_backlight->bl_id_lens, apollo_id[0]);

	return 0;
}

static int oplus_display_set_apollo_backlight_value(void *data)
{
	struct apollo_backlight_map_value *p_apollo = data;
	int index = p_apollo->index;
	int ret = 0;

	ret = oplus_display_fix_apollo_level();
	if (ret != 0) {
		return ret;
	}

	p_apollo_backlight->panel_bl_list[index] = p_apollo->bl_level;
	p_apollo_backlight->apollo_bl_list[index] = p_apollo->apollo_bl_level;
	LCD_INFO("panel_level = %d, apollo_level = %d\n",
		p_apollo->bl_level, p_apollo->apollo_bl_level);

	return ret;
}

static struct sg_table *panel_map_dma_buf(struct dma_buf_attachment *attachment,
					 enum dma_data_direction dir)
{
	return NULL;
}

static void panel_unmap_dma_buf(struct dma_buf_attachment *attachment,
			       struct sg_table *st,
			       enum dma_data_direction dir)
{
	return;
}

static void panel_dmabuf_release(struct dma_buf *dma_buf)
{
	kfree(p_apollo_backlight->vaddr);
}

static int panel_dmabuf_mmap(struct dma_buf *dma_buf, struct vm_area_struct *vma)
{
	void *vaddr = p_apollo_backlight->vaddr;
	int ret = 0;

	if (vma->vm_end - vma->vm_start > p_apollo_backlight->buf_size) {
		LCD_ERR("error: exceed the max size, please recheck\n");
		return -EINVAL;
	}

	ret = remap_pfn_range(vma, vma->vm_start, virt_to_pfn(vaddr),
		vma->vm_end - vma->vm_start, vma->vm_page_prot);
	LCD_ERR("mmap ret = %d, size = %d\n", ret, vma->vm_end - vma->vm_start);

	return ret;
}

static const struct dma_buf_ops oplus_dmabuf_ops = {
	.mmap = panel_dmabuf_mmap,
	.map_dma_buf = panel_map_dma_buf,
	.unmap_dma_buf = panel_unmap_dma_buf,
	.release = panel_dmabuf_release,
};

static int oplus_export_dmabuf(int buf_size)
{
	int retcode = 0;
	DEFINE_DMA_BUF_EXPORT_INFO(oplus_exp_info);
	struct dma_buf *dmabuf = NULL;
	unsigned long vaddr;
	char *bl_addr = NULL;
	int page_order = 0;

	if (buf_size%PAGE_SIZE != 0) {
		page_order = buf_size/PAGE_SIZE + 1;
	} else {
		page_order = buf_size/PAGE_SIZE;
	}

	p_apollo_backlight = kzalloc(sizeof(struct oplus_apollo_backlight_list), GFP_KERNEL);
	if (!p_apollo_backlight) {
		retcode = -ENOMEM;
		LCD_ERR("kzalloc fail\n");
		goto err_backlightbuf;
	}

	vaddr = __get_free_pages(GFP_KERNEL, page_order);
	if (!vaddr) {
		retcode = -ENOMEM;
		LCD_ERR("alloc_pages fail\n");
		goto err_dmabuf;
	}

	bl_addr = (char *)vaddr;
	sprintf(bl_addr, "dma test!");

	oplus_exp_info.ops = &oplus_dmabuf_ops;
	oplus_exp_info.size = page_order*PAGE_SIZE;
	oplus_exp_info.flags = O_CLOEXEC;
	oplus_exp_info.priv = p_apollo_backlight;

	dmabuf = dma_buf_export(&oplus_exp_info);
	if (IS_ERR(dmabuf)) {
		retcode = -EINVAL;
		LCD_ERR("dma_buf_export fail\n");
		goto err_export_dmabuf;
	}

	p_apollo_backlight->buf_size = page_order*PAGE_SIZE;
	p_apollo_backlight->dmabuf = dmabuf;
	p_apollo_backlight->vaddr = (void *)vaddr;
	p_apollo_backlight->apollo_bl_list = (unsigned short *)vaddr;
	p_apollo_backlight->panel_bl_list = (unsigned short *)(vaddr)
		+ APOLLO_BACKLIGHT_LENS/sizeof(unsigned int);
	p_apollo_backlight->bl_index_last = -1;
	p_apollo_backlight->bl_level_last = -125;
	LCD_INFO("buf_size = %d bytes, p_panel_backlight = %p, vaddr = %p\n",
		p_apollo_backlight->buf_size, p_apollo_backlight->panel_bl_list,
		p_apollo_backlight->vaddr);

	return 0;

err_export_dmabuf:
	free_pages(vaddr, page_order);
err_dmabuf:
	kfree(p_apollo_backlight);
err_backlightbuf:
	return retcode;
}

static int panel_open(struct inode *inode, struct file *filp)
{
	if (panel_ref > 3) {
		LCD_ERR("panel has already open\n");
		return -1;
	}

	if (panel_ref == 1) {
		try_module_get(THIS_MODULE);
	}

	++panel_ref;

	return 0;
}

extern ssize_t oplus_sde_evtlog_dump_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos);

static ssize_t panel_read(struct file *filp, char __user *buffer,
		size_t count, loff_t *offset)
{
	ssize_t lens = 0;

	lens += oplus_sde_evtlog_dump_read(filp, buffer, count, offset);
	if (lens < 0) {
		lens = 0;
	}
	/* other dump add here, as for lens add */

	return lens;
}

static ssize_t panel_write(struct file *file, const char __user *buffer,
		size_t count, loff_t *f_pos)
{
	LCD_INFO("panel write\n");
	return count;
}

long panel_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned int in_size, out_size, drv_size, ksize;
	unsigned int nr = PANEL_IOCTL_NR(cmd);
	char static_data[128];
	char *kdata = NULL;
	const struct panel_ioctl_desc *ioctl = NULL;
	oplus_panel_feature *func = NULL;
	int retcode = -EINVAL;

	if ((nr >= PANEL_COMMOND_MAX) || (nr <= PANEL_COMMOND_BASE)) {
		LCD_ERR("invalid cmd\n");
		return retcode;
	}

	ioctl = &panel_ioctls[nr];
	func = ioctl->func;

	if (unlikely(!func)) {
		LCD_ERR("no function\n");
		retcode = -EINVAL;
		return retcode;
	}

	in_size = out_size = drv_size = PANEL_IOCTL_SIZE(cmd);

	if ((cmd & ioctl->cmd & IOC_IN) == 0) {
		in_size = 0;
	}

	if ((cmd & ioctl->cmd & IOC_OUT) == 0) {
		out_size = 0;
	}

	ksize = max(max(in_size, out_size), drv_size);

	if (!strcmp(ioctl->name, "PANEL_IOCTL_GET_OPLUS_BRIGHTNESS")) {
		LCD_DEBUG_BACKLIGHT("pid = %d, cmd = %s\n",
				task_pid_nr(current), ioctl->name);
	} else {
		LCD_INFO("pid = %d, cmd = %s\n",
				task_pid_nr(current), ioctl->name);
	}

	if (ksize <= sizeof(static_data)) {
		kdata = static_data;

	} else {
		kdata = kmalloc(ksize, GFP_KERNEL);
		if (!kdata) {
			retcode = -ENOMEM;
			goto err_panel;
		}
	}

	if (copy_from_user(kdata, (void __user *)arg, in_size) != 0) {
		retcode = -EFAULT;
		goto err_panel;
	}

	if (ksize > in_size) {
		memset(kdata + in_size, 0, ksize - in_size);
	}

	retcode = func(kdata);  /*any lock here?*/

	if (copy_to_user((void __user *)arg, kdata, out_size) != 0) {
		retcode = -EFAULT;
		goto err_panel;
	}

err_panel:

	if (!ioctl) {
		LCD_ERR("invalid ioctl\n");
	}

	if (kdata != static_data) {
		kfree(kdata);
	}

	if (retcode) {
		LCD_ERR("pid = %d, retcode = %d\n", task_pid_nr(current), retcode);
	}

	return retcode;
}

int panel_release(struct inode *inode, struct file *filp)
{
	--panel_ref;
	module_put(THIS_MODULE);
	LCD_INFO("panel release\n");

	return 0;
}

static int panel_mmap(struct file *file, struct vm_area_struct *vma)
{
	return dma_buf_mmap(p_apollo_backlight->dmabuf, vma, 0);
}

static const struct file_operations panel_ops =
{
	.owner              = THIS_MODULE,
	.open               = panel_open,
	.release            = panel_release,
	.unlocked_ioctl     = panel_ioctl,
	.compat_ioctl       = panel_ioctl,
	.read               = panel_read,
	.write              = panel_write,
	.mmap               = panel_mmap,
};

int oplus_display_panel_init(void)
{
	int rc = 0;

	LCD_INFO("panel init\n");

	rc = alloc_chrdev_region(&dev_num, 0, 1, OPLUS_PANEL_NAME);

	if (rc < 0) {
		LCD_ERR("failed to alloc chrdev region\n");
		return rc;
	}

	panel_class = class_create(THIS_MODULE, OPLUS_PANEL_CLASS_NAME);

	if (IS_ERR(panel_class)) {
		LCD_ERR("class create error\n");
		goto err_class_create;
	}

	cdev_init(&panel_cdev, &panel_ops);
	rc = cdev_add(&panel_cdev, dev_num, 1);

	if (rc < 0) {
		LCD_ERR("failed to add cdev\n");
		goto err_cdev_add;
	}

	panel_dev = device_create(panel_class, NULL, dev_num, NULL, OPLUS_PANEL_NAME);

	if (IS_ERR(panel_dev)) {
		LCD_ERR("device create error\n");
		goto err_device_create;
	}

	rc = oplus_export_dmabuf(APOLLO_BACKLIGHT_LENS);
	if (rc < 0) {
		LCD_ERR("dmabuf alloc fail\n");
		goto err_device_create;
	}

	return 0;

err_device_create:
	cdev_del(&panel_cdev);
err_cdev_add:
	class_destroy(panel_class);
err_class_create:
	unregister_chrdev_region(dev_num, 1);

	return rc;
}
