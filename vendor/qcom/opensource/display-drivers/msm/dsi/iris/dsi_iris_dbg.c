// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include "dsi_iris_api.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_lightup_ocp.h"
#include "dsi_iris_pq.h"
#include "dsi_iris_timing_switch.h"
#include "dsi_iris_ioctl.h"
#include "dsi_iris_lut.h"
#include "dsi_iris_log.h"
#include <linux/kobject.h>

#define IRIS_DBG_TOP_DIR "iris"
#define IRIS_DBG_FUNCSTATUS_FILE "iris_func_status"

int iris_dbg_fstatus_open(struct inode *inode, struct file *file)
{
	if (inode->i_private)
		file->private_data = inode->i_private;
	return 0;
}

/**
 * read module's status
 */
static ssize_t iris_dbg_fstatus_read(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	char *kbuf = NULL;
	int size = count < PAGE_SIZE ? PAGE_SIZE : (int)count;
	int len = 0;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (*ppos)
		return 0;

	kbuf = vzalloc(size);
	if (kbuf == NULL) {
		IRIS_LOGE("Fatal erorr: No mem!\n");
		return -ENOMEM;
	}

	len += snprintf(kbuf+len, size - len,
			"***system_brightness***\n"
			"%-20s:\t%d\n",
			"system_brightness",
			pqlt_cur_setting->system_brightness);

	len += snprintf(kbuf+len, size - len,
			"***dspp_dirty***\n"
			"%-20s:\t%d\n",
			"dspp_dirty", pqlt_cur_setting->dspp_dirty);


	len += snprintf(kbuf+len, size - len,
			"***cm_setting***\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n",
			"cmcolortempmode",
			pqlt_cur_setting->pq_setting.cmcolortempmode,
			"colortempvalue", pqlt_cur_setting->colortempvalue,
			"min_colortempvalue",
			pqlt_cur_setting->min_colortempvalue,
			"max_colortempvalue",
			pqlt_cur_setting->max_colortempvalue,
			"cmcolorgamut",
			pqlt_cur_setting->pq_setting.cmcolorgamut,
			"demomode", pqlt_cur_setting->pq_setting.demomode,
			"source_switch", pqlt_cur_setting->source_switch);

	len += snprintf(kbuf+len, size - len,
			"***lux_value***\n"
			"%-20s:\t%d\n",
			"luxvalue", pqlt_cur_setting->luxvalue);

	len += snprintf(kbuf+len, size - len,
			"***cct_value***\n"
			"%-20s:\t%d\n",
			"cctvalue", pqlt_cur_setting->cctvalue);

	len += snprintf(kbuf+len, size - len,
			"***reading_mode***\n"
			"%-20s:\t%d\n",
			"readingmode", pqlt_cur_setting->pq_setting.readingmode);

	len += snprintf(kbuf+len, size - len,
			"***ambient_lut***\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n",
			"al_en", pqlt_cur_setting->pq_setting.alenable,
			"al_luxvalue", pqlt_cur_setting->luxvalue,
			"al_bl_ratio", pqlt_cur_setting->al_bl_ratio);

	len += snprintf(kbuf+len, size - len,
			"***sdr2hdr***\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n",
			"sdr2hdr", pqlt_cur_setting->pq_setting.sdr2hdr,
			"maxcll", pqlt_cur_setting->maxcll);

	len += snprintf(kbuf+len, size - len,
			"***analog_abypass***\n"
			"%-20s:\t%d\n",
			"abyp_mode", pcfg->abyp_ctrl.abypass_mode);

	len += snprintf(kbuf+len, size - len,
			"***n2m***\n"
			"%-20s:\t%d\n",
			"n2m_en", pcfg->n2m_enable);

	len += snprintf(kbuf+len, size - len,
			"***osd protect window***\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n",
			"osd0_tl", pcfg->frc_setting.iris_osd0_tl,
			"osd0_br", pcfg->frc_setting.iris_osd0_br,
			"osd1_tl", pcfg->frc_setting.iris_osd1_tl,
			"osd1_br", pcfg->frc_setting.iris_osd1_br,
			"osd2_tl", pcfg->frc_setting.iris_osd2_tl,
			"osd2_br", pcfg->frc_setting.iris_osd2_br,
			"osd3_tl", pcfg->frc_setting.iris_osd3_tl,
			"osd3_br", pcfg->frc_setting.iris_osd3_br,
			"osd4_tl", pcfg->frc_setting.iris_osd4_tl,
			"osd4_br", pcfg->frc_setting.iris_osd4_br,
			"osd_win_ctrl", pcfg->frc_setting.iris_osd_window_ctrl,
			"osd_win_ctrl", pcfg->frc_setting.iris_osd_win_dynCompensate);

	len += snprintf(kbuf+len, size - len,
			"***firmware_version***\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d%d/%d/%d\n",
			"version", pcfg->app_version,
			"date", pcfg->app_date[3], pcfg->app_date[2], pcfg->app_date[1], pcfg->app_date[0]);

	size = len;
	if (len >= count)
		size = count - 1;

	if (copy_to_user(ubuf, kbuf, size)) {
		vfree(kbuf);
		return -EFAULT;
	}

	vfree(kbuf);

	*ppos += size;

	return size;
}


static const struct file_operations iris_dbg_fstatus_fops = {
	.open = iris_dbg_fstatus_open,
	.read = iris_dbg_fstatus_read,
};

void iris_display_mode_name_update(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg == NULL)
		return;

	strlcpy(pcfg->display_mode_name, "Not Set", sizeof(pcfg->display_mode_name));

	if (pcfg->abyp_ctrl.abypass_mode == ANALOG_BYPASS_MODE) {
		if (pcfg->lp_ctrl.abyp_lp == 2)
			strlcpy(pcfg->display_mode_name, "SLEEP-ABYPASS", sizeof(pcfg->display_mode_name));
		else
			strlcpy(pcfg->display_mode_name, "STANDBY-ABYPASS", sizeof(pcfg->display_mode_name));
	} else {
		if (pcfg->dual_enabled) {
			if (pcfg->pwil_mode == FRC_MODE)
				strlcpy(pcfg->display_mode_name, "DUAL-MEMC", sizeof(pcfg->display_mode_name));
			else
				strlcpy(pcfg->display_mode_name, "DUAL-PT", sizeof(pcfg->display_mode_name));
		} else {
			if (pcfg->pwil_mode == FRC_MODE)
				strlcpy(pcfg->display_mode_name, "SINGLE-MEMC", sizeof(pcfg->display_mode_name));
			else if (pcfg->pwil_mode == RFB_MODE)
				strlcpy(pcfg->display_mode_name, "SINGLE-RFB", sizeof(pcfg->display_mode_name));
			else {
				if (pcfg->pt_sr_enable)
					strlcpy(pcfg->display_mode_name, "SINGLE-PT-SR", sizeof(pcfg->display_mode_name));
				else
					strlcpy(pcfg->display_mode_name, "SINGLE-PT", sizeof(pcfg->display_mode_name));
			}
		}
	}
}

int iris_debug_display_mode_get(char *kbuf, int size, bool debug)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_switch_dump *dump = &pcfg->switch_dump;
	int len = 0;

	iris_display_mode_name_update();

	len += snprintf(kbuf, size,
			"%-20s:\t%s\n", "Display mode", pcfg->display_mode_name);
	if (pcfg->pt_sr_enable) {
		len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d x %d\n", "Iris pt_sr res", pcfg->pt_sr_hsize, pcfg->pt_sr_vsize);
	}

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Memc mode", pcfg->memc_info.memc_mode);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Memc app index", pcfg->memc_info.memc_app);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Memc video fps", pcfg->memc_info.video_fps);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Memc panel fps", pcfg->memc_info.panel_fps);

	if (!debug)
		return len;

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Memc vfr enable", pcfg->memc_info.vfr_en);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Memc TNR mode", pcfg->memc_info.tnr_en);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Memc Low Latency", pcfg->memc_info.low_latency_mode);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Memc N2M mode", pcfg->memc_info.n2m_mode);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Memc frc label", pcfg->frc_label);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Memc demo window", pcfg->frc_demo_window);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "AP mipi1 power", pcfg->ap_mipi1_power_st);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "AP mipi1 refresh", pcfg->iris_osd_autorefresh_enabled);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "AP video wo osd", pcfg->video_update_wo_osd);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Iris mipi1 power", pcfg->iris_mipi1_power_st);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Iris pwil mode", pcfg->iris_pwil_mode_state);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Iris blend mode", pcfg->iris_pwil_blend_st);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Iris osd overflow", pcfg->iris_osd_overflow_st);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Iris osd irq count", pcfg->osd_irq_cnt);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Iris frc vfr state", pcfg->iris_frc_vfr_st);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%08x\n", "Iris fw version", pcfg->app_version1);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Iris fw setting",
			(pcfg->frc_setting.mv_hres << 16) | pcfg->frc_setting.mv_vres);

	if (dump->trigger) {
		len += snprintf(kbuf + len, size - len,
				"SWITCH TIMEOUT DUMP\n");

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Sys power ctrl", dump->sw_pwr_ctrl);

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Rx frame count0", dump->rx_frame_cnt0);

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Rx frame count1", dump->rx_frame_cnt1);

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Rx video meta", dump->rx_video_meta);

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Pwil cur meta0", dump->pwil_cur_meta0);

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Pwil status", dump->pwil_status);

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Pwil disp ctrl0", dump->pwil_disp_ctrl0);

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Pwil int", dump->pwil_int);

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Dport int", dump->dport_int);

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Fi debugbus0", dump->fi_debugbus[0]);

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Fi debugbus1", dump->fi_debugbus[1]);

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Fi debugbus2", dump->fi_debugbus[2]);
	}

	return len;
}

static ssize_t iris_dbg_display_mode_show(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	char *kbuf = NULL;
	int size = count < PAGE_SIZE ? PAGE_SIZE : count;

	if (*ppos)
		return 0;

	kbuf = vzalloc(size);
	if (kbuf == NULL) {
		IRIS_LOGE("Fatal erorr: No mem!\n");
		return -ENOMEM;
	}

	size = iris_debug_display_mode_get(kbuf, size, false);
	if (size >= count)
		size = count - 1;

	if (copy_to_user(ubuf, kbuf, size)) {
		vfree(kbuf);
		return -EFAULT;
	}

	vfree(kbuf);

	*ppos += size;

	return size;
}

static ssize_t display_mode_show(struct kobject *obj, struct kobj_attribute *attr, char *buf)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	iris_display_mode_name_update();
	return snprintf(buf, PAGE_SIZE, "%s\n", pcfg->display_mode_name);
}

#define IRIS_ATTR(_name, _mode, _show, _store)\
struct kobj_attribute iris_attr_##_name = __ATTR(_name, _mode, _show, _store)
static IRIS_ATTR(display_mode, S_IRUGO|S_IWUSR, display_mode_show, NULL);

static struct attribute *iris_dbg_dev_attrs[] = {
	&iris_attr_display_mode.attr,
	NULL,
};

static const struct attribute_group iris_dbg_attr_group = {
	.attrs = iris_dbg_dev_attrs,
};

static const struct file_operations iris_dbg_dislay_mode_fops = {
	.open = simple_open,
	.read = iris_dbg_display_mode_show,
};

int iris_debug_display_info_get(char *kbuf, int size)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_panel *panel;
	int len = 0;

	panel = pcfg->panel;
	if (panel) {
		len += snprintf(kbuf, size,
				"%-20s:\t%s\n", "panel name", panel->name);
		len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "panel state", panel->panel_initialized);

		if (panel->cur_mode && panel->cur_mode->priv_info) {
			len += snprintf(kbuf + len, size - len,
					"%-20s:\t%dx%d@%d\n", "panel timing", panel->cur_mode->timing.h_active,
					panel->cur_mode->timing.v_active, panel->cur_mode->timing.refresh_rate);

			len += snprintf(kbuf + len, size - len,
					"%-20s:\t%d/%d\n", "panel transfer time",
					panel->cur_mode->priv_info->mdp_transfer_time_us,
					panel->cur_mode->priv_info->dsi_transfer_time_us);

			len += snprintf(kbuf + len, size - len,
					"%-20s:\t%d\n", "panel clock", panel->cur_mode->priv_info->clk_rate_hz);

			len += snprintf(kbuf + len, size - len,
					"%-20s:\t%d\n", "panel dsc", panel->cur_mode->priv_info->dsc_enabled);

			if (panel->cur_mode->priv_info->dsc_enabled) {
				len += snprintf(kbuf + len, size - len,
						"%-20s:\t%d\n", "panel dsc bpc",
						panel->cur_mode->priv_info->dsc.config.bits_per_component);
				len += snprintf(kbuf + len, size - len,
						"%-20s:\t%d\n", "panel dsc bpp",
						panel->cur_mode->priv_info->dsc.config.bits_per_pixel >> 4);
			}
		}
	}

	panel = pcfg->panel2;
	if (panel) {
		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%s\n", "panel name", panel->name);
		len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "panel state", panel->panel_initialized);

		if (panel->cur_mode && panel->cur_mode->priv_info) {
			len += snprintf(kbuf + len, size - len,
					"%-20s:\t%dx%d@%d\n", "panel timing", panel->cur_mode->timing.h_active,
					panel->cur_mode->timing.v_active, panel->cur_mode->timing.refresh_rate);

			len += snprintf(kbuf + len, size - len,
					"%-20s:\t%d/%d\n", "panel transfer time",
					panel->cur_mode->priv_info->mdp_transfer_time_us,
					panel->cur_mode->priv_info->dsi_transfer_time_us);

			len += snprintf(kbuf + len, size - len,
					"%-20s:\t%d\n", "panel clock", panel->cur_mode->priv_info->clk_rate_hz);

			len += snprintf(kbuf + len, size - len,
					"%-20s:\t%d\n", "panel dsc", panel->cur_mode->priv_info->dsc_enabled);

			if (panel->cur_mode->priv_info->dsc_enabled) {
				len += snprintf(kbuf + len, size - len,
						"%-20s:\t%d\n", "panel dsc bpc",
						panel->cur_mode->priv_info->dsc.config.bits_per_component);
				len += snprintf(kbuf + len, size - len,
						"%-20s:\t%d\n", "panel dsc bpp",
						panel->cur_mode->priv_info->dsc.config.bits_per_pixel >> 4);
			}
		}
	}

	return len;
}

static ssize_t iris_dbg_display_info_show(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	char *kbuf = NULL;
	int size = count < PAGE_SIZE ? PAGE_SIZE : count;

	if (*ppos)
		return 0;

	kbuf = vzalloc(size);
	if (kbuf == NULL) {
		IRIS_LOGE("Fatal erorr: No mem!\n");
		return -ENOMEM;
	}

	size = iris_debug_display_info_get(kbuf, size);
	if (size >= count)
		size = count - 1;

	if (copy_to_user(ubuf, kbuf, size)) {
		vfree(kbuf);
		return -EFAULT;
	}

	vfree(kbuf);

	*ppos += size;

	return size;
}

static const struct file_operations iris_dbg_dislay_info_fops = {
	.open = simple_open,
	.read = iris_dbg_display_info_show,
};

extern void iris_set_dsi_cmd_log(uint32_t);
static ssize_t _iris_dsi_cmd_log_write(
		struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	uint32_t val = 0;

	if (count > SZ_32)
		return -EFAULT;

	if (kstrtouint_from_user(buff, count, 0, &val))
		return -EFAULT;

	iris_set_dsi_cmd_log(val);

	return count;
}

extern uint32_t iris_get_dsi_cmd_log(void);
static ssize_t _iris_dsi_cmd_log_read(
		struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	uint32_t val = 0;
	uint32_t len = 0;
	char bf[SZ_32];

	if (*ppos)
		return 0;

	val = iris_get_dsi_cmd_log();
	len += scnprintf(bf, SZ_32, "%u\n", val);

	len = min_t(size_t, count, len);
	if (copy_to_user(buff, bf, len))
		return -EFAULT;

	*ppos += len;

	return len;
}

static const struct file_operations iris_dsi_cmd_log_fops = {
	.open = simple_open,
	.write = _iris_dsi_cmd_log_write,
	.read = _iris_dsi_cmd_log_read,
};

static ssize_t _iris_tm_sw_log_write(
		struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	uint32_t val = 0;

	if (count > SZ_32)
		return -EFAULT;

	if (kstrtouint_from_user(buff, count, 0, &val))
		return -EFAULT;

	iris_set_tm_sw_loglevel(val);

	return count;
}

static ssize_t _iris_tm_sw_log_read(
		struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	uint32_t val = 0;
	uint32_t len = 0;
	char bf[SZ_32];

	if (*ppos)
		return 0;

	val = iris_get_tm_sw_loglevel();
	len += scnprintf(bf, SZ_32, "%u\n", val);

	len = min_t(size_t, count, len);
	if (copy_to_user(buff, bf, len))
		return -EFAULT;

	*ppos += len;

	return len;
}

static const struct file_operations iris_tm_sw_fops = {
	.open = simple_open,
	.write = _iris_tm_sw_log_write,
	.read = _iris_tm_sw_log_read,
};


static ssize_t _iris_cmd_list_write(
		struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	uint32_t val = 0;

	if (count > SZ_32)
		return -EFAULT;

	if (kstrtouint_from_user(buff, count, 0, &val))
		return -EFAULT;

	iris_dump_cmdlist(val);

	return count;
}

static const struct file_operations iris_cmd_list_fops = {
	.open = simple_open,
	.write = _iris_cmd_list_write,
};

int iris_dbgfs_status_init(struct dsi_display *display)
{
	struct iris_cfg *pcfg = NULL;
	int retval;
	pcfg = iris_get_cfg();

	if (pcfg->iris_kobj) {
		retval = sysfs_create_group(pcfg->iris_kobj, &iris_dbg_attr_group);
		if (retval) {
			kobject_put(pcfg->iris_kobj);
			IRIS_LOGE("sysfs create display_mode node fail");
		} else{
			IRIS_LOGI("sysfs create display_mode node successfully");
		}
	}

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir(IRIS_DBG_TOP_DIR, NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("create dir for iris failed, error %ld",
					PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}

	if (debugfs_create_file(IRIS_DBG_FUNCSTATUS_FILE, 0644,
				pcfg->dbg_root, display,
				&iris_dbg_fstatus_fops) == NULL)
		IRIS_LOGE("create file func_status failed");

	if (debugfs_create_file("display_info", 0644,
				pcfg->dbg_root, display,
				&iris_dbg_dislay_info_fops) == NULL)
		IRIS_LOGE("create file display_info failed");

	if (debugfs_create_file("display_mode", 0644,
				pcfg->dbg_root, display,
				&iris_dbg_dislay_mode_fops) == NULL)
		IRIS_LOGE("create file display_mode failed");

	if (debugfs_create_file("dsi_cmd_log", 0644,
				pcfg->dbg_root, display,
				&iris_dsi_cmd_log_fops) == NULL)
		IRIS_LOGE("create file dsi_cmd_log failed");

	if (debugfs_create_file("tm_sw_loglevel", 0644,
				pcfg->dbg_root, display,
				&iris_tm_sw_fops) == NULL)
		IRIS_LOGE("create file tm_sw_loglevel failed");

	if (debugfs_create_file("cmd_list_status", 0644,
				pcfg->dbg_root, display,
				&iris_cmd_list_fops) == NULL)
		IRIS_LOGE("create file cmd_list_status failed");


	return 0;
}
