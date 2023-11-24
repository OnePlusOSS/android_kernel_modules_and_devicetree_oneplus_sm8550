// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include <linux/kobject.h>
#include "dsi_iris_api.h"
#include "dsi_iris_loop_back.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_log.h"
#include "dsi_iris_gpio.h"
#include "dsi_iris_lp.h"
#include "sde_connector.h"

void iris_loop_back_reset(void)
{
	iris_send_one_wired_cmd(IRIS_POWER_DOWN_SYS);
	usleep_range(3500, 3501);
	iris_send_one_wired_cmd(IRIS_POWER_UP_SYS);
	usleep_range(3500, 3501);
}

void iris_set_esd_status(bool enable)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_display *display = pcfg->display;

	if (!display)
		return;

	if (!pcfg->panel || !display->drm_conn)
		return;

	if (!enable) {
		if (display->panel->esd_config.esd_enabled) {
			sde_connector_schedule_status_work(display->drm_conn, false);
			display->panel->esd_config.esd_enabled = false;
			IRIS_LOGD("disable esd work");
		}
	} else {
		if (!display->panel->esd_config.esd_enabled) {
			sde_connector_schedule_status_work(display->drm_conn, true);
			display->panel->esd_config.esd_enabled = true;
			IRIS_LOGD("enabled esd work");
		}
	}
}

int iris_loop_back_validate(void)
{
	int ret = 0;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (pcfg->iris_chip_type == CHIP_IRIS7P)
		ret = iris_loop_back_validate_i7p();
	else if (pcfg->iris_chip_type == CHIP_IRIS7)
		ret = iris_loop_back_validate_i7();

	return ret;
}

static ssize_t _iris_dbg_loop_back_read(struct file *file,
		char __user *buff, size_t count, loff_t *ppos)
{
	int ret = -1;
	int tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	ret = iris_loop_back_validate();
	IRIS_LOGI("%s(%d) ret: %d", __func__, __LINE__, ret);

	tot = scnprintf(bp, sizeof(bp), "0x%x\n", ret);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_loop_back_fops = {
	.open = simple_open,
	.read = _iris_dbg_loop_back_read,
};


int iris_mipi_rx0_validate(void)
{
	int ret = 0;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (pcfg->iris_chip_type == CHIP_IRIS7P)
		ret = iris_mipi_rx0_validate_i7p();
	else if (pcfg->iris_chip_type == CHIP_IRIS7)
		ret = iris_mipi_rx0_validate_i7();

	return ret;
}

static ssize_t _iris_mipi_rx_validate(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int ret = 0;
	int tot = 0;
	char bp[512];
	struct iris_cfg *pcfg = iris_get_cfg();

	if (*ppos)
		return 0;

	mutex_lock(&pcfg->panel->panel_lock);
	ret = iris_mipi_rx0_validate();
	mutex_unlock(&pcfg->panel->panel_lock);

	tot = scnprintf(bp, sizeof(bp), "0x%02x\n", ret);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;
	*ppos += tot;

	return tot;

}

static const struct file_operations iris_mipi_rx_validate_fops = {
	.open = simple_open,
	.write = NULL,
	.read = _iris_mipi_rx_validate,
};

static ssize_t iris_loop_back_show(struct kobject *obj, struct kobj_attribute *attr, char *buf)
{
	int ret;

	ret = iris_loop_back_validate();
	IRIS_LOGI("%s(%d) ret: %d", __func__, __LINE__, ret);

	return snprintf(buf, PAGE_SIZE, "0x%x\n", ret);
}

#define IRIS_ATTR(_name, _mode, _show, _store) \
	struct kobj_attribute iris_attr_##_name = __ATTR(_name, _mode, _show, _store)
static IRIS_ATTR(iris_loop_back, S_IRUGO, iris_loop_back_show, NULL);

static struct attribute *iris_dev_attrs[] = {
	&iris_attr_iris_loop_back.attr,
	NULL
};

static const struct attribute_group iris_attr_group = {
	.attrs = iris_dev_attrs,
};

static ssize_t _iris_loop_back_flag_write(
		struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	uint32_t val = 0;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (count > SZ_32)
		return -EFAULT;

	if (kstrtouint_from_user(buff, count, 0, &val))
		return -EFAULT;

	if (pcfg->iris_chip_type == CHIP_IRIS7P)
		iris_set_loopback_flag_i7p(val);
	else if (pcfg->iris_chip_type == CHIP_IRIS7)
		iris_set_loopback_flag_i7(val);

	return count;
}

static ssize_t _iris_loop_back_flag_read(
		struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	uint32_t val = 0;
	uint32_t len = 0;
	char bf[SZ_32];
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (*ppos)
		return 0;

	if (pcfg->iris_chip_type == CHIP_IRIS7P)
		val = iris_get_loopback_flag_i7p();
	else if (pcfg->iris_chip_type == CHIP_IRIS7)
		val = iris_get_loopback_flag_i7();

	len += scnprintf(bf, SZ_32, "%u\n", val);

	len = min_t(size_t, count, len);
	if (copy_to_user(buff, bf, len))
		return -EFAULT;

	*ppos += len;

	return len;
}

static const struct file_operations iris_loop_back_flag_fops = {
	.open = simple_open,
	.write = _iris_loop_back_flag_write,
	.read = _iris_loop_back_flag_read,
};

int iris_dbgfs_loop_back_init(struct dsi_display *display)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int retval;

	if (pcfg->iris_kobj == NULL) {
		pcfg->iris_kobj = kobject_create_and_add(IRIS_SYSFS_TOP_DIR, kernel_kobj);
	}
	if (pcfg->iris_kobj) {
		/* Create the files associated with this kobject */
		retval = sysfs_create_group(pcfg->iris_kobj, &iris_attr_group);
		if (retval) {
			kobject_put(pcfg->iris_kobj);
			IRIS_LOGW("sysfs create group iris_loop_back_show error");
		}
	} else {
		IRIS_LOGW("sysfs create iris dir error");
	}

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir("iris", NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("debugfs_create_dir for iris_debug failed, error %ld",
					PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}

	if (debugfs_create_file("iris_loop_back_flag",	0644, pcfg->dbg_root, display,
				&iris_loop_back_flag_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("iris_loop_back",	0644, pcfg->dbg_root, display,
				&iris_loop_back_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("iris_mipi_rx_validate",	0644, pcfg->dbg_root, display,
				&iris_mipi_rx_validate_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}

	return 0;
}
