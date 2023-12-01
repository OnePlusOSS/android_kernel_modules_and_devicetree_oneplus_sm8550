// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
/***************************************************************
** OPLUS_SYSTEM_QCOM_PMICWD
** File : qcom_pmicwd.c
** Description : qcom pmic watchdog driver
** Version : 1.0
******************************************************************/

/*
 * depend on msm export symbol: sys_reset_dev
 * sys_reset_dev: msm-5.4/drivers/input/misc/qpnp-power-on.c
*/

#include <linux/kthread.h>
#include <linux/rtc.h>
#include <linux/proc_fs.h>
#include <linux/mutex.h>
#include <uapi/linux/sched/types.h>
#include <linux/suspend.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include "soc/oplus/system/oplus_project.h"
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/version.h>

#include "qcom_pmicwd.h"

#define QPNP_PON_WD_RST_S1_TIMER(pon)		((pon)->base + 0x54)
#define QPNP_PON_WD_RST_S2_TIMER(pon)		((pon)->base + 0x55)
#define QPNP_PON_WD_RST_S2_CTL(pon)			((pon)->base + 0x56)
#define QPNP_PON_WD_RST_S2_CTL2(pon)		((pon)->base + 0x57)
#define QPNP_PON_WD_RESET_PET(pon)  		((pon)->base + 0x58)
#define QPNP_PON_RT_STS(pon)				((pon)->base + 0x10)

#define QPNP_PON_GEN3_INT_SET_TYPE(pon)       ((pon)->base + 0x11)
#define QPNP_PON_GEN3_INT_POLARITY_HIGH(pon)       ((pon)->base + 0x12)
#define QPNP_PON_GEN3_INT_POLARITY_LOW(pon)       ((pon)->base + 0x13)
#define QPNP_PON_GEN3_INT_LATCHED_CLR(pon)       ((pon)->base + 0x14)
#define QPNP_PON_GEN3_INT_EN_SET(pon)       ((pon)->base + 0x15)
#define QPNP_PON_GEN3_INT_EN_CLR(pon)       ((pon)->base + 0x16)
#define QPNP_PON_GEN3_WD_RST_S1_TIMER(pon)       ((pon)->base + 0x4c)
#define QPNP_PON_GEN3_WD_RST_S2_TIMER(pon)       ((pon)->base + 0x4d)
#define QPNP_PON_GEN3_WD_RST_S2_CTL(pon)         ((pon)->base + 0x4e)
#define QPNP_PON_GEN3_WD_RST_S2_CTL2(pon)        ((pon)->base + 0x4f)
#define QPNP_PON_GEN3_WD_RESET_PET(pon)          ((pon)->base + 0x50)
#define PMIC_WD_INT_BIT_MASK		BIT(3)

#define QPNP_PON_S2_CNTL_TYPE_MASK		(0xF)
#define QPNP_PON_WD_S2_TIMER_MASK		(0x7F)
#define QPNP_PON_WD_S1_TIMER_MASK		(0x7F)
#define QPNP_PON_WD_RESET_PET_MASK		BIT(0)

#define PMIC_WD_DEFAULT_TIMEOUT 254
#define PMIC_WD_DEFAULT_ENABLE 1

extern struct qpnp_pon *sys_reset_dev;
struct pmicwd_desc *pmicwd;

#define PON_GEN3_PBS                            0x08
#define PON_GEN3_HLOS                           0x09
#define QPNP_PON_WD_EN                          BIT(7)

static int raise_wdt_issue(void) {
	pr_info("%s wdt issue begin!\n", __func__);
	preempt_disable();
	local_irq_disable();
	while(1);
}


static bool is_pon_gen3(struct qpnp_pon *pon)
{
        return pon->subtype == PON_GEN3_PBS ||
                pon->subtype == PON_GEN3_HLOS;
}

static int oplus_qpnp_pon_wd_config(bool enable)
{
        if (!sys_reset_dev)
                return -EPROBE_DEFER;

	if (is_pon_gen3(sys_reset_dev)) {
		return dup_qpnp_pon_masked_write(sys_reset_dev,
							QPNP_PON_GEN3_WD_RST_S2_CTL2(sys_reset_dev),
							QPNP_PON_WD_EN, enable ? QPNP_PON_WD_EN : 0);
	} else {
		return dup_qpnp_pon_masked_write(sys_reset_dev,
							QPNP_PON_WD_RST_S2_CTL2(sys_reset_dev),
							QPNP_PON_WD_EN, enable ? QPNP_PON_WD_EN : 0);
	}
}

static int oplus_pon_pbs_int_cfg(void) {
	int rc = 0;

	if (!sys_reset_dev)
		return -EPROBE_DEFER;

	if (is_pon_gen3(sys_reset_dev)) {
		rc = dup_qpnp_pon_masked_write(pmicwd->pon, QPNP_PON_GEN3_INT_SET_TYPE(pmicwd->pon),
				PMIC_WD_INT_BIT_MASK, 0x00);
		if (rc) {
			PWD_ERR("Unable to write to addr=%x, rc(%d)\n",
					QPNP_PON_GEN3_INT_SET_TYPE(pmicwd->pon), rc);
		}

		rc = dup_qpnp_pon_masked_write(pmicwd->pon, QPNP_PON_GEN3_INT_POLARITY_HIGH(pmicwd->pon),
				PMIC_WD_INT_BIT_MASK, PMIC_WD_INT_BIT_MASK);
		if (rc) {
			PWD_ERR("Unable to write to addr=%x, rc(%d)\n",
					QPNP_PON_GEN3_INT_POLARITY_HIGH(pmicwd->pon), rc);
		}

		rc = dup_qpnp_pon_masked_write(pmicwd->pon, QPNP_PON_GEN3_INT_POLARITY_LOW(pmicwd->pon),
				PMIC_WD_INT_BIT_MASK, 0x00);
		if (rc) {
			PWD_ERR("Unable to write to addr=%x, rc(%d)\n",
					QPNP_PON_GEN3_INT_POLARITY_LOW(pmicwd->pon), rc);
		}

		rc = dup_qpnp_pon_masked_write(pmicwd->pon, QPNP_PON_GEN3_INT_LATCHED_CLR(pmicwd->pon),
				PMIC_WD_INT_BIT_MASK, PMIC_WD_INT_BIT_MASK);
		if (rc) {
			PWD_ERR("Unable to write to addr=%x, rc(%d)\n",
					QPNP_PON_GEN3_INT_LATCHED_CLR(pmicwd->pon), rc);
		}

		rc = dup_qpnp_pon_masked_write(pmicwd->pon, QPNP_PON_GEN3_INT_EN_SET(pmicwd->pon),
				PMIC_WD_INT_BIT_MASK, PMIC_WD_INT_BIT_MASK);
		if (rc) {
			PWD_ERR("Unable to write to addr=%x, rc(%d)\n",
					QPNP_PON_GEN3_INT_EN_SET(pmicwd->pon), rc);
		}

		rc = dup_qpnp_pon_masked_write(pmicwd->pon, QPNP_PON_GEN3_INT_EN_CLR(pmicwd->pon),
				PMIC_WD_INT_BIT_MASK, PMIC_WD_INT_BIT_MASK);
		if (rc) {
			PWD_ERR("Unable to write to addr=%x, rc(%d)\n",
					QPNP_PON_GEN3_INT_EN_CLR(pmicwd->pon), rc);
		}
	} else {
		//gen2 pmic do nothing
	}

	return rc;
}

static int qpnp_pon_wd_timer(unsigned char timer, enum pon_power_off_type reset_type)
{
	int rc = 0;
	u8 s1_timer,s2_timer;

	ASSERT(pmicwd);

	if(timer > 127)
	{
		s2_timer = 127;
		if(timer - 127 > 127)
			s1_timer = 127;
		else
			s1_timer = timer - 127;
	}else{
		s2_timer = timer&0xff;
		s1_timer = 0;
	}
	if (is_pon_gen3(sys_reset_dev)) {
		rc = dup_qpnp_pon_masked_write(pmicwd->pon, QPNP_PON_GEN3_WD_RST_S2_TIMER(pmicwd->pon),
				QPNP_PON_WD_S2_TIMER_MASK, s2_timer);
		if (rc) {
			PWD_ERR("Unable to write to addr=%x, rc(%d)\n",
					QPNP_PON_GEN3_WD_RST_S2_TIMER(pmicwd->pon), rc);
		}

		rc = dup_qpnp_pon_masked_write(pmicwd->pon, QPNP_PON_GEN3_WD_RST_S1_TIMER(pmicwd->pon),
				QPNP_PON_WD_S1_TIMER_MASK, s1_timer);
		if (rc) {
			PWD_ERR("Unable to write to addr=%x, rc(%d)\n",
					QPNP_PON_GEN3_WD_RST_S1_TIMER(pmicwd->pon), rc);
		}

		rc = dup_qpnp_pon_masked_write(pmicwd->pon, QPNP_PON_GEN3_WD_RST_S2_CTL(pmicwd->pon),
				QPNP_PON_S2_CNTL_TYPE_MASK, reset_type);
		if (rc) {
			PWD_ERR("Unable to write to addr=%x, rc(%d)\n",
					QPNP_PON_GEN3_WD_RST_S2_CTL(pmicwd->pon), rc);
		}
	} else {
		rc = dup_qpnp_pon_masked_write(pmicwd->pon, QPNP_PON_WD_RST_S2_TIMER(pmicwd->pon),
				QPNP_PON_WD_S2_TIMER_MASK, s2_timer);
		if (rc) {
			PWD_ERR("Unable to write to addr=%x, rc(%d)\n",
					QPNP_PON_WD_RST_S2_TIMER(pmicwd->pon), rc);
		}

		rc = dup_qpnp_pon_masked_write(pmicwd->pon, QPNP_PON_WD_RST_S1_TIMER(pmicwd->pon),
				QPNP_PON_WD_S1_TIMER_MASK, s1_timer);
		if (rc) {
			PWD_ERR("Unable to write to addr=%x, rc(%d)\n",
					QPNP_PON_WD_RST_S1_TIMER(pmicwd->pon), rc);
		}

		rc = dup_qpnp_pon_masked_write(pmicwd->pon, QPNP_PON_WD_RST_S2_CTL(pmicwd->pon),
				QPNP_PON_S2_CNTL_TYPE_MASK, reset_type);
		if (rc) {
			PWD_ERR("Unable to write to addr=%x, rc(%d)\n",
					QPNP_PON_WD_RST_S2_CTL(pmicwd->pon), rc);
		}
	}

	return rc;
}

int qpnp_pon_wd_pet(struct qpnp_pon *pon)
{
	int rc = 0;

	ASSERT(pon);

	if (is_pon_gen3(sys_reset_dev)) {
		rc = dup_qpnp_pon_masked_write(pon, QPNP_PON_GEN3_WD_RESET_PET(pon),
				QPNP_PON_WD_RESET_PET_MASK, 1);
		if (rc) {
			PWD_ERR("Unable to write to addr=%x, rc(%d)\n",
					QPNP_PON_GEN3_WD_RESET_PET(pon), rc);
		}
	} else {
		rc = dup_qpnp_pon_masked_write(pon, QPNP_PON_WD_RESET_PET(pon),
				QPNP_PON_WD_RESET_PET_MASK, 1);
		if (rc) {
			PWD_ERR("Unable to write to addr=%x, rc(%d)\n",
					QPNP_PON_WD_RESET_PET(pon), rc);
		}
	}

	return rc;
}

static int pmicwd_kthread(void *arg)
{
	struct sched_param param = {.sched_priority = MAX_RT_PRIO-1};

	sched_setscheduler(current, SCHED_FIFO, &param);

	PWD_INFO("pmicwd_kthread PET wd\n");
	qpnp_pon_wd_pet(pmicwd->pon);

	while (!kthread_should_stop()) {
		schedule_timeout_interruptible(msecs_to_jiffies((((pmicwd->pmicwd_state >> 8)&0xff)*1000)/2));
		PWD_INFO("pmicwd_kthread PET wd suspend state %d\n", pmicwd->suspend_state);
		qpnp_pon_wd_pet(pmicwd->pon);

		/* For detect the suspend resume block issue
		 * add at least 128 seconds ~ 256 seconds during resume suspend
		 */
		if ((pmicwd->suspend_state & 0x0F) >= 1) {
			panic("suspend resume state %d\n", pmicwd->suspend_state);
		} else if (pmicwd->suspend_state & 0xF0) {
			pmicwd->suspend_state++;
		}

	}
	oplus_qpnp_pon_wd_config(0);
	return 0;
}
static ssize_t pmicwd_proc_read(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	unsigned int val;
	char page[128] = {0};
	int len = 0;

	mutex_lock(&pmicwd->wd_task_mutex);
	if (is_pon_gen3(sys_reset_dev)) {
		regmap_read(pmicwd->pon->regmap, QPNP_PON_GEN3_WD_RST_S2_CTL2(pmicwd->pon), &val);
	} else {
		regmap_read(pmicwd->pon->regmap, QPNP_PON_WD_RST_S2_CTL2(pmicwd->pon), &val);
	}
	PWD_INFO("pmicwd_proc_read:%x wd=%x\n", pmicwd->pmicwd_state, val);
	//|reserver|rst type|timeout|enable|
	len = snprintf(&page[len],128 - len,"enable = %d timeout = %d rstype = %d\n",
					pmicwd->pmicwd_state & 0xff,
					(pmicwd->pmicwd_state >> 8) & 0xff,
					(pmicwd->pmicwd_state >> 16) & 0xff);
	mutex_unlock(&pmicwd->wd_task_mutex);

	if(len > *off)
	   len -= *off;
	else
	   len = 0;

	if(copy_to_user(buf,page,(len < count ? len : count))){
	   return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);

}

#define BUFF_SIZE 64
#define MAX_SYMBOL_LEN 64

static char symbol[MAX_SYMBOL_LEN]={"DISABLE"};

static ssize_t pmicwd_proc_write(struct file *file, const char __user *buf,
		size_t count,loff_t *off)
{
	int tmp_rstypt = 0;
	int tmp_timeout = 0;
	int tmp_enable = 0;
	int ret = 0;
	char buffer[64] = {0};
	unsigned int new_state;
	int max_len[] = {BUFF_SIZE, BUFF_SIZE, BUFF_SIZE};
	int part;
	char delim[] = {' ', ' ', '\n'};
	char *start, *end;

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count)) {
		PWD_ERR("%s: read proc input error.\n", __func__);
		return count;
	}

	buffer[count] = '\0';
	/*simulate abnormal PMIC into dump */
	snprintf(symbol, sizeof(symbol), "%s", buffer);
	symbol[count-1] = 0;
	if(!strcmp(symbol, "ENABLE_SUSPEND")||!strcmp(symbol, "ENABLE_RESUME")) {
		return count;
	}
	if(!strcmp(symbol, "ENABLE_WDT")) {
		raise_wdt_issue();
	}

	/* validate the length of each of the 3 parts */
	start = buffer;
	for (part = 0; part < 3; part++) {
		end = strchr(start, delim[part]);
		if (end == NULL || (end - start) > max_len[part]) {
			return count;
		}
		start = end + 1;
	}

	ret = sscanf(buffer, "%d %d %d", &tmp_enable, &tmp_timeout, &tmp_rstypt);
	if(ret <= 0){
		PWD_ERR("%s: input error\n", __func__);
		return count;
	}
	if(tmp_timeout < 60 || tmp_timeout > 255){
		tmp_timeout = PMIC_WD_DEFAULT_TIMEOUT;
	}
	if(tmp_rstypt >= PON_POWER_OFF_MAX_TYPE || tmp_rstypt <= PON_POWER_OFF_RESERVED){
		if (get_eng_version() == AGING || get_eng_version() == HIGH_TEMP_AGING || get_eng_version() == FACTORY) {
			tmp_rstypt = PON_POWER_OFF_WARM_RESET;
		} else {
			tmp_rstypt = PON_POWER_OFF_HARD_RESET;
		}
	}
	new_state = (tmp_enable & 0xff)|((tmp_timeout & 0xff) << 8)|((tmp_rstypt & 0xff)<< 16);
	PWD_ERR("pmicwd_proc_write:old:%x new:%x\n", pmicwd->pmicwd_state, new_state);

	if(new_state == pmicwd->pmicwd_state) {
		return count;
	}

	mutex_lock(&pmicwd->wd_task_mutex);
	if(pmicwd->wd_task) {
		oplus_qpnp_pon_wd_config(0);
		pmicwd->pmicwd_state &= ~0xff;
		kthread_stop(pmicwd->wd_task);
		pmicwd->wd_task = NULL;
	}

	qpnp_pon_wd_timer(tmp_timeout,tmp_rstypt);
	pmicwd->pmicwd_state = new_state;
	if(tmp_enable){
		pmicwd->wd_task = kthread_create(pmicwd_kthread, pmicwd->pon, "pmicwd");
		if(pmicwd->wd_task) {
			oplus_qpnp_pon_wd_config(1);
			wake_up_process(pmicwd->wd_task);
		}else{
			oplus_qpnp_pon_wd_config(0);
			pmicwd->pmicwd_state &= ~0xff;
		}
	}
	qpnp_pon_wd_pet(pmicwd->pon);
	mutex_unlock(&pmicwd->wd_task_mutex);
	return count;
}

void set_pmicWd_state(int enable)
{
	unsigned int new_state;

	ASSERT(pmicwd);

	new_state = (pmicwd->pmicwd_state & ~0x1) | (enable & 0x1);

	PWD_INFO("set_pmicWd_state:old:%x new:%x\n", pmicwd->pmicwd_state, new_state);
	if(new_state == pmicwd->pmicwd_state) {
		return;
	}

	mutex_lock(&pmicwd->wd_task_mutex);
	if(pmicwd->wd_task) {
		oplus_qpnp_pon_wd_config(0);
		pmicwd->pmicwd_state &= ~0xff;
		kthread_stop(pmicwd->wd_task);
		pmicwd->wd_task = NULL;
	}
	pmicwd->pmicwd_state = new_state;

	if(enable){
		pmicwd->wd_task = kthread_create(pmicwd_kthread, NULL, "pmicwd");
		if(pmicwd->wd_task) {
			oplus_qpnp_pon_wd_config(1);
			wake_up_process(pmicwd->wd_task);
		}else{
			oplus_qpnp_pon_wd_config(0);
			pmicwd->pmicwd_state &= ~0xff;
		}
	}
	qpnp_pon_wd_pet(pmicwd->pon);
	mutex_unlock(&pmicwd->wd_task_mutex);
}
EXPORT_SYMBOL(set_pmicWd_state);

/*
 * This function is register as callback function to get notifications
 * from the PM module on the system suspend state.
 */
static int pmicWd_pm_notifier(struct notifier_block *nb,
				  unsigned long event, void *unused)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		pmicwd->suspend_state = 0x80;
		if(!strcmp(symbol, "ENABLE_SUSPEND")) {
			pr_info("%s the phone will enter into dump\n", __func__);
			while(1);
		}
		PWD_INFO("pmicwd start suspend\n");
		break;

	case PM_POST_SUSPEND:
		pmicwd->suspend_state = 0;
		PWD_INFO("pmicwd finish resume\n");
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block pmicWd_pm_nb = {
	.notifier_call = pmicWd_pm_notifier,
	.priority = INT_MAX,
};

static struct proc_ops pmicwd_proc_pops = {
	.proc_read = pmicwd_proc_read,
	.proc_write = pmicwd_proc_write,
	.proc_lseek = default_llseek,
};

/* pmicwd debug start */
static bool pmicwd_test_flag = false;

static struct proc_ops pmicwd_config_pops = {
	.proc_open = simple_open,
	.proc_read = pmicwd_proc_read,
	.proc_write = pmicwd_proc_write,
	.proc_lseek = default_llseek,
};

static ssize_t pmicwd_test_read(struct file *file, char __user *buf,
		size_t count, loff_t *off)
{
	int len;
	char rbuffer[128] = {0};

	PWD_INFO("%s: pmicwd test flag: %d\n", __func__, pmicwd_test_flag);
	len = snprintf(rbuffer, 128, "%s: pmicwd test flag: %d\n", __func__, pmicwd_test_flag);

	if(len > *off) {
		len -= *off;
	} else {
		len = 0;
	}

	if (copy_to_user(buf, rbuffer, (len < count ? len : count))) {
		return -EFAULT;
	}
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

static ssize_t pmicwd_test_write(struct file *file, const char __user *buf,
		size_t count, loff_t *off)
{
	int enable = 0;
	char buffer[2] = {0};

	if (copy_from_user(buffer, buf, 1)) {
		PWD_ERR("%s: read proc input error.\n", __func__);
		return count;
	}
	enable = buffer[0] - '0';

	mutex_lock(&pmicwd->wd_task_mutex);
	if (enable) {
		if(pmicwd_test_flag){
			PWD_INFO("%s: pmicwd test already started!\n", __func__);
			mutex_unlock(&pmicwd->wd_task_mutex);
			return count;
		}
		/*stop wd_task, and keep wd enbled, cause wd timeout!*/
		if (pmicwd->wd_task) {
			kthread_stop(pmicwd->wd_task);
			oplus_qpnp_pon_wd_config(1);  // wd will be disabled in wd_task stop flow, so re-enabled 
			pmicwd->wd_task = NULL;
			PWD_INFO("%s: pmicwd test start: after %d seconds reboot or enter RAMDUMP! \n",
					__func__, (pmicwd->pmicwd_state >> 8)&0xFF);
		} else {
			PWD_INFO("%s: pmicwd test start: wd task not run, ignore! \n", __func__);
		}
		pmicwd_test_flag = true;
	} else {
		PWD_INFO("%s: pmicwd test exit!\n", __func__);
		if(!pmicwd->wd_task) {
			pmicwd->wd_task = kthread_create(pmicwd_kthread, pmicwd->pon, "pmicwd");
			if(pmicwd->wd_task) {
				oplus_qpnp_pon_wd_config(1);
				wake_up_process(pmicwd->wd_task);
			}
			qpnp_pon_wd_pet(pmicwd->pon);
		}
		pmicwd_test_flag = false;
	}
	mutex_unlock(&pmicwd->wd_task_mutex);

	return count;
}

static struct proc_ops pmicwd_test_pops = {
	.proc_read = pmicwd_test_read,
	.proc_write = pmicwd_test_write,
	.proc_lseek = default_llseek,
};

static void pmicwd_debug_init(void)
{
	proc_create("pmicwd_config", 0666, NULL, &pmicwd_config_pops);
	proc_create("pmicwd_test", 0666, NULL, &pmicwd_test_pops);
}
/* pmicwd debug end */

void pmicwd_init(struct platform_device *pdev)
{
	u32 pon_rt_sts = 0;
	int rc;

	/* try to get dts info */
	pmicwd->wd_task = NULL;
	pmicwd->suspend_state = 0;
	pmicwd->pmicwd_state = of_property_read_bool(pdev->dev.of_node, "qcom,pmicwd");
	PWD_INFO("pon pmicwd_state = 0x%x\n", pmicwd->pmicwd_state);

	if(pmicwd->pmicwd_state) {
		oplus_pon_pbs_int_cfg();
		if (get_eng_version() == AGING || get_eng_version() == HIGH_TEMP_AGING || get_eng_version() == FACTORY) {
			pmicwd->pmicwd_state = PMIC_WD_DEFAULT_ENABLE | (PMIC_WD_DEFAULT_TIMEOUT << 8) |
				(PON_POWER_OFF_WARM_RESET << 16);
		} else {
			pmicwd->pmicwd_state = PMIC_WD_DEFAULT_ENABLE | (PMIC_WD_DEFAULT_TIMEOUT << 8) |
				(PON_POWER_OFF_HARD_RESET << 16);
		}
		proc_create("pmicWd", 0666, NULL, &pmicwd_proc_pops);
		mutex_init(&pmicwd->wd_task_mutex);
		#if PMIC_WD_DEFAULT_ENABLE
		pmicwd->wd_task = kthread_create(pmicwd_kthread, NULL, "pmicwd");
		if(pmicwd->wd_task) {
			if (get_eng_version() == AGING  || get_eng_version() == HIGH_TEMP_AGING || get_eng_version() == FACTORY) {
				qpnp_pon_wd_timer(PMIC_WD_DEFAULT_TIMEOUT, PON_POWER_OFF_WARM_RESET);
			} else {
				qpnp_pon_wd_timer(PMIC_WD_DEFAULT_TIMEOUT, PON_POWER_OFF_HARD_RESET);
			}
			oplus_qpnp_pon_wd_config(1);
			wake_up_process(pmicwd->wd_task);
		}else{
			pmicwd->pmicwd_state &= ~0xff;
		}
		#endif

		rc = register_pm_notifier(&pmicWd_pm_nb);
		if (rc) {
			PWD_ERR("%s: pmicWd power state notif error %d\n", __func__, rc);
		}
	}

	regmap_read(pmicwd->pon->regmap, QPNP_PON_RT_STS(pmicwd->pon), &pon_rt_sts);
	PWD_INFO("probe keycode = 116, key_st = 0x%x\n", pon_rt_sts);

	pmicwd_debug_init();

	return;
}

static int  setalarm(unsigned long time,bool enable)
{
	static struct rtc_device *rtc;
	static struct rtc_wkalrm alm;
	static struct rtc_wkalrm org_alm;

	unsigned long now;
	int rc = -1;
	static bool store_alm_success = false;

	if(!rtc){
		rtc = rtc_class_open("rtc0");
	}

	if(!rtc) {
		PWD_ERR("open rtc fail %d\n", rc);
		return rc;
	}

	if(enable){
		rc = rtc_read_alarm(rtc, &org_alm);
		if (rc < 0) {
			PWD_ERR("setalarm read alarm fail %d\n", rc);
			store_alm_success = false;
			return rc;
		}
		store_alm_success = true;
		rc = rtc_read_time(rtc, &alm.time);
		if (rc < 0) {
			PWD_ERR("setalarm read time fail %d\n", rc);
			return rc;
		}

		now = rtc_tm_to_time64(&alm.time);
		memset(&alm, 0, sizeof alm);
		rtc_time64_to_tm(now + time, &alm.time);
		alm.enabled = true;
		rc = rtc_set_alarm(rtc, &alm);
		if (rc < 0) {
			PWD_ERR("setalarm  set alarm fail %d\n", rc);
			return rc;
		}
	} else if (store_alm_success) {
		alm.enabled = false;
		rc = rtc_set_alarm(rtc, &alm);
		if (rc < 0) {
			PWD_ERR("setalarm  set alarm fail %d\n", rc);
			return rc;
		}
		/* consider setting timer and orginal timer. we store orginal timer at pon suspend,
		and reset rtc from store at pon resume, no matter which one is greater. bottom
		driver would judge write to RTC or not. */
		rc = rtc_set_alarm(rtc, &org_alm);
		if (rc < 0) {
			PWD_ERR("setalarm  set org alarm fail %d\n", rc);
			return rc;
		}
	} else {
		PWD_INFO("%s store_alm_success:%d\n", __func__, store_alm_success);
	}
	return 0;
}
static int pmicwd_suspend(struct device *dev)
{
	unsigned long time = 0;

	ASSERT(pmicwd);

	if(!(pmicwd->pmicwd_state & 0xff)) {
		PWD_ERR("pmicwd_suspend disable wd\n");
		return 0;
	}
	pmicwd->suspend_state = 0;
	time = (pmicwd->pmicwd_state >> 8)&0xff;
	PWD_INFO("pmicwd_suspend enter && enable pet alarm\n");
	qpnp_pon_wd_pet(pmicwd->pon);
	setalarm(time - 30,true);
	return 0;
}

static int pmicwd_resume(struct device *dev)
{
	ASSERT(pmicwd);

	if(!(pmicwd->pmicwd_state & 0xff)) {
		PWD_INFO("pmicwd_resume fail && keep pet alarm on\n");
		return 0;
	}

	pmicwd->suspend_state = 0x70;
	PWD_INFO("pmicwd_resume enter && disable pet alarm\n");
	//disable alarm
	setalarm(0,false);
	qpnp_pon_wd_pet(pmicwd->pon);

	if(!strcmp(symbol, "ENABLE_RESUME")) {
		pr_info("%s the phone will enter into dump\n\n", __func__);
		while(1);
	}

	return 0;
}

static int pmicwd_poweroff(struct device *dev)
{
	PWD_INFO("pmicwd_poweroff && close wd\n");
	ASSERT(pmicwd);
	qpnp_pon_wd_pet(pmicwd->pon);
	oplus_qpnp_pon_wd_config(0);
	return 0;
}

static const struct dev_pm_ops pmicwd_pm_ops = {
	.suspend = pmicwd_suspend,
	.resume = pmicwd_resume,
	.poweroff = pmicwd_poweroff,
};

static int pmicwd_probe(struct platform_device *pdev)
{
	PWD_ERR("pmicwd_probe enter\n");

	if (!sys_reset_dev) {
		PWD_ERR("sys_reset_dev is NULL !\n");
		return -EPROBE_DEFER;
	}

	pmicwd = (struct pmicwd_desc *)kzalloc(sizeof(struct pmicwd_desc), GFP_KERNEL);
	if (!pmicwd) {
		PWD_ERR("pmicwd init failed !\n");
		return -1;
	}
	pmicwd->pon = sys_reset_dev;

	kpdpwr_init();

	pmicwd_init(pdev);

	return 0;
}

static int pmicwd_remove(struct platform_device *pdev)
{
	if (pmicwd) {
		kfree(pmicwd);
		pmicwd = NULL;
	}

	return 0;
}

static const struct of_device_id pmicwd_match_table[] = {
	{ .compatible = "oplus,pmicwd_qpnp-power-on" },
	{}
};

static struct platform_driver pmicwd_driver = {
	.driver = {
		.pm = &pmicwd_pm_ops,
		.name = "oplus,pmicwd_qpnp-power-on",
		.of_match_table = pmicwd_match_table,
	},
	.probe = pmicwd_probe,
	.remove = pmicwd_remove,
};
module_platform_driver(pmicwd_driver);

MODULE_DESCRIPTION("OPLUS QPNP-Power-on driver");
MODULE_LICENSE("GPL v2");
