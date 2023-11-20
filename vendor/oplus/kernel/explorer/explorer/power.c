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
 * Basecode Created :        2021/02/09 Author: zhengchao@zeku.com
 *
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#ifdef OPLUS_EXPLORER_PLATFORM_QCOM
#include <linux/clk.h>
#else
#include "mtk_clkbuf_common.h"
#include "mtk_clkbuf_ctl.h"
#endif
#include "include/main.h"
#include "include/power.h"
#include "include/exception.h"
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#define SDU_REG_READ_WAIT_COUNT 10
#define PMU_STATE_WAIT_COUNT 500

/* sdu clock switch direction */
#define HCLK_TO_SDCLK 0
#define SDCLK_TO_HCLK 1

#define ACTION_NONE    0
#define ACTION_WAKEUP  1
#define ACTION_SLEEP   2

static int read_sdu_register(struct explorer_plat_data *epd, unsigned int reg, unsigned char *val) {
	u8 val1 = 0, val2 = 0;
	int ret1 = 0, ret2 = 0;

	ret1 = explorer_hal_sync_read_internal(epd, reg, &val1);
	ret2 = explorer_hal_sync_read_internal(epd, reg, &val2);
	if ((ret1 < 0 ) || (ret2 < 0) || (val1 != val2)) {
		PM_LOG_E("read reg:0x%x failed.\n", reg);
		return -1;
	}

	*val = val1;
	return 0;
}

static int write_sdu_register(struct explorer_plat_data *epd, unsigned int reg, unsigned char val) {
	int ret = 0;

	ret = explorer_hal_sync_write_internal(epd, reg, val);
	if (ret < 0 ) {
		PM_LOG_E("write reg:0x%x to 0x%x failed.\n", reg, val);
		return -1;
	}

	return 0;
}

static int write_top_register(struct explorer_plat_data *epd, unsigned int reg, unsigned int val) {
    return epd ? explorer_hal_sync_write(epd, reg, &val, 4) : -1;
}

static int switch_sdu_clk_internal(struct explorer_plat_data *epd, int switch_direction) {
	int ret = 0;
	int count = 1000;
	u8 val = 0;
	u8 check_val = 0;

	mutex_lock(&epd->comm_lock);
	if (switch_direction == HCLK_TO_SDCLK) {
		PM_LOG_I("hclk -> sdclk\n");
	} else {
		PM_LOG_I("sdclk -> hclk\n");
	}

	ret = explorer_hal_sync_read_internal_nolock(epd, 0x806, &val);
	if (ret < 0) {
		PM_LOG_E("read reg:0x806 failed.\n");
		goto out;
	}
	if (switch_direction == HCLK_TO_SDCLK) {
		val |= 0x80;
	} else {
		val &= 0x7F;
	}

	ret = explorer_hal_sync_write_internal_nolock(epd, 0x806, val);
	if (ret < 0) {
		PM_LOG_E("write reg:0x806 failed.\n");
		goto out;
	}

	if (switch_direction == HCLK_TO_SDCLK) {
		check_val = 0x2;
	} else {
		check_val = 0x1;
	}

	do {
		ret = explorer_hal_sync_read_internal_nolock(epd, 0x81F, &val);
		if (ret < 0) {
			PM_LOG_E("read reg:0x81F failed.\n");
			goto out;
		}

		if (count-- < 0) {
			PM_LOG_E("TIMEOUT\n");
			break;
		}
		msleep(1);
	} while (!(val & check_val));

	if (switch_direction == HCLK_TO_SDCLK) {
		epd->is_sdu_clk_switched = true;
	} else {
		epd->is_sdu_clk_switched = false;
	}
out:
	mutex_unlock(&epd->comm_lock);
	return ret;
}

static int check_sleep_int(struct explorer_plat_data *epd) {
	unsigned int reg = 0x816;
	unsigned char val = 0;
	int count = SDU_REG_READ_WAIT_COUNT;
	bool success_read = false;

	do {
		int ret = read_sdu_register(epd, reg, &val);
		if (ret >= 0) {
			success_read = true;
			break;
		};
	} while (count-- > 0);

	if (success_read) {
		if (val & (0x1 << 1))
			return true;
		else
			return false;
	} else {
		PM_LOG_E("read error.\n");
		return false;
	}
}

static int check_wakeup_int(struct explorer_plat_data *epd) {
	unsigned int reg = 0x817;
	unsigned char val = 0;
	int count = SDU_REG_READ_WAIT_COUNT;
	bool success_read = false;

	do {
		int ret = read_sdu_register(epd, reg, &val);
		if (ret >= 0) {
			success_read = true;
			break;
		}
	} while (count-- > 0);

	if (success_read) {
		if (val & (0x1 << 4))
			return true;
		else
			return false;
	} else {
		PM_LOG_E("read error.\n");
		return false;
	}
}

int read_sdio_pwr_status(struct explorer_plat_data *epd) {
	unsigned int reg = 0x81a;
	unsigned char val = 0;
	int count = SDU_REG_READ_WAIT_COUNT;
	int success_read = false;
	int status = 0;

	do {
		int ret = read_sdu_register(epd, reg, &val);
		if (ret >= 0) {
			success_read = true;
			break;
		};
	} while (count-- > 0);

	if (success_read) {
		status = val & 0xf;
	} else {
		PM_LOG_E("read error.\n");
		status = -1;
	}

	return status;
}

static int write_ap_allow_enter_standby(struct explorer_plat_data *epd, bool is_true) {
	unsigned int reg = 0x806;
	unsigned char val = 0;
	unsigned char old_val = 0, new_val = 0;
	int ret = 0;

	ret = read_sdu_register(epd, reg, &val);
	if (ret < 0) {
		PM_LOG_E("read sdu reg:0x%x error, ret=%d\n", reg, ret);
		return -1;
	}

	old_val = val;
	if (is_true) {
		val = val | (0x1 << 6);
	} else {
		val = val & (~(0x1 << 6));
	}
	ret = write_sdu_register(epd, reg, val);
	if (ret < 0) {
		PM_LOG_E("write sdu reg:0x%x to 0x%x error, ret=%d\n", reg, val, ret);
		return -1;
	}

	ret = read_sdu_register(epd, reg, &new_val);
	if (ret < 0) {
		PM_LOG_E("read sdu reg:0x%x error, ret=%d\n", reg, ret);
		return -1;
	}

	PM_LOG_I("set(%d):(0x%x) 0x%x -> 0x%x\n", is_true, reg, old_val, new_val);

	return 0;
}

static int write_ap_allow_exit_standby(struct explorer_plat_data *epd, bool is_true) {
	unsigned int reg = 0x803;
	unsigned char val = 0;
	unsigned char old_val = 0, new_val = 0;
	int ret = 0;

	ret = read_sdu_register(epd, reg, &val);
	if (ret < 0) {
		PM_LOG_E("read sdu reg:0x%x error, ret=%d\n", reg, ret);
		return -1;
	}
	old_val = val;
	if (is_true) {
		val = val | (0x1 << 7);
	} else {
		val = val & (~(0x1 << 7));
	}
	ret = write_sdu_register(epd, reg, val);
	if (ret < 0) {
		PM_LOG_E("write sdu reg:0x%x to 0x%x error, ret=%d\n", reg, val, ret);
		return -1;
	}
	ret = read_sdu_register(epd, reg, &new_val);
	if (ret < 0) {
		PM_LOG_E("read sdu reg:0x%x error, ret=%d\n", reg, ret);
		return -1;
	}

	PM_LOG_I("set(%d):(0x%x) 0x%x -> 0x%x\n", is_true, reg, old_val, new_val);
	return 0;
}

static int trigger_explorer_pmu_reserved0_irq(struct explorer_plat_data *epd) {
	int ret = write_top_register(epd, REG_PMU_INT_SET, PMU_INT_SET_RESERVED0_MASK);

	if (ret < 0) {
		PM_LOG_E("trigger explorer reserved0 int error, ret=%d.\n", ret);
		return -1;
	}

	PM_LOG_I("trigger reserved0_int in PMU\n");

	return 0;
}

static int wakeup_explorer_internal(struct explorer_plat_data *epd) {
	int err = 0;
	long timeout = 3 * HZ;

	/* Add skip when explorer power off */
	if (atomic_read(&epd->is_explorer_on) == 0) {
		PM_LOG_I("skip wakeup [power-off]");
		return 0;
	}

	reinit_completion(&epd->wake_completion);

	err = trigger_explorer_pmu_reserved0_irq(epd);
	if (err < 0) {
		PM_LOG_E("trigger explorer reserved0 int error, err=%d.\n", err);
		goto out;
	}
	err = write_ap_allow_exit_standby(epd, true);
	if (err < 0) {
		PM_LOG_E("write ap allow exit standby to true error, err=%d.\n", err);
		goto out;
	}
	timeout = wait_for_completion_interruptible_timeout(&epd->wake_completion, timeout);
	if (timeout <= 0) {
		err = !timeout ? -ETIMEDOUT : -ERESTARTSYS;
		PM_LOG_E("err:%d\n", err);
		goto out;
	}
	err = write_ap_allow_exit_standby(epd, false);
	if (err < 0) {
		PM_LOG_E("write ap allow exit standby to false error, err=%d.\n", err);
		goto out;
	}
	err = switch_sdu_clk_internal(epd, SDCLK_TO_HCLK);
	if (err < 0) {
		PM_LOG_E("error in switch sdu clk, err:%d\n", err);
		goto out;
	}

out:
	if (err >= 0) {
		PM_LOG_I("explorer wakeup success\n");
	} else {
		PM_LOG_E("explorer wakeup fail, err=%d\n", err);
		report_power_exception(epd,
				EXCEPTION_POWER_MAJOR_TYPE_SUSPEND_RESUME,
				EXCEPTION_POWER_SUB_TYPE_WAKEUP_FAIL,
				EXCEPTION_ACT_NONE);
	}

	return err;
}

int wakeup_explorer(struct explorer_plat_data *epd) {
	int err = 0;

	mutex_lock(&epd->power_sync_lock);
	err = wakeup_explorer_internal(epd);
	mutex_unlock(&epd->power_sync_lock);

	return err;
}

static int sleep_explorer_internal(struct explorer_plat_data *epd, bool force) {
	int err = 0;
	long timeout = 3 * HZ;

	/* Add skip when explorer power off */
	if (atomic_read(&epd->is_explorer_on) == 0) {
		PM_LOG_I("skip sleep [power-off]");
		return 0;
	}

	err = switch_sdu_clk_internal(epd, HCLK_TO_SDCLK);
	if (err < 0) {
		PM_LOG_E("err in switch sdu clk, err:%d\n", err);
		goto out;
	}

	/* After prepare done, send power state message. */
	reinit_completion(&epd->power_state_completion);
	err = explorer_write_power_data(epd,
			!force ? POWER_AP2CC_CMD_ENTER_STANDBY : POWER_AP2CC_CMD_ENTER_STANDBY_FORCE,
			0);
	if (err < 0) {
		PM_LOG_E("err:%d\n", err);
		goto out;
	}

	timeout = wait_for_completion_interruptible_timeout(&epd->power_state_completion, timeout);
	if (timeout <= 0) {
		err = !timeout ? -ETIMEDOUT : -ERESTARTSYS;
		PM_LOG_E("err:%d\n", err);
		goto out;
	}

	if (epd->completed_power_state != POWER_IOC_STATE_STANDBY) {
		PM_LOG_E("explorer sleep return error, completed_power_state:%d\n",
				epd->completed_power_state);
		err = -EFAULT;
	}

out:
	if (err >= 0) {
		PM_LOG_I("explorer sleep success\n");
	} else {
		PM_LOG_E("explorer sleep fail, err=%d\n", err);
		report_power_exception(epd,
				EXCEPTION_POWER_MAJOR_TYPE_SUSPEND_RESUME,
				EXCEPTION_POWER_SUB_TYPE_SLEEP_FAIL,
				EXCEPTION_ACT_NONE);
	}

	return err;
}

int sleep_explorer(struct explorer_plat_data *epd, bool force) {
	int err = 0;

	mutex_lock(&epd->power_sync_lock);
	err = sleep_explorer_internal(epd, force);
	mutex_unlock(&epd->power_sync_lock);

	return err;
}

int set_power_state_explorer(struct explorer_plat_data *epd, unsigned int state) {
	int err = 0;
	long timeout = 3 * HZ;

	if ((state <= POWER_IOC_STATE_FIRST) || (state >= POWER_IOC_STATE_LAST)) {
		PM_LOG_E("invalid target power state: %d\n", state);
		return -EINVAL;
	}

	mutex_lock(&epd->power_sync_lock);

	if ((state == epd->completed_power_state) && (state != POWER_IOC_STATE_MISSION))  {
		PM_LOG_I("already in target power state: %d\n", state);
		goto out;
	}

	/* Add skip when explorer power off */
	if (atomic_read(&epd->is_explorer_on) == 0) {
		PM_LOG_I("skip set power state [power-off]");
		goto out;
	}

	/* If Explorer is in standby now, and it wants to change to other modes,
	 * needs to wakeup first.
	 */
	if (epd->completed_power_state == POWER_IOC_STATE_STANDBY) {
		PM_LOG_I("Explorer in standby mode, resume explorer first\n");
		err = wakeup_explorer_internal(epd);
		if (err < 0) {
			PM_LOG_I("Explorer in standby mode, resume fail, err:%d\n", err);
			goto out;
		}
	}

	if (state == POWER_IOC_STATE_STANDBY) {
		PM_LOG_I("Before explorer enter standby, switch hclk->sdclk\n");
		err = switch_sdu_clk_internal(epd, HCLK_TO_SDCLK);
		if (err < 0) {
			PM_LOG_E("err in switch sdu clk, err:%d\n", err);
			goto out;
		}
	}
	/* After prepare done, send power state message. */
	reinit_completion(&epd->power_state_completion);
	err = explorer_write_power_data(epd,
			POWER_AP2CC_CMD_SET_POWER_STATE,
			state);
	if (err < 0) {
		PM_LOG_E("err:%d\n", err);
		goto out;
	}
	timeout = wait_for_completion_interruptible_timeout(&epd->power_state_completion, timeout);
	if (timeout <= 0) {
		err = !timeout ? -ETIMEDOUT : -ERESTARTSYS;
		PM_LOG_E("err:%d\n", err);
		goto out;
	}
	if ((epd->completed_power_state == POWER_IOC_STATE_FIRST) || (epd->completed_power_state != state)) {
		PM_LOG_E("Explorer power state set return error, state=%d completed_power_state:%d\n",
				state, epd->completed_power_state);
		err = -EFAULT;
	}
out:
	mutex_unlock(&epd->power_sync_lock);

	return err;
}

#if POWER_STATE_GET_FROM_REMOTE
int get_power_state_explorer(struct explorer_plat_data *epd, unsigned int *state) {
	int err = 0;
	long timeout = 3 * HZ;

	mutex_lock(&epd->power_sync_lock);
	/* Add skip when explorer power off */
	if (atomic_read(&epd->is_explorer_on) == 0) {
		PM_LOG_I("skip get power state [power-off]");
		goto out;
	}

	reinit_completion(&epd->power_state_completion);
	err = explorer_write_power_data(epd,
			POWER_AP2CC_CMD_GET_POWER_STATE,
			0);
	if (err < 0) {
		PM_LOG_E("err:%d\n", err);
		goto out;
	}
	timeout = wait_for_completion_interruptible_timeout(&epd->power_state_completion, timeout);
	if (timeout <= 0) {
		err = !timeout ? -ETIMEDOUT : -ERESTARTSYS;
		PM_LOG_E("err:%d\n", err);
		goto out;
	}
	if (epd->completed_power_state == POWER_IOC_STATE_FIRST) {
		err = -EFAULT;
		PM_LOG_E("Explorer power state get return error, err=%d\n", err);
		goto out;
	}
	*state = epd->completed_power_state;
	PM_LOG_I("get power state success, state: %d\n", *state);

out:
	mutex_unlock(&epd->power_sync_lock);

	return err;
}
#else
/* TBD: When explorer power state changed, it must report to AP for the change */
int get_power_state_explorer(struct explorer_plat_data *epd, unsigned int *state) {
	*state = epd->completed_power_state;
	PM_LOG_I("get power state success, state: %d\n", *state);

	return 0;
}
#endif

static int power_control_explorer_internal(struct explorer_plat_data *epd, bool is_on) {
	int err = 0;
	long timeout = 5 * HZ;

	if (epd->ignore_pmic) {
		PM_LOG_I("pmic gpio ctl not support\n");
		return 0;
	}

	PM_LOG_D("time mark start");
	PM_LOG_I("is_on: %d\n", is_on);

	if (epd->is_pmic_pon == is_on) {
		PM_LOG_I("skip power control due to the same state");
		goto out;
	}

	if (is_on) {
		PM_LOG_I("do PMIC ON\n");
		err = gpio_direction_output(epd->pmic_pon_gpio, 1);
		if (err) {
			PM_LOG_E("operate pmic_pon_gpio to 1 fail\n");
			goto out;
		}
		err = gpio_direction_output(epd->pmic_reset_gpio, 0);
		if (err) {
			PM_LOG_E("operate pmic_reset_gpio to 0 fail\n");
			goto out;
		}
		usleep_range(10000, 10000);
		epd->completed_power_state = POWER_IOC_STATE_FIRST;
	} else {
		if (epd->is_poweroff_skip == true) {
			PM_LOG_E("skip power off [DEBUG]\n");
			goto out;
		}
		PM_LOG_I("do PMIC OFF\n");
		err = gpio_direction_output(epd->pmic_pon_gpio, 1);
		if (err) {
			PM_LOG_E("operate pmic_pon_gpio to 1 fail\n");
			goto out;
		}
		err = gpio_direction_output(epd->pmic_reset_gpio, 1);
		if (err) {
			PM_LOG_E("operate pmic_reset_gpio to 1 fail\n");
			goto out;
		}
		usleep_range(4000, 4000);
		err = gpio_direction_output(epd->pmic_pon_gpio, 0);
		if (err) {
			PM_LOG_E("operate pmic_pon_gpio to 0 fail\n");
			goto out;
		}
		err = gpio_direction_output(epd->pmic_reset_gpio, 0);
		if (err) {
			PM_LOG_E("operate pmic_reset_gpio to 0 fail\n");
			goto out;
		}
		usleep_range(35000, 35000);
		epd->completed_power_state = POWER_IOC_STATE_FIRST;
#ifdef SLT_ENABLE
		//Clean slt has booted flag if power off;
		epd->ebs.slt_has_booted = false;
#endif
		if (epd->sdio_data) {
			reinit_completion(&epd->sdio_remove_completion);
			timeout = wait_for_completion_interruptible_timeout(&epd->sdio_remove_completion, timeout);
		}
	}

	epd->is_pmic_pon = is_on;
out:
	PM_LOG_D("time mark end");
	return err;
}

int power_control_explorer(struct explorer_plat_data *epd, bool is_on) {
	int err = 0;

	mutex_lock(&epd->power_sync_lock);
	err = power_control_explorer_internal(epd, is_on);
	mutex_unlock(&epd->power_sync_lock);
	return err;
}

int power_reload_pmic_otp(struct explorer_plat_data *epd) {
	int err = 0;

	PM_LOG_I("do PMIC OTP reload\n");
	PM_LOG_D("time mark start");
	err = gpio_direction_output(epd->pmic_pon_gpio, 0);
	if (err) {
		PM_LOG_E("operate pmic_pon_gpio to 0 fail\n");
		goto out;
	}
	err = gpio_direction_output(epd->pmic_reset_gpio, 1);
	if (err) {
		PM_LOG_E("operate pmic_reset_gpio to 1 fail\n");
		goto out;
	}
	usleep_range(4000, 4000);
	err = gpio_direction_output(epd->pmic_reset_gpio, 0);
	if (err) {
		PM_LOG_E("operate pmic_reset_gpio to 0 fail\n");
		goto out;
	}
	usleep_range(22000, 22000);
out:
	PM_LOG_D("time mark end");
	return err;
}

int power_reset_explorer(struct explorer_plat_data *epd) {
	int err = 0;
	long timeout = 5 * HZ;

	if (epd->ignore_pmic) {
		PM_LOG_I("pmic gpio ctl not support\n");
		return 0;
	}

	mutex_lock(&epd->power_sync_lock);
	PM_LOG_I("do PMIC RESET\n");

	err = gpio_direction_output(epd->pmic_reset_gpio, 1);
	if (err) {
		PM_LOG_E("operate pmic_reset_gpio to 1 fail\n");
		goto out;
	}
	msleep(4);
	err = gpio_direction_output(epd->pmic_reset_gpio, 0);
	if (err) {
		PM_LOG_E("operate pmic_reset_gpio to 0 fail\n");
		goto out;
	}
	epd->completed_power_state = POWER_IOC_STATE_FIRST;
	if (epd->sdio_data) {
		reinit_completion(&epd->sdio_remove_completion);
		timeout = wait_for_completion_interruptible_timeout(&epd->sdio_remove_completion, timeout);
	}
out:
	mutex_unlock(&epd->power_sync_lock);
	return err;
}

static int clock_control_explorer_internal(struct explorer_plat_data *epd, bool is_on) {
	int err = 0;

	PM_LOG_D("time mark start");
	PM_LOG_I("is_on: %d\n", is_on);
	if (epd->is_clock_on == is_on) {
		PM_LOG_I("skip clock control due to the same state");
		goto out;
	}

	if (is_on) {
		PM_LOG_I("do CLOCK ON\n");
#ifndef ZEKU_EXPLORER_PLATFORM_RPI
#ifdef OPLUS_EXPLORER_PLATFORM_QCOM
 		err = clk_prepare_enable(epd->clk_ref);
#else
		err = clk_buf_hw_ctrl(epd->clk_ref, true);
#endif
#endif
		if (err) {
			PM_LOG_E("operate clk_ref on fail\n");
			goto out;
		}
	} else {
		PM_LOG_I("do CLOCK OFF\n");
		if (epd->is_poweroff_skip == true) {
			PM_LOG_E("skip clock off [DEBUG]\n");
			goto out;
		}
#ifndef ZEKU_EXPLORER_PLATFORM_RPI
#ifdef OPLUS_EXPLORER_PLATFORM_QCOM
		/* clk_disable_unprepare() has no return value */
		clk_disable_unprepare(epd->clk_ref);
#else
		err = clk_buf_hw_ctrl(epd->clk_ref, false);
		if (err) {
			PM_LOG_E("operate clk_ref off fail\n");
			goto out;
		}
#endif
#endif
	}

	epd->is_clock_on = is_on;
out:
	PM_LOG_D("time mark end");
	return err;
}

int clock_control_explorer(struct explorer_plat_data *epd, bool is_on) {
	int err = 0;

	mutex_lock(&epd->power_sync_lock);
	err = clock_control_explorer_internal(epd, is_on);
	mutex_unlock(&epd->power_sync_lock);

	return err;
}
static void explorer_set_awake(struct explorer_plat_data *chip, bool awake)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))

	if (awake) {
		wake_lock(&chip->suspend_lock);
	} else {
		wake_unlock(&chip->suspend_lock);
	}
#else
	static bool pm_flag = false;

	if (!chip->suspend_ws)
		return;

	if (awake && !pm_flag) {
		pm_flag = true;
		__pm_stay_awake(chip->suspend_ws);
		pr_err("%s, __pm_stay_awake.\n", __func__);
	} else if (!awake && pm_flag) {
		__pm_relax(chip->suspend_ws);
		pm_flag = false;
		pr_err("%s, __pm_relax.\n", __func__);
	}
#endif
}

int power_clock_control_explorer(struct explorer_plat_data *epd, bool is_on) {
	int ret = 0;
	int is_on_status = 0;
	int cur_status = 0;

	PM_LOG_D("time mark start");
	mutex_lock(&epd->power_sync_lock);

	if (is_on == false) {
		/* cancel heartbeat detect when explorer power off */
		epd->heartbeat_started = false;
		cancel_heartbeat_detect_work(epd);
	}

	if (is_on)
		is_on_status = 1;

	cur_status = atomic_read(&epd->is_explorer_on);
	if (cur_status == is_on_status) {
		PM_LOG_I("skip power&clock control due to the same state");
		goto out;
	}
	atomic_set(&epd->is_explorer_on, is_on_status);

	if (is_on == true) {
		explorer_set_awake(epd, true);
		ret = clock_control_explorer_internal(epd, true);
		if (!ret) {
			ret = power_control_explorer_internal(epd, true);
		}
	} else {
		ret = power_control_explorer_internal(epd, false);
		if (!ret) {
			ret = clock_control_explorer_internal(epd, false);
		}
		explorer_set_awake(epd, false);
	}
out:
	mutex_unlock(&epd->power_sync_lock);
	PM_LOG_D("time mark end");
	return ret;
}

int power_clock_suspend_explorer(struct explorer_plat_data *epd) {
	int ret = 0;
	int cur_status = 0;

	PM_LOG_D("time mark start");
	mutex_lock(&epd->power_sync_lock);

	cur_status = atomic_read(&epd->is_explorer_on);
	if (cur_status == 0) {
		goto out;
	}
	atomic_set(&epd->is_explorer_on, 0);
	/* cancel heartbeat detect when explorer suspend */
	epd->heartbeat_started = false;
	cancel_heartbeat_detect_work(epd);

	ret = power_control_explorer_internal(epd, false);
	if (!ret) {
		ret = clock_control_explorer_internal(epd, false);
	}

	report_power_exception(epd,
			EXCEPTION_POWER_MAJOR_TYPE_UNEXPECTED_STATE,
			EXCEPTION_POWER_SUB_TYPE_EXPLORER_SUSPEND,
			EXCEPTION_ACT_POWROFF);
out:
	mutex_unlock(&epd->power_sync_lock);

	PM_LOG_D("time mark end");
	return ret;
}

bool get_explorer_on_status(struct explorer_plat_data *epd) {
	bool is_on = false;
	int is_on_status = 0;

	is_on_status = atomic_read(&epd->is_explorer_on);

	if (is_on_status)
		is_on = true;

	return is_on;
}

#ifdef SLT_ENABLE
int regulator_control_explorer(struct explorer_plat_data *epd, bool is_on) {
	int err = 0;
	mutex_lock(&epd->power_sync_lock);
	PM_LOG_I("is_on: %d\n", is_on);
	if (epd->is_vcc_sdio_on == is_on) {
		PM_LOG_I("skip regulator control due to the same state");
		goto out;
	}

	if (is_on) {
		PM_LOG_I("do REGULATOR ON\n");
		err = regulator_enable(epd->vcc_sdio);
		if (err) {
			PM_LOG_E("operate vcc_sdio on fail\n");
			goto out;
		}
	} else {
		PM_LOG_I("do REGULATOR OFF\n");
		err = regulator_disable(epd->vcc_sdio);
		if (err) {
			PM_LOG_E("operate vcc_sdio off fail\n");
			goto out;
		}
	}

	epd->is_vcc_sdio_on = is_on;
out:
	mutex_unlock(&epd->power_sync_lock);
	return err;
}
#endif
int explorer_process_power_interrupts(struct explorer_plat_data *epd) {
	int err = 0;
	int status = 0;

	if (epd->is_sdu_clk_switched) {
		if(check_sleep_int(epd)) {
			int count = PMU_STATE_WAIT_COUNT;
			PM_LOG_I("explorer sleep int\n");

			epd->is_cc_standby = true;

			err = write_ap_allow_enter_standby(epd, true);
			if (err < 0) {
				PM_LOG_E("write_ap_allow_enter_standby to true error, err=%d\n", err);
				return -1;
			}

			do {
				status = read_sdio_pwr_status(epd);
				if (status < 0) {
					PM_LOG_E("read_sdio_pwr_status error, status=%d\n", status);
					return -1;
				}
				if (status == PWR_STATUS_ENTER_STANDBY) {
					PM_LOG_I("explorer enter standby\n");
					break;
				}
				usleep_range(1000, 1000);
			} while (count-- > 0);
			if ((count <= 0) && (status != PWR_STATUS_ENTER_STANDBY)) {
				PM_LOG_E("TIMEOUT waiting for explorer enter standby\n");
				return -ETIMEDOUT;
			}

			err = write_ap_allow_enter_standby(epd, false);
			if (err < 0) {
				PM_LOG_E("write_ap_allow_enter_standby to false error, err=%d\n", err);
				return -1;
			}
		} else if(check_wakeup_int(epd)) {
			int count = PMU_STATE_WAIT_COUNT;
			PM_LOG_I("explorer wakeup int\n");

			epd->is_cc_standby = false;
			err = write_ap_allow_exit_standby(epd, true);
			if (err < 0) {
				PM_LOG_E("write_ap_allow_exit_standby to true error, err=%d\n", err);
				return -1;
			}

			do {
				status = read_sdio_pwr_status(epd);
				if (status < 0) {
					PM_LOG_E("read_sdio_pwr_status error, status=%d\n", status);
					return -1;
				}
				if (status == PWR_STATUS_EXIT_STANDBY) {
					PM_LOG_I("explorer exit standby\n");
					break;
				}
				usleep_range(1000, 1000);
			} while (count-- > 0);
			if ((count <= 0) && (status != PWR_STATUS_EXIT_STANDBY)) {
				PM_LOG_E("TIMEOUT waiting for explorer exit standby\n");
				return -ETIMEDOUT;
			}

			err = write_ap_allow_exit_standby(epd, false);
			if (err < 0) {
				PM_LOG_E("write_ap_allow_exit_standby to false error, err=%d\n", err);
				return -1;
			}
		} else {
			PM_LOG_D("IRQ wrong\n");
			return -EFAULT;
		}
	}
	return 0;
}

void prepare_pmic_exception(struct explorer_plat_data *epd, struct exception_info *info)
{
	if ((epd->action_when_pmic_oc & PMIC_OC_ACTION_AFTER_BOOT_POWER_OFF) == 0) {
		return;
	}

	if ((info->moduleId == EXCEPTION_PMIC) && (info->majorType == OCOVUV)) {
		switch (info->subType) {
			case PMIC_ERR_LDO1_ILIM:
				PM_LOG_I("PMIC_ERR_LDO1_ILIM\n");
				info->level = EXCEPTION_FATAL;
				break;
			case PMIC_ERR_LDO2_ILIM:
				PM_LOG_I("PMIC_ERR_LDO2_ILIM\n");
				info->level = EXCEPTION_FATAL;
				break;
			case PMIC_ERR_LDO3_ILIM:
				PM_LOG_I("PMIC_ERR_LDO3_ILIM\n");
				info->level = EXCEPTION_FATAL;
				break;
			case PMIC_ERR_BUCK1_OC:
				PM_LOG_I("PMIC_ERR_BUCK1_OC\n");
				info->level = EXCEPTION_FATAL;
				break;
			case PMIC_ERR_BUCK2_OC:
				PM_LOG_I("PMIC_ERR_BUCK2_OC\n");
				info->level = EXCEPTION_FATAL;
				break;
			case PMIC_ERR_BUCK3_OC:
				PM_LOG_I("PMIC_ERR_BUCK3_OC\n");
				info->level = EXCEPTION_FATAL;
				break;
			case PMIC_ERR_BUCK4_OC:
				PM_LOG_I("PMIC_ERR_BUCK4_OC\n");
				info->level = EXCEPTION_FATAL;
				break;
			case PMIC_ERR_BUCK5_OC:
				PM_LOG_I("PMIC_ERR_BUCK5_OC\n");
				info->level = EXCEPTION_FATAL;
				break;
			case PMIC_ERR_BUCK6_OC:
				PM_LOG_I("PMIC_ERR_BUCK6_OC\n");
				info->level = EXCEPTION_FATAL;
				break;
			default:
				PM_LOG_I("error PMIC oc type:%d\n", info->subType);
		}
	}
}

int report_power_exception(struct explorer_plat_data *epd, int majorType, int subType, int action)
{
	int ret = 0;
	struct exception_info info;
	int exception_level = EXCEPTION_ERROR;

	switch (subType) {
		case EXCEPTION_POWER_SUB_TYPE_WAKEUP_FAIL:
			PM_LOG_I("report power exception: [WAKEUP_FAIL]\n");
			break;
		case EXCEPTION_POWER_SUB_TYPE_SLEEP_FAIL:
			PM_LOG_I("report power exception: [SLEEP_FAIL]\n");
			break;
		case EXCEPTION_POWER_SUB_TYPE_PMIC_HW_ERR_BY_HEARTBEAT:
			PM_LOG_I("report power exception: [HEARTBEAT]\n");
			if (epd->action_when_pmic_oc & PMIC_OC_ACTION_AFTER_BOOT_POWER_OFF) {
				PM_LOG_I("pmic hw err, report FATAL EXCEPTION to uplayer\n");
				exception_level = EXCEPTION_FATAL;
			}
			if (epd->is_heartbeat_poweroff_skip) {
				epd->is_poweroff_skip = true;
				PM_LOG_I("heartbeat exception, skip power off\n");
			}
			break;
		case EXCEPTION_POWER_SUB_TYPE_PMIC_HW_ERR_BY_PBL_DETECT:
			PM_LOG_I("report power exception: [PBL_DETECT]\n");
			break;
		case EXCEPTION_POWER_SUB_TYPE_PMIC_HW_ERR_BY_BOOT_TIMEOUT:
			PM_LOG_I("report power exception: [BOOT_TIMEOUT]\n");
			break;
		case EXCEPTION_POWER_SUB_TYPE_EXPLORER_SUSPEND:
			PM_LOG_I("report power exception: [EXPLORER_SUSPEND]\n");
			break;
		default:
			PM_LOG_E("err subtype:%d\n", subType);
	}

	info.moduleId = EXCEPTION_POWER;
	info.majorType = majorType;
	info.subType = subType;
	info.action = action;
	info.level = exception_level;

	EXCEPTION_LOG_W("exception:moduleID=%d, majorType=%d, subType=%d, action=%d, lvl=%d [power]\n",
			info.moduleId, info.majorType, info.subType, info.action, info.level);

	ret = exception_handle_func(epd, &info);
	if (ret < 0) {
		PM_LOG_E("exception handle error, ret:%d\n", ret);
	}

	return ret;
}

void heartbeat_detect_delayed_work(struct work_struct *work)
{
	struct explorer_plat_data *epd = container_of(work, struct explorer_plat_data,
			heartbeat_detect_work.work);

	if (atomic_read(&epd->is_explorer_on) == 0) {
		PM_LOG_I("skip hearbeat report [power-off]");
		return;
	} else if (epd->heartbeat_started == false) {
		PM_LOG_I("skip hearbeat report [heartbeat-stop]");
		return;
	} else {
		PM_LOG_E("heartbeat not received\n");
	}

	report_power_exception(epd,
			EXCEPTION_POWER_MAJOR_TYPE_PMIC_HW_ERR,
			EXCEPTION_POWER_SUB_TYPE_PMIC_HW_ERR_BY_HEARTBEAT,
			EXCEPTION_ACT_POWROFF);
}

int schedule_heartbeat_detect_work(struct explorer_plat_data *epd, unsigned int timeout)
{
	int ret = 0;
	PM_LOG_D("schedule heartbeat detect work after %u ms\n", timeout);

	if (timeout == 0) {
		PM_LOG_E("error heartbeat timeout\n");
		ret = -1;
		goto out;
	}

	if (atomic_read(&epd->is_explorer_on) == 0) {
		PM_LOG_I("skip hearbeat detect [power-off]");
		ret = -1;
		goto out;
	}

	if (epd->heartbeat_started == false) {
		PM_LOG_I("skip hearbeat detect [heartbeat-reset]");
		ret = -1;
		goto out;
	}

	schedule_delayed_work(&epd->heartbeat_detect_work,
			msecs_to_jiffies(timeout));

out:
	return ret;
}

int cancel_heartbeat_detect_work(struct explorer_plat_data *epd)
{
	PM_LOG_D("cancel heartbeat detect work\n");
	cancel_delayed_work(&epd->heartbeat_detect_work);
	return 0;
}
