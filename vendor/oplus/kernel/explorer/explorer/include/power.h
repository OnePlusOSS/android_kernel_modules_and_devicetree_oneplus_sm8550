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

#ifndef _EXPLORER_POWER_H
#define _EXPLORER_POWER_H

#define PM_LOG_E(fmt, ...) pr_err("[ExplorerPower] %s(%d) "fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define PM_LOG_I(fmt, ...) pr_info("[ExplorerPower] %s(%d) "fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define PM_LOG_D(fmt, ...) pr_debug("[ExplorerPower] %s(%d) "fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__)

/* Feature Definition */
#define POWER_STATE_GET_FROM_REMOTE 0

#define PWR_STATUS_DEFAULT 0x0
#define PWR_STATUS_WAIT_AP_ALLOW 0x1
#define PWR_STATUS_ENTER_STANDBY 0x2
#define PWR_STATUS_EXIT_STANDBY 0x4

/* Two uint32_t for data transfer between Explorer and AP
 * data[0]: for cmd type
 * data[1]: for cmd data
 */
#define POWER_CC2AP_CMD_RT_SUSPEND_STATUS           0x1
#define POWER_CC2AP_DAT_RT_SUSPEND_STATUS_NORMAL    (1 << 0)
#define POWER_CC2AP_DAT_RT_SUSPEND_STATUS_PD        (1 << 1)
#define POWER_CC2AP_DAT_RT_SUSPEND_STATUS_PLL       (1 << 2)
#define POWER_CC2AP_DAT_RT_SUSPEND_STATUS_SRAM      (1 << 3)
#define POWER_CC2AP_DAT_RT_SUSPEND_STATUS_THREAD    (1 << 4)
#define POWER_CC2AP_CMD_POWER_STATE_SET_RESULT      0x2
#define POWER_CC2AP_CMD_POWER_STATE_SET_RESULT_OK   1
#define POWER_CC2AP_CMD_POWER_STATE_SET_RESULT_FAIL 2
#define POWER_CC2AP_CMD_POWER_STATE_GET_RESULT      0x3
#define POWER_CC2AP_CMD_POWER_STATE_GET_RESULT_OK   1
#define POWER_CC2AP_CMD_POWER_STATE_GET_RESULT_FAIL 2
#define POWER_CC2AP_CMD_RT_RESUME_STATUS            0x4
#define POWER_CC2AP_CMD_RT_RESUME_STATUS_OK         1
#define POWER_CC2AP_CMD_HEARTBEAT                   0x5

#define POWER_AP2CC_CMD_ENTER_STANDBY               0x1
#define POWER_AP2CC_CMD_ENTER_STANDBY_FORCE         0x2
#define POWER_AP2CC_CMD_EXIT_STANDBY                0x3
#define POWER_AP2CC_CMD_SET_POWER_STATE             0x4
#define POWER_AP2CC_CMD_GET_POWER_STATE             0x5

#define POWER_SUSPEND_FAIL                          1
#define POWER_SUSPEND_SUCCESS                       0

#define POWER_RESUME_FAIL                           1
#define POWER_RESUME_SUCCESS                        0

#define POWER_DEBUG_COMMAND_BASE 0
#define POWER_DEBUG_AP_COMMAND_ENTER_STANDBY        (POWER_DEBUG_COMMAND_BASE + POWER_AP2CC_CMD_ENTER_STANDBY)
#define POWER_DEBUG_AP_COMMAND_EXIT_STANDBY         (POWER_DEBUG_COMMAND_BASE + POWER_AP2CC_CMD_EXIT_STANDBY)
#define POWER_DEBUG_AP_COMMAND_ENTER_STANDBY_FORCE  (POWER_DEBUG_COMMAND_BASE + POWER_AP2CC_CMD_ENTER_STANDBY_FORCE)
#define POWER_DEBUG_PMIC_CLOCK_GPIO_BASE 20
#define POWER_DEBUG_CLK_REF_ON                      (POWER_DEBUG_PMIC_CLOCK_GPIO_BASE + 0)
#define POWER_DEBUG_CLK_REF_OFF                     (POWER_DEBUG_PMIC_CLOCK_GPIO_BASE + 1)
#define POWER_DEBUG_PMIC_PON_HIGH                   (POWER_DEBUG_PMIC_CLOCK_GPIO_BASE + 2)
#define POWER_DEBUG_PMIC_PON_LOW                    (POWER_DEBUG_PMIC_CLOCK_GPIO_BASE + 3)
#define POWER_DEBUG_PMIC_RESET_HIGH                 (POWER_DEBUG_PMIC_CLOCK_GPIO_BASE + 4)
#define POWER_DEBUG_PMIC_RESET_LOW                  (POWER_DEBUG_PMIC_CLOCK_GPIO_BASE + 5)
#define POWER_DEBUG_PMIC_ON                         (POWER_DEBUG_PMIC_CLOCK_GPIO_BASE + 6)
#define POWER_DEBUG_PMIC_OFF                        (POWER_DEBUG_PMIC_CLOCK_GPIO_BASE + 7)
#define POWER_DEBUG_PMIC_RESET                      (POWER_DEBUG_PMIC_CLOCK_GPIO_BASE + 8)
#define POWER_DEBUG_PMIC_ON_CLK_ON                  (POWER_DEBUG_PMIC_CLOCK_GPIO_BASE + 9)
#define POWER_DEBUG_PMIC_OFF_CLK_OFF                (POWER_DEBUG_PMIC_CLOCK_GPIO_BASE + 10)

#define POWER_DEBUG_POWER_STATE_SET_GET_BASE 40
#define POWER_DEBUG_POWER_STATE_SET_STANDBY         (POWER_DEBUG_POWER_STATE_SET_GET_BASE + 0)
#define POWER_DEBUG_POWER_STATE_SET_BYPASS          (POWER_DEBUG_POWER_STATE_SET_GET_BASE + 1)
#define POWER_DEBUG_POWER_STATE_SET_AON             (POWER_DEBUG_POWER_STATE_SET_GET_BASE + 2)
#define POWER_DEBUG_POWER_STATE_SET_MISSION         (POWER_DEBUG_POWER_STATE_SET_GET_BASE + 3)
#define POWER_DEBUG_POWER_STATE_GET                 (POWER_DEBUG_POWER_STATE_SET_GET_BASE + 4)

#define POWER_DEBUG_PMIC_OC_BASE 60
#define POWER_DEBUG_PMIC_OC_ALL_REPORT                 (POWER_DEBUG_PMIC_OC_BASE + 0)
#define POWER_DEBUG_PMIC_OC_ALL_POWER_OFF              (POWER_DEBUG_PMIC_OC_BASE + 1)
#define POWER_DEBUG_PMIC_OC_BEFORE_BOOT_POWER_OFF      (POWER_DEBUG_PMIC_OC_BASE + 2)
#define POWER_DEBUG_PMIC_OC_AFTER_BOOT_POWER_OFF       (POWER_DEBUG_PMIC_OC_BASE + 3)

#define POWER_DEBUG_POWER_OFF_SKIP_BASE 70
#define POWER_DEBUG_SET_POWER_OFF_SKIP                 (POWER_DEBUG_POWER_OFF_SKIP_BASE + 0)
#define POWER_DEBUG_CLR_POWER_OFF_SKIP                 (POWER_DEBUG_POWER_OFF_SKIP_BASE + 1)
#define POWER_DEBUG_SET_HEARTBEAT_POWER_OFF_SKIP                 (POWER_DEBUG_POWER_OFF_SKIP_BASE + 2)
#define POWER_DEBUG_CLR_HEARTBEAT_POWER_OFF_SKIP                 (POWER_DEBUG_POWER_OFF_SKIP_BASE + 3)

#define PMIC_OC_ACTION_BEFORE_BOOT_POWER_OFF      (1 << 0)
#define PMIC_OC_ACTION_AFTER_BOOT_POWER_OFF      (1 << 1)

#define REG_PMU_INT_SET             0x48001300
#define PMU_INT_SET_RESERVED0_MASK  (1 << 18)

#define PMIC_ERR_LDO1_ILIM    21
#define PMIC_ERR_LDO2_ILIM    22
#define PMIC_ERR_LDO3_ILIM    23
#define PMIC_ERR_BUCK1_OC     24
#define PMIC_ERR_BUCK2_OC     27
#define PMIC_ERR_BUCK3_OC     30
#define PMIC_ERR_BUCK4_OC     33
#define PMIC_ERR_BUCK5_OC     36
#define PMIC_ERR_BUCK6_OC     39

enum pmic_exceptions {
    REG_ERROR = 0,
    HIGH_TEMP,
    OCOVUV,
    SUPPLY_HIGH,
    SUPPLY_LOW,
};

int explorer_process_power_interrupts(struct explorer_plat_data *epd);
int wakeup_explorer(struct explorer_plat_data *epd);
int sleep_explorer(struct explorer_plat_data *epd, bool force);
int read_sdio_pwr_status(struct explorer_plat_data *epd);
int power_control_explorer(struct explorer_plat_data *epd, bool is_on);
int power_reset_explorer(struct explorer_plat_data *epd);
int clock_control_explorer(struct explorer_plat_data *epd, bool is_on);
int power_clock_control_explorer(struct explorer_plat_data *epd, bool is_on);
int power_clock_suspend_explorer(struct explorer_plat_data *epd);
bool get_explorer_on_status(struct explorer_plat_data *epd);
#ifdef SLT_ENABLE
int regulator_control_explorer(struct explorer_plat_data *epd, bool is_on);
#endif
int set_power_state_explorer(struct explorer_plat_data *epd, unsigned int state);
int get_power_state_explorer(struct explorer_plat_data *epd, unsigned int *state);

/* WA: for heartbeat detect, endless detect */
void heartbeat_detect_delayed_work(struct work_struct *work);
int schedule_heartbeat_detect_work(struct explorer_plat_data *epd, unsigned int timeout);
int cancel_heartbeat_detect_work(struct explorer_plat_data *epd);
int report_power_exception(struct explorer_plat_data *epd, int majorType, int subType, int action);
void prepare_pmic_exception(struct explorer_plat_data *epd, struct exception_info *info);
/* reload PMIC OTP after reboot */
int power_reload_pmic_otp(struct explorer_plat_data *epd);
#endif /* _EXPLORER_POWER_H */
