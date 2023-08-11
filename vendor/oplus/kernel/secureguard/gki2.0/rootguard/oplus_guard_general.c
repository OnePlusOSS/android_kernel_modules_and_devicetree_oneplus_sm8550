// SPDX-License-Identifier: GPL-2.0-only
/**************************************************************
* Copyright (c)  2008- 2020  Oplus. All rights reserved..
*
* File		: oplus_guard_general.c
* Descriptio	: some common function for root guard
* Version		: 1.0
* Date		: 2019-12-19
* Author		:
* TAG		:
****************************************************************/
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>

#ifdef CONFIG_CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
enum{
        MSM_BOOT_MODE__NORMAL,
        MSM_BOOT_MODE__FASTBOOT,
        MSM_BOOT_MODE__RECOVERY,
        MSM_BOOT_MODE__FACTORY,
        MSM_BOOT_MODE__RF,
        MSM_BOOT_MODE__WLAN,
        MSM_BOOT_MODE__MOS,
        MSM_BOOT_MODE__CHARGE,
        MSM_BOOT_MODE__SILENCE,
        MSM_BOOT_MODE__SAU,

        MSM_BOOT_MODE__AGING = 998,
        MSM_BOOT_MODE__SAFE = 999,
};
/*remove is_normal_boot_mode(), because anti-remount is not support after Android S*/
/*
extern int get_boot_mode(void);
bool is_normal_boot_mode(void)
{
	return MSM_BOOT_MODE__NORMAL == get_boot_mode();
}
*/
#else
/*
#define MTK_NORMAL_BOOT	0
unsigned int get_boot_mode(void){
	return 0;
}
bool is_normal_boot_mode(void)
{
	return MTK_NORMAL_BOOT == get_boot_mode();
}
*/

#endif


enum{
        BOOT_STATE__GREEN,
        BOOT_STATE__ORANGE,
        BOOT_STATE__YELLOW,
        BOOT_STATE__RED,
};


static int __ro_after_init g_boot_state  = BOOT_STATE__GREEN;


bool is_unlocked(void)
{
	return  BOOT_STATE__ORANGE== g_boot_state;
}

extern char verified_bootstate[];

int oplus_boot_state_init(void)
{
    pr_info("[ROOTCHECK-RC-INFO]:verified_bootstate is %s .\n", verified_bootstate);
    if (strstr(verified_bootstate, "orange")) {
        g_boot_state = BOOT_STATE__ORANGE;
    } else {
        g_boot_state = BOOT_STATE__GREEN;
    }
    return 0;
}

void oplus_boot_state_exit(void)
{
	return ;
}

MODULE_LICENSE("GPL");

