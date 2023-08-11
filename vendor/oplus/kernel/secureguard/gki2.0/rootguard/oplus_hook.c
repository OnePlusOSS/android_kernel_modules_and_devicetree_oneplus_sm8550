// SPDX-License-Identifier: GPL-2.0-only
/**************************************************************
* Copyright (c)  2008- 2020  Oplus. All rights reserved..
*
* File		: oplus_hook.c
* Description	: hook the kernel syscall
* Version   	: 1.0
* Date        	: 2020-04-13
* Author   	:
* TAG        	:
****************************************************************/
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/cred.h>
#include <trace/events/syscalls.h>
#include "oplus_hook.h"
#include "oplus_root_hook.h"
#include "oplus_exec_hook.h"
#include "oplus_harden_hook.h"

static struct oplus_hook_str oplus_pre_hook_array[] = {
#ifdef CONFIG_OPLUS_FEATURE_SECURE_ROOTGUARD
    {oplus_root_check_pre_handler, NULL, 0},
#endif /* CONFIG_OPLUS_FEATURE_SECURE_ROOTGUARD */
#ifdef CONFIG_OPLUS_FEATURE_SECURE_CAPGUARD
	{oplus_harden_pre_handler, NULL, 0},
#endif /* CONFIG_OPLUS_FEATURE_SECURE_CAPGUARD */
//	{NULL, NULL, 0},
};

static struct oplus_hook_str oplus_post_hook_array[] = {
#ifdef CONFIG_OPLUS_FEATURE_SECURE_EXECGUARD
	{oplus_exe_block_ret_handler, NULL, 0},
#endif /* CONFIG_OPLUS_FEATURE_SECURE_EXECGUARD */
#ifdef CONFIG_OPLUS_FEATURE_SECURE_ROOTGUARD
    {oplus_root_check_post_handler, NULL, 0},
#endif /* CONFIG_OPLUS_FEATURE_SECURE_ROOTGUARD */
#ifdef CONFIG_OPLUS_FEATURE_SECURE_CAPGUARD
/*  {oplus_harden_post_handler, NULL, 0}, */
#endif /* CONFIG_OPLUS_FEATURE_SECURE_CAPGUARD */
//	{NULL, NULL, 0},
};

unsigned int oplus_get_pre_hook_num(void)
{
    return OPLUS_ARRAY_SIZE(oplus_pre_hook_array);
}

unsigned int oplus_get_post_hook_num(void)
{
    return OPLUS_ARRAY_SIZE(oplus_post_hook_array);
}

struct oplus_hook_str * oplus_get_pre_hook(void)
{
    struct oplus_hook_str * p_p_oplus_hook_str = NULL;
	p_p_oplus_hook_str = oplus_pre_hook_array;
	return p_p_oplus_hook_str;
}

struct oplus_hook_str * oplus_get_post_hook(void)
{
    struct oplus_hook_str * p_p_oplus_hook_str = NULL;
	p_p_oplus_hook_str = oplus_post_hook_array;
	return p_p_oplus_hook_str;
}





