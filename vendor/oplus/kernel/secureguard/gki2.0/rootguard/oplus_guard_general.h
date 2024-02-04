/* SPDX-License-Identifier: GPL-2.0-only */
/**************************************************************
* Copyright (c)  2008- 2030  OPLUS Mobile communication Corp.ltd All rights reserved.
*
* File       : oplus_guard_general.h
* Description: for rootguard general function
* Version   : 1.0
* Date        : 2019-12-19
* Author    :
* TAG         :
****************************************************************/
#ifndef OPLUS_GUARD_GENERAL_H_
#define OPLUS_GUARD_GENERAL_H_
#include <linux/version.h>
#define KERNEL_ADDR_LIMIT 0x0000008000000000

bool is_unlocked(void);

bool is_normal_boot_mode(void);

int  oplus_boot_state_init(void);

void  oplus_boot_state_exit(void);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
unsigned int get_fs(void);
#endif


#endif /*OPLUS_GUARD_GENERAL_H_*/
