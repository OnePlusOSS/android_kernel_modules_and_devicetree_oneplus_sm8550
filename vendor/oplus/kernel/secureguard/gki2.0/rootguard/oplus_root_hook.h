/**************************************************************
* Copyright (c)  2008- 2030  OPLUS Mobile communication Corp.ltd All rights reserved.
*
* File       : oplus_root_hook.h
* Description: For oplus hook use kretprobe
* Version   : 1.0
* Date        : 2021-04-13
* Author    : zhangyi
* TAG         :
****************************************************************/
#ifndef _OPLUS_ROOT_HOOK_H
#define _OPLUS_ROOT_HOOK_H
#define KERNEL_ADDR_LIMIT 0x0000008000000000

#define __NR_setreuid32 	203
#define __NR_setregid32 	204
#define __NR_setresuid32 	208
#define __NR_setresgid32 	210
#define __NR_setuid32 		213
#define __NR_setgid32 		214

#define __NR_setregid 		143
#define __NR_setgid 		144
#define __NR_setreuid 		145
#define __NR_setuid 		146
#define __NR_setresuid 		147
#define __NR_setresgid 		149

void oplus_root_check_pre_handler(void *data, struct pt_regs *regs, long id);
void oplus_root_check_post_handler(void *data, struct pt_regs *regs, long ret);
#endif