/**************************************************************
* Copyright (c)  2008- 2030  OPLUS Mobile communication Corp.ltd All rights reserved.
*
* File       : oplus_harden_hook.h
* Description: For oplus hook harden
* Version   : 1.0
* Date        : 2021-05-21
* Author    : zhangyi
* TAG         :
****************************************************************/
#ifndef _OPLUS_HARDEN_HOOK_H
#define _OPLUS_HARDEN_HOOK_H
#define __NR_setregid 143
#define __NR_setgid 144
#define __NR_setreuid 145
#define __NR_setuid 146
#define __NR_setresuid 147

#define __NR_setresgid 149

#define __NR_socket 198


void oplus_harden_pre_handler(void *data, struct pt_regs *regs, long id);
void oplus_harden_post_handler(void *data, struct pt_regs *regs, long ret);
#endif/*_OPLUS_HARDEN_HOOK_H*/