/**************************************************************
* Copyright (c)  2008- 2030  OPLUS Mobile communication Corp.ltd All rights reserved.
*
* File       : oplus_hook.h
* Description: oplus_hook frame
* Version   : 1.0
* Date        : 2021-04-13
* Author    : zhangyi
* TAG         :
****************************************************************/
#ifndef _OPLUS_HOOK_H
#define _OPLUS_HOOK_H
/*
* probe: funtion
* data: data
* probe_reg_flag: 1:probe register  0:probe not register
*/

#define OPLUS_ARRAY_SIZE(arr)    (sizeof(arr) / sizeof((arr)[0]))
struct oplus_hook_str {
    void * probe;
	void * data;
    unsigned short probe_reg_flag;
};

unsigned int oplus_get_pre_hook_num(void);
unsigned int oplus_get_post_hook_num(void);

struct oplus_hook_str * oplus_get_pre_hook(void);
struct oplus_hook_str * oplus_get_post_hook(void);
#endif