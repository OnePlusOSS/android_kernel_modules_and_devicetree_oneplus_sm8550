// SPDX-License-Identifier: GPL-2.0-only
/**************************************************************
* Copyright (c)  2008- 2020  Oplus. All rights reserved..
*
* File       	: oplus_root_check.c
* Description	: replace the syscall.c
* Version   	: 1.0
* Date        	: 2019-12-19
* Author    	:
* TAG         	:
****************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/syscalls.h>
#include <linux/kallsyms.h>
#include <linux/tracepoint.h>
#include <trace/hooks/vendor_hooks.h>

#include "oplus_hook.h"
#include "oplus_guard_general.h"


/*any vendor hook tracepoint will work, in order to find __tracepoint_sys_enter and __tracepoint_sys_exit*/
extern struct tracepoint  __tracepoint_dma_fence_emit;
#define ADDR_CALCULATIONS_NUMBER    2000
#define TRACEPOINT_SIZE    0x48 /* SM8450 tarcepoint size is 0x48 */
//#define TRACEPOINT_SIZE    0x30 /* SM8350 R tracepoint size is 0x30 */
#define OPLUS_SYS_EXIT_STRING     "__tracepoint_sys_exit"
#define OPLUS_SYS_ENTER_STRING     "__tracepoint_sys_enter"

struct tracepoint * p_tp_sys_enter;
struct tracepoint * p_tp_sys_exit;

static int __init __nocfi oplus_hook_init(void)
{
	int ret = 0;
	char test[256];
  	char test1[256], test2[256];
	unsigned long base_addr;
	unsigned long sys_exit_addr;
	unsigned long sys_enter_addr;
	unsigned char sys_exit_flag = 0;
	unsigned char sys_enter_flag = 0;

	unsigned int pre_hook_num = 0;
	unsigned int post_hook_num = 0;
	int cnt = 0;

	struct oplus_hook_str * p_pre_hook = NULL;
	struct oplus_hook_str * p_post_hook = NULL;

	oplus_boot_state_init();
	/*get __tracepoint_sys_exit __tracepoint_sys_enter addr*/
	base_addr = (unsigned long)(& __tracepoint_dma_fence_emit);
	pr_info("[ROOTCHECK-TRACEP-INFO]__tracepoint_oplus_test base_addr at 0x%lx\n", base_addr);
	for (cnt = 0; cnt < ADDR_CALCULATIONS_NUMBER; cnt ++){
		sprint_symbol(test, base_addr - cnt*TRACEPOINT_SIZE);
		if (strncmp(test, OPLUS_SYS_EXIT_STRING, strlen(OPLUS_SYS_EXIT_STRING)) == 0){
		    sys_exit_addr = base_addr - cnt*TRACEPOINT_SIZE;
		    sys_exit_flag = 1;
		}

		if (strncmp(test, OPLUS_SYS_ENTER_STRING, strlen(OPLUS_SYS_ENTER_STRING)) == 0){
		    sys_enter_addr = base_addr - cnt*TRACEPOINT_SIZE;
		    sys_enter_flag = 1;
		}

		if ((1 == sys_exit_flag)&&(1 == sys_enter_flag)){
		    break;
		}
	}
	if ( sys_exit_flag == 0 || sys_enter_flag == 0 ){
          pr_info("[ROOTCHECK-TRACEP-ERROR]Not get sys_exit or sys_enter, traceponit failed!! sys_enter_flag:%d, sys_exit_flag:%d\n", sys_enter_flag, sys_exit_flag);
          return ret;
        }
	p_tp_sys_exit = (struct tracepoint *)sys_exit_addr;
	p_tp_sys_enter = (struct tracepoint *)sys_enter_addr;
	sprint_symbol(test1, sys_exit_addr);
  	sprint_symbol(test2, sys_enter_addr);
	pr_info("[ROOTCHECK-TRACEP-INFO]%s: 0x%lx\n", test1, sys_exit_addr);
	pr_info("[ROOTCHECK-TRACEP-INFO]%s: 0x%lx\n", test2,  sys_enter_addr);

	pre_hook_num = oplus_get_pre_hook_num();
	if (0 == pre_hook_num) {
		pr_err("[ROOTCHECK-TRACEP-ERROR]:pre_hook_num 0 err\n");
		ret = -EINVAL;
		goto exit;
	}

	post_hook_num = oplus_get_post_hook_num();
	if (0 == post_hook_num) {
		pr_err("[ROOTCHECK-TRACEP-ERROR]:post_hook_num 0 err\n");
		ret = -EINVAL;
		goto exit;
	}

	p_pre_hook = oplus_get_pre_hook();
	if (NULL == p_pre_hook){
	    pr_err("[ROOTCHECK-TRACEP-ERROR]:p_pre_hook NULL!\n");
		ret = -EINVAL;
		goto exit;
	}

	p_post_hook = oplus_get_post_hook();
	if (NULL == p_post_hook){
	    pr_err("[ROOTCHECK-TRACEP-ERROR]:p_post_hook NULL!\n");
		ret = -EINVAL;
		goto exit;
	}

	for (cnt = 0; cnt < pre_hook_num; cnt++) {
	    /*register pre hook*/
	    ret = tracepoint_probe_register(p_tp_sys_enter, p_pre_hook[cnt].probe, NULL);
	    if (ret) {
		    pr_err("[ROOTCHECK-TRACEP-ERROR]:register_trace_sys_enter failed! ret=%d\n", ret);
			ret = -EPERM;
		    goto exit;
	    }
	    p_pre_hook[cnt].probe_reg_flag = 1;
	}

	for (cnt = 0; cnt < post_hook_num; cnt++) {
		/*register post hook*/
		ret = tracepoint_probe_register(p_tp_sys_exit, p_post_hook[cnt].probe, NULL);
	    if (ret) {
		    pr_err("[ROOTCHECK-TRACEP-ERROR]:register_trace_sys_exit failed! ret=%d\n", ret);
			ret = -EPERM;
		    goto exit;
	    }
	    p_post_hook[cnt].probe_reg_flag = 1;
	}

exit:
    if (ret){
	    for (cnt = post_hook_num -1 ; cnt >= 0; cnt--){
		/*unregister post hook*/
		if (1 == p_post_hook[cnt].probe_reg_flag){
		tracepoint_probe_unregister(p_tp_sys_exit, p_post_hook[cnt].probe, NULL);
		p_post_hook[cnt].probe_reg_flag = 0;
			}
		}

	    for (cnt = pre_hook_num -1 ; cnt >= 0; cnt--){
	        /*unregister post hook*/
		if (1 == p_pre_hook[cnt].probe_reg_flag){
		tracepoint_probe_unregister(p_tp_sys_enter, p_pre_hook[cnt].probe, NULL);
                p_pre_hook[cnt].probe_reg_flag = 0;
		}
	    }
	}
	return ret;
}

static void __exit __nocfi oplus_hook_exit(void)
{
	unsigned int pre_hook_num = 0;
	unsigned int post_hook_num = 0;
	int cnt = 0;

	struct oplus_hook_str * p_pre_hook = NULL;
	struct oplus_hook_str * p_post_hook = NULL;

	pre_hook_num = oplus_get_pre_hook_num();
	post_hook_num = oplus_get_post_hook_num();

	p_pre_hook = oplus_get_pre_hook();
	p_post_hook = oplus_get_post_hook();

        for (cnt = post_hook_num - 1; cnt >= 0; cnt--){
	    /*unregister post hook*/
	    if (1 == p_post_hook[cnt].probe_reg_flag){
	        tracepoint_probe_unregister(p_tp_sys_exit, p_post_hook[cnt].probe, NULL);
	        p_post_hook[cnt].probe_reg_flag = 0;
		}
	}

	for (cnt = pre_hook_num - 1; cnt >= 0; cnt--){
	    /*unregister post hook*/
	    if (1 == p_pre_hook[cnt].probe_reg_flag){
	    tracepoint_probe_unregister(p_tp_sys_enter, p_pre_hook[cnt].probe, NULL);
            p_pre_hook[cnt].probe_reg_flag = 0;
		}
	}

	oplus_boot_state_exit();
	return ;
}


module_init(oplus_hook_init);
module_exit(oplus_hook_exit);

MODULE_LICENSE("GPL");




