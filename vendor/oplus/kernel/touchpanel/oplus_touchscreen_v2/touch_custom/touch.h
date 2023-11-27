/***************************************************
 * File:touch.h
 * VENDOR_EDIT
 * Copyright (c)  2008- 2030  Oplus Mobile communication Corp.ltd.
 * Description:
 *             tp dev
 * Version:1.0:
 * Date created:2016/09/02
 * TAG: BSP.TP.Init
*/

#ifndef _TOUCH_H_
#define _TOUCH_H_

#include <linux/kernel.h>
#include <linux/module.h>
#include "../tp_devices.h"
#include "../touchpanel_common.h"


extern bool tp_judge_ic_match(char *tp_ic_name);
#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
extern int tp_util_get_vendor(struct hw_resource *hw_res, struct panel_info *panel_data);
#endif
extern int preconfig_power_control(struct touchpanel_data *ts);

extern int reconfig_power_control(struct touchpanel_data *ts);

extern void display_esd_check_enable_bytouchpanel(bool enable);
#endif /* _TOUCH_H_ */

