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
 * Basecode Created :        2021/5/27 Author: wangyingju@zeku.com
 *
 */

#ifndef _MTK_AON_SOC_UTIL_H_
#define _MTK_AON_SOC_UTIL_H_
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>

#include "../../include/aon_sensor_common.h"

/* maximum number of device regulator */
#define CAM_SOC_MAX_REGULATOR       10

/* maximum number of device clock */
#define CAM_SOC_MAX_CLK             32

struct aon_hw_soc_info {
	struct platform_device         *pdev;
	struct device                  *dev;
	u32                            index;
	const char                     *dev_name;

	u32                            num_rgltr;
	const char                     *rgltr_name[CAM_SOC_MAX_REGULATOR];
	struct regulator               *rgltr[CAM_SOC_MAX_REGULATOR];

	u32                            num_clk;
	const char                     *clk_name[CAM_SOC_MAX_CLK];
	struct clk                     *clk[CAM_SOC_MAX_CLK];
	bool                           clk_enabled[CAM_SOC_MAX_CLK];
};

#endif /* _MTK_AON_SOC_UTIL_H_ */
