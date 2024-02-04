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

#ifndef _QCOM_AON_SOC_UTIL_H_
#define _QCOM_AON_SOC_UTIL_H_
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/debugfs.h>

#include "../../include/aon_sensor_common.h"

#define NO_SET_RATE                 -1
#define INIT_RATE                   -2

/* maximum number of device block */
#define CAM_SOC_MAX_BLOCK           7

/* maximum number of device base */
#define CAM_SOC_MAX_BASE            CAM_SOC_MAX_BLOCK

/* maximum number of device regulator */
#define CAM_SOC_MAX_REGULATOR       10

/* maximum number of device clock */
#define CAM_SOC_MAX_CLK             32

enum cam_vote_level {
	CAM_SUSPEND_VOTE,
	CAM_MINSVS_VOTE,
	CAM_LOWSVS_VOTE,
	CAM_SVS_VOTE,
	CAM_SVSL1_VOTE,
	CAM_NOMINAL_VOTE,
	CAM_NOMINALL1_VOTE,
	CAM_TURBO_VOTE,
	CAM_MAX_VOTE,
};

/**
 * struct aon_soc_gpio_data:   Information about the gpio pins
 *
 * @aon_gpio_common_tbl:       It is list of al the gpios present in gpios node
 * @aon_gpio_common_tbl_size:  It is equal to number of gpios prsent in
 *                             gpios node in DTSI
 * @aon_gpio_req_tbl           It is list of al the requesetd gpios
 * @aon_gpio_req_tbl_size:     It is size of requested gpios
 **/
struct aon_soc_gpio_data {
	struct gpio                    *cam_gpio_common_tbl;
	u8                             cam_gpio_common_tbl_size;
	struct gpio                    *cam_gpio_req_tbl;
	u8                             cam_gpio_req_tbl_size;
};

struct aon_soc_reg_map {
	void __iomem                   *mem_base;
	u32                            mem_cam_base;
	resource_size_t                size;
};

struct aon_soc_pinctrl_info {
	struct pinctrl                 *pinctrl;
	struct pinctrl_state           *gpio_state_active;
	struct pinctrl_state           *gpio_state_suspend;
};

struct aon_hw_soc_info {
	struct platform_device         *pdev;
	struct device                  *dev;
	u32                            hw_version;
	u32                            index;
	const char                     *dev_name;
	const char                     *irq_name;
	struct resource                *irq_line;
	void                           *irq_data;

	u32                            num_mem_block;
	const char                     *mem_block_name[CAM_SOC_MAX_BLOCK];
	u32                            mem_block_cam_base[CAM_SOC_MAX_BLOCK];
	struct resource                *mem_block[CAM_SOC_MAX_BLOCK];
	struct aon_soc_reg_map         reg_map[CAM_SOC_MAX_BASE];
	u32                            num_reg_map;
	u32                            reserve_mem;

	u32                            num_rgltr;
	const char                     *rgltr_name[CAM_SOC_MAX_REGULATOR];
	u32                            rgltr_ctrl_support;
	u32                            rgltr_min_volt[CAM_SOC_MAX_REGULATOR];
	u32                            rgltr_max_volt[CAM_SOC_MAX_REGULATOR];
	u32                            rgltr_op_mode[CAM_SOC_MAX_REGULATOR];
	u32                            rgltr_type[CAM_SOC_MAX_REGULATOR];
	struct regulator               *rgltr[CAM_SOC_MAX_REGULATOR];
	u32                            rgltr_delay[CAM_SOC_MAX_REGULATOR];

	u32                            use_shared_clk;
	u32                            num_clk;
	const char                     *clk_name[CAM_SOC_MAX_CLK];
	struct clk                     *clk[CAM_SOC_MAX_CLK];
	s32                            clk_rate[CAM_MAX_VOTE][CAM_SOC_MAX_CLK];
	s32                            prev_clk_level;
	s32                            src_clk_idx;
	bool                           clk_level_valid[CAM_MAX_VOTE];

	struct aon_soc_gpio_data       *gpio_data;
	struct aon_soc_pinctrl_info    pinctrl_info;

	struct dentry                  *dentry;
	u32                            clk_level_override;
	bool                           clk_control_enable;
	bool                           cam_cx_ipeak_enable;
	s32                            cam_cx_ipeak_bit;
};

#endif /* _QCOM_AON_SOC_UTIL_H_ */
