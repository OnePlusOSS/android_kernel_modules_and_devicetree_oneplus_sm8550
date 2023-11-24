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
 * Basecode Created :        2020/10/11 Author: wt@zeku.com
 *
 */

#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include "include/main.h"
#include "include/ipc.h"
#include "include/hal_protocol.h"
#include "include/irq.h"
#include "include/exception.h"
#include "include/power.h"

#define SDIO_TRY_TIMES 3

#define PORT_SIZE	16
#define PORT_STR_LEN	15
#define PORT_SHIFT	8

#define SOC_CRG_WDT_TIMEOUT_WC 0x480086C8

char bus_port[PORT_SIZE][PORT_STR_LEN] = {
	"se_s_s5",
	"npu_s_s3",
	"npu_sysbuf",
	"mem",
	"mipi_tx",
	"se_m",
	"cpu_m2",
	"sysbuf_to_ddr",
	"npu_to_ddr",
	"npu",
	"isp2",
	"axi_damc",
	"npu0",
	"isp1",
	"isp2",
	"top3",
};

extern int explorer_hal_sync_read_internal(struct explorer_plat_data *epd, u32 regoffset, u8 *out);
extern int explorer_hal_sync_write_internal(struct explorer_plat_data *epd, u32 regoffset, u8 in);

int explorer_spec_exception_handler(struct explorer_plat_data *epd, struct exception_info *info);

exception_handle_table_entry exception_table[] =
{
	//{{EXCEPTION_XXX, EXCEPTION_YYY, EXCEPTION_ZZZ}, XXX_handle_func},
	{{EXCEPTION_NONE,}}
};

int exception_handle_func(struct explorer_plat_data *epd, struct exception_info *info)
{
	int ret = 0;

	explorer_spec_exception_handler(epd, info);

	ret = explorer_genl_mcast_data(epd, HAL_CMD_EXCEPTION2AP, info, sizeof(struct exception_info));
	if (ret < 0) {
		pr_err("%s, send exception netlink msg to userspace failed.\n", __func__);
	}

	return ret;
}

static void explorer_exception_process(struct explorer_plat_data *epd)
{
	int ret = 0;
	unsigned long stat = 0;
	int i = 0;
	u8 type0 = 0;
	u8 type1 = 0;
	u8 port0 = 0, port1 = 0, port2 = 0;
	struct exception_info info;
	u32 wdt_timeout_wc = 1;

	do {
		ret = explorer_hal_sync_read_internal(epd, AP_STAT_REG6_SDU, &type0);
		ret |= explorer_hal_sync_read_internal(epd, AP_STAT_REG7_SDU, &type1);
		i++;
	} while((ret != 0) && (i < SDIO_TRY_TIMES));

	if (SDIO_TRY_TIMES == i) {
		EXCEPTION_LOG_E("sdio bus error\n");
		return;
	}

	EXCEPTION_LOG_I("type0: 0x%x, type1: 0x%x\n", type0, type1);
	if (type1 & INT_BUS_MONITOR) {
		EXCEPTION_LOG_I("exception type: bushang\n");
		info.moduleId = EXCEPTION_BUS;

		ret = explorer_hal_sync_write_internal(epd, AP_CTRL_REG1_SDU, 0x0);
		ret |= explorer_hal_sync_write_internal(epd, AP_CTRL_REG1_SDU, 0x10);
		do {
			ret |= explorer_hal_sync_read_internal(epd, AP_STAT_REG5_SDU, &port0);
			i++;
		} while((ret != 0) && (i < SDIO_TRY_TIMES));

		if (SDIO_TRY_TIMES == i) {
			EXCEPTION_LOG_E("sdio bus error\n");
			goto DEFAULT_HANDLE;
		}

		i = 0;
		ret = explorer_hal_sync_write_internal(epd, AP_CTRL_REG1_SDU, 0x80);
		ret |= explorer_hal_sync_write_internal(epd, AP_CTRL_REG1_SDU, 0x90);
		do {
			ret |= explorer_hal_sync_read_internal(epd, AP_STAT_REG5_SDU, &port1);
			i++;
		} while((ret != 0) && (i < SDIO_TRY_TIMES));

		if (SDIO_TRY_TIMES == i) {
			EXCEPTION_LOG_E("sdio bus error\n");
			goto DEFAULT_HANDLE;
		}

		i = 0;
		ret = explorer_hal_sync_write_internal(epd, AP_CTRL_REG1_SDU, 0xa0);
		ret |= explorer_hal_sync_write_internal(epd, AP_CTRL_REG1_SDU, 0xb0);
		do {
			ret |= explorer_hal_sync_read_internal(epd, AP_STAT_REG5_SDU, &port2);
			i++;
		} while((ret != 0) && (i < SDIO_TRY_TIMES));

		if (SDIO_TRY_TIMES == i) {
			EXCEPTION_LOG_E("sdio bus error\n");
			goto DEFAULT_HANDLE;
		}

		EXCEPTION_LOG_I("AHB bus port stat: 0x%x, 0x%x, 0x%x\n", port0, port1, port2);

		if ((port0 & AHB_PORT_SDIO) && (port0 & AHB_PORT_MPU) && (port1 & AHB_PORT_LSAPB)) {
			EXCEPTION_LOG_E("bus monitor is accessable!\n");
		} else {
			EXCEPTION_LOG_E("bus monitor is not accessable!\n");
			goto DEFAULT_HANDLE;
		}

		explorer_hal_sync_read_internal(epd, AP_STAT_REG3_SDU, &port0);
		explorer_hal_sync_read_internal(epd, AP_STAT_REG4_SDU, &port1);

		EXCEPTION_LOG_I("bus monitor timeout: stat3: 0x%x, stat4: 0x%x\n",
				port0, port1);

		stat = (unsigned long)(port0 | (port1 << PORT_SHIFT));

		for_each_set_bit(i, &stat, 16) {
			EXCEPTION_LOG_I("hang port: %s\n", bus_port[i]);
		}

		info.majorType = port0;
		info.subType = port1;
	} else if (type0 & INT_WDT) {
		/* clear wdt bite irq flag */
		ret = explorer_hal_sync_write(epd, SOC_CRG_WDT_TIMEOUT_WC, &wdt_timeout_wc, 4);
		if (ret<0) {
			EXCEPTION_LOG_E("mask wdt bite interrupt flag in crg failed.\n");
		}

		EXCEPTION_LOG_I("exception type: wdtbite\n");
		info.moduleId = EXCEPTION_CPU;
		info.majorType = EXCEPTION_WDT_BITE;
		info.subType = EXCEPTION_WDT_BITE;
	} else {
		return;
	}

	info.level = EXCEPTION_FATAL;
	info.action = EXCEPTION_ACT_POWROFF;
DEFAULT_HANDLE:
	exception_handle_func(epd, &info);

	return;
}

int explorer_bus_hang_irq_handler(struct explorer_plat_data *epd)
{
	int ret = 0;
	u32 mask_int = INTC0_MASK_L_CCP3;

	/* clear bus hang irq flag */
	ret = explorer_hal_sync_write(epd, INTC0_MASK_L, &mask_int, 4);
	if (ret < 0) {
		EXCEPTION_LOG_E("mask bushang interrupt flag failed.\n");
		goto out;
	}

	explorer_exception_process(epd);

	EXCEPTION_LOG_I("done.\n");

out:
	return ret;
}

int explorer_wdt_bite_irq_handler(struct explorer_plat_data *epd)
{
	int ret = 0;
	u32 mask_int = INTC0_MASK_L_CCPO;

	ret = explorer_hal_sync_write(epd, INTC0_MASK_L, &mask_int, 4);
	if (ret<0) {
		EXCEPTION_LOG_E("mask wdt bite interrupt flag in intc failed.\n");
		goto out;
	}

	explorer_exception_process(epd);
	EXCEPTION_LOG_I("done.\n");

out:
	return ret;
}

int explorer_spec_exception_handler(struct explorer_plat_data *epd, struct exception_info *info)
{
	int ret = 0;
	int match_level = 0;
	struct exception_info curr_info;
	exception_handle_table_entry *curr_entry = NULL;
	exception_func_type func = NULL;

	for(curr_entry = exception_table, curr_info = curr_entry->info;
		curr_info.moduleId != EXCEPTION_NONE;)
	{
		if (curr_info.moduleId == info->moduleId &&
			curr_info.majorType == info->majorType &&
			curr_info.subType == info->subType) {
			func = curr_entry->handle;
			break;
		} else if (curr_info.moduleId == info->moduleId &&
					curr_info.majorType == info->majorType) {
			match_level = MID_MATCH;
			func = curr_entry->handle;
		} else if(curr_info.moduleId == info->moduleId) {
			if (match_level < MID_MATCH) {
				match_level = MIN_MATCH;
				func = curr_entry->handle;
			}
		}

		curr_entry++;
		curr_info = curr_entry->info;

	}

	if(func) {
		ret = func(epd, info);
		EXCEPTION_LOG_I("find spec func\n");
	} else {
		ret = -EINVAL;
		EXCEPTION_LOG_I("no spec func\n");
	}

	EXCEPTION_LOG_I("ret=%d done.\n", ret);

	return ret;
}

int explorer_exception_msg(struct explorer_plat_data *epd, struct hal_comm_data *comm_data)
{
	int ret = 0;
	struct exception_info info;

	info.moduleId = (uint8_t)(comm_data->data[0] & EXCEPTION_MASK);
	info.majorType = (uint8_t)((comm_data->data[0] >> EXCEPTION_MAJORTYPE_SHIF) & EXCEPTION_MASK);
	info.subType = (uint8_t)((comm_data->data[0] >> EXCEPTION_SUBTYPE_SHIF) & EXCEPTION_MASK);
	info.action = (uint8_t)((comm_data->data[0] >> EXCEPTION_ACTION_SHIF) & EXCEPTION_MASK_ACT);
	info.level = (uint8_t)((comm_data->data[0] >> EXCEPTION_LEVEL_SHIF) & EXCEPTION_MASK_LVL);

	/* Change some pmic exception parameters*/
	prepare_pmic_exception(epd, &info);

	EXCEPTION_LOG_W("exception:moduleID=%d, majorType=%d, subType=%d, action=%d, lvl=%d\n",
			info.moduleId, info.majorType, info.subType, info.action, info.level);

	if (info.moduleId >= EXCEPTION_MAX_MODULE) {
		EXCEPTION_LOG_E("Invaild Module ID: %d\n", info.moduleId);
		return -EINVAL;
	}

	if (info.moduleId == EXCEPTION_CPU) {
		PM_LOG_I("cancel heartbeat detect when hard fault\n");
		/* Disable heartbeat detect when cpu hardfault */
		epd->heartbeat_started = false;
		cancel_heartbeat_detect_work(epd);
	}

	ret = exception_handle_func(epd, &info);

	EXCEPTION_LOG_I("ret=%d done.\n", ret);

	return ret;
}

