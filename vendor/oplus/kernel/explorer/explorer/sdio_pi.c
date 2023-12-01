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
 * Basecode Created :        2020/09/28 Author: wangman@zeku.com
 *
 */

#define EXCEPTION_SDIO_DEBUG
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/circ_buf.h>
#include <linux/kfifo.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/completion.h>

#include <linux/mmc/host.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/pm_runtime.h>

#include <linux/mmc/sdio.h>
#include <linux/platform_device.h>
#include "include/sdio_pi.h"
#include "include/main.h"
#include "include/irq.h"
#include "include/ipc.h"

#ifdef EXCEPTION_SDIO_DEBUG
#include "include/exception.h"
#include "include/power.h"
#endif

struct explorer_sdio_ios {
	unsigned int	clock;			/* clock rate */
	unsigned char	bus_width;		/* data bus width */
	unsigned char	timing;			/* timing specification used */
};

struct explorer_plat_data *epd_save;

#define BLOCK_MODE	1
#define BYTE_MODE	0

/* for regoffset */
#define DEVICE_BLOCK_ADDR_MASK	0x03fffe00
#define DEVICE_BYTE_ADDR_MASK	0x0001ffff
#define DEVICE_MATCH_ADDR_START_BIT(k)	(12 + k)
#define DEVICE_BLOCK_REGOFFSET_SHIFT	9
#define DEVICE_BYTE_REGOFFSET_SHIFT		0

/* for function number */
#define DEVICE_BLOCK_FUNC_MASK		0x1c000000
#define DEVICE_BYTE_FUNC_MASK		0x000e0000
#define DEVICE_BLOCK_FUNC_SHIFT		26
#define DEVICE_BYTE_FUNC_SHIFT		17

/* for atu */
#define INDEX_OUT_OF_RANGE	65
#define DEVICE_BLOCK_ATU_NUM(k)		(1 << (DEVICE_BLOCK_FUNC_SHIFT + 3 - DEVICE_MATCH_ADDR_START_BIT(k)))
#define DEVICE_BYTE_ATU_NUM(k)		(1 << (DEVICE_BYTE_FUNC_SHIFT + 3 - DEVICE_MATCH_ADDR_START_BIT(k)))
#define DEVICE_BYTE_ATU_START_IDX		0
#define DEVICE_BLOCK_ATU_START_IDX(k)	(DEVICE_BLOCK_ATU_NUM(k))

/* for remap */
#define DEVICE_REMAP_ADDR_MASK(k)	(0xffffffff << DEVICE_MATCH_ADDR_START_BIT(k))

#define MIN(X, Y) ((X) < (Y) ? (X) : (Y))

#define SDIO_RETRY_TIMES 3

extern int explorer_bootstage_pre(struct explorer_plat_data *epd);

static struct atu_remap_item atu_remap_table[64] = {
	{0x3,0x40020000},
	{0x8003,0x40028000},
	{0x10003,0x48000000},
	{0x18003,0x48008000},
	{0x20003,0x48018000},
	{0x28003,0x48020000},
	{0x30003,0x48030000},
	{0x38003,0x48040000},
	{0x40003,0x48048000},
	{0x48003,0x48050000},
	{0x50003,0x48058000},
	{0x58003,0x50000000},
	{0x60003,0x50100000},
	{0x68003,0x58100000},	/* 13 */
	{0x70003,0x178000},
	{0x78003,0x0},
	{0x80003,0x0},
	{0x88003,0x0},
	{0x90003,0x0},
	{0x98003,0x0},
	{0xA0003,0x0},
	{0xA8003,0x0},
	{0xB0003,0x0},
	{0xB8003,0x0},
	{0xC0003,0x0},
	{0xC8003,0x0},
	{0xD0003,0x0},
	{0xD8003,0x0},
	{0xE0003,0x0},
	{0xE8003,0x0},
	{0xF0003,0x0},
	{0xF8003,0x0},
	{0x2C,0x20000000},
	{0x100002C,0x21000000},
	{0x200002C,0x22000000},
	{0x300002C,0x23000000},
	{0x400002C,0x24000000},
	{0x500002C,0x25000000},
	{0x600002C,0x26000000},
	{0x700002C,0x27000000},
	{0x800002C,0x28000000},
	{0x900002C,0x29000000},
	{0xA00002C,0x2A000000},
	{0xB00002C,0x2B000000},
	{0xC00002C,0x2C000000},
	{0xD00002C,0x2D000000},
	{0xE00002C,0x2E000000},
	{0xF00002C,0x2F000000},
	{0x1000002C,0x0},
	{0x1100002C,0x8000000},
	{0x1200002C,0x40000000},
	{0x1300002C,0x44000000},
	{0x1400002C,0x45000000},
	{0x1500002C,0x46000000},
	{0x1600002C,0x47000000},
	{0x1700002C,0x58000000},
	{0x1800002C,0x0},
	{0x1900002C,0x0},
	{0x1A00002C,0x0},
	{0x1B00002C,0x0},
	{0x1C00002C,0x0},
	{0x1D00002C,0x0},
	{0x1E00002C,0x0},
	{0x1F00002C,0x0},
};

/* Abstract layer of standard sdio API */
static int sdio_explorer_claim_func(struct sdio_func *pfunc)
{
	if (unlikely(!pfunc)) {
		pr_err("%s, NO sdio device", __func__);
		return -ENODEV;
	}
	sdio_claim_host(pfunc);
	return 0;
}

static void sdio_explorer_release_func(struct sdio_func *pfunc)
{
	if (unlikely(!pfunc)) {
		pr_err("%s, NO sdio device", __func__);
		return;
	}
	sdio_release_host(pfunc);
	return;
}

static int cmd53_abort(struct sdio_func *func)
{
	struct mmc_card *card = func->card;
	struct mmc_host *host = card->host;
	struct mmc_command cmd = {};
	struct explorer_plat_data *explorer_data;
	struct device *explorer_device;
	int ret = 0;

	/* find explorer platform device and set its drvdata */
	explorer_device = bus_find_device_by_name(&platform_bus_type, NULL, "soc:zeku,explorer@0");
	if (!explorer_device) {
		pr_err("zeku:%s, explorer platform device not found.\n", __func__);
		return -ENODEV;
	}
	explorer_data = dev_get_drvdata(explorer_device);
	pr_debug("%s:send cmd53 abort\n", __func__);
	cmd.opcode = SD_IO_RW_DIRECT;
	cmd.arg = 0x80000000 ;
	cmd.arg |= 0 << 28;
	cmd.arg |=  0x00000000;
	cmd.arg |= 0x06 << 9;
	cmd.arg |= 0x01;
	cmd.flags = MMC_RSP_SPI_R5 | MMC_RSP_R5 | MMC_CMD_AC;
	ret = mmc_wait_for_cmd(host, &cmd, 0);
	return ret;
}

static int sdio_send_cmd7(struct sdio_func *func)
{
	struct mmc_card *card = NULL;
	struct mmc_host *host = NULL;
	struct mmc_command cmd = {};
	struct explorer_plat_data *explorer_data = NULL;
	struct device *explorer_device = NULL;
	int err;

	if (!func)
		return -ENODEV;

	card = func->card;

	if (!card)
		return -ENODEV;

	host = card->host;

	/* find explorer platform device and set its drvdata */
	explorer_device = bus_find_device_by_name(&platform_bus_type, NULL, "soc:zeku,explorer@0");
	if (!explorer_device) {
		pr_err("zeku:%s, explorer platform device not found.\n", __func__);
		return -ENODEV;
	}

	explorer_data = dev_get_drvdata(explorer_device);

	pr_debug("%s: explorer send cmd7\n", __func__);
	cmd.opcode = MMC_SELECT_CARD;

	if (card) {
		cmd.arg = card->rca << 16;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
	} else {
		cmd.arg = 0;
		cmd.flags = MMC_RSP_NONE | MMC_CMD_AC;
	}

	err = mmc_wait_for_cmd(host, &cmd, CMD_RETRIES);
	return err;
}

static int check_alive(struct sdio_func *func, int err_state, int iscmd53)
{
	int err;
	int retry = 1;
	struct explorer_plat_data *explorer_data = NULL;
	struct device *explorer_device = NULL;
#ifdef EXCEPTION_SDIO_DEBUG
	struct exception_info *info = NULL;
	bool explorer_power_on;
#endif
	/* find explorer platform device and set its drvdata */
	explorer_device = bus_find_device_by_name(&platform_bus_type, NULL, "soc:zeku,explorer@0");
	if (!explorer_device) {
		pr_err("zeku:%s, explorer platform device not found.\n", __func__);
		return -ENODEV;
	}

	explorer_data = dev_get_drvdata(explorer_device);

#ifdef EXCEPTION_SDIO_DEBUG

	info = kmalloc(sizeof(struct exception_info), GFP_KERNEL);
	if (!info) {
		pr_err("%s: explorer sdio no memory for exception", __func__);
		return -ENOMEM;
	}
	info->moduleId = EXCEPTION_SDIO;
	explorer_power_on = get_explorer_on_status(explorer_data);
#endif

	if (err_state == -ETIMEDOUT) {
		pr_err("%s: explorer sdio transfers timeout, err: %d\n", __func__, err_state);
#ifdef EXCEPTION_SDIO_DEBUG
		info->subType = EXCEPTION_SDIO_TIMEOUT_RECOVERABLE;
#endif
	} else if (err_state == -EILSEQ) {
		pr_err("%s: explorer sdio transfers crc failed, err: %d\n", __func__, err_state);
#ifdef EXCEPTION_SDIO_DEBUG
		info->subType = EXCEPTION_SDIO_CRC_ERR_RECOVERABLE;
#endif
	} else {
		pr_err("%s: explorer sdio no dev error or sdio device busy, err: %d\n", __func__, err_state);
#ifdef EXCEPTION_SDIO_DEBUG
		info->subType = -(err_state);
		info->majorType = EXCEPTION_SDIO_DEV_ERR;
		info->action = EXCEPTION_ACT_NONE;
		info->level = EXCEPTION_ERROR;
#endif
		if (explorer_power_on) {
#ifdef EXCEPTION_SDIO_DEBUG
			exception_handle_func(explorer_data, info);
#endif
			retry = 1;
		}
		/* 1 means no need to retry */
		return retry;
	}

	if (iscmd53) {
		pr_err("%s: An error occurred in explorer sdio sending cmd53, abort", __func__);
#ifdef EXCEPTION_SDIO_DEBUG
		info->majorType = EXCEPTION_SDIO_CMD53_ERR;
		info->action = EXCEPTION_ACT_NONE;
		info->level = EXCEPTION_WARNING;
#endif

		/* 2 means need to retry */
		retry = 2;
		err = cmd53_abort(func);
		if (err)
			goto sendcmd7;
	} else {
		pr_err("%s: An error occurred in explorer sdio sending cmd52", __func__);
#ifdef EXCEPTION_SDIO_DEBUG
		info->majorType = EXCEPTION_SDIO_CMD52_ERR;
		info->action = EXCEPTION_ACT_NONE;
		info->level = EXCEPTION_WARNING;
#endif
		retry = 1;
	}
	pr_err("%s: explorer sdio alived, cmd53 abort successfully\n", __func__);
#ifdef EXCEPTION_SDIO_DEBUG
	if (explorer_power_on) {
		exception_handle_func(explorer_data, info);
	}
	kfree(info);
#endif
	return retry;

sendcmd7:
	/* explorer crash, need reboot and report to hal */
	explorer_data->hw_probe_done = false;
	atomic_set(&(explorer_data->ebs.flashable), 0);
	retry = 1;
#ifdef EXCEPTION_SDIO_DEBUG
	info->subType = EXCEPTION_SDIO_TIMEOUT;
	info->action = EXCEPTION_ACT_POWROFF;
	info->level = EXCEPTION_ERROR;
#endif

	err = sdio_send_cmd7(func);
	if (err)
		pr_err("%s: explorer sdio failed to send cmd7\n", __func__);
	else
		pr_err("%s: explorer sdio alived, but cmd52 failed\n", __func__);

#ifdef EXCEPTION_SDIO_DEBUG
	/* exception handle */
	if (explorer_power_on) {
		exception_handle_func(explorer_data, info);
	}
	kfree(info);
#endif
	return retry;
}

#define MMC_CARD_REMOVED	(1<<4)

int sdio_check_card(void)
{
	int ret = 0;

	mutex_lock(&epd_save->comm_lock);
	if ((epd_save->sdio_data == NULL) || (epd_save->sdio_data->func->card->state & MMC_CARD_REMOVED) || (epd_save->sdio_data->func->card->host->ops->get_cd(epd_save->sdio_data->func->card->host) == 0))
		ret = -ENODEV;
	mutex_unlock(&epd_save->comm_lock);
	return ret;
}

u8 sdio_explorer_writeb_readb(struct sdio_func *func, u8 write_byte,
	unsigned int addr, int *err_ret)
{
	u8 val;
	int retry = 0;
	int times = 0;
needretry:
	val = sdio_writeb_readb(func, write_byte, addr, err_ret);
	if (*err_ret) {
		pr_err("[%s:%d]sdio error: addr:0x%x, count:%d\n", __func__, __LINE__, addr, write_byte);
		retry = check_alive(func, *err_ret, 0);
	}
	if (retry == 2 && times < SDIO_RETRY_TIMES) {
		times++;
		pr_info("explorer:zeku, %s retry times: %d", __func__, times);
		goto needretry;
	} else if (retry == 2 && times >= SDIO_RETRY_TIMES) {
		pr_err("explorer zeku: %s retransmit more than 3 times", __func__);
		return val;
	}

	times = 0;
	return val;
}

u8 sdio_explorer_readb(struct sdio_func *func, unsigned int addr, int *err_ret)
{
	u8 val;
	int retry = 0;
	int times = 0;

needretry:
	val = sdio_readb(func, addr, err_ret);
	if (*err_ret) {
		pr_err("[%s:%d]sdio error: addr:0x%x\n", __func__, __LINE__, addr);
		retry = check_alive(func, *err_ret, 0);
	}
	if (retry == 2 && times < SDIO_RETRY_TIMES) {
		times++;
		pr_info("explorer:zeku, %s retry times: %d", __func__, times);
		goto needretry;
	} else if (retry == 2 && times >= SDIO_RETRY_TIMES) {
		pr_err("explorer zeku: %s retransmit more than 3 times", __func__);
		return val;
	}
	times = 0;
	return val;
}

void sdio_explorer_writeb(struct sdio_func *func, u8 b, unsigned int addr, int *err_ret)
{
	int retry = 0;
	int times = 0;

needretry:
	sdio_writeb(func, b, addr, err_ret);
	if (*err_ret) {
		pr_err("[%s:%d]sdio error: addr:0x%x, b:%d\n", __func__, __LINE__, addr, b);
		retry = check_alive(func, *err_ret, 0);
	}
	if (retry == 2 && times < SDIO_RETRY_TIMES) {
		times++;
		pr_info("explorer:zeku, %s retry times: %d", __func__, times);
		goto needretry;
	} else if (retry == 2 && times >= SDIO_RETRY_TIMES) {
		pr_err("explorer zeku: %s retransmit more than 3 times", __func__);
		return;
	}

	times = 0;
}

int sdio_asr_io_polling_func(struct sdio_func *func)
{
	int ret = 0;
	unsigned char reg = 0;
	unsigned long timeout = 0;
	struct explorer_sdio_data *pdata;
	if (!func)
		return -EINVAL;

	pdata = sdio_get_drvdata(func);
	pr_debug("zeku: %s, polling device %s...\n", __func__, sdio_func_id(func));

	timeout = jiffies + msecs_to_jiffies(func->enable_timeout);

	while(1) {
		ret = sdio_read_register(pdata, SDIO_FN1R_C2H_INT_EVENT, &reg, FUNC1);
		if (ret)
			goto err;

		if (reg & SDIO_FN1R_IO_READY)
			break;
		ret = -ETIME;
		if (time_after(jiffies, timeout))
			goto err;
	}

	pr_debug("zeku: %s, enabling device %s\n", __func__, sdio_func_id(func));

	return ret;
err:
	pr_err("zeku: %s, failed to enable device, err: %d", __func__, ret);
	return ret;
}

int sdio_asr_io_enable_func(struct sdio_func *func)
{
	int ret = 0;
	unsigned char reg = 0;
	unsigned long timeout = 0;
	struct explorer_sdio_data *pdata = NULL;

	if (!func)
		return -EINVAL;

	pdata = sdio_get_drvdata(func);
	pr_debug("zeku: %s, Enabling device %s...\n", __func__, sdio_func_id(func));

	ret = sdio_read_register(pdata, SDIO_CCCR_IOEx, &reg, FUNC0);
	if (ret)
		goto err;

	reg |= 0xfe;

	ret = sdio_write_register(pdata, SDIO_CCCR_IOEx, reg, FUNC0);
	if (ret)
		goto err;

	timeout = jiffies + msecs_to_jiffies(func->enable_timeout);

	while(1) {
		ret = sdio_read_register(pdata, SDIO_FN1R_C2H_INT_EVENT, &reg, FUNC1);
		if (ret)
			goto err;

		if (reg & SDIO_FN1R_IO_READY)
			break;
		ret = -ETIME;
		if (time_after(jiffies, timeout))
			goto err;
	}

	pr_debug("zeku: %s, enabling device %s\n", __func__, sdio_func_id(func));

	return ret;
err:
	pr_err("zeku: %s, failed to enable device, err: %d", __func__, ret);
	return ret;
}

int sdio_explorer_enable_func(struct sdio_func *func)
{
	int ret = 0;

	ret = sdio_asr_io_enable_func(func);
	if (ret) {
		pr_err("[%s:%d]sdio error\n", __func__, __LINE__);
		check_alive(func, ret, 0);
	}
	return ret;
}

int sdio_explorer_memcpy_fromio(struct sdio_func *func, void *dst,
	unsigned int addr, int count)
{
	int ret = 0;
	int retry = 0;
	int times = 0;

needretry:
	ret = sdio_memcpy_fromio(func, dst, addr, count);
	if (ret) {
		pr_err("[%s:%d]sdio error: addr:0x%x, count:%d\n", __func__, __LINE__, addr, count);
		retry = check_alive(func, ret, 1);
	}
	if (retry == 2 && times < SDIO_RETRY_TIMES) {
		times++;
		pr_info("explorer:zeku, %s retry times: %d", __func__, times);
		goto needretry;
	} else if (retry == 2 && times >= SDIO_RETRY_TIMES) {
		pr_err("explorer zeku: %s retransmit more than 3 times", __func__);
		return ret;
	}

	times = 0;

	return ret;
}

int sdio_asr_io_disable_func(struct sdio_func *func)
{
	int ret = 0;
	unsigned char reg = 0;
	struct explorer_sdio_data *pdata = NULL;

	if (!func)
		return -EINVAL;

	pdata = sdio_get_drvdata(func);
	pr_debug("zeku: %s, Disabling device %s...\n", __func__, sdio_func_id(func));

	ret = sdio_read_register(pdata, SDIO_CCCR_IOEx, &reg, FUNC0);
	if (ret)
		goto err;

	reg &= 0x1;

	ret = sdio_write_register(pdata, SDIO_CCCR_IOEx, reg, FUNC0);
	if (ret)
		goto err;

	pr_debug("zeku: %s, disabling device %s\n", __func__, sdio_func_id(func));

	return ret;
err:
	pr_err("zeku: %s, failed to disable device. err: %d", __func__, ret);
	return ret;
}



int sdio_explorer_disable_func(struct sdio_func *func)
{
	int ret = 0;

	ret = sdio_asr_io_disable_func(func);
	if (ret) {
		pr_err("[%s:%d]sdio error\n", __func__, __LINE__);
		check_alive(func, ret, 0);
	}
	return ret;
}

int sdio_explorer_memcpy_toio(struct sdio_func *func, unsigned int addr,
	void *src, int count)
{
	int ret = 0;
	int retry = 0;
	int times = 0;

needretry:
	ret = sdio_memcpy_toio(func, addr, src, count);
	if (ret) {
		pr_err("[%s:%d]sdio error: addr:0x%x, count:%d\n", __func__, __LINE__, addr, count);
		retry = check_alive(func, ret, 1);
	}
	if (retry == 2 && times < SDIO_RETRY_TIMES) {
		times++;
		pr_info("explorer:zeku, %s retry times: %d", __func__, times);
		goto needretry;
	} else if (retry == 2 && times >= SDIO_RETRY_TIMES) {
		pr_err("explorer zeku: %s retransmit more than 3 times", __func__);
		return ret;
	}

	times = 0;


	return ret;
}

int sdio_explorer_writesb(struct sdio_func *func, unsigned int addr, void *src,
	int count)
{
	int ret = 0;
	int retry = 0;
	int times = 0;

needretry:
	ret = sdio_writesb(func, addr, src, count);
	if (ret) {
		pr_err("[%s:%d]sdio error: addr:0x%x, count:%d\n", __func__, __LINE__, addr, count);
		retry = check_alive(func, ret, 1);
	}
	if (retry == 2 && times < SDIO_RETRY_TIMES) {
		times++;
		pr_info("explorer:zeku, %s retry times: %d", __func__, times);
		goto needretry;
	} else if (retry == 2 && times >= SDIO_RETRY_TIMES) {
		pr_err("explorer zeku: %s retransmit more than 3 times", __func__);
		return ret;
	}

	times = 0;


	return ret;
}

int sdio_explorer_set_block_size(struct sdio_func *func, unsigned blksz)
{
	int ret = 0;

	ret = sdio_set_block_size(func, blksz);
	if (ret) {
		pr_err("[%s:%d]sdio error\n", __func__, __LINE__);
		check_alive(func, ret, 0);
	}
	return ret;
}

/* for atu */
static int sdio_update_remap(struct sdio_func *func, u32 remap_addr, u32 index, unsigned funcnum)
{
	int ret = 0;
	u8 val_cmd52;
	struct explorer_sdio_data *pdata = sdio_get_drvdata(func);
	unsigned int dwold_funcnum = func->num;

	if (index < 0 || index > 63) {
		pr_err("%s, invalid index", __func__);
		return -EINVAL;
	}

	if (funcnum < 0 || funcnum > 7) {
		pr_err("%s, invalid function number", __func__);
		return -EINVAL;
	}


	pdata->remap_reg[index].value = remap_addr;

	ret = sdio_explorer_claim_func(func);
	if (ret)
		return ret;

	pr_debug("zeku: %s update remap\n", __func__);
	/*modify func num*/
	func->num = funcnum;
	/* remap */
	val_cmd52 = sdio_explorer_writeb_readb(func, pdata->remap_reg[index].item.remap0, 0xa00 + 0x4 * index, &ret);
	if (ret) {
		pr_err("zeku: error (%d) when configuring remap[7:0] of remap register %d", ret, index);
		goto writecmd52failed;
	}
	pr_debug("zeku: set remap 0x%x to 0x%x\n", (0xa00 + 0x4 * index),val_cmd52);
	val_cmd52 = sdio_explorer_writeb_readb(func, pdata->remap_reg[index].item.remap1, 0xa01 + 0x4 * index, &ret);
	if (ret) {
		pr_err("zeku: error (%d) when configuring remap[15:8] of remap register %d", ret, index);
		goto writecmd52failed;
	}
	pr_debug("zeku: set remap 0x%x to 0x%x\n", (0xa01 + 0x4 * index),val_cmd52);
	val_cmd52 = sdio_explorer_writeb_readb(func, pdata->remap_reg[index].item.remap2, 0xa02 + 0x4 * index, &ret);
	if (ret) {
		pr_err("zeku: error (%d) when configuring remap[23:16] of remap register %d", ret, index);
		goto writecmd52failed;
	}
	pr_debug("zeku: set remap 0x%x to 0x%x\n", (0xa02 + 0x4 * index),val_cmd52);
	val_cmd52 = sdio_explorer_writeb_readb(func, pdata->remap_reg[index].item.remap3, 0xa03 + 0x4 * index, &ret);
	if (ret) {
		pr_err("zeku: error (%d) when configuring remap[31:24] of remap register %d", ret, index);
		goto writecmd52failed;
	}
	pr_debug("zeku: set remap 0x%x to 0x%x\n", (0xa03 + 0x4 * index),val_cmd52);

writecmd52failed:
	/*recover original func num*/
	func->num = dwold_funcnum;
	sdio_explorer_release_func(func);
	return ret;
}

static int sdio_set_atu_remap(struct sdio_func *func, u32 atu_val, u32 remap_addr, u32 index, unsigned funcnum)
{
	int ret = 0;
	//u8 val_cmd52;
	struct explorer_sdio_data *pdata = sdio_get_drvdata(func);
	//unsigned int dwold_funcnum = func->num;

	if (index < 0 || index > 63) {
		pr_err("%s, invalid index", __func__);
		return -EINVAL;
	}

	if (funcnum < 0 || funcnum > 7) {
		pr_err("%s, invalid function number", __func__);
		return -EINVAL;
	}

	pdata->atu_reg[index].value = atu_val;
	pdata->remap_reg[index].value = remap_addr;
#if 0
	ret = sdio_explorer_claim_func(func);
	if (ret)
		return ret;
	pr_debug("zeku: %s configure atu and remap\n", __func__);
	/*modify func num*/
	func->num = funcnum;
	/* atu */
	val_cmd52 = sdio_explorer_writeb_readb(func, pdata->atu_reg[index].item.atu0, 0x900 + 0x4 * index, &ret);
	if (ret) {
		pr_err("zeku: error (%d) when configuring atu[7:0] of atu register %d", ret, index);
		goto writecmd52failed;
	}
	pr_debug("zeku: set atu 0x%x to 0x%x\n", (0x900 + 0x4 * index),val_cmd52);
	val_cmd52 = sdio_explorer_writeb_readb(func, pdata->atu_reg[index].item.atu1, 0x901 + 0x4 * index, &ret);
	if (ret) {
		pr_err("zeku: error (%d) when configuring atu[15:8] of atu register %d", ret, index);
		goto writecmd52failed;
	}
	pr_debug("zeku: set atu 0x%x to 0x%x\n",(0x901 + 0x4 * index),val_cmd52);
	val_cmd52 = sdio_explorer_writeb_readb(func, pdata->atu_reg[index].item.atu2, 0x902 + 0x4 * index, &ret);
	if (ret) {
		pr_err("zeku: error (%d) when configuring atu[23:16] of atu register %d", ret, index);
		goto writecmd52failed;
	}
	pr_debug("zeku: set atu 0x%x to 0x%x\n",(0x902 + 0x4 * index),val_cmd52);
	val_cmd52 = sdio_explorer_writeb_readb(func, pdata->atu_reg[index].item.atu3, 0x903 + 0x4 * index, &ret);
	if (ret) {
		pr_err("zeku: error (%d) when configuring atu[31:24] of atu register %d", ret, index);
		goto writecmd52failed;
	}
	pr_debug("zeku: set atu 0x%x to 0x%x\n",(0x903 + 0x4 * index), val_cmd52);

	/* remap */
	val_cmd52 = sdio_explorer_writeb_readb(func, pdata->remap_reg[index].item.remap0, 0xa00 + 0x4 * index, &ret);
	if (ret) {
		pr_err("zeku: error (%d) when configuring remap[7:0] of remap register %d", ret, index);
		goto writecmd52failed;
	}
	pr_debug("zeku: set remap 0x%x to 0x%x\n", (0xa00 + 0x4 * index),val_cmd52);
	val_cmd52 = sdio_explorer_writeb_readb(func, pdata->remap_reg[index].item.remap1, 0xa01 + 0x4 * index, &ret);
	if (ret) {
		pr_err("zeku: error (%d) when configuring remap[15:8] of remap register %d", ret, index);
		goto writecmd52failed;
	}
	pr_debug("zeku: set remap 0x%x to 0x%x\n", (0xa01 + 0x4 * index),val_cmd52);
	val_cmd52 = sdio_explorer_writeb_readb(func, pdata->remap_reg[index].item.remap2, 0xa02 + 0x4 * index, &ret);
	if (ret) {
		pr_err("zeku: error (%d) when configuring remap[23:16] of remap register %d", ret, index);
		goto writecmd52failed;
	}
	pr_debug("zeku: set remap 0x%x to 0x%x\n", (0xa02 + 0x4 * index),val_cmd52);
	val_cmd52 = sdio_explorer_writeb_readb(func, pdata->remap_reg[index].item.remap3, 0xa03 + 0x4 * index, &ret);
	if (ret) {
		pr_err("zeku: error (%d) when configuring remap[31:24] of remap register %d", ret, index);
		goto writecmd52failed;
	}
	pr_debug("zeku: set remap 0x%x to 0x%x\n", (0xa03 + 0x4 * index),val_cmd52);

	pr_info("zeku:%s,func=%d,old=%d,input=%d\n",__func__,func->num,dwold_funcnum,funcnum);

writecmd52failed:
	/*recover original func num*/
	func->num = dwold_funcnum;
	sdio_explorer_release_func(func);
#endif
	return ret;
}

/* for sdio cmd52 */
int sdio_read_register(struct explorer_sdio_data *sdio_data, unsigned int regoffset, unsigned char *out, unsigned int funcnum)
{
	int ret = 0;
	struct sdio_func *pfunc = NULL;
	unsigned int old_funcnum = 0;

	if (!sdio_data) {
		return -ENODEV;
	}
	pfunc = sdio_data->func;
	old_funcnum = pfunc->num;

	if (NULL == out || funcnum < 0 || funcnum > 7) {
		pr_err("%s, input data is null, or invalid function number", __func__);
		return -EINVAL;
	}
	pr_debug("zeku: %s send cmd52 to read register, regoffset=0x%x, out=0x%x", __func__, regoffset, *out);

	ret = sdio_explorer_claim_func(pfunc);
	if (ret)
		return ret;

	pfunc->num = funcnum;
	*out = sdio_explorer_readb(pfunc, regoffset, &ret);
	if (ret) {
		pr_err("zeku: read regoffset:0x%x, error:0x%x", regoffset, ret);
		goto rfailed;
	}
	pr_debug("zeku: read regoffset:0x%x, value:0x%x",regoffset, *out);
rfailed:
	pfunc->num = old_funcnum;

	sdio_explorer_release_func(pfunc);

	return ret;
}

int sdio_write_register(struct explorer_sdio_data *sdio_data, unsigned int regoffset, unsigned char input, unsigned int funcnum)
{
	int ret = 0;
	struct sdio_func *pfunc = NULL;
	unsigned int old_funcnum = 0;
	pr_debug("zeku: %s send cmd52 to write register, regoffset=0x%x, input=0x%x, old_funcnum=0x%x", __func__, regoffset, input, old_funcnum);

	if (!sdio_data) {
		return -ENODEV;
	}
	pfunc = sdio_data->func;
	old_funcnum = pfunc->num;

	if (funcnum < 0 || funcnum > 7) {
		pr_err("%s, invalid function number", __func__);
		return -EINVAL;
	}

	ret = sdio_explorer_claim_func(pfunc);
	if (ret)
		return ret;

	pfunc->num = funcnum;
	sdio_explorer_writeb(pfunc, input, regoffset, &ret);
	if (ret) {
		pr_err("zeku: write regoffset:0x%x, value:0x%x, error: %d", regoffset, input, ret);
		goto wfailed;
	}
	pr_debug("zeku: write regoffset:0x%x, value:0x%x", regoffset, input);
wfailed:
	pfunc->num = old_funcnum;

	sdio_explorer_release_func(pfunc);

	return ret;
}

int sdio_read_register_no_claim(struct explorer_sdio_data *sdio_data, unsigned int regoffset, unsigned char *out, unsigned int funcnum)
{
	int ret = 0;
	struct sdio_func *pfunc = NULL;
	unsigned int old_funcnum = 0;

	if (!sdio_data)
		return -ENODEV;

	pfunc = sdio_data->func;
	old_funcnum = pfunc->num;

	if (NULL == out || 0 > funcnum || 7 < funcnum) {
		pr_err("%s, input data is null, or invalid function number", __func__);
		return -EINVAL;
	}
	pr_debug("zeku: %s send cmd52 to read register, regoffset=0x%x, out=0x%x", __func__, regoffset, *out);

	pfunc->num = funcnum;
	*out = sdio_explorer_readb(pfunc, regoffset, &ret);
	if (ret) {
		pr_err("zeku: read regoffset:0x%x, error:0x%x", regoffset, ret);
		goto rfailed;
	}
	pr_debug("zeku: read regoffset:0x%x, value:0x%x", regoffset, *out);
rfailed:
	pfunc->num = old_funcnum;

	return ret;
}

int sdio_write_register_no_claim(struct explorer_sdio_data *sdio_data, unsigned int regoffset, unsigned char input, unsigned int funcnum)
{
	int ret = 0;
	struct sdio_func *pfunc = NULL;
	unsigned int old_funcnum = 0;

	pr_debug("zeku: %s send cmd52 to write register, regoffset=0x%x, input=0x%x, old_funcnum=0x%x", __func__, regoffset, input, old_funcnum);

	if (!sdio_data)
		return -ENODEV;

	pfunc = sdio_data->func;
	old_funcnum = pfunc->num;

	if (funcnum < 0 || funcnum > 7) {
		pr_err("%s, invalid function number", __func__);
		return -EINVAL;
	}

	pfunc->num = funcnum;
	sdio_explorer_writeb(pfunc, input, regoffset, &ret);
	if (ret) {
		pr_err("zeku: write regoffset:0x%x, value:0x%x, error: %d", regoffset, input, ret);
		goto wfailed;
	}
	pr_debug("zeku: write regoffset:0x%x, value:0x%x", regoffset, input);
wfailed:
	pfunc->num = old_funcnum;

	return ret;
}

static void free_current_list(struct list_head *head, int *num)
{
	struct current_atu_list *ptr = NULL;
	struct current_atu_list *next = NULL;

	if (!head)
		return;

	if (list_empty(head))
		return;

	list_for_each_entry_safe(ptr, next, head, list) {
		kfree(ptr);
		*num = *num - 1;
		break;
	}
}

/* create node and add it in the head of current list */
static int put_index_to_current_list(struct explorer_sdio_data *sdio_data, unsigned int index, int byte)
{
	struct current_atu_list *new = NULL;
	struct list_head *cur = NULL;
	unsigned int *cur_atu_num = NULL;


	pr_debug("explorer:%s, begin", __func__);
	cur = byte?(&sdio_data->byte_current_list):(&sdio_data->block_current_list);
	cur_atu_num = GET_CUR_ATU_NUM_ADDR(sdio_data, byte);


	new = kmalloc(sizeof(struct current_atu_list), GFP_KERNEL);
	if (!new) {
		return -ENOMEM;
	}

	new->index = index;
	INIT_LIST_HEAD(&new->list);

	pr_debug("explorer:%s, create node", __func__);
	list_add(&new->list, cur);
	*cur_atu_num = *cur_atu_num + 1;
	pr_debug("explorer:%s, done, byte num:%d, block num:%d", __func__, sdio_data->byte_current_atu_num, sdio_data->block_current_atu_num);
	return 0;
}

/* Get the tail node from current list, and remove it  */
static unsigned int get_index_from_current_list(struct explorer_sdio_data *sdio_data, int byte)
{
	unsigned int index = 0;
	struct current_atu_list *ptr = NULL;
	struct list_head *cur = NULL;
	unsigned int *cur_atu_num = NULL;

	pr_debug("explorer:%s, begin", __func__);
	if (byte && list_empty(&sdio_data->byte_current_list)) {
		pr_err("explorer:%s, byte current list is NULL", __func__);
		return -EINVAL;
	}

	if (!byte && list_empty(&sdio_data->block_current_list)) {
		pr_err("explorer:%s, block current list is NULL", __func__);
		return -EINVAL;
	}

	cur = byte?(&sdio_data->byte_current_list):(&sdio_data->block_current_list);
	cur_atu_num = GET_CUR_ATU_NUM_ADDR(sdio_data, byte);

	/* get index from the tail node */
	cur = cur->prev;
	ptr = list_entry(cur, struct current_atu_list, list);
	index = ptr->index;
	pr_debug("explorer:%s, index:%d", __func__, index);

	/* remove the tail node */
	list_del(cur);
	kfree(ptr);
	*cur_atu_num = *cur_atu_num - 1;
	pr_debug("explorer:%s, done, byte num:%d, block num:%d", __func__, sdio_data->byte_current_atu_num, sdio_data->block_current_atu_num);

	return index;
}

static void hit_atu_update_current_list(struct explorer_sdio_data *sdio_data, unsigned int index, int byte)
{
	struct current_atu_list *ptr = NULL;
	struct list_head *head = NULL;
	unsigned int *flist_cur = NULL;
	unsigned int *cur_atu_num = NULL;
	unsigned int flag = 0;
	unsigned int clear_state = 0;

	pr_debug("explorer:%s, begin", __func__);
	head = byte?(&sdio_data->byte_current_list):(&sdio_data->block_current_list);
	flist_cur = GET_FREE_LIST_ADDR(sdio_data, byte);
	cur_atu_num = GET_CUR_ATU_NUM_ADDR(sdio_data, byte);
	list_for_each_entry(ptr, head, list) {
		if (ptr->index == index) {
			flag = 1;
			break;
		}
	}

	if (!flag) {
		/* in table but not in current list */
		if (!(*flist_cur & INDEX_TO_FREE_STATE(index, byte))) {
			pr_debug("explorer:%s, done, match index: %d", __func__, index);
			return;
		}
		else {
			clear_state = ~INDEX_TO_FREE_STATE(index, byte);
			*flist_cur = *flist_cur & clear_state;
		}
	}
	else{
		/* in current list and in table */
		/* delete older node */
		list_del(&ptr->list);
		kfree(ptr);
		*cur_atu_num = *cur_atu_num - 1;
	}

	/* insert index to the head of current list */
	put_index_to_current_list(sdio_data, index, byte);
	pr_debug("explorer:%s, done, match index: %d", __func__, index);
}


static unsigned int miss_atu_update_current_list(struct explorer_sdio_data *sdio_data, int block)
{
	unsigned int index = 0;
	unsigned int clear_state = 0;
	unsigned int *flist_cur = NULL;

	pr_debug("explorer:%s, begin", __func__);
	flist_cur = GET_FREE_LIST_ADDR(sdio_data, !block);
	pr_debug("explorer:%s, %s free list status: 0x%x", __func__, block?"block":"byte", *flist_cur);
	if (*flist_cur != 0) {
		/* free list is not null */
		index = GET_FREE_STATE_TO_INDEX(*flist_cur, !block);
		clear_state = ~INDEX_TO_FREE_STATE(index, !block);
		*flist_cur = *flist_cur & clear_state;
		pr_debug("explorer:%s, get index %d from %s free list, clear state: 0x%x, free list state: 0x%x", __func__, index, block?"block":"byte", clear_state, *flist_cur);
		put_index_to_current_list(sdio_data, index, !block);
	} else {
		index = get_index_from_current_list(sdio_data, !block);
		pr_debug("explorer:%s, get index %d from %s current list", __func__, index, block?"block":"byte");
		put_index_to_current_list(sdio_data, index, !block);
	}
	pr_debug("explorer:%s, done", __func__);

	return index;
}


/* atu 0~31 for byte mode, atu 32-63 for block mode */
static int check_if_remapaddr_in_atutable(struct explorer_sdio_data *sdio_data, unsigned int addr, unsigned int k, unsigned char block, unsigned int *remap_addr)
{
	struct sdio_func *pfunc = sdio_data->func;
	unsigned int i, index = INDEX_OUT_OF_RANGE;
	unsigned atu_num = 0;
	unsigned atu_start_index = 0;

	/* The number of atu in each mode should not exceed 32 */
	if ((!block && k < 3) || (block && k < 12))
		goto outofrange;

	/* if address related ipc does not check in atu */
	if (!block && (addr >= 0x178000 && addr <= 0x180000)) {
		*remap_addr = 0x178000;
		return 14;
	}
	if (block && (addr >= 0x0 && addr <= 0x800000)) {
		*remap_addr = 0x0;
		return 48;
	}
	atu_num = block?DEVICE_BLOCK_ATU_NUM(k):DEVICE_BYTE_ATU_NUM(k);
	atu_start_index = block?DEVICE_BLOCK_ATU_START_IDX(k):DEVICE_BYTE_ATU_START_IDX;
	/* get remap addr from input addr */
	*remap_addr = addr & DEVICE_REMAP_ADDR_MASK(k);
	pr_debug("zeku: %s %s, remap_addr from input addr: 0x%x", __func__, (block?"block mode":"bytemode"), *remap_addr);
	/* find the corresponding match_addr */
	for (i = atu_start_index; i < atu_start_index + atu_num; i++) {
		if (sdio_data->remap_reg[i].value == *remap_addr) {
			index = i;
			/* update current list */
			hit_atu_update_current_list(sdio_data, index, !block);
			goto out;
		}
	}
	/* update remap addr, when remap addr is not at atu table */
	if (index > atu_start_index + atu_num) {
		/* todo: update strategy */
		pr_debug("zeku: %s no remap addr,update remap addr", __func__);
		index = miss_atu_update_current_list(sdio_data, block);
		sdio_update_remap(pfunc, *remap_addr, index, FUNC0);
	}
	goto out;
outofrange:
	pr_err("zeku: %s, k is assigned the wrong value", __func__);
out:
	return index;
}

unsigned int addr_align_4(unsigned int addr)
{
	unsigned int ret = 0;

	ret = addr % 4;
	if (ret != 0)
		ret = 4 - ret;
	return ret;
}

unsigned int addr_align_512(unsigned int addr)
{
	unsigned int ret = 0;

	ret = addr % 512;
	if (ret != 0)
		ret = 512 - ret;
	return ret;
}

/*
 *  Query  addr's atu remaining space
 *	@func: Query addr's atu remaining space
 *	@sdio_data: sdio cmd
 *	@addr: the addr to query
 *	@block: 1(block mode),0(byte mode)
 *	@return: byte count of remaining space
 */

static int atu_restspace(struct explorer_sdio_data *sdio_data, unsigned int addr,  unsigned char block)
{
	int len = -1;
	unsigned int remap_addr = 0;
	unsigned int index = INDEX_OUT_OF_RANGE;
	unsigned int k = block?12:3;
	/* get remap addr from input addr */
	index = check_if_remapaddr_in_atutable(sdio_data, addr, k, block, &remap_addr);
	if (index == INDEX_OUT_OF_RANGE) {
		pr_err("zeku: %s, wrong input param", __func__);
		return -EINVAL;
	}
	len = (1<<(12+k))-(addr - remap_addr);
	pr_debug("zeku: %s , restspace from input addr: %d", __func__, len);
	return len;
}

static void sdio_get_regoffset_funcnum(struct explorer_sdio_data *sdio_data, unsigned int addr, unsigned int index, unsigned int k, unsigned char block, unsigned int *funcnum, unsigned int *regoffset)
{
	unsigned int atu_value;
	unsigned int regoffset_high=0;
	unsigned int regoffset_low=0;
	unsigned int regoffset_high_mask=0;
	unsigned int regoffset_low_mask=0;
	unsigned int funcnum_mask=0;
	unsigned int funcnum_shift=0;
	unsigned int regoffset_shift=0;

	regoffset_high_mask = block?(DEVICE_BLOCK_ADDR_MASK & (0xffffffff << DEVICE_MATCH_ADDR_START_BIT(k))):(DEVICE_BYTE_ADDR_MASK & (0xffffffff << DEVICE_MATCH_ADDR_START_BIT(k)));
	regoffset_low_mask = block?((0xffffffff >> (32 - DEVICE_MATCH_ADDR_START_BIT(k))) & DEVICE_BLOCK_ADDR_MASK):((0xffffffff >> (32 - DEVICE_MATCH_ADDR_START_BIT(k)) & DEVICE_BYTE_ADDR_MASK));
	regoffset_shift = block?DEVICE_BLOCK_REGOFFSET_SHIFT:DEVICE_BYTE_REGOFFSET_SHIFT;
	funcnum_mask = block?DEVICE_BLOCK_FUNC_MASK:DEVICE_BYTE_FUNC_MASK;
	funcnum_shift = block?DEVICE_BLOCK_FUNC_SHIFT:DEVICE_BYTE_FUNC_SHIFT;
	/* get match addr from atu table */
	atu_value = sdio_data->atu_reg[index].value;
	pr_debug("zeku: %s match atu:0x%x", __func__, atu_value);
	/* get regoffset offset and function num */
	regoffset_high = atu_value & regoffset_high_mask;
	regoffset_low = addr & regoffset_low_mask;
	*regoffset = (regoffset_high | regoffset_low) >> regoffset_shift;
	*funcnum = (atu_value & funcnum_mask) >> funcnum_shift;

	pr_debug("zeku: %s regoffset:0x%x, func_num:0x%x", __func__, *regoffset, *funcnum);

}

/*
 *  for SDIO CMD53
 *	@func: Read the memory on the cc side
 *	@sdio_data: sdio cmd
 *	@addr: buffer to store the data(for CMD53)
 *	@dst: address to begin reading from(for CMD53)
 *	@count: number of bytes to read
 *	@return: 0(SUCCESS)
 */

static int sdio_read_data_helper(struct explorer_sdio_data *sdio_data,unsigned int addr,void *dst,int count, unsigned char block)
{
	int ret = 0;
	unsigned int k = block?12:3, index = INDEX_OUT_OF_RANGE;
	struct sdio_func *pfunc = sdio_data->func;
	unsigned int dwold_funcnum = sdio_data->func->num;
	unsigned int regoffset;
	unsigned int funcnum_addr;
	unsigned int remap_addr;

	/* Check input parameters */

	index = check_if_remapaddr_in_atutable(sdio_data, addr, k, block, &remap_addr);
	if (index == INDEX_OUT_OF_RANGE) {
		pr_err("zeku: %s, wrong input param", __func__);
		return -EINVAL;
	}
	sdio_get_regoffset_funcnum(sdio_data, addr, index, k, block, &funcnum_addr, &regoffset);

	ret = sdio_explorer_claim_func(pfunc);
	if (ret)
		return ret;


	/*modifying func num will affect sdio_explorer_enable_func and sdio_explorer_disable_func */
	pfunc->num = funcnum_addr;
	pr_debug("zeku: %s:%d read data:atu addr:%x,len:%x", __func__, __LINE__, addr, count);
	if (count <= 64) {
		ret = sdio_explorer_memcpy_fromio(pfunc, (void *)sdio_data->data,regoffset, count);
		if (ret) {
			pr_err("zeku: %s there is something wrong when send cmd53 to read sram: %d\n", __func__, ret);
			goto cmd53failed;
		}
		memcpy(dst, (void *)sdio_data->data, count);
	} else {
		ret = sdio_explorer_memcpy_fromio(pfunc, dst,regoffset, count);
		if (ret) {
			pr_err("zeku: %s there is something wrong when send cmd53 to read sram: %d\n", __func__, ret);
			goto cmd53failed;
		}
	}

	pr_debug("zeku: %s, CMD53 read done, first dst: 0x%x", __func__, *(unsigned int *)dst);
cmd53failed:
	/*recover original func num*/
	pfunc->num = dwold_funcnum;
	sdio_explorer_release_func(pfunc);

	return ret;
}

/*
 *  for SDIO CMD53 byte read
 *	@func: Read the memory(addr align 4 auto) on the cc side
 *	@sdio_data: sdio cmd
 *	@addr: address to begin reading from(for CMD53)
 *	@dst: buffer to store the data(for CMD53)
 *	@count: number of bytes to read
 *	@return: 0(SUCCESS)
 */

static int sdio_read_data_byte_align4(struct explorer_sdio_data *sdio_data, unsigned int addr, void *dst, int count)
{
	unsigned int byte_align = 0;
	unsigned int byte_send = 0;
	int ret = 0;
	u8 *buf = dst;

	while (count > 0) {
		byte_align = addr_align_4(addr);
		if (byte_align != 0)
			byte_send = byte_align;
		else
			byte_send = count;
		ret = sdio_read_data_helper(sdio_data, addr, buf, byte_send, BYTE_MODE);
		if (ret) {
			pr_err("zeku: %s:%d, addr: 0x%x, count_send: 0x%x", __func__, __LINE__, addr, byte_send);
			return ret;
		}
		buf += byte_send;
		addr += byte_send;
		count -= byte_send;
		byte_send = 0;
	}
	return 0;
}

/*
 *  for SDIO CMD53 byte read
 *	@func: Read the memory(addr align 4 auto) on the cc side,cross two atu auto
 *	@sdio_data: sdio cmd
 *	@addr: address to begin reading from(for CMD53)
 *	@dst: buffer to store the data(for CMD53)
 *	@count: number of bytes to read
 *	@return: 0(SUCCESS)
 */

static int sdio_read_data_byte_atu(struct explorer_sdio_data *sdio_data, unsigned int addr, void *dst, int count)
{
	int restspace = 0, count_send = 0;
	int ret = 0;
	u8 *buf = dst;

	while (count > 0) {
		restspace = atu_restspace(sdio_data, addr, BYTE_MODE);
		if (restspace < 0)
			return restspace;
		if (restspace >= count)
			count_send = count;
		else
			count_send = restspace;
		ret = sdio_read_data_byte_align4(sdio_data, addr, buf, count_send);
		if (ret) {
			pr_err("zeku: %s:%d, addr: 0x%x, count_send: 0x%x", __func__, __LINE__, addr, count_send);
			return ret;
		}
		buf += count_send;
		addr += count_send;
		count -= count_send;
		count_send = 0;
		restspace = 0;
	}
	return ret;
}

/*
 *  for SDIO CMD53 block read
 *	@func: Read the memory(addr align 4 auto) on the cc side,cross two atu auto
 *	@sdio_data: sdio cmd
 *	@addr: address to begin reading from(for CMD53)
 *	@dst: buffer to store the data(for CMD53)
 *	@count: number of bytes to read
 *	@return: 0(SUCCESS)
 */

static int sdio_read_data_block_atu(struct explorer_sdio_data *sdio_data, unsigned int addr, void *dst, int count)
{
	int restspace = 0, count_send = 0;
	int ret = 0;
	u8 *buf = dst;

	while (count > 0) {
		restspace = atu_restspace(sdio_data, addr, BLOCK_MODE);
		if (restspace < 0)
			return restspace;
		if (restspace >= count)
			count_send = count;
		else
			count_send = restspace;
		ret = sdio_read_data_helper(sdio_data, addr, buf, count_send, BLOCK_MODE);
		if (ret) {
			pr_err("zeku: %s:%d, addr: 0x%x, count_send: 0x%x", __func__, __LINE__, addr, count_send);
			return ret;
		}
		buf += count_send;
		addr += count_send;
		count -= count_send;
		restspace = 0;
	}
	return ret;
}

/*
 *  for SDIO CMD53 read(block and byte)
 *	@func: read a chunk of memory(CMD53) from a SDIO function
 *	@cmd: sdio cmd
 *	@dst: buffer to store the data(for CMD53)
 *	@addr: address to begin reading from(for CMD53)
 *	@count: number of bytes to read(for CMD53)
 *	@return: 0(SUCCESS)
 */

int sdio_read_data(struct explorer_sdio_data *sdio_data, unsigned int addr, void *dst, int count)
{
	int ret = 0;
	struct sdio_func *pfunc = NULL;
	unsigned int blocksize = 0;
	unsigned int blocks = 0;
	int blockcount = 0, bytecount = 0;
	unsigned int max_blksz = 0;
	u8 *buf = dst;
	int count_send = 0, byte_align512 = 0, count_send_temp = 0;
	int restspace = 0;
	int count_tatol = count;

	if (!sdio_data)
		return -ENODEV;

	if (!dst)
		return -EINVAL;

	pfunc = sdio_data->func;
	blocksize = pfunc->cur_blksize;
	max_blksz = pfunc->max_blksize;
	while (count_tatol) {
		if (count_tatol < blocksize * 2) {
			while (count_tatol > 0) {
				pr_debug("zeku: %s:%d, addr: 0x%x, count: 0x%x", __func__, __LINE__, addr, count_tatol);
				if (count_tatol >= blocksize)
					count_send = blocksize;
				else
					count_send = count_tatol;
				ret = sdio_read_data_byte_atu(sdio_data, addr, buf, count_send);
				if (ret) {
					pr_err("zeku: %s:%d, addr: 0x%x, count_send: 0x%x", __func__, __LINE__, addr, count_send);
					return ret;
				}
				buf += count_send;
				addr += count_send;
				count_tatol -= count_send;
			}
		} else {
			//1.align 512
			byte_align512 = addr_align_512(addr);
			if (byte_align512 > 0) {
				ret = sdio_read_data_byte_atu(sdio_data, addr, buf, byte_align512);
				if (ret) {
					pr_err("zeku: %s:%d, addr: 0x%x, count_send: 0x%x", __func__, __LINE__, addr, byte_align512);
					return ret;
				}
				buf += byte_align512;
				addr += byte_align512;
				count_tatol -= byte_align512;
			}

			//2. block send？
			restspace = atu_restspace(sdio_data, addr, BLOCK_MODE);
			count_send_temp = MIN(count_tatol, restspace);

			if (count_send_temp < blocksize * 2) {
				while (count_send_temp > 0) {
					pr_debug("zeku: %s:%d, addr: 0x%x, count: 0x%x", __func__, __LINE__, addr, count_send_temp);
					if (count_send_temp >= blocksize)
						count_send = blocksize;
					else
						count_send = count_send_temp;
					ret = sdio_read_data_byte_atu(sdio_data, addr, buf, count_send);
					if (ret) {
						pr_err("zeku: %s:%d, addr: 0x%x, count_send: 0x%x", __func__, __LINE__, addr, count_send);
						return ret;
					}
					buf += count_send;
					addr += count_send;
					count_send_temp -= count_send;
					count_tatol -= count_send;
				}
			}

			//3. real block send
			blocks = count_send_temp / blocksize;
			if (blocks % 511 == 1)
				blocks = blocks - 1;
			blockcount = blocks * blocksize;
			bytecount = count_send_temp - blockcount;

			while (blocks) {
				if (blocks >= 511) {
					ret = sdio_read_data_block_atu(sdio_data, addr, buf, 511 * blocksize);
					if (ret) {
						pr_err("zeku: %s:%d, addr: 0x%x, count_send: 0x%x", __func__, __LINE__, addr, 511 * blocksize);
						return ret;
					}
					buf += 511 * blocksize;
					addr += 511 * blocksize;
					count_send_temp -= 511 * blocksize;
					count_tatol -= 511 * blocksize;
					blockcount -= 511 * blocksize;
					blocks -= 511;
				} else if (blocks >= 2 && blocks < 511) {
					ret = sdio_read_data_block_atu(sdio_data, addr, buf, blocks * blocksize);
					if (ret) {
						pr_err("zeku: %s:%d, addr: 0x%x, count_send: 0x%x", __func__, __LINE__, addr, blocks * blocksize);
						return ret;
					}
					buf += blocks * blocksize;
					addr += blocks * blocksize;
					count_send_temp -= blocks * blocksize;
					count_tatol -= blocks * blocksize;
					blockcount = 0;
					blocks = 0;
				} else {
					pr_err("zeku: %s, wrong input param, ", __func__);
					return -EINVAL;
				}
			}
			pr_debug("zeku: %s, addr: 0x%x, blockcount: 0x%x, count: 0x%x", __func__, addr, blockcount, count_tatol);
		/* 3. data that length is less than two blocks is sent in byte mode */
			while (bytecount) {
				if ((bytecount > blocksize) && (bytecount < 2 * blocksize)) {
					ret = sdio_read_data_byte_atu(sdio_data, addr, buf, blocksize);
					if (ret) {
						pr_err("zeku: %s:%d, addr: 0x%x, count_send: 0x%x", __func__, __LINE__, addr, blocksize);
						return ret;
					}
					buf += blocksize;
					addr += blocksize;
					bytecount -= blocksize;
					count_send_temp -= blocksize;
					count_tatol -= blocksize;
				} else if (bytecount <= blocksize && bytecount > 0) {
					ret = sdio_read_data_byte_atu(sdio_data, addr, buf, bytecount);
					if (ret) {
						pr_err("zeku: %s:%d, addr: 0x%x, count_send: 0x%x", __func__, __LINE__, addr, bytecount);
						return ret;
					}
					buf += bytecount;
					addr += bytecount;
					count_send_temp -= bytecount;
					count_tatol -= bytecount;
					bytecount = 0;

				} else {
					pr_err("zeku: %s, wrong input param, ", __func__);
					return -EINVAL;
				}
			}
			if (count_send_temp)
				pr_err("zeku: %s:%d, count_send_temp left %d\n", __func__, __LINE__, count_send_temp);
		}
	}
	if (count_tatol)
		pr_err("zeku: %s:%d, count left %d\n", __func__, __LINE__, count_tatol);
	pr_debug("zeku: %s, read addr: 0x%x, value: 0x%x", __func__, addr, *(unsigned int *)dst);
	pr_debug("zeku: %s, read data done", __func__);
	return ret;
}

/*
 *  for send data size less than 512 bytes
 *	@func: write a single byte(CMD52) or a chunk of memory(CMD53) from a SDIO function
 *	@cmd: sdio cmd
 *  @value: value that cmd52 contains the data to write
 *	@addr: cc address to start writing to
 *	@src: buffer that cmd53 contains the data to write
 *	@count: number of bytes to write
 *  @return: 0 or -EINVAL or -ENOMEM or -ERANGE
 */
static int sdio_write_data_helper(struct explorer_sdio_data *sdio_data,unsigned int addr,void *src,int count,unsigned char block)
{
	int ret = 0;
	unsigned int k = block?12:3, index = INDEX_OUT_OF_RANGE;
	unsigned int addr_offset_mask = block?0xffffff:0x7fff;
	struct sdio_func *pfunc = sdio_data->func;
	unsigned int dwold_funcnum = sdio_data->func->num;
	unsigned int regoffset;
	unsigned int funcnum_addr;
	unsigned int remap_addr;
	unsigned int addr_offset;

	index = check_if_remapaddr_in_atutable(sdio_data, addr, k, block, &remap_addr);
	if (index == INDEX_OUT_OF_RANGE) {
		pr_err("zeku: %s, wrong input param", __func__);
		return -EINVAL;
	}

	sdio_get_regoffset_funcnum(sdio_data, addr, index, k, block, &funcnum_addr, &regoffset);

	addr_offset = addr & addr_offset_mask;

	ret = sdio_explorer_claim_func(pfunc);
	if (ret)
		return ret;


	/*modifying func num will affect sdio_explorer_enable_func and sdio_explorer_disable_func */
	pfunc->num = funcnum_addr;

	pr_debug("zeku: %s:%d send data:atu addr:%x,len:%x, first src: 0x%x", __func__, __LINE__, addr, count, *(unsigned int *)src);
	if (count<=64) {
		memcpy((void *)sdio_data->data, src, count);
		ret = sdio_explorer_memcpy_toio(pfunc, regoffset, (void *)sdio_data->data, count);
		if (ret) {
			pr_err("zeku: %s there is something wrong when send cmd53 to write sram: %d\n", __func__, ret);
			goto cmd53failed;
		}
	} else {
		ret = sdio_explorer_memcpy_toio(pfunc, regoffset, src, count);
		if (ret) {
			pr_err("zeku: %s there is something wrong when send cmd53 to write sram: %d\n", __func__, ret);
			goto cmd53failed;
		}
	}
	pr_debug("zeku: %s, send CMD53 write done", __func__);
cmd53failed:
	/*recover original func num*/
	pfunc->num = dwold_funcnum;
	sdio_explorer_release_func(pfunc);

	return ret;

}

/*
 *  for SDIO CMD53 byte send
 *	@func: write the memory(addr align 4 auto) on the cc side
 *	@sdio_data: sdio cmd
 *	@addr: address to begin write from(for CMD53)
 *	@dst: write buffer(for CMD53)
 *	@count: number of bytes to write
 *	@return: 0(SUCCESS)
 */

static int sdio_write_data_byte_align4(struct explorer_sdio_data *sdio_data, unsigned int addr, void *dst, int count)
{
	unsigned int byte_align = 0;
	unsigned int byte_send = 0;
	int ret = 0;
	u8 *buf = dst;

	while (count > 0) {
		pr_debug("zeku: %s:%d, addr: 0x%x, count: 0x%x", __func__, __LINE__, addr, count);
		byte_align = addr_align_4(addr);
		if (byte_align != 0)
			byte_send = byte_align;
		else
			byte_send = count;
		ret = sdio_write_data_helper(sdio_data, addr, buf, byte_send, BYTE_MODE);
		if (ret) {
			pr_err("zeku: %s:%d, addr: 0x%x, count_send: 0x%x", __func__, __LINE__, addr, byte_send);
			return ret;
		}
		buf += byte_send;
		addr += byte_send;
		count -= byte_send;
		byte_send = 0;
	}
	return 0;
}


/*
 *  for SDIO CMD53 byte send
 *	@func: write the memory(addr align 4 auto) on the cc side,cross two atu auto
 *	@sdio_data: sdio cmd
 *	@addr: address to begin write(for CMD53)
 *	@dst: write buffer(for CMD53)
 *	@count: number of bytes to write
 *	@return: 0(SUCCESS)
 */

static int sdio_write_data_byte_atu(struct explorer_sdio_data *sdio_data, unsigned int addr, void *dst, int count)
{
	int restspace = 0, count_send = 0;
	int ret = 0;
	u8 *buf = dst;

	while (count > 0) {
		pr_debug("zeku: %s:%d, addr: 0x%x, count: 0x%x", __func__, __LINE__, addr, count);
		restspace = atu_restspace(sdio_data, addr, BYTE_MODE);
		if (restspace < 0)
			return restspace;
		if (restspace >= count)
			count_send = count;
		else
			count_send = restspace;
		ret = sdio_write_data_byte_align4(sdio_data, addr, buf, count_send);
		if (ret) {
			pr_err("zeku: %s:%d, addr: 0x%x, count_send: 0x%x", __func__, __LINE__, addr, count_send);
			return ret;
		}
		buf += count_send;
		addr += count_send;
		count -= count_send;
		count_send = 0;
		restspace = 0;
	}
	return ret;
}

/*
 *  for SDIO CMD53 block send
 *	@func: write the memory(addr align 4 auto) on the cc side,cross two atu auto
 *	@sdio_data: sdio cmd
 *	@addr: address to begin write(for CMD53)
 *	@dst: write buffer(for CMD53)
 *	@count: number of bytes to write
 *	@return: 0(SUCCESS)
 */

static int sdio_write_data_block_atu(struct explorer_sdio_data *sdio_data, unsigned int addr, void *dst, int count)
{
	int restspace = 0, count_send = 0;
	int ret = 0;
	u8 *buf = dst;

	while (count > 0) {
		pr_debug("zeku: %s:%d, addr: 0x%x, count: 0x%x", __func__, __LINE__, addr, count);
		restspace = atu_restspace(sdio_data, addr, BLOCK_MODE);
		if (restspace < 0)
			return restspace;
		if (restspace >= count)
			count_send = count;
		else
			count_send = restspace;
		ret = sdio_write_data_helper(sdio_data, addr, buf, count_send, BLOCK_MODE);
		if (ret) {
			pr_err("zeku: %s:%d, addr: 0x%x, count_send: 0x%x", __func__, __LINE__, addr, count_send);
			return ret;
		}
		buf += count_send;
		addr += count_send;
		count -= count_send;
		restspace = 0;
	}
	return ret;
}

/*
 *  for SDIO CMD53 send(block and byte)
 *	@func: write the memory(addr align 4 auto) on the cc side,cross two atu auto
 *	@sdio_data: sdio cmd
 *	@addr: address to begin write(for CMD53)
 *	@dst: write buffer(for CMD53)
 *	@count: number of bytes to write
 *	@return: 0(SUCCESS)
 */

int sdio_write_data(struct explorer_sdio_data *sdio_data, unsigned int addr, void *src, int count)
{
	int ret = 0;
	struct sdio_func *pfunc = NULL;
	unsigned int blocksize = 0;
	unsigned int blocks = 0;
	int blockcount = 0, bytecount = 0;
	unsigned int max_blksz = 0;
	u8 *buf = src;
	int count_send = 0, byte_align512 = 0, count_send_temp = 0;
	int restspace = 0;
	int count_tatol = count;

	if (!sdio_data)
		return -ENODEV;

	if (!src)
		return -EINVAL;

	pfunc = sdio_data->func;
	blocksize = pfunc->cur_blksize;
	max_blksz = pfunc->max_blksize;

	while (count_tatol) {
		if (count_tatol < blocksize * 2) {
			pr_debug("zeku: %s:%d, addr: 0x%x, count: 0x%x", __func__, __LINE__, addr, count_tatol);
			while (count_tatol > 0) {
				pr_debug("zeku: %s:%d, addr: 0x%x, count: 0x%x", __func__, __LINE__, addr, count_tatol);
				if (count_tatol >= blocksize)
					count_send = blocksize;
				else
					count_send = count_tatol;
				ret = sdio_write_data_byte_atu(sdio_data, addr, buf, count_send);
				if (ret) {
					pr_err("zeku: %s:%d, addr: 0x%x, count_send: 0x%x", __func__, __LINE__, addr, count_send);
					return ret;
				}
				buf += count_send;
				addr += count_send;
				count_tatol -= count_send;
			}
		} else {
			//1.align 512
			pr_debug("zeku: %s:%d, addr: 0x%x, count: 0x%x", __func__, __LINE__, addr, count_tatol);
			byte_align512 = addr_align_512(addr);
			if (byte_align512 > 0) {
				ret = sdio_write_data_byte_atu(sdio_data, addr, buf, byte_align512);
				if (ret) {
					pr_err("zeku: %s:%d, addr: 0x%x, count_send: 0x%x", __func__, __LINE__, addr, byte_align512);
					return ret;
				}
				buf += byte_align512;
				addr += byte_align512;
				count_tatol -= byte_align512;
			}

			//2. block send？
			restspace = atu_restspace(sdio_data, addr, BLOCK_MODE);
			count_send_temp = MIN(count_tatol, restspace);
			if (count_send_temp < blocksize * 2) {
				while (count_send_temp > 0) {
					pr_debug("zeku: %s:%d, addr: 0x%x, count: 0x%x", __func__, __LINE__, addr, count_send_temp);
					if (count_send_temp >= blocksize)
						count_send = blocksize;
					else
						count_send = count_send_temp;
					ret = sdio_write_data_byte_atu(sdio_data, addr, buf, count_send);
					if (ret) {
						pr_err("zeku: %s:%d, addr: 0x%x, count_send: 0x%x", __func__, __LINE__, addr, count_send);
						return ret;
					}
					buf += count_send;
					addr += count_send;
					count_send_temp -= count_send;
					count_tatol -= count_send;
				}
			}

			//3. block send
			blocks = count_send_temp / blocksize;
			if (blocks % 511 == 1)
				blocks = blocks - 1;
			blockcount = blocks * blocksize;
			bytecount = count_send_temp - blockcount;

			while (blocks) {
				pr_debug("zeku: %s:%d, addr: 0x%x, count: 0x%x", __func__, __LINE__, addr, count_tatol);
				if (blocks >= 511) {
					ret = sdio_write_data_block_atu(sdio_data, addr, buf, 511 * blocksize);
					if (ret) {
						pr_err("zeku: %s:%d, addr: 0x%x, count_send: 0x%x", __func__, __LINE__, addr, 511 * blocksize);
						return ret;
					}
					buf += 511 * blocksize;
					addr += 511 * blocksize;
					count_send_temp -= 511 * blocksize;
					count_tatol -= 511 * blocksize;
					blockcount -= 511 * blocksize;
					blocks -= 511;
				} else if (blocks >= 2 && blocks < 511) {
					ret = sdio_write_data_block_atu(sdio_data, addr, buf, blocks * blocksize);
					if (ret) {
						pr_err("zeku: %s:%d, addr: 0x%x, count_send: 0x%x", __func__, __LINE__, addr, blocks * blocksize);
						return ret;
					}
					buf += blocks * blocksize;
					addr += blocks * blocksize;
					count_send_temp -= blocks * blocksize;
					count_tatol -= blocks * blocksize;
					blockcount = 0;
					blocks = 0;
				} else {
					pr_err("zeku: %s, wrong input param, ", __func__);
					return -EINVAL;
				}
			}
			pr_debug("zeku: %s, addr: 0x%x, blockcount: 0x%x, count: 0x%x", __func__, addr, blockcount, count_tatol);
		/* 3. data that length is less than two blocks is sent in byte mode */
			while (bytecount) {
				pr_debug("zeku: %s:%d, addr: 0x%x, count: 0x%x", __func__, __LINE__, addr, count_tatol);
				if ((bytecount > blocksize) && (bytecount < 2 * blocksize)) {
					ret = sdio_write_data_byte_atu(sdio_data, addr, buf, blocksize);
					if (ret) {
						pr_err("zeku: %s:%d, addr: 0x%x, count_send: 0x%x", __func__, __LINE__, addr, blocksize);
						return ret;
					}
					buf += blocksize;
					addr += blocksize;
					bytecount -= blocksize;
					count_send_temp -= blocksize;
					count_tatol -= blocksize;
				} else if (bytecount <= blocksize && bytecount > 0) {
					ret = sdio_write_data_byte_atu(sdio_data, addr, buf, bytecount);
					if (ret) {
						pr_err("zeku: %s:%d, addr: 0x%x, count_send: 0x%x", __func__, __LINE__, addr, bytecount);
						return ret;
					}
					buf += bytecount;
					addr += bytecount;
					count_send_temp -= bytecount;
					count_tatol -= bytecount;
					bytecount = 0;
				} else {
					pr_err("zeku: %s, wrong input param, ", __func__);
					return -EINVAL;
				}
			}
			if (count_send_temp)
				pr_err("zeku: %s:%d, count_send_temp left %d\n", __func__, __LINE__, count_send_temp);
		}
	}
	if (count_tatol)
		pr_err("zeku: %s:%d, count left %d\n", __func__, __LINE__, count_tatol);
	pr_debug("zeku: %s, write addr: 0x%x, value: 0x%x", __func__, addr, *(unsigned int *)src);
	pr_debug("zeku: %s, write data done", __func__);
	return ret;
}

/*
 *	@func: SDIO batch write_data
 *	@sdio_data: sdio private_data
 *	@addr: address to start writing to
 *	@src: buffer that cmd53 contains the data to write
 *	@count: number of bytes to write
 *  @return: 0 or -EINVAL or -ENOMEM or -ERANGE
 */
int sdio_batch_write_data(struct explorer_sdio_data *sdio_data, struct batch_data_item *src, int count)
{
	int ret = 0;
	struct sdio_func *pfunc = NULL;
	unsigned int dwold_funcnum = 0;
	unsigned int regoffset = 0;
	unsigned int funcnum_addr;

	if (!sdio_data)
		return -ENODEV;

	if (!src)
		return -EINVAL;

	if (count % sizeof(struct batch_data_item) != 0) {
		pr_err("%s, The buffer needs to be aligned with 8 bytes", __func__);
		return -EINVAL;
	}

	dwold_funcnum = sdio_data->func->num;
	pfunc = sdio_data->func;

	/* disable atu match */
	funcnum_addr = 0x7;
	ret = sdio_explorer_claim_func(pfunc);
	if (ret)
		return ret;


	/*modifying func num will affect sdio_explorer_enable_func and sdio_explorer_disable_func */
	pfunc->num = funcnum_addr;

	pr_debug("zeku: send CMD53 for batch write, src:0x%x", *(unsigned int *)src);
	ret = sdio_explorer_writesb(pfunc, regoffset, (void *)src, count);
	if (ret) {
		pr_err("zeku: there is something wrong when send cmd53 to write sram: %d\n", ret);
	}
	/*recover original func num*/
	pfunc->num = dwold_funcnum;

	sdio_explorer_release_func(pfunc);

	return ret;
}

/* for interrupt handler */
static void sdio_data1_irq(struct sdio_func *func)
{
	int ret = 0;
	u8 status = 0;
	u8 clearstatus = 0;
	u8 int0 = 0;
	u8 int1 = 0;
	struct explorer_sdio_data *pdata = sdio_get_drvdata(func);
	pr_debug("zeku: %s, enter sdio irq handler", __func__);
	/* 1. read irq status */
	ret = sdio_read_register(pdata, SDIO_FN1R_HOST_INT_STS, &status, FUNC1);
	if (ret && (status == 0xff)) {
		pr_err("zeku: %s read host interrupt status error: %d", __func__, ret);
		return;
	}
	/* 2. parse irq */
	if (status & SDIO_IRQ_INTC0_HOST_MASK) {
		/* 3.1 read int0 irq status */
		pr_debug("zeku: %s, trigger int0 irq", __func__);

		/* 3.2 clear int0 irq */

		ret = sdio_read_register(pdata, SDIO_FN1R_SOC_STS_REG1B3, &int0, FUNC0);
		if (int0 & INT_SYS_MIPI_RX_FS_MASK) {
			pr_debug("zeku: %s, trigger mipi rx frame start irq", __func__);

		}
		if (int0 & INT1_ISP_TO_AP_MASK) {
			pr_debug("zeku: %s, trigger intc0 isp to ap irq", __func__);

		}
		if (int0 & INT0_ISP_TO_AP_MASK) {
			pr_debug("zeku: %s, trigger intc1 isp to ap irq", __func__);

		}
		if (int0 & INT_CHIP_WAKEUP_MASK) {
			pr_debug("zeku: %s, trigger chip wakeup irq", __func__);

		}
		if (int0 & INT_CC_RESET_DONE_MASK) {
			pr_debug("zeku: %s, trigger cc reset done irq", __func__);

		}
		if (int0 & INT_PVT_IRQ_MASK) {
			pr_debug("zeku: %s, trigger pvt irq", __func__);

		}
		if (int0 & INT_SDU_HOST_MASK) {
			pr_debug("zeku: %s, trigger sdu host irq", __func__);

		}
		if (int0 & INT_BUS_MONITOR_MASK) {
			pr_debug("zeku: %s, trigger bus monitor irq", __func__);

		}

		/* 3.3 clear status reg */
		clearstatus = clearstatus | SDIO_IRQ_INTC0_HOST_MASK;
	}
	if (status & SDIO_IRQ_INTC1_HOST_MASK) {
		pr_debug("zeku: %s, trigger int1 irq", __func__);
		/* 4.1 read int1 irq status */
		ret = sdio_read_register(pdata, SDIO_FN1R_SOC_STS_REG1B2, &int1, FUNC0);

		/* 4.2 clear intc1 irq */
		if (int1 & MPU_INT_ST_RSP_ERR_MASK) {
			pr_debug("zeku: %s, trigger mpu st response error irq", __func__);

		}
		if (int1 & MPU_INT_ST_ILLEGAL_READ_MASK) {
			pr_debug("zeku: %s, trigger mpu illegal read irq", __func__);
		}
		if (int1 & MPU_INT_ST_ILLEGAL_WRITE_MASK) {
			pr_debug("zeku: %s, trigger mpu illegal write irq", __func__);

		}
		if (int1 & INT_SE_HOST1_IRQ_MASK) {
			pr_debug("zeku: %s, trigger se host1 irq", __func__);

		}
		if (int1 & INT_WDT_MASK) {
			pr_debug("zeku: %s, trigger wdt irq", __func__);

		}
		if (int1 & INT_MAILBOX_MASK) {
			pr_debug("zeku: %s, trigger mailbox irq", __func__);
		}
		if (int1 & INT_CHIP_SLEEP_MASK) {
			pr_debug("zeku: %s, trigger chip sleep irq", __func__);

		}
		if (int1 & INT_CPU_LOCKUP_MASK) {
			pr_debug("zeku: %s, trigger cpu lockup irq", __func__);

		}

		/* 3.3 clear status reg */
		clearstatus = clearstatus | SDIO_IRQ_INTC1_HOST_MASK;
	}
	if (status & SDIO_IRQ_MISC1_HOST_MASK) {
		pr_debug("zeku: %s, trigger misc1 irq", __func__);
		/* 5. clear MISC1 irq */

		clearstatus = clearstatus | SDIO_IRQ_MISC1_HOST_MASK;
	}
	if (status & SDIO_IRQ_MISC2_HOST_MASK) {
		pr_debug("zeku: %s, trigger misc2 irq", __func__);
		/* 6. clear MISC2 irq */

		clearstatus = clearstatus | SDIO_IRQ_MISC2_HOST_MASK;
	}
	if (status & SDIO_IRQ_UNDERFLOW_HOST_MASK) {
		pr_debug("zeku: %s, trigger underflow irq", __func__);
		/* 7. clear fifo underflow irq */

		clearstatus = clearstatus | SDIO_IRQ_UNDERFLOW_HOST_MASK;
	}
	if (status & SDIO_IRQ_OVERFLOW_HOST_MASK) {
		pr_debug("zeku: %s, trigger overflow irq", __func__);

		clearstatus = clearstatus | SDIO_IRQ_OVERFLOW_HOST_MASK;
	}
	if (status & SDIO_IRQ_DPLD_HOST_MASK) {
		pr_debug("zeku: %s, trigger dpld irq", __func__);

		clearstatus = clearstatus | SDIO_IRQ_DPLD_HOST_MASK;
	}
	if (status & SDIO_IRQ_UPLD_HOST_MASK) {
		pr_debug("zeku: %s, trigger upld irq", __func__);

		clearstatus = clearstatus | SDIO_IRQ_UPLD_HOST_MASK;
	}
	ret = sdio_write_register(pdata, SDIO_FN1R_HOST_INT_STS, ~clearstatus, FUNC1);
}

int sdio_get_sdclk(struct explorer_sdio_data *sdio_data)
{
	int ret = 0, value = 0;
	struct sdio_func *pfunc = NULL;
	struct mmc_card *card = NULL;
	struct mmc_host *host = NULL;

	if (!sdio_data) {
		return -ENODEV;
	}

	pfunc = sdio_data->func;

	ret = sdio_explorer_claim_func(pfunc);
	if (ret)
		return ret;

	card = pfunc->card;
	host = card->host;

	value = host->ios.clock;

	sdio_explorer_release_func(pfunc);

	return value;
}

int sdio_set_sdclk(struct explorer_sdio_data *sdio_data, int sdclk_value)
{
	int ret = 0;
	struct sdio_func *pfunc = NULL;
	struct mmc_card *card = NULL;
	struct mmc_host *host = NULL;

	if (sdclk_value <= 0) {
		pr_err("explorer: %s, clk: %d hz", __func__, sdclk_value);
		return -EINVAL;
	}
	pr_debug("explorer: %s, clk: %d hz", __func__, sdclk_value);

	if (!sdio_data) {
		return -ENODEV;
	}

	pfunc = sdio_data->func;
	ret = sdio_explorer_claim_func(pfunc);
	if (ret)
		return ret;

	card = pfunc->card;
	host = card->host;

	host->ios.clock = sdclk_value;
	host->ops->set_ios(host, &host->ios);

	sdio_explorer_release_func(pfunc);

	return ret;
}

static int explorer_change_to_4bits(struct sdio_func *func, struct explorer_sdio_ios sdio_ios)
{
	int ret = 0;
	struct mmc_card *card = func->card;
	struct mmc_host *host = card->host;
	struct explorer_sdio_data *pdata = sdio_get_drvdata(func);
	struct mmc_ios *ios = &host->ios;
	u8 speed = 0;
	u8 bus_width = 0;

	/* 1. Initialize the sdio device */
	ret = sdio_read_register_no_claim(pdata, SDIO_CCCR_SPEED, &speed, FUNC0);
	if (ret) {
		pr_err("%s, failed to read sdio device speed register, err(%d)", __func__, ret);
		goto still_in_1bits1;
	}
	speed &= ~SDIO_SPEED_BSS_MASK;
	switch (sdio_ios.timing) {
	case MMC_TIMING_UHS_SDR12:
		speed |= SDIO_SPEED_SDR12;
		break;
	case MMC_TIMING_UHS_SDR25:
		speed |= SDIO_SPEED_SDR25;
		break;
	case MMC_TIMING_UHS_SDR50:
		speed |= SDIO_SPEED_SDR50;
		break;
	case MMC_TIMING_UHS_SDR104:
		speed |= SDIO_SPEED_SDR104;
		break;
	default:
		sdio_ios.timing = SDIO_SPEED_SDR12;
		speed |= SDIO_SPEED_SDR12;
		break;
	}

	ret = sdio_write_register_no_claim(pdata, SDIO_CCCR_SPEED, speed, FUNC0);
	if (ret) {
		pr_err("%s, failed to change sdio device to UHS_SDR25, err(%d)", __func__, ret);
		goto still_in_1bits1;
	}
	ret = sdio_read_register_no_claim(pdata, SDIO_CCCR_IF, &bus_width, FUNC0);
	if (ret) {
		pr_err("%s, failed to read sdio device bus width register, err(%d)", __func__, ret);
		goto still_in_1bits2;
	}
	bus_width &= ~SDIO_BUS_WIDTH_MASK;
	bus_width |= sdio_ios.bus_width;
	ret = sdio_write_register_no_claim(pdata, SDIO_CCCR_IF, bus_width, FUNC0);
	if (ret) {
		pr_err("%s, failed to change sdio device to 4-bits mode, err(%d)", __func__, ret);
		goto still_in_1bits2;
	}

	/* 2. Initialize the sdio host */
	host->ios.bus_width = sdio_ios.bus_width;
	host->ios.timing = sdio_ios.timing;
#ifndef AP_BOOT_TEST
	host->ios.clock = sdio_ios.clock;
	host->ios.drv_type = 1;
#else
	host->ios.clock = 50000;
#endif
	host->ops->set_ios(host, &host->ios);
	pr_debug("%s: clock %uHz busmode %u powermode %u cs %u Vdd %u "
			"width %u timing %u\n",
			__func__, ios->clock, ios->bus_mode,
			ios->power_mode, ios->chip_select, ios->vdd,
			1 << ios->bus_width, ios->timing);
	pr_debug("%s, done", __func__);
	return ret;
still_in_1bits2:
	speed &= ~SDIO_SPEED_BSS_MASK;
	speed |= SDIO_SPEED_SDR12;
	ret = sdio_write_register_no_claim(pdata, SDIO_CCCR_SPEED, speed, FUNC0);
	if (ret) {
		pr_err("%s, failed to change sdio device to UHS_SDR25, err(%d)", __func__, ret);
	}
still_in_1bits1:
	return ret;
}

static int sdio_ios_default(struct explorer_sdio_data *pdata)
{
	struct explorer_sdio_ios sdio_ios = {};
	struct sdio_func *func = NULL;

	func = pdata->func;

	sdio_ios.bus_width = sdio_width;
	sdio_ios.clock = sdio_clock;
	sdio_ios.timing = sdio_timing;

	return explorer_change_to_4bits(func, sdio_ios);
}

static void explorer_retune_enable(struct mmc_host *host)
{
	host->can_retune = 1;
	if (host->retune_period)
		mod_timer(&host->retune_timer,
			  jiffies + host->retune_period * HZ);
}

static int sdio_execute_tuning(struct mmc_card *card)
{
	struct mmc_host *host = card->host;
	u32 opcode;
	int err;

	if (!host->ops->execute_tuning)
		return 0;

	if (host->cqe_on)
		host->cqe_ops->cqe_off(host);

	opcode = MMC_SEND_TUNING_BLOCK;

	err = host->ops->execute_tuning(host, opcode);

	if (err)
		pr_err("%s: tuning execution failed: %d\n",
			mmc_hostname(host), err);
	else
		explorer_retune_enable(host);

	return err;
}

static int sdio_execute_tuning_prepare(struct explorer_sdio_data *pdata)
{
	struct explorer_sdio_ios sdio_ios = {};
	struct sdio_func *func = NULL;

	func = pdata->func;

	sdio_ios.bus_width = tuning_width;
	sdio_ios.clock = tuning_clock;
	sdio_ios.timing = tuning_timing;

	return explorer_change_to_4bits(func, sdio_ios);
}

int explorer_sdio_execute_tuning(struct explorer_sdio_data *pdata)
{
	int ret = 0;
	struct mmc_card *card = NULL;
	struct sdio_func *func = NULL;

	func = pdata->func;
	card = func->card;

	mutex_lock(&epd_save->comm_lock);
	sdio_claim_host(func);
	ret = sdio_execute_tuning_prepare(pdata);
	if (ret) {
		pr_err("zeku:%s, explorer tuning prepare fail.\n", __func__);
		goto out;
	}

	ret = sdio_execute_tuning(card);
	sdio_release_host(func);
	if (ret) {
		if (!sdio_ios_default(pdata))
			ret = TUNING_FAIL_RECOVERABLE;
		goto out;
	}

out:
	mutex_unlock(&epd_save->comm_lock);
	return ret;
}


static int explorer_check_config_val(struct explorer_plat_data *explorer_data)
{
	if (explorer_data->cc_clk_drive_strength == 0 || explorer_data->cc_clk_drive_strength > 16)
		goto err;
	if (explorer_data->cc_cmd_drive_strength == 0 || explorer_data->cc_cmd_drive_strength > 16)
		goto err;
	if (explorer_data->cc_data_drive_strength == 0 || explorer_data->cc_data_drive_strength > 16)
		goto err;

	explorer_data->cc_clk_drive_strength = (explorer_data->cc_clk_drive_strength / 2) * 2;
	explorer_data->cc_cmd_drive_strength = (explorer_data->cc_cmd_drive_strength / 2) * 2;
	explorer_data->cc_data_drive_strength = (explorer_data->cc_data_drive_strength / 2) * 2;
	return 0;
err:
	pr_err("%s, failed, err:%d", __func__, -EINVAL);
	return -EINVAL;
}

#define DRV_STRENGTH_SHFT	9
#define DRV_STRENGTH_MASK	(0xFU << DRV_STRENGTH_SHFT)
#define DRV_STRENGTH_12	(0xCU << DRV_STRENGTH_SHFT)
#define DRV_STRENGTH_10	(0xAU << DRV_STRENGTH_SHFT)
#define DRV_STRENGTH_8	(0x8U << DRV_STRENGTH_SHFT)
#define DRV_STRENGTH_6	(0x6U << DRV_STRENGTH_SHFT)
#define DRV_STRENGTH_4	(0x4U << DRV_STRENGTH_SHFT)
#define DRV_STRENGTH_2	(0x2U << DRV_STRENGTH_SHFT)



static int explorer_config_cc_drive_strength(struct explorer_plat_data *explorer_data)
{
	struct batch_data_item *drive_strength_config = NULL;
	int i = 0, ret = 0;

	drive_strength_config = kzalloc(6 * sizeof(struct batch_data_item), GFP_KERNEL);
	if (!drive_strength_config)
		return -ENOMEM;

	ret = explorer_check_config_val(explorer_data);
	if (ret)
		goto err;
	/* IOMMUX GROUP1 BASE 0x48021000
	 * SDIO_CLK 0x48021000, default value: 0x9448
	 * SDIO_CMD 0x48021004, default value: 0x9428
	 * SDIO_DAT0 0x48021008, default value: 0x9428
	 * SDIO_DAT1 0x4802100C, default value: 0x9428
	 * SDIO_DAT2 0x48021010, default value: 0x9428
	 * SDIO_DAT3 0x48021014, default value: 0x9428
	 * */

	/* config cc clk drive strength */
	drive_strength_config[0].addr = 0x48021000;
	drive_strength_config[0].data = (0x9448 & (~DRV_STRENGTH_MASK)) | (explorer_data->cc_clk_drive_strength << DRV_STRENGTH_SHFT);

	/* config cc cmd drive strength */
	drive_strength_config[1].addr = 0x48021004;
	drive_strength_config[1].data = (0x9428 & (~DRV_STRENGTH_MASK)) | (explorer_data->cc_cmd_drive_strength << DRV_STRENGTH_SHFT);
	/* config cc data drive strength */
	for (i = 2; i < 6; i++) {
		drive_strength_config[i].addr = 0x48021000 + 0x4 * i;
		drive_strength_config[i].data = (0x9428 & (~DRV_STRENGTH_MASK)) | (explorer_data->cc_data_drive_strength << DRV_STRENGTH_SHFT);
	}

	ret = sdio_batch_write_data(explorer_data->sdio_data, drive_strength_config, 6 * sizeof(struct batch_data_item));
err:
	kfree(drive_strength_config);
	return ret;
}

static int explorer_sdio_probe(struct sdio_func *func,
		const struct sdio_device_id *id)
{
	int ret = 0;
	int i = 0;
	unsigned int state = 1;
	struct explorer_sdio_data *pdata = NULL;
	struct device *explorer_device = NULL;
	struct explorer_plat_data *explorer_data = NULL;
	struct mmc_card *card = func->card;

	pdata = kzalloc(sizeof(struct explorer_sdio_data), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	/* 1. initize sdio device data */
	pdata->func = func;
	card->quirks = MMC_QUIRK_BLKSZ_FOR_BYTE_MODE;
	/* 2. initize data related atu */
	INIT_LIST_HEAD(&pdata->byte_current_list);
	INIT_LIST_HEAD(&pdata->block_current_list);
	pdata->byte_free_list = BYTE_FREE_LIST_INIT_STATE;
	pdata->block_free_list = BLOCK_FREE_LIST_INIT_STATE;

	sdio_set_drvdata(func, pdata);

	/* find explorer platform device and set its drvdata */
	explorer_device = bus_find_device_by_name(&platform_bus_type, NULL, "soc:zeku,explorer@0");
	if (!explorer_device) {
		pr_err("zeku:%s, explorer platform device not found.\n", __func__);
		ret = -ENODEV;
		goto err_find_plt_err;
	}
	explorer_data = dev_get_drvdata(explorer_device);
	epd_save = explorer_data;
	explorer_data->sdio_data = pdata;

	sdio_claim_host(func);
	sdio_ios_default(pdata);
	sdio_release_host(func);
	pr_debug("zeku:explorer_sdio_probe is runed\n");

	ret = sdio_explorer_claim_func(func);
	if (ret)
		goto claimfailed;

	/* register and enable interrupt */
	ret = sdio_claim_irq(pdata->func, sdio_data1_irq);
	if (ret)
		goto claim_irq_failed;

	/* set block size */
	ret = sdio_explorer_set_block_size(func, func->max_blksize);
	if (ret) {
		pr_err("zeku: %s, set block size failed", __func__);
		goto setblockfailed;
	}

	sdio_explorer_release_func(func);
#ifdef ENABLEDATA1IRQ
	ret = sdio_write_register(pdata, SDIO_FN1R_HOST_INT_STSMSK, 0x3f, FUNC1);
	if (ret) {
		pr_err("zeku: %s, enable all card to host interrupts failed", __func__);
	}
#endif

	ret = sdio_update_remap(pdata->func, 0x178000, 14, FUNC0);

	/* configure atu and remap registers */
	for(i=0;i<SDIO_ATU_CONFIG_NUM;i++)
	{
		sdio_set_atu_remap(pdata->func, atu_remap_table[i].atu, atu_remap_table[i].remap, i,0);
	}

	ret = sdio_asr_io_enable_func(func);
	if (ret)
		goto enablefailed;

	ret = explorer_config_cc_drive_strength(explorer_data);
	if (ret) {
		pr_err("%s, failed to configure cc drive strength", __func__);
		goto config_cc_failed;
	}

	/* init irq */
	ret = explorer_init_irq(explorer_data);
	if (ret) {
		pr_err("%s, failed to init irq.\n", __func__);
		goto init_irq_failed;
	}

	explorer_bootstage_pre(explorer_data);

	atomic_set(&(explorer_data->ebs.flashable), 1);

	ret = explorer_genl_mcast_data(explorer_data, HAL_CMD_SDIO_DETECTED, (void *)&state, sizeof(unsigned int));
	if (ret)
		goto to_userspace_failed;

	pm_runtime_put_noidle(func->card->host->parent);
	explorer_data->hw_probe_done = true;

	pr_debug("zeku:explorer_sdio_probe done %d\n",__LINE__);
	return ret;
to_userspace_failed:
	devm_free_irq(&(epd_save->plat_dev->dev), epd_save->bsp_irq, epd_save);
init_irq_failed:
config_cc_failed:
enablefailed:
	sdio_explorer_claim_func(func);
setblockfailed:
	sdio_release_irq(func);
claim_irq_failed:
	sdio_explorer_release_func(func);
claimfailed:
err_find_plt_err:
	kfree(pdata);
	sdio_set_drvdata(func, NULL);
	explorer_data->sdio_data = NULL;
	return ret;

}

static void explorer_sdio_remove(struct sdio_func *func)
{
	struct explorer_sdio_data *pdata = sdio_get_drvdata(func);
	struct device *dev = &(epd_save->plat_dev->dev);
	unsigned int state = 0;
	int ret = 0;

	pm_runtime_get_noresume(func->card->host->parent);
	devm_free_irq(dev, epd_save->bsp_irq, epd_save);
	sdio_asr_io_disable_func(func);
	sdio_claim_host(func);
	sdio_release_irq(func);
	sdio_release_host(func);
	mutex_lock(&epd_save->comm_lock);
	atomic_set(&(epd_save->ebs.flashable), 0);
	atomic_set(&(epd_save->ebs.rtos_on), 0);
	if (pdata != NULL) {
		free_current_list(&pdata->byte_current_list, &pdata->byte_current_atu_num);
		free_current_list(&pdata->block_current_list, &pdata->block_current_atu_num);
		kfree(pdata);
		epd_save->sdio_data = NULL;
	}
	mutex_unlock(&epd_save->comm_lock);
	ret = explorer_genl_mcast_data(epd_save, HAL_CMD_SDIO_DETECTED, (void *)&state, sizeof(unsigned int));
	if (ret)
		pr_err("zeku:%s, failed to send msg to userspace, err:%d", __func__, ret);
	epd_save->hw_probe_done = false;
	complete(&epd_save->sdio_remove_completion);

	pr_err("zeku:%s, sdio driver is removed\n", __func__);

	return;
}


static const struct sdio_device_id explorer_sdio_ids[] = {
	{ SDIO_DEVICE(SDIO_VENDOR_ID,SDIO_DEVICE_ID)},
	{ /* end: all zeroes */			},
	{ /* end: all zeroes */			},
};

MODULE_DEVICE_TABLE(sdio,  explorer_sdio_ids);

static struct sdio_driver explorer_sdio_driver = {
	.probe      = explorer_sdio_probe,
	.remove     = explorer_sdio_remove,
	.name       = SDIO_DRIVER_NAME,
	.id_table   = explorer_sdio_ids,
};

int explorer_sdio_init(void)
{
	int ret = 0;
	ret = sdio_register_driver(&explorer_sdio_driver);
	if (ret)
		goto err;


	pr_debug("zeku:explorer_sdio_init is ok\n");

	return 0;

err:
	pr_debug("zeku:sdio_register_driver is err\n");
	sdio_unregister_driver(&explorer_sdio_driver);
	return ret;
}

void explorer_sdio_exit(void)
{
	sdio_unregister_driver(&explorer_sdio_driver);
	pr_debug("zeku:explorer_sdio_exit is runed\n");
}


