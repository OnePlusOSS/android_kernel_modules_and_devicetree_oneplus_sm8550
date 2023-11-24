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
 * Basecode Created :        2020/1/20 Author: zf@zeku.com
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_device.h>
#include <linux/uaccess.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/host.h>
#include <linux/delay.h>
#include <linux/thermal.h>

#ifdef OPLUS_EXPLORER_PLATFORM_QCOM
#include <linux/clk.h>
#else
#include "mtk_clkbuf_common.h"
#include "mtk_clkbuf_ctl.h"
#endif

#include <linux/time.h>

#include <linux/timekeeping.h>
#include "include/main.h"
#include "include/hal_protocol.h"
#include "include/irq.h"
#include "include/power.h"
#include "include/sdio_pi.h"
#include "include/uapi/explorer_uapi.h"
#include "include/ap_boot.h"
#include "include/rtt_debug.h"
#include "include/ipc.h"
#include "include/exception.h"
#ifdef SLT_ENABLE
#include "../slt/include/slt.h"
#endif

int sdio_clock = 50000000;
module_param(sdio_clock, int, 0644);
int tuning_clock = 100000000;//202000000
module_param(tuning_clock, int, 0644);
int sdio_width = SDIO_BUS_WIDTH_4BIT;
module_param(sdio_width, int, 0644);
int tuning_width = SDIO_BUS_WIDTH_4BIT;
module_param(tuning_width, int, 0644);
int sdio_timing = MMC_TIMING_UHS_SDR25;
module_param(sdio_timing, int, 0644);
int tuning_timing = MMC_TIMING_UHS_SDR104;
module_param(tuning_timing, int, 0644);

#ifdef SDIO_PRESSURE_TEST
static char *sdio_press_data;
#define EXPLORER_IOC_TEST_MSG_HDR   (12)
#define EXPLORER_IOC_TEST_MSG_LEN   (2*512*512+EXPLORER_IOC_TEST_MSG_HDR)
#endif
static bool log_stopped = 0;

#define EXPLORER_TOKEN_INVALID_CHECK(token) 						\
	if (token == NULL) {											\
		pr_err("%s, null pointer error! return EINVAL.\n", __func__);	\
		return -EINVAL;												\
	}
extern int explorer_load_fw(struct explorer_plat_data *epd,
                           const char *firmware_name, u32 addr);
extern int explorer_boot(struct explorer_plat_data *epd);
extern int explorer_boot_from_pbl(struct explorer_plat_data *epd);
extern int explorer_load_npu(struct explorer_plat_data *epd);
extern int explorer_load_npu_custom(struct explorer_plat_data *epd);
extern int explorer_stop_pbl_log(struct explorer_plat_data *epd);
extern int explorer_clear_pbl_log(struct explorer_plat_data *epd);
extern void explorer_get_pbl_log_config(struct explorer_plat_data *epd,
                                        struct explorer_log_reply *reply);
extern int explorer_send_mbox(struct explorer_plat_data *epd,
			      u32 sync, u32 mod_id,
			      u32 sub_id, void *data);

static void explorer_awake_init(struct explorer_plat_data *chip)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	wake_lock_init(&chip->suspend_lock, WAKE_LOCK_SUSPEND,
		       "explorer_wakelock");
#else
	chip->suspend_ws = wakeup_source_register(NULL, "explorer_wakelock");
	pr_err("%s wakeup_source_register:%p.\n", __func__, chip->suspend_ws);
#endif
}

static void explorer_awake_exit(struct explorer_plat_data *chip)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	wake_lock_destroy(&chip->suspend_lock);
#else
	wakeup_source_unregister(chip->suspend_ws);
#endif
}

static int explorer_open(struct inode *nodp, struct file *filp)
{
	struct miscdevice *miscdev = (struct miscdevice *)filp->private_data;
	filp->private_data = container_of(miscdev, struct explorer_plat_data, dev);
#ifdef SDIO_PRESSURE_TEST
	sdio_press_data = kzalloc(EXPLORER_IOC_TEST_MSG_LEN, GFP_KERNEL);
	if (!sdio_press_data) {
			pr_err("%s, alloc memory failed.\n", __func__);
			return -ENOMEM;
	}
#endif
	pr_info("%s, done.\n", __func__);
	return 0;
}

static int explore_close(struct inode *nodp, struct file *filp)
{
	struct explorer_plat_data *epd = filp->private_data;
	int ret = 0;
	struct aon_sensor_cmd *  data = NULL;
	data = kzalloc(sizeof(struct aon_sensor_cmd), GFP_KERNEL);
	if (!data) {
		pr_err("%s, alloc memory failed.\n", __func__);
		return -ENOMEM;
	}
#ifdef SDIO_PRESSURE_TEST
	if (sdio_press_data) {
		kfree(sdio_press_data);
		sdio_press_data = NULL;
	}
#endif
	data->aon_cmd_opcode = AON_SENSOR_CMD_RELEASE;
	data->aon_cmd_bufsize = 0;

	if (epd->aon_data){
		pr_info("%s before epd->aon_data: %p", __func__, epd->aon_data);
		ret = explorer_aon_drv_cmd((void**)(&epd->aon_data), data);
		pr_info("%s after epd->aon_data: %p", __func__, epd->aon_data);
		if (ret < 0)
			pr_err("%s, explorer_aon_drv_cmd failed.\n", __func__);
	}

	kfree(data);
	pr_info("%s, done.\n", __func__);
	return 0;
}

static int explorer_rw_allow_check(struct explorer_plat_data *epd, u32 rw_addr, size_t count) 
{
	int ret = 0;
	if ((!epd->hw_probe_done)
		|| (((rw_addr >= DDR_START_ADDR && rw_addr <= DDR_MAX_END_ADDR)
				|| (rw_addr < DDR_START_ADDR && (rw_addr + count) >= DDR_START_ADDR))
				&& (atomic_read(&epd->ebs.current_stage) < DDR_OK))) { //if addr and addr+count is in DDR area, check current stage
		pr_err("%s, CC area is not allowed to read/write, probe[%d], addr[0x%x], count[%d], stage[%d]\n",
				__func__, epd->hw_probe_done, rw_addr, count, atomic_read(&epd->ebs.current_stage));
		ret = -EINVAL;
	}
	return ret;
}

static ssize_t explorer_write(struct file *filp, const char __user *buffer, size_t count, loff_t *ppos)
{
	int ret = 0;
	struct explorer_plat_data *epd = filp->private_data;
	struct device *dev = &(epd->plat_dev->dev);
	struct explorer_gen_data_header data_header;
	void *ipc_buf = NULL;
	u32 aligned_size = 0;
	u32 data_mode = 0;

	/* copy generic header from userland */
	if (count != sizeof(struct explorer_gen_data_header)) {
		pr_err("%s, generic ipc data size is invalid.\n", __func__);
		goto out;
	}

	ret = copy_from_user(&data_header, buffer, count);
	if (ret) {
		pr_err("%s, could not copy data from userland.\n", __func__);
		goto out;
	}
	pr_debug("%s, src mod id: 0x%x.\n", __func__, data_header.src_ep.mod_id);
	pr_debug("%s, dst mod id: 0x%x.\n", __func__, data_header.dst_ep.mod_id);
	pr_debug("%s, xfer mode: 0x%x.\n", __func__, data_header.mode);
	pr_debug("%s, dst addr: 0x%x.\n", __func__, data_header.dst_addr);
	pr_debug("%s, payload size: 0x%x.\n", __func__, data_header.psize);

	/* check status */
	if ((data_header.dst_addr != IPC_AUTO_ALLOC_ADDR) &&
		(explorer_rw_allow_check(epd, data_header.dst_addr, count))) {
			pr_err("%s, check address failed.\n", __func__);
			return -EINVAL;
		}

	/* check payload size */
	if (data_header.psize>IPC_GEN_DATA_SIZE_MAX) {
		pr_err("%s, payload size is invalid.\n", __func__);
		goto out;
	}

	/* allocate ipc buffer */
	aligned_size = (data_header.psize + 0x03) & (~0x03);
	pr_debug("%s, aligned total size: %d.\n", __func__, aligned_size);
	ipc_buf = devm_kzalloc(dev, aligned_size, GFP_KERNEL);
	if (!ipc_buf) {
		pr_err("%s, allocate buffer failed.\n", __func__);
		return -ENOMEM;
	}

	/* copy payload data from userland */
	ret = copy_from_user(ipc_buf, data_header.payload, data_header.psize);
	if (ret) {
		pr_err("%s, could not copy data from userland.\n", __func__);
		goto err_free;
	}

	/* send total packet to Explorer */
	data_mode = IPC_GET_DATA_MODE(data_header.mode);
	if (IPC_GEN_DATA == data_mode) {
		ret = explorer_write_generic_data(epd, ipc_buf, aligned_size, &data_header);
		if (ret<0) {
			pr_err("%s, explorer_write_generic_data failed.\n", __func__);
			goto err_free;
		}
	} else if (IPC_GEN_CMD == data_mode) {
		if (aligned_size > MBOX_DATA_LEN) {
			pr_err("%s, invalid payload size: %d.\n", __func__, aligned_size);
			ret = -EINVAL;
			goto err_free;
		}
		ret = explorer_write_generic_cmd(epd, ipc_buf, aligned_size, &data_header);
		if (ret<0) {
			pr_err("%s, explorer_write_generic_cmd failed.\n", __func__);
			goto err_free;
		}
	} else {
		pr_err("%s, invalid xfer data mode: %d.\n", __func__, data_mode);
		ret = -EINVAL;
		goto err_free;
	}

	pr_debug("%s, done.\n", __func__);
err_free:
	devm_kfree(dev, ipc_buf);
out:
	return (ret?ret:count);
}

static loff_t explorer_seek(struct file *file, loff_t offset, int flag)
{
	loff_t new_pos = 0;

	switch (flag) {
		case SEEK_SET:
			new_pos = offset;
			break;
		case SEEK_CUR:
			new_pos = file->f_pos + offset;
			break;
		case SEEK_END:
			new_pos = SC_READ_DATA_SIZE_MAX - offset;
			break;
		default:
			pr_err("%s, unsupported flag %d.\n", __func__, flag);
			return -EINVAL;
	}

	if(new_pos < 0)
		new_pos = 0;
	file->f_pos = new_pos;
	pr_debug("%s, ppos = 0x%x.\n", __func__, (u32)new_pos);

	return new_pos;
}

static ssize_t explorer_read(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
	int ret = 0;
	struct explorer_plat_data *epd = filp->private_data;
	void *rd_buf = NULL;
	u32 cc_addr = *ppos;
	u32 rd_quotient = count / SC_READ_BUF_SIZE;
	u32 rd_remainder = count % SC_READ_BUF_SIZE;
	int i = 0;

	/* check size */
	if (count > SC_READ_DATA_SIZE_MAX) {
		pr_err("%s, size %d exceeds max limit %d.\n", __func__,
			count, SC_READ_DATA_SIZE_MAX);
		return -EINVAL;
	}
	/* check status */
	if (explorer_rw_allow_check(epd, cc_addr, count)) {
		return -EINVAL;
	}

	/* allocate ipc buffer */
	rd_buf = SHMEM_IPC_BATCH_BASE(epd->shared_buffer);

	/* read & copy to user */
	mutex_lock(&epd->sc_rd_lock);
	for (i = 0; i < rd_quotient; i++) {
		ret = explorer_hal_sync_read(epd, cc_addr+i*SC_READ_BUF_SIZE,
					     rd_buf, SC_READ_BUF_SIZE);
		if (ret<0) {
			pr_err("%s, explorer_hal_sync_read failed.\n", __func__);
			goto err;
		}

		/* data to user */
		ret = copy_to_user((char __user *)(buffer+i*SC_READ_BUF_SIZE),
				   rd_buf, SC_READ_BUF_SIZE);
		if (ret) {
			pr_err("%s, can not copy data to user.\n", __func__);
			goto err;
		}
	}

	if (rd_remainder != 0) {
		ret = explorer_hal_sync_read(epd, cc_addr+i*SC_READ_BUF_SIZE,
					     rd_buf, rd_remainder);
		if (ret<0) {
			pr_err("%s, explorer_hal_sync_read failed.\n", __func__);
			goto err;
		}
		/* data to user */
		ret = copy_to_user((char __user *)(buffer+i*SC_READ_BUF_SIZE),
				   rd_buf, rd_remainder);
		if (ret) {
			pr_err("%s, can not copy data to user.\n", __func__);
			goto err;
		}
	}

	pr_debug("%s, done.\n", __func__);
err:
	mutex_unlock(&epd->sc_rd_lock);
	return (ret?ret:count);
}

static long explorer_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	int ret = 0;
	struct explorer_plat_data *epd = filp->private_data;
	struct device *dev = &(epd->plat_dev->dev);
	int type = _IOC_TYPE(cmd);
	int nr = _IOC_NR(cmd);
	unsigned int param_size = _IOC_SIZE(cmd);
#ifdef SDIO_PRESSURE_TEST
	unsigned int addr, blocksize, bufsize;
#endif

	pr_debug("%s, type = %d,nr = %d,param size = %d \n", __func__, type, nr, param_size);
	//pr_debug("%s, nr = %d.\n", __func__, nr);
	//pr_debug("%s, param size = %d.\n", __func__, param_size);

	if (type != EXPLORER_IOC_MAGIC) {
		pr_err("%s, ioctl magic not match, type:0x%x\n", __func__, type);
		return -EINVAL;
	}

	/* send cmd to explorer */
	switch (nr) {
		case IOC_NR_WR_TEST:
		{
			void *data = NULL;
			void *user_data = NULL;
			u32 gen_cmd = ((IPC_GEN_MSGID_TEST) << 16) + HAL_CMD_GEN;
			struct explorer_sync_reply *sync_reply = NULL;
			if (param_size > EXPLORER_IOC_LEN_MAX) {
				pr_err("%s, ioctl param size exceeds max length.\n", __func__);
				return -EINVAL;
			}
			data = devm_kzalloc(dev, param_size, GFP_KERNEL);
			if (!data) {
				pr_err("%s, alloc memory failed.\n", __func__);
				return -ENOMEM;
			}
			ret = copy_from_user(data, (char __user *)arg, param_size);
			if (ret) {
				pr_err("%s, can not copy explorer_data from user, ret = 0x%x.\n", __func__, ret);
				goto test_wt_out;
			}
			user_data = (void *)((char *)data + sizeof(struct explorer_sync_reply));
			sync_reply = (struct explorer_sync_reply *)data;

			ret = explorer_write_ipc_data_wait(epd, gen_cmd, user_data, param_size, sync_reply);
			if (ret<0) {
				pr_err("%s, explorer_write_ipc_data_wait failed.\n", __func__);
				goto test_wt_out;
			}
			pr_debug("%s, write IOC_NR_WR_TEST done.\n", __func__);
test_wt_out:
			devm_kfree(dev, data);
			break;
		}
		case IOC_NR_ISP_NOWAIT:
		{
			void *data = NULL;
			if (param_size > EXPLORER_IOC_LEN_MAX) {
				pr_err("%s, ioctl param size exceeds max length.\n", __func__);
				return -EINVAL;
			}
			data = devm_kzalloc(dev, param_size, GFP_KERNEL);
			if (!data) {
				pr_err("%s, alloc memory failed.\n", __func__);
				return -ENOMEM;
			}
			ret = copy_from_user(data, (char __user *)arg, param_size);
			if (ret) {
				pr_err("%s, can not copy explorer_data from user, ret = 0x%x.\n", __func__, ret);
				goto isp_nw_out;
			}
			ret = explorer_write_ipc_data_nowait(epd, HAL_CMD_ISP, (void *)data, param_size);
			if (ret<0) {
				pr_err("%s, write IOC_NR_ISP_NOWAIT failed.\n", __func__);
				goto isp_nw_out;
			}
			pr_debug("%s, write IOC_NR_ISP_NOWAIT done.\n", __func__);
isp_nw_out:
			devm_kfree(dev, data);
			break;
		}
		case IOC_NR_ISP_WAIT:
		{
			void *data = NULL;
			void *user_data = NULL;
			struct explorer_sync_reply *sync_reply = NULL;
			if (param_size > EXPLORER_IOC_LEN_MAX) {
				pr_err("%s, ioctl param size exceeds max length.\n", __func__);
				return -EINVAL;
			}
			data = devm_kzalloc(dev, param_size, GFP_KERNEL);
			if (!data) {
				pr_err("%s, alloc memory failed.\n", __func__);
				return -ENOMEM;
			}
			ret = copy_from_user(data, (char __user *)arg, param_size);
			if (ret) {
				pr_err("%s, can not copy explorer_data from user, ret = 0x%x.\n", __func__, ret);
				goto isp_wt_out;
			}
			user_data = (void *)((char *)data + sizeof(struct explorer_sync_reply));
			sync_reply = (struct explorer_sync_reply *)data;

			ret = explorer_write_ipc_data_wait(epd, HAL_CMD_ISP, user_data, param_size, sync_reply);
			if (ret<0) {
				pr_err("%s, explorer_write_ipc_data_wait failed.\n", __func__);
				goto isp_wt_out;
			}
			pr_debug("%s, write IOC_NR_ISP_WAIT done.\n", __func__);
isp_wt_out:
			devm_kfree(dev, data);
			break;
		}
		case IOC_NR_CAMERA_NOWAIT:
		{
			void *data = NULL;
			if (param_size > EXPLORER_IOC_LEN_MAX) {
				pr_err("%s, ioctl param size exceeds max length.\n", __func__);
				return -EINVAL;
			}
			data = devm_kzalloc(dev, param_size, GFP_KERNEL);
			if (!data) {
				pr_err("%s, alloc memory failed.\n", __func__);
				return -ENOMEM;
			}
			ret = copy_from_user(data, (char __user *)arg, param_size);
			if (ret) {
				pr_err("%s, can not copy explorer_data from user, ret = 0x%x.\n", __func__, ret);
				goto camera_nw_out;
			}
			ret = explorer_write_ipc_data_nowait(epd, HAL_CMD_CAMERA, (void *)data, param_size);
			if (ret<0) {
				pr_err("%s, explorer_write_ipc_data_nowait failed.\n", __func__);
				goto camera_nw_out;
			}
			pr_debug("%s, write IOC_NR_CAMERA_NOWAIT done.\n", __func__);
camera_nw_out:
			devm_kfree(dev, data);
			break;
		}
		case IOC_NR_PBL:
		{
			void *data = NULL;
			void *user_data = NULL;
			struct explorer_pbl_info *pbl_data = NULL;
			u32 ipc_id = HAL_CMD_PBL;
			if (param_size > EXPLORER_IOC_LEN_MAX) {
				pr_err("%s, ioctl param size exceeds max length.\n", __func__);
				return -EINVAL;
			}
			data = devm_kzalloc(dev, param_size, GFP_KERNEL);
			if (!data) {
				pr_err("%s, alloc memory failed.\n", __func__);
				return -ENOMEM;
			}
			ret = copy_from_user(data, (char __user *)arg, param_size);
			if (ret) {
				pr_err("%s, can not copy explorer_data from user, ret = 0x%x.\n", __func__, ret);
				goto pbl_out;
			}
			user_data = (void *)((char *)data + sizeof(struct explorer_pbl_info));
			pbl_data = (struct explorer_pbl_info *)data;
			ipc_id = CON_IPC_ID(HAL_CMD_PBL,pbl_data->sub_id);
			ret = explorer_write_data_wait(epd, ipc_id, pbl_data->addr,
						       user_data, pbl_data->size, &pbl_data->reply);
			if (ret<0) {
				pr_err("%s, explorer_write_data_wait failed.\n", __func__);
				goto pbl_out;
			}
			pr_info("%s, write IOC_NR_PBL done.\n", __func__);
pbl_out:
			devm_kfree(dev, data);
			break;
		}
		case IOC_NR_PBL_LOG_GET:
		{
			struct explorer_log_reply elr={0};

			//fill struct
			explorer_get_pbl_log_config(epd, &elr);
			if (elr.len != 0) {
				//send reg bit
				log_stopped = !explorer_stop_pbl_log(epd);
			}

			ret = copy_to_user((char __user *)arg, &elr, param_size);
			if (ret) {
				pr_err("%s, can not copy explorer_log_reply to user.\n", __func__);
				return ret;
			}
			pr_debug("%s, IOC_NR_PBL_LOG_GET done, addr:0x%x, len:%d\n", __func__, elr.addr, elr.len);
			break;
		}
		case IOC_NR_PBL_LOG_CLEAN_FLAG:
		{
			//fill struct
			if (log_stopped) {
				log_stopped = explorer_clear_pbl_log(epd);
			}

			ret = !!log_stopped;
			pr_debug("%s, IOC_NR_PBL_LOG_CLEAN_FLAG done, ret:%d\n", __func__, ret);
			break;
		}
#ifdef SDIO_PRESSURE_TEST
		case IOC_NR_WRITE_TEST:
		{
			ret = copy_from_user(&param_size, (char __user *)(arg+8), 4);
			if (ret) {
				pr_err("%s, can not copy explorer_data from user.\n", __func__);
				return ret;
			}
			pr_info("%s, buf size = %d.\n", __func__, param_size);
			param_size += EXPLORER_IOC_TEST_MSG_HDR;
			pr_info("%s, param size = %d.\n", __func__, param_size);
			if (param_size > EXPLORER_IOC_TEST_MSG_LEN) {
				pr_err("%s, max data length is:%d.\n", __func__, EXPLORER_IOC_TEST_MSG_LEN - EXPLORER_IOC_TEST_MSG_HDR);
				return -EINVAL;
			}
			ret = copy_from_user(sdio_press_data, (char __user *)arg, param_size);
			if (ret) {
				pr_err("%s, can not copy explorer_data from user.\n", __func__);
				return ret;
			}
			addr = *((unsigned int *)sdio_press_data);
			blocksize = *((unsigned int *)(sdio_press_data + 4));
			bufsize = *((unsigned int *)(sdio_press_data + 8));
			pr_info("%s, addr = %d, bufsize = %d\n", __func__, addr, bufsize);
			ret = explorer_hal_sync_write(epd,
				addr,
				(sdio_press_data + EXPLORER_IOC_TEST_MSG_HDR),
				bufsize);
			if (ret < 0) {
				pr_info("%s, write EXPLORER_IOC_WR_TEST fail.\n", __func__);
				goto out;
			}
			pr_info("%s, write EXPLORER_IOC_WR_TEST done.\n", __func__);
			break;
		}
		case IOC_NR_READ_TEST:
		{
			ret = copy_from_user(&param_size, (char __user *)(arg+8), 4);
			if (ret) {
				pr_err("%s, can not copy explorer_data from user.\n", __func__);
				return ret;
			}
			param_size += EXPLORER_IOC_TEST_MSG_HDR;
			if (param_size > EXPLORER_IOC_TEST_MSG_LEN) {
				pr_err("%s, max data length is:%d.\n", __func__, EXPLORER_IOC_TEST_MSG_LEN - EXPLORER_IOC_TEST_MSG_HDR);
				return -EINVAL;
			}
			pr_info("%s, buf size = %d.param size = %d.\n", __func__, param_size, param_size);
			//pr_info("%s, param size = %d.\n", __func__, param_size);
			ret = copy_from_user(sdio_press_data, (char __user *)arg, param_size);
			if (ret) {
				pr_err("%s, can not copy explorer_data from user.\n", __func__);
				return ret;
			}
			addr = *((unsigned int *)sdio_press_data);
			blocksize = *((unsigned int *)(sdio_press_data + 4));
			bufsize = *((unsigned int *)(sdio_press_data + 8));
			pr_info("%s, addr = %d, bufsize = %d\n", __func__, addr, bufsize);
			ret = explorer_hal_sync_read(epd,
				addr,
				(sdio_press_data + EXPLORER_IOC_TEST_MSG_HDR),
				bufsize);
			if (ret < 0) {
				pr_info("%s, read EXPLORER_IOC_RD_TEST fail.\n", __func__);
				goto out;
			}
			pr_info("%s, read EXPLORER_IOC_RD_TEST done.\n", __func__);
			ret = copy_to_user((char __user *)arg, sdio_press_data, param_size);
			if (ret) {
				pr_err("%s, can not copy explorer_data to user.\n", __func__);
				return ret;
			}
			break;
		}
#endif
		case IOC_NR_WRITE_DATA:
		{
			struct explorer_training_data data_info;
			if (param_size > sizeof(struct explorer_training_data)) {
				pr_err("%s, ioctl param size (%d) exceeds the limit (%d).\n", __func__,
					param_size, sizeof(struct explorer_training_data));
				return -EINVAL;
			}
			ret = copy_from_user(&data_info, (char __user *)(arg), param_size);
			if (ret) {
				pr_err("%s, can not copy explorer_data from user.\n", __func__);
				return ret;
			}
			pr_info("%s, IOC_NR_WRITE_DATA get writing data size:%u.\n", __func__, data_info.len);
			training_data_size = data_info.len;
			if (training_data_size > TRAINING_DATA_BUF_SIZE) {
				pr_err("%s, ioctl training size (%d) exceeds the limit (%d).\n", __func__,
					training_data_size, TRAINING_DATA_BUF_SIZE);
				return -EINVAL;
			}
			ret = copy_from_user(training_data_buf, data_info.addr, data_info.len);
			if (ret) {
				pr_err("%s, can not copy explorer_data from user.\n", __func__);
				return ret;
			}
			pr_info("%s, IOC_NR_WRITE_DATA get writing data done.\n", __func__);
			break;
		}
		case IOC_NR_BOOT:
		{
			struct explorer_boot_info boot_info;
			if (param_size > sizeof(struct explorer_boot_info)) {
				pr_err("%s, ioctl param size (%d) exceeds the limit (%d).\n", __func__,
					param_size, sizeof(struct explorer_boot_info));
				return -EINVAL;
			}
			ret = copy_from_user(&boot_info, (char __user *)(arg), param_size);
			if (ret) {
				pr_err("%s, can not copy explorer_boot_info from user.\n", __func__);
				return -EFAULT;
			}
			epd->ebi = boot_info;

			if (epd->boot_force == FORCE_RLS) {
				pr_info("ap_boot was forced to Release\n");
				epd->ebi.rmod = RTOS_RLS;
				epd->ebi.rpos = RTOS_SRAM;
			} else if (epd->boot_force == FORCE_DBG) {
				pr_info("ap_boot was forced to Debug\n");
				epd->ebi.rmod = RTOS_NORMAL;
				epd->ebi.rpos = RTOS_DDR;
			} else if (epd->boot_force == FORCE_PLAT) {
				pr_info("ap_boot was forced to Plat\n");
				epd->ebi.rmod = RTOS_PLAT;
				epd->ebi.rpos = RTOS_SRAM;
			} else if (epd->boot_force == FORCE_DBG_NOPM) {
				pr_info("ap_boot was forced to debug_noPM\n");
				epd->ebi.rmod = RTOS_DBG_NOPM;
				epd->ebi.rpos = RTOS_DDR;
			} else if (epd->boot_force == FORCE_RLS_NOPM) {
				pr_info("ap_boot was forced to release_noPM\n");
				epd->ebi.rmod = RTOS_RLS_NOPM;
				epd->ebi.rpos = RTOS_SRAM;
			}

			ret = explorer_boot(epd);
			if (ret < 0) {
				pr_err("%s, explorer_boot failed, ret = %d.\n", __func__, ret);
			} else if (ret > 0) {
				pr_info("%s, explorer_boot success with ret = %d.\n", __func__, ret);
				ret = 0;
			}
			pr_info("%s, IOC_NR_BOOT done.\n", __func__);
			break;
		}
		/*
		case IOC_NR_LOAD_NPU:
		{
			enum explorer_cam_mode cmod;
			ret = copy_from_user(&cmod, (char __user *)(arg), param_size);
			if (ret) {
				pr_err("%s, can not copy explorer_cam_mode from user.\n", __func__);
				return -EFAULT;
			}
			epd->cmod = cmod;
			pr_info("ap_boot: the cam mode is %d.\n", epd->cmod);
			ret = explorer_load_npu(epd);
			if (ret < 0) {
				pr_err("%s, explorer_load_npu failed, ret = %d.\n", __func__, ret);
			} else if (ret > 0) {
				pr_info("%s, explorer_load_npu success with ret = %d.\n", __func__, ret);
				ret = 0;
			}
			pr_info("%s, IOC_NR_LOAD_NPU done.\n", __func__);
			break;
		}
		*/
		case IOC_NR_LOAD_NPU:
		{
			void *data = NULL;

			if (param_size > sizeof(struct explorer_npu_mod_info)) {
				pr_err("%s, ioctl param size (%d) exceeds the limit (%d).\n", __func__,
					param_size, sizeof(struct explorer_npu_mod_info));
				return -EINVAL;
			}

			data = devm_kzalloc(dev, param_size, GFP_KERNEL);
			if (!data) {
				pr_err("%s, alloc memory failed.\n", __func__);
				return -ENOMEM;
			}

			ret = copy_from_user(data, (char __user *)arg, param_size);
			if (ret) {
				pr_err("%s, can not copy explorer_nmod_info from user, ret = 0x%x.\n", __func__, ret);
				goto td_out;
			}

			epd->nmod_info = (struct explorer_npu_mod_info *)data;
			epd->nmod_info->name[NMOD_NAME_LEN_MAX] = '\0';
			ret = explorer_load_npu_custom(epd);
			if (ret < 0) {
				pr_err("%s, explorer_load_npu_custom failed.\n", __func__);
				goto ln_out;
			}
			pr_info("%s, IOC_NR_LOAD_NPU done.\n", __func__);
ln_out:
			devm_kfree(dev, data);
			break;
		}
		case IOC_NR_GET_SEC_STATE:
		{
			enum explorer_secure_state st;
			//a boot to bootrom.
			epd->ebi.target_stage = BOOTROM;
			epd->ebi.dmod = DDR_FIRST;
			epd->ebi.mmod = MIPI_BYPASS;
			epd->ebi.rmod = RTOS_NORMAL;
			epd->ebi.rpos = RTOS_SRAM;
			epd->ebi.pmod = PLAT_QCOM;
			epd->ebi.boot_timeout = 1100;

			pr_err("explorer_boot for secure state: tstg=%d, dmod=%d, mmod=%d, rmod=%d, rpos=%d, pmod=%d, to=%d.\n",
			epd->ebi.target_stage, epd->ebi.dmod, epd->ebi.mmod,
			epd->ebi.rmod, epd->ebi.rpos, epd->ebi.pmod,
			epd->ebi.boot_timeout);

			ret = explorer_boot(epd);

			if (ret < 0) {
				pr_err("%s, read sec state failed.\n", __func__);
				return -EINVAL;
			}

			if (epd->ebs.is_cc_sec_boot) {
				pr_info("%s, get secure state: SECURE.\n", __func__);
				st = STAT_SECURE;
			} else {
				pr_info("%s, get secure state: NON-SEC.\n", __func__);
				st = STAT_NONSEC;
			}

			ret = copy_to_user((char __user *)arg, &st, sizeof(enum explorer_secure_state));
			if (ret < 0) {
				pr_err("%s, copy sec_state to user failed.\n", __func__);
			}

			//a power off.
			power_clock_control_explorer(epd, false);

			break;
		}
		case IOC_NR_PBL_MIPI_BP:
		{
			struct mipi_params_t mpara;
			if (param_size > sizeof(struct mipi_params_t)) {
				pr_err("%s, ioctl param size (%d) exceeds the limit (%d).\n", __func__,
					param_size, sizeof(struct mipi_params_t));
				return -EINVAL;
			}
			ret = copy_from_user(&mpara, (char __user *)(arg), param_size);
			if (ret) {
				pr_err("%s, can not copy mipi_params_t from user.\n", __func__);
				return -EFAULT;
			}

			ret = explorer_send_mbox(epd, HAL_CMD_ASYNC, HAL_CMD_PBL, PBL_SUB_CMD_MB, &mpara);

			if (ret < 0) {
				pr_err("%s, explorer send MIPI_BP mbox failed, ret = %d.\n", __func__, ret);
			} else if (ret > 0) {
				pr_info("%s, explorer send MIPI_BP mbox success with ret = %d.\n", __func__, ret);
				ret = 0;
			}

			pr_info("%s, IOC_NR_PBL_MIPI_BP done.\n", __func__);
			break;
		}
		case IOC_NR_BOOT_CONTINUE:
		{
			struct explorer_boot_info_override bio;
			if (param_size > sizeof(struct explorer_boot_info_override)) {
				pr_err("%s, ioctl param size (%d) exceeds the limit (%d).\n", __func__,
					param_size, sizeof(struct explorer_boot_info_override));
				return -EINVAL;
			}
			ret = copy_from_user(&bio, (char __user *)(arg), param_size);
			if (ret) {
				pr_err("%s, can not copy explorer_boot_info_override from user.\n", __func__);
				return -EFAULT;
			}
			epd->ebi.target_stage = bio.target_stage;
			epd->ebi.rmod = bio.rmod;
			epd->ebi.rpos = bio.rpos;
			epd->ebi.boot_timeout = bio.boot_timeout;

			ret = explorer_boot_from_pbl(epd);
			if (ret < 0) {
				pr_err("%s, explorer_boot_from_pbl failed, ret = %d.\n", __func__, ret);
			} else if (ret > 0) {
				pr_info("%s, explorer_boot_from_pbl success with ret = %d.\n", __func__, ret);
				ret = 0;
			}
			pr_info("%s, IOC_NR_BOOT_CONTINUE done.\n", __func__);
			break;
		}
		case IOC_NR_POWER:
		{
			enum explorer_power_info info = POWER_IOC_DATA_FIRST;
			void *data = NULL;
			if (param_size > EXPLORER_IOC_LEN_MAX) {
				pr_err("%s, ioctl param size exceeds max length.\n", __func__);
				return -EINVAL;
			}
			data = devm_kzalloc(dev, param_size, GFP_KERNEL);
			if (!data) {
				pr_err("%s, alloc memory failed.\n", __func__);
				return -ENOMEM;
			}
			ret = copy_from_user(data, (char __user *)arg, param_size);
			if (ret) {
				pr_err("%s, can not copy explorer_data from user, ret = 0x%x.\n", __func__, ret);
				goto power_out;
			}
			info = *(enum explorer_power_info *)data;
			switch (info) {
				case POWER_IOC_DATA_SUSPEND:
					PM_LOG_I("calling SUSPEND. is_sdu_clk_switched=%d\n",
							epd->is_sdu_clk_switched);
					ret = sleep_explorer(epd, false);
					break;
				case POWER_IOC_DATA_SUSPEND_FORCE:
					PM_LOG_I("calling SUSPEND_FORCE. is_sdu_clk_switched=%d\n",
							epd->is_sdu_clk_switched);
					ret = sleep_explorer(epd, true);
					break;
				case POWER_IOC_DATA_RESUME:
					PM_LOG_I("calling RESUME. is_sdu_clk_switched=%d\n",
							epd->is_sdu_clk_switched);
					ret = wakeup_explorer(epd);
					break;
				case POWER_IOC_DATA_POWER_ON:
					PM_LOG_I("calling POWER_ON.\n");
					power_clock_control_explorer(epd, true);
					break;
				case POWER_IOC_DATA_POWER_OFF:
					PM_LOG_I("calling POWER_OFF.\n");
					power_clock_control_explorer(epd, false);
					break;
				case POWER_IOC_STATE_STANDBY:
				case POWER_IOC_STATE_BYPASS:
				case POWER_IOC_STATE_AON:
				case POWER_IOC_STATE_MISSION:
				{
					unsigned int state = (unsigned int)info;
					PM_LOG_I("calling power_state_set, state: %d.\n", state);
					ret = set_power_state_explorer(epd, state);
					break;
				}
				case POWER_IOC_STATE_GET:
				{
					unsigned int state = POWER_IOC_STATE_FIRST;
					ret = get_power_state_explorer(epd, &state);
					PM_LOG_I("calling power_state_get, state: %d, ret: %d.\n", state, ret);
					if (ret >= 0) {
						ret = copy_to_user((char __user *)arg, &state, sizeof(enum explorer_power_info));
						if (ret) {
							PM_LOG_E("can not copy explorer_data to user.\n");
						}
					}
					break;
				}
#ifdef SLT_ENABLE
				case POWER_IOC_SDIO_VDD_ON:
					PM_LOG_I("calling SDIO_VDD_ON, for slt test.\n");
					ret = regulator_control_explorer(epd, true);
					break;
				case POWER_IOC_SDIO_VDD_OFF:
					PM_LOG_I("calling SDIO_VDD_OFF, for slt test.\n");
					ret = regulator_control_explorer(epd, false);
					break;
#endif
				default:
					PM_LOG_E("%s, info: 0x%x not support.\n", __func__, info);
			}
			PM_LOG_I("%s, IOC_NR_POWER done. ret=%d\n", __func__, ret);
power_out:
			devm_kfree(dev, data);
			break;
		}
		case IOC_NR_READDUMP:
		{
			struct RttRamDumpMsg dump_msg;
			struct RttSdiMsg sdi_msg;
			dump_info get_info;
			bool enable, ddrlp2_result;
			int dump_type;
			u8 val = 0;
			long lp2timeout = 1 * HZ;
			long dumptimeout = 1 * HZ;

			if (param_size > sizeof(struct RttSdiMsg)) {
				pr_err("%s, ioctl param size (%d) exceeds the limit (%d).\n", __func__,
						param_size, sizeof(struct RttSdiMsg));
				return -EINVAL;
			}
			ret = copy_from_user(&sdi_msg, (char __user *)(arg), param_size);
			if (ret) {
				pr_err("[RAMDUMP] %s, can't copy RttSdiMsg from user.\n", __func__);
				return -EFAULT;
			}
			pr_info("[RAMDUMP] %s, subtype = 0x%x, datasize = 0x%x, data = 0x%p\n", __func__, sdi_msg.subtype, sdi_msg.datasize, sdi_msg.data);
			//pr_info("[RAMDUMP] %s, datasize = 0x%x\n", __func__,sdi_msg.datasize);
			//pr_info("[RAMDUMP] %s, data = 0x%p\n", __func__, sdi_msg.data);

			if (sdi_msg.subtype >= SDI_CMD_MAX) {
				pr_err("[RAMDUMP] %s, RttSdiMsg subtype error.\n", __func__);
				return -EINVAL;
			}

			if (sdi_msg.subtype == SDI_ENABLE_CTRL) {
				ret = explorer_hal_sync_read_internal(epd, SDIO_FN1R_SOC_CTRL_REG0, &val);
				if (ret) {
					pr_err("[RAMDUMP] get ramdump_debug failed\n");
					return -EINVAL;
				}

				if ((sdi_msg.datasize) > sizeof(bool)) {
					pr_err("%s, ioctl param size (%d) exceeds the limit (%d).\n", __func__,
							param_size, sizeof(bool));
					return -EINVAL;
				}
				ret = copy_from_user(&enable, (char __user *)(sdi_msg.data), sdi_msg.datasize);
				if (ret) {
					pr_err("[RAMDUMP] %s,can't copy RttSdiMsg.data from user.\n", __func__);
					return -EFAULT;
				}
				pr_info("[RAMDUMP] %s, enable=0x%x, val=0x%x\n", __func__, enable, val);

				if (enable) {
					val |= RAMDUMP_EN_SDU;
					ret = explorer_hal_sync_write_internal(epd, SDIO_FN1R_SOC_CTRL_REG0, val);
					if (ret) {
						pr_err("[RAMDUMP] write addr:0x%x, val: 0x%x failed!\n",
								SDIO_FN1R_SOC_CTRL_REG0, val);
						return -EINVAL;
					} else {
						pr_info("[RAMDUMP] %s, enable ramdup_debug\n", __func__);
					}
				} else {
					val &= ~RAMDUMP_EN_SDU;
					ret = explorer_hal_sync_write_internal(epd, SDIO_FN1R_SOC_CTRL_REG0, val);
					if (ret) {
						pr_err("[RAMDUMP] write addr:0x%x, val: 0x%x failed!\n",
								SDIO_FN1R_SOC_CTRL_REG0, val);
						return -EINVAL;
					} else {
						pr_info("[RAMDUMP] %s, disable ramdup_debug\n", __func__);
					}
				}
				break;
			}

			if (sdi_msg.subtype == SDI_DUMP_TYPE) {
				ret = explorer_hal_sync_read_internal(epd, SDIO_FN1R_SOC_CTRL_REG1B3, &val);
				if (ret) {
					pr_err("[RAMDUMP] get ramdump type failed\n");
					return -EINVAL;
				}

				if ((sdi_msg.datasize) > sizeof(int)) {
					pr_err("%s, ioctl param size (%d) exceeds the limit (%d).\n", __func__,
							param_size, sizeof(int));
					return -EINVAL;
				}
				ret = copy_from_user(&dump_type, (char __user *)(sdi_msg.data), sdi_msg.datasize);
				if (ret) {
					pr_err("[RAMDUMP] %s,can't copy RttSdiMsg.data from user.\n", __func__);
					return -EFAULT;
				}
				pr_info("[RAMDUMP] %s, dump_type=0x%x, reg_val=0x%x\n", __func__, dump_type, val);

				if (dump_type) {
					val |= RAMDUMP_TYPE_SDU;
					ret = explorer_hal_sync_write_internal(epd, SDIO_FN1R_SOC_CTRL_REG1B3, val);
					if (ret) {
						pr_err("[RAMDUMP] write addr:0x%x, val: 0x%x failed!\n",
								SDIO_FN1R_SOC_CTRL_REG0, val);
						return -EINVAL;
					} else {
						pr_info("[RAMDUMP] %s, set ramdump_type: FULLDUMP\n", __func__);
					}
				} else {
					val &= ~RAMDUMP_TYPE_SDU;
					ret = explorer_hal_sync_write_internal(epd, SDIO_FN1R_SOC_CTRL_REG1B3, val);
					if (ret) {
						pr_err("[RAMDUMP] write addr:0x%x, val: 0x%x failed!\n",
								SDIO_FN1R_SOC_CTRL_REG0, val);
						return -EINVAL;
					} else {
						pr_info("[RAMDUMP] %s, set ramdump_type: MINIDUMP\n", __func__);
					}
				}
				break;
			}

			if ((sdi_msg.subtype == SDI_TRIGGER_DUMP) || (sdi_msg.subtype == SDI_GET_DUMP_INFO)) {
				if ((0 == epd->is_pmic_pon) || (epd->is_pmic_pon && (0 == epd->hw_probe_done))) {
					EXCEPTION_LOG_E("can't triger dump when explorer is poweroff or not probe done\n");
					return -EINVAL;
				}

				if (1 == sdi_msg.datasize) {
					EXCEPTION_LOG_I("Set sdi just exit ddrlp2.\n");

					ret = explorer_hal_sync_read_internal(epd, AP_CTRL_REG8_SDU, &val);
					if (ret) {
						EXCEPTION_LOG_E("get SDI_DO_EXIT_DDRLP2 flag failed\n");
						return ret;
					}

					val |= SDI_DO_EXIT_DDRLP2;
					ret = explorer_hal_sync_write_internal(epd, AP_CTRL_REG8_SDU, val);
					if (ret) {
						EXCEPTION_LOG_E("set SDI_DO_EXIT_DDRLP2 flag failed\n");
						return ret;
					}
				} else {
					ret = explorer_hal_sync_read_internal(epd, SDIO_FN1R_SOC_CTRL_REG0, &val);
					if (val & RAMDUMP_EN_SDU) {
						EXCEPTION_LOG_I("trigger ramdump\n");
					} else {
						EXCEPTION_LOG_I("ramdump is disabled\n");
						return -EINVAL;
					}

				}

				EXCEPTION_LOG_W("warm reset explorer, jump to sdi\n");
				atomic_set(&(epd->ebs.rtos_on), 0); /* if ap reset M7, rtos_on flag should be cleared */
				/* Disable heartbeat detect when explorer is going to SDI */
				epd->heartbeat_started = false;
				cancel_heartbeat_detect_work(epd);

				ret = explorer_hal_sync_write_internal(epd, AP_CTRL_REG5_SDU, AP_RST_CPUCORE);
				mdelay(1);
				ret = explorer_hal_sync_write_internal(epd, AP_CTRL_REG5_SDU, 0);

				if (1 == sdi_msg.datasize) {
					lp2timeout = wait_for_completion_interruptible_timeout(&epd->sdi_ddrlp2_completion, lp2timeout);
					if (lp2timeout <= 0) {
						EXCEPTION_LOG_E("wait sdi_ddrlp2_completion timeout\n");
						ddrlp2_result = false;
					} else {
						ddrlp2_result = true;
						EXCEPTION_LOG_I("sdi exit ddrlp2 success, ap can catch log.\n");
					}

					ret = explorer_hal_sync_read_internal(epd, AP_CTRL_REG8_SDU, &val);
					if (ret) {
						EXCEPTION_LOG_E("get SDI_DO_EXIT_DDRLP2 flag failed\n");
						return -EINVAL;
					}
					val &= ~SDI_DO_EXIT_DDRLP2;
					ret = explorer_hal_sync_write_internal(epd, AP_CTRL_REG8_SDU, val);
					if (ret) {
						EXCEPTION_LOG_E("clear SDI_DO_EXIT_DDRLP2 flag failed\n");
						return -EINVAL;
					}
					EXCEPTION_LOG_E("clear SDI_DO_EXIT_DDRLP2 flag\n");

					ret = copy_to_user(sdi_msg.data, &ddrlp2_result, sizeof(bool));
				}

				if (sdi_msg.subtype == SDI_GET_DUMP_INFO) {
					epd->is_get_dumpinfo = true;
					dumptimeout = wait_for_completion_interruptible_timeout(&epd->get_dumpinfo_completion, dumptimeout);
					if (dumptimeout <= 0) {
						EXCEPTION_LOG_E("wait dump addr and len timed out\n");
						get_info.dump_addr = 0;
					} else {
						EXCEPTION_LOG_I("get dump addr and len after warm reset, ap can catch dump now.\n");
						get_info = epd->ramdump_info;
					}
					ret = copy_to_user(sdi_msg.data, &get_info, sizeof(dump_info));
				}

				break;
			}

			if (sdi_msg.subtype == SDI_RAM_DUMP) {
				if ((sdi_msg.datasize) > sizeof(struct RttRamDumpMsg)) {
					pr_err("%s, ioctl param size (%d) exceeds the limit (%d).\n", __func__,
							param_size, sizeof(struct RttRamDumpMsg));
					return -EINVAL;
				}
				ret = copy_from_user(&dump_msg, (char __user *)(sdi_msg.data), sdi_msg.datasize);
				if (ret) {
					pr_err("[RAMDUMP]%s,can't copy RttRamDumpMsg info from user.\n", __func__);
					return -EFAULT;
				}
				pr_info("[RAMDUMP] %s, source_addr = 0x%x, dump_len = 0x%x, target_addr = 0x%p\n", __func__, dump_msg.source_addr,dump_msg.dump_len,dump_msg.target_addr);
				//pr_info("[RAMDUMP] %s, dump_len = 0x%x\n", __func__,dump_msg.dump_len);
				//pr_info("[RAMDUMP] %s, target_addr = 0x%p\n", __func__, dump_msg.target_addr);

				if (dump_msg.dump_len > RAMDUMP_BUFFER_SIZE) {
					pr_err("[RAMDUMP] %s,dump_len =0x%x too big!\n", __func__, dump_msg.dump_len);
					dump_msg.dump_len = RAMDUMP_BUFFER_SIZE;
				}

				ret = explorer_hal_sync_read(epd, dump_msg.source_addr, epd->ap_buffer, dump_msg.dump_len);
				if (ret < 0) {
					pr_info("[RAMDUMP] %s, read EXPLORER_IOC_RD_TEST fail.\n", __func__);
					break;
				}

				ret = copy_to_user((char __user *)dump_msg.target_addr, epd->ap_buffer, dump_msg.dump_len)
						? -EFAULT : (dump_msg.dump_len);
				if (ret < 0) {
					pr_err("[RAMDUMP] %s, can not copy explorer_data to user.\n", __func__);
				}
				break;
			}

			break;
		}
		case IOC_NR_SEND_TUNNING_DATA:
		{
			void *data = NULL;
			struct explorer_tunning_data_info *tunning_info = NULL;
			if (param_size > sizeof(struct explorer_tunning_data_info)) {
				pr_err("%s, ioctl param size (%d) exceeds the limit (%d).\n", __func__,
					param_size, sizeof(struct explorer_tunning_data_info));
				return -EINVAL;
			}
			data = devm_kzalloc(dev, param_size, GFP_KERNEL);
			if (!data) {
				pr_err("%s, alloc memory failed.\n", __func__);
				return -ENOMEM;
			}
			ret = copy_from_user(data, (char __user *)arg, param_size);
			if (ret) {
				pr_err("%s, can not copy explorer_data from user, ret = 0x%x.\n", __func__, ret);
				goto td_out;
			}
			tunning_info = (struct explorer_tunning_data_info *)data;
			tunning_info->name[TD_NAME_LEN_MAX] = '\0';
			ret = explorer_load_fw(epd, tunning_info->name, tunning_info->addr);
			if (ret<0) {
				pr_err("%s, explorer_load_fw failed.\n", __func__);
				goto td_out;
			}
			pr_info("%s, IOC_NR_SEND_TUNNING_DATA done.\n", __func__);
td_out:
			devm_kfree(dev, data);
			break;
		}
		case IOC_NR_SECURITY:
		{
			struct explorer_security_info {
				uint32_t cmd_len;
				uint8_t cmd_buffer[256];
				struct explorer_sync_reply sync_reply;
			};

			struct explorer_security_info *sync_security_info = NULL;

			pr_info("%s, write EXPLORER_IOC_SECURITY begin.\n", __func__);
			if (param_size > EXPLORER_IOC_LEN_MAX) {
				pr_err("%s, ioctl param size exceeds max length.\n", __func__);
				return -EINVAL;
			}

			sync_security_info = (struct explorer_security_info*)
									devm_kzalloc(dev, sizeof(struct explorer_security_info),
									GFP_KERNEL);
			if (!sync_security_info) {
				pr_err("%s, alloc memory failed.\n", __func__);
				return -ENOMEM;
			}

			ret = copy_from_user((void *)sync_security_info, (char __user *)arg,
									sizeof(struct explorer_security_info));
			if (ret) {
				pr_err("%s, can not copy explorer_data from user, ret = 0x%x.\n",
						__func__, ret);
				goto security_out;
			}

			ret = explorer_write_ipc_data_wait(epd, HAL_CMD_SEC,
							   sync_security_info->cmd_buffer,
							   sync_security_info->cmd_len,
							   &sync_security_info->sync_reply);
			if (ret < 0) {
				pr_err("%s, explorer_write_ipc_data_wait failed.\n", __func__);
				goto security_out;
			}

			pr_info("%s, write IOC_NR_SECURITY done.\n", __func__);
security_out:
			devm_kfree(dev, sync_security_info);
			break;
		}
		case IOC_NR_SDCLK:
		{
			void *data = NULL;
			int sdclk_value;
			if (param_size > EXPLORER_IOC_LEN_MAX) {
				pr_err("%s, ioctl param size exceeds max length.\n", __func__);
				return -EINVAL;
			}
			data = devm_kzalloc(dev, param_size, GFP_KERNEL);
			if (!data) {
				pr_err("%s, alloc memory failed.\n", __func__);
				return -ENOMEM;
			}
			ret = copy_from_user(data, (char __user *)arg, param_size);
			if (ret) {
				pr_err("%s, can not copy explorer_data from user, ret = 0x%x.\n", __func__, ret);
				goto sdclk_out;
			}

			//get current sdclk
			sdclk_value = sdio_get_sdclk(epd->sdio_data);
			if (sdclk_value < 0) {
				pr_err("%s, sdio get sdclk failed.\n", __func__);
				goto sdclk_out;
			}

			if (*(unsigned int *)data <= sdclk_value) {
				//change sdclk directly
				sdio_set_sdclk(epd->sdio_data, *(unsigned int *)data);
			} else {
				struct hal_comm_data comm_data;

				//send sdclk to CC
				comm_data.cmd_is_sync = HAL_CMD_ASYNC;
				comm_data.cmd_is_reply = HAL_CMD_SEND;
				comm_data.cmd_is_end = HAL_CMD_START;
				comm_data.cmd_core_id = 0;
				comm_data.cmd_mod_id = HAL_CMD_SDCLK;
				comm_data.data_len = HAL_CMD_DATA_LEN_4;
				comm_data.data[0] = *(unsigned int *)data / 1000000;

				ret = explorer_write_mbox(epd, (union mbox_data *)&comm_data, 1);
				if (ret < 0) {
					pr_err("%s, explorer_hal_sync_write failed.\n", __func__);
				}
			}
			pr_info("%s, write EXPLORER_IOC_SDCLK done.\n", __func__);
sdclk_out:
			devm_kfree(dev, data);
			break;
		}
#ifdef SDIO_PRESSURE_TEST
		case IOC_BATCH_WRITE_TEST:
		{
			ret = copy_from_user(&param_size, (char __user *)(arg+8), 4);
			if (ret) {
				pr_err("%s, can not copy explorer_data from user.\n", __func__);
				return ret;
			}
			param_size += EXPLORER_IOC_TEST_MSG_HDR;
			pr_info("%s, param size = %d.\n", __func__, param_size);
			if (param_size > EXPLORER_IOC_TEST_MSG_LEN) {
				pr_err("%s, max data length is:%d.\n", __func__, EXPLORER_IOC_TEST_MSG_LEN - EXPLORER_IOC_TEST_MSG_HDR);
				return -EINVAL;
			}
			ret = copy_from_user(sdio_press_data, (char __user *)arg, param_size);
			if (ret) {
				pr_err("%s, can not copy explorer_data from user.\n", __func__);
				return ret;
			}
			addr = *(unsigned int *)sdio_press_data;
			blocksize = *(unsigned int *)(sdio_press_data + 4);
			bufsize = *(unsigned int *)(sdio_press_data + 8);
			pr_info("%s, addr = %d, bufsize = %d\n", __func__, addr, bufsize);

			ret = explorer_hal_sync_batch_write(epd, (struct batch_data_item *)((char *)sdio_press_data + EXPLORER_IOC_TEST_MSG_HDR), bufsize);
			pr_info("zeku:bufsize=%d\n", bufsize);
			if (ret < 0) {
				pr_info("%s, read EXPLORER_IOC_BW_TEST fail.\n", __func__);
				goto out;
			}
			pr_info("%s, read EXPLORER_IOC_BW_TEST done.\n", __func__);
			break;
		}
#endif
		case IOC_NR_THERMAL_GET:
		{
			pr_err("thermal ioctl not used");
			break;
		}
		case IOC_NR_SYNC_TIME:
		{
			struct timespec64 now;
			ktime_t kts = 0;
			struct rtc_time tm;

			ktime_get_real_ts64(&now);
			rtc_time_to_tm(now.tv_sec, &tm);
			pr_info("%s, sync rtc time: %d-%02d-%02d %02d:%02d:%02d.%09lu UTC\n",
				__func__, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
				tm.tm_hour, tm.tm_min, tm.tm_sec, now.tv_nsec);
			ret = explorer_send_mbox_nowait(epd, HAL_CMD_GEN, IPC_GEN_MSGID_TM1, (void *)&now.tv_sec);
			ret = explorer_send_mbox_nowait(epd, HAL_CMD_GEN, IPC_GEN_MSGID_TM2, (void *)&now.tv_nsec);

			pr_info("%s, sync timestamp.\n", __func__);
			kts = ktime_get_boottime();
			ret = explorer_send_mbox_nowait(epd, HAL_CMD_GEN, IPC_GEN_MSGID_TSS, (void *)&kts);
			if (ret < 0)
				pr_err("%s, explorer_send_mbox_nowait failed.\n", __func__);
			break;
		}
		case IOC_NR_CTL_ULOG:
		{
			int ret;
			void *data = NULL;
			enum explor_ulog_ctl_info info;
			unsigned int dummy_data[2] = {0, 0};
			if (param_size > EXPLORER_IOC_LEN_MAX) {
				pr_err("%s, ioctl param size exceeds max length.\n", __func__);
				return -EINVAL;
			}
			data = devm_kzalloc(dev, param_size, GFP_KERNEL);
			if (!data) {
				pr_err("%s, alloc memory failed.\n", __func__);
				return -ENOMEM;
			}
			ret = copy_from_user(data, (char __user *)arg, param_size);
			if (ret) {
				pr_err("%s, can not copy explorer_data from user, ret = 0x%x.\n", __func__, ret);
				goto ctl_ulog_out;
			}
			info = *(enum explor_ulog_ctl_info *)data;

			pr_info("%s, ulog ctl, sub cmd: %d\n", __func__, info);
			ret = explorer_send_mbox_nowait(epd, HAL_CMD_ULOG_CTL, info, &dummy_data);
			if (ret < 0) {
				pr_err("%s, ioctl ulog ctl send mbox fail.\n", __func__);
			}
ctl_ulog_out:
			devm_kfree(dev, data);
			break;
		}
		case IOC_NR_READ_REPLY:
		{
			void *data = NULL;
			struct explorer_rdmem_info *rdmem_info = NULL;
			u32 mbox_data[2] = {0};
			if (param_size > sizeof(struct explorer_rdmem_info)) {
				pr_err("%s, ioctl param size (%d) exceeds the limit (%d).\n", __func__,
					param_size, sizeof(struct explorer_tunning_data_info));
				return -EINVAL;
			}
			data = devm_kzalloc(dev, param_size, GFP_KERNEL);
			if (!data) {
				pr_err("%s, alloc memory failed.\n", __func__);
				return -ENOMEM;
			}
			ret = copy_from_user(data, (char __user *)arg, param_size);
			if (ret) {
				pr_err("%s, can not copy explorer_data from user, ret = 0x%x.\n", __func__, ret);
				goto rdmem_out;
			}
			rdmem_info = (struct explorer_rdmem_info *)data;
			ret = explorer_reply_mbox(epd, rdmem_info->id, 0, (void *)mbox_data);
			if (ret < 0) {
				pr_err("%s, explorer_reply_mbox failed.\n", __func__);
				goto rdmem_out;
			}
			pr_debug("%s, IOC_NR_READ_REPLY done.\n", __func__);
rdmem_out:
			devm_kfree(dev, data);
			break;
		}
		case IOC_NR_SOFT_RESET:
		{
			int ret;
			unsigned int dummy_data[2] = {0, 0};
			if (param_size > EXPLORER_IOC_LEN_MAX) {
				pr_err("%s, ioctl param size exceeds max length.\n", __func__);
				return -EINVAL;
			}
			pr_info("%s, do SOFT_RESET.\n", __func__);
			/* Disable heartbeat detect when warm reset */
			epd->heartbeat_started = false;
			cancel_heartbeat_detect_work(epd);
			/* flush power works */
			complete_all(&epd->power_state_completion);
			complete_all(&epd->wake_completion);
			/* flush workqueue */
			pr_info("%s, flush workqueue begin.\n", __func__);
			flush_workqueue(epd->mbox_msg_wq);
			pr_info("%s, flush workqueue done.\n", __func__);
			ret = explorer_send_mbox_nowait(epd, HAL_CMD_SOFT_RESET, 0, &dummy_data);
			if (ret < 0) {
				pr_err("%s, ioctl ulog ctl send mbox fail.\n", __func__);
			}
			break;
		}
		case IOC_NR_SYS_STATUS:
		{
			void *data = NULL;
			struct explorer_system_status *sys_status = NULL;
			if (param_size > sizeof(struct explorer_system_status)) {
				pr_err("%s, ioctl param size (%d) exceeds the limit (%d).\n", __func__,param_size, sizeof(struct explorer_system_status));
				return -EINVAL;
			}
			data = devm_kzalloc(dev, param_size, GFP_KERNEL);
			if (!data) {
				pr_err("%s, alloc memory failed.\n", __func__);
				return -ENOMEM;
			}
			ret = copy_from_user(data, (char __user *)arg, param_size);
			if (ret) {
				pr_err("%s, can not copy explorer_data from user, ret = 0x%x.\n", __func__, ret);
				goto syssts_out;
			}
			sys_status = (struct explorer_system_status *)data;
			if (sys_status->ops == 0) {
				ret = copy_to_user((char __user *)sys_status->value,
						    &epd->status, sizeof(epd->status));
				if (ret) {
					pr_err("%s, can not copy to user, ret = 0x%x.\n", __func__, ret);
					goto syssts_out;
				}
			} else if (sys_status->ops == 1) {
				epd->status = (uintptr_t)sys_status->value;
				pr_info("%s, status set to: %ld.\n", __func__, epd->status);
			} else {
				pr_err("%s, invalid ops cmd: %d.\n", __func__, sys_status->ops);
				goto syssts_out;
			}
			pr_info("%s, IOC_NR_SYS_STATUS done.\n", __func__);
syssts_out:
			devm_kfree(dev, data);
			break;
		}
#ifdef SLT_ENABLE
		case IOC_NR_SLT:
		{
			void *data = NULL;
			char *slt_command = NULL;
			struct explorer_slt_msg_header *slt_header = NULL;
			size_t slt_command_length = 0;
			if (param_size >= EXPLORER_IOC_LEN_MAX) {
				pr_err("%s, ioctl param size exceeds max length.\n", __func__);
				return -EINVAL;
			}
			data = devm_kzalloc(dev, param_size + 1, GFP_KERNEL);
			if (!data) {
				pr_err("%s, alloc memory failed.\n", __func__);
				return -ENOMEM;
			}
			ret = copy_from_user(data, (char __user *)arg, param_size);
			if (ret) {
				pr_err("%s, can not copy explorer_data from user, ret = 0x%x.\n", __func__, ret);
				goto slt_out;
			}
			slt_command = (char *)data + sizeof(struct explorer_slt_msg_header);
			slt_header = (struct explorer_slt_msg_header *)data;
			epd->ebs.start_jiffies = jiffies;
			slt_command_length = param_size - sizeof(struct explorer_slt_msg_header);
			slt_command[slt_command_length] = '\0';
			ret = explorer_slt_dispatch(epd, slt_command, slt_header->type, slt_command_length + 1);
			if (ret < 0) {
				pr_err("%s, explorer_slt_dispatch failed, ret = %d\n", __func__, ret);
				goto slt_out;
			}
			pr_debug("%s, read IOC_NR_SLT done.\n", __func__);
slt_out:
			devm_kfree(dev, data);
			break;
		}
		case IOC_NR_SLT_WAIT_FW:
		{
			struct explorer_slt_fw slt_fw;
			ret = copy_from_user((void *)&slt_fw, (char __user *)arg, sizeof(slt_fw));
			if (ret) {
				pr_err("%s, can not copy explorer_slt_fw from user, ret = 0x%x.\n", __func__, ret);
				break;
			}
			/* TODO: wait slt firmware */
			ret = wait_firmware_on(epd, &slt_fw);
			pr_info("%s, slt firmware type: %d, ret = %d.\n", __func__, slt_fw.type, ret);
			break;
		}
#endif

#if defined (QCOM_AON) || defined(MTK_AON)
		case IOC_NR_CAM_CONTROL:
		{
			void *data = NULL;

			if (param_size > EXPLORER_IOC_LEN_MAX) {
				pr_err("%s, ioctl param size exceeds max length.\n", __func__);
				pr_info("%s, param_size = %d\n", __func__, param_size);
				return -EINVAL;
			}

			data = kzalloc(param_size, GFP_KERNEL);
			pr_info("%s data mm-kzalloc-size: %lld", __func__, param_size);
			if (!data) {
				pr_err("%s, alloc memory failed.\n", __func__);
				return -ENOMEM;
			}
			ret = copy_from_user(data, (char __user *)arg, param_size);
			if (ret) {
				pr_err("%s, copy_from_user failed, ret = 0x%x.\n", __func__, ret);
				kfree(data);
				pr_info("%s data mm-kfree", __func__);
				break;
			}
			pr_info("%s before epd->aon_data: %p", __func__, epd->aon_data);
			ret = explorer_aon_drv_cmd((void**)(&epd->aon_data), data);
			pr_info("%s after epd->aon_data: %p", __func__, epd->aon_data);
			if (ret < 0)
				pr_err("%s, explorer_aon_drv_cmd failed.\n", __func__);
			kfree(data);
			pr_info("%s data mm-kfree", __func__);
			break;
		}
#endif
		default:
			pr_info("%s, execute test ioctl.\n", __func__);
			ret = explorer_test_ioctl(epd, nr, param_size, arg);
			break;
	}

	pr_debug("%s, done.\n", __func__);
#ifdef SDIO_PRESSURE_TEST
out:
#endif
	return ret;
}

static int explorer_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct explorer_plat_data *epd = filp->private_data;

	unsigned long pfn = (virt_to_phys(epd->shared_buffer) >> PAGE_SHIFT);

	return remap_pfn_range(vma, vma->vm_start, pfn,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot);
}

int explorer_send_uevent(struct explorer_plat_data *epd, char *uevent_buf)
{
	int ret = 0;
	char *const envp[] = { uevent_buf, NULL };
	struct device *dev = epd->explorer_dev;

	pr_debug("%s, sending uevent: %s.\n", __func__, uevent_buf);
	ret = kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, (char **)envp);

	if (ret)
		pr_err("%s, sending uevent failed, ret = %d.\n", __func__, ret);

	return ret;
}

static const struct file_operations explorer_ops = {
	.owner = THIS_MODULE,
	.llseek = explorer_seek,
	.open = explorer_open,
	.release = explore_close,
	.read = explorer_read,
	.write = explorer_write,
	.unlocked_ioctl = explorer_ioctl,
	.compat_ioctl = explorer_ioctl,
	.mmap = explorer_mmap,
};

#define	DEVICE_ATTR_PARAM_BUFFER_LEN		(64)
static ssize_t explorer_ipc_mode_show(struct device *dev,
				struct device_attribute *attr, char *buff)
{
	struct explorer_plat_data *epd = NULL;

	if (!dev) {
		pr_err("dev is NULL\n");
		return -ENODEV;
	}
	epd = dev_get_drvdata(dev);

	if (IPC_SINGLE_XFER == epd->ipc_mode)
		return sprintf(buff, "%s\n", "ipc in single mode");
	else if (IPC_BATCH_XFER == epd->ipc_mode)
		return sprintf(buff, "%s\n", "ipc in batch mode");

	return sprintf(buff, "%s\n", "ipc in invaild mode");
}

static ssize_t explorer_ipc_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t count)
{
	int ret = 0;
	struct explorer_plat_data *epd = NULL;
	char *sptr, *token;
	const char *delim = " ";
	char buf[DEVICE_ATTR_PARAM_BUFFER_LEN+1];
	u32 ipc_mode = 0;

	if (!dev) {
		pr_err("dev is NULL\n");
		return -ENODEV;
	}
	epd = dev_get_drvdata(dev);

	if (count > DEVICE_ATTR_PARAM_BUFFER_LEN) {
		pr_err("%s, parameter buffer count exceeds DEVICE_ATTR_PARAM_BUFFER_LEN.\n", __func__);
		return -ENOMEM;
	}
	memcpy(buf, buff, count);
	sptr = buf;
	buf[count] = '\0';
	token = strsep(&sptr, delim);
	EXPLORER_TOKEN_INVALID_CHECK(token);
	ret = kstrtou32(token, 0, &ipc_mode);
	if (ret) {
		pr_err("%s, get wrong ipc xfer mode.\n", __func__);
		return ret;
	}

	epd->ipc_mode = ipc_mode;

	return count;
}

#define	DEVICE_ATTR_WRITE_BUFFER_LEN		(512*1024)
static ssize_t explorer_write_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	int ret;
	char *sptr, *token;
	const char *delim = " ";
	char buf[DEVICE_ATTR_PARAM_BUFFER_LEN+1];
	u32 addr = 0, value = 0, len = 0, i = 0;
	void *buffer = NULL;
	struct explorer_plat_data *epd = dev_get_drvdata(pdev);

	if (count > DEVICE_ATTR_PARAM_BUFFER_LEN) {
		pr_err("%s, parameter buffer count exceeds DEVICE_ATTR_PARAM_BUFFER_LEN.\n", __func__);
		return -ENOMEM;
	}
	memcpy(buf, buff, count);
	sptr = buf;
	buf[count] = '\0';
	token = strsep(&sptr, delim);
	EXPLORER_TOKEN_INVALID_CHECK(token);
	ret = kstrtou32(token, 0, &addr);
	if (ret) {
		pr_err("%s, get wrong address.\n", __func__);
		return ret;
	}
	token = strsep(&sptr, delim);
	EXPLORER_TOKEN_INVALID_CHECK(token);
	ret = kstrtou32(token, 0, &value);
	if (ret) {
		pr_err("%s, get wrong value.\n", __func__);
		return ret;
	}
	token = strsep(&sptr, delim);
	EXPLORER_TOKEN_INVALID_CHECK(token);
	ret = kstrtou32(token, 0, &len);
	if (ret) {
		pr_err("%s, get wrong length.\n", __func__);
		return ret;
	}

	/* allocate write buffer */
	if (len > DEVICE_ATTR_WRITE_BUFFER_LEN) {
		pr_err("%s, write length too long.\n", __func__);
		return -EINVAL;
	}
	buffer = kzalloc(len, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;
	for (i = 0; i < len/4; i++)
		((u32 *)buffer)[i] = value;

	ret = explorer_hal_sync_write(epd, addr, buffer, len);
	if (ret<0)
		pr_err("%s, explorer_hal_sync_write failed.\n", __func__);

	pr_info("%s, addr = 0x%x, value = 0x%x, len = 0x%x, ret = %d.\n", __func__, addr, value, len, ret);

	kfree(buffer);

	return count;
}

#define	DEVICE_ATTR_READ_BUFFER_LEN		(64*1024)
static ssize_t explorer_read_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	int ret;
	char *sptr, *token;
	const char *delim = " ";
	char buf[DEVICE_ATTR_PARAM_BUFFER_LEN+1];
	u32 addr = 0, len = 0, i = 0, print_len = 0;
	void *buffer = NULL;
	struct explorer_plat_data *epd = dev_get_drvdata(pdev);

	if (count > DEVICE_ATTR_PARAM_BUFFER_LEN) {
		pr_err("%s, parameter buffer count exceeds DEVICE_ATTR_PARAM_BUFFER_LEN.\n", __func__);
		return -ENOMEM;
	}
	memcpy(buf, buff, count);
	sptr = buf;
	buf[count] = '\0';
	token = strsep(&sptr, delim);
	ret = kstrtou32(token, 0, &addr);
	if (ret) {
		pr_err("%s, get wrong address.\n", __func__);
		return ret;
	}
	token = strsep(&sptr, delim);
	ret = kstrtou32(token, 0, &len);
	if (ret) {
		pr_err("%s, get wrong length.\n", __func__);
		return ret;
	}

	/* allocate read buffer */
	if (len >= DEVICE_ATTR_READ_BUFFER_LEN) {
		pr_err("%s, read length too long.\n", __func__);
		return -EINVAL;
	}
	buffer = kzalloc(len, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	ret = explorer_hal_sync_read(epd, addr, buffer, len);
	if (ret<0)
		pr_err("%s, explorer_hal_sync_write failed.\n", __func__);

	pr_info("%s, addr = 0x%x, len = 0x%x, ret = %d.\n", __func__, addr, len, ret);
	print_len = len>16 ? 16 : len;
	for (i = 0; i < print_len; i++) {
		pr_info("%s, read buffer[%d] = 0x%x.\n", __func__, i, ((u8 *)buffer)[i]);
	}

	kfree(buffer);

	return count;
}

static ssize_t explorer_send_genl_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	int ret;
	char *sptr;
	char *token;
	const char *delim = " ";
	char buf[DEVICE_ATTR_PARAM_BUFFER_LEN+1];
	u32 value = 0;
	u32 len = 0;
	u32 i = 0;
	void *buffer = NULL;
	struct explorer_plat_data *epd = dev_get_drvdata(pdev);

	if (count > DEVICE_ATTR_PARAM_BUFFER_LEN) {
		pr_err("%s, parameter buffer count exceeds DEVICE_ATTR_PARAM_BUFFER_LEN.\n", __func__);
		return -ENOMEM;
	}
	memcpy(buf, buff, count);
	sptr = buf;
	buf[count] = '\0';
	token = strsep(&sptr, delim);
	EXPLORER_TOKEN_INVALID_CHECK(token);
	ret = kstrtou32(token, 0, &value);
	if (ret) {
		pr_err("%s, get wrong address.\n", __func__);
		return ret;
	}

	token = strsep(&sptr, delim);
	EXPLORER_TOKEN_INVALID_CHECK(token);
	ret = kstrtou32(token, 0, &len);
	if (ret) {
		pr_err("%s, get wrong length.\n", __func__);
		return ret;
	}
	/* allocate write buffer */
	if (len > GENL_PAYLOAD_TOTAL_MAX_LENGTH) {
		pr_err("%s, write length too long.\n", __func__);
		return -EINVAL;
	}
	buffer = kzalloc(len, GFP_KERNEL);
	if (!buffer) {
		pr_err("%s, kzalloc failed.\n", __func__);
		return -ENOMEM;
	}
	for (i = 0; i < len/4; i++)
		((u32 *)buffer)[i] = value;

	ret = explorer_genl_mcast_data(epd, HAL_CMD_ISP, buffer, len);
	if (ret<0)
		pr_err("%s, explorer_genl_mcast_data failed.\n", __func__);

	kfree(buffer);
	return count;
}

static ssize_t explorer_set_gpio_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	int ret;
	char *sptr, *token;
	const char *delim = " ";
	char buf[DEVICE_ATTR_PARAM_BUFFER_LEN+1];
	u32 value = 0;
	struct explorer_plat_data *plat_priv = dev_get_drvdata(pdev);

	if (count > DEVICE_ATTR_PARAM_BUFFER_LEN) {
		pr_err("%s, parameter buffer count exceeds DEVICE_ATTR_PARAM_BUFFER_LEN.\n", __func__);
		return -ENOMEM;
	}
	memcpy(buf, buff, count);
	sptr = buf;
	buf[count] = '\0';
	token = strsep(&sptr, delim);
	EXPLORER_TOKEN_INVALID_CHECK(token);
	ret = kstrtou32(token, 0, &value);
	if (ret) {
		pr_err("%s, get wrong address.\n", __func__);
		return ret;
	}

	if (!value)
		ret = gpio_direction_output(plat_priv->bsp_debug_gpio, 0);
	else
		ret = gpio_direction_output(plat_priv->bsp_debug_gpio, 1);
	if (ret) {
		pr_err("%s, gpio_direction_output for bsp_debug_gpio failed.\n", __func__);
		goto out;
	}

	pr_info("%s, set debug gpio value to 0x%x.\n", __func__, value);

out:
	return count;
}

static ssize_t explorer_send_mbox_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	int ret;
	char *sptr, *token;
	const char *delim = " ";
	char buf[DEVICE_ATTR_PARAM_BUFFER_LEN+1];
	u32 is_sync, is_reply, is_end, cmd_id, data_len, data0, data1;
	u32 i = 0;
	union mbox_data mbox_item = {0};
	struct explorer_plat_data *epd = dev_get_drvdata(pdev);

	/* parse user input argument*/
	if (count > DEVICE_ATTR_PARAM_BUFFER_LEN) {
		pr_err("%s, parameter buffer count exceeds DEVICE_ATTR_PARAM_BUFFER_LEN.\n", __func__);
		return -ENOMEM;
	}
	memcpy(buf, buff, count);
	sptr = buf;
	buf[count] = '\0';

	token = strsep(&sptr, delim);
	EXPLORER_TOKEN_INVALID_CHECK(token);
	ret = kstrtou32(token, 0, &is_sync);
	pr_info("%s, is_sync = 0x%x.\n", __func__, is_sync);
	if (ret) {
		pr_err("%s, get wrong test value.\n", __func__);
		return ret;
	}

	token = strsep(&sptr, delim);
	EXPLORER_TOKEN_INVALID_CHECK(token);
	ret = kstrtou32(token, 0, &is_reply);
	pr_info("%s, is_reply = 0x%x.\n", __func__, is_reply);
	if (ret) {
		pr_err("%s, get wrong test value.\n", __func__);
		return ret;
	}

	token = strsep(&sptr, delim);
	EXPLORER_TOKEN_INVALID_CHECK(token);
	ret = kstrtou32(token, 0, &is_end);
	pr_info("%s, is_end = 0x%x.\n", __func__, is_end);
	if (ret) {
		pr_err("%s, get wrong test value.\n", __func__);
		return ret;
	}

	token = strsep(&sptr, delim);
	EXPLORER_TOKEN_INVALID_CHECK(token);
	ret = kstrtou32(token, 0, &cmd_id);
	pr_info("%s, cmd_id = 0x%x.\n", __func__, cmd_id);
	if (ret) {
		pr_err("%s, get wrong test value.\n", __func__);
		return ret;
	}

	token = strsep(&sptr, delim);
	EXPLORER_TOKEN_INVALID_CHECK(token);
	ret = kstrtou32(token, 0, &data_len);
	pr_info("%s, data_len = 0x%x.\n", __func__, data_len);
	if (ret) {
		pr_err("%s, get wrong test value.\n", __func__);
		return ret;
	}

	token = strsep(&sptr, delim);
	EXPLORER_TOKEN_INVALID_CHECK(token);
	ret = kstrtou32(token, 0, &data0);
	pr_info("%s, data = 0x%x.\n", __func__, data0);
	if (ret) {
		pr_err("%s, get wrong test value.\n", __func__);
		return ret;
	}

	token = strsep(&sptr, delim);
	EXPLORER_TOKEN_INVALID_CHECK(token);
	ret = kstrtou32(token, 0, &data1);
	pr_info("%s, data = 0x%x.\n", __func__, data1);
	if (ret) {
		pr_err("%s, get wrong test value.\n", __func__);
		return ret;
	}

	mbox_item.comm_data.cmd_is_sync = is_sync;
	mbox_item.comm_data.cmd_is_reply = is_reply;
	mbox_item.comm_data.cmd_is_end = is_end;
	mbox_item.comm_data.cmd_core_id = 0;
	mbox_item.comm_data.cmd_mod_id = cmd_id;
	mbox_item.comm_data.data_len = data_len;
	mbox_item.comm_data.data[0] = data0;
	mbox_item.comm_data.data[1] = data1;

	ret = explorer_write_mbox(epd, &mbox_item, 1);
	if (ret<0) {
		pr_err("%s, write mbox failed.\n", __func__);
		goto out;
	}

	for (i = 0; i < COMM_BUF_NUM; i++)
		pr_info("%s, mbox fifo[i] is 0x%x.\n", __func__, mbox_item.fifo[i]);
out:
	return count;
}

#ifdef OPLUS_EXPLORER_PLATFORM_QCOM
static inline void do_gettimeofday(struct timeval *tv)
#else
static inline void do_gettimeofday(struct timeval_sdio *tv)
#endif
{
	struct timespec64 now;

	ktime_get_real_ts64(&now);
	tv->tv_sec = now.tv_sec;
	tv->tv_usec = now.tv_nsec/1000;
}

static ssize_t explorer_sdio_press_test_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	int ret;
	int param_num = 4, i = 0;
	char *sptr, *token;
	const char *delim = " ";
	char *data_buf = NULL;
	char buf[DEVICE_ATTR_PARAM_BUFFER_LEN+1];
	u32 param[5];
	u32 addr = 0, size = 0, buf_size = 0, send_times = 0;
	u32 addr_tmp = 0, size_tmp = 0, buf_size_tmp = 0;
	unsigned long long send_size_total = 0, recv_size_total = 0;
	u32 send_size_tmp = 0, speed = 0, us_diff = 0;
#ifdef OPLUS_EXPLORER_PLATFORM_QCOM
	struct timeval start, now;
#else
	struct timeval_sdio start, now;
#endif
	int pr_flags = 0;
	struct explorer_plat_data *plat_priv = dev_get_drvdata(pdev);

	if (count > DEVICE_ATTR_PARAM_BUFFER_LEN) {
		pr_err("zeku:%s, parameter buffer count exceeds DEVICE_ATTR_PARAM_BUFFER_LEN.\n", __func__);
		return -ENOMEM;
	}

	memcpy(buf, buff, count);
	sptr = buf;
	buf[count] = '\0';
	pr_info("zeku:%s, buf=%s\n", __func__, buf);
	for (i = 0; i < param_num; i++) {
		token = strsep(&sptr, delim);
		ret = kstrtou32(token, 0, &param[i]);

		if (ret) {
			pr_err("%s, get param.\n", __func__);
			return ret;
		}
	}

	addr = param[0];
	size = param[1];
	buf_size = param[2];
	send_times = param[3];
	if ((addr < 0x100000 || (0x180000 < addr && addr < 0x20000000) || addr > 0x30000000) ||
		(size > 0xfffffff) || (buf_size > 0x10000000)) {
		ret = -EINVAL;
		goto out;
	}

	data_buf =  kzalloc(buf_size, GFP_KERNEL);
	if (!data_buf)
		return -ENOMEM;
	for (i = 0; i < buf_size; i++)
		data_buf[i] = i;

	while (send_times--) {
		addr_tmp = addr;
		size_tmp = size;
		send_size_tmp = 0;
		do_gettimeofday(&start);
		while (size_tmp) {
			pr_flags = 1;
			buf_size_tmp =  size_tmp > buf_size ? buf_size : size_tmp;
			ret = sdio_write_data(plat_priv->sdio_data, addr_tmp, data_buf, buf_size_tmp);
			if (ret) {
				pr_err("%s, sdio_write_data error.\n", __func__);
				kfree(data_buf);
				return ret;
			}
			addr_tmp += buf_size_tmp;
			size_tmp -= buf_size_tmp;
			send_size_total += buf_size_tmp;
			send_size_tmp += buf_size_tmp;
			if (send_size_tmp > 0x2500000) {
				do_gettimeofday(&now);
				us_diff = (now.tv_sec - start.tv_sec) * 1000000 +
				(now.tv_usec - start.tv_usec);
				if (us_diff)
					speed = send_size_tmp / us_diff * 95 / 100;
				pr_info("zeku:tx_num = %lld MB,speed = %d MB/s\n", send_size_total/1024/1024, speed);
				send_size_tmp = 0;
				speed = 0;
				pr_flags = 0;
			}
		}
		if (pr_flags) {
			do_gettimeofday(&now);
			us_diff = (now.tv_sec - start.tv_sec) * 1000000 +
				(now.tv_usec - start.tv_usec);
			if (us_diff)
				speed = send_size_tmp / us_diff * 95 / 100;
			pr_info("zeku:tx_num = %lld MB,speed = %d MB/s\n", send_size_total/1024/1024, speed);
			pr_flags = 0;
		}

		addr_tmp = addr;
		size_tmp = size;
		send_size_tmp = 0;
		do_gettimeofday(&start);
		while (size_tmp) {
			pr_flags = 1;
			buf_size_tmp =  size_tmp > buf_size ? buf_size : size_tmp;
			ret = sdio_read_data(plat_priv->sdio_data, addr_tmp, data_buf, buf_size_tmp);
			if (ret) {
				pr_err("%s, sdio_read_data error.\n", __func__);
				kfree(data_buf);
				return ret;
			}
			addr_tmp += buf_size_tmp;
			size_tmp -= buf_size_tmp;
			recv_size_total += buf_size_tmp;
			send_size_tmp += buf_size_tmp;
			if (send_size_tmp > 0x2500000) {
				do_gettimeofday(&now);
				us_diff = (now.tv_sec - start.tv_sec) * 1000000 +
				(now.tv_usec - start.tv_usec);
				if (us_diff)
					speed = send_size_tmp / us_diff * 95 / 100;
				pr_info("zeku:rx_num = %lld MB,speed = %d MB/s\n", recv_size_total/1024/1024, speed);
				do_gettimeofday(&start);
				send_size_tmp = 0;
				speed = 0;
				pr_flags = 0;
			}
		}
		if (pr_flags) {
			do_gettimeofday(&now);
			us_diff = (now.tv_sec - start.tv_sec) * 1000000 +
				(now.tv_usec - start.tv_usec);
			if (us_diff)
				speed = send_size_tmp / us_diff * 95 / 100;
			pr_info("zeku:rx_num = %lld MB,speed = %d MB/s\n", recv_size_total/1024/1024, speed);
			pr_flags = 0;
		}
	}
	kfree(data_buf);
	return count;

out:
	return ret;
}

static ssize_t explorer_sdio_write_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	int ret;
	char *sptr, *token;
	const char *delim = " ";
	char buf[DEVICE_ATTR_PARAM_BUFFER_LEN+1];
	u32 addr = 0, value = 0;
	struct explorer_plat_data *plat_priv = dev_get_drvdata(pdev);

	pr_info("zeku:c=%d b=%s\n",count,buff);

	if (count > DEVICE_ATTR_PARAM_BUFFER_LEN) {
		pr_err("zeku:%s, parameter buffer count exceeds DEVICE_ATTR_PARAM_BUFFER_LEN.\n", __func__);
		return -ENOMEM;
	}
	memcpy(buf, buff, count);
	sptr = buf;
	buf[count] = '\0';
	pr_info("zeku:%s, buf=%s\n",__func__,buf);

	token = strsep(&sptr, delim);
	EXPLORER_TOKEN_INVALID_CHECK(token);
	ret = kstrtou32(token, 0, &addr);

	if (ret) {
		pr_err("%s, get wrong address.\n", __func__);
		return ret;
	}

	token = strsep(&sptr, delim);
	EXPLORER_TOKEN_INVALID_CHECK(token);
	ret = kstrtou32(token, 0, &value);

	if (ret) {
		pr_err("%s, get wrong value.\n", __func__);
		return ret;
	}

	ret = sdio_write_data(plat_priv->sdio_data, addr, (void *)&value, 4);

	return count;
}

static ssize_t explorer_sdio_batch_write_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	int ret, i;
	char *sptr, *token;
	const char *delim = " ";
	char buf[DEVICE_ATTR_PARAM_BUFFER_LEN+1];
	u32 addr = 0, value = 0;
	struct batch_data_item *sdiobatchtestbuff = NULL;
	struct explorer_plat_data *plat_priv = dev_get_drvdata(pdev);

	sdiobatchtestbuff = kzalloc(64 * sizeof(struct batch_data_item), GFP_KERNEL);
	if (!sdiobatchtestbuff)
		return -ENOMEM;

	for (i = 0; i < 64; i++) {
		sdiobatchtestbuff[i].addr = 0x100000+i*4;
		sdiobatchtestbuff[i].data = 0x12345678+i;
	}
	pr_info("zeku:c=%d b=%s\n", count, buff);
	if (count > DEVICE_ATTR_PARAM_BUFFER_LEN) {
		pr_err("zeku:%s, parameter buffer count exceeds DEVICE_ATTR_PARAM_BUFFER_LEN.\n", __func__);
		ret = -EINVAL;
		goto err;
	}
	memcpy(buf, buff, count);
	sptr = buf;
	buf[count] = '\0';
	pr_info("zeku:buf=%s\n",buf);
	token = strsep(&sptr, delim);
	ret = kstrtou32(token, 0, &addr);
	if (ret) {
		pr_err("%s, get wrong address.\n", __func__);
		goto err;
	}
	token = strsep(&sptr, delim);
	ret = kstrtou32(token, 0, &value);
	if (ret) {
		pr_err("%s, get wrong value.\n", __func__);
		goto err;
	}


	ret = sdio_batch_write_data(plat_priv->sdio_data, sdiobatchtestbuff, 64*sizeof(struct batch_data_item));

	kfree(sdiobatchtestbuff);
	return count;
err:
	kfree(sdiobatchtestbuff);
	return ret;
}

static ssize_t explorer_sdio_read_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	int ret;
	char *sptr, *token;
	const char *delim = " ";
	char buf[DEVICE_ATTR_PARAM_BUFFER_LEN+1];
	u32 addr = 0;
	int i;
	u8 sdiotestbuff[5] =  {0};
	struct explorer_plat_data *plat_priv = dev_get_drvdata(pdev);

	if (count > DEVICE_ATTR_PARAM_BUFFER_LEN) {
		pr_err("zeku:%s, parameter buffer count exceeds DEVICE_ATTR_PARAM_BUFFER_LEN.\n", __func__);
		return -ENOMEM;
	}

	memcpy(buf, buff, count);
	sptr = buf;
	buf[count] = '\0';
	token = strsep(&sptr, delim);
	EXPLORER_TOKEN_INVALID_CHECK(token);
	ret = kstrtou32(token, 0, &addr);

	if (ret) {
		pr_err("%s, get wrong address.\n", __func__);
		return ret;
	}

	ret = sdio_read_data(plat_priv->sdio_data, addr, (void *)sdiotestbuff, 4);
	if (ret) {
		pr_err("zeku: there is something wrong");
		goto sdioerr;
	}
	for(i=0;i<4;i++)
	{
		pr_info("%s, buff[%d]=%x\n", __func__, i,sdiotestbuff[i]);
	}
sdioerr:
	pr_info("%s, addr = 0x%x, ret = %d.\n", __func__, addr, ret);

	return count;

}

static ssize_t explorer_sdio_cmd52_read_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	int ret;
	char *sptr, *token;
	const char *delim = " ";
	char buf[DEVICE_ATTR_PARAM_BUFFER_LEN+1];
	unsigned int addr = 0;
	unsigned char out;
	struct explorer_plat_data *plat_priv = dev_get_drvdata(pdev);
	struct explorer_sdio_data *sdio_data = plat_priv->sdio_data;

	if (count > DEVICE_ATTR_PARAM_BUFFER_LEN) {
		pr_err("zeku:%s, parameter buffer count exceeds DEVICE_ATTR_PARAM_BUFFER_LEN.\n", __func__);
		return -ENOMEM;
	}

	memcpy(buf, buff, count);
	sptr = buf;
	buf[count] = '\0';
	token = strsep(&sptr, delim);
	ret = kstrtou32(token, 0, &addr);

	if (ret) {
		pr_err("%s, get wrong address.\n", __func__);
		return ret;
	}

	ret = sdio_read_register(sdio_data, addr, &out, 0);
	if (ret) {
		pr_err("zeku: there is something wrong");
		goto sdioerr;
	}
	pr_info("%s, regoffset = 0x%x, out = 0x%x",__func__, addr, out);
sdioerr:
	pr_info("%s, regoffset = 0x%x, ret = %d.\n", __func__, addr, ret);

	return count;

}

static ssize_t explorer_sdio_cmd52_f1_read_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	int ret;
	char *sptr, *token;
	const char *delim = " ";
	char buf[DEVICE_ATTR_PARAM_BUFFER_LEN+1];
	unsigned int addr = 0;
	unsigned char out;
	struct explorer_plat_data *plat_priv = dev_get_drvdata(pdev);
	struct explorer_sdio_data *sdio_data = plat_priv->sdio_data;

	if (count > DEVICE_ATTR_PARAM_BUFFER_LEN) {
		pr_err("zeku:%s, parameter buffer count exceeds DEVICE_ATTR_PARAM_BUFFER_LEN.\n", __func__);
		return -ENOMEM;
	}

	memcpy(buf, buff, count);
	sptr = buf;
	buf[count] = '\0';
	token = strsep(&sptr, delim);
	ret = kstrtou32(token, 0, &addr);

	if (ret) {
		pr_err("%s, get wrong address.\n", __func__);
		return ret;
	}

	ret = sdio_read_register(sdio_data, addr, &out, 1);
	if (ret) {
		pr_err("zeku: there is something wrong");
		goto sdioerr;
	}
	pr_info("%s, regoffset = 0x%x, out = 0x%x",__func__, addr, out);
sdioerr:
	pr_info("%s, regoffset = 0x%x, ret = %d.\n", __func__, addr, ret);

	return count;

}

static ssize_t explorer_sdio_cmd52_write_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	int ret;
	char *sptr, *token;
	const char *delim = " ";
	char buf[DEVICE_ATTR_PARAM_BUFFER_LEN+1];
	u32 addr = 0;
	unsigned char value;
	struct explorer_plat_data *plat_priv = dev_get_drvdata(pdev);
	struct explorer_sdio_data *sdio_data = plat_priv->sdio_data;

	if (count > DEVICE_ATTR_PARAM_BUFFER_LEN) {
		pr_err("zeku:%s, parameter buffer count exceeds DEVICE_ATTR_PARAM_BUFFER_LEN.\n", __func__);
		return -ENOMEM;
	}

	memcpy(buf, buff, count);
	sptr = buf;
	buf[count] = '\0';
	token = strsep(&sptr, delim);
	ret = kstrtou32(token, 0, &addr);

	if (ret) {
		pr_err("%s, get wrong address.\n", __func__);
		return ret;
	}

	token = strsep(&sptr, delim);
	ret = kstrtou8(token, 0, &value);
	if (ret) {
		pr_err("%s, get wrong value.\n", __func__);
		return ret;
	}

	ret = sdio_write_register(sdio_data, addr, value, 0);
	if (ret) {
		pr_err("zeku: there is something wrong");
		goto sdioerr;
	}

	pr_info("%s, regoffset = 0x%x, out = 0x%x", __func__, addr, value);
sdioerr:
	pr_info("%s, regoffset = 0x%x, ret = %d.\n", __func__, addr, ret);

	return count;

}

static ssize_t explorer_sdio_cmd52_f1_write_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	int ret;
	char *sptr, *token;
	const char *delim = " ";
	char buf[DEVICE_ATTR_PARAM_BUFFER_LEN+1];
	u32 addr = 0;
	unsigned char value;
	struct explorer_plat_data *plat_priv = dev_get_drvdata(pdev);
	struct explorer_sdio_data *sdio_data = plat_priv->sdio_data;

	if (count > DEVICE_ATTR_PARAM_BUFFER_LEN) {
		pr_err("zeku:%s, parameter buffer count exceeds DEVICE_ATTR_PARAM_BUFFER_LEN.\n", __func__);
		return -ENOMEM;
	}

	memcpy(buf, buff, count);
	sptr = buf;
	buf[count] = '\0';
	token = strsep(&sptr, delim);
	ret = kstrtou32(token, 0, &addr);

	if (ret) {
		pr_err("%s, get wrong address.\n", __func__);
		return ret;
	}

	token = strsep(&sptr, delim);
	ret = kstrtou8(token, 0, &value);
	if (ret) {
		pr_err("%s, get wrong value.\n", __func__);
		return ret;
	}

	ret = sdio_write_register(sdio_data, addr, value, 1);
	if (ret) {
		pr_err("zeku: there is something wrong");
		goto sdioerr;
	}

	pr_info("%s, regoffset = 0x%x, out = 0x%x", __func__, addr, value);
sdioerr:
	pr_info("%s, regoffset = 0x%x, ret = %d.\n", __func__, addr, ret);

	return count;

}

static unsigned int reg_address = 0;
static unsigned int reg_count = 1;
static ssize_t explorer_get_address(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	return sprintf(buf, "0x%x\n", reg_address);
}

static ssize_t explorer_set_address(struct device * dev, struct device_attribute *attr,
			     const char * buf, size_t n)
{
	if (kstrtouint(buf, 16, &reg_address) != 0)
		return -EINVAL;

	return n;
}

static ssize_t explorer_get_count(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	return sprintf(buf, "0x%x\n", reg_count);
}

static ssize_t explorer_set_count(struct device * dev, struct device_attribute *attr,
			     const char * buf, size_t n)
{
	if (kstrtouint(buf, 16, &reg_count) != 0)
		return -EINVAL;

	return n;
}

static ssize_t explorer_get_data(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	int i, ret;
	u8 sdio_buf[8] =  {0};
	struct explorer_plat_data *plat_priv = NULL;

	if (!dev) {
		PM_LOG_E("dev is NULL\n");
		return -EINVAL;
	}

	PM_LOG_I("addr: %d count: %d\n", reg_address, reg_count);
	if ((reg_address == 0) || (reg_count == 0)) {
		return -EINVAL;
	}

	plat_priv = dev_get_drvdata(dev);
	if (!plat_priv->sdio_data) {
		PM_LOG_E("plat_priv->sdio_data is NULL\n");
		return -EINVAL;
	}

	if (reg_address >= 0x1000) {
		ret = sdio_read_data(plat_priv->sdio_data, reg_address, (void *)sdio_buf, 4);
		if (ret) {
			PM_LOG_E("error in sdio_read_data() ret=%d\n", ret);
			return ret;
		}
	} else {
		ret = sdio_read_register(plat_priv->sdio_data, reg_address, &sdio_buf[0], 0);
		if (ret) {
			PM_LOG_E("error in sdio_read_register() ret=%d\n", ret);
			return ret;
		}
	}

	for(i = 0; i < 4; i++) {
		PM_LOG_I("sdio_buf[%d]=%x\n", i, sdio_buf[i]);
	}

	return sprintf(buf, "0x%x%02x%02x%02x\n", sdio_buf[3], sdio_buf[2], sdio_buf[1], sdio_buf[0]);
}

static ssize_t explorer_set_data(struct device * dev, struct device_attribute *attr,
			     const char * buf, size_t n)
{
	unsigned int data = 0;
	int ret;
	struct explorer_plat_data *plat_priv = NULL;

	if (!dev) {
		PM_LOG_E("dev is NULL\n");
		return -EINVAL;
	}

	if (kstrtouint(buf, 16, &data) != 0)
		return -EINVAL;

	PM_LOG_I("addr:0x%x count:%d data:0x%x\n", reg_address, reg_count, data);
	if ((reg_address == 0) || (reg_count == 0)) {
		return -EINVAL;
	}

	plat_priv = dev_get_drvdata(dev);
	if (!plat_priv->sdio_data) {
		PM_LOG_E("plat_priv->sdio_data is NULL\n");
		return -EINVAL;
	}
	if (reg_address >= 0x1000) {
		ret = sdio_write_data(plat_priv->sdio_data, reg_address, (void *)&data, 4);
		if (ret) {
			PM_LOG_E("error in sdio_write_data() ret=%d\n", ret);
			return ret;
		}
	} else {
		ret = sdio_write_register(plat_priv->sdio_data, reg_address, (unsigned char)data, 0);
		if (ret) {
			PM_LOG_E("error in sdio_write_register() ret=%d\n", ret);
			return ret;
		}
	}

	return n;
}

static ssize_t explorer_get_power_debug(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	int status;
	struct explorer_plat_data *plat_priv = NULL;

	if (!dev) {
		PM_LOG_E("dev is NULL\n");
		return -EINVAL;
	}

	plat_priv = dev_get_drvdata(dev);
	if (!plat_priv->sdio_data) {
		PM_LOG_E("plat_priv->sdio_data is NULL\n");
		return -EINVAL;
	}

	status = read_sdio_pwr_status(plat_priv);

	return sprintf(buf, "sdio_pwr_status:0x%x\n", status);
}

static ssize_t explorer_set_power_debug(struct device * dev, struct device_attribute *attr,
			     const char * buf, size_t n)
{
	unsigned int data = 0;
	struct explorer_plat_data *plat_priv = NULL;
	int ret = 0;
	unsigned int temp = 0;

	if (!dev) {
		PM_LOG_E("dev is NULL\n");
		return -EINVAL;
	}

	if (kstrtouint(buf, 10, &data) != 0)
		return -EINVAL;

	PM_LOG_I("data:%d\n", data);
	plat_priv = dev_get_drvdata(dev);

	switch (data) {
		case POWER_DEBUG_AP_COMMAND_ENTER_STANDBY:
			PM_LOG_I("POWER_DEBUG_AP_COMMAND_ENTER_STANDBY\n");
			sleep_explorer(plat_priv, false);
			break;
		case POWER_DEBUG_AP_COMMAND_ENTER_STANDBY_FORCE:
			PM_LOG_I("POWER_DEBUG_AP_COMMAND_ENTER_STANDBY_FORCE\n");
			sleep_explorer(plat_priv, true);
			break;
		case POWER_DEBUG_AP_COMMAND_EXIT_STANDBY:
			PM_LOG_I("POWER_DEBUG_AP_COMMAND_EXIT_STANDBY\n");
			wakeup_explorer(plat_priv);
			break;
		case POWER_DEBUG_CLK_REF_OFF:
			PM_LOG_I("POWER_DEBUG_CLK_REF_OFF\n");
			clock_control_explorer(plat_priv, false);
			break;
		case POWER_DEBUG_CLK_REF_ON:
			PM_LOG_I("POWER_DEBUG_CLK_REF_ON\n");
			clock_control_explorer(plat_priv, true);
			break;
		case POWER_DEBUG_PMIC_PON_HIGH:
			PM_LOG_I("POWER_DEBUG_PMIC_PON_HIGH\n");
			gpio_direction_output(plat_priv->pmic_pon_gpio, 1);
			break;
		case POWER_DEBUG_PMIC_PON_LOW:
			PM_LOG_I("POWER_DEBUG_PMIC_PON_LOW\n");
			gpio_direction_output(plat_priv->pmic_pon_gpio, 0);
			break;
		case POWER_DEBUG_PMIC_RESET_HIGH:
			PM_LOG_I("POWER_DEBUG_PMIC_RESET_HIGH\n");
			gpio_direction_output(plat_priv->pmic_reset_gpio, 1);
			break;
		case POWER_DEBUG_PMIC_RESET_LOW:
			PM_LOG_I("POWER_DEBUG_PMIC_RESET_LOW\n");
			gpio_direction_output(plat_priv->pmic_reset_gpio, 0);
			break;
		case POWER_DEBUG_PMIC_ON:
			PM_LOG_I("POWER_DEBUG_PMIC_ON\n");
			power_control_explorer(plat_priv, true);
			break;
		case POWER_DEBUG_PMIC_OFF:
			PM_LOG_I("POWER_DEBUG_PMIC_OFF\n");
			power_control_explorer(plat_priv, false);
			break;
		case POWER_DEBUG_PMIC_RESET:
			PM_LOG_I("POWER_DEBUG_PMIC_RESET\n");
			power_reset_explorer(plat_priv);
			break;
		case POWER_DEBUG_PMIC_ON_CLK_ON:
			PM_LOG_I("POWER_DEBUG_PMIC_ON_CLK_ON\n");
			power_clock_control_explorer(plat_priv, true);
			break;
		case POWER_DEBUG_PMIC_OFF_CLK_OFF:
			PM_LOG_I("POWER_DEBUG_PMIC_OFF_CLK_OFF\n");
			power_clock_control_explorer(plat_priv, false);
			break;
		case POWER_DEBUG_POWER_STATE_SET_STANDBY:
			PM_LOG_I("POWER_DEBUG_POWER_STATE_SET_STANDBY\n");
			set_power_state_explorer(plat_priv, POWER_IOC_STATE_STANDBY);
			break;
		case POWER_DEBUG_POWER_STATE_SET_BYPASS:
			PM_LOG_I("POWER_DEBUG_POWER_STATE_SET_BYPASS\n");
			set_power_state_explorer(plat_priv, POWER_IOC_STATE_BYPASS);
			break;
		case POWER_DEBUG_POWER_STATE_SET_AON:
			PM_LOG_I("POWER_DEBUG_POWER_STATE_SET_AON\n");
			set_power_state_explorer(plat_priv, POWER_IOC_STATE_AON);
			break;
		case POWER_DEBUG_POWER_STATE_SET_MISSION:
			PM_LOG_I("POWER_DEBUG_POWER_STATE_SET_MISSION\n");
			set_power_state_explorer(plat_priv, POWER_IOC_STATE_MISSION);
			break;
		case POWER_DEBUG_POWER_STATE_GET:
			temp = POWER_IOC_STATE_FIRST;
			PM_LOG_I("POWER_DEBUG_POWER_STATE_GET\n");
			ret = get_power_state_explorer(plat_priv, &temp);
			if(ret < 0) {
				PM_LOG_E("get_power_state fail, ret=%d\n", ret);
			} else {
				PM_LOG_I("get_power_state success, state=%d\n", temp);
			}
			break;
		case POWER_DEBUG_PMIC_OC_ALL_REPORT:
			PM_LOG_I("POWER_DEBUG_PMIC_OC_ALL_REPORT\n");
			plat_priv->action_when_pmic_oc = 0;
			break;
		case POWER_DEBUG_PMIC_OC_ALL_POWER_OFF:
			PM_LOG_I("POWER_DEBUG_PMIC_OC_ALL_POWER_OFF\n");
			plat_priv->action_when_pmic_oc = PMIC_OC_ACTION_BEFORE_BOOT_POWER_OFF
											| PMIC_OC_ACTION_AFTER_BOOT_POWER_OFF;
			break;
		case POWER_DEBUG_PMIC_OC_BEFORE_BOOT_POWER_OFF:
			PM_LOG_I("POWER_DEBUG_PMIC_OC_BEFORE_BOOT_POWER_OFF\n");
			plat_priv->action_when_pmic_oc = PMIC_OC_ACTION_BEFORE_BOOT_POWER_OFF;
			break;
		case POWER_DEBUG_PMIC_OC_AFTER_BOOT_POWER_OFF:
			PM_LOG_I("POWER_DEBUG_PMIC_OC_AFTER_BOOT_POWER_OFF\n");
			plat_priv->action_when_pmic_oc = PMIC_OC_ACTION_AFTER_BOOT_POWER_OFF;
			break;
		case POWER_DEBUG_SET_POWER_OFF_SKIP:
			PM_LOG_I("POWER_DEBUG_SET_POWER_OFF_SKIP\n");
			plat_priv->is_poweroff_skip = true;
			break;
		case POWER_DEBUG_CLR_POWER_OFF_SKIP:
			PM_LOG_I("POWER_DEBUG_CLR_POWER_OFF_SKIP\n");
			plat_priv->is_poweroff_skip = false;
			break;
		case POWER_DEBUG_SET_HEARTBEAT_POWER_OFF_SKIP:
			PM_LOG_I("POWER_DEBUG_SET_HEARTBEAT_POWER_OFF_SKIP\n");
			plat_priv->is_heartbeat_poweroff_skip = true;
			break;
		case POWER_DEBUG_CLR_HEARTBEAT_POWER_OFF_SKIP:
			PM_LOG_I("POWER_DEBUG_CLR_HEARTBEAT_POWER_OFF_SKIP\n");
			plat_priv->is_heartbeat_poweroff_skip = false;
			break;
		default:
			PM_LOG_E("unsupported data:%d\n", data);
	}

	return n;
}

static ssize_t explorer_sdio_bin_load_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	int ret;
	char buf[128], fw[128];
	char *sptr, *token;
	const char *delim = " ";
	u32 addr = 0;
	struct explorer_plat_data *epd = dev_get_drvdata(pdev);

	memset(fw, 0, 128);
	if (count >= 128) {
		pr_err("%s, parameter buffer count exceeds the limitation.\n", __func__);
		return -EINVAL;
	}
	memcpy(buf, buff, count);
	sptr = buf;
	buf[127] = '\0';
	fw[127] = '\0';

	token = strsep(&sptr, delim);
	ret = kstrtou32(token, 0, &addr);
	if (ret) {
		pr_err("%s, get wrong addr.\n", __func__);
		return ret;
	}
	pr_err("explorer: %s, get addr [0x%08X].\n", __func__, addr);

	token = strsep(&sptr, delim);
	if (!token) {
		pr_err("explorer: %s, get fw error.\n", __func__);
		return ret;
	}

	strncpy(fw, token, 128);
	pr_err("explorer: %s, get fw [%s].\n", __func__, fw);

	pr_err("explorer: load fs[%s] addr[0x%08X]\n", fw, addr);
	explorer_load_fw(epd, fw, addr);

	return count;
}

static void explorer_parse_tstg(struct explorer_plat_data *epd, char *tstg)
{
	if (strstr(tstg, "BOOTROM"))
		epd->ebi.target_stage = BOOTROM;
	else if (strstr(tstg, "PBL_NORMAL"))
		epd->ebi.target_stage = PBL_NORMAL;
	else if (strstr(tstg, "MIPI_BP_OK"))
		epd->ebi.target_stage = MIPI_BP_OK;
	else if (strstr(tstg, "DDR_OK"))
		epd->ebi.target_stage = DDR_OK;
	else if (strstr(tstg, "OS_OK"))
		epd->ebi.target_stage = OS_OK;
	else if (strstr(tstg, "NPU_OK"))
		epd->ebi.target_stage = NPU_OK;
	else if (strstr(tstg, "PBL_PROV"))
		epd->ebi.target_stage = PBL_PROV;
	else if (strstr(tstg, "PBL_SLT"))
		epd->ebi.target_stage = PBL_SLT;
	else
		epd->ebi.target_stage = OS_OK;
}

static void explorer_parse_cmod(struct explorer_plat_data *epd, const char *cmod)
{
	if (strstr(cmod, "CAM_NORMAL"))
		epd->cmod = CAM_NORMAL;
	else if (strstr(cmod, "CAM_BZ_BK"))
		epd->cmod = CAM_BZ_BK;
	else if (strstr(cmod, "CAM_BZ_FR"))
		epd->cmod = CAM_BZ_FR;
	else if (strstr(cmod, "CAM_BZ_BK_BS"))
		epd->cmod = CAM_BZ_BK_BS;
	else if (strstr(cmod, "CAM_BZ_FR_BS"))
		epd->cmod = CAM_BZ_FR_BS;
	else if (strstr(cmod, "CAM_LW_BK"))
		epd->cmod = CAM_LW_BK;
	else if (strstr(cmod, "CAM_LW_FR"))
		epd->cmod = CAM_LW_FR;
	else if (strstr(cmod, "CAM_LW_BK_BS"))
		epd->cmod = CAM_LW_BK_BS;
	else if (strstr(cmod, "CAM_LW_FR_BS"))
		epd->cmod = CAM_LW_FR_BS;
	else if (strstr(cmod, "CAM_AON"))
		epd->cmod = CAM_AON;
	else
		epd->cmod = CAM_NORMAL;
}

static ssize_t explorer_sdio_boot_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	char buf[128], tstg[16], dmod[16], mmod[16], rmod[16], rpos[16], pmod[16], mbuf[16];
	char *sptr, *token;
	const char *delim = " ";
	struct explorer_plat_data *epd = dev_get_drvdata(pdev);

	epd->ebi.target_stage = OS_OK;
	epd->ebi.dmod = DDR_QUICK;
	epd->ebi.mmod = MIPI_BYPASS;
	epd->ebi.rmod = RTOS_NORMAL;
	epd->ebi.rpos = RTOS_SRAM;
	epd->ebi.pmod = PLAT_QCOM;
	epd->ebi.boot_timeout = 100000;
	epd->ebi.buf_mb[0] = 1;
	epd->ebi.buf_mb[1] = 2;
	epd->ebi.buf_mb[2] = 3;
	epd->ebi.buf_mb[3] = 4;
	epd->ebi.buf_mb[4] = 5;
	epd->ebi.buf_mb[5] = 6;
	epd->ebi.buf_mb[6] = 7;
	epd->ebi.buf_mb[7] = 8;

	memset(tstg, 0, 16);
	memset(dmod, 0, 16);
	memset(mmod, 0, 16);
	memset(rmod, 0, 16);
	memset(rpos, 0, 16);
	memset(pmod, 0, 16);
	memset(mbuf, 0, 16);

	if (count >= 128) {
		pr_err("%s, parameter buffer count exceeds the limitation.\n", __func__);
		return -EINVAL;
	}
	memcpy(buf, buff, count);
	sptr = buf;
	buf[127] = '\0';

	token = strsep(&sptr, delim);
	if (!token) {
		pr_err("explorer_boot cmd: get target_stage err\n");
		goto do_boot;
	}
	strncpy(tstg, token, 16);
	explorer_parse_tstg(epd, tstg);

	token = strsep(&sptr, delim);
	if (!token) {
		pr_err("explorer_boot cmd: get ddr mode err\n");
		goto do_boot;
	}
	strncpy(dmod, token, 16);
	if (strstr(dmod, "DDR_FIRST"))
		epd->ebi.dmod = DDR_FIRST;
	else
		epd->ebi.dmod = DDR_QUICK;


	token = strsep(&sptr, delim);
	if (!token) {
		pr_err("explorer_boot cmd: get mipi mode err\n");
		goto do_boot;
	}
	strncpy(mmod, token, 16);
	if (strstr(mmod, "MIPI_BYPASS"))
		epd->ebi.mmod = MIPI_BYPASS;
	else
		epd->ebi.mmod = MIPI_NORMAL;


	token = strsep(&sptr, delim);
	if (!token) {
		pr_err("explorer_boot cmd: get rtos mode err\n");
		goto do_boot;
	}
	strncpy(rmod, token, 16);
	if (strstr(rmod, "RTOS_AON"))
		epd->ebi.rmod = RTOS_AON;
	else if (strstr(rmod, "RTOS_PLAT"))
		epd->ebi.rmod = RTOS_PLAT;
	else if (strstr(rmod, "RTOS_RLS"))
		epd->ebi.rmod = RTOS_RLS;
	else if (strstr(rmod, "RTOS_DBG_NOPM"))
		epd->ebi.rmod = RTOS_DBG_NOPM;
	else if (strstr(rmod, "RTOS_RLS_NOPM"))
		epd->ebi.rmod = RTOS_RLS_NOPM;
	else
		epd->ebi.rmod = RTOS_NORMAL;


	token = strsep(&sptr, delim);
	if (!token) {
		pr_err("explorer_boot cmd: get rtos position err\n");
		goto do_boot;
	}
	strncpy(rpos, token, 16);
	if (strstr(rpos, "RTOS_DDR"))
		epd->ebi.rpos = RTOS_DDR;
	else
		epd->ebi.rpos = RTOS_SRAM;


	token = strsep(&sptr, delim);
	if (!token) {
		pr_err("explorer_boot cmd: get platform mode err\n");
		goto do_boot;
	}
	strncpy(pmod, token, 16);
	if (strstr(pmod, "PLAT_MTK"))
		epd->ebi.pmod = PLAT_MTK;
	else
		epd->ebi.pmod = PLAT_QCOM;

	token = strsep(&sptr, delim);
	if (!token) {
		pr_err("explorer_boot cmd: get reserve para err\n");
		goto do_boot;
	}

	token = strsep(&sptr, delim);
	if (!token) {
		pr_err("explorer_boot cmd: get timeout err\n");
		goto do_boot;
	}
	if (kstrtou32(token, 0, &epd->ebi.boot_timeout)) {
		pr_err("explorer_boot cmd: parse timeout err\n");
		epd->ebi.boot_timeout = 100000;
		goto do_boot;
	}


	token = strsep(&sptr, delim);
	if (!token) {
		pr_err("explorer_boot cmd: get mipi parameter err\n");
		goto do_boot;
	}
	strncpy(mbuf, token, 16);
//	explorer_parse_mbuf(epd, mbuf);

do_boot:
	pr_err("explorer_boot cmd: tstg=%d, dmod=%d, mmod=%d, rmod=%d, rpos=%d, pmod=%d, to=%d.\n",
		epd->ebi.target_stage, epd->ebi.dmod, epd->ebi.mmod,
		epd->ebi.rmod, epd->ebi.rpos, epd->ebi.pmod,
		epd->ebi.boot_timeout);

	explorer_boot(epd);

	return count;
}

static ssize_t explorer_continue_boot_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	char buf[128], tstg[16], rmod[16], rpos[16];
	char *sptr, *token;
	const char *delim = " ";
	struct explorer_plat_data *epd = dev_get_drvdata(pdev);

	epd->ebi.target_stage = OS_OK;
	epd->ebi.rmod = RTOS_NORMAL;
	epd->ebi.rpos = RTOS_SRAM;
	epd->ebi.boot_timeout = 100000;

	memset(tstg, 0, 16);
	memset(rmod, 0, 16);
	memset(rpos, 0, 16);

	if (count >= 128) {
		pr_err("%s, parameter buffer count exceeds the limitation.\n", __func__);
		return -EINVAL;
	}
	memcpy(buf, buff, count);
	sptr = buf;
	buf[127] = '\0';

	token = strsep(&sptr, delim);
	if (!token) {
		pr_err("explorer_boot cmd: get target_stage err\n");
		goto do_boot;
	}
	strncpy(tstg, token, 16);
	explorer_parse_tstg(epd, tstg);


	token = strsep(&sptr, delim);
	if (!token) {
		pr_err("explorer_boot cmd: get rtos mode err\n");
		goto do_boot;
	}
	strncpy(rmod, token, 16);
	if (strstr(rmod, "RTOS_AON"))
		epd->ebi.rmod = RTOS_AON;
	else if (strstr(rmod, "RTOS_PLAT"))
		epd->ebi.rmod = RTOS_PLAT;
	else if (strstr(rmod, "RTOS_RLS"))
		epd->ebi.rmod = RTOS_RLS;
	else if (strstr(rmod, "RTOS_DBG_NOPM"))
		epd->ebi.rmod = RTOS_DBG_NOPM;
	else if (strstr(rmod, "RTOS_RLS_NOPM"))
		epd->ebi.rmod = RTOS_RLS_NOPM;
	else
		epd->ebi.rmod = RTOS_NORMAL;

	token = strsep(&sptr, delim);
	if (!token) {
		pr_err("explorer_boot cmd: get rtos position err\n");
		goto do_boot;
	}
	strncpy(rpos, token, 16);
	if (strstr(rpos, "RTOS_DDR"))
		epd->ebi.rpos = RTOS_DDR;
	else
		epd->ebi.rpos = RTOS_SRAM;

	token = strsep(&sptr, delim);
	if (!token) {
		pr_err("explorer_boot cmd: get timeout err\n");
		goto do_boot;
	}
	if (kstrtou32(token, 0, &epd->ebi.boot_timeout)) {
		pr_err("explorer_boot cmd: parse timeout err\n");
		epd->ebi.boot_timeout = 100000;
		goto do_boot;
	}

do_boot:
	pr_err("explorer_continue_boot cmd: tstg=%d, dmod=%d, mmod=%d, rmod=%d, rpos=%d, pmod=%d, to=%d.\n",
		epd->ebi.target_stage, epd->ebi.dmod, epd->ebi.mmod,
		epd->ebi.rmod, epd->ebi.rpos, epd->ebi.pmod,
		epd->ebi.boot_timeout);

	explorer_boot_from_pbl(epd);

	return count;
}

static ssize_t explorer_sdio_load_npu_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
//	char buf[128], cmod[16];
//	char *sptr, *token;
//	const char *delim = " ";
	struct explorer_plat_data *epd = dev_get_drvdata(pdev);

	epd->cmod = CAM_NORMAL;
//	memset(cmod, 0, 16);

//	memcpy(buf, buff, count);
//	sptr = buf;
//	buf[127] = '\0';

//	token = strsep(&sptr, delim);
//	if (!token) {
//		pr_err("explorer_boot cmd: get camera mode err\n");
//		goto do_load;
//	}
//	strncpy(cmod, token, 16);
	explorer_parse_cmod(epd, buff);

//do_load:
	pr_err("explorer_load_npu cmd: cmod=%d.\n", epd->cmod);

	explorer_load_npu(epd);

	return count;
}

static ssize_t explorer_boot_force_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	struct explorer_plat_data *epd = dev_get_drvdata(pdev);

	epd->boot_force = FORCE_NO;

	if (strstr(buff, "FORCE_DBG_NOPM"))
		epd->boot_force = FORCE_DBG_NOPM;
	else if (strstr(buff, "FORCE_RLS_NOPM"))
		epd->boot_force = FORCE_RLS_NOPM;
	else if (strstr(buff, "FORCE_RLS"))
		epd->boot_force = FORCE_RLS;
	else if (strstr(buff, "FORCE_DBG"))
		epd->boot_force = FORCE_DBG;
	else if (strstr(buff, "FORCE_PLAT"))
		epd->boot_force = FORCE_PLAT;
	else if (strstr(buff, "FORCE_NO"))
		epd->boot_force = FORCE_NO;
	else
		epd->boot_force = FORCE_NO;

	pr_err("explorer_boot_force cmd: boot_force=%d.\n", epd->boot_force);

	return count;
}

static ssize_t explorer_boot_force_show(struct device *dev,
				struct device_attribute *attr, char *buff)
{
	struct explorer_plat_data *epd = NULL;

	if (!dev) {
		pr_err("dev is NULL\n");
		return -ENODEV;
	}

	epd = dev_get_drvdata(dev);

	return sprintf(buff, "%d\n", epd->boot_force);
}

static ssize_t explorer_sdio_tuning_status_store(struct device *dev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	struct explorer_plat_data *plat_priv = NULL;
	unsigned int enable = 0;

	if (!dev) {
		pr_err("dev is NULL\n");
		return -ENODEV;
	}
	plat_priv = dev_get_drvdata(dev);
	if (!plat_priv->sdio_data) {
		pr_err("plat_priv->sdio_data is NULL\n");
		return -ENODEV;
	}

	if (kstrtouint(buff, 10, &enable) != 0)
		return -EINVAL;
	if (enable == 1)
		explorer_sdio_tuning(plat_priv);
	else
		return -EINVAL;

	return count;
}


static ssize_t explorer_sdio_tuning_status_show(struct device *dev,
				struct device_attribute *attr, char *buff)
{
	struct explorer_plat_data *plat_priv = NULL;

	if (!dev) {
		pr_err("dev is NULL\n");
		return -ENODEV;
	}
	plat_priv = dev_get_drvdata(dev);

	return sprintf(buff, "%d\n", plat_priv->est.sdio_tuning_status);
}

static ssize_t explorer_sdio_tuning_clock_store(struct device *dev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	struct explorer_plat_data *plat_priv = NULL;
	unsigned int clock = 0;

	if (!dev) {
		pr_err("dev is NULL\n");
		return -ENODEV;
	}
	plat_priv = dev_get_drvdata(dev);

	if (kstrtouint(buff, 10, &clock) != 0)
		return -EINVAL;

	if (clock > 0 && clock < 250000000)
		tuning_clock = clock;

	return count;
}


static ssize_t explorer_sdio_tuning_clock_show(struct device *dev,
				struct device_attribute *attr, char *buff)
{
	struct explorer_plat_data *plat_priv = NULL;

	if (!dev) {
		pr_err("dev is NULL\n");
		return -ENODEV;
	}
	plat_priv = dev_get_drvdata(dev);

	return sprintf(buff, "%d\n", tuning_clock);
}


static ssize_t explorer_get_ramdump_debug(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	int ret = 0;
	u8 val = 0;
	struct explorer_plat_data *plat_priv = NULL;

	if (!dev) {
		pr_err("dev is NULL\n");
		return -EINVAL;
	}

	plat_priv = dev_get_drvdata(dev);
	if (!plat_priv->sdio_data) {
		pr_err("plat_priv->sdio_data is NULL\n");
		return -EINVAL;
	}

	ret = explorer_hal_sync_read_internal(plat_priv, SDIO_FN1R_SOC_CTRL_REG0, &val);
	if (ret) {
		pr_err("zeku: get ramdump_debug failed\n");
		return -EINVAL;
	}
	pr_info("%s, regoffset = 0x%x, out = 0x%x\n",__func__, SDIO_FN1R_SOC_CTRL_REG0, val);
	val &= RAMDUMP_EN_SDU;
	return sprintf(buf, "sdio_ramdump_debug_status:0x%x\n", val);
}

static ssize_t explorer_set_ramdump_debug(struct device * dev, struct device_attribute *attr,
			     const char * buf, size_t n)
{
	unsigned int enable = 0;
	int ret = 0;
	u8 val = 0;
	struct explorer_plat_data *plat_priv = NULL;

	if (!dev) {
		pr_err("dev is NULL\n");
		return -EINVAL;
	}

	if (kstrtouint(buf, 10, &enable) != 0)
		return -EINVAL;

	pr_info("enable:%d\n", enable);
	plat_priv = dev_get_drvdata(dev);
	if (!plat_priv->sdio_data) {
		PM_LOG_E("plat_priv->sdio_data is NULL\n");
		return -EINVAL;
	}

	ret = explorer_hal_sync_read_internal(plat_priv, SDIO_FN1R_SOC_CTRL_REG0, &val);
	if (ret) {
		pr_err("zeku: set ramdump_debug failed\n");
		return -EINVAL;
	}

	if (enable) {
		val |= RAMDUMP_EN_SDU;
		ret = explorer_hal_sync_write_internal(plat_priv, SDIO_FN1R_SOC_CTRL_REG0, val);
		if (ret) {
			pr_err("write addr:0x%x, val: 0x%x failed!\n", SDIO_FN1R_SOC_CTRL_REG0, val);
			return -EINVAL;
		}
	} else {
		val &= ~RAMDUMP_EN_SDU;
		ret = explorer_hal_sync_write_internal(plat_priv, SDIO_FN1R_SOC_CTRL_REG0, val);
		if (ret) {
			pr_err("write addr:0x%x, val: 0x%x failed!\n", SDIO_FN1R_SOC_CTRL_REG0, val);
		}
	}

	return n;
}

static ssize_t explorer_get_ramdump_type(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	int ret = 0;
	u8 val = 0;
	struct explorer_plat_data *plat_priv = NULL;

	if (!dev) {
		pr_err("dev is NULL\n");
		return -EINVAL;
	}

	plat_priv = dev_get_drvdata(dev);
	if (!plat_priv->sdio_data) {
		pr_err("plat_priv->sdio_data is NULL\n");
		return -EINVAL;
	}

	ret = explorer_hal_sync_read_internal(plat_priv, SDIO_FN1R_SOC_CTRL_REG1B3, &val);
	if (ret) {
		pr_err("zeku: get ramdump type failed\n");
		return -EINVAL;
	}
	pr_info("%s, regoffset = 0x%x, out = 0x%x\n",__func__, SDIO_FN1R_SOC_CTRL_REG1B3, val);
	val &= RAMDUMP_TYPE_SDU;
	return sprintf(buf, "sdio_ramdump_debug_status:%s\n", val ? "FULLDUMP" : "MINIDUMP");
}

static ssize_t explorer_set_ramdump_type(struct device * dev, struct device_attribute *attr,
			     const char * buf, size_t n)
{
	unsigned int type = 0;
	int ret = 0;
	u8 val = 0;
	struct explorer_plat_data *plat_priv = NULL;

	if (!dev) {
		pr_err("dev is NULL\n");
		return -EINVAL;
	}

	if (kstrtouint(buf, 10, &type) != 0)
		return -EINVAL;

	pr_info("type:%d\n", type);
	plat_priv = dev_get_drvdata(dev);
	if (!plat_priv->sdio_data) {
		PM_LOG_E("plat_priv->sdio_data is NULL\n");
		return -EINVAL;
	}

	ret = explorer_hal_sync_read_internal(plat_priv, SDIO_FN1R_SOC_CTRL_REG1B3, &val);
	if (ret) {
		pr_err("zeku: set ramdump_debug failed\n");
		return -EINVAL;
	}

	if (type) {
		val |= RAMDUMP_TYPE_SDU;
		ret = explorer_hal_sync_write_internal(plat_priv, SDIO_FN1R_SOC_CTRL_REG1B3, val);
		if (ret) {
			pr_err("write addr:0x%x, val: 0x%x failed!\n", SDIO_FN1R_SOC_CTRL_REG1B3, val);
			return -EINVAL;
		}
	} else {
		val &= ~RAMDUMP_TYPE_SDU;
		ret = explorer_hal_sync_write_internal(plat_priv, SDIO_FN1R_SOC_CTRL_REG1B3, val);
		if (ret) {
			pr_err("write addr:0x%x, val: 0x%x failed!\n", SDIO_FN1R_SOC_CTRL_REG1B3, val);
		}
	}

	return n;
}

static ssize_t explorer_set_ramdump_triger(struct device * dev, struct device_attribute *attr,
			     const char * buf, size_t n)
{
	unsigned int enable = 0;
	int ret = 0;
	u32 gen_cmd = HAL_CMD_RAM_DUMP;
	u32 sub_id = 0;
	u32 data = 0;
	struct explorer_plat_data *plat_priv = NULL;

	if (!dev) {
		pr_err("dev is NULL\n");
		return -EINVAL;
	}

	if (kstrtouint(buf, 10, &enable) != 0)
		return -EINVAL;

	pr_info("enable:%d\n", enable);
	plat_priv = dev_get_drvdata(dev);
	if (!plat_priv->sdio_data) {
		PM_LOG_E("plat_priv->sdio_data is NULL\n");
		return -EINVAL;
	}

	if (enable) {
		ret = explorer_send_mbox_nowait(plat_priv, gen_cmd, sub_id, &data);
		if (ret < 0) {
			pr_err("%s, explorer_send_mbox_nowait failed.\n", __func__);
		}
	}

	return n;
}

static DEVICE_ATTR(address, S_IWUSR|S_IRUGO,
		explorer_get_address, explorer_set_address);
static DEVICE_ATTR(count, S_IWUSR|S_IRUGO,
		explorer_get_count, explorer_set_count);
static DEVICE_ATTR(data, S_IWUSR|S_IRUGO,
		explorer_get_data, explorer_set_data);
static DEVICE_ATTR(explorer_power_debug, S_IWUSR|S_IRUGO,
		explorer_get_power_debug, explorer_set_power_debug);

static DEVICE_ATTR(explorer_ipc_mode, S_IWUSR|S_IRUGO,
		explorer_ipc_mode_show, explorer_ipc_mode_store);
static DEVICE_ATTR(explorer_write, S_IWUSR|S_IRUGO,
		NULL, explorer_write_store);
static DEVICE_ATTR(explorer_read, S_IWUSR|S_IRUGO,
		NULL, explorer_read_store);
static DEVICE_ATTR(explorer_send_genl, S_IWUSR|S_IRUGO,
		NULL, explorer_send_genl_store);
static DEVICE_ATTR(explorer_set_gpio, S_IWUSR|S_IRUGO,
		NULL, explorer_set_gpio_store);
static DEVICE_ATTR(explorer_send_mbox, S_IWUSR|S_IRUGO,
		NULL, explorer_send_mbox_store);
static DEVICE_ATTR(explorer_sdio_press_test, S_IWUSR|S_IRUGO,
		NULL, explorer_sdio_press_test_store);
static DEVICE_ATTR(explorer_sdio_write, S_IWUSR|S_IRUGO,
		NULL, explorer_sdio_write_store);
static DEVICE_ATTR(explorer_sdio_batch_write, S_IWUSR|S_IRUGO,
		NULL, explorer_sdio_batch_write_store);
static DEVICE_ATTR(explorer_sdio_read, S_IWUSR|S_IRUGO,
		NULL, explorer_sdio_read_store);
static DEVICE_ATTR(explorer_sdio_cmd52_read, S_IWUSR|S_IRUGO,
		NULL, explorer_sdio_cmd52_read_store);
static DEVICE_ATTR(explorer_sdio_cmd52_write, S_IWUSR|S_IRUGO,
		NULL, explorer_sdio_cmd52_write_store);
static DEVICE_ATTR(explorer_sdio_cmd52_f1_write, S_IWUSR|S_IRUGO,
		NULL, explorer_sdio_cmd52_f1_write_store);
static DEVICE_ATTR(explorer_sdio_cmd52_f1_read, S_IWUSR|S_IRUGO,
		NULL, explorer_sdio_cmd52_f1_read_store);
static DEVICE_ATTR(explorer_sdio_bin_load, S_IWUSR,
		NULL, explorer_sdio_bin_load_store);
static DEVICE_ATTR(explorer_sdio_boot, S_IWUSR,
		NULL, explorer_sdio_boot_store);
static DEVICE_ATTR(explorer_continue_boot, S_IWUSR,
		NULL, explorer_continue_boot_store);
static DEVICE_ATTR(explorer_sdio_load_npu, S_IWUSR,
		NULL, explorer_sdio_load_npu_store);
static DEVICE_ATTR(explorer_boot_force, S_IWUSR|S_IRUGO,
		explorer_boot_force_show, explorer_boot_force_store);
static DEVICE_ATTR(explorer_sdio_tuning_status, S_IWUSR|S_IRUGO,
		explorer_sdio_tuning_status_show, explorer_sdio_tuning_status_store);
static DEVICE_ATTR(explorer_sdio_tuning_clock, S_IWUSR|S_IRUGO,
		explorer_sdio_tuning_clock_show, explorer_sdio_tuning_clock_store);
static DEVICE_ATTR(explorer_ramdump_debug, S_IWUSR|S_IRUGO,
		explorer_get_ramdump_debug, explorer_set_ramdump_debug);
static DEVICE_ATTR(explorer_ramdump_triger, S_IWUSR,
		NULL, explorer_set_ramdump_triger);
static DEVICE_ATTR(explorer_ramdump_type, S_IWUSR|S_IRUGO,
		explorer_get_ramdump_type, explorer_set_ramdump_type);

static struct attribute *explorer_common_attributes[] = {
	&dev_attr_address.attr,                 /*power*/
	&dev_attr_count.attr,                   /*power*/
	&dev_attr_data.attr,                    /*power*/
	&dev_attr_explorer_power_debug.attr,    /*power*/
	&dev_attr_explorer_ipc_mode.attr,
	&dev_attr_explorer_write.attr,
	&dev_attr_explorer_read.attr,
	&dev_attr_explorer_send_genl.attr,
	&dev_attr_explorer_set_gpio.attr,
	&dev_attr_explorer_send_mbox.attr,
	&dev_attr_explorer_sdio_press_test.attr,
	&dev_attr_explorer_sdio_write.attr,
	&dev_attr_explorer_sdio_batch_write.attr,
	&dev_attr_explorer_sdio_read.attr,
	&dev_attr_explorer_sdio_cmd52_read.attr,
	&dev_attr_explorer_sdio_cmd52_write.attr,
	&dev_attr_explorer_sdio_cmd52_f1_write.attr,
	&dev_attr_explorer_sdio_cmd52_f1_read.attr,
	&dev_attr_explorer_sdio_bin_load.attr,
	&dev_attr_explorer_sdio_boot.attr,
	&dev_attr_explorer_continue_boot.attr,
	&dev_attr_explorer_sdio_load_npu.attr,
	&dev_attr_explorer_boot_force.attr,
	&dev_attr_explorer_sdio_tuning_status.attr,
	&dev_attr_explorer_sdio_tuning_clock.attr,
	&dev_attr_explorer_ramdump_debug.attr,
	&dev_attr_explorer_ramdump_triger.attr,
	&dev_attr_explorer_ramdump_type.attr,
	NULL,
};
static const struct attribute_group explorer_common_attribute_group = {
	.attrs = explorer_common_attributes,
};

static int explorer_parse_gpios(struct explorer_plat_data *plat_priv)
{
	int ret = 0;
	struct device *dev = &(plat_priv->plat_dev->dev);
	struct device_node *np = dev->of_node;

	if (of_property_read_bool(np, "ignore-explorer-pmic")) {
		plat_priv->ignore_pmic = true;
	}

	if (of_property_read_bool(np, "ignore-dsleep")) {
		plat_priv->ignore_dsleep = true;
	}
	pr_info("%s, ignore_pmic=%d ignore_dsleep=%d.\n", __func__,
			plat_priv->ignore_pmic, plat_priv->ignore_dsleep);

	/* parse bsp irq gpio */
	ret = of_get_named_gpio(np, "bsp_irq_gpio", 0);
	if (ret < 0) {
		pr_err("%s, falied to get bsp irq gpio.\n", __func__);
		goto out;
	}
	plat_priv->bsp_irq_gpio = ret;
	ret = devm_gpio_request(dev, plat_priv->bsp_irq_gpio, "bsp_irq_gpio");
	if (ret) {
		pr_err("%s, failed to request bsp irq gpio, ret = %d.\n", __func__, ret);
		goto out;
	}
	ret = gpio_direction_input(plat_priv->bsp_irq_gpio);
	if (ret) {
		pr_err("%s, gpio_direction_output for bsp_irq_gpio failed.\n", __func__);
		goto out;
	}

#ifdef USE_DEBUG_GPIO
	/* parse bsp debug gpio */
	ret = of_get_named_gpio(np, "bsp_debug_gpio", 0);
	if (ret < 0) {
		pr_err("%s, falied to get bsp debug gpio.\n", __func__);
		goto out;
	}
	plat_priv->bsp_debug_gpio = ret;
	ret = devm_gpio_request(dev, plat_priv->bsp_debug_gpio, "bsp_debug_gpio");
	if (ret) {
		pr_err("%s, failed to request bsp debug gpio, ret = %d.\n", __func__, ret);
		goto out;
	}
	ret = gpio_direction_output(plat_priv->bsp_debug_gpio, 0);
	if (ret) {
		pr_err("%s, gpio_direction_output for bsp_debug_gpio failed.\n", __func__);
		goto out;
	}
#endif

#ifndef ZEKU_EXPLORER_PLATFORM_RPI
	if (plat_priv->ignore_pmic != true) {
		/* parse pmic pon_1 */
		ret = of_get_named_gpio(np, "pmic_pon_gpio", 0);
		if (ret < 0) {
			pr_err("%s, falied to get pmic_pon_gpio.\n", __func__);
			goto out;
		}
		plat_priv->pmic_pon_gpio = ret;
		ret = devm_gpio_request(dev, plat_priv->pmic_pon_gpio, "pmic_pon_gpio");
		if (ret) {
			pr_err("%s, failed to request pmic_pon_gpio, ret = %d.\n", __func__, ret);
			goto out;
		}

		/* parse pmic reset */
		ret = of_get_named_gpio(np, "pmic_reset_gpio", 0);
		if (ret < 0) {
			pr_err("%s, falied to get pmic_reset_gpio.\n", __func__);
			goto out;
		}
		plat_priv->pmic_reset_gpio = ret;
		ret = devm_gpio_request(dev, plat_priv->pmic_reset_gpio, "pmic_reset_gpio");
		if (ret) {
			pr_err("%s, failed to request pmic_reset_gpio, ret = %d.\n", __func__, ret);
			goto out;
		}
	}
	/* parse dsleep */
	if (plat_priv->ignore_dsleep != true) {
		/* parse dsleep */
		ret = of_get_named_gpio(np, "explorer_sleep_gpio", 0);
		if (ret < 0) {
			pr_err("%s, falied to get explorer_sleep_gpio.\n", __func__);
			goto out;
		}
		plat_priv->explorer_sleep_gpio = ret;
		ret = devm_gpio_request(dev, plat_priv->explorer_sleep_gpio, "explorer_sleep_gpio");
		if (ret) {
			pr_err("%s, failed to request explorer_sleep_gpio, ret = %d.\n", __func__, ret);
			goto out;
		}
	}
#endif
out:
	return ret;
}

static int explorer_parse_clocks(struct explorer_plat_data *plat_priv)
{
	int ret = 0;

#ifndef ZEKU_EXPLORER_PLATFORM_RPI
#ifdef OPLUS_EXPLORER_PLATFORM_QCOM
	struct device *dev = &(plat_priv->plat_dev->dev);
	plat_priv->clk_ref = devm_clk_get(dev, "clk_ref");
#else
	plat_priv->clk_ref = clk_buf_get_xo_name(9);
#endif
	if (plat_priv->clk_ref == NULL) {
		pr_err("%s, failed to request clk_ref.\n", __func__);
		ret = -1;
	}
#endif

	return ret;
}

static int explorer_parse_regulators(struct explorer_plat_data *plat_priv)
{
	int ret = 0;

#ifndef ZEKU_EXPLORER_PLATFORM_RPI
#ifdef SLT_ENABLE
	struct device *dev = &(plat_priv->plat_dev->dev);

	plat_priv->vcc_sdio = regulator_get(dev, "vcc_sdio");
	if (IS_ERR(plat_priv->vcc_sdio)) {
		pr_err("%s:No vcc_sdio regulator found\n",__func__);
		// temporary set to 0, need fix
		ret = 0;
	}
#endif
#endif

	return ret;
}

static int explorer_parse_cc_drive_strength(struct explorer_plat_data *plat_priv)
{
	int ret = 0;
	struct device *dev = &(plat_priv->plat_dev->dev);
	struct device_node *np = dev->of_node;
	u32 cmd_drive_strength = 0, data_drive_strength = 0, clk_drive_strength = 0;
#ifdef OPLUS_EXPLORER_NO_PROJECT
	u32 spmi_drive_strength = 0;
#endif

	if (of_property_read_u32(np, "explorer,clk-drive-strength", &clk_drive_strength) == 0)
		plat_priv->cc_clk_drive_strength = clk_drive_strength;
	else {
		pr_err("%s, failed to parse explorer clk drive strength", __func__);
		return -EINVAL;
	}

	if (of_property_read_u32(np, "explorer,cmd-drive-strength", &cmd_drive_strength) == 0)
		plat_priv->cc_cmd_drive_strength = cmd_drive_strength;
	else {
		pr_err("%s, failed to parse explorer cmd drive strength", __func__);
		return -EINVAL;
	}
	if (of_property_read_u32(np, "explorer,data-drive-strength", &data_drive_strength) == 0)
		plat_priv->cc_data_drive_strength = data_drive_strength;
	else {
		pr_err("%s, failed to parse explorer data drive strength", __func__);
		return -EINVAL;
	}

#ifdef OPLUS_EXPLORER_NO_PROJECT
	if (of_property_read_u32(np, "explorer,spmi-drive-strength", &spmi_drive_strength) == 0) {
        //add mask to tell cc set spmi_drive_strength directly, ignore project_id
		plat_priv->project_id = spmi_drive_strength | 0xFFFF0;
	}
	else {
		pr_err("%s, failed to parse explorer spmi drive strength", __func__);
		return -EINVAL;
	}
#endif

	return ret;
}

static int explorer_parse_dts(struct explorer_plat_data *plat_priv)
{
	int ret = 0;

	pr_info("%s, begin.\n", __func__);
	ret = explorer_parse_gpios(plat_priv);
	if (ret < 0) {
		pr_err("%s, falied to explorer_parse_gpios.\n", __func__);
		goto out;
	}

	ret = explorer_parse_clocks(plat_priv);
	if (ret < 0) {
		pr_err("%s, falied to explorer_parse_clocks.\n", __func__);
		goto out;
	}

	ret = explorer_parse_regulators(plat_priv);
	if (ret < 0) {
		pr_err("%s, falied to explorer_parse_regulators.\n", __func__);
		goto out;
	}

	ret = explorer_parse_cc_drive_strength(plat_priv);
	if (ret < 0) {
		pr_err("%s, falied to explorer_parse_cc_drive_strength.\n", __func__);
		goto out;
	}

	pr_info("%s, done.\n", __func__);
out:
	return ret;
}

static int explorer_get_temp(struct thermal_zone_device *thermal, int *temp)
{
	struct explorer_plat_data *epd = (struct explorer_plat_data*)thermal->devdata;
	struct explorer_gen_data_header gen_header;
	int tdata[7] = {0}, ret = 0, tmp = 0;

	if (true != epd->hw_probe_done) {
		pr_info("explorer not ready\n");
		/* send 0 as asked */
		*temp = 0;
		return 0;
	}

	/* init communication header */
	gen_header.src_ep.core_id = IPC_GEN_COREID_AP;
	gen_header.src_ep.sys_id = IPC_GEN_SYSID_ANDROID;
	gen_header.src_ep.mod_id = IPC_GEN_MSGID_THERMAL;
	gen_header.dst_ep.core_id = IPC_GEN_COREID_EXPLORER;
	gen_header.dst_ep.sys_id = IPC_GEN_SYSID_RTTHREAD;
	gen_header.dst_ep.mod_id = IPC_GEN_MSGID_THERMAL;
	gen_header.dst_addr = IPC_AUTO_ALLOC_ADDR;
	gen_header.mode = IPC_SET_XFER_MODE(IPC_GEN_CMD,IPC_SYNC_SEND_MODE);
	gen_header.payload = (void*)tdata;
	gen_header.psize = 0;
	gen_header.reply.size = sizeof(tdata);
	gen_header.reply.buffer = (void*)tdata;

	ret = explorer_write_generic_cmd_kernel(epd, (void*)tdata, 0, &gen_header);
	if (ret < 0) {
		pr_err("explorer io error\n");
		/* send 0 as asked */
		*temp = 0;
		return 0;
	}

	/*
	 * thermal data
	 * data  : | Level | UP/DOWN | Fit | TS0 | TS1 | TS2 | Count |
	 * format: |  int  |   1/0   | int | int | int | int |  u32  |
	 */

	pr_info("thermal level[%d] trend[%d] data[%d] raw[%d][%d][%d] count[%u]\n",
				tdata[0], tdata[1], tdata[2],
				tdata[3], tdata[4], tdata[5], tdata[6]);

	tmp = tdata[3];
	if (tdata[4] > tmp) {
		tmp = tdata[4];
	}
	if (tdata[5] > tmp) {
		tmp = tdata[5];
	}

	*temp = tmp * 10; //0.001 centigrade

	return 0;
}

struct thermal_zone_device_ops explorer_thermal_zone_ops = {
	.get_temp = explorer_get_temp,
};

static int explorer_probe(struct platform_device *plat_dev)
{
	int ret = 0;
	struct explorer_plat_data *plat_priv;

	pr_info("%s, begin.\n", __func__);

	/* allocate explorer platform data */
	plat_priv = devm_kzalloc(&plat_dev->dev, sizeof(*plat_priv), GFP_KERNEL);
	if (!plat_priv) {
		pr_err("%s, Failed to alloc plat_priv.\n", __func__);
		ret = -ENOMEM;
		return ret;
	}

	plat_priv->ap_buffer = devm_kzalloc(&plat_dev->dev, RAMDUMP_BUFFER_SIZE, GFP_KERNEL);
	if (!plat_priv->ap_buffer) {
		pr_err("[RAMDUMP] %s, alloc failed.\n", __func__);
		goto free_epd;
	}

	/* allocate shared buffer */
	plat_priv->shared_buffer = (void *)__get_free_pages(GFP_KERNEL, USER_BUFFER_PAGES);
	if (!plat_priv->shared_buffer) {
		pr_err("%s, get free pages failed.\n", __func__);
		ret = -ENOMEM;
		goto free_ab;
	}

	/* allocate trans buffer */
	plat_priv->trans_buf = kmalloc(FW_WRITE_BUF_SIZE, GFP_KERNEL);
	if (!plat_priv->trans_buf) {
		pr_err("%s, kmalloc trans buffer failed.\n", __func__);
		ret = -ENOMEM;
		goto free_spg;
	}

	/* register netlink family */
	ret = explorer_genetlink_init(plat_priv);
	if (ret) {
		pr_err("%s, explorer_genetlink_init failed.\n", __func__);
		goto dinit_genl;
	}

	plat_priv->bus_type = IPC_BUS_SDIO;
	plat_priv->ipc_mode = IPC_SINGLE_XFER;
	plat_priv->firmware_loaded = false;
	plat_priv->hw_probe_done = false;
	plat_priv->tuning_retry_num = 0;
	plat_priv->cspi = NULL;
	plat_priv->sdio_data = NULL;
	plat_priv->plat_dev = plat_dev;
#ifndef OPLUS_EXPLORER_NO_PROJECT
	plat_priv->project_id = explorer_get_project();
#else
	plat_priv->project_id = 0;
#endif
	plat_priv->cc_clk_drive_strength = 0;
	plat_priv->cc_cmd_drive_strength = 0;
	plat_priv->cc_data_drive_strength = 0;

	plat_priv->ignore_pmic = false;
	plat_priv->ignore_dsleep = false;
	atomic_set(&plat_priv->is_explorer_on, 0);
	plat_priv->is_poweroff_skip = false;
	plat_priv->is_heartbeat_poweroff_skip = false;
	plat_priv->is_pmic_pon = false;
	plat_priv->is_clock_on = false;
	plat_priv->is_cc_standby = false;
	plat_priv->is_sdu_clk_switched = false;
	plat_priv->action_when_pmic_oc = 0;
	atomic_set(&plat_priv->is_pmic_oc, 0);
	atomic_set(&plat_priv->is_ddr_failed, 0);
	plat_priv->heartbeat_started = false;
	INIT_DELAYED_WORK(&plat_priv->heartbeat_detect_work, heartbeat_detect_delayed_work);
#ifdef SLT_ENABLE
	plat_priv->is_vcc_sdio_on = false;
#endif
	plat_priv->est.sdio_tuning_status = -TUNING_BEGIN;
	mutex_init(&plat_priv->power_sync_lock);
	init_completion(&plat_priv->sleep_completion);
	init_completion(&plat_priv->wake_completion);
	init_completion(&plat_priv->power_state_completion);
	init_completion(&plat_priv->sdio_remove_completion);
	init_completion(&plat_priv->sdi_ddrlp2_completion);
	init_completion(&plat_priv->get_dumpinfo_completion);
	plat_priv->is_get_dumpinfo = false;
	plat_priv->completed_power_state = POWER_IOC_STATE_FIRST;
	platform_set_drvdata(plat_dev, plat_priv);

	atomic_set(&(plat_priv->ebs.flashable), 0);

	pr_info("%s, explorer platform device name is %s.\n", __func__, plat_dev->name);

	/* create misc dev */
	plat_priv->dev.minor = MISC_DYNAMIC_MINOR;
	plat_priv->dev.name = "explorer_preisp";
	plat_priv->dev.fops = &explorer_ops;
	plat_priv->dev.mode = S_IWUSR|S_IRUGO;
	ret = misc_register(&plat_priv->dev);
	if (ret) {
		pr_err("%s, Failed to register hawkeye misc device, err = %d\n", __func__, ret);
		goto ureg_misc;
	}

	/* create /sys/class/explorer dir */
	plat_priv->ns_class = class_create(THIS_MODULE, "explorer");
	if (IS_ERR(plat_priv->ns_class)) {
		pr_err("%s: failed to create class\n", __func__);
		goto err_class_destroy;
	}

	/* create sys/class/explorer/explorer_dev dir */
	plat_priv->explorer_dev = device_create(plat_priv->ns_class, NULL, 0, plat_priv,
			"explorer_dev");
	if (IS_ERR(plat_priv->explorer_dev)) {
		pr_err("%s: could not create explorer_dev device\n", __func__);
		goto err_device_unregister;
	}

	/* create device attr nodes */
	dev_set_drvdata(plat_priv->explorer_dev, plat_priv);
	ret = sysfs_create_group(&plat_priv->explorer_dev->kobj,
			&explorer_common_attribute_group);
	if (ret) {
		pr_err("%s: failed to create device sysfs group, err:%d.\n", __func__, ret);
		goto err_sysfs_remove_group;
	}

	/* parse dts */
	ret = explorer_parse_dts(plat_priv);
	if (ret) {
		pr_err("%s, failed to parse dts, err:%d.\n", __func__, ret);
		goto err_sysfs_remove_group;
	}

	ret = power_reload_pmic_otp(plat_priv);
	if (ret) {
		pr_err("%s, failed to reload pmic otp, err:%d.\n", __func__, ret);
		goto err_sysfs_remove_group;
	}

	/* init ipc */
	ret = explorer_init_ipc(plat_priv);
	if (ret) {
		pr_err("%s, failed to alloc mbox workqueue, err:%d", __func__, ret);
		goto err_init_ipc;
	}

	/* init cspi */
	ret = explorer_cspi_init();
	if (ret) {
		pr_err("%s, failed to register spi driver, err:%d", __func__, ret);
		goto err_cspi_init;
	}

	/* register sdio driver */
	ret = explorer_sdio_init();
	if (ret) {
		pr_err("%s, failed to register sdio driver, err:%d", __func__, ret);
		goto err_sdio_init;
	}

	training_data_buf = kzalloc(TRAINING_DATA_BUF_SIZE, GFP_KERNEL);
	if (!training_data_buf) {
		pr_err("%s, alloc TRAINING_DATA_BUF_SIZE memory failed.\n", __func__);
		goto err_sdio_init;
	}
	/* thermal zone */
	thermal_zone_device_register("zeku_explorer",
		 0, 0, (void*)plat_priv, &explorer_thermal_zone_ops, NULL, 0, 0);

#ifdef SLT_ENABLE
	/* init slt */
	explorer_sdio_slt_test_case_init();
#endif

	explorer_awake_init(plat_priv);

	pr_info("%s, done.\n", __func__);
	return 0;

err_sdio_init:
	explorer_sdio_exit();
err_cspi_init:
	explorer_cspi_exit();
err_init_ipc:
	explorer_dinit_ipc(plat_priv);
err_sysfs_remove_group:
	sysfs_remove_group(&plat_priv->explorer_dev->kobj,
			&explorer_common_attribute_group);
err_device_unregister:
	device_unregister(plat_priv->explorer_dev);
err_class_destroy:
	class_destroy(plat_priv->ns_class);
ureg_misc:
	misc_deregister(&plat_priv->dev);
dinit_genl:
	explorer_genetlink_exit(plat_priv);
	kfree(plat_priv->trans_buf);
free_spg:
	free_pages((unsigned long)plat_priv->shared_buffer, USER_BUFFER_PAGES);
free_ab:
        devm_kfree(&plat_dev->dev, plat_priv->ap_buffer);
free_epd:
	devm_kfree(&plat_dev->dev, plat_priv);
	return ret;
}

static int explorer_remove(struct platform_device *plat_dev)
{
	struct explorer_plat_data *plat_priv = platform_get_drvdata(plat_dev);

	explorer_cspi_exit();
	explorer_sdio_exit();

	PM_LOG_I("shutdown Explorer power&clock when remove\n");
	power_clock_control_explorer(plat_priv, false);

	misc_deregister(&plat_priv->dev);
	sysfs_remove_group(&plat_priv->explorer_dev->kobj,
			&explorer_common_attribute_group);
	device_unregister(plat_priv->explorer_dev);
	class_destroy(plat_priv->ns_class);

	explorer_dinit_ipc(plat_priv);
	explorer_genetlink_exit(plat_priv);

	if (plat_priv->trans_buf)
		kfree(plat_priv->trans_buf);

	/* free user shared buffer */
	if (plat_priv->shared_buffer)
		free_pages((unsigned long)plat_priv->shared_buffer, USER_BUFFER_PAGES);

	devm_kfree(&plat_dev->dev, plat_priv->ap_buffer);
	pr_info("%s, done.\n", __func__);

	if (training_data_buf) {
		kfree(training_data_buf);
		training_data_buf = NULL;
	}

	explorer_awake_exit(plat_priv);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int explorer_suspend(struct device *dev)
{
	struct explorer_plat_data *epd = dev_get_drvdata(dev);

	PM_LOG_I("suspend enter.");
	power_clock_suspend_explorer(epd);
	PM_LOG_I("suspend exit.");

	return 0;
}

static int explorer_resume(struct device *dev)
{
	PM_LOG_I("resume.");
	return 0;
}
#endif

static const struct of_device_id explorer_of_match_table[] = {
	{.compatible = "zeku,explorer", },
	{ },
};
MODULE_DEVICE_TABLE(of, explorer_of_match_table);

static const struct dev_pm_ops explorer_pm_ops = {
		SET_SYSTEM_SLEEP_PM_OPS(explorer_suspend, explorer_resume)
};

static struct platform_driver explorer_platform_driver = {
	.probe  = explorer_probe,
	.remove = explorer_remove,
	.driver = {
		.name = "explorer",
		.of_match_table = explorer_of_match_table,
		.pm	= &explorer_pm_ops,
#ifdef CONFIG_CNSS_ASYNC
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
#endif
	},
};

/*
 * Module init function
 */
static int __init explorer_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&explorer_platform_driver);

	pr_info("%s, done.\n", __func__);

	/*
	 * A non 0 return means init_module failed; module can't be loaded.
	 */
	return ret;
}

/*
 * Module exit function
 */
static void __exit explorer_exit(void)
{
	platform_driver_unregister(&explorer_platform_driver);

	pr_info("%s, done.\n", __func__);
}

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Zhou Feng <zf@zeku.com>");
MODULE_DESCRIPTION("Explorer Linux Kernel module");

module_init(explorer_init);
module_exit(explorer_exit);
