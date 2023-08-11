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
 * Basecode Created :        2020/10/11 Author: zf@zeku.com
 *
 */

#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <net/netlink.h>
#include <net/genetlink.h>
#include <linux/completion.h>
#include <linux/interrupt.h>

#include "include/main.h"
#include "include/ipc.h"
#include "include/irq.h"
#include "include/power.h"
#include "include/isp.h"
#include "include/rtt_debug.h"
#include "include/ap_boot.h"
#include "include/exception.h"
#ifdef SLT_ENABLE
#include "../slt/include/slt.h"
#endif

#ifndef ZEKU_EXPLORER_PLATFORM_RPI
#ifndef OPLUS_EXPLORER_NO_PROJECT
#include <soc/oplus/system/oplus_project.h>
#endif
#endif

DECLARE_WAIT_QUEUE_HEAD(comm_sync_wq);

/* xfer layer comm cmd specification */
static const char *const comm_cmd_spec[] = {
	[HAL_CMD_MALLOC]			"malloc command",
	[HAL_CMD_FREE]			"free command",
	[HAL_CMD_BOOT_COMPLETED]			"boot completed command",
	[HAL_CMD_SEC]			"all security releated commands",
	[HAL_CMD_SCRATCH_LOG]			"ap scratch log from cc command",
	[HAL_CMD_SEND_LOG_ADDR]			"send log buffer addr to ap",
	[HAL_CMD_ISP]			"isp command",
	[HAL_CMD_MAX]			"max cmd",
};

#ifndef OPLUS_EXPLORER_NO_PROJECT
u32 explorer_get_project(void)
{
#ifndef ZEKU_EXPLORER_PLATFORM_RPI
	return get_project();
#else
	return 0;
#endif
}
#endif

int explorer_send_project_id(struct explorer_plat_data *epd)
{
	int ret = 0;
	u32 mbox_data[2] = {0};

	if (!epd) {
		pr_err("%s, invalid argument.\n", __func__);
		goto out;
	}

	mbox_data[0] = epd->project_id;
	ret = explorer_send_mbox_nowait(epd, HAL_CMD_GEN, IPC_GEN_MSGID_PRJID, (void *)mbox_data);
	if (ret < 0) {
		pr_err("%s, explorer_send_mbox_nowait failed.\n", __func__);
		goto out;
	}

	pr_info("%s, done.\n", __func__);
out:
	return ret;
}

int explorer_write_project_id(struct explorer_plat_data *epd)
{
	int ret = 0;
	u32 cc_addr = 0x48001388; /* from power module's top reg */

	if (!epd) {
		pr_err("%s, invalid argument.\n", __func__);
		goto out;
	}

	ret = explorer_hal_sync_write(epd, cc_addr, (void *)&epd->project_id, 4);
	if (ret < 0) {
		pr_err("%s, write failed.\n", __func__);
		goto out;
	}

	pr_info("%s, done.\n", __func__);
out:
	return ret;
}

static int explorer_proc_isp_msg(struct explorer_plat_data *epd, void *ap_buffer, struct hal_comm_data *comm_data)
{
	int ret = 0;
	u32 i = 0;
	u32 size = comm_data->data_len;

	/* TODO: do some isp data */
	for (i = 0; i < 8; i++) {
		pr_debug("%s, isp data[%d] = 0x%x.\n", __func__, i, ((u32 *)ap_buffer)[i]);
	}

	/* this is a reply cmd */
	if (comm_data->cmd_is_reply == HAL_CMD_REPLY) {
		/* sync cmd, wake up wait queue. */
		if (comm_data->cmd_is_sync == HAL_CMD_SYNC) {
			/* copy reply buffer */
			memcpy(epd->shared_buffer, ap_buffer, size);
			epd->sync_reply = epd->shared_buffer;
			epd->sync_reply_size = size;
			atomic_set(&epd->mbox_rindex[comm_data->cmd_mod_id], comm_data->cmd_sync_idx);
			/* wake up wait queue */
			wake_up(&comm_sync_wq);
		} else {
			/* async cmd, netlink to userspace */
			ret = explorer_genl_mcast_data(epd, comm_data->cmd_mod_id, ap_buffer, size);
			if (ret) {
				pr_err("%s, async reply cmd, netlink to userspace failed.\n", __func__);
				goto out;
			}
		}
	} else {
		u32 mbox_data[2] = {0};
		/* sync send cmd, reply to Explorer. */
		if (comm_data->cmd_is_sync == HAL_CMD_SYNC) {
			ret = explorer_reply_mbox(epd, comm_data->cmd_mod_id, comm_data->cmd_sub_id, (void *)mbox_data);
			if (ret<0) {
				pr_err("%s, explorer_reply_mbox failed.\n", __func__);
				goto out;
			}
		}
		/* send cmd, netlink to userspace */
		ret = explorer_genl_mcast_data(epd, comm_data->cmd_mod_id, ap_buffer, size);
		if (ret) {
			pr_err("%s, async reply cmd, netlink to userspace failed.\n", __func__);
			goto out;
		}
	}

	pr_debug("%s, done.\n", __func__);
out:
	return ret;
}

static int explorer_proc_gen_msg(struct explorer_plat_data *epd, void *ap_buffer, struct hal_comm_data *comm_data)
{
	int ret = 0;
	u32 i = 0;
	u32 size = comm_data->data_len;
	u32 gen_cmd = 0;

	/* check payload position */
	if (comm_data->cmd_pld_pos == IPC_PLD_IN_MBOXFIFO) {
		pr_debug("%s, payload in mbox fifo.\n", __func__);
		ap_buffer = (void *)comm_data->data;
	} else
		pr_debug("%s, payload in ipc mem pool.\n", __func__);

	/* for debug */
	for (i = 0; i < 8; i++)
		pr_debug("%s, generic data[%d] = 0x%x.\n", __func__, i, ((u32 *)ap_buffer)[i]);

	gen_cmd = CON_IPC_ID(comm_data->cmd_mod_id, comm_data->cmd_sub_id);
	/* this is a reply cmd */
	if (comm_data->cmd_is_reply == HAL_CMD_REPLY) {
		/* sync cmd, wake up wait queue. */
		if (comm_data->cmd_is_sync == HAL_CMD_SYNC) {
			/* copy reply buffer */
			memcpy(epd->shared_buffer, ap_buffer, size);
			epd->sync_reply = epd->shared_buffer;
			epd->sync_reply_size = size;
			atomic_set(&epd->mbox_rindex[comm_data->cmd_mod_id], comm_data->cmd_sync_idx);
			/* wake up wait queue */
			wake_up(&comm_sync_wq);
		} else {
			/* async cmd, netlink to userspace */
			ret = explorer_genl_mcast_data(epd, gen_cmd, ap_buffer, size);
			if (ret) {
				pr_err("%s, async reply cmd, netlink to userspace failed.\n", __func__);
				goto out;
			}
		}
	} else {
		u32 mbox_data[2] = {0};
		/* sync send cmd, reply to Explorer. */
		if (comm_data->cmd_is_sync == HAL_CMD_SYNC) {
			ret = explorer_reply_mbox(epd, HAL_CMD_GEN, gen_cmd, (void *)mbox_data);
			if (ret<0) {
				pr_err("%s, explorer_reply_mbox failed.\n", __func__);
				goto out;
			}
		}
		/* send cmd, netlink to userspace */
		ret = explorer_genl_mcast_data(epd, gen_cmd, ap_buffer, size);
		if (ret) {
			pr_err("%s, async reply cmd, netlink to userspace failed.\n", __func__);
			goto out;
		}
	}

	pr_debug("%s, done.\n", __func__);
out:
	return ret;
}

/* xfer layer comm cmd process table */
static int (*ipcmem_cmd_ops[])(struct explorer_plat_data *epd,
			       void *buffer,
			       struct hal_comm_data *comm_data) = {
	[HAL_CMD_MALLOC]			NULL,
	[HAL_CMD_FREE]				NULL,
	[HAL_CMD_GEN]			explorer_proc_gen_msg,
	[HAL_CMD_SEC]			explorer_proc_gen_msg,
	[HAL_CMD_ISP]			explorer_proc_isp_msg,
	[HAL_CMD_CAMERA]			explorer_proc_isp_msg,
#ifdef SLT_ENABLE
	[HAL_CMD_SLT]			explorer_proc_slt_msg,
#endif
	[HAL_CMD_GDB]			explorer_proc_gen_msg,
	[HAL_CMD_MAX]			NULL,
};

static int explorer_ipc_checksum(void *ap_buffer, u32 len, u32 *result)
{
	u32 checksum = 0;
	u32 i = 0;
	u8 *buffer = (u8 *)ap_buffer;

	if (!buffer || !result) {
		pr_err("%s, input or output buffer is invalid.\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < len; i++)
		checksum += buffer[i];

	*result = checksum;

	return 0;
}

static int explorer_proc_ipcmem_msg(struct explorer_plat_data *epd, struct hal_comm_data *comm_data)
{
	int ret = 0;
	int crc_ret = 0;
	u32 cc_addr = 0;
	void *ap_buffer = NULL;
	struct ipc_mempool *mem_pool;
	struct ipc_mempage busy_page;
	u32 checksum = 0;
	static u32 retry_count = 0;

	/* allocate ap buffer */
	if (comm_data->data_len > IPC_MP_DATA_MAX_LENGTH) {
		ret = -EINVAL;
		pr_err("%s, ipc payload length exceeds limit.\n", __func__);
		goto out;
	}
	ap_buffer = kzalloc(comm_data->data_len, GFP_KERNEL);
	if (!ap_buffer) {
		pr_err("%s, kzalloc failed.\n", __func__);
		return -ENOMEM;
	}

	/* get free memory address */
	memcpy((void *)&busy_page, (void *)&comm_data->data, sizeof(struct ipc_mempage));
	mem_pool = &epd->ipc_mem_pool[busy_page.pool_index];
	cc_addr = mem_pool->base_addr + mem_pool->page_size*busy_page.page_index;
	pr_debug("%s, read ipc data at 0x%x, page num is %d.\n", __func__, cc_addr, busy_page.page_num);

	/* TODO: cc address verify */
	retry_count = 0;
retry:
	/* read cc data */
	ret = explorer_hal_sync_read(epd, cc_addr, ap_buffer, comm_data->data_len);
	if (ret < 0) {
		pr_err("%s, explorer_hal_sync_read failed.\n", __func__);
		goto freemem;
	}

	/* do checksum */
	explorer_ipc_checksum(ap_buffer, comm_data->data_len, &checksum);
	if (checksum != comm_data->data[1]) {
		if (retry_count++ < IPC_RETRY_COUNT) {
			pr_err("%s, do checksum failed, retry.\n", __func__);
			goto retry;
		}
		crc_ret = -EFAULT;
		goto crc_err;
	}
	/* TODO: check retry count, call exception process interface. */

	/* process command class of ipc */
	if (ipcmem_cmd_ops[comm_data->cmd_mod_id] != NULL) {
		ipcmem_cmd_ops[comm_data->cmd_mod_id](epd, ap_buffer, comm_data);
	}
crc_err:
	/* clear ipc mem bitmap */
	ret = explorer_ipc_clear_top_bitmap(epd, &busy_page);
	if (ret<0) {
		pr_err("%s, clear ipc mem bitmap failed.\n", __func__);
		goto freemem;
	}
	pr_debug("%s, done.\n", __func__);
freemem:
	kfree(ap_buffer);
out:
	return crc_ret?crc_ret:ret;
}

static int explorer_proc_ramdump_msg(struct explorer_plat_data *epd, struct hal_comm_data *comm_data)
{
	int ret = 0;
	dump_info info;
	info.dump_addr = comm_data->data[0];
	info.dump_len = comm_data->data_len;

	if (0 == info.dump_len) {
		pr_info("%s, receive full dump msg, chang len to 256M\n", __func__);
		info.dump_len = FULLDUMP_SIZE;
	}

	if (SDI_DDRLP2_MSG == info.dump_addr) {
		pr_info("%s, receive ddrlp2 exit msg\n", __func__);
		complete(&epd->sdi_ddrlp2_completion);
		return ret;
	}

	pr_info("[explorer]ramdump %s addr=0x%x, size=0x%x.\n", __func__,
			info.dump_addr, info.dump_len);

	if (epd->is_get_dumpinfo) {
		pr_info("%s,[RAMDUMP] receive dump addr and len msg\n", __func__);
		epd->ramdump_info = info;
		complete(&epd->get_dumpinfo_completion);
		epd->is_get_dumpinfo = false;
		return ret;
	}

	ret = explorer_genl_mcast_data(epd, HAL_CMD_RTT_RAMINFO, &info, sizeof(dump_info));
	if (ret < 0) {
		pr_err("%s, send dump msg failed.\n", __func__);
	} else {
		pr_info("%s, send dump msg done.\n", __func__);
	}

	return ret;
}

static int explorer_proc_hclk_msg(struct explorer_plat_data *epd, struct hal_comm_data *comm_data)
{
	int ret = 0;
	struct hal_comm_data comm_data_ack = {0};

	pr_info("%s data:0x%x\n", __func__, comm_data->data[0]);
	//change sdclk
	if (((comm_data->data[0] >> 16) & 0xFF) == 0) {

	} else {
		ret = sdio_set_sdclk(epd->sdio_data, ((comm_data->data[0] >> 16) & 0xFF) * 1000000);
		if (ret < 0) {
			pr_err("%s, sdio set sdclk failed.\n", __func__);
		}
	}

	//send ack to CC
	comm_data_ack.cmd_is_sync = HAL_CMD_ASYNC;
	comm_data_ack.cmd_is_reply = HAL_CMD_REPLY;
	comm_data_ack.cmd_is_end = HAL_CMD_START;
	comm_data_ack.cmd_mod_id = HAL_CMD_HCLK;
	comm_data_ack.data_len = HAL_CMD_DATA_LEN_4;
	comm_data_ack.data[0] = comm_data->data[0];

	ret = explorer_write_mbox(epd, (union mbox_data *)&comm_data_ack, 1);
	if (ret < 0) {
		pr_err("%s, explorer_hal_sync_write failed.\n", __func__);
	}

	pr_info("%s, done.\n", __func__);

	return ret;
}

static int explorer_proc_sdclk_msg(struct explorer_plat_data *epd, struct hal_comm_data *comm_data)
{
	int ret = 0;

	//change sdclk
	ret = sdio_set_sdclk(epd->sdio_data, comm_data->data[0]);
	if (ret < 0) {
		pr_err("%s, sdio set sdclk failed.\n", __func__);
	}

	return ret;
}

static int explorer_proc_power_msg(struct explorer_plat_data *epd, struct hal_comm_data *comm_data)
{
	int ret = 0;
	PM_LOG_D("data[0]:0x%x data[1]:0x%x\n", comm_data->data[0], comm_data->data[1]);

	switch (comm_data->data[0]) {
		case POWER_CC2AP_CMD_RT_SUSPEND_STATUS:
		{
			char *s_status = NULL;
			if (comm_data->data[1] & POWER_CC2AP_DAT_RT_SUSPEND_STATUS_PD) {
				s_status = "PD";
			} else if (comm_data->data[1] & POWER_CC2AP_DAT_RT_SUSPEND_STATUS_PLL) {
				s_status = "PLL";
			} else if (comm_data->data[1] & POWER_CC2AP_DAT_RT_SUSPEND_STATUS_SRAM) {
				s_status = "SRAM";
			} else if (comm_data->data[1] & POWER_CC2AP_DAT_RT_SUSPEND_STATUS_THREAD) {
				s_status = "THREAD";
			} else if (comm_data->data[1] & POWER_CC2AP_DAT_RT_SUSPEND_STATUS_NORMAL) {
				s_status = "NORMAL";
			} else {
				s_status = "UNKNOWN";
			}
			PM_LOG_I("Explorer suspend_status_report, cmd: 0x%x data: 0x%x(%s)\n",
					comm_data->data[0], comm_data->data[1], s_status);
			break;
		}
		case POWER_CC2AP_CMD_POWER_STATE_SET_RESULT:
		{
			unsigned int state = comm_data->data[1] >> 16;
			unsigned int result = comm_data->data[1] & 0xffff;
			PM_LOG_I("Explorer power state set result, cmd: 0x%x state: 0x%x result: 0x%x\n",
					comm_data->data[0], state, result);
			if (result == POWER_CC2AP_CMD_POWER_STATE_SET_RESULT_OK) {
				epd->completed_power_state = state;
			} else {
				epd->completed_power_state = POWER_IOC_STATE_FIRST;
			}
			complete(&epd->power_state_completion);
			break;
		}
		case POWER_CC2AP_CMD_POWER_STATE_GET_RESULT:
		{
			unsigned int state = comm_data->data[1] >> 16;
			unsigned int result = comm_data->data[1] & 0xffff;
			PM_LOG_I("Explorer power state get result, cmd: 0x%x state: 0x%x result: 0x%x\n",
					comm_data->data[0], state, result);
			if (result == POWER_CC2AP_CMD_POWER_STATE_GET_RESULT_OK) {
				epd->completed_power_state = state;
			} else {
				epd->completed_power_state = POWER_IOC_STATE_FIRST;
			}
			complete(&epd->power_state_completion);
			break;
		}
		case POWER_CC2AP_CMD_RT_RESUME_STATUS:
		{
			unsigned int result = comm_data->data[1];
			PM_LOG_I("Explorer resume status cmd: 0x%x result: 0x%x\n",
					comm_data->data[0], result);
			complete(&epd->wake_completion);
			break;
		}
		case POWER_CC2AP_CMD_HEARTBEAT:
		{
			unsigned int timeout = comm_data->data[1] >> 16;
			unsigned int count = comm_data->data[1] & 0xffff;
			PM_LOG_I("Explorer heartbeat received, period: %dms, count: %d\n",
					timeout, count);
			if ((epd->heartbeat_started == false) && (count == 0)) {
				epd->heartbeat_started = true;
			}

			if (timeout > 0) {
				/* heartbeat received, cancel the old one and start a new one */
				cancel_heartbeat_detect_work(epd);
				schedule_heartbeat_detect_work(epd, timeout);
			} else {
				PM_LOG_E("Explorer heartbeat received, error period:%dms\n", timeout);
			}
			break;
		}
		default:
			PM_LOG_E("Unknown Power Channel command: 0x%x\n", comm_data->data[0]);
	}

	PM_LOG_D("%s, done.\n", __func__);

	return ret;
}

static int explorer_proc_generic_msg(struct explorer_plat_data *epd, struct hal_comm_data *comm_data)
{
	int ret = 0;

	if (IPC_PLD_IN_MEMPOOL == comm_data->cmd_pld_pos) {
		ret = explorer_proc_ipcmem_msg(epd, comm_data);
		if (ret < 0) {
			pr_err("%s, explorer_proc_ipcmem_msg failed.\n", __func__);
			goto out;
		}
	} else if (IPC_PLD_IN_MBOXFIFO == comm_data->cmd_pld_pos) {
		ret = explorer_proc_gen_msg(epd, NULL, comm_data);
		if (ret < 0) {
			pr_err("%s, explorer_proc_gen_msg failed.\n", __func__);
			goto out;
		}
	} else {
		pr_err("%s, invalid payload pos flag: %d.\n", __func__, comm_data->cmd_pld_pos);
		goto out;
	}

	pr_debug("%s, done.\n", __func__);
out:
	return ret;
}

static int explorer_proc_mem_msg(struct explorer_plat_data *epd, struct hal_comm_data *comm_data)
{
	int ret = 0;
	u32 mbox_data[2] = {0};

	/* allocate ap buffer */
	if (comm_data->data_len > SC_READ_DATA_SIZE_MAX) {
		ret = -EINVAL;
		pr_err("%s, ipc payload length exceeds limit.\n", __func__);
		goto out;
	}

	ret = explorer_genl_mcast_data(epd, comm_data->cmd_mod_id, &comm_data->data[0], 8);
	if (ret) {
		pr_err("%s, netlink to userspace failed.\n", __func__);
		/* sync send cmd, reply to Explorer. */
		if (comm_data->cmd_is_reply == HAL_CMD_SEND)
			if (comm_data->cmd_is_sync == HAL_CMD_SYNC) {
				ret = explorer_reply_mbox(epd, comm_data->cmd_mod_id,
							  comm_data->cmd_sub_id, (void *)mbox_data);
				if (ret < 0) {
					pr_err("%s, explorer_reply_mbox failed.\n", __func__);
					goto out;
				}
			}
		goto out;
	}
out:
	return ret;
}

static int explorer_proc_cc_msg(struct explorer_plat_data *epd, struct hal_comm_data *comm_data)
{
	int ret = 0;

	if (IPC_PLD_IN_MEMPOOL == comm_data->cmd_pld_pos) {
		ret = explorer_proc_ipcmem_msg(epd, comm_data);
		if (ret < 0) {
			pr_err("%s, explorer_proc_ipcmem_msg failed.\n", __func__);
			goto out;
		}
	} else if (IPC_PLD_IN_MBOXFIFO == comm_data->cmd_pld_pos) {
		ret = explorer_proc_gen_msg(epd, NULL, comm_data);
		if (ret < 0) {
			pr_err("%s, explorer_proc_gen_msg failed.\n", __func__);
			goto out;
		}
	} else if (IPC_PLD_IN_MEMORY == comm_data->cmd_pld_pos) {
		ret = explorer_proc_mem_msg(epd, comm_data);
		if (ret < 0) {
			pr_err("%s, explorer_proc_gen_msg failed.\n", __func__);
			goto out;
		}
	}else {
		pr_err("%s, invalid payload pos flag: %d.\n", __func__, comm_data->cmd_pld_pos);
		goto out;
	}

	pr_debug("%s, done.\n", __func__);
out:
	return ret;
}

/* xfer layer comm cmd process table */
static int (*explorer_cmd_ops[])(struct explorer_plat_data *epd,
				 struct hal_comm_data *comm_data) = {
	[HAL_CMD_MALLOC]			NULL,
	[HAL_CMD_FREE]				NULL,
	[HAL_CMD_BOOTROM]			explorer_proc_bootrom_msg,
	[HAL_CMD_PBL]			explorer_proc_pbl_msg,
	[HAL_CMD_GEN]			explorer_proc_generic_msg,
	[HAL_CMD_SEND_LOG_ADDR]		explorer_proc_log_addr_msg,
	[HAL_CMD_SEC]			explorer_proc_ipcmem_msg,
	[HAL_CMD_ISP]			explorer_proc_cc_msg,
	[HAL_CMD_CAMERA]			explorer_proc_cc_msg,
	[HAL_CMD_RAM_DUMP]		explorer_proc_ramdump_msg,
	[HAL_CMD_HCLK]			explorer_proc_hclk_msg,
	[HAL_CMD_SDCLK]			explorer_proc_sdclk_msg,
#ifdef SLT_ENABLE
	[HAL_CMD_SLT]			explorer_proc_ipcmem_msg,
#endif
	[HAL_CMD_SUSPEND_RESUME]	explorer_proc_power_msg,
	[HAL_CMD_UPLOAD_BUFFER]		explorer_proc_logdump_msg,
	[HAL_CMD_GDB]			explorer_proc_ipcmem_msg,
	[HAL_CMD_EXCEPTION]		explorer_exception_msg,
	[HAL_CMD_MAX]			NULL,
};

static void explorer_process_msg(struct explorer_plat_data *epd, union mbox_data *mbox_item)
{
	struct hal_comm_data *comm_data;
	u32 cmd = 0;
	u32 i = 0;

	if (!epd || !mbox_item) {
		pr_err("%s, invalid argument.\n", __func__);
		return;
	}

	comm_data = (struct hal_comm_data *)mbox_item;

	for (i = 0; i < COMM_BUF_NUM; i++)
		pr_debug("%s, mbox fifo[%d] is 0x%x.\n", __func__, i, mbox_item->fifo[i]);

	/* process xfer layer communication protocol */
	cmd = HAL_CMD_DEFINE(0, comm_data->cmd_mod_id);
	if (cmd >= HAL_CMD_MAX) {
		pr_err("%s, cmd exceeds max number.\n", __func__);
		return;
	}

	pr_debug("%s, cmd sync: 0x%x\n", __func__, comm_data->cmd_is_sync);
	pr_debug("%s, cmd reply: 0x%x\n", __func__, comm_data->cmd_is_reply);
	pr_debug("%s, cmd end: 0x%x\n", __func__, comm_data->cmd_is_end);
	pr_debug("%s, cmd pld pos: 0x%x\n", __func__, comm_data->cmd_pld_pos);
	pr_debug("%s, cmd id: 0x%x cmd: %s\n", __func__, comm_data->cmd_mod_id,
	        comm_cmd_spec[comm_data->cmd_mod_id]);
	pr_debug("%s, cmd subid: 0x%x.\n", __func__, comm_data->cmd_sub_id);
	pr_debug("%s, data len: 0x%x\n", __func__, comm_data->data_len);
	pr_debug("%s, cmd sync index: 0x%x\n", __func__, comm_data->cmd_sync_idx);
	pr_debug("%s, cmd async index: 0x%x\n", __func__, comm_data->cmd_async_idx);
	pr_debug("%s, data value: 0x%x\n", __func__, comm_data->data[0]);

	if (explorer_cmd_ops[comm_data->cmd_mod_id] != NULL) {
		explorer_cmd_ops[comm_data->cmd_mod_id](epd, comm_data);
	}
}

static void explorer_mbox_msg_proc(struct work_struct *work)
{
	int ret = 0;
	struct mbox_work *mbox_worker = NULL;
	struct explorer_plat_data *epd = NULL;
	u32 i = 0, mbox_ap_msgnum = 0;
	union mbox_data *mbox_item;
	void *buffer = NULL;

	mbox_worker = container_of(work, struct mbox_work, worker);
	if (!mbox_worker) {
		pr_err("%s, work_data is null.\n", __func__);
		return;
	}
	pr_debug("%s, enter [mbox work %d].\n", __func__, mbox_worker->work_id);

	epd = (struct explorer_plat_data *)mbox_worker->userdata;
	if (!epd) {
		pr_err("%s, epd is null.\n", __func__);
		return;
	}

	/* read mbox msg to buffer */
	mutex_lock(&epd->mbox_rlock);
	/* read mbox msg num */
	ret = explorer_hal_sync_read(epd, MBOX_AP_MSGNUM, &mbox_ap_msgnum, 4);
	if (ret<0) {
		pr_err("%s, read mbox ap msg num failed.\n", __func__);
		mutex_unlock(&epd->mbox_rlock);
		return;
	}
	pr_debug("%s. mbox msg num = %d.\n", __func__, mbox_ap_msgnum);
	buffer = kzalloc(sizeof(union mbox_data)*mbox_ap_msgnum, GFP_KERNEL);
	if (!buffer) {
		pr_err("%s, kzalloc failed.\n", __func__);
		mutex_unlock(&epd->mbox_rlock);
		return;
	}
	mbox_item = (union mbox_data *)buffer;
#ifdef IPC_SOFT_DEBUG
	for (i = 0; i < mbox_ap_msgnum; i++) {
		static u32 count = 0;
		mbox_item->buf[0] = 0x80120000;	/* cmd 0x12 */
		mbox_item->buf[1] = 0x00000004;
		mbox_item->buf[2] = count++;
		mbox_item->buf[3] = count++;
		mbox_item++;
	}
#else
	if (mbox_ap_msgnum > 0) {
		ret = explorer_read_mbox(epd, mbox_item, mbox_ap_msgnum);
		if(ret < 0) {
			pr_err("%s, read mbox data failed.\n", __func__);
			kfree(buffer);
			mutex_unlock(&epd->mbox_rlock);
			return;
		}
	}
#endif
	mutex_unlock(&epd->mbox_rlock);

	/* parse mbox msg */
	mbox_item = (union mbox_data *)buffer;
	for (i = 0; i < mbox_ap_msgnum; i++) {
		explorer_process_msg(epd, mbox_item+i);
	}

	kfree(buffer);
	pr_debug("%s, exit [mbox work %d].\n", __func__, mbox_worker->work_id);
}

int explorer_mbox_irq_handler(struct explorer_plat_data *epd)
{
	int ret = 0;
	u32 clr_int = MBOX_INT_CLR_AP2CC_INT;
	static u32 work_count = 0;
	bool existed = false;
#ifdef IPC_SOFT_DEBUG
	clr_int = MBOX_INT_CLR_AP2CC_INT;
	mbox_ap_msgnum = 4;
#else
	/* clear mbox irq flag */
	ret = explorer_hal_sync_write(epd, MBOX_INT_CLR, &clr_int, 4);
	if (ret<0) {
		pr_err("%s, clear mbox interrupt flag failed.\n", __func__);
		goto out;
	}
#endif
	/* process mbox msg */
	epd->mbox_msg_work[work_count].work_id = work_count;
	epd->mbox_msg_work[work_count].userdata = (void*)epd;
	pr_debug("%s, [mbox work %d] is queued.\n", __func__, work_count);
	existed = queue_work(epd->mbox_msg_wq, &epd->mbox_msg_work[work_count].worker);
	if (existed == false) {
		pr_err("%s, mbox work[%d] already existed.\n", __func__, work_count);
		goto out;
	}
	if (++work_count >= MBOX_WORK_MAX_NUM)
		work_count = 0;

	/* mbox interrupt source really cleared!*/

	return 0;
out:
	return ret;
}

static int explorer_ipc_alloc_page(struct ipc_mempool *pool, u32 size, struct ipc_mempage *free_page);
static int explorer_ipc_set_cached_bitmap(struct ipc_mempool *pool);
static int explorer_ipc_get_free_space(struct ipc_mempool *pool);

int explorer_init_ipc(struct explorer_plat_data *epd)
{
	int ret = 0;
	u32 i = 0;
	char *version = IPC_API_VERSION;

	/* init ipc memory pool object */
	epd->ipc_mem_pool[IPC_MEM_AP2CC_CMD].base_addr = IPC_MEM_AP2CC_CMD_BASE;
	epd->ipc_mem_pool[IPC_MEM_AP2CC_CMD].pool_index = IPC_MEM_AP2CC_CMD;
	epd->ipc_mem_pool[IPC_MEM_AP2CC_CMD].page_pos = IPC_MEM_AP2CC_CMD_PAGE_POS;
	epd->ipc_mem_pool[IPC_MEM_AP2CC_CMD].page_num = IPC_MEM_AP2CC_CMD_PAGE_NUM;
	epd->ipc_mem_pool[IPC_MEM_AP2CC_CMD].page_size = IPC_MEM_AP2CC_CMD_PAGE_SIZE;
	epd->ipc_mem_pool[IPC_MEM_AP2CC_CMD].bitmap_cached = 0x0;
	epd->ipc_mem_pool[IPC_MEM_AP2CC_CMD].bitmap_mask = 0xFFFFFFFF;

	epd->ipc_mem_pool[IPC_MEM_AP2CC_DATA].base_addr = IPC_MEM_AP2CC_DATA_BASE;
	epd->ipc_mem_pool[IPC_MEM_AP2CC_DATA].pool_index = IPC_MEM_AP2CC_DATA;
	epd->ipc_mem_pool[IPC_MEM_AP2CC_DATA].page_pos = IPC_MEM_AP2CC_DATA_PAGE_POS;
	epd->ipc_mem_pool[IPC_MEM_AP2CC_DATA].page_num = IPC_MEM_AP2CC_DATA_PAGE_NUM;
	epd->ipc_mem_pool[IPC_MEM_AP2CC_DATA].page_size = IPC_MEM_AP2CC_DATA_PAGE_SIZE;
	epd->ipc_mem_pool[IPC_MEM_AP2CC_DATA].bitmap_cached = 0x0;
	epd->ipc_mem_pool[IPC_MEM_AP2CC_DATA].bitmap_mask = 0x00FFFFFF;

	epd->ipc_mem_pool[IPC_MEM_CC2AP_CMD].base_addr = IPC_MEM_CC2AP_CMD_BASE;
	epd->ipc_mem_pool[IPC_MEM_CC2AP_CMD].pool_index = IPC_MEM_CC2AP_CMD;
	epd->ipc_mem_pool[IPC_MEM_CC2AP_CMD].page_pos = IPC_MEM_CC2AP_CMD_PAGE_POS;
	epd->ipc_mem_pool[IPC_MEM_CC2AP_CMD].page_num = IPC_MEM_CC2AP_CMD_PAGE_NUM;
	epd->ipc_mem_pool[IPC_MEM_CC2AP_CMD].page_size = IPC_MEM_CC2AP_CMD_PAGE_SIZE;
	epd->ipc_mem_pool[IPC_MEM_CC2AP_CMD].bitmap_cached = 0x0;
	epd->ipc_mem_pool[IPC_MEM_CC2AP_CMD].bitmap_mask = 0xFFFFFFFF;

	epd->ipc_mem_pool[IPC_MEM_CC2AP_DATA].base_addr = IPC_MEM_CC2AP_DATA_BASE;
	epd->ipc_mem_pool[IPC_MEM_CC2AP_DATA].pool_index = IPC_MEM_CC2AP_DATA;
	epd->ipc_mem_pool[IPC_MEM_CC2AP_DATA].page_pos = IPC_MEM_CC2AP_DATA_PAGE_POS;
	epd->ipc_mem_pool[IPC_MEM_CC2AP_DATA].page_num = IPC_MEM_CC2AP_DATA_PAGE_NUM;
	epd->ipc_mem_pool[IPC_MEM_CC2AP_DATA].page_size = IPC_MEM_CC2AP_DATA_PAGE_SIZE;
	epd->ipc_mem_pool[IPC_MEM_CC2AP_DATA].bitmap_cached = 0x0;
	epd->ipc_mem_pool[IPC_MEM_CC2AP_DATA].bitmap_mask = 0x00FFFFFF;

	for (i = IPC_MEM_AP2CC_CMD; i < IPC_MEM_MAX; i++) {
		epd->ipc_mem_pool[i].alloc_page = explorer_ipc_alloc_page;
		epd->ipc_mem_pool[i].free_page = NULL;
		epd->ipc_mem_pool[i].set_cached_bitmap = explorer_ipc_set_cached_bitmap;
		epd->ipc_mem_pool[i].get_free_space = explorer_ipc_get_free_space;
		epd->ipc_mem_pool[i].private_data = (void *)epd;
	}

	/* init ipc memory range */
	epd->ipc_mem_range[IPC_MEM_RANGE0].addr_min = IPC_SRAM_RANGE_MIN;
	epd->ipc_mem_range[IPC_MEM_RANGE0].addr_max = IPC_SRAM_RANGE_MAX;
	epd->ipc_mem_range[IPC_MEM_RANGE1].addr_min = IPC_DDR_RANGE_MIN;
	epd->ipc_mem_range[IPC_MEM_RANGE1].addr_max = IPC_DDR_RANGE_MAX;

	/* init ipc lock */
	mutex_init(&epd->comm_lock);
	mutex_init(&epd->mbox_rlock);
	mutex_init(&epd->mbox_wlock);
	mutex_init(&epd->ipc_sync_lock);
	mutex_init(&epd->sc_rd_lock);
	spin_lock_init(&epd->ipc_page_slock);

	/* init mbox msg workqueue */
	epd->mbox_msg_wq = alloc_workqueue("mbox_wq", WQ_UNBOUND | WQ_HIGHPRI, 0);
	if(epd->mbox_msg_wq == NULL) {
		pr_err("%s, create mbox workqueue failed.\n", __func__);
		goto out;
	}

	for (i = 0; i < MBOX_WORK_MAX_NUM; i++)
		INIT_WORK(&epd->mbox_msg_work[i].worker, explorer_mbox_msg_proc);

	pr_info("%s, done, ipc api version: %s.\n", __func__, version);
out:
	return ret;
}

void explorer_dinit_ipc(struct explorer_plat_data *epd)
{
	if (epd == NULL) {
		pr_err("%s, epd is null.\n", __func__);
		return;
	}

	destroy_workqueue(epd->mbox_msg_wq);
}

static int explorer_ipc_verify_addr(struct explorer_plat_data *epd, u32 addr)
{
	int ret = 0;
	u32 i = 0;
	u32 count = 0;
	struct ipc_memrange *mem_range;

	if (!epd) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}
	mem_range = epd->ipc_mem_range;
	if (!mem_range) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < IPC_MEM_RANGE_MAX; i++) {
		if (mem_range[i].addr_min <= addr && addr <= mem_range[i].addr_max)
			break;
		else
			count++; /* doesn't match address */
	}

	if (count == IPC_MEM_RANGE_MAX) {
		pr_err("%s, ipc address verification failed at 0x%x.\n", __func__, addr);
		ret = -EINVAL;
		goto out;
	}

	pr_err("%s, done.\n", __func__);
out:
	return ret;
}

static int explorer_ipc_alloc_page(struct ipc_mempool *pool, u32 size, struct ipc_mempage *free_page)
{
	int ret = 0;
	struct explorer_plat_data *epd = NULL;
	u32 bitmap = 0;
	u32 free_count = 0;
	u32 bit_count = 0;
	u32 page_found = 0;
	u32 page_num_needed = 0;
	u32 i = 0;
	unsigned long flags;

	if (!pool) {
		pr_err("%s, pool is null.\n", __func__);
		return -EINVAL;
	}
	epd = (struct explorer_plat_data *)pool->private_data;
	if (!epd) {
		pr_err("%s, epd is null.\n", __func__);
		return -EINVAL;
	}

	page_num_needed = (size+pool->page_size-1)/pool->page_size;

	pr_debug("%s, pool[%d] cached bitmap before alloc is 0x%x.\n", __func__, pool->pool_index, pool->bitmap_cached);
	/* find successive bit in bitmap */
	bitmap = ~pool->bitmap_cached;
	while (bitmap) {
		if (++bit_count>pool->page_num)
			break;
		if (bitmap & 1) {
			if (++free_count>=page_num_needed) {
				page_found = 1;
				free_page->pool_index = pool->pool_index;
				free_page->page_index = bit_count - page_num_needed;
				free_page->page_num = page_num_needed;
				break;
			}
		}
		else
			free_count = 0;
		bitmap = bitmap >> 1;
	}
	if (!page_found) {
		pr_debug("%s, ipc memory pool[%d] has not enough free page.\n", __func__, pool->pool_index);
		ret = -ENOMEM;
		goto out;
	}

	spin_lock_irqsave(&epd->ipc_page_slock, flags);
	/* set cached bitmap */
	for (i = 0; i < free_page->page_num; i++)
		pool->bitmap_cached |= (0x1<<(free_page->page_index+i));
	spin_unlock_irqrestore(&epd->ipc_page_slock, flags);

	pr_debug("%s, pool[%d] cached bitmap after alloc is 0x%x.\n", __func__, pool->pool_index, pool->bitmap_cached);
	pr_debug("%s, free page index is %d, num is %d.\n", __func__, free_page->page_index, free_page->page_num);
	pr_debug("%s, done.\n", __func__);
out:
	return ret;
}

static int explorer_ipc_set_cached_bitmap(struct ipc_mempool *pool)
{
	struct explorer_plat_data *epd = NULL;
	u32 high_quotient = 0;
	u32 high_remainder = 0;
	u32 low_quotient = 0;
	u32 low_remainder = 0;
	unsigned long flags;

	if (!pool) {
		pr_err("%s, pool is null.\n", __func__);
		return -EINVAL;
	}
	epd = (struct explorer_plat_data *)pool->private_data;
	if (!epd) {
		pr_err("%s, epd is null.\n", __func__);
		return -EINVAL;
	}

	high_quotient = (pool->page_pos+pool->page_num-1)/32;
	high_remainder = (pool->page_pos+pool->page_num-1)%32;
	low_quotient = pool->page_pos/32;
	low_remainder = pool->page_pos%32;

	pr_debug("%s, high_quotient is %d, high_remainder is %d.\n", __func__, high_quotient, high_remainder);
	pr_debug("%s, low_quotient is %d, low_remainder is %d.\n", __func__, low_quotient, low_remainder);

	spin_lock_irqsave(&epd->ipc_page_slock, flags);
	if (high_quotient > low_quotient) {
		pool->bitmap_cached = (epd->ipc_mem_bitmap[high_quotient]<<(32-low_remainder)) |
			(epd->ipc_mem_bitmap[low_quotient]>>low_remainder);
	} else {
		pool->bitmap_cached = epd->ipc_mem_bitmap[low_quotient]>>low_remainder;
	}
	pool->bitmap_cached &= pool->bitmap_mask;
	pr_debug("%s, pool[%d] cached bitmap after update is 0x%x.\n", __func__, pool->pool_index, pool->bitmap_cached);
	spin_unlock_irqrestore(&epd->ipc_page_slock, flags);

	pr_debug("%s, done.\n", __func__);
	return 0;
}

int explorer_ipc_get_free_page(struct explorer_plat_data *epd, u32 payload_size, struct ipc_mempage *free_page)
{
	int ret = 0;
	u32 *pbitmap = NULL;
	enum ipc_mempool_index i;
	u32 retry_count = 0;

	if (!epd || !free_page) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}
	pbitmap = epd->ipc_mem_bitmap;

	/*
	  * Iterate all ipc memory pool, get free page from memory pool bitmap.
	  * If there is not enough free page, update cached bitmap from explorer's top ipc register.
	  */
	do {
		if (ret == -ENOMEM) {
			if (retry_count++>=3) {
				pr_err("%s, get free ap2cc ipc page failed.\n", __func__);
				goto out;
			}
			pr_debug("%s, no free ap2cc ipc page, update from explorer.\n", __func__);
			/* read ipc buff management register to update bitmap */
			ret = explorer_hal_sync_read(epd,
						     IPC_BUFCHAN_STATUS_BASE,
						     &epd->ipc_mem_bitmap,
						     IPC_MEM_AP2CC_BITMAP_LEN);
			if (ret<0) {
				pr_err("%s, explorer_hal_sync_read failed.\n", __func__);
				goto out;
			}
			pr_debug("%s, current ipc ap2cc bitmap = 0x%x_0x%x.\n", __func__, *(pbitmap+1), *pbitmap);
			for (i = IPC_MEM_AP2CC_CMD; i <= IPC_MEM_AP2CC_DATA; i++) {
				ret = (epd->ipc_mem_pool[i].set_cached_bitmap)(&epd->ipc_mem_pool[i]);
				if (ret<0) {
					pr_err("%s, set_bitmap failed.\n", __func__);
					goto out;
				}
			}
		}

		for (i = IPC_MEM_AP2CC_CMD; i <= IPC_MEM_AP2CC_DATA; i++) {
			ret = (epd->ipc_mem_pool[i].alloc_page)(&epd->ipc_mem_pool[i], payload_size, free_page);
			if (ret<0) {
				pr_debug("%s, memory pool[%d] doesn't have enough page.\n", __func__, i);
			} else {
				break;
			}
		}
	} while (ret == -ENOMEM);
	pr_debug("%s, done.\n", __func__);
out:
	return ret;
}

static int explorer_ipc_get_free_space(struct ipc_mempool *pool)
{
	int num = 0;
	u32 bitmap = 0;
	int i = 0;

	if (!pool) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}
	bitmap = ~pool->bitmap_cached;

	for (i = 0; i < pool->page_num; i++) {
		if((bitmap & 1) != 0)
			num++;
		bitmap >>= 1;
	}

	return num * pool->page_size;
}

static int explorer_ipc_update_cached_bitmap(struct explorer_plat_data *epd)
{
	int ret = 0;
	u32 *pbitmap = NULL;
	enum ipc_mempool_index i;
	u32 retry_count = 0;
	u32 free_space = 0;

	if (!epd) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}
	pbitmap = epd->ipc_mem_bitmap;

	/*
	  * Iterate all ipc memory pool, get free page from memory pool bitmap.
	  * If there is not enough free page, update cached bitmap from explorer's top ipc register.
	  */
	do {
		if (retry_count++>=3) {
			pr_err("%s, get free ap2cc ipc page failed.\n", __func__);
			goto out;
		}

		if (ret == -ENOMEM) {
			/* read ipc buff management register to update bitmap */
			ret = explorer_hal_sync_read(epd,
						     IPC_BUFCHAN_STATUS_BASE,
						     &epd->ipc_mem_bitmap,
						     IPC_MEM_AP2CC_BITMAP_LEN);
			if (ret<0) {
				pr_err("%s, explorer_hal_sync_read failed.\n", __func__);
				goto out;
			}
			pr_debug("%s, current ipc ap2cc bitmap = 0x%x_0x%x.\n", __func__, *(pbitmap+1), *pbitmap);

			for (i = IPC_MEM_AP2CC_CMD; i <= IPC_MEM_AP2CC_DATA; i++) {
				ret = (epd->ipc_mem_pool[i].set_cached_bitmap)(&epd->ipc_mem_pool[i]);
				if (ret<0) {
					pr_err("%s, set_bitmap failed.\n", __func__);
					goto out;
				}
			}
		}

		/* check ipc pool free space */
		free_space = (epd->ipc_mem_pool[IPC_MEM_AP2CC_DATA].get_free_space)(&epd->ipc_mem_pool[IPC_MEM_AP2CC_DATA]);
		if (free_space <= IPC_MEM_FREE_WATERMARK) {
			pr_debug("%s, ap2cc ipc pages are running out, update from explorer.\n", __func__);
			ret = -ENOMEM;
		} else {
			pr_debug("%s, ap2cc ipc pages are enough.\n", __func__);
			ret = 0;
		}
	} while (ret == -ENOMEM);
	pr_debug("%s, done.\n", __func__);
out:
	return ret;
}

static int explorer_ipc_cal_bitmap(struct explorer_plat_data *epd, u32 top,
				       struct ipc_mempage *free_page, u32 *mem_bitmap)
{
	int ret = 0;
	u32 i = 0;
	u32 high_quotient = 0;
	u32 high_remainder = 0;
	u32 low_quotient = 0;
	u32 low_remainder = 0;

	if (!epd || !free_page) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}

	high_quotient = (free_page->page_index+free_page->page_num-1)/32;
	high_remainder = (free_page->page_index+free_page->page_num-1)%32;
	low_quotient = free_page->page_index/32;
	low_remainder = free_page->page_index%32;
	pr_debug("%s, high_quotient is %d, high_remainder is %d.\n", __func__, high_quotient, high_remainder);
	pr_debug("%s, low_quotient is %d, low_remainder is %d.\n", __func__, low_quotient, low_remainder);
	if (high_quotient > low_quotient) {
		for (i = 0; i <= high_remainder; i++)
			mem_bitmap[high_quotient] |= (0x1<<i);
		for (i = 0; i <= (32-low_remainder-1); i++)
			mem_bitmap[low_quotient] |= (0x1<<(low_remainder+i));
	} else {
		for (i = low_remainder; i <= high_remainder; i++)
			mem_bitmap[low_quotient] |= (0x1<<i);
	}
	pr_debug("%s, pool[%d] top bitmap is 0x%x_0x%x.\n", __func__,
							   free_page->pool_index,
							   mem_bitmap[1],
							   mem_bitmap[0]);

	pr_debug("%s, done.\n", __func__);
	return ret;
}

static int explorer_ipc_update_top_bitmap(struct explorer_plat_data *epd, u32 top, struct ipc_mempage *free_page)
{
	int ret = 0;
	u32 ipc_mem_bitmap[IPC_BUFCHAN_NUM] = {0};

	ret = explorer_ipc_cal_bitmap(epd, top, free_page, ipc_mem_bitmap);
	if (ret < 0) {
		pr_err("%s, explorer_ipc_cal_bitmap failed.\n", __func__);
		goto out;
	}

	/* set ipc buff management register */
	ret = explorer_hal_sync_write(epd, top, (void *)ipc_mem_bitmap, IPC_BUFCHAN_LEN);
	if (ret<0) {
		pr_err("%s, explorer_hal_sync_write failed.\n", __func__);
		goto out;
	}

	pr_debug("%s, done.\n", __func__);
out:
	return ret;
}

int explorer_ipc_set_top_bitmap(struct explorer_plat_data *epd, struct ipc_mempage *free_page)
{
	int ret = 0;

	if (!epd || !free_page) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}

	ret = explorer_ipc_update_top_bitmap(epd, IPC_BUFCHAN_SET_BASE, free_page);
	if (ret<0) {
		pr_err("%s, explorer_ipc_update_top_bitmap failed.\n", __func__);
		return ret;
	}

	pr_debug("%s, done.\n", __func__);

	return ret;
}

int explorer_ipc_clear_top_bitmap(struct explorer_plat_data *epd, struct ipc_mempage *free_page)
{
	int ret = 0;

	if (!epd || !free_page) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}

	ret = explorer_ipc_update_top_bitmap(epd, IPC_BUFCHAN_CLEAR_BASE, free_page);
	if (ret<0) {
		pr_err("%s, explorer_ipc_update_top_bitmap failed.\n", __func__);
		return ret;
	}

	pr_debug("%s, done.\n", __func__);

	return ret;
}

int explorer_hal_sync_write(struct explorer_plat_data *epd, u32 cc_addr, void *ap_buffer, u32 len)
{
	int ret = 0, i = 0, count = 0;

	if (len == 0) {
		pr_err("%s, invalid write length.\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&epd->comm_lock);
	pr_debug("%s, begin.\n", __func__);
	if (epd->bus_type == IPC_BUS_CSPI) {
		for (i = 0; i < len/CSPI_PAYLOAD_MAX_LEN; i++) {
			ret = explorer_cspi_sync_write(epd->cspi, cc_addr+i*CSPI_PAYLOAD_MAX_LEN, \
							(u32 *)ap_buffer+i*CSPI_PAYLOAD_MAX_LEN/CSPI_WORD_LENGTH, CSPI_PAYLOAD_MAX_LEN>>2);
			if (ret<0) {
				pr_err("%s, cspi write failed.\n", __func__);
				goto out;
			}
			count += ret;
		}
		if ((len%CSPI_PAYLOAD_MAX_LEN) != 0) {
			len = len % CSPI_PAYLOAD_MAX_LEN;
			len = len >> 2; /* cspi word length */
			ret = explorer_cspi_sync_write(epd->cspi, cc_addr+i*CSPI_PAYLOAD_MAX_LEN, \
							(u32 *)ap_buffer+i*CSPI_PAYLOAD_MAX_LEN/CSPI_WORD_LENGTH, len);
			if (ret<0) {
				pr_err("%s, cspi write failed.\n", __func__);
				goto out;
			}
			count += ret;
		}
	} else if (epd->bus_type == IPC_BUS_SDIO) {
		ret = sdio_write_data(epd->sdio_data, cc_addr, ap_buffer, (int)len);
		if (ret) {
			pr_err("%s, sdio write failed, err: %d.\n", __func__, ret);
			goto out;
		}
		count = len;
	}
	pr_debug("%s, done.\n", __func__);
	mutex_unlock(&epd->comm_lock);
	return count;

out:
	mutex_unlock(&epd->comm_lock);
	return ret;
}

int explorer_hal_sync_batch_write(struct explorer_plat_data *epd, struct batch_data_item *ap_buffer, u32 count)
{
	int ret = 0;

	if (count == 0) {
		pr_err("%s, invalid write length.\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&epd->comm_lock);
	pr_debug("%s, begin.\n", __func__);
	if (epd->bus_type == IPC_BUS_SDIO) {
		ret = sdio_batch_write_data(epd->sdio_data, ap_buffer, count);
		if (ret) {
			pr_err("%s, sdio batch write failed, err: %d.\n", __func__, ret);
			goto failed;
		}
	}
	pr_debug("%s, done.\n", __func__);
failed:
	mutex_unlock(&epd->comm_lock);
	return ret;
}

int explorer_hal_sync_read(struct explorer_plat_data *epd, u32 cc_addr, void *ap_buffer, u32 len)
{
	int ret = 0, i = 0, count = 0;

	if (len == 0) {
		pr_err("%s, invalid read length.\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&epd->comm_lock);
	pr_debug("%s, begin.\n", __func__);
	if (epd->bus_type == IPC_BUS_CSPI) {
		for (i = 0; i < len/4; i++) {
			ret = explorer_cspi_sync_read(epd->cspi, cc_addr+i*CSPI_WORD_LENGTH, (u32 *)ap_buffer+i, 1);
			if (ret<0) {
				pr_err("%s, cspi read failed.\n", __func__);
				goto out;
			}
			count += ret;
		}
	} else if (epd->bus_type == IPC_BUS_SDIO) {
		ret = sdio_read_data(epd->sdio_data, cc_addr, ap_buffer, (int)len);
		if (ret) {
			pr_err("%s, sdio read failed, err: %d.\n", __func__, ret);
			goto out;
		}
		count = len;
	}
	pr_debug("%s, done.\n", __func__);
	mutex_unlock(&epd->comm_lock);
	return count;

out:
	mutex_unlock(&epd->comm_lock);
	return ret;
}

bool explorer_hal_get_hw_status(struct explorer_plat_data *epd)
{
	if (epd->bus_type == IPC_BUS_CSPI) {
		/* TODO */
		/* CSPI need add hw_probe_done = true */
		return true;
	} else if (epd->bus_type == IPC_BUS_SDIO) {
		return epd->hw_probe_done;
	}
	return true;
}


int explorer_hal_sync_write_internal_nolock(struct explorer_plat_data *epd,u32 regoffset, u8 in)
{
	int ret = 0;

	pr_debug("%s, begin.\n", __func__);
	if (epd->bus_type == IPC_BUS_CSPI) {
		/*TODO*/

	} else if (epd->bus_type == IPC_BUS_SDIO) {
		ret = sdio_write_register(epd->sdio_data, regoffset, in, FUNC0);
		if (ret) {
			pr_err("%s, sdio write internal register failed, err: %d.\n", __func__, ret);
			goto out;
		}
	}
	pr_debug("%s, done.\n", __func__);

out:
	return ret;

}

int explorer_hal_sync_read_internal_nolock(struct explorer_plat_data *epd,u32 regoffset, u8 *out) {
	int ret = 0;

	pr_debug("%s, begin.\n", __func__);
	if (epd->bus_type == IPC_BUS_CSPI) {
		/*TODO*/

	} else if (epd->bus_type == IPC_BUS_SDIO) {
		ret = sdio_read_register(epd->sdio_data, regoffset, out, FUNC0);
		if (ret) {
			pr_err("%s, sdio read internal register failed, err: %d.\n", __func__, ret);
			goto out;
		}
	}
	pr_debug("%s, done.\n", __func__);

out:
	return ret;

}


int explorer_hal_sync_write_internal(struct explorer_plat_data *epd,u32 regoffset, u8 in) {
	int ret = 0;

	mutex_lock(&epd->comm_lock);
	ret = explorer_hal_sync_write_internal_nolock(epd, regoffset, in);
	mutex_unlock(&epd->comm_lock);

	return ret;

}

int explorer_hal_sync_read_internal(struct explorer_plat_data *epd,u32 regoffset, u8 *out) {
	int ret = 0;

	mutex_lock(&epd->comm_lock);
	ret = explorer_hal_sync_read_internal_nolock(epd, regoffset, out);
	mutex_unlock(&epd->comm_lock);
	return ret;

}

/**
 * this interface is not thread safe.
 */
static u32 mbox_rmax_retry = 0;
static int explorer_update_mbox_fifo(struct explorer_plat_data *epd)
{
	int ret = 0;

	/* check cached mbox msg count */
	while (epd->mbox_cached_count >= (MBOX_FIFO_DEPTH_MAX-4)) {
		ret = explorer_hal_sync_read(epd, MBOX_CPU_MSGNUM, (void *)&epd->mbox_cached_count,
					      sizeof(u32));
		if (ret < 0) {
			pr_err("%s, explorer_hal_sync_write failed.\n", __func__);
			goto out;
		}
		if (mbox_rmax_retry++ >= MBOX_RETRY_MAX) {
			mbox_rmax_retry = 0;
			pr_err("%s, ap2cpu mailbox fifo is busy, num = %d.\n", __func__,
				epd->mbox_cached_count);
			goto out;
		}
		pr_debug("%s, update ap2cpu fifo msg num: %d.\n", __func__, epd->mbox_cached_count);
	}
	epd->mbox_cached_count++;
out:
	return ret;
}

static int explorer_cal_mbox(struct explorer_plat_data *epd, union mbox_data *data, int num)
{
	int ret = 0;
	u32 sindex = 0;
	u32 aindex = 0;
	u32 i = 0;
	struct hal_comm_data *comm_data = (struct hal_comm_data *)data;

	if (!epd || !data) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < num; i++,comm_data++) {
		if (comm_data->cmd_is_sync == HAL_CMD_SYNC) {
			sindex = atomic_add_return(1, &epd->mbox_ssindex[comm_data->cmd_mod_id]);
			aindex = atomic_read(&epd->mbox_saindex[comm_data->cmd_mod_id]);
		} else {
			sindex = atomic_read(&epd->mbox_ssindex[comm_data->cmd_mod_id]);
			aindex = atomic_add_return(1, &epd->mbox_saindex[comm_data->cmd_mod_id]);
		}
		comm_data->cmd_sync_idx = (sindex & CMD_INDEX_MASK);
		comm_data->cmd_async_idx = (aindex & CMD_INDEX_MASK);

		pr_debug("%s, mbox async index is %d.\n", __func__, comm_data->cmd_async_idx);
		pr_debug("%s, mbox sync index is %d.\n", __func__, comm_data->cmd_sync_idx);
	}

	return ret;
}

int explorer_write_mbox(struct explorer_plat_data *epd, union mbox_data *data, int num)
{
	int ret = 0;
	u32 i = 0;
	struct hal_comm_data *comm_data = (struct hal_comm_data *)data;

	if (!epd || !data) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&epd->mbox_wlock);
	ret = explorer_cal_mbox(epd, data, num);
	if (ret < 0) {
		pr_err("%s, explorer_cal_mbox failed.\n", __func__);
		goto out;
	}

	for (i = 0; i < num; i++,comm_data++) {
		ret = explorer_hal_sync_write(epd, MBOX_WR_AP2CPU, (void *)comm_data,
					      sizeof(union mbox_data));
		if (ret < 0) {
			pr_err("%s, explorer_hal_sync_write failed.\n", __func__);
			break;
		}
	}

	/* check ap2cpu mbox fifo msg number */
	ret = explorer_update_mbox_fifo(epd);
	if (ret < 0)
		pr_err("%s, explorer_update_mbox_fifo failed.\n", __func__);
out:
	mutex_unlock(&epd->mbox_wlock);
	return ret;
}

int explorer_send_mbox(struct explorer_plat_data *epd, u32 sync, u32 mod_id, u32 sub_id, void *data)
{
	int ret = 0;
	struct hal_comm_data comm_data = {0};

	/* construct mbox protocol */
	comm_data.cmd_is_sync = sync;
	comm_data.cmd_is_reply = HAL_CMD_SEND;
	comm_data.cmd_is_end = HAL_CMD_START;
	comm_data.cmd_pld_pos = IPC_PLD_IN_MBOXFIFO;
	comm_data.cmd_core_id = 0;
	comm_data.cmd_mod_id = mod_id;
	comm_data.data_len = MBOX_DATA_LEN;
	comm_data.cmd_sub_id = sub_id;
	comm_data.data[0] = *(u32 *)data;
	comm_data.data[1] = *((u32 *)data+1);
	ret = explorer_write_mbox(epd, (union mbox_data *)&comm_data, 1);
	if (ret<0) {
		pr_err("%s, explorer_write_mbox failed.\n", __func__);
		goto out;
	}

	pr_debug("%s, done.\n", __func__);
out:
	return ret;
}

int explorer_send_mbox_nowait(struct explorer_plat_data *epd, u32 mod_id, u32 sub_id, void *data)
{
	int ret = 0;

	ret = explorer_send_mbox(epd, HAL_CMD_ASYNC, mod_id, sub_id, data);
	if (ret<0) {
		pr_err("%s, explorer_write_generic_mbox failed.\n", __func__);
		goto out;
	}

	pr_debug("%s, done.\n", __func__);
out:
	return ret;
}

static int explorer_check_wait_condition(struct explorer_plat_data *epd, u32 ipc_id)
{
	u32 mod_id = EXT_MOD_ID(ipc_id);

	int sindex = atomic_read(&epd->mbox_ssindex[mod_id]);
	int rindex = atomic_read(&epd->mbox_rindex[mod_id]);

        if ((sindex & CMD_INDEX_MASK) == rindex)
                return 1;
        else
                return 0;
}

/*
 * Note: This interface writes mbox fifo directly and synchronously.
 * This interface is called from kernel and userspace.
 */
int explorer_send_mbox_wait(struct explorer_plat_data *epd, u32 mod_id, u32 sub_id,
				  void *data, struct ipc_sync_reply *reply)
{
	int ret = 0;
	long wq_ret = 0;
	struct hal_comm_data comm_data = {0};

	mutex_lock(&epd->ipc_sync_lock);
	pr_debug("%s, begin.\n", __func__);
	/* write data into cc directly */
	ret = explorer_send_mbox(epd, HAL_CMD_SYNC, mod_id, sub_id, data);
	if (ret<0) {
		pr_err("%s, explorer_write_generic_mbox failed.\n", __func__);
		goto out;
	}

	/* waiting reply from Explorer */
	pr_debug("%s, waiting for reply, related to cmd: 0x%x.\n", __func__, mod_id);
	wq_ret = wait_event_timeout(comm_sync_wq,
				    explorer_check_wait_condition(epd, mod_id),
				    HZ >> 3);
	if ((wq_ret == 0) || (wq_ret == 1)) {
		pr_err("%s, waiting timeout, cmd index=0x%x, wq_ret=%ld.\n",
			__func__, comm_data.cmd_sync_idx, wq_ret);
		ret = -ETIME;
		goto out;
	} else if (wq_ret > 1) {
		pr_debug("%s, waiting is complete, wq_ret=%ld.\n", __func__, wq_ret);
		if (epd->sync_reply_size > reply->size) {
			pr_err("%s, reply data size exceeds userspace size.\n", __func__);
			ret = -EINVAL;
			goto out;
		}
		if (reply->aflag == IPC_USER) {
			ret = copy_to_user((char __user *)reply->buffer, epd->sync_reply, epd->sync_reply_size)
				? -EFAULT : (epd->sync_reply_size);
			if (ret < 0) {
				pr_err("%s, can not copy data to userspace.\n", __func__);
				goto out;
			}
		} else if (reply->aflag == IPC_KERNEL) {
			memcpy(reply->buffer, epd->sync_reply, epd->sync_reply_size);
		} else {
			pr_err("%s, invalid address flag.\n", __func__);
			ret = -EINVAL;
			goto out;
		}
		ret = 0;
	}
	pr_debug("%s, done.\n", __func__);
out:
	mutex_unlock(&epd->ipc_sync_lock);
	return ret;
}

int explorer_reply_mbox(struct explorer_plat_data *epd, u32 mod_id, u32 sub_id, void *data)
{
	int ret = 0;
	struct hal_comm_data comm_data = {0};

	/* construct mbox protocol */
	comm_data.cmd_is_sync = HAL_CMD_SYNC;
	comm_data.cmd_is_reply = HAL_CMD_REPLY;
	comm_data.cmd_is_end = HAL_CMD_END;
	comm_data.cmd_pld_pos = IPC_PLD_IN_MBOXFIFO;
	comm_data.cmd_core_id = 0;
	comm_data.cmd_mod_id = mod_id;
	comm_data.data_len = MBOX_DATA_LEN;
	comm_data.cmd_sub_id = sub_id;
	comm_data.data[0] = *(u32 *)data;
	comm_data.data[1] = *((u32 *)data+1);
	ret = explorer_write_mbox(epd, (union mbox_data *)&comm_data, 1);
	if (ret<0) {
		pr_err("%s, explorer_write_mbox failed.\n", __func__);
		goto out;
	}

	pr_debug("%s, done.\n", __func__);
out:
	return ret;
}

/*
 * Note: This mbox read interface is not thread-safe.
 * AP_MSGNUM should be read before calling this interface.
 */
int explorer_read_mbox(struct explorer_plat_data *epd, union mbox_data *data, int num)
{
	int ret = 0;
	u32 i = 0;

	if (!epd || !data || (num>MBOX_FIFO_DEPTH_MAX)) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < num; i++) {
		ret = explorer_hal_sync_read(epd, MBOX_RD_AP, data+i, sizeof(union mbox_data));
		if (ret<0) {
			pr_err("%s, explorer_hal_sync_read failed.\n", __func__);
			break;
		}
	}

	return ret;
}

/**
 * this interface is not thread safe.
 * construct mbox packet and write mbox must in atomic.
 */
static int explorer_batch_write_ipc_data(struct explorer_plat_data *epd, u32 mbox, u32 ipc_id,
				       void *ap_buffer, u32 len)
{
	int ret = 0;
	u32 free_addr = 0;
	struct ipc_mempool *mem_pool;
	struct ipc_mempage free_page;
	struct hal_comm_data comm_data = {0};
	u32 checksum = 0;
	u32 mod_id = EXT_MOD_ID(ipc_id);
	u32 sub_id = EXT_SUB_ID(ipc_id);
	u32 i = 0;
	struct batch_data_item *batch_buffer = NULL;
	struct device *dev = NULL;
	u32 ipc_mem_bitmap[IPC_BUFCHAN_NUM] = {0};
	u32 aligned_len = 0;

	/* align xfer length to SDIO block size 512 */
	aligned_len = (len + 0xff) & (~0xff);

	if (!epd || (mod_id>HAL_CMD_MAX) || !ap_buffer || (aligned_len>IPC_MP_DATA_MAX_LENGTH)) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}
	dev = &(epd->plat_dev->dev);

	pr_debug("%s, begin. xfer len = %d, aligned len = %d.\n", __func__, len, aligned_len);

	/* get free page */
	ret = explorer_ipc_get_free_page(epd, aligned_len, &free_page);
	if (ret<0) {
		pr_err("%s, get ipc free memory failed.\n", __func__);
		goto out;
	}

	/* get free memory address */
	mem_pool = &epd->ipc_mem_pool[free_page.pool_index];
	free_addr = mem_pool->base_addr + mem_pool->page_size*free_page.page_index;
	pr_debug("%s, write ipc data at 0x%x, page num is %d.\n", __func__, free_addr, free_page.page_num);

	/**
	  * construct batch data, write at free address.
	  * ipc batch packet length: 2*(aligned_len+IPC_BUFCHAN_LEN+224+MBOX_ITEM_LEN),
	  * put it in shared memory.
	  */
	batch_buffer = (struct batch_data_item *)SHMEM_IPC_BATCH_BASE(epd->shared_buffer);
	for (i = 0; i < aligned_len / 4; i++) {
		batch_buffer[i].addr = free_addr + i * 4;
		batch_buffer[i].data = *((u32 *)ap_buffer+i);
	}

	/* calculate ipc memory bitmap */
	ret = explorer_ipc_cal_bitmap(epd, IPC_BUFCHAN_SET_BASE, &free_page, ipc_mem_bitmap);
	if (ret < 0) {
		pr_err("%s, explorer_ipc_cal_bitmap failed.\n", __func__);
		goto out;
	}
	for (i = 0; i < IPC_BUFCHAN_NUM+56; i++) {
		batch_buffer[aligned_len/4+i].addr = IPC_BUFCHAN_SET_BASE + (i%4)*4;
		batch_buffer[aligned_len/4+i].data = *((u32 *)ipc_mem_bitmap + (i%4));
	}

	/* construct mbox protocol */
	comm_data.cmd_is_sync = (mbox & 0xffff);
	comm_data.cmd_is_reply = HAL_CMD_SEND;
	comm_data.cmd_is_end = HAL_CMD_START;
	comm_data.cmd_pld_pos = ((mbox >> 16) & 0xffff);
	comm_data.cmd_core_id = 0;
	comm_data.cmd_mod_id = mod_id;
	comm_data.cmd_sub_id = sub_id;
	comm_data.data_len = len;
	memcpy((void *)&comm_data.data, (void *)&free_page, sizeof(free_page));
	explorer_ipc_checksum(ap_buffer, len, &checksum);
	comm_data.data[1] = checksum;
	ret = explorer_cal_mbox(epd, (union mbox_data *)&comm_data, 1);
	if (ret<0) {
		pr_err("%s, explorer_cal_mbox failed.\n", __func__);
		goto out;
	}
	for (i = 0; i < MBOX_ITEM_LEN/4; i++) {
		batch_buffer[aligned_len/4+IPC_BUFCHAN_NUM+56+i].addr = MBOX_WR_AP2CPU;
		batch_buffer[aligned_len/4+IPC_BUFCHAN_NUM+56+i].data = *((u32 *)&comm_data+i);
	}

	/* batch write */
	ret = sdio_batch_write_data(epd->sdio_data, batch_buffer,
				    2*(aligned_len+IPC_BUFCHAN_LEN+224+MBOX_ITEM_LEN));
	if (ret < 0) {
		pr_err("%s, sdio_batch_write_data failed.\n", __func__);
		goto out;
	}

	/* check ipc mem pool free space, update it as needed */
	ret = explorer_ipc_update_cached_bitmap(epd);
	if (ret < 0) {
		pr_err("%s, explorer_ipc_update_cached_bitmap failed.\n", __func__);
		goto out;
	}

	/* check ap2cpu mbox fifo msg number */
	ret = explorer_update_mbox_fifo(epd);
	if (ret < 0)
		pr_err("%s, explorer_update_mbox_fifo failed.\n", __func__);

	pr_debug("%s, done.\n", __func__);
out:
	return ret;
}

static int explorer_single_write_ipc_data(struct explorer_plat_data *epd, u32 mbox, u32 ipc_id,
					      void *ap_buffer, u32 len)
{
	int ret = 0;
	u32 free_addr = 0;
	struct ipc_mempool *mem_pool;
	struct ipc_mempage free_page;
	struct hal_comm_data comm_data = {0};
	u32 checksum = 0;
	u32 mod_id = EXT_MOD_ID(ipc_id);
	u32 sub_id = EXT_SUB_ID(ipc_id);

	if (!epd || (mod_id>HAL_CMD_MAX) || !ap_buffer || (len>IPC_MP_DATA_MAX_LENGTH)) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s, begin.\n", __func__);

	/* get free page */
	ret = explorer_ipc_get_free_page(epd, len, &free_page);
	if (ret<0) {
		pr_err("%s, get ipc free memory failed.\n", __func__);
		goto out;
	}

	/* get free memory address */
	mem_pool = &epd->ipc_mem_pool[free_page.pool_index];
	free_addr = mem_pool->base_addr + mem_pool->page_size*free_page.page_index;
	pr_debug("%s, write ipc data at 0x%x, page num is %d.\n", __func__, free_addr, free_page.page_num);

	/* write data at free address */
	ret = explorer_hal_sync_write(epd, free_addr, ap_buffer, len);
	if (ret<0) {
		pr_err("%s, explorer_hal_sync_write failed.\n", __func__);
		goto out;
	}

	/* set top bitmap */
	ret = explorer_ipc_set_top_bitmap(epd, &free_page);
	if (ret<0) {
		pr_err("%s, set free ipc mem bitmap failed.\n", __func__);
		goto out;
	}

	/* construct mbox protocol */
	comm_data.cmd_is_sync = (mbox & 0xffff);
	comm_data.cmd_is_reply = HAL_CMD_SEND;
	comm_data.cmd_is_end = HAL_CMD_START;
	comm_data.cmd_pld_pos = ((mbox >> 16) & 0xffff);
	comm_data.cmd_core_id = 0;
	comm_data.cmd_mod_id = mod_id;
	comm_data.cmd_sub_id = sub_id;
	comm_data.data_len = len;
	memcpy((void *)&comm_data.data, (void *)&free_page, sizeof(free_page));
	explorer_ipc_checksum(ap_buffer, len, &checksum);
	comm_data.data[1] = checksum;
	ret = explorer_write_mbox(epd, (union mbox_data *)&comm_data, 1);
	if (ret<0) {
		pr_err("%s, explorer_write_mbox failed.\n", __func__);
		goto out;
	}

	/* check ipc mem pool free space, update it as needed */
	ret = explorer_ipc_update_cached_bitmap(epd);
	if (ret < 0) {
		pr_err("%s, explorer_ipc_update_cached_bitmap failed.\n", __func__);
		goto out;
	}

	pr_debug("%s, done.\n", __func__);
out:
	return ret;
}

static int explorer_write_ipc_data(struct explorer_plat_data *epd, u32 mbox, u32 ipc_id,
				       void *ap_buffer, u32 len)
{
	int ret = 0;

	if (unlikely(IPC_SINGLE_XFER == epd->ipc_mode))
		ret = explorer_single_write_ipc_data(epd, mbox, ipc_id, ap_buffer, len);
	else if (IPC_BATCH_XFER == epd->ipc_mode)
		ret = explorer_batch_write_ipc_data(epd, mbox, ipc_id, ap_buffer, len);
	else {
		ret = -EINVAL;
		pr_err("%s, invalid ipc xfer mode %d.", __func__, epd->ipc_mode);
	}

	return ret;
}

/*
 * Note: This interface writes data to Explorer directly, by using IPC memory pool.
 */
int explorer_write_ipc_data_nowait(struct explorer_plat_data *epd, u32 ipc_id, void *ap_buffer, u32 len)
{
	int ret = 0;
	u32 mbox = 0;

	if (!epd || !ap_buffer || (len>IPC_MP_DATA_MAX_LENGTH)) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&epd->ipc_sync_lock);
	pr_debug("%s, begin.\n", __func__);
	mbox = (IPC_PLD_IN_MEMPOOL << 16) + HAL_CMD_ASYNC;
	ret = explorer_write_ipc_data(epd, mbox, ipc_id, ap_buffer, len);
	if (ret<0) {
		pr_err("%s, explorer_write_ipc_data failed.\n", __func__);
		goto out;
	}
	pr_debug("%s, done.\n", __func__);
out:
	mutex_unlock(&epd->ipc_sync_lock);
	return ret;
}

/*
 * Note: This interface writes data to Explorer directly, bypass IPC memory pool.
 */
static int explorer_write_data_directly(struct explorer_plat_data *epd, u32 mbox, u32 ipc_id,
					   u32 cc_addr, void *ap_buffer, u32 len)
{
	int ret = 0;
	struct hal_comm_data comm_data = {0};
	u32 checksum = 0;
	u32 mod_id = EXT_MOD_ID(ipc_id);
	u32 sub_id = EXT_SUB_ID(ipc_id);

	if (mod_id>HAL_CMD_MAX || sub_id>HAL_CMD_MAX) {
		pr_err("%s, invalid ipc id.\n", __func__);
		return -EINVAL;
	}

	/* write data at free address */
	ret = explorer_hal_sync_write(epd, cc_addr, ap_buffer, len);
	if (ret<0) {
		pr_err("%s, explorer_hal_sync_write failed.\n", __func__);
		goto out;
	}

	/* construct mbox protocol */
	comm_data.cmd_is_sync = (mbox & 0xffff);
	comm_data.cmd_is_reply = HAL_CMD_SEND;
	comm_data.cmd_is_end = HAL_CMD_START;
	comm_data.cmd_pld_pos = ((mbox >> 16) & 0xffff);
	comm_data.cmd_core_id = 0;
	comm_data.cmd_mod_id = mod_id;
	comm_data.data_len = len;
	comm_data.cmd_sub_id = sub_id;
	comm_data.data[0] = cc_addr;
	explorer_ipc_checksum(ap_buffer, len, &checksum);
	comm_data.data[1] = checksum;
	ret = explorer_write_mbox(epd, (union mbox_data *)&comm_data, 1);
	if (ret<0) {
		pr_err("%s, explorer_write_mbox failed.\n", __func__);
		goto out;
	}
out:
	return ret;
}

/*
 * Note: This interface writes data to Explorer directly and asynchronously, bypass IPC memory pool.
 */
int explorer_write_data_nowait(struct explorer_plat_data *epd, u32 ipc_id,
				    u32 cc_addr, void *ap_buffer, u32 len)
{
	int ret = 0;
	u32 mbox = 0;

	if (!epd || !ap_buffer || (len>IPC_GEN_DATA_SIZE_MAX)) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s, begin.\n", __func__);

	mbox = (IPC_PLD_IN_MEMORY << 16) + HAL_CMD_ASYNC;
	ret = explorer_write_data_directly(epd, mbox, ipc_id, cc_addr,
					   ap_buffer, len);
	if (ret<0) {
		pr_err("%s, explorer_write_data_directly failed.\n", __func__);
		goto out;
	}

	pr_debug("%s, done.\n", __func__);
out:
	return ret;
}

/*
 * Note: This interface writes data to Explorer directly and synchronously, bypass IPC memory pool.
 * This interface is called from userspace.
 */
int explorer_write_data_wait(struct explorer_plat_data *epd, u32 ipc_id, u32 cc_addr,
				void *ap_buffer, u32 len, struct explorer_sync_reply *reply)
{
	int ret = 0;
	struct hal_comm_data comm_data = {0};
	long wq_ret = 0;
	u32 mbox = 0;

	if (!epd || !ap_buffer || (len>IPC_GEN_DATA_SIZE_MAX)) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&epd->ipc_sync_lock);
	pr_debug("%s, begin.\n", __func__);

	mbox = (IPC_PLD_IN_MEMORY << 16) + HAL_CMD_SYNC;
	/* write data into cc directly */
	ret = explorer_write_data_directly(epd, mbox, ipc_id,
					   cc_addr, ap_buffer, len);
	if (ret<0) {
		pr_err("%s, explorer_write_data_directly failed.\n", __func__);
		goto out;
	}

	/* waiting reply from Explorer */
	pr_debug("%s, waiting for reply, related to cmd: 0x%x.\n", __func__, ipc_id);
        wq_ret = wait_event_timeout(comm_sync_wq,
				    explorer_check_wait_condition(epd, ipc_id),
				    COMM_WAIT_TIME*HZ);
        if ((wq_ret == 0) || (wq_ret == 1)) {
                pr_err("%s, waiting timeout, cmd index=0x%x, wq_ret=%ld.\n",
			__func__, comm_data.cmd_sync_idx, wq_ret);
                ret = -ETIME;
		goto out;
        } else if (wq_ret > 1) {
                pr_debug("%s, waiting is complete, wq_ret=%ld.\n", __func__, wq_ret);
		if (epd->sync_reply_size > reply->size) {
			pr_err("%s, reply data size exceeds userspace size.\n", __func__);
			ret = -EINVAL;
			goto out;
		}
		ret = copy_to_user((char __user *)reply->buffer, epd->sync_reply, epd->sync_reply_size)
			? -EFAULT : (epd->sync_reply_size);
		if (ret < 0) {
			pr_err("%s, can not copy data to userspace.\n", __func__);
			goto out;
		}
                ret = 0;
        }
	pr_debug("%s, done.\n", __func__);
out:
	mutex_unlock(&epd->ipc_sync_lock);
	return ret;
}

/*
 * Note: This interface writes data to Explorer synchronously, by using IPC memory pool.
 * This interface is called from userspace.
 */
int explorer_write_ipc_data_wait(struct explorer_plat_data *epd, u32 ipc_id,
				     void *ap_buffer, u32 len, struct explorer_sync_reply *reply)
{
	int ret = 0;
	long wq_ret = 0;
	struct hal_comm_data comm_data = {0};
	u32 mbox = 0;

	if (!epd || !ap_buffer || (len>IPC_MP_DATA_MAX_LENGTH)) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&epd->ipc_sync_lock);
	pr_debug("%s, begin.\n", __func__);
	mbox = (IPC_PLD_IN_MEMPOOL << 16) + HAL_CMD_SYNC;
	ret = explorer_write_ipc_data(epd, mbox, ipc_id, ap_buffer, len);
	if (ret<0) {
		pr_err("%s, explorer_write_ipc_data failed.\n", __func__);
		goto out;
	}

	/* waiting reply from Explorer */
	pr_debug("%s, waiting for reply, related to cmd: 0x%x.\n", __func__, ipc_id);
        wq_ret = wait_event_timeout(comm_sync_wq,
				    explorer_check_wait_condition(epd, ipc_id),
				    HZ >> 3);
        if ((wq_ret == 0) || (wq_ret == 1)) {
                pr_err("%s, waiting timeout, cmd index=0x%x, wq_ret=%ld.\n",
			__func__, comm_data.cmd_sync_idx, wq_ret);
                ret = -ETIME;
		goto out;
        } else if (wq_ret > 1) {
                pr_debug("%s, waiting is complete, wq_ret=%ld.\n", __func__, wq_ret);
		if (epd->sync_reply_size > reply->size) {
			pr_err("%s, reply data size exceeds userspace size.\n", __func__);
			ret = -EINVAL;
			goto out;
		}
		ret = copy_to_user((char __user *)reply->buffer, epd->sync_reply, epd->sync_reply_size)
			? -EFAULT : (epd->sync_reply_size);
		if (ret < 0) {
			pr_err("%s, can not copy data to userspace.\n", __func__);
			goto out;
		}
                ret = 0;
        }

	pr_debug("%s, done.\n", __func__);
out:
	mutex_unlock(&epd->ipc_sync_lock);
	return ret;
}

int explorer_write_generic_data(struct explorer_plat_data *epd, void *ap_buffer,
				    u32 len, struct explorer_gen_data_header *gen_header)
{
	int ret = 0;
	u32 gen_cmd = 0;
	u32 sync_mode = 0;

	if (!epd || !ap_buffer || (len>IPC_GEN_DATA_SIZE_MAX) || !gen_header) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s, begin.\n", __func__);

	/* set gen_cmd according to sys id */
	if (IPC_GEN_SYSID_PBL == gen_header->dst_ep.sys_id)
		gen_cmd = HAL_CMD_PBL;
	else if (IPC_GEN_SYSID_RTTHREAD == gen_header->dst_ep.sys_id)
		gen_cmd = HAL_CMD_GEN;
	else {
		pr_err("%s, invaild system id.\n", __func__);
		return -EINVAL;
	}

	/* generic command */
	gen_cmd = ((gen_header->dst_ep.mod_id) << 16) + gen_cmd;

	sync_mode = IPC_GET_SYNC_MODE(gen_header->mode);
	if (gen_header->dst_addr == IPC_AUTO_ALLOC_ADDR) {
		if (sync_mode == IPC_SYNC_SEND_MODE) {
			ret = explorer_write_ipc_data_wait(epd, gen_cmd, ap_buffer, len,
							   &gen_header->reply);
			if (ret<0) {
				pr_err("%s, explorer_write_ipc_data_wait failed.\n", __func__);
				goto out;
			}
		} else if (sync_mode == IPC_ASYNC_SEND_MODE) {
			ret = explorer_write_ipc_data_nowait(epd, gen_cmd, ap_buffer, len);
			if (ret<0) {
				pr_err("%s, explorer_write_ipc_data_nowait failed.\n", __func__);
				goto out;
			}
		} else {
			pr_err("%s, invalid sync flag: %d.\n", __func__, sync_mode);
			goto out;
		}
	} else {
		ret = explorer_ipc_verify_addr(epd, gen_header->dst_addr);
		if (ret) {
			pr_err("%s, explorer_ipc_verify_addr failed.\n", __func__);
			goto out;
		}
		if (sync_mode == IPC_SYNC_SEND_MODE) {
			ret = explorer_write_data_wait(epd, gen_cmd, gen_header->dst_addr, ap_buffer,
						       len, &gen_header->reply);
			if (ret<0) {
				pr_err("%s, explorer_write_data_wait failed.\n", __func__);
				goto out;
			}
		} else if (sync_mode == IPC_ASYNC_SEND_MODE) {
			ret = explorer_write_data_nowait(epd, gen_cmd, gen_header->dst_addr,
							 ap_buffer, len);
			if (ret<0) {
				pr_err("%s, explorer_write_data_nowait failed.\n", __func__);
				goto out;
			}
		} else {
			pr_err("%s, invalid sync flag: %d.\n", __func__, sync_mode);
			goto out;
		}
	}

	pr_debug("%s, done.\n", __func__);
out:
	return ret;
}

/**
 * This interface is only called from userspace.
 */
int explorer_write_generic_cmd(struct explorer_plat_data *epd, void *ap_buffer,
				    u32 len, struct explorer_gen_data_header *gen_header)
{
	int ret = 0;
	u32 gen_cmd = 0;
	u32 sub_id = 0;
	u32 sync_mode = 0;

	if (!epd || !ap_buffer || (len>MBOX_DATA_LEN) || !gen_header) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s, begin.\n", __func__);

	/* set gen_cmd according to sys id */
	if (IPC_GEN_SYSID_PBL == gen_header->dst_ep.sys_id)
		gen_cmd = HAL_CMD_PBL;
	else if (IPC_GEN_SYSID_RTTHREAD == gen_header->dst_ep.sys_id)
		gen_cmd = HAL_CMD_GEN;
	else {
		pr_err("%s, invaild system id.\n", __func__);
		return -EINVAL;
	}
	sub_id = gen_header->dst_ep.mod_id;

	sync_mode = IPC_GET_SYNC_MODE(gen_header->mode);
	if (sync_mode == IPC_SYNC_SEND_MODE) {
		struct ipc_sync_reply reply = {0};
		reply.aflag = IPC_USER;
		reply.buffer = gen_header->reply.buffer;
		reply.size = gen_header->reply.size;
		ret = explorer_send_mbox_wait(epd, gen_cmd, sub_id, ap_buffer, &reply);
		if (ret<0) {
			pr_err("%s, explorer_send_mbox_wait failed.\n", __func__);
			goto out;
		}
	} else if (sync_mode == IPC_ASYNC_SEND_MODE) {
		ret = explorer_send_mbox_nowait(epd, gen_cmd, sub_id, ap_buffer);
		if (ret<0) {
			pr_err("%s, explorer_send_mbox_nowait failed.\n", __func__);
			goto out;
		}
	} else {
		pr_err("%s, invalid sync flag: %d.\n", __func__, sync_mode);
		goto out;
	}

	pr_debug("%s, done.\n", __func__);
out:
	return ret;
}

/**
 * This interface called from kernel.
 */
int explorer_write_generic_cmd_kernel(struct explorer_plat_data *epd, void *ap_buffer,
				    u32 len, struct explorer_gen_data_header *gen_header)
{
	int ret = 0;
	u32 gen_cmd = 0;
	u32 sub_id = 0;
	u32 sync_mode = 0;

	if (!epd || !ap_buffer || (len>MBOX_DATA_LEN) || !gen_header) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s, begin.\n", __func__);

	/* set gen_cmd according to sys id */
	if (IPC_GEN_SYSID_PBL == gen_header->dst_ep.sys_id)
		gen_cmd = HAL_CMD_PBL;
	else if (IPC_GEN_SYSID_RTTHREAD == gen_header->dst_ep.sys_id)
		gen_cmd = HAL_CMD_GEN;
	else {
		pr_err("%s, invaild system id.\n", __func__);
		return -EINVAL;
	}
	sub_id = gen_header->dst_ep.mod_id;

	sync_mode = IPC_GET_SYNC_MODE(gen_header->mode);
	if (sync_mode == IPC_SYNC_SEND_MODE) {
		struct ipc_sync_reply reply = {0};
		reply.aflag = IPC_KERNEL;
		reply.buffer = gen_header->reply.buffer;
		reply.size = gen_header->reply.size;
		ret = explorer_send_mbox_wait(epd, gen_cmd, sub_id, ap_buffer, &reply);
		if (ret<0) {
			pr_err("%s, explorer_send_mbox_wait failed.\n", __func__);
			goto out;
		}
	} else if (sync_mode == IPC_ASYNC_SEND_MODE) {
		ret = explorer_send_mbox_nowait(epd, gen_cmd, sub_id, ap_buffer);
		if (ret<0) {
			pr_err("%s, explorer_send_mbox_nowait failed.\n", __func__);
			goto out;
		}
	} else {
		pr_err("%s, invalid sync flag: %d.\n", __func__, sync_mode);
		goto out;
	}

	pr_debug("%s, done.\n", __func__);
out:
	return ret;
}


int explorer_write_hclk_data(struct explorer_plat_data *epd, unsigned int data)
{
	int ret = 0;
	struct hal_comm_data comm_data_ack = {0};

	PM_LOG_I("begin.\n");

	//send ack to CC
	comm_data_ack.cmd_is_sync = HAL_CMD_ASYNC;
	comm_data_ack.cmd_is_reply = HAL_CMD_SEND;
	comm_data_ack.cmd_is_end = HAL_CMD_START;
	comm_data_ack.cmd_mod_id = HAL_CMD_HCLK;
	comm_data_ack.data_len = HAL_CMD_DATA_LEN_4;
	comm_data_ack.data[0] = data & 0xffff;

	ret = explorer_write_mbox(epd, (union mbox_data *)&comm_data_ack, 1);
	if (ret < 0) {
		pr_err("%s, explorer_hal_sync_write failed.\n", __func__);
	}

	PM_LOG_I("done.\n");

	return ret;
}

int explorer_write_power_data(struct explorer_plat_data *epd, unsigned int data0, unsigned int data1)
{
	int ret = 0;
	struct hal_comm_data comm_data_ack = {0};

	PM_LOG_I("begin.\n");

	//send ack to CC
	comm_data_ack.cmd_is_sync = HAL_CMD_ASYNC;
	comm_data_ack.cmd_is_reply = HAL_CMD_SEND;
	comm_data_ack.cmd_is_end = HAL_CMD_START;
	comm_data_ack.cmd_mod_id = HAL_CMD_SUSPEND_RESUME;
	comm_data_ack.data_len = HAL_CMD_DATA_LEN_8;
	comm_data_ack.data[0] = data0;
	comm_data_ack.data[1] = data1;

	ret = explorer_write_mbox(epd, (union mbox_data *)&comm_data_ack, 1);
	if (ret < 0) {
		pr_err("%s, explorer_hal_sync_write failed.\n", __func__);
	}
	PM_LOG_I("done.\n");

	return ret;
}

/**
 * Regular ".doit"-callback function if a Generic Netlink with command `GNL_FOOBAR_XMPL_C_ECHO` is received.
 * Please look into the comments where this is used as ".doit" callback above in
 * `struct genl_ops gnl_foobar_xmpl_ops[]` for more information about ".doit" callbacks.
*/
int explorer_genl_doit(struct sk_buff *sender_skb, struct genl_info *info)
{
	int ret = 0;

	pr_debug("%s bigin.\n", __func__);

	if (info == NULL) {
		pr_err("An error occurred in %s():\n", __func__);
		return -EINVAL;
	}

	return ret;
}

/**
 * Generic Netlink attribute and policy.
 */
enum {
	EXPLORER_GENL_ATTR_UNSPEC,
	EXPLORER_GENL_ATTR_ID,
	EXPLORER_GENL_ATTR_CC_DATA,
	EXPLORER_GENL_ATTR_START_FLAG,
	EXPLORER_GENL_ATTR_END_FLAG,
	__EXPLORER_GENL_ATTR_MAX,
};
#define EXPLORER_GENL_ATTR_MAX (__EXPLORER_GENL_ATTR_MAX - 1)

static struct nla_policy explorer_genl_policy[EXPLORER_GENL_ATTR_MAX + 1] = {
      [EXPLORER_GENL_ATTR_ID] = { .type = NLA_U32 },
      [EXPLORER_GENL_ATTR_CC_DATA] = { .type = NLA_BINARY },
      [EXPLORER_GENL_ATTR_START_FLAG] = { .type = NLA_BINARY },
      [EXPLORER_GENL_ATTR_END_FLAG] = { .type = NLA_U32 },
};

/**
 * Generic Netlink operations array.
 * cmd: from userspace.
 * doit: receive callback.
 */
enum {
        EXPLORER_GENL_CMD_UNSPEC,
        EXPLORER_GENL_CMD_SEND,
        __HWSIM_GENL_CMD_MAX,
};
#define EXPLORER_GENL_CMD_MAX (EXPLORER_GENL_CMD_MAX - 1)

static const struct genl_ops explorer_genl_ops[] = {
        {
                .cmd = EXPLORER_GENL_CMD_SEND,
                .validate = GENL_DONT_VALIDATE_STRICT | GENL_DONT_VALIDATE_DUMP,
                .doit = explorer_genl_doit,
                .flags = GENL_UNS_ADMIN_PERM,
        },
};

/* Generic Netlink multicast groups */
enum explorer_multicast_groups {
        EXPLORER_MCGRP,
};
static const struct genl_multicast_group explorer_mcgrps[] = {
        [EXPLORER_MCGRP] = { .name = "explorer_preisp", },
};

/* family definition */
static struct genl_family explorer_genl_family __ro_after_init = {
        .name = "EXPLORER_PREISP",
        .version = 1,
        .maxattr = EXPLORER_GENL_ATTR_MAX,
        .policy = explorer_genl_policy,
        .netnsok = false,
        .module = THIS_MODULE,
        .ops = explorer_genl_ops,
        .n_ops = ARRAY_SIZE(explorer_genl_ops),
        .mcgrps = explorer_mcgrps,
        .n_mcgrps = ARRAY_SIZE(explorer_mcgrps),
};

int explorer_genetlink_init(struct explorer_plat_data *epd)
{
	int ret = 0;

	if (!epd) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}

	/* register a family */
	ret = genl_register_family(&explorer_genl_family);
	if (ret) {
		epd->genl = false;
		pr_err("%s, register family failed, error code is %d.\n", __func__, ret);
		goto failed;
	}
	epd->genl = true;

	/* init ipc lock */
	mutex_init(&epd->genl_lock);

	pr_info("%s, init generic netlink done.\n", __func__);
 failed:
	return ret;
}

int explorer_genetlink_exit(struct explorer_plat_data *epd)
{
	int ret = 0;

	if (!epd) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}

	if (epd->genl)
		ret = genl_unregister_family(&explorer_genl_family);

	pr_info("%s, explorer generic netlink unloaded.", __func__);

	return ret;
}

static int explorer_genl_append_data(struct sk_buff *skb, void *ap_buffer, u32 size,
					   struct ipc_genl_header *genl_hdr, u32 start, u32 end)
{
	int ret = 0;
	u32 end_flag_magic = 0xAABBCCDD;

	/* add start flag */
	if (start == 1) {
		ret = nla_put(skb, EXPLORER_GENL_ATTR_START_FLAG, sizeof(struct ipc_genl_header),
			     (void *)genl_hdr);
		if (ret < 0) {
			pr_err("%s, put header failed, err code is %d.\n", __func__, ret);
			goto out;
		}
	}

	/* add data */
	ret = nla_put(skb, EXPLORER_GENL_ATTR_CC_DATA, size, ap_buffer);
	if (ret < 0) {
		pr_err("%s, put data failed, err code is %d.\n", __func__, ret);
		goto out;
	}

	/* add end flag */
	if (end == 1) {
		ret = nla_put_u32(skb, EXPLORER_GENL_ATTR_END_FLAG, end_flag_magic);
		if (ret < 0) {
			pr_err("%s, put tail failed, err code is %d.\n", __func__, ret);
			goto out;
		}
	}
	pr_debug("%s, done.\n", __func__);
out:
	return ret;
}

static int explorer_genl_mcast_frame(struct explorer_plat_data *epd, void *ap_buffer, u32 frame_size,
					  struct ipc_genl_header *genl_hdr, u32 start, u32 end)
{
	int ret = 0;
	struct sk_buff *mcast_skb;
	void *data;

	if (!ap_buffer || (frame_size > GENL_PAYLOAD_FRAME_MAX_LENGTH) || frame_size == 0) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s, begin.\n", __func__);

	mcast_skb = genlmsg_new(GENLMSG_DEFAULT_SIZE, GFP_KERNEL);
	if (!mcast_skb) {
		pr_err("%s, genlmsg_new failed.\n", __func__);
		return -ENOMEM;
	}

	data = genlmsg_put(mcast_skb, 0, 0, &explorer_genl_family, 0,
			   EXPLORER_GENL_CMD_SEND);
	if (!data) {
		pr_err("%s, genlmsg_put failed.\n", __func__);
		ret = -EFAULT;
		goto out_err;
	}

	ret = explorer_genl_append_data(mcast_skb, ap_buffer, frame_size, genl_hdr, start, end);
	if (ret < 0) {
		pr_err("%s, explorer_genl_append_data failed.\n", __func__);
		goto out_err;
	}

        genlmsg_end(mcast_skb, data);
        ret = genlmsg_multicast(&explorer_genl_family, mcast_skb, 0,
				EXPLORER_MCGRP, GFP_KERNEL);
	if (ret < 0) {
		if (ret == -ESRCH) {
			pr_err("%s, there is no listen thread in userspace, error code is %d.\n", __func__, ret);
			ret = 0;
		} else {
			pr_err("%s, genlmsg_multicast failed, error code is %d.\n", __func__, ret);
		}
	}
	pr_debug("%s, done.\n", __func__);

        return ret;
out_err:
        nlmsg_free(mcast_skb);
	return ret;
}

int explorer_genl_mcast_data(struct explorer_plat_data *epd, u32 id, void *ap_buffer, u32 size)
{
	int ret = 0;
	u32 i = 0;
	u8 *buffer = (u8 *)ap_buffer;
	u32 size_mod = size % GENL_PAYLOAD_FRAME_MAX_LENGTH;
	u32 size_quot = size / GENL_PAYLOAD_FRAME_MAX_LENGTH;
	struct ipc_genl_header genl_header;
#ifdef SLT_ENABLE
	struct slt_resp_msg *slt_resp = (struct slt_resp_msg *)ap_buffer;
#endif

	if (!ap_buffer || (size > GENL_PAYLOAD_TOTAL_MAX_LENGTH)) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s, begin, id = 0x%x.\n", __func__, id);
#ifdef SLT_ENABLE
	if (id == HAL_CMD_SLT) {
		pr_info("SLT_TEST_CASE: case id: %d, deltaTime: %d(ms)", slt_resp->testcase_id,
			jiffies_to_msecs(jiffies - epd->ebs.start_jiffies));
	}
#endif
	mutex_lock(&epd->genl_lock);
	genl_header.id = id;
	genl_header.size = size;
	for(i = 0; i < size_quot; i++) {
		ret = explorer_genl_mcast_frame(epd, (void *)(buffer+i*GENL_PAYLOAD_FRAME_MAX_LENGTH),
						GENL_PAYLOAD_FRAME_MAX_LENGTH, &genl_header, (i==0)?1:0,
						((i==(size_quot-1))&&(size_mod==0))?1:0);
		if (ret < 0) {
			pr_err("%s, explorer_genl_mcast_frame failed.\n", __func__);
			goto out;
		}
	}

	if (size_mod != 0) {
		ret = explorer_genl_mcast_frame(epd, (void *)(buffer+i*GENL_PAYLOAD_FRAME_MAX_LENGTH),
						size_mod, &genl_header, (i==0)?1:0, 1);
		if (ret < 0) {
			pr_err("%s, explorer_genl_mcast_frame failed.\n", __func__);
			goto out;
		}
	}
	pr_debug("%s, done.\n", __func__);
out:
	mutex_unlock(&epd->genl_lock);
        return ret;
}

/* for tuning */
#define RETRY_TUNING_TIMES 3

int explorer_execute_tuning(struct explorer_plat_data *explorer_data)
{
	int ret = 0;
	struct explorer_sdio_data *pdata = NULL;

	pdata = explorer_data->sdio_data;

	disable_irq(explorer_data->bsp_irq);
	if (explorer_data->tuning_retry_num < RETRY_TUNING_TIMES) {
		explorer_data->tuning_retry_num++;
		ret = explorer_sdio_execute_tuning(pdata);
		if (ret) {
			if (ret == TUNING_FAIL_RECOVERABLE)
				enable_irq(explorer_data->bsp_irq);
			pr_err("zeku:%s failed, err(%d)\n", __func__, ret);
			goto err;
		}
		explorer_data->tuning_retry_num = 0;
	} else {
		enable_irq(explorer_data->bsp_irq);
		pr_info("%s, tuning:by pass\n", __func__);
		goto err;
	}
	enable_irq(explorer_data->bsp_irq);
	pr_info("%s, tuning:%s.\n", __func__, ret ? "fail" : "done");

err:
	return ret;
}

