/*
 * Copyright (c) 2021 ZEKU Technology(Shanghai) Corp.,Ltd, all rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Basecode Created :        2021/03/17 Author: wangqinyuan@zeku.com
 *
 */

#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include "../explorer/include/main.h"
#include "../explorer/include/ipc.h"
#include "include/slt.h"

extern struct wait_queue_head comm_sync_wq;
static struct slt_cast_node slt_case[SLT_LOCAL_CASE_NUM];

/**
 * slt message from rtos
 */
int explorer_proc_slt_msg(struct explorer_plat_data *epd, void *ap_buffer, struct hal_comm_data *comm_data)
{
	int ret = 0;
	u32 size = comm_data->data_len;

	/* this is a reply cmd */
	if (comm_data->cmd_is_reply == HAL_CMD_REPLY) {
		/* sync cmd, wake up wait queue. */
		if (comm_data->cmd_is_sync == HAL_CMD_SYNC) {
			/* copy reply buffer */
			memcpy(epd->shared_buffer, ap_buffer, size);
			epd->sync_reply = epd->shared_buffer;
			epd->sync_reply_size = size;
			atomic_set(&epd->mbox_rindex[comm_data->cmd_mod_id], comm_data->cmd_sync_idx);
			pr_info("%s, done.\n", __func__);
			/* wake up wait queue */
			wake_up(&comm_sync_wq);
			return ret;
		} else {
			/* send cmd, netlink to userspace */
			ret = explorer_genl_mcast_data(epd, comm_data->cmd_mod_id, ap_buffer, size);
			if (ret)
				pr_err("%s, send cmd, netlink to userspace failed.\n", __func__);
		}
	} else {
		/* send cmd, netlink to userspace */
		ret = explorer_genl_mcast_data(epd, comm_data->cmd_mod_id, ap_buffer, size);
		if (ret)
			pr_err("%s, send cmd, netlink to userspace failed.\n", __func__);
	}

	return ret;
}

/**
 * slt message from pbl
 */
int explorer_proc_pbl_slt_msg(struct explorer_plat_data *epd, struct hal_comm_data *comm_data)
{
	int ret = 0;
	u32 cc_addr = comm_data->data[0];
	u32 size = comm_data->data_len;
	void *ap_buffer = NULL;
	struct device *dev = &(epd->plat_dev->dev);

	/* allocate ap buffer */
	if (comm_data->data_len > SLT_RESP_MSG_LEN_MAX) {
		pr_err("%s, slt payload length exceeds limit.\n", __func__);
		return -EINVAL;
	}
	ap_buffer = devm_kzalloc(dev, comm_data->data_len, GFP_KERNEL);
	if (!ap_buffer) {
		pr_err("%s, kzalloc failed.\n", __func__);
		return -ENOMEM;
	}

	/* read cc data */
	ret = explorer_hal_sync_read(epd, cc_addr, ap_buffer, comm_data->data_len);
	if (ret < 0) {
		pr_err("%s, explorer_hal_sync_read failed.\n", __func__);
		goto freemem;
	}

	/* this is a reply cmd */
	if (comm_data->cmd_is_reply == HAL_CMD_REPLY) {
		/* sync cmd, wake up wait queue. */
		if (comm_data->cmd_is_sync == HAL_CMD_SYNC) {
			/* copy reply buffer */
			memcpy(epd->shared_buffer, ap_buffer, size);
			epd->sync_reply = epd->shared_buffer;
			epd->sync_reply_size = size;
			devm_kfree(dev, ap_buffer);
			atomic_set(&epd->mbox_rindex[comm_data->cmd_mod_id], comm_data->cmd_sync_idx);
			pr_info("%s, done.\n", __func__);
			/* wake up wait queue */
			wake_up(&comm_sync_wq);
			return ret;
		} else {
			/* send cmd, netlink to userspace */
			ret = explorer_genl_mcast_data(epd, HAL_CMD_SLT, ap_buffer, size);
			if (ret) {
				pr_err("%s, send cmd, netlink to userspace failed.\n", __func__);
				goto freemem;
			}

		}
	} else {
		/* send cmd, netlink to userspace */
		ret = explorer_genl_mcast_data(epd, HAL_CMD_SLT, ap_buffer, size);
		if (ret) {
			pr_err("%s, send cmd, netlink to userspace failed.\n", __func__);
			goto freemem;
		}
	}

	pr_info("%s, done.\n", __func__);
freemem:
	devm_kfree(dev, ap_buffer);
	return ret;
}

int explorer_slt_parse_option(char *slt_option, char slt_tokens[][SLT_TOKEN_LEN_MAX])

{
	char option_buffer[SLT_OPTION_LEN_MAX];
	char *token0 = NULL;
	char *token1 = option_buffer;
	unsigned char token_count = 0;

	if (!slt_option) {
		pr_err("%s, option is null.\n", __func__);
		return -EINVAL;
	}

	/* extract each token from options */
	strcpy(option_buffer, slt_option);
	do {
		token0 = strsep(&token1, ",");
		if (token0 == NULL) {
			pr_err("%s, invalid option.\n", __func__);
			return -EINVAL;
		}
		if (++token_count>SLT_TOKEN_NUM_MAX) {
			pr_err("%s, token number exceeds limit.\n", __func__);
			return -EINVAL;
		}
		if (strlen(token0)>SLT_TOKEN_LEN_MAX) {
			pr_err("%s, token length exceeds limit.\n", __func__);
			return -EINVAL;
		}
		strcpy(slt_tokens[token_count-1], token0);
	} while (token1);

	return token_count;
}

int explorer_slt_parse_token(char *slt_token, struct slt_token *param)
{
	char token_buffer[SLT_TOKEN_LEN_MAX];
	char *key = NULL;
	char *value = token_buffer;

	if (!slt_token || !param) {
		pr_err("%s, argument is invalid.\n", __func__);
		return -EINVAL;
	}

	strcpy(token_buffer, slt_token);

	/* extract key & value from token */
	key = strsep(&value, "=");
	if (key == NULL) {
		pr_err("%s, invalid token.\n", __func__);
		return -EINVAL;
	} else if (strlen(key)>SLT_TOKEN_LEN_MAX) {
		pr_err("%s, key length exceeds limit.\n", __func__);
		return -EINVAL;
	}
	strcpy(param->name, key);
	if (value == NULL) {
		strcpy(param->value, "NULL");
		return 0;
	} else if (strlen(value)>SLT_TOKEN_LEN_MAX) {
		pr_err("%s, value length exceeds limit.\n", __func__);
		return -EINVAL;
	}
	strcpy(param->value, value);

	return 0;
}

int explorer_slt_dispatch(struct explorer_plat_data *epd, char *slt_command,
			    enum slt_case_type case_type, size_t count)
{
	int ret = 0;
	char *case_name;
	char slt_cmd_buff[SLT_CMD_LEN_MAX];
	char *option = slt_cmd_buff;
	u32 cnl = 0;
	u32 ipc_id = 0;

	if (!slt_command) {
		pr_err("%s, argument is invalid.\n", __func__);
		return -EINVAL;
	}
	if (count >= SLT_CMD_LEN_MAX) {
		pr_err("%s, the length of slt_command is too long\n", __func__);
		return -EINVAL;
	}
	strncpy(slt_cmd_buff, slt_command, count);
	slt_cmd_buff[count] = '\0';

	/* extract case name and option from original string */
	case_name = strsep(&option, ";");
	cnl = strlen(case_name);
	if(cnl<SLT_CASE_NAME_LEN_MIN) {
		pr_err("%s, case name is invalid.\n", __func__);
		return -EINVAL;
	}
	//pr_info("%s, case_name: %s\n", __func__, case_name);
	if (!option)
		pr_info("%s, option is null,case type: %d,case_name: %s\n", __func__,case_type, case_name);
	else
		pr_info("%s, option: %s,case type: %d,case_name: %s\n", __func__, option,case_type, case_name);

	/* dispatch cases */
	switch (case_type) {
		case CASE_IN_AP:
		{
			u32 i = 0;
			u32 count = 0;
			//pr_info("%s, do test case in AP.\n", __func__);
			for (i = 0; i < SLT_LOCAL_CASE_NUM; i++) {
				if (strcmp(case_name, slt_case[i].case_name) != 0) {
					count++;
					continue;
				}
				/* run case */
				if (slt_case[i].run_case == NULL) {
					pr_err("%s, run_case[%d] is null.\n", __func__, i);
					return -EINVAL;
				}
				ret = slt_case[i].run_case(epd, option);
				if (ret<0) {
					pr_err("%s, run %s failed.\n", __func__, slt_case[i].case_name);
					return ret;
				}
				break;
			}
			if (count == SLT_LOCAL_CASE_NUM) {
				pr_err("%s, no case found.\n", __func__);
				return -EINVAL;
			}
			break;
		}
		case CASE_IN_PBL:
			//pr_info("%s, do test case in PBL.\n", __func__);
			ipc_id = CON_IPC_ID(HAL_CMD_PBL, PBL_SUB_CMD_SLT);
			ret = explorer_write_data_nowait(epd, ipc_id, SLT_BUFFER_BASE,
						      (void *)slt_command, count);
			if (ret<0)
				pr_err("%s, write PBL_SUB_CMD_SLT failed.\n", __func__);
			break;
		case CASE_IN_RTOS:
			//pr_info("%s, do test case in RTOS.\n", __func__);
			ret = explorer_write_ipc_data_nowait(epd, HAL_CMD_SLT, (void *)slt_command,
							   count);
			if (ret<0)
				pr_err("%s, write HAL_CMD_SLT failed.\n", __func__);
			break;
		default:
			//pr_info("%s, unknown case type: %d.\n", __func__, case_type);
			break;
	}

	pr_info("%s, done.\n", __func__);

	return ret;
}

void explorer_slt_localcase_init(int index, const char *name, int (*run_test_case)(struct explorer_plat_data *epd, char *option))
{
	memset(slt_case[index].case_name, '\0', SLT_CASE_NAME_LEN_MAX);
	strcpy(slt_case[index].case_name, name);
	slt_case[index].run_case = run_test_case;

	pr_info("%s, done.\n", __func__);

	return;
}

