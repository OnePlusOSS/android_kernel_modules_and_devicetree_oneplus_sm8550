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
 * Basecode Created :        2021/03/19 Author: mayingbin@zeku.com
 *
 */

#include <linux/uaccess.h>
#include "include/rtt_debug.h"


static u32 log_addr0, log_addr1;

int explorer_test_ioctl(struct explorer_plat_data *epd, int nr,
            unsigned int param_size, unsigned long arg)
{
	int ret = 0;

	pr_info("%s, nr = %d.\n", __func__, nr);
	pr_info("%s, param size = %d.\n", __func__, param_size);

	/* send cmd to explorer */
	switch (nr) {
		case IOC_NR_GDB:
		{
			void *user_data = NULL;
			struct explorer_test_gdb_info *sync_gdb_info = NULL;
			if (param_size > EXPLORER_IOC_LEN_MAX) {
				pr_err("%s, ioctl param size exceeds max length.\n", __func__);
				return -EINVAL;
			}
			sync_gdb_info = (struct explorer_test_gdb_info *)kzalloc(sizeof(struct explorer_test_gdb_info),
			                                                        GFP_KERNEL);
			if (!sync_gdb_info) {
				pr_err("%s, alloc memory failed.\n", __func__);
				return -ENOMEM;
			}
			ret = copy_from_user((void *)sync_gdb_info, (char __user *)arg, sizeof(struct explorer_test_gdb_info));
			if (ret) {
				pr_err("%s, can not copy explorer_data from user, ret = 0x%x.\n",
                                    __func__, ret);
				goto gdb_test_out;
			}
			user_data = (void *)sync_gdb_info->cmd_buffer;
			ret = explorer_write_ipc_data_wait(epd, HAL_CMD_GDB, user_data,
                                                strlen(sync_gdb_info->cmd_buffer), &sync_gdb_info->sync_reply);
			if (ret<0) {
				pr_err("%s, explorer_write_ipc_data_wait failed.\n", __func__);
				goto gdb_test_out;
			}
			pr_info("%s, write IOC_NR_GDB done.\n", __func__);
gdb_test_out:
			kfree(sync_gdb_info);
			break;
		}
		default:
			break;
	}

	return ret;
}

int explorer_proc_logdump_msg(struct explorer_plat_data *epd,
            struct hal_comm_data *comm_data)
{
	int ret = 0;
	int systemLogId = 3001;
	u32 mbox_data[2] = {0};

	/* netlink to userspace */
	ret = explorer_genl_mcast_data(epd, systemLogId, &comm_data->data[0], 8);
	if (ret) {
		pr_err("%s, netlink to userspace failed.\n", __func__);
		ret = explorer_reply_mbox(epd, comm_data->cmd_mod_id, comm_data->cmd_sub_id, (void *)mbox_data);
		if (ret < 0) {
			pr_err("%s, explorer_reply_mbox failed.\n", __func__);
			goto out; 
		}
		goto out;
	}

	pr_debug("%s, done.\n", __func__);
out:
	return ret;
}


int explorer_proc_log_addr_msg(struct explorer_plat_data *epd,
            struct hal_comm_data *comm_data)
{
	int ret = 0;
	log_addr0 = comm_data->data[0];
	log_addr1 = comm_data->data[1];
	pr_debug("%s, log_addr0: 0x%x, log_addr1: 0x%x.\n", __func__, log_addr0, log_addr1);
	return ret;
}
