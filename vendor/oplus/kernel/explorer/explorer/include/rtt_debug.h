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

#ifndef _RTT_DEBUG_H
#define _RTT_DEBUG_H

#include "main.h"
#include "uapi/explorer_uapi.h"

#define GDB_CMD_MAX_LEN             (200)
#define LOGDUMP_REPLY_ACTION        "EXPLORER_LOGDUMP_MESSAGE=REPLY"

struct explorer_test_gdb_info {
    unsigned int cmd_len;
    char cmd_buffer[GDB_CMD_MAX_LEN];
    struct explorer_sync_reply sync_reply;
};

int explorer_test_ioctl(struct explorer_plat_data *epd, int nr,
            unsigned int param_size, unsigned long arg);
int explorer_proc_logdump_msg(struct explorer_plat_data *epd,
            struct hal_comm_data *comm_data);
int explorer_proc_log_addr_msg(struct explorer_plat_data *epd,
            struct hal_comm_data *comm_data);

#endif /* _RTT_TEST_H */
