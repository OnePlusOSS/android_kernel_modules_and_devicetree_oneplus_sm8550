/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2022, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _IRIS_CMPT_H_
#define _IRIS_CMPT_H_
struct dsi_cmd_desc;

bool iris_is_read_cmd(struct dsi_cmd_desc *pdesc);

bool iris_is_last_cmd(const struct mipi_dsi_msg  *pmsg);

bool iris_is_curmode_cmd_mode(void);

bool iris_is_curmode_vid_mode(void);

void iris_set_msg_flags(struct dsi_cmd_desc *pdesc, int type);

int iris_switch_cmd_type(int type);

void iris_set_msg_ctrl(struct dsi_cmd_desc *pdesc);
#endif
