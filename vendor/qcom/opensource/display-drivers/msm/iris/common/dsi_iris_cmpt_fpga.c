// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2022, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2022.
 */

#include <dsi_ctrl.h>
#include "dsi_iris_api.h"


void iris_fpga_split_set_max_return_size(struct dsi_ctrl *dsi_ctrl, u16 *pdflags)
{
	/* pls do not porting to customer, only used for 480*800 dsc video 60Hz panel*/
	if ((dsi_ctrl->host_config.panel_mode == DSI_OP_VIDEO_MODE) &&
		(dsi_ctrl->host_config.video_timing.h_active == 480) &&
		(dsi_ctrl->host_config.video_timing.v_active == 800))
			*pdflags |= BIT(3);
}

void iris_fpga_adjust_read_cnt(u32 read_offset,
							   u32 rx_byte,
							   u32 read_cnt,
							   int *pcnt)
{
	/*
	 *  Warn: pls do not port to customer, this change just used for
	 *        hx8379a fwvga vinvout panel, since this panel always return
	 *        long response data in esd read: 9c 01 00 13 1c 6a d5
	 */
	if (iris_need_short_read_workaround()) {
		if (read_offset == 0)
			if ((rx_byte == 4) && ((read_cnt == 7) || (read_cnt == 11)))
				*pcnt += (read_cnt - rx_byte) / 4 + (read_cnt - rx_byte) % 4 ? 1 : 0;
	}
}

void iris_fpga_adjust_read_buf(u32 repeated_bytes,
							   u32 read_offset,
							   u32 rx_byte,
							   u32 read_cnt,
							   u8 *rd_buf)
{
	int i = 0;
	/*
	 *  Warn: pls do not port to customer, this change just used for
	 *        hx8379a fwvga vinvout panel, since this panel always return
	 *        long response data in esd read: 9c 01 00 13 1c 6a d5
	 */
	if (iris_need_short_read_workaround()) {
		if (!repeated_bytes) {
			if (read_offset == 0)
				if ((rx_byte == 4) && ((read_cnt == 7) || (read_cnt == 11))) {
					for (i = 0; i < read_cnt; i++)
						rd_buf[i] = rd_buf[i+1];
				}
		}
	}
}
