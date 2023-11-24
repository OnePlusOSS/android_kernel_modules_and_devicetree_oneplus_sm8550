// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2022, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2022.
 */


#include <drm/drm_mipi_dsi.h>
#include <video/mipi_display.h>
#include <dsi_drm.h>

#include "dsi_iris_def.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_cmpt.h"


bool iris_is_read_cmd(struct dsi_cmd_desc *pdesc)
{
	if (!pdesc)
		return false;

	return (pdesc->msg.type == MIPI_DSI_DCS_READ);
}

bool iris_is_last_cmd(const struct mipi_dsi_msg *pmsg)
{
	if (!pmsg)
		return false;

	return (pmsg->flags & MIPI_DSI_MSG_LASTCOMMAND);
}

bool iris_is_curmode_cmd_mode(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 mode = pcfg->panel->cur_mode->panel_mode;

	if (mode == DSI_OP_CMD_MODE)
		return true;

	return false;
}

bool iris_is_curmode_vid_mode(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 mode = pcfg->panel->cur_mode->panel_mode;

	if (mode == DSI_OP_VIDEO_MODE)
		return true;

	return false;
}

void iris_set_msg_flags(struct dsi_cmd_desc *pdesc, int type)
{
	if (!pdesc)
		return;

	switch (type) {
	case LAST_FLAG:
		pdesc->msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;
		break;
	case READ_FLAG:
		pdesc->msg.flags |= MIPI_DSI_MSG_REQ_ACK;
		break;
	case BATCH_FLAG:
		break;
	default:
		break;
	}
}

int iris_switch_cmd_type(int type)
{
	int s_type = type;

	switch (type) {
	case MIPI_DSI_DCS_COMPRESSION_MODE:
		s_type = MIPI_DSI_DCS_SHORT_WRITE;
		break;
	case MIPI_DSI_PPS_LONG_WRITE:
		s_type = MIPI_DSI_DCS_LONG_WRITE;
		break;
	default:
		break;
	}
	return s_type;
}

void iris_set_msg_ctrl(struct dsi_cmd_desc *pdesc)
{
	if (!pdesc)
		return;

	pdesc->msg.ctrl = 0;
}
