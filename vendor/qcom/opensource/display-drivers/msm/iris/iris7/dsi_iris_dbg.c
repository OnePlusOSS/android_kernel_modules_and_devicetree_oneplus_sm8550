// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include "dsi_iris_api.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_lightup_ocp.h"
#include "dsi_iris_pq.h"
#include "dsi_iris_timing_switch.h"
#include "dsi_iris_ioctl.h"
#include "dsi_iris_lut.h"
#include "dsi_iris_log.h"
#include <linux/kobject.h>

int iris_debug_display_mode_get_i7(char *kbuf, int size, bool debug)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_switch_dump *dump = &pcfg->switch_dump;
	int len = 0;

	iris_display_mode_name_update();

	len += snprintf(kbuf, size,
			"%-20s:\t%s\n", "Display mode", pcfg->display_mode_name);
	if (pcfg->pt_sr_enable) {
		len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d x %d\n", "Iris pt_sr res", pcfg->pt_sr_hsize, pcfg->pt_sr_vsize);
	}

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Memc mode", pcfg->memc_info.memc_mode);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Memc app index", pcfg->memc_info.memc_app);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Memc video fps", pcfg->memc_info.video_fps);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Memc panel fps", pcfg->memc_info.panel_fps);

	if (!debug)
		return len;

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Memc vfr enable", pcfg->memc_info.vfr_en);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Memc TNR mode", pcfg->memc_info.tnr_en);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Memc Low Latency", pcfg->memc_info.low_latency_mode);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Memc N2M mode", pcfg->memc_info.n2m_mode);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Memc frc label", pcfg->frc_label);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Memc demo window", pcfg->frc_demo_window);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "AP mipi1 power", pcfg->ap_mipi1_power_st);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "AP mipi1 refresh", pcfg->iris_osd_autorefresh_enabled);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "AP video wo osd", pcfg->video_update_wo_osd);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Iris mipi1 power", pcfg->iris_mipi1_power_st);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Iris pwil mode", pcfg->iris_pwil_mode_state);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Iris blend mode", pcfg->iris_pwil_blend_st);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Iris osd overflow", pcfg->iris_osd_overflow_st);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Iris osd irq count", pcfg->osd_irq_cnt);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Iris frc vfr state", pcfg->iris_frc_vfr_st);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%08x\n", "Iris fw version", pcfg->app_version1);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Iris fw setting",
			(pcfg->frc_setting.mv_hres << 16) | pcfg->frc_setting.mv_vres);

	if (dump->trigger) {
		len += snprintf(kbuf + len, size - len,
				"SWITCH TIMEOUT DUMP\n");

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Sys power ctrl", dump->sw_pwr_ctrl);

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Rx frame count0", dump->rx_frame_cnt0);

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Rx frame count1", dump->rx_frame_cnt1);

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Rx video meta", dump->rx_video_meta);

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Pwil cur meta0", dump->pwil_cur_meta0);

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Pwil status", dump->pwil_status);

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Pwil disp ctrl0", dump->pwil_disp_ctrl0);

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Pwil int", dump->pwil_int);

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Dport int", dump->dport_int);

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Fi debugbus0", dump->fi_debugbus[0]);

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Fi debugbus1", dump->fi_debugbus[1]);

		len += snprintf(kbuf + len, size - len,
				"%-20s:\t%08x\n", "Fi debugbus2", dump->fi_debugbus[2]);
	}

	return len;
}

int iris_debug_pq_info_get_i7(char *kbuf, int size, bool debug)
{
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	int len = 0;

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Sdr2hdr level", pqlt_cur_setting->pq_setting.sdr2hdr);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Ai enable", (pqlt_cur_setting->pq_setting.sdr2hdr > 8) ? 1 : 0);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Ai TM case", pqlt_cur_setting->sdr2hdr_ai_tm);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "Ai LCE case", pqlt_cur_setting->sdr2hdr_ai_lce);

	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%d\n", "EDR enable", (pqlt_cur_setting->pq_setting.sdr2hdr == 8) ? 1 : 0);

	return len;
}
