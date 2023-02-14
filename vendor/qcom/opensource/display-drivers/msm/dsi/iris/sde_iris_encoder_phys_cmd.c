// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#if defined(CONFIG_ARCH_LAHAINA)
static void _iris_sde_encoder_autorefresh_disable_seq1(
		struct sde_encoder_phys *phys_enc)
{
	struct sde_encoder_phys_cmd *cmd_enc =
				to_sde_encoder_phys_cmd(phys_enc);
	u32 transfer_time_us = 0;
	u32 wr_line_count;
	u32 poll_time;
	u32 total_poll_time = 0;
	int vdisplay = phys_enc->cached_mode.vdisplay;

	sde_encoder_helper_get_transfer_time(phys_enc->parent,
				&transfer_time_us);
	_sde_encoder_phys_cmd_config_autorefresh(phys_enc, 0);

	while (_sde_encoder_phys_cmd_is_ongoing_pptx(phys_enc)) {
		wr_line_count =
			sde_encoder_phys_cmd_get_write_line_count(phys_enc);
		/* minimum 10 lines */
		if (wr_line_count > 0 && (wr_line_count + 5 < vdisplay))
			poll_time = (vdisplay - wr_line_count + 5)
					* transfer_time_us / vdisplay;
		else
			poll_time = 10 * transfer_time_us / vdisplay;
		total_poll_time += poll_time;
		if (total_poll_time > (KICKOFF_TIMEOUT_MS * USEC_PER_MSEC)) {
			SDE_ERROR_CMDENC(cmd_enc,
					"disable autorefresh failed\n");

			phys_enc->enable_state = SDE_ENC_ERR_NEEDS_HW_RESET;
			break;
		}
		usleep_range(poll_time, poll_time + 1);
	}
}
#elif defined(CONFIG_ARCH_WAIPIO) || defined(CONFIG_ARCH_KALAMA)
static void _iris_sde_encoder_autorefresh_disable_seq1(
		struct sde_encoder_phys *phys_enc)
{
	struct sde_encoder_phys_cmd *cmd_enc =
				to_sde_encoder_phys_cmd(phys_enc);
	u32 transfer_time_us = 0;
	u32 wr_line_count;
	u32 poll_time;
	u32 total_poll_time = 0;
	int vdisplay = phys_enc->cached_mode.vdisplay;

	sde_encoder_get_transfer_time(phys_enc->parent,
				&transfer_time_us);
	_sde_encoder_phys_cmd_config_autorefresh(phys_enc, 0);

	while (_sde_encoder_phys_cmd_is_ongoing_pptx(phys_enc)) {
		wr_line_count =
			sde_encoder_phys_cmd_te_get_line_count(phys_enc);
		/* minimum 10 lines */
		if (wr_line_count > 0 && (wr_line_count + 5 < vdisplay))
			poll_time = (vdisplay - wr_line_count + 5)
					* transfer_time_us / vdisplay;
		else
			poll_time = 10 * transfer_time_us / vdisplay;
		total_poll_time += poll_time;
		if (total_poll_time > (DEFAULT_KICKOFF_TIMEOUT_MS * USEC_PER_MSEC)) {
			SDE_ERROR_CMDENC(cmd_enc,
					"disable autorefresh failed\n");

			phys_enc->enable_state = SDE_ENC_ERR_NEEDS_HW_RESET;
			break;
		}
		usleep_range(poll_time, poll_time + 1);
	}
}
#endif
