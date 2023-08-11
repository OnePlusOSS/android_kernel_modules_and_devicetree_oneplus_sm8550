/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IRIS_LP_H_
#define _DSI_IRIS_LP_H_

/* option IDs */
#define ID_SYS_MPG 0xf0
#define ID_SYS_DPG 0xf1
#define ID_SYS_ULPS	0xf2
#define ID_SYS_ABYP_CTRL 0xf3
#define ID_SYS_TE_SWAP 0x06
#define ID_SYS_MEM_REPAIR 0x07
#define ID_SYS_TE_BYPASS 0x08
#define ID_SYS_DMA_CTRL 0x09
#define ID_SYS_DMA_GEN_CTRL 0x0a

#define ID_SYS_ENTER_ABYP 0x0104
#define ID_SYS_EXIT_ABYP 0x0105

#define ID_RX_ENTER_TTL 0xD0
#define ID_RX_EXIT_TTL 0xD1

#define ID_TX_TE_FLOW_CTRL  0x05
#define ID_TX_BYPASS_CTRL 0x06
#define ID_DTG_TE_SEL 0xF1

#define RETRY_MAX_CNT 3

enum iris_pmu_domain {
	MIPI_PWR = (0x1 << 1),
	MIPI2_PWR = (0x1 << 2),
	PQ_PWR = (0x1 << 3),
	FRC_PWR = (0x1 << 4),
	DSCU_PWR = (0x1 << 5),
	BSRAM_PWR = (0x1 << 6),
	HDR_PWR = (0x1 << 7),
};

enum iris_dma_channel {
	DMA_CH0 = (1 << 0),
	DMA_CH1 = (1 << 1),
	DMA_CH2 = (1 << 2),
	DMA_CH3 = (1 << 3),
	DMA_CH4 = (1 << 4),
	DMA_CH5 = (1 << 5),
	DMA_CH6 = (1 << 6),
	DMA_CH7 = (1 << 7),
	DMA_CH8 = (1 << 8),
	DMA_CH9 = (1 << 9),
	DMA_CH10 = (1 << 10),
	DMA_CH11 = (1 << 11),
	DMA_CH12 = (1 << 12),
	DMA_CH13 = (1 << 13),
	DMA_CH14 = (1 << 14),
	DMA_CH15 = (1 << 15),
};

/* parse low power control info */
int32_t iris_parse_lp_ctrl(struct device_node *np, struct iris_cfg *pcfg);

/* init iris low power*/
void iris_lp_init(void);
void iris_lp_enable_pre(void);
void iris_lp_enable_post(void);

/* dynamic power gating set */
void iris_dynamic_power_set(bool enable, bool chain);

/* dynamic power gating get */
bool iris_dynamic_power_get(void);

void iris_tx_buf_to_vc_set(bool enable);

/* switch low power setting for frc/pt mode
	frc_mode: 0 -- pt mode, 1 -- frc/dual mode
*/
void iris_frc_lp_switch(bool frc_mode, bool chain);

void iris_tx_pb_req_set(bool enable, bool chain);

/* power on & off mipi2 domain */
int iris_pmu_mipi2_set(bool on);

/* power on & off pq domain */
int iris_pmu_pq_set(bool on);

/* power on & off bulksram domain */
int iris_pmu_bsram_set(bool on);

/* power on & off frc domain */
int iris_pmu_frc_set(bool on);

/* power on & off dsc unit domain */
int iris_pmu_dscu_set(bool on);

bool iris_is_pmu_dscu_on(void);

int iris_pmu_hdr_set(bool on, bool chain);

void iris_abyp_mode_set(int mode);

void iris_abyp_mode_get(u32 count, u32 *values);

bool iris_is_sleep_abyp_mode(void);

void iris_sleep_abyp_power_down(void);

/* Switch PT and Bypass mode */
bool iris_abyp_switch_proc(struct dsi_display *display, int mode);

int iris_dbp_switch(bool enter, bool chain);

/* control dma channels trigger
   input: channels -- bit 0, ch0; bit 1, ch1; bit 2, ch2; bit 3, ch3
          source -- trigger source selection
          chain -- send command with chain or not
 */
void iris_dma_gen_ctrl(int channels, int source, bool chain);

/* control dma trigger
   input: channels -- dma channels. Bit [0] - pq, 1 - hdr, 2 - frc, 3 - dsc_unit, 4 - bulk_sram
                     5 - mipi2, 6 - mipi, .. 10 ~ 15: sw channels
          chain -- send command with chain or not
 */
void iris_dma_trig(int channels, bool chain);

void iris_ulps_enable(bool enable, bool chain);

bool iris_ulps_enable_get(void);

int iris_dbgfs_lp_init(struct dsi_display *display);

void iris_lp_setting_off(void);

void iris_linelock_set(bool enable, bool chain);
/* Get Iris lightup opt */
int iris_lightup_opt_get(void);

int iris_exit_abyp(bool one_wired);

/* Iris metadata set */
void iris_set_metadata(bool panel_lock);
void iris_dump_status(void);
void _iris_disable_temp_sensor(void);
void _iris_bulksram_power_domain_proc(void);
void iris_set_two_wire0_enable(void);
void iris_sysfs_status_deinit(void);

#endif // _DSI_IRIS_LP_H_
