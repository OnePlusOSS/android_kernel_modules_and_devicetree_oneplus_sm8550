/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IRIS_LIGHTUP_H_
#define _DSI_IRIS_LIGHTUP_H_

#include <linux/completion.h>
#include <linux/err.h>
#include <linux/clk.h>
#include "dsi_pwr.h"
#include "dsi_iris.h"

#define IRIS_CHIP_CNT   2
#define IRIS_SYSFS_TOP_DIR   "iris"

#define IRIS_EXT_CLK // use for external gpio clk

/* iris ip option, it will create according to opt_id.
 *  link_state will be create according to the last cmds
 */
struct iris_ip_opt {
	uint8_t opt_id; /*option identifier*/
	uint32_t cmd_cnt; /*option length*/
	uint8_t link_state; /*high speed or low power*/
	struct dsi_cmd_desc *cmd; /*the first cmd of desc*/
};

/*ip search index*/
struct iris_ip_index {
	int32_t opt_cnt; /*ip option number*/
	struct iris_ip_opt *opt; /*option array*/
};

struct iris_pq_ipopt_val {
	int32_t opt_cnt;
	uint8_t ip;
	uint8_t *popt;
};

struct iris_pq_init_val {
	int32_t ip_cnt;
	struct iris_pq_ipopt_val *val;
};

/*used to control iris_ctrl opt sequence*/
struct iris_ctrl_opt {
	uint8_t ip;
	uint8_t opt_id;
	uint8_t chain;
};

struct iris_ctrl_seq {
	int32_t cnt;
	struct iris_ctrl_opt *ctrl_opt;
};

//will pack all the commands here
struct iris_out_cmds {
	/* will be used before cmds sent out */
	struct dsi_cmd_desc *iris_cmds_buf;
	u32 cmds_index;
};

struct iris_pq_update_cmd {
	struct iris_update_ipopt *update_ipopt_array;
	u32 array_index;
};

struct iris_i2c_cfg {
	uint8_t *buf;
	uint32_t buf_index;
};

typedef int (*iris_i2c_read_cb)(u32 reg_addr, u32 *reg_val);
typedef int (*iris_i2c_write_cb)(u32 reg_addr, u32 reg_val);
typedef int (*iris_i2c_burst_write_cb)(u32 start_addr, u32 *lut_buffer, u16 reg_num);

enum IRIS_PARAM_VALID {
	PARAM_NONE = 0,
	PARAM_EMPTY,
	PARAM_PARSED,
	PARAM_PREPARED,
};

/* iris lightup configure commands */
struct iris_cfg {
	struct dsi_display *display;
	struct dsi_panel *panel;

	struct platform_device *pdev;
	struct {
		struct pinctrl *pinctrl;
		struct pinctrl_state *active;
		struct pinctrl_state *suspend;
	} pinctrl;
	int iris_reset_gpio;
	int iris_wakeup_gpio;
	int iris_abyp_ready_gpio;
	int iris_osd_gpio;
	int iris_vdd_gpio;
	int iris_vdd_0p9v_gpio; /* add for oneplus salami */

	/* hardware version and initialization status */
	uint8_t chip_id;
	uint32_t chip_ver;
	uint32_t chip_value[2];
	uint8_t valid; /* 0: none, 1: empty, 2: parse ok, 3: minimum light up, 4. full light up */
	uint32_t platform_type; /* 0: FPGA, 1~: ASIC */
	uint32_t cmd_param_from_fw;
	bool mcu_code_downloaded;
	bool switch_bl_endian;

	/* static configuration */
	uint8_t panel_type;
	uint8_t lut_mode;
	uint32_t split_pkt_size;
	uint32_t min_color_temp;
	uint32_t max_color_temp;
	uint8_t rx_mode; /* 0: DSI_VIDEO_MODE, 1: DSI_CMD_MODE */
	uint8_t tx_mode;
	uint8_t read_path; /* 0: DSI, 1: I2C */

	/* current state */
	struct iris_lp_ctrl lp_ctrl;
	struct iris_abyp_ctrl abyp_ctrl;
	uint16_t panel_nits;
	uint32_t panel_dimming_brightness;
	uint8_t panel_hbm[2];
	struct iris_frc_setting frc_setting;
	int pwil_mode;
	struct iris_vc_ctrl vc_ctrl;

	uint32_t panel_te;
	uint32_t ap_te;
	uint8_t power_mode;
	bool n2m_enable;
	u8 n2m_ratio;
	u32 dtg_ctrl_pt;

	/* secondary display related */
	struct dsi_display *display2;	// secondary display
	struct dsi_panel *panel2;	// secondary panel
	bool iris_osd_autorefresh_enabled;
	bool mipi2_pwr_st;	// secondary mipi power status
	atomic_t osd_irq_cnt;
	atomic_t video_update_wo_osd;

	char display_mode_name[16];
	uint32_t app_version;
	uint32_t app_version1;
	uint8_t app_date[4];
	uint8_t abyp_prev_mode;
	struct clk *ext_clk;

	int32_t panel_pending;
	int32_t panel_delay;
	int32_t panel_level;

	bool aod;
	bool fod;
	bool fod_pending;
	atomic_t fod_cnt;

	struct dsi_regulator_info iris_power_info; // iris pmic power

	/* configuration commands, parsed from dt, dynamically modified
	 * panel->panel_lock must be locked before access and for DSI command send
	 */
	uint32_t lut_cmds_cnt;
	uint32_t dtsi_cmds_cnt;
	uint32_t ip_opt_cnt;
	struct iris_ip_index ip_index_arr[IRIS_PIP_IDX_CNT][IRIS_IP_CNT];
	struct iris_ctrl_seq ctrl_seq[IRIS_CHIP_CNT];
	struct iris_ctrl_seq ctrl_seq_cs[IRIS_CHIP_CNT];
	struct iris_pq_init_val pq_init_val;
	struct iris_out_cmds iris_cmds;
	struct iris_pq_update_cmd pq_update_cmd;

	/* one wire gpio lock */
	spinlock_t iris_1w_lock;
	struct dentry *dbg_root;
	struct kobject *iris_kobj;
	struct work_struct lut_update_work;
	struct work_struct vfr_update_work;
	struct completion frame_ready_completion;

	/* hook for i2c extension */
	struct mutex gs_mutex;
	struct mutex ioctl_mutex;
	iris_i2c_read_cb iris_i2c_read;
	iris_i2c_write_cb iris_i2c_write;
	iris_i2c_burst_write_cb iris_i2c_burst_write;
	struct iris_i2c_cfg iris_i2c_cfg;

	uint32_t metadata;

	/* iris status */
	bool iris_mipi1_power_st;
	bool ap_mipi1_power_st;
	bool iris_pwil_blend_st;
	bool iris_mipi1_power_on_pending;
	bool iris_osd_overflow_st;
	bool iris_frc_vfr_st;
	u32 iris_pwil_mode_state;
	bool dual_enabled;
	bool frc_enabled;
	bool proFPGA_detected;

	struct iris_switch_dump switch_dump;
	struct iris_mspwil_setting chip_mspwil_setting;

	/* memc info */
	struct iris_memc_info memc_info;
	int osd_label;
	int frc_label;
	int frc_demo_window;
	int osd_protect_mode;

	/* emv info */
	struct extmv_frc_meta emv_info;

	/* emv perf info */
	struct extmv_clockinout emv_perf;

	/* pt_sr info*/
	bool pt_sr_enable;
	int pt_sr_hsize;
	int pt_sr_vsize;
	int pt_sr_guided_level;
	int pt_sr_dejaggy_level;
	int pt_sr_peaking_level;
	int pt_sr_DLTI_level;
	/* frc_pq info*/
	int frc_pq_guided_level;
	int frc_pq_dejaggy_level;
	int frc_pq_peaking_level;
	int frc_pq_DLTI_level;
	/* frcgame_pq info*/
	int frcgame_pq_guided_level;
	int frcgame_pq_dejaggy_level;
	int frcgame_pq_peaking_level;
	int frcgame_pq_DLTI_level;
	/*dsi send mode select*/
	uint8_t dsi_trans_mode[2];
	uint8_t *dsi_trans_buf;
	/*lightup pqupdate*/
	uint32_t dsi_trans_len[3][2];
	uint32_t ovs_delay;
	uint32_t ovs_delay_frc;
	uint32_t vsw_vbp_delay;
	bool dtg_eco_enabled;
	u8 ocp_read_by_i2c;
	int aux_width_in_using;
	int aux_height_in_using;
	/* calibration golden fw name */
	const char *ccf1_name;
	const char *ccf2_name;
	const char *ccf3_name;
#ifdef IRIS_EXT_CLK
	bool clk_enable_flag;
#endif
};

struct iris_data {
	const uint8_t *buf;
	uint32_t size;
};

struct iris_cfg *iris_get_cfg(void);

#ifdef IRIS_EXT_CLK
void iris_clk_enable(struct dsi_panel *panel);
void iris_clk_disable(struct dsi_panel *panel);
#endif

int iris_lightup(struct dsi_panel *panel);
int iris_lightoff(struct dsi_panel *panel, bool dead, struct dsi_panel_cmd_set *off_cmds);
int32_t iris_send_ipopt_cmds(int32_t ip, int32_t opt_id);
void iris_update_pq_opt(uint8_t path, bool bcommit);
void iris_update_bitmask_regval_nonread(
		struct iris_update_regval *pregval, bool is_commit);

void iris_alloc_seq_space(void);

void iris_init_update_ipopt(struct iris_update_ipopt *popt,
		uint8_t ip, uint8_t opt_old, uint8_t opt_new, uint8_t chain);
struct iris_pq_ipopt_val  *iris_get_cur_ipopt_val(uint8_t ip);

int iris_init_update_ipopt_t(uint8_t ip, uint8_t opt_old, uint8_t opt_new, uint8_t chain);

/*
 * @description  get assigned position data of ip opt
 * @param ip       ip sign
 * @param opt_id   option id of ip
 * @param pos      the position of option payload
 * @return   fail NULL/success payload data of position
 */
uint32_t  *iris_get_ipopt_payload_data(uint8_t ip, uint8_t opt_id, int32_t pos);
uint32_t iris_get_ipopt_payload_len(uint8_t ip, uint8_t opt_id, int32_t pos);
void iris_set_ipopt_payload_data(uint8_t ip, uint8_t opt_id, int32_t pos, uint32_t value);
/*
 *@Description: get current continue splash stage
 first light up panel only
 second pq effect
 */
uint8_t iris_get_cont_splash_type(void);

/*
 *@Description: print continuous splash commands for bootloader
 *@param: pcmd: cmds array  cnt: cmds could
 */
void iris_print_desc_cmds(struct dsi_cmd_desc *pcmd, int cmd_cnt, int state);

int iris_init_cmds(void);
void iris_get_cmds(struct dsi_panel_cmd_set *cmds, char **ls_arr);
void iris_get_lightoff_cmds(struct dsi_panel_cmd_set *cmds, char **ls_arr);

int32_t iris_attach_cmd_to_ipidx(const struct iris_data *data,
		int32_t data_cnt, struct iris_ip_index *pip_index);

struct iris_ip_index *iris_get_ip_idx(int32_t type);

void iris_change_type_addr(struct iris_ip_opt *dest, struct iris_ip_opt *src);

struct iris_ip_opt *iris_find_specific_ip_opt(uint8_t ip, uint8_t opt_id, int32_t type);
struct iris_ip_opt *iris_find_ip_opt(uint8_t ip, uint8_t opt_id);

struct dsi_cmd_desc *iris_get_specific_desc_from_ipopt(uint8_t ip,
		uint8_t opt_id, int32_t pos, uint32_t type);

int iris_wait_vsync(void);
int iris_set_pending_panel_brightness(int32_t pending, int32_t delay, int32_t level);

bool iris_virtual_display(const struct dsi_display *display);
void iris_free_ipopt_buf(uint32_t ip_type);
void iris_free_seq_space(void);

void iris_send_assembled_pkt(struct iris_ctrl_opt *arr, int seq_cnt);
int32_t iris_parse_dtsi_cmd(const struct device_node *lightup_node,
		uint32_t cmd_index);
int32_t iris_parse_optional_seq(struct device_node *np, const uint8_t *key,
		struct iris_ctrl_seq *pseq);

int iris_display_cmd_engine_enable(struct dsi_display *display);
int iris_display_cmd_engine_disable(struct dsi_display *display);
void iris_insert_delay_us(uint32_t payload_size, uint32_t cmd_num);
int iris_driver_register(void);
void iris_driver_unregister(void);
int iris_parse_cmd_param(struct device_node *lightup_node);

#endif // _DSI_IRIS_LIGHTUP_H_
