/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 */

#ifndef __DP_PLL_H
#define __DP_PLL_H

#include <linux/io.h>
#include <linux/clk-provider.h>
#include <linux/of_device.h>
#include "dp_parser.h"
#include "sde_dbg.h"

#define DP_VCO_HSCLK_RATE_1620MHZDIV1000	1620000UL
#define DP_VCO_HSCLK_RATE_2700MHZDIV1000	2700000UL
#define DP_VCO_HSCLK_RATE_5400MHZDIV1000	5400000UL
#define DP_VCO_HSCLK_RATE_8100MHZDIV1000	8100000UL
#define DP_PHY_VCO_DIV				0x0070

#define dp_pll_get_base(x) pll->io.x->io.base

#define dp_pll_read(x, offset) ({ \
		readl_relaxed((dp_pll_get_base(x)) + (offset)); \
})

#define dp_pll_write(x, offset, data) ({ \
		DP_DEBUG(#offset", addr=0x%llx, val=0x%x\n", \
				((u64)(dp_pll_get_base(x)) + (offset)), (data)); \
		SDE_EVT32_VERBOSE((dp_pll_get_base(x)) + (offset), (data)); \
		writel_relaxed((data), (dp_pll_get_base(x)) + (offset)); \
})

enum dp_pll_revision {
	DP_PLL_UNKNOWN,
	DP_PLL_5NM_V1,
	DP_PLL_5NM_V2,
	DP_PLL_4NM_V1,
	DP_PLL_4NM_V1_1,
};

enum hsclk_rate {
	HSCLK_RATE_1620MHZ,
	HSCLK_RATE_2700MHZ,
	HSCLK_RATE_5400MHZ,
	HSCLK_RATE_8100MHZ,
	HSCLK_RATE_MAX,
};

static inline const char *dp_pll_get_revision(enum dp_pll_revision rev)
{
	switch (rev) {
	case DP_PLL_UNKNOWN:	return "DP_PLL_UNKNOWN";
	case DP_PLL_5NM_V1:	return "DP_PLL_5NM_V1";
	case DP_PLL_5NM_V2:	return "DP_PLL_5NM_V2";
	case DP_PLL_4NM_V1:	return "DP_PLL_4NM_V1";
	case DP_PLL_4NM_V1_1:	return "DP_PLL_4NM_V1_1";
	default:		return "???";
	}
}

struct dp_pll_io {
	struct dp_io_data *dp_phy;
	struct dp_io_data *dp_pll;
	struct dp_io_data *dp_ln_tx0;
	struct dp_io_data *dp_ln_tx1;
	struct dp_io_data *gdsc;
};

struct dp_pll_vco_clk {
	struct clk_hw hw;
	void		*priv;
};

struct dp_pll {
	/* target pll revision information */
	u32 revision;
	/* save vco current rate */
	unsigned long vco_rate;
	/*
	 * PLL index if multiple index are available. Eg. in case of
	 * DSI we have 2 plls.
	 */
	uint32_t index;

	bool ssc_en;
	bool bonding_en;

	void *priv;
	struct platform_device *pdev;
	struct dp_parser *parser;
	struct dp_power *power;
	struct dp_aux *aux;
	struct dp_pll_io io;
	struct clk_onecell_data *clk_data;
	u32 dp_core_revision;

	int (*pll_cfg)(struct dp_pll *pll, unsigned long rate);
	int (*pll_prepare)(struct dp_pll *pll);
	int (*pll_unprepare)(struct dp_pll *pll);
};

struct dp_pll_params {
	/* COM PHY settings */
	u32 hsclk_sel;
	u32 integloop_gain0_mode0;
	u32 integloop_gain1_mode0;
	u32 lock_cmp_en;
	/* PHY vco divider */
	u32 phy_vco_div;
	u32 dec_start_mode0;
	u32 div_frac_start1_mode0;
	u32 div_frac_start2_mode0;
	u32 div_frac_start3_mode0;
	u32 lock_cmp1_mode0;
	u32 lock_cmp2_mode0;
	u32 ssc_step_size1_mode0;
	u32 ssc_step_size2_mode0;
	u32 ssc_per1;
	u32 ssc_per2;
	u32 cmp_code1_mode0;
	u32 cmp_code2_mode0;
	u32 pll_ivco;
	u32 bg_timer;
	u32 core_clk_en;
	u32 lane_offset_tx;
	u32 lane_offset_rx;
};

struct dp_pll_db {
	struct dp_pll *pll;
	/* lane and orientation settings */
	u8 lane_cnt;
	u8 orientation;
	u32 rate_idx;
	const struct dp_pll_params *pll_params;
};

static inline struct dp_pll_vco_clk *to_dp_vco_hw(struct clk_hw *hw)
{
	return container_of(hw, struct dp_pll_vco_clk, hw);
}

static inline bool is_gdsc_disabled(struct dp_pll *pll)
{
	return (dp_pll_read(gdsc, 0x0) & BIT(31)) ? false : true;
}

int dp_pll_clock_register_5nm(struct dp_pll *pll);
void dp_pll_clock_unregister_5nm(struct dp_pll *pll);
int dp_pll_clock_register_4nm(struct dp_pll *pll);
void dp_pll_clock_unregister_4nm(struct dp_pll *pll);

struct dp_pll_in {
	struct platform_device *pdev;
	struct dp_aux *aux;
	struct dp_parser *parser;
	u32 dp_core_revision;
};

int dp_pll_clock_register_helper(struct dp_pll *pll, struct dp_pll_vco_clk *clks, int num_clks);
struct dp_pll *dp_pll_get(struct dp_pll_in *in);
void dp_pll_put(struct dp_pll *pll);
#endif /* __DP_PLL_H */
