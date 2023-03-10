// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019, 2021, The Linux Foundation. All rights reserved.
 */

#ifndef _SDE_HW_QDSS_H
#define _SDE_HW_QDSS_H

#include "sde_hw_catalog.h"
#include "sde_hw_mdss.h"
#include "sde_hw_util.h"

struct sde_hw_qdss;

/**
 * struct sde_hw_qdss_ops - interface to the qdss hardware driver functions
 * Assumption is these functions will be called after clocks are enabled
 */
struct sde_hw_qdss_ops {
	/**
	 * enable_qdss_events - enable qdss events
	 * @hw_qdss: Pointer to qdss context
	 */
	void (*enable_qdss_events)(struct sde_hw_qdss *hw_qdss, bool enable);
};

struct sde_hw_qdss {
	struct sde_hw_blk_reg_map hw;

	/* qdss */
	enum sde_qdss idx;
	const struct sde_qdss_cfg *caps;

	/* ops */
	struct sde_hw_qdss_ops ops;
};

/**
 * to_sde_hw_qdss -  convert base hw object to sde_hw_qdss container
 * @hw: Pointer to hardware block register map object
 * return: Pointer to hardware block container
 */
static inline struct sde_hw_qdss *to_sde_hw_qdss(struct sde_hw_blk_reg_map *hw)
{
	return container_of(hw, struct sde_hw_qdss, hw);
}

/**
 * sde_hw_qdss_init - initializes the qdss block for the passed qdss idx
 * @idx:  QDSS index for which driver object is required
 * @addr: Mapped register io address of MDP
 * @m:    Pointer to mdss catalog data
 * Returns: Error code or allocated sde_hw_qdss context
 */
struct sde_hw_blk_reg_map *sde_hw_qdss_init(enum sde_qdss idx,
				void __iomem *addr,
				struct sde_mdss_cfg *m);

/**
 * sde_hw_qdss_destroy - destroys qdss driver context
 *			 should be called to free the context
 * @hw: Pointer to hardware block register map object
 */
void sde_hw_qdss_destroy(struct sde_hw_blk_reg_map *hw);

#endif /*_SDE_HW_QDSS_H */
