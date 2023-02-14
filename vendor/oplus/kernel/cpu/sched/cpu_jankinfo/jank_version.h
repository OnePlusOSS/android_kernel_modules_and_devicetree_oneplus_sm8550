/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2022 Oplus. All rights reserved.
 */

#ifndef __OPLUS_CPU_JANK_VERSION_H__
#define __OPLUS_CPU_JANK_VERSION_H__

#include "jank_base.h"

#define JANK_VERSION			"v3.7@20220420"

struct proc_dir_entry *jank_version_proc_init(
			struct proc_dir_entry *pde);
void jank_version_proc_deinit(
			struct proc_dir_entry *pde);

#endif  /* endif */

