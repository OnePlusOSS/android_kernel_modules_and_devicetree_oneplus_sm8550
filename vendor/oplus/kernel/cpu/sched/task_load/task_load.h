/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2021 Oplus. All rights reserved.
 */

#ifndef _TASK_LOAD_H
#define _TASK_LOAD_H

#define task_load_err(fmt, ...) \
		printk(KERN_ERR "[TASK_LOAD_INFO_ERR][%s]"fmt, __func__, ##__VA_ARGS__)

enum {
	CAMERA = 0,
	CAMERASERVER,
	CAMERAPROVIDER,
};

#endif /* _TASK_LOAD_H */

