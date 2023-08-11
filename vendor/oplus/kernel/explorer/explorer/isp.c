/*
 * Copyright (c) 2021 ZEKU Technology(Shanghai) Corp.,Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * Basecode Created :        2020/12/12 Author: zf@zeku.com
 *
 */

#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include "include/main.h"
#include "include/ipc.h"
#include "include/irq.h"
#include "include/isp.h"

int explorer_mipisof_irq_handler(struct explorer_plat_data *epd)
{
	int ret = 0;

	/* TODO: isp0 process */
	ret = explorer_send_uevent(epd, MIPISOF_IRQ_ACTION);
	if (ret)
		pr_err("%s, failed.\n", __func__);
	else
		pr_info("%s, done.\n", __func__);

	return ret;
}

int explorer_isp0_irq_handler(struct explorer_plat_data *epd)
{
	int ret = 0;

	/* TODO: isp0 process */
	ret = explorer_send_uevent(epd, ISP0_IRQ_ACTION);
	if (ret)
		pr_err("%s, failed.\n", __func__);
	else
		pr_info("%s, done.\n", __func__);

	return ret;
}

int explorer_isp1_irq_handler(struct explorer_plat_data *epd)
{
	int ret = 0;

	/* TODO: isp1 process */
	ret = explorer_send_uevent(epd, ISP1_IRQ_ACTION);
	if (ret)
		pr_err("%s, failed.\n", __func__);
	else
		pr_info("%s, done.\n", __func__);

	return ret;
}

