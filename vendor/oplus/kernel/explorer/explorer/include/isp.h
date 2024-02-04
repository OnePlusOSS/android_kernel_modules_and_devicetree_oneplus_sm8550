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

#ifndef _EXPLORER_ISP_H
#define _EXPLORER_ISP_H

#define	ISP_TUNNING_DATA_SIZE			(150*PAGE_SIZE)

/* isp uevent */
#define	MIPISOF_IRQ_ACTION			"EXPLORER_ISP_MESSAGE=MIPI_SOF"
#define	ISP0_IRQ_ACTION			"EXPLORER_ISP_MESSAGE=ISP0_TO_AP"
#define	ISP1_IRQ_ACTION			"EXPLORER_ISP_MESSAGE=ISP1_TO_AP"
#define	ISP_SEND_DATA_ACTION			"EXPLORER_ISP_MESSAGE=ISP_SEND_DATA"

int explorer_mipisof_irq_handler(struct explorer_plat_data *epd);
int explorer_isp0_irq_handler(struct explorer_plat_data *epd);
int explorer_isp1_irq_handler(struct explorer_plat_data *epd);

#endif /* _EXPLORER_ISP_H */
