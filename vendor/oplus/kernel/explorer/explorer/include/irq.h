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
 * Basecode Created :        2020/10/25 Author: zf@zeku.com
 *
 */

#ifndef _EXPLORER_IRQ_H
#define _EXPLORER_IRQ_H

/**
  * intc memory mapping
  */
#define	INTC0_BASE            (0x48018000)
#define	INTC0_MASK_L_OFFSET   (0x8)

#define	INTC0_MASK_L          (INTC0_BASE+INTC0_MASK_L_OFFSET)
#define	INTC0_MASK_L_CCPO     (0x1)
#define	INTC0_MASK_L_CCP3     (0x1 << 3)

#define	INTC0_ISR_OFFSET      (0x30)
#define	INTC0_ISR             (INTC0_BASE+INTC0_ISR_OFFSET)
#define CC_RESETDONE_IRQ      (0)
#define	CC_BUS_MONITOR_IRQ    (3)
#define	CC_WDT_BITE_IRQ       (4)
#define	CC2AP_MBOX_IRQ        (5)
#define	PMU_SLEEP_IRQ         (6)
#define	MIPI_SOF_IRQ          (7)
#define	PMU_WAKEUP_IRQ        (9)
#define ISP0_IRQ              (10)
#define ISP1_IRQ              (11)
#define	INTC0_MAX_IRQ         (32)

struct explorer_plat_data;
int explorer_init_irq(struct explorer_plat_data *epd);
int explorer_register_intc0_irq(struct explorer_plat_data *epd, u32 irq_vector,
								int (*handler)(struct explorer_plat_data *epd));
#endif /* _EXPLORER_IPC_H */
