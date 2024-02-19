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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include "include/main.h"
#include "include/ipc.h"
#include "include/irq.h"
#include "include/isp.h"
#include "include/power.h"
#include "include/exception.h"

static int explorer_cc_resetdone_irq_handler(struct explorer_plat_data *epd)
{
	int ret = 0;
	u32 mask_int = INTC0_MASK_L_CCPO;

	/* clear mbox irq flag */
	ret = explorer_hal_sync_write(epd, INTC0_MASK_L, &mask_int, 4);
	if (ret<0) {
		pr_err("%s, mask cc power on interrupt flag failed.\n", __func__);
		goto out;
	}

	pr_info("%s, done.\n", __func__);

out:
	return ret;
}

int explorer_register_intc0_irq(struct explorer_plat_data *epd, u32 irq_vector,
								int (*handler)(struct explorer_plat_data *epd))
{
	if ((epd==NULL) || (irq_vector>=INTC0_MAX_IRQ) || (handler==NULL)) {
		pr_err("%s, invalid argument.\n", __func__);
		return -EINVAL;
	}

	epd->explorer_intc0_ivt[irq_vector] = handler;
	return 0;
}

irqreturn_t explorer_intc0_irq_handler(int irq, void *dev_id)
{
	struct explorer_plat_data *epd = (struct explorer_plat_data *)dev_id;
	int ret = 0;
	u32 intc0_isr = 0, i = 0;

	pr_debug("%s, enter.\n", __func__);

	if (epd == NULL) {
		pr_err("%s, epd is null.\n", __func__);
		goto out;
	}

	if (sdio_check_card()) {
		disable_irq_nosync(irq);
		goto out;
	}

#ifdef IPC_SOFT_DEBUG
	intc0_isr = 0x20;
	ret = IRQ_HANDLED;
	msleep(5);
#else
	/* read intc0 interrupt status register */
	ret = explorer_hal_sync_read(epd, INTC0_ISR, &intc0_isr, 4);
	if (ret<0) {
		pr_err("%s, read intc0 isr failed.\n", __func__);
		goto out;
	}
	if (intc0_isr == CSPI_ERROR_DATA) {
		/* occurred in haps, intc0 gpio is pulled up, and trace32 init script is not executed. */
		pr_err("%s, read intc0 isr failed, link not ready.\n", __func__);
		msleep(1);
		goto out;
	}
#endif
	for (i = 0; i < INTC0_MAX_IRQ; i++) {
		if (((intc0_isr>>i)&0x1) && (epd->explorer_intc0_ivt[i]!=0)) {
			epd->explorer_intc0_ivt[i](epd);
		}
	}

	/* make sure interrupt source really cleared!*/

	pr_debug("%s, done.\n", __func__);

out:
	return IRQ_HANDLED;
}

int explorer_init_irq(struct explorer_plat_data *epd)
{
	int ret = 0;
	struct device *dev = &(epd->plat_dev->dev);

	if (epd->bsp_irq_gpio) {
		pr_info("%s, using bsp gpio irq.\n", __func__);
		epd->bsp_irq = gpio_to_irq(epd->bsp_irq_gpio);
	} else {
		pr_err("%s, bsp gpio irq to irq failed.\n", __func__);
		ret = -1;
		goto out;
	}

	if (!epd->bsp_irq) {
		pr_err("%s, no irq specified.\n", __func__);
		ret = -1;
		goto out;
	}

	/* register mbox irq handler */
	ret = explorer_register_intc0_irq(epd, CC2AP_MBOX_IRQ, explorer_mbox_irq_handler);
	if (ret) {
		pr_err("%s, explorer_register_intc0_irq failed.\n", __func__);
		goto out;
	}

	/* register cc reset done irq handler */
	ret = explorer_register_intc0_irq(epd, CC_RESETDONE_IRQ, explorer_cc_resetdone_irq_handler);
	if (ret) {
		pr_err("%s, explorer_register_intc0_irq failed.\n", __func__);
		goto out;
	}

	/* register mipi_sof to ap irq handler */
	ret = explorer_register_intc0_irq(epd, MIPI_SOF_IRQ, explorer_mipisof_irq_handler);
	if (ret) {
		pr_err("%s, explorer_register_intc0_irq failed.\n", __func__);
		goto out;
	}

	/* register isp0 to ap irq handler */
	ret = explorer_register_intc0_irq(epd, ISP0_IRQ, explorer_isp0_irq_handler);
	if (ret) {
		pr_err("%s, explorer_register_intc0_irq failed.\n", __func__);
		goto out;
	}

	/* register isp1 to ap irq handler */
	ret = explorer_register_intc0_irq(epd, ISP1_IRQ, explorer_isp1_irq_handler);
	if (ret) {
		pr_err("%s, explorer_register_intc0_irq failed.\n", __func__);
		goto out;
	}

	/* register bushang irq handler */
	ret = explorer_register_intc0_irq(epd, CC_BUS_MONITOR_IRQ, explorer_bus_hang_irq_handler);
	if (ret) {
		pr_err("%s, explorer_register_intc0_irq  bushang failed.\n", __func__);
		goto out;
	}

	/* register wdt bite irq handler */
	ret = explorer_register_intc0_irq(epd, CC_WDT_BITE_IRQ, explorer_wdt_bite_irq_handler);
	if (ret) {
		pr_err("%s, explorer_register_intc0_irq wdtbite failed.\n", __func__);
		goto out;
	}

	/* register pmu sleep bite irq handler */
	ret = explorer_register_intc0_irq(epd, PMU_SLEEP_IRQ, explorer_process_power_interrupts);
	if (ret) {
		pr_err("%s, explorer_register_intc0_irq PMU_SLEEP_IRQ failed.\n", __func__);
		goto out;
	}

	/* register pmu wakeup bite irq handler */
	ret = explorer_register_intc0_irq(epd, PMU_WAKEUP_IRQ, explorer_process_power_interrupts);
	if (ret) {
		pr_err("%s, explorer_register_intc0_irq PMU_WAKEUP_IRQ failed.\n", __func__);
		goto out;
	}

	ret = devm_request_threaded_irq(dev, epd->bsp_irq, NULL, explorer_intc0_irq_handler,
	                                IRQF_TRIGGER_HIGH | IRQF_ONESHOT, "explorer", epd);
	if (ret) {
		pr_err("%s, failed to request irq: %d.\n", __func__, epd->bsp_irq);
		goto out;
	}

	pr_info("%s, init irq done.\n", __func__);

	return 0;
out:
	return ret;
}

