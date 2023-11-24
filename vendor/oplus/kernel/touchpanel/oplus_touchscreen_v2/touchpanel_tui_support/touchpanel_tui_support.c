#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/task_work.h>
#include <linux/rtc.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/of_gpio.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/input/mt.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/iio/consumer.h>
#include <linux/alarmtimer.h>
#include "touchpanel_common.h"
/****************trusted_touch********************/
#ifdef CONFIG_TOUCHPANEL_TRUSTED_TOUCH
#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include "linux/gunyah/gh_msgq.h"
#include "linux/gunyah/gh_rm_drv.h"
#include <linux/sort.h>
#include <linux/pinctrl/qcom-pinctrl.h>
#include "../touchpanel_common.h"
#include "touch_comon_api/touch_comon_api.h"
#include "touchpanel_tui_support/touchpanel_tui_support.h"

static void touchpanel_trusted_touch_abort_handler(struct touchpanel_data *ts, int error);

void touchpanel_trusted_touch_release_all_finger(struct touchpanel_data *ts)
{
	TPD_INFO("%s: enter.\n", __func__);
	tp_touch_btnkey_release(ts->tp_index);
}

void touchpanel_trusted_touch_irq_disable(struct touchpanel_data *ts)
{
	unsigned long irqflags;

	TPD_INFO("%s: enter.\n", __func__);
	spin_lock_irqsave(&ts->irq_lock, irqflags);

	if (!ts->irq_disabled) {
		if (atomic_read(&ts->trusted_touch_transition))
			disable_irq_wake(ts->irq);
		else
			disable_irq_nosync(ts->irq);

		ts->irq_disabled = true;
	}

	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

void touchpanel_trusted_touch_irq_enable(struct touchpanel_data *ts)
{
	unsigned long irqflags = 0;

	TPD_INFO("%s: enter.\n", __func__);
	spin_lock_irqsave(&ts->irq_lock, irqflags);

	if (ts->irq_disabled) {
		if (atomic_read(&ts->trusted_touch_transition))
			enable_irq_wake(ts->irq);
		else
			enable_irq(ts->irq);
		ts->irq_disabled = false;
	}

	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

void touchpanel_trusted_touch_irq_free(struct touchpanel_data *ts)
{
	unsigned long irqflags;

	TPD_INFO("%s: enter.\n", __func__);
	spin_lock_irqsave(&ts->irq_lock, irqflags);

	if (!ts->irq_disabled) {
		if (!atomic_read(&ts->trusted_touch_transition)) {
			devm_free_irq(ts->dev, ts->irq, ts);
		}
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

static struct gh_acl_desc *touchpanel_vm_get_acl(enum gh_vm_names vm_name)
{
	struct gh_acl_desc *acl_desc;
	gh_vmid_t vmid;

	gh_rm_get_vmid(vm_name, &vmid);

	acl_desc = kzalloc(offsetof(struct gh_acl_desc, acl_entries[1]),
			GFP_KERNEL);
	if (!acl_desc)
		return ERR_PTR(ENOMEM);

	acl_desc->n_acl_entries = 1;
	acl_desc->acl_entries[0].vmid = vmid;
	acl_desc->acl_entries[0].perms = GH_RM_ACL_R | GH_RM_ACL_W;

	return acl_desc;
}

static struct gh_sgl_desc *touchpanel_vm_get_sgl(struct trusted_touch_vm_info *vm_info)
{
	struct gh_sgl_desc *sgl_desc;
	int i;

	sgl_desc = kzalloc(offsetof(struct gh_sgl_desc,
			sgl_entries[vm_info->iomem_list_size]), GFP_KERNEL);
	if (!sgl_desc)
		return ERR_PTR(ENOMEM);

	sgl_desc->n_sgl_entries = vm_info->iomem_list_size;

	for (i = 0; i < vm_info->iomem_list_size; i++) {
		sgl_desc->sgl_entries[i].ipa_base = vm_info->iomem_bases[i];
		sgl_desc->sgl_entries[i].size = vm_info->iomem_sizes[i];
	}

	return sgl_desc;
}

static int touchpanel_populate_vm_info_iomem(struct touchpanel_data *ts)
{
	int i, gpio, rc = 0;
	int num_regs, num_sizes, num_gpios, list_size;
	struct resource res;
	struct device_node *np = ts->dev->of_node;
	struct trusted_touch_vm_info *vm_info = ts->vm_info;

	num_regs = of_property_count_u32_elems(np, "touchpanel,trusted-touch-io-bases");
	if (num_regs < 0) {
		TPD_INFO("Invalid number of IO regions specified\n");
		return -EINVAL;
	}

	num_sizes = of_property_count_u32_elems(np, "touchpanel,trusted-touch-io-sizes");
	if (num_sizes < 0) {
		TPD_INFO("Invalid number of IO regions specified\n");
		return -EINVAL;
	}

	if (num_regs != num_sizes) {
		TPD_INFO("IO bases and sizes array lengths mismatch\n");
		return -EINVAL;
	}

	num_gpios = of_gpio_named_count(np, "touchpanel,trusted-touch-vm-gpio-list");
	if (num_gpios < 0) {
		dev_warn(ts->dev, "Ignoring invalid trusted gpio list: %d\n", num_gpios);
		num_gpios = 0;
	}

	list_size = num_regs + num_gpios;
	vm_info->iomem_list_size = list_size;
	TPD_INFO("num_regs = %d, num_gpios = %d\n", num_regs, num_gpios);
	vm_info->iomem_bases = devm_kcalloc(ts->dev, list_size, sizeof(*vm_info->iomem_bases),
			GFP_KERNEL);
	if (!vm_info->iomem_bases)
		return -ENOMEM;

	vm_info->iomem_sizes = devm_kcalloc(ts->dev, list_size, sizeof(*vm_info->iomem_sizes),
			GFP_KERNEL);
	if (!vm_info->iomem_sizes)
		return -ENOMEM;

	for (i = 0; i < num_gpios; ++i) {
		gpio = of_get_named_gpio(np, "touchpanel,trusted-touch-vm-gpio-list", i);
		if (gpio < 0 || !gpio_is_valid(gpio)) {
			TPD_INFO("Invalid gpio %d at position %d\n", gpio, i);
			return gpio;
		}

		if (!msm_gpio_get_pin_address(gpio, &res)) {
			TPD_INFO("Failed to retrieve gpio-%d resource\n", gpio);
			return -ENODATA;
		}

		vm_info->iomem_bases[i] = res.start;
		vm_info->iomem_sizes[i] = resource_size(&res);
	}

	rc = of_property_read_u32_array(np, "touchpanel,trusted-touch-io-bases",
			&vm_info->iomem_bases[i], list_size - i);
	if (rc) {
		TPD_INFO("Failed to read trusted touch io bases:%d\n", rc);
		return rc;
	}

	rc = of_property_read_u32_array(np, "touchpanel,trusted-touch-io-sizes",
			&vm_info->iomem_sizes[i], list_size - i);
	if (rc) {
		TPD_INFO("Failed to read trusted touch io sizes:%d\n", rc);
		return rc;
	}

	for (i = 0;i < vm_info->iomem_list_size; i++) {
		TPD_INFO("[%d]:iomem_bases = 0x%x && iomem_sizes = 0x%x\n",
			i, vm_info->iomem_bases[i], vm_info->iomem_sizes[i]);
	}

	/* parse reset and irq gpio address */
	rc = of_property_read_u32(np, "touchpanel,trusted-touch-reset-io-base", &ts->vm_info->iomem_rst_base);
	if (rc) {
		TPD_INFO("Failed to read trusted touch reset io base:%d\n", rc);
	}

	rc = of_property_read_u32(np, "touchpanel,trusted-touch-reset-io-size", &ts->vm_info->iomem_rst_size);
	if (rc) {
		TPD_INFO("Failed to read trusted touch reset io size:%d\n", rc);
	}

	rc = of_property_read_u32(np, "touchpanel,trusted-touch-reset-io-offset", &ts->vm_info->iomem_rst_offset);
	if (rc) {
		TPD_INFO("Failed to read trusted touch reset io offset:%d\n", rc);
	}

	rc = of_property_read_u32(np, "touchpanel,trusted-touch-irq-io-base", &ts->vm_info->iomem_irq_base);
	if (rc) {
		TPD_INFO("Failed to read trusted touch irq io base:%d\n", rc);
	}

	rc = of_property_read_u32(np, "touchpanel,trusted-touch-irq-io-size", &ts->vm_info->iomem_irq_size);
	if (rc) {
		TPD_INFO("Failed to read trusted touch irq io size:%d\n", rc);
	}

	rc = of_property_read_u32(np, "touchpanel,trusted-touch-irq-io-offset", &ts->vm_info->iomem_irq_offset);
	if (rc) {
		TPD_INFO("Failed to read trusted touch irq io offset:%d\n", rc);
	}

	TPD_INFO("iomem_rst_base = 0x%x,iomem_rst_size = 0x%x,iomem_rst_offset = 0x%x,iomem_irq_base = 0x%x,iomem_irq_size = 0x%x,iomem_irq_offset = 0x%x\n",
			ts->vm_info->iomem_rst_base,
			ts->vm_info->iomem_rst_size,
			ts->vm_info->iomem_rst_offset,
			ts->vm_info->iomem_irq_base,
			ts->vm_info->iomem_irq_size,
			ts->vm_info->iomem_irq_offset);

	return 0;
}

static int touchpanel_populate_vm_info(struct touchpanel_data *ts)
{
	int rc;
	struct trusted_touch_vm_info *vm_info;
	struct device_node *np = ts->dev->of_node;

	vm_info = devm_kzalloc(ts->dev, sizeof(struct trusted_touch_vm_info), GFP_KERNEL);
	if (!vm_info) {
		TPD_INFO("Failed to devm_kzalloc vm_info\n");
		return -ENOMEM;
	}

	ts->vm_info = vm_info;
	vm_info->vm_name = GH_TRUSTED_VM;
	rc = of_property_read_u32(np, "touchpanel,trusted-touch-spi-irq", &vm_info->hw_irq);
	if (rc) {
		TPD_INFO("Failed to read trusted touch SPI irq:%d\n", rc);
		return rc;
	}

	rc = touchpanel_populate_vm_info_iomem(ts);
	if (rc) {
		TPD_INFO("Failed to read trusted touch mmio ranges:%d\n", rc);
		return rc;
	}

	rc = of_property_read_string(np, "touchpanel,trusted-touch-type", &vm_info->trusted_touch_type);
	if (rc) {
		TPD_INFO("%s: No trusted touch type selection made\n", __func__);
		vm_info->mem_tag = GH_MEM_NOTIFIER_TAG_TOUCH_PRIMARY;
		vm_info->irq_label = GH_IRQ_LABEL_TRUSTED_TOUCH_PRIMARY;
		rc = 0;
	} else if (!strcmp(vm_info->trusted_touch_type, "primary")) {
		vm_info->mem_tag = GH_MEM_NOTIFIER_TAG_TOUCH_PRIMARY;
		vm_info->irq_label = GH_IRQ_LABEL_TRUSTED_TOUCH_PRIMARY;
	} else if (!strcmp(vm_info->trusted_touch_type, "secondary")) {
		vm_info->mem_tag = GH_MEM_NOTIFIER_TAG_TOUCH_SECONDARY;
		vm_info->irq_label = GH_IRQ_LABEL_TRUSTED_TOUCH_SECONDARY;
	}

	return 0;
}

static void touchpanel_destroy_vm_info(struct touchpanel_data *ts)
{
	kfree(ts->vm_info->iomem_sizes);
	kfree(ts->vm_info->iomem_bases);
	kfree(ts->vm_info);
}

static void touchpanel_vm_deinit(struct touchpanel_data *ts)
{
	if (ts->vm_info->mem_cookie)
		gh_mem_notifier_unregister(ts->vm_info->mem_cookie);
	touchpanel_destroy_vm_info(ts);
}

static int touchpanel_trusted_touch_get_vm_state(struct touchpanel_data *ts)
{
	return atomic_read(&ts->vm_info->vm_state);
}

static void touchpanel_trusted_touch_set_vm_state(struct touchpanel_data *ts, int state)
{
	atomic_set(&ts->vm_info->vm_state, state);
}

#ifdef CONFIG_ARCH_QTI_VM
static void touchpanel_trusted_touch_event_notify(struct touchpanel_data *ts, int event);
static void touchpanel_trusted_touch_abort_tvm(struct touchpanel_data *ts);

__attribute__((weak)) int request_firmware_select(const struct firmware **firmware_p, const char *name, struct device *device)
{
	TPD_INFO("%s: weak enter.\n", __func__);
	return -1;
}
__attribute__((weak)) int opticalfp_irq_handler(struct fp_underscreen_info *fp_tpinfo)
{
	TPD_INFO("%s: weak enter.\n", __func__);
	return 0;
}

void touchpanel_tvm_i2c_failure_report(struct touchpanel_data *ts)
{
	if(!ts->trusted_touch_support)
		return;
	TPD_INFO("initiating trusted touch abort due to i2c failure\n");
	touchpanel_trusted_touch_abort_handler(ts, TRUSTED_TOUCH_EVENT_I2C_FAILURE);
}

static void touchpanel_trusted_touch_reset_gpio_toggle(struct touchpanel_data *ts)
{
	void __iomem *base;
	struct trusted_touch_vm_info *vm_info;

	vm_info = ts->vm_info;
	base = ioremap(vm_info->iomem_rst_base, vm_info->iomem_rst_size);
	writel_relaxed(0x1, base + vm_info->iomem_rst_offset);
	/* wait until toggle to finish*/
	wmb();
	writel_relaxed(0x0, base + vm_info->iomem_rst_offset);
	/* wait until toggle to finish*/
	wmb();
	iounmap(base);
}

static void touchpanel_trusted_touch_intr_gpio_toggle(struct touchpanel_data *ts, bool enable)
{
	void __iomem *base;
	u32 val;
	struct trusted_touch_vm_info *vm_info;

	vm_info = ts->vm_info;
	base = ioremap(vm_info->iomem_irq_base, vm_info->iomem_irq_size);
	val = readl_relaxed(base + vm_info->iomem_rst_offset);
	if (enable) {
		val |= BIT(0);
		writel_relaxed(val, base + vm_info->iomem_irq_offset);
		/* wait until toggle to finish*/
		wmb();
	} else {
		val &= ~BIT(0);
		writel_relaxed(val, base + vm_info->iomem_irq_offset);
		/* wait until toggle to finish*/
		wmb();
	}
	iounmap(base);
}

static int touchpanel_sgl_cmp(const void *a, const void *b)
{
	struct gh_sgl_entry *left = (struct gh_sgl_entry *)a;
	struct gh_sgl_entry *right = (struct gh_sgl_entry *)b;

	return (left->ipa_base - right->ipa_base);
}

static int touchpanel_vm_compare_sgl_desc(struct gh_sgl_desc *expected,
		struct gh_sgl_desc *received)
{
	int idx;

	if (expected->n_sgl_entries != received->n_sgl_entries)
		return -E2BIG;
	sort(received->sgl_entries, received->n_sgl_entries,
			sizeof(received->sgl_entries[0]), touchpanel_sgl_cmp, NULL);
	sort(expected->sgl_entries, expected->n_sgl_entries,
			sizeof(expected->sgl_entries[0]), touchpanel_sgl_cmp, NULL);

	for (idx = 0; idx < expected->n_sgl_entries; idx++) {
		struct gh_sgl_entry *left = &expected->sgl_entries[idx];
		struct gh_sgl_entry *right = &received->sgl_entries[idx];

		if ((left->ipa_base != right->ipa_base) ||
				(left->size != right->size)) {
			TPD_INFO("sgl mismatch: left_base:%d right base:%d left size:%d right size:%d\n",
					left->ipa_base, right->ipa_base,
					left->size, right->size);
			return -EINVAL;
		}
	}
	return 0;
}

static int touchpanel_vm_handle_vm_hardware(struct touchpanel_data *ts)
{
	int rc = 0;

	tp_debug = 2;
	if (atomic_read(&ts->delayed_vm_probe_pending)) {
		rc = tp_register_irq_func(ts);
		if (rc) {
			TPD_INFO(" Delayed probe failure on VM!\n");
			return rc;
		}
		atomic_set(&ts->delayed_vm_probe_pending, 0);
		return rc;
	}

	touchpanel_trusted_touch_irq_enable(ts);
	touchpanel_trusted_touch_set_vm_state(ts, TVM_INTERRUPT_ENABLED);
	return rc;
}

static void touchpanel_trusted_touch_tvm_vm_mode_enable(struct touchpanel_data *ts)
{
	struct gh_sgl_desc *sgl_desc, *expected_sgl_desc;
	struct gh_acl_desc *acl_desc;
	struct irq_data *irq_data;
	int rc = 0;
	int irq = 0;

	TPD_INFO("start enable tvm mode\n");

	if (touchpanel_trusted_touch_get_vm_state(ts) != TVM_ALL_RESOURCES_LENT_NOTIFIED) {
		TPD_INFO("All lend notifications not received\n");
		touchpanel_trusted_touch_event_notify(ts,
				TRUSTED_TOUCH_EVENT_NOTIFICATIONS_PENDING);
		return;
	}

	acl_desc = touchpanel_vm_get_acl(GH_TRUSTED_VM);
	if (IS_ERR(acl_desc)) {
		TPD_INFO("failed to populated acl data:rc=%d\n",
				PTR_ERR(acl_desc));
		goto accept_fail;
	}

	sgl_desc = gh_rm_mem_accept(ts->vm_info->vm_mem_handle,
			GH_RM_MEM_TYPE_IO,
			GH_RM_TRANS_TYPE_LEND,
			GH_RM_MEM_ACCEPT_VALIDATE_ACL_ATTRS |
			GH_RM_MEM_ACCEPT_VALIDATE_LABEL |
			GH_RM_MEM_ACCEPT_DONE,  TRUSTED_TOUCH_MEM_LABEL,
			acl_desc, NULL, NULL, 0);
	if (IS_ERR_OR_NULL(sgl_desc)) {
		TPD_INFO("failed to do mem accept :rc=%d\n",
				PTR_ERR(sgl_desc));
		goto acl_fail;
	}
	touchpanel_trusted_touch_set_vm_state(ts, TVM_IOMEM_ACCEPTED);

	/* Initiate session on tvm */
	if (ts->bus_type == TP_BUS_I2C)
		rc = pm_runtime_get_sync(ts->client->adapter->dev.parent);
	else
		rc = pm_runtime_get_sync(ts->s_client->master->dev.parent);

	if (rc < 0) {
		TPD_INFO("failed to get sync rc:%d\n", rc);
		goto acl_fail;
	}
	touchpanel_trusted_touch_set_vm_state(ts, TVM_I2C_SESSION_ACQUIRED);

	expected_sgl_desc = touchpanel_vm_get_sgl(ts->vm_info);
	if (touchpanel_vm_compare_sgl_desc(expected_sgl_desc, sgl_desc)) {
		TPD_INFO("IO sg list does not match\n");
		goto sgl_cmp_fail;
	}

	kfree(expected_sgl_desc);
	kfree(acl_desc);

	irq = gh_irq_accept(ts->vm_info->irq_label, -1, ts->irq_tui_flags);
	touchpanel_trusted_touch_intr_gpio_toggle(ts, false);
	if (irq < 0) {
		TPD_INFO("failed to accept irq\n");
		goto accept_fail;
	}
	touchpanel_trusted_touch_set_vm_state(ts, TVM_IRQ_ACCEPTED);

	irq_data = irq_get_irq_data(irq);
	if (!irq_data) {
		TPD_INFO("Invalid irq data for trusted touch\n");
		goto accept_fail;
	}
	if (!irq_data->hwirq) {
		TPD_INFO("Invalid irq in irq data\n");
		goto accept_fail;
	}
	if (irq_data->hwirq != ts->vm_info->hw_irq) {
		TPD_INFO("Invalid irq lent %d %d\n",irq_data->hwirq, ts->vm_info->hw_irq);
		goto accept_fail;
	}

	TPD_INFO("irq:returned from accept:%d\n", irq);
	ts->irq = irq;

	rc = touchpanel_vm_handle_vm_hardware(ts);
	if (rc) {
		TPD_INFO(" Delayed probe failure on VM!\n");
		goto accept_fail;
	}
	atomic_set(&ts->trusted_touch_enabled, 1);
	TPD_INFO("trusted touch enabled\n");
	return;
sgl_cmp_fail:
	kfree(expected_sgl_desc);
acl_fail:
	kfree(acl_desc);
accept_fail:
	touchpanel_trusted_touch_abort_handler(ts,
			TRUSTED_TOUCH_EVENT_ACCEPT_FAILURE);
}

static void touchpanel_vm_irq_on_lend_callback(void *data,
					unsigned long notif_type,
					enum gh_irq_label label)
{
	struct touchpanel_data *trusted_touch_data = data;

	TPD_INFO("received irq lend request for label:%d\n", label);
	if (touchpanel_trusted_touch_get_vm_state(trusted_touch_data) == TVM_IOMEM_LENT_NOTIFIED)
		touchpanel_trusted_touch_set_vm_state(trusted_touch_data, TVM_ALL_RESOURCES_LENT_NOTIFIED);
	else
		touchpanel_trusted_touch_set_vm_state(trusted_touch_data, TVM_IRQ_LENT_NOTIFIED);
}

static void touchpanel_vm_mem_on_lend_handler(enum gh_mem_notifier_tag tag,
		unsigned long notif_type, void *entry_data, void *notif_msg)
{
	struct gh_rm_notif_mem_shared_payload *payload;
	struct trusted_touch_vm_info *vm_info;
	struct touchpanel_data *trusted_touch_data;

	trusted_touch_data = (struct touchpanel_data *)entry_data;
	vm_info = trusted_touch_data->vm_info;
	if (!vm_info) {
		TPD_INFO("Invalid vm_info\n");
		return;
	}

	if (notif_type != GH_RM_NOTIF_MEM_SHARED ||
			tag != vm_info->mem_tag) {
		TPD_INFO("Invalid command passed from rm\n");
		return;
	}

	if (!entry_data || !notif_msg) {
		TPD_INFO("Invalid entry data passed from rm\n");
		return;
	}


	payload = (struct gh_rm_notif_mem_shared_payload  *)notif_msg;
	if (payload->trans_type != GH_RM_TRANS_TYPE_LEND ||
			payload->label != TRUSTED_TOUCH_MEM_LABEL) {
		TPD_INFO("Invalid label or transaction type\n");
		return;
	}

	vm_info->vm_mem_handle = payload->mem_handle;
	TPD_INFO("received mem lend request with handle:%d\n", vm_info->vm_mem_handle);
	if (touchpanel_trusted_touch_get_vm_state(trusted_touch_data) == TVM_IRQ_LENT_NOTIFIED)
		touchpanel_trusted_touch_set_vm_state(trusted_touch_data, TVM_ALL_RESOURCES_LENT_NOTIFIED);
	else
		touchpanel_trusted_touch_set_vm_state(trusted_touch_data, TVM_IOMEM_LENT_NOTIFIED);
}

static int touchpanel_vm_mem_release(struct touchpanel_data *ts)
{
	int rc = 0;

	if (!ts->vm_info->vm_mem_handle) {
		TPD_INFO("Invalid memory handle\n");
		return -EINVAL;
	}

	rc = gh_rm_mem_release(ts->vm_info->vm_mem_handle, 0);
	if (rc)
		TPD_INFO("VM mem release failed: rc=%d\n", rc);

	rc = gh_rm_mem_notify(ts->vm_info->vm_mem_handle,
				GH_RM_MEM_NOTIFY_OWNER_RELEASED,
				ts->vm_info->mem_tag, 0);
	if (rc)
		TPD_INFO("Failed to notify mem release to PVM: rc=%d\n");
	TPD_INFO("vm mem release succeded\n");

	ts->vm_info->vm_mem_handle = 0;
	return rc;
}

static void touchpanel_trusted_touch_tvm_vm_mode_disable(struct touchpanel_data *ts)
{
	int rc = 0;

	if (atomic_read(&ts->trusted_touch_abort_status)) {
		touchpanel_trusted_touch_abort_tvm(ts);
		return;
	}

	touchpanel_trusted_touch_irq_disable(ts);
	touchpanel_trusted_touch_set_vm_state(ts, TVM_INTERRUPT_DISABLED);

	rc = gh_irq_release(ts->vm_info->irq_label);
	if (rc) {
		TPD_INFO("Failed to release irq rc:%d\n", rc);
		goto error;
	} else {
		touchpanel_trusted_touch_set_vm_state(ts, TVM_IRQ_RELEASED);
	}
	rc = gh_irq_release_notify(ts->vm_info->irq_label);
	if (rc)
		TPD_INFO("Failed to notify release irq rc:%d\n", rc);

	TPD_INFO("vm irq release succeded\n");

	touchpanel_trusted_touch_release_all_finger(ts);

	if (ts->bus_type == TP_BUS_I2C)
		pm_runtime_put_sync(ts->client->adapter->dev.parent);
	else
		pm_runtime_put_sync(ts->s_client->master->dev.parent);

	touchpanel_trusted_touch_set_vm_state(ts, TVM_I2C_SESSION_RELEASED);
	rc = touchpanel_vm_mem_release(ts);
	if (rc) {
		TPD_INFO("Failed to release mem rc:%d\n", rc);
		goto error;
	} else {
		touchpanel_trusted_touch_set_vm_state(ts, TVM_IOMEM_RELEASED);
	}
	touchpanel_trusted_touch_set_vm_state(ts, TRUSTED_TOUCH_TVM_INIT);
	atomic_set(&ts->trusted_touch_enabled, 0);
	TPD_INFO("trusted touch disabled\n");
	return;
error:
	touchpanel_trusted_touch_abort_handler(ts, TRUSTED_TOUCH_EVENT_RELEASE_FAILURE);
}

int touchpanel_handle_trusted_touch_tvm(struct touchpanel_data *ts, int value)
{
	int err = 0;

	switch (value) {
	case 0:
		if ((atomic_read(&ts->trusted_touch_enabled) == 0) &&
			(atomic_read(&ts->trusted_touch_abort_status) == 0)) {
			TPD_INFO("Trusted touch is already disabled\n");
			break;
		}
		if (atomic_read(&ts->trusted_touch_mode) == TRUSTED_TOUCH_VM_MODE) {
			touchpanel_trusted_touch_tvm_vm_mode_disable(ts);
		} else {
			TPD_INFO("Unsupported trusted touch mode\n");
		}
		break;

	case 1:
		if (atomic_read(&ts->trusted_touch_enabled)) {
			TPD_INFO("Trusted touch usecase underway\n");
			err = -EBUSY;
			break;
		}
		if (atomic_read(&ts->trusted_touch_mode) == TRUSTED_TOUCH_VM_MODE) {
			touchpanel_trusted_touch_tvm_vm_mode_enable(ts);
		} else {
			TPD_INFO("Unsupported trusted touch mode\n");
		}
		break;

	default:
		TPD_INFO("unsupported value: %lu\n", value);
		err = -EINVAL;
		break;
	}

	return err;
}

static void touchpanel_trusted_touch_abort_tvm(struct touchpanel_data *ts)
{
	int rc = 0;
	int vm_state = touchpanel_trusted_touch_get_vm_state(ts);

	if (vm_state >= TRUSTED_TOUCH_TVM_STATE_MAX) {
		TPD_INFO("invalid tvm driver state: %d\n", vm_state);
		return;
	}

	switch (vm_state) {
	case TVM_INTERRUPT_ENABLED:
		touchpanel_trusted_touch_irq_disable(ts);
	case TVM_IRQ_ACCEPTED:
	case TVM_INTERRUPT_DISABLED:
		rc = gh_irq_release(ts->vm_info->irq_label);
		if (rc)
			TPD_INFO("Failed to release irq rc:%d\n", rc);
		rc = gh_irq_release_notify(ts->vm_info->irq_label);
		if (rc)
			TPD_INFO("Failed to notify irq release rc:%d\n", rc);
	case TVM_I2C_SESSION_ACQUIRED:
	case TVM_IOMEM_ACCEPTED:
	case TVM_IRQ_RELEASED:
		touchpanel_trusted_touch_release_all_finger(ts);
		if (ts->bus_type == TP_BUS_I2C)
			pm_runtime_put_sync(ts->client->adapter->dev.parent);
		else
			pm_runtime_put_sync(ts->s_client->master->dev.parent);
	case TVM_I2C_SESSION_RELEASED:
		rc = touchpanel_vm_mem_release(ts);
		if (rc)
			TPD_INFO("Failed to release mem rc:%d\n", rc);
	case TVM_IOMEM_RELEASED:
	case TVM_ALL_RESOURCES_LENT_NOTIFIED:
	case TRUSTED_TOUCH_TVM_INIT:
	case TVM_IRQ_LENT_NOTIFIED:
	case TVM_IOMEM_LENT_NOTIFIED:
		atomic_set(&ts->trusted_touch_enabled, 0);
	}

	atomic_set(&ts->trusted_touch_abort_status, 0);
	touchpanel_trusted_touch_set_vm_state(ts, TRUSTED_TOUCH_TVM_INIT);
}

#else

static void touchpanel_bus_put(struct touchpanel_data *ts);
static void touchpanel_trusted_touch_abort_pvm(struct touchpanel_data *ts)
{
	int rc = 0;
	int vm_state = touchpanel_trusted_touch_get_vm_state(ts);

	if (vm_state >= TRUSTED_TOUCH_PVM_STATE_MAX) {
		TPD_INFO("Invalid driver state: %d\n", vm_state);
		return;
	}

	switch (vm_state) {
	case PVM_IRQ_RELEASE_NOTIFIED:
	case PVM_ALL_RESOURCES_RELEASE_NOTIFIED:
	case PVM_IRQ_LENT:
	case PVM_IRQ_LENT_NOTIFIED:
		rc = gh_irq_reclaim(ts->vm_info->irq_label);
		if (rc)
			TPD_INFO("failed to reclaim irq on pvm rc:%d\n", rc);
	case PVM_IRQ_RECLAIMED:
	case PVM_IOMEM_LENT:
	case PVM_IOMEM_LENT_NOTIFIED:
	case PVM_IOMEM_RELEASE_NOTIFIED:
		rc = gh_rm_mem_reclaim(ts->vm_info->vm_mem_handle, 0);
		if (rc)
			TPD_INFO("failed to reclaim iomem on pvm rc:%d\n", rc);
		ts->vm_info->vm_mem_handle = 0;
	case PVM_IOMEM_RECLAIMED:
	case PVM_INTERRUPT_DISABLED:
		touchpanel_trusted_touch_irq_enable(ts);
	case PVM_I2C_RESOURCE_ACQUIRED:
	case PVM_INTERRUPT_ENABLED:
		touchpanel_bus_put(ts);
	case TRUSTED_TOUCH_PVM_INIT:
	case PVM_I2C_RESOURCE_RELEASED:
		atomic_set(&ts->trusted_touch_enabled, 0);
		atomic_set(&ts->trusted_touch_transition, 0);
	}

	atomic_set(&ts->trusted_touch_abort_status, 0);

	touchpanel_trusted_touch_set_vm_state(ts, TRUSTED_TOUCH_PVM_INIT);
}

static int touchpanel_clk_prepare_enable(struct touchpanel_data *ts)
{
	int ret;

	ret = clk_prepare_enable(ts->iface_clk);
	if (ret) {
		TPD_INFO("error on clk_prepare_enable(iface_clk):%d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(ts->core_clk);
	if (ret) {
		clk_disable_unprepare(ts->iface_clk);
		TPD_INFO("error clk_prepare_enable(core_clk):%d\n", ret);
	}
	return ret;
}

static void touchpanel_clk_disable_unprepare(struct touchpanel_data *ts)
{
	clk_disable_unprepare(ts->core_clk);
	clk_disable_unprepare(ts->iface_clk);
}

static int touchpanel_bus_get(struct touchpanel_data *ts)
{
	int rc = 0;
	struct device *dev = NULL;
	TP_INFO(ts->tp_index, "%s enter\n", __func__);

	cancel_work_sync(&ts->speed_up_work);
	reinit_completion(&ts->trusted_touch_powerdown);
	TP_INFO(ts->tp_index, "%s reinit_completion\n", __func__);

	if (ts->bus_type == TP_BUS_I2C) {
		dev = ts->client->adapter->dev.parent;
	} else {
		dev = ts->s_client->master->dev.parent;
	}
	TP_INFO(ts->tp_index, "%s dev\n", __func__);

	mutex_lock(&ts->clk_io_ctrl_mutex);

	TP_INFO(ts->tp_index, "%s clk_io_ctrl_mutex\n", __func__);
	rc = pm_runtime_get_sync(dev);

	TP_INFO(ts->tp_index, "%s pm_runtime_get_sync\n", __func__);
	if (rc >= 0 &&  ts->core_clk != NULL &&
				ts->iface_clk != NULL) {
		rc = touchpanel_clk_prepare_enable(ts);
		if (rc)
			pm_runtime_put_sync(dev);
	}

	TP_INFO(ts->tp_index, "%s exit\n", __func__);

	mutex_unlock(&ts->clk_io_ctrl_mutex);
	return rc;
}

static void touchpanel_bus_put(struct touchpanel_data *ts)
{
	struct device *dev = NULL;
	TP_INFO(ts->tp_index, "%s enter\n", __func__);

	if (ts->bus_type == TP_BUS_I2C)
		dev = ts->client->adapter->dev.parent;
	else
		dev = ts->s_client->master->dev.parent;

	mutex_lock(&ts->clk_io_ctrl_mutex);
	if (ts->core_clk != NULL && ts->iface_clk != NULL)
		touchpanel_clk_disable_unprepare(ts);
	pm_runtime_put_sync(dev);
	mutex_unlock(&ts->clk_io_ctrl_mutex);
	complete(&ts->trusted_touch_powerdown);

	TP_INFO(ts->tp_index, "%s exit\n", __func__);
}

static struct gh_notify_vmid_desc *touchpanel_vm_get_vmid(gh_vmid_t vmid)
{
	struct gh_notify_vmid_desc *vmid_desc;

	vmid_desc = kzalloc(offsetof(struct gh_notify_vmid_desc,
				vmid_entries[1]), GFP_KERNEL);
	if (!vmid_desc)
		return ERR_PTR(ENOMEM);

	vmid_desc->n_vmid_entries = 1;
	vmid_desc->vmid_entries[0].vmid = vmid;
	return vmid_desc;
}

static void  touchpanel_trusted_touch_pvm_vm_mode_disable(struct touchpanel_data *ts)
{
	int rc = 0;

	atomic_set(&ts->trusted_touch_transition, 1);

	if (atomic_read(&ts->trusted_touch_abort_status)) {
		touchpanel_trusted_touch_abort_pvm(ts);
		return;
	}

	if (touchpanel_trusted_touch_get_vm_state(ts) != PVM_ALL_RESOURCES_RELEASE_NOTIFIED)
		TPD_INFO("all release notifications are not received yet\n");

	rc = gh_rm_mem_reclaim(ts->vm_info->vm_mem_handle, 0);
	if (rc) {
		TPD_INFO("Trusted touch VM mem reclaim failed rc:%d\n", rc);
		goto error;
	}
	touchpanel_trusted_touch_set_vm_state(ts, PVM_IOMEM_RECLAIMED);
	ts->vm_info->vm_mem_handle = 0;
	TPD_INFO("vm mem reclaim succeded!\n");

	rc = gh_irq_reclaim(ts->vm_info->irq_label);
	if (rc) {
		TPD_INFO("failed to reclaim irq on pvm rc:%d\n", rc);
		goto error;
	}
	touchpanel_trusted_touch_set_vm_state(ts, PVM_IRQ_RECLAIMED);
	TPD_INFO("vm irq reclaim succeded!\n");

	touchpanel_trusted_touch_irq_enable(ts);
	touchpanel_trusted_touch_set_vm_state(ts, PVM_INTERRUPT_ENABLED);
	touchpanel_bus_put(ts);
	atomic_set(&ts->trusted_touch_transition, 0);
	touchpanel_trusted_touch_set_vm_state(ts, PVM_I2C_RESOURCE_RELEASED);
	touchpanel_trusted_touch_set_vm_state(ts, TRUSTED_TOUCH_PVM_INIT);
	atomic_set(&ts->trusted_touch_enabled, 0);
	TPD_INFO("trusted touch disabled\n");
	return;
error:
	touchpanel_trusted_touch_abort_handler(ts,
			TRUSTED_TOUCH_EVENT_RECLAIM_FAILURE);
}

static void touchpanel_vm_irq_on_release_callback(void *data,
					unsigned long notif_type,
					enum gh_irq_label label)
{
	struct touchpanel_data *ts = data;

	if (notif_type != GH_RM_NOTIF_VM_IRQ_RELEASED) {
		TPD_INFO("invalid notification type\n");
		return;
	}

	if (touchpanel_trusted_touch_get_vm_state(ts) == PVM_IOMEM_RELEASE_NOTIFIED)
		touchpanel_trusted_touch_set_vm_state(ts, PVM_ALL_RESOURCES_RELEASE_NOTIFIED);
	else
		touchpanel_trusted_touch_set_vm_state(ts, PVM_IRQ_RELEASE_NOTIFIED);
}

static void touchpanel_vm_mem_on_release_handler(enum gh_mem_notifier_tag tag,
		unsigned long notif_type, void *entry_data, void *notif_msg)
{
	struct gh_rm_notif_mem_released_payload *release_payload;
	struct trusted_touch_vm_info *vm_info;
	struct touchpanel_data *ts;

	ts = (struct touchpanel_data *)entry_data;
	vm_info = ts->vm_info;
	if (!vm_info) {
		TPD_INFO(" Invalid vm_info\n");
		return;
	}

	if (notif_type != GH_RM_NOTIF_MEM_RELEASED) {
		TPD_INFO(" Invalid notification type\n");
		return;
	}

	if (tag != vm_info->mem_tag) {
		TPD_INFO(" Invalid tag\n");
		return;
	}

	if (!entry_data || !notif_msg) {
		TPD_INFO(" Invalid data or notification message\n");
		return;
	}

	release_payload = (struct gh_rm_notif_mem_released_payload  *)notif_msg;
	if (release_payload->mem_handle != vm_info->vm_mem_handle) {
		TPD_INFO("Invalid mem handle detected\n");
		return;
	}

	if (touchpanel_trusted_touch_get_vm_state(ts) == PVM_IRQ_RELEASE_NOTIFIED)
		touchpanel_trusted_touch_set_vm_state(ts, PVM_ALL_RESOURCES_RELEASE_NOTIFIED);
	else
		touchpanel_trusted_touch_set_vm_state(ts, PVM_IOMEM_RELEASE_NOTIFIED);
}

static int touchpanel_vm_mem_lend(struct touchpanel_data *ts)
{
	struct gh_acl_desc *acl_desc;
	struct gh_sgl_desc *sgl_desc;
	struct gh_notify_vmid_desc *vmid_desc;
	gh_memparcel_handle_t mem_handle;
	gh_vmid_t trusted_vmid;
	int rc = 0;

	acl_desc = touchpanel_vm_get_acl(GH_TRUSTED_VM);
	if (IS_ERR(acl_desc)) {
		TPD_INFO("Failed to get acl of IO memories for Trusted touch\n");
		PTR_ERR(acl_desc);
		return -EINVAL;
	}

	sgl_desc = touchpanel_vm_get_sgl(ts->vm_info);
	if (IS_ERR(sgl_desc)) {
		TPD_INFO("Failed to get sgl of IO memories for Trusted touch\n");
		PTR_ERR(sgl_desc);
		rc = -EINVAL;
		goto sgl_error;
	}

	rc = gh_rm_mem_lend(GH_RM_MEM_TYPE_IO, 0, TRUSTED_TOUCH_MEM_LABEL,
			acl_desc, sgl_desc, NULL, &mem_handle);
	if (rc) {
		TPD_INFO("Failed to lend IO memories for Trusted touch rc:%d\n",
							rc);
		goto error;
	}

	TPD_INFO("vm mem lend succeded\n");

	touchpanel_trusted_touch_set_vm_state(ts, PVM_IOMEM_LENT);

	gh_rm_get_vmid(GH_TRUSTED_VM, &trusted_vmid);

	vmid_desc = touchpanel_vm_get_vmid(trusted_vmid);

	rc = gh_rm_mem_notify(mem_handle, GH_RM_MEM_NOTIFY_RECIPIENT_SHARED,
			ts->vm_info->mem_tag, vmid_desc);
	if (rc) {
		TPD_INFO("Failed to notify mem lend to hypervisor rc:%d\n", rc);
		goto vmid_error;
	}

	touchpanel_trusted_touch_set_vm_state(ts, PVM_IOMEM_LENT_NOTIFIED);

	ts->vm_info->vm_mem_handle = mem_handle;
vmid_error:
	kfree(vmid_desc);
error:
	kfree(sgl_desc);
sgl_error:
	kfree(acl_desc);

	return rc;
}

static int touchpanel_trusted_touch_pvm_vm_mode_enable(struct touchpanel_data *ts)
{
	int rc = 0;
	struct trusted_touch_vm_info *vm_info = ts->vm_info;

	atomic_set(&ts->trusted_touch_transition, 1);
	mutex_lock(&ts->transition_lock);

	if (ts->is_suspended) {
		TPD_INFO("Invalid power state for operation\n");
		atomic_set(&ts->trusted_touch_transition, 0);
		rc =  -EPERM;
		goto error;
	}

	/* i2c session start and resource acquire */
	if (touchpanel_bus_get(ts) < 0) {
		TPD_INFO("touchpanel_ts_bus_get failed\n");
		rc = -EIO;
		goto error;
	}

	touchpanel_trusted_touch_set_vm_state(ts, PVM_I2C_RESOURCE_ACQUIRED);
	/* flush pending interurpts from FIFO */
	touchpanel_trusted_touch_irq_disable(ts);
	touchpanel_trusted_touch_set_vm_state(ts, PVM_INTERRUPT_DISABLED);
	touchpanel_trusted_touch_release_all_finger(ts);

	rc = touchpanel_vm_mem_lend(ts);
	if (rc) {
		TPD_INFO("Failed to lend memory\n");
		goto abort_handler;
	}
	TPD_INFO("vm mem lend succeded\n");
	rc = gh_irq_lend_v2(vm_info->irq_label, vm_info->vm_name,
		ts->irq, &touchpanel_vm_irq_on_release_callback, ts);
	if (rc) {
		TPD_INFO("Failed to lend irq\n");
		goto abort_handler;
	}

	TPD_INFO("vm irq lend succeded for irq:%d\n", ts->irq);
	touchpanel_trusted_touch_set_vm_state(ts, PVM_IRQ_LENT);

	rc = gh_irq_lend_notify(vm_info->irq_label);
	if (rc) {
		TPD_INFO("Failed to notify irq\n");
		goto abort_handler;
	}
	touchpanel_trusted_touch_set_vm_state(ts, PVM_IRQ_LENT_NOTIFIED);

	mutex_unlock(&ts->transition_lock);
	atomic_set(&ts->trusted_touch_transition, 0);
	atomic_set(&ts->trusted_touch_enabled, 1);
	TPD_INFO("trusted touch enabled\n");
	return rc;

abort_handler:
	touchpanel_trusted_touch_abort_handler(ts, TRUSTED_TOUCH_EVENT_LEND_FAILURE);

error:
	mutex_unlock(&ts->transition_lock);
	return rc;
}

int touchpanel_handle_trusted_touch_pvm(struct touchpanel_data *ts, int value)
{
	int err = 0;

	switch (value) {
	case 0:
		if (atomic_read(&ts->trusted_touch_enabled) == 0 &&
			(atomic_read(&ts->trusted_touch_abort_status) == 0)) {
			TPD_INFO("Trusted touch is already disabled\n");
			break;
		}
		if (atomic_read(&ts->trusted_touch_mode) ==
				TRUSTED_TOUCH_VM_MODE) {
			 touchpanel_trusted_touch_pvm_vm_mode_disable(ts);
		} else {
			TPD_INFO("Unsupported trusted touch mode\n");
		}
		break;

	case 1:
		if (atomic_read(&ts->trusted_touch_enabled)) {
			TPD_INFO("Trusted touch usecase underway\n");
			err = -EBUSY;
			break;
		}
		if (atomic_read(&ts->trusted_touch_mode) ==
				TRUSTED_TOUCH_VM_MODE) {
			err = touchpanel_trusted_touch_pvm_vm_mode_enable(ts);
		} else {
			TPD_INFO("Unsupported trusted touch mode\n");
		}
		break;

	default:
		TPD_INFO("unsupported value: %lu\n", value);
		err = -EINVAL;
		break;
	}
	return err;
}
#endif

static void touchpanel_trusted_touch_event_notify(struct touchpanel_data *ts, int event)
{
	atomic_set(&ts->trusted_touch_event, event);
	sysfs_notify(&ts->dev->kobj, NULL, "trusted_touch_event");
}

static void touchpanel_trusted_touch_abort_handler(struct touchpanel_data *ts, int error)
{
	atomic_set(&ts->trusted_touch_abort_status, error);
	TPD_INFO("trusted_touch session aborted with failure:%d\n", error);
	touchpanel_trusted_touch_event_notify(ts, error);
#ifdef CONFIG_ARCH_QTI_VM
	TPD_INFO("Resetting touch controller\n");
	if (touchpanel_trusted_touch_get_vm_state(ts) >= TVM_IOMEM_ACCEPTED &&
			error == TRUSTED_TOUCH_EVENT_I2C_FAILURE) {
		TPD_INFO("Resetting touch controller\n");
		touchpanel_trusted_touch_reset_gpio_toggle(ts);
	}
#endif
}

static int touchpanel_vm_init(struct touchpanel_data *ts)
{
	int rc = 0;
	struct trusted_touch_vm_info *vm_info;
	void *mem_cookie;

	rc = touchpanel_populate_vm_info(ts);
	if (rc) {
		TPD_INFO("Cannot setup vm pipeline\n");
		rc = -EINVAL;
		goto fail;
	}

	vm_info = ts->vm_info;
#ifdef CONFIG_ARCH_QTI_VM
	mem_cookie = gh_mem_notifier_register(vm_info->mem_tag,
			touchpanel_vm_mem_on_lend_handler, ts);
	if (!mem_cookie) {
		TPD_INFO("Failed to register on lend mem notifier\n");
		rc = -EINVAL;
		goto init_fail;
	}
	vm_info->mem_cookie = mem_cookie;
	rc = gh_irq_wait_for_lend_v2(vm_info->irq_label, GH_PRIMARY_VM,
			&touchpanel_vm_irq_on_lend_callback, ts);
	touchpanel_trusted_touch_set_vm_state(ts, TRUSTED_TOUCH_TVM_INIT);
#else
	mem_cookie = gh_mem_notifier_register(vm_info->mem_tag,
			touchpanel_vm_mem_on_release_handler, ts);
	if (!mem_cookie) {
		TPD_INFO("Failed to register on release mem notifier\n");
		rc = -EINVAL;
		goto init_fail;
	}
	vm_info->mem_cookie = mem_cookie;
	touchpanel_trusted_touch_set_vm_state(ts, TRUSTED_TOUCH_PVM_INIT);
#endif
	return rc;
init_fail:
	touchpanel_vm_deinit(ts);
fail:
	return rc;
}

static void touchpanel_dt_parse_trusted_touch_info(struct touchpanel_data *ts)
{
	struct device_node *np = ts->dev->of_node;
	int rc = 0;
	const char *selection;
	const char *environment;

#ifdef CONFIG_ARCH_QTI_VM
	ts->touch_environment = "tvm";
#else
	ts->touch_environment = "pvm";
#endif

	rc = of_property_read_string(np, "touchpanel,trusted-touch-mode", &selection);
	if (rc) {
		dev_warn(ts->dev,
			"%s: No trusted touch mode selection made\n", __func__);
		atomic_set(&ts->trusted_touch_mode, TRUSTED_TOUCH_MODE_NONE);
		return;
	}

	if (!strcmp(selection, "vm_mode")) {
		atomic_set(&ts->trusted_touch_mode, TRUSTED_TOUCH_VM_MODE);
		pr_err("Selected trusted touch mode to VM mode\n");
	} else {
		atomic_set(&ts->trusted_touch_mode, TRUSTED_TOUCH_MODE_NONE);
		pr_err("Invalid trusted_touch mode\n");
	}

	rc = of_property_read_string(np, "touchpanel,touch-environment", &environment);
	if (rc) {
		dev_warn(ts->dev, "%s: No trusted touch mode environment\n", __func__);
	}
	ts->touch_environment = environment;
	TPD_INFO("Trusted touch environment:%s\n", ts->touch_environment);
}

/********************trusted_touch proc************************/
static ssize_t trusted_touch_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct touchpanel_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		return -1;
	}

	return scnprintf(buf, PAGE_SIZE, "%d",
			atomic_read(&ts->trusted_touch_enabled));
}

static ssize_t trusted_touch_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long value;
	int err = 0;
	struct touchpanel_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		return -1;
	}

	if (count > 2)
		return -EINVAL;
	err = kstrtoul(buf, 10, &value);
	if (err != 0)
		return err;

	if (!atomic_read(&ts->trusted_touch_initialized))
		return -EIO;

#ifdef CONFIG_ARCH_QTI_VM
	err = touchpanel_handle_trusted_touch_tvm(ts, value);
	if (err) {
		TPD_INFO("Failed to handle trusted touch in tvm\n");
		return -EINVAL;
	}

    if(value == 0x01) {
	    if (!ts->ts_ops->get_chip_info) {
		    err = -EINVAL;
		    TP_INFO(ts->tp_index, "tp get_chip_info NULL!\n");
	    }

	    err = ts->ts_ops->get_chip_info(ts->chip_data);
	    if (err < 0) {
		    err = -EINVAL;
		    TP_INFO(ts->tp_index, "tp get_chip_info failed!\n");
	    }
	}
#else
	err = touchpanel_handle_trusted_touch_pvm(ts, value);
	if (err) {
		TPD_INFO("Failed to handle trusted touch in pvm\n");
		return -EINVAL;
	}
#endif

	err = count;
	return err;
}

static ssize_t trusted_touch_event_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct touchpanel_data *ts = dev_get_drvdata(dev);
	if (!ts) {
		return -1;
	}
	return scnprintf(buf, PAGE_SIZE, "%d", atomic_read(&ts->trusted_touch_event));
}

static ssize_t trusted_touch_event_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct touchpanel_data *ts = dev_get_drvdata(dev);
	unsigned long value;
	int err = 0;

	if (!ts) {
		return -1;
	}

	if (count > 2)
		return -EINVAL;

	err = kstrtoul(buf, 10, &value);
	if (err != 0)
		return err;

	if (!atomic_read(&ts->trusted_touch_initialized))
		return -EIO;

	if (value)
		return -EIO;

	atomic_set(&ts->trusted_touch_event, value);

	return count;
}

static ssize_t trusted_touch_type_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct touchpanel_data *ts = dev_get_drvdata(dev);

	if (!ts) {
		return -1;
	}

	return scnprintf(buf, PAGE_SIZE, "%s", ts->vm_info->trusted_touch_type);
}

static DEVICE_ATTR(trusted_touch_enable, 0664, trusted_touch_enable_show, trusted_touch_enable_store);
static DEVICE_ATTR(trusted_touch_event, 0664, trusted_touch_event_show, trusted_touch_event_store);
static DEVICE_ATTR(trusted_touch_type, 0664, trusted_touch_type_show, NULL);

/* add your attr in here*/
static struct attribute *trusted_touch_te_attributes[] = {
	&dev_attr_trusted_touch_enable.attr,
	&dev_attr_trusted_touch_event.attr,
	&dev_attr_trusted_touch_type.attr,
	NULL
};

static struct attribute_group trusted_touch_te_attribute_group = {
	.attrs = trusted_touch_te_attributes
};

/********************trusted_touch External  API***********************/
int touchpanel_trusted_touch_create_sysfs(struct touchpanel_data *ts)
{
	int ret = 0;

	if(!ts->trusted_touch_support)
		return -1;

	TPD_INFO("%s: enter.\n", __func__);

	ret = sysfs_create_group(&ts->dev->kobj, &trusted_touch_te_attribute_group);
	if (ret) {
		TPD_INFO("[EX]: sysfs_create_group() failed!!");
		sysfs_remove_group(&ts->dev->kobj, &trusted_touch_te_attribute_group);
		return -ENOMEM;
	} else {
		TPD_INFO("[EX]: sysfs_create_group() succeeded!!");
	}

	return ret;
}

int touchpaneltrusted_touch_remove_sysfs(struct touchpanel_data *ts)
{
	if(!ts->trusted_touch_support)
		return -1;

	TPD_INFO("%s: enter.\n", __func__);

	sysfs_remove_group(&ts->dev->kobj, &trusted_touch_te_attribute_group);
	return 0;
}

void touchpanel_trusted_touch_init(struct touchpanel_data *ts)
{
	int rc = 0;

	if(!ts->trusted_touch_support)
		return;

	mutex_init(&(ts->clk_io_ctrl_mutex));
	mutex_init(&ts->transition_lock);
	spin_lock_init(&ts->irq_lock);

	atomic_set(&ts->trusted_touch_initialized, 0);
	touchpanel_dt_parse_trusted_touch_info(ts);

	if (atomic_read(&ts->trusted_touch_mode) == TRUSTED_TOUCH_MODE_NONE)
		return;

	init_completion(&ts->trusted_touch_powerdown);

	/* Get clocks */
	switch (ts->bus_type) {
	case TP_BUS_I2C:
		TP_INFO(ts->tp_index, "%s: bus_type = I2C\n", __func__);
		ts->core_clk = devm_clk_get(ts->client->dev.parent, "m-ahb");
		if (IS_ERR(ts->core_clk)) {
			ts->core_clk = NULL;
			dev_warn(ts->dev, "%s: core_clk is not defined\n", __func__);
		}

		ts->iface_clk = devm_clk_get(ts->client->dev.parent, "se-clk");
		if (IS_ERR(ts->iface_clk)) {
			ts->iface_clk = NULL;
			dev_warn(ts->dev, "%s: iface_clk is not defined\n", __func__);
		}
		break;
	case TP_BUS_SPI:
		TP_INFO(ts->tp_index, "%s: bus_type = SPI\n", __func__);
		ts->core_clk = devm_clk_get(ts->s_client->dev.parent, "m-ahb");
		if (IS_ERR(ts->core_clk)) {
			ts->core_clk = NULL;
			dev_warn(ts->dev, "%s: core_clk is not defined\n", __func__);
		}

		ts->core_clk = devm_clk_get(ts->s_client->dev.parent, "se-clk");
		if (IS_ERR(ts->iface_clk)) {
			ts->iface_clk = NULL;
			dev_warn(ts->dev, "%s: iface_clk is not defined\n", __func__);
		}
		break;
	default:
		TP_INFO(ts->tp_index, "%s: unknown bus_type = %d\n", __func__, ts->bus_type);
		break;
	}

	if (atomic_read(&ts->trusted_touch_mode) == TRUSTED_TOUCH_VM_MODE) {
		rc = touchpanel_vm_init(ts);
		if (rc)
			TPD_INFO("Failed to init VM\n");
	}
	atomic_set(&ts->trusted_touch_initialized, 1);
}
#endif

void touchpanel_trusted_touch_completion(struct touchpanel_data *ts)
{
    if(!ts->trusted_touch_support)
		return;
#ifdef CONFIG_TOUCHPANEL_TRUSTED_TOUCH
	if (atomic_read(&ts->trusted_touch_transition) || atomic_read(&ts->trusted_touch_enabled))
		wait_for_completion_interruptible(&ts->trusted_touch_powerdown);
#endif
}
