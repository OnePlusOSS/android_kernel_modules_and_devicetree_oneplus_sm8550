/****************trusted_touch********************/
#ifdef CONFIG_TOUCHPANEL_TRUSTED_TOUCH
#include <linux/dma-mapping.h>
#include <linux/gunyah/gh_irq_lend.h>
#include <linux/gunyah/gh_mem_notifier.h>
#include "../touchpanel_common.h"

#define TRUSTED_TOUCH_MEM_LABEL 0x7
#define TOUCH_RESET_GPIO_BASE 0xF114000
#define TOUCH_RESET_GPIO_SIZE 0x1000
#define TOUCH_RESET_GPIO_OFFSET 0x4
#define TOUCH_INTR_GPIO_BASE 0xF115000
#define TOUCH_INTR_GPIO_SIZE 0x1000
#define TOUCH_INTR_GPIO_OFFSET 0x8

#define TRUSTED_TOUCH_EVENT_LEND_FAILURE -1
#define TRUSTED_TOUCH_EVENT_LEND_NOTIFICATION_FAILURE -2
#define TRUSTED_TOUCH_EVENT_ACCEPT_FAILURE -3
#define	TRUSTED_TOUCH_EVENT_FUNCTIONAL_FAILURE -4
#define	TRUSTED_TOUCH_EVENT_RELEASE_FAILURE -5
#define	TRUSTED_TOUCH_EVENT_RECLAIM_FAILURE -6
#define	TRUSTED_TOUCH_EVENT_I2C_FAILURE -7
#define	TRUSTED_TOUCH_EVENT_NOTIFICATIONS_PENDING 5

enum trusted_touch_mode_config {
	TRUSTED_TOUCH_VM_MODE,
	TRUSTED_TOUCH_MODE_NONE
};

enum trusted_touch_pvm_states {
	TRUSTED_TOUCH_PVM_INIT,
	PVM_I2C_RESOURCE_ACQUIRED,
	PVM_INTERRUPT_DISABLED,
	PVM_IOMEM_LENT,
	PVM_IOMEM_LENT_NOTIFIED,
	PVM_IRQ_LENT,
	PVM_IRQ_LENT_NOTIFIED,
	PVM_IOMEM_RELEASE_NOTIFIED,
	PVM_IRQ_RELEASE_NOTIFIED,
	PVM_ALL_RESOURCES_RELEASE_NOTIFIED,
	PVM_IRQ_RECLAIMED,
	PVM_IOMEM_RECLAIMED,
	PVM_INTERRUPT_ENABLED,
	PVM_I2C_RESOURCE_RELEASED,
	TRUSTED_TOUCH_PVM_STATE_MAX
};

enum trusted_touch_tvm_states {
	TRUSTED_TOUCH_TVM_INIT,
	TVM_IOMEM_LENT_NOTIFIED,
	TVM_IRQ_LENT_NOTIFIED,
	TVM_ALL_RESOURCES_LENT_NOTIFIED,
	TVM_IOMEM_ACCEPTED,
	TVM_I2C_SESSION_ACQUIRED,
	TVM_IRQ_ACCEPTED,
	TVM_INTERRUPT_ENABLED,
	TVM_INTERRUPT_DISABLED,
	TVM_IRQ_RELEASED,
	TVM_I2C_SESSION_RELEASED,
	TVM_IOMEM_RELEASED,
	TRUSTED_TOUCH_TVM_STATE_MAX
};

struct trusted_touch_vm_info {
	enum gh_irq_label irq_label;
	enum gh_mem_notifier_tag mem_tag;
	enum gh_vm_names vm_name;
	const char *trusted_touch_type;
	u32 hw_irq;
	gh_memparcel_handle_t vm_mem_handle;
	u32 *iomem_bases;
	u32 *iomem_sizes;
	u32 iomem_list_size;
	void *mem_cookie;
	atomic_t vm_state;
	u32 iomem_rst_base;
	u32 iomem_rst_size;
	u32 iomem_rst_offset;
	u32 iomem_irq_base;
	u32 iomem_irq_size;
	u32 iomem_irq_offset;
};

void touchpanel_tvm_i2c_failure_report(struct touchpanel_data *ts);
void touchpanel_trusted_touch_init(struct touchpanel_data *ts);
void touchpanel_trusted_touch_irq_disable(struct touchpanel_data *ts);
void touchpanel_trusted_touch_irq_enable(struct touchpanel_data *ts);
void touchpanel_trusted_touch_irq_free(struct touchpanel_data *ts);
int touchpanel_trusted_touch_create_sysfs(struct touchpanel_data *ts);
int touchpanel_trusted_touch_remove_sysfs(struct touchpanel_data *ts);
#endif
void touchpanel_trusted_touch_completion(struct touchpanel_data *ts);
