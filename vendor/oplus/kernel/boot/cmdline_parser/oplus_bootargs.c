#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/of_fdt.h>
#include <linux/of.h>

char *bootargs_ptr = NULL;
EXPORT_SYMBOL(bootargs_ptr);

#define BOOTARGS_LEN (4*1024) //4K
static int __init oplus_bootargs_init(void)
{
    int ret;
    struct device_node *of_chosen;

    bootargs_ptr = (char *)kmalloc(BOOTARGS_LEN, GFP_KERNEL);
    if (!bootargs_ptr) {
        printk(KERN_ERR "kmalloc for bootargs ptr failed !\n");
        return -1;
    }

    of_chosen = of_find_node_by_path("/chosen");
    if (!of_chosen) {
        of_chosen = of_find_node_by_path("/chosen@0");
    }

    if (of_chosen) {
        ret = of_property_read_string(of_chosen, "bootargs", (const char **)&bootargs_ptr);
        if (ret) {
            printk(KERN_ERR "try to get bootargs failed !");
            return -1;
        }
    }

    return 0;
}

module_init(oplus_bootargs_init);
MODULE_LICENSE("GPL v2");
