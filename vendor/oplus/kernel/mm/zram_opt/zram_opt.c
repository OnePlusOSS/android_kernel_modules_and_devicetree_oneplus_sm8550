// SPDX-License-Identifier: GPL-2.0-only
/*
 * balance_anon_file_reclaim
 * Additions taken from modifications to drivers/soc/qcom/mem-hooks.c
 * as of commit 3d2c3049d26e soc: qcom: mem-hooks: install balance_anon_file_reclaim
 *
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "zram_opt: " fmt

#include <linux/module.h>
#include <linux/types.h>
#include <trace/hooks/vmscan.h>
#include <linux/swap.h>
#include <linux/proc_fs.h>
#if IS_ENABLED(CONFIG_OPLUS_OOM_REAPER_OPT)
#include <trace/hooks/signal.h>
#include <linux/mm.h>
#include <linux/sched/task.h>
#endif /* CONFIG_OPLUS_OOM_REAPER_OPT */

static int g_direct_swappiness = 60;
static int g_swappiness = 160;

#ifdef CONFIG_DYNAMIC_TUNING_SWAPPINESS
static int threshold1_vm_swappiness;
static int threshold2_vm_swappiness;
static int threshold1_swappiness_size;
static int threshold2_swappiness_size;
static struct proc_dir_entry *dynamic_swappiness_entry;

#define check_swappiness(val) (((val) > 200) || ((val) < 0))
#define check_vm_threshold(val) ((val) < 0)
#endif

#define PARA_BUF_LEN 128
static int g_hybridswapd_swappiness = 200;
static struct proc_dir_entry *para_entry;

static int g_kswapd_pid = -1;
#define KSWAPD_COMM "kswapd0"

#ifdef CONFIG_DYNAMIC_TUNING_SWAPPINESS
int tune_dynamic_swappines(void)
{
	unsigned long nr_file_pages = 0;

	nr_file_pages = global_node_page_state(NR_ACTIVE_FILE) +
		global_node_page_state(NR_INACTIVE_FILE);

	if (threshold1_swappiness_size &&
			(nr_file_pages >= (threshold1_swappiness_size << 8)))
		return threshold1_vm_swappiness ? : g_swappiness;
	else if (threshold2_swappiness_size &&
			(nr_file_pages >= (threshold2_swappiness_size << 8)))
		return threshold2_vm_swappiness ? : g_swappiness;

	return g_swappiness;
}
#endif

static void zo_set_swappiness(void *data, int *swappiness)
{
	if (current_is_kswapd()) {
#ifdef CONFIG_DYNAMIC_TUNING_SWAPPINESS
		*swappiness = tune_dynamic_swappines();
#else
		*swappiness = g_swappiness;
#endif
	} else
		*swappiness = g_direct_swappiness;

	return;
}

/* FIXME: We do not get the vendor_hook back for now, so we skip tune_inactive_ratio temporally */
/*
static void zo_set_inactive_ratio(void *data, unsigned long *inactive_ratio, int file)
{
	if (file)
		*inactive_ratio = min(2UL, *inactive_ratio);
	else
		*inactive_ratio = 1;

	return;
}
*/

#if IS_ENABLED(CONFIG_OPLUS_BALANCE_ANON_FILE_RECLAIM)
static void balance_reclaim(void *unused, bool *balance_anon_file_reclaim)
{
	pg_data_t *pgdat;
	struct zone *zone;
	unsigned long free_pages_threshold = 0;
	unsigned long normal_zone_free_pages = 0;

	pgdat = NODE_DATA(0);
	zone = &pgdat->node_zones[ZONE_NORMAL];
	free_pages_threshold = low_wmark_pages(zone) + ((high_wmark_pages(zone) - low_wmark_pages(zone)) >> 1);

	/* We do not balance reclaim anon and page cache files when free < low + (high - low)/2; */
	normal_zone_free_pages = zone_page_state(zone, NR_FREE_PAGES);
	if(normal_zone_free_pages <  free_pages_threshold) {
		*balance_anon_file_reclaim = false;
	} else {
		*balance_anon_file_reclaim = true;
	}
}
#endif /* CONFIG_OPLUS_BALANCE_ANON_FILE_RECLAIM */

#if IS_ENABLED(CONFIG_OPLUS_OOM_REAPER_OPT)
#define REAPER_SZ (SZ_1M * 32 / PAGE_SIZE)
static void android_vh_killed_process_handler(void *unused, struct task_struct *killer,
											struct task_struct *killee, bool *reap)
{
	unsigned long pages = 0;

	if (!strcmp(killer->comm, "lmkd") || !strcmp(killer->comm, "athena_killer") ||
						!strcmp(killer->comm, "PreKillActionT")) {
		task_lock(killee);
		if (killee->mm)
			pages = get_mm_counter(killee->mm, MM_ANONPAGES) +
					get_mm_counter(killee->mm, MM_SWAPENTS);
		task_unlock(killee);

		if (pages > REAPER_SZ)
			*reap = true;
	}
}
#endif /*CONFIG_OPLUS_OOM_REAPER_OPT*/

static int register_zram_opt_vendor_hooks(void)
{
	int ret = 0;

	ret = register_trace_android_vh_tune_swappiness(zo_set_swappiness, NULL);
	if (ret != 0) {
		pr_err("register_trace_android_vh_set_swappiness failed! ret=%d\n", ret);
		goto out;
	}

	/* FIXME: We do not get the vendor_hook back for now, so we skip tune_inactive_ratio temporally */
	/*
	ret = register_trace_android_vh_tune_inactive_ratio(zo_set_inactive_ratio, NULL);
	if (ret != 0) {
		pr_err("register_trace_android_vh_tune_inactive_ratio failed! ret=%d\n", ret);
		goto out;
	}
	*/

#if IS_ENABLED(CONFIG_OPLUS_BALANCE_ANON_FILE_RECLAIM)
	ret = register_trace_android_rvh_set_balance_anon_file_reclaim(balance_reclaim,
								       NULL);
	if (ret) {
		pr_err("Failed to register balance_anon_file_reclaim hooks\n");
		return ret;
	}
#endif /* CONFIG_OPLUS_BALANCE_ANON_FILE_RECLAIM */

#if IS_ENABLED(CONFIG_OPLUS_OOM_REAPER_OPT)
	ret = register_trace_android_vh_killed_process(android_vh_killed_process_handler, NULL);
	if (ret) {
		pr_err("Failed to register killed_process hooks\n");
		return ret;
	}
#endif /*CONFIG_OPLUS_OOM_REAPER_OPT*/

out:
	return ret;
}

static void unregister_zram_opt_vendor_hooks(void)
{
	unregister_trace_android_vh_tune_swappiness(zo_set_swappiness, NULL);
	/* unregister_trace_android_vh_tune_inactive_ratio(zo_set_inactive_ratio, NULL); */
#if IS_ENABLED(CONFIG_OPLUS_OOM_REAPER_OPT)
	unregister_trace_android_vh_killed_process(android_vh_killed_process_handler, NULL);
#endif /*CONFIG_OPLUS_OOM_REAPER_OPT*/

	return;
}

static inline bool debug_get_val(char *buf, char *token, unsigned long *val)
{
	int ret = -EINVAL;
	char *str = strstr(buf, token);

	if (!str)
		return ret;

	ret = kstrtoul(str + strlen(token), 0, val);
	if (ret)
		return -EINVAL;

	if (*val > 200) {
		pr_err("%lu is invalid\n", *val);
		return -EINVAL;
	}

	return 0;
}

static ssize_t swappiness_para_write(struct file *file,
		const char __user *buff, size_t len, loff_t *ppos)
{
	char kbuf[PARA_BUF_LEN] = {'\0'};
	char *str;
	long val;

	if (len > PARA_BUF_LEN - 1) {
		pr_err("len %d is too long\n", len);
		return -EINVAL;
	}

	if (copy_from_user(&kbuf, buff, len))
		return -EFAULT;
	kbuf[len] = '\0';

	str = strstrip(kbuf);
	if (!str) {
		pr_err("buff %s is invalid\n", kbuf);
		return -EINVAL;
	}

	if (!debug_get_val(str, "vm_swappiness=", &val)) {
		g_swappiness = val;
		return len;
	}

	if (!debug_get_val(str, "direct_swappiness=", &val)) {
		g_direct_swappiness = val;
		return len;
	}

	if (!debug_get_val(str, "swapd_swappiness=", &val)) {
		g_hybridswapd_swappiness = val;
		return len;
	}

	return -EINVAL;
}

static ssize_t swappiness_para_read(struct file *file,
		char __user *buffer, size_t count, loff_t *off)
{
	char kbuf[PARA_BUF_LEN] = {'\0'};
	int len;

	len = snprintf(kbuf, PARA_BUF_LEN, "vm_swappiness: %d\n", g_swappiness);
	len += snprintf(kbuf + len, PARA_BUF_LEN - len,
			"direct_swappiness: %d\n", g_direct_swappiness);
	len += snprintf(kbuf + len, PARA_BUF_LEN - len,
			"swapd_swappiness: %d\n", g_hybridswapd_swappiness);
#ifdef CONFIG_DYNAMIC_TUNING_SWAPPINESS
	len += snprintf(kbuf + len, PARA_BUF_LEN - len,
			"kswapd_swappiness: %d\n",
			tune_dynamic_swappines());
#endif /* CONFIG_DYNAMIC_TUNING_SWAPPINESS */
#if IS_ENABLED(CONFIG_OPLUS_BALANCE_ANON_FILE_RECLAIM)
	len += snprintf(kbuf + len, PARA_BUF_LEN - len,
			"balance_anon_file_reclaim: true\n");
#endif /* CONFIG_OPLUS_BALANCE_ANON_FILE_RECLAIM */

	if (len == PARA_BUF_LEN)
		kbuf[len - 1] = '\0';

	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buffer, kbuf + *off, (len < count ? len : count)))
		return -EFAULT;

	*off += (len < count ? len : count);
	return (len < count ? len : count);
}

static const struct proc_ops proc_swappiness_para_ops = {
	.proc_write          = swappiness_para_write,
	.proc_read		= swappiness_para_read,
	.proc_lseek		= default_llseek,
};

static int __init create_swappiness_para_proc(void)
{
	struct proc_dir_entry *root_dir_entry = proc_mkdir("oplus_mem", NULL);

	para_entry = proc_create((root_dir_entry ?
				"swappiness_para" : "oplus_mem/swappiness_para"),
			0666, root_dir_entry, &proc_swappiness_para_ops);

	if (para_entry) {
		printk("Register swappiness_para interface passed.\n");
		return 0;
	}

	pr_err("Register swappiness_para interface failed.\n");
	return -ENOMEM;
}

static void __exit destroy_swappiness_para_proc(void)
{
	proc_remove(para_entry);
	para_entry = NULL;
}

#ifdef CONFIG_DYNAMIC_TUNING_SWAPPINESS
static ssize_t dynamic_swappiness_write(struct file *file,
		const char __user *buff, size_t len, loff_t *ppos)
{
	char kbuf[PARA_BUF_LEN] = {'\0'};
	char *str;
	int swappiness1, swappiness2;
	int size1, size2, ret;

	if ((len > PARA_BUF_LEN - 1) || (len == 0)) {
		pr_err("len %d is invalid\n", len);
		return -EINVAL;
	}

	if (copy_from_user(&kbuf, buff, len))
		return -EFAULT;

	str = strstrip(kbuf);
	if (!str) {
		pr_err("buff %s is invalid\n", kbuf);
		return -EINVAL;
	}

	ret = sscanf(str, "%d %d %d %d", &swappiness1, &size1,
			&swappiness2, &size2);
	if (ret != 4) {
		pr_err("dynamic swappiess parameter is invalid, %s\n", str);
		return -EINVAL;
	}

	if (check_swappiness(swappiness1) || check_swappiness(swappiness2)) {
		pr_err("error swappiness %d %d\n", swappiness1, swappiness2);
		return -EINVAL;
	}

	if (check_vm_threshold(size1) || check_vm_threshold(size2)) {
		pr_err("error threshold %d %d\n", size1, size2);
		return -EINVAL;
	}

	threshold1_vm_swappiness = swappiness1;
	threshold1_swappiness_size = size1;
	threshold2_vm_swappiness = swappiness2;
	threshold2_swappiness_size = size2;

	return len;
}

static ssize_t dynamic_swappiness_read(struct file *file,
		char __user *buffer, size_t count, loff_t *off)
{
	char kbuf[PARA_BUF_LEN] = {'\0'};
	int len;

	len = scnprintf(kbuf, PARA_BUF_LEN, "%d %d %d %d\n",
			threshold1_vm_swappiness,
			threshold1_swappiness_size,
			threshold2_vm_swappiness,
			threshold2_swappiness_size);

	if (len == PARA_BUF_LEN)
		kbuf[len - 1] = '\0';

	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buffer, kbuf + *off, (len < count ? len : count)))
		return -EFAULT;

	*off += (len < count ? len : count);
	return (len < count ? len : count);
}

static const struct proc_ops proc_dynamic_swappiness_ops = {
	.proc_write	= dynamic_swappiness_write,
	.proc_read	= dynamic_swappiness_read,
	.proc_lseek	= default_llseek,
};

static int __init create_dynamic_swappiness_proc(void)
{
	dynamic_swappiness_entry = proc_create("oplus_mem/dynamic_swappiness",
			0666, NULL, &proc_dynamic_swappiness_ops);

	if (dynamic_swappiness_entry) {
		pr_err("Register dynamic_swappiness interface passed.\n");
		return 0;
	}

	pr_err("Register dynamic_swappiness interface failed.\n");
	return -ENOMEM;
}

static void __exit destroy_dynamic_swappiness_proc(void)
{
	proc_remove(dynamic_swappiness_entry);
	dynamic_swappiness_entry = NULL;
}
#endif

static int __init zram_opt_init(void)
{
	int ret = 0;
	struct task_struct *p = NULL;

	ret = create_swappiness_para_proc();
	if (ret)
		return ret;

	ret = register_zram_opt_vendor_hooks();
	if (ret != 0) {
		destroy_swappiness_para_proc();
		return ret;
	}

	rcu_read_lock();
	for_each_process(p) {
		if (p->flags & PF_KTHREAD) {
			if (!strncmp(p->comm, KSWAPD_COMM,
				     sizeof(KSWAPD_COMM) - 1)) {
				g_kswapd_pid = p->pid;
				break;
			}
		}
	}
	rcu_read_unlock();

#ifdef CONFIG_DYNAMIC_TUNING_SWAPPINESS
	/* must called after create_swappiness_para_proc */
	ret = create_dynamic_swappiness_proc();
	if (ret) {
		unregister_zram_opt_vendor_hooks();
		destroy_swappiness_para_proc();
		return ret;
	}
#endif

	pr_info("zram_opt_init succeed kswapd %d!\n", g_kswapd_pid);
	return 0;
}

static void __exit zram_opt_exit(void)
{
#ifdef CONFIG_DYNAMIC_TUNING_SWAPPINESS
	destroy_dynamic_swappiness_proc();
#endif
	unregister_zram_opt_vendor_hooks();
	destroy_swappiness_para_proc();

	pr_info("zram_opt_exit succeed!\n");

	return;
}

module_init(zram_opt_init);
module_exit(zram_opt_exit);

module_param_named(vm_swappiness, g_swappiness, int, S_IRUGO | S_IWUSR);
module_param_named(direct_vm_swappiness, g_direct_swappiness, int, S_IRUGO | S_IWUSR);
module_param_named(kswapd_pid, g_kswapd_pid, int, S_IRUGO | S_IWUSR);
module_param_named(hybridswapd_swappiness, g_hybridswapd_swappiness, int, S_IRUGO | S_IWUSR);
#ifdef CONFIG_DYNAMIC_TUNING_SWAPPINESS
module_param_named(vm_swappiness_threshold1, threshold1_vm_swappiness, int, S_IRUGO | S_IWUSR);
module_param_named(vm_swappiness_threshold2, threshold2_vm_swappiness, int, S_IRUGO | S_IWUSR);
module_param_named(swappiness_threshold1_size, threshold1_swappiness_size, int, S_IRUGO | S_IWUSR);
module_param_named(swappiness_threshold2_size, threshold2_swappiness_size, int, S_IRUGO | S_IWUSR);
#endif
MODULE_LICENSE("GPL v2");
