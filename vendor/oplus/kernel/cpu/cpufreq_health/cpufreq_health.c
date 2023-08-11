// SPDX-License-Identifier: GPL-2.0-only
#include <linux/init.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/kthread.h>
#include <linux/cpufreq.h>
#include <linux/types.h>
#include <linux/cpu.h>
#include <linux/topology.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/cpufreq_health.h>
#include <linux/cpumask.h>
#include <linux/pm_opp.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#ifdef CONFIG_ARCH_MEDIATEK
#include "eem-dbg-lite.h"
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
static struct eemsn_log *eemsn_log; /* eem data from csram */
#endif

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "OCH: " fmt

/* add for midas collection */
static DEFINE_MUTEX(och_mutex);
static struct cpufreq_health_info och_data[64];
static int record_count;

struct oplus_cpufreq_health_attr {
	struct attribute attr;
	ssize_t (*show)(struct oplus_cpufreq_health *och, char *buf);
	ssize_t (*store)(struct oplus_cpufreq_health *och, const char *buf,
		size_t count);
};

static ssize_t show_floor_limit(struct oplus_cpufreq_health *och, char *buf)
{
	unsigned long flags;
	int i, count = 0;

	if (!och->table)
		return 0;

	spin_lock_irqsave(&och->lock, flags);
	for (i = 0; i < och->len; i++) {
		count += snprintf(buf + count, PAGE_SIZE - count, "%-10u\t%-10llu\t%-10llu\n",
				  och->table[i].frequency, och->table[i].floor_time, och->table[i].floor_count);
	}
	count += snprintf(buf + count, PAGE_SIZE - count, "elapsed %lld\n",
				jiffies_64_to_clock_t(och->last_update_time - och->bootup_time));
	spin_unlock_irqrestore(&och->lock, flags);
	return count;
}

static ssize_t show_ceiling_limit(struct oplus_cpufreq_health *och, char *buf)
{
	unsigned long flags;
	int i, count = 0;

	if (!och->table)
		return 0;

	spin_lock_irqsave(&och->lock, flags);
	for (i = 0; i < och->len; i++) {
		count += snprintf(buf + count, PAGE_SIZE - count, "%-10u\t%-10llu\t%-10llu\n",
				  och->table[i].frequency, och->table[i].ceiling_time, och->table[i].ceiling_count);
	}
	count += snprintf(buf + count, PAGE_SIZE - count, "elapsed %lld\n",
				jiffies_64_to_clock_t(och->last_update_time - och->bootup_time));
	spin_unlock_irqrestore(&och->lock, flags);
	return count;
}

static ssize_t show_cpu_voltage(struct oplus_cpufreq_health *och, char *buf)
{
	unsigned long flags;
	int i, count = 0;

	if (!och->table)
		return 0;

	spin_lock_irqsave(&och->lock, flags);
	for (i = och->len - 1; i >= 0; i--) {
		count += snprintf(buf + count, PAGE_SIZE - count, "%u,",
				  och->table[i].voltage);
	}
	spin_unlock_irqrestore(&och->lock, flags);
	return count;
}

#if IS_ENABLED(CONFIG_SCHED_WALT)
static ssize_t show_util_boost(struct oplus_cpufreq_health *och, char *buf)
{
	unsigned long flags;
	int count = 0;
	u64 now;

	if (!och->table)
		return 0;

	spin_lock_irqsave(&och->lock, flags);

	count += snprintf(buf + count, PAGE_SIZE - count, "%s\t%-20llu\t%-10llu\n", "newtask_boost",
				  och->newtask_boost_time/10000000, och->newtask_count);

	count += snprintf(buf + count, PAGE_SIZE - count, "%s\t%-20llu\t%-10llu\n", "edtask_boost",
				  och->edtask_boost_time/10000000, och->edtask_count);

	now = get_jiffies_64();

	count += snprintf(buf + count, PAGE_SIZE - count, "elapsed %lld\n",
				jiffies_64_to_clock_t(now - och->bootup_time));
	spin_unlock_irqrestore(&och->lock, flags);
	return count;
}
#endif

#define oplus_cpufreq_health_attr_ro(_name)		\
static struct oplus_cpufreq_health_attr _name =	\
__ATTR(_name, 0444, show_##_name, NULL)

#define oplus_cpufreq_health_attr_rw(_name)			\
static struct oplus_cpufreq_health_attr _name =		\
__ATTR(_name, 0644, show_##_name, store_##_name)

oplus_cpufreq_health_attr_ro(floor_limit);
oplus_cpufreq_health_attr_ro(ceiling_limit);
oplus_cpufreq_health_attr_ro(cpu_voltage);
#if IS_ENABLED(CONFIG_SCHED_WALT)
oplus_cpufreq_health_attr_ro(util_boost);
#endif

static struct attribute *default_attrs[] = {
	&floor_limit.attr,
	&ceiling_limit.attr,
	&cpu_voltage.attr,
#if IS_ENABLED(CONFIG_SCHED_WALT)
	&util_boost.attr,
#endif
	NULL
};

#define to_oplus_cpufreq_health(k) \
		container_of(k, struct oplus_cpufreq_health, kobj)
#define to_attr(a) container_of(a, struct oplus_cpufreq_health_attr, attr)

static ssize_t show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	struct oplus_cpufreq_health *och = to_oplus_cpufreq_health(kobj);
	struct oplus_cpufreq_health_attr *cattr = to_attr(attr);
	ssize_t ret = -EIO;

	if (cattr->show)
		ret = cattr->show(och, buf);

	return ret;
}

static ssize_t store(struct kobject *kobj, struct attribute *attr,
		     const char *buf, size_t count)
{
	struct oplus_cpufreq_health *och = to_oplus_cpufreq_health(kobj);
	struct oplus_cpufreq_health_attr *cattr = to_attr(attr);
	ssize_t ret = -EIO;

	if (cattr->store)
		ret = cattr->store(och, buf, count);

	return ret;
}

static const struct sysfs_ops sysfs_ops = {
	.show	= show,
	.store	= store,
};

static struct kobj_type ktype_oplus_cpufreq_health = {
	.sysfs_ops	= &sysfs_ops,
	.default_attrs	= default_attrs,
};

#define MAX_CLUSTERS 3

static struct oplus_cpufreq_health oplus_cpufreq_health[MAX_CLUSTERS];

#if IS_ENABLED(CONFIG_SCHED_WALT)
void cpufreq_health_get_edtask_state(int cpu, int edtask_flag)
{
	struct cpufreq_policy *policy;
	unsigned int first_cpu;
	int cluster_id;
	int now_edtask_flag = 0;
	u64 delta, now;
	struct oplus_cpufreq_health *och;
	unsigned long flags;
	int idx, i;

	policy = cpufreq_cpu_get(cpu);

	if (!policy) {
		pr_info("cpu %d, policy is null\n", cpu);
		return;
	}

	first_cpu = cpumask_first(policy->related_cpus);
	cluster_id = topology_physical_package_id(first_cpu);
	cpufreq_cpu_put(policy);

	if (cluster_id >= MAX_CLUSTERS)
		return;

	och = &oplus_cpufreq_health[cluster_id];

	if (och->is_init == 0)
		return;

	idx = cpu - first_cpu;

	if (idx < 0 || idx >= och->cpu_count)
		return;

	idx = cpu - first_cpu;

	if (idx < 0 || idx >= och->cpu_count)
		return;

	now = ktime_get_ns();

	delta = time_after64(now, och->last_update_edtask_time)
		? now - och->last_update_edtask_time : 0;

	spin_lock_irqsave(&och->lock, flags);

	och->edtask_table[idx].edtask_state = edtask_flag;

	for (i = 0; i < och->cpu_count; i++) {
		if (och->edtask_table[i].edtask_state == 1) {
			now_edtask_flag = 1;
			break;
		}
	}

	if (now_edtask_flag == 1) {
		if (och->pre_edtask_state == 1)
			och->edtask_boost_time += delta;
		else
			och->edtask_count++;
	}

	if (now_edtask_flag == 0 && och->pre_edtask_state == 1) {
		och->edtask_count++;
		och->edtask_boost_time += delta;
	}

	och->pre_edtask_state = now_edtask_flag;
	och->last_update_edtask_time = now;
	spin_unlock_irqrestore(&och->lock, flags);
}
EXPORT_SYMBOL_GPL(cpufreq_health_get_edtask_state);

void cpufreq_health_get_newtask_state(struct cpufreq_policy *policy, int newtask_flag)
{
	unsigned int first_cpu = cpumask_first(policy->related_cpus);
	int cluster_id;
	u64 delta, now;
	struct oplus_cpufreq_health *och;
	unsigned long flags;

	cluster_id = topology_physical_package_id(first_cpu);
	if (cluster_id >= MAX_CLUSTERS)
		return;

	och = &oplus_cpufreq_health[cluster_id];

	if (och->is_init == 0)
		return;

	now = ktime_get_ns();

	delta = time_after64(now, och->last_update_util_time)
		? now - och->last_update_util_time : 0;

	spin_lock_irqsave(&och->lock, flags);

	if (newtask_flag == 1) {
		if (och->pre_newtask_state == 1)
			och->newtask_boost_time += delta;
		else
			och->newtask_count++;
	}
	if (newtask_flag == 0 && och->pre_newtask_state == 1) {
		och->newtask_count++;
		och->newtask_boost_time += delta;
	}
	och->pre_newtask_state = newtask_flag;
	och->last_update_util_time = now;

	spin_unlock_irqrestore(&och->lock, flags);
}
EXPORT_SYMBOL_GPL(cpufreq_health_get_newtask_state);
#endif

int freq_to_voltage(int cluster_id, unsigned int target_freq)
{
	int idx;
	struct oplus_cpufreq_health *och;
	unsigned int freq;

	och = &oplus_cpufreq_health[cluster_id];

	if (och->is_init == 0) {
		pr_info("och driver init failed!\n");
		return -EINVAL;
	}

	for (idx = 0; idx < och->len; idx++) {
		freq = och->table[idx].frequency;

		if (freq == target_freq)
			return och->table[idx].voltage;
	}


	return 0;
}
EXPORT_SYMBOL_GPL(freq_to_voltage);

static int freq_table_get_closest_index(struct oplus_cpufreq_health *och, unsigned int target_freq)
{
	int idx, best = -1;
	unsigned int freq;

	for (idx = 0; idx < och->len; idx++) {
		freq = och->table[idx].frequency;

		if (freq == target_freq)
			return idx;

		if (freq < target_freq) {
			best = idx;
			continue;
		}

		/* No freq found below target_freq */
		if (best == -1)
			return idx;

		/* Choose the closest freq */
		if (target_freq - och->table[best].frequency > freq - target_freq)
			return idx;

		return best;
	}
	return best;
}

void cpufreq_health_get_state(struct cpufreq_policy *policy)
{
	unsigned int first_cpu = cpumask_first(policy->related_cpus);
	int cluster_id, floor_idx, ceiling_idx;
	u64 delta, now;
	struct oplus_cpufreq_health *och;
	unsigned long flags;

	cluster_id = topology_physical_package_id(first_cpu);
	if (cluster_id >= MAX_CLUSTERS)
		return;

	och = &oplus_cpufreq_health[cluster_id];

	if (och->is_init == 0)
		return;

	if (och->curr_floor_freq == policy->min &&
		och->curr_ceiling_freq == policy->max)
		return;

	now = get_jiffies_64();

	delta = time_after64(now, och->last_update_time)
		? now - och->last_update_time : 0;
	delta = jiffies_64_to_clock_t(delta);

	floor_idx = freq_table_get_closest_index(och, och->curr_floor_freq);
	ceiling_idx = freq_table_get_closest_index(och, och->curr_ceiling_freq);

	if (floor_idx >= och->len || floor_idx < 0 || ceiling_idx >= och->len || ceiling_idx < 0) {
		pr_err("invalid idx = %d %d\n", floor_idx, ceiling_idx);
		return;
	}

	spin_lock_irqsave(&och->lock, flags);
	/*statistics floor*/
	och->table[floor_idx].floor_time += delta;
	if (policy->min != och->curr_floor_freq)
		och->table[floor_idx].floor_count++;
	och->prev_floor_freq = och->curr_floor_freq;
	och->curr_floor_freq = policy->min;

	/*statistics ceiling*/
	och->table[ceiling_idx].ceiling_time += delta;
	if (policy->max != och->curr_ceiling_freq)
		och->table[ceiling_idx].ceiling_count++;
	och->prev_ceiling_freq = och->curr_ceiling_freq;
	och->curr_ceiling_freq = policy->max;

	/*updat last_update_time*/
	och->last_update_time = now;
	spin_unlock_irqrestore(&och->lock, flags);
	pr_debug("cluster = %d freq = %d  %d, delta = %lld\n",
			cluster_id,
			och->prev_floor_freq,
			och->prev_ceiling_freq,
			delta);
}
EXPORT_SYMBOL_GPL(cpufreq_health_get_state);

void update_och_data(void)
{
	int cluster_id, i;
	struct oplus_cpufreq_health *och;
	unsigned long flags;

	record_count = 0;
	for (cluster_id = 0; cluster_id < MAX_CLUSTERS; cluster_id++) {
		och = &oplus_cpufreq_health[cluster_id];

		spin_lock_irqsave(&och->lock, flags);

		for (i = 0; i < och->len; i++) {
			och_data[record_count].ceiling_time = och->table[i].ceiling_time;
			och_data[record_count].ceiling_count = och->table[i].ceiling_count;
			och_data[record_count].floor_time = och->table[i].floor_time;
			och_data[record_count].floor_count = och->table[i].floor_count;
			/* the sum of all cluster freq num is less than 64 */
			record_count++;
		}

		spin_unlock_irqrestore(&och->lock, flags);
	}
}

void get_cpufreq_health_info(int *cnt, struct cpufreq_health_info *val)
{
	if (!val)
		return;

	update_och_data();
	mutex_lock(&och_mutex);
	*cnt = record_count;
	memcpy(val, och_data, sizeof(struct cpufreq_health_info) * record_count);

	mutex_unlock(&och_mutex);
}
EXPORT_SYMBOL_GPL(get_cpufreq_health_info);

int cpufreq_health_register(struct cpufreq_policy *policy)
{
	int idx, first_cpu, cluster_id;
	struct device *cpu_dev;
	struct oplus_cpufreq_health *och;
	struct cpufreq_frequency_table *pos;
	unsigned long mV, flags;
#if IS_ENABLED(CONFIG_SCHED_WALT)
	unsigned long freq;
	struct dev_pm_opp *opp;
	int index;
#endif

	first_cpu = cpumask_first(policy->related_cpus);
	cpu_dev = get_cpu_device(first_cpu);
	if (!cpu_dev)
		return -ENODEV;

	cluster_id = topology_physical_package_id(first_cpu);
	if (cluster_id >= MAX_CLUSTERS) {
		pr_err("Unsupported number of clusters(%d). Only %u supported\n",
				cluster_id, MAX_CLUSTERS);
		return -EINVAL;
	}

	och = &oplus_cpufreq_health[cluster_id];

	/* avoid init repeatedlly */
	if (och->is_init)
		return 0;

	pr_info("cluster idx = %d, cpumask = 0x%x\n", cluster_id,
			(int)cpumask_bits(policy->related_cpus)[0]);

	/*how many freq entry*/
	cpufreq_for_each_valid_entry_idx(pos, policy->freq_table, idx);
	och->len = idx;

	och->table = kzalloc(sizeof(struct oplus_cpufreq_frequency_table) * och->len,
			GFP_KERNEL);
	if (!och->table) {
		pr_err("No memory\n");
		return -ENOMEM;
	}

#if IS_ENABLED(CONFIG_SCHED_WALT)
	och->cpu_count = cpumask_last(policy->related_cpus) - first_cpu + 1;

	och->edtask_table = kzalloc(
			sizeof(struct oplus_cpu_edtask_table) * och->cpu_count,
			GFP_KERNEL);

	if (!och->edtask_table) {
		pr_err("No memory for edtask_table\n");
		return -ENOMEM;
	}

	for (index = 0; index < och->cpu_count; index++)
		och->edtask_table[index].edtask_state = 0;
#endif

	cpufreq_for_each_valid_entry_idx(pos, policy->freq_table, idx) {
		if (idx > och->len)
			break;
		och->table[idx].frequency = pos->frequency;
	}

	och->curr_floor_freq = och->prev_floor_freq = policy->min;
	och->curr_ceiling_freq = och->prev_ceiling_freq = policy->max;

	och->bootup_time = och->last_update_time = get_jiffies_64();

#if IS_ENABLED(CONFIG_SCHED_WALT)
	och->last_update_util_time = ktime_get_ns();
	och->last_update_edtask_time = ktime_get_ns();
#endif
	spin_lock_init(&och->lock);
	och->is_init = 1;

	/* voltage table init */
#ifdef CONFIG_ARCH_MEDIATEK
	for (idx = 0; idx < och->len; idx++) {
		if (eemsn_log->det_log[cluster_id].freq_tbl[idx] == 0)
			break;
		mV = eemsn_log->det_log[cluster_id].volt_tbl_init2[idx] * VOLT_STEP;

		spin_lock_irqsave(&och->lock, flags);
		och->table[idx].voltage = mV;
		spin_unlock_irqrestore(&och->lock, flags);
	}
#else
	cpufreq_for_each_valid_entry(pos, policy->freq_table) {
		idx = freq_table_get_closest_index(och, pos->frequency);
		freq = pos->frequency * 1000;
		opp = dev_pm_opp_find_freq_ceil(cpu_dev, &freq);

		if (IS_ERR(opp))
			goto out;

		mV = dev_pm_opp_get_voltage(opp) / 1000;
		dev_pm_opp_put(opp);
		if (!mV)
			goto out;

		spin_lock_irqsave(&och->lock, flags);
		och->table[idx].voltage = mV;
		spin_unlock_irqrestore(&och->lock, flags);

	}
out:
#endif /* CONFIG_ARCH_MEDIATEK  */
	kobject_init(&och->kobj, &ktype_oplus_cpufreq_health);
	return kobject_add(&och->kobj, &cpu_dev->kobj, "cpufreq_health");
}
EXPORT_SYMBOL_GPL(cpufreq_health_register);

#ifdef CONFIG_ARCH_MEDIATEK
static int mtk_emm_init(struct platform_device *pdev)
{
	int err = 0;
	struct resource *eem_res;

	eem_res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (eem_res)
		eemsn_log = ioremap(eem_res->start, resource_size(eem_res));
	else {
		pr_info("%s can't get resource, ret:%d\n", __func__, err);
		return 0;
	}

	return 0;
}
#endif /* CONFIG_ARCH_MEDIATEK  */

int __init oplus_cpufreq_health_init(void)
{
#ifdef CONFIG_ARCH_MEDIATEK
	struct platform_device *pdev;
	struct device_node *dvfs_node;
	int ret;

	dvfs_node = of_find_node_by_name(NULL, "cpuhvfs");
	if (dvfs_node == NULL) {
		pr_info("failed to find node @ %s\n", __func__);
		return -ENODEV;
	}

	pdev = of_find_device_by_node(dvfs_node);
	if (pdev == NULL) {
		pr_info("failed to find pdev @ %s\n", __func__);
		return -EINVAL;
	}

	ret = mtk_emm_init(pdev);
	if (ret)
		pr_info("eem dbg init failed: %d\n", ret);
#endif
	pr_info("cpufreq health driver begin to init!\n");
	return 0;
}

fs_initcall(oplus_cpufreq_health_init);

MODULE_DESCRIPTION("CpuFreq health");
MODULE_VERSION("2.0");
MODULE_LICENSE("GPL");
