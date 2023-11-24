// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Oplus. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>

#include "game_ctrl.h"

/* To handle cpufreq min/max request */
struct cpu_freq_status {
	unsigned int min;
	unsigned int max;
};

static DEFINE_PER_CPU(struct cpu_freq_status, game_cpu_stats);
static DEFINE_PER_CPU(struct freq_qos_request, qos_req_min);
static DEFINE_PER_CPU(struct freq_qos_request, qos_req_max);

static cpumask_var_t limit_mask_min;
static cpumask_var_t limit_mask_max;

/*
 * sameone[ORMS] can disable GPA cpufreq limit,
 * by writing 1 to /proc/game_opt/disable_cpufreq_limit.
 */
static int disable_cpufreq_limit = 0;
static bool timeout_release_cpufreq_limit = false;
static DEFINE_MUTEX(g_mutex);

/*
 * This is a safeguard mechanism.
 *
 * If GPA made frequency QoS request but not released under some extreme conditions.
 * Kernel releases the frequency QoS request after FREQ_QOS_REQ_MAX_MS.
 * It will probably never happen.
 */
#define FREQ_QOS_REQ_MAX_MS  (60 * MSEC_PER_SEC) /* 60s */
static struct delayed_work freq_qos_req_reset_work;

static int freq_qos_request_init(void)
{
	unsigned int cpu;
	int ret;

	struct cpufreq_policy *policy;
	struct freq_qos_request *req;

	for_each_present_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy) {
			pr_err("%s: Failed to get cpufreq policy for cpu%d\n",
				__func__, cpu);
			ret = -EINVAL;
			goto cleanup;
		}
		per_cpu(game_cpu_stats, cpu).min = FREQ_QOS_MIN_DEFAULT_VALUE;
		req = &per_cpu(qos_req_min, cpu);
		ret = freq_qos_add_request(&policy->constraints, req,
			FREQ_QOS_MIN, FREQ_QOS_MIN_DEFAULT_VALUE);
		if (ret < 0) {
			pr_err("%s: Failed to add min freq constraint (%d)\n",
				__func__, ret);
			cpufreq_cpu_put(policy);
			goto cleanup;
		}

		per_cpu(game_cpu_stats, cpu).max = FREQ_QOS_MAX_DEFAULT_VALUE;
		req = &per_cpu(qos_req_max, cpu);
		ret = freq_qos_add_request(&policy->constraints, req,
			FREQ_QOS_MAX, FREQ_QOS_MAX_DEFAULT_VALUE);
		if (ret < 0) {
			pr_err("%s: Failed to add max freq constraint (%d)\n",
				__func__, ret);
			cpufreq_cpu_put(policy);
			goto cleanup;
		}

		cpufreq_cpu_put(policy);
	}
	return 0;

cleanup:
	for_each_present_cpu(cpu) {
		req = &per_cpu(qos_req_min, cpu);
		if (req && freq_qos_request_active(req))
			freq_qos_remove_request(req);

		req = &per_cpu(qos_req_max, cpu);
		if (req && freq_qos_request_active(req))
			freq_qos_remove_request(req);

		per_cpu(game_cpu_stats, cpu).min = FREQ_QOS_MIN_DEFAULT_VALUE;
		per_cpu(game_cpu_stats, cpu).max = FREQ_QOS_MAX_DEFAULT_VALUE;
	}
	return ret;
}

static void __freq_qos_request_restore(void)
{
	int i, j, cpu;
	struct cpumask present_mask;
	struct cpu_freq_status *i_cpu_stats;
	struct cpufreq_policy policy;
	struct freq_qos_request *req;

	cpumask_copy(&present_mask, cpu_present_mask);

	cpus_read_lock();
	for_each_cpu(i, &present_mask) {
		if (cpufreq_get_policy(&policy, i))
			continue;

		for_each_cpu(j, policy.related_cpus)
			cpumask_clear_cpu(j, &present_mask);

		cpu = policy.cpu;

		i_cpu_stats = &per_cpu(game_cpu_stats, cpu);

		req = &per_cpu(qos_req_min, cpu);
		freq_qos_update_request(req, i_cpu_stats->min);
		req = &per_cpu(qos_req_max, cpu);
		freq_qos_update_request(req, i_cpu_stats->max);
	}
	cpus_read_unlock();
}

static void __freq_qos_request_reset(void)
{
	int i, j, cpu;
	struct cpumask present_mask;
	struct cpufreq_policy policy;
	struct freq_qos_request *req;

	cpumask_copy(&present_mask, cpu_present_mask);

	cpus_read_lock();
	for_each_cpu(i, &present_mask) {
		if (cpufreq_get_policy(&policy, i))
			continue;

		for_each_cpu(j, policy.related_cpus)
			cpumask_clear_cpu(j, &present_mask);

		cpu = policy.cpu;

		req = &per_cpu(qos_req_min, cpu);
		freq_qos_update_request(req, FREQ_QOS_MIN_DEFAULT_VALUE);
		req = &per_cpu(qos_req_max, cpu);
		freq_qos_update_request(req, FREQ_QOS_MAX_DEFAULT_VALUE);
	}
	cpus_read_unlock();
}

static void freq_qos_request_reset(struct work_struct *work)
{
	unsigned int cpu;

	mutex_lock(&g_mutex);
	timeout_release_cpufreq_limit = true;
	for_each_present_cpu(cpu) {
		per_cpu(game_cpu_stats, cpu).min = FREQ_QOS_MIN_DEFAULT_VALUE;
		per_cpu(game_cpu_stats, cpu).max = FREQ_QOS_MAX_DEFAULT_VALUE;
	}
	__freq_qos_request_reset();
	mutex_unlock(&g_mutex);
}

static ssize_t set_cpu_min_freq(const char *buf, size_t count)
{
	int i, j, ntokens = 0;
	unsigned int val, cpu;
	unsigned int min_freq;
	const char *cp = buf;
	struct cpu_freq_status *i_cpu_stats;
	struct cpufreq_policy policy;
	struct freq_qos_request *req;

	while ((cp = strpbrk(cp + 1, " :")))
		ntokens++;

	/* CPU:value pair */
	if (!(ntokens % 2))
		return -EINVAL;

	cp = buf;
	cpumask_clear(limit_mask_min);
	for (i = 0; i < ntokens; i += 2) {
		if (sscanf(cp, "%u:%u", &cpu, &val) != 2)
			return -EINVAL;
		if (cpu > (num_present_cpus() - 1))
			return -EINVAL;

		i_cpu_stats = &per_cpu(game_cpu_stats, cpu);

		i_cpu_stats->min = val;
		cpumask_set_cpu(cpu, limit_mask_min);

		cp = strnchr(cp, strlen(cp), ' ');
		cp++;
	}

	/*
	 * Since on synchronous systems policy is shared amongst multiple
	 * CPUs only one CPU needs to be updated for the limit to be
	 * reflected for the entire cluster. We can avoid updating the policy
	 * of other CPUs in the cluster once it is done for at least one CPU
	 * in the cluster
	 */
	cpus_read_lock();
	for_each_cpu(i, limit_mask_min) {
		if (cpufreq_get_policy(&policy, i))
			continue;

		i_cpu_stats = &per_cpu(game_cpu_stats, i);
		min_freq = i_cpu_stats->min;
		for_each_cpu(j, policy.related_cpus) {
			cpumask_clear_cpu(j, limit_mask_min);
			i_cpu_stats = &per_cpu(game_cpu_stats, j);
			i_cpu_stats->min = min_freq;
		}

		if (!disable_cpufreq_limit) {
			req = &per_cpu(qos_req_min, policy.cpu);
			if (freq_qos_update_request(req, min_freq) < 0)
				break;
		}
	}
	cpus_read_unlock();

	return count;
}

static ssize_t cpu_min_freq_proc_write(struct file *file,
	const char __user *buf, size_t count, loff_t *ppos)
{
	char page[256] = {0};
	int ret;

	ret = simple_write_to_buffer(page, sizeof(page) - 1, ppos, buf, count);
	if (ret <= 0)
		return ret;

	mutex_lock(&g_mutex);
	timeout_release_cpufreq_limit = false;
	cancel_delayed_work_sync(&freq_qos_req_reset_work);
	ret = set_cpu_min_freq(page, ret);
	schedule_delayed_work(&freq_qos_req_reset_work, msecs_to_jiffies(FREQ_QOS_REQ_MAX_MS));
	mutex_unlock(&g_mutex);

	return ret;
}

static int cpu_min_freq_show(struct seq_file *m, void *v)
{
	int cpu;

	mutex_lock(&g_mutex);
	for_each_present_cpu(cpu) {
		if (disable_cpufreq_limit)
			seq_printf(m, "%d:%u ", cpu, FREQ_QOS_MIN_DEFAULT_VALUE);
		else
			seq_printf(m, "%d:%u ", cpu, per_cpu(game_cpu_stats, cpu).min);
	}
	seq_printf(m, "\n");
	mutex_unlock(&g_mutex);

	return 0;
}

static int cpu_min_freq_proc_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, cpu_min_freq_show, inode);
}

static const struct proc_ops cpu_min_freq_proc_ops = {
	.proc_open		= cpu_min_freq_proc_open,
	.proc_write 	= cpu_min_freq_proc_write,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release	= single_release,
};

static ssize_t set_cpu_max_freq(const char *buf, size_t count)
{
	int i, j, ntokens = 0;
	unsigned int val, cpu;
	unsigned int max_freq;
	const char *cp = buf;
	struct cpu_freq_status *i_cpu_stats;
	struct cpufreq_policy policy;
	struct freq_qos_request *req;

	while ((cp = strpbrk(cp + 1, " :")))
		ntokens++;

	/* CPU:value pair */
	if (!(ntokens % 2))
		return -EINVAL;

	cp = buf;
	cpumask_clear(limit_mask_max);
	for (i = 0; i < ntokens; i += 2) {
		if (sscanf(cp, "%u:%u", &cpu, &val) != 2)
			return -EINVAL;
		if (cpu > (num_present_cpus() - 1))
			return -EINVAL;

		i_cpu_stats = &per_cpu(game_cpu_stats, cpu);

		i_cpu_stats->max = val;
		cpumask_set_cpu(cpu, limit_mask_max);

		cp = strnchr(cp, strlen(cp), ' ');
		cp++;
	}

	cpus_read_lock();
	for_each_cpu(i, limit_mask_max) {
		if (cpufreq_get_policy(&policy, i))
			continue;

		i_cpu_stats = &per_cpu(game_cpu_stats, i);
		max_freq = i_cpu_stats->max;
		for_each_cpu(j, policy.related_cpus) {
			cpumask_clear_cpu(j, limit_mask_max);
			i_cpu_stats = &per_cpu(game_cpu_stats, j);
			i_cpu_stats->max = max_freq;
		}

		if (!disable_cpufreq_limit) {
			req = &per_cpu(qos_req_max, policy.cpu);
			if (freq_qos_update_request(req, max_freq) < 0)
				break;
		}
	}
	cpus_read_unlock();

	return count;
}

static ssize_t cpu_max_freq_proc_write(struct file *file,
	const char __user *buf, size_t count, loff_t *ppos)
{
	char page[256] = {0};
	int ret;

	ret = simple_write_to_buffer(page, sizeof(page) - 1, ppos, buf, count);
	if (ret <= 0)
		return ret;

	mutex_lock(&g_mutex);
	timeout_release_cpufreq_limit = false;
	cancel_delayed_work_sync(&freq_qos_req_reset_work);
	ret = set_cpu_max_freq(page, ret);
	schedule_delayed_work(&freq_qos_req_reset_work, msecs_to_jiffies(FREQ_QOS_REQ_MAX_MS));
	mutex_unlock(&g_mutex);

	return ret;
}

static int cpu_max_freq_show(struct seq_file *m, void *v)
{
	int cpu;

	mutex_lock(&g_mutex);
	for_each_present_cpu(cpu) {
		if (disable_cpufreq_limit)
			seq_printf(m, "%d:%u ", cpu, FREQ_QOS_MAX_DEFAULT_VALUE);
		else
			seq_printf(m, "%d:%u ", cpu, per_cpu(game_cpu_stats, cpu).max);
	}
	seq_printf(m, "\n");
	mutex_unlock(&g_mutex);

	return 0;
}

static int cpu_max_freq_proc_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, cpu_max_freq_show, inode);
}

static const struct proc_ops cpu_max_freq_proc_ops = {
	.proc_open		= cpu_max_freq_proc_open,
	.proc_write 	= cpu_max_freq_proc_write,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release	= single_release,
};

static ssize_t disable_cpufreq_limit_proc_write(struct file *file,
	const char __user *buf, size_t count, loff_t *ppos)
{
	char page[32] = {0};
	int ret;
	int disable;

	ret = simple_write_to_buffer(page, sizeof(page) - 1, ppos, buf, count);
	if (ret <= 0)
		return ret;

	ret = sscanf(page, "%d", &disable);
	if (ret != 1)
		return -EINVAL;

	if (disable != 0 && disable != 1)
		return -EINVAL;

	mutex_lock(&g_mutex);

	if (disable_cpufreq_limit == disable)
		goto unlock;

	disable_cpufreq_limit = disable;

	if (timeout_release_cpufreq_limit)
		goto unlock;

	if (disable_cpufreq_limit)
		__freq_qos_request_reset();
	else
		__freq_qos_request_restore();

unlock:
	mutex_unlock(&g_mutex);
	return count;
}

static ssize_t disable_cpufreq_limit_proc_read(struct file *file,
	char __user *buf, size_t count, loff_t *ppos)
{
	char page[32] = {0};
	int len;

	mutex_lock(&g_mutex);
	len = sprintf(page, "%d\n", disable_cpufreq_limit);
	mutex_unlock(&g_mutex);

	return simple_read_from_buffer(buf, count, ppos, page, len);
}

static const struct proc_ops disable_cpufreq_limit_proc_ops = {
	.proc_write		= disable_cpufreq_limit_proc_write,
	.proc_read		= disable_cpufreq_limit_proc_read,
	.proc_lseek		= default_llseek,
};

int cpufreq_limits_init(void)
{
	int ret;

	if (unlikely(!game_opt_dir))
		return -ENOTDIR;

	if (!alloc_cpumask_var(&limit_mask_min, GFP_KERNEL))
		return -ENOMEM;

	if (!alloc_cpumask_var(&limit_mask_max, GFP_KERNEL)) {
		free_cpumask_var(limit_mask_min);
		return -ENOMEM;
	}

	ret = freq_qos_request_init();
	if (ret) {
		pr_err("%s: Failed to init qos requests policy for ret=%d\n",
			__func__, ret);
		return ret;
	}

	INIT_DELAYED_WORK(&freq_qos_req_reset_work, freq_qos_request_reset);

	proc_create_data("cpu_min_freq", 0664, game_opt_dir, &cpu_min_freq_proc_ops, NULL);
	proc_create_data("cpu_max_freq", 0664, game_opt_dir, &cpu_max_freq_proc_ops, NULL);
	proc_create_data("disable_cpufreq_limit", 0664, game_opt_dir, &disable_cpufreq_limit_proc_ops, NULL);

	return 0;
}
