// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "fqm_monitor: " fmt

#include <linux/module.h>
#include <linux/pm_qos.h>
#include <linux/cpumask.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/cpufreq.h>
#include <linux/rwlock.h>
#include <linux/spinlock.h>
#include <trace/hooks/power.h>

#include "fqm_sysfs.h"

#define FREQ_REQUEST_TIMEOUT_DEFAULE (180 * MSEC_PER_SEC)
#define FQM_PERIOD (180)

bool fqm_cpufreq_policy_ready;
int default_fqm_threshold;
int max_cluster_num;
int threshold[5];

struct freq_request_monitor {
	struct list_head node;
	struct freq_qos_request *req;
	int pid;
	u64 last_update_time;
	unsigned long callstack[4];
};

static DEFINE_RWLOCK(moni_rwlock);
static struct freq_request_monitor monitors;

static struct workqueue_struct *monitor_work;
static struct delayed_work monitor_delay_work;

static int tracing_mark_write(const char *buf)
{
	trace_printk(buf);
	return 0;
}

void freq_qos_req_systrace_c(int type, int new_value)
{
	char buf[256] = {0};
	bool is_min = true;

	if (type == FREQ_QOS_MAX)
		is_min = false;

	snprintf(buf, sizeof(buf), "C|8888|%s_freq_qos|%d\n", is_min ? "min" : "max", new_value);

	tracing_mark_write(buf);
}

static inline bool valid_cluster_id(unsigned int cluster_id)
{
	return (cluster_id >= 0) && (cluster_id < max_cluster_num);
}

void fqm_set_threshold(int value, unsigned int cluster_id)
{
	unsigned long flags;

	if (!valid_cluster_id(cluster_id))
		return;

	write_lock_irqsave(&moni_rwlock, flags);
	threshold[cluster_id] = value;
	write_unlock_irqrestore(&moni_rwlock, flags);
}

int fqm_get_threshold(unsigned int cluster_id)
{
	int thres;
	unsigned long flags;

	if (!valid_cluster_id(cluster_id))
		return 0;

	read_lock_irqsave(&moni_rwlock, flags);
	thres = threshold[cluster_id];
	read_unlock_irqrestore(&moni_rwlock, flags);

	return thres;
}

void freq_qos_add_request_handler(void *unused, struct freq_constraints *qos,
				struct freq_qos_request *req, enum freq_qos_req_type type,
				int value, int ret)
{
	unsigned long flags;
	struct freq_request_monitor *new_moni = kzalloc(sizeof(struct freq_request_monitor), GFP_ATOMIC);
	struct task_struct *task = current;
	u64 now = ktime_to_ms(ktime_get());

	if (!new_moni) {
		pr_err("Failed to allocate new_moni\n");
		return;
	}

	req->android_oem_data1[0]= (u64)new_moni;

	write_lock_irqsave(&moni_rwlock, flags);
	list_add_tail(&(new_moni->node), &(monitors.node));
	new_moni->req = req;
	new_moni->pid = task->pid;
	new_moni->last_update_time = now;
	new_moni->callstack[0] = (unsigned long)__builtin_return_address(0);
	new_moni->callstack[1] = (unsigned long)__builtin_return_address(1);
	new_moni->callstack[2] = (unsigned long)__builtin_return_address(2);
	new_moni->callstack[3] = (unsigned long)__builtin_return_address(3);
	write_unlock_irqrestore(&moni_rwlock, flags);

	queue_delayed_work(monitor_work, &monitor_delay_work, FQM_PERIOD * HZ);

	if (unlikely(g_fqm_debug_enable)) {
		trace_printk("%s: comm=%s type=%d value=%d ret=%d\n", __func__, task->comm, type, value, ret);
		pr_info("new_moni add! request_from pid=%d comm=%s last_update_time=%llu callstack:(%ps<-%ps<-%ps<-%ps)\n",
		task->pid, task->comm, new_moni->last_update_time, new_moni->callstack[0], new_moni->callstack[1],
		new_moni->callstack[2], new_moni->callstack[3]);
	}
}

void freq_qos_remove_request_handler(void *unused, struct freq_qos_request *req)
{
	struct freq_constraints *qos = req->qos;
	int type = req->type;
	struct freq_request_monitor *monitor, *temp;
	unsigned long flags;
	bool found = false;

	write_lock_irqsave(&moni_rwlock, flags);
	list_for_each_entry_safe(monitor, temp, &(monitors.node), node) {
		if (monitor->req == req) {
			found = true;
			list_del(&(monitor->node));
			req->android_oem_data1[0] = 0;
			kfree(monitor);
			break;
		}
	}
	write_unlock_irqrestore(&moni_rwlock, flags);

	if (unlikely(g_fqm_debug_enable)) {
		if (found) {
			if (type == FREQ_QOS_MIN)
				pr_info("%s: comm=%s type=%d value=%d\n", __func__, current->comm, type, qos->min_freq.target_value);
			else if (type == FREQ_QOS_MAX)
				pr_info("%s: comm=%s type=%d value=%d\n", __func__, current->comm, type, qos->max_freq.target_value);
		} else {
			pr_warn("req not found in monitors! comm=%s type=%d\n", current->comm, type);
		}
	}
}

static void freqqos_min_release(struct freq_qos_request *req)
{
	struct freq_constraints *qos;
	struct cpufreq_policy *policy;
	int cpu;
	int cluster_id;
	unsigned int min_freq;

	if (unlikely(!g_fqm_monitor_enable))
		return;

	if (unlikely(!fqm_cpufreq_policy_ready))
		return;

	qos = req->qos;
	if (qos == NULL)
		return;

	policy = container_of(qos, struct cpufreq_policy, constraints);

	cpu = policy->cpu;
	cluster_id = topology_physical_package_id(cpu);
	min_freq = qos->min_freq.target_value;

	if (min_freq > threshold[cluster_id]) {
		freq_qos_update_request(req, FREQ_QOS_MIN_DEFAULT_VALUE);
		pr_info("%sFREQ_QOS_MIN lasted for too long, release min freq for cluster%d\n", __func__, cluster_id);
	}
}

static void fqm_delayed_work_handler(struct work_struct *work)
{
	struct freq_request_monitor *moni;
	u64 now = ktime_to_ms(ktime_get());
	bool need_show = false;
	unsigned long flags;

	if (unlikely(!g_fqm_monitor_enable))
		return;

	if (list_empty(&monitors.node))
		return;

	write_lock_irqsave(&moni_rwlock, flags);
	list_for_each_entry(moni, &monitors.node, node) {
		if ((now - moni->last_update_time) > FREQ_REQUEST_TIMEOUT_DEFAULE) {
			if (moni->req->type == FREQ_QOS_MIN) {
				freqqos_min_release(moni->req);
				pr_info("req_type=%d req_min_value=%d callstack:(%ps<-%ps<-%ps<-%ps)\n",
					moni->req->type, moni->req->qos->min_freq.target_value, moni->callstack[0],
					moni->callstack[1], moni->callstack[2], moni->callstack[3]);
			} else if (moni->req->type == FREQ_QOS_MAX) {
				/*freq_qos_update_request(moni->req, FREQ_QOS_MAX_DEFAULT_VALUE);*/
				pr_info("FREQ_QOS_MAX lasted for too long.\n");
				pr_info("req_type=%d req_max_value=%d callstack:(%ps<-%ps<-%ps<-%ps)\n",
					moni->req->type, moni->req->qos->max_freq.target_value, moni->callstack[0],
					moni->callstack[1], moni->callstack[2], moni->callstack[3]);
			}
			need_show = true;
		}
	}
	write_unlock_irqrestore(&moni_rwlock, flags);

	if (need_show) {
		read_lock_irqsave(&moni_rwlock, flags);
		list_for_each_entry(moni, &monitors.node, node) {
			pr_info("dump all request req_type=%d req_min_value=%d req_max_value=%d req_from_pid=%d"
					"last_update_time=%llu req_interval=%llu, callstack:(%ps<-%ps<-%ps<-%ps)\n",
				moni->req->type, moni->req->qos->min_freq.target_value, moni->req->qos->max_freq.target_value,
				moni->pid, moni->last_update_time, (now - moni->last_update_time),
				moni->callstack[0], moni->callstack[1], moni->callstack[2], moni->callstack[3]);
		}
		read_unlock_irqrestore(&moni_rwlock, flags);
	}
}

static void monitor_req_update(struct freq_qos_request *req)
{
	struct freq_request_monitor *monitor;
	u64 now = ktime_to_ms(ktime_get());

	if (req == NULL || list_empty(&monitors.node))
		return;

	list_for_each_entry(monitor, &(monitors.node), node) {
		if (monitor->req == req) {
			monitor->callstack[0] = (unsigned long)__builtin_return_address(0);
			monitor->callstack[1] = (unsigned long)__builtin_return_address(1);
			monitor->callstack[2] = (unsigned long)__builtin_return_address(2);
			monitor->callstack[3] = (unsigned long)__builtin_return_address(3);
			monitor->last_update_time = now;
			queue_delayed_work(monitor_work, &monitor_delay_work, FQM_PERIOD * HZ);
			break;
		}
	}
}

void freq_qos_update_request_handler(void *unused, struct freq_qos_request *req, int new_value)
{
	struct freq_constraints *qos = req->qos;
	int type = req->type;
	int old_value = 0;
	unsigned long flags;

	if (unlikely(!g_fqm_monitor_enable))
		return;

	/* FREQ_QOS_MIN = 1, FREQ_QOS_MAX = 2,*/
	if (type == FREQ_QOS_MIN)
		old_value = qos->min_freq.target_value;
	else if (type == FREQ_QOS_MAX)
		old_value = qos->max_freq.target_value;

	if (old_value == new_value)
		return;

	write_lock_irqsave(&moni_rwlock, flags);
	monitor_req_update(req);
	write_unlock_irqrestore(&moni_rwlock, flags);

	if (unlikely(g_fqm_debug_enable))
		freq_qos_req_systrace_c(type, new_value);
}

static int fqm_cpufreq_policy_notifier_callback(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_policy *policy = (struct cpufreq_policy *)data;

	if (IS_ERR_OR_NULL(policy)) {
		pr_err("%s:null cpu policy\n", __func__);
		return NOTIFY_DONE;
	}
	if (val != CPUFREQ_CREATE_POLICY)
		return NOTIFY_DONE;

	fqm_cpufreq_policy_ready = true;

	return NOTIFY_DONE;
}

static struct notifier_block fqm_cpufreq_policy_notifier = {
	.notifier_call = fqm_cpufreq_policy_notifier_callback,
};

static void fqm_cluster_init(void)
{
	unsigned int cpu, idx, i;
	int max_cluster_id = 0;

	for_each_possible_cpu(cpu) {
		idx = topology_physical_package_id(cpu);
		if (idx > max_cluster_id)
			max_cluster_id = idx;
	}

	max_cluster_num = max_cluster_id + 1;

	for (i = 0; i < max_cluster_num; i++)
		threshold[i] = default_fqm_threshold;

	pr_info("fqm_max_cluster_num_get! max_cluster_num = %d\n", max_cluster_num);
}

static void unregister_fqm_vendor_hooks(void)
{
	unregister_trace_android_vh_freq_qos_update_request(freq_qos_update_request_handler, NULL);
	unregister_trace_android_vh_freq_qos_add_request(freq_qos_add_request_handler, NULL);
	unregister_trace_android_vh_freq_qos_remove_request(freq_qos_remove_request_handler, NULL);
}

static int register_fqm_vendor_hooks(void)
{
	int ret = 0;

	/* register vendor hook in kernel/power/qos.c*/
	ret = register_trace_android_vh_freq_qos_update_request(freq_qos_update_request_handler, NULL);
	ret = register_trace_android_vh_freq_qos_add_request(freq_qos_add_request_handler, NULL);
	ret = register_trace_android_vh_freq_qos_remove_request(freq_qos_remove_request_handler, NULL);
	if (ret) {
		pr_err("fqm register vendor hooks failed!\n");
		return ret;
	}

	return 0;
}

static int __init oplus_freqqos_monitor_init(void)
{
	struct freq_request_monitor *moni = NULL;
	int ret = 0;

	moni = &monitors;
	INIT_LIST_HEAD(&moni->node);
	moni->req = NULL;
	moni->pid = 0;
	moni->last_update_time = 0;
	moni->callstack[0] = 0;
	moni->callstack[1] = 0;
	moni->callstack[2] = 0;
	moni->callstack[3] = 0;
	default_fqm_threshold = 4000000;

	fqm_cluster_init();

	ret = register_fqm_vendor_hooks();
	if (ret)
		return ret;

	monitor_work = create_workqueue("freqqos monitor task");
	if (!monitor_work) {
		pr_err("create workqueue failed!\n");
		return -EFAULT;
	}

	INIT_DELAYED_WORK(&monitor_delay_work, fqm_delayed_work_handler);

	ret = freqqos_monitor_proc_init();
	if (ret)
		return ret;

	ret = cpufreq_register_notifier(&fqm_cpufreq_policy_notifier, CPUFREQ_POLICY_NOTIFIER);
	if (ret)
		return ret;

	g_fqm_monitor_enable = 1;
	pr_info("oplus_freqqos_monitor init\n");

	return ret;
}

static void __exit oplus_freqqos_monitor_exit(void)
{
	unregister_fqm_vendor_hooks();

	freqqos_monitor_proc_exit();

	g_fqm_monitor_enable = 0;

	pr_info("oplus_freqqos_monitor exit\n");
}

module_init(oplus_freqqos_monitor_init);
module_exit(oplus_freqqos_monitor_exit);
MODULE_DESCRIPTION("OPLUS FREQQOS_MONITOR");
MODULE_LICENSE("GPL v2");
