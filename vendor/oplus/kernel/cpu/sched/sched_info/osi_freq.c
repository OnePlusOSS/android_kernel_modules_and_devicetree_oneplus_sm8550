// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Oplus. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/pm_opp.h>
#include <linux/completion.h>
#include <uapi/linux/sched/types.h>
#include <trace/hooks/cpufreq.h>

#include "osi_topology.h"
#include "osi_freq.h"
#include "osi_base.h"

struct cur_freq {
	u64 last_update_time;
	u32 freq[JANK_WIN_CNT];
};

struct count {
	u32 cnt;
	u64 timestamp;
};

struct freq_cnt {
	u64 last_update_time;
	struct count increase[JANK_WIN_CNT];
	struct count clamp[JANK_WIN_CNT];
};

struct cur_freq cur_freq[CPU_NUMS];
struct freq_cnt freq_cnt[CPU_NUMS];

void jank_currfreq_update_win(u64 now)
{
	u64 last_update_time;
	u32 idx, now_idx, last_idx;
	u32 freq;
	u32 win_cnt, i;
	u32 cpu;

	jank_dbg("e0\n");
	jank_systrace_print_idx(JANK_SYSTRACE_CPUFREQ, "0update-cur", 0, 1);

	now_idx = time2winidx(now);

	for_each_possible_cpu(cpu) {
		last_update_time = cur_freq[cpu].last_update_time;

		if (now < last_update_time) {
			cur_freq[cpu].last_update_time = now;
			continue;
		}

		win_cnt = time2wincnt(now, last_update_time);
		last_idx = time2winidx(last_update_time);
		freq = cur_freq[cpu].freq[last_idx];

		for (i = 0; i < win_cnt; i++) {
			idx = winidx_sub(now_idx, i);
			if (i == 0)
				freq = cpufreq_quick_get(cpu);

			cur_freq[cpu].freq[idx] = freq;

			jank_dbg("e1: cpu=%d, now_idx=%d, last_idx=%d, win_cnt=%d, "
						"idx=%d, i=%d, freq=%d\n",
						cpu, now_idx, last_idx, win_cnt,
						idx, i, freq);
		}
		cur_freq[cpu].last_update_time = now;
	}
}

static inline int cpufreq_table_find_index(struct cpufreq_policy *policy,
					     unsigned int target_freq)
{
	if (policy->freq_table_sorted == CPUFREQ_TABLE_SORTED_ASCENDING)
		return cpufreq_table_find_index_al(policy, target_freq);
	else
		return cpufreq_table_find_index_dl(policy, target_freq);
}

void get_cpufreq_info(bool *is_sample)
{
	int cls;
	int start_cpu;
	int cur_idx, max_idx, min_idx, orig_max_id, orig_min_id;
	struct cpufreq_policy *pol;

	for (cls = 0; cls < cluster_num; cls++) {
		start_cpu = cluster[cls].start_cpu;
		pol = cpufreq_cpu_get(start_cpu);
		if (likely(pol)) {
			cur_idx = cpufreq_table_find_index(pol, pol->cur);
			min_idx = cpufreq_table_find_index(pol, pol->min);
			max_idx = cpufreq_table_find_index(pol, pol->max);
			orig_max_id = cpufreq_table_find_index(pol, pol->cpuinfo.max_freq);
			orig_min_id = cpufreq_table_find_index(pol, pol->cpuinfo.min_freq);
			osi_debug("cpu:%d,pol->cur:%d,idx:%d min_idx:%d,max_idx:%d,orig_min_idx:%d,orig_max_idx:%d",
				start_cpu, pol->cur, cur_idx, min_idx, max_idx, orig_min_id, orig_max_id);
#ifdef CONFIG_ARCH_MEDIATEK
			if ((max_idx < orig_min_id - 2) && (cur_idx <= min_idx - 2))
				*is_sample = true;
#else
			if ((max_idx > 2) && (cur_idx >= max_idx - 2))
				*is_sample = true;
#endif
			cpufreq_cpu_put(pol);
		}
	}
}

void jank_curr_freq_show(struct seq_file *m, u32 win_idx, u64 now)
{
	u32 cls;
	u32 start_cpu;
	u32 now_index, idx;


	now_index = time2winidx(now);
	idx = winidx_sub(now_index, win_idx);

	for (cls = 0; cls < cluster_num; cls++) {
		start_cpu = cluster[cls].start_cpu;
		seq_printf(m, "%d ", cur_freq[start_cpu].freq[idx]);

		jank_dbg("e3: cls=%d, start_cpu=%d, "
				"now_index=%d, win_idx=%d, idx=%d "
				"freq=%d\n",
				cls, start_cpu,
				now_index, win_idx, idx,
				cur_freq[start_cpu].freq[idx]);
	}
}

/* target_freq > policy->max */
static void update_freq_count(struct cpufreq_policy *policy,
			u32 old_freq, u32 new_freq, u32 flags)
{
	u64 last_update_time, now, delta, timestamp;
	u32 i, now_idx, win_cnt, idx;
	u32 cpu;
	bool freq_increase, freq_clamp;
	u32 target_freq;
	int index;

	if (!policy)
		return;

	index = policy->cached_resolved_idx;
	if (index < 0)
		return;

	target_freq = policy->freq_table[index].frequency;

	freq_increase = target_freq > policy->cur;
	freq_clamp = new_freq < old_freq;

#ifdef JANKINFO_DEBUG
	if (policy->cpu == 0) {
		jank_systrace_print_idx(JANK_SYSTRACE_CPUFREQ,
			"0flags", 0, flags);
		jank_systrace_print_idx(JANK_SYSTRACE_CPUFREQ,
			"0increase", 0, freq_increase);
		jank_systrace_print_idx(JANK_SYSTRACE_CPUFREQ,
			"1clamp", 0, freq_clamp);
		jank_systrace_print_idx(JANK_SYSTRACE_CPUFREQ,
			"2cpu", 0, policy->cpu);
		jank_systrace_print_idx(JANK_SYSTRACE_CPUFREQ,
			"3cur", 0, policy->cur);
		jank_systrace_print_idx(JANK_SYSTRACE_CPUFREQ,
			"4min", 0, policy->min);
		jank_systrace_print_idx(JANK_SYSTRACE_CPUFREQ,
			"5max", 0, policy->max);
		jank_systrace_print_idx(JANK_SYSTRACE_CPUFREQ,
			"6target_freq", 0, target_freq);
		jank_systrace_print_idx(JANK_SYSTRACE_CPUFREQ,
			"7new_freq", 0, new_freq);
		jank_systrace_print_idx(JANK_SYSTRACE_CPUFREQ,
			"8old_freq", 0, old_freq);
	}
#endif

	if (!freq_increase  && !freq_clamp)
		return;

	cpu = policy->cpu;

	last_update_time = freq_cnt[cpu].last_update_time;

	/*
	 * Note: the jiffies value will overflow 5 minutes after boot
	 */
	now = jiffies_to_nsecs(jiffies);
	if (unlikely(now < last_update_time)) {
		freq_cnt[cpu].last_update_time = now;
		delta = 0;
	} else {
		delta = now - last_update_time;
	}

	if (!delta)
		return;

	now_idx = time2winidx(now);
	win_cnt = time2wincnt(now, last_update_time) + 1;

	jank_dbg("m1: cpu=%d, old_freq=%d, new_freq=%d "
		"cur=%d, min=%d, max=%d "
		"freq_increase=%d, freq_clamp=%d "
		"now_idx=%d, last_idx=%d, win_cnt=%d\n",
		cpu, old_freq, new_freq,
		policy->cur, policy->min, policy->max,
		freq_increase, freq_clamp,
		now_idx, time2winidx(last_update_time), win_cnt);

	for (i = 0; i < win_cnt; i++) {
		idx = winidx_sub(now_idx, i);

		if (freq_increase) {
			timestamp = freq_cnt[cpu].increase[idx].timestamp;

			if (policy->cpu == 0) {
				jank_systrace_print_idx(JANK_SYSTRACE_CPUFREQ,
					"1win_cnt", 0, win_cnt);
				jank_systrace_print_idx(JANK_SYSTRACE_CPUFREQ,
					"2now_idx", 0, now_idx);
				jank_systrace_print_idx(JANK_SYSTRACE_CPUFREQ,
					"3idx", 0, idx);
				jank_systrace_print_idx(JANK_SYSTRACE_CPUFREQ,
					"4valid", 0, timestamp_is_valid(timestamp, now));
				jank_systrace_print_idx(JANK_SYSTRACE_CPUFREQ,
					"5cnt", 0, freq_cnt[cpu].increase[idx].cnt);
			}

			jank_dbg("m2: cpu=%d, i=%d, idx=%d, now_idx=%d, "
						"valid=%d, cnt=%d\n",
					cpu, i, idx, now_idx,
					timestamp_is_valid(timestamp, now),
					freq_cnt[cpu].clamp[idx].cnt);

			if (timestamp_is_valid(timestamp, now)) {
				if (i == 0) {
					freq_cnt[cpu].increase[idx].cnt++;
					freq_cnt[cpu].increase[idx].timestamp =  now;
				}
			} else {
				if (i == 0)
					freq_cnt[cpu].increase[idx].cnt = 1;
				else
					freq_cnt[cpu].increase[idx].cnt = 0;

				freq_cnt[cpu].increase[idx].timestamp =  now;
			}

			if (policy->cpu == 0) {
				jank_systrace_print_idx(JANK_SYSTRACE_CPUFREQ,
					"6cnt", 0, freq_cnt[cpu].increase[idx].cnt);
			}
		}

		if (freq_clamp) {
			timestamp = freq_cnt[cpu].clamp[idx].timestamp;

			jank_dbg("m3: cpu=%d, i=%d, idx=%d, now_idx=%d, "
					"valid=%d, cnt=%d\n",
					cpu, i, idx, now_idx,
					timestamp_is_valid(timestamp, now),
					freq_cnt[cpu].clamp[idx].cnt);

			if (timestamp_is_valid(timestamp, now)) {
				if (i == 0) {
					freq_cnt[cpu].clamp[idx].cnt++;
					freq_cnt[cpu].clamp[idx].timestamp =  now;
				}
			} else {
				if (i == 0)
					freq_cnt[cpu].clamp[idx].cnt = 1;
				else
					freq_cnt[cpu].clamp[idx].cnt = 0;

				freq_cnt[cpu].clamp[idx].timestamp =  now;
			}
		}
	}
	freq_cnt[cpu].last_update_time = now;
}

void jankinfo_update_freq_reach_limit_count(
			struct cpufreq_policy *policy,
			u32 old_target_freq, u32 new_target_freq, u32 flags)
{
	update_freq_count(policy, old_target_freq, new_target_freq, flags);
}

/* target_freq > policy->max */
void jank_burst_freq_show(struct seq_file *m, u32 win_idx, u64 now)
{
	u64 timestamp;
	u32 tmp_cnt;
	u32 now_index, idx, i;
	struct cpufreq_policy *policy;

	now_index = time2winidx(now);
	idx = winidx_sub(now_index, win_idx);

	for_each_possible_cpu(i) {
		policy = cpufreq_cpu_get_raw(i);
		if (!policy)
			continue;

		if (policy->cpu != i)
			continue;

		timestamp = freq_cnt[i].clamp[idx].timestamp;
		if (!timestamp_is_valid(timestamp, now)) {
			seq_printf(m, "%d ", 0);
		} else {
			tmp_cnt = freq_cnt[i].clamp[idx].cnt;
			seq_printf(m, "%d ", tmp_cnt);
		}
	}
}

/* target_freq > policy->cur */
void jank_increase_freq_show(struct seq_file *m,
			u32 win_idx, u64 now)
{
	u32 tmp_cnt;
	u64 timestamp;
	u32 now_index, idx, i;
	struct cpufreq_policy *policy;

	now_index = time2winidx(now);
	idx = winidx_sub(now_index, win_idx);

	for_each_possible_cpu(i) {
		policy = cpufreq_cpu_get_raw(i);
		if (!policy)
			continue;

		if (policy->cpu != i)
			continue;

		timestamp = freq_cnt[i].increase[idx].timestamp;
		if (!timestamp_is_valid(timestamp, now)) {
			seq_printf(m, "%d ", 0);
		} else {
			tmp_cnt = freq_cnt[i].increase[idx].cnt;
			seq_printf(m, "%d ", tmp_cnt);
		}
	}
}

struct freq_duration {
	u8  opp_num;
	u8  prev_index;
	u32 prev_freq;
	u32 *freq_table;
	u64 *duration_table;
	ktime_t prev_timestamp;
} freq_duration;

static DEFINE_PER_CPU(struct freq_duration, freq_duration_info);
static DEFINE_PER_CPU(int, opp_num);

void osi_cpufreq_transition_handler(void *unused, struct cpufreq_policy *policy)
{
	u8 cpu, idx, prev_index;
	u32 new_freq;
	ktime_t now;
	u64 time_delta;
	struct freq_duration *freq_duration;
	now = ktime_get();
	new_freq = policy->cur;
	cpu = cpumask_first(policy->related_cpus);

	freq_duration = per_cpu_ptr(&freq_duration_info, cpu);
	idx = cpufreq_table_find_index(policy, new_freq);
	prev_index = freq_duration->prev_index;
	if (unlikely((prev_index == idx) || idx >= per_cpu(opp_num, cpu)))
		return;
	freq_duration->freq_table[idx] = new_freq;
	time_delta = ktime_us_delta(now, freq_duration->prev_timestamp);
	freq_duration->duration_table[prev_index] = max(time_delta, freq_duration->duration_table[prev_index]);
	freq_duration->prev_timestamp = now;
	freq_duration->prev_freq = new_freq;
	freq_duration->prev_index = idx;
}

static void osi_opp_init(void)
{
	int  i, nr_caps, opp_idx;
	struct freq_duration *freq_duration;
	struct cpufreq_policy *pol;
	struct em_perf_domain *pd;

	for_each_possible_cpu(i) {
		struct device *cpu_dev = get_cpu_device(i);
		if (!cpu_dev)
			return;
		pol = cpufreq_cpu_get(i);
		pd = em_cpu_get(i);
		if (!pd || !pol) {
			pr_info("cpu_get return NULL for cpu#%d", i);
			continue;
		}
		nr_caps = pd->nr_perf_states;
		per_cpu(opp_num, i) = nr_caps;
		opp_idx = dev_pm_opp_get_opp_count(cpu_dev);
		pr_info("opp_num:%d, opp_idx:%d,i:%d", nr_caps, opp_idx, i);
		freq_duration = per_cpu_ptr(&freq_duration_info, i);
		freq_duration->opp_num = nr_caps;
		freq_duration->freq_table = kcalloc(nr_caps, sizeof(u32), GFP_KERNEL);
		freq_duration->duration_table = kcalloc(nr_caps, sizeof(u64), GFP_KERNEL);
		freq_duration->prev_timestamp = ktime_get();
		freq_duration->prev_index = cpufreq_table_find_index(pol, pol->cur);
		cpufreq_cpu_put(pol);
	}
}

static void osi_opp_exit(void)
{
	int  i;
	struct freq_duration *freq_duration;

	for_each_possible_cpu(i) {
		struct device *cpu_dev = get_cpu_device(i);
		if (!cpu_dev)
			return;
		freq_duration = per_cpu_ptr(&freq_duration_info, i);
		kfree(freq_duration->freq_table);
		kfree(freq_duration->duration_table);
	}
}

static void clear_freq_duration(struct task_struct *p)
{
	struct freq_duration *freq_duration;
	int im_flag = IM_FLAG_NONE;
	int cpu;

	if (p && p->group_leader)
		im_flag = oplus_get_im_flag(p->group_leader);
	if (im_flag == IM_FLAG_MIDASD) {
		pr_info("clear_freq_duration, %d, %s", p->pid, p->comm);
		for_each_possible_cpu(cpu) {
			if (!is_cluster_cpu(cpu))
				continue;
			freq_duration = per_cpu_ptr(&freq_duration_info, cpu);
			memset(freq_duration->duration_table, 0, freq_duration->opp_num);
		}
	}
}

static int proc_long_freq_duration_show(struct seq_file *m, void *v)
{
	int cpu, i;
	struct freq_duration *freq_duration;

	for_each_possible_cpu(cpu) {
		if (!is_cluster_cpu(cpu))
			continue;
		freq_duration = per_cpu_ptr(&freq_duration_info, cpu);
		seq_printf(m, "cpu(%d), %7s, %10s\n", cpu, "opp", "duration");
		for (i = 0; i < freq_duration->opp_num; i++) {
			seq_printf(m, "%6d, %7u, %10lu\n", i, freq_duration->freq_table[i],
				freq_duration->duration_table[i] >> 10);
		}
		seq_printf(m, "\n");
	}
	clear_freq_duration(current);
	return 0;
}

static int proc_long_freq_duration_open(struct inode *inode,
		struct file *file)
{
	return single_open(file, proc_long_freq_duration_show, inode);
}

static const struct proc_ops proc_long_freq_duration_operations = {
	.proc_open	=	proc_long_freq_duration_open,
	.proc_read	=	seq_read,
	.proc_lseek	=	seq_lseek,
	.proc_release   =	single_release,
};

static void osi_freq_proc_init(struct proc_dir_entry *pde)
{
	struct proc_dir_entry *entry = NULL;

	entry = proc_create("long_freq_duration", S_IRUGO | S_IWUGO,
				pde, &proc_long_freq_duration_operations);
	if (!entry) {
		osi_err("create long_freq_duration fail\n");
		return;
	}
}

static void osi_freq_proc_deinit(struct proc_dir_entry *pde)
{
	remove_proc_entry("long_freq_duration", pde);
}

int osi_freq_init(struct proc_dir_entry *pde)
{
	int ret = 0;
	osi_opp_init();
	osi_freq_proc_init(pde);
	REGISTER_TRACE_RVH(android_rvh_cpufreq_transition, osi_cpufreq_transition_handler);
	return ret;
}

void osi_freq_exit(struct proc_dir_entry *pde)
{
	osi_opp_exit();
	osi_freq_proc_deinit(pde);
}
