#include <trace/hooks/cpufreq.h>
#include "game_ctrl.h"

static atomic_t fake_cpu7_cpuinfo_max_freq = ATOMIC_INIT(0);

static void show_cpuinfo_max_freq_hook(void *data,
	struct cpufreq_policy *policy, unsigned int *max_freq)
{
	struct cpufreq_policy *bpolicy;

	if ((!atomic_read(&fake_cpu7_cpuinfo_max_freq)) || (policy->cpu != 7))
		return;

	/*
	 * when fake_cpu7_cpuinfo_max_freq == 1,
	 * return cpu7 cpuinfo_max_freq with cpu4 cpuinfo_max_freq
	 */
	bpolicy = cpufreq_cpu_get(4);
	*max_freq = bpolicy->cpuinfo.max_freq;
	cpufreq_cpu_put(bpolicy);
}

static ssize_t fake_cpu7_cpuinfo_max_freq_proc_write(struct file *file,
			const char __user *buf, size_t count, loff_t *ppos)
{
	char page[32] = {0};
	int ret;
	int fake;

	ret = simple_write_to_buffer(page, sizeof(page) - 1, ppos, buf, count);
	if (ret <= 0)
		return ret;

	ret = sscanf(page, "%d", &fake);
	if (ret != 1)
		return -EINVAL;

	if (fake != 0 && fake != 1)
		return -EINVAL;

	if (atomic_read(&fake_cpu7_cpuinfo_max_freq) == fake)
		return count;

	atomic_set(&fake_cpu7_cpuinfo_max_freq, fake);

	return count;
}

static ssize_t fake_cpu7_cpuinfo_max_freq_proc_read(struct file *file,
			char __user *buf, size_t count, loff_t *ppos)
{
	char page[32] = {0};
	int fake, len;

	fake = atomic_read(&fake_cpu7_cpuinfo_max_freq);
	len = sprintf(page, "%d\n", fake);

	return simple_read_from_buffer(buf, count, ppos, page, len);
}

static const struct proc_ops fake_cpu7_cpuinfo_max_freq_proc_ops = {
	.proc_write		= fake_cpu7_cpuinfo_max_freq_proc_write,
	.proc_read		= fake_cpu7_cpuinfo_max_freq_proc_read,
	.proc_lseek		= default_llseek,
};

int fake_cpufreq_init(void)
{
	if (unlikely(!game_opt_dir))
		return -ENOTDIR;

	proc_create_data("fake_cpu7_cpuinfo_max_freq", 0664, game_opt_dir,
			&fake_cpu7_cpuinfo_max_freq_proc_ops, NULL);

	register_trace_android_rvh_show_max_freq(show_cpuinfo_max_freq_hook, NULL);

	return 0;
}
