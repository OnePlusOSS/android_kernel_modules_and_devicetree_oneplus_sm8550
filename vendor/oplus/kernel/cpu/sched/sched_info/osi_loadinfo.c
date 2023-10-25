// SPDX-License-Identifier: GPL-2.0
#include <linux/cpumask.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel_stat.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/sched/stat.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/irqnr.h>
#include <linux/sched/cputime.h>
#include <linux/tick.h>
#include "osi_healthinfo.h"


#ifndef arch_irq_stat_cpu
#define arch_irq_stat_cpu(cpu) 0
#endif
#ifndef arch_irq_stat
#define arch_irq_stat() 0
#endif

#ifdef arch_idle_time

static u64 oplus_get_idle_time(struct kernel_cpustat *kcs, int cpu)
{
	u64 idle;

	idle = kcs->cpustat[CPUTIME_IDLE];
	if (cpu_online(cpu) && !nr_iowait_cpu(cpu))
		idle += arch_idle_time(cpu);
	return idle;
}

static u64 get_iowait_time(struct kernel_cpustat *kcs, int cpu)
{
	u64 iowait;

	iowait = kcs->cpustat[CPUTIME_IOWAIT];
	if (cpu_online(cpu) && nr_iowait_cpu(cpu))
		iowait += arch_idle_time(cpu);
	return iowait;
}

#else

static u64 oplus_get_idle_time(struct kernel_cpustat *kcs, int cpu)
{
	u64 idle, idle_usecs = -1ULL;

	if (cpu_online(cpu))
		idle_usecs = get_cpu_idle_time_us(cpu, NULL);

	if (idle_usecs == -1ULL)
		/* !NO_HZ or cpu offline so we can rely on cpustat.idle */
		idle = kcs->cpustat[CPUTIME_IDLE];
	else
		idle = idle_usecs * NSEC_PER_USEC;

	return idle;
}

static u64 get_iowait_time(struct kernel_cpustat *kcs, int cpu)
{
	u64 iowait, iowait_usecs = -1ULL;

	if (cpu_online(cpu))
		iowait_usecs = get_cpu_iowait_time_us(cpu, NULL);

	if (iowait_usecs == -1ULL)
		/* !NO_HZ or cpu offline so we can rely on cpustat.iowait */
		iowait = kcs->cpustat[CPUTIME_IOWAIT];
	else
		iowait = iowait_usecs * NSEC_PER_USEC;

	return iowait;
}

#endif

struct cpu_load_stat {
	u64 t_user;
	u64 t_system;
	u64 t_idle;
	u64 t_iowait;
	u64 t_irq;
	u64 t_softirq;
};


int ohm_get_cur_cpuload(bool ctrl)
{
	int i;
	struct cpu_load_stat cpu_load = { 0, 0, 0, 0, 0, 0};
	struct cpu_load_stat cpu_load_temp = { 0, 0, 0, 0, 0, 0};
	clock_t ct_user, ct_system, ct_idle, ct_iowait, ct_irq, ct_softirq, load, sum = 0;

	if (!ctrl)
		return 0;

	for_each_online_cpu(i) {
		struct kernel_cpustat *kcs = &kcpustat_cpu(i);

		cpu_load_temp.t_user += kcpustat_cpu(i).cpustat[CPUTIME_USER];
		cpu_load_temp.t_system += kcpustat_cpu(i).cpustat[CPUTIME_SYSTEM];
		cpu_load_temp.t_idle += oplus_get_idle_time(kcs, i);
		cpu_load_temp.t_iowait += get_iowait_time(kcs, i);
		cpu_load_temp.t_irq += kcpustat_cpu(i).cpustat[CPUTIME_IRQ];
		cpu_load_temp.t_softirq += kcpustat_cpu(i).cpustat[CPUTIME_SOFTIRQ];
	}
	msleep(25);
	for_each_online_cpu(i) {
		struct kernel_cpustat *kcs = &kcpustat_cpu(i);

		cpu_load.t_user += kcpustat_cpu(i).cpustat[CPUTIME_USER];
		cpu_load.t_system += kcpustat_cpu(i).cpustat[CPUTIME_SYSTEM];
		cpu_load.t_idle += oplus_get_idle_time(kcs, i);
		cpu_load.t_iowait += get_iowait_time(kcs, i);
		cpu_load.t_irq += kcpustat_cpu(i).cpustat[CPUTIME_IRQ];
		cpu_load.t_softirq += kcpustat_cpu(i).cpustat[CPUTIME_SOFTIRQ];
	}

	ct_user = nsec_to_clock_t(cpu_load.t_user) - nsec_to_clock_t(cpu_load_temp.t_user);
	ct_system = nsec_to_clock_t(cpu_load.t_system) - nsec_to_clock_t(cpu_load_temp.t_system);
	ct_idle = nsec_to_clock_t(cpu_load.t_idle) - nsec_to_clock_t(cpu_load_temp.t_idle);
	ct_iowait = nsec_to_clock_t(cpu_load.t_iowait) - nsec_to_clock_t(cpu_load_temp.t_iowait);
	ct_irq = nsec_to_clock_t(cpu_load.t_irq) - nsec_to_clock_t(cpu_load_temp.t_irq);
	ct_softirq = nsec_to_clock_t(cpu_load.t_softirq) - nsec_to_clock_t(cpu_load_temp.t_softirq);

	sum = ct_user + ct_system + ct_idle + ct_iowait + ct_irq + ct_softirq;
	load = ct_user + ct_system + ct_iowait + ct_irq + ct_softirq;

	if (sum == 0)
		return 0;

	return 100 * load / sum;
}
EXPORT_SYMBOL(ohm_get_cur_cpuload);
