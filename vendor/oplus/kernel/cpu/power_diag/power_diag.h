/*
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */
#define MAX_CTP_WINDOW_TICK (10 * NSEC_PER_SEC / TICK_NSEC)

#define power_debug_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0644,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

extern int mod_timer(struct timer_list *timer, unsigned long expires);
extern void pm_get_active_wakeup_sources(char *pending_sources, size_t max);
extern void cmdq_dump_usage(void);

struct kthread_cp {
	unsigned long flags;
	unsigned int cpu;
	int (*threadfn)(void *);
	void *data;
	mm_segment_t oldfs;
	struct completion parked;
	struct completion exited;
#ifdef CONFIG_BLK_CGROUP
	struct cgroup_subsys_state *blkcg_css;
#endif
};

struct kworker_stat {
	bool kworker;
	work_func_t	current_func;
	work_func_t	last_func;
};

struct task_stat {
	pid_t pid;
	pid_t tgid;
	unsigned int r_time;
	unsigned int pwr;
	unsigned int lcore_pwr;
	struct kworker_stat worker;
	char comm[TASK_COMM_LEN];
};

struct task_cpustat_cp {
	pid_t pid;
	pid_t tgid;
	enum cpu_usage_stat type;
	bool l_core;
	int freq;
	struct kworker_stat worker;
	unsigned long begin;
	unsigned long end;
	char comm[TASK_COMM_LEN];
};
struct kernel_task_cpustat_cp {
	unsigned int idx;
	struct task_cpustat_cp cpustat[MAX_CTP_WINDOW_TICK];
};

struct power_debug {
	struct notifier_block 	debug_load_pm_event;
	struct timer_list 		debug_power_timer;
};
