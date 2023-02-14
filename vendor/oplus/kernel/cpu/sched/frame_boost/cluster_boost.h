#ifndef _CLUSTER_BOOST_H
#define _CLUSTER_BOOST_H
int __fbg_set_task_preferred_cluster(pid_t tid, int cluster_id);
bool fbg_cluster_boost(struct task_struct *p, int *target_cpu);
inline unsigned long capacity_of(int cpu);
unsigned long cpu_util_without(int cpu, struct task_struct *p);
#endif
