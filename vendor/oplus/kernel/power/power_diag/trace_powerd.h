#undef TRACE_SYSTEM
#define TRACE_SYSTEM powerd

#if !defined(_TRACE_POWERD_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_POWERD_H
#include <linux/tracepoint.h>
#include <linux/sched.h>

TRACE_EVENT(sched_setuclamp,

    TP_PROTO(struct task_struct *p,
         unsigned int uclamp_val),

    TP_ARGS(p, uclamp_val),

    TP_STRUCT__entry(
        __array(    char,   comm,  TASK_COMM_LEN   )
        __field(    pid_t,  pid            )
        __field(    pid_t,  tgid            )
        __field(    unsigned int,  cpu            )
        __field(    unsigned int,    uclamp_val           )
    ),

    TP_fast_assign(
        memcpy(__entry->comm, p->comm, TASK_COMM_LEN);
        __entry->pid    = p->pid;
        __entry->tgid   = p->tgid;
        __entry->cpu    = task_cpu(p);
        __entry->uclamp_val  = uclamp_val;
    ),

    TP_printk("task =%s pid=%d tgid=%d cpu=%u set uclamp_min to %d",
        __entry->comm, __entry->pid, __entry->tgid,
        __entry->cpu, __entry->uclamp_val)
);

#endif
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE trace_powerd
#include <trace/define_trace.h>