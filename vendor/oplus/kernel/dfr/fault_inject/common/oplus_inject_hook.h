#ifndef __OPLUS_INJECT_Hook_H__
#define __OPLUS_INJECT_Hook_H__

#include <linux/kprobes.h>
#include <asm/ptrace.h>
#include <linux/list.h>
#include <linux/spinlock.h>

#define oplus_hook_instance kretprobe_instance
#define oplus_hook kretprobe

struct OplusHookRegs {
	void *args[4];
};

#define oplus_hook_name(fun)    oplus_hook_##fun##_kretname
#define oplus_hook_handler(fun) oplus_hook_##fun##_handler

#define oplus_hook_init(fun) { \
        .kp.symbol_name = #fun, \
        .handler = oplus_hook_handler(fun), \
        .entry_handler = oplus_hook_entry, \
        .data_size = sizeof(struct OplusHookRegs), \
}

#define oplus_hook_define(fun) \
        static struct oplus_hook oplus_hook_name(fun) = oplus_hook_init(fun)

static inline int post_null_handler(struct oplus_hook_instance *ri,
				    struct pt_regs *regs)
{
	return 0;
}

#define oplus_hook_handler_entry(fun) oplus_hook_##fun##_handler_entry

#define oplus_hook_init_entry(fun) { \
        .kp.symbol_name = #fun, \
        .handler = post_null_handler, \
        .entry_handler = oplus_hook_handler_entry(fun), \
        .data_size = sizeof(struct OplusHookRegs), \
}

#define oplus_hook_entry_define(fun) \
        static struct oplus_hook oplus_hook_name(fun) = oplus_hook_init_entry(fun)

/* both entry and return hooked function */
#define oplus_hook_init_entry_exit(fun) { \
        .kp.symbol_name = #fun, \
        .handler = oplus_hook_handler(fun), \
        .entry_handler = oplus_hook_handler_entry(fun), \
        .data_size = sizeof(struct OplusHookRegs), \
}
#define oplus_hook_entry_exit_define(fun) \
        static struct oplus_hook oplus_hook_name(fun) = oplus_hook_init_entry_exit(fun)

int oplus_hook_entry(struct oplus_hook_instance *ri, struct pt_regs *regs);
void oplus_hook_return(struct pt_regs *regs, long val);
void oplus_hook_setarg(struct pt_regs *regs, int index, long val);

int register_oplus_hooks(struct oplus_hook **rps, int num);
void unregister_oplus_hooks(struct oplus_hook **rps, int num);

#endif
