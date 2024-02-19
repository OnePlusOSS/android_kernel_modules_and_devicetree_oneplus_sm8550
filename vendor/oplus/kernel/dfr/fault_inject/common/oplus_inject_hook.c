#include "oplus_inject_hook.h"

#ifdef __ASM_PTRACE_H
/*ARM64bit arm arg's regs : r0-r3 */
#define REG_RET(regs)  regs->regs[0]
#define REG_ARG0(regs) regs->regs[0]
#define REG_ARG1(regs) regs->regs[1]
#define REG_ARG2(regs) regs->regs[2]
#define REG_ARG3(regs) regs->regs[3]

#elif defined __ASM_ARM_PTRACE_H
/*ARM32bit arm arg's regs : r0-r3*/
#define REG_RET(regs)  regs->uregs[0]
#define REG_ARG0(regs) regs->uregs[0]
#define REG_ARG1(regs) regs->uregs[1]
#define REG_ARG2(regs) regs->uregs[2]
#define REG_ARG3(regs) regs->uregs[3]

#endif

int oplus_hook_entry(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	struct OplusHookRegs *rd = (struct OplusHookRegs *)ri->data;
	/*arm arg's regs : r0-r3*/
	rd->args[0] = (void *)REG_ARG0(regs);
	rd->args[1] = (void *)REG_ARG1(regs);
	rd->args[2] = (void *)REG_ARG2(regs);
	rd->args[3] = (void *)REG_ARG3(regs);
	return 0;
}
EXPORT_SYMBOL(oplus_hook_entry);

void oplus_hook_return(struct pt_regs *regs, long val)
{
	REG_RET(regs) = val;
}
EXPORT_SYMBOL(oplus_hook_return);

void oplus_hook_setarg(struct pt_regs *regs, int index, long val)
{
	switch (index) {
	case 0:
		REG_ARG0(regs) = val;
		break;

	case 1:
		REG_ARG1(regs) = val;
		break;

	case 2:
		REG_ARG2(regs) = val;
		break;

	case 3:
		REG_ARG3(regs) = val;
		break;

	default:
		break;
	}
}
EXPORT_SYMBOL(oplus_hook_setarg);

int register_oplus_hooks(struct oplus_hook **poh, int num)
{
	int i, ret;

	if (num <= 0) {
		return -EINVAL;
	}

	for (i = 0; i < num; i++) {
                /* kernel bug fix the register and unregister test meet next register failed issue */
                if(poh[i])
                        poh[i]->kp.addr = 0;
		ret = register_kretprobe(poh[i]);

		if (ret < 0) {
			pr_err("register kretprobe failed\n");
			unregister_oplus_hooks(poh, i);
			break;
		}
	}

	return ret;
}
EXPORT_SYMBOL(register_oplus_hooks);

void unregister_oplus_hooks(struct oplus_hook **poh, int num)
{
	int i;

	for (i = 0; i < num; i++) {
		unregister_kretprobe(poh[i]);
	}
}
EXPORT_SYMBOL(unregister_oplus_hooks);
