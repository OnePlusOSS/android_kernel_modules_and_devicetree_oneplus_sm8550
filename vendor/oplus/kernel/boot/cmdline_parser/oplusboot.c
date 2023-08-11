#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/serio.h>
#include <linux/regulator/consumer.h>


#define MAX_CMDLINE_PARAM_LEN 1024
char startup_mode[MAX_CMDLINE_PARAM_LEN];
char bootmode[MAX_CMDLINE_PARAM_LEN];
char serial_no[MAX_CMDLINE_PARAM_LEN];
char verified_bootstate[MAX_CMDLINE_PARAM_LEN];
char prj_name[MAX_CMDLINE_PARAM_LEN];
char stackup_pcb_absent_status[MAX_CMDLINE_PARAM_LEN];

EXPORT_SYMBOL(startup_mode);
EXPORT_SYMBOL(bootmode);
EXPORT_SYMBOL(serial_no);
EXPORT_SYMBOL(verified_bootstate);
EXPORT_SYMBOL(prj_name);
EXPORT_SYMBOL(stackup_pcb_absent_status);

module_param_string(startupmode, startup_mode, MAX_CMDLINE_PARAM_LEN,
0600);
MODULE_PARM_DESC(startupmode,
"oplusboot.startupmode=<startupmode>");

module_param_string(mode, bootmode, MAX_CMDLINE_PARAM_LEN,
0600);
MODULE_PARM_DESC(mode,
"oplusboot.mode=<mode>");

module_param_string(serialno, serial_no, MAX_CMDLINE_PARAM_LEN,
0600);
MODULE_PARM_DESC(serialno,
"oplusboot.serialno=<serialno>");

module_param_string(verifiedbootstate, verified_bootstate, MAX_CMDLINE_PARAM_LEN,
0600);
MODULE_PARM_DESC(verifiedbootstate,
"oplusboot.verifiedbootstate=<verifiedbootstate>");

module_param_string(prjname, prj_name, MAX_CMDLINE_PARAM_LEN,
0600);
MODULE_PARM_DESC(prjname,
"oplusboot.prjname=<prjname>");

module_param_string(stackup_pcb_absent, stackup_pcb_absent_status, MAX_CMDLINE_PARAM_LEN,
0600);
MODULE_PARM_DESC(stackup_pcb_absent_status,
"oplusboot.stackup_pcb_absent=<stackup_pcb_detect_absent>");

MODULE_LICENSE("GPL v2");
