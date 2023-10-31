#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/soc/qcom/smem.h>
#include <linux/seq_file.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/syscalls.h>
#include <linux/version.h>
#include <linux/types.h>

#include <soc/oplus/boot/oplus_project.h>

#define SMEM_PROJECT    135

#define UINT2Ptr(n)        (void *)(uintptr_t)(n)
#define Ptr2UINT32(p)    (uintptr_t)(p)

#define PROJECT_VERSION            (0x1)
#define PCB_VERSION                (0x2)
#define RF_INFO                    (0x3)
#define MODEM_TYPE                (0x4)
#define OPLUS_BOOTMODE            (0x5)
/*
#define SECURE_TYPE                (0x6)
#define SECURE_STAGE            (0x7)
*/
#define OCP_NUMBER                (0x8)
#define SERIAL_NUMBER            (0x9)
#define ENG_VERSION                (0xA)
#define CONFIDENTIAL_STATUS        (0xB)
#define CDT_INTEGRITY            (0xC)
#define OPLUS_FEATURE            (0xD)
#define OPERATOR_NAME            (0xE)
#define PROJECT_TEST            (0x1F)

static ProjectInfoOCDT *g_project = NULL;

static struct pcb_match pcb_str[] = {
    {.version=PRE_EVB1, .str="PRE_EVB1"},
    {.version=PRE_EVB2, .str="PRE_EVB2"},
    {.version=EVB1, .str="EVB1"},
    {.version=EVB2, .str="EVB2"},
    {.version=EVB3, .str="EVB3"},
    {.version=EVB4, .str="EVB4"},
    {.version=EVB5, .str="EVB5"},
    {.version=EVB6, .str="EVB6"},
    {.version=T0, .str="T0"},
    {.version=T1, .str="T1"},
    {.version=T2, .str="T2"},
    {.version=T3, .str="T3"},
    {.version=T4, .str="T4"},
    {.version=T5, .str="T5"},
    {.version=T6, .str="T6"},
    {.version=EVT1, .str="EVT1"},
    {.version=EVT2, .str="EVT2"},
    {.version=EVT3, .str="EVT3"},
    {.version=EVT4, .str="EVT4"},
    {.version=EVT5, .str="EVT5"},
    {.version=EVT6, .str="EVT6"},
    {.version=DVT1, .str="DVT1"},
    {.version=DVT2, .str="DVT2"},
    {.version=DVT3, .str="DVT3"},
    {.version=DVT4, .str="DVT4"},
    {.version=DVT5, .str="DVT5"},
    {.version=DVT6, .str="DVT6"},
    {.version=PVT1, .str="PVT1"},
    {.version=PVT2, .str="PVT2"},
    {.version=PVT3, .str="PVT3"},
    {.version=PVT4, .str="PVT4"},
    {.version=PVT5, .str="PVT5"},
    {.version=PVT6, .str="PVT6"},
    {.version=MP1, .str="MP1"},
    {.version=MP2, .str="MP2"},
    {.version=MP3, .str="MP3"},
    {.version=MP4, .str="MP4"},
    {.version=MP5, .str="MP5"},
    {.version=MP6, .str="MP6"},
};

struct proc_dir_entry *oplus_info = NULL;

extern char build_variant[];
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0)
extern char sim_card_num[];
#endif
extern char cdt[];
extern char serial_no[];

static void init_project_version(void)
{
    /*for qcom's smem*/
    size_t smem_size;
    void *smem_addr;
    char *PCB_version_name = NULL;
    uint16_t index = 0;

    if (g_project) {
        return;
    } else {
        smem_addr = qcom_smem_get(QCOM_SMEM_HOST_ANY,
        SMEM_PROJECT,
        &smem_size);
        if (IS_ERR(smem_addr)) {
            pr_err("unable to acquire smem SMEM_PROJECT entry\n");
            return;
        }

        g_project = (ProjectInfoOCDT *)smem_addr;
        if (g_project == ERR_PTR(-EPROBE_DEFER)) {
            g_project = NULL;
            return;
        }

        do {
            if(pcb_str[index].version == g_project->nDataSCDT.PCB){
                PCB_version_name = pcb_str[index].str;
                break;
            }
            index++;
        }while(index < sizeof(pcb_str)/sizeof(struct pcb_match));

        pr_err("KE Project:%d, Audio:%d, nRF:%d, PCB:%s\n",
            g_project->nDataBCDT.ProjectNo,
            g_project->nDataBCDT.AudioIdx,
            g_project->nDataSCDT.RF,PCB_version_name);
        pr_err("OCP: %d 0x%x %c %d 0x%x %c\n",
            g_project->nDataSCDT.PmicOcp[0],
            g_project->nDataSCDT.PmicOcp[1],
            g_project->nDataSCDT.PmicOcp[2],
            g_project->nDataSCDT.PmicOcp[3],
            g_project->nDataSCDT.PmicOcp[4],
            g_project->nDataSCDT.PmicOcp[5]);
    }

    pr_err("get_project:%d, is_new_cdt:%d, get_PCB_Version:%d, get_Oplus_Boot_Mode:%d, get_Modem_Version:%d\n", 
            get_project(),
            is_new_cdt(),
            get_PCB_Version(),
            get_Oplus_Boot_Mode(),
            get_Modem_Version());
    pr_err("get_Operator_Version:%d, get_dtsiNo:%d, get_audio_project:%d\n",
            get_Operator_Version(),
            get_dtsiNo(),
            get_audio());
    pr_err("oplus project info loading finished\n");
}

static bool cdt_integrity = false;
static int __init cdt_setup(char *str)
{
    if (str[0] == '1')
        cdt_integrity = true;

    return 1;
}
//__setup("cdt_integrity=", cdt_setup);

unsigned int get_project(void)
{
    init_project_version();

    return g_project? g_project->nDataBCDT.ProjectNo : 0;
}
EXPORT_SYMBOL(get_project);

unsigned int is_project(int project)
{
    init_project_version();

    return ((get_project() == project)?1:0);
}
EXPORT_SYMBOL(is_project);

unsigned int is_new_cdt(void)/*Q and R is new*/
{
	init_project_version();
    if(get_cdt_version() == OCDT_VERSION_1_0)
        return 1;
    else
        return 0;
}

unsigned int get_PCB_Version(void)
{
    init_project_version();

    return g_project? g_project->nDataSCDT.PCB:-EINVAL;
}
EXPORT_SYMBOL(get_PCB_Version);

unsigned int get_Oplus_Boot_Mode(void)
{
    init_project_version();

    return g_project?g_project->nDataSCDT.OplusBootMode:0;
}
EXPORT_SYMBOL(get_Oplus_Boot_Mode);

int32_t get_Modem_Version(void)
{
    init_project_version();

    /*cdt return modem,ocdt return RF*/
    return g_project?g_project->nDataSCDT.RF:-EINVAL;
}
EXPORT_SYMBOL(get_Modem_Version);

int32_t get_Operator_Version(void)
{
    init_project_version();

    return g_project?g_project->nDataSCDT.Operator:-EINVAL;
}
EXPORT_SYMBOL(get_Operator_Version);

unsigned int get_dtsiNo(void)
{
    return (g_project && is_new_cdt()) ? g_project->nDataBCDT.DtsiNo : 0;
}
EXPORT_SYMBOL(get_dtsiNo);

unsigned int get_audio(void)
{
    return (g_project && is_new_cdt()) ? g_project->nDataBCDT.AudioIdx : 0;
}
EXPORT_SYMBOL(get_audio);

int rpmb_is_enable(void)
{
#define RPMB_KEY_PROVISIONED 0x00780178

    unsigned int rmpb_rd = 0;
    void __iomem *rpmb_addr = NULL;
    static unsigned int rpmb_enable = 0;

    if (rpmb_enable)
        return rpmb_enable;

    rpmb_addr = ioremap(RPMB_KEY_PROVISIONED , 4);    
    if (rpmb_addr) {
        rmpb_rd = __raw_readl(rpmb_addr);
        iounmap(rpmb_addr);
        rpmb_enable = (rmpb_rd >> 24) & 0x01;
    } else {
        rpmb_enable = 0;
    }

    return rpmb_enable;
}
EXPORT_SYMBOL(rpmb_is_enable);


unsigned int get_eng_version(void)
{
    init_project_version();

    return g_project?g_project->nDataECDT.Version:-EINVAL;
}
EXPORT_SYMBOL(get_eng_version);

bool oplus_daily_build(void)
{
    static int daily_build = -1;
    int eng_version = 0;

    if (daily_build != -1)
        return daily_build;

    if (strstr(build_variant, "userdebug") ||
        strstr(build_variant, "eng")) {
        daily_build = true;
    } else {
        daily_build = false;
    }

    eng_version = get_eng_version();
    if ((ALL_NET_CMCC_TEST == eng_version) || (ALL_NET_CMCC_FIELD == eng_version) ||
        (ALL_NET_CU_TEST == eng_version) || (ALL_NET_CU_FIELD == eng_version) ||
        (ALL_NET_CT_TEST == eng_version) || (ALL_NET_CT_FIELD == eng_version)){
        daily_build = false;
    }

    return daily_build;
}
EXPORT_SYMBOL(oplus_daily_build);

bool is_confidential(void)
{
    init_project_version();

    return g_project?g_project->nDataECDT.Is_confidential:-EINVAL;
}
EXPORT_SYMBOL(is_confidential);

unsigned char get_oplus_feature(OplusBoardFeatureMask feature_num)
{
    if(is_new_cdt()) {
		init_project_version();
		if (feature_num < 0 || feature_num >= FEATURE_COUNT)
			return 0;
		return g_project?g_project->nDataBCDT.Feature[feature_num]:0;
	}
	else
		return 0;
}
EXPORT_SYMBOL(get_oplus_feature);

#define SERIALNO_LEN 16
unsigned int get_serialID()
{
    unsigned int serial_id = 0xFFFFFFFF;

    sscanf(serial_no, "%x", &serial_id);
    return serial_id;
}
EXPORT_SYMBOL(get_serialID);

static void dump_ocp_info(struct seq_file *s)
{
    init_project_version();

    if (!g_project)
        return;

    seq_printf(s, "ocp: %d 0x%x %d 0x%x %c %c",
        g_project->nDataSCDT.PmicOcp[0],
        g_project->nDataSCDT.PmicOcp[1],
        g_project->nDataSCDT.PmicOcp[2],
        g_project->nDataSCDT.PmicOcp[3],
        g_project->nDataSCDT.PmicOcp[4],
        g_project->nDataSCDT.PmicOcp[5]);
}

static void dump_serial_info(struct seq_file *s)
{
    seq_printf(s, "0x%x", get_serialID());
}

static void dump_oplus_feature(struct seq_file *s)
{
    int i = 0;
	init_project_version();

	if (!g_project)
		return;

	for(i = 0;i < FEATURE_COUNT; i++) {
		seq_printf(s, "%u\t", g_project->nDataBCDT.Feature[i]);
	}
	seq_printf(s, "\n");

	return;
}

static void dump_eng_version(struct seq_file *s)
{
    seq_printf(s, "%d", get_eng_version());
    return;
}

static void dump_confidential_status(struct seq_file *s)
{
    seq_printf(s, "%d", is_confidential());
    return;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0)
static void update_manifest(struct proc_dir_entry *parent)
{
    static const char* manifest_src[2] = {
        "/vendor/odm/etc/vintf/manifest_ssss.xml",
        "/vendor/odm/etc/vintf/manifest_dsds.xml",
    };
    mm_segment_t fs;


    fs = get_fs();
    set_fs(KERNEL_DS);

    if (parent) {
#if IS_MODULE(CONFIG_OPLUS_FEATURE_SIMCARDNUM)
		if (sim_card_num[0] == '0') {
			proc_symlink("manifest", parent, manifest_src[0]);//single sim
		}
		else {
			proc_symlink("manifest", parent, manifest_src[1]);
		}
#else
		proc_symlink("manifest", parent, manifest_src[1]);
#endif
}
    set_fs(fs);
}

static void update_telephony_manifest(struct proc_dir_entry *parent)
{
    static const char* manifest_src[2] = {
        "/vendor/odm/etc/vintf/telephony_manifest_ssss.xml",
        "/vendor/odm/etc/vintf/telephony_manifest_dsds.xml",
    };
    mm_segment_t fs;

    fs = get_fs();
    set_fs(KERNEL_DS);

    if (parent) {
#if IS_MODULE(CONFIG_OPLUS_FEATURE_SIMCARDNUM)
		if (sim_card_num[0] == '0') {
			proc_symlink("telephony_manifest", parent, manifest_src[0]);//single sim
		}
		else {
			proc_symlink("telephony_manifest", parent, manifest_src[1]);
		}
#else 
		proc_symlink("telephony_manifest", parent, manifest_src[1]);
#endif
    }

    set_fs(fs);
}
#endif

static int project_read_func(struct seq_file *s, void *v)
{
    void *p = s->private;

    switch(Ptr2UINT32(p)) {
    case PROJECT_VERSION:
        if (get_project() > 0x20000) {
            seq_printf(s, "%X", get_project());
        } else {
            seq_printf(s, "%d", get_project());
        }
        break;
    case PCB_VERSION:
        seq_printf(s, "%d", get_PCB_Version());
        break;
    case OPLUS_BOOTMODE:
        seq_printf(s, "%d", get_Oplus_Boot_Mode());
        break;
    case MODEM_TYPE:
    case RF_INFO:
        seq_printf(s, "%d", get_Modem_Version());
        break;
    case OCP_NUMBER:
        dump_ocp_info(s);
        break;
    case SERIAL_NUMBER:
        dump_serial_info(s);
        break;
    case ENG_VERSION:
        dump_eng_version(s);
        break;
    case CONFIDENTIAL_STATUS:
        dump_confidential_status(s);
        break;
    case CDT_INTEGRITY:
        seq_printf(s, "%d", cdt_integrity);
        break;
    case OPLUS_FEATURE:
        dump_oplus_feature(s);
        break;
    case OPERATOR_NAME:
        seq_printf(s, "%d", get_Operator_Version());
        break;
    default:
        seq_printf(s, "not support\n");
        break;
    }

    return 0;
}

unsigned int get_cdt_version()
{
    init_project_version();

    return g_project?g_project->Version:0;
}

static int projects_open(struct inode *inode, struct file *file)
{
    return single_open(file, project_read_func, PDE_DATA(inode));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static const struct proc_ops project_info_fops = {
    .proc_open  = projects_open,
    .proc_read  = seq_read,
    .proc_release = single_release,
    .proc_lseek  = seq_lseek,
};
#else
static const struct file_operations project_info_fops = {
    .owner = THIS_MODULE,
    .open  = projects_open,
    .read  = seq_read,
    .release = single_release,
};
#endif

static int __init oplus_project_init(void)
{
    struct proc_dir_entry *p_entry;

    oplus_info = proc_mkdir("oplusVersion", NULL);

    if (!oplus_info) {
        goto error_init;
    }

#ifdef CONFIG_OPLUS_FEATURE_CDT
	cdt_setup(cdt);
#else
    cdt_setup("1");
#endif

    p_entry = proc_create_data("prjName", S_IRUGO, oplus_info, &project_info_fops, UINT2Ptr(PROJECT_VERSION));
    if (!p_entry)
        goto error_init;

    p_entry = proc_create_data("pcbVersion", S_IRUGO, oplus_info, &project_info_fops, UINT2Ptr(PCB_VERSION));
    if (!p_entry)
        goto error_init;

    p_entry = proc_create_data("oplusBootmode", S_IRUGO, oplus_info, &project_info_fops, UINT2Ptr(OPLUS_BOOTMODE));
    if (!p_entry)
        goto error_init;

    p_entry = proc_create_data("RFType", S_IRUGO, oplus_info, &project_info_fops, UINT2Ptr(RF_INFO));
    if (!p_entry)
        goto error_init;

    p_entry = proc_create_data("operatorName", S_IRUGO, oplus_info, &project_info_fops, UINT2Ptr(OPERATOR_NAME));
    if (!p_entry)
        goto error_init;

    p_entry = proc_create_data("ocp", S_IRUGO, oplus_info, &project_info_fops, UINT2Ptr(OCP_NUMBER));
    if (!p_entry)
        goto error_init;

    p_entry = proc_create_data("serialID", S_IRUGO, oplus_info, &project_info_fops, UINT2Ptr(SERIAL_NUMBER));
    if (!p_entry)
        goto error_init;

    p_entry = proc_create_data("engVersion", S_IRUGO, oplus_info, &project_info_fops, UINT2Ptr(ENG_VERSION));
    if (!p_entry)
        goto error_init;

    p_entry = proc_create_data("isConfidential", S_IRUGO, oplus_info, &project_info_fops, UINT2Ptr(CONFIDENTIAL_STATUS));
    if (!p_entry)
        goto error_init;

    p_entry = proc_create_data("cdt", S_IRUGO, oplus_info, &project_info_fops, UINT2Ptr(CDT_INTEGRITY));
    if (!p_entry)
        goto error_init;

    p_entry = proc_create_data("feature", S_IRUGO, oplus_info, &project_info_fops, UINT2Ptr(OPLUS_FEATURE));
    if (!p_entry)
        goto error_init;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0)
    /*update single or double cards*/
    update_manifest(oplus_info);
    update_telephony_manifest(oplus_info);
#endif
    return 0;

error_init:
    remove_proc_entry("oplusVersion", NULL);
    return -ENOENT;
}

arch_initcall(oplus_project_init);

MODULE_DESCRIPTION("OPLUS project version");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Joshua");
