// SPDX-License-Identifier: GPL-2.0-only
/*===========================================================================
#  Copyright (c) 2021, OPLUS Technologies Inc. All rights reserved.
===========================================================================

                              EDIT HISTORY

 when       who        what, where, why
 --------   ---        ----------------------------------------------------------
 06/24/21   Yang.Wang   Created file
=============================================================================*/
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/err.h>
#include <linux/module.h>
#include "oplus_pmic_info_mtk.h"

/**********************************************
PONSTS[0XC]
4 		STS_RBOOT		Power on for cold reset
2		STS_CHRIN 		Power on for charger insertion
0		STS_PWRKEY		Power on for PWREKY press
**********************************************/
static char * const mt6363_pon_reason_str[] = {
	[0] = ":Power on for PWREKY press",
	[1] = ":Reserved bit, meaningless",
	[2] = ":Power on for charger insertion",
	[3] = ":Reserved bit, meaningless",
	[4] = ":Power on for cold reset",
};
/**********************************************
[PMIC]POFFSTS[0xE]
15      STS_NORMOFF    Power off for PWRHOLD clear
14		STS_PKEYLP	   Power off for power key(s) long press
13		STS_CRST	   Power off for cold reset
12		STS_WRST	   Power reset for warm reset
11		STS_THRDN	   Power off for thermal shutdown
10		STS_PSOC 	   Power off for default on BUCK OC
9		STS_PGFAIL	   Power off for PWRGOOD failure
8		STS_UVLO	   Power off for UVLO event
7		STS_UVLO_VBB   Power off for EN H go L event
6		STS_OVLO	   Power off for OVLO event
5		STS_PKSP	   PWRKEY short press
4		STS_KEYPWR	   Critical power is turned off during system on
3		STS_PUPSRC	   Power off for power source missing
2		STS_WDT	       AP WDT
1		STS_DDLO	   DDLO occurs after system on
0		STS_BWDT	   Power off for BWDT
**********************************************/
static char * const mt6363_pon_poff_reason_str[] = {
	[0]=":Power off for UVLO event",
	[1]=":Power off for PWRGOOD failure",
	[2]=":Power off for default on BUCK OC",
	[3]=":Power off for thermal shutdown",
	[4]=":Power reset for warm reset",
	[5]=":Power off for cold reset",
	[6]=":Power off for power key(s) long press",
	[7]=":Power off for PWRHOLD clear",
	[8]=":Power off for BWDT",
	[9]=":DDLO occurs after system on",
	[10]=":AP WDT",
	[11]=":Power off for power source missing",
	[12]=":Critical power is turned off during system on",
	[13]=":PWRKEY short press",
	[14]=":Power off for OVLO event",
	[15]=":Power off for EN H go L event",
};

/**********************************************
[PMIC]POFFSTS2[0xE]
11		STS_THR_LOC	           Power off thermal shutdown
10		STS_THR_LOC 	       Power off thermal shutdown
9		STS_THR_LOC	           Power off thermal shutdown
8		STS_THR_LOC	           Power off thermal shutdown
3		STS_PMICWDT_TMR2	   Power off for PMIC self WDT by Timer2
2		STS_PMICWDT_TMR1	   Power off for PMIC self WDT by Timer1
**********************************************/
static char * const mt6363_pon_poff_reason2_str[] = {
	[0]=":Reserved bit, meaningless",
	[1]=":Reserved bit, meaningless",
	[2]=":Power off for PMIC self WDT by Timer1",
	[3]=":Power off for PMIC self WDT by Timer2",
	[4]=":Reserved bit, meaningless",
	[5]=":Reserved bit, meaningless",
	[6]=":Reserved bit, meaningless",
	[7]=":Reserved bit, meaningless",
	[8]=":Power off thermal shutdown",
	[9]=":Power off thermal shutdown",
	[10]=":Power off thermal shutdown",
	[11]=":Power off thermal shutdown",
	[12]=":Reserved bit, meaningless",
	[13]=":Reserved bit, meaningless",
	[14]=":Reserved bit, meaningless",
	[15]=":Reserved bit, meaningless",
};
void mt6363_pmic_show(struct PMICHistoryKernelStruct *pmic_history_ptr, char page[], int *len, u64 pmic_history_count){

	u8 pmic_device_index=0;
	struct PMICRecordKernelStruct pmic_first_record={0};
	struct PMICRegStruct pmic_reg_value = {0};
	//int len=0;

	int pon_reason_bit=-1, poff_reason_bit=-1, poff_reason2_bit=-1;

	unsigned int cur_pon_reason=0, cur_pon_reason_bak=0;
	unsigned int cur_poff_reason=0, cur_poff_reason_bak=0;
	unsigned int cur_poff_reason2=0, cur_poff_reason2_bak=0; 
	unsigned int cur_pg_sdn_sts0=0, cur_pg_sdn_sts1=0;
	unsigned int cur_oc_sdn_sts0=0, cur_oc_sdn_sts1=0;

	for(;pmic_device_index<pmic_history_count;pmic_device_index++){
		pon_reason_bit = -1;
		poff_reason_bit = -1;
		poff_reason2_bit = -1;

		pmic_first_record = pmic_history_ptr->pmic_record[pmic_device_index];
		pmic_reg_value = pmic_first_record.pmic_pon_poff_reason[0];
		if (DATA_VALID_FLAG != pmic_reg_value.data_is_valid) {
			continue;
		}

		//show pon_reason
		cur_pon_reason = pmic_reg_value.pon_reason;
		cur_pon_reason_bak = cur_pon_reason;
		while(0 != cur_pon_reason_bak) {
            cur_pon_reason_bak = cur_pon_reason_bak >> 1;
            pon_reason_bit++;
        }
		if(pon_reason_bit>=0 && pon_reason_bit<=4){
			printk(KERN_INFO "mt6363_show_pon_reason = %d\n",pon_reason_bit);
			(*len) += snprintf(&page[(*len)],2048-(*len), "PMIC->pon_reason|%u|0x%02X|:%s\n",
						pmic_device_index,
						cur_pon_reason,
						mt6363_pon_reason_str[pon_reason_bit]);
		}else{
			printk(KERN_INFO "mt6363_show_pon_reason = %d\n",pon_reason_bit);
			(*len) += snprintf(&page[(*len)],2048-(*len), "PMIC->pon_reason|%u|0x%02X|:%s\n",
						pmic_device_index,
						cur_pon_reason,
						"adb reboot");
		}
		
		//show poff_reason and poff_reason2
		cur_poff_reason = pmic_reg_value.poff_reason;
		cur_poff_reason2 = pmic_reg_value.poff_reason2;
		cur_poff_reason_bak = cur_poff_reason;
		while(0 != cur_poff_reason_bak) {
            cur_poff_reason_bak = cur_poff_reason_bak >> 1;
            poff_reason_bit++;
        }
		cur_poff_reason2_bak = cur_poff_reason2;
		while(0 != cur_poff_reason2_bak) {
            cur_poff_reason2_bak = cur_poff_reason2_bak >> 1;
            poff_reason2_bit++;
        }
		if(poff_reason_bit>=0 && poff_reason_bit<=15){
			(*len) += snprintf(&page[(*len)], 2048-(*len), "PMIC->poff_reason|%u|0x%x|:%s\n",
				pmic_device_index,
				cur_poff_reason,
				mt6363_pon_poff_reason_str[poff_reason_bit]);
		}else{
			(*len) += snprintf(&page[(*len)], 2048-(*len), "PMIC->poff_reason|%u|0x%0x|:%s\n",
			pmic_device_index,
			cur_poff_reason,
			"Battery Loss");
		}
		if(poff_reason2_bit>=0 && poff_reason2_bit<=15){
			(*len) += snprintf(&page[(*len)], 2048-(*len), "PMIC->poff_reason2|%u|0x%x|:%s\n",
				pmic_device_index,
				cur_poff_reason2,
				mt6363_pon_poff_reason2_str[poff_reason2_bit]);
		}else{
			(*len) += snprintf(&page[(*len)], 2048-(*len), "PMIC->poff_reason2|%u|0x%x|:%s\n",
				pmic_device_index,
				cur_poff_reason2,
				"closed_source");
		}

		//show pg_sdn_sts0 and pg_sdn_sts1
		cur_pg_sdn_sts0=pmic_reg_value.pg_sdn_sts0;
		cur_pg_sdn_sts1=pmic_reg_value.pg_sdn_sts1;
		(*len) += snprintf(&page[(*len)], 2048-(*len), "PMIC->pg_sdn_sts0|%u|0x%x|:%s\n",
				pmic_device_index,
				cur_pg_sdn_sts0,
				"closed_source");
		(*len) += snprintf(&page[(*len)], 2048-(*len), "PMIC->pg_sdn_sts1|%u|0x%x|:%s\n",
				pmic_device_index,
				cur_pg_sdn_sts1,
				"closed_source");

		//show oc_sdn_sts0 and oc_sdn_sts1
		cur_oc_sdn_sts0=pmic_reg_value.oc_sdn_sts0;
		cur_oc_sdn_sts1=pmic_reg_value.oc_sdn_sts1;
		(*len) += snprintf(&page[(*len)], 2048-(*len), "PMIC->oc_sdn_sts0|%u|0x%x|:%s\n",
				pmic_device_index,
				cur_oc_sdn_sts0,
				"closed_source");
		(*len) += snprintf(&page[(*len)], 2048-(*len), "PMIC->oc_sdn_sts1|%u|0x%x|:%s\n",
				pmic_device_index,
				cur_oc_sdn_sts1,
				"closed_source");
		
	}
}

/**********************************************
PONSTS[0XC]
4 		STS_RBOOT		Power on for cold reset
3 		STS_SPAR	    Power on for SPAR event
2		STS_CHRIN 		Power on for charger insertion
1 		STS_RTCA		Power on for RTC alarm
0		STS_PWRKEY		Power on for PWREKY press
**********************************************/
static char * const mt6359_pon_reason_str[] = {
	[0] = ":Power on for PWREKY press",
	[1] = ":Power on for RTC alarm",
	[2] = ":Power on for charger insertion",
	[3] = ":Power on for SPAR event",
	[4] = ":Power on for cold reset",
};
/**********************************************
[PMIC]POFFSTS[0xE]
14		STS_OVLO	   Power off for OVLO event
13		STS_PKSP	   PWRKEY short press
12		STS_KEYPWR	   Critical power is turned off during system on
11		STS_PUPSRC	   Power off for power source missing
10		STS_WDT 	   AP WDT
9		STS_DDLO	   DDLO occurs after system on
8		STS_BWDT	   Power off for BWDT
7		STS_NORMOFF    Power off for PWRHOLD clear
6		STS_PKEYLP	   Power off for power key(s) long press
5		STS_CRST	   Power off for cold reset
4		STS_WRST	   Power reset for warm reset
3		STS_THRDN	   Power off for thermal shutdown
2		STS_PSOC	   Power off for default on BUCK OC
1		STS_PGFAIL	   Power off for PWRGOOD failure
0		STS_UVLO	   Power off for UVLO event
**********************************************/
static char * const mt6359_poff_reason_str[] = {
	[0]=":Power off for UVLO event",
	[1]=":Power off for PWRGOOD failure",
	[2]=":Power off for default on BUCKOC",
	[3]=":Power off for thermal shutdown",
	[4]=":Power reset for warm reset",
	[5]=":Power off for cold reset",
	[6]=":Power off for powerkey(s) long press",
	[7]=":Power off for PWRHOLD clear",
	[8]=":Power off for BWDT",
	[9]=":DDLO occurs after system on",
	[10]=":APWDT",
	[11]=":Power off for power source missing",
	[12]=":Critical power is turned off during system on",
	[13]=":PWRKEY short press",
	[14]=":Power off for OVLO event",
};

void mt6359_pmic_show(struct PMICHistoryKernelStruct *pmic_history_ptr, char page[], int *len, u64 pmic_history_count){
	
	u8 pmic_device_index=0;
	struct PMICRecordKernelStruct pmic_first_record={0};
	struct PMICRegStruct pmic_reg_value = {0};
	
	
	int pon_reason_bit=-1, poff_reason_bit=-1;
	unsigned int cur_pon_reason=0, cur_pon_reason_bak=0;
	unsigned int cur_poff_reason=0, cur_poff_reason_bak=0;
	unsigned int cur_pg_sdn_sts0=0, cur_pg_sdn_sts1=0;
	unsigned int cur_oc_sdn_sts0=0, cur_oc_sdn_sts1=0;

	for(;pmic_device_index<pmic_history_count;pmic_device_index++){

		poff_reason_bit = -1;
		pon_reason_bit = -1;

		pmic_first_record = pmic_history_ptr->pmic_record[pmic_device_index];
		pmic_reg_value = pmic_first_record.pmic_pon_poff_reason[0];
		if (DATA_VALID_FLAG != pmic_reg_value.data_is_valid) {
			continue;
		}

		//show pon_reason
		cur_pon_reason = pmic_reg_value.pon_reason;
		cur_pon_reason_bak = cur_pon_reason;
		while(0 != cur_pon_reason_bak) {
            cur_pon_reason_bak = cur_pon_reason_bak >> 1;
            pon_reason_bit++;
        }
		if(pon_reason_bit>=0 && pon_reason_bit<=4){
			(*len) += snprintf(&page[(*len)],2048-(*len), "PMIC->pon_reason|%u|0x%02X|:%s\n",
						pmic_device_index,
						cur_pon_reason,
						mt6359_pon_reason_str[pon_reason_bit]);
		}else{
			(*len) += snprintf(&page[(*len)],2048-(*len), "PMIC->pon_reason|%u|0x%02X|:%s\n",
						pmic_device_index,
						cur_pon_reason,
						"adb reboot");
		}
		
		//show poff_reason
		cur_poff_reason = pmic_reg_value.poff_reason;
		cur_poff_reason_bak = cur_poff_reason;
		while(0 != cur_poff_reason_bak) {
            cur_poff_reason_bak = cur_poff_reason_bak >> 1;
            poff_reason_bit++;
        }
		if (poff_reason_bit>=0 && poff_reason_bit<=14) {
        	(*len) += snprintf(&page[(*len)], 2048-(*len), "PMIC->poff_reason|%u|0x%0x|:%s\n",
			pmic_device_index,
			cur_poff_reason,
			mt6359_poff_reason_str[poff_reason_bit]);
        }else{
			(*len) += snprintf(&page[(*len)], 2048-(*len), "PMIC->poff_reason|%u|0x%x|:%s\n",
				pmic_device_index,
				cur_poff_reason,
				"Battery Loss");
		}

		//show pg_sdn_sts0 and pg_sdn_sts1
		cur_pg_sdn_sts0=pmic_reg_value.pg_sdn_sts0;
		cur_pg_sdn_sts1=pmic_reg_value.pg_sdn_sts1;
		(*len) += snprintf(&page[(*len)], 2048-(*len), "PMIC->pg_sdn_sts0|%u|0x%x|:%s\n",
				pmic_device_index,
				cur_pg_sdn_sts0,
				"closed_source");
		(*len) += snprintf(&page[(*len)], 2048-(*len), "PMIC->pg_sdn_sts1|%u|0x%x|:%s\n",
				pmic_device_index,
				cur_pg_sdn_sts1,
				"closed_source");

		//show oc_sdn_sts0 and oc_sdn_sts1
		cur_oc_sdn_sts0=pmic_reg_value.oc_sdn_sts0;
		cur_oc_sdn_sts1=pmic_reg_value.oc_sdn_sts1;
		(*len) += snprintf(&page[(*len)], 2048-(*len), "PMIC->oc_sdn_sts0|%u|0x%x|:%s\n",
				pmic_device_index,
				cur_oc_sdn_sts0,
				"closed_source");
		(*len) += snprintf(&page[(*len)], 2048-(*len), "PMIC->oc_sdn_sts1|%u|0x%x|:%s\n",
				pmic_device_index,
				cur_oc_sdn_sts1,
				"closed_source");

	}
	printk(KERN_INFO "mt6359_pmic_show end\n");
}
/**********************************************/

static ssize_t pmic_monitor_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf) {
	char page[2048] = {0};
	int len = 0;
	struct PMICHistoryKernelStruct *pmic_history_ptr = NULL;
	u64 pmic_history_count=0;

	pmic_history_ptr = (struct PMICHistoryKernelStruct *)get_pmic_history();

	if (NULL == pmic_history_ptr) {
		len += snprintf(&page[len],512-len, "PMIC|0|0x00|0x0000|NULL\n");
		memcpy(buf,page,len);
		return len;
	}

	//show pmic_magic
	//len += snprintf(&page[len], 2048-len, "PMIC->pmic_magic = %s\n",&(pmic_history_ptr->pmic_magic));
	//show pmic_history_count
	len += snprintf(&page[len], 2048-len, "PMIC->pmic_history_count = %u\n",pmic_history_ptr->log_count);
	len += snprintf(&page[len], 2048-len, "PMIC->chip_code = %x\n",pmic_history_ptr->pmic_record[0].pmic_pon_poff_reason[0].chip_code);

	pmic_history_count = pmic_history_ptr->log_count;

	printk(KERN_INFO "pmic_history_count = %llu\n",pmic_history_count);
	printk(KERN_INFO "chip_code = %d\n",pmic_history_ptr->pmic_record[0].pmic_pon_poff_reason[0].chip_code);

	if(pmic_history_ptr->pmic_record[0].pmic_pon_poff_reason[0].chip_code==25392){
		printk(KERN_INFO "mt6363_pmic_show\n");
		mt6363_pmic_show(pmic_history_ptr, page, &len, pmic_history_count);
	}else{
		printk(KERN_INFO "mt6359_pmic_show\n");
		mt6359_pmic_show(pmic_history_ptr, page, &len, pmic_history_count);
	}

	memcpy(buf,page,len);
	return len;

}
pmic_info_attr_ro(pmic_monitor);
/**********************************************/

static struct attribute * g[] = {
    &pmic_monitor_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

struct kobject *pmic_info_kobj;

static int __init pmic_info_init(void)
{
	int error;

	pmic_info_kobj = kobject_create_and_add("pmic_info", NULL);
	if (!pmic_info_kobj)
		return -ENOMEM;
	error = sysfs_create_group(pmic_info_kobj, &attr_group);
	if (error)
		return error;

	printk(KERN_INFO "wy pmic_info_init ok \n");
	return 0;
}

core_initcall(pmic_info_init);

MODULE_LICENSE("GPL v2");


