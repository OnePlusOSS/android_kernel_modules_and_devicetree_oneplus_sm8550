/**************************************************************
* Copyright (C) 2021 - 2022 Oplus. All rights reserved.
*
* File : cpufreq_effiency.c
* Description: Source file for cpufreq_effiency.
* To optimize frequence point select base on power effiency.
* Version : 1.1
* Date : 2022-4-1
* Author : chengzhangtao
* ---------------- Revision History: --------------------------
* <version> <date> < author > <desc>
* Revision 1.0， 2021-5-30, chengzhangtao
* Modified to be suitable to the new coding rules in all functions.
****************************************************************/

#ifndef _CPUFREQ_EFFIENCY_H
#define _CPUFREQ_EFFIENCY_H

#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/cpufreq.h>
#include <linux/sched.h>
#include <linux/cpu.h>
#include <linux/types.h>
#include <linux/sysfs.h>
#include <linux/string.h>
#include <linux/topology.h>
#include <linux/slab.h>
#include <linux/cpumask.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_opp.h>
#include <linux/platform_device.h>

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

/* the SOC_ID was define in /soc/qcom/socinfo.c */
#define ABSENT_SOC_ID    0
#define SM8350_SOC_ID    415
#define SM8450_SOC_ID    457
#define SM7450_SOC_ID    506
#define PLATFORM_SM8350  "lahaina"
#define PLATFORM_SM8450  "waipio"
#define PLATFORM_SM7450  "diwali"

/* define the cluster information */
#define MAX_CLUSTER      3
#define SLIVER_CLUSTER   0
#define GOLDEN_CLUSTER   1
#define GOPLUS_CLUSTER   2

static unsigned int opp_number[MAX_CLUSTER] = {0, 0, 0};

/* define the parameter size */
#define MAX_CLUSTER_PARAMETERS 5
#define AFFECT_FREQ_VALUE1     0
#define AFFECT_THRES_SIZE1     1
#define AFFECT_FREQ_VALUE2     2
#define AFFECT_THRES_SIZE2     3
#define MASK_FREQ_VALUE        4

/* Power Domain just configuration for SM8350,
 * Table need refer to sm8350 clock_plan datasheet.
 */
static unsigned int sm8350_cluster_pd[MAX_CLUSTER] = {16, 16, 19};
static unsigned int sm8350_pd_sliver[16] = {
	0,/* LowSVS */
	0,/* LowSVS */
	0,/* LowSVS */
	0,/* LowSVS */
	0,/* LowSVS */
	1,/* SVS */
	1,/* SVS */
	2,/* SVS_L1 */
	2,/* SVS_L1 */
	2,/* SVS_L1 */
	3,/* Nominal */
	3,/* Nominal */
	3,/* Nominal */
	4,/* Nominal_L1 */
	5,/* Turbo */
	5 /* Turbo */
};

static unsigned int sm8350_pd_golden[16] = {
	0,/* LowSVS */
	1,/* SVS */
	1,/* SVS */
	2,/* SVS_L1 */
	2,/* SVS_L1 */
	3,/* Nominal */
	3,/* Nominal */
	4,/* Nominal_L1 */
	4,/* Nominal_L1 */
	4,/* Nominal_L1 */
	5,/* Turbo */
	6,/* Turbo_L1 */
	6,/* Turbo_L1 */
	7,/* Turbo_L3 */
	7,/* Turbo_L3 */
	8 /* Boost */
};

static unsigned int sm8350_pd_goplus[19] = {
	0,/* LowSVS */
	1,/* SVS */
	1,/* SVS */
	1,/* SVS */
	2,/* SVS_L1 */
	2,/* SVS_L1 */
	3,/* Nominal */
	3,/* Nominal */
	4,/* Nominal_L1 */
	4,/* Nominal_L1 */
	4,/* Nominal_L1 */
	5,/* Turbo */
	6,/* Turbo_L1 */
	7,/* Turbo_L3 */
	7,/* Turbo_L3 */
	7,/* Turbo_L3 */
	7,/* Turbo_L3 */
	8,/* Boost */
	8 /* Boost */
};

/*  Power Domain just configuration for SM8450,
 *  Table need refer to sm8450 clock_plan datasheet.
 */
static unsigned int sm8450_cluster_pd[MAX_CLUSTER] = {15, 18, 21};
static unsigned int sm8450_pd_sliver[15] = {
	0,/* LowSVS */
	0,/* LowSVS */
	0,/* LowSVS */
	0,/* LowSVS */
	1,/* SVS */
	1,/* SVS */
	2,/* SVS_L1 */
	2,/* SVS_L1 */
	3,/* Nominal */
	3,/* Nominal */
	3,/* Nominal */
	4,/* Nominal_L1 */
	5,/* Turbo */
	6,/* Turbo_L1 */
	6 /* Turbo_L1 */
};

static unsigned int sm8450_pd_golden[18] = {
	0,/* LowSVS */
	1,/* SVS */
	1,/* SVS */
	2,/* SVS_L1 */
	2,/* SVS_L1 */
	3,/* Nominal */
	3,/* Nominal */
	4,/* Nominal_L1 */
	4,/* Nominal_L1 */
	4,/* Nominal_L1 */
	5,/* Turbo */
	6,/* Turbo_L1 */
	6,/* Turbo_L1 */
	7,/* Turbo_L3 */
	7,/* Turbo_L3 */
	7,/* Turbo_L3 */
	8,/* Boost */
	9 /* BoostPlus */
};

static unsigned int sm8450_pd_goplus[21] = {
	0,/* LowSVS */
	1,/* SVS */
	1,/* SVS */
	1,/* SVS */
	2,/* SVS_L1 */
	2,/* SVS_L1 */
	3,/* Nominal */
	3,/* Nominal */
	3,/* Nominal */
	4,/* Nominal_L1 */
	4,/* Nominal_L1 */
	4,/* Nominal_L1 */
	5,/* Turbo */
	6,/* Turbo_L1 */
	6,/* Turbo_L1 */
	7,/* Turbo_L3 */
	7,/* Turbo_L3 */
	7,/* Turbo_L3 */
	7,/* Turbo_L3 */
	8,/* Boost */
	9 /* BoostPlus */
};

/*  Power Domain just configuration for SM7450,
 *  Table need refer to sm7450 clock_plan datasheet.
 */
static unsigned int sm7450_cluster_pd[MAX_CLUSTER] = {9, 10, 10};
static unsigned int sm7450_pd_sliver[9] = {
	0,/* LowSVS */
	1,/* SVS */
	1,/* SVS */
	2,/* SVS_L1 */
	3,/* Nominal */
	3,/* Nominal */
	4,/* Nominal_L1 */
	5,/* Turbo */
	6 /* Turbo_L1 */
};

static unsigned int sm7450_pd_golden[10] = {
	0,/* LowSVS */
	1,/* SVS */
	2,/* SVS_L1 */
	3,/* Nominal */
	4,/* Nominal_L1 */
	4,/* Nominal_L1 */
	5,/* Turbo */
	6,/* Turbo_L1 */
	7,/* Turbo_L3 */
	7 /* Turbo_L3 */
};

static unsigned int sm7450_pd_goplus[10] = {
	0,/* LowSVS */
	1,/* SVS */
	2,/* SVS_L1 */
	3,/* Nominal */
	4,/* Nominal_L1 */
	4,/* Nominal_L1 */
	5,/* Turbo */
	6,/* Turbo_L1 */
	7,/* Turbo_L3 */
	7 /* Turbo_L3 */
};

extern unsigned int update_power_effiency_lock(struct cpufreq_policy *policy, unsigned int freq, unsigned int loadadj_freq);
extern void frequence_opp_init(struct cpufreq_policy *policy);
#endif /* _CPUFREQ_EFFIENCY_H */
