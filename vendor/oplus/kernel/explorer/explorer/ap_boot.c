/*
 * Copyright (c) 2021 ZEKU Technology(Shanghai) Corp.,Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * Basecode Created :        2020/10/26 Author: zhangao@zeku.com
 *
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include "include/main.h"
#include "include/ap_boot.h"
#include "include/irq.h"
#include "include/uapi/explorer_uapi.h"
#include "include/power.h"
#include "include/exception.h"
#ifdef SLT_ENABLE
#include "../slt/include/slt.h"
#include "../slt/include/slt_loader.h"
#endif

#ifndef USE_RAW_SDIO
extern int explorer_hal_sync_read_internal(struct explorer_plat_data *epd,u32 regoffset, u8 *out);
extern int explorer_hal_sync_write_internal(struct explorer_plat_data *epd,u32 regoffset, u8 in);
#endif

char *training_data_buf = NULL;
unsigned int training_data_size = 0;

extern int explorer_send_mbox(struct explorer_plat_data *epd, u32 sync, u32 mod_id, u32 sub_id, void *data);

char buf_mb[8] = {1,2,3,4,5,6,7,8};

void time_tag(struct explorer_plat_data *epd, const char *s)
{
#ifdef BOOT_TIME_LOG
	pr_info("boot info=%s, time_from_boot= %d ms.\n", s,
			jiffies_to_msecs(jiffies - epd->ebs.start_jiffies));
#endif
}

static int explorer_read_glb_reg(struct explorer_plat_data *epd, u32 addr, u32 *data)
{
#ifdef USE_RAW_SDIO
	return epd ? sdio_read_data(epd->sdio_data, addr, data, 4) : -1;
#else
	return epd ? explorer_hal_sync_read(epd, addr, data, 4) : -1;
#endif
}

static int explorer_write_glb_reg(struct explorer_plat_data *epd, u32 addr, u32 data)
{
#ifdef USE_RAW_SDIO
	return epd ? sdio_write_data(epd->sdio_data, addr, &data, 4) : -1;
#else
    return epd ? explorer_hal_sync_write(epd, addr, &data, 4) : -1;
#endif
}

int explorer_set_glb_reg_bits(struct explorer_plat_data *epd, u32 addr, u32 val)
{
	int ret = 0;
	unsigned int rega = 0;

	if (!epd) {
		return -R_BAD_ARG;
	}

	ret = explorer_read_glb_reg(epd, addr, &rega);
	if (ret < 0) {
		return ret;
	}

	ret = explorer_write_glb_reg(epd, addr, rega | val);

	return ret;
}

int explorer_clr_glb_reg_bits(struct explorer_plat_data *epd, u32 addr, u32 val)
{
	int ret = 0, trycnt = 10;
	unsigned int rega = 0xFFFFFFFF, regb = 0;

	if (!epd) {
		return -R_BAD_ARG;
	}

	ret = explorer_read_glb_reg(epd, addr, &rega);
	if (ret < 0) {
		return ret;
	}

	/* to avoid influence of "cross clock domain", we read glb reg twice */
	while (trycnt-- > 0) {
		ret = explorer_read_glb_reg(epd, addr, &regb);
		if (ret < 0) {
			return ret;
		}
		if (rega == regb) {
			break;
		}
		rega = regb;
	}

	if (trycnt > 0) {
		ret = explorer_write_glb_reg(epd, addr, rega & (~val));
	} else {
		ret = -R_REG_VAL_CHG;
	}

	return ret;
}

static int explorer_read_reg(struct explorer_plat_data *epd, u32 addr, u8 *data)
{
#ifdef USE_RAW_SDIO
	return epd ? sdio_read_register(epd->sdio_data, addr, data, 0) : -1;
#else
	return epd ? explorer_hal_sync_read_internal(epd, addr, data) : -1;
#endif
}

static int explorer_write_reg(struct explorer_plat_data *epd, u32 addr, u8 data)
{
#ifdef USE_RAW_SDIO
	return epd ? sdio_write_register(epd->sdio_data, addr, data, 0) : -1;
#else
	return epd ? explorer_hal_sync_write_internal(epd, addr, data) : -1;
#endif
}

static int explorer_set_reg_bits(struct explorer_plat_data *epd, u32 addr, u8 val)
{
	int ret = 0;
	unsigned char reg = 0;

	if (!epd) {
		return -R_BAD_ARG;
	}

	ret = explorer_read_reg(epd, addr, &reg);
	if (ret < 0)
		return ret;

	ret = explorer_write_reg(epd, addr, reg | val);

	return ret;
}

static int explorer_clr_reg_bits(struct explorer_plat_data *epd, u32 addr, u8 val)
{
	int ret = 0;
	unsigned char reg = 0;

	if (!epd) {
		return -R_BAD_ARG;
	}

	ret = explorer_read_reg(epd, addr, &reg);
	if (ret < 0)
		return ret;

	ret = explorer_write_reg(epd, addr, reg & (~val));

	return ret;
}

int explorer_load_buffer(struct explorer_plat_data *epd, u32 addr)
{
	int ret = 0, dsize, fsize, wpos, trycnt;
	char *dbuf;
	dbuf = epd->trans_buf;
	if (!dbuf) {
		AP_BOOT_ERR("transfer bufffer Null\n");
		return -ENOMEM;
	}
	fsize = training_data_size;
	wpos = 0;
	trycnt = 10;
	while(fsize > 0) {
		dsize = fsize >= FW_WRITE_BUF_SIZE ? FW_WRITE_BUF_SIZE : fsize;
		memcpy(dbuf, training_data_buf + wpos, dsize);
#ifdef USE_RAW_SDIO
		ret = sdio_write_data(epd->sdio_data, addr + wpos, dbuf, dsize);
#else
		ret = explorer_hal_sync_write(epd, addr + wpos, dbuf, dsize);
#endif
		if (ret < 0) {
			AP_BOOT_ERR("write sdio data to addr %08X fail, try left[%d]\n",
						addr, trycnt);
			if (trycnt-- > 0) {
				continue;
			}
		}
		wpos += dsize;
		fsize -= dsize;
	}
	return ret;
}

int explorer_load_fw(struct explorer_plat_data *epd, const char *firmware_name, u32 addr)
{
	int ret = 0, dsize, fsize, wpos, trycnt;
	const struct firmware *fw;
	char *dbuf;

	if (!epd || !firmware_name) {
		return -R_BAD_ARG;
	}

	dbuf = epd->trans_buf;
	if (!dbuf) {
		AP_BOOT_ERR("transfer bufffer Null\n");
		return -ENOMEM;
	}

	AP_BOOT_INFO("load firmware: %s 0x%08x\n", firmware_name, addr);

	ret = request_firmware(&fw, firmware_name, &epd->plat_dev->dev);
	if (ret < 0) {
		AP_BOOT_ERR("failed to request firmware: %s\n", firmware_name);
		return -R_FW_REQ_FAIL;
	}

	fsize = fw->size;
	wpos = 0;
	trycnt = 10;
	while(fsize > 0) {
		dsize = fsize >= FW_WRITE_BUF_SIZE ? FW_WRITE_BUF_SIZE : fsize;
		memcpy(dbuf, fw->data + wpos, dsize);
#ifdef USE_RAW_SDIO
		ret = sdio_write_data(epd->sdio_data, addr + wpos, dbuf, dsize);
#else
		ret = explorer_hal_sync_write(epd, addr + wpos, dbuf, dsize);
#endif
		if (ret < 0) {
			AP_BOOT_ERR("write sdio data to addr %08X fail, try left[%d]\n",
						addr, trycnt);
			if (trycnt-- > 0) {
				continue;
			}
		}
		wpos += dsize;
		fsize -= dsize;
	}

	release_firmware(fw);

	return ret;
}

int explorer_intc_config(struct explorer_plat_data *epd)
{
	int ret = 0;
	unsigned en_bits = 0;

	/* clear and mask int_reset_done IRQ status */
	AP_BOOT_INFO("===== set int masks.\n");

	en_bits |= 0x1 << CC_BUS_MONITOR_IRQ;
	en_bits |= 0x1 << CC_WDT_BITE_IRQ;
	en_bits |= 0x1 << CC2AP_MBOX_IRQ;
	en_bits |= 0x1 << MIPI_SOF_IRQ;
	en_bits |= 0x1 << ISP0_IRQ;
	en_bits |= 0x1 << ISP1_IRQ;
	en_bits |= 0x1 << PMU_WAKEUP_IRQ;
	en_bits |= 0x1 << PMU_SLEEP_IRQ;

	ret = explorer_write_glb_reg(epd, CC_INTC0_L_EN, en_bits);
	if (ret < 0)
		goto out;

	ret = explorer_write_glb_reg(epd, CC_INTC0_L_MASK, (~en_bits));
	if (ret < 0)
		goto out;

	ret = explorer_write_glb_reg(epd, CC_INTC1_L_EN, en_bits);
	if (ret < 0)
		goto out;

	ret = explorer_write_glb_reg(epd, CC_INTC1_L_MASK, (~en_bits));
	if (ret < 0)
		goto out;

	ret = explorer_write_glb_reg(epd, CC_INTC2_L_EN, en_bits);
	if (ret < 0)
		goto out;

	ret = explorer_write_glb_reg(epd, CC_INTC2_L_MASK, (~en_bits));
	if (ret < 0)
		goto out;

	ret = explorer_write_glb_reg(epd, CC_INTC3_L_EN, en_bits);
	if (ret < 0)
		goto out;

	ret = explorer_write_glb_reg(epd, CC_INTC3_L_MASK, (~en_bits));

out:
	return ret;
}

int explorer_stop_pbl_log(struct explorer_plat_data *epd)
{
	int ret = 0;

	ret = explorer_set_reg_bits(epd, SDU_AP_CTRL_REG7, PBL_LOG_BUF_STOP);

	return ret;
}

int explorer_clear_pbl_log(struct explorer_plat_data *epd)
{
	int ret = 0;

	ret = explorer_clr_reg_bits(epd, SDU_AP_CTRL_REG7, PBL_LOG_BUF_STOP);

	return ret;
}
void explorer_get_pbl_log_config(struct explorer_plat_data *epd, struct explorer_log_reply *reply) {
#ifdef SLT_ENABLE
	if (epd->ebi.target_stage == PBL_SLT) {
	#ifdef PBL_2nd
		reply->addr = SLT_PBL_LOG_ADDR;
		reply->len = SLT_PBL_LOG_LEN;
	#endif
	} else if ((epd->ebi.rmod == RTOS_SLT) || (epd->ebi.rmod == RTOS_SLT_AON)) {
		if ((atomic_read(&epd->ebs.current_stage) < OS_OK)
			&& (atomic_read(&epd->ebs.current_stage) >= PBL_NORMAL)) {
			reply->addr = NORMAL_PBL_LOG_ADDR;
			reply->len = NORMAL_PBL_LOG_LEN;
		}
	} else {
#endif
		if ((atomic_read(&epd->ebs.current_stage) >= PBL_NORMAL)
			&& (atomic_read(&epd->ebs.rtos_on) == 0)) {
			reply->addr = NORMAL_PBL_LOG_ADDR;
			reply->len = NORMAL_PBL_LOG_LEN;
		}
#ifdef SLT_ENABLE
	}
#endif
}

int explorer_sdu_reg_config(struct explorer_plat_data *epd)
{
	int ret = 0;

	/* config platform type: qcom or mtk*/
	if (epd->ebi.pmod == PLAT_MTK) {
		ret = explorer_set_reg_bits(epd, SDU_AP_CTRL_REG7, PLAT_TYPE_BIT_L); //mtk
		ret = explorer_clr_reg_bits(epd, SDU_AP_CTRL_REG7, PLAT_TYPE_BIT_H);
		if (ret < 0)
			goto out;
	} else {
		ret = explorer_clr_reg_bits(epd, SDU_AP_CTRL_REG7, PLAT_TYPE_BIT_L); //qcom
		ret = explorer_clr_reg_bits(epd, SDU_AP_CTRL_REG7, PLAT_TYPE_BIT_H);
		if (ret < 0)
			goto out;
	}

	/* config mipi bypass flag */
	if (epd->ebi.mmod == MIPI_BYPASS) {
		ret = explorer_set_reg_bits(epd, SDU_AP_CTRL_REG7, MIPI_BP_EN);
		if (ret < 0)
			goto out;

		ret = explorer_set_reg_bits(epd, SDU_AP_CTRL_REG7, MIPI_BP_EN_VLD);
		if (ret < 0)
			goto out;

	} else {
		ret = explorer_clr_reg_bits(epd, SDU_AP_CTRL_REG7, MIPI_BP_EN);
		if (ret < 0)
			goto out;

		ret = explorer_set_reg_bits(epd, SDU_AP_CTRL_REG7, MIPI_BP_EN_VLD);
		if (ret < 0)
			goto out;
	}

	/* config ddr fw type */
	if (epd->ebi.dmod == DDR_FIRST) {
		ret = explorer_set_reg_bits(epd, SDU_AP_CTRL_REG1, DDR_FW_TYPE);
		if (ret < 0)
			goto out;

		ret = explorer_set_reg_bits(epd, SDU_AP_CTRL_REG1, DDR_FW_TYPE_VLD);
		if (ret < 0)
			goto out;
	} else {
		ret = explorer_clr_reg_bits(epd, SDU_AP_CTRL_REG1, DDR_FW_TYPE);
		if (ret < 0)
			goto out;

		ret = explorer_set_reg_bits(epd, SDU_AP_CTRL_REG1, DDR_FW_TYPE_VLD);
		if (ret < 0)
			goto out;
	}

	if (epd->ebi.rpos == RTOS_DDR) {
		ret = explorer_set_reg_bits(epd, SDU_AP_CTRL_REG7, RTOS_LOAD_POS); //set this to inform rtos in DDR
		if (ret < 0)
			goto out;
	} else {
		ret = explorer_clr_reg_bits(epd, SDU_AP_CTRL_REG7, RTOS_LOAD_POS); //clear this to inform rtos in SRAM
		if (ret < 0)
			goto out;
	}

	/* write project id */
	ret = explorer_write_project_id(epd);
	if (ret < 0)
		pr_err("%s, write project id failed.\n", __func__);
out:
	return ret;
}

int explorer_ram_boot_chk(struct explorer_plat_data *epd)
{
	int ret = 0;
	unsigned char reg = 0;

	if (!epd) {
		return -R_BAD_ARG;
	}

	/* judge 0x810 bit7 = 0 to tell if it's ram boot. */
	ret = explorer_read_reg(epd, SDU_BOOT_STAT_REG0, &reg);
	if (ret < 0) {
		return ret;
	} else if (!(reg & SDU_BOOT_STAT_ROM_BOOT)) {
		/* if so, send bootrom code & release CPU.*/
		time_tag(epd, "load-bootrom start");

		ret = explorer_load_fw(epd, BOOTROM_NAME, FW_BOOTROM_ADDR);
		if (ret < 0) {
			AP_BOOT_ERR("transfer br_code failed.\n");
			return ret;
		}

		ret = explorer_set_glb_reg_bits(epd, GLB_AP_CTRL_BOOT_STEP,
			BOOT_TRANS_BIT_BOOTROM | BOOT_STEP_BOOTROM_DONE);
		if (ret < 0) {
			AP_BOOT_ERR("set br_trans_done failed.\n");
			return ret;
		}

		time_tag(epd, "load-bootrom end");

		ret = explorer_set_glb_reg_bits(epd, GLB_CPU_RELEASE_RESET, CPU_RELEASE_RESET);
		if (ret < 0) {
			AP_BOOT_ERR("set glb_cpu_release failed.\n");
			return ret;
		}

	}
	return 0;
}

int explorer_abnormal_reboot_chk(struct explorer_plat_data *epd)
{
	int ret = 0, try_cnt;
	unsigned char reg = 0, bits;


	/* check if abnornal reboot */
	AP_BOOT_INFO("===== check abnormal reboot.\n");
	ret = explorer_read_reg(epd, SDU_BOOT_STAT_REG1, &reg);
	if (ret < 0) {
		AP_BOOT_ERR("read abnornal-reboot flag failed.\n");
		return -1;
	} else if (reg & SDU_BOOT_STAT_ABNORMAL_BOOT) {
		AP_BOOT_INFO("abnormal reboot\n");
	} else {
		AP_BOOT_INFO("normal boot\n");
		return 0;
	}

	/*wait for SDI_CRC_CHK OK*/
	try_cnt = 20000; //TODO
	while(try_cnt-- > 0) {
		reg = 0;
		bits = SDU_BOOT_STAT_CRC_CHK_PASS_SDI;

		AP_BOOT_INFO("===== polling sdi_crc_chk ok.\n");
		ret = explorer_read_reg(epd, SDU_BOOT_STAT_REG2, &reg);
		if (!ret && ((reg & bits) == bits)) {
			break;
		}

		mdelay(1);
	}

	if (try_cnt <= 0) {
		AP_BOOT_ERR("verify sdi_crc_chk timeout!\n");
		return -1;
	}

	return R_ABN_RBT_OK;
}

int explorer_load_remedy(struct explorer_plat_data *epd)
{
	int ret = 0;

	if (!epd) {
		return -R_BAD_ARG;
	}

	/* transfer remedy to CC and set TRANS_DONE_REMEDY */
	time_tag(epd, "load-remedy start");

	/*
	 * for status of the chip. don't need any remedy.
	 * so remove the searching & load of remedy to save time.
	 * ret = explorer_load_fw(epd, REMEDY_NAME, FW_REMEDY_ADDR);
	 * if (ret == -R_FW_REQ_FAIL) {
	 * 	AP_BOOT_ERR("No remedy packet.\n");
	 * } else if (ret < 0) {
	 * 	AP_BOOT_ERR("transfer remedy failed.\n");
	 * 	return ret;
	 * }
	 */

	ret = explorer_set_glb_reg_bits(epd, GLB_AP_CTRL_BOOT_STEP,
		BOOT_TRANS_BIT_REMEDY | BOOT_STEP_REMEDY_DONE);
	if (ret < 0) {
		AP_BOOT_ERR("set remedy_trans_done failed.\n");
		return ret;
	}

	time_tag(epd, "load-remedy end");

	return 0;
}

int explorer_check_sec(struct explorer_plat_data *epd)
{
	int ret = 0;
	unsigned char reg = 0;

	/* check if secure boot */
	AP_BOOT_INFO("===== check secure boot.\n");
	ret = explorer_read_reg(epd, SDU_BOOT_STAT_REG1, &reg);
	if (ret < 0) {
		AP_BOOT_ERR("read secure-boot flag failed.\n");
		return ret;
	} else if (reg & SDU_BOOT_STAT_SECURE_BOOT) {
		AP_BOOT_INFO("start secure boot\n");
		epd->ebs.is_cc_sec_boot = true;
	} else {
		AP_BOOT_INFO("start non-secure boot\n");
		epd->ebs.is_cc_sec_boot = false;
	}

	return 0;
}
int explorer_load_se(struct explorer_plat_data *epd, unsigned long deadline)
{
	int ret = 0;
	unsigned char reg = 0, bits;
	bool done = false;

	if (epd->ebs.is_cc_sec_boot == false) {
		return 0;
	}

	time_tag(epd, "load-se-hdr start");

#ifdef SLT_ENABLE
	if(epd->ebi.target_stage == PBL_SLT
		|| epd->ebi.rmod == RTOS_SLT
		|| epd->ebi.rmod == RTOS_SLT_AON) {
		ret = explorer_load_fw(epd, SE_HEAD_NAME, FW_SE_HEAD_ADDR);
		if (ret < 0) {
			ret = explorer_load_stl_se_header(epd);
		}
	} else {
#endif
		ret = explorer_load_fw(epd, SE_HEAD_NAME, FW_SE_HEAD_ADDR);
#ifdef SLT_ENABLE
	}
#endif
	if (ret < 0) {
		AP_BOOT_ERR("load se header failed.\n");
		return ret;
	}

	ret = explorer_set_glb_reg_bits(epd, GLB_AP_CTRL_BOOT_STEP,
                 BOOT_TRANS_BIT_SE_HEAD_FW);
	if (ret < 0) {
		AP_BOOT_ERR("set se_hdr_trans_done failed.\n");
		return ret;
	}

	time_tag(epd, "load-se-hdr end");

	while(!time_after(jiffies, deadline)) {
		AP_BOOT_DBG("===== polling se_init_ready.\n");
		reg = 0;
		bits = SDU_STAT_SE_INIT_READY;

		ret = explorer_read_reg(epd, SDU_AP_STAT_REGC, &reg);
		if (ret >= 0 && ((reg & bits) == bits)) {
			time_tag(epd, "can-load-se-fw polled");
			done = true;
			break;
		}

		mdelay(1);
	}

	if (!done) {
		AP_BOOT_ERR("failed to get se init ready flag\n");
		return ret;
	}

	/* load SE fw */
	time_tag(epd, "load-se-fw start");
#ifdef SLT_ENABLE
	if(epd->ebi.target_stage == PBL_SLT
		|| epd->ebi.rmod == RTOS_SLT
		|| epd->ebi.rmod == RTOS_SLT_AON) {
		ret = explorer_load_fw(epd, SE_DATA_NAME, FW_SE_DATA_ADDR);

		if (ret < 0) {
			ret = explorer_load_stl_se_fw(epd);
		}
	} else {
#endif
		ret = explorer_load_fw(epd, SE_DATA_NAME, FW_SE_DATA_ADDR);
#ifdef SLT_ENABLE
}
#endif
	if (ret < 0) {
		AP_BOOT_ERR("load se fw failed\n");
		return ret;
	}

	explorer_set_glb_reg_bits(epd, GLB_AP_CTRL_BOOT_STEP,
                 BOOT_TRANS_BIT_SE_FW | BOOT_STEP_SE_DONE);
	if (ret < 0) {
		AP_BOOT_ERR("set se_fw_trans_done failed.\n");
		return ret;
	}

	time_tag(epd, "load-se-fw end");

	return 0;
}

int explorer_load_sdi(struct explorer_plat_data *epd)
{
	int ret = 0;

	/* fws needed in both sec & non-sec boot */
	time_tag(epd, "load-sdi start");
#ifdef SLT_ENABLE
	if(epd->ebi.target_stage == PBL_SLT
		|| epd->ebi.rmod == RTOS_SLT
		|| epd->ebi.rmod == RTOS_SLT_AON) {
		ret = explorer_load_stl_sdi(epd);
	} else {
#endif
		if (epd->ebs.is_cc_sec_boot) {
			ret = explorer_load_fw(epd, SDI_SEC_NAME, FW_SDI_ADDR);
		} else {
			ret = explorer_load_fw(epd, SDI_NAME, FW_SDI_ADDR);
		}
#ifdef SLT_ENABLE
	}
#endif

	if (ret == -R_FW_REQ_FAIL) {
		AP_BOOT_ERR("No SDI FW.\n");
		return ret;
	} else if (ret < 0) {
		AP_BOOT_ERR("load SDI failed.\n");
		return ret;
	}

	ret = explorer_set_glb_reg_bits(epd, GLB_AP_CTRL_BOOT_STEP,
			BOOT_TRANS_BIT_SDI_FW);
	if (ret < 0) {
		AP_BOOT_ERR("set sdi_trans_done failed.\n");
		return ret;
	}

	time_tag(epd, "load-sdi end");

	return ret;
}

int explorer_load_ddr(struct explorer_plat_data *epd)
{
	int ret = 0;

	/* fws needed in both sec & non-sec boot */
	time_tag(epd, "load-ddrfw start");
	if (epd->ebi.dmod == DDR_QUICK) {
		ret = explorer_load_fw(epd, DDR_QCK_NAME, FW_DDR_ADDR);
		if(ret < 0) {
			AP_BOOT_ERR("send ddr_quick.bin fail!");
			pr_err("%s, error code: %d !\n", __func__, ret);
			return ret;
		}
		ret = explorer_load_fw(epd, DDR_QCK_DATA_NAME, FW_DDR_DATA_ADDR);
		if(ret < 0) {
			AP_BOOT_ERR("send ddr_data.bin fail!");
			pr_err("%s, error code: %d !\n", __func__, ret);
			return ret;
		}
	} else if (epd->ebi.dmod == DDR_BUFFER){
		ret = explorer_load_fw(epd, DDR_QCK_NAME, FW_DDR_ADDR);
		if(ret < 0) {
			AP_BOOT_ERR("DDR_BUFFER send ddr_quick.bin fail!");
			pr_err("%s, error code: %d !\n", __func__, ret);
			return ret;
		}
		ret = explorer_load_buffer(epd, FW_DDR_DATA_ADDR);
		if(ret < 0) {
			AP_BOOT_ERR("DDR_BUFFER send ddr_data.bin buffer fail!");
			pr_err("%s, error code: %d !\n", __func__, ret);
			return ret;
		}
	} else {
		ret = explorer_load_fw(epd, DDR_1ST_NAME, FW_DDR_ADDR);
	}
	if (ret == -R_FW_REQ_FAIL) {
		AP_BOOT_ERR("No DDR FW.\n");
	} else if (ret < 0) {
		AP_BOOT_ERR("load DDR fw failed.");
		return ret;
	}

	ret = explorer_set_glb_reg_bits(epd, GLB_AP_CTRL_BOOT_STEP,
			BOOT_TRANS_BIT_DDR_FW);
	if (ret < 0) {
		AP_BOOT_ERR("set ddr_fw_trans_done failed.\n");
		return ret;
	}

	time_tag(epd, "load-ddrfw end");

	return ret;
}

int explorer_load_pbl(struct explorer_plat_data *epd)
{
	int ret = 0;

	/* fws needed in both sec & non-sec boot */
	time_tag(epd, "load-pbl start");

	switch (epd->ebi.target_stage) {
#ifdef SLT_ENABLE
		case PBL_SLT:
			ret = explorer_load_stl_pbl(epd);
			break;
#endif
		case PBL_PROV:
			//do nothing
			break;
		default:
#ifdef SLT_ENABLE
			if(epd->ebi.rmod == RTOS_SLT || epd->ebi.rmod == RTOS_SLT_AON) {
				ret = explorer_load_stl_normal_pbl(epd);
			} else {
#endif
				if (epd->ebs.is_cc_sec_boot) {
					ret = explorer_load_fw(epd, PBL_SEC_NAME, FW_PBL_ADDR);
				} else {
					ret = explorer_load_fw(epd, PBL_NAME, FW_PBL_ADDR);
				}
#ifdef SLT_ENABLE
			}
#endif
			break;
	}

	if (ret < 0) {
		AP_BOOT_ERR("load PBL failed.");
		return ret;
	}

	ret = explorer_set_glb_reg_bits(epd, GLB_AP_CTRL_BOOT_STEP,
			BOOT_TRANS_BIT_PBL_FW | BOOT_STEP_3_FW_DONE);

	if (ret < 0) {
		AP_BOOT_ERR("set pbl_trans_done failed.\n");
		return ret;
	}

#ifdef PBL_2nd
	ret = explorer_set_glb_reg_bits(epd, GLB_AP_CTRL_BOOT_STEP,
		BOOT_TRANS_BIT_2ND_PBL);
	if (ret < 0) {
		AP_BOOT_ERR("set 2nd_pbl_trans_done failed.\n");
		return ret;
	}
#endif
	time_tag(epd, "load-pbl end");

	return ret;
}

int explorer_load_os(struct explorer_plat_data *epd)
{
	int ret = 0;

	time_tag(epd, "load-os-fw start");

	switch (epd->ebi.rmod) {
#ifdef SLT_ENABLE
		case RTOS_SLT:
		case RTOS_SLT_AON:
			ret = explorer_load_stl_os(epd);
			break;
#endif
		case RTOS_AON:
			if (epd->ebs.is_cc_sec_boot) {
				ret = explorer_load_fw(epd, OS_AON_SEC_NAME, epd->ebi.rpos==RTOS_SRAM ? FW_OS_ADDR:FW_OS_DDR_ADDR);
			} else {
				ret = explorer_load_fw(epd, OS_AON_NAME, epd->ebi.rpos==RTOS_SRAM ? FW_OS_ADDR:FW_OS_DDR_ADDR);
			}
			break;
		case RTOS_PLAT:
			if (epd->ebs.is_cc_sec_boot) {
				ret = explorer_load_fw(epd, OS_PLAT_SEC_NAME, epd->ebi.rpos==RTOS_SRAM ? FW_OS_ADDR:FW_OS_DDR_ADDR);
			} else {
				ret = explorer_load_fw(epd, OS_PLAT_NAME, epd->ebi.rpos==RTOS_SRAM ? FW_OS_ADDR:FW_OS_DDR_ADDR);
			}
			break;
		case RTOS_RLS:
			if (epd->ebs.is_cc_sec_boot) {
				ret = explorer_load_fw(epd, OS_RLS_SEC_NAME, epd->ebi.rpos==RTOS_SRAM ? FW_OS_ADDR:FW_OS_DDR_ADDR);
			} else {
				ret = explorer_load_fw(epd, OS_RLS_NAME, epd->ebi.rpos==RTOS_SRAM ? FW_OS_ADDR:FW_OS_DDR_ADDR);
			}
			break;
		case RTOS_DBG_NOPM:
			if (epd->ebs.is_cc_sec_boot) {
				ret = explorer_load_fw(epd, OS_DBG_NOPM_SEC_NAME, epd->ebi.rpos==RTOS_SRAM ? FW_OS_ADDR:FW_OS_DDR_ADDR);
			} else {
				ret = explorer_load_fw(epd, OS_DBG_NOPM_NAME, epd->ebi.rpos==RTOS_SRAM ? FW_OS_ADDR:FW_OS_DDR_ADDR);
			}
			break;
		case RTOS_RLS_NOPM:
			if (epd->ebs.is_cc_sec_boot) {
				ret = explorer_load_fw(epd, OS_RLS_NOPM_SEC_NAME, epd->ebi.rpos==RTOS_SRAM ? FW_OS_ADDR:FW_OS_DDR_ADDR);
			} else {
				ret = explorer_load_fw(epd, OS_RLS_NOPM_NAME, epd->ebi.rpos==RTOS_SRAM ? FW_OS_ADDR:FW_OS_DDR_ADDR);
			}
			break;
		default:
			//RTOS_NORMAL
			if (epd->ebs.is_cc_sec_boot) {
				ret = explorer_load_fw(epd, OS_SEC_NAME, epd->ebi.rpos==RTOS_SRAM ? FW_OS_ADDR:FW_OS_DDR_ADDR);
			} else {
				ret = explorer_load_fw(epd, OS_NAME, epd->ebi.rpos==RTOS_SRAM ? FW_OS_ADDR:FW_OS_DDR_ADDR);
			}
			break;
	}

	if (ret < 0) {
		AP_BOOT_ERR("load OS fw failed.");
		return ret;
	}

	ret = explorer_set_glb_reg_bits(epd, GLB_AP_CTRL_BOOT_STEP,
			BOOT_TRANS_BIT_OS_IMG |
			BOOT_STEP_OS_DONE);
	if (ret < 0) {
		AP_BOOT_ERR("set os trans_done failed.\n");
		return ret;
	}
	AP_BOOT_INFO("set os trans_done bit.\n");

	time_tag(epd, "load-os-fw end");

	return ret;
}

int explorer_load_npu(struct explorer_plat_data *epd)
{
	int ret = 0;
	int data[2] = {0, 0};

	if (epd->cmod >= CAM_MAX) {
		return -R_BAD_CMOD;
	}

	/* dowand NPU imgs */
	time_tag(epd, "load-npu-fw start");
	if (epd->cmod == CAM_AON) {
		if (epd->ebs.is_cc_sec_boot) {
			ret = explorer_load_fw(epd, NPU_AON_SEC_NAME, FW_NPU_AON_ADDR);
		} else {
			ret = explorer_load_fw(epd, NPU_AON_NAME, FW_NPU_AON_ADDR);
		}
	} else {
		switch (epd->cmod) {
		case CAM_BZ_BK:
			if (epd->ebs.is_cc_sec_boot) {
				ret = explorer_load_fw(epd, NPU_BZ_BK_SEC_NAME, FW_NPU_SEC_ADDR);
			} else {
				ret = explorer_load_fw(epd, NPU_BZ_BK_NAME, FW_NPU_ADDR);
			}
			break;
		case CAM_BZ_FR:
			if (epd->ebs.is_cc_sec_boot) {
				ret = explorer_load_fw(epd, NPU_BZ_FR_SEC_NAME, FW_NPU_SEC_ADDR);
			} else {
				ret = explorer_load_fw(epd, NPU_BZ_FR_NAME, FW_NPU_ADDR);
			}
			break;
		case CAM_BZ_BK_BS:
			if (epd->ebs.is_cc_sec_boot) {
				ret = explorer_load_fw(epd, NPU_BZ_BK_BS_SEC_NAME, FW_NPU_SEC_ADDR);
			} else {
				ret = explorer_load_fw(epd, NPU_BZ_BK_BS_NAME, FW_NPU_ADDR);
			}
			break;
		case CAM_BZ_FR_BS:
			if (epd->ebs.is_cc_sec_boot) {
				ret = explorer_load_fw(epd, NPU_BZ_FR_BS_SEC_NAME, FW_NPU_SEC_ADDR);
			} else {
				ret = explorer_load_fw(epd, NPU_BZ_FR_BS_NAME, FW_NPU_ADDR);
			}
			break;
		case CAM_LW_BK:
			if (epd->ebs.is_cc_sec_boot) {
				ret = explorer_load_fw(epd, NPU_LW_BK_SEC_NAME, FW_NPU_SEC_ADDR);
			} else {
				ret = explorer_load_fw(epd, NPU_LW_BK_NAME, FW_NPU_ADDR);
			}
			break;
		case CAM_LW_FR:
			if (epd->ebs.is_cc_sec_boot) {
				ret = explorer_load_fw(epd, NPU_LW_FR_SEC_NAME, FW_NPU_SEC_ADDR);
			} else {
				ret = explorer_load_fw(epd, NPU_LW_FR_NAME, FW_NPU_ADDR);
			}
		      break;
		case CAM_LW_BK_BS:
			if (epd->ebs.is_cc_sec_boot) {
				ret = explorer_load_fw(epd, NPU_LW_BK_BS_SEC_NAME, FW_NPU_SEC_ADDR);
			} else {
				ret = explorer_load_fw(epd, NPU_LW_BK_BS_NAME, FW_NPU_ADDR);
			}
		      break;
		case CAM_LW_FR_BS:
			if (epd->ebs.is_cc_sec_boot) {
				ret = explorer_load_fw(epd, NPU_LW_FR_BS_SEC_NAME, FW_NPU_SEC_ADDR);
			} else {
				ret = explorer_load_fw(epd, NPU_LW_FR_BS_NAME, FW_NPU_ADDR);
			}
			break;
		default:
			if (epd->ebs.is_cc_sec_boot) {
				ret = explorer_load_fw(epd, NPU_BZ_BK_SEC_NAME, FW_NPU_SEC_ADDR);
			} else {
				ret = explorer_load_fw(epd, NPU_BZ_BK_NAME, FW_NPU_ADDR);
			}
			break;
		}
	}

	if (ret < 0) {
		AP_BOOT_ERR("load NPU imgs failed.");
		return ret;
	}

	ret = explorer_set_glb_reg_bits(epd, GLB_AP_CTRL_BOOT_STEP,
			BOOT_TRANS_BIT_NPU_IMG |
			BOOT_STEP_NPU_DONE);
	if (ret < 0) {
		AP_BOOT_ERR("set npu trans_done failed.\n");
		return ret;
	}
	AP_BOOT_INFO("set npu trans_done bit.\n");

	ret = explorer_send_mbox(epd, HAL_CMD_ASYNC, HAL_CMD_NPU, NPU_SUB_CMD_TRANS_DONE, data);
	if (ret < 0) {
		AP_BOOT_ERR("===== send NPU_trans_done mbox failed.\n");
		return ret;
	}
	AP_BOOT_INFO("===== send NPU_trans_done mbox OK.\n");

	time_tag(epd, "load-npu-fw end");

	return ret;
}

int explorer_load_npu_custom(struct explorer_plat_data *epd)
{
	int ret = 0;
	int data[2] = {0, 0};

	if (epd->nmod_info == NULL) {
		return -R_BAD_CMOD;
	}

	/* download NPU imgs */
	time_tag(epd, "load-npu-fw start");
	ret = explorer_load_fw(epd, epd->nmod_info->name, epd->nmod_info->addr);
	if (ret < 0) {
		AP_BOOT_ERR("load NPU imgs custom failed.");
		return ret;
	}
	time_tag(epd, "load-npu-fw end");


	data[0] = epd->nmod_info->index;
	ret = explorer_send_mbox(epd, HAL_CMD_ASYNC, HAL_CMD_NPU, NPU_SUB_CMD_TRANS_DONE, data);
	if (ret < 0) {
		AP_BOOT_ERR("===== send NPU_trans_done mbox failed.\n");
		return ret;
	}
	time_tag(epd, "send-npu-mbox ok");

	return ret;
}
#ifndef AUTO_TRIG
int stat_timeout_to_errcode(enum wait_stage stage) {
	int ret = 0;

	switch (stage) {
		case WAIT_SE_OK:
			AP_BOOT_ERR("wait se init timeout\n");
			ret = -R_SE_INIT_TO;
			break;
		case WAIT_PBL_OK:
			AP_BOOT_ERR("verify SDI, PBL, DDR failed!\n");
			ret = -R_PBL_VER_TO;
			break;
		case WAIT_DDR_OK:
			AP_BOOT_ERR("wait ddr init failed\n");
			ret = -R_DDR_INIT_TO;
			break;
		case WAIT_MIPI_OK:
			AP_BOOT_ERR("wait mipi bypass failed\n");
			ret = -R_MIPI_BP_TO;
			break;
		case WAIT_OS_OK:
			AP_BOOT_ERR("wait os ver failed\n");
			ret =  -R_OS_VER_TO;
			break;
		case WAIT_NPU_OK:
			AP_BOOT_ERR("wait NPU ver failed\n");
			ret = -R_NPU_VER_TO;
			break;
#ifdef PBL_2nd
		case WAIT_2ND_PBL_OK:
			AP_BOOT_ERR("verify PBL_SLT failed!\n");
			ret = -R_SLT_VER_TO;
			break;
#endif
		default:
			break;
	}
	return ret;
}
int explorer_poll_stat(struct explorer_plat_data *epd, unsigned long deadline, enum wait_stage stage)
{
	int ret = 0;
	bool done = false;
	unsigned char reg = 0, bits = 0;
	u32 reg_addr = 0;

	//default setting for reg_addr;
	reg_addr = SDU_BOOT_STAT_REG2;

	switch (stage) {
		case WAIT_SE_OK:
			bits = SDU_BOOT_STAT_SE_INIT_DONE | SDU_BOOT_STAT_SE_INIT_SUCCESS;
			break;
		case WAIT_PBL_OK:
			bits = SDU_BOOT_STAT_VERIFY_PASS_PBL;
			break;
		case WAIT_DDR_OK:
			bits = SDU_BOOT_STAT_DDR_TRAINING_DONE;
			break;
		case WAIT_MIPI_OK:
			reg_addr = SDU_BOOT_STAT_REG0;
			bits = SDU_BOOT_STAT_MIPI_BP_OK;
			break;
		case WAIT_OS_OK:
			bits = SDU_BOOT_STAT_VERIFY_PASS_RTOS;
			break;
		case WAIT_NPU_OK:
			bits = SDU_BOOT_STAT_VERIFY_PASS_NPU;
			break;
#ifdef PBL_2nd
		case WAIT_2ND_PBL_OK:
			reg_addr = SDU_BOOT_STAT_REG9;
			bits = SDU_BOOT_STAT_VERIFY_PASS_SLT;
			break;
#endif
		default:
			break;
	}
	if (bits == 0) {
		//no state need wait, return directly;
		return 0;
	}
	/* poll flag */
	while(!time_after(jiffies, deadline)) {
		reg = 0;
		ret = explorer_read_reg(epd, reg_addr, &reg);
		if (!ret && ((reg & bits) == bits)) {
			AP_BOOT_INFO("===== Type(%d) verify pass.\n", stage);
			done = true;
			break;
		}
		if (!ret && (stage == WAIT_SE_OK)
			&& ((reg & bits) == SDU_BOOT_STAT_SE_INIT_DONE)) {
			AP_BOOT_ERR("===== se init finished but failed.\n");
			return -R_SE_INIT_FAIL;
		}
		if (!ret && ((stage == WAIT_DDR_OK) || (stage == WAIT_MIPI_OK))) {
			if (atomic_read(&epd->is_pmic_oc) == 1) {
				AP_BOOT_ERR("===== pmic hw error, boot failed.\n");
				report_power_exception(epd,
						EXCEPTION_POWER_MAJOR_TYPE_PMIC_HW_ERR,
						EXCEPTION_POWER_SUB_TYPE_PMIC_HW_ERR_BY_PBL_DETECT,
						EXCEPTION_ACT_NONE);
				return -R_PMIC_OC_FAIL;
			}
		}
		if (stage == WAIT_DDR_OK) {
			if (atomic_read(&epd->is_ddr_failed) == 1) {
				AP_BOOT_ERR("===== ddr already failed, boot failed.\n");
				return -R_DDR_INIT_FAIL;
			}
		}
		mdelay(1);
	}
	if (!done) {
		return stat_timeout_to_errcode(stage);
	}


	return 0;
}
#endif

int explorer_bootstage_pre(struct explorer_plat_data *epd)
{
	int ret = 0;

	epd->ebs.cur_os_type = UNKNOWN_OS;

	time_tag(epd, "stage-pre start");

	epd->ebs.is_cc_sec_boot = false; /* clear flags before every boot */

	ret = explorer_intc_config(epd);
	if (ret < 0)
		goto out;

	ret = explorer_ram_boot_chk(epd);
	if (ret < 0)
		goto out;
	//check secure boot or not before download
	ret = explorer_check_sec(epd);
	if (ret < 0)
		goto out;

	ret = explorer_load_remedy(epd);
	if (ret < 0)
		goto out;

	atomic_set(&epd->ebs.current_stage, BOOTROM);
	time_tag(epd, "stage-pre end");

out:
	return ret;
}

int explorer_bootstage_pbl(struct explorer_plat_data *epd, unsigned long deadline)
{
	int ret = 0;

	time_tag(epd, "stage-pbl start");
	if (epd->ebi.target_stage == PBL_PROV) {
		/* load pbl_prov from buffer */
		ret = explorer_load_fw(epd, PROV_NAME, FW_PBL_ADDR);
		if (ret < 0) {
			AP_BOOT_ERR("load prov failed, exit boot\n");
			goto out;
		}
		ret = explorer_set_glb_reg_bits(epd, GLB_AP_CTRL_BOOT_STEP,
				BOOT_TRANS_BIT_PBL_FW | BOOT_STEP_3_FW_DONE);
		if (ret < 0) {
			AP_BOOT_ERR("set prov_trans_done failed.\n");
			goto out;
		}
	} else {
		ret = explorer_load_se(epd, deadline);
		if (ret < 0)
			goto out;

		/* check PLL & do sdio tuning */
		ret = explorer_sdio_tuning(epd);
		if (ret < 0) {
			AP_BOOT_ERR("explorer_sdio_tuning failed, exit boot\n");
			goto out;
		}

		ret = explorer_load_sdi(epd);
		if (ret < 0)
			goto out;
#ifdef SLT_ENABLE
		if (epd->ebi.target_stage != PBL_SLT) {
#endif
			ret = explorer_load_ddr(epd);
			if (ret < 0)
				goto out;

#ifdef SLT_ENABLE
		}
#endif
		ret = explorer_load_pbl(epd);

		if (ret < 0)
			goto out;
	}
	time_tag(epd, "stage-pbl trans done");

out:
	return ret;
}

int explorer_ddr_continue_boot(struct explorer_plat_data *epd)
{
	int ret = 0;
	pr_info("%s, notify explorer ddr to continue boot.\n", __func__);
	/* set register to notify explorer to continue boot */
	ret = explorer_set_reg_bits(epd, SDU_AP_CTRL_REG7, DDR_TRAIN_DATA_DONE);
	if (ret < 0)
		pr_err("%s, ID_DDR_TRAIN_DATA set ddr_training_data_done flag fail. \n", __func__);

	return ret;
}

int explorer_clear_ddr_boot_flag(struct explorer_plat_data *epd)
{
	int ret = 0;
	pr_info("%s, ddr ok, clear training data flag.\n", __func__);
	/* clear register flag */
	ret = explorer_clr_reg_bits(epd, SDU_AP_CTRL_REG7, DDR_TRAIN_DATA_DONE);
	if (ret < 0)
		pr_err("%s, ID_DDR_TRAIN_DATA clear ddr_training_data_done flag fail. \n", __func__);

	return ret;
}


int explorer_store_ddr_training_data(struct explorer_plat_data *epd, struct hal_comm_data *comm_data)
{
	int ret = 0;
	u32 cc_addr = comm_data->data[0];
	u32 data_size = comm_data->data[1];
	void *ap_buffer = NULL;
	struct device *dev = &(epd->plat_dev->dev);

	ap_buffer = devm_kzalloc(dev, data_size, GFP_KERNEL);
	if (!ap_buffer) {
		pr_err("%s, kzalloc failed.\n", __func__);
		return -ENOMEM;
	}

	pr_info("%s, start: cc_addr:%d, data_size:%d .\n", __func__, cc_addr, data_size);

	ret = explorer_hal_sync_read(epd, cc_addr, ap_buffer, data_size);
	if (ret < 0) {
		pr_err("%s, explorer_hal_sync_read trainging data failed.\n", __func__);
		goto freemem;
	}

	/* send cmd, netlink to userspace */
	ret = explorer_genl_mcast_data(epd, HAL_CMD_DDR_DATA, ap_buffer, data_size);
	if (ret < 0) {
		pr_err("%s, send store training data cmd, netlink to userspace failed.\n", __func__);
		goto freemem;
	}

	pr_info("%s,store training data done.\n", __func__);
freemem:
	devm_kfree(dev, ap_buffer);

	return ret;
}


int explorer_bootstage_os(struct explorer_plat_data *epd)
{
	int ret = 0;

	time_tag(epd, "stage-os-fw start");
	ret = explorer_load_os(epd);
	if (ret < 0)
		return ret;

	time_tag(epd, "stage-os-fw trans done");
	return ret;
}

int explorer_bootstage_npu(struct explorer_plat_data *epd)
{
	int ret = 0;
	struct exception_info info;
	info.moduleId = EXCEPTION_BOOT;

	time_tag(epd, "stage-npu-fw start");
	ret = explorer_load_npu(epd);
	if (ret < 0)
		return ret;

	info.majorType = MAJOR_TYPE_STAGE;
	info.subType = STAGE_TYPE_NPU_DONE;
	explorer_genl_mcast_data(epd, HAL_CMD_EXCEPTION2AP, &info, sizeof(struct exception_info));

	time_tag(epd, "stage-npu-fw trans done");
	return ret;
}

#ifndef AUTO_TRIG
int explorer_wait_bootstage(struct explorer_plat_data *epd, enum boot_stage wstg, unsigned long deadline)
{
	int ret = 0;
	struct exception_info info;

	info.moduleId = EXCEPTION_BOOT;

	switch (wstg) {
	case BOOTROM:
		break;
	case PBL_PROV:
		ret = explorer_poll_stat(epd, deadline, WAIT_PBL_OK);
		if (ret < 0) {
			return ret;
		}
		time_tag(epd, "pbl_prov was received by CC");
		break;
#ifdef SLT_ENABLE
	case PBL_SLT:
		/*
		  SLT mode still need mailbox message notification,
		  so here only wait 1st PBL, and not set current_stage;
		*/
		ret = explorer_poll_stat(epd, deadline, WAIT_PBL_OK);
		if (ret < 0) {
			return ret;
		}
		time_tag(epd, "stage-pbl slt end polled");
		break;
#endif
	case PBL_NORMAL:
		ret = explorer_poll_stat(epd, deadline, WAIT_PBL_OK);
		if (ret < 0) {
			return ret;
		}
		atomic_set(&epd->ebs.current_stage, PBL_NORMAL);
		time_tag(epd, "stage-pbl end polled");
		break;
	case MIPI_BP_OK:
		if (epd->ebi.mmod == MIPI_BYPASS) {
			ret = explorer_poll_stat(epd, deadline, WAIT_MIPI_OK);
			if (ret < 0)
				return ret;
		}
		atomic_set(&epd->ebs.current_stage, MIPI_BP_OK);
		time_tag(epd, "stage-mipi-bp end polled");
		break;
	case DDR_OK:
		ret = explorer_poll_stat(epd, deadline, WAIT_DDR_OK);
		if (ret < 0) {
			return ret;
		}
		atomic_set(&epd->ebs.current_stage, DDR_OK);
		time_tag(epd, "stage-ddr-train end polled");
		explorer_clear_ddr_boot_flag(epd);
		break;
	case OS_OK:
		ret = explorer_poll_stat(epd, deadline, WAIT_OS_OK);
		if (ret < 0) {
			return ret;
		}
		/* received when os starts ok */
		if (epd->ebi.rmod == RTOS_AON) {
			epd->ebs.cur_os_type = AON_OS;
		} else if (epd->ebi.rmod == RTOS_PLAT) {
			epd->ebs.cur_os_type = PLAT_OS;
		} else if (epd->ebi.rmod == RTOS_NORMAL || epd->ebi.rmod == RTOS_RLS ||
			   epd->ebi.rmod == RTOS_DBG_NOPM || epd->ebi.rmod == RTOS_RLS_NOPM) {
			epd->ebs.cur_os_type = NORMAL_OS;
		}
#ifdef SLT_ENABLE
		if ((epd->ebi.rmod != RTOS_SLT) && (epd->ebi.rmod != RTOS_SLT_AON)) {
#endif
			atomic_set(&epd->ebs.current_stage, OS_OK);
			time_tag(epd, "stage-os-fw end polled");
#ifdef SLT_ENABLE
		}
#endif
		if (atomic_read(&epd->ebs.rtos_on) == 0) {
			info.majorType = MAJOR_TYPE_STAGE;
			info.subType = STAGE_TYPE_RTOS_VERIFIED;
			explorer_genl_mcast_data(epd, HAL_CMD_EXCEPTION2AP, &info, sizeof(struct exception_info));
		}

		break;
	case NPU_OK:
		ret = explorer_poll_stat(epd, deadline, WAIT_NPU_OK);
		if (ret < 0) {
			return ret;
		}
		#if 0
		epd->ebs.current_stage = NPU_OK;
		time_tag(epd, epd->ebs.current_stage, "stage-npu-fw end polled");
		#endif
		break;
	default:
		break;
	}

	return ret;
}
#endif

void explorer_power_on(struct explorer_plat_data *epd)
{
	power_clock_control_explorer(epd, true);
}

int explorer_mipi_bypass(struct explorer_plat_data *epd)
{
  int ret = 0;
	time_tag(epd, "stage-mipi-bp start");
	if (epd->ebi.mmod == MIPI_BYPASS) {
		/* send 64bit info to cc */
		ret = explorer_send_mbox(epd, HAL_CMD_ASYNC, HAL_CMD_PBL, PBL_SUB_CMD_MB, epd->ebi.buf_mb);
		if (ret < 0) {
			AP_BOOT_ERR("===== send MIPI mbox failed.\n");
		}
	}

	time_tag(epd, "stage-mipi-bp trans done");
	return ret;
}

int explorer_sdio_tuning(struct explorer_plat_data *epd)
{
	int ret = 0;
	bool done = false;
	unsigned long deadline;
	unsigned char reg = 0, bits, spd = 0;

	/* check PLL enable bits */
	reg = 0;
	bits = SDU_BOOT_STAT_PLL_DONE;
	epd->est.sdio_tuning_status = -TUNING_CLOCK_ERR;

	ret = explorer_read_reg(epd, SDU_BOOT_STAT_REG8, &reg);
	if (ret) {
		return ret;
	}

	/* poll flag */
	deadline = epd->ebs.start_jiffies + msecs_to_jiffies(epd->ebi.boot_timeout);
	while(!time_after(jiffies, deadline)) {
		ret = explorer_read_reg(epd, SDU_BOOT_STAT_REG8, &reg);
		if (!ret && ((reg & bits) == bits)) {
			AP_BOOT_INFO("===== PLL enable finished.\n");
			done = true;
			break;
		} else if (ret < 0) {
			AP_BOOT_ERR("===== read PLL enable status failed.\n");
			return ret;
		}
		mdelay(1);
	}

	if(!done) {
		AP_BOOT_ERR("===== PLL enable timeout.\n");
		return -R_PLL_ENA_FAIL;
	}

	bits = SDU_BOOT_STAT_PLL_SPD;
	ret = explorer_read_reg(epd, SDU_BOOT_STAT_REG8, &reg);
	if (ret) {
		return ret;
	}

	spd = reg & bits;
	switch (spd) {
	case SDU_BOOT_STAT_PLL_HS:
		AP_BOOT_INFO("===== PLL enable to High speed.\n");
		break;
	case SDU_BOOT_STAT_PLL_MS:
		AP_BOOT_ERR("===== PLL enable to Middle speed.\n");
		return -R_PLL_ENA_FAIL;
	case SDU_BOOT_STAT_PLL_LS:
		AP_BOOT_ERR("===== PLL enable failed, Low Speed.\n");
		return -R_PLL_ENA_FAIL;
	default:
		AP_BOOT_ERR("===== PLL enable failed, Low Speed.\n");
		return -R_PLL_ENA_FAIL;
	}

	epd->est.sdio_tuning_status = -TUNING_SDIO_ERR;

	ret = explorer_execute_tuning(epd);
	if (ret) {
		AP_BOOT_ERR("===== TUNING FAIL.\n");
		return -R_PLL_ENA_FAIL;
	}

	epd->est.sdio_tuning_status = TUNING_OK;
	return 0;
}

int explorer_ddr_train(struct explorer_plat_data *epd)
{
	time_tag(epd, "stage-ddr-train start");
	if (epd->ebi.dmod == DDR_QUICK) {
		/* do quickboot */
	} else {
		/* do firstboot */
	}

	time_tag(epd, "stage-ddr-train trans done");
	return 0;
}

void init_timeout(struct explorer_plat_data *epd)
{
	struct explorer_boot_info *ebi = &(epd->ebi);
	switch (ebi->target_stage){
	case BOOTROM:
		ebi->boot_timeout = 10000;
		break;
	case PBL_PROV:
		ebi->boot_timeout = 20000;
		break;
#ifdef SLT_ENABLE
	case PBL_SLT:
		ebi->boot_timeout = 20000;
		break;
#endif
	case PBL_NORMAL:
		ebi->boot_timeout = 20000;
		break;
	case MIPI_BP_OK:
		ebi->boot_timeout = 40000;
		break;
	case DDR_OK:
		ebi->boot_timeout = 60000;
		break;
	case OS_OK:
		ebi->boot_timeout = 80000;
		break;
	case NPU_OK:
		ebi->boot_timeout = 100000;
		break;
	default:
		break;
	}
}

int explorer_boot(struct explorer_plat_data *epd)
{
	int ret = 0;
	struct explorer_boot_info *ebi;
	struct explorer_boot_status *ebs;
	unsigned long deadline;
	unsigned long flashable_deadline;
	bool done = false;
	struct exception_info info;
	info.moduleId = EXCEPTION_BOOT;

	if (!epd) {
		info.majorType = MAJOR_TYPE_ERROR;
		info.subType = ERROR_TYPE_ARGUMENT;
		explorer_genl_mcast_data(epd, HAL_CMD_EXCEPTION2AP, &info, sizeof(struct exception_info));

		AP_BOOT_ERR("epd bad, exit boot\n");
		return -R_BAD_ARG;
	}


	ebi = &(epd->ebi);
	/* TODO: ebi sanity check: stage, mmod, dmod, rmod, pmod*/
	if (ebi->target_stage == UNKNOWN || ebi->target_stage >= STG_MAX) {
		info.majorType = MAJOR_TYPE_ERROR;
		info.subType = ERROR_TYPE_ARGUMENT;
		explorer_genl_mcast_data(epd, HAL_CMD_EXCEPTION2AP, &info, sizeof(struct exception_info));

		AP_BOOT_ERR("target_stage bad, exit boot\n");
		return -R_BAD_TSTG;
	}

	if (ebi->target_stage == NPU_OK) {
		ebi->target_stage = OS_OK;
	}

	if (ebi->dmod >= DDR_MAX_MODE) {
		info.majorType = MAJOR_TYPE_ERROR;
		info.subType = ERROR_TYPE_ARGUMENT;
		explorer_genl_mcast_data(epd, HAL_CMD_EXCEPTION2AP, &info, sizeof(struct exception_info));

		AP_BOOT_ERR("dmod bad, exit boot\n");
		return -R_BAD_DMOD;
	}

	if (ebi->mmod >= MIPI_MAX_MODE) {
		info.majorType = MAJOR_TYPE_ERROR;
		info.subType = ERROR_TYPE_ARGUMENT;
		explorer_genl_mcast_data(epd, HAL_CMD_EXCEPTION2AP, &info, sizeof(struct exception_info));

		AP_BOOT_ERR("mmod bad, exit boot\n");
		return -R_BAD_MMOD;
	}

	if (ebi->rmod >= RTOS_MAX) {
		info.majorType = MAJOR_TYPE_ERROR;
		info.subType = ERROR_TYPE_ARGUMENT;
		explorer_genl_mcast_data(epd, HAL_CMD_EXCEPTION2AP, &info, sizeof(struct exception_info));

		AP_BOOT_ERR("rmod bad, exit boot\n");
		return -R_BAD_RMOD;
	}

	if (ebi->rpos >= MAX_RTOS_ADDR_TYPE) {
		info.majorType = MAJOR_TYPE_ERROR;
		info.subType = ERROR_TYPE_ARGUMENT;
		explorer_genl_mcast_data(epd, HAL_CMD_EXCEPTION2AP, &info, sizeof(struct exception_info));

		AP_BOOT_ERR("rpos bad, exit boot\n");
		return -R_BAD_RPOS;
	}

	if (ebi->pmod >= MAX_PLAT) {
		info.majorType = MAJOR_TYPE_ERROR;
		info.subType = ERROR_TYPE_ARGUMENT;
		explorer_genl_mcast_data(epd, HAL_CMD_EXCEPTION2AP, &info, sizeof(struct exception_info));

		AP_BOOT_ERR("pmod bad, exit boot\n");
		return -R_BAD_PMOD;
	}

	if (ebi->boot_timeout <=0) {
		init_timeout(epd);
	}

	if (ebi->boot_timeout > BOOT_TIMEOUT_MAX)
		ebi->boot_timeout = BOOT_TIMEOUT_MAX;

	ebs = &(epd->ebs);
	ebs->start_jiffies = jiffies;
	deadline = ebs->start_jiffies + msecs_to_jiffies(ebi->boot_timeout);
	flashable_deadline = ebs->start_jiffies + msecs_to_jiffies(1000);
	AP_BOOT_INFO("%s: boot_timeout = %dms, start: 0x%lx, deadline: 0x%lx\n",
			__func__, ebi->boot_timeout, ebs->start_jiffies, deadline);

	time_tag(epd, "boot start");

	info.majorType = MAJOR_TYPE_STAGE;
	info.subType = STAGE_TYPE_START;
	explorer_genl_mcast_data(epd, HAL_CMD_EXCEPTION2AP, &info, sizeof(struct exception_info));

	//power on explorer if not
	explorer_power_on(epd);

	/* wa: for PMIC hw, init the pmic oc flag to 0 */
	atomic_set(&epd->is_pmic_oc, 0);
	/* set ddr flag to 0 */
	atomic_set(&epd->is_ddr_failed, 0);
	/* when boot again, cancel heartbeat delayed work */
	epd->heartbeat_started = false;
	cancel_heartbeat_detect_work(epd);

	/* flashable will be set after ps_hold up/int_rst_done & sdio re-attached.*/
	while (!atomic_read(&ebs->flashable)) {
		if (!time_after(jiffies, flashable_deadline)) {
			msleep(1);
		} else {
			AP_BOOT_ERR("%s: Repeated Boot! No power_off before boot, exit.\n", __func__);
			report_power_exception(epd,
					EXCEPTION_POWER_MAJOR_TYPE_PMIC_HW_ERR,
					EXCEPTION_POWER_SUB_TYPE_PMIC_HW_ERR_BY_BOOT_TIMEOUT,
					EXCEPTION_ACT_NONE);
			return -R_REPEATED_BOOT;
		}
	}
	AP_BOOT_INFO("===== SDIO device detected & stage-pre done.\n");

	atomic_set(&(epd->ebs.rtos_on), 0);

	if (ebi->target_stage == BOOTROM) {
		goto final_wait;
	}

	//Initialize
	//Set MIPI BYPASS or not, DDR quick or first boot flag
	ret = explorer_sdu_reg_config(epd);
	if (ret < 0) {
		AP_BOOT_ERR("explorer_sdu_reg_config failed, exit boot\n");
		goto exit;
	}

	ret = explorer_bootstage_pbl(epd, deadline);
	if (ret < 0) {
		AP_BOOT_ERR("explorer_bootstage_pbl failed, exit boot\n");
		goto exit;
	}

#ifdef SLT_ENABLE
	if (ebi->target_stage == PBL_SLT) {
		ret = explorer_wait_bootstage(epd, PBL_SLT, deadline);
	} else {
#endif
		if (ebi->target_stage == PBL_PROV) {
			ret = explorer_wait_bootstage(epd, PBL_PROV, deadline);
		} else {
			ret = explorer_wait_bootstage(epd, PBL_NORMAL, deadline);
		}
#ifdef SLT_ENABLE
	}
#endif
	if (ret < 0) {
		AP_BOOT_ERR("explorer_wait_bootstage PBL failed, exit boot\n");
		goto exit;
	}

	ret = explorer_mipi_bypass(epd);
	if (ret < 0) {
		AP_BOOT_ERR("explorer_mipi_bypass failed, exit boot\n");
		goto exit;
	}

#ifndef AUTO_TRIG
	if (ebi->target_stage == PBL_NORMAL ||
	    ebi->target_stage == PBL_PROV ) {
		goto final_wait;
	}
#ifdef SLT_ENABLE
	if (ebi->target_stage == PBL_SLT) {
		goto final_wait;
	}
#endif

	ret = explorer_wait_bootstage(epd, MIPI_BP_OK, deadline);
	if (ret < 0) {
		AP_BOOT_ERR("explorer_wait_bootstage MIPI_BP_OK failed, exit boot\n");
		goto exit;
	}

	if (ebi->target_stage == MIPI_BP_OK) {
		goto final_wait;
	}

	ret = explorer_ddr_train(epd);
	if (ret < 0) {
		AP_BOOT_ERR("explorer_ddr_train failed, exit boot\n");
		goto exit;
	}

	ret = explorer_wait_bootstage(epd, DDR_OK, deadline);
	if (ret < 0) {
		AP_BOOT_ERR("explorer_wait_bootstage DDR_OK failed, exit boot\n");
		goto exit;
	}

	info.majorType = MAJOR_TYPE_STAGE;
	info.subType = STAGE_TYPE_DDR_OK;
	explorer_genl_mcast_data(epd, HAL_CMD_EXCEPTION2AP, &info, sizeof(struct exception_info));

	if (ebi->target_stage == DDR_OK) {
		goto final_wait;
	}

	ret = explorer_bootstage_os(epd);
	if (ret < 0) {
		AP_BOOT_ERR("explorer_bootstage_os failed, exit boot\n");
		goto exit;
	}

	ret = explorer_wait_bootstage(epd, OS_OK, deadline);
	if (ret < 0) {
		AP_BOOT_ERR("explorer_wait_bootstage OS_OK failed, exit boot\n");
		goto exit;
	}

	/*
	if (ebi->target_stage == OS_OK) {
		goto final_wait;
	}

	ret = explorer_bootstage_npu(epd);
	if (ret < 0) {
		return ret;
	}

	ret = explorer_wait_bootstage(epd, NPU_OK, deadline);
	if (ret < 0) {
		return ret;
	}

	if (ebi->target_stage == NPU_OK) { //if NPU load, don't wait response
		atomic_set(&epd->ebs.current_stage, NPU_OK);
		time_tag(epd, "stage-npu-fw end polled");
	}
	 */

#endif

final_wait:
	do {
		if (atomic_read(&ebs->current_stage) == ebi->target_stage) {
			done = true;
			AP_BOOT_INFO("%s: done = %d.\n", __func__, done);
		}
#ifdef AUTO_TRIG
		if (atomic_read(&epd->is_pmic_oc) == 1) {
			AP_BOOT_ERR("===== pmic hw error, boot failed.\n");
			return -R_PMIC_OC_FAIL;
		}
#endif
	} while (!done && !time_after(jiffies, deadline));

	if (!done) {
		/* time out, not success */
		info.majorType = MAJOR_TYPE_ERROR;
		info.subType = ERROR_TYPE_TIMEOUT;
		explorer_genl_mcast_data(epd, HAL_CMD_EXCEPTION2AP, &info, sizeof(struct exception_info));

		AP_BOOT_ERR("%s: explorer boot timeout, deadline:%lx, jiffies:%lx\n", __func__, deadline, jiffies);
		ret = -ETIMEDOUT;
	} else {
		AP_BOOT_INFO("===== explorer boot done, current stage: %d, target stage: %d=====\n",
			     atomic_read(&ebs->current_stage),
			     ebi->target_stage);
	}
exit:
	atomic_set(&ebs->flashable, 0);
	return ret;
}

int explorer_boot_from_pbl(struct explorer_plat_data *epd)
{
	int ret = 0;
	struct explorer_boot_info *ebi;
	struct explorer_boot_status *ebs;
	unsigned long deadline;
	struct mipi_params_t go_os;
	bool done = false;

	if (!epd) {
		AP_BOOT_ERR("epd bad, exit boot\n");
		return -R_BAD_ARG;
	}


	ebi = &(epd->ebi);
	/*  ebi sanity check: stage, mmod, dmod, rmod, pmod*/
	if (ebi->target_stage < OS_OK || ebi->target_stage >= STG_MAX) {
		AP_BOOT_ERR("target_stage bad, exit boot\n");
		return -R_BAD_TSTG;
	}

	if (ebi->target_stage == NPU_OK) {
		ebi->target_stage = OS_OK;
	}

	if (ebi->dmod >= DDR_MAX_MODE) {
		AP_BOOT_ERR("dmod bad, exit boot\n");
		return -R_BAD_DMOD;
	}

	if (ebi->mmod >= MIPI_MAX_MODE) {
		AP_BOOT_ERR("mmod bad, exit boot\n");
		return -R_BAD_MMOD;
	}

	if (ebi->rmod >= RTOS_MAX) {
		AP_BOOT_ERR("rmod bad, exit boot\n");
		return -R_BAD_RMOD;
	}

	if (ebi->rpos >= MAX_RTOS_ADDR_TYPE) {
		AP_BOOT_ERR("rpos bad, exit boot\n");
		return -R_BAD_RPOS;
	}

	if (ebi->pmod >= MAX_PLAT) {
		AP_BOOT_ERR("pmod bad, exit boot\n");
		return -R_BAD_PMOD;
	}

	if (ebi->boot_timeout <=0) {
		init_timeout(epd);
	}

	if (ebi->boot_timeout > BOOT_TIMEOUT_MAX)
		ebi->boot_timeout = BOOT_TIMEOUT_MAX;

	ebs = &(epd->ebs);
	ebs->start_jiffies = jiffies;
	deadline = ebs->start_jiffies + msecs_to_jiffies(ebi->boot_timeout);
	AP_BOOT_INFO("%s: boot_timeout = %dms, start: 0x%lx, deadline: 0x%lx\n",
			__func__, ebi->boot_timeout, ebs->start_jiffies, deadline);

	time_tag(epd, "boot from pbl, start");

	atomic_set(&(epd->ebs.rtos_on), 0);

	/* add cmd to tell CC go forward */
	go_os.op_cmd = OP_GOTO_RTOS;
	ret = explorer_send_mbox(epd, HAL_CMD_ASYNC, HAL_CMD_PBL, PBL_SUB_CMD_MB, &go_os);
	if (ret < 0) {
		AP_BOOT_ERR("explorer send mbox to make PBL go forward failed, exit boot\n");
		goto exit;
	}

#ifndef AUTO_TRIG

	ret = explorer_wait_bootstage(epd, MIPI_BP_OK, deadline);
	if (ret < 0) {
		AP_BOOT_ERR("explorer_wait_bootstage MIPI_BP_OK failed, exit boot\n");
		goto exit;
	}

	ret = explorer_wait_bootstage(epd, DDR_OK, deadline);
	if (ret < 0) {
		AP_BOOT_ERR("explorer_wait_bootstage DDR_OK failed, exit boot\n");
		goto exit;
	}

	ret = explorer_bootstage_os(epd);
	if (ret < 0) {
		AP_BOOT_ERR("explorer_bootstage_os failed, exit boot\n");
		goto exit;
	}

	ret = explorer_wait_bootstage(epd, OS_OK, deadline);
	if (ret < 0) {
		AP_BOOT_ERR("explorer_wait_bootstage OS_OK failed, exit boot\n");
		goto exit;
	}

#endif
	do {
		if (atomic_read(&ebs->current_stage) == ebi->target_stage) {
			done = true;
			AP_BOOT_INFO("%s: done = %d.\n", __func__, done);
		}
#ifdef AUTO_TRIG
		if (atomic_read(&epd->is_pmic_oc) == 1) {
			AP_BOOT_ERR("===== pmic hw error, boot failed.\n");
			return -R_PMIC_OC_FAIL;
		}
#endif
	} while (!done && !time_after(jiffies, deadline));

	if (!done) {
		/* time out, not success */
		AP_BOOT_ERR("%s: explorer boot timeout, deadline:%lx, jiffies:%lx\n", __func__, deadline, jiffies);
		ret = -ETIMEDOUT;
	} else {
		AP_BOOT_INFO("===== explorer boot done, current stage: %d, target stage: %d=====\n",
			     atomic_read(&ebs->current_stage),
			     ebi->target_stage);
	}
exit:
	return ret;
}

extern struct wait_queue_head comm_sync_wq;

int explorer_proc_bootrom_msg(struct explorer_plat_data *epd, struct hal_comm_data *comm_data)
{
	int ret = 0;
	unsigned long deadline;

	switch (comm_data->data[0]) {
	case ID_SE_RLD_SET:
		explorer_clr_glb_reg_bits(epd, GLB_AP_CTRL_BOOT_STEP,
				BOOT_TRANS_BIT_SE_HEAD_FW | BOOT_TRANS_BIT_SE_FW);
		break;
	case ID_SE_RLD_CLR:
		deadline = jiffies + msecs_to_jiffies(epd->ebi.boot_timeout);
		explorer_load_se(epd, deadline);
		break;
	case ID_SDI_RLD_SET:
		explorer_clr_glb_reg_bits(epd, GLB_AP_CTRL_BOOT_STEP,
				BOOT_TRANS_BIT_SDI_FW);
		break;
	case ID_SDI_RLD_CLR:
		explorer_load_sdi(epd);
		break;
	case ID_PBL_RLD_SET:
		explorer_clr_glb_reg_bits(epd, GLB_AP_CTRL_BOOT_STEP,
				BOOT_TRANS_BIT_PBL_FW);
		break;
	case ID_PBL_RLD_CLR:
		ret = explorer_load_pbl(epd);
		break;
	case ID_INFORM_CBOOT:
#ifdef AUTO_TRIG
		atomic_set(&epd->ebs.current_stage, BOOTROM);
		epd->ebs.cur_os_type = UNKNOWN_OS;
#endif
		/* TODO: cold boot of CC */

		break;
	case ID_INFORM_HBOOT:
#ifdef AUTO_TRIG
		atomic_set(&epd->ebs.current_stage, BOOTROM);
		epd->ebs.cur_os_type = UNKNOWN_OS;
#endif
		/* TODO: hot boot of CC */
		break;
	case ID_INFORM_AP_RST:
		/* TODO: reset CC */
		break;
	default:
		break;
	}
	pr_info("%s, done.\n", __func__);

	return ret;
}

static int explorer_proc_mipi_bypass(struct explorer_plat_data *epd, struct hal_comm_data *comm_data)
{
	int ret = 0;

	/* this is a reply cmd */
	if (comm_data->cmd_is_reply == HAL_CMD_REPLY) {
		/* sync cmd, wake up wait queue. */
		if (comm_data->cmd_is_sync == HAL_CMD_SYNC) {
			atomic_set(&epd->mbox_rindex[comm_data->cmd_mod_id], comm_data->cmd_sync_idx);
			pr_info("%s, done.\n", __func__);
			/* wake up wait queue */
			wake_up(&comm_sync_wq);
			return ret;
		} else {
			/* for mipi bypass cmd, should not go here */
			pr_err("%s, no async mipi bypass cmd.\n", __func__);
		}
	} else {
		pr_err("%s, no send mipi bypass cmd.\n", __func__);
	}

	return ret;
}

int explorer_proc_pbl_msg(struct explorer_plat_data *epd, struct hal_comm_data *comm_data)
{
	int ret = 0;
	struct exception_info info;
	u32 sub_cmd = comm_data->cmd_sub_id;

	info.moduleId = EXCEPTION_BOOT;

	if (sub_cmd > PBL_SUB_CMD_MAX) {
		pr_err("%s, invalid sub command.\n", __func__);
		return -EINVAL;
	}

	switch (sub_cmd) {
	case PBL_SUB_CMD_DFT:
		switch (comm_data->data[0]) {
#ifdef PBL_2nd
		case ID_2ND_RLD_SET:
			explorer_clr_glb_reg_bits(epd, GLB_AP_CTRL_BOOT_STEP,
					BOOT_TRANS_BIT_2ND_PBL);
			break;
		case ID_2ND_RLD_CLR:
			ret = explorer_load_stl_2nd_pbl(epd);
			if (ret < 0) {
				AP_BOOT_ERR("load 2nd PBL failed.");
				return ret;
			}

			ret = explorer_set_glb_reg_bits(epd, GLB_AP_CTRL_BOOT_STEP,
					BOOT_TRANS_BIT_2ND_PBL);
			if (ret < 0) {
				AP_BOOT_ERR("set 2nd_pbl_trans_done failed.\n");
				return ret;
			}
			break;
#endif
		case ID_DDRFW_RLD_SET:
			explorer_clr_glb_reg_bits(epd, GLB_AP_CTRL_BOOT_STEP,
					BOOT_TRANS_BIT_DDR_FW);
			break;
		case ID_DDRFW_RLD_CLR:
			//explorer_load_ddr(epd);
			break;
		case ID_OS_RLD_SET:
			explorer_clr_glb_reg_bits(epd, GLB_AP_CTRL_BOOT_STEP,
					BOOT_TRANS_BIT_OS_IMG);
			break;
		case ID_OS_RLD_CLR:
			explorer_load_os(epd);
			break;
		case ID_NPU_RLD_SET:
			explorer_clr_glb_reg_bits(epd, GLB_AP_CTRL_BOOT_STEP,
					BOOT_TRANS_BIT_NPU_IMG);
			break;
		case ID_NPU_RLD_CLR:
			explorer_load_npu(epd);
			break;
		case ID_ISP_RLD_SET:
			explorer_clr_glb_reg_bits(epd, GLB_AP_CTRL_BOOT_STEP,
					BOOT_TRANS_BIT_ISP_IMG);
			break;
		case ID_ISP_RLD_CLR:
			/* re-load fw & set transdone flag */
			break;
#ifdef SLT_ENABLE
		case ID_SLT_BEGIN:
			atomic_set(&epd->ebs.current_stage, PBL_SLT);
			time_tag(epd, "stage-pbl-slt end triggered");
			break;
#endif
		case ID_PROV_DONE:
			atomic_set(&epd->ebs.current_stage, PBL_PROV);
			time_tag(epd, "stage-pbl-prov end triggered");
			break;
		case ID_PBL_BEGIN:
#ifdef AUTO_TRIG
			/* received when pbl begins */
			atomic_set(&epd->ebs.current_stage, PBL_NORMAL);
			time_tag(epd, "stage-pbl end triggered");
			if (epd->ebi.target_stage >= MIPI_BP_OK){
				explorer_mipi_bypass(epd);
			}
#endif
			break;
		case ID_PMIC_HW_ERR:
			time_tag(epd, "pmic hw error");
			PM_LOG_E("pmic_hw_err, data=0x%x\n", comm_data->data[1]);
			atomic_set(&epd->is_pmic_oc, 1);
			break;
		case ID_MIPI_OK:
#ifdef AUTO_TRIG
			/* received when mipi_bp ok */
			atomic_set(&epd->ebs.current_stage, MIPI_BP_OK);
			time_tag(epd, "stage-mipi-bp end triggered");
			if (epd->ebi.target_stage >= DDR_OK){
				explorer_ddr_train(epd);
			}
#endif
			break;
		case ID_DDR_OK:
#ifdef AUTO_TRIG
			/* received when ddr_train ok */
			atomic_set(&epd->ebs.current_stage, DDR_OK);
			time_tag(epd, "stage-ddr-train end triggered");
			explorer_clear_ddr_boot_flag(epd);
			if (epd->ebi.target_stage >= OS_OK){
				explorer_bootstage_os(epd);
				if (epd->ebi.target_stage == NPU_OK) {
					explorer_bootstage_npu(epd);
				}
			}
#endif
			break;
		case ID_OS_OK:
			atomic_set(&epd->ebs.rtos_on, 1);
			info.majorType = MAJOR_TYPE_STAGE;
			info.subType = STAGE_TYPE_RTOS_RUN;
			explorer_genl_mcast_data(epd, HAL_CMD_EXCEPTION2AP, &info, sizeof(struct exception_info));
#ifdef AUTO_TRIG
			if (epd->ebi.rmod == RTOS_AON) {
				epd->ebs.cur_os_type = AON_OS;
			} else if (epd->ebi.rmod == RTOS_PLAT) {
				epd->ebs.cur_os_type = PLAT_OS;
#ifdef SLT_ENABLE
			} else if (epd->ebi.rmod == RTOS_SLT) {
				epd->ebs.cur_os_type = SLT_OS;
			} else if (epd->ebi.rmod == RTOS_SLT_AON) {
				epd->ebs.cur_os_type = SLT_OS_AON;
#endif /*end of SLT_ENABLE */
			} else if (epd->ebi.rmod == RTOS_NORMAL || epd->ebi.rmod == RTOS_RLS ||
				   epd->ebi.rmod == RTOS_DBG_NOPM || epd->ebi.rmod == RTOS_RLS_NOPM) {
				epd->ebs.cur_os_type = NORMAL_OS;
			}
			atomic_set(&epd->ebs.current_stage, OS_OK);
			time_tag(epd, "stage-os-fw end triggered");

			if (epd->ebi.target_stage >= NPU_OK){ /*if NPU load, don't wait response*/
				atomic_set(&epd->ebs.current_stage, NPU_OK);
				time_tag(epd, "stage-npu-fw end triggered");
			}
#else /* else of AUTO_TRIG */
#ifdef SLT_ENABLE
			if (epd->ebi.rmod == RTOS_SLT) {
				epd->ebs.cur_os_type = SLT_OS;
			} else if (epd->ebi.rmod == RTOS_SLT_AON) {
				epd->ebs.cur_os_type = SLT_OS_AON;
			}

			if (epd->ebi.rmod == RTOS_SLT || epd->ebi.rmod == RTOS_SLT_AON) {
				atomic_set(&epd->ebs.current_stage, OS_OK);
				time_tag(epd, "stage-os-fw end triggered");
			}
#endif /*end of SLT_ENABLE */
#endif /*end of AUTO_TRIG */
			break;
		case ID_NPU_OK:
			/* received when npu verify ok */
			//atomic_set(&epd->ebs.current_stage, NPU_OK);
			//time_tag(epd, "stage-npu-fw end triggered");
			break;
		case ID_INFORM_AP_RST:
			/* TODO: reset CC */
			break;
#ifdef SLT_ENABLE
		case ID_SLT_RLD_CE:
			ret = explorer_load_fw(epd, SE_DATA_NAME, FW_SE_DATA_ADDR);
			if (ret < 0) {
				ret = explorer_load_stl_se_fw(epd);
			}
			if (ret < 0) {
				AP_BOOT_ERR("load se fw failed in slt\n");
				return ret;
			}
			explorer_set_glb_reg_bits(epd, GLB_AP_CTRL_BOOT_STEP, BOOT_TRANS_BIT_SE_FW | BOOT_STEP_SE_DONE);
			break;
#endif
		case ID_DDR_FAIL:
			atomic_set(&epd->is_ddr_failed, 1);
			info.moduleId = EXCEPTION_DDR;
			info.majorType = EXCEPTION_DDR_INIT_FAIL;
			info.subType = comm_data->data[1];
			info.level = EXCEPTION_ERROR;
			info.action = EXCEPTION_ACT_POWROFF;
			ret = exception_handle_func(epd, &info);

			break;
		default:
			break;
		}
		break;
	case PBL_SUB_CMD_TD:
		/* received to store ddr_training data */
		ret = explorer_store_ddr_training_data(epd, comm_data);
		if (ret < 0){
			pr_err("%s, explorer_store_ddr_training_data failed.\n", __func__);
		}
		explorer_ddr_continue_boot(epd);
		break;
#ifdef SLT_ENABLE
	case PBL_SUB_CMD_SLT:
		ret = explorer_proc_pbl_slt_msg(epd, comm_data);
		if (ret<0) {
			pr_err("%s, explorer_proc_pbl_slt_msg failed.\n", __func__);
			return ret;
		}
		break;
#endif
	case PBL_SUB_CMD_MB:
		ret = explorer_proc_mipi_bypass(epd, comm_data);
		if (ret<0) {
			pr_err("%s, explorer_proc_pbl_slt_msg failed.\n", __func__);
			return ret;
		}
		break;
	default:
		break;
	}

	pr_info("%s, done.\n", __func__);

	return ret;
}
