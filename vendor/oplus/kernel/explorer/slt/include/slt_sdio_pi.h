/*
 * Copyright (c) 2021 ZEKU Technology(Shanghai) Corp.,Ltd, all rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Basecode Created :        2021/05/01 Author: wangqinyuan@zeku.com
 *
 */

#ifndef _SLT_SDIO_PI_H
#define _SLT_SDIO_PI_H

#define	SLT_SDIO_TOKEN_NUM_MAX		(5)
#define	SLT_SDIO_TOKEN_LEN_MAX		(32)

#define CORE_FREQ_100MHZ	(100 * 1000 * 1000)

/* for sdio test case */
int zk_sdio_test_sdio_func_en(struct explorer_sdio_data *sdio_data);
int zk_sdio_test_fn0_regs_def_value_check(struct explorer_sdio_data *sdio_data);
int zk_sdio_test_fn1_regs_def_value_check(struct explorer_sdio_data *sdio_data);
int zk_sdio_test_transfer_16kb_test(struct explorer_sdio_data *sdio_data);

int explorer_sdio_slt_test_case(struct explorer_sdio_data *sdio_data, int clk, unsigned char tuning, unsigned int times);

#endif
