/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <sound/soc.h>

SND_SOC_DAILINK_DEFS(usb_audio_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(usb_audio_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(pcm_dummy_tx0,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(slimbus_7_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("btfmslim_slave",
			"btfm_bt_sco_a2dp_slim_rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(slimbus_7_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("btfmslim_slave",
			"btfm_bt_sco_slim_tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(slimbus_8_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("btfmslim_slave",
			"btfm_bt_sco_slim_tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(display_port,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC(
			"soc:qcom,msm-ext-disp:qcom,msm-ext-disp-audio-codec-rx",
			"msm_dp_audio_codec_rx_dai")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(display_port1,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC(
			"soc:qcom,msm-ext-disp:qcom,msm-ext-disp-audio-codec-rx",
			"msm_dp_audio_codec_rx1_dai")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(auxpcm_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(auxpcm_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(sec_auxpcm_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(sec_auxpcm_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(tert_auxpcm_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(tert_auxpcm_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(quat_auxpcm_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(quat_auxpcm_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(rx_dma_rx0,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("bolero-codec", "rx_macro_rx1"),
			   COMP_CODEC("wcd937x_codec", "wcd937x_cdc"),
			   COMP_CODEC("wsa-codec0", "wsa_rx0")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(rx_dma_rx1,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("bolero-codec", "rx_macro_rx2"),
			   COMP_CODEC("wcd937x_codec", "wcd937x_cdc"),
			   COMP_CODEC("wsa-codec0", "wsa_rx0")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(rx_dma_rx2,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("bolero-codec", "rx_macro_rx3"),
			   COMP_CODEC("wcd937x_codec", "wcd937x_cdc")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(rx_dma_rx3,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("bolero-codec", "rx_macro_rx4"),
			   COMP_CODEC("wcd937x_codec", "wcd937x_cdc")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(rx_dma_rx5,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("bolero-codec", "rx_macro_rx5"),
			   COMP_CODEC("wcd937x_codec", "wcd937x_cdc")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(rx_dma_rx6,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("bolero-codec", "rx_macro_rx6"),
			   COMP_CODEC("wcd937x_codec", "wcd937x_cdc")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(tx_dma_tx3,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("bolero-codec", "tx_macro_tx1"),
			   COMP_CODEC("wcd937x_codec", "wcd937x_cdc")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(tx_dma_tx4,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("bolero-codec", "tx_macro_tx2"),
			   COMP_CODEC("wcd937x_codec", "wcd937x_cdc")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(va_dma_tx0,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("bolero-codec", "va_macro_tx1")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(va_dma_tx1,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("bolero-codec", "va_macro_tx2")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(va_dma_tx2,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("bolero-codec", "va_macro_tx3")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(proxy_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(proxy_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(pri_mi2s_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(pri_mi2s_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(sec_mi2s_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(sec_mi2s_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(tert_mi2s_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(tert_mi2s_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(quat_mi2s_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(quat_mi2s_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(quin_mi2s_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(quin_mi2s_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(sen_mi2s_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(sen_mi2s_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(sep_mi2s_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(sep_mi2s_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(pri_tdm_rx_0,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(pri_tdm_tx_0,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(sec_tdm_rx_0,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(sec_tdm_tx_0,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(tert_tdm_rx_0,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(tert_tdm_tx_0,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(quat_tdm_rx_0,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(quat_tdm_tx_0,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(quin_tdm_rx_0,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(quin_tdm_tx_0,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(sen_tdm_rx_0,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(sen_tdm_tx_0,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(sep_tdm_rx_0,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(sep_tdm_tx_0,
	DAILINK_COMP_ARRAY(COMP_CPU("snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));
