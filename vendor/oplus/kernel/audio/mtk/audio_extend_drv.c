/************************************************************************************
** File: -
** Copyright (C), 2020-2025, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**     add audio extend driver
** Version: 1.0
** --------------------------- Revision History: --------------------------------
**               <author>                                <date>          <desc>
**
************************************************************************************/

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#if 1
#define AUDIO_EXTEND_DRIVER_NAME "audio-extend-drv"
#define OPLUS_SPK_NAME "Speaker Codec"
#define OPLUS_SPK_REF_NAME "Speaker Codec Ref"

enum {
	CODEC_PA_NXP = 0,
	CODEC_PA_AWINIC,
	CODEC_PA_END,
	CODEC_PA_MAX = CODEC_PA_END,
};

enum {
	CODEC_I2S_ID = 0,
	CODEC_I2S_ID_IN,
	CODEC_NAME,
	CODEC_DAI_NAME,
	CODEC_VENDOR,
	CODEC_PROP_END,
	CODEC_PROP_MAX = CODEC_PROP_END,
};

#define I2S_PROP_MAX 10

static const char *extend_pa_vendor[CODEC_PA_MAX] = {
	[CODEC_PA_NXP] = "nxp",
	[CODEC_PA_AWINIC] = "awinic",
};

static const char *extend_speaker_prop[CODEC_PROP_MAX] = {
	[CODEC_I2S_ID] = "oplus,speaker-i2s-id",
	[CODEC_I2S_ID_IN] = "oplus,speaker-i2s-id-in",
	[CODEC_NAME] = "oplus,speaker-codec-name",
	[CODEC_DAI_NAME] = "oplus,speaker-codec-dai-name",
	[CODEC_VENDOR] = "oplus,speaker-vendor",
};

static const char *extend_dac_prop[CODEC_PROP_MAX] = {
	[CODEC_I2S_ID] = "oplus,dac-i2s-id",
	[CODEC_NAME] = "oplus,dac-codec-name",
	[CODEC_DAI_NAME] = "oplus,dac-codec-dai-name",
	[CODEC_VENDOR] = "oplus,dac-vendor",
};

static const char *extend_i2s_prop[I2S_PROP_MAX] = {
	"I2S0", "I2S1", "I2S2", "I2S3", "I2S4", "I2S5", "ETDMIN", "ETDMOUT", "I2SOUT4", "I2SIN4"
};

struct codec_prop_info {
	int dev_cnt;
	u32 i2s_id;
	u32 i2s_id_in;
	bool b_ivdata_support;
	const char **codec_name;
	const char **codec_dai_name;
	const char *codec_vendor;
	int spk_i2s_in_index;
	int spk_i2s_out_index;
	bool spk_index_support;
};

struct audio_extend_data {
	struct codec_prop_info *spk_pa_info;
	struct codec_prop_info *hp_dac_info;
	bool use_extern_dailink_spk;
	bool use_extern_dailink_dac;
};

static struct audio_extend_data *g_extend_pdata = NULL;

/*For tfa98xx default stereo*/
static struct snd_soc_dai_link_component tfa98xx_dails[] = {
	{
		.of_node = NULL,
		.name = "tfa98xx.6-0035",
		.dai_name = "tfa98xx-aif-6-35",
	},

	{
		.of_node = NULL,
		.name = "tfa98xx.6-0034",
		.dai_name = "tfa98xx-aif-6-34",
	},
};
/*For tfa98xx default 3rd*/
static struct snd_soc_dai_link_component tfa98xx_dails_3rd[] = {
	{
		.of_node = NULL,
		.name = "tfa98xx.6-0036",
		.dai_name = "tfa98xx-aif-6-36",
	},
	{
		.of_node = NULL,
		.name = "tfa98xx.6-0035",
		.dai_name = "tfa98xx-aif-6-35",
	},

	{
		.of_node = NULL,
		.name = "tfa98xx.6-0034",
		.dai_name = "tfa98xx-aif-6-34",
	},
};

static int extend_codec_prop_parse(struct device *dev, const char *codec_prop[], struct codec_prop_info *codec_info)
{
	int ret = 0;

	u32 i2s_set[2];
	const int i2s_num = 2;
	ret = of_property_read_u32_array(dev->of_node, "oplus,spk-i2s-index", i2s_set, i2s_num);
	if (ret) {
		codec_info->spk_index_support = false;
		pr_warn("%s: spk i2s idx not set\n", __func__);
	} else {
		codec_info->spk_i2s_out_index = i2s_set[0];
		codec_info->spk_i2s_in_index = i2s_set[1];
		codec_info->spk_index_support = true;
		pr_info("%s: spk i2s index(%d,%d)\n", __func__, codec_info->spk_i2s_out_index, codec_info->spk_i2s_in_index);
	}

	ret = of_property_read_string(dev->of_node, codec_prop[CODEC_VENDOR], &codec_info->codec_vendor);
	if (ret) {
		pr_warn("%s: Looking up '%s' property in node %s failed\n",
			__func__, codec_prop[CODEC_VENDOR], dev->of_node->full_name);
		return -EINVAL;
	} else {
		pr_info("%s: codec vendor: %s\n", __func__, codec_info->codec_vendor);
	}

	ret = of_property_read_u32(dev->of_node, codec_prop[CODEC_I2S_ID], &codec_info->i2s_id);
	if (ret) {
		pr_warn("%s: Looking up '%s' property in node %s failed\n",
			__func__, codec_prop[CODEC_I2S_ID], dev->of_node->full_name);
		codec_info->i2s_id = I2S_PROP_MAX;
		return -EINVAL;
	} else {
		pr_info("%s: i2s id: %d\n", __func__, codec_info->i2s_id);
	}

	ret = of_property_read_u32(dev->of_node, codec_prop[CODEC_I2S_ID_IN], &codec_info->i2s_id_in);
	if (ret) {
		pr_warn("%s: Looking up '%s' property in node %s failed\n",
			__func__, codec_prop[CODEC_I2S_ID_IN], dev->of_node->full_name);
		codec_info->i2s_id_in = 0;
		codec_info->b_ivdata_support = false;
	} else {
		if (codec_info->i2s_id_in < I2S_PROP_MAX) {
			codec_info->b_ivdata_support = true;
		}
		pr_info("%s: i2s in id: %d\n", __func__, codec_info->i2s_id_in);
	}

	ret = of_property_count_strings(dev->of_node, codec_prop[CODEC_NAME]);
	if (ret <= 0) {
		pr_warn("%s: Invalid number of codecs, ret=%d\n",
			__func__, ret);
		return -EINVAL;
	} else {
		codec_info->dev_cnt = ret;
		pr_info("%s: dev_cnt %d\n", __func__, codec_info->dev_cnt);
	}

	codec_info->codec_name = devm_kzalloc(dev, codec_info->dev_cnt * sizeof(char *), GFP_KERNEL);
	if (!codec_info->codec_name) {
		pr_warn("%s: kzalloc fail for codec_name!\n", __func__);
		return -ENOMEM;
	}
	ret = of_property_read_string_array(dev->of_node, codec_prop[CODEC_NAME], codec_info->codec_name, codec_info->dev_cnt);
	if (ret < 0) {
		pr_warn("%s: Looking up '%s' property in node %s failed\n",
			__func__, codec_prop[CODEC_NAME], dev->of_node->full_name);
		return -EINVAL;
	}

	codec_info->codec_dai_name = devm_kzalloc(dev, codec_info->dev_cnt * sizeof(char *), GFP_KERNEL);
	if (!codec_info->codec_dai_name) {
		pr_warn("%s: kzalloc fail for codec_dai_name!\n", __func__);
		return -ENOMEM;
	}

	ret = of_property_read_string_array(dev->of_node, codec_prop[CODEC_DAI_NAME], codec_info->codec_dai_name, codec_info->dev_cnt);
	if (ret < 0) {
		pr_warn("%s: Looking up '%s' property in node %s failed\n",
			__func__, codec_prop[CODEC_DAI_NAME], dev->of_node->full_name);
		return -EINVAL;
	}

	return 0;
}

static void extend_codec_be_dailink(struct codec_prop_info *codec_info, struct snd_soc_dai_link *dailink, size_t size)
{
	int i2s_id = 0;
	int i2s_id_in = 0;
	bool b_ivdata_support = false;
	int i = 0;
	int j = 0;

	if (!codec_info) {
		pr_err("%s: codec_info param invalid!\n", __func__);
		return;
	}

	if (!dailink) {
		pr_err("%s: dailink param invalid!\n", __func__);
		return;
	}

	i2s_id = codec_info->i2s_id;
	pr_info("%s: i2s_id=%d, size=%zu!\n", __func__, i2s_id, size);
	if (i2s_id >= I2S_PROP_MAX) {
		pr_err("%s: i2s_id param invalid!\n", __func__);
		return;
	}

	b_ivdata_support = codec_info->b_ivdata_support;
	i2s_id_in = codec_info->i2s_id_in;
	pr_info("%s: b_ivdata_support=%d, i2s_id_in=%d, size=%zu!\n", __func__, b_ivdata_support, i2s_id_in, size);
	if (i2s_id_in >= I2S_PROP_MAX) {
		pr_err("%s: i2s_id param invalid!\n", __func__);
		return;
	}

	pr_info("%s: codec vendor: %s, dev_cnt: %d.\n", __func__, codec_info->codec_vendor, codec_info->dev_cnt);

	for (i = 0; i < size; i++) {
		if (strncmp(codec_info->codec_vendor, extend_pa_vendor[CODEC_PA_NXP], strlen(extend_pa_vendor[CODEC_PA_NXP]))) {
			continue;
		}

        pr_info("%s: current dailink name: %s, \n", __func__, dailink[i].name);
		if (!strncmp(dailink[i].name, extend_i2s_prop[i2s_id], strlen(extend_i2s_prop[i2s_id]))
			|| (b_ivdata_support && !strncmp(dailink[i].name, extend_i2s_prop[i2s_id_in], strlen(extend_i2s_prop[i2s_id_in])))
		    || (!strncmp(dailink[i].name, OPLUS_SPK_NAME, strlen(OPLUS_SPK_NAME)))
			|| (b_ivdata_support && !strncmp(dailink[i].name, OPLUS_SPK_REF_NAME, strlen(OPLUS_SPK_REF_NAME)))) {
			if (codec_info->dev_cnt == 1) {
				pr_info("%s: use %s mono dailink replace\n", __func__, codec_info->codec_vendor);
				if (!snd_soc_find_dai(&tfa98xx_dails[1])) {
					pr_info("%s cannot find dailink %s", __func__, tfa98xx_dails[1].dai_name);
				}
				pr_info("%s: dailink name = %s\n", __func__, dailink[i].name);
				dailink[i].codecs->of_node = NULL;
				dailink[i].codecs->name = codec_info->codec_name[0];
				dailink[i].codecs->dai_name = codec_info->codec_dai_name[0];
				dailink[i].num_codecs = 1;
			} else if (codec_info->dev_cnt == 2) {
				pr_info("%s: use %s stereo dailink replace\n", __func__, codec_info->codec_vendor);
				for (j = 0; j < codec_info->dev_cnt; j++) {
					tfa98xx_dails[j].name = codec_info->codec_name[j];
					tfa98xx_dails[j].dai_name = codec_info->codec_dai_name[j];
				}
				pr_info("%s: tfa98xx_dails[0] name:%s, dai_name:%s \n", __func__, tfa98xx_dails[0].name, tfa98xx_dails[0].dai_name);
				pr_info("%s: tfa98xx_dails[1] name:%s, dai_name:%s \n", __func__, tfa98xx_dails[1].name, tfa98xx_dails[1].dai_name);
				dailink[i].codecs = tfa98xx_dails;
				dailink[i].num_codecs = ARRAY_SIZE(tfa98xx_dails);
			} else if (codec_info->dev_cnt == 3) {
				pr_info("%s: use %s 3rd dailink replace\n", __func__, codec_info->codec_vendor);
				for (j = 0; j < codec_info->dev_cnt; j++) {
					tfa98xx_dails_3rd[j].name = codec_info->codec_name[j];
					tfa98xx_dails_3rd[j].dai_name = codec_info->codec_dai_name[j];
				}
				pr_info("%s: tfa98xx_dails[0] name:%s, dai_name:%s \n", __func__, tfa98xx_dails_3rd[0].name, tfa98xx_dails_3rd[0].dai_name);
				pr_info("%s: tfa98xx_dails[1] name:%s, dai_name:%s \n", __func__, tfa98xx_dails_3rd[1].name, tfa98xx_dails_3rd[1].dai_name);
				pr_info("%s: tfa98xx_dails[2] name:%s, dai_name:%s \n", __func__, tfa98xx_dails_3rd[2].name, tfa98xx_dails_3rd[2].dai_name);
				dailink[i].codecs = tfa98xx_dails_3rd;
				dailink[i].num_codecs = ARRAY_SIZE(tfa98xx_dails_3rd);
			}
		}
	}
}

void extend_codec_i2s_be_dailinks(struct snd_soc_dai_link *dailink, size_t size)
{
	if (!g_extend_pdata) {
		pr_err("%s: No extend data, do nothing.\n", __func__);
		return;
	}

	pr_info("%s: use_extern_dailink_spk %d\n", __func__, g_extend_pdata->use_extern_dailink_spk);
	if (g_extend_pdata->use_extern_dailink_spk && g_extend_pdata->spk_pa_info) {
		extend_codec_be_dailink(g_extend_pdata->spk_pa_info, dailink, size);
	}
}
EXPORT_SYMBOL(extend_codec_i2s_be_dailinks);

bool extend_codec_i2s_compare(struct snd_soc_dai_link *dailink, int dailink_num)
{
	int i2s_id;

	if (!g_extend_pdata || !g_extend_pdata->spk_pa_info) {
		pr_err("%s: No extend data, do nothing.\n", __func__);
		return false;
	} else {
		i2s_id = g_extend_pdata->spk_pa_info->i2s_id;
	}

	if (i2s_id >= I2S_PROP_MAX) {
		pr_err("%s: i2s_id param invalid!\n", __func__);
		return false;
	}

	if (!dailink) {
		pr_err("%s: dailink param invalid!\n", __func__);
		return false;
	}

	if ((!strcmp(dailink[dailink_num].name, extend_i2s_prop[i2s_id]))
		&& g_extend_pdata->use_extern_dailink_spk) {
		pr_info("%s: use extern dailink spk no need codec_node\n", __func__);
		return true;
	} else {
		return false;
	}
}
EXPORT_SYMBOL(extend_codec_i2s_compare);


int audio_spk_get_i2s_in_type(void)
{
	return g_extend_pdata->spk_pa_info->spk_i2s_in_index;
}
EXPORT_SYMBOL(audio_spk_get_i2s_in_type);

int audio_spk_get_i2s_out_type(void)
{
	return g_extend_pdata->spk_pa_info->spk_i2s_out_index;
}
EXPORT_SYMBOL(audio_spk_get_i2s_out_type);

bool audio_spk_index_support(void)
{
	return g_extend_pdata->spk_pa_info->spk_index_support;
}
EXPORT_SYMBOL(audio_spk_index_support);


static int audio_extend_probe(struct platform_device *pdev)
{
	int ret = 0;

	dev_info(&pdev->dev, "%s: dev name %s\n", __func__,
		dev_name(&pdev->dev));

	if (!pdev->dev.of_node) {
		pr_err("%s: No dev node from device tree\n", __func__);
		return -EINVAL;
	}

	g_extend_pdata = devm_kzalloc(&pdev->dev, sizeof(struct audio_extend_data), GFP_KERNEL);
	if (!g_extend_pdata) {
		pr_err("%s: kzalloc mem fail!\n", __func__);
		return -ENOMEM;
	}

	g_extend_pdata->spk_pa_info =  devm_kzalloc(&pdev->dev, sizeof(struct codec_prop_info), GFP_KERNEL);
	if (g_extend_pdata->spk_pa_info) {
		ret = extend_codec_prop_parse(&pdev->dev, extend_speaker_prop, g_extend_pdata->spk_pa_info);
		if (ret == 0) {
			g_extend_pdata->use_extern_dailink_spk = true;
		} else {
			g_extend_pdata->use_extern_dailink_spk = false;
		}
	} else {
		g_extend_pdata->use_extern_dailink_spk = false;
		pr_warn("%s: kzalloc for spk pa info fail!\n", __func__);
	}

	g_extend_pdata->hp_dac_info =  devm_kzalloc(&pdev->dev, sizeof(struct codec_prop_info), GFP_KERNEL);
	if (g_extend_pdata->hp_dac_info) {
		ret = extend_codec_prop_parse(&pdev->dev, extend_dac_prop, g_extend_pdata->hp_dac_info);
		if (ret == 0) {
			g_extend_pdata->use_extern_dailink_dac = true;
		} else {
			g_extend_pdata->use_extern_dailink_dac = false;
		}
	} else {
		g_extend_pdata->use_extern_dailink_dac = false;
		pr_warn("%s: kzalloc for hp dac info fail!\n", __func__);
	}

	return 0;
}

static int audio_extend_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "%s: dev name %s\n", __func__,
		dev_name(&pdev->dev));

	return 0;
}

static const struct of_device_id audio_extend_of_match[] = {
	{.compatible = "oplus,asoc-audio"},
	{ }
};
MODULE_DEVICE_TABLE(of, audio_extend_of_match);

static struct platform_driver audio_extend_driver = {
	.probe          = audio_extend_probe,
	.remove         = audio_extend_remove,
	.driver         = {
		.name   = AUDIO_EXTEND_DRIVER_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = audio_extend_of_match,
		.suppress_bind_attrs = true,
	},
};
#endif
static int __init audio_extend_init(void)
{
	return platform_driver_register(&audio_extend_driver);
}

static void __exit audio_extend_exit(void)
{
	platform_driver_unregister(&audio_extend_driver);
}

module_init(audio_extend_init);
module_exit(audio_extend_exit);
MODULE_DESCRIPTION("ASoC Oplus Audio Driver");
MODULE_LICENSE("GPL v2");
