/************************************************************************************
** File: - oplus_speaker_manager_codec.c
**
** Copyright (C), 2020-2024, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**      Add for oplus audio speaker manager
**
** Version: 1.0
**
** --------------------------- Revision History: --------------------------------
**    <author>       <date>          <desc>
************************************************************************************/

#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/scatterlist.h>
#include <linux/list.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "oplus_speaker_manager_codec.h"
/*------------------------------------------------------------------------------*/
static struct list_head oplus_speaker_list = LIST_HEAD_INIT(oplus_speaker_list);
static DEFINE_MUTEX(list_mutex);

struct oplus_amp_status *contrl_status = NULL;
/*------------------------------------------------------------------------------*/
struct oplus_speaker_device* get_speaker_dev(enum oplus_pa_type pa_type)
{
	struct list_head *p;
	struct oplus_spk_dev_node *entry;

	if (list_empty(&oplus_speaker_list)) {
		pr_err("%s, %d, oplus_speaker_list is empty!\n", __func__, __LINE__);

		return NULL;
	}

	list_for_each(p, &oplus_speaker_list) {
		entry = list_entry(p, struct oplus_spk_dev_node, list);
		if (entry->device && entry->device->type == pa_type) {
			return entry->device;
		}
	}

	return NULL;
}
EXPORT_SYMBOL(get_speaker_dev);
/*------------------------------------------------------------------------------*/
enum oplus_pa_manufacturer get_speaker_manufacturer(enum oplus_pa_type pa_type)
{
	struct list_head *p;
	struct oplus_spk_dev_node *entry;

	if (list_empty(&oplus_speaker_list)) {
		pr_err("%s, %d, oplus_speaker_list is empty!\n", __func__, __LINE__);
		return -1;
	}

	list_for_each(p, &oplus_speaker_list) {
		entry = list_entry(p, struct oplus_spk_dev_node, list);
		if (entry->device && entry->device->type == pa_type) {
			pr_err("%s, %d, entry->device->speaker_manufacture = %d\n", __func__, __LINE__, entry->device->speaker_manufacture);

			return entry->device->speaker_manufacture;
		}
	}

	pr_err("%s, %d, No speaker device!\n", __func__, __LINE__);

	return -1;
}
EXPORT_SYMBOL(get_speaker_manufacturer);
/*------------------------------------------------------------------------------*/
void *oplus_speaker_pa_register(struct oplus_speaker_device *speaker_device)
{
	struct oplus_spk_dev_node *spk_dev_node;

	if (speaker_device == NULL) {
		pr_err("[%s],bad param!\n", __func__);
		return NULL;
	}

	spk_dev_node = kzalloc(sizeof(struct oplus_spk_dev_node), GFP_KERNEL);
	if (spk_dev_node == NULL) {
		pr_err("[%s],node kzalloc fail!\n", __func__);
		return NULL;
	}

	spk_dev_node->device = speaker_device;

	mutex_lock(&list_mutex);
	list_add(&spk_dev_node->list, &oplus_speaker_list);
	mutex_unlock(&list_mutex);

	if (contrl_status != NULL) {
		contrl_status->chipset |= speaker_device->type;
		pr_info("%s(),register id:%d, type:%d, contrl_status.chipset = %d\n", __func__, speaker_device->chipset, speaker_device->type, contrl_status->chipset);
	} else {
		pr_info("%s(),register id:%d, type:%d \n", __func__, speaker_device->chipset, speaker_device->type);
	}

	return spk_dev_node;
}
EXPORT_SYMBOL(oplus_speaker_pa_register);
/*------------------------------------------------------------------------------*/
int oplus_speaker_pa_unregister(void *node)
{
	struct oplus_spk_dev_node *spk_dev_node = node;

	if (node == NULL) {
		pr_err("[%s],bab param!\n", __func__);
		return -1;
	}

	mutex_lock(&list_mutex);
	list_del(&spk_dev_node->list);
	mutex_unlock(&list_mutex);

	pr_info("%s(),register id:%d, type:%d \n", __func__, spk_dev_node->device->chipset, spk_dev_node->device->type);
	kfree(node);

	return 0;
}
EXPORT_SYMBOL(oplus_speaker_pa_unregister);
/*------------------------------------------------------------------------------*/
MODULE_DESCRIPTION("PA Manager ASoC driver");
MODULE_LICENSE("GPL");

