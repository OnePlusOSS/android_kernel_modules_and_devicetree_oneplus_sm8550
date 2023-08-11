#define pr_fmt(fmt) "[VIRTUAL_SWITCHING]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/iio/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/list.h>
#include <linux/of_irq.h>
#include <soc/oplus/system/boot_mode.h>
#include <soc/oplus/device_info.h>
#include <soc/oplus/system/oplus_project.h>
#include <oplus_chg.h>
#include <oplus_chg_module.h>
#include <oplus_chg_ic.h>
#include <oplus_hal_vooc.h>
#include <oplus_mms_gauge.h>

struct oplus_virtual_switching_child {
	struct oplus_chg_ic_dev *ic_dev;
	int ic_type;
	bool initialized;
};

struct oplus_virtual_switching_ic {
	struct device *dev;
	struct oplus_chg_ic_dev *ic_dev;
	struct oplus_virtual_switching_child *child_list;;
	int child_num;
};


static struct oplus_chg_ic_virq oplus_switching_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
};

static void oplus_switching_err_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	struct oplus_virtual_switching_ic *chip = virq_data;

	oplus_chg_ic_move_err_msg(chip->ic_dev, ic_dev);
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ERR);
}

static int oplus_switching_virq_register(struct oplus_virtual_switching_ic *chip)
{
	int i, rc;

	for (i = 0; i < chip->child_num; i++) {
		rc = oplus_chg_ic_virq_register(
			chip->child_list[i].ic_dev, OPLUS_IC_VIRQ_ERR,
			oplus_switching_err_handler, chip);
		if (rc < 0)
			chg_err("register OPLUS_IC_VIRQ_ERR error, rc=%d",
				rc);
	}

	return 0;
}

static int oplus_chg_switching_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_switching_ic *chip;
	struct device_node *node;
	bool retry = false;
	int i;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	node = chip->dev->of_node;

	ic_dev->online = true;
	for (i = 0; i < chip->child_num; i++) {
		if (chip->child_list[i].initialized)
			continue;
		chip->child_list[i].ic_dev =
			of_get_oplus_chg_ic(node, "oplus,switch_ic", i);
		if (chip->child_list[i].ic_dev == NULL) {
			chg_debug("switch ic[%d] not found\n", i);
			retry = true;
			continue;
		}
		chip->child_list[i].ic_dev->parent = ic_dev;
		rc = oplus_chg_ic_func(chip->child_list[i].ic_dev, OPLUS_IC_FUNC_INIT);
		chip->child_list[i].initialized = true;
		if (rc >= 0) {
			chg_info("switch ic(=%s) init success\n",
				 chip->child_list[i].ic_dev->name);
			chip->child_list[i].ic_type = rc;
			goto init_done;
		} else {
			chg_err("switch ic(=%s) init error, rc=%d\n",
				chip->child_list[i].ic_dev->name, rc);
		}
	}

	if (retry) {
		return -EAGAIN;
	} else {
		chg_err("all switch ic init error\n");
		return -EINVAL;
	}

init_done:
	oplus_switching_virq_register(chip);
	return rc;
}

static int oplus_chg_switching_exit(struct oplus_chg_ic_dev *ic_dev)
{
	struct oplus_virtual_switching_ic *chip;
	int i;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (!ic_dev->online)
		return 0;

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	ic_dev->online = false;
	for (i = 0; i < chip->child_num; i++) {
		oplus_chg_ic_virq_release(chip->child_list[i].ic_dev, OPLUS_IC_VIRQ_ERR, chip);
		oplus_chg_ic_func(chip->child_list[i].ic_dev, OPLUS_IC_FUNC_EXIT);
	}

	return 0;
}

static int oplus_chg_switching_set_enable(struct oplus_chg_ic_dev *ic_dev, bool enable)
{
	struct oplus_virtual_switching_ic *chip;
	int i;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < chip->child_num; i++) {
		oplus_chg_ic_func(chip->child_list[i].ic_dev,
				  OPLUS_IC_FUNC_SET_HW_ENABLE, enable);
	}

	return 0;
}

static int oplus_chg_switching_get_enable(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct oplus_virtual_switching_ic *chip;
	int i;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < chip->child_num; i++) {
		oplus_chg_ic_func(chip->child_list[i].ic_dev,
				  OPLUS_IC_FUNC_GET_HW_ENABLE, enable);
	}

	return 0;
}

static int oplus_chg_switching_get_type(struct oplus_chg_ic_dev *ic_dev, int *type)
{
	struct oplus_virtual_switching_ic *chip;
	int i;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	for (i = 0; i < chip->child_num; i++) {
		*type = chip->child_list[i].ic_type;
	}

	return 0;
}

static void *oplus_chg_switching_get_func(struct oplus_chg_ic_dev *ic_dev,
				   enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT,
					       oplus_chg_switching_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT,
					       oplus_chg_switching_exit);
		break;
	case OPLUS_IC_FUNC_SET_HW_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_HW_ENABLE,
					       oplus_chg_switching_set_enable);
		break;
	case OPLUS_IC_FUNC_GET_HW_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_HW_ENABLE,
					       oplus_chg_switching_get_enable);
		break;
	case OPLUS_IC_FUNC_GET_DEVICE_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_DEVICE_TYPE,
					       oplus_chg_switching_get_type);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

static int oplus_switching_child_init(struct oplus_virtual_switching_ic *chip)
{
	struct device_node *node = chip->dev->of_node;
	int rc = 0;

	rc = of_property_count_elems_of_size(node, "oplus,switch_ic",
					     sizeof(u32));
	if (rc < 0) {
		chg_err("can't get switch_ic number, rc=%d\n", rc);
		return rc;
	}
	chip->child_num = rc;
	chip->child_list = devm_kzalloc(chip->dev,
				       sizeof(struct oplus_virtual_switching_child) *
					       chip->child_num,
				       GFP_KERNEL);
	if (chip->child_list == NULL) {
		rc = -ENOMEM;
		chg_err("alloc switch_ic table memory error\n");
		return rc;
	}

	return 0;
}

static int oplus_virtual_switching_probe(struct platform_device *pdev)
{
	struct oplus_virtual_switching_ic *chip;
	struct device_node *node = pdev->dev.of_node;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	int rc = 0;

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_virtual_switching_ic),
			    GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc memory error\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);

	if (rc < 0)
		chg_err("gpio init error, rc=%d\n", rc);

	rc = oplus_switching_child_init(chip);
	if (rc < 0) {
		chg_err("child list init error, rc=%d\n", rc);
		goto child_init_err;
	}

	rc = of_property_read_u32(node, "oplus,ic_type", &ic_type);
	if (rc < 0) {
		chg_err("can't get ic type, rc=%d\n", rc);
		goto reg_ic_err;
	}

	rc = of_property_read_u32(node, "oplus,ic_index", &ic_index);
	if (rc < 0) {
		chg_err("can't get ic index, rc=%d\n", rc);
		goto reg_ic_err;
	}

	ic_cfg.name = node->name;
	ic_cfg.index = ic_index;
	sprintf(ic_cfg.manu_name, "virtual switching");
	sprintf(ic_cfg.fw_id, "0x00");
	ic_cfg.type = ic_type;
	ic_cfg.get_func = oplus_chg_switching_get_func;
	ic_cfg.virq_data = oplus_switching_virq_table;
	ic_cfg.virq_num = ARRAY_SIZE(oplus_switching_virq_table);
	chip->ic_dev =
		devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
	if (!chip->ic_dev) {
		rc = -ENODEV;
		chg_err("register %s error\n", node->name);
		goto reg_ic_err;
	}

	chg_err("probe success\n");
	return 0;

reg_ic_err:
	devm_kfree(&pdev->dev, chip->child_list);
child_init_err:
	devm_kfree(&pdev->dev, chip);
	platform_set_drvdata(pdev, NULL);

	chg_err("probe error\n");
	return rc;
}

static int oplus_virtual_switching_remove(struct platform_device *pdev)
{
	struct oplus_virtual_switching_ic *chip = platform_get_drvdata(pdev);

	if (chip == NULL)
		return -ENODEV;

	if (chip->ic_dev->online)
		oplus_chg_switching_exit(chip->ic_dev);
	devm_oplus_chg_ic_unregister(&pdev->dev, chip->ic_dev);
	devm_kfree(&pdev->dev, chip->child_list);
	devm_kfree(&pdev->dev, chip);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static void oplus_virtual_switching_shutdown(struct platform_device *pdev)
{
/* TODO
*/
}

static const struct of_device_id oplus_virtual_switching_match[] = {
	{ .compatible = "oplus,virtual_switching" },
	{},
};

static struct platform_driver oplus_virtual_switching_driver = {
	.driver		= {
		.name = "oplus-virtual_switching",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_virtual_switching_match),
	},
	.probe		= oplus_virtual_switching_probe,
	.remove		= oplus_virtual_switching_remove,
	.shutdown	= oplus_virtual_switching_shutdown,
};

static __init int oplus_virtual_switching_init(void)
{
	return platform_driver_register(&oplus_virtual_switching_driver);
}

static __exit void oplus_virtual_switching_exit(void)
{
	platform_driver_unregister(&oplus_virtual_switching_driver);
}

oplus_chg_module_register(oplus_virtual_switching);
