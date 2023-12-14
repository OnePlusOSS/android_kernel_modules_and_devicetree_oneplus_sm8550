// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "cam_actuator_dev.h"
#include "cam_req_mgr_dev.h"
#include "cam_actuator_soc.h"
#include "cam_actuator_core.h"
#include "cam_trace.h"
#include "camera_main.h"
#include "cam_compat.h"

static struct cam_i3c_actuator_data {
	struct cam_actuator_ctrl_t                  *a_ctrl;
	struct completion                            probe_complete;
} g_i3c_actuator_data[MAX_CAMERAS];

struct completion *cam_actuator_get_i3c_completion(uint32_t index)
{
	return &g_i3c_actuator_data[index].probe_complete;
}

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#include "oplus_cam_actuator_dev.h"

#define VIDIOC_CAM_ACTUATOR_LOCK 0x9003
#define VIDIOC_CAM_ACTUATOR_UNLOCK 0x9004
#define VIDIOC_CAM_ACTUATOR_SHAKE_DETECT_ENABLE 0x9005
#endif

static int cam_actuator_subdev_close_internal(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	struct cam_actuator_ctrl_t *a_ctrl =
		v4l2_get_subdevdata(sd);

	if (!a_ctrl) {
		CAM_ERR(CAM_ACTUATOR, "a_ctrl ptr is NULL");
		return -EINVAL;
	}

	mutex_lock(&(a_ctrl->actuator_mutex));
	cam_actuator_shutdown(a_ctrl);
	mutex_unlock(&(a_ctrl->actuator_mutex));

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if(a_ctrl->camera_actuator_shake_detect_enable &&
		a_ctrl->cam_act_last_state == CAM_ACTUATOR_LOCK){
		oplus_cam_actuator_unlock(a_ctrl);
		CAM_INFO(CAM_ACTUATOR, "oplus_cam_actuator_unlock");
	}
#endif

	return 0;
}

static int cam_actuator_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	bool crm_active = cam_req_mgr_is_open();

	if (crm_active) {
		CAM_DBG(CAM_ACTUATOR,
			"CRM is ACTIVE, close should be from CRM");
		return 0;
	}

	return cam_actuator_subdev_close_internal(sd, fh);
}

static long cam_actuator_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	int rc = 0;
	struct cam_actuator_ctrl_t *a_ctrl =
		v4l2_get_subdevdata(sd);

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	mutex_lock(&(a_ctrl->actuator_ioctl_mutex));
#endif

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_actuator_driver_cmd(a_ctrl, arg);
		if (rc) {
			if (rc == -EBADR)
				CAM_INFO(CAM_ACTUATOR,
					"Failed for driver_cmd: %d, it has been flushed",
					rc);
			else
				CAM_ERR(CAM_ACTUATOR,
					"Failed for driver_cmd: %d", rc);
		}
		break;
	case CAM_SD_SHUTDOWN:
		if (!cam_req_mgr_is_shutdown()) {
			CAM_ERR(CAM_CORE, "SD shouldn't come from user space");
#ifdef OPLUS_FEATURE_CAMERA_COMMON
			mutex_unlock(&(a_ctrl->actuator_ioctl_mutex));
#endif
			return 0;
		}

		rc = cam_actuator_subdev_close_internal(sd, NULL);
		break;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	case VIDIOC_CAM_ACTUATOR_SHAKE_DETECT_ENABLE:
		oplus_cam_actuator_sds_enable(a_ctrl);
		break;
	case VIDIOC_CAM_ACTUATOR_LOCK:
		down(&a_ctrl->actuator_sem);
		rc = oplus_cam_actuator_lock(a_ctrl);
		up(&a_ctrl->actuator_sem);
		break;
	case VIDIOC_CAM_ACTUATOR_UNLOCK:
		down(&a_ctrl->actuator_sem);
		rc = oplus_cam_actuator_unlock(a_ctrl);
		up(&a_ctrl->actuator_sem);
		break;
#endif
	default:
		CAM_ERR(CAM_ACTUATOR, "Invalid ioctl cmd: %u", cmd);
		rc = -ENOIOCTLCMD;
		break;
	}

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	mutex_unlock(&(a_ctrl->actuator_ioctl_mutex));
#endif

	return rc;
}

#ifdef CONFIG_COMPAT
static long cam_actuator_init_subdev_do_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, unsigned long arg)
{
	struct cam_control cmd_data;
	int32_t rc = 0;

	if (copy_from_user(&cmd_data, (void __user *)arg,
		sizeof(cmd_data))) {
		CAM_ERR(CAM_ACTUATOR,
			"Failed to copy from user_ptr=%pK size=%zu",
			(void __user *)arg, sizeof(cmd_data));
		return -EFAULT;
	}

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		cmd = VIDIOC_CAM_CONTROL;
		rc = cam_actuator_subdev_ioctl(sd, cmd, &cmd_data);
		if (rc) {
			CAM_ERR(CAM_ACTUATOR,
				"Failed in actuator subdev handling rc: %d",
				rc);
			return rc;
		}
		break;
	default:
		CAM_ERR(CAM_ACTUATOR, "Invalid compat ioctl: %d", cmd);
		rc = -ENOIOCTLCMD;
		break;
	}

	if (!rc) {
		if (copy_to_user((void __user *)arg, &cmd_data,
			sizeof(cmd_data))) {
			CAM_ERR(CAM_ACTUATOR,
				"Failed to copy to user_ptr=%pK size=%zu",
				(void __user *)arg, sizeof(cmd_data));
			rc = -EFAULT;
		}
	}
	return rc;
}
#endif

static struct v4l2_subdev_core_ops cam_actuator_subdev_core_ops = {
	.ioctl = cam_actuator_subdev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = cam_actuator_init_subdev_do_ioctl,
#endif
};

static struct v4l2_subdev_ops cam_actuator_subdev_ops = {
	.core = &cam_actuator_subdev_core_ops,
};

static const struct v4l2_subdev_internal_ops cam_actuator_internal_ops = {
	.close = cam_actuator_subdev_close,
};

static int cam_actuator_init_subdev(struct cam_actuator_ctrl_t *a_ctrl)
{
	int rc = 0;

	a_ctrl->v4l2_dev_str.internal_ops =
		&cam_actuator_internal_ops;
	a_ctrl->v4l2_dev_str.ops =
		&cam_actuator_subdev_ops;
	strlcpy(a_ctrl->device_name, CAMX_ACTUATOR_DEV_NAME,
		sizeof(a_ctrl->device_name));
	a_ctrl->v4l2_dev_str.name =
		a_ctrl->device_name;
	a_ctrl->v4l2_dev_str.sd_flags =
		(V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS);
	a_ctrl->v4l2_dev_str.ent_function =
		CAM_ACTUATOR_DEVICE_TYPE;
	a_ctrl->v4l2_dev_str.token = a_ctrl;
	a_ctrl->v4l2_dev_str.close_seq_prior =
		 CAM_SD_CLOSE_MEDIUM_PRIORITY;

	rc = cam_register_subdev(&(a_ctrl->v4l2_dev_str));
	if (rc)
		CAM_ERR(CAM_ACTUATOR,
			"Fail with cam_register_subdev rc: %d", rc);

	return rc;
}

static int cam_actuator_i2c_component_bind(struct device *dev,
	struct device *master_dev, void *data)
{
	int32_t                          rc = 0;
	int32_t                          i = 0;
	struct i2c_client               *client;
	struct cam_actuator_ctrl_t      *a_ctrl;
	struct cam_hw_soc_info          *soc_info = NULL;
	struct cam_actuator_soc_private *soc_private = NULL;

	client = container_of(dev, struct i2c_client, dev);
	if (!client) {
		CAM_ERR(CAM_ACTUATOR,
			"Failed to get i2c client");
		return -EFAULT;
	}

	/* Create sensor control structure */
	a_ctrl = kzalloc(sizeof(*a_ctrl), GFP_KERNEL);
	if (!a_ctrl)
		return -ENOMEM;

	i2c_set_clientdata(client, a_ctrl);

	soc_private = kzalloc(sizeof(struct cam_actuator_soc_private),
		GFP_KERNEL);
	if (!soc_private) {
		rc = -ENOMEM;
		goto free_ctrl;
	}
	a_ctrl->soc_info.soc_private = soc_private;

	a_ctrl->io_master_info.client = client;
	soc_info = &a_ctrl->soc_info;
	soc_info->dev = &client->dev;
	soc_info->dev_name = client->name;
	a_ctrl->io_master_info.master_type = I2C_MASTER;

	rc = cam_actuator_parse_dt(a_ctrl, &client->dev);
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "failed: cam_sensor_parse_dt rc %d", rc);
		goto free_soc;
	}

	rc = cam_actuator_init_subdev(a_ctrl);
	if (rc)
		goto free_soc;

	if (soc_private->i2c_info.slave_addr != 0)
		a_ctrl->io_master_info.client->addr =
			soc_private->i2c_info.slave_addr;

	a_ctrl->i2c_data.per_frame =
		kzalloc(sizeof(struct i2c_settings_array) *
		MAX_PER_FRAME_ARRAY, GFP_KERNEL);
	if (a_ctrl->i2c_data.per_frame == NULL) {
		rc = -ENOMEM;
		goto unreg_subdev;
	}

	INIT_LIST_HEAD(&(a_ctrl->i2c_data.init_settings.list_head));

	for (i = 0; i < MAX_PER_FRAME_ARRAY; i++)
		INIT_LIST_HEAD(&(a_ctrl->i2c_data.per_frame[i].list_head));

	a_ctrl->bridge_intf.device_hdl = -1;
	a_ctrl->bridge_intf.link_hdl = -1;
	a_ctrl->bridge_intf.ops.get_dev_info =
		cam_actuator_publish_dev_info;
	a_ctrl->bridge_intf.ops.link_setup =
		cam_actuator_establish_link;
	a_ctrl->bridge_intf.ops.apply_req =
		cam_actuator_apply_request;
	a_ctrl->last_flush_req = 0;
	a_ctrl->cam_act_state = CAM_ACTUATOR_INIT;

	return rc;

unreg_subdev:
	cam_unregister_subdev(&(a_ctrl->v4l2_dev_str));
free_soc:
	kfree(soc_private);
free_ctrl:
	kfree(a_ctrl);
	return rc;
}

static void cam_actuator_i2c_component_unbind(struct device *dev,
	struct device *master_dev, void *data)
{
	struct i2c_client               *client = NULL;
	struct cam_actuator_ctrl_t      *a_ctrl = NULL;
	struct cam_actuator_soc_private *soc_private;
	struct cam_sensor_power_ctrl_t  *power_info;

	client = container_of(dev, struct i2c_client, dev);
	if (!client) {
		CAM_ERR(CAM_ACTUATOR,
			"Failed to get i2c client");
		return;
	}

	a_ctrl = i2c_get_clientdata(client);
	/* Handle I2C Devices */
	if (!a_ctrl) {
		CAM_ERR(CAM_ACTUATOR, "Actuator device is NULL");
		return;
	}

	CAM_INFO(CAM_ACTUATOR, "i2c remove invoked");
	mutex_lock(&(a_ctrl->actuator_mutex));
	cam_actuator_shutdown(a_ctrl);
	mutex_unlock(&(a_ctrl->actuator_mutex));
	cam_unregister_subdev(&(a_ctrl->v4l2_dev_str));
	soc_private =
		(struct cam_actuator_soc_private *)a_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	/*Free Allocated Mem */
	kfree(a_ctrl->i2c_data.per_frame);
	a_ctrl->i2c_data.per_frame = NULL;
	a_ctrl->soc_info.soc_private = NULL;
	v4l2_set_subdevdata(&a_ctrl->v4l2_dev_str.sd, NULL);
	kfree(a_ctrl);
}

const static struct component_ops cam_actuator_i2c_component_ops = {
	.bind = cam_actuator_i2c_component_bind,
	.unbind = cam_actuator_i2c_component_unbind,
};

static int32_t cam_actuator_driver_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;

	if (client == NULL || id == NULL) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Args client: %pK id: %pK",
			client, id);
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CAM_ERR(CAM_ACTUATOR, "%s :: i2c_check_functionality failed",
			 client->name);
		return -EFAULT;
	}

	CAM_DBG(CAM_ACTUATOR, "Adding sensor actuator component");
	rc = component_add(&client->dev, &cam_actuator_i2c_component_ops);
	if (rc)
		CAM_ERR(CAM_ACTUATOR, "failed to add component rc: %d", rc);

	return rc;
}

static int32_t cam_actuator_driver_i2c_remove(
	struct i2c_client *client)
{
	component_del(&client->dev, &cam_actuator_i2c_component_ops);
	return 0;
}

static int cam_actuator_platform_component_bind(struct device *dev,
	struct device *master_dev, void *data)
{
	int32_t                          rc = 0;
	int32_t                          i = 0;
	bool                             i3c_i2c_target;
	struct cam_actuator_ctrl_t       *a_ctrl = NULL;
	struct cam_actuator_soc_private  *soc_private = NULL;
	struct platform_device *pdev = to_platform_device(dev);

	i3c_i2c_target = of_property_read_bool(pdev->dev.of_node, "i3c-i2c-target");
	if (i3c_i2c_target)
		return 0;

	/* Create actuator control structure */
	a_ctrl = devm_kzalloc(&pdev->dev,
		sizeof(struct cam_actuator_ctrl_t), GFP_KERNEL);
	if (!a_ctrl)
		return -ENOMEM;

	/*fill in platform device*/
	a_ctrl->v4l2_dev_str.pdev = pdev;
	a_ctrl->soc_info.pdev = pdev;
	a_ctrl->soc_info.dev = &pdev->dev;
	a_ctrl->soc_info.dev_name = pdev->name;
	a_ctrl->io_master_info.master_type = CCI_MASTER;

	a_ctrl->io_master_info.cci_client = kzalloc(sizeof(
		struct cam_sensor_cci_client), GFP_KERNEL);
	if (!(a_ctrl->io_master_info.cci_client)) {
		rc = -ENOMEM;
		goto free_ctrl;
	}

	soc_private = kzalloc(sizeof(struct cam_actuator_soc_private),
		GFP_KERNEL);
	if (!soc_private) {
		rc = -ENOMEM;
		goto free_cci_client;
	}
	a_ctrl->soc_info.soc_private = soc_private;
	soc_private->power_info.dev = &pdev->dev;

	a_ctrl->i2c_data.per_frame =
		kzalloc(sizeof(struct i2c_settings_array) *
		MAX_PER_FRAME_ARRAY, GFP_KERNEL);
	if (a_ctrl->i2c_data.per_frame == NULL) {
		rc = -ENOMEM;
		goto free_soc;
	}

	INIT_LIST_HEAD(&(a_ctrl->i2c_data.init_settings.list_head));

	for (i = 0; i < MAX_PER_FRAME_ARRAY; i++)
		INIT_LIST_HEAD(&(a_ctrl->i2c_data.per_frame[i].list_head));

	rc = cam_actuator_parse_dt(a_ctrl, &(pdev->dev));
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "Paring actuator dt failed rc %d", rc);
		goto free_mem;
	}

	/* Fill platform device id*/
	pdev->id = a_ctrl->soc_info.index;

	rc = cam_actuator_init_subdev(a_ctrl);
	if (rc)
		goto free_mem;

	a_ctrl->bridge_intf.device_hdl = -1;
	a_ctrl->bridge_intf.link_hdl = -1;
	a_ctrl->bridge_intf.ops.get_dev_info =
		cam_actuator_publish_dev_info;
	a_ctrl->bridge_intf.ops.link_setup =
		cam_actuator_establish_link;
	a_ctrl->bridge_intf.ops.apply_req =
		cam_actuator_apply_request;
	a_ctrl->bridge_intf.ops.flush_req =
		cam_actuator_flush_request;
	a_ctrl->last_flush_req = 0;

	platform_set_drvdata(pdev, a_ctrl);
	a_ctrl->cam_act_state = CAM_ACTUATOR_INIT;
	CAM_DBG(CAM_ACTUATOR, "Component bound successfully %d",
		a_ctrl->soc_info.index);

	g_i3c_actuator_data[a_ctrl->soc_info.index].a_ctrl = a_ctrl;
	init_completion(&g_i3c_actuator_data[a_ctrl->soc_info.index].probe_complete);

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	a_ctrl->cam_act_last_state = CAM_ACTUATOR_INIT;
	if (a_ctrl->is_update_pid)
	{
		CAM_INFO(CAM_ACTUATOR, "create af download fw thread");
		a_ctrl->actuator_update_pid_thread = kthread_run(oplus_cam_actuator_update_pid, a_ctrl, a_ctrl->device_name);
	}
	else
	{
		CAM_INFO(CAM_ACTUATOR, "not need update pid");
	}
#endif
	return rc;

free_mem:
	kfree(a_ctrl->i2c_data.per_frame);
free_soc:
	kfree(soc_private);
free_cci_client:
	kfree(a_ctrl->io_master_info.cci_client);
free_ctrl:
	devm_kfree(&pdev->dev, a_ctrl);
	return rc;
}

static void cam_actuator_platform_component_unbind(struct device *dev,
	struct device *master_dev, void *data)
{
	struct cam_actuator_ctrl_t      *a_ctrl;
	struct cam_actuator_soc_private *soc_private;
	struct cam_sensor_power_ctrl_t  *power_info;
	bool                             i3c_i2c_target;
	struct platform_device *pdev = to_platform_device(dev);

	i3c_i2c_target = of_property_read_bool(pdev->dev.of_node, "i3c-i2c-target");
	if (i3c_i2c_target)
		return;

	a_ctrl = platform_get_drvdata(pdev);
	if (!a_ctrl) {
		CAM_ERR(CAM_ACTUATOR, "Actuator device is NULL");
		return;
	}

	mutex_lock(&(a_ctrl->actuator_mutex));
	cam_actuator_shutdown(a_ctrl);
	mutex_unlock(&(a_ctrl->actuator_mutex));
	cam_unregister_subdev(&(a_ctrl->v4l2_dev_str));

	soc_private =
		(struct cam_actuator_soc_private *)a_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	kfree(a_ctrl->io_master_info.cci_client);
	a_ctrl->io_master_info.cci_client = NULL;
	kfree(a_ctrl->soc_info.soc_private);
	a_ctrl->soc_info.soc_private = NULL;
	kfree(a_ctrl->i2c_data.per_frame);
	a_ctrl->i2c_data.per_frame = NULL;
	v4l2_set_subdevdata(&a_ctrl->v4l2_dev_str.sd, NULL);
	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, a_ctrl);
	CAM_INFO(CAM_ACTUATOR, "Actuator component unbinded");
}

const static struct component_ops cam_actuator_platform_component_ops = {
	.bind = cam_actuator_platform_component_bind,
	.unbind = cam_actuator_platform_component_unbind,
};

static int32_t cam_actuator_platform_remove(
	struct platform_device *pdev)
{
	component_del(&pdev->dev, &cam_actuator_platform_component_ops);
	return 0;
}

static const struct of_device_id cam_actuator_driver_dt_match[] = {
	{.compatible = "qcom,actuator"},
	{}
};

static int32_t cam_actuator_driver_platform_probe(
	struct platform_device *pdev)
{
	int rc = 0;

	CAM_DBG(CAM_ACTUATOR, "Adding sensor actuator component");
	rc = component_add(&pdev->dev, &cam_actuator_platform_component_ops);
	if (rc)
		CAM_ERR(CAM_ACTUATOR, "failed to add component rc: %d", rc);

	return rc;
}

MODULE_DEVICE_TABLE(of, cam_actuator_driver_dt_match);

struct platform_driver cam_actuator_platform_driver = {
	.probe = cam_actuator_driver_platform_probe,
	.driver = {
		.name = "qcom,actuator",
		.owner = THIS_MODULE,
		.of_match_table = cam_actuator_driver_dt_match,
		.suppress_bind_attrs = true,
	},
	.remove = cam_actuator_platform_remove,
};

static const struct i2c_device_id i2c_id[] = {
	{ACTUATOR_DRIVER_I2C, (kernel_ulong_t)NULL},
	{ }
};

static const struct of_device_id cam_actuator_i2c_driver_dt_match[] = {
	{.compatible = "qcom,cam-i2c-actuator"},
	{}
};
MODULE_DEVICE_TABLE(of, cam_actuator_i2c_driver_dt_match);

struct i2c_driver cam_actuator_i2c_driver = {
	.id_table = i2c_id,
	.probe  = cam_actuator_driver_i2c_probe,
	.remove = cam_actuator_driver_i2c_remove,
	.driver = {
		.of_match_table = cam_actuator_i2c_driver_dt_match,
		.owner = THIS_MODULE,
		.name = ACTUATOR_DRIVER_I2C,
		.suppress_bind_attrs = true,
	},
};

static struct i3c_device_id actuator_i3c_id[MAX_I3C_DEVICE_ID_ENTRIES + 1];

static int cam_actuator_i3c_driver_probe(struct i3c_device *client)
{
	int32_t rc = 0;
	struct cam_actuator_ctrl_t       *a_ctrl = NULL;
	uint32_t                          index;
	struct device                    *dev;

	if (!client) {
		CAM_INFO(CAM_ACTUATOR, "Null Client pointer");
		return -EINVAL;
	}

	dev = &client->dev;

	CAM_DBG(CAM_ACTUATOR, "Probe for I3C Slave %s", dev_name(dev));

	rc = of_property_read_u32(dev->of_node, "cell-index", &index);
	if (rc) {
		CAM_ERR(CAM_ACTUATOR, "device %s failed to read cell-index", dev_name(dev));
		return rc;
	}

	if (index >= MAX_CAMERAS) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Cell-Index: %u for %s", index, dev_name(dev));
		return -EINVAL;
	}

	a_ctrl = g_i3c_actuator_data[index].a_ctrl;
	if (!a_ctrl) {
		CAM_ERR(CAM_ACTUATOR,
			"a_ctrl is null. I3C Probe before platfom driver probe for %s",
			dev_name(dev));
		return -EINVAL;
	}

	a_ctrl->io_master_info.i3c_client = client;

	complete_all(&g_i3c_actuator_data[index].probe_complete);

	CAM_DBG(CAM_ACTUATOR, "I3C Probe Finished for %s", dev_name(dev));
	return rc;
}

static struct i3c_driver cam_actuator_i3c_driver = {
	.id_table = actuator_i3c_id,
	.probe = cam_actuator_i3c_driver_probe,
	.remove = cam_i3c_driver_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = ACTUATOR_DRIVER_I3C,
		.of_match_table = cam_actuator_driver_dt_match,
		.suppress_bind_attrs = true,
	},
};

int cam_actuator_driver_init(void)
{
	int32_t rc = 0;
	struct device_node                      *dev;
	int num_entries = 0;

	rc = platform_driver_register(&cam_actuator_platform_driver);
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR,
			"platform_driver_register failed rc = %d", rc);
		return rc;
	}

	rc = i2c_add_driver(&cam_actuator_i2c_driver);
	if (rc) {
		CAM_ERR(CAM_ACTUATOR, "i2c_add_driver failed rc = %d", rc);
		goto i2c_register_err;
	}

	memset(actuator_i3c_id, 0, sizeof(struct i3c_device_id) * (MAX_I3C_DEVICE_ID_ENTRIES + 1));

	dev = of_find_node_by_path(I3C_SENSOR_DEV_ID_DT_PATH);
	if (!dev) {
		CAM_DBG(CAM_ACTUATOR, "Couldnt Find the i3c-id-table dev node");
		return 0;
	}

	rc = cam_sensor_count_elems_i3c_device_id(dev, &num_entries,
		"i3c-actuator-id-table");
	if (rc)
		return 0;

	rc = cam_sensor_fill_i3c_device_id(dev, num_entries,
		"i3c-actuator-id-table", actuator_i3c_id);
	if (rc)
		goto i3c_register_err;

	rc = i3c_driver_register_with_owner(&cam_actuator_i3c_driver, THIS_MODULE);
	if (rc) {
		CAM_ERR(CAM_ACTUATOR, "i3c_driver registration failed, rc: %d", rc);
		goto i3c_register_err;
	}

	return 0;

i3c_register_err:
	i2c_del_driver(&cam_actuator_i2c_driver);
i2c_register_err:
	platform_driver_unregister(&cam_actuator_platform_driver);

	return rc;
}

void cam_actuator_driver_exit(void)
{
	struct device_node *dev;

	platform_driver_unregister(&cam_actuator_platform_driver);
	i2c_del_driver(&cam_actuator_i2c_driver);

	dev = of_find_node_by_path(I3C_SENSOR_DEV_ID_DT_PATH);
	if (!dev) {
		CAM_DBG(CAM_ACTUATOR, "Couldnt Find the i3c-id-table dev node");
		return;
	}

	i3c_driver_unregister(&cam_actuator_i3c_driver);
}

MODULE_DESCRIPTION("cam_actuator_driver");
MODULE_LICENSE("GPL v2");
