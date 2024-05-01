/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor ctrl driver
 */
#include "vendor_ctrl.h"
#include <securec.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include "cam_actuator_dev.h"
#include "cam_ois_dev.h"
#include "cam_sensor_dev.h"
#include "vendor_debug_adapter.h"

#define INIT_LIST_HEAD(ptr) do { \
	(ptr)->next = (ptr); (ptr)->prev = (ptr); \
} while (0)

static DEFINE_MUTEX(g_lock);
static LIST_HEAD(g_camera_list);

static void vendor_register_sensor_node(struct camera_elem_t *dev)
{
	struct camera_elem_t *camera = NULL;
	camera = (struct camera_elem_t *)kzalloc(sizeof(struct camera_elem_t), GFP_KERNEL);
	if (!camera) {
		debug_err("sensor_adapter kzalloc error!");
		return;
	}
	camera->slot_id = dev->slot_id;
	camera->s_ctrl = dev->s_ctrl;
	camera->vcm_id = dev->vcm_id;
	camera->ois_id = dev->ois_id;
	camera->eeprom_id = dev->eeprom_id;
	camera->phy_id = dev->phy_id;
	list_add(&camera->node, &g_camera_list);
}

static int vendor_register_vcm_node(int vcm_id, struct cam_actuator_ctrl_t *a_ctrl)
{
	struct camera_elem_t *camera = NULL;

	mutex_lock(&g_lock);
	list_for_each_entry(camera, &g_camera_list, node) {
		if (camera->vcm_id == vcm_id) {
			debug_dbg("enter! slot_id %d", camera->slot_id);
			camera->a_ctrl = a_ctrl;
			mutex_unlock(&g_lock);
			return camera->slot_id;
		}
	}
	mutex_unlock(&g_lock);
	return -1;
}

static int vendor_register_ois_node(int ois_id, struct cam_ois_ctrl_t *o_ctrl)
{
	struct camera_elem_t *camera = NULL;
	mutex_lock(&g_lock);
	list_for_each_entry(camera, &g_camera_list, node) {
		if (camera->ois_id == ois_id) {
			debug_dbg("enter! slot_id %d", camera->slot_id);
			camera->o_ctrl = o_ctrl;
			mutex_unlock(&g_lock);
			return camera->slot_id;
		}
	}
	mutex_unlock(&g_lock);
	return -1;
}

struct cam_actuator_ctrl_t* vendor_get_vcm_ctrl(int slot_id)
{
	struct camera_elem_t *camera = NULL;
	mutex_lock(&g_lock);
	list_for_each_entry(camera, &g_camera_list, node) {
		if (camera->slot_id == slot_id) {
			mutex_unlock(&g_lock);
			debug_dbg("get dev info");
			return camera->a_ctrl;
		}
	}
	mutex_unlock(&g_lock);
	return NULL;
}

struct cam_sensor_ctrl_t* vendor_get_sensor_ctrl(int slot_id, int64_t device_type)
{
	struct camera_elem_t *camera = NULL;
	int camera_slot_id;
	mutex_lock(&g_lock);
	list_for_each_entry(camera, &g_camera_list, node) {
		switch (device_type) {
		case CAM_SENSOR:
			camera_slot_id = camera->slot_id;
			break;
		case CAM_ACTUATOR:
			camera_slot_id = camera->vcm_id;
			break;
		case CAM_EEPROM:
			camera_slot_id = camera->eeprom_id;
			break;
		case CAM_OIS:
			camera_slot_id = camera->ois_id;
			break;
		case CAM_CSIPHY:
			camera_slot_id = camera->phy_id;
			break;
		default:
			debug_err("Not support device Type: %d", device_type);
			mutex_unlock(&g_lock);
			return NULL;
		}
		if (slot_id == camera_slot_id) {
			debug_dbg("get dev info");
			mutex_unlock(&g_lock);
			return camera->s_ctrl;
		}
	}
	mutex_unlock(&g_lock);
	return NULL;
}

struct cam_ois_ctrl_t* vendor_get_ois_ctrl(int slot_id)
{
	struct camera_elem_t *camera = NULL;
	mutex_lock(&g_lock);
	list_for_each_entry(camera, &g_camera_list, node) {
		if (camera->slot_id == slot_id) {
			debug_dbg("get dev info");
			mutex_unlock(&g_lock);
			return camera->o_ctrl;
		}
	}
	mutex_unlock(&g_lock);
	return NULL;
}

static void vendor_ctrl_dev_update(struct cam_sensor_ctrl_t *s_ctrl, struct dev_msg_t* ctrl_dev)
{
	if (ctrl_dev->type == SENSOR)
		vendor_sensor_debug_update(s_ctrl, ctrl_dev);
	else if (ctrl_dev->type == OIS)
		vendor_ois_debug_update(ctrl_dev);
	else if (ctrl_dev->type == VCM)
		vendor_actuator_debug_update(ctrl_dev);
	else
		CAM_ERR(CAM_OIS, "not found dev type");
}

void vendor_actuator_ctrl_register(struct cam_actuator_ctrl_t *a_ctrl)
{
	struct dev_msg_t ctrl_dev;

	/* register vcm slotid */
	ctrl_dev.type = VCM;
	ctrl_dev.dev_id = vendor_register_vcm_node(a_ctrl->soc_info.index, a_ctrl);
	if (ctrl_dev.dev_id == -1) {
		CAM_ERR(CAM_SENSOR, "register_actuator_node fail");
	}
	vendor_ctrl_dev_update(NULL, &ctrl_dev);
}

void vendor_ois_ctrl_register(struct cam_ois_ctrl_t *o_ctrl)
{
	struct dev_msg_t ctrl_dev;

	ctrl_dev.type = OIS;
	ctrl_dev.dev_id = vendor_register_ois_node(o_ctrl->soc_info.index, o_ctrl);
	if (ctrl_dev.dev_id == -1) {
		CAM_ERR(CAM_SENSOR, "register_ois_node fail");
	}
	vendor_ctrl_dev_update(NULL, &ctrl_dev);
}

void vendor_sensor_ctrl_register(struct cam_sensor_ctrl_t *s_ctrl)
{
	struct dev_msg_t ctrl_dev;
	struct camera_elem_t sensor_adapter;

	ctrl_dev.type = SENSOR;
	sensor_adapter.slot_id = s_ctrl->soc_info.index;
	sensor_adapter.vcm_id = s_ctrl->sensordata->subdev_id[SUB_MODULE_ACTUATOR];
	sensor_adapter.ois_id = s_ctrl->sensordata->subdev_id[SUB_MODULE_OIS];
	sensor_adapter.eeprom_id = s_ctrl->sensordata->subdev_id[SUB_MODULE_EEPROM];
	sensor_adapter.s_ctrl = s_ctrl;
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.sensor_fuseid_reg_settings.list_head));
	vendor_register_sensor_node(&sensor_adapter);
	vendor_ctrl_dev_update(s_ctrl, &ctrl_dev);
}

int cam_crtl_init(void)
{
	return 0;
}

void cam_crtl_exit(void)
{
	struct camera_elem_t *camera = NULL;
	struct list_head *pos = NULL;
	struct list_head *q = NULL;
	mutex_destroy(&g_lock);
	list_for_each_safe(pos, q, &g_camera_list) {
		camera = list_entry(pos, struct camera_elem_t, node);
		list_del(pos);
		kfree(camera);
	}
}
