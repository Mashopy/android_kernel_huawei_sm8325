/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: Camera fs virtul device
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include "securec.h"
#include "cam_debug_util.h"
#include "vendor_cam_fs.h"
#include <linux/pm_wakeup.h>

extern int vibrator_register_notifier(struct notifier_block *nb);

struct camerafs_class {
	struct class *classptr;
	struct device *p_device;
};

static struct camerafs_class g_camerafs_ois;
static struct camerafs_class g_camerafs;

static dev_t g_devnum;
static dev_t g_ois_devnum;

struct wakeup_source *g_actuator_protect_wl;

#define CAMERAFS_NODE "node"
#define CAMERAFS_OIS_NODE "ois"
#define CAMERAFS_ID_MAX 3
#define CAMERAFS_ID_MIN 0

wait_queue_head_t g_ois_que;
#define OIS_TEST_TIMEOUT ((HZ) * 8)
static int g_cross_width = -1;
static int g_cross_height = -1;

static struct cameraprotect_info g_protect_info;
static char g_protect_data_updated = false;
static wait_queue_head_t g_read_wait = __WAIT_QUEUE_HEAD_INITIALIZER(g_read_wait);

spinlock_t pix_lock = __SPIN_LOCK_UNLOCKED("camerafs");
spinlock_t protect_lock = __SPIN_LOCK_UNLOCKED("cameraprotect");

/* add for ois mmi test */
static ssize_t hw_ois_test_mmi_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret = -1;

	CAM_INFO(CAM_CUSTOM, "Enter: %s", __func__);
	spin_lock(&pix_lock);
	ret = scnprintf(buf, PAGE_SIZE, "%d,%d\n",
		g_cross_width, g_cross_height);
	g_cross_width = -1;
	g_cross_height = -1;
	spin_unlock(&pix_lock);

	return ret;
}

static ssize_t hw_ois_test_mmi_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int width, height;

	spin_lock(&pix_lock);

	if (sscanf_s(buf, "%d%d", &width, &height) <= 0) {
		CAM_INFO(CAM_CUSTOM, "%s: write data width height error", __func__);
		spin_unlock(&pix_lock);
		return -1;
	}

	g_cross_width = width;
	g_cross_height = height;
	spin_unlock(&pix_lock);
	CAM_INFO(CAM_CUSTOM, "Enter: %s %d, %d", __func__, g_cross_width, g_cross_height);

	return count;
}

static struct device_attribute g_hw_ois_pixel =
	__ATTR(ois_pixel, 0664, hw_ois_test_mmi_show, hw_ois_test_mmi_store);

static ssize_t hw_actuator_protect_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret = -1;

	CAM_INFO(CAM_CUSTOM, "Enter: %s", __func__);

	if (!g_protect_data_updated)
		if (wait_event_interruptible(g_read_wait, g_protect_data_updated) != 0)
			return 0;
	CAM_INFO(CAM_CUSTOM, "Enter: %s, get event %d", __func__, g_protect_data_updated);
	spin_lock(&pix_lock);

	g_protect_data_updated = false;

	if (g_protect_info.actuator_status == 0) {
		__pm_wakeup_event(g_actuator_protect_wl, 2900 + 1000); /* wake up pm core for 2900ms + 1000ms for schedule */
		CAM_INFO(CAM_CUSTOM, "Enter: %s, wake up", __func__);
	}

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", g_protect_info.actuator_status);

	CAM_INFO(CAM_CUSTOM, "Enter: %s, %d, %d", __func__, g_protect_info.actuator_status, g_protect_data_updated);

	spin_unlock(&pix_lock);

	return ret;
}

void hw_actuator_protect_work(struct cameraprotect_info *protect_info)
{
	spin_lock(&pix_lock);
	if (protect_info != NULL && g_protect_info.actuator_status != protect_info->actuator_status) {
		memcpy_s(&g_protect_info, sizeof(g_protect_info), protect_info, sizeof(struct cameraprotect_info));
		g_protect_data_updated = true;
		wake_up_interruptible(&g_read_wait);
		CAM_ERR(CAM_CUSTOM, "Enter: %s, updated %d", __func__, g_protect_data_updated);
	}
	CAM_ERR(CAM_CUSTOM, "Enter: %s %d, %d, protect_info %d, %d, g_protect_data_updated %d", __func__,
		g_protect_info.actuator_status, g_protect_info.actuator_shake_time,
		protect_info->actuator_status, protect_info->actuator_shake_time, g_protect_data_updated);
	spin_unlock(&pix_lock);
}

static ssize_t hw_actuator_protect_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct cameraprotect_info protect_info;
	if (sscanf_s(buf, "%d%d", &protect_info.actuator_status, &protect_info.actuator_shake_time) <= 0) {
		CAM_INFO(CAM_CUSTOM, "%s: write data width height error", __func__);
		return -1;
	}

	hw_actuator_protect_work(&protect_info);

	return count;
}

static struct device_attribute g_hw_actuator_protect =
	__ATTR(actuator_protect, 0664, hw_actuator_protect_show, hw_actuator_protect_store);

int register_camerafs_attr(struct device_attribute *attr, struct camerafs_class *fs_class);

int cam_vib_notifier_cb(struct notifier_block *nb,
	unsigned long time, void *bar)
{
	struct cameraprotect_info protect_info;
	if (time >= 59) { // 59 shake model min duration
		protect_info.actuator_status = true;
		protect_info.actuator_shake_time = time;
	} else {
		protect_info.actuator_status = false;
	}
	hw_actuator_protect_work(&protect_info);
	return 0;
}

static struct notifier_block vib_notify_to_cam = {
	.notifier_call = cam_vib_notifier_cb,
	.priority = -1,
};

int camerafs_module_init(void)
{
	int ret;

	g_camerafs.classptr = NULL;
	g_camerafs.p_device = NULL;
	spin_lock_init(&pix_lock);
	ret = alloc_chrdev_region(&g_ois_devnum, 0, 1, CAMERAFS_OIS_NODE);
	if (ret) {
		CAM_ERR(CAM_CUSTOM, "error %s fail to alloc a dev_t", __func__);
		return -1;
	}

	g_camerafs.classptr = class_create(THIS_MODULE, "camerafs");
	g_camerafs_ois.classptr = g_camerafs.classptr;
	if (IS_ERR(g_camerafs.classptr)) {
		CAM_ERR(CAM_CUSTOM, "class_create failed %d", ret);
		ret = PTR_ERR(g_camerafs.classptr);
		return -1;
	}

	g_camerafs.p_device = device_create(g_camerafs.classptr, NULL, g_devnum,
		NULL, "%s", CAMERAFS_NODE);
	g_camerafs_ois.p_device = device_create(g_camerafs_ois.classptr, NULL,
		g_ois_devnum, NULL, "%s", CAMERAFS_OIS_NODE);

	if (IS_ERR(g_camerafs.p_device)) {
		CAM_ERR(CAM_CUSTOM, "class_device_create failed %s", CAMERAFS_NODE);
		ret = PTR_ERR(g_camerafs.p_device);
		return -1;
	}

	register_camerafs_attr(&g_hw_ois_pixel, &g_camerafs_ois);

	// for actuator protect
	register_camerafs_attr(&g_hw_actuator_protect, &g_camerafs);
	vibrator_register_notifier(&vib_notify_to_cam);
	g_actuator_protect_wl = wakeup_source_register(NULL, "actuator_protect_wakelock");

	init_waitqueue_head(&g_ois_que);
	CAM_INFO(CAM_CUSTOM, "%s end", __func__);
	return 0;
}
EXPORT_SYMBOL(camerafs_module_init);

int register_camerafs_attr(struct device_attribute *attr, struct camerafs_class *fs_class)
{
	int ret;

	ret = device_create_file(fs_class->p_device, attr);
	if (ret < 0) {
		CAM_ERR(CAM_CUSTOM, "camera fs creat dev attr[%s] fail", attr->attr.name);
		return -1;
	}
	CAM_INFO(CAM_CUSTOM, "camera fs creat dev attr[%s] OK", attr->attr.name);
	return 0;
}
EXPORT_SYMBOL(register_camerafs_attr);

void camerafs_module_deinit(void)
{
	device_destroy(g_camerafs.classptr, g_devnum);
	device_destroy(g_camerafs_ois.classptr, g_ois_devnum);
	class_destroy(g_camerafs.classptr);
	unregister_chrdev_region(g_devnum, 1);
	unregister_chrdev_region(g_ois_devnum, 1);

	wakeup_source_unregister(g_actuator_protect_wl);
}
EXPORT_SYMBOL(camerafs_module_deinit);

MODULE_DESCRIPTION("Camera fs virtul device");
MODULE_LICENSE("GPL");