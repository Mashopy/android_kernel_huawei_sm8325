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

struct camerafs_class {
	struct class *classptr;
	struct device *p_device;
};

struct camerafs_ois_class {
	struct class *classptr;
	struct device *p_device;
};

static struct camerafs_ois_class g_camerafs_ois;

static struct camerafs_class g_camerafs;

static dev_t g_devnum;
static dev_t g_osi_devnum;

#define CAMERAFS_NODE "node"
#define CAMERAFS_OIS_NODE "ois"
#define CAMERAFS_ID_MAX 3
#define CAMERAFS_ID_MIN 0

wait_queue_head_t g_ois_que;
#define OIS_TEST_TIMEOUT ((HZ) * 8)
static int g_cross_width = -1;
static int g_cross_height = -1;

spinlock_t pix_lock = __SPIN_LOCK_UNLOCKED("camerafs");

int register_camerafs_ois_attr(struct device_attribute *attr);
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

int register_camerafs_attr(struct device_attribute *attr);

int camerafs_module_init(void)
{
	int ret;

	g_camerafs.classptr = NULL;
	g_camerafs.p_device = NULL;
	spin_lock_init(&pix_lock);
	ret = alloc_chrdev_region(&g_osi_devnum, 0, 1, CAMERAFS_OIS_NODE);
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
		g_osi_devnum, NULL, "%s", CAMERAFS_OIS_NODE);

	if (IS_ERR(g_camerafs.p_device)) {
		CAM_ERR(CAM_CUSTOM, "class_device_create failed %s", CAMERAFS_NODE);
		ret = PTR_ERR(g_camerafs.p_device);
		return -1;
	}

	register_camerafs_ois_attr(&g_hw_ois_pixel);

	init_waitqueue_head(&g_ois_que);
	CAM_INFO(CAM_CUSTOM, "%s end", __func__);
	return 0;
}
EXPORT_SYMBOL(camerafs_module_init);

int register_camerafs_attr(struct device_attribute *attr)
{
	int ret;

	ret = device_create_file(g_camerafs.p_device, attr);
	if (ret < 0) {
		CAM_ERR(CAM_CUSTOM, "camera fs creat dev attr[%s] fail", attr->attr.name);
		return -1;
	}
	CAM_INFO(CAM_CUSTOM, "camera fs creat dev attr[%s] OK", attr->attr.name);
	return 0;
}
EXPORT_SYMBOL(register_camerafs_attr);

int register_camerafs_ois_attr(struct device_attribute *attr)
{
	int ret;

	ret = device_create_file(g_camerafs_ois.p_device, attr);
	if (ret < 0) {
		CAM_ERR(CAM_CUSTOM, "camera oiscreat dev attr[%s] fail", attr->attr.name);
		return -1;
	}
	CAM_INFO(CAM_CUSTOM, "camera ois creat dev attr[%s] OK", attr->attr.name);
	return 0;
}

void camerafs_module_deinit(void)
{
	device_destroy(g_camerafs.classptr, g_devnum);
	device_destroy(g_camerafs_ois.classptr, g_osi_devnum);
	class_destroy(g_camerafs.classptr);
	unregister_chrdev_region(g_devnum, 1);
	unregister_chrdev_region(g_osi_devnum, 1);
}
EXPORT_SYMBOL(camerafs_module_deinit);

MODULE_DESCRIPTION("Camera fs virtul device");
MODULE_LICENSE("GPL");