/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor_soc_util function
 *
 */
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include "cam_debug_util.h"
#include "vendor_soc_util.h"

static const char* g_product_name = NULL;

/*
 *---------------------------------------------------
 * this is workaround bugfix only for bali start
 *---------------------------------------------------
 */
static bool g_is_sensor_stream_on = false;

void vendor_set_cam_stream_state(bool value)
{
	g_is_sensor_stream_on = value;
}

static bool vendor_get_cam_stream_state()
{
	return g_is_sensor_stream_on;
}

bool vendor_skip_pre_stream_overflow(uint32_t csiphy_index)
{
	if (!vendor_get_cam_stream_state()) {
		/* bali sub csiphy_index = 2 */
		if ((!strcmp(vendor_get_product_name(), "bali")) &&
			(csiphy_index == 2))
			return true;
	}
	return false;
}
/*
 *---------------------------------------------------
 * this is workaround bugfix only for bali end
 *---------------------------------------------------
 */

const char* vendor_get_product_name()
{
	struct device_node* ofdts_node = NULL;
	int ret = 0;

	if (!g_product_name)
		g_product_name = "";

	ofdts_node = of_find_node_by_name(NULL, "product_name_camera");
	if (!ofdts_node) {
		CAM_ERR(CAM_UTIL, "ofdts_node is null");
		return "";
	}
	ret = of_property_read_string(ofdts_node, "product-name", &g_product_name);
	if (ret < 0) {
		CAM_ERR(CAM_UTIL, "product-name is not found");
		return "";
	} else {
		CAM_DBG(CAM_UTIL, "g_product_name(%s)", g_product_name);
	}

	return g_product_name;
}

void vendor_soc_util_get_dt_properties(struct cam_hw_soc_info *soc_info)
{
	struct device_node *of_node = NULL;
	int rc = 0;

	if (!soc_info || !soc_info->dev)
		return;

	of_node = soc_info->dev->of_node;

	rc = of_property_read_u32(of_node, "sensor-mixed-pin",
		&soc_info->sensor_mixed_pin);
	if (rc) {
		CAM_DBG(CAM_UTIL, "No sensor-mixed-pin preset for: %s",
			soc_info->dev_name);
		rc = 0;
	}

	soc_info->btb_check_enable = of_property_read_bool(of_node,
		"btb_check_support");
	if (soc_info->btb_check_enable)
		CAM_INFO(CAM_UTIL, "device %s support btb check",
			soc_info->dev_name);

	soc_info->btb_check_skip_reset = of_property_read_bool(of_node,
		"btb_check_skip_reset");
	if (soc_info->btb_check_skip_reset)
		CAM_INFO(CAM_UTIL, "device %s don't do btb check reset",
			soc_info->dev_name);

	soc_info->boost5v_enable = of_property_read_bool(of_node,
		"5v_boost_support");
	if (soc_info->boost5v_enable)
		CAM_INFO(CAM_UTIL, "device %s power up needs 5v boost regulator",
		soc_info->dev_name);
}