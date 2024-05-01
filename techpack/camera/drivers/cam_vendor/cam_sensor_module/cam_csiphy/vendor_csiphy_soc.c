/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor csiphy soc driver
 */
#include "cam_csiphy_core.h"
#include "vendor_csiphy_soc.h"
#include "vendor_soc_util.h"
#include "include/cam_csiphy_1_2_1_hwreg_rena.h"

#define config_product_csiphy_reg(name) do { \
	csiphy_dev->ctrl_reg->csiphy_2ph_reg = csiphy_2ph_reg_##name; \
	csiphy_dev->ctrl_reg->csiphy_2ph_combo_mode_reg = csiphy_2ph_combo_mode_reg_##name; \
	csiphy_dev->ctrl_reg->csiphy_3ph_reg = csiphy_3ph_reg_##name; \
	csiphy_dev->ctrl_reg->csiphy_2ph_3ph_mode_reg = NULL; \
	csiphy_dev->ctrl_reg->csiphy_irq_reg = csiphy_irq_reg_##name; \
	csiphy_dev->ctrl_reg->csiphy_common_reg = csiphy_common_reg_##name; \
	csiphy_dev->ctrl_reg->csiphy_reset_reg = csiphy_reset_reg_##name; \
	csiphy_dev->ctrl_reg->csiphy_reg = csiphy_reg_##name; \
	csiphy_dev->ctrl_reg->data_rates_settings_table = &data_rate_delta_table_##name##_default; \
} while (0)

#define config_csiphy_data_rate_setting(name, phy_num) \
	csiphy_dev->ctrl_reg->data_rates_settings_table = &data_rate_delta_table_##name##_##phy_num

static void vendor_csiphy_reg_config_rena(struct csiphy_device *csiphy_dev,
	bool private_setting_support, uint32_t index)
{
	config_product_csiphy_reg(rena);
	/*
	 * The following code is added only when the data rate setting
	 * of different PHYs needs to be adjusted.
	 * Add only the PHYs that need to be modified.
	 */
	if (private_setting_support) {
		CAM_INFO(CAM_CSIPHY, "config phy(%u) data rate setting", index);
		if (index == 0)
			config_csiphy_data_rate_setting(rena, phy0);
	}
}

static struct vendor_csiphy_reg_map vendor_reg_list_map[] = {
	{ "rena", vendor_csiphy_reg_config_rena },
};

void vendor_csiphy_reg_config(struct cam_hw_soc_info *soc_info,
	struct csiphy_device *csiphy_dev)
{
	uint32_t i, list_size;
	struct device_node *of_node = NULL;
	bool private_reg_support = false;
	bool private_setting_support = false;

	if (!soc_info || !soc_info->dev || !csiphy_dev) {
		CAM_ERR(CAM_CSIPHY, "param is NULL");
		return;
	}

	of_node = soc_info->dev->of_node;
	private_reg_support = of_property_read_bool(of_node, "private-csiphy-reg-support");
	if (!private_reg_support)
		return;

	private_setting_support =
		of_property_read_bool(of_node, "private-data-rates-setting-support");

	list_size = sizeof(vendor_reg_list_map) / sizeof(vendor_reg_list_map[0]);
	for (i = 0; i < list_size; i++) {
		if (strcmp(vendor_get_product_name(), vendor_reg_list_map[i].product_name))
			continue;

		vendor_reg_list_map[i].config_csiphy_reg(csiphy_dev,
			private_setting_support, soc_info->index);
		CAM_INFO(CAM_CSIPHY, "config %s csiphy(%u) reg",
			vendor_reg_list_map[i].product_name, soc_info->index);
	}
}

static bool vendor_csiphy_need_restart(struct csiphy_device *csiphy_dev,
	void *arg)
{
	struct cam_hw_soc_info *soc_info = NULL;
	uint32_t i;
	if (!csiphy_dev || !arg) {
		CAM_ERR(CAM_CSIPHY, "Invalid input args");
		return false;
	}
	/* product main need restart */
	if (((struct cam_control *)arg)->op_code == CAM_START_DEV) {
		 for (i = 0; (unsigned long)i < ARRAY_SIZE(g_cam_csiphy_index_tb1); i++) {
			 if (!strcmp(vendor_get_product_name(), g_cam_csiphy_index_tb1[i].product_name)) {
				soc_info = &(csiphy_dev->soc_info);
				if (!soc_info) {
					CAM_ERR(CAM_CSIPHY, "Null Soc_info");
					return false;
				}
				if (soc_info->index == g_cam_csiphy_index_tb1[i].index) {
					CAM_DBG(CAM_CSIPHY, "soc_info->index = %u", soc_info->index);
					return true;
				}
			 }
		}
	}

	return false;
}

int vendor_csiphy_restart(struct csiphy_device *csiphy_dev,
	void *arg)
{
	int rc = 0;

	if (vendor_csiphy_need_restart(csiphy_dev, arg)) {
		rc = cam_csiphy_core_cfg(csiphy_dev, arg);
		if (rc) {
			CAM_ERR(CAM_CSIPHY,
				"Failed in configuring the device: %d", rc);
			return rc;
		}

		((struct cam_control *)arg)->op_code = CAM_STOP_DEV;
		rc = cam_csiphy_core_cfg(csiphy_dev, arg);
		if (rc) {
			CAM_ERR(CAM_CSIPHY,
				"Failed in configuring the device: %d", rc);
			return rc;
		}

		((struct cam_control *)arg)->op_code = CAM_START_DEV;
	}
	return rc;
}
