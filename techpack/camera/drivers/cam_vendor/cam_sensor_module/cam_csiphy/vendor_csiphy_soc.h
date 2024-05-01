/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor csiphy soc driver
 */
#ifndef _VENDOR_CSIPHY_SOC_H_
#define _VENDOR_CSIPHY_SOC_H_

#include "cam_csiphy_dev.h"

#define NAME_LEN 32
#define PHY_NUM  5

#define BALI_RESET_MAIN_CSIPHY_INDEX_1 1
#define BALI_RESET_MAIN_CSIPHY_INDEX_2 4

#define PALAU_RESET_MAIN_CSIPHY_INDEX_1 0

#define JADE_RESET_MAIN_CSIPHY_INDEX_1 1

#define ABR_RESET_MAIN_CSIPHY_INDEX_1 1

#define PRODUCT_JADE_NAME         "jade"
#define PRODUCT_PALAU_NAME        "palau"
#define PRODUCT_BALI_NAME         "bali"
#define PRODUCT_ABR_NAME          "amber"

struct vendor_csiphy_reg_map {
	char product_name[NAME_LEN];
	void (*config_csiphy_reg)(struct csiphy_device *csiphy_dev,
		bool private_setting_support, uint32_t index);
};

struct vendor_camera_csiphy {
	char product_name[NAME_LEN];
	uint32_t index;
};

static const struct vendor_camera_csiphy g_cam_csiphy_index_tb1[] = {
	{ PRODUCT_JADE_NAME, JADE_RESET_MAIN_CSIPHY_INDEX_1 },
	{ PRODUCT_BALI_NAME, BALI_RESET_MAIN_CSIPHY_INDEX_1 },
	{ PRODUCT_BALI_NAME, BALI_RESET_MAIN_CSIPHY_INDEX_2 },
	{ PRODUCT_PALAU_NAME, PALAU_RESET_MAIN_CSIPHY_INDEX_1 },
	{ PRODUCT_ABR_NAME, ABR_RESET_MAIN_CSIPHY_INDEX_1 },
};

void vendor_csiphy_reg_config(struct cam_hw_soc_info *soc_info,
	struct csiphy_device *csiphy_dev);

int vendor_csiphy_restart(struct csiphy_device *csiphy_dev,
	void *arg);

#endif /* _VENDOR_CSIPHY_SOC_H_ */
