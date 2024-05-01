/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor_soc_util function
 *
 */
#ifndef _VENDOR_SOC_UTIL_H_
#define _VENDOR_SOC_UTIL_H_

#include "cam_soc_util.h"

void vendor_set_cam_stream_state(bool value);
bool vendor_skip_pre_stream_overflow(uint32_t csiphy_index);
const char* vendor_get_product_name(void);
void vendor_soc_util_get_dt_properties(struct cam_hw_soc_info *soc_info);

#endif /* _VENDOR_SOC_UTIL_H_ */
