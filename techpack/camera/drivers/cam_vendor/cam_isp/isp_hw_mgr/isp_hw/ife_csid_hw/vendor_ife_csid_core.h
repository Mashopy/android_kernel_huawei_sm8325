/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor_ife_csid_core function
 *
 */
#ifndef _VENDOR_IFE_CSID_HW_H_
#define _VENDOR_IFE_CSID_HW_H_

#include "cam_ife_csid_core.h"

#define VENDOR_IFE_CFG_TIMEOUT                  500

/* Minimum number of CSI PHY ERROR reporting times */
#define MIN_CSIPHY_ERROR_TIMES                  3

typedef enum {
	CSIPHY_ERROR_SCENE_INVALID            =     -1,

	/* csi phy error scene */
	CSIPHY_LANE_0_OVERFLOW,
	CSIPHY_LANE_1_OVERFLOW,
	CSIPHY_LANE_2_OVERFLOW,
	CSIPHY_LANE_3_OVERFLOW,
	CSIPHY_TG_OVERFLOW,
	CSIPHY_CPHY_EOT_RECEPTION,
	CSIPHY_CPHY_SOT_RECEPTION,
	CSIPHY_CPHY_PH_CRC,
	CSIPHY_ERROR_CRC,
	CSIPHY_ERROR_ECC,
	CSIPHY_MMAPPED_VC_DT,
	CSIPHY_UNMAPPED_VC_DT,
	CSIPHY_ERROR_STREAM_UNDERFLOW,
	CSIPHY_UNBOUNDED_FRAME,
	CSIPHY_ERROR_SCENE_MAX,
} csiphy_error_scene;

void vendor_ife_hiview_handle(void);
void vendor_ife_error_notify(csiphy_error_scene errno);
bool vendor_ife_error_found(struct cam_ife_csid_hw *csid_hw);

#endif /* _VENDOR_IFE_CSID_HW_H_ */
