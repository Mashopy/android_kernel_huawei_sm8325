/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor_ife_csid_core function
 *
 */
#include <chipset_common/camera/cam_hiview.h>
#include "vendor_ife_csid_core.h"
#include "cam_debug_util.h"
#include "vendor_soc_util.h"

const char* g_driver_error_scene_info[CSIPHY_ERROR_SCENE_MAX] = {
	/* csi phy error scene */
	"(csiphy lane 0 over flow)",
	"(csiphy lane 1 over flow)",
	"(csiphy lane 2 over flow)",
	"(csiphy lane 3 over flow)",
	"(csiphy tg over flow)",
	"(csiphy cphy eot reception)",
	"(csiphy cphy sot reception)",
	"(csiphy cphy ph crc)",
	"(csiphy error crc)",
	"(csiphy error ec)",
	"(csiphy mmapped vc dt)",
	"(csiphy unmapped vc dt)",
	"(csiphy error stream underflow)",
	"(csiphy unbounded frame)",
};

static csiphy_error_scene g_error_scene = CSIPHY_ERROR_SCENE_INVALID;
static int g_error_times = 0;

void vendor_ife_hiview_handle()
{
	struct camera_dmd_info cam_info;

	if (g_error_scene > CSIPHY_ERROR_SCENE_INVALID &&
		g_error_scene <= CSIPHY_LANE_3_OVERFLOW) {

		g_error_times++;
		if (g_error_times < MIN_CSIPHY_ERROR_TIMES) {
			g_error_scene = CSIPHY_ERROR_SCENE_INVALID;
			return;
		}

		if (!camkit_hiview_init(&cam_info)) {
			cam_info.error_type = CSI_PHY_ERR;
			if (strncpy_s(cam_info.extra_info.error_scene,
				sizeof(cam_info.extra_info.error_scene),
				g_driver_error_scene_info[g_error_scene],
				sizeof(cam_info.extra_info.error_scene) - 1)) {
				CAM_ERR(CAM_ISP, "strncpy_s fail");
				return;
			}
			camkit_hiview_report(&cam_info);
			g_error_scene = CSIPHY_ERROR_SCENE_INVALID;
			g_error_times = 0;
		}
	}
}

void vendor_ife_error_notify(csiphy_error_scene errno)
{
	g_error_scene = errno;
}

bool vendor_ife_error_found(struct cam_ife_csid_hw *csid_hw)
{
	/* this is only workaround only for bali */
	if (vendor_skip_pre_stream_overflow(csid_hw->csi2_rx_cfg.phy_sel) == false)
		return false;
	return true;
}