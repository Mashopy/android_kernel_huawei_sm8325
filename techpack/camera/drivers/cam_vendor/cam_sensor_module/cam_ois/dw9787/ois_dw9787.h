/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor dw9787 OIS driver
 */
#ifndef _OIS_DW9787_H_
#define _OIS_DW9787_H_

#include <linux/cma.h>
#include <cam_ois_dev.h>

#define MCS_START_ADDRESS              0x8000
#define IF_START_ADDRESS               0x0000
#define MCS_PKT_SIZE                   32    /* 32 word */
#define IF_PKT_SIZE                    32    /* 32 word */
#define MCS_VALUE                      0x0002
#define IF_VALUE                       0x0001
#define MCS_FW_OFFSET_W                0x4000 /* MCS fw offset with 2bytes */

enum mem_type {
	CODE_SECTION = 0,  /* MCS_SEC */
	DATA_SECTION,      /* IF_SEC */
};

int32_t dw9787_driver_init(void);
void dw9787_driver_exit(void);

#endif
/* _OIS_DW9787_H_ */
