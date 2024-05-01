/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: lc898129 OIS driver
 */
#ifndef _OIS_LC898129_H_
#define _OIS_LC898129_H_

#include <linux/cma.h>
#include <cam_ois_dev.h>

#define FW_UPDATA_NAME                  "lc898129_updatacode.prog"

#define UP_DATA_CODE_SIZE               0x0000014c
#define UP_DATA_CODE_CHECKSUM           0x000071d6ac6f9fbf

#define OIS129_READ_STATUS_INI          0x01000000
#define OIS129_CNT050MS                 676
#define OIS129_CMD_READ_STATUS          0xF100
/* for oisdata protocol2.0 */
#define EIS_VERSION_MASK                0xff
#define EIS_VERSION_MASK_SHIFT          16
/* 1[Block] = 4[KByte] (14*4 = 56[KByte]) */
#define FLASH_BLOCKS                    14
/* Reserved for customer data blocks */
#define USER_RESERVE                    0
#define ERASE_BLOCKS                    (FLASH_BLOCKS - USER_RESERVE)
#define EIS_DATA_VERSION                0x8004
/* checksum size 4bytes + checksum 4bytes + fw version 4bytes */
#define FW_OFFSET                       12

int32_t lc898129_driver_init(void);
void lc898129_driver_exit(void);

#endif
/* _OIS_LC898129_ */