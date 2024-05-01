/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor dw9781 OIS driver
 */
#ifndef _VENDOR_OIS_DW9781_H_
#define _VENDOR_OIS_DW9781_H_

#include <linux/cma.h>
#include <cam_ois_dev.h>

/* Project Name */
#define PTAURUS                         0x00
#define PVOGUE                          0x01
#define PLION1                          0x02
#define PLION2                          0x03
#define PLION3                          0x04
#define PCOCOPLUS                       0x08 /* 2019. 09. 02 Add Coco plus */
#define PTIANSHAN                       0x0B /* 2019. 11. 27 Add */
#define POWEN                           0x0C /* 2020. 01. 16 Add */
#define PNOLAN                          0x0D /* 2020. 03. 03 Add */
#define PSHUIXIANHUA                    0x0E /* 2020. 11. 10 Add */
#define PMEIGUI                         0x0F /* 2020. 11. 24 Add */

#define DW9781_SLAVE_ID                (0x54 >> 1)
#define DW9781_LOGIC_SLAVE_ID          (0xE4 >> 1)
#define DW9781_LOGIC_RESET_ADDRESS     0xD002
#define DW9781_LOGIC_RESET_VAL         0x0001

#define DW9781_CHIP_ID_ADDRESS            0x7000

#define RETRY_TIME                        2
#define SECTOR_NUM                        5

#define MTP_START_ADDRESS                 0x8000
#define VERIFY_OK                         0
#define VERIFY_ERROR                      1

#define DW9781_FW_SIZE                    0x2800
#define DW9781_VERSION_OFFSET             5
#define DW9781_CHECKSUM_OFFSET            6
#define DW9781_CHIPID_OFFSET              3
#define DATPKT_SIZE                       32

int32_t dw9781_driver_init(void);
void dw9781_driver_exit(void);

#endif
/* _VENDOR_OIS_DW9781_H_ */
