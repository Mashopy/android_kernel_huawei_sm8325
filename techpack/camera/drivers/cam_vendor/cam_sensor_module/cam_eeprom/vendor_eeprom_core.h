/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor eeprom core driver
 */
#ifndef _VENDOR_EEPROM_CORE_H_
#define _VENDOR_EEPROM_CORE_H_

#include "cam_eeprom_core.h"

bool vendor_eeprom_read_need_retry(struct cam_eeprom_ctrl_t *e_ctrl);

#endif/* _VENDOR_EEPROM_CORE_H_ */
