/*
 * SPDX-FileCopyrightText: 2020-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "tusb.h"

/* Be carefull if you want to change the index. It must be bigger then the size of string_desc_arr
   For now 7 would be also ok. But lets reserve some fields fot the future additions
   Also it must be match with the value in the openocd/esp_usb_bridge.cfg file
   Currently it is defined as < esp_usb_jtag_caps_descriptor 0x030A >
*/
#define EUB_DEBUG_PROBE_STR_DESC_INX    0x0A

void eub_debug_probe_init(void);
int eub_debug_probe_get_proto_caps(void *dest);
void eub_debug_probe_task_suspend(void);
void eub_debug_probe_task_resume(void);
bool eub_debug_probe_target_is_esp32(void);
bool eub_debug_probe_control_handler(const uint8_t rhport, const uint8_t stage, tusb_control_request_t const *request);
