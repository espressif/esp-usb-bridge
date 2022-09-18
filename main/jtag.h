// Copyright 2020-2021 Espressif Systems (Shanghai) CO LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <stdint.h>

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
#include "esp_chip_info.h"
#else
#include "esp_system.h"
#endif

/* Be carefull if you want to change the index. It must be bigger then the size of string_desc_arr
   For now 7 would be also ok. But lets reserve some fields fot the future additions
   Also it must be match with the value in the openocd/esp_usb_bridge.cfg file
   Currently it is defined as < esp_usb_jtag_caps_descriptor 0x030A >
*/
#define JTAG_STR_DESC_INX   0x0A

int jtag_get_proto_caps(uint16_t *dest);
int jtag_get_target_model(void);
void jtag_init(void);
void jtag_task_suspend(void);
void jtag_task_resume(void);
