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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define JTAG_PROTO_CAPS_VER 1   /*Version field. */
typedef struct __attribute__((packed))
{
    uint8_t proto_ver;  /*Protocol version. Expects JTAG_PROTO_CAPS_VER for now. */
    uint8_t length; /*of this plus any following descriptors */
} jtag_proto_caps_hdr_t;

#define JTAG_PROTO_CAPS_SPEED_APB_TYPE 1
typedef struct __attribute__((packed))
{
    uint8_t type;
    uint8_t length;
} jtag_gen_hdr_t;

typedef struct __attribute__((packed))
{
    uint8_t type;   /*Type, always JTAG_PROTO_CAPS_SPEED_APB_TYPE */
    uint8_t length; /*Length of this */
    uint16_t apb_speed_10khz;   /*ABP bus speed, in 10KHz increments. Base speed is half
                     * this. */
    uint16_t div_min;   /*minimum divisor (to base speed), inclusive */
    uint16_t div_max;   /*maximum divisor (to base speed), inclusive */
} jtag_proto_caps_speed_apb_t;

typedef struct {
    jtag_proto_caps_hdr_t proto_hdr;
    jtag_proto_caps_speed_apb_t caps_apb;
} jtag_proto_caps_t;

extern jtag_proto_caps_t jtag_proto_caps;

#define VEND_JTAG_SETDIV    0
#define VEND_JTAG_SETIO     1
#define VEND_JTAG_GETTDO    2

void jtag_task(void *pvParameters);
