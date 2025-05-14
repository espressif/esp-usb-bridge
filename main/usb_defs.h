// Copyright 2020-2025 Espressif Systems (Shanghai) CO LTD
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

// USB Interface Numbers
enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_VENDOR,
    ITF_NUM_MSC,
    ITF_NUM_TOTAL
};

// USB Endpoint Numbers
#define EPNUM_CDC       2
#define EPNUM_VENDOR    3
#define EPNUM_MSC       4
