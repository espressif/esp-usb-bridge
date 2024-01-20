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

#define EUB_VENDORD_EPSIZE          64 /* full speed */
#define EUB_VENDORD_IFACE_SUBCLASS  0xFF
#define EUB_VENDORD_IFACE_PROTOCOL  0x01
#define EUB_VENDORD_IFACE_STR_IDX   5
#define EUB_VENDORD_USB_TASK_PRI    4

void eub_vendord_start(void);
int eub_vendord_send_acquire_item(const void *buf, const size_t size);
void *eub_vendord_recv_acquire_item(size_t *item_size);
void eub_vendord_free_item(void *item);
