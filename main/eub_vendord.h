/*
 * SPDX-FileCopyrightText: 2020-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#define EUB_VENDORD_EPSIZE          64 /* full speed */
#define EUB_VENDORD_IFACE_SUBCLASS  0xFF
#define EUB_VENDORD_IFACE_PROTOCOL  0x01
#define EUB_VENDORD_IFACE_STR_IDX   5
#define EUB_VENDORD_USB_TASK_PRI    4

#define VENDOR_REQUEST_MICROSOFT 0x20  // Can be any value between 0x20 and 0xFF
#define MS_OS_20_DESC_LEN  0xB2

void eub_vendord_start(void);
int eub_vendord_send_item(const void *buf, const size_t size);
void *eub_vendord_recv_acquire_item(size_t *item_size);
void eub_vendord_free_item(void *item);
