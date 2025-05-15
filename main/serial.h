/*
 * SPDX-FileCopyrightText: 2020-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

void serial_init(void);
void serial_set(const bool enable);
bool serial_set_baudrate(const uint32_t baud);
