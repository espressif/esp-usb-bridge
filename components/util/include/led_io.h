/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>

typedef enum {
    LED_IO_TX = 0,  // LED1
    LED_IO_RX,      // LED2
    LED_IO_JTAG,    // LED3
} led_io_t;

/**
 * @brief Initialize the status LEDs (discrete GPIO LEDs or the WS2812 RGB LED)
 *
 * Must be called before any other led_io function.
 */
void led_io_init(void);

/**
 * @brief Turn one status LED on or off
 *
 * Safe to call from multiple tasks.
 */
void led_io_set(led_io_t led, bool on);

/**
 * @brief Set all status LEDs at once
 */
void led_io_set_all(bool tx, bool rx, bool jtag);
