/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <esp_system.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "util.h"
#include "sdkconfig.h"
#include "esp_io.h"

void __attribute__((noreturn)) eub_abort(void)
{
    const int led_patterns[][3] = {
        {LED_TX_ON,  LED_RX_ON,  LED_JTAG_ON},
        {LED_TX_ON,  LED_RX_OFF, LED_JTAG_ON},
        {LED_TX_OFF, LED_RX_ON,  LED_JTAG_OFF},
        {LED_TX_ON,  LED_RX_OFF, LED_JTAG_ON},
        {LED_TX_OFF, LED_RX_ON,  LED_JTAG_OFF},
        {LED_TX_ON,  LED_RX_OFF, LED_JTAG_ON},
        {LED_TX_ON,  LED_RX_ON,  LED_JTAG_ON},
    };

    for (int i = 0; i < sizeof(led_patterns) / sizeof(led_patterns[0]); ++i) {
        gpio_set_level(LED_TX, led_patterns[i][0]);
        gpio_set_level(LED_RX, led_patterns[i][1]);
        gpio_set_level(LED_JTAG, led_patterns[i][2]);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    abort();
}
