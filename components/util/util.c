/*
 * SPDX-FileCopyrightText: 2021-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "util.h"
#include "led_io.h"

void __attribute__((noreturn)) eub_abort(void)
{
    // { TX, RX, JTAG }
    const bool led_patterns[][3] = {
        {true,  true,  true},
        {true,  false, true},
        {false, true,  false},
        {true,  false, true},
        {false, true,  false},
        {true,  false, true},
        {true,  true,  true},
    };

    for (int i = 0; i < sizeof(led_patterns) / sizeof(led_patterns[0]); ++i) {
        led_io_set_all(led_patterns[i][0], led_patterns[i][1], led_patterns[i][2]);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    abort();
}
