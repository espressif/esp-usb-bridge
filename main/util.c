// Copyright 2021 Espressif Systems (Shanghai) CO LTD
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

#include <esp_system.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "util.h"
#include "sdkconfig.h"

void __attribute__((noreturn)) eub_abort(void)
{
    const int led_patterns[][3] = {
        {0, 0, 0},
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0},
        {0, 0, 0},
    };

    for (int i = 0; i < sizeof(led_patterns)/sizeof(led_patterns[0]); ++i) {
        gpio_set_level(CONFIG_BRIDGE_GPIO_LED1, led_patterns[i][0]);
        gpio_set_level(CONFIG_BRIDGE_GPIO_LED2, led_patterns[i][1]);
        gpio_set_level(CONFIG_BRIDGE_GPIO_LED3, led_patterns[i][2]);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    abort();
}
