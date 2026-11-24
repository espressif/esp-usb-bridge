/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "led_io.h"
#ifdef CONFIG_BRIDGE_USE_RGB_LED
#include "led_strip.h"
#else
#include "driver/gpio.h"
#endif

static const char *TAG = "led_io";

#ifdef CONFIG_BRIDGE_USE_RGB_LED

#define RGB_LED_REFRESH_MS  20

static led_strip_handle_t s_strip;
static TaskHandle_t s_task;
static atomic_uint s_state;     // LEDs currently on
static atomic_uint s_pending;   // LEDs turned on since the last refresh

static void rgb_led_show(unsigned state)
{
    if (state == 0) {
        led_strip_clear(s_strip);
        return;
    }

    // Red = JTAG, Green = LED1 (target -> host), Blue = LED2 (host -> target)
    const uint32_t level = CONFIG_BRIDGE_BRIGHTNESS_RGB_LED;
    led_strip_set_pixel(s_strip, 0,
                        (state & (1u << LED_IO_JTAG)) ? level : 0,
                        (state & (1u << LED_IO_TX)) ? level : 0,
                        (state & (1u << LED_IO_RX)) ? level : 0);
    led_strip_refresh(s_strip);
}

// Only this task touches the strip, so activity callbacks never block on RMT
static void rgb_led_task(void *arg)
{
    unsigned shown = 0;

    while (1) {
        ulTaskNotifyTake(pdTRUE, shown ? pdMS_TO_TICKS(RGB_LED_REFRESH_MS) : portMAX_DELAY);

        // Activity pulses shorter than the refresh period stay visible for one period
        const unsigned state = atomic_load(&s_state) | atomic_exchange(&s_pending, 0);
        if (state != shown) {
            rgb_led_show(state);
            shown = state;
        }
        vTaskDelay(pdMS_TO_TICKS(RGB_LED_REFRESH_MS));
    }
}

void led_io_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = CONFIG_BRIDGE_GPIO_RGB_LED,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10 MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_strip));
    ESP_ERROR_CHECK(led_strip_clear(s_strip));

    if (xTaskCreate(rgb_led_task, "rgb_led", 2048, NULL, 1, &s_task) != pdPASS) {
        ESP_LOGE(TAG, "Cannot create RGB LED task");
        abort();
    }

    ESP_LOGI(TAG, "RGB LED init done on GPIO%d", CONFIG_BRIDGE_GPIO_RGB_LED);
}

void led_io_set(led_io_t led, bool on)
{
    const unsigned mask = 1u << led;

    if (on) {
        atomic_fetch_or(&s_pending, mask);
        atomic_fetch_or(&s_state, mask);
    } else {
        atomic_fetch_and(&s_state, ~mask);
    }
    if (s_task) {
        xTaskNotifyGive(s_task);
    }
}

void led_io_set_all(bool tx, bool rx, bool jtag)
{
    const unsigned mask = (tx ? 1u << LED_IO_TX : 0) | (rx ? 1u << LED_IO_RX : 0) | (jtag ? 1u << LED_IO_JTAG : 0);

    atomic_store(&s_pending, mask);
    atomic_store(&s_state, mask);
    if (s_task) {
        xTaskNotifyGive(s_task);
    }
}

#else

static const struct {
    int gpio;
    int active_level;
} s_leds[] = {
    [LED_IO_TX]   = { CONFIG_BRIDGE_GPIO_LED1, CONFIG_BRIDGE_GPIO_LED1_ACTIVE },
    [LED_IO_RX]   = { CONFIG_BRIDGE_GPIO_LED2, CONFIG_BRIDGE_GPIO_LED2_ACTIVE },
    [LED_IO_JTAG] = { CONFIG_BRIDGE_GPIO_LED3, CONFIG_BRIDGE_GPIO_LED3_ACTIVE },
};

void led_io_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << CONFIG_BRIDGE_GPIO_LED1) | (1ULL << CONFIG_BRIDGE_GPIO_LED2) |
        (1ULL << CONFIG_BRIDGE_GPIO_LED3),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    led_io_set_all(false, false, false);

    ESP_LOGI(TAG, "LED GPIO init done");
}

void led_io_set(led_io_t led, bool on)
{
    gpio_set_level(s_leds[led].gpio, on ? s_leds[led].active_level : !s_leds[led].active_level);
}

void led_io_set_all(bool tx, bool rx, bool jtag)
{
    led_io_set(LED_IO_TX, tx);
    led_io_set(LED_IO_RX, rx);
    led_io_set(LED_IO_JTAG, jtag);
}

#endif
