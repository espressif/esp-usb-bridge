/*
 * SPDX-FileCopyrightText: 2020-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "debug_gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "debug_probe.h"

#define GET_IDX(mask)   (__builtin_ctz(mask))

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*(x)))
#endif

dedic_gpio_bundle_handle_t dedic_gpio_out_bundle;
dedic_gpio_bundle_handle_t dedic_gpio_io_bundle;
gpio_dev_t *const dedic_gpio_dev = GPIO_LL_GET_HW(GPIO_PORT_0);
uint32_t dedic_gpio_conf;

static int s_debug_gpio_init = 0;

static const char *TAG = "debug_gpio";

static debug_activity_notify_cb_t s_activity_callback = NULL;

void debug_probe_register_activity_callback(debug_activity_notify_cb_t callback)
{
    s_activity_callback = callback;
}

void debug_probe_notify_activity(bool active)
{
    if (s_activity_callback) {
        s_activity_callback(active);
    }
}

#if CONFIG_DEBUG_PROBE_IFACE_JTAG

void debug_probe_init_jtag_pins(void)
{
    if (!s_debug_gpio_init) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = BIT64(GPIO_TDI) | BIT64(GPIO_TCK) | BIT64(GPIO_TMS),
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&io_conf));

        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = BIT64(GPIO_TDO);
        ESP_ERROR_CHECK(gpio_config(&io_conf));

        int bundle_out_gpios[] = { GPIO_TCK, GPIO_TDI, GPIO_TMS };
        int bundle_in_gpios[] = { GPIO_TDO };

        dedic_gpio_bundle_config_t out_bundle_config = {
            .gpio_array = bundle_out_gpios,
            .array_size = ARRAY_SIZE(bundle_out_gpios),
            .flags = {
                .out_en = 1,
            },
        };

        dedic_gpio_bundle_config_t in_bundle_config = {
            .gpio_array = bundle_in_gpios,
            .array_size = ARRAY_SIZE(bundle_in_gpios),
            .flags = {
                .in_en = 1,
            },
        };

        dedic_gpio_new_bundle(&out_bundle_config, &dedic_gpio_out_bundle);
        dedic_gpio_new_bundle(&in_bundle_config, &dedic_gpio_io_bundle);

        dedic_gpio_cpu_ll_write_mask(GPIO_TMS_MASK, GPIO_TMS_MASK);
        dedic_gpio_cpu_ll_write_mask(GPIO_TCK_MASK, 0);

        s_debug_gpio_init = 1;

        ESP_LOGI(TAG, "JTAG GPIO init done");
    }
}

#else

void debug_probe_init_swd_pins(void)
{
    if (!s_debug_gpio_init) {
        gpio_reset_pin(GPIO_SWDIO);
        gpio_reset_pin(GPIO_SWCLK);
        debug_probe_mode_in_out_enable(GPIO_SWDIO);
        gpio_set_pull_mode(GPIO_SWDIO, GPIO_PULLUP_ONLY);
        debug_probe_mode_out_enable(GPIO_SWCLK);

        int bundle_out_gpios[GET_IDX(GPIO_SWD_OUT_MAX_MASK)] = { 0 };
        int bundle_io_gpios[GET_IDX(GPIO_SWDIO_MAX_MASK)] = { 0 };

        bundle_io_gpios[GET_IDX(GPIO_SWDIO_MASK)] = GPIO_SWDIO;
        dedic_gpio_bundle_config_t io_bundle_config = {
            .gpio_array = bundle_io_gpios,
            .array_size = ARRAY_SIZE(bundle_io_gpios),
            .flags = {
                .out_en = 0, /* We will output without dedic_gpio, or we cannot disable output */
                .in_en = 1,
            },
        };

        bundle_out_gpios[GET_IDX(GPIO_SWCLK_MASK)] = GPIO_SWCLK;
        dedic_gpio_bundle_config_t out_bundle_config = {
            .gpio_array = bundle_out_gpios,
            .array_size = ARRAY_SIZE(bundle_out_gpios),
            .flags = {
                .out_en = 1,
            },
        };

        dedic_gpio_new_bundle(&out_bundle_config, &dedic_gpio_out_bundle);
        dedic_gpio_new_bundle(&io_bundle_config, &dedic_gpio_io_bundle);
        dedic_gpio_conf = REG_READ(GPIO_FUNC0_OUT_SEL_CFG_REG + (GPIO_SWDIO * 4));

        s_debug_gpio_init = 1;

        ESP_LOGI(TAG, "SWD GPIO init done");
    }
}

void debug_probe_reset_pins(void)
{
    gpio_reset_pin(GPIO_SWDIO); // GPIO_TMS
    gpio_reset_pin(GPIO_SWCLK); // GPIO_TCK
    gpio_reset_pin(GPIO_TDI);
    gpio_reset_pin(GPIO_TDO);

    dedic_gpio_del_bundle(dedic_gpio_io_bundle);
    dedic_gpio_del_bundle(dedic_gpio_out_bundle);

    s_debug_gpio_init = 0;
}

#endif
