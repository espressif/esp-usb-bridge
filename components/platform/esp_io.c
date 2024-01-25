/*
 * SPDX-FileCopyrightText: 2020-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_io.h"

#define GET_IDX(mask)   (__builtin_ctz(mask))

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*(x)))
#endif

dedic_gpio_bundle_handle_t s_gpio_out_bundle;
dedic_gpio_bundle_handle_t s_gpio_io_bundle;
static int s_init_gpio = 0;
gpio_dev_t *const s_gpio_dev = GPIO_LL_GET_HW(GPIO_PORT_0);
uint32_t s_gpio_conf;

static const char *TAG = "esp_io";

#if CONFIG_BRIDGE_DEBUG_IFACE_JTAG

void esp_init_jtag_pins(void)
{
    if (!s_init_gpio) {
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

        dedic_gpio_new_bundle(&out_bundle_config, &s_gpio_out_bundle);
        dedic_gpio_new_bundle(&in_bundle_config, &s_gpio_io_bundle);

        dedic_gpio_cpu_ll_write_mask(GPIO_TMS_MASK, GPIO_TMS_MASK);
        dedic_gpio_cpu_ll_write_mask(GPIO_TCK_MASK, 0);

        s_init_gpio = 1;

        ESP_LOGI(TAG, "JTAG GPIO init done");
    }
}

#else

void esp_init_swd_pins(void)
{
    if (!s_init_gpio) {
        gpio_reset_pin(GPIO_SWDIO);
        gpio_reset_pin(GPIO_SWCLK);
        esp_gpio_mode_in_out_enable(GPIO_SWDIO);
        gpio_set_pull_mode(GPIO_SWDIO, GPIO_PULLUP_ONLY);
        esp_gpio_mode_out_enable(GPIO_SWCLK);

        int bundle_out_gpios[GET_IDX(GPIO_SWD_OUT_MAX_MASK)] = { 0 };
        int bundle_io_gpios[GET_IDX(GPIO_SWDIO_MAX_MASK)] = { 0 };

        esp_gpio_clear(LED_JTAG_ON);
        esp_gpio_mode_out_enable(LED_JTAG_ON);
        bundle_out_gpios[GET_IDX(GPIO_SWD_BLINK_MASK)] = LED_JTAG_ON;

        bundle_io_gpios[GET_IDX(GPIO_SWDIO_MASK)] = GPIO_SWDIO;
        dedic_gpio_bundle_config_t io_bundle_config = {
            .gpio_array = bundle_io_gpios,
            .array_size = ARRAY_SIZE(bundle_io_gpios),
            .flags = {
                .out_en = 1,
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

        dedic_gpio_new_bundle(&out_bundle_config, &s_gpio_out_bundle);
        dedic_gpio_new_bundle(&io_bundle_config, &s_gpio_io_bundle);
        s_gpio_conf = REG_READ(GPIO_FUNC0_OUT_SEL_CFG_REG + (GPIO_SWDIO * 4));

        s_init_gpio = 1;

        ESP_LOGI(TAG, "SWD GPIO init done");
    }
}

void esp_reset_dap_pins(void)
{
    gpio_reset_pin(GPIO_SWDIO); // GPIO_TMS
    gpio_reset_pin(GPIO_SWCLK); // GPIO_TCK
    gpio_reset_pin(GPIO_TDI);
    gpio_reset_pin(GPIO_TDO);

    dedic_gpio_del_bundle(s_gpio_io_bundle);
    dedic_gpio_del_bundle(s_gpio_out_bundle);

    s_init_gpio = 0;
}

#endif
