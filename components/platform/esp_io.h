/*
 * SPDX-FileCopyrightText: 2020-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <driver/gpio.h>
#include <driver/dedic_gpio.h>
#include <hal/dedic_gpio_cpu_ll.h>
#include "hal/gpio_ll.h"

#include "compiler.h"

/* jtag */
#define GPIO_TDI        CONFIG_BRIDGE_GPIO_TDI
#define GPIO_TDO        CONFIG_BRIDGE_GPIO_TDO
#define GPIO_TCK        CONFIG_BRIDGE_GPIO_TCK
#define GPIO_TMS        CONFIG_BRIDGE_GPIO_TMS

/* serial */
#define GPIO_BOOT       CONFIG_BRIDGE_GPIO_BOOT
#define GPIO_RST        CONFIG_BRIDGE_GPIO_RST
#define GPIO_RXD        CONFIG_BRIDGE_GPIO_RXD
#define GPIO_TXD        CONFIG_BRIDGE_GPIO_TXD

/* leds */
#define LED_TX          CONFIG_BRIDGE_GPIO_LED1
#define LED_RX          CONFIG_BRIDGE_GPIO_LED2
#define LED_JTAG        CONFIG_BRIDGE_GPIO_LED3

#define LED_TX_ON       CONFIG_BRIDGE_GPIO_LED1_ACTIVE
#define LED_TX_OFF      (!CONFIG_BRIDGE_GPIO_LED1_ACTIVE)

#define LED_RX_ON       CONFIG_BRIDGE_GPIO_LED2_ACTIVE
#define LED_RX_OFF      (!CONFIG_BRIDGE_GPIO_LED2_ACTIVE)

#define LED_JTAG_ON     CONFIG_BRIDGE_GPIO_LED3_ACTIVE
#define LED_JTAG_OFF    (!CONFIG_BRIDGE_GPIO_LED3_ACTIVE)

/* JTAG out pin mask values */
#define GPIO_TCK_MASK           0x01
#define GPIO_TDI_MASK           0x02
#define GPIO_TMS_MASK           0x04
#define GPIO_TMS_TDI_MASK       0x06

/* JTAG input pin mask values */
#define GPIO_TDO_MASK           0x01

extern dedic_gpio_bundle_handle_t s_gpio_out_bundle;
extern dedic_gpio_bundle_handle_t s_gpio_io_bundle;
extern gpio_dev_t *const s_gpio_dev;
extern uint32_t s_gpio_conf;

__STATIC_FORCEINLINE void esp_gpio_jtag_led_off(void)
{
    gpio_ll_set_level(s_gpio_dev, LED_JTAG, LED_JTAG_OFF);
}

__STATIC_FORCEINLINE void esp_gpio_jtag_led_on(void)
{
    gpio_ll_set_level(s_gpio_dev, LED_JTAG, LED_JTAG_ON);
}

__STATIC_FORCEINLINE int esp_gpio_tdo_read(void)
{
    return dedic_gpio_cpu_ll_read_in();
}

__STATIC_FORCEINLINE void esp_gpio_tck_set(void)
{
    dedic_gpio_cpu_ll_write_mask(GPIO_TCK_MASK, GPIO_TCK_MASK);
}

__STATIC_FORCEINLINE void esp_gpio_tck_clr(void)
{
    dedic_gpio_cpu_ll_write_mask(GPIO_TCK_MASK, 0);
}

__STATIC_FORCEINLINE void esp_gpio_write_tmstck(uint8_t tms_tdi_mask)
{
    dedic_gpio_cpu_ll_write_mask(GPIO_TMS_TDI_MASK, tms_tdi_mask);
}

void esp_init_jtag_pins(void);
