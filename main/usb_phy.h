/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the internal USB OTG PHY used by the USB device stack
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t eub_usb_phy_init(void);

/**
 * @brief Tear down the USB OTG PHY before forcing download mode
 *
 * Disconnects from the bus and deletes the OTG PHY created by
 * eub_usb_phy_init(). On ESP32-S3, also reconfigures the USB pads so the
 * built-in USB Serial/JTAG controller can take over and the host sees a bus
 * disconnect. On ESP32-S2 there is no USB Serial/JTAG block; only disconnect
 * and PHY delete are performed.
 */
void eub_usb_phy_deinit(void);

#ifdef __cplusplus
}
#endif
