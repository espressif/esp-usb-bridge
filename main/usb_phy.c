/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "usb_phy.h"
#include "esp_private/usb_phy.h"
#include "esp_rom_sys.h"
#include "sdkconfig.h"
#include "tusb.h"
#if CONFIG_IDF_TARGET_ESP32S3
#include "soc/rtc_cntl_reg.h"
#include "soc/usb_serial_jtag_reg.h"
#include "soc/usb_pins.h"
#include "driver/gpio.h"
#endif  // CONFIG_IDF_TARGET_ESP32S3

static usb_phy_handle_t s_usb_phy;

esp_err_t eub_usb_phy_init(void)
{
    const usb_phy_config_t phy_config = {
        .controller = USB_PHY_CTRL_OTG,
        .target = USB_PHY_TARGET_INT,
        .otg_mode = USB_OTG_MODE_DEVICE,
        .otg_speed = USB_PHY_SPEED_FULL,
        .ext_io_conf = NULL,
        .otg_io_conf = NULL,
    };
    return usb_new_phy(&phy_config, &s_usb_phy);
}

void eub_usb_phy_deinit(void)
{
    /*
     * Detach from the bus before tearing down the PHY. With MSC mounted, the host
     * keeps bulk transfers in flight; deleting the PHY / reconfiguring the pads
     * while those are active can hang the teardown path so download-mode entry
     * never completes until the volume is ejected.
     */
    tud_disconnect();
    esp_rom_delay_us(10 * 1000);

    if (s_usb_phy) {
        usb_del_phy(s_usb_phy);
        s_usb_phy = NULL;
    }

#if CONFIG_IDF_TARGET_ESP32S3
    // Switch to hardware CDC+JTAG:
    // Clear SW override and SW PHY sel bits so hardware (eFuse default) takes over,
    // which maps internal PHY to USB Serial/JTAG
    CLEAR_PERI_REG_MASK(RTC_CNTL_USB_CONF_REG,
                        (RTC_CNTL_SW_HW_USB_PHY_SEL | RTC_CNTL_SW_USB_PHY_SEL | RTC_CNTL_USB_PAD_ENABLE));

    // Do not use external PHY for Serial/JTAG
    CLEAR_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_PHY_SEL);

    // Release GPIO pins from USB Serial/JTAG pad
    CLEAR_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_USB_PAD_ENABLE);

    // Force host to see a USB disconnect (BUS_RESET) by pulling D+/D- LOW
    gpio_set_direction(USBPHY_DM_NUM, GPIO_MODE_OUTPUT_OD);
    gpio_set_direction(USBPHY_DP_NUM, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(USBPHY_DM_NUM, 0);
    gpio_set_level(USBPHY_DP_NUM, 0);

    SET_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_USB_PAD_ENABLE);
#endif  // CONFIG_IDF_TARGET_ESP32S3
}
