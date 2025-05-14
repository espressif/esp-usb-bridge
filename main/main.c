// Copyright 2020-2021 Espressif Systems (Shanghai) CO LTD
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

#include <stdlib.h>
#include "util.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "serial.h"
#include "tusb.h"
#include "msc.h"
#include "hal/usb_phy_types.h"
#include "soc/usb_periph.h"
#include "rom/gpio.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_idf_version.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/usb_phy.h"
#include "eub_vendord.h"
#include "eub_debug_probe.h"
#include "usb_defs.h"

static const char *TAG = "bridge_main";

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_VENDOR_DESC_LEN + TUD_MSC_DESC_LEN)

static const tusb_desc_device_t descriptor_config = {
    .bLength = sizeof(descriptor_config),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0210, // at least 2.1 or 3.x for BOS
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
#ifdef CFG_TUD_ENDPOINT0_SIZE
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
#else  // earlier versions have a typo in the name
    .bMaxPacketSize0 = CFG_TUD_ENDOINT0_SIZE,
#endif
    .idVendor = CONFIG_BRIDGE_USB_VID,
    .idProduct = CONFIG_BRIDGE_USB_PID,
    .bcdDevice = BCDDEVICE,     // defined in CMakeLists.txt
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

/*
    ESP usb builtin jtag subclass and protocol is 0xFF and 0x01 respectively.
    However, Tinyusb default values are 0x00.
    In order to use same protocol without tinyusb customization we are re-defining
    vendor descriptor here.
*/
// Interface number, string index, EP Out & IN address, EP size
#define TUD_VENDOR_EUB_DESCRIPTOR(_itfnum, _stridx, _epout, _epin, _epsize) \
  /* Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 2, TUSB_CLASS_VENDOR_SPECIFIC, EUB_VENDORD_IFACE_SUBCLASS, EUB_VENDORD_IFACE_PROTOCOL, _stridx,\
  /* Endpoint Out */\
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0,\
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0

static uint8_t const desc_configuration[] = {
    // config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, 0, 100),

    // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, 0x81, 8, EPNUM_CDC, 0x80 | EPNUM_CDC, CFG_TUD_CDC_EP_BUFSIZE),

    // Interface number, string index, EP Out & IN address, EP size
    TUD_VENDOR_EUB_DESCRIPTOR(ITF_NUM_VENDOR, EUB_VENDORD_IFACE_STR_IDX, EPNUM_VENDOR, 0x80 | EPNUM_VENDOR, 64),

    // Interface number, string index, EP Out & EP In address, EP size
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 6, EPNUM_MSC, 0x80 | EPNUM_MSC, 64),
};

#define MAC_BYTES       6

static char serial_descriptor[MAC_BYTES * 2 + 1] = {'\0'}; // 2 chars per hexnumber + '\0'

static char const *string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 }, // 0: is supported language is English (0x0409)
    CONFIG_BRIDGE_MANUFACTURER,    // 1: Manufacturer
#if CONFIG_BRIDGE_DEBUG_IFACE_JTAG
    CONFIG_BRIDGE_PRODUCT_NAME,    // 2: Product
#else
    "CMSIS-DAP",                   // OpenOCD expects "CMSIS-DAP" as a product name
#endif
    serial_descriptor,             // 3: Serials
    "CDC",
    CONFIG_BRIDGE_DEBUG_IFACE_NAME, // JTAG or CMSIS-DAP
    "MSC",

    /* JTAG_STR_DESC_INX 0x0A */
};

#define BOS_TOTAL_LEN      (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

// BOS Descriptor with Microsoft OS 2.0 support
static uint8_t const desc_bos[] = {
    // total length, number of device caps
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),

    // Microsoft OS 2.0 descriptor
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, VENDOR_REQUEST_MICROSOFT)
};

uint8_t const *tud_descriptor_bos_cb(void)
{
    return desc_bos;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    return desc_configuration;
}

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *) &descriptor_config;
}

void tud_mount_cb(void)
{
    ESP_LOGI(TAG, "Mounted");

    eub_vendord_start();
    eub_debug_probe_init();
}

static void init_serial_no(void)
{
    uint8_t m[MAC_BYTES] = {0};
    esp_err_t ret = esp_efuse_mac_get_default(m);

    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Cannot read MAC address and set the device serial number");
        eub_abort();
    }

    snprintf(serial_descriptor, sizeof(serial_descriptor),
             "%02X%02X%02X%02X%02X%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
}

uint16_t const *tud_descriptor_string_cb(const uint8_t index, const uint16_t langid)
{
    static uint16_t _desc_str[32];  // Static, because it must exist long enough for transfer to complete
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else if (index == EUB_DEBUG_PROBE_STR_DESC_INX) {
        chr_count = eub_debug_probe_get_proto_caps(&_desc_str[1]) / 2;
    } else {
        // Convert ASCII string into UTF-16

        if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) {
            return NULL;
        }

        const char *str = string_desc_arr[index];

        // Cap at max char
        chr_count = strlen(str);
        if (chr_count > 31) {
            chr_count = 31;
        }

        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }

    // first byte is length (including header), second byte is string type
    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);

    return _desc_str;
}

static void tusb_device_task(void *pvParameters)
{
    while (1) {
        tud_task();
    }
    vTaskDelete(NULL);
}

static void init_led_gpios(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_BRIDGE_GPIO_LED1) | (1ULL << CONFIG_BRIDGE_GPIO_LED2) |
                           (1ULL << CONFIG_BRIDGE_GPIO_LED3);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_level(CONFIG_BRIDGE_GPIO_LED1, !CONFIG_BRIDGE_GPIO_LED1_ACTIVE);
    gpio_set_level(CONFIG_BRIDGE_GPIO_LED2, !CONFIG_BRIDGE_GPIO_LED2_ACTIVE);
    gpio_set_level(CONFIG_BRIDGE_GPIO_LED3, !CONFIG_BRIDGE_GPIO_LED3_ACTIVE);

    ESP_LOGI(TAG, "LED GPIO init done");
}

static void int_usb_phy(void)
{
    usb_phy_config_t phy_config = {
        .controller = USB_PHY_CTRL_OTG,
        .target = USB_PHY_TARGET_INT,
        .otg_mode = USB_OTG_MODE_DEVICE,
        .otg_speed = USB_PHY_SPEED_FULL,
        .ext_io_conf = NULL,
        .otg_io_conf = NULL,
    };
    usb_phy_handle_t phy_handle;
    usb_new_phy(&phy_config, &phy_handle);
}

void app_main(void)
{
    init_led_gpios(); // Keep this at the begining. LEDs are used for error reporting.

    init_serial_no();

    int_usb_phy();

    tusb_init();
    msc_init();
    serial_init();

    xTaskCreate(tusb_device_task, "tusb_device_task", 4 * 1024, NULL, 5, NULL);
}
