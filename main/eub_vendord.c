// Copyright 2020-2024 Espressif Systems (Shanghai) CO LTD
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
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "tusb.h"
#include "device/usbd_pvt.h"
#include "tusb_config.h"
#include "esp_timer.h"

#include "eub_vendord.h"
#include "util.h"

static const char *TAG = "eub_vendor";

/* 4K ringbuffer size is more than enough */
#define USB_RCVBUF_SIZE     4096
#define USB_SNDBUF_SIZE     4096
#define USB_BUSY_TOUT_US    100000 /* 100ms */

typedef struct {
    uint8_t ep_in;
    uint8_t ep_out;
    RingbufHandle_t usb_rcvbuf;
    RingbufHandle_t usb_sndbuf;
    TaskHandle_t usb_tx_task_handle;
    uint8_t epout_buf[EUB_VENDORD_EPSIZE];  /* Endpoint Transfer buffer */
} eub_vendord_interface_t;

static eub_vendord_interface_t s_eub_vendord_itf;
static const uint8_t s_rhport = 0;

static void eub_vendord_init(void)
{
    ESP_LOGD(TAG, "%s", __func__);

    memset(&s_eub_vendord_itf, 0x00, sizeof(s_eub_vendord_itf));
}

static void eub_vendord_reset(uint8_t rhport)
{
    ESP_LOGD(TAG, "%s", __func__);

    s_eub_vendord_itf.ep_in = 0;
    s_eub_vendord_itf.ep_out = 0;
    /* do not reset the FreeRTOS handlers */
}

static uint16_t eub_vendord_open(uint8_t rhport, tusb_desc_interface_t const *desc_intf, uint16_t max_len)
{
    ESP_LOGD(TAG, "%s", __func__);

    TU_VERIFY(TUSB_CLASS_VENDOR_SPECIFIC == desc_intf->bInterfaceClass &&
              EUB_VENDORD_IFACE_SUBCLASS == desc_intf->bInterfaceSubClass &&
              EUB_VENDORD_IFACE_PROTOCOL == desc_intf->bInterfaceProtocol &&
              EUB_VENDORD_IFACE_STR_IDX == desc_intf->iInterface, 0);

    uint8_t const *p_desc = tu_desc_next(desc_intf);
    eub_vendord_interface_t *p_vendor = &s_eub_vendord_itf;

    // Open endpoint pair with usbd helper
    usbd_open_edpt_pair(rhport, p_desc, desc_intf->bNumEndpoints, TUSB_XFER_BULK, &p_vendor->ep_out, &p_vendor->ep_in);

    ESP_LOGI(TAG, "EP_OUT:0x%x EP_IN:0x%x", p_vendor->ep_out, p_vendor->ep_in);

    p_desc += desc_intf->bNumEndpoints * sizeof(tusb_desc_endpoint_t);

    // Prepare for incoming data
    if (p_vendor->ep_out) {
        usbd_edpt_xfer(rhport, p_vendor->ep_out, p_vendor->epout_buf, sizeof(p_vendor->epout_buf));
    }

    return (uint16_t)((uintptr_t)p_desc - (uintptr_t)desc_intf);
}

static bool eub_vendord_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    ESP_LOGI(TAG, "%s", __func__);

    /* Nothing to do here. `tud_vendor_control_xfer_cb` will take care of the settings. */
    return true;
}

static bool eub_endpt_transfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buf, uint16_t total_bytes)
{
    uint64_t end = esp_timer_get_time() + USB_BUSY_TOUT_US;

    while (usbd_edpt_busy(rhport, ep_addr)) {
        if (esp_timer_get_time() > end) {
            ESP_LOGE(TAG, "usbd_edpt_busy timeout!");
            return false;
        }
    }

    return usbd_edpt_xfer(s_rhport, ep_addr, buf, total_bytes);
}

static bool eub_vendord_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
    TU_VERIFY(xferred_bytes > 0 && xferred_bytes <= 64, false);
    TU_VERIFY(ep_addr == s_eub_vendord_itf.ep_out || ep_addr == s_eub_vendord_itf.ep_in, false);

    const uint8_t ep_dir = tu_edpt_dir(ep_addr);

    ESP_LOGD(TAG, "%s xfer from ep:%x recvd:%" PRId32 " bytes", __func__, ep_addr, xferred_bytes);

    if (ep_dir == TUSB_DIR_IN) {
        /* nothing to do for now */
        return true;
    } else if (ep_dir == TUSB_DIR_OUT) {
        BaseType_t res = xRingbufferSend(s_eub_vendord_itf.usb_rcvbuf,
                                         s_eub_vendord_itf.epout_buf, xferred_bytes, pdMS_TO_TICKS(1000));
        if (res != pdTRUE) {
            ESP_LOGE(TAG, "Cannot write to usb_rcvbuf ringbuffer (free %d of %d)!",
                     xRingbufferGetCurFreeSize(s_eub_vendord_itf.usb_rcvbuf), USB_RCVBUF_SIZE);
            eub_abort();
        }

        // Prepare for next incoming data
        if (!eub_endpt_transfer(rhport, ep_addr, s_eub_vendord_itf.epout_buf, EUB_VENDORD_EPSIZE)) {
            ESP_LOGE(TAG, "USB tranfer error on EP:%x", ep_addr);
            eub_abort();
        }

        return true;
    }

    return false;
}

static void usb_send_task(void *pvParameters)
{
    size_t total_bytes;

    ESP_LOGI(TAG, "usb_send_task is ready!");

    for (;;) {
        uint8_t *buf = xRingbufferReceive(s_eub_vendord_itf.usb_sndbuf, &total_bytes, portMAX_DELAY);

        ESP_LOG_BUFFER_HEXDUMP("res", buf, total_bytes, ESP_LOG_DEBUG);

        if (!eub_endpt_transfer(s_rhport, s_eub_vendord_itf.ep_in, buf, total_bytes)) {
            ESP_LOGE(TAG, "USB tranfer error on EP:%x", s_eub_vendord_itf.ep_in);
            eub_abort();
        }

        vRingbufferReturnItem(s_eub_vendord_itf.usb_sndbuf, buf);
    }

    vTaskDelete(NULL);
}

int eub_vendord_send_item(const void *buf, const size_t size)
{
    if (xRingbufferSend(s_eub_vendord_itf.usb_sndbuf, buf, size, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Cannot write to usb_sndbuf ringbuffer (free %d of %d)!",
                 xRingbufferGetCurFreeSize(s_eub_vendord_itf.usb_sndbuf), USB_SNDBUF_SIZE);
        return 0;
    }
    return size;
}

void *eub_vendord_recv_acquire_item(size_t *item_size)
{
    return xRingbufferReceive(s_eub_vendord_itf.usb_rcvbuf, item_size, portMAX_DELAY);
}

void eub_vendord_free_item(void *item)
{
    vRingbufferReturnItem(s_eub_vendord_itf.usb_rcvbuf, item);
}

void eub_vendord_start(void)
{
    ESP_LOGD(TAG, "%s", __func__);

    static bool init = false;

    if (!init) {

        /* We would like to process OpenOCD packages one by one. RINGBUF_TYPE_NOSPLIT allows us to do it. */
        s_eub_vendord_itf.usb_rcvbuf = xRingbufferCreate(USB_RCVBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
        if (!s_eub_vendord_itf.usb_rcvbuf) {
            ESP_LOGE(TAG, "Cannot allocate USB receive buffer!");
            eub_abort();
        }

        s_eub_vendord_itf.usb_sndbuf = xRingbufferCreate(USB_SNDBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
        if (!s_eub_vendord_itf.usb_sndbuf) {
            ESP_LOGE(TAG, "Cannot allocate USB send buffer!");
            eub_abort();
        }

        if (xTaskCreate(usb_send_task,
                        "usb_send_task",
                        4 * 1024,
                        NULL,
                        EUB_VENDORD_USB_TASK_PRI,
                        &s_eub_vendord_itf.usb_tx_task_handle) != pdPASS) {
            ESP_LOGE(TAG, "Cannot create USB send task!");
            eub_abort();
        }

        init = true;
    }
}

const usbd_class_driver_t s_eub_vendord_driver = {
#if CFG_TUSB_DEBUG >= 2
    .name = "EUB-VENDOR",
#endif
    .init = eub_vendord_init,
    .reset = eub_vendord_reset,
    .open = eub_vendord_open,
    .control_xfer_cb = eub_vendord_control_xfer_cb,
    .xfer_cb = eub_vendord_xfer_cb,
    .sof = NULL,
};

const usbd_class_driver_t *usbd_app_driver_get_cb(uint8_t *driver_count)
{
    ESP_LOGD(TAG, "%s", __func__);

    *driver_count = 1;
    return &s_eub_vendord_driver;
}
