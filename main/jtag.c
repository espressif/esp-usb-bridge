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

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "jtag.h"
#include "tusb.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "util.h"

#define GPIO_TDI                    CONFIG_BRIDGE_GPIO_TDI
#define GPIO_TDO                    CONFIG_BRIDGE_GPIO_TDO
#define GPIO_TCK                    CONFIG_BRIDGE_GPIO_TCK
#define GPIO_TMS                    CONFIG_BRIDGE_GPIO_TMS

#define USB_RCVBUF_SIZE             1024
#define USB_SNDBUF_SIZE             1024

#define ESP_REMOTE_HEADER_LEN       4
#define ESP_REMOTE_CMD_VER_1        1
#define ESP_REMOTE_CMD_RESET        1
#define ESP_REMOTE_CMD_SCAN         2
#define ESP_REMOTE_CMD_TMS_SEQ      3

static const char *TAG = "bridge_jtag";

static uint8_t last_tms = 0;

static RingbufHandle_t usb_rcvbuf;
static RingbufHandle_t usb_sndbuf;

static inline uint8_t *alloc_with_abort(const char *field_name, int size)
{
    uint8_t *f = malloc(size * sizeof(uint8_t));
    if (!f) {
        ESP_LOGE(TAG, "Cannot allocate %d bytes for %s!", size, field_name);
        abort();
    }
    return f;
}

static void init_jtag_gpio()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1UL << GPIO_TDO) | (1UL << GPIO_TCK) | (1UL << GPIO_TMS);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1UL << GPIO_TDI);
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_LOGI(TAG, "JTAG GPIO init done");

    gpio_set_level(GPIO_TMS, 1);
    gpio_set_level(GPIO_TCK, 0);
}

static void usb_reader_task(void *pvParameters)
{
    uint8_t buf[CFG_TUD_VENDOR_RX_BUFSIZE];
    for (;;) {
        if (tud_vendor_n_available(0)) {
            uint32_t r;
            while ((r = tud_vendor_n_read(0, buf, sizeof(buf))) > 0) {
                if (xRingbufferSend(usb_rcvbuf, buf, r, pdMS_TO_TICKS(1000)) != pdTRUE) {
                    ESP_LOGE(TAG, "Cannot write to usb_rcvbuf ringbuffer (free %d of %d)!",
                            xRingbufferGetCurFreeSize(usb_rcvbuf), USB_RCVBUF_SIZE);
                }
            }

            if (xRingbufferGetCurFreeSize(usb_rcvbuf) < 0.8 * USB_RCVBUF_SIZE) {
                ESP_LOGW(TAG, "Ringbuffer is getting full!");
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
        // Even vTaskDelay(1) would give poor performance with 99% IDLE task.
        taskYIELD();
    }
    vTaskDelete(NULL);
}

static void usb_send_task(void *pvParameters)
{
    for (;;) {
        if (!tud_vendor_n_mounted(0)) {
            ESP_LOGD(TAG, "USB connection is not available!");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        size_t n = 0;
        uint8_t *buf = (uint8_t *) xRingbufferReceiveUpTo(usb_sndbuf, &n, portMAX_DELAY, CFG_TUD_VENDOR_TX_BUFSIZE);
        for (int transferred = 0, to_send = n; transferred < n;) {
            int space = tud_vendor_n_write_available(0);
            if (space == 0) {
                // Openocd sometime sends a lot of SCAN requests and reads them later. If these requests are 32-bit
                // registers then the USB buffer will fill up very quickly. It is everything fine until there is space
                // in the usb_sndbuf ringbuffer.
                const size_t ring_free = xRingbufferGetCurFreeSize(usb_sndbuf);
                if (ring_free < 0.3 * USB_SNDBUF_SIZE) {
                    ESP_LOGW(TAG, "USB send buffer is full, usb_sndbuf ringbuffer is getting full "
                            "(has %d free bytes of %d)", ring_free, USB_SNDBUF_SIZE);
                }
                vTaskDelay(1);
                continue;
            }
            const int sent = tud_vendor_n_write(0, buf + transferred, MIN(space, to_send));
            transferred += sent;
            to_send -= sent;
            ESP_LOGD(TAG, "Space was %d, USB sent %d bytes", space, sent);
            ESP_LOG_BUFFER_HEXDUMP("USB sent", buf + transferred - sent, sent, ESP_LOG_DEBUG);
            // there seems to be no flush for vendor class
        }
        vRingbufferReturnItem(usb_sndbuf, (void *) buf);
    }
    vTaskDelete(NULL);
}

static int usb_send(const uint8_t *buf, int size)
{
    if (xRingbufferSend(usb_sndbuf, buf, size, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Cannot write to usb_sndbuf ringbuffer (free %d of %d)!",
                xRingbufferGetCurFreeSize(usb_sndbuf), USB_SNDBUF_SIZE);
        return 0;
    }
    return size;
}

static void do_jtag(int n_bits, int n_bytes, const uint8_t *tms_bytes, const uint8_t *tdo_bytes, uint8_t *tdi_bytes)
{
    if (tdi_bytes) {
        // the last byte might be incomplete (i.e. when n_bits is not a multiply of 8) so lets initialize it before the
        // loop:
        tdi_bytes[n_bytes - 1] = 0;
    }

    // The recommended TCK frequency is at most 1/4 of the CPU frequency based on Xtensa Debug Guide. The following
    // code does TCK bitbanging at the highest possible frequency which is below the max frequency. This check is not
    // done here and is possible for example with a logic analyzer.

    for (int i = 0, byte = 0, bit = 0; i < n_bits; ++i, byte = i >> 3, bit = i & 7) {
        gpio_set_level(GPIO_TDO, (tdo_bytes[byte] & (1u << bit)) != 0);
        gpio_set_level(GPIO_TMS, (tms_bytes[byte] & (1u << bit)) != 0);

        gpio_set_level(GPIO_TCK, 1);

        // TDO changes are sampled on failing edge of TCK
        if (tdi_bytes) {
            if (gpio_get_level(GPIO_TDI)) {
                tdi_bytes[byte] |= (1u << bit);
            } else {
                tdi_bytes[byte] &= ~(1u << bit);
            }
        }

        gpio_set_level(GPIO_TCK, 0);
    }
}

static void cmd_scan(int n_bits, int n_bytes, bool do_flip_tms, const uint8_t *data_out, uint8_t *data_in)
{
    uint8_t *tms_bytes = alloc_with_abort("tms_bytes", n_bytes);
    memset(tms_bytes, last_tms ? 0xFF : 0, n_bytes);
    if (do_flip_tms) {
        tms_bytes[(n_bits - 1) / 8] ^= (1u << ((n_bits - 1) % 8));
    }
    do_jtag(n_bits, n_bytes, tms_bytes, data_out, data_in);
    free(tms_bytes);
}

static void cmd_tms_seq(int n_bits, int n_bytes, const uint8_t *data_out)
{
    uint8_t *tdo_bytes = alloc_with_abort("tdo_bytes", n_bytes);
    memset(tdo_bytes, 0xFF, n_bytes);

    do_jtag(n_bits, n_bytes, data_out, tdo_bytes, NULL);
    last_tms = (data_out[n_bits / 8] & (1u << (n_bits % 8))) != 0;

    free(tdo_bytes);
}

static void ringbuffer_wait_for_full_string(uint8_t *buf, int size)
{
    for (int i = 0; i < size;) {
        size_t received = 0;
        uint8_t *x = xRingbufferReceiveUpTo(usb_rcvbuf, &received, portMAX_DELAY, size - i);
        memcpy(buf + i, x, received);
        vRingbufferReturnItem(usb_rcvbuf, (void *) x);
        i += received;
    }
}


void jtag_task(void *pvParameters)
{
    init_jtag_gpio();

    usb_rcvbuf = xRingbufferCreate(USB_RCVBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (!usb_rcvbuf) {
        ESP_LOGE(TAG, "Cannot allocate USB receive buffer!");
        abort();
    }

    if (xTaskCreate(usb_reader_task, "usb_reader_task", 4 * 1024, NULL, uxTaskPriorityGet(NULL) - 1, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Cannot create USB reader task!");
        abort();
    }

    usb_sndbuf = xRingbufferCreate(USB_SNDBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (!usb_sndbuf) {
        ESP_LOGE(TAG, "Cannot allocate USB send buffer!");
        abort();
    }

    if (xTaskCreate(usb_send_task, "usb_send_task", 4 * 1024, NULL, uxTaskPriorityGet(NULL) - 1, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Cannot create USB send task!");
        abort();
    }

    while (1) {
        uint8_t header[ESP_REMOTE_HEADER_LEN];
        ringbuffer_wait_for_full_string(header, sizeof(header));
        while ((header[0] >> 4) != ESP_REMOTE_CMD_VER_1) {
            ESP_LOGE(TAG, "Incorrect header was received!");
            ESP_LOG_BUFFER_HEXDUMP(TAG, header, sizeof(header), ESP_LOG_ERROR);

            memmove(header, header + 1, sizeof(header) - 1);                 // drop the first byte
            ringbuffer_wait_for_full_string(header + sizeof(header) - 1, 1); // read another byte
        }

        const uint8_t cmd = header[1];
        const uint16_t func_specific = (((uint16_t) header[3]) << 8) | header[2];

        size_t n = 0;
        if (cmd == ESP_REMOTE_CMD_SCAN) {
            const uint16_t n_bits = func_specific & 0xFFF;
            const uint16_t n_bytes = (n_bits + 7) / 8;
            const uint8_t read = (func_specific >> 12) & 1;
            const uint8_t flip_tms = (func_specific >> 13) & 1;
            ESP_LOGD(TAG, "Received CMD_SCAN: n_bits = %d, read = %d, flip_tms = %d", n_bits, read, flip_tms);
            for (int to_read = n_bytes, to_read_bits = n_bits; to_read > 0; to_read -= n, to_read_bits -= (8 * n)) {
                uint8_t *buf = (uint8_t *) xRingbufferReceiveUpTo(usb_rcvbuf, &n, portMAX_DELAY, to_read);
                ESP_LOGD(TAG, "[SCAN] Received: %d (need %d)", n, to_read);
                ESP_LOG_BUFFER_HEXDUMP("SCAN received", buf, n, ESP_LOG_DEBUG);
                uint8_t *data_out = read ? alloc_with_abort("the readout data", n) : NULL;
                cmd_scan(MIN(n * 8, to_read_bits), n, n == to_read ? flip_tms : 0, buf, data_out);
                vRingbufferReturnItem(usb_rcvbuf, (void *) buf);
                if (data_out) {
                    usb_send(data_out, n);
                    free(data_out);
                }
            }
        } else if (cmd == ESP_REMOTE_CMD_TMS_SEQ) {
            const uint16_t n_bits = func_specific & 0xFFF;
            const uint16_t n_bytes = (n_bits + 7) / 8;
            ESP_LOGD(TAG, "Received CMD_TMS_SEQ: n_bits = %d", n_bits);
            for (int to_read = n_bytes, to_read_bits = n_bits; to_read > 0; to_read -= n, to_read_bits -= (8 * n)) {
                uint8_t *buf = (uint8_t *) xRingbufferReceiveUpTo(usb_rcvbuf, &n, portMAX_DELAY, to_read);
                ESP_LOGD(TAG, "[TMS] Received: %d (need %d)", n, to_read);
                ESP_LOG_BUFFER_HEXDUMP("TMS received", buf, n, ESP_LOG_DEBUG);
                cmd_tms_seq(MIN(n * 8, to_read_bits), n, buf);
                vRingbufferReturnItem(usb_rcvbuf, (void *) buf);
            }
        } else if (cmd == ESP_REMOTE_CMD_RESET) {
            const uint8_t srst = func_specific & 1;
            const uint8_t trst = (func_specific >> 1) & 1;
            ESP_LOGD(TAG, "Received CMD_RESET: srst = %d, trst = %d", srst, trst);

            const uint8_t buf[] = {0xff}; // 8 TMS=1 is more than enough to return the TAP state to RESET
            cmd_tms_seq(sizeof(buf) * 8, sizeof(buf), buf);
        } else {
            ESP_LOGE(TAG, "Received an unsupported command: %d", cmd);
        }
    }
    vTaskDelete(NULL);
}
