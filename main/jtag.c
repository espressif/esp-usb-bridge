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
#include "io.h"

#define USB_RCVBUF_SIZE             4096
#define USB_SNDBUF_SIZE             32*1024

#define ESP_REMOTE_HEADER_LEN       4
#define ESP_REMOTE_CMD_VER_1        1
#define ESP_REMOTE_CMD_RESET        1
#define ESP_REMOTE_CMD_SCAN         2
#define ESP_REMOTE_CMD_TMS_SEQ      3
#define ESP_REMOTE_CMD_SET_CLK      4

#define ESP_REMOTE_CMD_MAX_BITS     12
#define ESP_REMOTE_CMD_MAX_BYTE     (1 << (ESP_REMOTE_CMD_MAX_BITS - 3))

static const char *TAG = "bridge_jtag";

static RingbufHandle_t usb_rcvbuf;
static RingbufHandle_t usb_sndbuf;

bool jtag_tdi_bootstrapping = false;  //TODO check in do_jtag_one

static uint8_t s_tdo_bytes[1024];
static uint16_t s_total_tdo_bits = 0;
static uint16_t s_usb_sent_bits = 0;

static void init_jtag_gpio()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_TDO) | (1ULL << GPIO_TCK) | (1ULL << GPIO_TMS);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_TDI);
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
                    eub_abort();
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
    uint8_t local_buf[CFG_TUD_VENDOR_TX_BUFSIZE];
    for (;;) {
        if (!tud_vendor_n_mounted(0)) {
            ESP_LOGD(TAG, "USB connection is not available!");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        size_t n = 0;
        uint8_t *buf = (uint8_t *) xRingbufferReceiveUpTo(usb_sndbuf, &n, portMAX_DELAY, sizeof(local_buf));
        memcpy(local_buf, buf, n);
        vRingbufferReturnItem(usb_sndbuf, (void *) buf);
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
            const int sent = tud_vendor_n_write(0, local_buf + transferred, MIN(space, to_send));
            transferred += sent;
            to_send -= sent;
            ESP_LOGD(TAG, "Space was %d, USB sent %d bytes", space, sent);
            ESP_LOG_BUFFER_HEXDUMP("USB sent", local_buf + transferred - sent, sent, ESP_LOG_DEBUG);
            // there seems to be no flush for vendor class
        }
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

static void do_jtag_one(uint8_t tdo_req, uint8_t tms, uint8_t tdi)
{
    gpio_set_level(GPIO_TDO, tdi);
    gpio_set_level(GPIO_TMS, tms);

    gpio_set_level(GPIO_TCK, 1);

    if (tdo_req) {
        s_total_tdo_bits++;
        s_tdo_bytes[(s_total_tdo_bits - 1) / 8] |= (gpio_get_level(GPIO_TDI) << ((s_total_tdo_bits - 1) % 8));
    }

    gpio_set_level(GPIO_TCK, 0);
}

void jtag_task(void *pvParameters)
{
    init_jtag_gpio();

    usb_rcvbuf = xRingbufferCreate(USB_RCVBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (!usb_rcvbuf) {
        ESP_LOGE(TAG, "Cannot allocate USB receive buffer!");
        eub_abort();
    }

    if (xTaskCreate(usb_reader_task, "usb_reader_task", 4 * 1024, NULL, uxTaskPriorityGet(NULL) - 1, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Cannot create USB reader task!");
        eub_abort();
    }

    usb_sndbuf = xRingbufferCreate(USB_SNDBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (!usb_sndbuf) {
        ESP_LOGE(TAG, "Cannot allocate USB send buffer!");
        eub_abort();
    }

    if (xTaskCreate(usb_send_task, "usb_send_task", 4 * 1024, NULL, uxTaskPriorityGet(NULL) + 1, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Cannot create USB send task!");
        eub_abort();
    }

    enum e_cmds {
        CMD_CLK_0 = 0, CMD_CLK_1, CMD_CLK_2, CMD_CLK_3,
        CMD_CLK_4, CMD_CLK_5, CMD_CLK_6, CMD_CLK_7,
        CMD_SRST0, CMD_SRST1, CMD_FLUSH, CMD_RSV,
        CMD_REP0, CMD_REP1, CMD_REP2, CMD_REP3
    };

    struct {
        uint8_t tdo_req;
        uint8_t tms;
        uint8_t tdi;
    } pin_levels[] = {
        { 0, 0, 0 }, //CMD_CLK_0
        { 0, 0, 1 }, //CMD_CLK_1
        { 0, 1, 0 }, //..
        { 0, 1, 1 },
        { 1, 0, 0 },
        { 1, 0, 1 },
        { 1, 1, 0 },
        { 1, 1, 1 }, //CMD_CLK_7
        { 0, 1, 0 }, //CMD_SRST0
        { 0, 1, 0 }, //CMD_SRST1
    };

    size_t cnt = 0;
    int prev_cmd = CMD_SRST0, rep_cnt = 0;

    while (1) {
        char *nibbles = (char *)xRingbufferReceive(usb_rcvbuf,
                        &cnt,
                        portMAX_DELAY);

        ESP_LOG_BUFFER_HEXDUMP(TAG, nibbles, cnt, ESP_LOG_DEBUG);

        for (size_t n = 0; n < cnt * 2 ; n++) {
            int cmd = (n & 1) ? (nibbles[n / 2] & 0x0F) : (nibbles[n / 2] >> 4);
            int cmd_exec = cmd, cmd_rpt_cnt = 1;

            switch (cmd) {
            case CMD_REP0:
            case CMD_REP1:
            case CMD_REP2:
            case CMD_REP3:
                //(r1*2+r0)<<(2*n)
                cmd_rpt_cnt = (cmd - CMD_REP0) << (2 * rep_cnt++);
                cmd_exec = prev_cmd;
                break;
            case CMD_SRST0:         // JTAG Tap reset command is not expected from host but still we are ready
                cmd_rpt_cnt = 8;    // 8 TMS=1 is more than enough to return the TAP state to RESET
                break;
            case CMD_SRST1:         // system reset
                gpio_set_level(GPIO_RST, 0);
                ets_delay_us(100);
                gpio_set_level(GPIO_RST, 1);
                break;
            default:
                rep_cnt = 0;
                break;
            }

            for (int i = 0; i < cmd_rpt_cnt; i++) {
                if (cmd_exec < CMD_SRST1) {
                    do_jtag_one(pin_levels[cmd_exec].tdo_req, pin_levels[cmd_exec].tms, pin_levels[cmd_exec].tdi);
                } else if (cmd_exec == CMD_FLUSH ) {
                    s_total_tdo_bits = (s_total_tdo_bits + 7) & (~7); // roundup
                    if (s_usb_sent_bits < s_total_tdo_bits) {
                        int waiting_to_send_bits = s_total_tdo_bits - s_usb_sent_bits;
                        while (waiting_to_send_bits > 0) {
                            int send_bits = waiting_to_send_bits > 512 ? 512 : waiting_to_send_bits;
                            usb_send(s_tdo_bytes + (s_usb_sent_bits / 8), send_bits / 8);
                            s_usb_sent_bits += send_bits;
                            waiting_to_send_bits -= send_bits;
                        }
                        memset(s_tdo_bytes, 0x00, sizeof(s_tdo_bytes));
                        s_total_tdo_bits = s_usb_sent_bits = 0;
                        continue;
                    }
                }
            }

            /* As soon as either 64 bytes (512 bits) have been collected or a CMD_FLUSH command is executed,
                make the usb buffer available for the host to receive.
            */
            int waiting_to_send_bits = s_total_tdo_bits - s_usb_sent_bits;
            if (waiting_to_send_bits >= 512) {
                int send_bits = waiting_to_send_bits > 512 ? 512 : waiting_to_send_bits;
                int n = (send_bits + 7) / 8;  // roundup
                usb_send(s_tdo_bytes + (s_usb_sent_bits / 8), n);
                memset(s_tdo_bytes + (s_usb_sent_bits / 8), 0x00, n);
                s_usb_sent_bits += n * 8;
                waiting_to_send_bits -= n * 8;
                if (waiting_to_send_bits <= 0) {
                    s_total_tdo_bits = s_usb_sent_bits = 0;
                }
            }

            if (cmd < CMD_REP0 && cmd != CMD_FLUSH) {
                prev_cmd = cmd;
            }
        }

        vRingbufferReturnItem(usb_rcvbuf, (void *)nibbles);
    }

    vTaskDelete(NULL);
}
