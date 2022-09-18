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
#include "hal/gpio_ll.h"
#include "hal/gpio_hal.h"

#define USB_RCVBUF_SIZE             4096
#define USB_SNDBUF_SIZE             (32*1024)

#define ROUND_UP_BITS(x)            ((x + 7) & (~7))

static const char *TAG = "bridge_jtag";

/* esp usb serial protocol specific definitions */
#define JTAG_PROTO_MAX_BITS      (CFG_TUD_VENDOR_RX_BUFSIZE * 8)
#define JTAG_PROTO_CAPS_VER 1   /*Version field. */
typedef struct __attribute__((packed))
{
    uint8_t proto_ver;  /*Protocol version. Expects JTAG_PROTO_CAPS_VER for now. */
    uint8_t length; /*of this plus any following descriptors */
} jtag_proto_caps_hdr_t;

#define JTAG_PROTO_CAPS_SPEED_APB_TYPE 1
typedef struct __attribute__((packed))
{
    uint8_t type;
    uint8_t length;
} jtag_gen_hdr_t;

typedef struct __attribute__((packed))
{
    uint8_t type;   /*Type, always JTAG_PROTO_CAPS_SPEED_APB_TYPE */
    uint8_t length; /*Length of this */
    uint16_t apb_speed_10khz;   /*ABP bus speed, in 10KHz increments. Base speed is half
                     * this. */
    uint16_t div_min;   /*minimum divisor (to base speed), inclusive */
    uint16_t div_max;   /*maximum divisor (to base speed), inclusive */
} jtag_proto_caps_speed_apb_t;

typedef struct {
    jtag_proto_caps_hdr_t proto_hdr;
    jtag_proto_caps_speed_apb_t caps_apb;
} jtag_proto_caps_t;

#define VEND_JTAG_SETDIV        0
#define VEND_JTAG_SETIO         1
#define VEND_JTAG_GETTDO        2
#define VEND_JTAG_SET_CHIPID    3

// TCK frequency is around 750KHZ and we do not support selective clock for now.
#define TCK_FREQ(khz) ((khz * 2) / 10)
static const jtag_proto_caps_t jtag_proto_caps = {
    {.proto_ver = JTAG_PROTO_CAPS_VER, .length = sizeof(jtag_proto_caps_hdr_t) + sizeof(jtag_proto_caps_speed_apb_t)},
    {.type = JTAG_PROTO_CAPS_SPEED_APB_TYPE, .length = sizeof(jtag_proto_caps_speed_apb_t), .apb_speed_10khz = TCK_FREQ(750), .div_min = 1, .div_max = 1}
};

static RingbufHandle_t usb_rcvbuf;
static RingbufHandle_t usb_sndbuf;

static uint8_t s_tdo_bytes[1024];
static uint16_t s_total_tdo_bits = 0;
static uint16_t s_usb_sent_bits = 0;
static esp_chip_model_t s_target_model;
static TaskHandle_t s_jtag_task_handle = NULL;
static TaskHandle_t s_usb_tx_task_handle = NULL;
static gpio_dev_t *const s_gpio_dev = GPIO_HAL_GET_HW(GPIO_PORT_0);

void tud_vendor_rx_cb(uint8_t itf)
{
    (void)itf;

    uint8_t buf[CFG_TUD_VENDOR_RX_BUFSIZE];
    uint32_t r;

    while ((r = tud_vendor_n_read(0, buf, sizeof(buf))) > 0) {
        if (xRingbufferSend(usb_rcvbuf, buf, r, pdMS_TO_TICKS(1000)) != pdTRUE) {
            ESP_LOGE(TAG, "Cannot write to usb_rcvbuf ringbuffer (free %d of %d)!",
                     xRingbufferGetCurFreeSize(usb_rcvbuf), USB_RCVBUF_SIZE);
            eub_abort();
        }
    }

    if (xRingbufferGetCurFreeSize(usb_rcvbuf) < (USB_RCVBUF_SIZE * 8) / 10) {
        ESP_LOGW(TAG, "Ringbuffer is getting full!");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void tud_mount_cb(void)
{
    ESP_LOGI(TAG, "Mounted");
    xTaskNotifyGive(s_usb_tx_task_handle);
}

bool tud_vendor_control_xfer_cb(const uint8_t rhport, const uint8_t stage, tusb_control_request_t const *request)
{
    // nothing to with DATA & ACK stage
    if (stage != CONTROL_STAGE_SETUP) {
        return true;
    }

    switch (request->bmRequestType_bit.type) {
    case TUSB_REQ_TYPE_VENDOR:
        ESP_LOGI(TAG, "bRequest: (%d) wValue: (%d) wIndex: (%d)",
                 request->bRequest, request->wValue, request->wIndex);

        switch (request->bRequest) {
        case VEND_JTAG_SETDIV:
        case VEND_JTAG_SETIO:
            // TODO: process the commands
            break;
        case VEND_JTAG_GETTDO: {
            uint8_t buf = gpio_get_level(GPIO_TDI);
            return tud_control_xfer(rhport, request, (void *)&buf, 1);
        }
        break;
        case VEND_JTAG_SET_CHIPID:
            s_target_model = request->wValue;
        }

        // response with status OK
        return tud_control_status(rhport, request);
    }

    return false;
}

static void init_jtag_gpio(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << GPIO_TDI) | (1ULL << GPIO_TCK) | (1ULL << GPIO_TMS),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_TDO);
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_LOGI(TAG, "JTAG GPIO init done");

    gpio_set_level(GPIO_TMS, 1);
    gpio_set_level(GPIO_TCK, 0);
}

static void usb_send_task(void *pvParameters)
{
    uint8_t local_buf[CFG_TUD_VENDOR_TX_BUFSIZE];

    /* When the device is mounted the tud_mount_cb will notify this task. */
    ESP_LOGD(TAG, "Waiting for the device to be mounted...");
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ESP_LOGD(TAG, "Device mounted!");

    for (;;) {
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
                if (ring_free < (USB_SNDBUF_SIZE * 3) / 10) {
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

static int usb_send(const uint8_t *buf, const int size)
{
    if (xRingbufferSend(usb_sndbuf, buf, size, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Cannot write to usb_sndbuf ringbuffer (free %d of %d)!",
                 xRingbufferGetCurFreeSize(usb_sndbuf), USB_SNDBUF_SIZE);
        return 0;
    }
    return size;
}

inline static void do_jtag_one(const uint8_t tdo_req, const uint8_t tms, const uint8_t tdi)
{
    gpio_ll_set_level(s_gpio_dev, GPIO_TDI, tdi);
    gpio_ll_set_level(s_gpio_dev, GPIO_TMS, tms);

    gpio_ll_set_level(s_gpio_dev, GPIO_TCK, 1);

    if (tdo_req) {
        s_tdo_bytes[s_total_tdo_bits / 8] |= (gpio_ll_get_level(s_gpio_dev, GPIO_TDO) << (s_total_tdo_bits % 8));
        s_total_tdo_bits++;
    }

    gpio_ll_set_level(s_gpio_dev, GPIO_TCK, 0);
}

int jtag_get_proto_caps(uint16_t *dest)
{
    memcpy(dest, (uint16_t *)&jtag_proto_caps, sizeof(jtag_proto_caps));
    return sizeof(jtag_proto_caps);
}

int jtag_get_target_model(void)
{
    return s_target_model;
}

void jtag_task_suspend(void)
{
    if (s_jtag_task_handle) {
        vTaskSuspend(s_jtag_task_handle);
    }
}

void jtag_task_resume(void)
{
    if (s_jtag_task_handle) {
        vTaskResume(s_jtag_task_handle);
    }
}

static void jtag_task(void *pvParameters)
{
    enum e_cmds {
        CMD_CLK_0 = 0, CMD_CLK_1, CMD_CLK_2, CMD_CLK_3,
        CMD_CLK_4, CMD_CLK_5, CMD_CLK_6, CMD_CLK_7,
        CMD_SRST0, CMD_SRST1, CMD_FLUSH, CMD_RSV,
        CMD_REP0, CMD_REP1, CMD_REP2, CMD_REP3
    };

    const struct {
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

    s_jtag_task_handle = xTaskGetCurrentTaskHandle();

    size_t cnt = 0;
    int prev_cmd = CMD_SRST0, rep_cnt = 0;

    while (1) {
        gpio_ll_set_level(s_gpio_dev, LED_JTAG, LED_JTAG_OFF);
        char *nibbles = (char *)xRingbufferReceive(usb_rcvbuf,
                        &cnt,
                        portMAX_DELAY);
        gpio_ll_set_level(s_gpio_dev, LED_JTAG, LED_JTAG_ON);

        ESP_LOG_BUFFER_HEXDUMP(TAG, nibbles, cnt, ESP_LOG_DEBUG);

        for (size_t n = 0; n < cnt * 2 ; n++) {
            const int cmd = (n & 1) ? (nibbles[n / 2] & 0x0F) : (nibbles[n / 2] >> 4);
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
            case CMD_SRST1:         // FIXME: system reset may cause an issue during openocd examination
                cmd_rpt_cnt = 8;    // for now this is also used for the tap reset
                // gpio_set_level(GPIO_RST, 0);
                // ets_delay_us(100);
                // gpio_set_level(GPIO_RST, 1);
                break;
            default:
                rep_cnt = 0;
                break;
            }

            for (int i = 0; i < cmd_rpt_cnt; i++) {
                if (cmd_exec < CMD_FLUSH) {
                    do_jtag_one(pin_levels[cmd_exec].tdo_req, pin_levels[cmd_exec].tms, pin_levels[cmd_exec].tdi);
                } else if (cmd_exec == CMD_FLUSH ) {
                    s_total_tdo_bits = ROUND_UP_BITS(s_total_tdo_bits);
                    if (s_usb_sent_bits < s_total_tdo_bits) {
                        int waiting_to_send_bits = s_total_tdo_bits - s_usb_sent_bits;
                        while (waiting_to_send_bits > 0) {
                            int send_bits = waiting_to_send_bits > JTAG_PROTO_MAX_BITS ? JTAG_PROTO_MAX_BITS : waiting_to_send_bits;
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
            if (waiting_to_send_bits >= JTAG_PROTO_MAX_BITS) {
                int send_bits = ROUND_UP_BITS(waiting_to_send_bits > JTAG_PROTO_MAX_BITS ? JTAG_PROTO_MAX_BITS : waiting_to_send_bits);
                int n_byte = send_bits / 8;
                usb_send(s_tdo_bytes + (s_usb_sent_bits / 8), n_byte);
                memset(s_tdo_bytes + (s_usb_sent_bits / 8), 0x00, n_byte);
                s_usb_sent_bits += send_bits;
                waiting_to_send_bits -= send_bits;
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

void jtag_init(void)
{
    init_jtag_gpio();

    usb_rcvbuf = xRingbufferCreate(USB_RCVBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (!usb_rcvbuf) {
        ESP_LOGE(TAG, "Cannot allocate USB receive buffer!");
        eub_abort();
    }

    usb_sndbuf = xRingbufferCreate(USB_SNDBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (!usb_sndbuf) {
        ESP_LOGE(TAG, "Cannot allocate USB send buffer!");
        eub_abort();
    }

    if (xTaskCreate(usb_send_task, "usb_send_task", 4 * 1024, NULL,
                    uxTaskPriorityGet(NULL) + 1, &s_usb_tx_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "Cannot create USB send task!");
        eub_abort();
    }

    if (xTaskCreate(jtag_task, "jtag_task", 4 * 1024, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Cannot create JTAG task!");
        eub_abort();
    }
}
