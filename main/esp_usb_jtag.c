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
#include "esp_chip_info.h"
#include "tusb.h"

#include "eub_vendord.h"
#include "util.h"
#include "esp_io.h"

static const char *TAG = "esp_usb_jtag";

static esp_chip_model_t s_target_model;
static TaskHandle_t s_jtag_task_handle = NULL;
static uint8_t s_tdo_bytes[1024];
static uint16_t s_total_tdo_bits = 0;
static uint16_t s_usb_sent_bits = 0;

#define ROUND_UP_BITS(x)        ((x + 7) & (~7))

/* esp usb serial protocol specific definitions */
#define JTAG_PROTO_MAX_BITS     (EUB_VENDORD_EPSIZE * 8)
#define JTAG_PROTO_CAPS_VER     1   /*Version field. */
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

// TCK frequency is around 4800KHZ and we do not support selective clock for now.
#define TCK_FREQ(khz) ((khz * 2) / 10)
static const jtag_proto_caps_t jtag_proto_caps = {
    {.proto_ver = JTAG_PROTO_CAPS_VER, .length = sizeof(jtag_proto_caps_hdr_t) + sizeof(jtag_proto_caps_speed_apb_t)},
    {.type = JTAG_PROTO_CAPS_SPEED_APB_TYPE, .length = sizeof(jtag_proto_caps_speed_apb_t), .apb_speed_10khz = TCK_FREQ(4800), .div_min = 1, .div_max = 1}
};

void eub_debug_probe_init(void) __attribute__((alias("esp_usb_jtag_init")));
int eub_debug_probe_get_proto_caps(void *dest) __attribute__((alias("esp_usb_jtag_get_proto_caps")));
void eub_debug_probe_task_suspend(void) __attribute__((alias("esp_usb_jtag_task_suspend")));
void eub_debug_probe_task_resume(void) __attribute__((alias("esp_usb_jtag_task_resume")));
bool eub_debug_probe_target_is_esp32(void) __attribute__((alias("esp_usb_jtag_target_is_esp32")));

static int esp_usb_jtag_get_proto_caps(void *dest)
{
    memcpy(dest, (uint16_t *)&jtag_proto_caps, sizeof(jtag_proto_caps));
    return sizeof(jtag_proto_caps);
}

static bool esp_usb_jtag_target_is_esp32(void)
{
    return s_target_model == CHIP_ESP32;
}

static void esp_usb_jtag_task_suspend(void)
{
    if (s_jtag_task_handle) {
        vTaskSuspend(s_jtag_task_handle);
    }
}

static void esp_usb_jtag_task_resume(void)
{
    if (s_jtag_task_handle) {
        vTaskResume(s_jtag_task_handle);
    }
}

/* Control settings are not directly related to the vendor class.
   e.g; dap protocol also communicates through vendor class but handles settings within the protocol commands.
   Therefore, we're adding the callback here to keep `eub_vendord.c` isolated and common for swd also.
*/
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

inline static void do_jtag_one(const uint8_t tdo_req, const uint8_t tms_tdi_mask)
{
    esp_gpio_write_tmstck(tms_tdi_mask);
    esp_gpio_tck_set();

    if (tdo_req) {
        s_tdo_bytes[s_total_tdo_bits / 8] |= (esp_gpio_tdo_read() << (s_total_tdo_bits % 8));
        s_total_tdo_bits++;
    }

    esp_gpio_tck_clr();
}

static void esp_usb_jtag_task(void *pvParameters)
{
    enum e_cmds {
        CMD_CLK_0 = 0, CMD_CLK_1, CMD_CLK_2, CMD_CLK_3,
        CMD_CLK_4, CMD_CLK_5, CMD_CLK_6, CMD_CLK_7,
        CMD_SRST0, CMD_SRST1, CMD_FLUSH, CMD_RSV,
        CMD_REP0, CMD_REP1, CMD_REP2, CMD_REP3
    };

    const struct {
        uint8_t tdo_req;
        uint8_t tms_tdi_mask;
    } pin_levels[] = {                          // { tdo_req, tms, tdi }
        { 0, 0 },                               // { 0, 0, 0 },  CMD_CLK_0
        { 0, GPIO_TDI_MASK },                   // { 0, 0, 1 },  CMD_CLK_1
        { 0, GPIO_TMS_MASK },                   // { 0, 1, 0 },  CMD_CLK_2
        { 0, GPIO_TMS_TDI_MASK },               // { 0, 1, 1 },  CMD_CLK_3
        { 1, 0 },                               // { 1, 0, 0 },  CMD_CLK_4
        { 1, GPIO_TDI_MASK },                   // { 1, 0, 1 },  CMD_CLK_5
        { 1, GPIO_TMS_MASK },                   // { 1, 1, 0 },  CMD_CLK_6
        { 1, GPIO_TMS_TDI_MASK },               // { 1, 1, 1 },  CMD_CLK_7
        { 0, GPIO_TMS_MASK },                   // { 0, 1, 0 },  CMD_SRST0
        { 0, GPIO_TMS_MASK },                   // { 0, 1, 0 },  CMD_SRST1
    };

    s_jtag_task_handle = xTaskGetCurrentTaskHandle();

    size_t cnt = 0;
    int prev_cmd = CMD_SRST0, rep_cnt = 0;

    while (1) {
        esp_gpio_jtag_led_off();
        char *nibbles = eub_vendord_recv_acquire_item(&cnt);
        esp_gpio_jtag_led_on();

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

            if (cmd_exec < CMD_FLUSH) {
                for (int i = 0; i < cmd_rpt_cnt; i++) {
                    do_jtag_one(pin_levels[cmd_exec].tdo_req, pin_levels[cmd_exec].tms_tdi_mask);
                }
            } else if (cmd_exec == CMD_FLUSH) {
                s_total_tdo_bits = ROUND_UP_BITS(s_total_tdo_bits);
                if (s_usb_sent_bits < s_total_tdo_bits) {
                    int waiting_to_send_bits = s_total_tdo_bits - s_usb_sent_bits;
                    while (waiting_to_send_bits > 0) {
                        int send_bits = waiting_to_send_bits > JTAG_PROTO_MAX_BITS ? JTAG_PROTO_MAX_BITS : waiting_to_send_bits;
                        eub_vendord_send_acquire_item(s_tdo_bytes + (s_usb_sent_bits / 8), send_bits / 8);
                        s_usb_sent_bits += send_bits;
                        waiting_to_send_bits -= send_bits;
                    }
                    memset(s_tdo_bytes, 0x00, sizeof(s_tdo_bytes));
                    s_total_tdo_bits = s_usb_sent_bits = 0;
                }
            }

            /* As soon as either 64 bytes (512 bits) have been collected or a CMD_FLUSH command is executed,
                make the usb buffer available for the host to receive.
            */
            int waiting_to_send_bits = s_total_tdo_bits - s_usb_sent_bits;
            if (waiting_to_send_bits >= JTAG_PROTO_MAX_BITS) {
                int send_bits = ROUND_UP_BITS(waiting_to_send_bits > JTAG_PROTO_MAX_BITS ? JTAG_PROTO_MAX_BITS : waiting_to_send_bits);
                int n_byte = send_bits / 8;
                eub_vendord_send_acquire_item(s_tdo_bytes + (s_usb_sent_bits / 8), n_byte);
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

        eub_vendord_free_item(nibbles);
    }

    vTaskDelete(NULL);
}

static void esp_usb_jtag_init(void)
{
    /* dedicated GPIO will be binded to the CPU who invokes this API */
    /* we will create a jtag task pinned to this core */
    esp_init_jtag_pins();

    BaseType_t res = xTaskCreatePinnedToCore(esp_usb_jtag_task,
                     "jtag_task",
                     4 * 1024,
                     NULL,
                     EUB_VENDORD_USB_TASK_PRI + 1,
                     NULL,
                     esp_cpu_get_core_id());
    if (res != pdPASS) {
        ESP_LOGE(TAG, "Cannot create JTAG task!");
        eub_abort();
    }
}
