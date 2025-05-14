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

#if CONFIG_BRIDGE_DEBUG_IFACE_SWD

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "tusb.h"
#include "eub_vendord.h"
#include "util.h"
#include "esp_io.h"
#include "DAP_config.h"
#include "DAP.h"

static const char *TAG = "eub_swd";

static TaskHandle_t s_swd_task_handle = NULL;

void eub_debug_probe_init(void) __attribute__((alias("eub_swd_init")));
int eub_debug_probe_get_proto_caps(void *dest) __attribute__((alias("eub_swd_get_proto_caps")));
void eub_debug_probe_task_suspend(void) __attribute__((alias("eub_swd_task_suspend")));
void eub_debug_probe_task_resume(void) __attribute__((alias("eub_swd_task_resume")));
bool eub_debug_probe_target_is_esp32(void) __attribute__((alias("eub_swd_target_is_esp32")));

static int eub_swd_get_proto_caps(void *dest)
{
    return 0;
}

static bool eub_swd_target_is_esp32(void)
{
    return false;
}

static void eub_swd_task_suspend(void)
{
    if (s_swd_task_handle) {
        vTaskSuspend(s_swd_task_handle);
    }
}

static void eub_swd_task_resume(void)
{
    if (s_swd_task_handle) {
        vTaskResume(s_swd_task_handle);
    }
}

bool eub_debug_probe_control_handler(const uint8_t rhport, const uint8_t stage, tusb_control_request_t const *request)
{
    // Function to handle the TUSB_REQ_TYPE_VENDOR control requests from the host,
    // called by the tusb_control_xfer_cb function in eub_vendord.c.
    return false;
}

static void swd_task(void *pvParameters)
{
    size_t total_bytes = 0;
    uint8_t response[DAP_PACKET_SIZE];

    while (1) {
        uint8_t *request = eub_vendord_recv_acquire_item(&total_bytes);

        ESP_LOG_BUFFER_HEXDUMP("req", request, total_bytes, ESP_LOG_DEBUG);

        uint32_t resp_len = DAP_ProcessCommand(request, response) & 0xFFFF; //lower 16 bits are response len

        ESP_LOG_BUFFER_HEXDUMP("res", response, resp_len, ESP_LOG_DEBUG);

        eub_vendord_send_item(response, resp_len);

        eub_vendord_free_item(request);
    }

    vTaskDelete(NULL);
}

static void eub_swd_init(void)
{
    DAP_Setup();

    BaseType_t res = xTaskCreatePinnedToCore(swd_task,
                     "swd_task",
                     4 * 1024,
                     NULL,
                     EUB_VENDORD_USB_TASK_PRI + 1,
                     &s_swd_task_handle,
                     esp_cpu_get_core_id());
    if (res != pdPASS) {
        ESP_LOGE(TAG, "Cannot create SWD task!");
        eub_abort();
    }
}

#endif /* CONFIG_BRIDGE_DEBUG_IFACE_SWD */
