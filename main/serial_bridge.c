/*
 * SPDX-FileCopyrightText: 2020-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <sys/param.h>

#include "serial_bridge.h"
#include "serial_handler.h"
#include "tusb_config.h"
#include "tusb.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "util.h"
#include "debug_probe.h"
#include "usb_phy.h"
#include "soc/rtc_cntl_reg.h"

#define USB_SEND_RINGBUFFER_SIZE (16 * 1024)
/* tud_cdc_n_get_line_state(): bit 0 = DTR, bit 1 = RTS */
#define CDC_LINE_STATE_DTR (1u << 0)

static const char *TAG = "serial_bridge";

static RingbufHandle_t usb_sendbuf;
static SemaphoreHandle_t usb_tx_done = NULL;
static esp_timer_handle_t state_change_timer;
static bool download_mode_armed;

// Transport data received callback - called by serial handler when data arrives
static void transport_data_received_callback(const uint8_t *data, size_t len)
{
    // With the new API, the callback is only called when bridge mode is active
    // (i.e., when flashing is not in progress), so we don't need to check mode
    ESP_LOGD(TAG, "Transport -> USB ringbuffer (%zu bytes)", len);
    ESP_LOG_BUFFER_HEXDUMP("Transport -> USB", data, len, ESP_LOG_DEBUG);

    // Retry rather than drop. Discarding here silently corrupts a bulk transfer,
    // which is what held the target -> host path to 115200. Bounded so a host
    // that has stopped reading cannot wedge the UART task indefinitely.
    for (int retry = 0; retry < 10; retry++) {
        if (xRingbufferSend(usb_sendbuf, data, len, pdMS_TO_TICKS(10)) == pdTRUE) {
            return;
        }
    }
    ESP_LOGW(TAG, "USB send ringbuffer full, dropped %zu bytes", len);
}

static void usb_sender_task(void *pvParameters)
{
    while (1) {
        size_t ringbuf_received;
        uint8_t *buf = xRingbufferReceiveUpTo(usb_sendbuf, &ringbuf_received, portMAX_DELAY,
                                              CFG_TUD_CDC_TX_BUFSIZE);
        if (!buf) {
            continue;
        }

        uint8_t int_buf[CFG_TUD_CDC_TX_BUFSIZE];
        memcpy(int_buf, buf, ringbuf_received);
        vRingbufferReturnItem(usb_sendbuf, (void *) buf);

        // Fill the CDC FIFO as space frees. Nothing is discarded here: back-pressure
        // belongs in the ringbuffer and then the UART, not in lost bytes.
        size_t sent = 0;
        while (sent < ringbuf_received) {
            const uint32_t avail = tud_cdc_write_available();
            if (avail == 0) {
                tud_cdc_write_flush();
                if (xSemaphoreTake(usb_tx_done, pdMS_TO_TICKS(100)) != pdTRUE && !tud_mounted()) {
                    ESP_LOGW(TAG, "no host, dropped %u bytes", (unsigned)(ringbuf_received - sent));
                    break;
                }
                continue;
            }
            sent += tud_cdc_write(int_buf + sent, MIN(ringbuf_received - sent, avail));
        }
        tud_cdc_write_flush();
    }
    vTaskDelete(NULL);
}

// Signals only that TX FIFO space may have freed; no per-write accounting to slip.
void tud_cdc_tx_complete_cb(const uint8_t itf)
{
    xSemaphoreGive(usb_tx_done);
}

void tud_cdc_rx_cb(const uint8_t itf)
{
    uint8_t buf[CFG_TUD_CDC_RX_BUFSIZE];

    const uint32_t rx_size = tud_cdc_n_read(itf, buf, CFG_TUD_CDC_RX_BUFSIZE);
    if (rx_size > 0) {
        ESP_LOGD(TAG, "USB CDC -> Transport (%" PRIu32 " bytes)", rx_size);
        ESP_LOG_BUFFER_HEXDUMP("USB CDC -> Transport", buf, rx_size, ESP_LOG_DEBUG);

        // Send to transport (could be UART, SPI, I2C, etc.)
        serial_handler_send_data(buf, rx_size);
    } else {
        ESP_LOGW(TAG, "tud_cdc_rx_cb receive error");
    }
}

static void enter_download_mode(void);

void tud_cdc_line_coding_cb(const uint8_t itf, cdc_line_coding_t const *p_line_coding)
{
    if (CONFIG_BRIDGE_DOWNLOAD_MAGIC_BAUD > 0 && p_line_coding->bit_rate == CONFIG_BRIDGE_DOWNLOAD_MAGIC_BAUD) {
        ESP_LOGI(TAG, "Magic baud %" PRIu32 " detected, arming download mode reset",
                 p_line_coding->bit_rate);
        download_mode_armed = true;
        // Fire immediately if the host already cleared DTR (e.g. baud change on a closed port).
        if (!(tud_cdc_n_get_line_state(itf) & CDC_LINE_STATE_DTR)) {
            enter_download_mode();
        }
        return;
    }

    download_mode_armed = false;
    if (serial_handler_set_baudrate(p_line_coding->bit_rate) != ESP_OK) {
        ESP_LOGE(TAG, "Could not set the baudrate to %" PRIu32, p_line_coding->bit_rate);
        eub_abort();
    }
}

void tud_cdc_line_state_cb(const uint8_t itf, const bool dtr, const bool rts)
{
    // Magic baud arms download mode; clearing DTR (typically on port close) fires it.
    if (download_mode_armed && !dtr) {
        enter_download_mode();
        return;
    }

    // The following transformation of DTR & RTS signals to BOOT & RST is done based on auto reset circutry shown in
    // schematics of ESP boards.

    // defaults for ((dtr && rts) || (!dtr && !rts))
    bool rst = true;
    bool boot = true;

    if (!dtr && rts) {
        rst = false;
        boot = true;
    } else if (dtr && !rts) {
        rst = true;
        boot = false;
    }

    esp_timer_stop(state_change_timer);  // maybe it is not started so not check the exit value

    if (dtr & rts) {
        // The assignment of BOOT=1 and RST=1 is postponed and it is done only if no other state change occurs in time
        // period set by the timer.
        // This is a patch for Esptool. Esptool generates DTR=0 & RTS=1 followed by DTR=1 & RTS=0. However, a callback
        // with DTR = 1 & RTS = 1 is received between. This would prevent to put the target chip into download mode.
        ESP_ERROR_CHECK(esp_timer_start_once(state_change_timer, 10 * 1000 /*us*/));

    } else {
        ESP_LOGI(TAG, "DTR = %d, RTS = %d -> BOOT = %d, RST = %d", dtr, rts, boot, rst);

        serial_handler_set_boot_reset_pins(boot, rst);

        if (!rst) {
            const uint32_t default_baud = 115200;
            if (serial_handler_set_baudrate(default_baud) != ESP_OK) {
                eub_abort();
            }
        }

        // On ESP32, TDI jtag signal is on GPIO12, which is also a strapping pin that determines flash voltage.
        // If TDI is high when ESP32 is released from external reset, the flash voltage is set to 1.8V, and the chip will fail to boot.
        // As a solution, MTDI signal forced to be low when RST is about to go high.
        if (boot) {
            debug_probe_handle_esp32_tdi_bootstrapping(!rst);
        }
    }
}

static void state_change_timer_cb(void *arg)
{
    ESP_LOGI(TAG, "BOOT = 1, RST = 1");
    serial_handler_set_boot_reset_pins(true, true); // BOOT=1, RST=1 (not in reset)
}

static void enter_download_mode(void)
{
    if (CONFIG_BRIDGE_DOWNLOAD_MAGIC_BAUD <= 0) {
        return;
    }

    ESP_LOGI(TAG, "Forcing download mode and resetting bridge chip");
    // Tear down USB before restart; avoid logging after this point (PHY/console go away).
    // Called from tusb_device_task (CDC callbacks); esp_restart() never returns to tud_task.
    eub_usb_phy_deinit();
    REG_SET_BIT(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
    esp_restart();
}

static void init_state_change_timer(void)
{
    const esp_timer_create_args_t timer_args = {
        .callback = state_change_timer_cb,
        .name = "serial_bridge_state_change"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &state_change_timer));
}

esp_err_t serial_bridge_init(void)
{
    // Create ring buffer for USB sending
    usb_sendbuf = xRingbufferCreate(USB_SEND_RINGBUFFER_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (!usb_sendbuf) {
        ESP_LOGE(TAG, "Cannot create ringbuffer for USB sender");
        return ESP_ERR_NO_MEM;
    }

    // Create semaphores for USB TX synchronization
    usb_tx_done = xSemaphoreCreateBinary();
    if (!usb_tx_done) {
        ESP_LOGE(TAG, "Cannot create USB TX semaphore");
        return ESP_ERR_NO_MEM;
    }

    // Register callback for transport data
    esp_err_t ret = serial_handler_register_data_callback(transport_data_received_callback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register transport data callback");
        return ret;
    }

    // Initialize state change timer
    init_state_change_timer();

    // Start USB sender task
    xTaskCreate(usb_sender_task, "usb_sender_task", 4 * 1024, NULL, SERIAL_HANDLER_TASK_PRI, NULL);

    ESP_LOGI(TAG, "Serial bridge initialized");
    return ESP_OK;
}
