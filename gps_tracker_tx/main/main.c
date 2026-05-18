#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "gps.h"
#include "e220.h"
#include <stdio.h>

// ─────────────────────────────────────────────
//  GPS Tracker TX
//
//  app_main → initializes GPS (UART2) and E220 (UART1) → creates tx_task
//
//  tx_task every 5s:
//    gps_read   → waits for a valid fix (lat, lon, alt, sat)
//    build payload → "ID:01,LAT:...,LON:...,SEQ:N\n"
//    e220_send  → transmits over 900 MHz LoRa
// ─────────────────────────────────────────────

static const char *TAG  = "MAIN_TX";
#define TX_INTERVAL_MS    5000   // delay between transmissions
#define NODE_ID           0x01   // identifier of this tracker node

static void tx_task(void *pvParameters) {
    char       payload[96];  // buffer where the payload string is built
    uint8_t    seq = 0;      // sequence counter, increments per packet
    gps_data_t gps;          // holds the parsed GPS fix

    while (1) {
        seq++;

        // Read GPS — blocks up to 10s waiting for a valid fix.
        // If no fix, log a warning, skip this cycle and retry later.
        if (!gps_read(&gps)) {
            ESP_LOGW(TAG, "No GPS fix | SEQ=%d — skipping TX", seq);
            vTaskDelay(pdMS_TO_TICKS(TX_INTERVAL_MS));
            continue;
        }

        // Build the payload string. The trailing '\n' is the delimiter
        // the receiver uses to know where the packet ends.
        int len = snprintf(payload, sizeof(payload),
                           "ID:%02X,LAT:%.4f,LON:%.4f,ALT:%.1f,SPD:%.1f,SAT:%d,SEQ:%d\n",
                           NODE_ID,
                           gps.lat,
                           gps.lon,
                           gps.altitude,
                           gps.speed_knots,
                           gps.satellites,
                           seq);

        // %.*s with len-1 prints the payload without the trailing '\n'
        // so the log line does not break onto an extra row.
        ESP_LOGI(TAG, "TX → '%.*s'", len - 1, payload);

        // Transmit over the E220. sent = number of bytes actually written.
        int sent = e220_send((uint8_t *)payload, len);
        if (sent == len) {
            ESP_LOGI(TAG, "✅ SEQ=%d | LAT=%.4f LON=%.4f ALT=%.1fm SAT=%d",
                     seq, gps.lat, gps.lon, gps.altitude, gps.satellites);
        } else {
            ESP_LOGE(TAG, "❌ TX error | SEQ=%d | sent=%d/%d", seq, sent, len);
        }

        // Wait before the next cycle
        vTaskDelay(pdMS_TO_TICKS(TX_INTERVAL_MS));
    }
}


void app_main(void) {
    ESP_LOGI(TAG, "=== GPS Tracker TX ===");
    ESP_LOGI(TAG, "GPS UART2 RX=GPIO%d | E220 UART1 TX=GPIO%d RX=GPIO%d",
             GPS_RX_PIN, E220_TX_PIN, E220_RX_PIN);

    // Initialize the GPS module (UART2)
    if (!gps_init()) {
        ESP_LOGE(TAG, "GPS init failed");
        return;
    }

    // Initialize the E220 LoRa module (UART1)
    if (!e220_init()) {
        ESP_LOGE(TAG, "E220 init failed");
        return;
    }

    ESP_LOGI(TAG, "Waiting for GPS fix...");
    // Launch the transmitter task — runs forever
    xTaskCreate(tx_task, "tx_task", 4096, NULL, 5, NULL);
}