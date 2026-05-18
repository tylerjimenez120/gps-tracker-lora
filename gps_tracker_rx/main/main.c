#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "e220.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "MAIN_RX";

// ─────────────────────────────────────────────
//  Statistics state — persists across packets
// ─────────────────────────────────────────────
static uint32_t s_pkt_ok   = 0;   // packets received correctly
static uint32_t s_pkt_lost = 0;   // packets lost (detected via SEQ gaps)
static int      s_last_seq = -1;  // last SEQ received, -1 = none yet

// ─────────────────────────────────────────────
//  Extract a float field from the payload
//  The payload is a string like "ID:01,LAT:-11.4187,LON:..."
//  We need the number that follows a given label (LAT:, LON:, etc)
// ─────────────────────────────────────────────
static float extract_float(const char *data, const char *key) {
    // strstr returns a pointer to where 'key' starts inside 'data'
    char *ptr = strstr(data, key);
    // key not found → return 0 to avoid dereferencing NULL
    if (!ptr) return 0.0f;
    // ptr + strlen(key) skips over the label text,
    // leaving the pointer at the numeric value;
    // atof reads chars until a non-numeric one (the comma) and converts
    return atof(ptr + strlen(key));
}

// ─────────────────────────────────────────────
//  Same as extract_float but converts to int (atoi)
//  Used for fields that are whole numbers (SAT, SEQ)
// ─────────────────────────────────────────────
static int extract_int(const char *data, const char *key) {
    char *ptr = strstr(data, key);
    if (!ptr) return 0;
    return atoi(ptr + strlen(key));
}

// ─────────────────────────────────────────────
//  Detect lost packets by analyzing SEQ numbers
//  TX increments SEQ on every transmission (1,2,3...)
//  A gap in SEQ means packets were lost over the air
// ─────────────────────────────────────────────
static void check_sequence(const char *data) {
    int seq = extract_int(data, "SEQ:");
    // SEQ:0 is never valid — TX starts counting at 1.
    // A 0 means the field was missing or the packet is corrupt
    if (seq == 0) return;

    // First condition: s_last_seq >= 0 → at least one packet arrived before
    //   (s_last_seq starts at -1, nothing to compare against on first packet)
    // Second condition: seq > s_last_seq + 1 → there is a gap
    //   if no loss, seq is always exactly previous + 1
    if (s_last_seq >= 0 && seq > s_last_seq + 1) {
        // Number of missing packets between last and current.
        // -1 because the current packet did arrive, only the gap counts.
        // e.g. last=5, seq=9 → lost = 9 - 5 - 1 = 3 (packets 6,7,8)
        int lost = seq - s_last_seq - 1;
        s_pkt_lost += lost;
        ESP_LOGW(TAG, "⚠️  %d lost | SEQ %d → %d", lost, s_last_seq, seq);
    }
    // Always update — runs whether there was loss or not,
    // keeps the tracking chain alive for the next packet
    s_last_seq = seq;
}

// ─────────────────────────────────────────────
//  RX task — receives packets, isolates them, logs GPS data
//
//  Flow:
//    read UART (200ms) → nothing? → retry
//    null-terminate raw buffer
//    find "ID:"  → not present? → retry
//    find '\n'   → if missing, use end of raw buffer
//    compute length → validate it is sane
//    copy isolated packet into pkt → null-terminate
//    count packet + check sequence
//    extract lat, lon, alt, spd, sat, seq
//    log data + Google Maps link
//    every 10 packets → print statistics
// ─────────────────────────────────────────────
static void rx_task(void *pvParameters) {
    uint8_t raw[128];   // raw bytes straight from UART (may hold junk)
    char    pkt[128];   // clean isolated single packet, ready to parse

    while (1) {
        // Read whatever is in the UART buffer.
        // sizeof(raw)-1 leaves room for the '\0' terminator.
        // 200ms timeout: long enough for a full frame, short enough
        // to avoid two packets being merged into one read.
        int len = e220_receive(raw, sizeof(raw) - 1, 200);
        // 0 = timeout with no data, <0 = error → nothing to process
        if (len <= 0) continue;

        // UART gave us raw bytes — turn them into a C string
        // by placing the terminator right after the last byte
        raw[len] = '\0';

        // The packet may not start at raw[0] — there could be leftover
        // bytes before it. Find the "ID:" marker that begins every packet.
        char *start = strstr((char *)raw, "ID:");
        if (!start) continue;  // no valid packet in this read → retry

        // The TX ends every packet with '\n' — that marks the end.
        char *end = strchr(start, '\n');
        // If '\n' is missing (packet split across reads), fall back
        // to the end of whatever data we received.
        if (!end) end = (char *)raw + len;

        // Pointer arithmetic: end - start gives the number of bytes
        // between them — the packet length, excluding the '\n'.
        int pkt_len = end - start;

        // Validate: pkt_len <= 0 means no real content;
        // pkt_len >= sizeof(pkt) would overflow the pkt buffer.
        if (pkt_len <= 0 || pkt_len >= (int)sizeof(pkt)) continue;

        // Copy only the isolated packet into pkt, then null-terminate
        // so it can be parsed safely with string functions.
        memcpy(pkt, start, pkt_len);
        pkt[pkt_len] = '\0';

        // One more valid packet received
        s_pkt_ok++;
        // Compare SEQ against the previous one to detect losses
        check_sequence(pkt);

        // Extract every GPS field from the clean packet.
        // Floats for coordinates/altitude/speed, ints for counts.
        float lat = extract_float(pkt, "LAT:");
        float lon = extract_float(pkt, "LON:");
        float alt = extract_float(pkt, "ALT:");
        float spd = extract_float(pkt, "SPD:");
        int   sat = extract_int(pkt,   "SAT:");
        int   seq = extract_int(pkt,   "SEQ:");

        ESP_LOGI(TAG, "✅ SEQ=%d | SAT=%d", seq, sat);
        ESP_LOGI(TAG, "   LAT=%.4f  LON=%.4f", lat, lon);
        ESP_LOGI(TAG, "   ALT=%.1fm  SPD=%.1f kn", alt, spd);
        // Ready-to-click Google Maps link — 6 decimals for map precision
        ESP_LOGI(TAG, "   https://maps.google.com/?q=%.6f,%.6f", lat, lon);

        // Every 10 packets print a success-rate summary
        if (s_pkt_ok % 10 == 0) {
            uint32_t total = s_pkt_ok + s_pkt_lost;
            // ternary guards against division by zero
            float rate = total > 0 ? (s_pkt_ok * 100.0f / total) : 0;
            ESP_LOGI(TAG, "📊 OK=%lu | Lost=%lu | Rate=%.1f%%",
                     s_pkt_ok, s_pkt_lost, rate);
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "=== GPS Tracker RX ===");

    // Initialize the E220 LoRa module (UART1)
    if (!e220_init()) {
        ESP_LOGE(TAG, "E220 init failed");
        return;
    }

    // Launch the receiver task — runs forever
    xTaskCreate(rx_task, "rx_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Listening for GPS packets...");
}