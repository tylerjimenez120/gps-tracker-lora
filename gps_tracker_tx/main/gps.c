#include "gps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "GPS";

// ─────────────────────────────────────────────
//  Init UART2 for GPS
//  Same 8N1 setup as the E220, only the TX buffer is 0
//  because we never send commands to the GPS module
// ─────────────────────────────────────────────
bool gps_init(void) {
    uart_config_t cfg = {
        .baud_rate  = GPS_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(GPS_UART_PORT, &cfg);
    uart_set_pin(GPS_UART_PORT, GPS_TX_PIN, GPS_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Activates the UART and creates the internal buffer where
    // incoming bytes accumulate until we read them.
    uart_driver_install(GPS_UART_PORT, GPS_BUF_SIZE * 2, 0, 0, NULL, 0);

    ESP_LOGI(TAG, "GPS UART ready | RX=GPIO%d | %d baud", GPS_RX_PIN, GPS_UART_BAUD);
    return true;
}

// ─────────────────────────────────────────────
//  Verify NMEA checksum
//  The checksum is the XOR of all bytes between $ and *
//  The GPS computes it and appends it; we recompute and compare
//  to confirm the sentence did not arrive corrupted.
// ─────────────────────────────────────────────
static bool nmea_checksum_valid(const char *sentence) {
    const char *start = strchr(sentence, '$');  // points to the '$'
    const char *star  = strchr(sentence, '*');  // points to the '*'
    // Reject if either marker is missing, or if '*' comes before
    // or right after '$' (meaning there is no data in between).
    if (!start || !star || star <= start + 1) return false;

    // Accumulate XOR of every byte strictly between '$' and '*'
    uint8_t calc = 0;
    for (const char *p = start + 1; p < star; p++) {
        calc ^= (uint8_t)*p;
    }

    // The two hex chars after '*' are the checksum the GPS sent.
    // strtol with base 16 converts that text to a number.
    uint8_t expected = (uint8_t)strtol(star + 1, NULL, 16);
    return calc == expected;
}

// ─────────────────────────────────────────────
//  Convert NMEA coordinate to decimal degrees
//  NMEA format is DDDMM.MMMM (degrees and minutes glued together),
//  NOT decimal degrees. We must split them and recombine.
//  hemisphere: 'S' or 'W' → result is negative
// ─────────────────────────────────────────────
static float nmea_to_decimal(const char *coord, char hemisphere) {
    // Guard against empty or too-short field
    if (!coord || strlen(coord) < 3) return 0.0f;

    float raw     = atof(coord);            // e.g. 1125.1234
    int   degrees = (int)(raw / 100);       // 1125.1234 / 100 → 11
    float minutes = raw - (degrees * 100);  // 1125.1234 - 1100 → 25.1234
    float decimal = degrees + minutes / 60.0f;  // 11 + 25.1234/60 → 11.4187

    // South and West are negative in standard coordinates
    if (hemisphere == 'S' || hemisphere == 'W') decimal = -decimal;
    return decimal;
}

// ─────────────────────────────────────────────
//  Split NMEA sentence into fields by comma
//  Replaces every comma with '\0' and stores a pointer to the
//  start of each field in fields[] (this modifies buf).
//  Returns the number of fields found.
// ─────────────────────────────────────────────
static int nmea_split(char *buf, char *fields[], int max_fields) {
    int count = 0;
    char *p = buf;

    while (count < max_fields) {
        // Store where this field begins (pointer into buf)
        fields[count++] = p;
        // Find the next comma
        p = strchr(p, ',');
        if (!p) break;          // no more commas → done
        // Replace comma with '\0' (terminates current field),
        // then advance p to the start of the next field
        *p++ = '\0';
    }
    return count;
}

// ─────────────────────────────────────────────
//  Parse $GPRMC sentence
//  Fills lat, lon, speed and sets data->valid.
//  Returns true only if the fix is valid (field[2] == 'A').
// ─────────────────────────────────────────────
static bool parse_gprmc(char *sentence, gps_data_t *data) {
    char  *fields[20];
    int    count = nmea_split(sentence, fields, 20);

    // Need at least 8 fields to reach the speed field
    if (count < 8) return false;

    // field[2]: 'A' = valid fix, 'V' = invalid (no fix yet)
    if (fields[2][0] != 'A') {
        data->valid = false;
        return false;
    }

    // field[3]/[4] = latitude + hemisphere, [5]/[6] = longitude + hemisphere
    data->lat         = nmea_to_decimal(fields[3], fields[4][0]);
    data->lon         = nmea_to_decimal(fields[5], fields[6][0]);
    data->speed_knots = atof(fields[7]);  // field[7] = speed in knots
    data->valid       = true;
    return true;
}

// ─────────────────────────────────────────────
//  Parse $GPGGA sentence
//  Splits the sentence into fields and extracts altitude
//  and satellite count. $GPGGA never touches data->valid.
// ─────────────────────────────────────────────
static bool parse_gpgga(char *sentence, gps_data_t *data) {
    char *fields[20];
    int   count = nmea_split(sentence, fields, 20);

    // Need at least 10 fields to reach the altitude field
    if (count < 10) return false;

    // field[6]: fix quality, '0' means no fix
    if (fields[6][0] == '0') return false;

    data->satellites = (uint8_t)atoi(fields[7]);  // field[7] = satellite count
    data->altitude   = atof(fields[9]);           // field[9] = altitude in meters
    return true;
}

// ─────────────────────────────────────────────
//  Read one complete GPS fix
//
//  Reads UART byte by byte until TWO complete sentences arrive:
//    $GPRMC → lat, lon, speed + sets data->valid (fix=A)
//    $GPGGA → altitude, satellites
//  Each sentence is accumulated byte by byte until '\n',
//  then checksum-verified and parsed into data.
//  Returns true when both sentences received and fix is valid.
//  Returns false on 10s timeout.
// ─────────────────────────────────────────────
bool gps_read(gps_data_t *data) {
    char     line[128];       // accumulator for one NMEA line
    int      pos = 0;         // current write position in line[]
    bool     got_rmc = false; // have we parsed a valid $GPRMC yet?
    bool     got_gga = false; // have we parsed a valid $GPGGA yet?
    uint8_t  byte;            // single byte read from UART
    uint32_t timeout_ms = 10000;  // maximum 10s waiting for a fix
    uint32_t elapsed    = 0;

    // Clear the struct before filling it
    memset(data, 0, sizeof(gps_data_t));

    while (elapsed < timeout_ms) {
        // Read one byte from the UART buffer, 10ms timeout
        int len = uart_read_bytes(GPS_UART_PORT, &byte, 1, pdMS_TO_TICKS(10));
        elapsed += 10;  // count down regardless of whether a byte arrived

        if (len <= 0) continue;  // no byte this round → keep waiting

        // A '\n' or '\r' means the line is complete.
        // Every other byte falls into the else branch and is accumulated.
        if (byte == '\n' || byte == '\r') {
            // Fewer than 6 chars cannot be a valid NMEA sentence → discard
            if (pos < 6) { pos = 0; continue; }

            // Turn the accumulated bytes into a proper C string
            line[pos] = '\0';
            pos = 0;  // reset for the next line

            // Only process sentences that start with '$'
            if (line[0] != '$') continue;

            // Verify checksum before doing anything with the data
            if (!nmea_checksum_valid(line)) continue;

            // Identify the sentence type by its first 6 characters.
            // $GNxxx is the multi-constellation variant (GPS+GLONASS),
            // same format as $GPxxx.
            if (strncmp(line, "$GPRMC", 6) == 0 ||
                strncmp(line, "$GNRMC", 6) == 0) {
                // Parse a COPY — nmea_split destroys the string with '\0',
                // and we want the original line intact.
                char copy[128];
                strncpy(copy, line, sizeof(copy) - 1);
                if (parse_gprmc(copy, data)) got_rmc = true;
            }
            else if (strncmp(line, "$GPGGA", 6) == 0 ||
                     strncmp(line, "$GNGGA", 6) == 0) {
                char copy[128];
                strncpy(copy, line, sizeof(copy) - 1);
                if (parse_gpgga(copy, data)) got_gga = true;
            }

            // got_rmc and got_gga are flags that accumulate across loop
            // iterations. Each sentence arrives in a separate iteration;
            // we return only once BOTH have arrived with a valid fix.
            if (got_rmc && got_gga && data->valid) return true;

        } else {
            // Accumulate this byte into the line buffer
            if (pos < (int)sizeof(line) - 1) {
                line[pos++] = (char)byte;
            } else {
                pos = 0;  // line too long → discard and start over
            }
        }
    }

    ESP_LOGW(TAG, "GPS timeout — no valid fix in %lums", (unsigned long)timeout_ms);
    return false;
}