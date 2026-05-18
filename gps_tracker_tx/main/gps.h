#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"

// ─────────────────────────────────────────────
//  GPS UART pins
//  UART0 = debug monitor, UART1 = E220, UART2 = GPS
// ─────────────────────────────────────────────
#define GPS_UART_PORT   UART_NUM_2  // third hardware UART of the ESP32
#define GPS_TX_PIN      12          // ESP32 TX → GPS RX (config only, unused here)
#define GPS_RX_PIN      34          // GPS TX → ESP32 RX (data) — GPIO34 is input-only
#define GPS_UART_BAUD   9600        // NEO-6M default baud rate
#define GPS_BUF_SIZE    512         // UART RX buffer — large, GPS emits many sentences/sec

// ─────────────────────────────────────────────
//  GPS data struct — final parsed result,
//  only the useful values already converted
// ─────────────────────────────────────────────
typedef struct {
    float   lat;          // decimal degrees, negative = South
    float   lon;          // decimal degrees, negative = West
    float   altitude;     // meters above sea level (from $GPGGA)
    float   speed_knots;  // speed in knots (from $GPRMC)
    uint8_t satellites;   // number of satellites used in the fix (from $GPGGA)
    bool    valid;        // true only when fix is active (field A in $GPRMC)
} gps_data_t;

// ─────────────────────────────────────────────
//  API
// ─────────────────────────────────────────────
bool gps_init(void);               // configures UART2 for the GPS module
bool gps_read(gps_data_t *data);   // blocks until valid sentence or 10s timeout