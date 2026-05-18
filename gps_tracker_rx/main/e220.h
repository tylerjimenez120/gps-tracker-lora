#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "driver/gpio.h"

// ─────────────────────────────────────────────
//  Pines E220
// ─────────────────────────────────────────────
#define E220_UART_PORT      UART_NUM_1
#define E220_TX_PIN         17      // ESP32 TX → E220 RXD
#define E220_RX_PIN         16      // ESP32 RX → E220 TXD
#define E220_M0_PIN         4       // selector de modo bit0
#define E220_M1_PIN         5       // selector de modo bit1
#define E220_AUX_PIN        18      // bajo=ocupado, alto=listo
#define E220_UART_BAUD      9600
#define E220_BUF_SIZE       256

// ─────────────────────────────────────────────
//  Modos de operación — datasheet E220-900T30D
//
//  Mode 0 Normal:     M1=0 M0=0 — transmisión transparente
//  Mode 1 WOR:        M1=0 M0=1 — Wake on Radio
//  Mode 2 Config:     M1=1 M0=0 — comandos AT (solo 9600 8N1)
//  Mode 3 Deep Sleep: M1=1 M0=1 — consumo mínimo
//
//  e220_set_mode usa el valor del enum como máscara de bits:
//  bit0 → M0, bit1 → M1
// ─────────────────────────────────────────────
typedef enum {
    E220_MODE_NORMAL = 0,   // M0=0 M1=0
    E220_MODE_WOR    = 1,   // M0=1 M1=0
    E220_MODE_CONFIG = 2,   // M0=0 M1=1
    E220_MODE_SLEEP  = 3,   // M0=1 M1=1
} e220_mode_t;

// ─────────────────────────────────────────────
//  API pública
// ─────────────────────────────────────────────
bool e220_init(void);
void e220_set_mode(e220_mode_t mode);
void e220_wait_aux(void);
int  e220_send(const uint8_t *data, size_t len);
int  e220_receive(uint8_t *buf, size_t buf_size, uint32_t timeout_ms);