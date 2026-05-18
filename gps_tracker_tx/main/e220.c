#include "e220.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "E220";

// ─────────────────────────────────────────────
//  Initialization
//  Sets up the control GPIOs (M0, M1, AUX) and the UART,
//  then puts the module into Normal transparent mode
// ─────────────────────────────────────────────
bool e220_init(void) {
    // M0 and M1 select the operating mode → configured as outputs.
    // pin_bit_mask uses one bit per GPIO; OR-ing combines both pins
    // into a single config call.
    gpio_config_t m_cfg = {
        .pin_bit_mask = (1ULL << E220_M0_PIN) | (1ULL << E220_M1_PIN),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&m_cfg);

    // AUX is driven by the module → configured as input.
    // Pull-up keeps the line defined while the module boots.
    gpio_config_t aux_cfg = {
        .pin_bit_mask = (1ULL << E220_AUX_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&aux_cfg);

    // UART parameters — must match the E220 default: 9600 8N1,
    // no parity, 1 stop bit, no hardware flow control.
    uart_config_t uart_cfg = {
        .baud_rate  = E220_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    // Apply the parameters to the UART peripheral
    uart_param_config(E220_UART_PORT, &uart_cfg);
    // Assign the physical TX/RX pins (RTS/CTS unused → NO_CHANGE)
    uart_set_pin(E220_UART_PORT, E220_TX_PIN, E220_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Install the driver and allocate the internal RX buffer
    // where incoming bytes accumulate until we read them.
    uart_driver_install(E220_UART_PORT, E220_BUF_SIZE * 2, 0, 0, NULL, 0);

    // Start in Normal mode (transparent TX/RX) and wait until
    // the module signals it is ready before returning.
    e220_set_mode(E220_MODE_NORMAL);
    e220_wait_aux();

    ESP_LOGI(TAG, "E220 ready | UART%d | %d baud | TX=%d RX=%d",
             E220_UART_PORT, E220_UART_BAUD, E220_TX_PIN, E220_RX_PIN);
    return true;
}

// ─────────────────────────────────────────────
//  Change operating mode via M0 / M1
//  The enum value is used as a 2-bit mask:
//    bit0 → M0 , bit1 → M1
//  NORMAL=0(00) WOR=1(01) CONFIG=2(10) SLEEP=3(11)
// ─────────────────────────────────────────────
void e220_set_mode(e220_mode_t mode) {
    gpio_set_level(E220_M0_PIN, (mode >> 0) & 1);  // bit0 → M0
    gpio_set_level(E220_M1_PIN, (mode >> 1) & 1);  // bit1 → M1
    vTaskDelay(pdMS_TO_TICKS(10));  // let the module settle into the new mode
}

// ─────────────────────────────────────────────
//  Wait until AUX goes HIGH — module ready
//  AUX LOW  = module busy (transmitting / receiving / configuring)
//  AUX HIGH = module ready for the next operation
//
//  The while loop runs only while BOTH conditions hold:
//    AUX == 0 (busy) AND timeout > 0 (time left)
//  It exits as soon as either fails:
//    AUX == 1   → module ready
//    timeout==0 → safety exit, module never responded
// ─────────────────────────────────────────────
void e220_wait_aux(void) {
    uint32_t timeout = 3000;  // 3 second safety limit
    while (gpio_get_level(E220_AUX_PIN) == 0 && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));  // yield CPU while module is busy
        timeout -= 10;                  // count down 10ms per iteration
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // small extra margin after AUX goes high
}

// ─────────────────────────────────────────────
//  Transmit data over the E220
//  Wrapped with wait_aux before and after:
//    before → do not write while the module is busy
//    after  → block until the RF transmission finishes
// ─────────────────────────────────────────────
int e220_send(const uint8_t *data, size_t len) {
    e220_wait_aux();
    int sent = uart_write_bytes(E220_UART_PORT, data, len);
    e220_wait_aux();  // wait until the module finished transmitting
    return sent;      // returns how many bytes were written
}

// ─────────────────────────────────────────────
//  Receive data from the E220
//  Reads from the UART's internal buffer — the bytes are already
//  there, so no AUX check is needed. Returns after timeout_ms
//  or as soon as buf_size bytes are available.
// ─────────────────────────────────────────────
int e220_receive(uint8_t *buf, size_t buf_size, uint32_t timeout_ms) {
    return uart_read_bytes(E220_UART_PORT, buf, buf_size,
                           pdMS_TO_TICKS(timeout_ms));
}