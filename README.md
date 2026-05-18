# GPS Tracker over LoRa — ESP32 + NEO-6M + E220-900T30D

A GPS tracker built from scratch with no external libraries. An ESP32 reads
real coordinates from a NEO-6M GPS module, transmits them over 900 MHz LoRa
using an E220-900T30D, and a second ESP32 receives, decodes, and prints a
ready-to-click Google Maps link.

Fourth lab in a wireless communication series:
bare 433 MHz RF protocol → SX1278 LoRa driver → E220 sensor node → GPS tracker.
Each lab reuses the previous one's drivers.

## Hardware

- 2x ESP32
- 1x GY-GPS6MV2 (u-blox NEO-6M)
- 2x E220-900T30D (LLCC68 LoRa, 900 MHz, 30 dBm)

## Wiring — TX node

| GPS (NEO-6M) | ESP32   |
|--------------|---------|
| VCC          | 5V      |
| GND          | GND     |
| TX           | GPIO34  |

| E220   | ESP32   |
|--------|---------|
| VCC    | 3.3V    |
| GND    | GND     |
| RXD    | GPIO17  |
| TXD    | GPIO16  |
| M0     | GPIO4   |
| M1     | GPIO5   |
| AUX    | GPIO18  |

## Wiring — RX node

Only the E220, wired exactly as on the TX node.

## How it works

The NEO-6M streams NMEA sentences over UART. The firmware parses them
byte by byte:

1. Accumulate bytes into a line buffer until `\n`
2. Verify the XOR checksum between `$` and `*`
3. Split the sentence into fields by comma
4. Validate the fix flag (`A` = valid, `V` = no fix)
5. Convert `DDDMM.MMMM` to decimal degrees
6. Build a payload and transmit over LoRa

`$GPRMC` provides latitude, longitude and speed. `$GPGGA` provides altitude
and satellite count. Both are required before a reading is sent.

The TX node uses all three ESP32 UARTs: UART0 for the debug monitor,
UART1 for the E220, UART2 for the GPS.

## Payload format

```
ID:01,LAT:-12.0228,LON:-77.0737,ALT:107.5,SPD:0.5,SAT:5,SEQ:15
```

The trailing `\n` is the delimiter the receiver uses to isolate packets.

## Build and flash

```bash
cd gps_tracker_tx
idf.py build flash monitor
```

```bash
cd gps_tracker_rx
idf.py build flash monitor
```

The GPS cold start can take a few minutes — place the module near a window
with the antenna facing up. A blinking blue LED means it has a fix.

## Project structure

```
gps_tracker_tx/
  main/
    main.c      transmitter task
    gps.h/.c    NEO-6M NMEA parser
    e220.h/.c   E220 LoRa driver (reused from previous lab)
gps_tracker_rx/
  main/
    main.c      receiver task + statistics
    e220.h/.c   E220 LoRa driver (reused)
```

## Result

With a 5-satellite fix the receiver prints stable coordinates with zero
packet loss, and the Google Maps link drops a pin at the tracker's location.

## Stack

ESP-IDF v5.5 | FreeRTOS | UART driver | NEO-6M | E220-900T30D | no external libraries
