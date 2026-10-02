// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The few places where the Arduino cores differ, kept out of the services.
//
// GPIO modes: ESP32 spells open drain OUTPUT_OPEN_DRAIN and lets INPUT_PULLUP |
// INPUT_PULLDOWN mean both pulls; the RP2040/RP2350 core has neither, so open
// drain is emulated (release = input, low = driven) and both pulls go through
// the SDK. UART: ESP32 takes the pins in begin(), the RP2 core takes them from
// setRX/setTX beforehand, and the two spell their buffer sizing differently.
#pragma once

#include <Arduino.h>

#include "Oep.h"

#if defined(ARDUINO_ARCH_RP2040)
#include <hardware/gpio.h>
#include <pico/unique_id.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <driver/gpio.h>
#endif

namespace oep {

// Wire values of fixture.gpio's mode byte.
enum FixtureGpioMode : uint8_t {
  kGpioInputFloating = 0, kGpioInputPullUp = 1, kGpioInputPullDown = 2, kGpioInputPullUpDown = 3,
  kGpioOutputLow = 4, kGpioOutputHigh = 5, kGpioOpenDrainLow = 6, kGpioOpenDrainRelease = 7,
};

// The UART type a FixtureUart instance drives: the RP2 core's pin setters live
// on SerialUART, not on HardwareSerial.
#if defined(ARDUINO_ARCH_RP2040)
using OepUart = SerialUART;
#else
using OepUart = HardwareSerial;
#endif

// Park every pin the probe does not own as a floating input. A probe must not drag a
// target's line anywhere it was not asked to: the RP2 pad comes out of reset with a
// pull-down, and on the CH32L103 jig (2026-09-23) that was enough to leave the target's
// debug module reachable but its hart unhaltable, because the half-wired header brings a
// reset line to one of these pads. Called by the probe sketches before any service runs.
inline void platformParkPins(const uint8_t *pins, size_t count);

inline void platformPresetLevel(int pin, int level) {
#if defined(ARDUINO_ARCH_ESP32)
  gpio_set_level(static_cast<gpio_num_t>(pin), level);
#else
  (void)pin; (void)level;
#endif
}

inline void platformGpio(int pin, uint8_t mode) {
#if defined(ARDUINO_ARCH_RP2040)
  switch (mode) {
    case kGpioInputFloating: pinMode(pin, INPUT); break;
    case kGpioInputPullUp: pinMode(pin, INPUT_PULLUP); break;
    case kGpioInputPullDown: pinMode(pin, INPUT_PULLDOWN); break;
    case kGpioInputPullUpDown: pinMode(pin, INPUT); gpio_set_pulls(pin, true, true); break;
    // The level goes into the output latch before the output is enabled: the other order drives whatever the
    // latch held last, which on a reset line can be a brief high. gpio_put, not digitalWrite: after pinMode
    // INPUT_PULLUP / INPUT_PULLDOWN arduino-pico's digitalWrite only turns the output on or off and leaves the latch
    // at the 0 / 1 those modes put there, so a pull-up pin set to output high would come up driven low.
    case kGpioOutputLow: gpio_put(pin, 0); pinMode(pin, OUTPUT); break;
    case kGpioOutputHigh: gpio_put(pin, 1); pinMode(pin, OUTPUT); break;
    // No open-drain output on the RP2 pad: release as an input, drive the low.
    case kGpioOpenDrainLow: gpio_put(pin, 0); pinMode(pin, OUTPUT); break;
    case kGpioOpenDrainRelease: pinMode(pin, INPUT); break;
    default: break;
  }
#else
  switch (mode) {
    case kGpioInputFloating: pinMode(pin, INPUT); break;
    case kGpioInputPullUp: pinMode(pin, INPUT_PULLUP); break;
    case kGpioInputPullDown: pinMode(pin, INPUT_PULLDOWN); break;
    case kGpioInputPullUpDown: pinMode(pin, INPUT_PULLUP | INPUT_PULLDOWN); break;
    // The level into the output register first (gpio_set_level: digitalWrite refuses a pin not yet set up as a GPIO),
    // then the output on: an output idle (probe.config §1) on a power switch comes up at its level, no pulse.
    case kGpioOutputLow: platformPresetLevel(pin, LOW); pinMode(pin, OUTPUT); digitalWrite(pin, LOW); break;
    case kGpioOutputHigh: platformPresetLevel(pin, HIGH); pinMode(pin, OUTPUT); digitalWrite(pin, HIGH); break;
    case kGpioOpenDrainLow: pinMode(pin, OUTPUT_OPEN_DRAIN); digitalWrite(pin, LOW); break;
    case kGpioOpenDrainRelease: pinMode(pin, OUTPUT_OPEN_DRAIN); digitalWrite(pin, HIGH); break;
    default: break;
  }
#endif
}

// Output drive strengths (oep-if-fixture §1.1): the levels a fixture gpio output (mode 3 / 4, an output idle) can be
// driven at, ascending, with their approximate mA, and the default level - the pad's own reset strength, so a pin
// nobody set is already at it. count 0: this chip's strength is not switched (no drive_levels in describe).
//   classic ESP32, ESP32-P4: gpio_drive_cap_t GPIO_DRIVE_CAP_0..3, about 5 / 10 / 20 / 40 mA (the datasheets' typical
//                            source currents); GPIO_DRIVE_CAP_DEFAULT is 2.
//   RP2040 / RP2350: the pad's 2 / 4 / 8 / 12 mA (enum gpio_drive_strength 0..3); 4 mA after reset.
// Other chips declare none. Debug wires and the other fixtures never go through these: their PHYs and drivers set the
// strength they need (the wires the weakest).
struct DriveLevels {
  const uint16_t *ma;
  uint8_t count;
  uint8_t default_level;
};
inline DriveLevels platformDriveLevels() {
#if defined(OEP_HOST_FAKE_DRIVE) || \
    (defined(ARDUINO_ARCH_ESP32) && (defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32P4)))
  static const uint16_t kMa[] = {5, 10, 20, 40};
  return {kMa, 4, 2};
#elif defined(ARDUINO_ARCH_RP2040)
  static const uint16_t kMa[] = {2, 4, 8, 12};
  return {kMa, 4, 1};
#else
  return {nullptr, 0, 0};
#endif
}

// The pad's strength alone (a level of platformDriveLevels).
inline void platformDrive(int pin, uint8_t level) {
#if defined(OEP_HOST_FAKE_DRIVE)
  if (pin >= 0 && pin < 64) g_pin_drive[pin] = level;
#elif defined(ARDUINO_ARCH_ESP32) && (defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32P4))
  gpio_set_drive_capability(static_cast<gpio_num_t>(pin), static_cast<gpio_drive_cap_t>(level));
#elif defined(ARDUINO_ARCH_RP2040)
  gpio_set_drive_strength(pin, static_cast<gpio_drive_strength>(level));
#else
  (void)pin; (void)level;
#endif
}

// Output low / high (kGpioOutputLow / kGpioOutputHigh) at a level: the strength is in place before the output is
// enabled, so no edge goes out at another one. arduino-pico's pinMode(OUTPUT) sets 4 mA itself, so the RP2 takes its
// OUTPUT_xMA mode; on the ESP32 the strength is set again after pinMode (a peripheral the pin left may reset it).
inline void platformGpioDriven(int pin, uint8_t mode, uint8_t level) {
#if defined(ARDUINO_ARCH_RP2040) && !defined(OEP_HOST_FAKE_DRIVE)
  static const PinMode kOutput[] = {OUTPUT_2MA, OUTPUT_4MA, OUTPUT_8MA, OUTPUT_12MA};
  gpio_put(pin, mode == kGpioOutputHigh);   // the latch first, as platformGpio
  pinMode(pin, kOutput[level & 3]);
#else
  platformDrive(pin, level);
  platformGpio(pin, mode);
  platformDrive(pin, level);
#endif
}

inline void platformParkPins(const uint8_t *pins, size_t count) {
  for (size_t i = 0; i < count; ++i) platformGpio(pins[i], kGpioInputFloating);
}

// The same, for every channel set in `mask`.
// Pins this very chip uses itself and a sketch must never touch, found at start-up from what the chip says it is: one
// firmware image runs on every board of a chip family, so the pin list cannot be fixed at build time. ESP32: the
// ESP32-PICO-D4 / PICO-V3 / PICO-V3-02 and the D2WD wire their in-package flash to GPIO16 / 17 (always taken), and a
// PSRAM in use (WROVER, D0WDR2) takes them too - only when the build enabled it: psramFound() is false on a build with
// PSRAM off, and the pins are then free. Parking them Hi-Z crashed the PICO-D4 of an M5Stack ATOM at boot (TG1WDT,
// 2026-10-01). Other chips: none found so far.
inline uint64_t platformUnusablePins() {
#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32)
  const char *model = ESP.getChipModel();
  const bool flash = model && (strstr(model, "PICO") || strstr(model, "D2WD"));
  if (flash || psramFound()) return (uint64_t{1} << 16) | (uint64_t{1} << 17);
#endif
  return 0;
}

inline void platformParkMask(uint64_t mask) {
  for (int pin = 0; pin < 64; ++pin) if ((mask >> pin) & 1) platformGpio(pin, kGpioInputFloating);
}

// This probe's own unit id, the same on any transport: the eFuse MAC on the ESP32 family, the flash's unique id on
// the RP2040 / RP2350. -> bytes written (at most `capacity`).
// The unit id (oep-core §7.5): the chip's own number as lowercase hex text - the RP2's flash unique id (16), the
// ESP32's base MAC (12) - the same on every transport, and the USB serial number (core §3.3). Needs 17 bytes of out
// with the 0 after it. -> the text's length.
inline size_t platformUnitId(uint8_t *out, size_t capacity) {
  uint8_t raw[8];
  size_t n = 0;
#if defined(ARDUINO_ARCH_RP2040)
  pico_unique_board_id_t id;
  pico_get_unique_board_id(&id);
  n = sizeof id.id < sizeof raw ? sizeof id.id : sizeof raw;
  memcpy(raw, id.id, n);
#elif defined(ARDUINO_ARCH_ESP32)
  const uint64_t mac = ESP.getEfuseMac();
  n = 6;
  for (size_t i = 0; i < n; ++i) raw[i] = static_cast<uint8_t>(mac >> (8 * i));
#endif
  static const char kHex[] = "0123456789abcdef";
  size_t at = 0;
  for (size_t i = 0; i < n && at + 2 < capacity; ++i) {
    out[at++] = static_cast<uint8_t>(kHex[raw[i] >> 4]);
    out[at++] = static_cast<uint8_t>(kHex[raw[i] & 15]);
  }
  if (at < capacity) out[at] = 0;
  return at;
}

// The MCU's part and revision for oep.core's describe chip (core §7.5): "esp32p4 v1.0", "rp2350 v2" (the SDK's chip
// version number, not a stepping letter). -> bytes written, 0 when unknown.
inline size_t platformChip(char *out, size_t room) {
  int n = 0;
#if defined(ARDUINO_ARCH_RP2040)
#if defined(PICO_RP2350)
  n = snprintf(out, room, "rp2350 v%u", static_cast<unsigned>(rp2350_chip_version()));
#else
  n = snprintf(out, room, "rp2040 v%u", static_cast<unsigned>(rp2040_chip_version()));
#endif
#elif defined(ARDUINO_ARCH_ESP32)
  const unsigned rev = ESP.getChipRevision();   // major x 100 + minor
  // the part lowercase without its hyphens (core §7.5): "ESP32-P4" -> "esp32p4", "ESP32-D0WD-V3" -> "esp32d0wdv3"
  const char *model = ESP.getChipModel();
  size_t at = 0;
  for (const char *c = model; *c && at + 1 < room; ++c)
    if (*c != '-') out[at++] = (*c >= 'A' && *c <= 'Z') ? static_cast<char>(*c - 'A' + 'a') : *c;
  n = static_cast<int>(at) + snprintf(out + at, room > at ? room - at : 0, " v%u.%u", rev / 100, rev % 100);
#else
  (void)out; (void)room;
#endif
  return n < 0 ? 0 : (static_cast<size_t>(n) < room ? static_cast<size_t>(n) : room);
}

// oep.core's describe chip (core §7.5, optional): a sketch's describe calls it after describeCore.
inline bool describeChip(TlvWriter &w) {
  char chip[32];
  const size_t n = platformChip(chip, sizeof chip);
  return n ? w.put(reg::core::kTlvDescribeChip, chip, n) : true;
}

// A hardware random number (the endpoint's boot id, core §7.1).
inline uint32_t platformRandom32() {
#if defined(ARDUINO_ARCH_RP2040)
  return rp2040.hwrand32();
#elif defined(ARDUINO_ARCH_ESP32)
  return esp_random();
#else
  return static_cast<uint32_t>(micros());
#endif
}

// Sizes are hints: a core that cannot resize its buffers keeps its default.
inline void platformUartBuffers(OepUart &serial, size_t rx, size_t tx) {
#if defined(ARDUINO_ARCH_RP2040)
  (void)tx;
  serial.setFIFOSize(rx);   // must precede begin()
#elif defined(ARDUINO_ARCH_ESP32)
  serial.setRxBufferSize(rx);
  serial.setTxBufferSize(tx);
#else
  (void)serial; (void)rx; (void)tx;
#endif
}

// config: the core's SERIAL_xyz frame format (8N1 unless told otherwise).
#if defined(ARDUINO_ARCH_RP2040)
// The pins an RP2 UART (uart_index 0 = Serial1, 1 = Serial2) reaches as RX / TX, per chip (arduino-pico's SerialUART
// tables). Anything else makes the core panic() in setRX / setTX, so the fixture must never ask.
inline uint64_t platformUartMask(const uint8_t *pins, size_t n) {
  uint64_t m = 0;
  for (size_t i = 0; i < n; ++i) m |= uint64_t{1} << pins[i];
  return m;
}
inline uint64_t platformUartRxMask(int uart_index) {
#if defined(PICO_RP2350) && !PICO_RP2350A
  static const uint8_t k0[] = {1, 3, 13, 15, 17, 19, 29, 31, 33, 35, 45, 47}, k1[] = {5, 7, 9, 11, 21, 23, 25, 27, 37, 39, 41, 43};
#elif defined(PICO_RP2350)
  static const uint8_t k0[] = {1, 3, 13, 15, 17, 19, 29}, k1[] = {5, 7, 9, 11, 21, 23, 25, 27};
#else
  static const uint8_t k0[] = {1, 13, 17, 29}, k1[] = {5, 9, 21, 25};
#endif
  return uart_index ? platformUartMask(k1, sizeof k1) : platformUartMask(k0, sizeof k0);
}
inline uint64_t platformUartTxMask(int uart_index) {
#if defined(PICO_RP2350) && !PICO_RP2350A
  static const uint8_t k0[] = {0, 2, 12, 14, 16, 18, 28, 30, 32, 34, 44, 46}, k1[] = {4, 6, 8, 10, 20, 22, 24, 26, 36, 38, 40, 42};
#elif defined(PICO_RP2350)
  static const uint8_t k0[] = {0, 2, 12, 14, 16, 18, 28}, k1[] = {4, 6, 8, 10, 20, 22, 24, 26};
#else
  static const uint8_t k0[] = {0, 12, 16, 28}, k1[] = {4, 8, 20, 24};
#endif
  return uart_index ? platformUartMask(k1, sizeof k1) : platformUartMask(k0, sizeof k0);
}
#endif

inline bool platformUartBegin(OepUart &serial, uint32_t baud, int rx, int tx, uint32_t config = SERIAL_8N1) {
#if defined(ARDUINO_ARCH_RP2040)
  // Only pins the UART reaches (setRX / setTX panic otherwise; the fixture's role masks keep them out upstream).
  const int index = &serial == &Serial1 ? 0 : 1;   // Serial1 = uart0, Serial2 = uart1 in arduino-pico
  if (rx < 0 || tx < 0 || rx > 63 || tx > 63) return false;
  if (!((platformUartRxMask(index) >> rx) & 1) || !((platformUartTxMask(index) >> tx) & 1)) return false;
  if (!serial.setRX(rx) || !serial.setTX(tx)) return false;
  serial.begin(baud, static_cast<uint16_t>(config));
  return true;
#elif defined(ARDUINO_ARCH_ESP32)
  serial.begin(baud, config, rx, tx);
  return true;
#else
  (void)serial; (void)baud; (void)rx; (void)tx; (void)config;
  return false;
#endif
}

// The rate the UART actually runs at (its divider), where the core can tell; else the one asked for.
inline uint32_t platformUartBaud(OepUart &serial, uint32_t requested) {
#if defined(ARDUINO_ARCH_ESP32)
  const uint32_t actual = serial.baudRate();
  return actual ? actual : requested;
#else
  (void)serial;
  return requested;
#endif
}

// The SERIAL_xyz config for 7 or 8 data bits, parity 0 none / 1 even / 2 odd, 1 or 2 stop bits.
inline uint32_t platformUartConfig(uint8_t data_bits, uint8_t parity, uint8_t stop_bits) {
  static const uint32_t k8[3][2] = {{SERIAL_8N1, SERIAL_8N2}, {SERIAL_8E1, SERIAL_8E2}, {SERIAL_8O1, SERIAL_8O2}};
  static const uint32_t k7[3][2] = {{SERIAL_7N1, SERIAL_7N2}, {SERIAL_7E1, SERIAL_7E2}, {SERIAL_7O1, SERIAL_7O2}};
  return (data_bits == 7 ? k7 : k8)[parity > 2 ? 0 : parity][stop_bits == 2 ? 1 : 0];
}

}  // namespace oep
