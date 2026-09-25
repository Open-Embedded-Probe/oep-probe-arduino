// The few places where the Arduino cores differ, kept out of the services.
//
// GPIO modes: ESP32 spells open drain OUTPUT_OPEN_DRAIN and lets INPUT_PULLUP |
// INPUT_PULLDOWN mean both pulls; the RP2040/RP2350 core has neither, so open
// drain is emulated (release = input, low = driven) and both pulls go through
// the SDK. UART: ESP32 takes the pins in begin(), the RP2 core takes them from
// setRX/setTX beforehand, and the two spell their buffer sizing differently.
#pragma once

#include <Arduino.h>

#if defined(ARDUINO_ARCH_RP2040)
#include <hardware/gpio.h>
#include <pico/unique_id.h>
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

inline void platformGpio(int pin, uint8_t mode) {
#if defined(ARDUINO_ARCH_RP2040)
  switch (mode) {
    case kGpioInputFloating: pinMode(pin, INPUT); break;
    case kGpioInputPullUp: pinMode(pin, INPUT_PULLUP); break;
    case kGpioInputPullDown: pinMode(pin, INPUT_PULLDOWN); break;
    case kGpioInputPullUpDown: pinMode(pin, INPUT); gpio_set_pulls(pin, true, true); break;
    // The level goes into the output latch before the output is enabled: the other order drives whatever the
    // latch held last, which on a reset line can be a brief high.
    case kGpioOutputLow: digitalWrite(pin, LOW); pinMode(pin, OUTPUT); break;
    case kGpioOutputHigh: digitalWrite(pin, HIGH); pinMode(pin, OUTPUT); break;
    // No open-drain output on the RP2 pad: release as an input, drive the low.
    case kGpioOpenDrainLow: digitalWrite(pin, LOW); pinMode(pin, OUTPUT); break;
    case kGpioOpenDrainRelease: pinMode(pin, INPUT); break;
    default: break;
  }
#else
  switch (mode) {
    case kGpioInputFloating: pinMode(pin, INPUT); break;
    case kGpioInputPullUp: pinMode(pin, INPUT_PULLUP); break;
    case kGpioInputPullDown: pinMode(pin, INPUT_PULLDOWN); break;
    case kGpioInputPullUpDown: pinMode(pin, INPUT_PULLUP | INPUT_PULLDOWN); break;
    case kGpioOutputLow: pinMode(pin, OUTPUT); digitalWrite(pin, LOW); break;
    case kGpioOutputHigh: pinMode(pin, OUTPUT); digitalWrite(pin, HIGH); break;
    case kGpioOpenDrainLow: pinMode(pin, OUTPUT_OPEN_DRAIN); digitalWrite(pin, LOW); break;
    case kGpioOpenDrainRelease: pinMode(pin, OUTPUT_OPEN_DRAIN); digitalWrite(pin, HIGH); break;
    default: break;
  }
#endif
}

inline void platformParkPins(const uint8_t *pins, size_t count) {
  for (size_t i = 0; i < count; ++i) platformGpio(pins[i], kGpioInputFloating);
}

// The same, for every channel set in `mask`.
inline void platformParkMask(uint64_t mask) {
  for (int pin = 0; pin < 64; ++pin) if ((mask >> pin) & 1) platformGpio(pin, kGpioInputFloating);
}

// This probe's own unit id, the same on any transport: the eFuse MAC on the ESP32 family, the flash's unique id on
// the RP2040 / RP2350. -> bytes written (at most `capacity`).
inline size_t platformUnitId(uint8_t *out, size_t capacity) {
#if defined(ARDUINO_ARCH_RP2040)
  pico_unique_board_id_t id;
  pico_get_unique_board_id(&id);
  const size_t n = sizeof id.id < capacity ? sizeof id.id : capacity;
  memcpy(out, id.id, n);
  return n;
#elif defined(ARDUINO_ARCH_ESP32)
  const uint64_t mac = ESP.getEfuseMac();
  const size_t n = capacity < 6 ? capacity : 6;
  for (size_t i = 0; i < n; ++i) out[i] = static_cast<uint8_t>(mac >> (8 * i));
  return n;
#else
  (void)out; (void)capacity;
  return 0;
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
inline bool platformUartBegin(OepUart &serial, uint32_t baud, int rx, int tx, uint32_t config = SERIAL_8N1) {
#if defined(ARDUINO_ARCH_RP2040)
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
