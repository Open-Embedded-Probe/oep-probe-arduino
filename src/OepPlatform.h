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

inline bool platformUartBegin(OepUart &serial, uint32_t baud, int rx, int tx) {
#if defined(ARDUINO_ARCH_RP2040)
  if (!serial.setRX(rx) || !serial.setTX(tx)) return false;
  serial.begin(baud, SERIAL_8N1);
  return true;
#elif defined(ARDUINO_ARCH_ESP32)
  serial.begin(baud, SERIAL_8N1, rx, tx);
  return true;
#else
  (void)serial; (void)baud; (void)rx; (void)tx;
  return false;
#endif
}

}  // namespace oep
