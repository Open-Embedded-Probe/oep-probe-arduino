// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The distributed OEP probe firmware as a library part: include this from the sketch's .ino (one file only) and begin
// it. Nothing is wired in - the host chooses every pin at run time (oep-spec oep-core §8) - so one binary serves every
// jig; a jig is this firmware plus its settings (core §13).
//
//   #include <OepFirmware.h>
//   void setup() { oep::firmware.begin(); }   // the distributed firmware's set (oep::kDefaults)
//   void loop() { oep::firmware.loop(); }
//
//   oep::firmware.begin<oep::kDefaults & ~oep::kSwd>();                          // the set but SWD
//   oep::firmware.begin<oep::kNone | oep::kBulk | oep::kRvswd | oep::kConsole>(); // just these
//
// A feature left out links none of its code. The sketch's own interfaces: oep::firmware.add / onSetup / onLoop before
// begin() (OepFirmwareParts.h, from any of the sketch's files). The chip's part is in src/firmware:
//
//   Rp2.h       RP2040 / RP2350: USB CDC, vendor bulk, HID (UART bridge); RVSWD, SWD, SWIO, consoles, gpio, uart,
//               settings, analog, logic (PIO), group, link, I2C / SPI targets, I2C / SPI controllers, pwm, counter, memo
//   Esp32P4.h   ESP32-P4 / P4X: HS vendor bulk, HID, CDC, USB-Serial/JTAG, UART bridge, DFU; RVSWD, SWIO, SWD,
//               consoles, gpio, uart x2, capture, I2C / SPI targets, settings, analog, group, link, I2C / SPI
//               controllers, pwm, counter, memo
//   Ch32.h      WCH CH32 on ArduinoCore-CH32RV: vendor bulk, HID (CDC, UART bridge); RVSWD, console, gpio, link - the
//               X035's 62 KiB taking these - and on the larger chips SWD, SWIO, uart, I2C / SPI targets, analog,
//               logic, counter, dac, settings without a store, I2C / SPI controllers
//   Esp32.h     classic ESP32: a UART bridge (port_speed), TCP over Wi-Fi; SWIO, RVSWD, SWD, consoles, gpio, uart,
//               capture (sampler), I2C / SPI targets, settings, analog, group, link, I2C / SPI controllers, pwm,
//               counter, dac, memo
#pragma once

#include "OepFirmwareParts.h"

#if defined(ARDUINO_ARCH_RP2040)
#include "firmware/Rp2.h"
#elif defined(CONFIG_IDF_TARGET_ESP32P4)
#include "firmware/Esp32P4.h"
#elif defined(CONFIG_IDF_TARGET_ESP32)
#include "firmware/Esp32.h"
#elif defined(ARDUINO_ARCH_CH32RV)
#include "firmware/Ch32.h"
#else
#error "OepFirmware: no firmware for this chip yet (RP2040, RP2350, ESP32-P4, ESP32, CH32)"
#endif

// The firmware's objects are this file's: a second file including it stops the link here, not two firmwares at run
// time
void oepFirmwareOnce() {}

namespace oep {

template <uint64_t F>
void Firmware::begin(const FirmwareConfig &config) {
  static_assert((F & ~kAvailable) == 0, "OepFirmware: a feature this chip does not have (oep::kAvailable)");
  static_assert((F & kTransports) != 0, "OepFirmware: no transport (kBulk, kHid, kCdc, kUsj, kBridge, kWifi)");
  static_assert((F & kConsole) == 0 || (F & kRvswd) != 0, "OepFirmware: kConsole needs kRvswd");
  static_assert((F & kSwioConsole) == 0 || (F & kSwio) != 0, "OepFirmware: kSwioConsole needs kSwio");
  static_assert((F & kPortSpeed) == 0 || (F & kBridge) != 0, "OepFirmware: kPortSpeed needs kBridge");
  firmwareSetup<F>(config);
}

inline void Firmware::begin(const FirmwareConfig &config) { begin<kDefaults>(config); }

inline void Firmware::loop() { firmwareLoop(); }

inline void Firmware::runSetup(Endpoint &endpoint, PinTable &pins) {
  for (Node *n = setups_; n; n = n->next) n->setup(endpoint, pins);
  for (Node *n = interfaces_; n; n = n->next) endpoint.add(*n->interface);
}

}  // namespace oep
