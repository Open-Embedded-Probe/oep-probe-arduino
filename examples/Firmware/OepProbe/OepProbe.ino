// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP probe firmware, one per chip family: nothing is wired in - the host chooses every pin at run time (oep-spec
// docs/oep-if-debug.ja.md §1, the plan of oep-core §8) - so one binary serves every jig; a jig is this firmware plus
// its settings (the settings (core §13)). The chip's own part is in its header; what it takes is a choice there (OepFeatures.h:
// a sketch's own oepWithX() replaces the header's):
//
//   Rp2.h       RP2040 / RP2350 (profiles rp2040 / rp2350): USB CDC, vendor bulk, HID (UART bridge); RVSWD, SWD, SWIO,
//               consoles, gpio, uart, settings, analog, logic (PIO), group, link, I2C / SPI targets, I2C / SPI
//               controllers, pwm, counter
//   Esp32P4.h   ESP32-P4 / P4X (profiles esp32p4 / esp32p4x): HS vendor bulk, HID, CDC, USB-Serial/JTAG (UART bridge),
//               DFU; RVSWD, SWIO, SWD, consoles, gpio, uart x2, capture, I2C / SPI targets, settings, analog, group,
//               link, I2C / SPI controllers, pwm, counter (build_opt.h: the direct HS vendor build; compile with --clean)
//   Ch32.h      WCH CH32 on ArduinoCore-CH32RV (profiles ch32x035 / ch32v307 / ch32h417): vendor bulk, HID (CDC, UART
//               bridge); RVSWD, console, gpio, link - the X035's 62 KiB taking these - and on the larger chips SWD,
//               SWIO, uart, I2C / SPI targets, analog, logic, counter, dac, settings without a store, I2C / SPI
//               controllers
//   Esp32.h     classic ESP32 (profile esp32): a UART bridge, TCP over Wi-Fi (the settings' networks); SWIO, RVSWD, SWD,
//               consoles, gpio, uart, capture (sampler), I2C / SPI targets, settings, analog, group, link, I2C / SPI
//               controllers, pwm, counter, dac
//
// Your own interfaces on top of these: oepUserSetup / oepUserLoop in a file of your own beside this one (OepFeatures.h,
// docs/guide/firmware-features.md).
#if defined(ARDUINO_ARCH_RP2040)
#include "Rp2.h"
#elif defined(CONFIG_IDF_TARGET_ESP32P4)
#include "Esp32P4.h"
#elif defined(CONFIG_IDF_TARGET_ESP32)
#include "Esp32.h"
#elif defined(ARDUINO_ARCH_CH32RV)
#include "Ch32.h"
#else
#error "OepProbe: no firmware for this chip yet (RP2040, RP2350, ESP32-P4, ESP32, CH32)"
#endif
