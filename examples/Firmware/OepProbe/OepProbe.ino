// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP probe firmware, one per chip family: nothing is wired in - the host chooses every pin at run time (oep-spec
// docs/oep-if-debug.ja.md §1, the plan of oep-core §8) - so one binary serves every jig; a jig is this firmware plus
// its settings (oep.probe.config). The chip's own part is in its header:
//
//   Rp2.h       RP2040 / RP2350 (profiles rp2040 / rp2350): USB CDC; RVSWD, SWD, gpio, uart
//   Esp32P4.h   ESP32-P4 / P4X (profiles esp32p4 / esp32p4x): HS vendor bulk, HID, CDC, USB-Serial/JTAG; RVSWD, gpio, uart x2, capture,
//               SPI / I2C devices (build_opt.h: the direct HS vendor build; compile with --clean)
//   Esp32.h     classic ESP32 (profile esp32): a UART bridge, TCP over Wi-Fi (the settings' networks; -DOEP_WIFI=0 drops
//               it); SWIO, gpio, uart, capture (sampler), SPI / I2C devices
#if defined(ARDUINO_ARCH_RP2040)
#include "Rp2.h"
#elif defined(CONFIG_IDF_TARGET_ESP32P4)
#include "Esp32P4.h"
#elif defined(CONFIG_IDF_TARGET_ESP32)
#include "Esp32.h"
#else
#error "OepProbe: no firmware for this chip yet (RP2040, RP2350, ESP32-P4, ESP32)"
#endif
