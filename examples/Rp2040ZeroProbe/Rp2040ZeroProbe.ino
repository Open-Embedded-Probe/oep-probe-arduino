// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP v1 probe on a Waveshare RP2040-Zero (oep-spec docs/oep-core.ja.md, docs/oep-if-debug.ja.md): the ordinary-ARM counterpart
// of the CH32 probes. Its GP0/GP1 reach the Pro Micro RP2350's SWD port (SWCLK = GP0, SWDIO = GP1, DPIDR 0x4c013477,
// measured 2026-09-23).
//
// Transport: USB CDC (Serial), a serial port: COBS frames (oep-core §3.1).
// oep.core (the probe in its describe), oep.wire.swd, oep.target.arm-adi, oep.fixture.gpio / uart (revision 1).
#include <OepPinTable.h>
#include <OepEndpoint.h>
#include <OepFixture.h>
#include <OepSwd.h>

static uint8_t rxBuffer[1100];   // the encoded candidate: cobsFrameMax(1024)
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {1024, 4096, 8},
                                  oep::Endpoint::kUsbCdc, 0);   // a serial port: COBS frames (oep-core §3.1)
static constexpr uint8_t kSwclk = 0, kSwdio = 1;
// GP16 drives the on-board WS2812; GP0/GP1 are the SWD pair. GP0..GP15 and GP26..GP29 reach the castellated edge.
static constexpr uint64_t kReserved = (uint64_t{1} << kSwclk) | (uint64_t{1} << kSwdio) | (uint64_t{1} << 16);
static constexpr uint64_t kBonded = 0xffffu | (0xfull << 26);
static constexpr uint64_t kFixtures = kBonded & ~kReserved;

static oep::SwdPort port{kSwdio, kSwclk};
static oep::WireSwd wire(port, 1);
static oep::TargetArmAdi adi(port, 1);
static oep::PinTable pins(kFixtures);
static oep::FixtureGpio gpio(pins, 2);
static oep::FixtureUart uart(pins, Serial1, 3, 2);
static uint8_t probeTlv[200];

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[8];
  oep::describeCore(w, "waveshare-rp2040-zero", id, oep::platformUnitId(id, sizeof id), 30, kReserved);
  w.text(oep::kCoreProfile, "io.github.ch32-riscv-ug.rp2040zero-rp2350-swd");
  w.label(kSwclk, "SWCLK");
  w.label(kSwdio, "SWDIO");
  return w.ok() ? w.length() : 0;
}

void setup() {
  Serial.ignoreFlowControl(true);   // answer whatever DTR the host left (probe-development-guide §1)
  Serial.begin(115200);
  // Hi-Z everything the probe does not own (RP2 pads boot with a pull-down).
  oep::platformParkMask(kFixtures);
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setBootId(rp2040.hwrand32());
  endpoint.add(wire);
  endpoint.add(adi);
  endpoint.add(gpio);
  endpoint.add(uart);
}

void loop() {
  endpoint.poll();
  uart.poll();
}
