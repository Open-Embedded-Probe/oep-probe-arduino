// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP v1 probe on a SparkFun Pro Micro RP2350 for the CH32L103 jig (oep-spec docs/oep-core.ja.md, docs/oep-if-debug.ja.md).
// Transport: USB CDC (Serial), a serial port: COBS frames (oep-core §3.1). Target link: RVSWD on two GPIOs through the SIO
// backend in OepRvswdPhy.cpp (same frame as the ESP32-P4 probe).
//
// oep.core (the probe in its describe), oep.wire.rvswd, oep.target.riscv-dm, oep.target.console,
// oep.fixture.gpio / uart (all revision 1). GP2 is the target's NRST: a gpio channel labelled NRST, which oep.wire.rvswd
// attach-under-reset takes when the host names it (no default reset line, oep-if-debug §3) - open drain only, never
// driven high. How the line rests and how fast it may go are the CH32L103's, so the host passes them at attach
// (idle_clock low, max_speed 1 MHz; oep-if-debug §3), or a slot carries them.
#include <OepCh32Dm.h>
#include <OepPinTable.h>
#include <OepRvswdPhy.h>
#include <OepDmConsole.h>
#include <OepConsole.h>
#include <OepEndpoint.h>
#include <OepFixture.h>
#include <OepTarget.h>

// Measured 2026-09-23 once the CH32L103 had power: the pair sweep found its debug module on
// SWDIO=GP0, SWCLK=GP1 (DMSTATUS 0x00000c82) and on no other ordered pair. These are also
// Serial1's default pins, so fixture.uart has to take a different pair from the plan.
// Override with -DOEP_RVSWD_SWDIO= / -DOEP_RVSWD_SWCLK=.
#ifndef OEP_RVSWD_SWDIO
#define OEP_RVSWD_SWDIO 0
#endif
#ifndef OEP_RVSWD_SWCLK
#define OEP_RVSWD_SWCLK 1
#endif

static uint8_t rxBuffer[1100];   // the encoded candidate: cobsFrameMax(1024)
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {1024, 4096, 8},
                                  oep::Endpoint::kUsbCdc, 0);   // a serial port: COBS frames (oep-core §3.1)
static constexpr uint8_t kSwdio = OEP_RVSWD_SWDIO, kSwclk = OEP_RVSWD_SWCLK;
// GP2 is the target's NRST: measured 2026-09-23 by pulling each spare channel down on its
// own and watching which one took the debug module away. It idles high on the CH32's own
// pull-up, which is why the RP2 pad's default pull-down used to hold the part in reset.
static constexpr uint8_t kNrst = 2;
// GP19 is the PSRAM chip select on this board; leave it alone.
static constexpr uint64_t kReserved = (uint64_t{1} << kSwdio) | (uint64_t{1} << kSwclk) | (uint64_t{1} << 19);
static constexpr uint64_t kFixtures = ((1ull << 30) - 1) & ~kReserved;

// CH32L103C8T6. The flash layout is the host's business.
static oep::RvswdPhy phy;
static oep::Ch32Dm dm(phy);
static oep::DebugPort port{dm, kSwdio, kSwclk};
static oep::WireRvswd wire(port, 1);
static oep::TargetRiscvDm riscvDm(port, 1);
static oep::DmConsole consoleDriver(dm, phy);
static oep::TargetConsoleStream console(port, consoleDriver, 1);
static oep::PinTable pins(kFixtures);
static oep::FixtureGpio gpio(pins, 2);
static oep::FixtureUart uart(pins, Serial1, 3, 2);   // UART0 reaches GP0/1, GP12/13, GP16/17 and GP28/29 on this part
static uint8_t probeTlv[200];

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[8];   // the flash's unique id: the probe says who it is on any transport
  oep::describeCore(w, "sparkfun-promicro-rp2350", id, oep::platformUnitId(id, sizeof id), 30, kReserved);
  w.text(oep::kCoreProfile, "io.github.ch32-riscv-ug.rp2350-l103");
  w.label(kSwdio, "SWDIO");
  w.label(kSwclk, "SWCLK");
  w.label(kNrst, "NRST");
  return w.ok() ? w.length() : 0;
}

void setup() {
  // This jig's wiring (fixed): channel 12 drives the DUT PB7 / RX, which must not float while no UART holds it (a floating
  // RX line fed the DUT's command parser noise, 2026-09-22). Idle = pull-up; every other free pin stays Hi-Z.
  pins.setIdle(12, oep::PinTable::kIdlePullUp);
  Serial.ignoreFlowControl(true);   // answer whatever DTR the host left (probe-development-guide §1)
  Serial.begin(115200);
  // The RP2 pad's default pull-down on NRST holds the target in reset. Release it and never drive it high -
  // the target pulls it up.
  pinMode(kNrst, INPUT);
  phy.begin(kSwdio, kSwclk);
  // Hi-Z everything the probe does not own. RP2 pads boot with a pull-down, and this jig is only half wired:
  // on the CH32L103 that pull-down held a line the target cares about and the hart would not halt, though its
  // debug module answered normally (2026-09-23).
  oep::platformParkMask(kFixtures);
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setBootId(rp2040.hwrand32());
  endpoint.add(wire);
  endpoint.add(riscvDm);
  port.reset_allowed = kFixtures;   // attach-under-reset: the channel the host names (no default), nobody holding it
  port.pins = &pins;
  endpoint.add(console);
  endpoint.add(gpio);
  endpoint.add(uart);
}

void loop() {
  endpoint.poll();
  console.poll();
  uart.poll();
}
