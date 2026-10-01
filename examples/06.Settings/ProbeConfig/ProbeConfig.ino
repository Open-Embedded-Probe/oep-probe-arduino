// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A jig that sets itself up at boot: oep.probe.config keeps what the host set, and the probe does it again every boot
// (oep-spec docs/oep-if-probe-config.ja.md). Nothing about the jig is in this sketch; its settings say:
//
//   slot   a place a target is wired to (a wire and its pins), with a name - attach at boot and retry, the console's
//          mechanism, the line's settings (max_speed_hz, idle_clock), a lock on the chip id
//   bind   what a serial port carries outside the OEP frames: the slot's console (so a terminal on the probe's port
//          shows the target's output, on the same line as OEP), a fixture UART, or several, marked by name
//   plan   pins an interface keeps (a fixture UART on the DUT's TX / RX)
//   idle   how a free pin rests (pull-up on a DUT input that must not float)
//   disable a channel the probe never uses or touches (not on this board, or wired to another part): every request
//          naming it is refused (unavailable, cause 5), and the pin is never parked - not even at boot
//
// The host sets them with `oep config` (oep-client-python), and `--save` writes them to flash (NVS on ESP32, the last
// sector on RP2040 / RP2350):
//
//   oep config slot  <port> --name dut --wire rvswd --pins 2,3 --attach at-boot --retry 1 --mechanism dmseq
//   oep config bind  <port> --port 0 --mode last-reset --stream slot:dut
//   oep config plan  <port> oep.fixture.uart#1 rx=5 tx=4
//   oep config idle  <port> 5 pull-up --save
//   oep config disable <port> 28 29 --save
//   oep config show  <port>
//
// Then the probe's USB serial port shows the target's console from boot, and a flash tool still talks OEP on the same
// port: during its session the console is held and resumes, afterwards, from the target's last reset (core §3.4).
// Saved settings belong to this firmware's interface list: another firmware leaves them unapplied.
#include <OepBind.h>
#include <OepCh32Dm.h>
#include <OepConfig.h>
#include <OepConsole.h>
#include <OepDmConsole.h>
#include <OepEndpoint.h>
#include <OepFixture.h>
#include <OepPinTable.h>
#include <OepPlatform.h>
#include <OepRvswdPhy.h>
#include <OepTarget.h>

static constexpr uint8_t kSwdio = 2, kSwclk = 3;   // the wire's pair; a slot names it
// The rest of a Pico's pins are fixture channels: GP0-GP22 and GP26-GP28 but the pair. UART0 RX / TX may be GP1/0,
// GP5/4, GP13/12, GP17/16 or GP29/28.
static constexpr uint64_t kPair = (1ull << kSwdio) | (1ull << kSwclk);
static constexpr uint64_t kChannels = (((1ull << 23) - 1) | (0x7ull << 26)) & ~kPair;

static uint8_t rxBuffer[1100];
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {1024, 4096, 8},
                              oep::Endpoint::kUsbCdc);   // transport 0, serial port 0: the bind's "--port 0"

static oep::RvswdPhy phy;
static oep::Ch32Dm dm(phy);
static oep::DebugPort port{dm, kSwdio, kSwclk};
static oep::WireRvswd wire(port, 0);
static oep::TargetRiscvDm riscvDm(port, 0);
static oep::DmConsole consoleDriver(dm, phy);
static oep::TargetConsoleStream console(port, consoleDriver, 0);
static oep::PinTable pins(kChannels);
static oep::FixtureGpio gpio(pins, 0, 1);
static oep::FixtureUart uart(pins, Serial1, 0, 2);

// Binds: what the serial ports carry outside the frames. ProbeConfig: the settings, their storage, the slots' attach.
static oep::Binds binds;
static oep::ProbeConfig config(endpoint, binds);
static uint8_t probeTlv[64];

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[17];
  oep::describeCore(w, "probe-config", id, oep::platformUnitId(id, sizeof id), 30, ((1ull << 30) - 1) & ~kChannels);
  return w.ok() ? w.length() : 0;
}

void setup() {
  Serial.ignoreFlowControl(true);   // answer whatever DTR the host left (probe-development-guide §1)
  Serial.begin(115200);
  // RP2 pads boot with a pull-down: Hi-Z until the plan or an idle item says - except the channels the saved settings
  // disable, which are never touched (read first: probe.config §2 applies them before any park)
  config.load();
  pins.setDisabled(config.savedDisabled());
  oep::platformParkMask(kChannels & ~pins.disabledMask());
  phy.begin(kSwdio, kSwclk);
  endpoint.setRawPorts(&binds);       // the serial ports' raw bytes go through the binds
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setBootId(oep::platformRandom32());
  endpoint.add(wire);
  endpoint.add(riscvDm);
  endpoint.add(console);
  endpoint.add(gpio);
  endpoint.add(uart);
  endpoint.add(config);   // last, so the fns before it keep their numbers when the sketch grows
  config.addPlace(wire, console);   // a slot may name this wire (and its console)
  config.addUart(uart);             // a bind may carry this UART
  config.setPins(&pins);            // idle items set these pins, disable items take them away
  config.applySaved();              // what load() read, done again now that every interface is there
}

void loop() {
  endpoint.poll();
  console.poll();
  uart.poll();
  config.poll();   // the slots' attach at boot and their retries, the bound consoles
}
