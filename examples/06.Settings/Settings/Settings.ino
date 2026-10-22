// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A jig that keeps its settings: the core's settings (oep-spec oep-core §13) hold what the host set, and the probe applies
// them again every boot. Nothing about the jig is in this sketch; its settings say:
//
//   channel.<pin>.label    a channel's name for the host (a port "dut" is the labels "dut.swdio", "dut.nrst", ...: host
//                          guide §5.3)
//   channel.<pin>.idle     how a free pin rests (pull-up on a DUT input that must not float)
//   channel.<pin>.disable  a channel the probe never uses or touches (not on this board, or wired to another part): every
//                          request naming it is refused (unavailable, cause 5), and the pin is never parked - not even at boot
//   passthrough.<port>.*   (oep.fixture.uart's) a serial port carries the UART's receive outside the OEP frames while no
//                          session uses it
//   passthrough.<port>.*   (oep.target.console's) a serial port carries the target's debug console the same way (read over
//                          the wire, no halt)
//
// Every item is declared by its owner (fn 0 for the channels, the interface for its own) and named; values name pins and
// ports by name (GPIO2, cdc0). The host sets them with `oep settings` (oep-client-python), saves them to flash (NVS on
// ESP32, the last sector on RP2040 / RP2350, by name), and exports them to a file to put back after a firmware update:
//
//   oep settings set <probe> core channel.GPIO2.label=dut.swdio channel.GPIO3.label=dut.swclk channel.GPIO5.idle=pull-up
//   oep settings set <probe> oep.target.console/0 passthrough.cdc0.wire=oep.wire.rvswd/0 passthrough.cdc0.swdio=GPIO2 \
//       passthrough.cdc0.swclk=GPIO3 passthrough.cdc0.mechanism=dmseq --save
//   oep settings show <probe>
//   oep settings export <probe> jig.oep-settings.json        (and import <probe> jig.oep-settings.json)
//
// Then the probe's USB serial port shows the target's console while no tool uses the probe, and a flash tool still talks
// OEP on the same port: during its session the console pass-through stops, and starts again after it (console §2.1).
#include <OepBind.h>
#include <OepCh32Dm.h>
#include <OepConsole.h>
#include <OepDmConsole.h>
#include <OepEndpoint.h>
#include <OepFixture.h>
#include <OepPinTable.h>
#include <OepPlatform.h>
#include <OepSettings.h>
#include <OepRvswdPhy.h>
#include <OepTarget.h>

static constexpr uint8_t kSwdio = 2, kSwclk = 3;   // the wire's pair
// The rest of a Pico's pins are fixture channels: GP0-GP22 and GP26-GP28 but the pair. UART0 RX / TX may be GP1/0,
// GP5/4, GP13/12, GP17/16 or GP29/28.
static constexpr uint64_t kPair = (1ull << kSwdio) | (1ull << kSwclk);
static constexpr uint64_t kChannels = (((1ull << 23) - 1) | (0x7ull << 26)) & ~kPair;

static uint8_t rxBuffer[1100];
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {1024, 4096, 8},
                              oep::Endpoint::kUsbCdc, 0);   // transport 0, serial port 0: a pass-through's port 0; CDC interface 0

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

// Binds: what the serial ports carry outside the frames. Settings: fn 0's settings, their storage, the pass-throughs.
static oep::Binds binds;
static oep::Settings settings(endpoint, binds);
static uint8_t probeTlv[64];

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[17];
  oep::describeCore(w, "settings", id, oep::platformUnitId(id, sizeof id), 30);
  return w.ok() ? w.length() : 0;
}

void setup() {
  Serial.ignoreFlowControl(true);   // answer whatever DTR the host left (probe-development-guide §1)
  Serial.begin(115200);
  // RP2 pads boot with a pull-down: Hi-Z until the plan or an idle item says - except the channels the saved settings
  // disable, which are never touched (read first: core §13.7 applies disable and idle before anything else)
  settings.load();
  pins.setDisabled(settings.savedDisabled());
  oep::platformParkMask(kChannels & ~pins.disabledMask());
  phy.begin(kSwdio, kSwclk);
  endpoint.setRawPorts(&binds);       // the serial ports' raw bytes go through the binds
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.add(wire);
  endpoint.add(riscvDm);
  endpoint.add(console);
  endpoint.add(gpio);
  endpoint.add(uart);
  settings.usePassthroughs();         // the pass-through items (a serial port's raw stream carries them)
  settings.addPlace(wire, console);   // the console's pass-through may name this wire; the wire's hold and rest
  settings.addUart(uart);             // the UART's pass-through
  settings.setPins(&pins);            // channel.{}.idle sets these pins, channel.{}.disable takes them away
  settings.applySaved();              // the save read again, now that every interface is there (they are named in it)
}

void loop() {
  endpoint.poll();
  console.poll();
  uart.poll();
  settings.poll();   // the pass-throughs, the wire's hold
}
