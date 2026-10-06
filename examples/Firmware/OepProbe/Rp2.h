// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// RP2040 / RP2350 (built for the Raspberry Pi Pico / Pico 2; profiles rp2040 / rp2350).
//
// Transport: USB CDC (Serial), a serial port: COBS frames (oep-transports §1). The USB device is the project's VID:PID
// 1209:4F45 (registry usb; PID-USE.md), serial = the unit id, describe discoverable 1; its iProduct "OEP probe (RP2040)" /
// "(RP2350)" is a name for people.
//
// Interfaces (revision 1): oep.core; oep.wire.rvswd + oep.target.riscv-dm + oep.target.console (WCH CH32, 2 wires);
// oep.wire.swd + oep.target.arm-adi (ARM); oep.fixture.gpio / uart; oep.fixture.analog (GP26-28, 500 kS/s in all).
// Every channel below may be SWDIO / SWCLK of either
// wire, the reset line of attach's reset TLV, a gpio or a UART pin (UART0: GP0/1, GP12/13, GP16/17, GP28/29); a live
// debug connection holds its pair, a plan holds its pins (oep-core §8.1). What a target needs of its line (idle_clock,
// max_speed) comes from the host (oep-if-debug §3). oep.probe.config keeps a jig's settings in flash (slots on any pair,
// a bind of the CDC port - the target's console on the same line as OEP -, labels, idle states).
#pragma once
#include <USB.h>

#include <OepAnalog.h>
#include <OepBind.h>
#include <OepCh32Dm.h>
#include <OepConfig.h>
#include <OepConsole.h>
#include <OepDmConsole.h>
#include <OepEndpoint.h>
#include <OepFixture.h>
#include <OepPinTable.h>
#include <OepRvswdPhy.h>
#include <OepSwd.h>
#include <OepTarget.h>

#if defined(ARDUINO_ARCH_RP2350) || defined(PICO_RP2350)
static constexpr const char *kProduct = "OEP probe (RP2350)", *kModel = "rp2350";
#else
static constexpr const char *kProduct = "OEP probe (RP2040)", *kModel = "rp2040";
#endif

#if defined(ARDUINO_SPARKFUN_PROMICRO_RP2350)
// SparkFun Pro Micro RP2350 (profile promicrorp2350): GP0-GP29 are pins, but GP19, its PSRAM's chip select. The L103
// bench's RVSWD is GP24 / GP23 - the Pico 2 build kept those as the Pico's own (0.0.18).
static constexpr uint64_t kChannels = ((1ull << 30) - 1) & ~(1ull << 19);
#else
// The pins a Pico / Pico 2 brings out: GP0-GP22, GP26-GP28 (GP23-GP25 and GP29 are the board's own there). Other boards
// with the same chip run this too; their own parts on these pins (an LED, a PSRAM chip select) are for the host to
// leave alone.
static constexpr uint64_t kChannels = ((1ull << 23) - 1) | (0x7ull << 26);
#endif
static constexpr uint64_t kReserved = ((1ull << 30) - 1) & ~kChannels;
static constexpr uint16_t kUnset = 0xfffe;   // no pair chosen yet

static uint8_t rxBuffer[1100];   // the encoded candidate: cobsFrameMax(1024)
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {1024, 4096, 8},
                              oep::Endpoint::kUsbCdc, 0);
static oep::Link oepLink(endpoint);   // oep.link (oep-if-link)

static oep::PinTable pins(kChannels);

static oep::RvswdPhy phy;
static oep::Ch32Dm dm(phy);
static oep::DebugPort rvswd{dm, kUnset, kUnset};
static oep::WireRvswd wireRvswd(rvswd, 0);
static oep::TargetRiscvDm riscvDm(rvswd, 0);
static oep::DmConsole consoleDriver(dm, phy);
static oep::TargetConsoleStream console(rvswd, consoleDriver, 0);

static oep::SwdPort swd{kUnset, kUnset};
static oep::WireSwd wireSwd(swd, 0);
static oep::TargetArmAdi adi(swd, 0);

static oep::FixtureGpio gpio(pins, 0, 1);
static oep::FixtureUart uart(pins, Serial1, 0, 2);
static oep::Binds binds;
static oep::ProbeConfig config(endpoint, binds);
// the ADC on GP26-28 (a Pico's GP29 reads VSYS), channels in turn, copied by DMA
static constexpr uint64_t kAdc = 0x7ull << 26;
static oep::AnalogCapture analog(endpoint, kAdc, 0);
static uint8_t probeTlv[200];

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[17];   // the flash's unique id: the probe says who it is on any transport
  oep::describeCore(w, kModel, id, oep::platformUnitId(id, sizeof id), 30, kReserved);
  oep::describeChip(w);   // the MCU and its revision (a capture records what it was taken on)
  return w.ok() ? w.length() : 0;
}

void setup() {
  USB.disconnect();
  USB.setManufacturer("Open Embedded Probe");
  USB.setVIDPID(oep::reg::kUsbProjectVid, oep::reg::kUsbProjectPid);   // the project's VID:PID (registry usb, PID-USE.md)
  USB.setProduct(kProduct);   // a name for people; no host identifies the probe by it
  static uint8_t serial[17];
  oep::platformUnitId(serial, sizeof serial);
  USB.setSerialNumber(reinterpret_cast<const char *>(serial));   // the unit id, lowercase (transports §3)
  USB.connect();
  Serial.ignoreFlowControl(true);   // answer whatever DTR the host left (probe-development-guide §1)
  Serial.begin(115200);
  // Hi-Z every channel (RP2 pads boot with a pull-down) until the host takes one.
  // The saved settings are read first: their disable items' channels are never parked (probe.config §2: applied
  // before any idle / park; applySaved below gives them back if the settings are not applied).
  config.load();
  pins.setDisabled(config.savedDisabled());
  oep::platformParkMask(kChannels & ~pins.disabledMask());
  rvswd.pin_choice = kChannels;
  rvswd.pins = &pins;
  rvswd.reset_allowed = kChannels;   // attach's reset TLV: the channel the host names (no default), nobody holding it
  swd.pin_choice = kChannels;
  swd.pins = &pins;
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setDiscoverable(true);   // its one transport is USB with the project's VID:PID (transports §3, core §7.5)
  endpoint.add(wireRvswd);
  endpoint.add(riscvDm);
  endpoint.add(console);
  endpoint.add(wireSwd);
  endpoint.add(adi);
  endpoint.add(gpio);
  endpoint.add(uart);
  uart.setRoleChannels(oep::platformUartRxMask(0), oep::platformUartTxMask(0));   // Serial1 = UART0: the pins it reaches
  endpoint.setRawPorts(&binds);
  endpoint.add(config);   // last: the fns before it keep their numbers
  config.addPlace(wireRvswd, console);
  config.addUart(uart);
  config.setPins(&pins);   // the idle item sets these pins' free state
  analog.setPins(&pins, 7);   // PinTable owners: gpio 1, uart 2, the analog 7 (its pads go analog)
  endpoint.add(analog);   // after config: the fns before it keep their numbers
  endpoint.add(oepLink);   // oep.link (the link test), last: the fns before it keep their numbers
  // Last, once every interface is added: the saved settings name fns, and are kept only for the same interface list
  // (applied before the analog and the group were added, they never matched it: unreadable after every reboot, 0.0.11-0.0.16).
  // In the order of probe.config §2: every idle (outputs driven) first, then the plans, the uarts, and the at-boot
  // slots' attach last (on its poll), so a target powered through an output idle is up before it.
  config.applySaved();   // read by config.load() at the top
}

void loop() {
  endpoint.poll();
  console.poll();
  uart.poll();
  config.poll();
  analog.poll();
}
