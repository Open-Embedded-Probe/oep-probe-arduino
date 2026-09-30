// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// RP2040 / RP2350 (built for the Raspberry Pi Pico / Pico 2; profiles rp2040 / rp2350).
//
// Transport: USB CDC (Serial), a serial port: COBS frames (oep-core §3.1). The USB device says iProduct "OEP probe
// (RP2040)" / "(RP2350)" (how discovery knows it, oep-core §3.3), the board's own VID:PID and serial number until the
// OEP PID is granted (PID-USE.md).
//
// Interfaces (revision 1): oep.core; oep.wire.rvswd + oep.target.riscv-dm + oep.target.console (WCH CH32, 2 wires);
// oep.wire.swd + oep.target.arm-adi (ARM); oep.fixture.gpio / uart; oep.fixture.analog (GP26-28, 500 kS/s in all).
// Every channel below may be SWDIO / SWCLK of either
// wire, the reset line of attach_under_reset, a gpio or a UART pin (UART0: GP0/1, GP12/13, GP16/17, GP28/29); a live
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

// The pins a Pico / Pico 2 brings out: GP0-GP22, GP26-GP28 (GP23-GP25 and GP29 are the board's own there). Other boards
// with the same chip run this too; their own parts on these pins (an LED, a PSRAM chip select) are for the host to
// leave alone.
static constexpr uint64_t kChannels = ((1ull << 23) - 1) | (0x7ull << 26);
static constexpr uint64_t kReserved = ((1ull << 30) - 1) & ~kChannels;
static constexpr uint16_t kUnset = 0xfffe;   // no pair chosen yet

static uint8_t rxBuffer[1100];   // the encoded candidate: cobsFrameMax(1024)
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {1024, 4096, 8},
                              oep::Endpoint::kUsbCdc, 0);

static oep::PinTable pins(kChannels);

static oep::RvswdPhy phy;
static oep::Ch32Dm dm(phy);
static oep::DebugPort rvswd{dm, kUnset, kUnset};
static oep::WireRvswd wireRvswd(rvswd, 1);
static oep::TargetRiscvDm riscvDm(rvswd, 1);
static oep::DmConsole consoleDriver(dm, phy);
static oep::TargetConsoleStream console(rvswd, consoleDriver, 1);

static oep::SwdPort swd{kUnset, kUnset};
static oep::WireSwd wireSwd(swd, 1);
static oep::TargetArmAdi adi(swd, 1);

static oep::FixtureGpio gpio(pins, 1, 1);
static oep::FixtureUart uart(pins, Serial1, 1, 2);
static oep::Binds binds;
static oep::ProbeConfig config(endpoint, binds);
// the ADC on GP26-28 (a Pico's GP29 reads VSYS), channels in turn, copied by DMA
static constexpr uint64_t kAdc = 0x7ull << 26;
static oep::AnalogCapture analog(endpoint, kAdc, 1);
static uint8_t probeTlv[200];

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[8];   // the flash's unique id: the probe says who it is on any transport
  oep::describeCore(w, kModel, id, oep::platformUnitId(id, sizeof id), 30, kReserved);
  oep::describeChip(w);   // the MCU and its revision (a capture records what it was taken on)
  return w.ok() ? w.length() : 0;
}

void setup() {
  USB.disconnect();
  USB.setManufacturer("Open Embedded Probe");
  USB.setProduct(kProduct);   // iProduct "OEP...": discovery (oep-core §3.3)
  USB.connect();
  Serial.ignoreFlowControl(true);   // answer whatever DTR the host left (probe-development-guide §1)
  Serial.begin(115200);
  // Hi-Z every channel (RP2 pads boot with a pull-down) until the host takes one.
  oep::platformParkMask(kChannels);
  rvswd.pin_choice = kChannels;
  rvswd.pins = &pins;
  rvswd.reset_allowed = kChannels;   // attach_under_reset: the channel the host names (no default), nobody holding it
  swd.pin_choice = kChannels;
  swd.pins = &pins;
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setBootId(rp2040.hwrand32());
  endpoint.add(wireRvswd);
  endpoint.add(riscvDm);
  endpoint.add(console);
  endpoint.add(wireSwd);
  endpoint.add(adi);
  endpoint.add(gpio);
  endpoint.add(uart);
  endpoint.setRawPorts(&binds);
  endpoint.add(config);   // last: the fns before it keep their numbers
  config.addPlace(wireRvswd, console);
  config.addUart(uart);
  config.setPins(&pins);   // the idle item sets these pins' free state
  config.load();
  config.applySaved();
  analog.setPins(&pins, 7);   // PinTable owners: gpio 1, uart 2, the analog 7 (its pads go analog)
  endpoint.add(analog);   // after config: the fns before it keep their numbers
}

void loop() {
  endpoint.poll();
  console.poll();
  uart.poll();
  config.poll();
  analog.poll();
}
