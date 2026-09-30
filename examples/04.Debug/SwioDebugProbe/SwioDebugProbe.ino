// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A debugger for the WCH CH32V00x (V003, V002, V006 and the like) on their one-wire SWIO link, on a classic ESP32 DevKit
// (oep-spec docs/oep-if-debug.ja.md). The same three interfaces as RvswdDebugProbe, on one wire:
//
//   oep.wire.swio        attach / detach on the SWIO pin (swclk 0xffff: one wire)
//   oep.target.riscv-dm  the debug module: halt / resume, reset, block read / write, run a loader
//   oep.target.console   the target's console through the debug module
//
// SwioPhy times the bits by counting CPU cycles on a classic ESP32 (the closest to the WCH-LinkE's timing), on any
// GPIO0-31. Wire kSwio to the target's SWIO (PD1 on a V003) and GND; the line needs a pull-up (the ESP32's own is used).
//
// The transport is UART0 through the board's USB-UART bridge (Serial, 115200). Its frames are 512 bytes and one at a
// time: a bridge without flow control drops bytes in long bursts. A host opens the port with DTR and RTS both on, or the
// bridge's auto-reset circuit resets the ESP32 (probe guide §1).
#include <OepCh32Dm.h>
#include <OepConsole.h>
#include <OepDmConsole.h>
#include <OepEndpoint.h>
#include <OepPlatform.h>
#include <OepSwioPhy.h>
#include <OepTarget.h>

static constexpr uint8_t kSwio = 16;   // GPIO16 -> SWIO

static uint8_t rxBuffer[1024];
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {512, 512, 1},
                              oep::Endpoint::kUartBridge);

static oep::SwioPhy phy;
static oep::Ch32Dm dm(phy);
static oep::DebugPort port{dm, kSwio, 0xffff};             // one wire
static oep::WireRvswd wire(port, 1, "oep.wire.swio");      // the same class, told it is the one-wire link
static oep::TargetRiscvDm riscvDm(port, 1);
static oep::DmConsole consoleDriver(dm, phy);
static oep::TargetConsoleStream console(port, consoleDriver, 1);
static uint8_t probeTlv[64];

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[17];
  oep::describeCore(w, "swio-debug-probe", id, oep::platformUnitId(id, sizeof id), 40, 1ull << kSwio);
  return w.ok() ? w.length() : 0;
}

void setup() {
  Serial.setRxBufferSize(8192);
  Serial.setTxBufferSize(8192);
  Serial.begin(115200);
  phy.begin(kSwio);
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setBootId(oep::platformRandom32());
  endpoint.add(wire);
  endpoint.add(riscvDm);
  console.setMaxRead(480);   // a console read's data fits a 512-byte frame
  endpoint.add(console);
}

void loop() {
  endpoint.poll();
  console.poll();
}
