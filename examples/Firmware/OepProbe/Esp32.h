// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Classic ESP32 (profile esp32), on a board with a USB-UART bridge (DevKitC and the like).
//
// Transport: UART0 through the bridge at 115200, fixed (probe guide §3.5) - the probe's one transport, serial port 0:
// OEP frames (0x00 <COBS> 0x00) and the raw bytes of its bind on one line (oep-core §3.4). The bridge's auto-reset
// circuit resets the ESP32 when the port is opened with DTR / RTS in the wrong order: a host opens it with both on
// (host guide §1). A UART has no iProduct: a host finds this probe by opening the port and asking (confirm).
//
// Interfaces (revision 1): oep.core; oep.wire.swio + oep.target.riscv-dm + oep.target.console (WCH CH32V00x, one wire);
// oep.fixture.gpio / uart / capture (the core-0 GPIO sampler: up to 8 lines, 0.4-2 MHz, one-shot); the ESP-IDF SPI / I2C
// devices io.github.ch32-riscv-ug.esp32.spi-target / i2c-target; oep.probe.config (saved in NVS); oep.fixture.analog
// (ADC1 on 32-36 / 39) and oep.fixture.capture-group (the analog with the sampler). The host chooses every
// pin: SWIO any output GPIO below 32, the reset line and the fixtures any channel below.
#pragma once
#include <OepAnalog.h>
#include <OepBind.h>
#include <OepCaptureGroup.h>
#include <OepCh32Dm.h>
#include <OepConfig.h>
#include <OepConsole.h>
#include <OepDmConsole.h>
#include <OepEndpoint.h>
#include <OepFixture.h>
#include <OepP4I2cTarget.h>
#include <OepP4SpiTarget.h>
#include <OepPinTable.h>
#include <OepSampler.h>
#include <OepSwioPhy.h>
#include <OepTarget.h>

// UART0 runs through the board's USB-UART bridge (no flow control): long bursts of pipelined responses lost bytes
// (2026-09-22, 2026-09-24). One 512-byte frame in flight keeps the outstanding data small.
static uint8_t rxBuffer[1024];   // the encoded candidate: cobsFrameMax(512)
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {512, 512, 1},
                              oep::Endpoint::kUartBridge);

// The GPIOs a DevKitC brings out, less UART0 (1, 3: the transport), the SPI flash (6-11) and the boot straps (0, 2, 12,
// 15). 34-39 are inputs only.
static constexpr uint64_t kChannels = (1ull << 4) | (1ull << 5) | (1ull << 13) | (1ull << 14) | (1ull << 16) | (1ull << 17) |
                                      (1ull << 18) | (1ull << 19) | (1ull << 21) | (1ull << 22) | (1ull << 23) | (1ull << 25) |
                                      (1ull << 26) | (1ull << 27) | (1ull << 32) | (1ull << 33) | (1ull << 34) | (1ull << 35) |
                                      (1ull << 36) | (1ull << 39);
static constexpr uint64_t kReserved = ((1ull << 40) - 1) & ~kChannels;
static constexpr uint64_t kSwioChoice = kChannels & 0xffffffffull;   // SwioPhy drives GPIO0-31
static constexpr uint16_t kUnset = 0xfffe;                           // no pin chosen yet

static oep::SwioPhy phy;
static oep::Ch32Dm dm(phy);
static oep::DebugPort swio{dm, kUnset, 0xffff};   // one wire: swclk stays 0xffff
static oep::WireRvswd wire(swio, 1, "oep.wire.swio");
static oep::TargetRiscvDm riscvDm(swio, 1);
static oep::DmConsole consoleDriver(dm, phy);
static oep::TargetConsoleStream console(swio, consoleDriver, 1);
static oep::PinTable pins(kChannels);
// PinTable owners: gpio 1, uart 2 (the I2C device is 3, the SPI device 6, the SWIO wire 0xf0)
static oep::FixtureGpio gpio(pins, 1, 1);
static oep::FixtureUart uart(pins, Serial2, 1, 2);
static oep::SamplerCapture capture(endpoint, kReserved);
static oep::P4I2cTarget i2c(pins);
static oep::P4SpiTarget spi(pins);
static oep::Binds binds;
static oep::ProbeConfig config(endpoint, binds);
// ADC1 in DMA mode on the pins brought out (32-36, 39), and a group that starts it with the sampler
static constexpr uint64_t kAdc1 = kChannels & (0xffull << 32);
static oep::AnalogCapture analog(endpoint, kAdc1, 1);
static oep::CaptureGroup group(endpoint, 1);
static uint8_t probeTlv[160];

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[8];
  oep::describeCore(w, "esp32", id, oep::platformUnitId(id, sizeof id), 40, kReserved);
  oep::describeChip(w);   // the MCU and its revision (a capture records what it was taken on)
  return w.ok() ? w.length() : 0;
}

void setup() {
  Serial.setRxBufferSize(8192);
  Serial.setTxBufferSize(8192);
  Serial.begin(115200);
  // Every channel genuinely Hi-Z until the host takes it (a pull on a target's USB line breaks its enumeration, E132).
  oep::platformParkMask(kChannels);
  endpoint.setRawPorts(&binds);
  swio.pin_choice = kSwioChoice;
  swio.pins = &pins;
  swio.reset_allowed = kChannels;   // attach_under_reset: the channel the host names (no default), nobody holding it
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setBootId(esp_random());
  endpoint.add(wire);
  endpoint.add(riscvDm);
  console.setMaxRead(480);   // 512-byte frames
  endpoint.add(console);
  endpoint.add(gpio);
  endpoint.add(uart);
  endpoint.add(i2c);
  endpoint.add(spi);
  endpoint.add(capture);
  endpoint.add(config);   // last: the fns before it keep their numbers
  config.addPlace(wire, console);
  config.addUart(uart);
  config.setPins(&pins);
  config.load();
  config.applySaved();
  endpoint.add(analog);   // after config: the fns before it keep their numbers
  endpoint.add(group);
  group.addTrack(capture, capture);
  group.addTrack(analog, analog);
}

void loop() {
  endpoint.poll();
  console.poll();
  config.poll();
  uart.poll();
  capture.poll();
  i2c.service();
  spi.service();
  analog.poll();
  group.poll();
}
