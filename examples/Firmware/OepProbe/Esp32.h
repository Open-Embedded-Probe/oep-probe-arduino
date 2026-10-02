// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Classic ESP32 (profile esp32), on a board with a USB-UART bridge (DevKitC and the like).
//
// Transport: UART0 through the bridge at 115200 (probe guide §3.5; a host may raise it for its session: port_speed,
// below) - the probe's one transport, serial port 0: OEP frames (0x00 <COBS> 0x00) and the raw bytes of its bind on one line (oep-core §3.4). The bridge's auto-reset
// circuit resets the ESP32 when the port is opened with DTR / RTS in the wrong order: a host opens it with both on
// (host guide §1). A UART has no iProduct: a host finds this probe by opening the port and asking (confirm).
//
// Interfaces (revision 1): oep.core; oep.wire.swio + oep.target.riscv-dm + oep.target.console (WCH CH32V00x, one wire);
// oep.fixture.gpio / uart / capture (the core-0 GPIO sampler: up to 8 lines, 0.4-2 MHz, one-shot); the ESP-IDF SPI / I2C
// devices oep.fixture.spi-target / i2c-target; oep.probe.config (saved in NVS); oep.fixture.analog
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
// (2026-09-22, 2026-09-24), so few 512-byte frames are in flight. Two (window 1024) rather than one: with port_speed
// raised the link waits on the bridge's round trip, and a second frame in flight nearly doubles what moves (1500000:
// 58-65 -> 78 KB/s on an ATOM's FTDI); at 115200 it changes nothing (9.3 -> 9.5 KB/s, no more broken frames) (oep-spec
// docs/uart-speed-negotiation.ja.md §3b, 2026-10-01).
static uint8_t rxBuffer[1024];   // the encoded candidate: cobsFrameMax(512)
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {512, 1024, 2},
                              oep::Endpoint::kUartBridge);

// port_speed (oep-core §3.5): the host may raise UART0's baud for its session; every revert goes back to 115200, the
// boot speed. On unless built with -DOEP_PORT_SPEED=0 (then no describe port_speed, the op unknown_operation).
#ifndef OEP_PORT_SPEED
#define OEP_PORT_SPEED 1
#endif
static constexpr uint32_t kBootBaud = 115200;
#if OEP_PORT_SPEED
// The rate UART0 runs at for `baud`, as arduino-esp32 3.3 sets it: the 1 MHz REF_TICK up to 250000, the 80 MHz APB
// above, a 20.4 fixed-point divider (0: not makeable; 5 Mbaud is the UART's limit).
static uint32_t uartRate(uint32_t baud) {
  if (baud < 300 || baud > 5000000) return 0;
  const uint32_t sclk = baud <= 250000 ? 1000000 : 80000000;
  const uint32_t div = (sclk << 4) / baud;
  if (div < 16 || (div >> 4) > 0xFFFFF) return 0;
  return (sclk << 4) / div;
}
static uint32_t portSpeed(uint8_t, uint32_t baud, bool apply) {   // port 0, UART0: the only UART bridge
  if (!apply) return uartRate(baud);
  Serial.updateBaudRate(baud);
  return Serial.baudRate();
}
#endif

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
static oep::WireRvswd wire(swio, 0, "oep.wire.swio");
static oep::TargetRiscvDm riscvDm(swio, 0);
static oep::DmConsole consoleDriver(dm, phy);
static oep::TargetConsoleStream console(swio, consoleDriver, 0);
static oep::PinTable pins(kChannels);
// PinTable owners: gpio 1, uart 2 (the I2C device is 3, the SPI device 6, the SWIO wire 0xf0, the analog 7)
static oep::FixtureGpio gpio(pins, 0, 1);
static oep::FixtureUart uart(pins, Serial2, 0, 2);
static oep::SamplerCapture capture(endpoint, pins);
static oep::P4I2cTarget i2c(pins);
static oep::P4SpiTarget spi(pins);
static oep::Binds binds;
static oep::ProbeConfig config(endpoint, binds);
// ADC1 in DMA mode on the pins brought out (32-36, 39), and a group that starts it with the sampler
static constexpr uint64_t kAdc1 = kChannels & (0xffull << 32);
static oep::AnalogCapture analog(endpoint, kAdc1, 0);
static oep::CaptureGroup group(endpoint, 0);
static uint8_t probeTlv[160];

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[17];
  oep::describeCore(w, "esp32", id, oep::platformUnitId(id, sizeof id), 40, kReserved | oep::platformUnusablePins());
  oep::describeChip(w);   // the MCU and its revision (a capture records what it was taken on)
  return w.ok() ? w.length() : 0;
}

void setup() {
  esp_log_level_set("*", ESP_LOG_NONE);   // no log on a port that carries OEP (probe guide §2.5): UART0 is the transport
  Serial.setRxBufferSize(8192);
  Serial.setTxBufferSize(8192);
  Serial.begin(kBootBaud);
  // Every channel genuinely Hi-Z until the host takes it (a pull on a target's USB line breaks its enumeration, E132).
  // Pins this chip's package uses itself (the PICO-D4's flash on GPIO16 / 17, a PSRAM): never a channel, never parked.
  // The saved settings are read first: their disable items' channels are never parked (probe.config §2: applied
  // before any idle / park; applySaved below gives them back if the settings are not applied).
  const uint64_t unusable = oep::platformUnusablePins();
  pins.forbid(unusable);
  pins.setInputOnly(0xf0ull << 32);   // GPIO34-39: no output idle (probe.config §1, rejected unsupported)
  config.load();
  pins.setDisabled(config.savedDisabled());
  oep::platformParkMask(kChannels & ~unusable & ~pins.disabledMask());
  endpoint.setRawPorts(&binds);
  swio.pin_choice = kSwioChoice & ~unusable;
  swio.pins = &pins;
  swio.reset_allowed = kChannels & ~unusable;   // attach's reset TLV: the channel the host names (no default), nobody holding it
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setBootId(esp_random());
#if OEP_PORT_SPEED
  endpoint.setPortSpeed(portSpeed, kBootBaud);
#endif
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
  analog.setPins(&pins, 7);   // its pads go analog: claimed against wires and settings
  endpoint.add(analog);   // after config: the fns before it keep their numbers
  endpoint.add(group);
  group.addTrack(capture, capture);
  group.addTrack(analog, analog);
  // Last, once every interface is added: the saved settings name fns, and are kept only for the same interface list
  // (applied before the analog and the group were added, they never matched it: unreadable after every reboot, 0.0.11-0.0.16).
  // In the order of probe.config §2: every idle (outputs driven) first, then the plans, the uarts, and the at-boot
  // slots' attach last (on its poll), so a target powered through an output idle is up before it.
  config.applySaved();   // read by config.load() at the top
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
