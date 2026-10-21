// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Classic ESP32 (profile esp32), on a board with a USB-UART bridge (DevKitC and the like).
//
// Transport: UART0 through the bridge at 115200 (probe guide §5; a host may raise it for its session: port_speed,
// below) - serial port 0: OEP frames (0x00 <COBS> 0x00) and the raw bytes of its bind on one line (oep-transports §4).
// With Wi-Fi (the Wifi choice, in by default): TCP, transport 1 - port kTcpPort, up to three connections
// at once, each a transport of its own (oep-transports §1); the networks come from oep.probe.config's wifi item (no
// network until one is set; no credentials in the build), announced by mDNS as _oep._tcp (OepWifi.h). The bridge's auto-reset
// circuit resets the ESP32 when the port is opened with DTR / RTS in the wrong order: a host opens it with both on
// (host guide §1). The bridge's USB ID is not the project's VID:PID: a host reaches this probe by the user choosing its
// port, then asking (confirm).
//
// Interfaces (revision 1): fn 0 (the core); oep.wire.swio + oep.target.riscv-dm + oep.target.console (WCH CH32V00x, one wire);
// oep.fixture.gpio / uart / capture (the core-1 GPIO sampler: up to 8 lines, 1-300 kHz, one-shot); the ESP-IDF SPI / I2C
// devices oep.fixture.spi-target / i2c-target; oep.probe.config (saved in NVS); oep.fixture.analog
// (ADC1 on 32-36 / 39) and oep.fixture.capture-group (the analog with the sampler); oep.probe.link; oep.probe.plan and
// oep.probe.restart (the endpoint's own, listed last). The host chooses every
// pin: SWIO any output GPIO below 32, the reset line and the fixtures any channel below. Each is a choice (OepFeatures.h:
// a weak oepWithX() here adds it, a sketch's own replaces it; OepChoice.h): Swio (and Console in it), Gpio, Uart, I2c, Spi,
// Capture, Config, Analog, Group, Link; Wifi (the TCP transport) and PortSpeed (port_speed on UART0); all in by default.
//
// Cores: loop() (all of OEP, the SWIO wire, every interrupt the sketch sets up: UART0, the fixture UART, the SPI / I2C
// devices, the ADC) and the Arduino events run on core 0, with the Wi-Fi driver and the TCP/IP stack (pinned to core 0
// by the core's build); core 1 is the logic sampler's alone (its windows mask that core for up to 250 ms). The profile
// esp32 builds with LoopCore=0 and EventsCore=0 (sketch.yaml); a build refuses another arrangement (with the
// sampler on the radio's core, a window held the Wi-Fi driver up: 0.0.29-dev 028554d had to cut spans to 50 ms and
// segments to 25 ms, and a trigger search was blind while the radio had the core).
#pragma once
#if !defined(ARDUINO_RUNNING_CORE) || !defined(ARDUINO_EVENT_RUNNING_CORE) || ARDUINO_RUNNING_CORE != 0 || \
    ARDUINO_EVENT_RUNNING_CORE != 0
#error "OepProbe (classic ESP32): build with LoopCore=0 and EventsCore=0 (the sketch.yaml profile esp32): core 1 is the sampler's"
#endif
#include <OepAnalog.h>
#include <OepBind.h>
#include <OepBootGuard.h>
#include <OepCaptureGroup.h>
#include <OepCh32Dm.h>
#include <OepChoice.h>
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
#include <OepWifi.h>

#include "OepFeatures.h"

// UART0 runs through the board's USB-UART bridge (no flow control): long bursts of pipelined responses lost bytes
// (2026-09-22, 2026-09-24), so few 512-byte frames are in flight. Two (window 1024) rather than one: with port_speed
// raised the link waits on the bridge's round trip, and a second frame in flight nearly doubles what moves (1500000:
// 58-65 -> 78 KB/s on an ATOM's FTDI); at 115200 it changes nothing (9.3 -> 9.5 KB/s, no more broken frames) (oep-spec
// docs/uart-speed-negotiation.ja.md §3b, 2026-10-01).
static constexpr uint16_t kMaxFrame = 512;   // confirm's max_frame on every transport (UART0 and each TCP connection)
static uint8_t rxBuffer[1024];   // the encoded candidate: cobsFrameMax(512)
static_assert(sizeof rxBuffer >= oep::cobsFrameMax(kMaxFrame), "UART0's encoded candidate");
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {kMaxFrame, 1024, 2},
                              oep::Endpoint::kUartBridge);

// TCP (oep-transports §1): the port and the discovery are this implementation's (docs/implementation-limits).
static constexpr uint16_t kTcpPort = 7450;
static constexpr size_t kTcpConnections = 3;
static oep::Slot<oep::TcpListener<kTcpConnections>> tcp;
static oep::Slot<oep::WifiStation> wifi;
// probe.config §1.4: with the wifi item, every transport answers max_frame 112 or more (one set of the longest wifi item)
static_assert(kMaxFrame >= oep::reg::kLimitWifiMinMaxFrame, "a probe with the wifi item answers max_frame >= wifi_min_max_frame");
// A slot's send buffer holds all a host within its window can have outstanding (OepTcp.h): two answers in flight, the
// push queue (1024, the endpoint's default) and one push frame, the event queue (32 x 48 bytes)
static_assert(oep::TcpSlot::kTxBytes >= 2 * (kMaxFrame + 2) + 1024 + (kMaxFrame + 2) + 32 * 48,
              "a TCP slot never waits for a host keeping to its window");

// port_speed (oep-if-link §3): the host may raise UART0's baud for its session; every revert goes back to 115200, the
// boot speed. The PortSpeed choice (without it no describe port_speed, the op unknown_operation).
static constexpr uint32_t kBootBaud = 115200;
// UART0's RX interrupt fires at kRxFifoFull bytes in the 128-byte hardware FIFO (arduino-esp32's default 120 left 8 bytes:
// 160 us at 500000, 87 us at 921600, before a byte is lost); the SWIO frames keep loop()'s core - where the UART's
// interrupt runs - from it for up to SwioPhy::kIrqOffMaxUs. The rest of the FIFO must outlast that at the fastest
// port_speed in use (2000000 checked; 96 bytes there 480 us): a byte lost from a request breaks its frame, and three
// broken in a row revert a raised speed (oep-if-link §3).
static constexpr uint8_t kRxFifoFull = 32;
static constexpr uint32_t kUartFifo = 128, kFastestCheckedBaud = 2000000;
static_assert((kUartFifo - kRxFifoFull) * 10ull * 1000000ull / kFastestCheckedBaud >= 2 * oep::SwioPhy::kIrqOffMaxUs,
              "UART0's RX FIFO must outlast an interrupt-off SWIO frame twice over");
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

// oep.probe.restart (oep-if-restart): esp_restart, UART0 back at 115200 as at every boot. restart_max_ms (its describe):
// the UART bridge stays on the bus (no re-enumeration, the host's port stays), and the chip is back in about 0.5 s -
// the ROM (its banner goes out on UART0 as raw bytes), the bootloader checking the app image (about 0.4 MB), setup()
// reading the settings; the host's reopen and its confirms at 115200 add little. 1500 ms
// is about three times that (an estimate from the boot path, to be measured on the bench).
// With Wi-Fi (its oepAddWifi sets these) the TCP transport is back once the probe has booted, joined and has its address again: 5.5-6.9 s
// from the answer to the first confirm answered on a new connection, ten restarts on the ATOM (2026-10-07, with a scan
// before joining then); 15000 is twice that. A list whose earlier entries do not connect takes longer (WifiStation::kTryMs each,
// docs/implementation-limits).
static constexpr uint32_t kRestartMaxMs = 1500, kRestartWifiMaxMs = 15000;

// The GPIOs a DevKitC brings out, less UART0 (1, 3: the transport), the SPI flash (6-11) and the boot straps (0, 2, 12,
// 15). 34-39 are inputs only.
static constexpr uint64_t kChannels = (1ull << 4) | (1ull << 5) | (1ull << 13) | (1ull << 14) | (1ull << 16) | (1ull << 17) |
                                      (1ull << 18) | (1ull << 19) | (1ull << 21) | (1ull << 22) | (1ull << 23) | (1ull << 25) |
                                      (1ull << 26) | (1ull << 27) | (1ull << 32) | (1ull << 33) | (1ull << 34) | (1ull << 35) |
                                      (1ull << 36) | (1ull << 39);
static constexpr uint64_t kReserved = ((1ull << 40) - 1) & ~kChannels;
// GPIO34-39: inputs only, no internal pulls (0xf0 << 32 here was GPIO36-39 only: 34 / 35 were offered to I2C SDA / SCL,
// UART TX, SPI MISO and attach's reset, and a plan putting SDA / SCL on them was taken, then configure failed)
static constexpr uint64_t kInputOnly = oep::kEsp32InputOnlyPins;
static constexpr uint64_t kSwioChoice = kChannels & 0xffffffffull;   // SwioPhy drives GPIO0-31
static constexpr uint16_t kUnset = 0xfffe;                           // no pin chosen yet

static oep::PinTable pins(kChannels);
static uint8_t probeTlv[256];   // with the firmware text's note of the boot before (BootGuard::lastBoot)
// PinTable owners: gpio 1, uart 2 (the I2C device is 3, the SPI device 6, the SWIO wire 0xf0, the analog 7)

// What one choice leaves to the others, each set only from its own oepAddX(): its part of the settings, its track in the
// group, its loop work; the settings' fn at its place, what comes last
static oep::Hooks<void (*)(oep::ProbeConfig &), 4> settingsParts;
static oep::Hooks<void (*)(oep::CaptureGroup &), 2> groupTracks;
static oep::Hooks<void (*)(), 10> loopWork;
static oep::Hooks<void (*)(), 1> settingsFn;
static oep::Hooks<void (*)(), 1> lastly;

// The fixture UART's interrupt stays on loop()'s core (core 0): core 1 is the sampler's, with interrupts off for up to
// SamplerCapture's 250 ms bursts. On core 0 the SWIO frames hold it off; its RX FIFO threshold (oep::kUartRxFifoFull,
// set by platformUartBegin; arduino-esp32's 120 left 8 bytes, 40 us at 2000000) leaves room for one twice over at the
// fastest rate configure takes.
static_assert((oep::kUartRxFifo - oep::kUartRxFifoFull) * 10ull * 1000000ull / oep::FixtureUart::kMaxBaud >=
                  2 * oep::SwioPhy::kIrqOffMaxUs,
              "the fixture UART's RX FIFO must outlast an interrupt-off SWIO frame twice over");
static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[17];
  oep::describeCore(w, "esp32", id, oep::platformUnitId(id, sizeof id), 40,
                    oep::BootGuard::lastBoot());
  oep::describeChip(w);   // the MCU and its revision (a capture records what it was taken on)
  return w.ok() ? w.length() : 0;
}

// The SWIO wire's parts, the console's too (its choice is made in the wire's: no wire, no console)
struct SwioParts {
  oep::SwioPhy phy;
  oep::Ch32Dm dm{phy};
  oep::DebugPort port{dm, kUnset, 0xffff};   // one wire: swclk stays 0xffff
  oep::WireRvswd wire{port, 0, "oep.wire.swio"};
  oep::TargetRiscvDm riscvDm{port, 0};
  explicit SwioParts(uint64_t unusable) {
    port.pin_choice = kSwioChoice & ~unusable;
    port.pins = &pins;
    port.reset_allowed = kChannels & ~unusable & ~kInputOnly;   // attach's reset TLV: the channel the host names (no default), nobody holding it
  }
};
static oep::Slot<SwioParts> swio;

void oepAddSwio() {
  SwioParts &w = swio.make(oep::platformUnusablePins());
  endpoint.add(w.wire);
  endpoint.add(w.riscvDm);
  oepWithConsole();
}

void oepAddConsole() {
  static oep::Slot<oep::DmConsole> driver;
  static oep::Slot<oep::TargetConsoleStream> console;
  console.make(swio->port, driver.make(swio->dm, swio->phy), 0).setMaxRead(480);   // 512-byte frames
  endpoint.add(*console);
  settingsParts.add([](oep::ProbeConfig &c) { c.addPlace(swio->wire, *console); });
  loopWork.add([] { console->poll(); });
}

void oepAddGpio() {
  static oep::Slot<oep::FixtureGpio> gpio;
  endpoint.add(gpio.make(pins, 0, 1));
}

// The fixture UART's interrupt stays on loop()'s core (core 0): core 1 is the sampler's, with interrupts off for up to
// SamplerCapture's 250 ms bursts. On core 0 the SWIO frames hold it off; its RX FIFO threshold (oep::kUartRxFifoFull,
// set by platformUartBegin; arduino-esp32's 120 left 8 bytes, 40 us at 2000000) leaves room for one twice over at the
// fastest rate configure takes.
static_assert((oep::kUartRxFifo - oep::kUartRxFifoFull) * 10ull * 1000000ull / oep::FixtureUart::kMaxBaud >=
                  2 * oep::SwioPhy::kIrqOffMaxUs,
              "the fixture UART's RX FIFO must outlast an interrupt-off SWIO frame twice over");
void oepAddUart() {
  static oep::Slot<oep::FixtureUart> uart;
  endpoint.add(uart.make(pins, Serial2, 0, 2));
  settingsParts.add([](oep::ProbeConfig &c) { c.addUart(*uart); });
  loopWork.add([] { uart->poll(); });
}

void oepAddI2c() {
  static oep::Slot<oep::P4I2cTarget> i2c;
  endpoint.add(i2c.make(pins));
  loopWork.add([] { i2c->service(); });
}

void oepAddSpi() {
  static oep::Slot<oep::P4SpiTarget> spi;
  endpoint.add(spi.make(pins));
  loopWork.add([] { spi->service(); });
}

// The sampler and the SWIO wire never on the GPIO registers' bus at once (OepWireGate.h): a request's frames wait out a
// sampler window; capture.poll() in loop() lets the wire go so that the next window can open.
void oepAddCapture() {
  static oep::Slot<oep::SamplerCapture> capture;
  endpoint.add(capture.make(endpoint, pins));
  groupTracks.add([](oep::CaptureGroup &g) { g.addTrack(*capture, *capture); });
  loopWork.add([] { capture->poll(); });
}

// oep.probe.config, saved in NVS: read first (its disable items' channels are never parked: probe.config §2, disable
// before anything else; applySaved gives them back if the settings are not applied), its fn after the capture's
void oepAddConfig() {
  static oep::Slot<oep::Binds> binds;
  static oep::Slot<oep::ProbeConfig> config;
  oep::ProbeConfig &c = config.make(endpoint, binds.make());
  c.load();
  pins.setDisabled(c.savedDisabled());
  endpoint.setRawPorts(&*binds);
  settingsFn.add([] {
    endpoint.add(*config);   // the fns before it keep their numbers
    config->usePassthroughs();   // a serial port's raw stream carries them (probe.config §1.1)
    settingsParts.run(*config);
    config->setPins(&pins);
    loopWork.add([] { config->poll(); });
  });
  // Last, once every interface is added: the saved settings name fns, and are kept only for the same interface list
  // (applied before the analog and the group were added, they never matched it: unreadable after every reboot,
  // 0.0.11-0.0.16). probe.config §2: disable and idle (outputs driven) before every other item; the at-boot slots' attach
  // comes on config.poll(), so a target powered through an output idle is up before it. No USB device of its own (a
  // bridge carries the UART): no gate on the at-boot attach. After BootGuard::kSafeAfter fast crash-boots the saved
  // at-boot slots wait for the host.
  lastly.add([] {
    if (oep::BootGuard::safe()) config->skipBootAttach();
    config->applySaved();
  });
}

// ADC1 in DMA mode on the pins brought out (32-36, 39)
static constexpr uint64_t kAdc1 = kChannels & (0xffull << 32);
void oepAddAnalog() {
  static oep::Slot<oep::AnalogCapture> analog;
  analog.make(endpoint, kAdc1, 0).setPins(&pins, 7);   // its pads go analog: claimed against wires and settings
  endpoint.add(*analog);
  groupTracks.add([](oep::CaptureGroup &g) { g.addTrack(*analog, *analog); });
  loopWork.add([] { analog->poll(); });
}

// oep.fixture.capture-group: the analog with the sampler (the tracks of those in)
void oepAddGroup() {
  static oep::Slot<oep::CaptureGroup> group;
  endpoint.add(group.make(endpoint, 0));
  groupTracks.run(*group);
  loopWork.add([] { group->poll(); });
}

void oepAddLink() {
  static oep::Slot<oep::Link> link;
  endpoint.add(link.make(endpoint));   // oep.probe.link: the link test and port_speed (oep-if-link)
}

// The answer to restart out on a TCP connection before the reset: the slots' buffers into the socket, then a moment
// for the network stack to send them (esp_restart closes nothing).
static void restartWifi() {
  tcp->stopListening();   // a host reconnecting now is refused, not accepted by this boot (OepTcp.h)
  for (size_t k = 0; k < kTcpConnections; ++k) tcp->slot(k).flushAll();
  delay(100);
  oep::platformRestart();
}

// After the UART: the listener's entry is the last (its connections are transports 1..3, describe entry 1)
void oepAddWifi() {
  static uint8_t rxTcp[kTcpConnections][kMaxFrame + 2];   // one frame of max_frame each
  uint8_t *rx[kTcpConnections];
  for (size_t k = 0; k < kTcpConnections; ++k) rx[k] = rxTcp[k];
  endpoint.addTcpListener(tcp.make().slots(), rx, sizeof rxTcp[0], kTcpConnections);
  uint8_t id[17];
  const size_t n = oep::platformUnitId(id, sizeof id);
  char unit[18] = {};
  memcpy(unit, id, n < sizeof unit - 1 ? n : sizeof unit - 1);
  wifi.make(*tcp, kTcpPort).begin(unit);
  endpoint.setRestart(restartWifi, kRestartWifiMaxMs);
  settingsParts.add([](oep::ProbeConfig &c) { c.setWifi(&*wifi); });   // the wifi item: the networks to join (applied with the saved settings)
  loopWork.add([] {
    wifi->poll();
    tcp->poll();
  });
}

void oepAddPortSpeed() { endpoint.setPortSpeed(portSpeed, kBootBaud); }

// This build's choices (OepFeatures.h: a sketch's own replaces one)
__attribute__((weak)) void oepWithWifi() { oepAddWifi(); }
__attribute__((weak)) void oepWithPortSpeed() { oepAddPortSpeed(); }
__attribute__((weak)) void oepWithSwio() { oepAddSwio(); }
__attribute__((weak)) void oepWithConsole() { oepAddConsole(); }
__attribute__((weak)) void oepWithGpio() { oepAddGpio(); }
__attribute__((weak)) void oepWithUart() { oepAddUart(); }
__attribute__((weak)) void oepWithI2c() { oepAddI2c(); }
__attribute__((weak)) void oepWithSpi() { oepAddSpi(); }
__attribute__((weak)) void oepWithCapture() { oepAddCapture(); }
__attribute__((weak)) void oepWithConfig() { oepAddConfig(); }
__attribute__((weak)) void oepWithAnalog() { oepAddAnalog(); }
__attribute__((weak)) void oepWithGroup() { oepAddGroup(); }
__attribute__((weak)) void oepWithLink() { oepAddLink(); }

void setup() {
  oep::BootGuard::begin();   // loop() under the task watchdog, and the count of fast crash-boots
  esp_log_level_set("*", ESP_LOG_NONE);   // no log on a port that carries OEP (probe guide §3): UART0 is the transport
  Serial.setRxBufferSize(8192);
  Serial.setTxBufferSize(8192);
  Serial.begin(kBootBaud);
  Serial.setRxFIFOFull(kRxFifoFull);   // after begin(): begin() sets its own (120 above 57600 baud)
  // Every channel genuinely Hi-Z until the host takes it (a pull on a target's USB line breaks its enumeration, E132).
  // Pins this chip's package uses itself (the PICO-D4's flash on GPIO16 / 17, a PSRAM): never a channel, never parked.
  const uint64_t unusable = oep::platformUnusablePins();
  pins.forbid(unusable);
  // GPIO34-39 are inputs without pulls (ESP32 datasheet, GPIO): no output idle, no pull-up / pull-down idle (probe.config
  // §1, rejected unsupported), and not offered to a role that drives its line (UART TX, I2C SDA / SCL, SPI MISO, reset)
  pins.setInputOnly(kInputOnly);
  pins.setNoPull(kInputOnly);
  oepWithConfig();   // the saved settings read before the channels are parked
  oep::platformParkMask(kChannels & ~unusable & ~pins.disabledMask());
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setRestart(oep::platformRestart, kRestartMaxMs);
  oepWithPortSpeed();
  oepWithWifi();
  // The fns in this order, whichever are in (a build's set numbers them; a host finds each by name)
  oepWithSwio();   // and the console's choice
  oepWithGpio();
  oepWithUart();
  oepWithI2c();
  oepWithSpi();
  oepWithCapture();
  settingsFn.run();
  oepWithAnalog();   // after the settings: the fns before it keep their numbers
  oepWithGroup();
  oepWithLink();   // last: the fns before it keep their numbers
  lastly.run();
}

void loop() {
  // loop() on core 0 runs without blocking: core 0's idle task (which the task watchdog watches, and which frees the
  // stacks of tasks deleted on core 0) gets a tick every 50 ms; the Wi-Fi driver and the TCP/IP stack, above loop()'s
  // priority, take the core whenever they have work
  static uint32_t yielded = 0;
  if (millis() - yielded >= 50) {
    vTaskDelay(1);
    yielded = millis();
  }
  oep::BootGuard::poll();
  endpoint.poll();
  loopWork.run();
}
