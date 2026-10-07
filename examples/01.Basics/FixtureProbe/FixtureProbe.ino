// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A test fixture: GPIO and a UART on the board's own pins, driven from the PC (pytest, a script). MinimalProbe plus two
// standard interfaces (oep-spec docs/oep-if-fixture.ja.md):
//
//   oep.fixture.gpio   set pins to input / output / open drain, read them
//   oep.fixture.uart   open a UART on two pins at a baud rate, send, and read what came (position-addressed)
//
// Which pins they use is the host's choice at run time (the plan, oep-core §8): the probe declares the channels it offers
// (role_channels), the host assigns them, and a pin one interface holds is refused to another. From Python:
//
//   from oep_client import core, link
//   hst = link.open_host("<port>")
//   hst.open(3000)                # the session lock (3 s lease)
//   core.plan_apply(hst, [(1, 1, 2), (2, 1, 1), (2, 2, 0)])   # (fn, role, channel): gpio on pin 2; uart RX 1, TX 0
//
// or look first with `oep dump --port <port>`.
#include <OepEndpoint.h>
#include <OepFixture.h>
#include <OepPinTable.h>
#include <OepPlatform.h>

#if defined(ARDUINO_ARCH_RP2040)
static constexpr uint8_t kTransport = oep::Endpoint::kUsbCdc;
static constexpr uint8_t kInterface = 0;   // the CDC communication interface's bInterfaceNumber (core §7.5)
// A Pico's GP0-GP22 and GP26-GP28. The UART is UART0: RX / TX on GP1/0, GP13/12, GP17/16 or GP29/28.
static constexpr uint64_t kChannels = ((1ull << 23) - 1) | (0x7ull << 26);
static constexpr uint16_t kChannelCount = 30;
#define FIXTURE_UART Serial1
#else
static constexpr uint8_t kTransport = oep::Endpoint::kUartBridge;
static constexpr uint8_t kInterface = 0xff;   // a UART bridge: no USB interface of the probe's (core §7.5)
// A classic ESP32 DevKitC's free GPIOs (not UART0 1/3, the flash 6-11 or the straps 0/2/12/15; 34-39 input only).
static constexpr uint64_t kChannels = (1ull << 4) | (1ull << 5) | (1ull << 13) | (1ull << 14) | (1ull << 16) | (1ull << 17) |
                                      (1ull << 18) | (1ull << 19) | (1ull << 21) | (1ull << 22) | (1ull << 23) | (1ull << 25) |
                                      (1ull << 26) | (1ull << 27) | (1ull << 32) | (1ull << 33) | (1ull << 34) | (1ull << 35) |
                                      (1ull << 36) | (1ull << 39);
static constexpr uint16_t kChannelCount = 40;
#define FIXTURE_UART Serial2
#endif

static uint8_t rxBuffer[1100];
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {1024, 4096, 8}, kTransport,
                              kInterface);

// The pins the fixtures share. Each interface claims what its plan names under its own owner id (1, 2), so one never
// drives a pin another holds (core §8.1); a released pin goes back to Hi-Z (or its idle state).
static oep::PinTable pins(kChannels);
static oep::FixtureGpio gpio(pins, 0, 1);                  // instance 0, owner 1
static oep::FixtureUart uart(pins, FIXTURE_UART, 0, 2);    // instance 0, owner 2

static uint8_t probeTlv[64];

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[17];
  // every pin that is not a channel is reserved: the host never plans it
  oep::describeCore(w, "fixture-probe", id, oep::platformUnitId(id, sizeof id), kChannelCount);
  return w.ok() ? w.length() : 0;
}

void setup() {
#if defined(ARDUINO_ARCH_RP2040)
  Serial.ignoreFlowControl(true);   // answer whatever DTR the host left (probe-development-guide §1)
#elif defined(ARDUINO_ARCH_ESP32)
  esp_log_level_set("*", ESP_LOG_NONE);   // no log on a port that carries OEP (probe guide §3): UART0 is the transport
  // GPIO34-39 are inputs without pulls: not offered to UART TX (its role_channels), no output or pull idle
  pins.setInputOnly(oep::platformInputOnlyPins());
  pins.setNoPull(oep::platformInputOnlyPins());
#endif
  Serial.begin(115200);
  oep::platformParkMask(kChannels);   // every channel Hi-Z until the host plans it
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.add(gpio);   // fn 1
  endpoint.add(uart);   // fn 2: the fn numbers follow the order of add()
}

void loop() {
  endpoint.poll();
  uart.poll();   // moves received bytes into the stream the host reads
}
