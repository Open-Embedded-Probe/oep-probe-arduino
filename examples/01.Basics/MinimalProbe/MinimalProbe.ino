// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The smallest OEP probe: fn 0 (the core) alone, no interface - list is empty. It answers confirm, list, describe and
// clock (oep-spec docs/oep-core.ja.md §7), holds the session lock (§6) and nothing else - the frame every probe starts from. Flash it, then on the PC:
//
//   pip install oep-client-python
//   oep dump --port <the board's serial port>
//
// shows the probe: its model, unit id, transport and limits. Add interfaces with endpoint.add() (FixtureProbe,
// CustomInterface).
#include <OepEndpoint.h>
#include <OepPlatform.h>

// The transport: the board's Serial. On an RP2040 / RP2350 that is USB CDC; on a classic ESP32 it is UART0 through the
// board's USB-UART bridge. Both are serial ports: OEP frames go as 0x00 <COBS> 0x00 (transports §1), and the kind is what
// the describe tells the host (core §7.5).
#if defined(ARDUINO_ARCH_RP2040)
static constexpr uint8_t kTransport = oep::Endpoint::kUsbCdc;
static constexpr uint8_t kInterface = 0;   // the CDC communication interface's bInterfaceNumber (core §7.5)
#else
static constexpr uint8_t kTransport = oep::Endpoint::kUartBridge;
static constexpr uint8_t kInterface = 0xff;   // a UART bridge: no USB interface of the probe's (core §7.5)
#endif

// The endpoint reads a frame into rx and writes a result from tx. {max_frame, window, max_in_flight} are what confirm
// promises the host (core §7.1): 1024-byte frames, 4 KiB in flight, 8 requests at once. A serial port's rx holds the
// encoded frame, a little more than max_frame.
static uint8_t rxBuffer[1100];
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {1024, 4096, 8}, kTransport,
                              kInterface);

// fn 0's describe (core §7.5): the model, a unit id that is the same on every transport, the number of channels
// (pins) and the ones the probe keeps for itself. Written once, then handed to the endpoint.
static uint8_t probeTlv[64];

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[17];
  oep::describeCore(w, "minimal-probe", id, oep::platformUnitId(id, sizeof id), 0, 0);
  return w.ok() ? w.length() : 0;
}

void setup() {
#if defined(ARDUINO_ARCH_RP2040)
  Serial.ignoreFlowControl(true);   // answer whatever DTR the host left (probe-development-guide §1)
#elif defined(ARDUINO_ARCH_ESP32)
  esp_log_level_set("*", ESP_LOG_NONE);   // no log on a port that carries OEP (probe guide §3): UART0 is the transport
#endif
  Serial.begin(115200);
  endpoint.setProbeDescription(probeTlv, describeProbe());
}

void loop() {
  endpoint.poll();   // read what came, answer it; never blocks for long
}
