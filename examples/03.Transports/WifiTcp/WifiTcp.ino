// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP over TCP on an ESP32 with its own radio (oep-spec docs/oep-transports.ja.md §1): the board's serial port as
// transport 0 and a TCP listener as transport 1, up to three connections at once - each its own transport, all sharing
// the one session and lock. No credentials in the build: the networks are oep.probe.config's wifi item (item 0x08,
// probe.config §1.4), set over the serial port and saved like any other setting; the probe joins the first that works.
//
// Discovery (transports §3): this sketch announces _oep._tcp by DNS-SD over mDNS (host "oep-<unit_id>", TXT
// unit_id), and the config state's wifi TLV gives the address (docs/implementation-limits.ja.md). Then on the PC:
//
//   oep dump --port tcp://<address>:7450
//
// Use TCP only on a trusted network or inside an authenticated tunnel: OEP has no authentication (transports §1).
#include <OepBind.h>
#include <OepConfig.h>
#include <OepEndpoint.h>
#include <OepPlatform.h>
#include <OepWifi.h>

static constexpr uint16_t kPort = 7450;
static constexpr size_t kConnections = 3;

static constexpr uint16_t kMaxFrame = 1024;   // confirm's max_frame on every transport (the UART and each connection)
// probe.config §1.4: with the wifi item, every transport answers max_frame 112 or more (one set of the longest wifi item)
static_assert(kMaxFrame >= oep::kWifiMinMaxFrame, "a probe with the wifi item answers max_frame >= wifi_min_max_frame");
static uint8_t rxBuffer[1100];   // the serial port's encoded frame (a little more than max_frame)
static_assert(sizeof rxBuffer >= oep::cobsFrameMax(kMaxFrame), "the serial port's encoded candidate");
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {kMaxFrame, 4096, 4},
                              oep::Endpoint::kUartBridge);
static oep::TcpListener<kConnections> tcp;
static uint8_t rxTcp[kConnections][kMaxFrame + 2];   // one frame of max_frame per connection
static oep::WifiStation wifi(tcp, kPort);
static oep::Binds binds;
static oep::ProbeConfig config(endpoint, binds);   // the wifi item (and save) lives here
static uint8_t probeTlv[64];

void setup() {
  esp_log_level_set("*", ESP_LOG_NONE);   // no log on a port that carries OEP (probe guide §3)
  Serial.begin(115200);
  uint8_t id[17];
  const size_t n = oep::platformUnitId(id, sizeof id);
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  oep::describeCore(w, "wifi-tcp-probe", id, n, 0);
  endpoint.setProbeDescription(probeTlv, w.ok() ? w.length() : 0);
  uint8_t *rx[kConnections];
  for (size_t k = 0; k < kConnections; ++k) rx[k] = rxTcp[k];
  endpoint.addTcpListener(tcp.slots(), rx, sizeof rxTcp[0], kConnections);   // after every other transport
  endpoint.add(config);
  char unit[18] = {};
  memcpy(unit, id, n < sizeof unit - 1 ? n : sizeof unit - 1);
  wifi.begin(unit);
  config.setWifi(&wifi);
  config.load();
  config.applySaved();   // the saved networks: joined from here on
}

void loop() {
  wifi.poll();       // the list of networks, the listener once there is an address
  tcp.poll();        // new connections, what waits to go out
  endpoint.poll();   // every transport: serial and each TCP connection
  config.poll();
}
