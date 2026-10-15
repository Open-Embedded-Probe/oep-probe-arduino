// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe
// The portable core of the library served over TCP on a PC (no hardware): the endpoint (fn 0, oep.probe.plan's
// absence, sessions, resend, the TCP listener's connection slots of OepTcp.h on POSIX sockets) answers a host or an
// independent checker at tcp://127.0.0.1:PORT. It prints "PORT <n>" once listening and runs until stdin closes.
//   g++ -std=gnu++17 -Itests/host/shim -Isrc -o serve tests/host/serve_tcp.cpp src/OepFrame.cpp src/OepEndpoint.cpp
//   ./serve 47011 [unit_id]
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "OepEndpoint.h"
#include "OepFrame.h"
#define ARDUINO_ARCH_ESP32 1   // OepTcp.h's lwIP sockets: the host's (shim/lwip/sockets.h)
#include "OepTcp.h"

uint32_t g_millis = 0;

using namespace oep;

class NullStream final : public Stream {   // the endpoint's first transport: a UART bridge nobody talks on
 public:
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  size_t write(uint8_t) override { return 1; }
  size_t write(const uint8_t *, size_t n) override { return n; }
  int availableForWrite() override { return 4096; }
};

static uint32_t nowMs() {
  timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  static const uint64_t start = uint64_t(t.tv_sec) * 1000 + t.tv_nsec / 1000000;
  return static_cast<uint32_t>(uint64_t(t.tv_sec) * 1000 + t.tv_nsec / 1000000 - start);
}

int main(int argc, char **argv) {
  const int port = argc > 1 ? atoi(argv[1]) : 47011;
  const char *unit = argc > 2 ? argv[2] : "arduino-host-1";
  static NullStream uart;
  static uint8_t rx_uart[1100], tx[2200], rx[3][1100], desc[64];
  static Endpoint ep(uart, rx_uart, sizeof rx_uart, tx, sizeof tx, {1024, 4096, 4}, Endpoint::kUartBridge);
  timespec t;
  clock_gettime(CLOCK_REALTIME, &t);
  ep.setBootId(static_cast<uint32_t>(t.tv_nsec ^ (t.tv_sec << 7) ^ getpid()));
  ep.setMaxOpMs(1000);
  TlvWriter w(desc, sizeof desc);
  w.text(kCoreUnitId, unit);
  ep.setProbeDescription(desc, w.length());
  static TcpListener<3> tcp;
  uint8_t *rxs[3] = {rx[0], rx[1], rx[2]};
  if (!ep.addTcpListener(tcp.slots(), rxs, sizeof rx[0], tcp.kSlots)) return 2;
  g_millis = nowMs();
  if (!tcp.begin(static_cast<uint16_t>(port))) { perror("listen"); return 1; }
  printf("PORT %d\n", port);
  fflush(stdout);
  for (;;) {
    pollfd in{0, POLLIN, 0};
    if (::poll(&in, 1, 0) > 0) {
      char c;
      if (::read(0, &c, 1) <= 0) break;   // stdin closed: stop
    }
    g_millis = nowMs();
    tcp.poll();
    ep.poll();
    for (size_t k = 0; k < tcp.kSlots; ++k) tcp.slot(k).flush();
    usleep(200);
  }
  tcp.end();
  return 0;
}
