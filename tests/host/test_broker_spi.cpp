// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host test: a relaying broker's sequence (oep-core §3.1) on a UART bridge, to the classic ESP32's SPI target on the fake
// spi_slave driver and the fake ESP-IDF GPIO / IPC (tests/host/shim). The broker (ch32rv 0.18.0) opens one session
// without a session_id in the header (role 0x01, the id in open's payload), raises the port (port_speed try, confirms at
// the new rate, link_source / link_sink without a session, commit in the session), keeps it alive, and relays a client's
// 0x81 requests with its own session_id and its 0x01 ones without one, with confirms of its own (0x01) in between. Every
// answer is completed: on 0.0.28+ec38b1d the first spi-target configure installed the GPIO ISR service from inside an
// IPC call, which never returned on the chip; the bench then saw the probe come back from a restart and the next 0x81
// answered no_session.
#include <stdio.h>

#include <vector>

#include "OepEndpoint.h"
#include "OepFrame.h"
#include "OepP4SpiTarget.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using Bytes = std::vector<uint8_t>;
using namespace oep;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

class MemStream final : public Stream {
 public:
  Bytes rx, tx;   // host -> probe, probe -> host
  size_t at = 0;
  int available() override { return static_cast<int>(rx.size() - at); }
  int read() override { return at < rx.size() ? rx[at++] : -1; }
  int peek() override { return at < rx.size() ? rx[at] : -1; }
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override { tx.insert(tx.end(), b, b + n); return n; }
  int availableForWrite() override { return 4096; }
};

// The host side of the framing: 0x00 <COBS(message + CRC)> 0x00, and the last frame that came back.
static Bytes frame(const Bytes &message) {
  MemStream s;
  writeCobsFrame(s, message.data(), message.size());
  return s.tx;
}
static bool unframe(const Bytes &enc, Bytes &out) {
  out.clear();
  size_t in = 0;
  while (in < enc.size()) {
    const uint8_t code = enc[in++];
    if (code == 0 || in + code - 1 > enc.size()) return false;
    out.insert(out.end(), enc.begin() + in, enc.begin() + in + code - 1);
    in += code - 1;
    if (code != 0xff && in < enc.size()) out.push_back(0);
  }
  if (out.size() < 3) return false;
  const uint16_t got = static_cast<uint16_t>(out[out.size() - 2] | out[out.size() - 1] << 8);
  out.resize(out.size() - 2);
  return crc16Ccitt(out.data(), out.size()) == got;
}
static Bytes lastFrame(const Bytes &tx) {
  Bytes last, msg;
  for (size_t i = 0; i < tx.size();) {
    if (tx[i] != 0) { ++i; continue; }
    size_t j = i + 1;
    while (j < tx.size() && tx[j] != 0) ++j;
    if (j < tx.size() && j > i + 1 && unframe(Bytes(tx.begin() + i + 1, tx.begin() + j), msg)) last = msg;
    i = j;
  }
  return last;
}

static Bytes u16(uint16_t v) { return {uint8_t(v), uint8_t(v >> 8)}; }
static Bytes u32(uint32_t v) { return {uint8_t(v), uint8_t(v >> 8), uint8_t(v >> 16), uint8_t(v >> 24)}; }
static void append(Bytes &to, const Bytes &b) { to.insert(to.end(), b.begin(), b.end()); }

static uint32_t speedHook(uint8_t, uint32_t baud, bool) { return baud; }

struct Probe {
  MemStream stream;
  uint8_t rx[1100], tx[1100];
  Endpoint ep{stream, rx, sizeof rx, tx, sizeof tx, {512, 1024, 2}, Endpoint::kUartBridge};
  uint16_t corr = 0;
  // One request; the answer's resolution, detail and payload (empty: no answer).
  Bytes send(uint16_t fn, uint8_t op, const Bytes &payload, bool session, uint32_t id) {
    ++corr;
    Bytes m = {uint8_t(session ? 0x81 : 0x01)};
    append(m, u16(corr));
    append(m, u16(fn));
    m.push_back(op);
    if (session) append(m, u32(id));
    append(m, payload);
    const Bytes f = frame(m);
    stream.rx.insert(stream.rx.end(), f.begin(), f.end());
    stream.tx.clear();
    ep.poll();
    const Bytes r = lastFrame(stream.tx);
    if (r.size() < 5 || r[0] != 0x02 || r[1] != uint8_t(corr) || r[2] != uint8_t(corr >> 8)) return {};
    return Bytes(r.begin() + 3, r.end());
  }
};
static bool completed(const Bytes &r) { return r.size() >= 2 && r[0] == kResolutionCompleted && r[1] == kOutcomeSuccess; }
static Bytes confirm() { return {'O', 'E', 'P', '?', 1, 1}; }

int main() {
  static PinTable pins((1ull << 4) | (1ull << 5) | (1ull << 6) | (1ull << 7));
  static P4SpiTarget spi(pins);
  static Probe p;
  p.ep.add(spi);
  p.ep.setPins(&pins);
  p.ep.setPortSpeed(speedHook, 115200);
  const uint16_t fn = 1;
  const uint32_t sid = 0x5a17c3e1;   // the broker's random non-zero id

  CHECK(completed(p.send(0, reg::core::kOpConfirm, confirm(), false, 0)));
  Bytes open = u32(sid);
  append(open, u32(10000));
  open.push_back(0);
  CHECK(completed(p.send(0, reg::core::kOpOpen, open, false, 0)));
  // port_speed: try in the session, confirms at the new rate (0x01), the link measured without a session, commit
  Bytes speed = {0};
  append(speed, u32(921600));
  speed.push_back(reg::core::kPortSpeedStepTry);
  append(speed, u16(2000));
  append(speed, u32(0));
  CHECK(completed(p.send(0, reg::core::kOpPortSpeed, speed, true, sid)));
  for (int i = 0; i < 3; ++i) CHECK(completed(p.send(0, reg::core::kOpConfirm, confirm(), false, 0)));
  CHECK(completed(p.send(0, reg::core::kOpLinkSource, u32(64), false, 0)));
  CHECK(completed(p.send(0, reg::core::kOpLinkSink, Bytes(64, 0x55), false, 0)));
  speed[5] = reg::core::kPortSpeedStepCommit;
  CHECK(completed(p.send(0, reg::core::kOpPortSpeed, speed, true, sid)));
  CHECK(completed(p.send(0, reg::core::kOpKeepalive, {}, true, sid)));
  CHECK(completed(p.send(fn, P4SpiTarget::kOpStatus, {}, false, 0)));   // a client's lock-free request, 0x01

  // a client's plan and configure, relayed as 0x81 with the broker's id; a broker confirm between them
  Bytes plan;
  const uint8_t roles[][2] = {{P4SpiTarget::kRoleSck, 4}, {P4SpiTarget::kRoleMosi, 5}, {P4SpiTarget::kRoleMiso, 6},
                              {P4SpiTarget::kRoleCs, 7}};
  for (const auto &r : roles) {
    plan.push_back(kTagRoleAssignment | kTagCritical);
    plan.push_back(5);
    append(plan, u16(fn));
    plan.push_back(r[0]);
    append(plan, u16(r[1]));
  }
  g_millis += 14000 - 10000;   // the bench's 14 s in, keepalives every second
  CHECK(completed(p.send(0, reg::core::kOpKeepalive, {}, true, sid)));
  CHECK(completed(p.send(0, reg::core::kOpPlanApply, plan, true, sid)));
  CHECK(completed(p.send(0, reg::core::kOpConfirm, confirm(), false, 0)));
  const Bytes configured = p.send(fn, P4SpiTarget::kOpConfigure, {0, 0}, true, sid);
  CHECK(completed(configured));
  CHECK(g_fake_ipc_deadlocks == 0);   // 0.0.28+ec38b1d: the install's IPC call from inside an IPC call
  CHECK(g_fake_gpio.service && g_fake_gpio.service_core == 0);
  Bytes arm = u16(4);
  append(arm, u16(2));
  arm.push_back(0x81);
  arm.push_back(0x42);
  CHECK(completed(p.send(fn, P4SpiTarget::kOpArm, arm, true, sid)));
  for (uint8_t mode = 1; mode <= 3; ++mode)   // configured again: the service is not installed twice
    CHECK(completed(p.send(fn, P4SpiTarget::kOpConfigure, {mode, 0}, true, sid)));
  CHECK(g_fake_gpio_error_logs == 0 && g_fake_ipc_deadlocks == 0);
  const Bytes st = p.send(fn, P4SpiTarget::kOpStatus, {}, true, sid);
  CHECK(completed(st) && st.size() >= 4 && st[2] == 1 && st[3] == 3);   // running, mode 3
  CHECK(completed(p.send(0, reg::core::kOpKeepalive, {}, true, sid)));   // the session still held

  printf("broker-spi: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
