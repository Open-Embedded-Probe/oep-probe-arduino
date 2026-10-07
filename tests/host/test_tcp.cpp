// Host tests: a TCP listener's connection slots on the endpoint (oep-transports §1, §2, §3; core §7.1, §11.4). One
// describe entry for the listening socket (kind 6, interface 0xFF) after a UART bridge's, confirm on every connection
// naming it; an answer goes back on the connection the request came on; a frame split over many writes with long
// pauses is read whole (no gap rule on TCP), frames merged into one write are all answered; a length over max_frame
// closes the connection (Connection::drop); a new connection in a slot starts with a fresh reader. The one lock across
// connections (a second connection's open refused locked, lock_state from it). Notifications go to the subscriber's
// connection only: none to a new connection in that slot, none after it closed (the session stays, transports §3),
// and an open with the same session id from another connection moves them there (core §6.2). No transport is added
// after a listener.
#include <stdio.h>
#include <string.h>
#include <vector>

#include "OepEndpoint.h"
#include "OepFrame.h"

uint32_t g_millis = 1000;

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
  Bytes rx, tx;
  size_t at = 0;
  int available() override { return static_cast<int>(rx.size() - at); }
  int read() override { return at < rx.size() ? rx[at++] : -1; }
  int peek() override { return at < rx.size() ? rx[at] : -1; }
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override { tx.insert(tx.end(), b, b + n); return n; }
  int availableForWrite() override { return 4096; }
};

// A slot: one connection at a time, its bytes in memory.
class FakeSlot final : public Connection {
 public:
  Bytes rx, tx;
  size_t at = 0;
  uint32_t id = 0, next = 0;
  int drops = 0;
  void accept() { id = ++next; rx.clear(); tx.clear(); at = 0; }
  void close() { id = 0; rx.clear(); at = 0; }
  uint32_t connection() const override { return id; }
  void drop() override { ++drops; close(); }
  int available() override { return id ? static_cast<int>(rx.size() - at) : 0; }
  int read() override { return at < rx.size() ? rx[at++] : -1; }
  int peek() override { return at < rx.size() ? rx[at] : -1; }
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override { if (id) tx.insert(tx.end(), b, b + n); return n; }
  int availableForWrite() override { return id ? 4096 : 0; }
  void send(const Bytes &b) { rx.insert(rx.end(), b.begin(), b.end()); }
};

// Pushes on demand (an interface that notifies).
class Ticker final : public Interface {
 public:
  size_t ready = 0;
  const char *name() const override { return "io.github.test.ticker"; }
  uint16_t instance() const override { return 0; }
  uint8_t revision() const override { return 1; }
  bool offers(uint8_t op) const override { return op == 0x01; }
  bool notifies() const override { return true; }
  size_t pending() override { return ready; }
  size_t pull(uint8_t *out, size_t capacity) override {
    const size_t n = ready < capacity ? ready : capacity;
    memset(out, 0xAB, n);
    ready -= n;
    return n;
  }
  Result handle(uint8_t, const uint8_t *, size_t, uint8_t *, size_t) override { return completed(); }
};

static Bytes u32(uint32_t v) { return {uint8_t(v), uint8_t(v >> 8), uint8_t(v >> 16), uint8_t(v >> 24)}; }
static Bytes cat(Bytes a, const Bytes &b) { a.insert(a.end(), b.begin(), b.end()); return a; }
static Bytes request(uint16_t corr, uint16_t fn, uint8_t op, const Bytes &payload, uint32_t session = 0) {
  return cat(cat({0x01, uint8_t(corr), uint8_t(corr >> 8), uint8_t(fn), uint8_t(fn >> 8), op}, u32(session)), payload);
}
static Bytes framed(const Bytes &m) { return cat({uint8_t(m.size()), uint8_t(m.size() >> 8)}, m); }
static const Bytes kConfirm = {'O', 'E', 'P', '?', 1, 1};
// The messages in a slot's output (length(u16) message ...).
static std::vector<Bytes> messages(const Bytes &tx) {
  std::vector<Bytes> out;
  for (size_t i = 0; i + 2 <= tx.size();) {
    const size_t n = tx[i] | tx[i + 1] << 8;
    if (i + 2 + n > tx.size()) break;
    out.emplace_back(tx.begin() + i + 2, tx.begin() + i + 2 + n);
    i += 2 + n;
  }
  return out;
}
static bool okAnswer(const Bytes &m, uint16_t corr) {
  return m.size() >= 5 && m[0] == kRoleResult && (m[1] | m[2] << 8) == corr && m[3] == kResolutionCompleted && m[4] == 0;
}

int main() {
  static MemStream uart;
  static uint8_t rx_uart[1100], tx[600], rx[3][520];
  static FakeSlot slot[3];
  static Endpoint ep(uart, rx_uart, sizeof rx_uart, tx, sizeof tx, {512, 1024, 2}, Endpoint::kUartBridge);
  static Ticker ticker;
  Connection *slots[3] = {&slot[0], &slot[1], &slot[2]};
  uint8_t *rxs[3] = {rx[0], rx[1], rx[2]};
  CHECK(ep.add(ticker));   // fn 1
  CHECK(ep.addTcpListener(slots, rxs, sizeof rx[0], 3));
  static MemStream late;
  static uint8_t rx_late[600];
  CHECK(!ep.addTransport(late, rx_late, sizeof rx_late, Endpoint::kVendorBulk, 0));   // nothing after a listener
  CHECK(ep.transportCount() == 4);
  CHECK(!ep.isSerialPort(1));

  slot[0].accept();
  slot[1].accept();
  // confirm on each connection: the listener's entry, 1
  for (int k = 0; k < 2; ++k) {
    slot[k].send(framed(request(10 + k, 0, 0x01, kConfirm)));
    ep.poll();
    const auto ms = messages(slot[k].tx);
    CHECK(ms.size() == 1 && okAnswer(ms[0], 10 + k) && ms[0].size() == 5 + 21 && ms[0][5 + 20] == 1);
    CHECK(slot[1 - k].tx.empty() || k == 1);
    slot[k].tx.clear();
  }
  CHECK(uart.tx.empty());
  // describe: two entries, 0 the UART bridge, 1 TCP (kind 6, interface 0xFF), the slots not listed one by one
  {
    slot[0].send(framed(request(20, 0, 0x03, {0, 0, 0, 0})));
    ep.poll();
    const auto ms = messages(slot[0].tx);
    CHECK(ms.size() == 1 && okAnswer(ms[0], 20));
    std::vector<Bytes> entries;
    if (ms.size() == 1)
      for (size_t i = 6; i + 3 <= ms[0].size();) {
        const size_t len = ms[0][i + 1] | ms[0][i + 2] << 8;
        if (ms[0][i] == reg::core::kTlvDescribeTransport) entries.emplace_back(ms[0].begin() + i + 3, ms[0].begin() + i + 3 + len);
        i += 3 + len;
      }
    CHECK(entries.size() == 2 && entries[0] == Bytes({0, Endpoint::kUartBridge, 0xff}) &&
          entries[1] == Bytes({1, Endpoint::kTcp, 0xff}));
    slot[0].tx.clear();
  }
  // a frame split byte by byte with pauses far over probe_frame_gap_ms: read whole (transports §2)
  {
    const Bytes f = framed(request(30, 0, 0x04, {}));
    for (uint8_t b : f) {
      slot[1].send({b});
      g_millis += 500;
      ep.poll();
    }
    const auto ms = messages(slot[1].tx);
    CHECK(ms.size() == 1 && okAnswer(ms[0], 30));
    slot[1].tx.clear();
  }
  // three frames in one write: all answered, in order
  {
    slot[0].send(cat(cat(framed(request(40, 0, 0x04, {})), framed(request(41, 0, 0x13, {}))), framed(request(42, 0, 0x04, {}))));
    ep.poll();
    const auto ms = messages(slot[0].tx);
    CHECK(ms.size() == 3 && okAnswer(ms[0], 40) && okAnswer(ms[1], 41) && okAnswer(ms[2], 42));
    slot[0].tx.clear();
  }
  // a new connection in a slot: the old one's half frame is gone
  {
    const Bytes f = framed(request(50, 0, 0x04, {}));
    slot[2].accept();
    slot[2].send(Bytes(f.begin(), f.begin() + 4));
    ep.poll();
    slot[2].close();
    ep.poll();
    slot[2].accept();
    slot[2].send(framed(request(51, 0, 0x04, {})));
    ep.poll();
    const auto ms = messages(slot[2].tx);
    CHECK(ms.size() == 1 && okAnswer(ms[0], 51));
    slot[2].tx.clear();
  }
  // a length over max_frame (512): the connection closes, nothing answered
  {
    slot[2].send({0x01, 0x02, 0, 0, 0});
    ep.poll();
    CHECK(slot[2].drops == 1 && slot[2].id == 0 && slot[2].tx.empty());
  }
  // the one lock: connection 0 opens; connection 1's open is locked, its lock_state sees it
  const uint32_t sid = 0x12345678, other = 0x0badf00d;
  {
    slot[0].send(framed(request(60, 0, 0x10, cat(u32(3000), {0}), sid)));
    ep.poll();
    auto ms = messages(slot[0].tx);
    CHECK(ms.size() == 1 && okAnswer(ms[0], 60));
    slot[0].tx.clear();
    slot[1].send(framed(request(61, 0, 0x10, cat(u32(3000), {0}), other)));
    slot[1].send(framed(request(62, 0, 0x13, {})));
    ep.poll();
    ms = messages(slot[1].tx);
    CHECK(ms.size() == 2 && ms[0].size() >= 5 && ms[0][3] == kResolutionRejected && ms[0][4] == kRejectLocked);
    CHECK(ms.size() == 2 && okAnswer(ms[1], 62) && ms[1].size() >= 6 && ms[1][5] == 1);
    slot[1].tx.clear();
  }
  // notifications: to the subscriber's connection only
  {
    slot[0].send(framed(request(70, 1, kOpSubscribe, cat({0, 0}, u32(0)), sid)));
    ep.poll();
    auto ms = messages(slot[0].tx);
    CHECK(ms.size() == 1 && okAnswer(ms[0], 70));
    slot[0].tx.clear();
    ticker.ready = 10;
    ep.poll();
    ms = messages(slot[0].tx);
    CHECK(ms.size() == 1 && ms[0].size() == kPushHeader + 10 && ms[0][0] == kRolePush);
    CHECK(slot[1].tx.empty() && uart.tx.empty());
    slot[0].tx.clear();
    // the connection closes and another takes its slot: the session stays, the newcomer gets no notification
    slot[0].close();
    ep.poll();
    slot[0].accept();
    ticker.ready = 10;
    ep.poll();
    CHECK(slot[0].tx.empty() && slot[1].tx.empty() && ep.locked());
    // the same session's open from connection 1: the notifications go there now (core §6.2)
    slot[1].send(framed(request(71, 0, 0x10, cat(u32(3000), {0}), sid)));
    ep.poll();
    ms = messages(slot[1].tx);
    CHECK(ms.size() >= 1 && okAnswer(ms[0], 71));
    slot[1].tx.clear();
    ticker.ready = 10;
    ep.poll();
    ms = messages(slot[1].tx);
    CHECK(ms.size() == 1 && ms[0].size() >= 1 && ms[0][0] == kRolePush);
    CHECK(slot[0].tx.empty());
  }

  printf("tcp: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
