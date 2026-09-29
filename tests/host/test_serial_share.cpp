// Host tests: the serial-port reader (oep-core §3.1, §3.4), the endpoint's serial-port rules (raw bytes, the ports a
// session holds, the resume from its last host reset), owner and the transport list, and the binds' modes.
#include <stdio.h>
#include <string>
#include <vector>

#include "OepFrame.h"
#include "OepBind.h"
#include "OepEndpoint.h"

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
  Bytes rx, tx;   // host -> probe, probe -> host
  size_t at = 0;
  int room = 4096;
  int available() override { return static_cast<int>(rx.size() - at); }
  int read() override { return at < rx.size() ? rx[at++] : -1; }
  int peek() override { return at < rx.size() ? rx[at] : -1; }
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override {
    const size_t k = room < 0 ? 0 : (n < static_cast<size_t>(room) ? n : static_cast<size_t>(room));
    tx.insert(tx.end(), b, b + k);
    return k;
  }
  int availableForWrite() override { return room; }
  void send(const Bytes &b) { rx.insert(rx.end(), b.begin(), b.end()); }
};

// The host side of the framing: 0x00 <COBS(message + CRC)> 0x00, and the frames found in what came back.
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
// Splits what the probe sent into frames (results) and the rest (raw bytes).
static void split(const Bytes &tx, std::vector<Bytes> &frames, Bytes &raw) {
  frames.clear();
  raw.clear();
  size_t i = 0;
  while (i < tx.size()) {
    if (tx[i] != 0) { raw.push_back(tx[i++]); continue; }
    size_t j = i + 1;
    while (j < tx.size() && tx[j] != 0) ++j;
    Bytes body(tx.begin() + i + 1, tx.begin() + j), msg;
    if (j < tx.size() && !body.empty() && unframe(body, msg)) { frames.push_back(msg); i = j + 1; continue; }
    if (body.empty()) { i = j; continue; }   // 0x00 0x00
    raw.push_back(0);
    raw.insert(raw.end(), body.begin(), body.end());
    i = j;
  }
}

static Bytes u32(uint32_t v) { return {uint8_t(v), uint8_t(v >> 8), uint8_t(v >> 16), uint8_t(v >> 24)}; }
static Bytes request(uint16_t corr, uint16_t fn, uint8_t op, const Bytes &payload, bool session = false, uint32_t id = 0) {
  Bytes m = {uint8_t(session ? 0x81 : 0x01), uint8_t(corr), uint8_t(corr >> 8), uint8_t(fn), uint8_t(fn >> 8), op};
  if (session) { const Bytes s = u32(id); m.insert(m.end(), s.begin(), s.end()); }
  m.insert(m.end(), payload.begin(), payload.end());
  return m;
}
static Bytes openPayload(uint32_t id, uint32_t lease, const char *owner = nullptr) {
  Bytes p = u32(id);
  const Bytes l = u32(lease);
  p.insert(p.end(), l.begin(), l.end());
  p.push_back(0);
  if (owner) { p.push_back(0x01); p.push_back(static_cast<uint8_t>(strlen(owner))); p.insert(p.end(), owner, owner + strlen(owner)); }
  return p;
}

// A bindable stream (a console): bytes the target says, the host resets counted, what the port sent it.
class FakeSource final : public BindSource {
 public:
  uint8_t buffer[256];
  PositionStream::Mark marks[4];
  PositionStream stream{buffer, sizeof buffer, marks, 4};
  uint32_t resets = 0;
  Bytes input;
  void say(const char *text) { for (const char *p = text; *p; ++p) stream.put(static_cast<uint8_t>(*p)); }
  const PositionStream *bindStream() const override { return &stream; }
  uint16_t bindStreamNumber() const override { return 1; }
  size_t bindInput(const uint8_t *d, size_t n) override { input.insert(input.end(), d, d + n); return n; }
  uint32_t hostResets() const override { return resets; }
};

static std::string text(const Bytes &b) { return std::string(b.begin(), b.end()); }

// ---- SerialReader -------------------------------------------------------------------------------------------------------

static Bytes g_raw;
static void sink(void *, const uint8_t *d, size_t n) { g_raw.insert(g_raw.end(), d, d + n); }

static void testReader() {
  uint8_t enc[64], dec[64];
  SerialReader r;
  r.reset(enc, sizeof enc, dec, sizeof dec);
  g_raw.clear();
  Bytes in = {'h', 'i'};
  const Bytes f = frame({1, 2, 3});
  in.insert(in.end(), f.begin(), f.end());
  in.insert(in.end(), {0, 'z', 'z', 0});   // a broken candidate: raw with its leading 0x00
  const uint8_t *p = in.data();
  size_t n = in.size();
  CHECK(r.feed(p, n, sink, nullptr));
  CHECK(r.length() == 3 && r.message()[0] == 1 && r.message()[2] == 3);
  CHECK(text(g_raw) == "hi");
  CHECK(!r.feed(p, n, sink, nullptr));
  // "0x00 z z": the frame's closing 0x00 started a candidate, then 0x00 0x00 (empty) and "zz" closed by a 0x00
  CHECK(g_raw.size() == 2 + 3 && g_raw[2] == 0 && g_raw[3] == 'z' && g_raw[4] == 'z');
  g_raw.clear();
  const Bytes open = {0, 'a', 'b'};
  p = open.data();
  n = open.size();
  r.feed(p, n, sink, nullptr);
  CHECK(g_raw.empty());
  g_millis += 250;   // stopped for 200 ms: raw
  r.idle(sink, nullptr);
  CHECK(g_raw.size() == 3 && g_raw[0] == 0 && g_raw[1] == 'a');
  // a frame's closing 0x00 opens a candidate; the 200 ms gap after it gives nothing raw (it was a delimiter)
  g_raw.clear();
  const Bytes lone = frame({9, 9, 9});
  p = lone.data();
  n = lone.size();
  CHECK(r.feed(p, n, sink, nullptr));
  CHECK(!r.feed(p, n, sink, nullptr));
  g_millis += 250;
  r.idle(sink, nullptr);
  CHECK(g_raw.empty());
  // a full block at the end: no empty block after it, both forms taken
  Bytes big(254, 7);
  const Bytes fb = frame(big);
  CHECK(fb.size() == 1 + 1 + 254 + 1 + 2 + 1 || fb.size() == big.size() + 2 + 2 + 2);   // 0 FF 254 03 crc crc 0
  uint8_t enc2[400], dec2[400];
  SerialReader r2;
  r2.reset(enc2, sizeof enc2, dec2, sizeof dec2);
  p = fb.data();
  n = fb.size();
  CHECK(r2.feed(p, n, sink, nullptr) && r2.length() == 254);
}

// ---- the endpoint on a serial port, with binds ---------------------------------------------------------------------------

static void testEndpointSerialPort() {
  MemStream usj, bulk;
  static uint8_t rx[1100], rx2[1100], tx[1100];
  Endpoint ep(bulk, rx2, sizeof rx2, tx, sizeof tx, {1024, 4096, 8}, Endpoint::kVendorBulk, 1);
  ep.addTransport(usj, rx, sizeof rx, Endpoint::kUsbSerialJtag);
  Binds binds;
  ep.setRawPorts(&binds);
  FakeSource console;
  Binds::Spec spec;
  spec.set = true;
  spec.mode = Binds::kLastReset;
  spec.count = 1;
  spec.sources[0] = {Binds::kSlotConsole, 0, &console};
  binds.set(1, spec);

  console.say("boot\n");
  ep.poll();
  std::vector<Bytes> frames;
  Bytes raw;
  split(usj.tx, frames, raw);
  CHECK(text(raw) == "boot\n" && frames.empty());
  usj.tx.clear();

  usj.send({'t', 'y', 'p', 'e', 'd'});   // raw from the host: to the console's input
  ep.poll();
  CHECK(text(console.input) == "typed");

  usj.send(frame(request(1, 0, 0x10, openPayload(0x51, 3000, "test owner"))));   // open over the serial port
  ep.poll();
  split(usj.tx, frames, raw);
  CHECK(frames.size() == 1 && frames[0][3] == 1 && frames[0][4] == 0);
  CHECK(ep.held(1));
  usj.tx.clear();
  console.say("held\n");
  usj.send({'x'});
  ep.poll();
  CHECK(usj.tx.empty() && text(console.input) == "typed");   // held: nothing out, the raw byte dropped

  bulk.send({6, 0});   // lock_state over vendor bulk (length framing): the owner shows
  const Bytes ls = request(2, 0, 0x13, {});
  bulk.send(ls);
  bulk.rx[bulk.rx.size() - ls.size() - 2] = static_cast<uint8_t>(ls.size());
  ep.poll();
  const std::string lsr = text(bulk.tx);
  CHECK(lsr.find("test owner") != std::string::npos);

  console.resets = 1;   // a host reset during the session (riscv-dm reset)
  ep.poll();
  console.say("after reset\n");
  usj.send(frame(request(3, 0, 0x11, {}, true, 0x51)));   // end
  ep.poll();
  split(usj.tx, frames, raw);
  CHECK(frames.size() == 1 && !ep.held(1));
  CHECK(text(raw) == "after reset\n");   // from the reset, not "held"

  // the transport list in oep.core's describe
  usj.tx.clear();
  usj.send(frame(request(4, 0, 0x03, {0, 0, 0, 0})));
  ep.poll();
  split(usj.tx, frames, raw);
  CHECK(frames.size() == 1);
  const Bytes &d = frames[0];
  bool vendor = false, usjSeen = false;
  for (size_t i = 6; i + 5 <= d.size(); ++i)
    if (d[i] == 0x49 && d[i + 1] == 3) { vendor |= d[i + 2] == 0 && d[i + 3] == 4 && d[i + 4] == 1; usjSeen |= d[i + 2] == 1 && d[i + 3] == 3; }
  CHECK(vendor && usjSeen);
}

// ---- mixed ------------------------------------------------------------------------------------------------------------------

static size_t namer(void *, uint8_t, uint16_t id, char *out, size_t room) {
  return static_cast<size_t>(snprintf(out, room, "s%u", id));
}

static void testMixed() {
  MemStream cdc;
  static uint8_t rx[1100], tx[1100];
  Endpoint ep(cdc, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 8}, Endpoint::kUsbCdc, 0);
  Binds binds;
  binds.setNamer(namer, nullptr);
  ep.setRawPorts(&binds);
  FakeSource a, b;
  Binds::Spec spec;
  spec.set = true;
  spec.mode = Binds::kMixed;
  spec.count = 2;
  spec.sources[0] = {Binds::kSlotConsole, 0, &a};
  spec.sources[1] = {Binds::kSlotConsole, 1, &b};
  binds.set(0, spec);
  a.say("one\ntw");
  b.say("two\n");
  ep.poll();
  ep.poll();
  CHECK(text(cdc.tx) == "[s0] one\n[s1] two\n");
  cdc.tx.clear();
  g_millis += 150;
  ep.poll();
  CHECK(text(cdc.tx) == "[s0] tw\n");   // closed by quiet
  cdc.send({'n', 'o'});
  ep.poll();
  CHECK(a.input.empty() && b.input.empty());   // mixed takes no input

  // a port that takes nothing holds its position
  cdc.tx.clear();
  cdc.room = 0;
  a.say("later\n");
  ep.poll();
  CHECK(cdc.tx.empty());
  cdc.room = 4096;
  ep.poll();
  CHECK(text(cdc.tx) == "[s0] later\n");
}

int main() {
  testReader();
  testEndpointSerialPort();
  testMixed();
  printf("TEST done %d/%d\n", checks - failures, checks);
  return failures ? 1 : 0;
}
