// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host test: every byte vector of oep-spec tests/vectors (copied into tests/vectors by tools/sync_registry.sh, the commit
// in tests/vectors/SPEC_COMMIT) run against this library's endpoint, byte for byte, as oep-client-python runs them
// against its fake probe: headers, cobs, checks, confirm, discovery, sessions, refusals, ops and ops_encoding.
//
// The probes the vectors assume are built from the library's own interfaces:
//   - the example probe of confirm.json / discovery.json / sessions.json: fn 0 only (no interface: list is empty), one
//     UART bridge (index 0, interface 0xFF), unit_id "a1b2c3d4", max_op_ms 1000, max_frame 1024, window
//     4096, max_inflight 4, boot_id 0x12345678;
//   - the probe of refusals.json / ops.json: fn 1 oep.probe.link, 2 oep.fixture.gpio, 3 oep.fixture.i2c-target, 4
//     oep.wire.rvswd, 5 oep.fixture.uart, 6 oep.target.riscv-dm, 7 oep.target.console, 8 oep.probe.config, 9
//     oep.fixture.logic, and the endpoint's own after them: 10 oep.probe.plan, 11 oep.probe.restart (in the cases that
//     name it: the restart handler set), on a UART bridge (transport 0, the port a bind names) and a vendor bulk
//     (transport 1, where the requests go), max_frame 1024; each case starts from boot (resource numbers from 1) in the
//     state its `state` says, with the session 0x11223344 open when the request carries it.
// ops.json's cases of other probes run on their own: fns 12 - 14 (capture-group with two tracks, spi-target) on
// WideProbe, the console marks of a ring of 4 on a PositionStream, arm-adi in test_swd, the dropped segment of a
// 1000-byte segment in test_capture (this probe's are 4 KiB); multirate (capture §5) is not offered here. ops.json's
// events: the group's on WideProbe, fn 9's from the logic capture (testLogicEvents), seq aside.
// clock's uptime_ns (sessions.json) is the example probe's in the vector: here it is checked to be this probe's clock as
// it reads it for the answer (the fake clock, which the test moves on before each clock), the rest byte for byte.
// Two fields are the probe's own and not the vector's, both in the logic segment record: start_uncertainty_ns is this
// capture's declared value (LogicCapture::kStartUncertaintyNs, 5000) where the vector's example probe has 50, and
// samples is the actual_samples this capture's configure answered for the 1000 asked (a one-shot segment is whole
// 128-byte cache lines: 1024 at one line); the rest of that answer is compared byte for byte.
// The test's own setup requests are numbered just before the vector's corr (core §4.1: the host numbers its requests
// in order, and a corr no newer than the newest is result_lost, core §5.2).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <map>
#include <string>
#include <vector>

#include <esp_heap_caps.h>
#include <freertos/queue.h>

#include "OepCapture.h"
#include "OepConfig.h"
#include "OepConsole.h"
#include "OepDmConsole.h"
#include "OepEndpoint.h"
#include "OepFixture.h"
#include "OepFrame.h"
#include "OepP4I2cTarget.h"
#include "OepP4SpiTarget.h"
#include "OepPinTable.h"
#include "OepTarget.h"

uint32_t g_millis = 0;
void (*g_on_wait)() = nullptr;

using namespace oep;
using Bytes = std::vector<uint8_t>;

#ifndef OEP_VECTORS_DIR
#define OEP_VECTORS_DIR "tests/vectors"
#endif

static int failures = 0, checks = 0, cases = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

// ---- a small JSON reader (objects, arrays, strings, integers, booleans, null) -------------------------------------

struct Json {
  enum Kind { kNull, kBool, kNumber, kString, kArray, kObject } kind = kNull;
  long long number = 0;
  bool boolean = false;
  std::string string;
  std::vector<Json> items;
  std::vector<std::pair<std::string, Json>> members;
  const Json &operator[](const char *key) const {
    static const Json none;
    for (const auto &m : members) if (m.first == key) return m.second;
    return none;
  }
  bool has(const char *key) const {
    for (const auto &m : members) if (m.first == key) return true;
    return false;
  }
};

struct JsonReader {
  const char *p;
  void space() { while (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') ++p; }
  std::string str() {
    std::string s;
    ++p;   // "
    while (*p && *p != '"') {
      if (*p == '\\') {
        ++p;
        if (*p == 'u') {   // \uXXXX (BMP), as UTF-8
          const unsigned c = static_cast<unsigned>(strtoul(std::string(p + 1, 4).c_str(), nullptr, 16));
          p += 5;
          if (c < 0x80) s += char(c);
          else if (c < 0x800) { s += char(0xC0 | c >> 6); s += char(0x80 | (c & 0x3F)); }
          else { s += char(0xE0 | c >> 12); s += char(0x80 | ((c >> 6) & 0x3F)); s += char(0x80 | (c & 0x3F)); }
          continue;
        }
        s += *p == 'n' ? '\n' : *p == 't' ? '\t' : *p;
        ++p;
        continue;
      }
      s += *p++;
    }
    ++p;   // "
    return s;
  }
  Json value() {
    space();
    Json j;
    if (*p == '{') {
      j.kind = Json::kObject;
      ++p;
      space();
      while (*p != '}') {
        space();
        std::string key = str();
        space();
        ++p;   // :
        j.members.push_back({key, value()});
        space();
        if (*p == ',') ++p;
        space();
      }
      ++p;
    } else if (*p == '[') {
      j.kind = Json::kArray;
      ++p;
      space();
      while (*p != ']') {
        j.items.push_back(value());
        space();
        if (*p == ',') ++p;
        space();
      }
      ++p;
    } else if (*p == '"') {
      j.kind = Json::kString;
      j.string = str();
    } else if (!strncmp(p, "true", 4)) {
      j.kind = Json::kBool;
      j.boolean = true;
      p += 4;
    } else if (!strncmp(p, "false", 5)) {
      j.kind = Json::kBool;
      p += 5;
    } else if (!strncmp(p, "null", 4)) {
      p += 4;
    } else {
      j.kind = Json::kNumber;
      char *end = nullptr;
      j.number = strtoll(p, &end, 10);
      p = end;
    }
    return j;
  }
};

static Json load(const char *name) {
  const std::string path = std::string(OEP_VECTORS_DIR) + "/" + name;
  FILE *f = fopen(path.c_str(), "rb");
  if (!f) { printf("FAIL cannot read %s\n", path.c_str()); ++failures; return {}; }
  std::string text;
  char buf[4096];
  for (size_t n; (n = fread(buf, 1, sizeof buf, f)) > 0;) text.append(buf, n);
  fclose(f);
  JsonReader r{text.c_str()};
  return r.value();
}

static Bytes hex(const std::string &h) {
  Bytes b;
  for (size_t i = 0; i + 1 < h.size(); i += 2) b.push_back(static_cast<uint8_t>(strtoul(h.substr(i, 2).c_str(), nullptr, 16)));
  return b;
}
static std::string toHex(const Bytes &b) {
  std::string s;
  char t[3];
  for (uint8_t c : b) { snprintf(t, sizeof t, "%02x", c); s += t; }
  return s;
}
static bool same(const char *what, const std::string &name, const Bytes &got, const Bytes &want) {
  if (got == want) return true;
  printf("  %s \"%s\":\n    got  %s\n    want %s\n", what, name.c_str(), toHex(got).c_str(), toHex(want).c_str());
  return false;
}

// ---- transports -------------------------------------------------------------------------------------------------

class MemStream final : public Stream {
 public:
  Bytes rx, tx;
  size_t at = 0;
  int available() override { return static_cast<int>(rx.size() - at); }
  int read() override { return at < rx.size() ? rx[at++] : -1; }
  int peek() override { return at < rx.size() ? rx[at] : -1; }
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override { tx.insert(tx.end(), b, b + n); return n; }
  int availableForWrite() override { return 1 << 20; }
};

// Bytes in on `s`, the endpoint polled once: what it wrote back.
static Bytes wire(Endpoint &ep, MemStream &s, const Bytes &in) {
  s.rx.insert(s.rx.end(), in.begin(), in.end());
  s.tx.clear();
  ep.poll();
  return s.tx;
}
static Bytes cobsFrame(const Bytes &m) {
  MemStream f;
  writeCobsFrame(f, m.data(), m.size());
  return f.tx;
}
// The messages of 0x00 <COBS(message + CRC)> 0x00 frames (CRC checked).
static std::vector<Bytes> unframe(const Bytes &tx) {
  std::vector<Bytes> out;
  size_t i = 0;
  while (i < tx.size()) {
    if (tx[i] != 0) { ++i; continue; }
    size_t j = i + 1;
    while (j < tx.size() && tx[j] != 0) ++j;
    if (j >= tx.size()) break;
    if (j > i + 1) {
      Bytes d(j - i - 1 + 8);
      size_t n = 0;
      if (cobsDecode(tx.data() + i + 1, j - i - 1, d.data(), d.size(), n) && n >= 3 &&
          crc16Ccitt(d.data(), n - 2) == uint16_t(d[n - 2] | d[n - 1] << 8)) {
        d.resize(n - 2);
        out.push_back(d);
      }
    }
    i = j;
  }
  return out;
}
static Bytes serialMessage(Endpoint &ep, MemStream &s, const Bytes &m) {
  const std::vector<Bytes> a = unframe(wire(ep, s, cobsFrame(m)));
  return a.empty() ? Bytes{} : a.back();
}
static Bytes bulkMessage(Endpoint &ep, MemStream &s, const Bytes &m) {
  Bytes f = {uint8_t(m.size()), uint8_t(m.size() >> 8)};
  f.insert(f.end(), m.begin(), m.end());
  const Bytes t = wire(ep, s, f);
  return t.size() >= 2 ? Bytes(t.begin() + 2, t.end()) : Bytes{};
}

// A probe starts from boot: its clock from 0 (ns since boot, core §2.6a), its resource numbers from 1 (core §9).
static void boot(uint32_t ms = 0) {
  g_millis = ms;
  g_micros_part = 0;
  g_micros_extender = MicrosExtender{};
  ResourceNumbers::reset();
}

// ---- the example probe of confirm / discovery / sessions --------------------------------------------------------

struct Example {
  MemStream s;
  uint8_t rx[2200], tx[1100], desc[32];
  Endpoint ep;
  explicit Example(uint8_t kind = Endpoint::kUartBridge)
      : ep(s, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 4}, kind, kind == Endpoint::kUartBridge ? 0xff : 0) {
    ep.setBootId(0x12345678);
    ep.setMaxOpMs(1000);
    TlvWriter w(desc, sizeof desc);
    w.text(kCoreUnitId, "a1b2c3d4");
    ep.setProbeDescription(desc, w.length());
  }
  Bytes send(const Bytes &m) { return serialMessage(ep, s, m); }
};

static void testHeaders() {
  const Json v = load("headers.json");
  for (const Json &r : v["requests"].items) {
    ++cases;
    const Bytes m = hex(r["message_hex"].string);
    CHECK(m.size() >= kRequestHeader && m[0] == kRoleRequest && m[0] == r["role"].number);
    CHECK(getU16(&m[1]) == r["corr"].number && getU16(&m[3]) == r["fn"].number && m[5] == r["op"].number);
    CHECK(getU32(&m[6]) == static_cast<uint32_t>(r["session_id"].number));
    CHECK(same("request payload", r["name"].string, Bytes(m.begin() + kRequestHeader, m.end()), hex(r["payload_hex"].string)));
  }
  // the answers' header (core §4.2) as the endpoint writes it: corr 0x1234 completed success, corr 0xFFFF malformed
  for (const Json &a : v["answers"].items) {
    ++cases;
    boot();
    Example p;
    const uint16_t corr = static_cast<uint16_t>(a["corr"].number);
    const bool ok = a["resolution"].number == kResolutionCompleted;
    // keepalive-free requests: a lock_state (success, payload 5 bytes) or a confirm with min_rev > max_rev (malformed)
    Bytes m = {kRoleRequest, uint8_t(corr), uint8_t(corr >> 8), 0, 0, uint8_t(ok ? kOpLockState : kOpConfirm), 0, 0, 0, 0};
    if (!ok) m.insert(m.end(), {'O', 'E', 'P', '?', 2, 1});
    const Bytes got = p.send(m), want = hex(a["message_hex"].string);
    CHECK(same("answer header", a["name"].string, Bytes(got.begin(), got.begin() + (got.size() < 5 ? got.size() : 5)), want));
  }
  for (const Json &t : v["tlvs"].items) {
    ++cases;
    const Bytes want = hex(t["tlv_hex"].string);
    Bytes value = t.has("value_hex") ? hex(t["value_hex"].string) : Bytes(t["value_len"].number, uint8_t(t["value_byte"].number));
    uint8_t buf[600];
    TlvWriter w(buf, sizeof buf);
    CHECK(w.put(static_cast<uint8_t>(t["tag"].number), value.data(), value.size()));
    CHECK(same("tlv", t["name"].string, Bytes(buf, buf + w.length()), want));
    uint8_t tag = 0;
    const uint8_t *val = nullptr;
    size_t len = 0, next = 0;
    CHECK(tlvAt(want.data(), want.size(), 0, tag, val, len, next) && tag == t["tag"].number && len == value.size() &&
          next == want.size() && Bytes(val, val + len) == value);
  }
}

static void testCobs() {
  const Json v = load("cobs.json");
  for (const Json &e : v["encode"].items) {
    ++cases;
    const Bytes data = hex(e["data_hex"].string), want = hex(e["encoded_hex"].string);
    Bytes out(data.size() + 8);
    out.resize(cobsEncode(data.data(), data.size(), out.data(), out.size()));
    CHECK(same("cobs encode", e["name"].string, out, want));
    Bytes back(data.size() + 8);
    size_t n = 0;
    CHECK(cobsDecode(want.data(), want.size(), back.data(), back.size(), n) && Bytes(back.begin(), back.begin() + n) == data);
  }
  for (const Json &e : v["decode_also_accepts"].items) {
    ++cases;
    const Bytes enc = hex(e["encoded_hex"].string), data = hex(e["data_hex"].string);
    Bytes back(data.size() + 8);
    size_t n = 0;
    CHECK(cobsDecode(enc.data(), enc.size(), back.data(), back.size(), n) && Bytes(back.begin(), back.begin() + n) == data);
  }
  for (const Json &f : v["frames"].items) {
    ++cases;
    const Bytes m = hex(f["message_hex"].string), frame = hex(f["frame_hex"].string);
    CHECK(crc16Ccitt(m.data(), m.size()) == f["crc16"].number);
    CHECK(same("cobs frame", f["name"].string, cobsFrame(m), frame));
    // and SerialReader takes it back
    static uint8_t enc[1200], dec[1200];
    SerialReader reader;
    reader.reset(enc, sizeof enc, dec, sizeof dec);
    const uint8_t *p = frame.data();
    size_t n = frame.size();
    bool got = false;
    while (n && !got) got = reader.feed(p, n, [](void *, const uint8_t *, size_t) {}, nullptr);
    CHECK(got && Bytes(reader.message(), reader.message() + reader.length()) == m);
  }
}

static void testChecks() {
  const Json v = load("checks.json");
  for (const Json &c : v["cases"].items) {
    ++cases;
    const Bytes in = hex(c["input_hex"].string);
    const std::string algorithm = c["algorithm"].string;
    long long got = -1;
    if (algorithm == "crc16-ccitt-false") got = crc16Ccitt(in.data(), in.size());
    else if (algorithm == "crc32-ieee") got = crc32Ieee(in.data(), in.size());
    else if (algorithm == "crc8-dmseq") got = DmConsole::crc8(in.data(), in.size());
    if (got != c["crc"].number) printf("  check \"%s\": got %lld want %lld\n", c["name"].string.c_str(), got, c["crc"].number);
    CHECK(got == c["crc"].number);
  }
  for (const Json &w : v["dmseq_words"].items) {   // byte0, the payload, the CRC right after it: DATA0's bytes
    ++cases;
    Bytes b = {uint8_t(w["byte0"].number)};
    const Bytes payload = hex(w["payload_hex"].string);
    b.insert(b.end(), payload.begin(), payload.end());
    const uint8_t crc = DmConsole::crc8(b.data(), b.size());
    CHECK(crc == w["crc"].number);
    b.push_back(crc);
    uint32_t data0 = 0;
    for (size_t i = 0; i < b.size() && i < 4; ++i) data0 |= uint32_t(b[i]) << (8 * i);
    CHECK(data0 == static_cast<uint32_t>(w["data0"].number));
  }
}

static void testConfirm() {
  const Json v = load("confirm.json");
  for (const Json &e : v["exchanges"].items) {
    ++cases;
    boot();
    Example p;
    CHECK(same("confirm", e["name"].string, p.send(hex(e["request_hex"].string)), hex(e["answer_hex"].string)));
    if (e.has("request_serial_frame_hex")) {   // the whole serial-port frames, byte for byte
      boot();
      Example q;
      CHECK(same("confirm frame", e["name"].string, wire(q.ep, q.s, hex(e["request_serial_frame_hex"].string)),
                 hex(e["answer_serial_frame_hex"].string)));
    }
    if (e.has("request_length_frame_hex")) {   // a length-prefixed transport (vendor bulk), transport index 0
      boot();
      Example b(Endpoint::kVendorBulk);
      Bytes want = hex(e["answer_hex"].string);
      want.insert(want.begin(), {uint8_t(want.size()), uint8_t(want.size() >> 8)});
      CHECK(same("confirm length frame", e["name"].string, wire(b.ep, b.s, hex(e["request_length_frame_hex"].string)), want));
    }
  }
}

static void testDiscovery() {
  const Json v = load("discovery.json");
  boot();
  Example p;
  for (const char *list : {"exchanges", "refusals"}) {
    for (const Json &e : v[list].items) {
      ++cases;
      CHECK(same("discovery", e["name"].string, p.send(hex(e["request_hex"].string)), hex(e["answer_hex"].string)));
      if (e.has("request_serial_frame_hex"))
        CHECK(same("discovery frame", e["name"].string, wire(p.ep, p.s, hex(e["request_serial_frame_hex"].string)),
                   hex(e["answer_serial_frame_hex"].string)));
    }
  }
}

static void testSessions() {
  const Json v = load("sessions.json");
  for (const Json &sc : v["scenarios"].items) {
    ++cases;
    boot();
    Example p;
    for (const Json &st : sc["steps"].items) {
      const Bytes req = hex(st["request_hex"].string);
      Bytes want = hex(st["answer_hex"].string);
      const bool clock = req.size() >= kRequestHeader && getU16(&req[3]) == 0 && req[5] == kOpClock;
      if (clock) advanceMicros(123);   // the clock moves on (less than a ms: the lease's times stay as the vector's)
      const Bytes got = p.send(req);
      if (clock && want.size() == 5 + 12 && got.size() == want.size()) {
        CHECK(getU64(&got[5 + 4]) == nowNs());   // read for this answer
        putU64(&want[5 + 4], getU64(&got[5 + 4]));
      }
      CHECK(same("session step", sc["name"].string + ": " + st["note"].string, got, want));
    }
  }
}

// core §7.4: the ops value - 2 to 33 bytes, base + 8 x bitmap bytes <= 256, bit 0 set, the last byte non-zero - decoded
// here on its own; a valid one is the set the vector gives, and the endpoint's ops tag for an interface offering that set
// (ops 0x01 - 0xEF only: 0x00 and the experimental ones are never offered) is the same bytes.
static bool opsDecode(const Bytes &v, std::vector<int> &ops) {
  ops.clear();
  // core §7.4: base(u8) and a bitmap of 1 byte or more, base + 8 x bitmap bytes <= 256 (several values may give one set)
  if (v.size() < 2 || v[0] + 8 * (v.size() - 1) > 256) return false;
  for (size_t i = 0; i < 8 * (v.size() - 1); ++i)
    if (v[1 + i / 8] >> (i % 8) & 1) ops.push_back(v[0] + static_cast<int>(i));
  return true;
}
class OpSet final : public Interface {
 public:
  explicit OpSet(const std::vector<int> &ops) : ops_(ops) {}
  const char *name() const override { return "io.github.test.ops"; }
  uint16_t instance() const override { return 0; }
  bool offers(uint8_t op) const override {
    for (int o : ops_) if (o == op) return true;
    return false;
  }
  Result handle(uint8_t, const uint8_t *, size_t, uint8_t *, size_t) override { return completed(); }
 private:
  std::vector<int> ops_;
};
static void testOpsEncoding() {
  const Json v = load("ops_encoding.json");
  for (const Json &c : v["cases"].items) {
    ++cases;
    const Bytes value = hex(c["value_hex"].string);
    std::vector<int> ops, want;
    const bool valid = opsDecode(value, ops);
    for (const Json &o : c["ops"].items) want.push_back(static_cast<int>(o.number));
    if (valid != c["valid"].boolean || (valid && ops != want)) printf("  ops_encoding \"%s\": decoded wrong\n", c["name"].string.c_str());
    CHECK(valid == c["valid"].boolean && (!valid || ops == want));
    bool offerable = valid;
    for (int o : ops) offerable &= o >= 1 && o < 0xF0 && o != kOpSubscribe && o != kOpUnsubscribe;
    if (!offerable) continue;
    boot();
    Example p;
    OpSet it(ops);
    CHECK(p.ep.add(it));
    const Bytes d = p.send({kRoleRequest, 1, 0, 0, 0, kOpDescribe, 0, 0, 0, 0, 1, 0, 0, 0});
    // the probe's own value for that set: valid, and the same set
    std::vector<int> back;
    const bool tagged = d.size() >= 6 + 3 && d[6] == kTagOps && d.size() >= 9u + getU16(&d[7]);
    CHECK(tagged && opsDecode(Bytes(d.begin() + 9, d.begin() + 9 + getU16(&d[7])), back) && back == ops);
  }
}

// ---- the probe of refusals.json / ops.json ----------------------------------------------------------------------

// A debug module behind a link that always answers (as tests/host/test_console.cpp's): DMSTATUS 0x00400382 while the
// hart is halted (version 2, authenticated, allhalted, impebreak), DMI 0x7F the target id 0x00203500, DATA0 / DATA1 as
// written, DMCONTROL's haltreq / resumereq acted on.
class VecPhy final : public DmiPhy {
 public:
  bool attached_flag = false, halted = true;
  uint32_t data0 = 0, data1 = 0;
  // the wire goes silent (nothing answers, writes lost) at the run's first abstract command (this fake runs none: the
  // preparation - dcsr, a0, dpc - is where it fails)
  bool silent_at_a0 = false, silent = false;
  bool attach() override { attached_flag = true; return true; }
  void release() override { attached_flag = false; }
  bool attached() const override { return attached_flag; }
  void write(uint8_t address, uint32_t value) override {
    if (silent) return;
    if (silent_at_a0 && address == 0x17) { silent = true; return; }
    if (address == 0x04) data0 = value;
    if (address == 0x05) data1 = value;
    if (address == 0x10) {
      if (value & (1u << 31)) halted = true;
      if (value & (1u << 30)) halted = false;
    }
  }
  bool setIdleClockLow(bool) override { return true; }
  bool canIdleClockLow() const override { return true; }
  bool setMaxHz(uint32_t) override { return true; }
  bool keepsMaxHz(uint32_t) const override { return true; }
  bool usePins(int, int) override { return true; }
  uint32_t dmiNs() const override { return 1000; }
  uint32_t clockHz() const override { return 1000000; }
  uint32_t retries() const override { return 0; }
  uint32_t transactions() const override { return 0; }

 protected:
  bool readWire(uint8_t address, uint32_t &value) override {
    if (!attached_flag || silent) return false;
    switch (address) {
      case 0x04: value = data0; break;
      case 0x05: value = data1; break;
      case 0x10: value = 1; break;
      case 0x11: value = 2 | (1u << 7) | (halted ? (3u << 8) : (3u << 10)) | (1u << 22); break;
      case 0x12: value = 0x0002'1000u | 0x380; break;
      case 0x16: value = 2; break;
      case 0x7f: value = 0x00203500; break;
      default: value = 0; break;
    }
    return true;
  }
};

class NullUartStream final : public Stream {   // nothing on the fixture UART's line
 public:
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  size_t write(uint8_t) override { return 1; }
};

constexpr uint32_t kS = 0x11223344;

// The radio behind probe.config's wifi item (the cases whose state says "items has wifi"): takes the list, reports st.
struct VecWifi final : WifiControl {
  Status st;
  size_t count = 0;
  void apply(const WifiEntry *, size_t n) override { count = n; }
  Status status() const override { return st; }
};

struct OpsProbe {
  MemStream uart_bridge, bulk;
  uint8_t rx[2200], rx2[2200], tx[1100];
  Endpoint ep{uart_bridge, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 4}, Endpoint::kUartBridge};
  PinTable pins{0xFFFFull};   // channels 0-15
  Link link{ep};
  FixtureGpio gpio{pins, 0, 1};
  P4I2cTarget i2c{pins};
  VecPhy phy;
  Ch32Dm dm{phy};
  DebugPort port{dm, 1, 2};
  WireRvswd wire{port, 0};
  HardwareSerial serial;
  FixtureUart uart{pins, serial, 0, 2};
  TargetRiscvDm riscv{port, 0};
  DmConsole driver{dm, phy};
  TargetConsoleStream console{port, driver, 0};
  Binds binds;
  ProbeConfig config{ep, binds};
  LogicCapture logic{ep, pins};
  VecWifi wifi;
  // The wifi cases' hashes are the probe's own (probe.config §2): the vectors write 0x5A5A0001 for the settings with
  // wifi entry 0 and 0x5A5A0002 for the empty ones; these are what this probe answered for them.
  uint32_t hash_entry = 0, hash_empty = 0;
  uint16_t corr = 1;
  uint32_t actual_samples = 0;   // the logic configure's answer
  bool no_open = false;          // the case's state has the lock free: the vector's session is not opened
  static inline int restarts = 0;   // oep.probe.restart's handler (a chip would restart there)
  static void restartHook() { ++restarts; }
  OpsProbe() {
    ep.addTransport(bulk, rx2, sizeof rx2, Endpoint::kVendorBulk, 0);
    ep.setBootId(0x12345678);
    port.pin_choice = (1ull << 1) | (1ull << 2);
    port.pins = &pins;
    ep.add(link);      // fn 1
    ep.add(gpio);      // fn 2
    ep.add(i2c);       // fn 3
    ep.add(wire);      // fn 4
    ep.add(uart);      // fn 5
    ep.add(riscv);     // fn 6
    ep.add(console);   // fn 7
    ep.add(config);    // fn 8
    ep.add(logic);     // fn 9
    ep.setRawPorts(&binds);
    config.addPlace(wire, console);
    config.addUart(uart);
    config.setPins(&pins);
  }
  ~OpsProbe() { logic.planRelease(); }
  Bytes send(const Bytes &m) { return bulkMessage(ep, bulk, m); }
  // A request of this test's own (set `corr` before the vector's): the whole answer.
  Bytes request(uint16_t fn, uint8_t op, const Bytes &payload, uint32_t session = kS) {
    Bytes m = {kRoleRequest, uint8_t(corr), uint8_t(corr >> 8), uint8_t(fn), uint8_t(fn >> 8), op,
               uint8_t(session), uint8_t(session >> 8), uint8_t(session >> 16), uint8_t(session >> 24)};
    ++corr;
    m.insert(m.end(), payload.begin(), payload.end());
    return send(m);
  }
  bool ok(const Bytes &a) const { return a.size() >= 5 && a[3] == kResolutionCompleted && a[4] == kOutcomeSuccess; }
  // get's hash (lock-free, no session): the probe's hash of its settings now
  uint32_t hash() {
    const Bytes a = request(8, reg::probe_config::kOpGet, {0, 0}, 0);
    return ok(a) && a.size() >= 10 ? getU32(&a[6]) : 0;
  }
  bool open() { return ok(request(0, kOpOpen, {0xd0, 0x07, 0, 0, 0})); }   // session S, lease 2000 ms
  bool plan(uint16_t fn, uint8_t role, uint16_t channel) {
    return ok(request(ep.planFn(), kOpPlanApply,
                      {0x90, 5, 0, uint8_t(fn), uint8_t(fn >> 8), role, uint8_t(channel), uint8_t(channel >> 8)}));
  }
  // attach (rvswd, fn 4) on channels 1 / 2 at 1 MHz, the hart left as it is: connection 1 from boot
  bool attach() {
    return ok(request(4, reg::wire_rvswd::kOpAttach,
                      {0, 0x81, 4, 0, 0x40, 0x42, 0x0f, 0x00, 0x83, 4, 0, 1, 0, 2, 0}));
  }
  // The target's dmseq console writes `text` (frames of 2 bytes at most, the first a SYN), the hart running.
  void say(const char *text) {
    phy.halted = false;
    uint8_t s = 0;
    bool syn = true;
    for (size_t at = 0, n = strlen(text); at < n;) {
      const uint8_t m = n - at < 2 ? uint8_t(n - at) : 2;
      uint8_t b[4] = {uint8_t(0x80 | (s << 5) | (1u << 4) | (syn ? 0x08 : 0) | m), 0, 0, 0};
      for (uint8_t i = 0; i < m; ++i) b[1 + i] = uint8_t(text[at + i]);
      b[1 + m] = DmConsole::crc8(b, 1u + m);
      phy.data0 = b[0] | uint32_t(b[1]) << 8 | uint32_t(b[2]) << 16 | uint32_t(b[3]) << 24;
      for (int k = 0; k < 100 && (phy.data0 & 0x80u); ++k) { advanceMicros(1000); console.poll(); }
      at += m;
      s ^= 1;
      syn = false;
    }
  }
};

static uint32_t sessionOf(const Bytes &m) { return m.size() >= kRequestHeader ? getU32(&m[6]) : 0; }

// Whether a case's `fns` names the interface (`fns` maps fn numbers to names).
static bool names(const Json &fns, const char *name) {
  for (const auto &m : fns.members) if (m.second.string == name) return true;
  return false;
}

// The state a case of ops.json / refusals.json assumes (its name, `state` and `fns`), from boot; false: not set up.
static bool setUp(OpsProbe &p, const std::string &name, const std::string &state, const Json &fns = Json{}) {
  auto has = [&](const char *s) { return name.find(s) != std::string::npos || state.find(s) != std::string::npos; };
  if (names(fns, "oep.probe.restart")) {   // listed with a handler (fn 11), which here only counts
    p.ep.setRestart(OpsProbe::restartHook, 2000);
    p.no_open = has("lock free");
    return p.ep.restartFn() == 11;
  }
  if (names(fns, "oep.probe.plan")) {   // fn 10: gpio (fn 2) has plan roles
    if (p.ep.planFn() != 10) return false;
    if (has("fn 2 has the plan above")) return p.open() && p.plan(2, reg::fixture_gpio::kRoleLine, 3);
    return true;
  }
  if (has("fn 9 subscribed") || has("fn 9 not subscribed") || has("subscribe set in fn 9's ops")) {
    if (has("fn 9 subscribed")) return p.open() && p.ok(p.request(9, kOpSubscribe, {0, 0, 0, 0, 0, 0}));
    return true;
  }
  if (has("ignored TLV") || has("is ignored") || (has("gpio") && (has("channel 3") || has("not in the plan")))) {
    if (!p.open() || !p.plan(2, reg::fixture_gpio::kRoleLine, 3)) return false;
    if (has("gpio read")) {   // after the set: channel 3 output high
      if (!p.ok(p.request(2, reg::fixture_gpio::kOpSet, {1, 3, 0, reg::fixture_gpio::kModeOutputHigh}))) return false;
    }
    return true;
  }
  if (has("the preparation fails")) {   // run: its preparation gets no answer from the wire (VecPhy::silent_at_a0)
    if (!p.open() || !p.attach()) return false;
    p.phy.silent_at_a0 = true;
    return true;
  }
  if (has("rvswd connections") || has("riscv-dm halt: halted") || has("riscv-dm dmi: one read") || has("riscv-dm dmi: n = 0")) {
    return p.open() && p.attach();
  }
  if (has("the hart is running")) {   // connection 1, the hart let run
    if (!p.open() || !p.attach()) return false;
    p.phy.halted = false;
    return true;
  }
  if (has("roles 0 and 1 in fn 9's plan")) {   // one plan_apply with both roles, nothing configured
    g_fake_heap = FakeHeap{};
    g_fake_heap.internal_free = 512 * 1024;
    g_fake_heap.psram_total = g_fake_heap.psram_free = size_t(32) << 20;
    return p.open() && p.ok(p.request(p.ep.planFn(), kOpPlanApply, {0x90, 5, 0, 9, 0, 0, 0, 0, 0x90, 5, 0, 9, 0, 1, 1, 0}));
  }
  if (has("console")) {
    if (has("read from 4")) return true;
    if (!p.open() || !p.attach()) return false;
    if (g_millis >= 1) return false;   // the attach mark at 1 ms
    g_millis = 1;
    g_micros_part = 0;
    const Bytes r = p.request(7, reg::target_console::kOpOpen, {1, 0, reg::target_console::kMechanismDmseq});
    if (!p.ok(r) || r.size() < 8 || getU16(&r[5]) != 2) return false;   // stream 2 on connection 1
    if (has("5 bytes written")) p.say("hello");
    return true;
  }
  if (has("items has wifi")) {   // wifi_max 4; settings "as above": entry 0 "lab", passphrase "password1"
    p.config.setWifi(&p.wifi);
    p.hash_empty = p.hash();
    if (has("connected through entry 0")) {   // -52 dBm, 192.168.1.23
      p.wifi.st.state = WifiControl::kStateConnected;
      p.wifi.st.entry = 0;
      p.wifi.st.rssi = -52;
      const uint8_t ip[4] = {192, 168, 1, 23};
      memcpy(p.wifi.st.ipv4, ip, 4);
      return true;
    }
    if (!p.open()) return false;
    if (has("settings: the wifi entry") || has("settings as above")) {
      const Bytes item = {reg::probe_config::kTlvItemWifi, 15, 0, 0, 3, 'l', 'a', 'b', 9,
                          'p', 'a', 's', 's', 'w', 'o', 'r', 'd', '1'};
      if (!p.ok(p.request(8, reg::probe_config::kOpSet, item))) return false;
      p.hash_entry = p.hash();
      return p.wifi.count == 1 && p.hash_entry != p.hash_empty;
    }
    return true;
  }
  if (has("probe.config state")) {
    // slot 0 at boot on the rvswd wire (fn 4), pins 1 / 2, 1 MHz, dmseq console; port 0 bound to its console; set at
    // 1 ms, so the automatic attach is tried then
    if (!p.open()) return false;
    Bytes slot = {0, 4, 0, 1, 0, 2, 0, reg::probe_config::kSlotAttachAtBoot, 0, 0, 0, 0, 0x40, 0x42, 0x0f, 0x00, 0,
                  reg::target_console::kMechanismDmseq, 3, 'd', 'u', 't'};
    Bytes items = {uint8_t(reg::probe_config::kTlvItemSlot | kTagCritical), uint8_t(slot.size()), 0};
    items.insert(items.end(), slot.begin(), slot.end());
    const Bytes bind = {uint8_t(reg::probe_config::kTlvItemBind | kTagCritical), 4, 0, 0, Binds::kSlotConsole, 0, 0};
    items.insert(items.end(), bind.begin(), bind.end());
    g_millis = 1;
    g_micros_part = 0;
    if (!p.ok(p.request(8, reg::probe_config::kOpSet, items))) return false;
    p.config.poll();
    p.ep.poll();
    return p.port.connected && p.port.number == 1;
  }
  if (has("save not offered")) { p.config.setStorage(false); return true; }
  if (has("probe.config set")) return p.open();
  if (has("riscv-dm run not offered")) { p.riscv.offerOptional(false); return true; }
  if (has("logic")) {
    // a one-shot of 1000 samples on one line (channel 0), started at 5 ms, done: generation 1
    g_fake_heap = FakeHeap{};
    g_fake_heap.internal_free = 512 * 1024;
    g_fake_heap.psram_total = g_fake_heap.psram_free = size_t(32) << 20;
    if (!p.open() || !p.plan(9, 0, 0)) return false;
    Bytes c = {uint8_t(reg::fixture_logic::kTlvConfigureMode | kTagCritical), 1, 0, reg::fixture_logic::kModeOneShot,
               uint8_t(reg::fixture_logic::kTlvConfigureRate | kTagCritical), 4, 0, 0x40, 0x42, 0x0f, 0x00,
               reg::fixture_logic::kTlvConfigureSamples, 4, 0, 0xe8, 0x03, 0, 0};
    const Bytes a = p.request(9, reg::fixture_logic::kOpConfigure, c);
    if (!p.ok(a)) return false;
    for (size_t at = 5; at + kTlvHeader <= a.size(); at += kTlvHeader + getU16(&a[at + 1]))   // actual_samples
      if (a[at] == reg::fixture_logic::kTlvConfigureAnswerActualSamples) p.actual_samples = getU32(&a[at + kTlvHeader]);
    g_millis = 5;
    g_micros_part = 0;
    if (!p.ok(p.request(9, reg::fixture_logic::kOpStart, {}))) return false;
    if (has("segments")) {   // the PARLIO's DMA writes the segment, then its done
      g_fake_tasks_deferred = true;
      g_fake_task_fn = nullptr;
      size_t at = 0;
      for (size_t n = g_fake_parlio_size; n;) {
        size_t k = n < 4096 ? n : 4096;
        memset(g_fake_parlio_buffer + at, 0x55, k);
        parlio_rx_event_data_t e = {g_fake_parlio_buffer + at, k};
        g_fake_parlio_callbacks.on_partial_receive(nullptr, &e, g_fake_parlio_context);
        at += k;
        n -= k;
      }
      parlio_rx_event_data_t e = {g_fake_parlio_buffer, g_fake_parlio_size};
      g_fake_parlio_callbacks.on_receive_done(nullptr, &e, g_fake_parlio_context);
      g_fake_tasks_deferred = false;
    }
    return true;
  }
  return true;   // the refusals that come before any state (core §4.3 orders 1, 5, 6), oep.probe.link
}

// A case of capture §5 (multirate, the second definition on TLVs 0x60): this probe does not offer it (its describe has
// no 0x60), so only the case of a fn without it applies - run on fn 9.
static bool multirate(const Json &c) {
  const std::string &name = c["name"].string;
  return name.find("multirate") != std::string::npos && name.find("does not declare") == std::string::npos;
}
// A case whose answer depends on the example probe's segment of 1000 bytes (this one's repeat segments are whole 4 KiB):
// the dropped segment of capture §2.2, checked with this probe's sizes in tests/host/test_capture.cpp (testLostChunk).
static bool ownSegments(const Json &c) { return c["state"].string.find("overflowed inside serial 2") != std::string::npos; }
// Cases of a probe other than OpsProbe's fns 1 to 11 (their own small probes below, or a host test of their own).
static bool elsewhere(const Json &c) {
  if (c["name"].string.find("does not declare") != std::string::npos) return false;   // fn 16: run as fn 9
  for (const auto &m : c["fns"].members) if (atoi(m.first.c_str()) >= 12) return true;
  return c["state"].string.find("marks 5 to 8 kept") != std::string::npos;
}

static void testOps(const char *file, const char *list) {
  const Json v = load(file);
  for (const Json &c : v[list].items) {
    if (elsewhere(c)) continue;
    if (multirate(c) || ownSegments(c)) {
      printf("  %s \"%s\": %s\n", file, c["name"].string.c_str(), multirate(c) ? "multirate, not offered" : "in test_capture");
      continue;
    }
    ++cases;
    boot();
    OpsProbe p;
    const std::string name = c["name"].string;
    Bytes req = hex(c["request_hex"].string);
    if (name.find("does not declare") != std::string::npos && req.size() > 5) req[3] = 9, req[4] = 0;   // fn 16 -> fn 9
    p.corr = static_cast<uint16_t>(getU16(&req[1]) - 40);
    if (p.corr == 0 || p.corr > 0xffd0) p.corr = static_cast<uint16_t>(p.corr - 48);   // never 0 on the way (core §4.1)
    if (!setUp(p, name, c["state"].string, c["fns"])) {
      printf("  ops \"%s\": the state could not be set up\n", name.c_str());
      CHECK(false);
      continue;
    }
    if (sessionOf(req) == kS && !p.ep.locked() && !p.no_open) CHECK(p.open());
    Bytes want = hex(c["answer_hex"].string);
    OpsProbe::restarts = 0;
    Bytes got = p.send(req);
    if (c["state"].string.find("items has wifi") != std::string::npos) {
      // the placeholders 0x5A5A0001 (entry 0 set) / 0x5A5A0002 (empty) / 0x5A5A0003 (the new settings of a set, the
      // 112-byte one of the longest wifi item) -> this probe's hashes; a set that made entry 0 has it now
      if (!p.hash_entry && name.find("set: wifi entry 0") != std::string::npos) p.hash_entry = p.hash();
      CHECK(!p.hash_entry || p.hash_entry != p.hash_empty);
      uint32_t hash_new = 0;
      if (c["state"].string.find("the probe's hash for the new settings is 0x5A5A0003") != std::string::npos) {
        CHECK(req.size() == reg::kLimitWifiMinMaxFrame && p.wifi.count == 1);   // a 112-byte set, taken
        hash_new = p.hash();
        CHECK(hash_new != p.hash_empty);
      }
      for (size_t i = 5; i + 4 <= want.size(); ++i)
        if (want[i + 1] == 0 && want[i + 2] == 0x5a && want[i + 3] == 0x5a && want[i] >= 1 && want[i] <= 3) {
          putU32(&want[i], want[i] == 1 ? p.hash_entry : want[i] == 2 ? p.hash_empty : hash_new);
          i += 3;
        }
    }
    if (names(c["fns"], "oep.probe.restart"))   // the restart follows its success answer only (oep-if-restart §2)
      CHECK(OpsProbe::restarts == (name.find("completed success") != std::string::npos ? 1 : 0));
    if (c["state"].string.find("roles 0 and 1") != std::string::npos && got.size() == want.size()) {
      // actual_samples (0x52) is the probe's own: 200000 asked at w = 2 is whole 128-byte cache lines (200192)
      for (size_t at = 5; at + kTlvHeader + 4 <= want.size(); at += kTlvHeader + getU16(&want[at + 1]))
        if (want[at] == reg::fixture_logic::kTlvConfigureAnswerActualSamples && got[at] == want[at]) {
          CHECK(getU32(&got[at + kTlvHeader]) >= getU32(&want[at + kTlvHeader]));
          putU32(&want[at + kTlvHeader], getU32(&got[at + kTlvHeader]));
        }
    }
    if (name.find("logic segments") != std::string::npos && want.size() == 5 + 2 + 37 && got.size() == want.size()) {
      putU32(&want[5 + 2 + 12], p.actual_samples);                   // the probe's own values (see the top)
      putU32(&want[5 + 2 + 24], LogicCapture::kStartUncertaintyNs);
    }
    CHECK(same(file, name, got, want));
  }
}

// core §7.3: every describe TLV fits the smallest max_frame of the firmware's transports (512, the classic ESP32's)
// with the answer's header 5 and more 1: its value at most 512 - 9. `page(first)` answers describe from TLV `first`.
template <typename Page>
static bool describeFits(Page page) {
  constexpr size_t kSmallestMaxFrame = 512;
  bool fits = true;
  for (uint16_t first = 0;;) {
    const Bytes d = page(first);
    if (d.size() < 6 || d[3] != kResolutionCompleted) return false;
    uint16_t n = 0;
    for (size_t at = 6; at + kTlvHeader <= d.size(); at += kTlvHeader + getU16(&d[at + 1]), ++n)
      if (getU16(&d[at + 1]) > kSmallestMaxFrame - 9) {
        printf("  describe TLV 0x%02x of %u bytes\n", d[at], getU16(&d[at + 1]));
        fits = false;
      }
    if (!d[5] || n == 0) return fits;
    first = static_cast<uint16_t>(first + n);
  }
}

// ---- the cases of other probes (`fns` 12 to 15, a small mark ring) -----------------------------------------------

// An interface that only fills a fn number (the vectors' probe has other interfaces there).
class Filler final : public Interface {
 public:
  const char *name() const override { return "io.github.test.filler"; }
  uint16_t instance() const override { return 0; }
  bool offers(uint8_t op) const override { return op == 1; }
  Result handle(uint8_t, const uint8_t *, size_t, uint8_t *, size_t) override { return rejected(kRejectUnknownOperation); }
};

// A capture track for the group's vectors: it starts at once (one-shot, immediate), its generation one up per start
// (as the captures', nextGeneration), and is told / tells the trigger's time.
class VecTrack final : public Interface, public GroupTrack {
 public:
  explicit VecTrack(const char *name) : name_(name) {}
  const char *name() const override { return name_; }
  uint16_t instance() const override { return 0; }
  bool offers(uint8_t op) const override { return op == 1; }
  Result handle(uint8_t, const uint8_t *, size_t, uint8_t *, size_t) override { return rejected(kRejectUnknownOperation); }
  bool trackReady() const override { return state != 2 && state != 3; }
  uint8_t trackMode() const override { return 1; }
  bool trackTriggered() const override { return trigger; }
  uint32_t trackLoad() const override { return 0; }
  bool trackStart() override { state = 3; fired = false; generation = nextGeneration(generation); return true; }
  bool trackCanFollow() const override { return trackReady(); }
  bool trackStartFollowing() override { return trackStart(); }
  bool trackTriggerNs(uint64_t &ns) const override { if (fired) ns = fired_ns; return fired; }
  uint32_t trackGeneration() const override { return generation; }
  void trackStop() override { state = 1; }
  uint8_t trackState() const override { return state; }
  bool trackRate(uint32_t &n, uint32_t &d) const override { n = num; d = den; return true; }
  uint32_t trackPretrigger() const override { return pretrigger; }
  bool trackCanKeep(uint32_t p, uint64_t) const override { return p <= max_pretrigger && p < samples; }
  uint8_t state = 1;
  bool trigger = false, fired = false;
  uint64_t fired_ns = 0;
  uint32_t generation = 0;
  uint32_t num = 1000000, den = 1, pretrigger = 0, max_pretrigger = 0, samples = 1000;

 private:
  const char *name_;
};

// fn 9 logic and fn 13 analog (VecTrack), fn 12 oep.fixture.capture-group, fn 14 oep.fixture.spi-target, on a vendor
// bulk with max_frame 1024; the session 0x11223344 open.
struct WideProbe {
  MemStream bulk;
  uint8_t rx[2200], tx[1100];
  Endpoint ep{bulk, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 4}, Endpoint::kVendorBulk, 0};
  Filler fill[11];
  VecTrack logic{reg::fixture_logic::kName}, analog{reg::fixture_analog::kName};
  CaptureGroup group{ep};
  PinTable pins{(1ull << 4) | (1ull << 5) | (1ull << 6) | (1ull << 7)};
  P4SpiTarget spi{pins};
  uint16_t corr = 1;
  WideProbe() {
    for (int k = 0; k < 8; ++k) ep.add(fill[k]);   // fns 1 - 8
    ep.add(logic);                                 // fn 9
    ep.add(fill[8]);
    ep.add(fill[9]);                               // fns 10, 11
    ep.add(group);                                 // fn 12
    ep.add(analog);                                // fn 13
    ep.add(spi);                                   // fn 14
    group.addTrack(logic, logic);
    group.addTrack(analog, analog);
    const RoleAssignment roles[] = {{14, P4SpiTarget::kRoleSck, 4}, {14, P4SpiTarget::kRoleMosi, 5},
                                    {14, P4SpiTarget::kRoleMiso, 6}, {14, P4SpiTarget::kRoleCs, 7}};
    spi.planApply(roles, 4);
  }
  ~WideProbe() { spi.planRelease(); }
  Bytes send(const Bytes &m) { return bulkMessage(ep, bulk, m); }
  Bytes request(uint16_t fn, uint8_t op, const Bytes &payload, uint32_t session = kS) {
    Bytes m = {kRoleRequest, uint8_t(corr), uint8_t(corr >> 8), uint8_t(fn), uint8_t(fn >> 8), op,
               uint8_t(session), uint8_t(session >> 8), uint8_t(session >> 16), uint8_t(session >> 24)};
    ++corr;
    m.insert(m.end(), payload.begin(), payload.end());
    return send(m);
  }
  static bool ok(const Bytes &a) { return a.size() >= 5 && a[3] == kResolutionCompleted && a[4] == kOutcomeSuccess; }
  bool open() { return ok(request(0, kOpOpen, {0xd0, 0x07, 0, 0, 0})); }
  // The frames the endpoint sends at its next poll (length-prefixed messages).
  std::vector<Bytes> frames() {
    const Bytes t = wire(ep, bulk, {});
    std::vector<Bytes> out;
    for (size_t at = 0; at + 2 <= t.size();) {
      const size_t n = getU16(&t[at]);
      if (at + 2 + n > t.size()) break;
      out.emplace_back(t.begin() + at + 2, t.begin() + at + 2 + n);
      at += 2 + n;
    }
    return out;
  }
};

// The group's generations "before" (the vector's state: group 4, fn 9 3, fn 13 1): fn 13 started alone once, fn 9
// alone three times - every start of the group one up for it and for each track it binds.
static bool groupBefore(WideProbe &p) {
  const Bytes alone13 = {1, 13, 0}, alone9 = {1, 9, 0}, none = {0};
  for (int run = 0; run < 4; ++run) {
    if (!p.ok(p.request(12, reg::fixture_capture_group::kOpBind, run == 0 ? alone13 : alone9))) return false;
    if (!p.ok(p.request(12, reg::fixture_capture_group::kOpStart, {}))) return false;
    if (!p.ok(p.request(12, reg::fixture_capture_group::kOpStop, {}))) return false;
    if (!p.ok(p.request(12, reg::fixture_capture_group::kOpBind, none))) return false;
  }
  return p.logic.generation == 3 && p.analog.generation == 1;
}

static void testWideCases() {
  const Json v = load("ops.json");
  for (const Json &c : v["cases"].items) {
    if (!elsewhere(c)) continue;
    const std::string name = c["name"].string;
    if (multirate(c)) {
      printf("  ops.json \"%s\": multirate, not offered\n", name.c_str());
      continue;
    }
    if (names(c["fns"], "oep.target.arm-adi")) {   // a simulated SWD target: tests/host/test_swd.cpp runs it
      printf("  ops \"%s\": in test_swd\n", name.c_str());
      continue;
    }
    ++cases;
    boot();
    const Bytes req = hex(c["request_hex"].string);
    Bytes want = hex(c["answer_hex"].string), got;
    if (names(c["fns"], "oep.target.console")) {
      // the console's marks (common §1.3) from its PositionStream: a ring of 4 marks with serials 0 - 8 made (5 - 8
      // kept), mark k at position 10 k (k >= 5), time k ms, kind 7 (reset) detail k; max_frame 64: 59 bytes of payload
      uint8_t buffer[1024];
      PositionStream::Mark marks[4];
      PositionStream st(buffer, sizeof buffer, marks, 4);
      for (uint32_t k = 0; k <= 8; ++k) {
        while (st.end() < 10 * k) st.put(0);
        g_millis = k;
        g_micros_part = 0;
        st.mark(7, static_cast<uint8_t>(k));
      }
      uint8_t out[64 - kResultHeader];
      got = {kRoleResult, req[1], req[2], kResolutionCompleted, kOutcomeSuccess};
      const size_t n = st.marks(getU32(&req[12]), out, sizeof out);
      got.insert(got.end(), out, out + n);
    } else if (names(c["fns"], "oep.fixture.spi-target")) {
      // configured with the case's bit order, armed for 4 bytes (tx ff ff: the work registers' stale bits), then 12
      // MOSI bits 1 1 0 0 0 0 0 0 1 0 1 1 and CS high
      WideProbe p;
      const uint8_t order = c["state"].string.find("bit_order 1") != std::string::npos ? 1 : 0;
      CHECK(p.open());
      CHECK(p.ok(p.request(14, P4SpiTarget::kOpConfigure, {0, order})));
      CHECK(p.ok(p.request(14, P4SpiTarget::kOpArm, {4, 0, 2, 0, 0xff, 0xff})));
      const uint8_t wire_bits[] = {0xc0, 0xb0};
      CHECK(fakeSpiTransfer(12, wire_bits));
      p.spi.service();
      p.corr = static_cast<uint16_t>(getU16(&req[1]) - 1);
      got = p.send(req);
    } else if (name.find("cannot keep the group's pretrigger") != std::string::npos) {
      // fn 9: 20 MHz, an edge trigger, pretrigger 100000 (5 ms); fn 13: 48 kHz, 4800 samples, max_pretrigger 128
      WideProbe p;
      p.logic.trigger = true;
      p.logic.num = 20000000;
      p.logic.pretrigger = 100000;
      p.analog.num = 48000;
      p.analog.samples = 4800;
      p.analog.max_pretrigger = 128;
      CHECK(p.open());
      p.corr = static_cast<uint16_t>(getU16(&req[1]) - 1);
      got = p.send(req);
    } else if (names(c["fns"], "oep.fixture.capture-group")) {
      WideProbe p;
      CHECK(p.open() && groupBefore(p));
      CHECK(p.ok(p.request(12, reg::fixture_capture_group::kOpBind, {2, 9, 0, 13, 0})));
      g_millis = 7;   // acquisition starts at 7 ms
      g_micros_part = 0;
      p.corr = static_cast<uint16_t>(getU16(&req[1]) - 1);
      got = p.send(req);
    } else {
      printf("  ops \"%s\": no probe for it here\n", name.c_str());
      CHECK(false);
      continue;
    }
    CHECK(same("ops.json", name, got, want));
  }
  {   // core §7.3: fn 12 (capture-group) and fn 14 (spi-target) describe TLVs fit the smallest max_frame
    boot();
    WideProbe w;
    for (const uint16_t fn : {uint16_t(12), uint16_t(14)})
      CHECK(describeFits([&](uint16_t first) { return w.request(0, kOpDescribe, {uint8_t(fn), 0, uint8_t(first), uint8_t(first >> 8)}, 0); }));
  }
  // The notification frames (core §11.2) with the generation each carries: the group's from WideProbe (fn 12
  // subscribed, group generation 5, fn 9's trigger at 7.05 ms, then every track done); fn 9's own in testLogicEvents.
  boot();
  WideProbe p;
  CHECK(p.open() && groupBefore(p));
  CHECK(p.ok(p.request(12, kOpSubscribe, {0, 0, 0, 0, 0, 0})));
  p.logic.trigger = true;   // fn 9 configured with a trigger: the trigger track
  CHECK(p.ok(p.request(12, reg::fixture_capture_group::kOpBind, {2, 9, 0, 13, 0, 0x01, 2, 0, 9, 0})));
  CHECK(p.ok(p.request(12, reg::fixture_capture_group::kOpStart, {})));
  p.group.poll();   // the trigger track starts once the follower is armed
  p.frames();
  p.logic.fired = true;
  p.logic.fired_ns = 7050000;
  p.logic.state = 4;
  p.analog.state = 4;
  p.group.poll();   // triggered, then stopped (every track done)
  std::vector<Bytes> sent = p.frames();
  for (const Json &e : v["events"].items) {
    if (!names(e["fns"], "oep.fixture.capture-group")) {
      continue;   // fn 9's own: testLogicEvents
    }
    ++cases;
    const Bytes want = hex(e["event_hex"].string);
    bool found = false;
    for (const Bytes &f : sent) found |= f == want;
    if (!found) for (const Bytes &f : sent) printf("    sent %s\n", toHex(f).c_str());
    CHECK(same("events", e["name"].string, found ? want : Bytes{}, want));
  }
}

// ops.json's events of fn 9: the logic capture's own stopped and triggered, each with the generation it was born in
// (capture §3.4), and the stopped of a segment lost to the queue (§2.2). Generation 3 stopped by the host, then generation 4 - one-shot, 20 MHz on one line, a falling edge
// on role 0 with pretrigger 1000, started at 7 ms - with the edge at sample 1000 (7.05 ms). The frames' seq is this
// run's own (the vector's probe sent other events before); the rest byte for byte.
static void testLogicEvents() {
  const Json v = load("ops.json");
  boot();
  OpsProbe p;
  p.corr = 1;
  g_fake_heap = FakeHeap{};
  g_fake_heap.internal_free = 512 * 1024;
  g_fake_heap.psram_total = g_fake_heap.psram_free = size_t(32) << 20;
  CHECK(p.open() && p.plan(9, 0, 0) && p.ok(p.request(9, kOpSubscribe, {0, 0, 0, 0, 0, 0})));
  const Bytes immediate = {uint8_t(reg::fixture_logic::kTlvConfigureMode | kTagCritical), 1, 0, reg::fixture_logic::kModeOneShot,
                           uint8_t(reg::fixture_logic::kTlvConfigureRate | kTagCritical), 4, 0, 0x00, 0x2d, 0x31, 0x01,
                           reg::fixture_logic::kTlvConfigureSamples, 4, 0, 0x00, 0x10, 0, 0};   // 20 MHz, 4096 samples
  CHECK(p.ok(p.request(9, reg::fixture_logic::kOpConfigure, immediate)));
  for (int k = 0; k < 3; ++k) {   // generations 1 to 3, each stopped before any data: stopped reason 1 only
    CHECK(p.ok(p.request(9, reg::fixture_logic::kOpStart, {})));
    CHECK(p.ok(p.request(9, reg::fixture_logic::kOpStop, {})));
  }
  auto frames = [&](const Bytes &t) {
    std::vector<Bytes> out;
    for (size_t at = 0; at + 2 <= t.size();) {
      const size_t n = getU16(&t[at]);
      if (at + 2 + n > t.size()) break;
      out.emplace_back(t.begin() + at + 2, t.begin() + at + 2 + n);
      at += 2 + n;
    }
    return out;
  };
  std::vector<Bytes> sent = frames(p.bulk.tx);
  Bytes edge = immediate;
  edge.insert(edge.end(), {uint8_t(reg::fixture_logic::kTlvConfigureTrigger | kTagCritical), 6, 0,
                           reg::fixture_logic::kTriggerEdge, 0, 1, 0, 0, 0,
                           uint8_t(reg::fixture_logic::kTlvConfigurePretrigger | kTagCritical), 4, 0, 0xe8, 0x03, 0, 0});
  CHECK(p.ok(p.request(9, reg::fixture_logic::kOpConfigure, edge)));
  g_millis = 7;
  g_micros_part = 0;
  g_fake_tasks_deferred = true;
  g_fake_task_fn = nullptr;
  CHECK(p.ok(p.request(9, reg::fixture_logic::kOpStart, {})));
  for (const uint8_t value : {uint8_t(0xff), uint8_t(0x00)}) {   // samples 0 - 999 high, then low from 1000 on
    memset(g_fake_parlio_buffer + (value ? 0 : 125), value, 125);
    parlio_rx_event_data_t e = {g_fake_parlio_buffer + (value ? 0 : 125), 125};
    g_fake_parlio_callbacks.on_partial_receive(nullptr, &e, g_fake_parlio_context);
  }
  struct Idle {};
  g_fake_queue_empty = [] { throw Idle{}; };
  try { if (g_fake_task_fn) g_fake_task_fn(g_fake_task_arg); } catch (const Idle &) {}
  g_fake_queue_empty = nullptr;
  g_fake_tasks_deferred = false;
  p.logic.poll();
  const std::vector<Bytes> later = frames(wire(p.ep, p.bulk, {}));
  sent.insert(sent.end(), later.begin(), later.end());
  p.logic.planRelease();   // the one PARLIO RX unit back
  {   // generation 1 of a repeat (4096-byte segments) whose queue overflowed inside serial 2: stopped reason 3, error 2
    boot();
    OpsProbe q;
    q.corr = 1;
    g_fake_heap = FakeHeap{};
    g_fake_heap.internal_free = 512 * 1024;
    g_fake_heap.psram_total = g_fake_heap.psram_free = size_t(32) << 20;
    CHECK(q.open() && q.plan(9, 0, 0) && q.ok(q.request(9, kOpSubscribe, {0, 0, 0, 0, 0, 0})));
    const Bytes repeat = {uint8_t(reg::fixture_logic::kTlvConfigureMode | kTagCritical), 1, 0, reg::fixture_logic::kModeRepeat,
                          uint8_t(reg::fixture_logic::kTlvConfigureRate | kTagCritical), 4, 0, 0x00, 0x2d, 0x31, 0x01,
                          reg::fixture_logic::kTlvConfigureSamples, 4, 0, 0x00, 0x80, 0, 0};   // 32768 samples: 4096 bytes
    CHECK(q.ok(q.request(9, reg::fixture_logic::kOpConfigure, repeat)));
    g_fake_tasks_deferred = true;
    g_fake_task_fn = nullptr;
    CHECK(q.ok(q.request(9, reg::fixture_logic::kOpStart, {})));
    size_t at = 0;
    auto deliver = [&](size_t n, size_t chunk) {   // the DMA ring, in chunks the queue (128) takes
      for (; n; n -= chunk, at = (at + chunk) % g_fake_parlio_size) {
        parlio_rx_event_data_t e = {g_fake_parlio_buffer + at, chunk};
        g_fake_parlio_callbacks.on_partial_receive(nullptr, &e, g_fake_parlio_context);
      }
    };
    auto harvest = [] {
      struct Idle {};
      g_fake_queue_empty = [] { throw Idle{}; };
      try { if (g_fake_task_fn) g_fake_task_fn(g_fake_task_arg); } catch (const Idle &) {}
      g_fake_queue_empty = nullptr;
    };
    deliver(2 * 4096 + 1024, 1024);   // serials 0 and 1, a quarter of 2
    harvest();
    deliver(128 * 8 + 16, 8);         // the queue full: 2 chunks lost inside serial 2
    harvest();
    deliver(1024, 1024);              // seen at the next one
    harvest();
    g_fake_tasks_deferred = false;
    q.logic.poll();
    const std::vector<Bytes> more = frames(wire(q.ep, q.bulk, {}));
    sent.insert(sent.end(), more.begin(), more.end());
    q.logic.planRelease();
  }
  for (const Json &e : v["events"].items) {
    if (!names(e["fns"], "oep.fixture.logic") || names(e["fns"], "oep.fixture.capture-group")) continue;
    ++cases;
    const Bytes want = hex(e["event_hex"].string);
    bool found = false;
    for (const Bytes &f : sent)
      if (f.size() == want.size() && f.size() >= kEventHeader) {
        Bytes g = f;
        g[3] = want[3];   // seq: this run's own
        g[4] = want[4];
        found |= g == want;
      }
    if (!found) for (const Bytes &f : sent) printf("    sent %s\n", toHex(f).c_str());
    CHECK(same("events", e["name"].string, found ? want : Bytes{}, want));
  }
}

// oep.probe.restart on the whole probe (oep-if-restart §2): a slot's connection on the rvswd wire with its console bound (the
// settings'), the session's gpio plan, a settings plan - all let go of before the handler runs: the connection closed
// without a reset of the target (the hart left halted), the stream closed, every channel free; the answer out first.
static void testRestartLetsGo() {
  ++cases;
  boot();
  OpsProbe p;
  p.corr = 100;
  static OpsProbe *seen;
  static bool connected, open, held, halted;
  seen = &p;
  OpsProbe::restarts = 0;
  p.ep.setRestart([]() {   // before the first poll: listed as fn 11
    ++OpsProbe::restarts;
    connected = seen->port.connected;
    open = seen->console.isOpen();
    held = seen->pins.owner(1) || seen->pins.owner(2) || seen->pins.owner(3) || seen->pins.owner(6);
    halted = seen->phy.halted;
  }, 2000);
  CHECK(p.ep.restartFn() == 11);
  CHECK(setUp(p, "probe.config state", ""));   // the slot at boot attached, its console bound (session S open)
  CHECK(p.port.connected && p.console.isOpen() && p.phy.halted);
  CHECK(p.plan(2, reg::fixture_gpio::kRoleLine, 3));
  const RoleAssignment settings[] = {{5, reg::fixture_uart::kRoleTx, 6}};
  const uint16_t fn_uart = 5;
  CHECK(p.ep.replacePlan(settings, 1, &fn_uart, 1) == 0);
  CHECK(p.pins.owner(3) != 0 && p.pins.owner(6) != 0);
  const Bytes a = p.request(11, kOpRestart, {});
  CHECK(a.size() == 5 && a[3] == kResolutionCompleted && a[4] == kOutcomeSuccess);
  CHECK(OpsProbe::restarts == 1 && !connected && !open && !held && halted && !p.ep.locked());
  CHECK(p.request(0, kOpLockState, {}, 0).empty());   // nothing served after it
}

// core §7.4: every fn's ops (fn 0, the nine interfaces, oep.probe.plan and oep.probe.restart) is a valid, canonical
// value, the first TLV of its describe; list has fn 1 to 11 and never fn 0.
static void testEveryOpsCanonical() {
  ++cases;
  boot();
  OpsProbe p;
  p.ep.setRestart(OpsProbe::restartHook, 2000);
  for (uint16_t fn = 0; fn <= 11; ++fn) {
    const Bytes d = p.request(0, kOpDescribe, {uint8_t(fn), 0, 0, 0}, 0);
    std::vector<int> ops;
    const bool tagged = d.size() >= 6 + 3 && d[6] == kTagOps && d.size() >= 9u + getU16(&d[7]);
    CHECK(tagged && opsDecode(Bytes(d.begin() + 9, d.begin() + 9 + getU16(&d[7])), ops) && !ops.empty());
    CHECK(describeFits([&](uint16_t first) { return p.request(0, kOpDescribe, {uint8_t(fn), 0, uint8_t(first), uint8_t(first >> 8)}, 0); }));
  }
  const Bytes l = p.request(0, kOpList, {0, 0}, 0);
  CHECK(l.size() >= 8 && getU16(&l[5]) == 11 && getU16(&l[8]) == 1);
  CHECK(p.ep.interfaceAt(10) && strcmp(p.ep.interfaceAt(10)->name(), "oep.probe.plan") == 0);
  CHECK(p.ep.interfaceAt(11) && strcmp(p.ep.interfaceAt(11)->name(), "oep.probe.restart") == 0);
  CHECK(!p.ep.interfaceAt(12));
}

int main() {
  testHeaders();
  testCobs();
  testChecks();
  testConfirm();
  testDiscovery();
  testSessions();
  testOpsEncoding();
  testOps("refusals.json", "cases");
  testOps("ops.json", "cases");
  testWideCases();
  testLogicEvents();
  testRestartLetsGo();
  testEveryOpsCanonical();
  printf("vectors: %d cases, %d checks, %d failures\n", cases, checks, failures);
  return failures ? 1 : 0;
}
