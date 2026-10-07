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

static void testOps(const char *file, const char *list) {
  const Json v = load(file);
  for (const Json &c : v[list].items) {
    ++cases;
    boot();
    OpsProbe p;
    const std::string name = c["name"].string;
    const Bytes req = hex(c["request_hex"].string);
    p.corr = static_cast<uint16_t>(getU16(&req[1]) - 40);
    if (p.corr == 0 || p.corr > 0xffd0) p.corr = static_cast<uint16_t>(p.corr - 48);   // never 0 on the way (core §4.1)
    if (!setUp(p, name, c["state"].string, c["fns"])) {
      printf("  ops \"%s\": the state could not be set up\n", name.c_str());
      CHECK(false);
      continue;
    }
    if (sessionOf(req) == kS && !p.ep.locked() && !p.no_open) CHECK(p.open());
    Bytes want = hex(c["answer_hex"].string);
    if (name == "rvswd scan: count > 0 with skip") {
      // A stale vector at oep-spec 0f455a0: it still answers malformed, but oep-if-debug §1 (b9b30ad, item 25) says a
      // count > 0 scan does not look at skip. This probe follows the text: the scan runs.
      const Bytes got = p.send(req);
      CHECK(got.size() >= 5 && got[3] == kResolutionCompleted);
      continue;
    }
    OpsProbe::restarts = 0;
    Bytes got = p.send(req);
    if (names(c["fns"], "oep.probe.restart"))   // the restart follows its success answer only (oep-if-restart §2)
      CHECK(OpsProbe::restarts == (name.find("completed success") != std::string::npos ? 1 : 0));
    if (name.find("logic segments") != std::string::npos && want.size() == 5 + 2 + 37 && got.size() == want.size()) {
      putU32(&want[5 + 2 + 12], p.actual_samples);                   // the probe's own values (see the top)
      putU32(&want[5 + 2 + 24], LogicCapture::kStartUncertaintyNs);
    }
    CHECK(same(file, name, got, want));
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
  testRestartLetsGo();
  testEveryOpsCanonical();
  printf("vectors: %d cases, %d checks, %d failures\n", cases, checks, failures);
  return failures ? 1 : 0;
}
