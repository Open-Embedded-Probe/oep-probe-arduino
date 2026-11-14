// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the H41x UHSIF logic stream (H41xLogicCapture, oep-spec oep-if-capture §2.1 / §3.4, core §10.2 / §10.4)
// on a fake of the core's CH32RVUhsif (OEP_HOST_FAKE_UHSIF) and a transport that keeps what it is handed: the rates
// (SYSCLK / n and the crystal's 480 MHz / n, the crystal's on a tie), the data lines, then a stream whose transfers,
// put end to end, are whole frames - each a buffer of the stream at its position, with its generation, segment_info and
// padding, the seq one up each frame - every transfer but the lone header 4-byte aligned, the frame closed for a waiting
// result, the oldest buffers dropped when the host does not take IN (the position jumps, the framing holds), a buffer
// written again while it went out (state 6), and the stream ended whole at the stop.
#include <stdio.h>
#include <stdlib.h>

#include <vector>

#include "OepEndpoint.h"
#include "OepH41xLogic.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using namespace oep;
using Bytes = std::vector<uint8_t>;
namespace cap = reg::fixture_logic;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }

// Keeps every buffer it is handed (in order) and gives it back when the test says (the host taking IN)
class HeldDirect final : public DirectTransport {
 public:
  struct Held { const uint8_t *buffer; Done done; void *context; };
  std::vector<Bytes> transfers;
  std::vector<uintptr_t> starts;
  std::vector<bool> opens;
  std::vector<Held> held;
  bool waiting = false;
  bool queueData(const uint8_t *b, size_t n, Done done, void *context) override { return take(b, n, done, context, false); }
  bool queueDataOpen(const uint8_t *b, size_t n, Done done, void *context) override { return take(b, n, done, context, true); }
  bool resultsWaiting() const override { return waiting; }
  size_t queued() const override { return held.size(); }
  void giveBack() {
    std::vector<Held> h;
    h.swap(held);
    for (const Held &e : h) e.done(e.context, e.buffer);
  }
  Bytes all() const {
    Bytes a;
    for (const Bytes &t : transfers) a.insert(a.end(), t.begin(), t.end());
    return a;
  }

 private:
  bool take(const uint8_t *b, size_t n, Done done, void *context, bool open) {
    transfers.emplace_back(b, b + n);   // a copy as handed: what the USB DMA reads now
    starts.push_back(reinterpret_cast<uintptr_t>(b));
    opens.push_back(open);
    if (done) held.push_back({b, done, context});
    return true;
  }
};

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

static uint16_t pattern(uint64_t sample) { return static_cast<uint16_t>(sample * 2654435761u >> 7); }

static void tlv(Bytes &p, uint8_t tag, const Bytes &v) {
  p.push_back(tag);
  p.push_back(static_cast<uint8_t>(v.size()));
  p.push_back(static_cast<uint8_t>(v.size() >> 8));
  p.insert(p.end(), v.begin(), v.end());
}
static Bytes u32(uint32_t v) { Bytes b(4); putU32(b.data(), v); return b; }

static void testRates() {
  CH32RVUhsifClock s;
  uint8_t d;
  uint32_t num, den;
  CHECK(H41xLogicCapture::rateOf(200000000, s, d, num, den) && s == CH32RVUhsifClock::Sysclk && d == 1 &&
        num == 200000000 && den == 1);
  CHECK(H41xLogicCapture::rateOf(160000000, s, d, num, den) && s == CH32RVUhsifClock::UsbhsPll && d == 2 &&
        num == 160000000 && den == 1);
  // 80 MHz both ways (400 / 5, 480 / 6): the crystal's
  CHECK(H41xLogicCapture::rateOf(80000000, s, d, num, den) && s == CH32RVUhsifClock::UsbhsPll && d == 5);
  CHECK(H41xLogicCapture::rateOf(133000000, s, d, num, den) && s == CH32RVUhsifClock::Sysclk && d == 2 &&
        num == 400000000 && den == 3);
  CHECK(H41xLogicCapture::rateOf(500000000, s, d, num, den) && num / den == 200000000);   // the top
  CHECK(H41xLogicCapture::dataLineOf(3 * 24 + 10) == 0 && H41xLogicCapture::dataLineOf(5 * 24 + 2) == 8 &&
        H41xLogicCapture::dataLineOf(0 * 24 + 15) == 15 && H41xLogicCapture::dataLineOf(1 * 24 + 0) == -1);
}

struct Rig {
  MemStream bulk;
  uint8_t rx[512], tx[512];
  Endpoint ep{bulk, rx, sizeof rx, tx, sizeof tx, {512, 1024, 2}, Endpoint::kVendorBulk, 0};
  PinTable pins{~ChannelMask{}};
  H41xLogicCapture logic{ep, pins};
  HeldDirect direct;
  uint16_t corr = 1;
  Rig() {
    ep.add(logic);   // fn 1
    ep.setDirect(&direct);
  }
  bool request(uint16_t fn, uint8_t op, const Bytes &payload) {
    Bytes m = {kRoleRequest, uint8_t(corr), uint8_t(corr >> 8), uint8_t(fn), uint8_t(fn >> 8), op, 0x44, 0x33, 0x22, 0x11};
    ++corr;
    m.insert(m.end(), payload.begin(), payload.end());
    Bytes f = {uint8_t(m.size()), uint8_t(m.size() >> 8)};
    f.insert(f.end(), m.begin(), m.end());
    bulk.rx.insert(bulk.rx.end(), f.begin(), f.end());
    bulk.tx.clear();
    ep.poll();
    return bulk.tx.size() >= 7 && bulk.tx[5] == kResolutionCompleted && bulk.tx[6] == kOutcomeSuccess;
  }
  Bytes out = Bytes(600);
  Result op(uint8_t code, const Bytes &p = {}) {
    out.assign(600, 0);
    const Result r = logic.handle(code, p.data(), p.size(), out.data(), out.size());
    out.resize(r.length);
    return r;
  }
  // roles on DATA0 (PD10), DATA8 (PF2), DATA15 (PA15); open, subscribe; a stream at `rate` with buffers of `frame`
  bool setup(uint32_t rate, size_t frame) {
    RoleAssignment roles[3] = {{1, 0, 3 * 24 + 10}, {1, 1, 5 * 24 + 2}, {1, 2, 0 * 24 + 15}};
    if (logic.planCheck(roles, 3) != 0) { printf("  setup: plan refused\n"); return false; }
    logic.planApply(roles, 3);
    if (!request(0, kOpOpen, {0xd0, 0x07, 0, 0, 0})) { printf("  setup: open\n"); return false; }
    if (!request(1, reg::kOpSubscribe, {0, 0, 0, 0, 0, 0})) { printf("  setup: subscribe\n"); return false; }
    logic.setFrameLimit(frame);
    Bytes c;
    tlv(c, cap::kTlvConfigureMode | 0x80, {cap::kModeStreaming});
    tlv(c, cap::kTlvConfigureRate | 0x80, u32(rate));
    const Result r = op(cap::kOpConfigure, c);
    if (!ok(r)) printf("  setup: configure %u %u\n", r.resolution, r.detail);
    return ok(r);
  }
};

struct Frame { uint64_t position; uint16_t seq; Bytes data; uint32_t generation; uint64_t info_position; uint32_t samples; bool padded; };

// The transfers end to end as the host reads them: whole frames, or false
static bool parse(const Bytes &s, std::vector<Frame> &frames, size_t *rest = nullptr) {
  size_t at = 0;
  while (at + 2 <= s.size()) {
    const size_t len = getU16(s.data() + at);
    if (at + 2 + len > s.size()) break;
    const uint8_t *m = s.data() + at + 2;
    if (len < 15 || m[0] != kRolePush) return false;
    Frame f{};
    f.seq = getU16(m + 3);
    f.position = getU64(m + 5);
    const size_t n = getU16(m + 13);
    if (15 + n > len) return false;
    f.data.assign(m + 15, m + 15 + n);
    for (size_t t = 15 + n; t + kTlvHeader <= len;) {
      const uint8_t tag = m[t];
      const size_t vl = getU16(m + t + 1);
      if (t + kTlvHeader + vl > len) return false;
      const uint8_t *v = m + t + kTlvHeader;
      if (tag == cap::kTlvDataGeneration && vl == 4) f.generation = getU32(v);
      if (tag == cap::kTlvDataSegmentInfo && vl == 24) { f.info_position = getU64(v); f.samples = getU32(v + 8); }
      if (tag == reg::kTagDataPadding) f.padded = true;
      t += kTlvHeader + vl;
    }
    frames.push_back(f);
    at += 2 + len;
  }
  if (rest) *rest = s.size() - at;
  return true;
}

// Every frame's data the stream's at its position, a buffer long, generation and segment_info matching; seq one up
static bool frameContent(const std::vector<Frame> &frames, uint32_t generation, size_t bytes, size_t *jumps) {
  uint64_t next = 0;
  bool same = true;
  if (jumps) *jumps = 0;
  for (size_t i = 0; i < frames.size(); ++i) {
    const Frame &f = frames[i];
    // seq one up a frame, or more where an event took one (data and events share a fn's seq, core §10.2)
    if (i && static_cast<uint16_t>(f.seq - frames[i - 1].seq - 1) > 2) {
      if (same && getenv("UHSIF_DUMP")) printf("  frame %zu: seq %u after %u\n", i, f.seq, frames[i - 1].seq);
      same = false;
    }
    if (f.position != next && jumps) ++*jumps;
    if (f.position < next || f.data.size() != bytes || f.generation != generation || f.info_position != f.position ||
        f.samples != bytes / 2 || !f.padded) {
      if (same && getenv("UHSIF_DUMP"))
        printf("  frame %zu: position %llu (next %llu) len %zu gen %u/%u info %llu samples %u padded %d seq %u\n", i,
               (unsigned long long)f.position, (unsigned long long)next, f.data.size(), f.generation, generation,
               (unsigned long long)f.info_position, f.samples, int(f.padded), f.seq);
      same = false;
    }
    for (size_t k = 0; k + 1 < f.data.size() && same; k += 2) {
      same = getU16(f.data.data() + k) == pattern((f.position + k) / 2);
      if (!same && getenv("UHSIF_DUMP")) printf("  frame %zu (position %llu): byte %zu differs\n", i, (unsigned long long)f.position, k);
    }
    next = f.position + f.data.size();
  }
  return same;
}

static void testStream() {
  Rig rig;
  const size_t frame = 4096 + 64;
  CHECK(rig.setup(100000000, frame));
  const size_t bytes = rig.logic.hostBufferBytes();
  CHECK(bytes == 4096);   // the host's frame less a frame's own 64 bytes (length, header, tail), whole 64s
  uint32_t data_rate = 0;
  CHECK(ok(rig.op(cap::kOpStart)));
  const uint32_t generation = getU32(rig.out.data());
  CH32RVUhsif &u = rig.logic.hostUhsif();
  // the layout: w 16, each role its DATA line
  for (int round = 0; round < 12; ++round) {   // 3 buffers a round; polls and the host's IN keep up (2 a run at most)
    u.hostWrite(3 * bytes / 2, pattern);
    for (int k = 0; k < 3; ++k) { rig.logic.poll(); rig.direct.giveBack(); }
  }
  (void)data_rate;
  rig.logic.poll();
  rig.direct.giveBack();
  // every transfer 4-byte aligned but the stream's first header (17 bytes alone)
  bool aligned = true;
  for (size_t i = 0; i < rig.direct.starts.size(); ++i)
    if ((rig.direct.starts[i] & 3) && rig.direct.transfers[i].size() != 17) aligned = false;
  CHECK(aligned);
  std::vector<Frame> frames;
  size_t rest = 0;
  if (getenv("UHSIF_DUMP")) {
    for (size_t i = 0; i < rig.direct.transfers.size() && i < 6; ++i) {
      const Bytes &t = rig.direct.transfers[i];
      printf("  transfer %zu: %zu bytes, at %%4 %u, open %d:", i, t.size(), unsigned(rig.direct.starts[i] & 3), int(rig.direct.opens[i]));
      for (size_t k = 0; k < t.size() && k < 24; ++k) printf(" %02x", t[k]);
      printf("\n");
    }
  }
  CHECK(parse(rig.direct.all(), frames, &rest));
  CHECK(rest > 0);   // the last frame open: its tail goes with the next run
  size_t jumps = 0;
  CHECK(frameContent(frames, generation, bytes, &jumps));
  CHECK(jumps == 0);
  CHECK(frames.size() == 12 * 3 - 1);   // the last open (its tail with the next run)
  // a result waits: the open frame closes alone (whole frames on the wire), the next run starts with its header alone
  rig.direct.waiting = true;
  rig.logic.poll();
  rig.direct.giveBack();
  frames.clear();
  CHECK(parse(rig.direct.all(), frames, &rest) && rest == 0);
  rig.direct.waiting = false;
  u.hostWrite(2 * bytes / 2, pattern);
  rig.logic.poll();
  rig.direct.giveBack();
  CHECK(rig.direct.transfers.size() >= 2 && rig.direct.transfers[rig.direct.transfers.size() - 2].size() == 17);   // the header alone, then the run
  // the stop: the finished buffers go, the stream ends on a frame
  u.hostWrite(bytes / 4, pattern);   // part of a buffer: not finished, not sent
  CHECK(ok(rig.op(cap::kOpStop)));
  for (int i = 0; i < 4; ++i) { rig.logic.poll(); rig.direct.giveBack(); }
  frames.clear();
  CHECK(parse(rig.direct.all(), frames, &rest) && rest == 0);
  CHECK(frameContent(frames, generation, bytes, &jumps) && jumps == 0);
  CHECK(ok(rig.op(cap::kOpStatus)) && rig.out[0] == cap::kStateConfigured && !(rig.out[13] & cap::kStatusFlagDropped));
  CHECK(getU64(rig.out.data() + 5) == frames.back().position + bytes);   // write_pos: the last finished buffer's end
}

// The host takes no IN: the run out stays; the writer gets ahead, the oldest unsent buffers go (position jumps, the
// framing holds) - and once it comes back on the buffers out, the capture stops on the error (state 6)
static void testBehindAndOverwritten() {
  Rig rig;
  CHECK(rig.setup(100000000, 4096 + 64));
  const size_t bytes = rig.logic.hostBufferBytes();
  CHECK(ok(rig.op(cap::kOpStart)));
  const uint32_t generation = getU32(rig.out.data());
  CH32RVUhsif &u = rig.logic.hostUhsif();
  u.hostWrite(bytes / 2, pattern);   // buffer 0: header alone, run [0]
  rig.logic.poll();
  rig.direct.giveBack();
  u.hostWrite(uint64_t(rig.logic.hostBuffers()) / 2 * bytes, pattern);   // far ahead: the whole ring's worth
  rig.logic.poll();   // run out, not given back
  CHECK(ok(rig.op(cap::kOpStatus)) && (rig.out[13] & cap::kStatusFlagDropped));
  rig.direct.giveBack();
  rig.logic.poll();
  rig.direct.giveBack();
  CHECK(ok(rig.op(cap::kOpStop)));
  for (int i = 0; i < 4; ++i) { rig.logic.poll(); rig.direct.giveBack(); }
  std::vector<Frame> frames;
  size_t rest = 0, jumps = 0;
  CHECK(parse(rig.direct.all(), frames, &rest) && rest == 0);
  CHECK(frameContent(frames, generation, bytes, &jumps) && jumps >= 1);

  // the host holds a run while the writer comes round: polled as it goes, the UHSIF stops two buffers short of the
  // run; given back, everything finished goes, the frame closes and the UHSIF starts again - no error, the position
  // jumps past what was not taken, the framing holds
  Rig r3;
  CHECK(r3.setup(100000000, 4096 + 64));
  CHECK(ok(r3.op(cap::kOpStart)));
  const uint32_t gen3 = getU32(r3.out.data());
  CH32RVUhsif &u3 = r3.logic.hostUhsif();
  u3.hostWrite(bytes / 2, pattern);
  r3.logic.poll();   // header + run [0] out, held
  for (uint8_t k = 0; k < r3.logic.hostBuffers(); ++k) { u3.hostWrite(bytes / 2, pattern); r3.logic.poll(); }
  CHECK(!u3.running());   // stopped short of buffer 0
  const int starts = u3.starts;
  for (int i = 0; i < 6; ++i) { r3.direct.giveBack(); r3.logic.poll(); }
  CHECK(u3.running() && u3.starts == starts + 1);   // started again
  CHECK(ok(r3.op(cap::kOpStatus)) && r3.out[0] == cap::kStateCapturing && (r3.out[13] & cap::kStatusFlagDropped));
  u3.hostRewind();   // the new epoch's samples: the pattern from its own first sample (positions say where they are)
  for (int i = 0; i < 3; ++i) { u3.hostWrite(bytes / 2, pattern); r3.logic.poll(); r3.direct.giveBack(); }
  CHECK(ok(r3.op(cap::kOpStop)));
  for (int i = 0; i < 4; ++i) { r3.logic.poll(); r3.direct.giveBack(); }
  std::vector<Frame> f3;
  CHECK(parse(r3.direct.all(), f3, &rest) && rest == 0);
  size_t j3 = 0;
  bool positions_up = true;
  for (size_t i = 1; i < f3.size(); ++i) {
    if (f3[i].position != f3[i - 1].position + bytes) ++j3;
    if (f3[i].position < f3[i - 1].position + bytes) positions_up = false;
    if (f3[i].generation != gen3) positions_up = false;
  }
  CHECK(positions_up && j3 >= 1);

  // the subscription ends with a frame open (the host went away): the frame is finished all the same, so the port's
  // results do not wait for it for ever (core §10.3)
  Rig r4;
  CHECK(r4.setup(100000000, 4096 + 64));
  CHECK(ok(r4.op(cap::kOpStart)));
  CH32RVUhsif &u4 = r4.logic.hostUhsif();
  for (int i = 0; i < 3; ++i) { u4.hostWrite(bytes / 2, pattern); r4.logic.poll(); r4.direct.giveBack(); }
  CHECK(r4.request(1, reg::kOpUnsubscribe, {}));
  for (int i = 0; i < 3; ++i) { u4.hostWrite(bytes / 2, pattern); r4.logic.poll(); r4.direct.giveBack(); }
  std::vector<Frame> f4;
  CHECK(parse(r4.direct.all(), f4, &rest) && rest == 0 && !f4.empty());

  // a buffer written again while it went out all the same (a pause cannot come: not polled meanwhile): state 6
  Rig r2;
  CHECK(r2.setup(100000000, 4096 + 64));
  CHECK(ok(r2.op(cap::kOpStart)));
  CH32RVUhsif &u2 = r2.logic.hostUhsif();
  u2.hostWrite(bytes / 2, pattern);
  r2.logic.poll();   // header + run [0] out, held
  u2.hostWrite(uint64_t(r2.logic.hostBuffers()) * bytes / 2, pattern);   // round the ring onto buffer 0
  r2.direct.giveBack();   // done: the writer was back on it
  r2.logic.poll();
  CHECK(ok(r2.op(cap::kOpStatus)) && r2.out[0] == cap::kStateError);
  CHECK(getU64(r2.out.data() + 5) == 0);   // write_pos: buffer 0's start, the one written again while it went out
  for (int i = 0; i < 4; ++i) { r2.logic.poll(); r2.direct.giveBack(); }
  std::vector<Frame> f2;
  CHECK(parse(r2.direct.all(), f2, &rest) && rest == 0);   // the frame left open is closed even so
}

int main() {
  setvbuf(stdout, nullptr, _IONBF, 0);
  testRates();
  testStream();
  testBehindAndOverwritten();
  printf("logic-uhsif: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
