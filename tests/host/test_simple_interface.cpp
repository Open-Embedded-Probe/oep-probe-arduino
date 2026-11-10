// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: oep::SimpleInterface (an interface from functions, oep-core §7.4 op_name / op_field) - registration's
// refusals (op below 0x10, taken op or name, a bad name, a repeated field name, a bad type), the describe's op_name and
// op_field TLVs (the failed part's code / message), the request's fields checked before the function (short, a bool
// not 0 / 1, bytes / text / list lengths, a tail with an unknown critical TLV refused unsupported, a non-critical one
// left), the answer packed in order (all the types), fail(), an answer that is not the registered one said as failed,
// lockFree, unsupported() and busy().
#include <stdio.h>
#include <string.h>

#include <string>
#include <vector>

#include "OepSimpleInterface.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using namespace oep;
using Bytes = std::vector<uint8_t>;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

static Result call(SimpleInterface &s, uint8_t op, const Bytes &payload, Bytes &out, size_t capacity = 256) {
  out.assign(capacity, 0);
  const Result r = s.handle(op, payload.data(), payload.size(), out.data(), out.size());
  out.resize(r.length);
  return r;
}
static bool success(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }
static bool failedOutcome(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeFailed; }
static bool rejectedWith(const Result &r, uint8_t reason) { return r.resolution == kResolutionRejected && r.detail == reason; }

static Result addFn(Args &in, Reply &out) {
  const int32_t a = in.i32(), b = in.i32();
  out.i32(a + b);
  return out.done();
}

// every type back: u8 u16 u32 u64 i8 i16 i32 i64 bool, then bytes, text and a u32 list from the request
static Result echoFn(Args &in, Reply &out) {
  out.u8(in.u8());
  out.u16(in.u16());
  out.u32(in.u32());
  out.u64(in.u64());
  out.i8(in.i8());
  out.i16(in.i16());
  out.i32(in.i32());
  out.i64(in.i64());
  out.boolean(in.boolean());
  const Span b = in.bytes();
  out.bytes(b.data, b.length);
  const Span t = in.text();
  out.printf("%.*s!", static_cast<int>(t.length), reinterpret_cast<const char *>(t.data));
  const uint16_t n = in.count();
  uint32_t v[8] = {};
  for (uint16_t i = 0; i < n && i < 8; ++i) v[i] = in.u32();
  out.list(v, n, 4);
  return out.done();
}

static Result failFn(Args &, Reply &out) { return out.fail(7, "no answer at 0x44"); }
static Result shortFn(Args &, Reply &out) { return out.done(); }   // registered one answer field, writes none
static Result wrongFn(Args &, Reply &out) { out.u8(1); return out.done(); }   // registered i32
static Result readFn(Args &, Reply &out) { out.i32(-5); return out.done(); }
static Result rangeFn(Args &in, Reply &out) { return in.u8() > 2 ? out.unsupported() : out.busy(); }

static void put16(Bytes &b, uint16_t v) { b.push_back(v & 0xff); b.push_back(v >> 8); }
static void put32(Bytes &b, uint32_t v) { for (int i = 0; i < 4; ++i) b.push_back((v >> (8 * i)) & 0xff); }
static void put64(Bytes &b, uint64_t v) { for (int i = 0; i < 8; ++i) b.push_back((v >> (8 * i)) & 0xff); }

static void testRegistration() {
  SimpleInterface s("io.github.example.mine", 1);
  CHECK(s.op(0x10, "add", {{"a", i32}, {"b", i32}}, {{"sum", i32}}, addFn));
  CHECK(!s.op(0x0f, "low", {}, {}, readFn));                 // 0x01-0x0F are every interface's
  CHECK(!s.op(0x10, "again", {}, {}, readFn));               // the op taken
  CHECK(!s.op(0x11, "add", {}, {}, readFn));                 // the name taken
  CHECK(!s.op(0x12, "Bad", {}, {}, readFn));                 // a-z 0-9 - _ only
  CHECK(!s.op(0x13, "", {}, {}, readFn));
  CHECK(!s.op(0x14, "a-name-longer-than-24-bytes", {}, {}, readFn));
  CHECK(!s.op(0x15, "dup", {{"x", u8}, {"x", u8}}, {}, readFn));
  CHECK(!s.op(0x16, "type", {{"x", 0x0c}}, {}, readFn));     // no type 12
  CHECK(!s.op(0x17, "listtext", {{"x", list(text)}}, {}, readFn));   // no list of text
  CHECK(!s.op(0x18, "nofn", {}, {}, nullptr));
  CHECK(s.op(0x20, "read-temp_2", {}, {{"temp_mc", i32}}, readFn, kLockFree));
  CHECK(s.offers(0x10) && s.offers(0x20) && !s.offers(0x11));
  CHECK(s.lockFree(0x20) && !s.lockFree(0x10));
  CHECK(std::string(s.name()) == "io.github.example.mine" && s.revision() == 1 && s.instance() == 0);
}

static void testDescribe() {
  SimpleInterface s("io.github.example.mine", 1);
  s.op(0x10, "add", {{"a", i32}, {"b", i32}}, {{"sum", i32}}, addFn);
  s.op(0x20, "read", {}, {{"t", i32}}, readFn, kLockFree);
  uint8_t d[512];
  const size_t n = s.describe(d, sizeof d);
  CHECK(n > 0);
  // walk the TLVs: tag(u8) len(u16) value
  std::vector<std::pair<uint8_t, Bytes>> tlvs;
  for (size_t at = 0; at + 3 <= n;) {
    const uint16_t len = getU16(d + at + 1);
    tlvs.push_back({d[at], Bytes(d + at + 3, d + at + 3 + len)});
    at += 3 + len;
  }
  // add: op_name, request a / b, success sum, failed code / message; read: op_name, success t, failed code / message
  CHECK(tlvs.size() == 1 + 2 + 1 + 2 + 1 + 1 + 2);
  const uint8_t name = reg::kDescribeOpName, field = reg::kDescribeOpField;
  CHECK(tlvs[0].first == name && tlvs[0].second == (Bytes{0x10, reg::core::kOpNameFlagForm, 'a', 'd', 'd'}));
  CHECK(tlvs[1].first == field && tlvs[1].second == (Bytes{0x10, reg::core::kOpFieldPartRequest, i32, 'a'}));
  CHECK(tlvs[2].second == (Bytes{0x10, reg::core::kOpFieldPartRequest, i32, 'b'}));
  CHECK(tlvs[3].second == (Bytes{0x10, reg::core::kOpFieldPartSuccess, i32, 's', 'u', 'm'}));
  CHECK(tlvs[4].second == (Bytes{0x10, reg::core::kOpFieldPartFailed, i32, 'c', 'o', 'd', 'e'}));
  CHECK(tlvs[5].second == (Bytes{0x10, reg::core::kOpFieldPartFailed, text, 'm', 'e', 's', 's', 'a', 'g', 'e'}));
  CHECK(tlvs[6].first == name &&
        tlvs[6].second == (Bytes{0x20, reg::core::kOpNameFlagForm | reg::core::kOpNameFlagLockFree, 'r', 'e', 'a', 'd'}));
  CHECK(tlvs[7].second == (Bytes{0x20, reg::core::kOpFieldPartSuccess, i32, 't'}));
  // too small a buffer: nothing rather than a part
  CHECK(s.describe(d, 8) == 0);
}

static void testRequests() {
  SimpleInterface s("io.github.example.mine", 1);
  s.op(0x10, "add", {{"a", i32}, {"b", i32}}, {{"sum", i32}}, addFn);
  s.op(0x11, "echo",
       {{"a", u8}, {"b", u16}, {"c", u32}, {"d", u64}, {"e", i8}, {"f", i16}, {"g", i32}, {"h", i64}, {"i", boolean},
        {"j", bytes}, {"k", text}, {"l", list(u32)}},
       {{"a", u8}, {"b", u16}, {"c", u32}, {"d", u64}, {"e", i8}, {"f", i16}, {"g", i32}, {"h", i64}, {"i", boolean},
        {"j", bytes}, {"k", text}, {"l", list(u32)}},
       echoFn);
  Bytes out, p;
  put32(p, 2);
  put32(p, static_cast<uint32_t>(-5));
  Result r = call(s, 0x10, p, out);
  CHECK(success(r) && out.size() == 4 && static_cast<int32_t>(getU32(out.data())) == -3);
  // short: malformed, the function not run
  r = call(s, 0x10, Bytes(p.begin(), p.end() - 1), out);
  CHECK(rejectedWith(r, kRejectMalformed));
  // a tail: a non-critical unknown TLV is left, a critical one refused unsupported, a broken one malformed
  Bytes tail = p;
  tail.insert(tail.end(), {0x40, 0x01, 0x00, 0x99});
  CHECK(success(call(s, 0x10, tail, out)));
  tail = p;
  tail.insert(tail.end(), {0xC0, 0x01, 0x00, 0x99});
  r = call(s, 0x10, tail, out);
  CHECK(rejectedWith(r, kRejectUnsupported) && out.size() >= 1 && out[0] == 0xC0);
  tail = p;
  tail.insert(tail.end(), {0x40, 0x05, 0x00});
  CHECK(rejectedWith(call(s, 0x10, tail, out), kRejectMalformed));
  // every type, round trip
  Bytes e;
  e.push_back(0xAB);
  put16(e, 0x1234);
  put32(e, 0x89ABCDEF);
  put64(e, 0x0102030405060708ull);
  e.push_back(0x80);
  put16(e, 0x8000);
  put32(e, 0x80000000u);
  put64(e, 0x8000000000000000ull);
  e.push_back(1);
  put16(e, 3);
  e.insert(e.end(), {1, 2, 3});
  put16(e, 2);
  e.insert(e.end(), {'h', 'i'});
  put16(e, 2);
  put32(e, 10);
  put32(e, 20);
  r = call(s, 0x11, e, out);
  Bytes want = e;
  // the text answered is "hi!": its len 3, one byte more
  const size_t text_at = 1 + 2 + 4 + 8 + 1 + 2 + 4 + 8 + 1 + 2 + 3;
  want[text_at] = 3;
  want.insert(want.begin() + text_at + 2 + 2, '!');
  CHECK(success(r) && out == want);
  // a bool not 0 / 1: malformed
  Bytes bad = e;
  bad[1 + 2 + 4 + 8 + 1 + 2 + 4 + 8] = 2;
  CHECK(rejectedWith(call(s, 0x11, bad, out), kRejectMalformed));
  // a bytes len past the request: malformed
  bad = e;
  bad[1 + 2 + 4 + 8 + 1 + 2 + 4 + 8 + 1] = 200;
  CHECK(rejectedWith(call(s, 0x11, bad, out), kRejectMalformed));
  // a list count past the request: malformed
  bad = e;
  bad[e.size() - 8 - 2] = 3;
  CHECK(rejectedWith(call(s, 0x11, bad, out), kRejectMalformed));
  // an op it does not have
  CHECK(rejectedWith(call(s, 0x30, {}, out), kRejectUnknownOperation));
}

static void testAnswers() {
  SimpleInterface s("io.github.example.mine", 1);
  s.op(0x10, "fail", {}, {{"x", i32}}, failFn);
  s.op(0x11, "short", {}, {{"x", i32}}, shortFn);
  s.op(0x12, "wrong", {}, {{"x", i32}}, wrongFn);
  s.op(0x13, "range", {{"v", u8}}, {}, rangeFn);
  s.op(0x14, "read", {}, {{"x", i32}}, readFn);
  Bytes out;
  Result r = call(s, 0x10, {}, out);
  const char *msg = "no answer at 0x44";
  CHECK(failedOutcome(r) && out.size() == 4 + 2 + strlen(msg) && getU32(out.data()) == 7 && getU16(out.data() + 4) == strlen(msg) &&
        memcmp(out.data() + 6, msg, strlen(msg)) == 0);
  // the message does not fit: the code alone, an empty message
  r = call(s, 0x10, {}, out, 8);
  CHECK(failedOutcome(r) && out.size() == 6 && getU32(out.data()) == 7 && getU16(out.data() + 4) == 0);
  // an answer that is not the registered one: failed, no payload
  r = call(s, 0x11, {}, out);
  CHECK(failedOutcome(r) && out.empty());
  r = call(s, 0x12, {}, out);
  CHECK(failedOutcome(r) && out.empty());
  // the answer past the capacity: failed
  r = call(s, 0x14, {}, out, 3);
  CHECK(failedOutcome(r));
  CHECK(rejectedWith(call(s, 0x13, {3}, out), kRejectUnsupported));
  CHECK(rejectedWith(call(s, 0x13, {1}, out), kRejectUnavailable));
}

int main() {
  testRegistration();
  testDescribe();
  testRequests();
  testAnswers();
  printf("simple-interface: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
