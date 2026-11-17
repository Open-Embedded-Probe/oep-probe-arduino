// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: HidIn (transports §1, the HID rules) beyond the shared vectors (test_vectors.cpp runs hid_reports.json):
// a length over max_frame found by the reader breaks the stream till the next first; bytes that do not fit break it;
// marks wait in their queue for a reader that falls behind, and reports wait for room in it.
#include <stdio.h>

#include <vector>

#include "OepFrame.h"
#include "OepHid.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;
static int failures = 0, checks = 0;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); } } while (0)

using Bytes = std::vector<uint8_t>;
static constexpr size_t kRoom = 12;

// A report of `data` with `first` (count = data's size)
static Bytes report(const Bytes &data, uint16_t first) {
  Bytes r(oep::kHidHead + kRoom, 0);
  r[0] = static_cast<uint8_t>(data.size()), r[1] = 0, r[2] = first & 0xff, r[3] = first >> 8;
  for (size_t i = 0; i < data.size(); ++i) r[oep::kHidHead + i] = data[i];
  return r;
}
static Bytes frame(const Bytes &msg) {
  Bytes f{static_cast<uint8_t>(msg.size()), static_cast<uint8_t>(msg.size() >> 8)};
  f.insert(f.end(), msg.begin(), msg.end());
  return f;
}

// The endpoint's way of reading a HID transport: marks restart the reader; an overlong length asks for a break
static std::vector<Bytes> drain(oep::HidIn &in, oep::FrameReader &reader) {
  std::vector<Bytes> got;
  for (;;) {
    if (in.takeRestart()) reader.consume();
    uint8_t chunk[16];
    size_t n = in.read(chunk, sizeof chunk);
    if (!n) break;
    const uint8_t *p = chunk;
    while (n)
      if (reader.feed(p, n)) {
        if (reader.length()) got.emplace_back(reader.message(), reader.message() + reader.length());
        reader.consume();
      }
    if (reader.overlong()) in.askBreak();
  }
  return got;
}

int main() {
  {   // an overlong length: the reader discards, asks for the break; the next first is read again
    uint8_t ring[128], buf[64];
    oep::HidIn in(ring, sizeof ring);
    oep::FrameReader reader(buf, sizeof buf, 16);
    reader.setTcp(true);
    in.take(report({0x40, 0x00, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10}, 0).data(), oep::kHidHead + kRoom);   // 64 > 16
    CHECK(drain(in, reader).empty());
    const Bytes ok = frame({0x01, 0x02, 0x03});
    in.take(report(ok, 0).data(), oep::kHidHead + kRoom);         // taken after the break: from its first
    CHECK((drain(in, reader) == std::vector<Bytes>{Bytes{1, 2, 3}}));
    CHECK(in.losses() == 1);
  }
  {   // bytes that do not fit: the stream broken, read again from the next first once there is room
    uint8_t ring[16], buf[64];
    oep::HidIn in(ring, sizeof ring);
    oep::FrameReader reader(buf, sizeof buf, 64);
    reader.setTcp(true);
    const Bytes a = frame({1, 2, 3, 4, 5, 6, 7, 8, 9, 10});        // 12 bytes
    in.take(report(a, 0).data(), oep::kHidHead + kRoom);
    in.take(report(a, 0).data(), oep::kHidHead + kRoom);            // 24 > 16: dropped, broken
    CHECK(in.losses() == 1);
    CHECK(drain(in, reader).size() == 1);
    in.take(report(a, 0).data(), oep::kHidHead + kRoom);
    CHECK(drain(in, reader).size() == 1);
  }
  {   // more losses than marks before the reader reads: reports wait till there is room for a mark
    uint8_t ring[256], buf[64];
    oep::HidIn in(ring, sizeof ring);
    oep::FrameReader reader(buf, sizeof buf, 64);
    reader.setTcp(true);
    const Bytes head = frame({1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20});   // 22: 2 reports
    for (int k = 0; k < 6; ++k) {
      in.take(report(Bytes(head.begin(), head.begin() + kRoom), 0).data(), oep::kHidHead + kRoom);
      in.take(report(frame({0x7f}), 0).data(), oep::kHidHead + kRoom);   // first 0 where 10 was due: one missing
    }
    const auto got = drain(in, reader);
    CHECK(got.size() == 4);                                         // a mark each, as many as the queue held
    CHECK(in.losses() == 5);                                        // the 6th pair came while the 5th waited: dropped
    in.take(report(frame({0x55}), 0).data(), oep::kHidHead + kRoom);   // room again: read on from its first
    CHECK((drain(in, reader) == std::vector<Bytes>{Bytes{0x55}}));
  }
  {   // at open: a report without a first is not read, one with is read from it
    uint8_t ring[64], buf[64];
    oep::HidIn in(ring, sizeof ring);
    oep::FrameReader reader(buf, sizeof buf, 64);
    reader.setTcp(true);
    in.take(report({9, 9, 9}, oep::kHidNoStart).data(), oep::kHidHead + kRoom);
    Bytes r = {0xAA, 0xBB};
    const Bytes f = frame({7});
    r.insert(r.end(), f.begin(), f.end());
    in.take(report(r, 2).data(), oep::kHidHead + kRoom);
    CHECK((drain(in, reader) == std::vector<Bytes>{Bytes{7}} && in.losses() == 0));
  }
  printf("hid-in: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
