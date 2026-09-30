// Host tests: the vendor bulk stream writes a result frame whole or drops it whole (never half a frame).
#include <stdio.h>
#include <vector>

#include "OepDirectBulkStream.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;
static int failures = 0, checks = 0;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); } } while (0)

static EspUsbDeviceVendor *g_vendor = nullptr;
static int g_complete_after = -1;   // complete one IN transfer after this many waits (-1: never)
static void onWait() {
  if (g_complete_after > 0 && --g_complete_after == 0) g_vendor->complete();
}

static void frame(oep::DirectBulkStream &s, size_t body, uint8_t fill) {
  const uint8_t prefix[2] = {uint8_t(body), uint8_t(body >> 8)};
  std::vector<uint8_t> b(body, fill);
  CHECK(s.write(prefix, 2) == 2);
  CHECK(s.write(b.data(), b.size()) == body);
}

// The host takes every IN transfer queued; then every transfer sent, joined, must parse as whole frames.
static bool whole(EspUsbDeviceVendor &v, std::vector<uint8_t> &fills) {
  for (int k = 0; k < 100 && v.in_flight; ++k) v.complete();
  std::vector<uint8_t> all;
  for (auto &t : v.sent) all.insert(all.end(), t.begin(), t.end());
  size_t i = 0;
  while (i + 2 <= all.size()) {
    const size_t n = all[i] | all[i + 1] << 8;
    if (i + 2 + n > all.size()) return false;
    for (size_t k = 0; k < n; ++k) if (all[i + 2 + k] != all[i + 2]) return false;
    fills.push_back(n ? all[i + 2] : 0);
    i += 2 + n;
  }
  return i == all.size();
}

int main() {
  g_on_wait = onWait;
  {   // a slow host: the two buffers fill, the third frame waits until the host takes one, nothing is lost
    EspUsbDeviceVendor v;
    g_vendor = &v;
    oep::DirectBulkStream s(v);
    CHECK(s.begin());
    for (int k = 0; k < 16; ++k) frame(s, 1000, uint8_t(1 + k));   // 16 KiB: one buffer
    s.flush();
    for (int k = 0; k < 16; ++k) frame(s, 1000, uint8_t(20 + k));
    s.flush();
    g_complete_after = 50;                                           // the host takes an IN 50 waits later
    frame(s, 1000, 99);
    s.flush();
    std::vector<uint8_t> fills;
    CHECK(whole(v, fills) && fills.size() == 33 && fills.back() == 99 && s.dropped() == 0);
  }
  {   // a host that never takes IN: after kWaitMs the frame is dropped whole, and the stream stays in step
    EspUsbDeviceVendor v;
    g_vendor = &v;
    g_complete_after = -1;
    oep::DirectBulkStream s(v);
    CHECK(s.begin());
    for (int k = 0; k < 16; ++k) frame(s, 1000, uint8_t(1 + k));
    s.flush();
    for (int k = 0; k < 16; ++k) frame(s, 1000, uint8_t(20 + k));
    s.flush();
    const uint32_t t0 = g_millis;
    frame(s, 1000, 77);                                              // no room: dropped whole
    CHECK(s.dropped() == 1 && g_millis - t0 >= oep::DirectBulkStream::kWaitMs);
    v.complete();                                                    // the host comes back
    frame(s, 10, 88);
    s.flush();
    std::vector<uint8_t> fills;
    CHECK(whole(v, fills) && fills.back() == 88);
    for (uint8_t f : fills) CHECK(f != 77);                          // not a byte of the dropped frame
  }
  {   // the largest frame (16 KiB + 2) fits one buffer whole
    EspUsbDeviceVendor v;
    g_vendor = &v;
    oep::DirectBulkStream s(v);
    CHECK(s.begin());
    frame(s, 16384, 5);
    s.flush();
    std::vector<uint8_t> fills;
    CHECK(whole(v, fills) && fills.size() == 1 && s.dropped() == 0);
  }
  printf("TEST done %d/%d\n", checks - failures, checks);
  return failures ? 1 : 0;
}
