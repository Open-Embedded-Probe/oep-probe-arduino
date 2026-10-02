// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: unit_id (oep-spec oep-core §7.5) is mandatory in describe. On a platform without a chip number the build
// constant OEP_UNIT_ID is the unit id (the shim gives "host-test"; run.sh checks that a build without one, or with a
// value outside a-z 0-9 -, fails); describeCore writes it, and fails rather than leave it out.
#include <stdio.h>

#include "OepPlatform.h"

using namespace oep;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

// The value of tag `tag` in a TLV run, or nullptr.
static const uint8_t *find(const uint8_t *p, size_t n, uint8_t tag, size_t &length) {
  size_t at = 0;
  while (at + 2 <= n) {
    const uint8_t t = p[at];
    size_t l = p[at + 1], head = 2;
    if (l == 255) { l = p[at + 2] | (p[at + 3] << 8); head = 4; }
    if (t == tag) { length = l; return p + at + head; }
    at += head + l;
  }
  return nullptr;
}

int main() {
  uint8_t id[17];
  const size_t n = platformUnitId(id, sizeof id);
  CHECK(n == 9 && memcmp(id, "host-test", 10) == 0);
  CHECK(platformUnitId(id, 5) == 0);   // no room: nothing, not a cut value

  {   // written, as given
    uint8_t buffer[128];
    TlvWriter w(buffer, sizeof buffer);
    CHECK(describeCore(w, "m", reinterpret_cast<const uint8_t *>("host-test"), 9, 8, 0));
    size_t length = 0;
    const uint8_t *value = find(buffer, w.length(), reg::core::kTlvDescribeUnitId, length);
    CHECK(value && length == 9 && memcmp(value, "host-test", 9) == 0);
  }
  {   // none, or too long: the writer fails (no describe without it)
    uint8_t buffer[128], long_id[33];
    memset(long_id, 'a', sizeof long_id);
    TlvWriter w(buffer, sizeof buffer);
    CHECK(!describeCore(w, "m", id, 0, 8, 0));
    CHECK(!w.ok());
    TlvWriter w2(buffer, sizeof buffer);
    CHECK(!describeCore(w2, "m", long_id, sizeof long_id, 8, 0));
    CHECK(!w2.ok());
  }
  printf("unit-id: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
