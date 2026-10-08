// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe
#include <assert.h>
#include <stdio.h>
#include "../../examples/Firmware/OepProbe/P4FirmwareIdentity.h"

int main() {
  using Identity = P4FirmwareIdentity;
  const Identity p4{Identity::kMagic, Identity::kVersion, Identity::kP4, 1, 199, 0};
  const Identity p4x{Identity::kMagic, Identity::kVersion, Identity::kP4X, 301, 399, 0};
  assert(p4.compatible(Identity::kP4, 1));
  assert(p4.compatible(Identity::kP4, 103));
  assert(p4.compatible(Identity::kP4, 199));
  assert(p4x.compatible(Identity::kP4X, 301));
  assert(p4x.compatible(Identity::kP4X, 302));
  assert(p4x.compatible(Identity::kP4X, 399));
  // The wrong build is refused in both directions, even if its declared revision range is widened.
  assert(!p4.compatible(Identity::kP4X, 302));
  assert(!p4x.compatible(Identity::kP4, 103));
  Identity bad = p4;
  bad.maxRevision = 399;
  assert(!bad.compatible(Identity::kP4X, 302));
  assert(!p4.compatible(Identity::kP4, 0));
  assert(!p4.compatible(Identity::kP4, 200));
  assert(!p4x.compatible(Identity::kP4X, 300));
  assert(!p4x.compatible(Identity::kP4X, 400));
  // An old firmware without the descriptor, unknown versions and invalid descriptors fail closed.
  assert(!Identity{}.compatible(Identity::kP4, 103));
  bad = p4; bad.magic ^= 1;
  assert(!bad.compatible(Identity::kP4, 103));
  bad = p4; bad.version = 2;
  assert(!bad.compatible(Identity::kP4, 103));
  bad = p4; bad.variant = 0;
  assert(!bad.compatible(0, 103));
  bad = p4; bad.reserved = 1;
  assert(!bad.compatible(Identity::kP4, 103));
  puts("firmware identity: 18 checks passed");
}
