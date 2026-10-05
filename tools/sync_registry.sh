#!/bin/sh
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Open Embedded Probe
# Copy the generated OEP v1 registry header from the sibling oep-spec checkout (never edit the copy).
set -e
here=$(cd "$(dirname "$0")/.." && pwd)
spec=${OEP_SPEC_DIR:-$here/../oep-spec}
# the library's license lines first (every source file carries them), then the generated header as it is (its own SPDX
# line, when it has one, left out: the library's says the same)
{ printf '// SPDX-License-Identifier: MIT\n// Copyright (c) 2026 Open Embedded Probe\n\n'
  sed '1{/^\/\/ SPDX-License-Identifier/d}' "$spec/generated/oep-v1/oep_v1_registry.h"; } > "$here/src/OepRegistry.h"
echo "synced the registry from oep-spec $(git -C "$spec" rev-parse --short HEAD)"
