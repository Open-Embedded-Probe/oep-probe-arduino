#!/bin/sh
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Open Embedded Probe
# Copy the generated OEP v1 registry header and the shared test vectors from the sibling oep-spec checkout (never edit
# the copies): src/OepRegistry.h, tests/vectors/*.json (tests/host/test_vectors.cpp runs them against the endpoint).
# OEP_SPEC_DIR: the checkout (default ../oep-spec); OEP_SPEC_REF: the commit to take them from (default HEAD) - read
# from git, so the checkout's working tree is left as it is.
set -e
here=$(cd "$(dirname "$0")/.." && pwd)
spec=${OEP_SPEC_DIR:-$here/../oep-spec}
ref=$(git -C "$spec" rev-parse "${OEP_SPEC_REF:-HEAD}")
# the library's license lines first (every source file carries them), then the generated header as it is (its own SPDX
# line, when it has one, left out: the library's says the same)
{ printf '// SPDX-License-Identifier: MIT\n// Copyright (c) 2026 Open Embedded Probe\n\n'
  git -C "$spec" show "$ref:generated/oep-v1/oep_v1_registry.h" | sed '1{/^\/\/ SPDX-License-Identifier/d}'; } > "$here/src/OepRegistry.h"
mkdir -p "$here/tests/vectors"
rm -f "$here/tests/vectors/"*.json
for f in $(git -C "$spec" ls-tree --name-only "$ref" tests/vectors/ | grep '\.json$'); do
  git -C "$spec" show "$ref:$f" > "$here/tests/vectors/$(basename "$f")"
done
echo "$ref" > "$here/tests/vectors/SPEC_COMMIT"
echo "synced the registry and the test vectors from oep-spec $(git -C "$spec" rev-parse --short "$ref")"
