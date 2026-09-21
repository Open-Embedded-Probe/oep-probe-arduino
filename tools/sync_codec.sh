#!/bin/sh
# Copy the generated OEP v0 codec from the sibling oep-spec checkout into src/.
# The wire numbers live only in oep-spec/registry/oep-v0.yaml; never edit the copies.
set -e
here=$(cd "$(dirname "$0")/.." && pwd)
spec=${OEP_SPEC_DIR:-$here/../oep-spec}
cp "$spec/generated/oep-v0-c/src/oep_v0.h" "$spec/generated/oep-v0-c/src/oep_v0.c" "$here/src/"
printf 'oep-spec %s\n' "$(git -C "$spec" rev-parse --short HEAD)" > "$here/src/OEP_V0_CODEC_SOURCE.txt"
echo "synced codec from oep-spec $(git -C "$spec" rev-parse --short HEAD)"
