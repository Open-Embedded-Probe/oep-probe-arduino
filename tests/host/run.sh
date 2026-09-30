#!/bin/sh
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Open Embedded Probe
# Host tests of the portable core (serial-port framing, the endpoint's serial-port rules, the binds): g++ and a shim.
set -e
here=$(cd "$(dirname "$0")" && pwd)
src=$here/../../src
out=${TMPDIR:-/tmp}/oep-probe-host-test
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I"$here/shim" -I"$src" -o "$out" "$here/test_serial_share.cpp" \
  "$src/OepFrame.cpp" "$src/OepEndpoint.cpp" "$src/OepBind.cpp" "$src/OepCaptureGroup.cpp"
"$out"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I"$here/shim" -I"$src" -o "$out-bulk" "$here/test_bulk_stream.cpp"
"$out-bulk"
