#!/bin/sh
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Open Embedded Probe
# Host tests of the portable core (serial-port framing, the endpoint's serial-port rules, the binds) and of the
# I2C / SPI targets (the SPI one on a fake spi_slave driver, the I2C one with a fake controller), of the pin table's idle
# states and the gpio fixture's take: g++ and a shim.
set -e
here=$(cd "$(dirname "$0")" && pwd)
src=$here/../../src
out=${TMPDIR:-/tmp}/oep-probe-host-test
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I"$here/shim" -I"$src" -o "$out" "$here/test_serial_share.cpp" \
  "$src/OepFrame.cpp" "$src/OepEndpoint.cpp" "$src/OepBind.cpp" "$src/OepCaptureGroup.cpp"
"$out"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I"$here/shim" -I"$src" -o "$out-bulk" "$here/test_bulk_stream.cpp"
"$out-bulk"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -DOEP_HOST_FAKE_SPI_SLAVE -I"$here/shim" -I"$src" -o "$out-spi" \
  "$here/test_spi_target.cpp" "$src/OepP4SpiTarget.cpp"
"$out-spi"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -DOEP_HOST_FAKE_I2C_SLAVE -I"$here/shim" -I"$src" -o "$out-i2c" \
  "$here/test_i2c_target.cpp" "$src/OepP4I2cTarget.cpp"
"$out-i2c"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I"$here/shim" -I"$src" -o "$out-idle" "$here/test_idle.cpp" \
  "$src/OepFixture.cpp"
"$out-idle"
