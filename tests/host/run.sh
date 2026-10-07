#!/bin/sh
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Open Embedded Probe
# Host tests of the portable core (the capture-group and a bound track's plan; fn 0's conformance: confirm's transport TLV on every transport kind, discoverable, clock, oep.probe.plan / restart,
# the header refusals, the length-prefixed reader's over-long length and TCP pause; serial-port framing, the endpoint's serial-port rules, the binds) and of the
# I2C / SPI targets (the SPI one on a fake spi_slave driver, also behind a relaying broker's sequence on a UART bridge,
# the I2C one with a fake controller), of the pin table's idle
# states and the gpio fixture's take (with its output drive strength), of the fixture UART's receive on a fake UART
# (the interrupt's core, the fastest rate, a lost mark at or before the first byte after each loss: the ESP-IDF driver's
# events in order and arduino-pico's queue and overrun), of the label convention's line names, of the
# unit id, of the boot guard's count of fast crash-boots and its USB gate, of the RVSWD wire on a fake DMI PHY (pins freed, a version-3 module) and of the target console on it (core §4.3's order,
# the send slot, the marks), and of the RVSWD PHY's attach on a
# simulated target (what is written before the speed is verified, the checks, the budget), of the swio stack on a
# simulated CH32V003 whose reset goes through its bootloader (the connection kept, a silent bootloader waited out by
# the reset op and by attach's reset TLV), of the classic ESP32's gate between its sampler's windows and the SWIO frames
# (two threads; the console paused in an immediate window; the sampler's loop and a trigger search's turns; its interrupts-off spans bounded by the clock), of the whole rvswd stack's
# attach / detach on it (the attach budget as a hard bound, min_clock_hz), and of the SWD wire on a
# simulated SWD target (idle items, wire loss, retries), and of the ESP32-P4 logic capture's configure on a fake PARLIO RX
# driver and heap (samples rounded down, streaming without internal RAM for stages), of the analog scale from a frontend's
# range, and of the analog capture's RP2 build on fakes of its ADC and DMA (configure's order and refusals, the plan,
# stop and plan_release): g++ and a shim.
set -e
here=$(cd "$(dirname "$0")" && pwd)
src=$here/../../src
out=${TMPDIR:-/tmp}/oep-probe-host-test
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I"$here/shim" -I"$src" -o "$out" "$here/test_serial_share.cpp" \
  "$src/OepFrame.cpp" "$src/OepEndpoint.cpp" "$src/OepBind.cpp" "$src/OepCaptureGroup.cpp"
"$out"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I"$here/shim" -I"$src" -o "$out-group" "$here/test_capture_group.cpp" \
  "$src/OepFrame.cpp" "$src/OepEndpoint.cpp" "$src/OepBind.cpp" "$src/OepCaptureGroup.cpp"
"$out-group"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I"$here/shim" -I"$src" -o "$out-core" "$here/test_core_conformance.cpp" \
  "$src/OepFrame.cpp" "$src/OepEndpoint.cpp"
"$out-core"
# a TCP listener's connection slots: one describe entry, answers on their connection, no gap rule, over-long closes,
# the one lock and the notifications' connection
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I"$here/shim" -I"$src" -o "$out-tcp" "$here/test_tcp.cpp" \
  "$src/OepFrame.cpp" "$src/OepEndpoint.cpp"
"$out-tcp"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I"$here/shim" -I"$src" -o "$out-bulk" "$here/test_bulk_stream.cpp"
"$out-bulk"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -DOEP_HOST_FAKE_SPI_SLAVE -I"$here/shim" -I"$src" -o "$out-spi" \
  "$here/test_spi_target.cpp" "$src/OepP4SpiTarget.cpp"
"$out-spi"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -DOEP_HOST_FAKE_SPI_SLAVE -I"$here/shim" -I"$src" -o "$out-broker-spi" \
  "$here/test_broker_spi.cpp" "$src/OepP4SpiTarget.cpp" "$src/OepEndpoint.cpp" "$src/OepFrame.cpp" "$src/OepBind.cpp" \
  "$src/OepCaptureGroup.cpp"
"$out-broker-spi"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -DOEP_HOST_FAKE_I2C_SLAVE -I"$here/shim" -I"$src" -o "$out-i2c" \
  "$here/test_i2c_target.cpp" "$src/OepP4I2cTarget.cpp"
"$out-i2c"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -DOEP_HOST_FAKE_DRIVE -I"$here/shim" -I"$src" -o "$out-idle" "$here/test_idle.cpp" \
  "$src/OepFixture.cpp" "$src/OepEndpoint.cpp" "$src/OepFrame.cpp" "$src/OepBind.cpp" "$src/OepCaptureGroup.cpp"
"$out-idle"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I"$here/shim" -I"$src" -o "$out-idle-nodrive" "$here/test_idle.cpp" \
  "$src/OepFixture.cpp" "$src/OepEndpoint.cpp" "$src/OepFrame.cpp" "$src/OepBind.cpp" "$src/OepCaptureGroup.cpp"
"$out-idle-nodrive"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -DOEP_HOST_FAKE_UART -I"$here/shim" -I"$src" -o "$out-fixture-uart" \
  "$here/test_fixture_uart.cpp" "$src/OepFixture.cpp" "$src/OepEndpoint.cpp" "$src/OepFrame.cpp" "$src/OepBind.cpp" \
  "$src/OepCaptureGroup.cpp"
"$out-fixture-uart"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -DOEP_HOST_FAKE_UART -DOEP_HOST_FAKE_UART_RP2 -I"$here/shim" -I"$src" \
  -o "$out-fixture-uart-rp2" "$here/test_fixture_uart.cpp" "$src/OepFixture.cpp" "$src/OepEndpoint.cpp" "$src/OepFrame.cpp" \
  "$src/OepBind.cpp" "$src/OepCaptureGroup.cpp"
"$out-fixture-uart-rp2"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I"$here/shim" -I"$src" -o "$out-lines" "$here/test_lines.cpp" \
  "$src/OepConfig.cpp"
"$out-lines"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I"$here/shim" -I"$src" -o "$out-unit-id" "$here/test_unit_id.cpp"
"$out-unit-id"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -DOEP_HOST_FAKE_BOOT -I"$here/shim" -I"$src" -o "$out-boot-guard" \
  "$here/test_boot_guard.cpp" "$src/OepBootGuard.cpp"
"$out-boot-guard"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I"$here/shim" -I"$src" -o "$out-wire" "$here/test_wire.cpp" \
  "$src/OepTarget.cpp" "$src/OepCh32Dm.cpp" "$src/OepFrame.cpp" "$src/OepDmConsole.cpp"
"$out-wire"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I"$here/shim" -I"$src" -o "$out-console" "$here/test_console.cpp" \
  "$src/OepConsole.cpp" "$src/OepTarget.cpp" "$src/OepCh32Dm.cpp" "$src/OepFrame.cpp" "$src/OepDmConsole.cpp" \
  "$src/OepFixture.cpp" "$src/OepEndpoint.cpp" "$src/OepBind.cpp" "$src/OepCaptureGroup.cpp"
"$out-console"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -DOEP_HOST_FAKE_RVSWD -I"$here/shim" -I"$src" -o "$out-rvswd" \
  "$here/test_rvswd_phy.cpp" "$src/OepRvswdPhy.cpp"
"$out-rvswd"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -DOEP_HOST_FAKE_RVSWD -I"$here/shim" -I"$src" -o "$out-attach-cycle" \
  "$here/test_attach_cycle.cpp" "$src/OepTarget.cpp" "$src/OepCh32Dm.cpp" "$src/OepRvswdPhy.cpp" "$src/OepFrame.cpp"
"$out-attach-cycle"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -DOEP_HOST_FAKE_SWIO -I"$here/shim" -I"$src" -o "$out-swio" \
  "$here/test_swio.cpp" "$src/OepTarget.cpp" "$src/OepCh32Dm.cpp" "$src/OepSwioPhy.cpp" "$src/OepFrame.cpp" \
  "$src/OepDmConsole.cpp"
"$out-swio"
# the classic's gate between the sampler and the SWIO frames (no frame meets a read; an immediate window exclusive; a
# trigger search's turns inside its bursts, on the sampler's own loop OepSamplerRun.h: what a command makes the target
# do fires the trigger)
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -pthread -DOEP_HOST_FAKE_SWIO \
  -I"$here/shim" -I"$src" -o "$out-wire-gate" "$here/test_wire_gate.cpp" "$src/OepSwioPhy.cpp" "$src/OepCh32Dm.cpp" \
  "$src/OepDmConsole.cpp"
"$out-wire-gate"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -DOEP_HOST_FAKE_SWD -I"$here/shim" -I"$src" -o "$out-swd" \
  "$here/test_swd.cpp" "$src/OepSwd.cpp" "$src/OepFrame.cpp"
"$out-swd"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -DOEP_HOST_FAKE_CONFIG -I"$here/shim" -I"$src" -o "$out-config" "$here/test_config.cpp" \
  "$src/OepConfig.cpp" "$src/OepEndpoint.cpp" "$src/OepFrame.cpp" "$src/OepBind.cpp" "$src/OepCaptureGroup.cpp" \
  "$src/OepTarget.cpp" "$src/OepCh32Dm.cpp" "$src/OepConsole.cpp" "$src/OepDmConsole.cpp" "$src/OepFixture.cpp"
"$out-config"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -DOEP_HOST_FAKE_PARLIO -I"$here/shim" -I"$src" -o "$out-capture" \
  "$here/test_capture.cpp" "$src/OepCapture.cpp" "$src/OepEndpoint.cpp" "$src/OepFrame.cpp" "$src/OepBind.cpp" \
  "$src/OepCaptureGroup.cpp"
"$out-capture"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I"$here/shim" -I"$src" -o "$out-analog-scale" "$here/test_analog_scale.cpp"
"$out-analog-scale"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -DOEP_HOST_FAKE_RP2_ADC -I"$here/shim" -I"$src" -o "$out-analog" \
  "$here/test_analog.cpp" "$src/OepAnalog.cpp" "$src/OepEndpoint.cpp" "$src/OepFrame.cpp" "$src/OepBind.cpp" \
  "$src/OepCaptureGroup.cpp"
"$out-analog"
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -DOEP_HOST_FAKE_PARLIO -DOEP_HOST_FAKE_I2C_SLAVE -DOEP_HOST_FAKE_CONFIG \
  -DOEP_HOST_FAKE_SPI_SLAVE -DOEP_VECTORS_DIR="\"$here/../vectors\"" -I"$here/shim" -I"$src" -o "$out-vectors" "$here/test_vectors.cpp" \
  "$src/OepEndpoint.cpp" "$src/OepFrame.cpp" "$src/OepBind.cpp" "$src/OepCaptureGroup.cpp" "$src/OepCapture.cpp" \
  "$src/OepConfig.cpp" "$src/OepConsole.cpp" "$src/OepDmConsole.cpp" "$src/OepTarget.cpp" "$src/OepCh32Dm.cpp" \
  "$src/OepFixture.cpp" "$src/OepP4I2cTarget.cpp" "$src/OepP4SpiTarget.cpp"
"$out-vectors"
# A platform without a chip number does not build without OEP_UNIT_ID, nor with one outside a-z 0-9 - (core §7.5).
for bad in -DOEP_HOST_NO_UNIT_ID "-DOEP_UNIT_ID=\"Host\"" "-DOEP_UNIT_ID=\"\""; do
  if g++ -std=gnu++17 -fsyntax-only "$bad" -I"$here/shim" -I"$src" "$here/test_unit_id.cpp" 2>/dev/null; then
    echo "unit-id: a build with $bad was accepted"; exit 1
  fi
done
echo "unit-id: builds without a usable OEP_UNIT_ID refused"
