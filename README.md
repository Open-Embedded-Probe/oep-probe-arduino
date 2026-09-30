# OpenEmbeddedProbe (an Arduino library for OEP probes)

[日本語](README.ja.md)

## What is OEP?

Open Embedded Probe (OEP) is an open protocol between a **probe** - a small board wired to the chip you develop on - and the
**host** software on your PC. One probe can be a debugger (RVSWD / SWIO for WCH's CH32, SWD for ARM), a console to the target,
and a test fixture (GPIO, UART, logic capture) at once, and every host (a flash tool, an IDE monitor, pytest) talks to it
the same way:

- the probe **declares what it can do** (interfaces found by name, with their pins and limits), so a host needs no table of
  boards;
- a **session lock** keeps two programs from driving the probe at the same time;
- it runs over **USB vendor bulk, HID, USB CDC, USB-Serial/JTAG or a plain UART** bridge; a serial port carries the OEP frames
  and the target's console on the same line;
- the probe knows only its wires; **what a target is** (flash layout, loaders) **stays in the host**.

This library turns an ESP32-P4, a classic ESP32, an RP2350 or an RP2040 into such a probe; `examples/` are ready-made
firmware for the jigs we use.

- Specification: [oep-spec](https://github.com/Open-Embedded-Probe/oep-spec) - start with [the review guide](https://github.com/Open-Embedded-Probe/oep-spec/blob/main/docs/review-guide.ja.md); the protocol
  core is [docs/oep-core.ja.md](https://github.com/Open-Embedded-Probe/oep-spec/blob/main/docs/oep-core.ja.md), the wire numbers [registry/oep-v1.toml](https://github.com/Open-Embedded-Probe/oep-spec/blob/main/registry/oep-v1.toml)
  (Japanese first; English follows once it settles).
- Host library: [oep-client-python](https://github.com/Open-Embedded-Probe/oep-client-python) (`pip install oep-client-python`, the
  `oep` command, a fake probe for tests).
- USB VID:PID: until pid.codes grants a PID the reference firmware uses `303a:0002` with an iProduct starting `OEP`; who may use
  the OEP PID once granted is in [PID-USE.md](PID-USE.md).

## This library

A library for writing Open Embedded Probe (OEP) probes with Arduino, and the firmware of each probe (`examples/`). It speaks
the v1 protocol of [oep-spec](https://github.com/Open-Embedded-Probe/oep-spec) (`docs/oep-core.ja.md` and the standard
interfaces `docs/oep-if-*.ja.md`, a candidate being settled). This is an experimental stage: breaking changes are expected and
no compatibility is promised.

The wire numbers are defined only in oep-spec's `registry/oep-v1.toml`; its generated header is copied to
`src/OepRegistry.h`. For a map of the specification, start with oep-spec's `docs/review-guide.ja.md`.

## Layout

| Path | Contents |
|---|---|
| `src/Oep.h`, `src/OepEndpoint.*`, `src/OepRegistry.h` | the core (oep-core): frames, interfaces by name, the lock, several transports (the describe transport list), serial ports shared by frames and raw bytes (core §3.4), the plan, notifications |
| `src/OepBind.*` | what each serial port carries (binds: last-reset / manual / mixed, held during a session and resumed from its last reset) |
| `src/OepStream.h`, `src/OepDebug.h` | parts the standard interfaces share (position streams; wire / target status and pin pairs) |
| `src/OepTarget.*`, `src/OepSwd.*`, `src/OepConsole.*`, `src/OepFixture.*`, `src/OepCapture.*`, `src/OepSampler.*`, `src/OepConfig.*` | the standard interfaces: wires and targets (`oep.wire.rvswd` / `swio` / `swd`, `oep.target.riscv-dm` / `arm-adi`), the console, fixtures (gpio / uart / capture), `oep.probe.config` (slots, binds, NVS storage; ESP32). Each file starts with the spec sections it follows |
| `src/OepP4I2cTarget.*`, `src/OepP4SpiTarget.*` | the custom interfaces `io.github.ch32-riscv-ug.esp32.i2c-target` / `spi-target` (revision 1, the ESP-IDF I2C / SPI slaves) |
| `src/OepCh32Dm.*`, `src/OepRvswdPhy.*`, `src/OepSwioPhy.*`, `src/OepDmConsole.*`, `src/OepPinTable.h`, `src/OepPlatform.h`, `src/OepFrame.*` and others | parts (the CH32 debug module, wire physical layers, console framings, the pin table and idle states, Arduino core differences, frames) |
| `examples/` | probe firmware: ESP32-P4 + CH32X035, classic ESP32 + CH32V003, RP2350 + CH32L103, RP2040 Zero, the P4 HS logic capture `Esp32P4CaptureProbe` (with `host/stream_test.py`) |
| `tests/host/` | host tests of the portable parts (the serial-port reader, the endpoint's sharing rules, binds): `tests/host/run.sh` (g++) |
| `tools/sync_registry.sh` | copies oep-spec's `generated/oep-v1/oep_v1_registry.h` to `src/OepRegistry.h` |
| `tools/bump_version.py`, `tools/sync_release_assets.py`, `.github/workflows/release.yml` | releases (arduino-library-release-toolkit's, used as is; not edited here) |
| `docs/` | dated work records (history) |

## Use

**Flash the firmware before using a probe** (you cannot tell what is on the board). For example, the X035 jig:

```sh
arduino-cli compile --clean examples/Esp32P4X035Probe
arduino-cli upload -p /run/board-identify/by-id/esp32-series-30eda0e31108 examples/Esp32P4X035Probe
```

The host is [oep-client-python](https://github.com/Open-Embedded-Probe/oep-client-python) (`pip install oep-client-python`,
`oep_client.link.open_host()`). A serial port (USB-Serial/JTAG, USB CDC, a UART bridge) carries OEP frames
(`0x00 <COBS> 0x00`) and its bind's raw bytes on one line. The host opens it exclusively (TIOCEXCL) and skips the bytes outside
frames as noise (oep-spec host guide §1.6, §2).

To carry a target's console on a serial port, register a slot and a bind (`oep.probe.config`, kept in NVS on ESP32):

```sh
oep config slot <probe> --name x035 --wire rvswd --pins 2,54 --attach at-boot --retry 1 --mechanism dmseq
oep config bind <probe> --port 1 --mode last-reset --stream slot:x035 --save
oep config show <probe>
```

## Known traps

- Arduino.h makes `word(...)` a macro for `makeWord(...)`: a lambda or function named `word` returns its argument.
- On the ESP32-P4, `pinMode` / `digitalWrite` on the same pins after `RvswdPhy::begin` takes them out of the dedicated GPIO
  bundle for good (a chip reset is needed).
- Sketches with a direct build (`build_opt.h`, EspUsbDevice's vendor written directly) are built with `--clean` after
  `build_opt.h` changes.
- Vendor bulk OUT is received a packet at a time (`CFG_TUD_VENDOR_RX_NEED_ZLP=0`). Receiving 16 KiB transfers ended by a
  ZLP (`=1`) delayed, on the P4, the completion of a request that ends on a 512-byte boundary (a 1024-byte write_block)
  until the next OUT: the probe did not answer for 3 s (2026-09-30, the X035 jig).

## Releases

The shared [arduino-library-release-toolkit](https://github.com/tanakamasayuki/arduino-library-release-toolkit) is used as is.
Record changes under `## Unreleased` in `CHANGELOG.md`, (EN) and (JA), and run the GitHub Actions workflow Release
(workflow_dispatch): it bumps the version in `library.properties`, writes `src/openembeddedprobe_version.h`, rewrites the
examples' `sketch.yaml` `dir: ../..` to `OpenEmbeddedProbe (<version>)` on the `release` branch, and makes the ZIP (without
`tests/`), the tag and the GitHub Release. Then the separate `.github/workflows/firmware.yml` (not part of the toolkit)
builds every example from the tag and attaches `<Example>-<version>.merged.bin` (ESP32, flash at 0x0) / `.uf2` (RP2040 /
RP2350) and `firmware-<version>.json` (sha256) to the Release. The probe reports the same version as its describe firmware
string.
