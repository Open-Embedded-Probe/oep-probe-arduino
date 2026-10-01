# OpenEmbeddedProbe (an Arduino library for OEP probes)

[日本語](README.ja.md)

## What is OEP?

Open Embedded Probe (OEP) is an open protocol between a **probe** - a small board wired to the chip you develop on - and the
**host** software on your PC. One probe can be a debugger (RVSWD / SWIO for WCH's CH32, SWD for ARM), a console to the target,
and a test fixture (GPIO, UART, SPI / I2C devices, logic capture) at once, and every host (a flash tool, an IDE monitor,
pytest) talks to it the same way.

**The protocol is fixed; what a probe can do is not.** OEP specifies how requests, results, the lock and discovery work, and
leaves the capabilities open:

- a probe **declares what it can do** - interfaces found by name, each with its pins and limits - so a host needs no table of
  boards and adapts to whatever the probe has;
- the standard interfaces (`oep.wire.*`, `oep.target.*`, `oep.fixture.*`, ...) are specified in oep-spec, and **anyone can add
  their own** under a reverse-DNS name of theirs (`io.github.<you>.<name>`, with its own revision) without asking anybody: a
  host that does not know it just does not use it. This library's ESP32 SPI / I2C devices are such extensions;
- a **session lock** keeps two programs from driving the probe at the same time;
- it runs over **USB vendor bulk, HID, USB CDC, USB-Serial/JTAG or a plain UART** bridge; a serial port carries the OEP frames
  and the target's console on the same line;
- the probe knows only its wires; **what a target is** (flash layout, loaders, how its debug line wants to be driven) **stays
  in the host**.

## Transports: why OEP runs over several kinds of link

OEP's frames are the same on every link; only the framing around them differs (COBS on serial ports, a length on USB bulk and
HID). A probe carries OEP on whichever links its chip has - several at once, sharing one session and lock - and the host picks
the fastest it finds. Each kind exists because some probe needs it:

| Link | What it is good for | Probe → host | Host → probe | Round trip | Chips in this library |
|---|---|---|---|---|---|
| **USB vendor bulk** (high speed) | Logic and analog capture streamed live, big flash images: the only link fast enough for a 2-channel 150 Msample/s logic capture (37.5 MB/s) | 33-37 MB/s | 5.5-10 MB/s | 0.37 ms | ESP32-P4 (HS port) |
| **USB HID** (vendor-defined reports) | No driver on any OS, opens from a browser (WebHID). The one class a **software (bit-banged) USB device** can offer: a low-speed device has no bulk endpoints, so a probe built on a chip without a USB peripheral can still be an OEP probe over HID | 0.8-1.1 MB/s (HS); a low-speed device about 8 KB/s | 1.0 MB/s (HS) | 0.66 ms | ESP32-P4 (HS port) |
| **USB CDC** (serial port) | Shows up as a COM port: the Arduino IDE's port and serial monitor work as they are, and the target's console shares the line with OEP | 8.1 MB/s (HS) | 6.7 MB/s (HS) | 0.60 ms | ESP32-P4 (HS port), RP2040 / RP2350 (full speed) |
| **USB-Serial/JTAG** (ESP32's built-in serial port) | Needs no USB stack in the firmware; the same port flashes the probe (esptool) | about 0.8 MB/s (full speed) | - | - | ESP32-P4 (FS port), ESP32-S3 / C3 / C6 |
| **UART** through a USB-UART bridge (CP2102, CH340, ...) | Any board with a UART and a bridge chip - most cheap boards - can be a probe; slow, but enough for flashing and debugging small targets | about 11 KB/s (115200 baud) | about 11 KB/s | about 5 ms | classic ESP32 |

Figures are what the bench measured (ESP32-P4 through usbipd, 2026-09-25 / 26; classic ESP32 at 115200 baud;
oep-spec docs/logic-capture.ja.md §2.7, docs/probe-cdc-and-persistence.ja.md §5.3 / §7). The low-speed HID figure is the
class's own ceiling (8-byte reports, one per millisecond), not a measurement.

**Why a probe needs its own USB identity**: a host finds OEP probes among all the USB devices without opening each one, and
the same probe is seen on every link it has (vendor bulk, HID, CDC) as one device with one serial number (= the probe's
`unit_id`). A serial port that the host cannot tell apart (a USB-UART bridge's, a USB-Serial/JTAG port, a CDC port with
another device's VID:PID) is different: the only way to know whether an OEP probe is behind it is to open it and see whether
OEP frames come back, and opening an arbitrary serial port can disturb whatever is on it (a board that resets on DTR, a modem,
another tool's device). A host cannot do that on its own for every port, so the user has to pick such a port explicitly. The
links above also need USB devices that are not serial ports - vendor bulk for speed, HID for software USB and
browsers - and those need a VID:PID of their own to be recognised. Until the project has a PID of its own, the reference firmware
uses `303a:0002` and hosts recognise the probe by an iProduct starting with `OEP` (oep-spec docs/usb-identity.ja.md).

A UART's 115200 baud is the one speed every board and bridge manages; a faster rate is not something a probe can assume
(some bridges and boards do not run 921600 reliably), so a host stays at 115200 unless the probe and the host agree on more.

This library turns an ESP32-P4, a classic ESP32, an RP2350 or an RP2040 into such a probe.

- Guides: [Getting started](docs/guide/getting-started.md) (flash, find, use a probe from Python), [Writing a probe](docs/guide/writing-a-probe.md)
  (the library from the inside, your own interfaces), [Boards](docs/guide/boards.md) (what each chip does, building for
  another board).
- Specification: [oep-spec](https://github.com/Open-Embedded-Probe/oep-spec) - start with [the review guide](https://github.com/Open-Embedded-Probe/oep-spec/blob/main/docs/review-guide.ja.md); the protocol
  core is [docs/oep-core.ja.md](https://github.com/Open-Embedded-Probe/oep-spec/blob/main/docs/oep-core.ja.md), the wire numbers [registry/oep-v1.toml](https://github.com/Open-Embedded-Probe/oep-spec/blob/main/registry/oep-v1.toml)
  (Japanese first; English follows once it settles).
- Host library: [oep-client-python](https://github.com/Open-Embedded-Probe/oep-client-python) (`pip install oep-client-python`, the
  `oep` command, a fake probe for tests).
- USB VID:PID: until the project has a PID of its own the reference firmware uses `303a:0002` with an iProduct starting `OEP`; who may use
  the OEP PID once granted is in [PID-USE.md](PID-USE.md).

## An example: an ESP32-P4 testing a CH32L103

<img src="docs/images/p4-ch32l103-bench.jpg" alt="An ESP32-P4 board wired to a CH32L103 board: RVSWD, UART, GPIO and the ADC inputs on jumper wires" width="420">

One ESP32-P4 board, wired to a CH32L103 on every pin you want to watch, is the whole test bench:

- **Write and debug the target** over RVSWD: flash it, halt / resume / step it, read and write its memory
  (`oep.wire.rvswd`, `oep.target.riscv-dm`). [ArduinoCore-CH32RV](https://github.com/ch32-riscv-ug/ArduinoCore-CH32RV) uploads
  through the probe from the Arduino IDE (an `oep://...` port).
- **Read its console** through the debug module (`oep.target.console`, no UART needed) or a UART (`oep.fixture.uart`).
- **Check what the target's code does with its pins**: the P4 answers as the device on the other end - an SPI device, an
  I2C device - so a test sees the bytes the target's driver really sent, and drives GPIOs and UARTs back
  (`oep.fixture.gpio` / `uart` / `spi-target` / `i2c-target`).
- **Capture up to 16 pins at the same time**, while it acts as that SPI device (`oep.fixture.logic`, the P4's PARLIO):

  | Channels | Sample rate |
  |---:|---:|
  | 2 | 160 Msps |
  | 8 | 40 Msps |
  | 16 | 20 Msps |

  Which pins are captured is chosen per request (the plan), so each test captures the pins it cares about - no rewiring and
  no reflashing between tests. The samples go to a sigrok session file (`LogicCapture.to_sr`, PulseView) or to a run
  recorder such as [WireSkein](https://github.com/Open-Embedded-Probe/wireskein), so tests can check signals, not only
  results.

Everything above is the host's choice at run time: the same firmware serves every test.

## Electrical notes

This library does not deal with voltage levels: the probe's pins are the MCU's own pins.

- The ESP32-P4 is a 3.3 V part. To connect it to a 5 V target, put a **bidirectional, fast level converter** in between.
- Lines that are only received (capture) can go through a **fast buffer** instead to meet a 5 V target.
- A **fast comparator** in front of an input makes the logic threshold adjustable. The ESP32-P4 has an adjustable LDO that
  can supply that threshold, so no other parts are needed for it.

## This library

A library for writing Open Embedded Probe (OEP) probes with Arduino, and the firmware of each probe (`examples/`). It speaks
the v1 protocol of [oep-spec](https://github.com/Open-Embedded-Probe/oep-spec) (`docs/oep-core.ja.md` and the standard
interfaces `docs/oep-if-*.ja.md`, a candidate being settled). This is an experimental stage: breaking changes are expected and
no compatibility is promised.

The wire numbers are defined only in oep-spec's `registry/oep-v1.toml`; its generated header is copied to
`src/OepRegistry.h`.

## Layout

| Path | Contents |
|---|---|
| `src/Oep.h`, `src/OepEndpoint.*`, `src/OepRegistry.h` | the core (oep-core): frames, interfaces by name, the lock, several transports (the describe transport list), serial ports shared by frames and raw bytes (core §3.4), the plan, notifications |
| `src/OepBind.*` | what each serial port carries (binds: last-reset / manual / mixed, held during a session and resumed from its last reset) |
| `src/OepStream.h`, `src/OepDebug.h` | parts the standard interfaces share (position streams; wire / target status and pin pairs) |
| `src/OepTarget.*`, `src/OepSwd.*`, `src/OepConsole.*`, `src/OepFixture.*`, `src/OepCapture.*`, `src/OepSampler.*`, `src/OepConfig.*` | the standard interfaces: wires and targets (`oep.wire.rvswd` / `swio` / `swd`, `oep.target.riscv-dm` / `arm-adi`), the console, fixtures (gpio / uart / capture), `oep.probe.config` (slots, binds, saved in NVS on ESP32 / flash on RP2040 / RP2350). Each file starts with the spec sections it follows |
| `src/OepP4I2cTarget.*`, `src/OepP4SpiTarget.*` | the standard interfaces `oep.fixture.i2c-target` / `spi-target` (revision 1) on the ESP-IDF I2C / SPI slaves (custom `io.github.ch32-riscv-ug.esp32.*` until 2026-09-30) |
| `src/OepCh32Dm.*`, `src/OepRvswdPhy.*`, `src/OepSwioPhy.*`, `src/OepDmConsole.*`, `src/OepPinTable.h`, `src/OepPlatform.h`, `src/OepFrame.*` and others | parts (the CH32 debug module, wire physical layers, console framings, the pin table and idle states, Arduino core differences, frames) |
| `examples/` | the board firmware and examples to learn from: see [Examples](#examples) |
| `tests/host/` | host tests of the portable parts (the serial-port reader, the endpoint's sharing rules, binds): `tests/host/run.sh` (g++) |
| `tools/sync_registry.sh` | copies oep-spec's `generated/oep-v1/oep_v1_registry.h` to `src/OepRegistry.h` |
| `tools/bump_version.py`, `tools/sync_release_assets.py`, `.github/workflows/release.yml` | releases (arduino-library-release-toolkit's, used as is; not edited here) |
| `docs/guide/` | the guides (English and Japanese): getting started, writing a probe, boards |
| `docs/` | the rest: the examples plan and dated work records (history) |

## Examples

Open them from `File > Examples > OpenEmbeddedProbe` in the Arduino IDE, or build one with its `sketch.yaml` profile
(`arduino-cli compile --profile <profile> <dir>`). Every sketch starts with a comment that explains it and shows how a host
uses it.

| Example | Boards (profiles) | What it shows |
|---|---|---|
| `Firmware/OepProbe` | Pico / Pico 2 and other RP2040 / RP2350 boards (rp2040, rp2350), ESP32-P4 (esp32p4), classic ESP32 (esp32) | **The firmware to flash** (also on the Releases): everything the chip can do, every pin chosen by the host, a jig = its settings |
| `01.Basics/MinimalProbe` | RP2040 / RP2350, classic ESP32 | the smallest probe: oep.core alone - the endpoint, a transport, the describe |
| `01.Basics/FixtureProbe` | RP2040 / RP2350, classic ESP32 | a test fixture: GPIO and a UART on pins the host plans (the pin table, owners, the plan) |
| `02.Interfaces/CustomInterface` | RP2040 / RP2350, classic ESP32 | **extending OEP**: your own interface under your own name - describe, the plan, ops, TLV tails |
| `03.Transports/MultipleTransports` | ESP32-P4 | one endpoint on four USB transports at once (HS vendor bulk, HID, CDC, USB-Serial/JTAG), USB identity |
| `04.Debug/RvswdDebugProbe` | RP2040 / RP2350, ESP32-P4 | a CH32 debugger on RVSWD: wire, riscv-dm, console |
| `04.Debug/SwioDebugProbe` | classic ESP32 | a CH32V00x debugger on the one-wire SWIO |
| `04.Debug/SwdDebugProbe` | RP2040 / RP2350 | an ARM debugger on SWD: wire, arm-adi |
| `05.Capture/LogicCapture` | ESP32-P4 | a logic analyzer at full speed (up to 16 channels, 160 Msps at 2), streaming over HS; `host/stream_test.py` |
| `06.Settings/ProbeConfig` | RP2040 / RP2350 | a jig that sets itself up at boot: slots, binds (the target's console on the probe's port), plans, idle states, saved in flash |
| `Tools/SwdPinSurvey` | RP2040 / RP2350 | a bring-up tool (text on Serial, not OEP): which pins are a debug port |

## Getting started

The short version; [the guide](docs/guide/getting-started.md) has more (GPIO / UART, debugging, capture, settings).

1. **Firmware.** Either take a built one from the [Releases](https://github.com/Open-Embedded-Probe/oep-probe-arduino/releases)
   (`OepProbe-rp2040-<version>.uf2` / `OepProbe-rp2350-<version>.uf2` for any RP2040 / RP2350 board - copy it to the
   board in BOOTSEL mode - and `OepProbe-esp32p4-<version>.merged.bin` / `OepProbe-esp32-<version>.merged.bin` for an ESP32-P4 / a classic ESP32;
   the pins are chosen by the host; `<Example>-<profile>-<version>.merged.bin` for ESP32:
   `esptool.py write_flash 0x0 <file>`; `firmware-<version>.json` has the sha256), or build an example yourself: install
   **OpenEmbeddedProbe** from the Arduino Library Manager, open `File > Examples > OpenEmbeddedProbe`, and build with its
   `sketch.yaml` profile (the core version and libraries pinned there):

   ```sh
   arduino-cli compile --clean --profile esp32p4 examples/Firmware/OepProbe
   arduino-cli upload -p <port> --profile esp32p4 examples/Firmware/OepProbe
   ```

   Flash the firmware before using a probe: you cannot tell what is on a board.
2. **Host.** `pip install oep-client-python`, then see what the probe offers:

   ```sh
   oep dump --port <port>
   ```

   In Python, `oep_client.link.open_host(<port>)` gives a host session. A serial port (USB-Serial/JTAG, USB CDC, a UART bridge)
   carries OEP frames (`0x00 <COBS> 0x00`) and its bind's raw bytes on one line; the host opens it exclusively (TIOCEXCL)
   and skips the bytes outside frames as noise (oep-spec host guide §1.6, §2).
3. **Settings.** To carry a target's console on a serial port from boot, register a slot and a bind (`oep.probe.config`,
   kept in NVS on ESP32, in flash on RP2040 / RP2350). The line's settings are the target's and come from the host (a CH32L103 wants SWCLK low while the
   line rests and at most 1 MHz right after a reset):

   ```sh
   oep config slot <probe> --name l103 --wire rvswd --pins 2,54 --attach at-boot --retry 1 --mechanism dmseq \
       --max-speed 1000000 --idle-clock low
   oep config bind <probe> --port 1 --mode last-reset --stream slot:l103 --save
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
builds every profile of every example from the tag and attaches `<Example>-<profile>-<version>.merged.bin` (ESP32, flash
at 0x0) / `.bin` (ESP32, the app image for an update, the P4's DFU) / `.uf2` (RP2040 / RP2350) of `examples/Firmware/` and `firmware-<version>.json` (sha256) to the Release (the other examples are built only
to check them). The probe reports the same version as its describe firmware
string.

### Release assets

`firmware-<version>.json` is this repository's own (OEP does not define it; oep-spec freeze decision 9):

```json
{"schema": 1, "library": "OpenEmbeddedProbe", "version": "0.0.20", "firmware": [
  {"example": "Firmware/OepProbe", "profile": "esp32p4", "file": "OepProbe-esp32p4-0.0.20.bin", "kind": "app",
   "model": "esp32p4", "fqbn": "esp32:esp32:esp32p4:...", "flash_offset": null, "sha256": "..."}]}
```

| Field | Meaning |
|---|---|
| `schema` | 1. A reader refuses another number; fields are only added |
| `kind` | `merged` (ESP32, the whole flash from `flash_offset` 0), `app` (ESP32, the app image: an update into the other app partition, the P4's DFU), `uf2` (RP2040 / RP2350) |
| `model` | the probe's describe model this image reports (`esp32p4`, `esp32`, `rp2040`, `rp2350`): the chip built for. The describe chip TLV is another thing: `<model> v<rev>` of the running part |
| `fqbn`, `flash_offset` | what it was built with; where a `merged` image goes (`null` for the others) |
| `sha256` | of the file |

The describe firmware string of every sketch built from this library is the library's release version (`0.0.20`,
`OPENEMBEDDEDPROBE_VERSION_STR`), the same as the release and `version` above; a build from a work tree reports the
version in its `library.properties`. OEP itself leaves the string free (core §7.5); this is this repository's rule.

## License

MIT ([LICENSE](LICENSE)); every source file says so (`SPDX-License-Identifier: MIT`). The USB VID:PID is not covered by
it: see [PID-USE.md](PID-USE.md).
