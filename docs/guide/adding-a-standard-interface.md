# Adding a standard interface (for contributors)

[日本語](adding-a-standard-interface.ja.md)

How to add an `oep.*` interface that oep-spec defines to this library and the released firmware. To add an interface of
your own (`io.github.<you>.*`) to your own probe you need none of this: see [your first interface](your-first-interface.md)
and [firmware features](firmware-features.md).

Work from the top down. A recent example to follow is `oep.fixture.pwm` / `counter` / `dac` (commit 9317805).

## 1. The spec and its numbers

- [ ] oep-spec has the interface's document (a section of `interfaces/oep-if-*.md`) and its entries in `registry/oep-v1.toml`
      (the name, ops, TLV tags, enums). **The wire numbers are defined only in the registry**: the library writes no
      number, it takes them by name.
- [ ] Copy `src/OepRegistry.h` and `tests/vectors/` with `tools/sync_registry.sh` (`OEP_SPEC_REF` picks the commit);
      `tests/vectors/SPEC_COMMIT` is updated. The copies are never edited.

## 2. The class

- [ ] Make `src/Oep<Name>.h` / `.cpp`, deriving from `oep::Interface`. Start it with a comment that gives:
  - the spec sections it follows
  - the ops (in the form `0x10 set(...) -> ...`)
  - how each chip implements it
  - anything not yet run on hardware (e.g. `NOT YET RUN ON HARDWARE`)
- [ ] Take the name, revision and ops from `reg::<interface>::kName` / `kRevision` / `kOp...`, and the describe tags from the
      registry too.
- [ ] `offers()` lists every required op of the table and the optional ones this implementation has. `lockFree()` only for
      ops that change nothing (status and the like).
- [ ] If it takes pins, do two things:
  - Add its **owner id** to `oep::owners` in `src/OepPinTable.h`, and use `kOwnerId = owners::k<Name>` in the class.
    Implementations that never share a firmware (the per-chip SPI targets) may share one id.
  - Write the plan's four functions (`planRoles`, `planCheck`, `planApply`, `planRelease`) the way the existing fixtures do
    (`OepPwm.cpp` and others), taking only the roles and pins its describe declares with `roleChannels`.
- [ ] `sessionOver()` puts back everything the session made. Nothing waits in `handle()`; give it a `poll()` called from
      `loop()`.
- [ ] Split chip-specific code with `#if defined(ARDUINO_ARCH_...)`; on a chip without it, do not add the interface, or
      declare nothing in its describe. Give the host tests a fake behind `OEP_HOST_FAKE_<NAME>`.

## 3. Size (the CH32X035)

- [ ] Not on the X035 (62 KiB) by default: wrap the whole `.cpp` in `#if !defined(ARDUINO_CH32X035)` (a library `.cpp` is
      linked, and merely referencing the core's interrupt handlers from it pulls them in), and add it only `if (kRoomy)` in
      `Ch32.h`.
- [ ] Check the X035 build's size has not changed (with `--clean`: a stale include cache can linger).

## 4. Host tests

- [ ] Write `tests/host/test_<name>.cpp` and add it to `tests/host/run.sh`: the ops, the plan, the reasons it refuses, the
      clean-up at the session's end, on the fake.
- [ ] `tests/host/run.sh` passes. `test_vectors.cpp` runs the spec's shared vectors against the endpoint.

## 5. Wiring it into the firmware

In every chip header that has it (`examples/Firmware/OepProbe/Rp2.h`, `Esp32P4.h`, `Esp32.h`, `Ch32.h`):

- [ ] `#include <Oep<Name>.h>`
- [ ] `void oepAdd<Name>()`: `static oep::Slot<oep::<Class>> x; endpoint.add(x.make(pins));` (`HeapSlot` on a chip short of
      static RAM: the classic ESP32's DRAM); loop work goes through `loopWork.add(...)`.
- [ ] `__attribute__((weak)) void oepWith<Name>() { oepAdd<Name>(); }` (`{}` if it is off by default; `if (kRoomy)` on the CH32)
- [ ] `oepWith<Name>();` **last** in `setup()`'s list of `oepWith...()`: anywhere else renumbers the existing fns and breaks
      reading back saved settings.
- [ ] Declare `oepWith<Name>()` and `oepAdd<Name>()` in `OepFeatures.h`.

## 6. Building

- [ ] Build every profile: `rp2040`, `rp2350`, `esp32p4`, `esp32p4x`, `esp32`, `ch32x035`, `ch32v307`, `ch32h417`. CI
      (`.github/workflows/`) builds every example's `sketch.yaml` too.

## 7. Documents

- [ ] The table in [writing a probe](writing-a-probe.md) §5 (English and Japanese)
- [ ] The table in [boards](boards.md) (both): which chips have it, their limits, whether it was checked on hardware
- [ ] The list at the top of `examples/Firmware/OepProbe/OepProbe.ino`, the X table in [firmware features](firmware-features.md)
- [ ] The README layout table (both), `CHANGELOG.md`

## 8. Hosts and hardware

- [ ] Add a class, unit tests and a hardware test to oep-client-python. A hardware test that needs wiring opts in by
      labels and skips without them, never assuming fixed wiring. Add it to oep-client-js too.
- [ ] Pass the hardware test on the bench and keep its results (`tests/hw/results`). What cannot be checked is said to be
      unchecked at the top of the class and in the boards table.
