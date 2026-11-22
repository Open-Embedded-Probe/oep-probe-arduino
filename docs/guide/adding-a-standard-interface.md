# Adding a standard interface (for contributors to the library)

[日本語](adding-a-standard-interface.ja.md)

These are the steps to add an `oep.*` interface defined by oep-spec to this library and the released firmware. If you only
add your own interface (`io.github.<you>.*`) to your own probe, you do not need these steps. See
[Your first interface](your-first-interface.md) and [Firmware features](firmware-features.md).

Go from top to bottom. `oep.fixture.pwm` / `counter` / `dac` (`OepPwm.*`, `OepCounter.*`, `OepDac.*`, and their wiring in each
chip header) make good models.

## 1. Specification and numbers

- [ ] oep-spec has the interface's document (a section in `interfaces/oep-if-*.md`) and its registration in
      `registry/oep-v1.toml` (name, ops, TLV tags, enumerations). **The numbers on the wire are defined only in the
      registry.** The library writes no numbers and looks them up by name.
- [ ] Copy `src/OepRegistry.h` and `tests/vectors/` with `tools/sync_registry.sh` (`OEP_SPEC_REF` picks the commit).
      `tests/vectors/SPEC_COMMIT` is updated. Do not edit what is copied.

## 2. The class

- [ ] Create `src/Oep<Name>.h` / `.cpp` and derive from `oep::Interface`. The opening comment states:
  - the specification sections it follows
  - the list of ops (in the form `0x10 set(...) -> ...`)
  - implementation differences per chip
- [ ] Take the name, revision and ops from `reg::<interface>::kName` / `kRevision` / `kOp...`. describe's tags also come from
      the registry.
- [ ] `offers()` lists all the required ops in the table and the optional ops this implementation has. `lockFree()` is only for
      ops that change nothing (such as status).
- [ ] If it uses pins, do these two things.
  - Add an **owner number** to `oep::owners` in `src/OepPinTable.h`, and use `kOwnerId = owners::k<Name>` in the class.
    Implementations that are never in the same firmware together (such as the per-chip SPI targets) may share a number.
  - Write the four plan functions (`planRoles`, `planCheck`, `planApply`, `planRelease`) in the same form as the existing
    fixtures (such as `OepPwm.cpp`). Accept only the roles and pins declared in describe's `roleChannels`.
- [ ] In `sessionOver()`, undo everything the session made. Do not put waiting work in `handle()`; call `poll()` from `loop()`.
- [ ] Separate chip-dependent code with `#if defined(ARDUINO_ARCH_...)`. On chips without it, either do not offer the interface
      or declare nothing in describe. For host tests, provide `OEP_HOST_FAKE_<NAME>`, which swaps in a fake.

## 3. Size (CH32X035)

- [ ] It does not go into the X035 (62 KiB) by default. Wrap the whole `.cpp` in `#if !defined(ARDUINO_CH32X035)`.
      The library's `.cpp` files are linked, and merely referencing something from them pulls in the core's interrupt
      handlers and the like. In `src/firmware/Ch32.h`, put it in the defaults (`oep::kDefaults`) only when `kRoomy`.
- [ ] Check that the size of the X035 build has not changed (with `--clean`; a stale include cache can remain).

## 4. Host tests

- [ ] Write `tests/host/test_<name>.cpp` and add it to `tests/host/run.sh`. On the fake (`OEP_HOST_FAKE_<NAME>`), check the ops,
      the plan, the reasons for refusing, and the cleanup at the end of a session.
- [ ] All of `tests/host/run.sh` passes. `test_vectors.cpp` runs the specification's shared vectors against the endpoint.

## 5. Wiring it into the firmware

- [ ] Add the feature bit `k<Name>` (the next free number) to `src/OepFirmwareParts.h`.

In every chip header that has it (`src/firmware/Rp2.h`, `Esp32P4.h`, `Esp32.h`, `EspSoc.h`, `Ch32.h`), add:

- [ ] `#include <Oep<Name>.h>`
- [ ] `void oepAdd<Name>()`: `static oep::Slot<oep::<Class>> x; endpoint.add(x.make(pins));`. On chips short of static RAM, use
      `HeapSlot` (the classic ESP32's DRAM). Hand `loop()` work over with `loopWork.add(...)`. Do not make it `static`
      (`OepChoice.h`).
- [ ] Add `k<Name>` to `oep::kAvailable`, and if it is in by default, to `oep::kDefaults` too (on the `kRoomy` side for CH32).
- [ ] Add `if constexpr ((F & oep::k<Name>) != 0) oepAdd<Name>();` at the **end** of the sequence in `firmwareSetup()`. Inserting
      it in the middle shifts the existing fn numbers and the read-back of saved settings.

## 6. Build

- [ ] Build all profiles: `rp2040`, `rp2350`, `esp32p4`, `esp32p4x`, `esp32`, `esp32s3`, `ch32x035`, `ch32v307`, `ch32h417`. CI
      (`.github/workflows/`) also builds every `sketch.yaml` in examples.

## 7. Documents

- [ ] The table in [Writing a probe](writing-a-probe.md) §5 (English and Japanese)
- [ ] The tables in [Boards](boards.md) (English and Japanese): presence and limits per chip
- [ ] The list at the top of `src/OepFirmware.h`, the X table in [Firmware features](firmware-features.md)
- [ ] The README's layout table (English and Japanese), `CHANGELOG.md`

## 8. Host side and hardware

- [ ] Add the class, unit tests and hw tests to oep-client-python. If an hw test needs wiring, make it opt-in by label and skip
      when absent. Do not assume fixed wiring.
- [ ] Pass the hw tests on the bench and keep the results (`tests/hw/results`).
