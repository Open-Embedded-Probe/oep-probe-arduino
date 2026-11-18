# Verifying the probe implementation and updates

[日本語](testing.ja.md)

This repository verifies and judges the probe firmware's implementation quality, its images, updates, the recovery after an update, and the effect on the routes that use it. The responsibility does not move even when test code lives on the Python client side.

For what is guaranteed in common and how it is judged, see [the OEP-wide test policy](https://github.com/Open-Embedded-Probe/oep-client-python/blob/main/docs/testing-policy.md); for the responsibilities of projects that use installed probes, see [the guide for consuming projects](https://github.com/Open-Embedded-Probe/oep-client-python/blob/main/docs/testing-consumers.md). oep-spec owns the communication norms, and the equipment settings TOML is not part of the wire protocol.

The Arduino run environment lives in the uv / pytest workspace in `tests/`. See [the hardware transfer entry point](firmware-transfer.md) and `tests/.env.example`. Normal operation transfers the candidate and then uses it, with no flash backup or restore of the old firmware assumed.

## Entry points and what they guarantee

| Entry point | What it guarantees |
|---|---|
| `cd tests && uv run pytest` | transfer decisions, existing host checks, release profile checks. Touches no hardware |
| `uv run --env-file .env pytest -c pyproject.toml ../examples/Firmware/OepProbe/test_transfer.py --profile esp32` | for the chosen ESP32 / P4 / P4X / Pico: build, unit / image check, transfer, re-enumeration, re-check of the declarations, settings and memo area |
| `tests/host/run.sh` | portable protocol, state and resource checks on the host with a shim and C++ |
| `uv run --project tests --env-file tests/.env pytest tests/equipment` ([hardware conformance checks](../tests/equipment/README.md)) | checks of the core and the common declarations of the hardware's firmware against a chosen oep-spec checkout. No writing and no settings changes |
| `cd tests && uv run pytest oneshot/<name> --profile <profile> --port <port>` ("One-off tests" below) | hardware tests that run only with specific equipment and wiring. The plugin builds, writes from that port inside the device lock, then checks. Not collected automatically |
| `python3 tests/compile/test_profiles.py` | regression checks of the profile/model definitions |
| `.github/workflows/tests.yml` | host checks and builds for P4/P4X and others. Separate from guaranteeing the hardware's electrical behavior |
| the Python client's `tests/hw` | integration checks on hardware by an independent host. The constraints of the current setup are in its README |
| `.github/workflows/firmware.yml` | generating the images and the manifest/hash. Booting and recovery on hardware are checked separately |

The new equipment format comes with an offline loader/planner, a virtual smoke test, and a hardware probe preflight with the shared lock and result saving; try it from [the minimal setup check](https://github.com/Open-Embedded-Probe/oep-client-python/blob/main/docs/hardware-quickstart.md). A passing probe preflight does not certify conformance of wire/DM, fixtures or USB pairs.

## One-off tests

Hardware tests that run only with specific equipment or wiring go one per folder in `tests/oneshot/<name>/`. They are not in
the testpaths of `tests/pyproject.toml`, so `uv run pytest` does not collect them. To run one, name the folder and pass the
equipment's port with `--port`. Build, writing and the device lock follow the defaults of the pytest-embedded-arduino-cli
plugin.

- The folder is the plugin's sketch folder. To test the released firmware, the `.ino` holds only
  `#include <../examples/Firmware/OepProbe/OepProbe.ino>` (the library's src is on the include path, and the headers inside
  the firmware are found at their original place). `sketch.yaml` holds only the needed profiles, the same as the released firmware.
- `build_opt.h`, which cannot be included, is the same as the released firmware's (not present if the firmware has none). `tests/unit/test_oneshot_sketches.py`
  checks that they are the same.
- The top of the test file states the equipment, the wiring and how to run it.

| Folder | What it checks | Equipment |
|---|---|---|
| `p4_uart_bridge` | writing through the ESP32-P4 / P4X UART bridge (GPIO37 / 38), OEP answering on the same port, and those 2 pins not offered to fixtures | a P4 board with a USB-UART on GPIO37 / 38 |
| `esp32_uart_bridge` | writing through the classic ESP32 UART bridge (UART0), OEP answering on the same port, and the Wi-Fi TCP listener being created at boot and listed among the transports | a classic ESP32 board with a USB-UART on UART0 |

## Order of verifying candidate firmware

1. Choose the contracts the change affects and the platform/profile/transport differences, and state what is required and what is out of scope.
2. Pass the unit tests, generation consistency and build, and record the candidate image's source commit/dirty, SPEC version, profile and hash.
3. Check the equipment settings, the unit, power and wiring, and take the shared lock. Record the firmware and saved settings before the change and the policy for the end state after the update.
4. Update, naming the image and the update route explicitly. After reconnecting, check identity, the declared version, revision, ops/limits, settings (including pass-through) and the memo area.
5. Check protocol/session/config, transport, wire/DM, fixture/capture and the hardware contracts related to the change. A missing declaration is never turned into a skip for missing equipment.
6. Use the existing test entry points of the affected consuming projects to regression-check flash/debug, upload/monitor, DUT behavior and so on. The consumer's expectations are not relaxed through equipment settings.
7. Save the update and regression results, the untested axes, the cleanup and the remaining image/settings, and decide on release within the range where the required contracts hold.

The provider manages the probe's version and does not uniformly require users to set a minimum firmware version or update automatically. The provider explains the SPEC it implements, the declared features, compatibility and known defects. When the configuration changes after an update, recheck the needed wiring and capabilities and update the equipment's confirmed record.

## Hardware contracts to add

Besides passing the shared vectors and mocks, check the real transport's boundaries, disconnection and re-enumeration, the wire candidate search and its ambiguity, keeping existing connections, fixture/capture against an independent reference, and releasing drive and restoring settings after a failure.

On a platform that takes several targets, implement and declare the connection/console/config limits, and check that A/B reset, operations and streams on the same probe do not cross. A platform with a limit of 1 is not assigned to a contract that needs simultaneous connections. The target's USB data and the probe's own USB transport are checked as separate contracts.

## Host checks that depend on real time

`tests/host/test_wire_gate.cpp` makes and captures short real-time waveforms in 2 host threads. It depends on the host's load, so
`tests/host/run.sh` tries it up to 3 times and fails only when all 3 fail. In that case, run it again alone when the load is low.
Passing here does not guarantee the hardware's timing accuracy.
