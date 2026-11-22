# Contributing

[日本語](CONTRIBUTING.ja.md)

## Documents

Documents are changed in Japanese (`.ja.md`) and English (`.md`) in the same change. Until oep-spec's freeze (`v1.0.0`) the
Japanese is authoritative. Documents describe the current state only; the history of changes goes in CHANGELOG.md.

## Releases

For who is responsible for implementation, updates and regressions after updates, and the order of checks, see
[probe verification](docs/testing.md). Image generation and hardware quality checks are kept apart.

The shared mechanism of [arduino-library-release-toolkit](https://github.com/tanakamasayuki/arduino-library-release-toolkit) is
used as is. Add changes under `## Unreleased` in `CHANGELOG.md` with (EN) / (JA), and start the GitHub Actions Release
(workflow_dispatch). It bumps the version in `library.properties`, writes `src/openembeddedprobe_version.h`, rewrites the
examples' `sketch.yaml` `dir: ../..` to `OpenEmbeddedProbe (<version>)` on the `release` branch, and makes the ZIP without
`tests/`, the tag and the GitHub Release. Then the separate `.github/workflows/firmware.yml` (not the toolkit's) builds every
profile of every example from the tag, and attaches `<Example>-<profile>-<version>.merged.bin` (ESP32, written at 0x0) /
`.bin` (ESP32, the app image for updates; the P4's DFU) / `.uf2` (RP2040 / RP2350) / `.bin` (CH32, the whole code flash)
and `firmware-<version>.json` (sha256) to the Release (only those of `examples/Firmware/`; the other examples are built to
check them). The probe reports the same version in describe's firmware string.
