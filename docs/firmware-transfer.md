# Probe transfer with uv / pytest

[日本語](firmware-transfer.ja.md)

The Arduino side's Python environment lives in `tests/`. `pytest-embedded-arduino-cli` handles the build with a pinned profile and the Arduino CLI upload, and the OEP client checks the unit before the transfer and the communication after it. Dependencies are managed in `tests/pyproject.toml` and `tests/uv.lock`. The entry point follows the plugin's [first test guide](https://github.com/tanakamasayuki/pytest-embedded-arduino-cli/blob/main/FIRST_TEST.md).

## Minimal entry point

The hardware entry point is `examples/Firmware/OepProbe`, on a unit where OEP already runs. The profile and the target unit are given explicitly every time. The first write to a new or unresponsive unit is out of scope. There is no automatic transfer to all devices.

| profile | Transfer route | Target |
|---|---|---|
| `esp32` | the normal Arduino CLI upload | classic ESP32 UART bridge |
| `esp32p4` | dfu-util from the Arduino CLI upload recipe | P4 v0.1–v1.99, HS DFU |
| `esp32p4x` | the same. The P4X app image | P4 v3.1–v3.99, HS DFU |
| `rp2040` | from the Arduino CLI recipe, picotool / UF2 write to the selected BOOTSEL | Pico and others. Use the profile that matches the physical board |
| `rp2350` | the same | Raspberry Pi Pico 2 |

```sh
cd tests
uv sync --locked
cp .env.example .env
# edit the port, unit ID, candidate version, existing shared lock and this run's result directory in .env
uv run --env-file .env pytest -c pyproject.toml ../examples/Firmware/OepProbe/test_transfer.py \
  --profile esp32 --run-mode=all -vv --junitxml=build/transfer.xml
```

You need `arduino-cli` and a package index that resolves the profile. The profile pins the ESP32 core version in `sketch.yaml`. The Python plugin's `dut` fixture is not used; after the upload the OEP client owns the binary serial port. Do not open the same port in a text monitor at the same time.

`--run-mode=all` is build → unit / silicon / image check → upload → OEP check. `--run-mode=test` also **transfers** the existing build before checking. A build alone needs neither equipment settings nor hardware.

```sh
uv run pytest -c pyproject.toml ../examples/Firmware/OepProbe/test_transfer.py --profile esp32 --run-mode=build -vv
uv run pytest
```

The last command touches no hardware and runs the regression checks of the transfer decisions, the existing C++ host checks and the release profile checks. In build mode the hardware test shows as skipped after the build completes.

## Settings and exclusion

`.env` is outside Git and is read only by an explicit `uv run --env-file`. The template is `tests/.env.example`. Nothing depends on where a private equipment repository is.

| Setting | Meaning |
|---|---|
| `TEST_SERIAL_PORT` | the target unit's port. A generic by-id of a bridge without a serial number is ambiguous when several are connected, so pass the path that belongs to the unit |
| `OEP_TRANSFER_UNIT_ID` | the OEP unit ID checked before and after the transfer |
| `OEP_TRANSFER_FIRMWARE` | the declared version of the image transferred this time. Not a minimum version required of users |
| `OEP_HW_LOCK` | an existing lock file shared by every equipment user |
| `OEP_TRANSFER_RESULTS` | a new result directory for this run only. Refused if it exists |
| `OEP_TRANSFER_SPEC_COMMIT` | the SPEC commit the candidate implements. `unknown` if not settled |
| `OEP_TRANSFER_ALLOW_UNREADABLE_CONFIG` | only when `1`, a failure to read the settings and declarations of a known old image before the update is recorded and the run goes on. The criteria after the update are not relaxed |
| `OEP_HW_UF2_DRIVE` | the explicit path where the Pico's BOOTSEL is mounted. A drive that does not match the original USB connection point is not written |
| `OEP_TRANSFER_RESUME_FROM` | the previous `transfer.json` when this gate stopped in BOOTSEL after checking the unit. Pico family only |

After the build, the shared lock is taken without waiting and held from the unit check through upload, reconnection, checks and saving the results. The shared lock provides this gate's equipment exclusion. It uses the plugin's build and its public flasher fixture, and passes a recipe limited to the target through the upload fixture. If another process holds the shared lock, the run fails before the transfer. If the port is also passed with `--port`, it is refused unless it is the same device as in `.env`.

This entry point handles one probe only and does not read the wiring or target assignments in the equipment TOML. Only the needed settings are given explicitly, and unconfirmed wiring is not guessed and driven.

## What is checked and the end state

It uses the normal Arduino CLI upload, with no flash backup and no restore of the old firmware. The transferred candidate stays and is used. The normal upload does not erase the LittleFS area that holds the settings and the memo area, but whether the saved settings survive is judged by measurement after the transfer.

`transfer.json` records the source commit/dirty, the SPEC commit, the build image's SHA-256, the unit, the versions before and after the transfer, the declarations, the saved settings and the memo area, the settings state, and the verdict. A failure during upload also stays as a pytest setup error and in the log. The results contain equipment information, so use a destination outside Git.

This entry point checks identity, confirm/list/describe and the required declarations, the boot ID change, that the saved settings and the memo area match, and that the saved settings are applied. It does not count as passing DUT flash/debug, fixture/capture accuracy, or a full regression of the Arduino Core's upload/monitor. Those are added as separate contracts, following [the probe verification policy](testing.md).

## P4 / P4X and re-enumeration

Old firmware shows v3.x silicon as `esp32p4` / ESP32-P4 too. Before the update, the real revision is checked with `chip`, and the image's custom descriptor, supported revision range, and the ELF's model/product are checked. After the update, `esp32p4x` / ESP32-P4X is required. A P4 image is never tried on a P4X.

DFU names the USB serial and the DFU interface that match the unit ID, and writes only the app image. After the restart, even if the product name or the tty number changes, the CDC port is found again uniquely from the USB serial. There is no fallback to another unit.

When dfu-util returns `LIBUSB_ERROR_NO_DEVICE` / exit 74 right after recording the download complete and the manifest in the success state, it is taken as the disconnection from the restart and **the verdict is deferred to the later stages**. It is PASS only once the candidate's unit, version, model, boot ID, declarations and settings are actually confirmed. A failure in the middle of the transfer or a general exit error is never treated as success.

The upload recipe is overridden with Arduino CLI's [upload-property](https://docs.arduino.cc/arduino-cli/commands-reference/arduino-cli_upload). It sets `upload.pattern`, which applies after the tool is chosen.

## Pico BOOTSEL and resuming after a failure

Before the update, the unit ID and the USB bus / port path are recorded, and the confirmed CDC port is switched to BOOTSEL with 1200 baud. Only the expected RP2040 / RP2350 bootloader at the same USB connection point is taken. Several candidates or another connection point are refused.

picotool, when used, is the one shipped with the core, called with bus/address. An ordinary user needs access rights to the USB bootloader. When the explicit `OEP_HW_UF2_DRIVE` is used, INFO_UF2 and the mount's sysfs origin are checked and only that one drive is written. The core's search of all UF2 drives is not used. It waits the short delay until automount sets the user ACL.

If it stopped in BOOTSEL, name the `transfer.json` left in the results with `OEP_TRANSFER_RESUME_FROM`. The next results are saved to a new directory. The previous unit and profile are checked against the current bootloader's connection point, and the candidate is transferred again before the checks. This assumes the unit checked last time has not been swapped. An arbitrary bootloader is never chosen "because there is only one".

When the old image's settings cannot be read, or the storage after the update is unreadable, keeping and applying the saved settings cannot be guaranteed. `settings_preservation` is recorded as unknown, and the reads and declaration checks after the update still run.

The sketch of the transfer command is outside `tests/`, so `-c pyproject.toml` names the test workspace's pytest settings explicitly. This keeps imports such as `probe_checks` and the plugin settings stable. With picotool, leave `OEP_HW_UF2_DRIVE` unset or empty, and give a new result directory every time. Arduino CLI's `--port UF2_Board` is a placeholder of the upload recipe; check the actual transfer route in `transfer.json`'s `upload_command` and in `upload.log`.
