# Choosing the firmware's features, adding your own

[日本語](firmware-features.ja.md)

The released firmware is the library part `OepFirmware.h`. By default it has everything the chip can do. The sketch just calls
`begin()`, and updating the library version also updates the firmware itself. This guide explains two things.

- Leaving out features you do not need, or building with only what you need (to fit a small chip, to not offer what is unused)
- Adding your own interface (the rest of the firmware stays as it is)

## 1. The shortest sketch

```cpp
#include <OepFirmware.h>

void setup() { oep::firmware.begin(); }   // all of the chip's default features

void loop() { oep::firmware.loop(); }
```

The released [OepProbe](../../examples/Firmware/OepProbe/OepProbe.ino) has this form too.

- Include `OepFirmware.h` in one place only, in the sketch's `.ino`. Including it in two files stops at link time. From other
  files, include `OepFirmwareParts.h` (the feature names and the `add` / `onSetup` / `onLoop` below).
- On ESP32-P4 / P4X, put the EspUsbDevice library in the sketch's `sketch.yaml` (the same as the released OepProbe's profile).

## 2. Choosing features

Features are chosen with the F of `begin<F>()` (a compile-time constant). Features not in F link no code at all.

```cpp
oep::firmware.begin<oep::kDefaults & ~oep::kSwd & ~oep::kPwm>();          // the defaults without SWD and PWM
oep::firmware.begin<oep::kDefaults | oep::kCdc>();                         // on CH32, add USB CDC, which is off by default
oep::firmware.begin<oep::kNone | oep::kBulk | oep::kRvswd | oep::kConsole>(); // from nothing, only these
```

| Name | Contents |
|---|---|
| `kBulk`, `kHid`, `kCdc`, `kUsj`, `kBridge`, `kWifi` | transports: USB vendor bulk, HID, CDC, USB-Serial/JTAG, UART bridge, TCP over Wi-Fi |
| `kDfu`, `kPortSpeed` | the ESP32-P4's DFU (outside OEP), the UART bridge's port_speed |
| `kRvswd`, `kSwio`, `kSwd` | debug wires (and their targets) |
| `kConsole`, `kSwioConsole` | the target console (the RVSWD wire's, the SWIO wire's) |
| `kGpio`, `kUart`, `kI2cTarget`, `kSpiTarget` | fixtures: GPIO, UART, I2C / SPI target |
| `kI2cController`, `kSpiController`, `kPwm`, `kCounter`, `kDac` | fixtures: I2C / SPI controller, PWM, edge counting, DAC |
| `kCapture`, `kLogic`, `kAnalog`, `kGroup` | capture: logic (`kCapture` on ESP32-P4 / classic ESP32, `kLogic` on RP2 / CH32), analog, the group that starts together |
| `kSettings`, `kLink`, `kMemo` | the core's settings (saving), `oep.probe.link` (the wire test), `oep.probe.memo` |

- `oep::kDefaults` is the released firmware's features, and `oep::kAvailable` is what the chip has. The contents per chip are in
  [Boards](boards.md).
- These combinations stop at compile time with a reason: a feature the chip does not have, no transport at all, `kConsole`
  without `kRvswd`, `kSwioConsole` without `kSwio`, `kPortSpeed` without `kBridge`.
- Leaving something out changes the fn numbers of the interfaces after it. The host finds interfaces by name, so that is no
  problem, but saved settings can be read back only with that order of interfaces. After leaving something out, enter the
  settings again.

## 3. Run-time values

Names can be changed with `FirmwareConfig`. What you do not write keeps the released firmware's value.

```cpp
oep::FirmwareConfig cfg;
cfg.model = "my-jig";              // fn 0's describe model
cfg.product = "My jig";            // the USB product name (a name for people; the host does not tell probes apart by it)
oep::firmware.begin(cfg);          // begin<F>(cfg) works the same
```

## 4. Adding your own interface

Register it before `begin()`. It can be called from any file (`OepFirmwareParts.h`).

```cpp
#include <OepFirmware.h>

class Hello final : public oep::Interface { /* the Hello of your first interface */ };
static Hello hello;

void setup() {
  oep::firmware.onSetup([](oep::Endpoint &endpoint, oep::PinTable &pins) {
    pins.claim(4, oep::owners::kSketch);   // pins you use
  });
  oep::firmware.add(hello);                // added after the firmware's parts
  oep::firmware.onLoop([] { /* work to advance in loop(), if any */ });
  oep::firmware.begin();
}

void loop() { oep::firmware.loop(); }
```

- Interfaces you `add` are added after all of the firmware's parts, in the order registered. Saved settings are applied after
  that.
- `onSetup` is called inside `begin()`, right before your interfaces are added. Take pins from the `pins` passed in, and use
  owner numbers from `oep::owners::kSketch` (0x80) up.
- `onLoop` is called on every `loop()`, after the firmware's own work. Do not wait in it.
- Registration takes a few bytes each from the heap (there is no limit on the count).
- For how to write an interface, see [Your first interface](your-first-interface.md).

Example: [extras/LibraryTest](../../extras/LibraryTest/README.md) runs calls of a library you want to test on the probe, and ties
their log (`OepProbeLog.h`: written to RAM and stamped with the probe's clock) to the captured waveforms.

## 5. Building

- **Arduino IDE**: write the form above in a new sketch (starting from `File > Examples > OpenEmbeddedProbe > Firmware > OepProbe`
  is the same).
- **arduino-cli**: copy the released `OepProbe`'s `sketch.yaml` to your sketch and change `libraries: - dir:` to this library's
  location (originally `../../..`). Then run `arduino-cli compile --profile <profile> <folder>`.
- Mind the size. The 62 KiB of the CH32X035 is nearly full with the default configuration. Leave something out to make room.

## 6. Checking

With `oep dump --port <port>`, check that the interfaces you added are in list and that the parts you left out are gone.
