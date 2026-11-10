# Choosing the firmware's features, adding your own

[日本語](firmware-features.ja.md)

The released firmware is a library part, `OepFirmware.h`. By default it has everything its chip can do. A sketch only
calls `begin()`, and a newer library brings a newer firmware with it. This guide covers two things:

- leaving out what you do not need, or building from only what you do (to fit a small chip, or not to offer what you
  do not use)
- adding an interface of your own (the rest of the firmware as it is)

## 1. The shortest sketch

```cpp
#include <OepFirmware.h>

void setup() { oep::firmware.begin(); }   // every default feature of the chip

void loop() { oep::firmware.loop(); }
```

The released [OepProbe](../../examples/Firmware/OepProbe/OepProbe.ino) is this sketch.

- Include `OepFirmware.h` from the sketch's `.ino` alone. Included from two files, the link stops. Other files include
  `OepFirmwareParts.h` (the feature names, and `add` / `onSetup` / `onLoop` below).
- On an ESP32-P4 / P4X, the sketch's `sketch.yaml` takes the EspUsbDevice library (as the released OepProbe's profiles
  do).

## 2. Choosing the features

The features are F of `begin<F>()`, a compile-time constant. A feature F leaves out links none of its code.

```cpp
oep::firmware.begin<oep::kDefaults & ~oep::kSwd & ~oep::kPwm>();          // the defaults but SWD and PWM
oep::firmware.begin<oep::kDefaults | oep::kCdc>();                         // the defaults and USB CDC, off by default
oep::firmware.begin<oep::kNone | oep::kBulk | oep::kRvswd | oep::kConsole>(); // nothing but these
```

| Name | What |
|---|---|
| `kBulk`, `kHid`, `kCdc`, `kUsj`, `kBridge`, `kWifi` | transports: USB vendor bulk, HID, CDC, USB-Serial/JTAG, a UART bridge, TCP over Wi-Fi |
| `kDfu`, `kPortSpeed` | the ESP32-P4's DFU (outside OEP), the UART bridge's port_speed |
| `kRvswd`, `kSwio`, `kSwd` | the debug wires (and their targets) |
| `kConsole`, `kSwioConsole` | the target consoles (the RVSWD wire's, the SWIO wire's) |
| `kGpio`, `kUart`, `kI2cTarget`, `kSpiTarget` | fixtures: GPIO, UART, I2C / SPI targets |
| `kI2cController`, `kSpiController`, `kPwm`, `kCounter`, `kDac` | fixtures: I2C / SPI controllers, PWM, edge counting, DAC |
| `kCapture`, `kLogic`, `kAnalog`, `kGroup` | capture: logic (`kCapture` on the ESP32-P4 / classic ESP32, `kLogic` on the RP2 / CH32), analog, a group started together |
| `kSettings`, `kLink`, `kMemo` | the core's settings (saved), `oep.probe.link` (testing the lines), `oep.probe.memo` |

- `oep::kDefaults` is the released firmware's set, `oep::kAvailable` what the chip has. Each chip's are in
  [boards](boards.md).
- These stop at compile time, saying why: a feature the chip does not have, no transport at all, `kConsole` without
  `kRvswd`, `kSwioConsole` without `kSwio`, `kPortSpeed` without `kBridge`.
- Leaving one out renumbers the fns of the interfaces after it. A host finds them by name, so that is fine, but saved
  settings read back only with the interfaces in the order they were saved with: set them again after leaving one out.

## 3. Values at run time

The names can be changed with `FirmwareConfig`; what it leaves out is the released firmware's.

```cpp
oep::FirmwareConfig cfg;
cfg.model = "my-jig";              // fn 0's describe model
cfg.product = "My jig";            // the USB product name (for people; no host tells probes apart by it)
oep::firmware.begin(cfg);          // begin<F>(cfg) alike
```

## 4. Adding your own interface

Register it before `begin()`, from any of the sketch's files (`OepFirmwareParts.h`).

```cpp
#include <OepFirmware.h>

class Hello final : public oep::Interface { /* Hello of your first interface */ };
static Hello hello;

void setup() {
  oep::firmware.onSetup([](oep::Endpoint &endpoint, oep::PinTable &pins) {
    pins.claim(4, oep::owners::kSketch);   // a pin of your own
  });
  oep::firmware.add(hello);                // after the firmware's parts
  oep::firmware.onLoop([] { /* any work it moves on in loop() */ });
  oep::firmware.begin();
}

void loop() { oep::firmware.loop(); }
```

- The interfaces added go in after all of the firmware's parts, in the order registered; the saved settings are applied
  after them.
- `onSetup` is called in `begin()`, right before your interfaces are added. Take pins from the `pins` it is handed,
  with owner numbers from `oep::owners::kSketch` (0x80) up.
- `onLoop` is called every `loop()`, after the firmware's own work. It must not wait.
- Each registration takes a few bytes of heap (no limit on how many).
- How to write an interface: [your first interface](your-first-interface.md).

Example: [extras/LibraryTest](../../extras/LibraryTest/README.md) runs the calls of a library under test on the probe
and places its log (`OepProbeLog.h`: written to RAM with the probe's clock) on the capture's waveform.

## 5. Building it

- **Arduino IDE**: write the sketch above in a new sketch (or start from `File > Examples > OpenEmbeddedProbe >
  Firmware > OepProbe`).
- **arduino-cli**: copy the released `OepProbe`'s `sketch.yaml` into your sketch and point its `libraries: - dir:` at
  this library (it reads `../../..`), then run `arduino-cli compile --profile <profile> <folder>`.
- Mind the size: the CH32X035's 62 KiB are nearly full with the defaults. Leave something out to make room.

## 6. Checking it

`oep dump --port <port>` shows the interface you added in the list, and the parts you left out gone.
