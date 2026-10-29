# Choosing the firmware's features, adding your own

[日本語](firmware-features.ja.md)

The released firmware (`examples/Firmware/OepProbe`) has everything its chip can do by default. This guide covers two things:

- leaving out what you do not need (to fit a small chip, or not to offer what you do not use)
- adding an interface of your own (the rest of the firmware as it is)

Both take one `.cpp` of your own in the `OepProbe` folder; no file of the library's is edited.

## 1. How it works

Each part of the firmware (a transport or an interface) is a choice:

```cpp
// in the chip's header (Rp2.h, ...)
void oepAddPwm() { static oep::Slot<oep::FixturePwm> pwm; endpoint.add(pwm.make(pins)); }   // make the part, add it
__attribute__((weak)) void oepWithPwm() { oepAddPwm(); }                                       // the default: add it
```

- Only `oepAddX()` names part X. If it is never called, none of X's code is linked.
- `oepWithX()` is weak: a function of the same name in the sketch is used instead.

The list is in [OepFeatures.h](../../examples/Firmware/OepProbe/OepFeatures.h); what each chip has by default is at the top
of [OepProbe.ino](../../examples/Firmware/OepProbe/OepProbe.ino) and in [boards](boards.md).

## 2. Leaving out, adding

Put a file, say `MyChoices.cpp`, in the `OepProbe` folder:

```cpp
#include "OepFeatures.h"

void oepWithSwd() {}                 // no oep.wire.swd / oep.target.arm-adi
void oepWithPwm() {}                 // no oep.fixture.pwm
void oepWithCdc() { oepAddCdc(); }   // a USB CDC port, off by default (where the chip has oepAddCdc)
```

| X | What |
|---|---|
| `Bulk`, `Hid`, `Cdc`, `Usj`, `Bridge`, `Wifi` | transports: USB vendor bulk, HID, CDC, USB-Serial/JTAG, a UART bridge, TCP over Wi-Fi |
| `Dfu`, `PortSpeed` | the ESP32-P4's DFU (outside OEP), fn 0's port_speed |
| `Rvswd`, `Swio`, `Swd` | debug wires (and their targets) |
| `Console`, `SwioConsole` | the target console (the RVSWD wire's, the SWIO wire's) |
| `Gpio`, `Uart`, `I2c`, `Spi` | fixtures: GPIO, UART, I2C / SPI targets |
| `I2cController`, `SpiController`, `Pwm`, `Counter`, `Dac` | fixtures: I2C / SPI controllers, PWM, edge counting, DAC |
| `Capture`, `Logic`, `Analog`, `Group` | captures: logic (P4 / classic ESP32 / RP2 / CH32), analog, a group started together |
| `Settings`, `Link` | the core's settings (saved), `oep.probe.link` (the link test) |

- An X whose `oepAddX` the chip's header does not have cannot be added on that chip (the link fails).
- Leaving one out renumbers the fns after it. Hosts find them by name, so that is fine, but saved settings read back only
  for the same list of interfaces: put the settings in again after a change.

## 3. Adding your own interface

Define `oepUserSetup` and `oepUserLoop` in the same `.cpp`. Left undefined, they do nothing.

```cpp
#include <OepEndpoint.h>
#include <OepPinTable.h>
#include "OepFeatures.h"

class Hello final : public oep::Interface { /* the Hello of "your first interface" */ };
static Hello hello;

void oepUserSetup(oep::Endpoint &endpoint, oep::PinTable &pins) {
  endpoint.add(hello);   // after the firmware's parts
}
void oepUserLoop() {
  // hello's loop() work, if it has any (never waiting)
}
```

- `oepUserSetup` is called once in `setup()`, after the firmware has added all its parts; the saved settings are applied
  after it.
- `oepUserLoop` is called every `loop()`, after the firmware's own work.
- Take pins from the `pins` you are given, under owner ids `oep::owners::kSketch` (0x80) and up, so they never clash with
  the firmware's parts.
- How to write the interface: [your first interface](your-first-interface.md).

An example: [extras/LibraryTest](../../extras/LibraryTest/README.md) runs a library's calls on the probe and lines its log
(`OepProbeLog.h`: written to RAM with the probe's clock) up with the capture's wires.

## 4. Building it

- **Arduino IDE**: open `File > Examples > OpenEmbeddedProbe > Firmware > OepProbe`, save it under a name of yours, then
  add your `.cpp` in a new tab (the saved sketch uses the installed library).
- **arduino-cli**: copy the `OepProbe` folder, add your `.cpp`, point the copied `sketch.yaml`'s `libraries: - dir:` at this
  library (it is `../../..`), then `arduino-cli compile --profile <profile> <folder>`. On the ESP32-P4 add `--clean`
  (`build_opt.h`).
- Mind the size: the CH32X035's 62 KiB is nearly full with the default set. Leave something out
  to make room.

## 5. Checking it

`oep dump --port <port>` shows your interface in the list and the parts you left out gone.
