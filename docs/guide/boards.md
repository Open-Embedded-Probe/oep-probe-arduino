# Boards

[日本語](boards.ja.md)

What each chip can do with this library, which pins the released firmware offers, and how to build for a board that has
no released firmware.

## What each chip does

| | RP2040 / RP2350 | ESP32-P4 | classic ESP32 | other ESP32 (S3, C3, C6, ...) |
|---|---|---|---|---|
| Released firmware | `OepProbe-rp2040` / `-rp2350` / `-promicrorp2350` | `OepProbe-esp32p4` | `OepProbe-esp32` | none: build it (below) |
| Checked on a bench | starting, on a Pro Micro RP2350 (CH32L103) | yes (CH32X035 jig) | yes (CH32V003 jig) | no |
| Transports | USB CDC | HS vendor bulk, HID, USB CDC, USB-Serial/JTAG | UART bridge (115200) | USB-Serial/JTAG or USB CDC |
| RVSWD (CH32 2-wire) | yes (SIO) | yes (dedicated GPIO) | no | yes (dedicated GPIO; built, not checked) |
| SWIO (CH32V00x 1-wire) | no | no | yes | no |
| SWD (ARM) | yes | no | no | no |
| Target console (through the debug module) | yes | yes | yes | yes |
| GPIO, UART fixtures | yes | yes (2 UARTs) | yes | yes |
| Logic capture | no | PARLIO: up to 16 ch, 2 ch 160 Msps / 8 ch 40 Msps / 16 ch 20 Msps; one-shot level / edge trigger, pretrigger up to 32 Ki samples (16 ch) | GPIO sampler: 8 ch, 0.4-2 MHz, one-shot; level / edge trigger searched in bursts | no |
| Analog capture (one-shot, raw values, a threshold trigger with pretrigger) | GP26-28, 4 ch, 500 kS/s in all | ADC1 GPIO16-23, 4 ch, 46 kHz in all, attenuations 0-12 dB | ADC1 GPIO32-36 / 39, 4 ch, 20-100 kHz in all | no (not built) |
| Tracks started together (capture-group) | no | logic + analog | logic (sampler) + analog | no |
| SPI / I2C device (the target's peer) | no | yes | yes | yes (not checked) |
| Settings saved | flash (last sector) | NVS | NVS | NVS |

"No" means the library has no backend for it on that chip yet, not that the chip cannot: SWIO times its bits by counting
classic-ESP32 cycles, SWD's bit-bang is the RP2's, the capture drivers are the P4's PARLIO and the classic ESP32's
sampler.

An analog capture takes its pads to their analog function while it runs, which cuts their digital input and output, so
an analog channel is shared with nothing: a plan that puts a logic capture, a fixture or a wire on it (or the analog on
their pin) is refused. Watch one signal as logic and analog on two pads.

All of them are 3.3 V parts. The library does not shift levels: see the README's electrical notes.

## Pins of the released firmware

Every listed pin can be anything - an RVSWD / SWD pair, a reset line, a GPIO, a UART, a capture channel - as the host
plans it; a pin one interface holds is refused to another. The others are the board's own.

| Firmware | Pins offered | Left alone |
|---|---|---|
| RP2040 / RP2350 | GP0-GP22, GP26-GP28 | GP23-GP25, GP29 (a Pico's SMPS, VBUS sense, LED, VSYS). On other boards, parts on offered pins (an LED, a PSRAM chip select) are for the host to leave alone |
| SparkFun Pro Micro RP2350 (`promicrorp2350`) | GP0-GP18, GP20-GP29 | GP19 (the PSRAM's chip select) |
| ESP32-P4 | GPIO0-GPIO54 but 24, 25 | GPIO24 / 25 (USB-Serial/JTAG) |
| classic ESP32 | GPIO4, 5, 13, 14, 16-19 (16 / 17 dropped at start-up on a PICO-D4 / PICO-V3 / D2WD, or with a PSRAM the build enabled), 21-23, 25-27, 32, 33, 34-36, 39 (34-39 input only); SWIO on the outputs below 32 | 1, 3 (UART0: the transport), 6-11 (flash), 0, 2, 12, 15 (boot straps) |

The UART fixture on an RP2 uses UART0, whose RX / TX may be GP1/0, GP13/12, GP17/16 or GP29/28. On the ESP32s any pin
works (the GPIO matrix).

## USB identity

| Firmware | VID:PID | iProduct | Serial number |
|---|---|---|---|
| RP2040 / RP2350 | `1209:4F45` | `OEP probe (RP2040)` / `(RP2350)` | the unit id (the flash's unique id) |
| ESP32-P4 (HS port) | `1209:4F45` | `OEP probe (ESP32-P4)` | the unit id (the MAC, lowercase hex) |
| ESP32-P4 (USB-Serial/JTAG) | the chip's fixed ID | the chip's | the chip's |
| classic ESP32 | the bridge's | the bridge's | the bridge's |

`1209:4F45` is the project's own VID:PID: hosts identify a probe by it and tell probes apart by the serial number (the unit
id). iProduct is only a name for people. Who may ship firmware with the VID:PID: [PID-USE.md](../../PID-USE.md). A port with a
fixed ID (a USB-UART bridge, USB-Serial/JTAG) is chosen by the user. On Linux, the udev rule shipped in oep-client-python,
[`udev/70-oep-probe.rules`](https://github.com/Open-Embedded-Probe/oep-client-python/blob/main/udev/70-oep-probe.rules), lets an ordinary user open `1209:4F45`'s
vendor bulk, DFU and HID interfaces; installing it needs administrator rights.

## Building for another board

Boards with no bench get no released firmware: an unchecked binary helps nobody. Building one yourself takes a few
minutes. The example: an ESP32-S3 DevKitC as a CH32 debugger.

1. **Start from the example closest to what you want.** A CH32 debugger: `04.Debug/RvswdDebugProbe`; a fixture:
   `01.Basics/FixtureProbe`; everything: `Firmware/OepProbe` (copy the part of the chip nearest yours, e.g. `Esp32P4.h`).
2. **Add a profile** to its `sketch.yaml`: the board's FQBN, the core version, the library.

   ```yaml
   profiles:
     esp32s3:
       fqbn: esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc
       platforms:
         - platform: esp32:esp32 (3.3.12)
           platform_index_url: https://espressif.github.io/arduino-esp32/package_esp32_index.json
       libraries:
         - OpenEmbeddedProbe (0.0.9)
   ```

   `RvswdDebugProbe` already has this profile: `arduino-cli compile --profile esp32s3 examples/04.Debug/RvswdDebugProbe`
   builds.
3. **Pick the transport** by what `Serial` is: USB-Serial/JTAG (`USBMode=hwcdc`: `kUsbSerialJtag`), TinyUSB CDC
   (`USBMode=default`: `kUsbCdc`; `USB.VID(oep::reg::kUsbProjectVid)`, `USB.PID(oep::reg::kUsbProjectPid)`, the unit id
   as `USB.serialNumber(...)` when the firmware may carry the project's VID:PID, [PID-USE.md](../../PID-USE.md)), or a
   USB-UART bridge (`kUartBridge`, a classic ESP32 or an ESP32-C3 on its UART0). USB-Serial/JTAG and a bridge keep their
   fixed ID: a host reaches them by the port the user chooses.
4. **Choose the pins.** Leave out the flash / PSRAM pins, the USB pins, the boot straps and anything the board wires to a
   part (keep them out of the pin table and out of the start-up parking), offer the rest (the pin table's mask,
   `pin_choice` for host-chosen pins).
5. **Build and flash**: `arduino-cli compile --profile esp32s3 <dir>`, then `arduino-cli upload -p <port> --profile esp32s3
   <dir>`.
6. **Check** with `oep dump --port <port>`, then attach a target from Python ([Getting started](getting-started.md)).

If it works, tell us (an issue on the repository): a board with a bench can get released firmware.

## A chip the library does not know

The parts that differ between Arduino cores are few and in one place, `src/OepPlatform.h` (pin modes, the unit id, a
random number, UART set-up). A debug wire needs a PHY backend for the chip (`OepRvswdPhy.cpp` has the ESP32 dedicated
GPIO and the RP2 SIO ones; others get a stub that never attaches). The endpoint, the pin table, the fixtures and the
settings are portable - `tests/host` builds them on a PC.
