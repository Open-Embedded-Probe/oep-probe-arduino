# Boards

[日本語](boards.ja.md)

What each chip can do with this library, which pins the released firmware offers, and how to build for boards without a
released firmware.

## What each chip can do

| | RP2040 / RP2350 | ESP32-P4 / P4X | classic ESP32 | WCH CH32 (X035 / V307 / H417) | Other ESP32s (S3, C3, C6 and so on) |
|---|---|---|---|---|---|
| Released firmware | `OepProbe-rp2040` / `-rp2350` | `OepProbe-esp32p4` / `-esp32p4x` | `OepProbe-esp32` | `OepProbe-ch32x035` / `-ch32v307` / `-ch32h417` | none: choose parts from an example and build it yourself (below; the all-in `OepFirmware.h` does not exist for these chips). This column is what those parts can make |
| Transports | USB CDC, vendor bulk, HID (UART bridge when chosen: GP8 / GP9) | HS vendor bulk, HID, USB CDC, USB-Serial/JTAG, UART bridge (GPIO37 / 38, 115200) | UART bridge (115200). With the wifi settings, also TCP over Wi-Fi (port 7450, mDNS `_oep._tcp`; Getting started §6) | USB vendor bulk, HID (X035: vendor bulk only). USB CDC and UART bridge when chosen | USB-Serial/JTAG or USB CDC |
| RVSWD (CH32's 2 wires) | yes (SIO) | yes (dedicated GPIO) | yes | yes | yes (dedicated GPIO) |
| SWIO (CH32V00x's 1 wire) | yes | yes | yes | yes (except X035) | none |
| SWD (ARM) | yes | yes | yes | yes (except X035) | none |
| Target console (through the debug module) | yes | yes | yes | yes | yes |
| GPIO, UART fixtures | yes | yes (2 UARTs) | yes | GPIO. UART except X035 | yes |
| Logic capture | PIO + DMA: 8 ch, one-shot, edge / level trigger | PARLIO: up to 16 ch. Rates in the table in [Capture](capture.md#esp32-p4--p4x-speeds-and-lines). One-shot level / edge trigger. The pretrigger limit is in [implementation limits §1.7](../implementation-limits.md#17-capture-and-fixtures) | GPIO sampler: 8 ch, 1-300 kHz, one-shot. Level / edge trigger searched per slice | 8 ch from bits 0-15 of one port, up to 250 kS/s, one-shot, edge / level trigger (except X035). H417 also has UHSIF's 16 ch (streaming, up to 200 MS/s) | none |
| Analog capture (one-shot, raw values, threshold trigger and pretrigger) | GP26-28, 4 ch, 500 kS/s total | ADC1's GPIO16-23, 4 ch, 46 kHz total, attenuation 0-12 dB | ADC1's GPIO32-36 / 39, 4 ch, 20-100 kHz total | ADC1, 4 ch, steps of the ADC sample time (tens to hundreds of kS/s). Except X035 | none (not built) |
| Starting tracks together (capture-group) | logic + analog. The event source is an analog crossing (analog tracks only) | logic + analog. The event source is a logic level / edge or an analog crossing | logic (sampler) + analog. The event source is an analog crossing (analog tracks only) | none | none |
| SPI / I2C target (the peer of the DUT's controller) | I2C, SPI (PIO) | yes | yes (SPI mode 0 / 2) | yes (except X035. SPI up to 200 kHz) | yes |
| SPI / I2C controller (the peer of the DUT's target) | yes (GPIO, up to 250 kHz) | yes (GPIO, I2C up to 400 kHz / SPI up to 750 kHz) | yes (GPIO, I2C up to 400 kHz / SPI up to 2 MHz) | yes (except X035. GPIO, up to 250 kHz) | none (not built) |
| PWM output | yes (PWM slices) | yes (LEDC) | yes (LEDC) | none | none (not built) |
| Counting edges (counter) | yes (interrupts, up to 100 kHz) | yes (PCNT, up to 40 MHz) | yes (PCNT, up to 10 MHz) | yes (except X035) | none (not built) |
| Analog output (DAC) | none | none | GPIO25 / 26, 8 bit | PA4 / PA5, 12 bit (V307, H417) | none |
| Saving settings | a LittleFS file (FS area: 256 KB / 512 KB) | a LittleFS file (partition `spiffs`) | a LittleFS file (partition `spiffs`) | none (not saved; X035 has no settings at all) | a LittleFS file (partition `spiffs`) |
| Memo area (`oep.probe.memo`) | 52 (RP2040) / 64 (RP2350) objects of 2 KB | 64 objects of 2 KB | 64 objects of 2 KB | none | 64 objects of 2 KB |

"none" means the library has no implementation for that chip, not that the chip cannot do it.

A capture-group event source (oep-spec capture §4.4) cuts the segments of all grouped tracks at the same instant, on a
condition of one track. Every grouped track is configured with a linked trigger. The RP2's PIO logic and the classic ESP32's
sampler have no linked trigger, so they cannot be in a group with an event source (they can start together in a group without
one).

While capturing, analog capture switches its pad to the analog function and cuts off digital input and output. So analog
channels are shared with nobody. A plan that puts a logic capture, a fixture or a wire on them is refused, and so is a plan
that puts analog on those pins. To watch one signal with both logic and analog, wire it to two pads.

All are 3.3 V parts. The library does not convert levels (see the README's electrical notes).

Each P4 example has the profiles `esp32p4` (`ChipVariant=prev3`) and `esp32p4x` (`ChipVariant=postv3`). A build example for
P4X: `arduino-cli compile --clean --profile esp32p4x examples/Firmware/OepProbe`. For choosing the firmware and the DFU
compatibility check, see [Getting started](getting-started.md#1-flash-the-firmware).

## Pins of the released firmware

The listed pins become anything, as the host's plan says (an RVSWD / SWD pair, a reset line, GPIO, UART, a capture channel).
A pin held by one interface is refused to the others. Pins not in the list belong to the board.

| firmware | Pins offered | Pins left alone |
|---|---|---|
| RP2040 / RP2350 | GP0-GP22, GP26-GP28 | GP23-GP25, GP29 (the Pico's SMPS, VBUS detection, LED, VSYS). On other boards, the host keeps away from parts connected to offered pins (LED, PSRAM chip select) |
| SparkFun Pro Micro RP2350 (the `rp2350` build) | the same as Pico 2: GP0-GP22, GP26-GP28 | GP23-GP25, GP29 (reserved by the Pico 2 build). GP19 is the PSRAM chip select, so the host does not use it |
| ESP32-P4 / P4X | GPIO0-GPIO54 (except 24, 25, 37, 38) | GPIO24 / 25 (USB-Serial/JTAG), 37 / 38 (UART0's pins: the UART bridge transport) |
| classic ESP32 | GPIO4, 5, 13, 14, 16-19 (16 / 17 removed at boot on PICO-D4 / PICO-V3 / D2WD and when the build enables PSRAM), 21-23, 25-27, 32, 33, 34-36, 39 (34-39 input only). SWIO needs an output pin below 32 | 1, 3 (UART0: the transport), 6-11 (flash), 0, 2, 12, 15 (boot straps) |

The RP2's UART fixtures are the chip's UARTs. Instance 0 (UART0) takes RX on one of GP1, 13, 17, 29 and TX on one of GP0, 12,
16, 28; instance 1 (UART1) RX on one of GP5, 9, 21, 25 and TX on one of GP4, 8, 20, 24 (the RP2040's; a build whose UART
bridge or trace uses UART1 has no instance 1). On ESP32 any pin works (the GPIO matrix).

## USB identity

| firmware | VID:PID | iProduct | serial number |
|---|---|---|---|
| RP2040 / RP2350 | `1209:4F45` | `OEP probe (RP2040)` / `(RP2350)` | unit id (the flash's unique id) |
| ESP32-P4 (HS port) | `1209:4F45` | `OEP probe (ESP32-P4)` | unit id (the MAC in lowercase hex) |
| ESP32-P4X (HS port) | `1209:4F45` | `OEP probe (ESP32-P4X)` | the same |
| ESP32-P4 / P4X (USB-Serial/JTAG) | the chip's fixed ID | the chip's | the chip's |
| classic ESP32 | the bridge's | the bridge's | the bridge's |

`1209:4F45` is the project's own VID:PID. The host recognizes probes by it and tells units apart by serial number (unit id).
iProduct is a name for people to read. The conditions for shipping firmware with this VID:PID are in
[PID-USE.md](../../PID-USE.md). Ports with fixed IDs (USB-UART bridge, USB-Serial/JTAG) are chosen by the user. On Linux, the
udev rule shipped by oep-client-python,
[`udev/70-oep-probe.rules`](https://github.com/Open-Embedded-Probe/oep-client-python/blob/main/udev/70-oep-probe.rules), lets
ordinary users open the vendor bulk, DFU and HID interfaces of `1209:4F45`. Installing it needs administrator rights.

## Building for other boards

Chips not in the table have no released firmware. Building it yourself takes a few minutes. Example: making an ESP32-S3
DevKitC a CH32 debugger.

1. **Start from the example closest to what you want.** `04.Debug/RvswdDebugProbe` for a CH32 debugger,
   `01.Basics/FixtureProbe` for a jig, `Firmware/OepProbe` for everything (`OepFirmware.h`; for a new chip, start from the part
   of a nearby chip, for example `src/firmware/Esp32P4.h`).
2. **Add a profile** to its `sketch.yaml`: the board's FQBN, the core version, the libraries.

   ```yaml
   profiles:
     esp32s3:
       fqbn: esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc
       platforms:
         - platform: esp32:esp32 (3.3.13)
           platform_index_url: https://espressif.github.io/arduino-esp32/package_esp32_index.json
       libraries:
         - OpenEmbeddedProbe (<library version>)
   ```

   `RvswdDebugProbe` already has this profile, and builds with
   `arduino-cli compile --profile esp32s3 examples/04.Debug/RvswdDebugProbe`.
3. **Choose the transport** by what `Serial` is. USB-Serial/JTAG (`USBMode=hwcdc`: `kUsbSerialJtag`), TinyUSB's CDC
   (`USBMode=default`: `kUsbCdc`; for firmware allowed to use the project's VID:PID, `USB.VID(oep::reg::kUsbProjectVid)`,
   `USB.PID(oep::reg::kUsbProjectPid)`, and the unit id in `USB.serialNumber(...)`; [PID-USE.md](../../PID-USE.md)), a
   USB-UART bridge (`kUartBridge`; classic ESP32, the ESP32-C3's UART0). USB-Serial/JTAG and bridges keep their fixed IDs, and
   the host reaches them through the port the user chooses.
4. **Choose the pins.** Leave out the flash / PSRAM pins, the USB pins, the boot straps and the pins the board connects to
   parts (keep them out of the pin table, and do not put them in the idle state at boot either), and offer the rest (the pin
   table's mask; `pin_choice` if the host chooses). Things to consider:
   - **Boot straps**: they are read only at the moment reset is released, so if the target holds one low / high, the probe
     boots in another mode (download mode if the ESP32's GPIO0 is low). Whether the target pulls it is unknown until wired, so
     leave them out. RP2040 / RP2350 have none (BOOTSEL is on the flash chip select side).
   - **The default UART**: the ESP32's boot ROM logs to UART0 at every reset, and that flows into whatever is connected, so
     leave it out. On RP2 the default is only the core's convention (`Serial1`), and those pins can be used.
   - **Leave unused pins idle** (Hi-Z, no pull). Call `oep::platformParkMask` in `setup()` before the first answer. RP2 pads
     boot with a pull-down and can keep pulling the reset of a half-wired target.
   - **RP2350 erratum E9**: a released input can stick high. When reading a pin, read it with the pad's pull applied.
   - Keep lines of the same function next to each other (the RP2's PIO assumes consecutive pins).
5. **Build and flash**: `arduino-cli compile --profile esp32s3 <dir>`, `arduino-cli upload -p <port> --profile esp32s3 <dir>`.
6. **Check** with `oep dump --port <port>`, and attach the target from Python ([Getting started](getting-started.md)).

Let us know when it works (an issue in the repository).

## Chips the library does not know

The Arduino core differences are few and gathered in one place (`src/OepPlatform.h`: pin modes, unit id, random numbers, UART
settings). A debug wire needs a PHY implementation for that chip (`OepRvswdPhy.cpp` has the ESP32's dedicated GPIO and the
RP2's SIO; the others are an empty implementation that does not attach). The endpoint, pin table, fixtures and settings are
portable (`tests/host` builds on a PC).
