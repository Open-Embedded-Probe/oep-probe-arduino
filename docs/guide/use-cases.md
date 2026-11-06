# What OEP does for you (a guide by use case)

[日本語](use-cases.ja.md)

Put the OEP firmware on a development board and it becomes a "probe" that software on your PC drives. The probe sits
between the PC and the chip or board you work on, and does debugging, communication, signal input and output and
waveform capture, all in one way.

This guide starts from what you want to do: what to prepare, how to wire, what to set, and what it solves. The settings
as a whole are in [Settings](settings.md), each debug target in [Debug targets](debug-targets.md), and the ways to
capture in [Capture: which way](capture.md).

## Choose by what you want to do

| You want to | What it solves | Section |
|---|---|---|
| Flash and debug a CH32 (WCH's RISC-V MCUs) | Flash from the Arduino IDE or the command line, halt the chip and look inside, with no WCH-Link | [1](#1-flash-and-debug-a-ch32) |
| See the target's log | Read the log in a plain serial monitor or a tool, whether the target's UART is free or not | [2](#2-see-the-targets-log-send-it-commands) |
| Look inside an ARM Cortex-M | Read and write memory and registers over SWD, halt and run it | [3](#3-look-inside-an-arm-cortex-m) |
| Capture waveforms (a logic analyzer) | Take signal changes with their times, keep them in a file, decode the protocols | [4](#4-capture-waveforms) |
| Talk I2C / SPI to a sensor or an IC | The probe is the side that starts the transfer: read and write the part's registers | [5](#5-talk-i2c--spi-to-a-sensor-or-an-ic) |
| Try your MCU's I2C / SPI driver | The probe stands in for the sensor: you decide what it answers and see what came | [6](#6-try-your-mcus-i2c--spi-driver) |
| Drive and measure signals | GPIO, PWM, pulse counts and analog values from the PC | [7](#7-drive-and-measure-signals) |
| Make a test fixture | Flashing, logs, signals and waveforms from test code, again and again, with pass / fail | [8](#8-make-a-test-fixture) |
| Use it from elsewhere | Use the probe over Wi-Fi, without USB | [9](#9-use-it-from-elsewhere) |
| Keep one as a fixed jig | The same wiring and the same idle states at every boot, the same names from any PC | [10](#10-keep-a-probe-as-a-fixed-jig) |

## 0. Before you start

### What you need

- **A board to make the probe**: RP2040 / RP2350 (Pico, Pico 2 and others), ESP32-P4 / P4X, classic ESP32 (DevKitC and
  others). Check which board has which function in the table in [Boards](boards.md#what-each-chip-does). As a rule of thumb:
  - Debugging and communication, simply: RP2040 / RP2350.
  - Fast capture, many channels, a fixed test jig: ESP32-P4 / P4X.
  - Wi-Fi, a DAC: classic ESP32.
- **A USB cable that carries data**.
- **Software on the PC**: from "Tools" below, what fits what you do.
- **Wires**: jumper wires and the like. **Always connect the probe's GND to the target's.**

### Electrical notes

- Every probe pin is 3.3 V. Put a level shifter in front of a 5 V target; the probe does no level shifting.
- When the probe powers the target, stay within what the board's 3.3 V / 5 V pins allow. When the target has its own
  supply, do not join the supplies: share only GND.
- Do not keep driving a target pin from the probe while the target is unpowered (it feeds the target through its pins).

### Put the firmware on

Flash the firmware for your board as in [Getting started](getting-started.md#1-flash-the-firmware). The firmware has no
wiring built in: the same firmware serves any wiring.

### Tools

| Tool | What for | Where |
|---|---|---|
| The `oep` command and Python's `oep_client` | Look at the probe (`oep dump`), set it (`oep settings`), find the wiring (`oep pins`); every function from Python | [oep-client-python](https://github.com/Open-Embedded-Probe/oep-client-python) (`pip install oep-client-python`) |
| `ch32rv` | Flash, debug (GDB server), show the log and recover a CH32 | [ch32rv](https://github.com/ch32-riscv-ug/ch32rv) |
| The Arduino IDE with ArduinoCore-CH32RV | Upload CH32 sketches through the probe | [ArduinoCore-CH32RV](https://github.com/ch32-riscv-ug/ArduinoCore-CH32RV) |
| WireSkein | Capture, decode and show waveforms, and check them in tests | [wireskein](https://github.com/Open-Embedded-Probe/wireskein) |
| The pytest plugin | Keep and check waveforms per test | [pytest-embedded-wireskein](https://github.com/Open-Embedded-Probe/pytest-embedded-wireskein) |

### Name the port

Give the set of pins wired to a target a name, and every tool reaches that target as `oep://<probe>/<port>`. The name is
kept on the probe, so it stays the same whatever PC or tool you use.

- The name comes from the probe's pin labels (the setting `channel.{}.label`), as `port.line@target-pin`. Examples:
  `mcu.swdio@pa13`, `mcu.swclk@pa14`, `mcu.rx@pa9`, `mcu.nrst`. The usual line names are `swdio`, `swclk`, `swio`, `rx`,
  `tx`, `nrst`. A line with no role is written `mcu@pb6`.
- `<probe>` is the probe's own ID (its unit id), shown by `oep dump`.
- Every function also works without labels, given pin numbers. Names are a handy record for targets you come back to.
- Debug lines can be found with `oep pins` and saved as labels with `--save` ([1](#1-flash-and-debug-a-ch32)).

The full rules are in [Settings](settings.md#pin-labels-port-and-line-names).

## 1. Flash and debug a CH32

**What it solves**: flash a CH32 with a Pico or an ESP32 you have, without buying a WCH-Link, and halt it to see memory
and registers. One probe can carry several targets, picked by port name.

**What you need**: a probe, the CH32 (an evaluation board or your own), `ch32rv` or the Arduino IDE with ArduinoCore-CH32RV.

**Wiring**:

| Target | Wire | Target pins |
|---|---|---|
| CH32V002 / V003 / V004 | one wire (SWIO) | PD1 |
| CH32V005 / V006 / V007, CH32M007 | one wire (SWIO) | PD1 |
| CH32X033 / X035 | two wires (RVSWD) | PC18 (SWDIO), PC19 (SWCLK) |
| CH32V103, V203 / V208, V30x, L103 / M103, V205, V407 / V467, X305 / X315 | two wires (RVSWD) | PA13 (SWDIO), PA14 (SWCLK) |
| CH32M030 | two wires (RVSWD) | PA3 (SWDIO), PA2 (SWCLK) |

- On the probe's side, any pin will do (those the board's table gives). Connect GND.
- Wire the target's NRST to a probe pin and you can attach under reset, and recover when the debug lines no longer answer.
  On the CH32V003 and others the NRST pin may be off in the option bytes ([Debug targets](debug-targets.md)).
- Notes per target (how the lines rest, the speed limit) are in [Debug targets](debug-targets.md).

**Settings**:

1. Find the lines and make them labels (recommended). If you know the pins you wired, writing the labels with
   `oep settings set` is enough.

   ```sh
   oep pins oep://<probe> --wire rvswd --name mcu --save    # --wire swio for a one-wire target
   ```

   `oep pins` talks to the target on free pin pairs and saves the pins that answered as `mcu.swdio` / `mcu.swclk`
   (`mcu.swio` on one wire). When it finds the reset line, it labels `mcu.nrst` too. The scan drives the target's lines,
   so use it when you know what is wired.
2. How the lines rest and the speed are normally not set: the probe and `ch32rv` choose them for the target. Only to fix
   them for a given target, use `channel.{}.rest` ([Settings](settings.md#debug-lines)).

**Use**:

- Arduino IDE: choose the CH32 board of ArduinoCore-CH32RV, choose `oep://<probe>/<port>` (or the probe's serial port)
  as the port, and Upload.
- Command line:

  ```sh
  ch32rv --probe oep://<probe>/mcu target info     # see what is there
  ch32rv --probe oep://<probe>/mcu flash app.elf   # flash, verify, reset, run
  ch32rv --probe oep://<probe>/mcu gdb             # GDB server (127.0.0.1:3333)
  ```

- From Python, `Wire` and `RiscvDm` in `oep_client.riscv` attach, halt, read and write memory and registers
  ([Getting started](getting-started.md#debug-a-ch32-on-rvswd)).

**Notes**:

- An application that turns the debug pin into a GPIO, or sleeps deeply, cannot be reached. With NRST wired,
  `ch32rv recover --method nrst` brings it back.
- Closing the debug connection may leave the target halted (when it was closed halted). Resume before closing to have it run.

## 2. See the target's log, send it commands

**What it solves**: read the log your MCU prints, on the PC. Both a log on the UART and a log through the debug lines
without the UART. With a setting, opening the probe's serial port in a plain serial monitor shows the log.

There are three ways.

| Way | How the target prints | Wiring | Direction |
|---|---|---|---|
| A. Over the UART | The target's UART (`Serial.print` and so on) | target TX → probe RX, (to send) probe TX → target RX, GND | both |
| B. Over the debug lines | The debug module's mailbox (CH32's `sdi`, `dmdata`, `dmseq`) | The debug lines of [1](#1-flash-and-debug-a-ch32), GND | `sdi` target → PC; `dmdata` / `dmseq` both |
| C. Passed through to the serial port | A or B, sent to the probe's serial port while no session is open | As A or B | target → PC |

**A. Over the UART**

- Wire the target's TX to a pin that can be the probe UART's RX (any pin on an ESP32; fixed pairs on an RP2, see [Boards](boards.md)).
- The rate (baud) and format (8N1 and so on) are given by the tool when it starts reading; they are not a probe setting.
- Read with Python's `FixtureUartIO` or `ch32rv monitor --source uart`. What is read is kept in the probe, and reading
  can go on from where it stopped.

**B. Over the debug lines**

- For targets that cannot spare the UART pins or use the UART for something else. The sketch prints through the output
  the core provides (ArduinoCore-CH32RV's
  [debug output](https://github.com/ch32-riscv-ug/ArduinoCore-CH32RV/blob/main/docs/debug-output.ja.md)).
- `ch32rv monitor --source dmseq` (or `sdi`, `dmdata`) reads it and sends input. Flashing and the log share the one port.

**C. Passed through to the serial port**

- For seeing the log just by opening the probe's serial port in the Arduino IDE's serial monitor or a terminal.
- Settings (saved, they apply at every boot):
  - The UART: `passthrough.<port>.rx`, `.baud`, `.format` of `oep.fixture.uart`.
  - The debug-line log: `passthrough.<port>.wire`, `.swdio`, `.swclk`, `.mechanism` of `oep.target.console`.
  - `<port>` is the name of the probe's serial port (`cdc0` and so on, see [Settings](settings.md#passthrough)).
- Passthrough stops while another tool uses the probe and comes back after. A flashing tool keeps using the same port.
- Passthrough goes target → PC only. To send to the target, use A or B.

## 3. Look inside an ARM Cortex-M

**What it solves**: attach to an ARM Cortex-M over SWD, read and write memory and peripheral registers, halt and run the
core. From Python you can also call a function placed in the target's RAM.

**What you need**: a probe with SWD ([Boards](boards.md#what-each-chip-does)), the Cortex-M, Python's `oep_client`.

**Wiring**: the target's SWDIO and SWCLK (PA13 / PA14 on most STM32), GND; NRST when needed.

**Use**: `SwdWire` (attach; TARGETSEL for a multidrop target), `ArmAdi` (DP / AP), `MemAp` (memory) and `CortexM` (halt,
resume, registers, calling a function, system reset) in `oep_client.arm`.

**Notes**: no flash loader for Cortex-M is provided; flash it from Python yourself. There is no GDB server either.

## 4. Capture waveforms

**What it solves**: as a logic analyzer, take signal changes with their times, keep them in a file, and decode UART, SPI,
I2C, CAN, LIN, the CH32 debug lines and more. Analog values can be taken alongside (on boards that have it).

**What you need**: a board that captures (speed and channels in [Boards](boards.md#what-each-chip-does); ESP32-P4 / P4X for
fast signals), WireSkein.

**Wiring**: the signals to probe pins, and GND. Capture only listens, so it can take pins another function is using.

**Use**:

```sh
wireskein capture --source oep:oep://<probe> --channels SDA=GPIO47,SCL=GPIO48 --rate 20M --samples 200k -o i2c.wireskein
wireskein gui i2c.wireskein             # show it in the browser
wireskein convert i2c.wireskein i2c.sr  # to open it in PulseView
```

The way to capture differs between capture on its own and capture on the labels of a debug or test jig. The differences
are in [Capture: which way](capture.md).

## 5. Talk I2C / SPI to a sensor or an IC

**What it solves**: without writing MCU code, talk from the PC to an I2C / SPI part (a sensor, an EEPROM, a display),
read and write its registers and see how it behaves.

**What you need**: a probe with the I2C / SPI controller, the part, Python's `oep_client`.

**Wiring**:

- I2C: SDA, SCL, GND. Pull-up resistors (a few kΩ) are needed; add them if the part's board has none.
- SPI: SCK, MOSI, MISO, CS, GND.

**Use**: `oep.fixture.i2c-controller` / `oep.fixture.spi-controller` from Python. The address, speed and SPI mode are
given with each transfer; they are not probe settings. The probe's I2C / SPI are made with GPIO, so the speed limit is
in the table in [Boards](boards.md#what-each-chip-does).

## 6. Try your MCU's I2C / SPI driver

**What it solves**: the probe plays the part your MCU (the controller) talks to. Without the real sensor, decide what it
answers and try the driver. The writes and reads your MCU made are kept in the probe and read afterwards.

**What you need**: a probe with the I2C / SPI target ([Boards](boards.md#what-each-chip-does)), your MCU, Python's `oep_client`.

**Wiring**: the same lines as [5](#5-talk-i2c--spi-to-a-sensor-or-an-ic), between your MCU and the probe. The I2C pull-ups on one side or the other.

**Use**: `oep.fixture.i2c-target` / `oep.fixture.spi-target` from Python. Set the I2C address or the SPI mode, queue the
data to answer, and wait.

## 7. Drive and measure signals

**What it solves**: move a pin instead of pressing a switch, read the target's output, try an LED or motor driver with
PWM, measure a frequency, output or take analog voltages, all from the PC.

| Function | Interface | What it does |
|---|---|---|
| GPIO | `oep.fixture.gpio` | input (with or without a pull), output high / low, open drain (for reset lines and the like) |
| PWM | `oep.fixture.pwm` | a frequency and a duty |
| Counting pulses | `oep.fixture.counter` | edges in a set time, and the frequency from them |
| Analog output | `oep.fixture.dac` | boards with a DAC only (classic ESP32 and others) |
| Analog input | `oep.fixture.analog` | voltages taken at a set rate |

**Wiring**: the pins used and GND. Do not let an output fight the target's output.

**Use**: Python's `oep_client.fixture` ([Getting started](getting-started.md#gpio-and-uart-a-test-fixture)). A pin one function
holds is refused to the others, so test code never drives two outputs into each other. A pin a function lets go goes
back to its idle state; to choose that state, use `channel.{}.idle` ([Settings](settings.md#idle-states-and-disabled-pins)).

## 8. Make a test fixture

**What it solves**: repeat flashing, reset, checking the log, driving and reading signals, and capturing and checking
waveforms, from test code. Results are pass / fail with the measured values, ready for CI.

**What you need**: a probe for the jig (an ESP32-P4 / P4X with its many channels suits it), the target board, pytest,
`oep_client`, WireSkein and pytest-embedded-wireskein; for a CH32, ch32rv and ArduinoCore-CH32RV's test tools.

**Wiring**: always the debug lines, the standard UART, NRST and the BOOT pins (when the target has them), and as many
other pins as you can, so that the lines of functions you will want to test later are already there.

**Settings**: those of a fixed jig ([10](#10-keep-a-probe-as-a-fixed-jig)). With the wiring in labels, test code finds a
line by name, such as `mcu.rx`, not by pin number.

**Use**: in a test, take the probe's session, use the functions needed, and let it go at the end. When the session ends,
every pin and connection the test took is released, and nothing carries over to the next test. Waveforms become test
results through WireSkein's record and checks
([WireSkein's capture tests](https://github.com/Open-Embedded-Probe/wireskein/blob/main/docs/capture-test-guide.md)).

## 9. Use it from elsewhere

**What it solves**: use the probe from a PC on the same network, without USB.

**What you need**: a probe with Wi-Fi (classic ESP32).

**Settings**: set `wifi.<n>.ssid` and `wifi.<n>.passphrase` from the serial port and save
([Getting started](getting-started.md#6-over-wi-fi-classic-esp32)). For a fixed address, `ipv4.<n>.*`.

**Use**: find it with `oep find` and open `oep://<probe>?via=net`. Every tool takes the same address.

**Notes**: OEP has no authentication. Use it only on a network you trust, or inside an authenticated tunnel.

## 10. Keep a probe as a fixed jig

**What it solves**: a jig that is the same at every boot: the names of its wiring, the idle states of unused pins, the
pins kept out of use and the log passthrough are the same from any PC.

**Set in this order**:

1. **Pins to keep out of use**: pins wired to the board's own parts (LEDs, buttons, boot straps) get `channel.{}.disable`.
2. **Name the wiring**: label each target's lines by port (`oep pins --save` or `oep settings set`).
3. **Idle states**: lines whose target input must not float (BOOT0, reset, chip selects) get `channel.{}.idle`.
4. **Log passthrough** (if used): C of [2](#2-see-the-targets-log-send-it-commands).
5. **Save and keep a copy**: `oep settings save` to apply them at every boot, `oep settings export` for a file copy.
   After a firmware update, or to make the same jig on another unit, `oep settings import` puts them back.

The target's part number and notes, kept in the probe's memo store (`oep.probe.memo/port/<port>`), read the same from any PC.

What each item means and when it applies is in [Settings](settings.md).

## What it does not do

- OEP's debug lines are RVSWD and SWIO (WCH's RISC-V) and SWD (ARM). It does not do JTAG, flash ESP32s or AVRs, or other
  proprietary lines.
- Each target's flashing procedure belongs to the PC tool (ch32rv and the like). Reaching a target's debug lines does
  not mean any target can be flashed.
- The probe does no level shifting and switches no power. Use outside circuits when needed.
