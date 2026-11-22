# What OEP does for you (a guide by use case)

[日本語](use-cases.ja.md)

Put the OEP firmware on a development board and it becomes a "probe" used from PC software. The probe sits between the PC and
the peer chip or board, and handles debugging, communication, driving and reading signals, and capturing waveforms, all the same
way.

This document starts from what you want to do and sums up what to prepare, how to wire, what to set, and what it solves. All the
settings are in [Settings](settings.md), the details per debug peer in [Debug targets](debug-targets.md), and the differences
between ways of capturing in [Capture: which way](capture.md).

## Choose by what you want to do

| What you want to do | What it solves | Section |
|---|---|---|
| Flash and debug a CH32 (WCH's RISC-V MCU) | without a WCH-Link, flash from the Arduino IDE or a CLI, halt it and look inside | [1](#1-flash-and-debug-a-ch32) |
| Watch the peer MCU's log | whether or not a UART is free, read the log with an ordinary serial monitor or tool | [2](#2-watch-the-peers-log-send-commands) |
| Look inside an ARM Cortex-M | read and write memory and registers over SWD, halt and run it | [3](#3-look-inside-an-arm-cortex-m) |
| Capture waveforms (as a logic analyzer) | capture the timing of signals, keep it in a file, decode protocols | [4](#4-capture-waveforms) |
| Talk to sensors and ICs over I2C / SPI | the probe starts the communication and reads and writes the peer's registers | [5](#5-talk-to-sensors-and-ics-over-i2c--spi) |
| Test your MCU's I2C / SPI driver | the probe stands in for the peer sensor, and you decide which requests came and what to return | [6](#6-test-your-mcus-i2c--spi-driver) |
| Drive and measure signals | GPIO, PWM, pulse counts and analog values in and out from the PC | [7](#7-drive-and-measure-signals) |
| Make an automated test fixture | flash, log, waveforms and signal I/O repeated from test code, with pass/fail | [8](#8-make-an-automated-test-fixture) |
| Use it from a distance | use the probe over Wi-Fi without connecting USB | [9](#9-use-it-from-a-distance) |
| Install one as a fixture | used with the same wiring and the same idle states at every boot. Other PCs name it the same way | [10](#10-install-the-probe-as-a-fixture) |

## 0. Common preparation

### What to prepare

- **A board to make the probe**: RP2040 / RP2350 (Pico, Pico 2 and so on), ESP32-P4 / P4X, classic ESP32 (DevKitC and so on),
  ESP32-S3, WCH CH32 (X035 / V307 / H417). Check which features each board has in the table in
  [Boards](boards.md#what-each-chip-can-do). If unsure, use this as a guide.
  - Debugging and communication made easy: RP2040 / RP2350.
  - Fast waveform capture, many channels, an installed fixture: ESP32-P4 / P4X.
  - Use over Wi-Fi, use a DAC: classic ESP32.
- **A USB cable that carries data**.
- **PC software**: install what fits your purpose from "Tools" below.
- **Wires**: jumper wires and so on. **Always connect the GND** of the probe and the peer.

### Electrical notes

- Every probe pin is a 3.3 V part. Put a level converter in between for a 5 V peer. The probe does not convert levels.
- When powering the peer from the probe, stay within the current the board's 3.3 V / 5 V pins allow. When the peer has its own
  supply, do not connect the supplies together; share only GND.
- Do not keep applying voltage from the probe's pins to the peer while the peer is unpowered (it flows from the peer's pins into
  its supply).

### Putting the firmware on

As in [Getting started](getting-started.md#1-flash-the-firmware), write the firmware that fits the board. No wiring is built
into the firmware, so the same firmware serves any wiring.

### Tools

| Tool | What for | Where to get it |
|---|---|---|
| the `oep` command and Python's `oep_client` | checking the probe (`oep dump`), settings (`oep settings`), finding wiring (`oep pins`). Every feature from Python | [oep-client-python](https://github.com/Open-Embedded-Probe/oep-client-python) (`pip install oep-client-python`) |
| `ch32rv` | flashing, debugging (GDB server), showing logs and recovering CH32 | [ch32rv](https://github.com/ch32-riscv-ug/ch32rv) |
| the Arduino IDE and ArduinoCore-CH32RV | upload CH32 sketches through the probe | [ArduinoCore-CH32RV](https://github.com/ch32-riscv-ug/ArduinoCore-CH32RV) |
| WireSkein | capture, decode, view waveforms, check them in tests | [wireskein](https://github.com/Open-Embedded-Probe/wireskein) |
| the pytest plugin | keep and check waveforms per test | [pytest-embedded-wireskein](https://github.com/Open-Embedded-Probe/pytest-embedded-wireskein) |

### Naming ports

Give a name to the set of pins wired to a peer chip, and every tool names the same peer with `oep://<probe>/<port>`. The name is
saved on the probe, so it stays the same when you change PCs or tools.

- Names come from the probe's pin labels (the setting `channel.{}.label`). The form is `port.line@peer pin`. Examples:
  `mcu.swdio@pa13`, `mcu.swclk@pa14`, `mcu.rx@pa9`, `mcu.nrst`. The conventional line names are `swdio`, `swclk`, `swio`, `rx`,
  `tx`, `nrst`. A line without a role can be written `mcu@pb6`.
- `<probe>` is the probe unit's ID (unit id), shown by `oep dump`.
- Without labels, every feature still works by giving pin numbers directly. Names are a convenient record for peers used
  repeatedly.
- Debug wires can be scanned for with `oep pins` and turned into labels as is with `--save` ([1](#1-flash-and-debug-a-ch32)).

The detailed rules are in [Settings](settings.md#pin-labels-port-and-line-names).

## 1. Flash and debug a CH32

**What it solves**: without buying a WCH-Link, flash a CH32 with a Pico or ESP32 you have, halt it and look at memory and
registers. Several peers can be connected to one probe and chosen by port name.

**What to prepare**: a probe, the peer CH32 (an evaluation board or your own board), `ch32rv` or the Arduino IDE and
ArduinoCore-CH32RV.

**Wiring**:

| Peer | Wire | Peer pins |
|---|---|---|
| CH32V002 / V003 / V004 | 1 wire (SWIO) | PD1 |
| CH32V005 / V006 / V007, CH32M007 | 1 wire (SWIO) | PD1 |
| CH32X033 / X035 | 2 wires (RVSWD) | PC18 (SWDIO), PC19 (SWCLK) |
| CH32V103, V203 / V208, V30x, L103 / M103, V205, V407 / V467, X305 / X315 | 2 wires (RVSWD) | PA13 (SWDIO), PA14 (SWCLK) |
| CH32M030 | 2 wires (RVSWD) | PA3 (SWDIO), PA2 (SWCLK) |

- On the probe side any pin works (the pins offered in the board table). Connect GND.
- Wiring the peer's NRST to a probe pin enables connecting while holding reset, and recovery when the debug wire stops working.
  On CH32V003 and others the NRST pin may be disabled by the option bytes ([Debug targets](debug-targets.md)).
- Detailed notes per peer (wire rest state, speed limit and so on) are in [Debug targets](debug-targets.md).

**Settings**:

1. Find the wires and turn them into labels (recommended). If you know which pins you wired, writing the labels with
   `oep settings set` is enough.

   ```sh
   oep pins oep://<probe> --wire rvswd --name mcu --save    # --wire swio for 1-wire peers
   ```

   `oep pins` talks to the peer over pairs of free pins and saves the pins that answered as `mcu.swdio` / `mcu.swclk` (`mcu.swio`
   for 1 wire). If it finds the reset line, it adds `mcu.nrst` too. The scan puts signals onto the peer, so use it when you know
   what is connected.
2. The wire's rest state and speed are normally not set. The probe and `ch32rv` choose them to fit the peer. Only to fix them for
   a known peer, use `channel.{}.rest` in [Settings](settings.md#debug-wires).

**How to use**:

- Arduino IDE: choose ArduinoCore-CH32RV's CH32 as the board and `oep://<probe>/<port>` (or the probe's serial port) as the port,
  then "Upload".
- Command line:

  ```sh
  ch32rv --probe oep://<probe>/mcu target info     # check the peer
  ch32rv --probe oep://<probe>/mcu flash app.elf   # flash, verify, reset, start
  ch32rv --probe oep://<probe>/mcu gdb             # GDB server (127.0.0.1:3333)
  ```

- From Python, `Wire` and `RiscvDm` of `oep_client.riscv` handle attach, halt, memory reads and writes, and registers
  ([Getting started](getting-started.md#debugging-a-ch32-over-rvswd)).

**Notes**:

- If the application turns the debug pins into GPIO or enters deep sleep, it does not connect. With NRST wired,
  `ch32rv recover --method nrst` brings it back.
- After closing the debug connection the peer may stay halted (when closed while halted). To let it run, resume before closing.

## 2. Watch the peer's log, send commands

**What it solves**: read the log the peer MCU outputs on the PC. Both a log sent over UART and a log sent over the debug wire
without a UART can be read. With the settings in place, opening the probe's serial port in an ordinary serial monitor is enough
for the log to flow.

There are 3 ways.

| Way | How the peer outputs | Wiring | Direction |
|---|---|---|---|
| A. Receive over UART | the peer's UART (`Serial.print` and so on) | peer TX → probe RX, (to send) probe TX → peer RX, GND | both |
| B. Receive over the debug wire | the debug module's mailbox (CH32's `sdi`, `dmdata`, `dmseq`) | the same debug wire as [1](#1-flash-and-debug-a-ch32), GND | `sdi` peer → PC, `dmdata` / `dmseq` both |
| C. Pass through to a serial port | carries A or B to the probe's serial port only while there is no session | the same as A or B | peer → PC |

**A. Receive over UART**

- Connect the peer's TX to a pin that can be the probe's UART RX (any pin on ESP32, only fixed pairs on RP2; [Boards](boards.md)).
- The speed (baud) and format (8N1 and so on) are given by the tool when starting to receive. They are not probe settings.
- Read with Python's `FixtureUartIO` or `ch32rv monitor --source uart`. The log read accumulates in the probe, and the rest can be
  read later.

**B. Receive over the debug wire**

- For peers that cannot free their UART pins or use the UART for something else. The peer's sketch outputs through the output
  the core provides (ArduinoCore-CH32RV's
  [debug output](https://github.com/ch32-riscv-ug/ArduinoCore-CH32RV/blob/main/docs/debug-output.ja.md)).
- Read with `ch32rv monitor --source dmseq` (or `sdi`, `dmdata`), which can also send input. Flashing and showing the log work on
  the same port.

**C. Pass through to a serial port**

- Use it when you want to see the log just by opening the probe's serial port in the Arduino IDE's serial monitor or a terminal
  program.
- Settings (once saved, they apply at every boot):
  - Carry the UART: `passthrough.<port>.rx`, `.baud`, `.format` of `oep.fixture.uart`.
  - Carry the debug wire's log: `passthrough.<port>.wire`, `.swdio`, `.swclk`, `.mechanism` of `oep.target.console`.
  - `<port>` is the name of the probe's serial port (`cdc0` and so on) ([Settings](settings.md#pass-through)).
- Pass-through stops while another tool uses the probe and comes back when it ends. A flash tool works on the same port as is.
- Pass-through is peer → PC only. To send to the peer, use A or B.

## 3. Look inside an ARM Cortex-M

**What it solves**: connect to an ARM Cortex-M over SWD, read and write memory and peripheral registers, halt the core and run
it. From Python you can also call functions placed in the peer's RAM.

**What to prepare**: a probe with SWD ([Boards](boards.md#what-each-chip-can-do)), the peer Cortex-M, Python's `oep_client`.

**Wiring**: the peer's SWDIO and SWCLK (on STM32 mostly PA13 / PA14), GND. NRST if needed.

**How to use**: use `SwdWire` (attach; give TARGETSEL for multidrop peers), `ArmAdi` (DP / AP), `MemAp` (memory) and `CortexM`
(halt, resume, registers, function calls, system reset) of `oep_client.arm`.

**Notes**: flash writing and a GDB server exist only for an RP2040 / RP2350 peer (`python -m oep_client.rp2040_swd flash`,
`oep_client.gdb_server`). Write the flash of other Cortex-Ms yourself from Python.

## 4. Capture waveforms

**What it solves**: as a logic analyzer, capture signal changes with their times, keep them in a file, and decode UART, SPI,
I2C, CAN, LIN, CH32 debug wires and so on. Analog values can be captured together (on supporting boards).

**What to prepare**: a board that supports capture (rates and channel counts in [Boards](boards.md#what-each-chip-can-do);
ESP32-P4 / P4X for fast signals), WireSkein.

**Wiring**: connect the signals to watch to probe pins, and connect GND. Capture only listens, so pins used by other features can
be captured too.

**How to use**:

```sh
wireskein capture --source oep:oep://<probe> --channels SDA=GPIO47,SCL=GPIO48 --rate 20M --samples 200k -o i2c.wireskein
wireskein gui i2c.wireskein          # show in a browser
wireskein convert i2c.wireskein i2c.sr   # to open in PulseView
```

How to capture differs between using capture alone and combining it with debug or fixture labels. The differences are summed up
in [Capture: which way](capture.md).

## 5. Talk to sensors and ICs over I2C / SPI

**What it solves**: without writing an MCU program, communicate with I2C / SPI peers (sensors, EEPROMs, displays and so on) from
the PC, read and write registers, and check the peer's behavior.

**What to prepare**: a probe with an I2C / SPI controller, the peer part, Python's `oep_client`.

**Wiring**:

- I2C: SDA, SCL, GND. Pull-up resistors (a few kΩ) are needed. Add them if the peer's board has none.
- SPI: SCK, MOSI, MISO, CS, GND.

**How to use**: use `oep.fixture.i2c-controller` / `oep.fixture.spi-controller` from Python. The address, speed and SPI mode are
conditions given per transfer, not probe settings. The probe's I2C / SPI are made with GPIO, so the speed limits are as in the
table in [Boards](boards.md#what-each-chip-can-do).

## 6. Test your MCU's I2C / SPI driver

**What it solves**: the probe plays the peer of the MCU under development (the controller). Without a real sensor, decide the
values to return and test the driver. Write and read requests from the controller accumulate in the probe and can be read later.

**What to prepare**: a probe with an I2C / SPI target ([Boards](boards.md#what-each-chip-can-do)), the MCU under development,
Python's `oep_client`.

**Wiring**: the same lines as [5](#5-talk-to-sensors-and-ics-over-i2c--spi), between the MCU under development and the probe.
Put the I2C pull-ups on either side.

**How to use**: use `oep.fixture.i2c-target` / `oep.fixture.spi-target` from Python. Decide your own address for I2C or the mode
for SPI, load the data to return in advance, and wait.

## 7. Drive and measure signals

**What it solves**: from the PC, move a pin instead of pressing a switch, read the peer's outputs, try an LED or motor driver with
PWM, measure frequencies, output and capture analog voltages.

| Feature | Interface | What it does |
|---|---|---|
| GPIO | `oep.fixture.gpio` | input (with or without pull), output high / low, open drain (for reset lines and the like) |
| PWM | `oep.fixture.pwm` | output of frequency and duty |
| Counting pulses | `oep.fixture.counter` | the number of edges in a set time, and the frequency from it |
| Analog output | `oep.fixture.dac` | only on boards with a DAC (classic ESP32 and others) |
| Analog capture | `oep.fixture.analog` | captures voltage values at a set rate |

**Wiring**: the pins used and GND. Do not drive outputs against the peer's outputs.

**How to use**: use Python's `oep_client.fixture` ([Getting started](getting-started.md#gpio-and-uart-a-test-fixture)). A pin
used by one feature cannot be used by others, so test code never drives two outputs against each other. Pins that are done go
back to their idle state. To decide the idle state, use `channel.{}.idle` in
[Settings](settings.md#idle-states-and-forbidden-pins).

## 8. Make an automated test fixture

**What it solves**: repeat flashing, resets, log checks, signal I/O, and waveform capture and checking from test code. Results
stay as pass/fail with measured values and can run in CI.

**What to prepare**: a probe for the fixture (an ESP32-P4 / P4X with many channels suits it), the peer board, pytest,
`oep_client`, WireSkein and pytest-embedded-wireskein. For CH32, ch32rv and ArduinoCore-CH32RV's test tools.

**Wiring**: always connect the debug wire, the standard UART, NRST and the BOOT-type pins (if the peer has them), and connect as
many other pins as you can. This way the lines of features you want to test later are already connected.

**Settings**: make the installed settings of [10](#10-install-the-probe-as-a-fixture). With the wiring written in labels, test
code finds lines by names such as `mcu.rx` instead of pin numbers.

**How to use**: in a test, take the probe's session, use the needed features, and release it at the end. When the session ends,
all the pins and connections that test assigned are released, so nothing carries over to the next test. Waveforms become test
results through WireSkein's record and checks
([WireSkein's capture tests](https://github.com/Open-Embedded-Probe/wireskein/blob/main/docs/capture-test-guide.md)).

## 9. Use it from a distance

**What it solves**: use the probe from a PC on the same network without connecting USB.

**What to prepare**: a probe with Wi-Fi (classic ESP32).

**Settings**: set `wifi.<number>.ssid` and `wifi.<number>.passphrase` from the serial port and save
([Getting started](getting-started.md#6-over-wi-fi-classic-esp32)). For a fixed address, use `ipv4.<netif>.*`.

**How to use**: find it with `oep find` and open it with `oep://<probe>?via=net`. Every tool uses the same address.

**Notes**: OEP has no authentication. Use it only on a trusted network or inside an authenticated tunnel.

## 10. Install the probe as a fixture

**What it solves**: it becomes a fixture used in the same state at every boot. The wiring names, the idle states of unused pins,
the pins kept untouched and the log pass-through are the same from every PC.

**Order of settings**:

1. **Decide the pins to keep untouched**: make pins connected to parts on the board (LED, buttons, boot straps)
   `channel.{}.disable`.
2. **Name the wiring**: give labels with a port name per peer (`oep pins --save` or `oep settings set`).
3. **Decide the idle states**: give `channel.{}.idle` to lines where the peer's input must not float (BOOT0, reset, chip
   selects and so on).
4. **Decide the log pass-through** (if used): C of [2](#2-watch-the-peers-log-send-commands).
5. **Save and back up**: make it apply at every boot with `oep settings save`, and back it up to a file with
   `oep settings export`. After updating the firmware or when moving to another unit, restore it with `oep settings import`.

Put the peer's part number and notes in the probe's memo area (`oep.probe.memo/port/<port>`), and every PC reads the same record.

What each item means and when it takes effect is in [Settings](settings.md).

## What it does not do

- OEP's debug wires are RVSWD, SWIO (WCH RISC-V) and SWD (ARM). JTAG, flashing ESP32 or AVR, and other proprietary wires are not
  handled.
- The flashing procedure per peer belongs to the PC tool (ch32rv and others). Being connected to a debug wire does not mean any
  peer can be flashed.
- The probe neither converts levels nor switches power. Use external circuits if needed.
