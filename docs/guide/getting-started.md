# Getting started

[日本語](getting-started.ja.md)

From an empty board to a probe a PC drives, in 4 stages: flash the firmware, install the host library, find the probe, use it.
The Arduino IDE is not needed.

## 1. Flash the firmware

Take the firmware for your chip from the [Releases](https://github.com/Open-Embedded-Probe/oep-probe-arduino/releases)
(`firmware-<version>.json` lists the files and their sha256).

| Chip | File | How to write |
|---|---|---|
| RP2040 (Pico, RP2040-Zero and so on) | `OepProbe-rp2040-<version>.uf2` | plug in while holding BOOTSEL and copy to the drive that appears |
| RP2350 (Pico 2 and so on) | `OepProbe-rp2350-<version>.uf2` | the same |
| SparkFun Pro Micro RP2350 | `OepProbe-rp2350-<version>.uf2` (the same as Pico 2) | the same pins as Pico 2 (GP19 is the PSRAM select, so the host does not use it) |
| ESP32-P4 | `OepProbe-esp32p4-<version>.merged.bin` | `esptool.py --chip esp32p4 write_flash 0x0 <file>` |
| ESP32-P4X (v3.x) | `OepProbe-esp32p4x-<version>.merged.bin` | the same (esptool's chip name is `esp32p4`) |
| classic ESP32 (DevKitC and so on) | `OepProbe-esp32-<version>.merged.bin` | `esptool.py --chip esp32 write_flash 0x0 <file>` |
| ESP32-S3 | `OepProbe-esp32s3-<version>.merged.bin` | `esptool.py --chip esp32s3 write_flash 0x0 <file>` |
| WCH CH32X035 / CH32V307 / CH32H417 | `OepProbe-ch32x035-<version>.bin` / `-ch32v307-` / `-ch32h417-` | from another OEP probe (or WCH-LinkE) with [ch32rv](https://github.com/ch32-riscv-ug/ch32rv), from the start of the code flash: `ch32rv flash --probe oep://<unit_id of the writing probe>/<port> <file> --offset 0x08000000` (below) |

No wiring is built into the firmware. The host decides which pins to use when it uses the probe. The same file serves every
fixture. Other chips (ESP32-C3, C6 and so on) have no released firmware. Build it yourself ([Boards](boards.md)).

Flash it before use. You cannot tell what is on a board.

A CH32 probe is written by connecting it to another probe's SWIO / RVSWD. `<port>` is the port name of the labels given to that
CH32's wires on the writing probe (`<port>.swdio`, `<port>.swclk` and so on). It can also be written from the chip's factory
bootloader (USB or UART) (`ch32rv isp`).

An ESP32-P4 running OepProbe can be updated over the HS port alone: `dfu-util -D OepProbe-esp32p4-<version>.bin` (the app image,
not merged) writes to the other app area, checks it, and restarts. Settings remain (to start clean, `oep settings unset --all
<port>` then save). The new firmware stays once it boots: no later reset goes back to the previous firmware. When the bootloader
did not boot the new firmware, describe's firmware string says so after the version (for example `<version> (update to app1 did
not reach setup: software)`). A reset that was neither power on nor the probe's own restart shows there the same way
(`brownout at 3 s`). A chip that was never flashed, or that does not boot, is flashed with esptool over USB-Serial/JTAG as above.

The P4 and P4X firmware are different. P4X is built with the `esp32p4x` profile, and DFU uses
`OepProbe-esp32p4x-<version>.bin`. In `oep dump --port <port>`, model is `esp32p4` / `esp32p4x`, and chip is the actual
revision read from eFuse (for example `esp32p4 v3.2`). The HS USB product names also differ: `OEP probe (ESP32-P4)` /
`OEP probe (ESP32-P4X)`. The app image range of the Arduino core pinned in `sketch.yaml` is v0.1–v1.99 for P4 and v3.1–v3.99
for P4X (v3.0 is not covered). The image header Arduino generates does not carry this restriction, so the firmware embeds the
model and the supported revisions as a
[custom app descriptor](https://docs.espressif.com/projects/esp-idf/en/v5.5.5/esp32p4/api-reference/system/app_image_format.html#adding-a-custom-structure-to-an-application).
A DFU on firmware with this check refuses a mismatched, out-of-range or old image without identification, puts the boot target
back, and does not restart. A DFU from firmware without this check writes without checking, so check the file before updating.
To write an image without the check, use esptool over USB-Serial/JTAG.

## 2. Install the host library

```sh
pip install oep-client-python
```

This installs the `oep` command and Python's `oep_client`. Other hosts (ch32rv, ArduinoCore-CH32RV's upload) speak the same
protocol. This guide uses the Python one.

What `--port usb` in the next step needs (`libusb1` for asynchronous vendor bulk transfers, `hidapi` for HID connections) is
installed with it. On Windows, if the probe offers HID, it can connect over HID without changing the driver to WinUSB.

On Linux, opening the probe's vendor bulk, DFU and HID interfaces as an ordinary user needs the udev rule shipped by
oep-client-python, [`udev/70-oep-probe.rules`](https://github.com/Open-Embedded-Probe/oep-client-python/blob/main/udev/70-oep-probe.rules)
(serial ports need only the usual `dialout` group). Install it once with administrator rights:

```sh
sudo install -m 0644 udev/70-oep-probe.rules /etc/udev/rules.d/   # inside an oep-client-python checkout or sdist
sudo udevadm control --reload-rules && sudo udevadm trigger   # or unplug and replug the probe
```

## 3. Find the probe and see what it has

```sh
oep dump --port /dev/ttyACM0        # the probe's serial port (COM5 on Windows)
oep dump --port usb                 # or the first OEP probe on USB (vendor bulk / HID interface)
```

`dump` lists the interfaces the probe has, with names, pins, limits and features. It needs no lock, so it does not get in the
way of another program using the probe. It also shows the list of transports (ESP32-P4: HS vendor bulk, USB-Serial/JTAG, HID,
the CDC port, UART bridge; RP2040 / RP2350: the CDC port, vendor bulk, HID).

A USB probe enumerates with the project's VID:PID `1209:4F45`, and its USB serial number is the unit_id. The host finds probes
by this VID:PID (`usb`), and finds a probe named by unit_id (`usb:<unit_id>`) by serial number. iProduct `OEP probe (...)` is a
name for people to read. Ports with fixed IDs (USB-UART bridge, the ESP32-P4's USB-Serial/JTAG) are used by naming them.

## 4. Use it

The host takes the session lock, decides the needed pins with the plan, uses the interfaces, and ends the session. The lock
keeps two programs from driving the probe at the same time. When the lease (5 seconds below) runs out, the lock is released
even if the program crashed. When the session ends or the lease runs out, everything the session made (the plan, its share of
connections and console streams, subscriptions) is released, and nothing passes to the next program. What should remain
between programs goes in the settings (pass-through carries the target's console or UART to the probe's serial port while there
is no session). A console reopened at the same place returns its stream together with what was read.

### GPIO and UART (a test fixture)

```python
from oep_client import core, link
from oep_client.fixture import FixtureUartIO, Gpio

hst = link.open_host("/dev/ttyACM0")
core.take(hst, 5000, owner="my-test")               # the lock. Takes over a lock left by a crashed program

gpio = Gpio(hst)
uart = FixtureUartIO(hst)
core.plan_apply(hst, [(gpio.fn, 1, 15),             # (fn, role, channel): the gpio line to pin 15
                      (uart.uart.fn, 1, 1), (uart.uart.fn, 2, 0)])   # the uart's RX to pin 1, TX to pin 0
gpio.set([(15, Gpio.OUTPUT_HIGH)])
print(gpio.read([15]))
uart.configure(115200)
uart.write(b"hello\r\n")
print(uart.read(64))
hst.end()
```

A pin held by one interface is refused to the others (`rejected: unavailable`). A test never drives two outputs against each
other.

### Debugging a CH32 over RVSWD

```python
from oep_client import core, link, riscv

hst = link.open_host("/dev/ttyACM0")
core.take(hst, 5000, owner="my-debugger")
wire = riscv.Wire(hst)                               # oep.wire.rvswd
found = wire.scan()                                  # tries every free pin pair: where does a debug module answer
conn, dmstatus = wire.attach(halt=True, pins=found[0].pins)
dm = riscv.RiscvDm(hst, conn)
print(dm.read_block(0x08000000, 4).hex())            # the first words of flash
dm.resume()
wire.detach(conn)
hst.end()
```

The host states what the target needs from the wire (rest state, speed). For CH32L103, pass `idle_clock="low"` and
`max_speed=1_000_000` to attach (max_speed is required). To reset the target with an NRST pin, name that channel in attach's
reset TLV (`wire.attach(halt=True, reset=(channel, hold_ms))`: halts before the first instruction; with `halt=False` it keeps
running). There is no default reset line.

Flashing, the console and so on are the host tools' work. ch32rv and ArduinoCore-CH32RV's upload do them over OEP.

### Capturing signals (ESP32-P4)

```python
from oep_client import capture, core, link

hst = link.open_host("usb")
core.take(hst, 5000, owner="my-capture")
lc = capture.LogicCapture(hst)
core.plan_apply(hst, [(lc.fn, 0, 20), (lc.fn, 1, 21)])   # channel 0 to GPIO20, channel 1 to GPIO21
cfg = lc.configure(rate=20_000_000, samples=200_000)
lc.start()
(segment,) = lc.wait()
data = lc.read_segment(segment)
lc.to_sr("capture.sr", data, segment.samples)          # opens in PulseView
hst.end()
```

Up to 16 channels, on any pins. Pins held by other interfaces can be captured too (capture only listens), so you can capture
while watching the SPI device under test. The rate limits are in the table in [Capture](capture.md#esp32-p4--p4x-speeds-and-lines).

To catch an event, give a one-shot a trigger (the level or edge of one channel) and how much to keep before it:

```python
lc.configure(rate=20_000_000, samples=200_000,
             trigger=(capture.EDGE, 1, 1),   # falling edge of channel 1 (value 0 rising, 1 falling, 2 both)
             pretrigger=1_000)               # keep the 1000 samples before it
lc.start()                                   # waits for the trigger
(segment,) = lc.wait(timeout=30)             # lc.force() starts without a trigger
data = lc.read_segment(segment)              # the trigger is sample number segment.trigger_index
```

The classic ESP32's sampler can trigger too (8 channels, 1 kHz-300 kHz). It samples with interrupts stopped, so it searches in
slices of up to 250 ms. Edges that come in the gap between slices (about 1 ms) are missed. An immediate capture window and the
SWIO wire do not run at the same time: while sampling, no SWIO frame goes out (requests (riscv-dm, wire ops) wait for the window
to end, and the console does not read). The window waits for the request in progress to end. The trigger search gives the wire
turns within a slice and keeps sampling meanwhile (at once for a waiting request, up to 5 ms per slice for the console). What
the target does because of commands or resets sent during the search is sampled. If the trigger fires before the pretrigger has
filled, the segment is short and trigger_index is small. An immediate capture window lasts samples ÷ rate (at most 200 ms:
configure rounds samples down to 200 ms worth; 60000 at 300 kHz), so console commands or debug resets sent after the start reach
the target after the window and do not show in the capture. To capture what the target does after something you send, use a
trigger: arm, then send a command or reset; it arrives within a search turn, and the event it causes fires the trigger
([implementation limits §4.1](../implementation-limits.md#41-classic-esp32)). One interrupts-off stretch is bounded by the clock:
a segment whose samples were late (frames of a request running at the time of the trigger) ends early, with fewer samples and
slipped set. The sampler has core 1 to itself (the firmware is built with LoopCore=0, EventsCore=0, and OEP, the Wi-Fi driver
and the TCP/IP stack are on core 0), so these limits do not change with Wi-Fi on. Radio traffic can delay samples (slipped).

To capture logic and analog together on a logic event, use a capture-group event source (ESP32-P4). Configure every track with
a linked trigger, and the group's event cuts every segment at the same instant.

```python
an = capture.AnalogCapture(hst)
grp = capture.CaptureGroup(hst)
core.plan_apply(hst, [(lc.fn, 0, 20), (lc.fn, 1, 21), (an.fn, 0, 16)])
linked = (capture.LINKED, 0, 0)
lc.configure(rate=20_000_000, samples=200_000, trigger=linked, pretrigger=1_000)
an.configure(rate=40_000, samples=4_000, trigger=linked, pretrigger=40)
edge = capture.Cond.edge(lc, 1, 1)                          # falling edge of logic channel 1
grp.bind([lc, an], capture.TriggerSource([capture.Stage([[edge]])]))
grp.start()
st = grp.wait(timeout=30)                                   # st.trigger: (the judging fn, time ns, ±ns)
(logic,), (analog,) = lc.segments(), an.segments()          # logic.trigger_index, analog.trigger_index: that instant
```

Every track starts with the group and keeps capturing until the event. pretrigger is that track's own sample count. The RP2's
PIO logic and the classic ESP32's sampler have no linked trigger, so they cannot be in a group with an event source
([Boards](boards.md#what-each-chip-can-do)).

## 5. Give the probe the fixture's settings

A jig = the firmware + its settings. Write them once and save, and the probe applies them again at every boot.

```sh
oep settings set /dev/ttyACM0 core channel.GPIO2.label=dut.swdio channel.GPIO3.label=dut.swclk   # port dut (oep://<probe>/dut)
oep settings set /dev/ttyACM0 oep.target.console/0 passthrough.cdc0.wire=oep.wire.rvswd/0 passthrough.cdc0.swdio=GPIO2 \
    passthrough.cdc0.swclk=GPIO3 passthrough.cdc0.mechanism=dmseq
oep settings set /dev/ttyACM0 core channel.GPIO5.idle=pull-up channel.GPIO28.disable=true channel.GPIO29.disable=true --save
oep settings show /dev/ttyACM0
oep settings export /dev/ttyACM0 jig.oep-settings.json   # export before updating the firmware, import after the update
```

Now, while there is no session, the target's console shows in a terminal on the probe's serial port. A flash tool keeps
speaking OEP on the same port (pass-through stops meanwhile). Each item is explained in `06.Settings/Settings`.

## 6. Over Wi-Fi (classic ESP32)

Once on a network, the classic ESP32 firmware also speaks OEP on TCP port 7450 (up to 3 connections at once). Which network to
join is a setting, not a build option: put up to 4 entries in the core settings' `wifi.{}.ssid` and `wifi.{}.passphrase`
(oep-spec transports §7.1; this implementation's values are in [implementation limits §6.3](../implementation-limits.md)), and
they are tried in number order. Set and save them from the serial port. The passphrase is write-only and appears neither in
`oep settings show` nor in any answer:

```sh
oep settings set /dev/ttyUSB0 core wifi.0.ssid=lab wifi.0.passphrase=... --save
```

Once connected, the probe announces itself over mDNS (`oep-<unit_id>.local`, service `_oep._tcp`, TXT `unit_id`). The address
is also available from fn 0's network (`oep settings network <port>`):

```sh
oep dump --port tcp://oep-<unit_id>.local:7450
oep linktest tcp://<address>:7450
```

Use it only on a trusted network (or inside an authenticated tunnel). OEP has no authentication. The serial port keeps working,
and every connection shares one lock. The smallest example is `03.Transports/WifiTcp`.

## Next

- [What OEP does for you](use-cases.md): per thing you want to do, what to prepare, wiring, settings. [Debug targets](debug-targets.md),
  [Capture: which way](capture.md), [Settings](settings.md).
- [Firmware features](firmware-features.md): leave features out of the released firmware, add your own interface.
- [Your first interface](your-first-interface.md): your own interface, step by step.
- [Writing a probe](writing-a-probe.md): the detailed guide to the library's internals.
- [Boards](boards.md): what each chip can do, building for boards without a released firmware.
- Protocol: [oep-spec](https://github.com/Open-Embedded-Probe/oep-spec).
