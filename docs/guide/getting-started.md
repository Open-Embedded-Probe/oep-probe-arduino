# Getting started

[日本語](getting-started.ja.md)

From a bare board to a probe your PC drives, in four steps: flash the firmware, install the host library, find the probe,
use it. Nothing here needs the Arduino IDE.

## 1. Flash the firmware

Take the firmware for your chip from the [Releases](https://github.com/Open-Embedded-Probe/oep-probe-arduino/releases)
(`firmware-<version>.json` lists the files with their sha256):

| Chip | File | How |
|---|---|---|
| RP2040 (Pico, RP2040-Zero, ...) | `OepProbe-rp2040-<version>.uf2` | hold BOOTSEL, plug it in, copy the file to the drive that appears |
| RP2350 (Pico 2, ...) | `OepProbe-rp2350-<version>.uf2` | the same |
| SparkFun Pro Micro RP2350 | `OepProbe-promicrorp2350-<version>.uf2` | the same (every GPIO but GP19, its PSRAM select) |
| ESP32-P4 | `OepProbe-esp32p4-<version>.merged.bin` | `esptool.py --chip esp32p4 write_flash 0x0 <file>` |
| classic ESP32 (DevKitC, ...) | `OepProbe-esp32-<version>.merged.bin` | `esptool.py --chip esp32 write_flash 0x0 <file>` |

The firmware has nothing wired in: every pin is chosen by the host when it uses the probe. The same file serves every
jig. Other chips (ESP32-S3, C3, C6, ...) have no released firmware yet: build one yourself ([Boards](boards.md)).

Flash before you use a board: you cannot tell what is on it.

Updating an ESP32-P4 that runs OepProbe needs only its HS port: `dfu-util -D OepProbe-esp32p4-<version>.bin` (the app
image, not the merged one) writes the other app partition, checks it and restarts into it; the settings stay (clear
them with `oep config erase <port>` when you want a fresh probe). If the new firmware does not get as far as enumerating, the
next reset goes back to the one before. A chip that was never flashed, or one that does not start, is flashed with
esptool on USB-Serial/JTAG as above.

## 2. Install the host library

```sh
pip install oep-client-python
```

This gives the `oep` command and the Python package `oep_client`. Other hosts speak the same protocol (ch32rv, the
ArduinoCore-CH32RV upload tool); this guide uses the Python one.

## 3. Find the probe and see what it offers

```sh
oep dump --port /dev/ttyACM0        # the probe's serial port (COM5 on Windows)
oep dump --port usb                 # or: the first OEP probe on USB (its vendor bulk / HID interface)
```

`dump` lists every interface the probe offers - by name, with its pins, limits and features - and needs no lock, so it
never disturbs another program using the probe. An ESP32-P4 shows four transports (HS vendor bulk, USB-Serial/JTAG, HID,
a CDC port); the others show one.

USB probes say iProduct `OEP probe (...)`: that is how a host finds them without opening every port.

## 4. Use it

A host takes the session lock, plans the pins it needs, uses the interfaces, and ends the session. The lock keeps two
programs from driving the probe at once; the lease (5 s below) frees it if the program dies.

### GPIO and UART (a test fixture)

```python
from oep_client import core, link
from oep_client.fixture import FixtureUartIO, Gpio

hst = link.open_host("/dev/ttyACM0")
core.take(hst, 5000, owner="my-test")               # the lock, taken by force if a crashed program left it

gpio = Gpio(hst)
uart = FixtureUartIO(hst)
core.plan_apply(hst, [(gpio.fn, 1, 15),             # (fn, role, channel): gpio's line on pin 15
                      (uart.uart.fn, 1, 1), (uart.uart.fn, 2, 0)])   # uart RX on pin 1, TX on pin 0
gpio.set([(15, Gpio.OUTPUT_HIGH)])
print(gpio.read([15]))
uart.configure(115200)
uart.write(b"hello\r\n")
print(uart.read(64))
hst.end()
```

A pin one interface holds is refused to another (`rejected: unavailable`), so a test cannot short two drivers.

### Debug a CH32 on RVSWD

```python
from oep_client import core, link, target

hst = link.open_host("/dev/ttyACM0")
core.take(hst, 5000, owner="my-debugger")
wire = target.Wire(hst)                              # oep.wire.rvswd
found = wire.scan()                                  # try every free pin pair: where does a debug module answer?
conn, dmstatus = wire.attach(halt=True, pins=found[0].pins)
dm = target.RiscvDm(hst, conn)
print(dm.read_block(0x08000000, 4).hex())            # the first words of flash
dm.resume()
wire.detach(conn)
hst.end()
```

What a target needs of its line is the host's to say: a CH32L103 wants `idle_clock="low"` and `max_speed=1_000_000`
at attach. To reset a target through its NRST pin, name the channel: `wire.attach_under_reset(channel)` - there is no
default reset line.

Flashing, the console and the rest are the host tools' job: ch32rv and the ArduinoCore-CH32RV upload do it over OEP.

### Capture signals (ESP32-P4)

```python
from oep_client import capture, core, link

hst = link.open_host("usb")
core.take(hst, 5000, owner="my-capture")
lc = capture.LogicCapture(hst)
core.plan_apply(hst, [(lc.fn, 0, 20), (lc.fn, 1, 21)])   # channel 0 on GPIO20, channel 1 on GPIO21
cfg = lc.configure(rate=20_000_000, samples=200_000)
lc.start()
(segment,) = lc.wait()
data = lc.read_segment(segment)
lc.to_sr("capture.sr", data, segment.samples)          # open it in PulseView
hst.end()
```

Up to 16 channels, on any pins - also pins another interface holds (a capture only listens), so you can watch the SPI
device you are testing: 2 channels at 160 Msps, 8 at 40 Msps, 16 at 20 Msps.

To catch an event, give the one-shot a trigger - a level or an edge on one channel - and how much to keep before it:

```python
lc.configure(rate=20_000_000, samples=200_000,
             trigger=(capture.EDGE, 1, 1),   # channel 1 falling (value 0 rising, 1 falling, 2 either)
             pretrigger=1_000)               # keep 1000 samples before it
lc.start()                                   # waiting for the trigger
(segment,) = lc.wait(timeout=30)             # lc.force() starts it without the trigger
data = lc.read_segment(segment)              # the trigger is sample segment.trigger_index
```

The classic ESP32's sampler takes triggers too (8 channels, up to 2 MHz). It samples with interrupts off, so it
searches in bursts of up to 250 ms: an edge that falls in the gap between two bursts (about 1 ms) is missed.

Logic and analog together, on the logic's trigger (the capture-group): the analog follows it, and each track's segment
marks the same instant.

```python
an = capture.AnalogCapture(hst)
grp = capture.CaptureGroup(hst)
core.plan_apply(hst, [(lc.fn, 0, 20), (lc.fn, 1, 21), (an.fn, 0, 16)])
lc.configure(rate=20_000_000, samples=200_000, trigger=(capture.EDGE, 1, 1), pretrigger=1_000)
an.configure(rate=40_000, samples=4_000, pretrigger=40)    # no trigger of its own: it follows the logic's
grp.bind([lc, an], trigger=lc)
grp.start()
st = grp.wait(timeout=30)                                   # st.trigger_ns: when it fired
(logic,), (analog,) = lc.segments(), an.segments()          # logic.trigger_index, analog.trigger_index: that instant
```

A track that follows starts with the group and keeps converting until the trigger; its pretrigger counts its own
samples. The classic ESP32's sampler can be the trigger but not follow one (its bursts leave gaps).

## 5. Keep a jig's settings in the probe

A jig is the firmware plus its settings. Write them once and save them; the probe applies them at every boot:

```sh
oep config slot /dev/ttyACM0 --name dut --wire rvswd --pins 2,3 --attach at-boot --retry 1 --mechanism dmseq
oep config bind /dev/ttyACM0 --port 0 --mode last-reset --stream slot:dut      # the target's console on this port
oep config idle /dev/ttyACM0 5 pull-up --save
oep config show /dev/ttyACM0
```

Now a terminal on the probe's serial port shows the target's console from boot, and a flash tool still talks OEP on the
same port. `06.Settings/ProbeConfig` explains each item.

## Next

- [Writing a probe](writing-a-probe.md): the library from the inside, your own interfaces.
- [Boards](boards.md): what each chip can do, building for a board with no released firmware.
- The protocol: [oep-spec](https://github.com/Open-Embedded-Probe/oep-spec).
