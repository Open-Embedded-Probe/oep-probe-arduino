# Capture: which way

[日本語](capture.ja.md)

An OEP probe can capture signals as a logic analyzer. What you prepare, the settings and the steps differ between using the
probe **for capture only** and **combining it with debug or fixture labels**. This document sums up the differences.
How many channels each board captures is in [Boards](boards.md#what-each-chip-can-do), and the ESP32-P4 / P4X rate limits are
in [the section below](#esp32-p4--p4x-speeds-and-lines).

## Which to choose

| Way | Good for | Settings on the probe | How channels are named |
|---|---|---|---|
| [A. Capture only](#a-capture-only) | watching signals of the circuit at hand, decoding protocols | none | the probe's pin names (`GPIO47`) |
| [B. Capture by labeled wiring](#b-capture-by-labeled-wiring) | observing the wires connected for debugging or a fixture as they are | wiring labels (persistent settings) | the port's line names (`mcu.swdio`, `mcu@pb6`) |
| [C. Capture together with operations](#c-capture-together-with-operations) | recording what happened during an operation such as flashing, reset or communication | the same as B | the same as B |
| [D. Check in tests](#d-check-in-tests) | judging automatically whether waveforms are as expected | the same as B | the same as B |

## Common points

- **Listen only**: capture does not drive pins. Pins used by other features (debug wires, UART, SPI and so on) can be captured
  as they are.
- **Analog is different**: analog capture switches its pins to the analog function while it runs, so they cannot be shared
  with other features. To watch the same signal with both logic and analog, wire it to two pins.
- **Kinds of capture**:
  - One-shot (`--mode oneshot`): captures a fixed number into the probe's memory. You can give a trigger (edge or level) and
    how many to keep before it (pretrigger).
  - Streaming (`--mode stream`): sends to the PC while capturing. It can capture longer than the probe's memory. The rate
    depends on the transport (the USB speed).
  - Repeat (`--mode repeat`): repeats a one-shot at every trigger and gathers them into one file.
- **Transport**: USB suits reading out fast captures (the ESP32-P4's HS vendor bulk; `oep://<probe>?via=usb`). A probe
  connected over UART (classic ESP32) raises the link speed only while reading out (WireSkein chooses it automatically).
- **Files**: captures are kept in WireSkein's format (`.wireskein`). They contain the rate actually used, the trigger
  position, the probe's time and the channel names. `wireskein convert` turns them into PulseView's `.sr` or VCD.

## ESP32-P4 / P4X speeds and lines

The probe captures at the sustainable rate closest to the requested one and answers the rate actually used (`actual_rate` in
configure's answer; it is also kept in the WireSkein file). Before capturing, ask the probe with query (an inquiry that does
not change the settings) to learn the rate actually available for that line count and mode. In Python it is the `rate` of the
answer of `oep_client.capture.LogicCapture(hst).configure(query=True, rate=…, mode=…)` (ask after fixing the line count with
the plan). The tables below show how that answer is decided.

### Available rates

- From 627.451 kHz to 160 MHz. Of the values 160 MHz divided by integers and fractions, the one closest to the requested value
  is used. Values in 10 MHz steps such as 60 MHz can be requested too.
- At rates that divide 160 MHz by an integer (160, 80, 53.3, 40, 32 MHz and so on), the sample intervals are even. Other rates
  use fractional division, and the sample times sit on the 160 MHz grid (6.25 ns), so the intervals are uneven. Capturing a
  crystal-driven square wave and fitting its rising edges to an ideal grid, the largest deviation is under half a sample + 1 ns
  for integer division, and half a sample + 2-5.5 ns for fractional division (60, 64, 70, 90, 100, 120 MHz). The answer's
  `actual_rate` is the average rate and does not show this jitter. For captures that look closely at timing, choose an
  integer-division rate.

### Line count and sample width

The number of bits in one sample (the width) depends on the number of lines captured. The streaming limit depends on this
width.

| Lines | 1 | 2 | 3-4 | 5-8 | 9-16 |
|---|---|---|---|---|---|
| Width (bits) | 1 | 2 | 4 | 8 | 16 |

### Limits per mode

A rate above the limit is captured at the closest rate within the limit (not refused).

| Mode | Limit | Notes |
|---|---|---|
| One-shot (no trigger) | 160 MHz (1-16 lines) | |
| One-shot (with trigger) | 100 MB/s. 8 lines 100 MHz, 16 lines 50 MHz (up to 4 lines 160 MHz) | the first capture after boot stops midway at 120 MB/s, so the limit is set below that |
| Repeat | the one-shot limits do not apply | |
| Streaming (USB HS, when enough send segments can be allocated) | 40 MB/s. Up to 2 lines 160 MHz, 4 lines 80 MHz, 8 lines 40 MHz, 16 lines 20 MHz | the same together with the dmseq console |
| Streaming (through the ch32rv broker, P4X) | 40 MB/s (4 lines 80 MHz) | 35 MB/s (4 lines 70 MHz) together with the dmseq console. At 40 MB/s segments are dropped |
| Streaming (little internal RAM, small send segments) | 36 MB/s. 8 lines 36 MHz | 8 lines at 40 MHz lose data |
| Streaming (a transport that cannot use segments, not enough internal RAM) | 8.5 MB/s. 8 lines 8.42 MHz | |
| Streaming, per-line resolution (below) | 13.4 MB/s with the width at 8. 13.3 MHz | a value set from the worst case |

- Which limit applies depends on the probe's memory and the transport at the time. The actual rate is answered in
  `actual_rate`, and how much can be held before sending in `stream_buffer`.

### Per-line resolution

With data format 1 (logic blocks, oep-if-capture §1.3), the way of carrying can be chosen per line.

- **raw**: carries every sample.
- **Reductions**: carry one value per interval of D samples (2 or more). `sample` is the first sample of the interval,
  `any_high` / `any_low` whether that level was in the interval, and `change_latch` the last level of the interval and whether
  it changed within the interval. With change_latch a pulse shorter than the interval still shows as a change; only the
  position within the interval is lost. Reducing a slow line such as CS cuts the amount carried.
- In Python, give the lines to reduce, the reduction and D like
  `oep_client.acquire.Request(..., resolution={"CS": ("change_latch", 64)})`. The other lines are raw.
- Available up to 8 lines. In streaming, only on transports that can use send segments.

### Using it together with other operations

- While streaming, the capture client keeps reading (inside `during`, `wait` reads). If you wait inside `during` for another
  process to finish (such as a ch32rv write), nothing is read meanwhile. Data beyond what the probe or the broker can hold
  (configure's answer's `stream_buffer` and so on) is lost. Run long operations that do not use the session with
  `wait.beside(fn)` (it runs them in another thread and keeps reading until they finish).

## A. Capture only

Use the probe as a logic analyzer. The probe needs no settings.

**What to prepare**: a probe that supports capture, WireSkein (`pip install "wireskein[oep]"`).

**Wiring**: the signals you want to watch to free pins of the probe, with a common GND. Avoid pins connected to parts on the
board (LED, buttons, boot straps).

**Steps**:

1. Check the pin names: `oep dump --port oep://<probe>` shows the pins that can be captured (`GPIO47` and so on).
2. Capture:

   ```sh
   wireskein capture --source oep:oep://<probe> \
       --channels SDA=GPIO47,SCL=GPIO48 --rate 20M --samples 200k \
       --trigger SCL:fall --pretrigger 1000 -o i2c.wireskein
   ```

   The `NAME` of `NAME=ID` is the channel's name in the file. You give it yourself on every capture.
3. View and decode:

   ```sh
   wireskein gui i2c.wireskein                 # show in a browser
   wireskein analyze i2c.wireskein             # find buses and decode (UART, SPI, I2C, CAN, LIN and so on)
   wireskein decode i2c.wireskein --spec '{"buses":[{"protocol":"i2c","pins":{"sda":"SDA","scl":"SCL"}}]}'
   ```

**What you decide (given on every capture)**: channels, rate, sample count (time when streaming), trigger, pretrigger. None of
them stays on the probe.

## B. Capture by labeled wiring

On a probe whose wiring has names (labels) for debugging or a fixture, channels can be named by line name. When the wiring
changes, fixing the labels lets the same command capture.

**What to prepare**: A, plus wiring labels.

**Settings (once, saved on the probe)**:

- Debug wires can be found and turned into labels with `oep pins --wire <rvswd|swio> --name <port> --save`.
- Other wires are written like `oep settings set <probe> core channel.GPIO12.label=mcu.rx@pa9 channel.GPIO6.label=mcu@pb6 --save`.
  The naming rules are in [Settings](settings.md#pin-labels-port-and-line-names).

**Steps**:

```sh
wireskein capture --source oep:oep://<probe> \
    --channels SWDIO=mcu.swdio,SWCLK=mcu.swclk,RX=mcu.rx --rate 40M --samples 1M -o debug.wireskein
wireskein analyze debug.wireskein           # CH32 debug wires (RVSWD, SWIO) can be decoded too
```

**Differences from A**:

| | A. Capture only | B. Labeled |
|---|---|---|
| Probe settings | none | labels (saved, persistent) |
| How channels are named | `GPIO47` | `mcu.swdio`, `mcu@pb6` (case-insensitive) |
| When the wiring changes | fix the pins in the command | fix the labels. The command stays |
| Other PCs, other tools | a person passes on the pin mapping | the same names point at the same lines |

## C. Capture together with operations

Capturing while flashing, resetting, connecting the debugger, doing I2C transfers and so on records what the operation did to
the lines. Operate with other pins of the same probe while capturing those lines (with B's labels, the operation and the
capture use the same names).

**Steps (Python)**: write it so the operation runs during the capture.

```python
from oep_client import acquire, core, link, riscv
from wireskein import capture as wc
from wireskein.sources import oep as wso

hst = link.open_host("oep://<probe>?via=usb")
core.take(hst, 30000, owner="capture-during-attach")
try:
    w = riscv.Wire(hst, "oep.wire.rvswd")
    def steps(wait):
        wait(0.01)
        conn, _ = w.attach(halt=True, pins=(18, 17))   # an operation during the capture
        w.detach(conn)
        wait(0.01)
    res = acquire.capture(hst, acquire.Request([("dio", "GPIO18"), ("clk", "GPIO17")], 40_000_000, 0,
                                               mode="stream", during=steps))
    wc.write("attach.wireskein", wso.capture_of(res))
finally:
    hst.end()
```

- If the operation uses a trigger in the form "arm, then operate", the event the operation causes fires the trigger.
- The classic ESP32's sampler stops the SWIO wire during its capture window. Use a trigger to capture the operation's effect
  ([Getting started](getting-started.md#capturing-signals-esp32-p4)).
- To cut logic and analog on the same trigger, use capture-group (giving `--analog` and `--trigger` together makes WireSkein
  group them).

## D. Check in tests

Besides capturing waveforms, judge in tests whether they are as expected. Write the test steps and expectations in WireSkein's
record (runlog) and check them with `wireskein verify`. In pytest, pytest-embedded-wireskein handles the per-test recording and
checking.

- Expectations are written as `square` (period and duty), `uart`, `i2c`, `voltage` and so on. The result is pass/fail with the
  measured values, and also JUnit XML.
- Naming lines by label keeps the test code unchanged when the fixture's wiring changes.
- The steps are in [WireSkein's capture tests](https://github.com/Open-Embedded-Probe/wireskein/blob/main/docs/capture-test-guide.md).
