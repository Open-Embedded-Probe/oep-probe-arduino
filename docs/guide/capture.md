# Capture: which way

[日本語](capture.ja.md)

An OEP probe captures signals as a logic analyzer. What you prepare, set and do differs between using the probe **for
capture alone** and **capturing on the labels of a debug or test jig**. This page puts the differences side by side.
How many channels each board takes is in [Boards](boards.md#what-each-chip-does); the ESP32-P4 / P4X speed limits are in
[the section below](#esp32-p4--p4x-speeds-and-lines).

## Which to choose

| Way | Suits | Probe settings | Channels named by |
|---|---|---|---|
| [A. Capture alone](#a-capture-alone) | Look at signals in a circuit, decode protocols | none | the probe's pin names (`GPIO47`) |
| [B. On labelled wiring](#b-on-labelled-wiring) | Watch the lines a debug or test jig wired, as they are | wiring labels (a fixed jig's settings) | the port's line names (`mcu.swdio`, `mcu@pb6`) |
| [C. Along with an operation](#c-along-with-an-operation) | Record what flashing, a reset or a transfer did on the lines | as B | as B |
| [D. Checked in tests](#d-checked-in-tests) | Judge automatically whether the waveform is as expected | as B | as B |

## Common points

- **It only listens**: capture never drives a pin. It takes pins other functions use (debug lines, UART, SPI) as they are.
- **Analog is apart**: analog capture switches its pins to the analog function while it runs, so they are not shared.
  To see one signal both as logic and analog, wire it to two pins.
- **Modes**:
  - One shot (`--mode oneshot`): a set number of samples in the probe's memory, with a trigger (edge or level) and the
    samples kept before it (pretrigger).
  - Streaming (`--mode stream`): sent to the PC as taken, longer than the probe's memory; the link (USB speed) sets the rate.
  - Repeat (`--mode repeat`): a one shot per trigger, gathered in one file.
- **Link**: fast captures read back best over USB (the ESP32-P4's HS vendor bulk, `oep://<probe>?via=usb`). A probe on
  a UART (classic ESP32) raises its link speed for the read back only (WireSkein picks the rate).
- **File**: a capture is kept as WireSkein's format (`.wireskein`), with the rate actually used, the trigger position,
  the probe's clock and the channel names. `wireskein convert` makes PulseView's `.sr` or VCD.

## ESP32-P4 / P4X speeds and lines

The probe captures at the rate it can keep nearest to the one asked and answers the rate it used (`actual_rate` in the
configure answer; the WireSkein file keeps it too). Before capturing, a query (an ask that sets nothing) tells the rate
this line count and mode get: in Python the `rate` of `oep_client.capture.LogicCapture(hst).configure(query=True,
rate=…, mode=…)`, asked once the plan has set the lines. The table below is how that answer comes about.

### Rates

- 627.451 kHz to 160 MHz: the value nearest the one asked among 160 MHz divided by a whole number and a fraction. A rate
  in 10 MHz steps such as 60 MHz can be asked.
- At a rate that divides 160 MHz evenly (160, 80, 53.3, 40, 32 MHz and so on) the samples are evenly spaced. Any other
  rate uses a fractional divider: the samples fall on the 160 MHz grid (6.25 ns) and are not evenly spaced. Capturing a
  crystal-driven square and fitting its rising edges to an ideal grid, the largest distance was half a sample plus less
  than 1 ns with whole dividers, and half a sample plus 2 to 5.5 ns with fractional ones (60, 64, 70, 90, 100, 120 MHz).
  The answer's `actual_rate` is the mean rate and does not show this. For fine timing choose a whole-divider rate.

### Lines and sample width

The bits a sample takes (its width) follow from the number of lines; streaming's limits follow from the width.

| Lines | 1 | 2 | 3-4 | 5-8 | 9-16 |
|---|---|---|---|---|---|
| Width (bits) | 1 | 2 | 4 | 8 | 16 |

### Limits by mode

A rate above the limit is not refused: the probe captures at the nearest rate within it.

| Mode | Limit | Checked on hardware |
|---|---|---|
| One-shot (no trigger) | 160 MHz (1 to 16 lines) | the square's periods right at 1, 2, 4, 8, 16 lines and 160 / 120 / 100 / 80 MHz |
| One-shot (with a trigger) | 100 MB/s: 100 MHz at 8 lines, 50 MHz at 16 (160 MHz up to 4) | the first capture after a boot kept 110 MB/s 3 of 3 and stopped part way at 120 MB/s 3 of 3; later captures keep 200 MB/s |
| Repeat | the one-shot's limit does not apply | no limit checked |
| Streaming (HS USB, enough send stages) | 40 MB/s: 160 MHz up to 2 lines, 80 MHz at 4, 40 MHz at 8, 20 MHz at 16 | no loss at 2 lines 160 MHz, 4 lines 80 / 60 / 40 MHz (10 s), 8 lines 40 MHz (30 s, P4X) |
| Streaming (through ch32rv's broker, P4X) | measured (20 s, 4 lines) | nothing else: no loss up to 35 MB/s (70 MHz), segments dropped at 40 MB/s (80 MHz at 3 or 4 lines). With a dmseq console: no loss at 30 MB/s (60 MHz) and 26.7 MB/s (53.3 MHz), 32 MB/s (64 MHz) dropped once in 2, 35 MB/s every time |
| Streaming (little internal RAM, small send stages) | 36 MB/s: 36 MHz at 8 lines | no loss at 8 lines 36 MHz (30 s); about 30 % lost at 40 MHz |
| Streaming (a route without send stages, or not enough internal RAM) | 8.5 MB/s: 8.42 MHz at 8 lines | no loss at 8 lines 8.42 MHz; 11 % lost at 16 MHz |
| Streaming with per-line resolution (below) | width raised to 8, 13.4 MB/s: 13.3 MHz | not checked (a value from the worst case) |

- Which limit applies depends on the probe's memory and the route at the time. The answer gives the rate used
  (`actual_rate`) and what it can hold before sending (`stream_buffer`).
- Limits while streaming alongside other functions (a console, spi-controller and so on) have not been checked.

### Per-line resolution

Data format 1 (logic blocks, oep-if-capture §1.3) carries each line its own way.

- **raw**: every sample.
- **events**: a record only when the line changed within a tick of D samples (a power of 2). A short pulse inside a tick
  is not lost: the count of changes stays. Only where in the tick it happened is lost. Slow lines such as CS carried as
  events take less bandwidth.
- In Python, name the events lines and their D: `oep_client.acquire.Request(..., resolution={"CS": 64})`. The other
  lines are raw.
- Up to 8 lines. Streaming takes it only over a route with send stages.

### With other operations

- While streaming, the capturing client keeps reading (inside `during`, `wait` reads). Waiting inside `during` for another
  process to finish (a ch32rv flash, say) stops the reading for that long: data past what the probe or the broker can
  hold (`stream_buffer` in the configure answer, and so on) is lost. Run a long operation that does not use the session
  through `wait.beside(fn)` (it runs in another thread and the stream is read until it returns).
- ch32rv's broker up to 0.29 stalls on sending to a client that does not read, and so do other clients' requests (the
  session lapses meanwhile). Its next version fixes it.

## A. Capture alone

The probe as a logic analyzer. Nothing to set in the probe.

**What you need**: a probe that captures, WireSkein (`pip install "wireskein[oep]"`).

**Wiring**: the signals to free probe pins, GND shared. Avoid pins wired to the board's parts (LEDs, buttons, boot straps).

**Steps**:

1. Find the pin names: `oep dump --port oep://<probe>` lists the pins that capture (`GPIO47` and so on).
2. Capture:

   ```sh
   wireskein capture --source oep:oep://<probe> \
       --channels SDA=GPIO47,SCL=GPIO48 --rate 20M --samples 200k \
       --trigger SCL:fall --pretrigger 1000 -o i2c.wireskein
   ```

   `NAME` in `NAME=ID` is the channel's name in the file, given each time.
3. Look and decode:

   ```sh
   wireskein gui i2c.wireskein                 # show it in the browser
   wireskein analyze i2c.wireskein             # find the buses and decode them (UART, SPI, I2C, CAN, LIN and more)
   wireskein decode i2c.wireskein --spec '{"buses":[{"protocol":"i2c","pins":{"sda":"SDA","scl":"SCL"}}]}'
   ```

**Given each time**: channels, rate, samples (or a duration when streaming), trigger, pretrigger. None stays in the probe.

## B. On labelled wiring

On a probe whose wiring has names (labels) for debugging or a test jig, channels are named by line. When the wiring
changes, fix the labels and the same command still works.

**What you need**: A's, plus the wiring labels.

**Settings (once, saved in the probe)**:

- `oep pins --wire <rvswd|swio> --name <port> --save` finds the debug lines and makes them labels.
- Other lines: `oep settings set <probe> core channel.GPIO12.label=mcu.rx@pa9 channel.GPIO6.label=mcu@pb6 --save`.
  The naming rules are in [Settings](settings.md#pin-labels-port-and-line-names).

**Steps**:

```sh
wireskein capture --source oep:oep://<probe> \
    --channels SWDIO=mcu.swdio,SWCLK=mcu.swclk,RX=mcu.rx --rate 40M --samples 1M -o debug.wireskein
wireskein analyze debug.wireskein           # decodes the CH32 debug lines (RVSWD, SWIO) too
```

**How it differs from A**:

| | A. Capture alone | B. Labelled |
|---|---|---|
| Probe settings | none | labels (saved, a fixed jig) |
| Channels named by | `GPIO47` | `mcu.swdio`, `mcu@pb6` (case does not matter) |
| When the wiring changes | fix the pins in the command | fix the labels; the command stays |
| Other PCs, other tools | someone tells them the pin mapping | the same name reaches the same line |

## C. Along with an operation

Capture while flashing, resetting, attaching a debugger or making an I2C transfer, and you have a record of what the
operation did on the lines. Run the operation on the same probe while capturing its lines (with B's labels, the
operation and the capture use the same names).

**Steps (Python)**: run the operation inside the capture.

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
        conn, _ = w.attach(halt=True, pins=(18, 17))   # the operation, while capturing
        w.detach(conn)
        wait(0.01)
    res = acquire.capture(hst, acquire.Request([("dio", "GPIO18"), ("clk", "GPIO17")], 40_000_000, 0,
                                               mode="stream", during=steps))
    wc.write("attach.wireskein", wso.capture_of(res))
finally:
    hst.end()
```

- With a trigger, "arm, then operate" makes the event the operation causes fire the trigger.
- The classic ESP32's sampler holds the SWIO line while its window runs. To catch what an operation does, use a trigger
  ([Getting started](getting-started.md#capture-signals-esp32-p4)).
- For logic and analog cut on one trigger, use the capture-group (give `--analog` with `--trigger` and WireSkein pairs them).

## D. Checked in tests

Not just capture: judge in a test whether the waveform is as expected. Write the test's steps and expectations in
WireSkein's record (runlog) and check it with `wireskein verify`. Under pytest, pytest-embedded-wireskein keeps and
checks the record per test.

- Expectations are written as `square` (period and duty), `uart`, `i2c`, `voltage` and more. Results are pass / fail
  with the measured values, and JUnit XML.
- Name lines by label, and the test code survives changes to the jig's wiring.
- The steps are in [WireSkein's capture tests](https://github.com/Open-Embedded-Probe/wireskein/blob/main/docs/capture-test-guide.md).
