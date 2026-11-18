# Adding your own commands (SimpleInterface)

[日本語](simple-interface.ja.md)

How to use `oep::SimpleInterface`, the part that turns functions you write and register into an OEP interface that the host can
call by name or by number.

Example uses:

- **Adding work to the probe**: try a library's behavior, read a sensor, drive a peripheral circuit in a fixed sequence, collect
  data.
- **Test fixtures**: wait for a change on the peer's pin and measure the delay, send pulses of a fixed width and count, count a
  frequency, cut power at a chosen time.
- **Your own device that speaks only the OEP core**: products such as thermometers, power supplies and displays that do not leave
  their pins to OEP and have only their own interface.

In every case the host calls it the same way, with `oep call` and `oep_client.call`.

Some things do not fit: features whose pin roles are chosen through the plan, features with detailed lock rules, features that
stream data, features with a lot of state (like SWD or logic capture). Those implement `oep::Interface` directly
([Your first interface](your-first-interface.md)).

## 1. The smallest example

```cpp
#include <OepFirmware.h>
#include <OepSimpleInterface.h>

oep::Result add(oep::Args &in, oep::Reply &out) {
  const int32_t a = in.i32();
  const int32_t b = in.i32();
  out.i32(a + b);
  return out.done();
}

oep::SimpleInterface mine("io.github.yourname.mine", 1);   // the interface's name and revision

void setup() {
  mine.op(0x10, "add", {{"a", oep::i32}, {"b", oep::i32}}, {{"sum", oep::i32}}, add);
  oep::firmware.add(mine);   // the released firmware's features stay as they are
  oep::firmware.begin();
}

void loop() { oep::firmware.loop(); }
```

The `sketch.yaml` for adding to the released firmware and how to choose features are in
[Firmware features](firmware-features.md).

### Making your own core-only device

As in [MinimalProbe](../../examples/01.Basics/MinimalProbe/MinimalProbe.ino), create an `oep::Endpoint` in your own sketch and
add to it. None of the firmware's parts go in.

```cpp
void setup() {
  Serial.begin(115200);
  endpoint.setProbeDescription(probeTlv, describeProbe());
  mine.op(0x10, "add", {{"a", oep::i32}, {"b", oep::i32}}, {{"sum", oep::i32}}, add);
  endpoint.add(mine);
}

void loop() { endpoint.poll(); }
```

### Calling it from the host

```
$ oep dump --port /dev/ttyACM0
...
io.github.yourname.mine  rev 1
  add(a: i32, b: i32) -> sum: i32

$ oep call /dev/ttyACM0 io.github.yourname.mine add a=2 b=3
completed  rev 1  sum=5
```

## 2. Names and numbers

- The **interface name** is a reverse-DNS name of your own (`io.github.<account>.<name>`, up to 48 bytes). Names starting with
  `oep.` cannot be used. With a name that breaks the rules, both `op()` and `oep::firmware.add()` return false.
- **revision** starts at 1. Raise it when a command's arguments or answer change form. The host judges the form by name and
  revision.
- **op numbers** are chosen from 0x10-0xFF (0x01-0x0F are the ops common to all interfaces). Choose the numbers yourself and do
  not change them.
- **Command names** are 1-24 characters of `a-z 0-9 - _`. The host can call by name or by number.
- When registration is impossible (a duplicate number or name, a duplicate field name, a missing type), `op()` returns false.

## 3. Arguments and answers

Pass `op()` the argument fields and the answer fields as lists of `{name, type}`. In the function, take them out in the
registered order and pack them in the registered order. Field names and types appear in describe, so the host can call without
knowing anything in advance.

| Type | Contents | Take out / pack |
|---|---|---|
| `oep::u8` `u16` `u32` `u64` | unsigned integers | `in.u8()` / `out.u8(v)` and so on |
| `oep::i8` `i16` `i32` `i64` | signed integers | `in.i32()` / `out.i32(v)` and so on |
| `oep::boolean` | true or false | `in.boolean()` / `out.boolean(v)` |
| `oep::bytes` | a byte string (with length) | `in.bytes()` → `oep::Span` / `out.bytes(p, n)` |
| `oep::text` | a string (UTF-8, with length) | `in.text()` → `oep::Span` / `out.text("...")`, `out.printf(...)` |
| `oep::list(oep::u32)` and so on | a list of integers or booleans (with count) | `in.count()`, then the elements one by one / `out.list(p, count, bytes per element)` |

- **Units go at the end of the field name**: `_ns`, `_us`, `_ms`, `_hz`, `_samples` (for example `width_ns`, `edges_samples`).
  Times returned in the probe's clock (`oep::nowNs()`) line up on the same time axis as captures and logs.
- The part checks argument lengths. A request that is too short is refused without calling the function (malformed).
- Check value ranges in the function, and for values it cannot handle, `return out.unsupported();`. When it cannot do it right
  now, `return out.busy();`.
- Returning `out.done()` without packing the answer as registered gives the host failed.
- For a command without arguments or answers, pass `{}`.

### Returning a failure

When it was accepted but did not succeed, return `out.fail(number, "short text")`. The interface decides what the numbers mean.

```cpp
oep::Result readSensor(oep::Args &in, oep::Reply &out) {
  if (!sensor.begin()) return out.fail(1, "no answer at 0x44");
  out.i32(sensor.temperature());
  return out.done();
}
```

The host receives the failed op's name, the number and the text (`CommandFailed` in Python).

### Accepting without the lock

Give commands that only read and change nothing the mark `oep::kLockFree`. The host can call them without taking the session
lock, and can read even while someone else holds the lock.

```cpp
mine.op(0x20, "read", {}, {{"temp_mc", oep::i32}}, readTemp, oep::kLockFree);
```

## 4. Long-running work (the worker)

Command functions run in the thread that handles OEP requests. Work that ends at once can be written as is. Longer work, and
work that starts peripheral circuits, goes to an `oep::Worker`.

```cpp
oep::Worker worker;

// A job: runs in the worker, and the command answers at once. Watch it with status
void drawFrame(oep::Args &in, oep::Worker &w) {
  const uint32_t color = in.u32();
  for (int row = 0; row < 64; ++row) {
    if (w.cancelled()) return;               // cancel came: status becomes cancelled
    if (!display.drawRow(row, color)) {
      w.fail(2, "no answer");                // status becomes failed, with number 2 and the text
      return;
    }
  }
}

// Runs inside the worker and answers after it finishes (peripheral begin / end, settings)
oep::Result beginBus(oep::Args &in, oep::Reply &out) {
  Wire.begin(kSda, kScl);
  Wire.setClock(in.u32());
  return out.done();
}

void setup() {
  mine.addWorker(worker, 0x20);   // adds status (0x20) and cancel (0x21). Before job()
  mine.op(0x10, "begin", {{"clock_hz", oep::u32}}, {}, beginBus, oep::kInWorker);
  mine.job(0x11, "draw", {{"color", oep::u32}}, drawFrame);
  ...
}
```

- A command registered with `job()` copies the request's fields, starts running in the worker, and answers at once. Calling
  another job or a `kInWorker` op while a job is running is refused (unavailable).
- The status that `addWorker()` adds returns state (0 idle, 1 running, 2 done, 3 failed, 4 cancelled), the last job's op, the
  failure number and text, and the start and end times (`start_ns`, `end_ns`, the probe's clock). It can be called without the
  lock.
- **Cancelling is cooperative.** cancel only sets a flag, and the job leaves by looking at `w.cancelled()`. A job that hangs
  without looking cannot be stopped, so restart the probe (`oep restart`).
- A `kInWorker` op answers after it finishes running inside the worker. Keep it short.

Where the worker runs is fixed per chip.

| Chip | Worker |
|---|---|
| ESP32 / ESP32-P4 | a FreeRTOS task, pinned to a core other than loop()'s (core 0 on ESP32-P4). Created at the first job |
| RP2040 / RP2350 | call `worker.poll()` from `loop1()` (runs on the second core) |
| CH32 | there is no other thread, so call `worker.poll()` from `loop()`. It does not answer while running, so keep to short work |

On ESP32 / ESP32-P4 and RP2, it keeps answering OEP requests while a job runs.

```cpp
void loop1() { worker.poll(); }   // RP2040 / RP2350
```

**Start peripherals that use interrupts (Wire, SPI and so on) inside the worker** (in a `kInWorker` op or in a job). On ESP32, a
peripheral's interrupt is attached to the core that called `begin()`. On the ESP32-P4's core 1, USB and the probe's own work
run. When the I2C interrupt is delayed there, ESP-IDF's I2C driver has been seen to malfunction.

## 5. Adding a log and lining it up with captures

With a log, lines written inside commands get the probe's clock time. It is the same clock as logic capture, so the lines can be
placed on the waveform's samples.

```cpp
#include <OepProbeLog.h>

oep::ProbeLog logbook(8192, 256);   // an 8 KiB ring (a power of 2), 256 marks. Taken from the heap (logbook.ok())

void drawFrame(oep::Args &in, oep::Worker &w) {
  logbook.line("## draw");
  ...
  logbook.printf("rows=%d", rows);
  logbook.line("##");
}

void setup() {
  mine.addLog(logbook);   // adds log-read, log-marks, log-clear at 0x11-0x13
  ...
}
```

- `line()` and `printf()` only write to RAM and do not wait for sending to finish (a few hundred ns). No pin is needed. Any
  thread may write.
- `addLog()` uses ops 0x11-0x13. Do not use those 3 for your own commands.
- Lines starting with `#` are separators (WireSkein headings); others are recorded as events.
- On the host, read it from `oep_client.probe_log.ProbeLog` with the same interface name. When the ring overflows and lines are
  lost, that is visible too.

## 6. Pins

When adding to the released firmware, take the pins the commands use at boot. Taken pins can no longer be planned by other
interfaces (logic capture only listens, so it can capture the same pins).

```cpp
oep::firmware.onSetup([](oep::Endpoint &, oep::PinTable &pins) {
  pins.claim(kSda, oep::owners::kSketch);
  pins.claim(kScl, oep::owners::kSketch);
});
```

A core-only device has no channels, so it does nothing.

## 7. Calling from the host

### Command line

```
oep call PROBE INTERFACE OP [ARGS...] [--wait S]
```

- `OP` is a name (`add`) or a number (`0x10`).
- Arguments are `name=value`. Integers in decimal or `0x`, booleans `true` / `false`, lists `1,2,3`, `bytes` in hex, `text` as
  is.
- Ops that do not expose their field forms (such as standard interface ops) take a hex byte string (`--text` for a string).
- `--wait S` starts a job, waits up to S seconds for its end through status, and prints status.
- The answer is the result (completed / failed / the refusal reason), the revision, and the field values. `--json` is
  available too.

### Python and pytest

```python
from oep_client import link, core
from oep_client import call as c

hst = link.open_host("/dev/ttyACM0")
core.take(hst, 10000, owner="my-test")

r = c.call(hst, "io.github.yourname.mine", "add", a=2, b=3)
assert r["sum"] == 5 and r.revision == 1

st = c.call_wait(hst, "io.github.yourname.mine", "draw", color=0xFF0000, timeout=5)   # start the job, wait for its end through status
print(st["start_ns"], st["end_ns"])
```

- Results can be looked up by field name (`r["sum"]`, `r.sum`). Check `r.revision` too. Calls by number with raw bytes work
  too (`c.call(hst, name, 0x10, b"...")`, `r.payload`).
- Errors are exceptions.
  - `CommandRejected`: refused (with the reason).
  - `CommandFailed`: failed (the number and text of `fail()`).
  - `JobFailed`, `JobCancelled`, `JobTimeout`: how the job ended. `JobTimeout` sends cancel first, then raises.
  - `ProbeRestarted`: the probe restarted while waiting (boot_id changed).
  - `InterfaceMissing`, `RevisionMismatch`, `OpMissing`: all children of `NotOffered`. In pytest they can be turned into
    `pytest.skip`.
- Lock-free ops are called from outside the session (`call()` looks at describe's mark). `call_wait`'s `wait=` accepts the
  `wait` that logic capture's `during=steps` passes. The capture continues while waiting.

## 8. Examples

- [extras/LibraryTest](../../extras/LibraryTest/README.md): runs a library's calls in the worker, captures its bus, and places
  the log lines on the waveform (job, status, `result`, log).
