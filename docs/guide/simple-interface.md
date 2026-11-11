# Adding your own commands (SimpleInterface)

[日本語](simple-interface.ja.md)

`oep::SimpleInterface` turns functions you write into an OEP interface: register them, and a host calls them by name or
by number.

Where it fits:

- **Adding work to a probe**: try a library, read a sensor, run a peripheral through a fixed sequence, collect data.
- **A test fixture**: wait for a pin of the device to change and measure the delay, send pulses of a set width and
  count, count a frequency, cut the power at a chosen time.
- **Your own device that speaks only OEP's core**: a thermometer, a power supply, a display - a product that leaves no
  pins to OEP and has just its own interface.

In every case the host calls it with the same `oep call` and `oep_client.call`.

It does not fit features that let the plan choose pin roles, have fine lock rules, stream data or keep much state (an
SWD port, a logic capture). Those implement `oep::Interface` directly ([Your first interface](your-first-interface.md)).

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
  oep::firmware.add(mine);   // the released firmware's features stay
  oep::firmware.begin();
}

void loop() { oep::firmware.loop(); }
```

The `sketch.yaml` for building on the released firmware, and how to choose its features, are in
[Firmware features](firmware-features.md).

### Your own core-only device

As in [MinimalProbe](../../examples/01.Basics/MinimalProbe/MinimalProbe.ino), make an `oep::Endpoint` in your sketch and
add the interface to it. None of the firmware's parts come in.

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

- **The interface's name** is a reverse-DNS name of yours (`io.github.<account>.<name>`, up to 48 bytes). Names starting
  with `oep.` are not yours to use. With a name outside the rule, `op()` and `oep::firmware.add()` return false.
- **The revision** starts at 1. Raise it when a command's arguments or answer change: a host judges the forms by name
  and revision.
- **An op's number** is one of 0x10 - 0xFF (0x01 - 0x0F are the ops every interface shares). You choose it and keep it.
- **A command's name** is 1 - 24 of `a-z 0-9 - _`. A host calls it by name or by number.
- `op()` returns false when it cannot register (a number or name taken, a field name repeated, an unknown type).

## 3. Arguments and answers

`op()` takes the argument fields and the answer fields as lists of `{name, type}`. The function reads them in the order
registered and packs the answer in the order registered. The fields' names and types are in the describe, so a host
that knows nothing about the interface can call it.

| Type | What | Read / pack |
|---|---|---|
| `oep::u8` `u16` `u32` `u64` | unsigned integers | `in.u8()` / `out.u8(v)` and so on |
| `oep::i8` `i16` `i32` `i64` | signed integers | `in.i32()` / `out.i32(v)` and so on |
| `oep::boolean` | true or false | `in.boolean()` / `out.boolean(v)` |
| `oep::bytes` | bytes (with a length) | `in.bytes()` -> `oep::Span` / `out.bytes(p, n)` |
| `oep::text` | a string (UTF-8, with a length) | `in.text()` -> `oep::Span` / `out.text("...")`, `out.printf(...)` |
| `oep::list(oep::u32)` etc. | a list of integers or bools (with a count) | `in.count()`, then each element / `out.list(p, count, element bytes)` |

- **Say the unit at the end of the field's name**: `_ns`, `_us`, `_ms`, `_hz`, `_samples` (`width_ns`, `edges_samples`).
  A time on the probe's clock (`oep::nowNs()`) lines up with a capture and with the log.
- The part checks the arguments' lengths: a short request is refused without calling the function (malformed).
- The function checks the values: for one it cannot take, `return out.unsupported();`; for not now, `return out.busy();`.
- Returning `out.done()` without the answer packed as registered sends the host failed.
- A command with no arguments or no answer passes `{}`.

### Returning a failure

When the command was taken but did not work, return `out.fail(code, "a short text")`. The interface decides what its
codes mean.

```cpp
oep::Result readSensor(oep::Args &in, oep::Reply &out) {
  if (!sensor.begin()) return out.fail(1, "no answer at 0x44");
  out.i32(sensor.temperature());
  return out.done();
}
```

The host gets the op's name, the code and the text (`CommandFailed` in Python).

### Taking it without the lock

A command that only reads and changes nothing gets the flag `oep::kLockFree`: a host calls it without the session's
lock, even while someone else holds the lock.

```cpp
mine.op(0x20, "read", {}, {{"temp_mc", oep::i32}}, readTemp, oep::kLockFree);
```

## 4. Long work (the worker)

A command's function runs on the thread that handles OEP's requests: work that ends at once is fine there. Longer work,
and work that begins a peripheral, goes to an `oep::Worker`.

```cpp
oep::Worker worker;

// A job: it runs on the worker and the command answers at once; status says how it goes
void drawFrame(oep::Args &in, oep::Worker &w) {
  const uint32_t color = in.u32();
  for (int row = 0; row < 64; ++row) {
    if (w.cancelled()) return;               // a cancel came: status says cancelled
    if (!display.drawRow(row, color)) {
      w.fail(2, "no answer");                // status says failed, code 2 and the text
      return;
    }
  }
}

// Runs on the worker and answers when it ends (a peripheral's begin / end, a setting)
oep::Result beginBus(oep::Args &in, oep::Reply &out) {
  Wire.begin(kSda, kScl);
  Wire.setClock(in.u32());
  return out.done();
}

void setup() {
  mine.addWorker(worker, 0x20);   // adds status (0x20) and cancel (0x21); before job()
  mine.op(0x10, "begin", {{"clock_hz", oep::u32}}, {}, beginBus, oep::kInWorker);
  mine.job(0x11, "draw", {{"color", oep::u32}}, drawFrame);
  ...
}
```

- A command registered with `job()` copies its request's fields, starts on the worker and answers at once. While a job
  runs, another job or a `kInWorker` op is refused (unavailable).
- The status `addWorker()` adds gives the state (0 idle, 1 running, 2 done, 3 failed, 4 cancelled), the last job's op, a
  failure's code and text, and the start and end times (`start_ns`, `end_ns`, the probe's clock). It is lock-free.
- **Cancelling is cooperative**: cancel only sets a flag, and the job leaves when it sees `w.cancelled()`. A job stuck
  without looking cannot be stopped: restart the probe (`oep restart`).
- A `kInWorker` op runs on the worker to its end before it answers. Keep it short.

Where the worker runs depends on the chip.

| Chip | The worker |
|---|---|
| ESP32 / ESP32-P4 | a FreeRTOS task pinned to the core loop() does not run on (core 0 on an ESP32-P4), made at the first job |
| RP2040 / RP2350 | `worker.poll()` called from `loop1()` (it runs on the second core) |
| CH32 | no other thread: `worker.poll()` called from `loop()`. The probe does not answer while a job runs: short work only |

On an ESP32 / ESP32-P4 and an RP2 the probe keeps answering OEP's requests while a job runs.

```cpp
void loop1() { worker.poll(); }   // RP2040 / RP2350
```

**Begin a peripheral that uses interrupts (Wire, SPI, ...) on the worker** (in a `kInWorker` op or in a job). On an
ESP32 a peripheral's interrupts go to the core that called `begin()`. An ESP32-P4's core 1 runs USB and the probe's own
work; I2C interrupts delayed there have made ESP-IDF's I2C driver misbehave.

## 5. A log, lined up with a capture

With a log, every line written in a command carries the probe's clock - the logic capture's clock - so a line can be
placed at a sample of the waveform.

```cpp
#include <OepProbeLog.h>

oep::ProbeLog logbook(8192, 256);   // an 8 KiB ring (a power of two), 256 marks, from the heap (logbook.ok())

void drawFrame(oep::Args &in, oep::Worker &w) {
  logbook.line("## draw");
  ...
  logbook.printf("rows=%d", rows);
  logbook.line("##");
}

void setup() {
  mine.addLog(logbook);   // adds log-read, log-marks and log-clear at 0x11 - 0x13
  ...
}
```

- `line()` and `printf()` only write to RAM and wait for nothing to be sent (a few hundred ns); no pin is needed. Any
  thread may write.
- `addLog()` takes ops 0x11 - 0x13: do not use those three for your commands.
- A line starting with `#` is a marker (WireSkein's heading), the others events.
- The host reads it by the interface's name with `oep_client.probe_log.ProbeLog`; lines lost to a full ring show.

## 6. Pins

On the released firmware, take the pins your commands use at start-up: no other interface can then be planned on them
(the logic capture only listens, so it can take the same pins).

```cpp
oep::firmware.onSetup([](oep::Endpoint &, oep::PinTable &pins) {
  pins.claim(kSda, oep::owners::kSketch);
  pins.claim(kScl, oep::owners::kSketch);
});
```

A core-only device has no channels: nothing to do.

## 7. Calling from the host

### Command line

```
oep call PROBE INTERFACE OP [ARGS...] [--wait S]
```

- `OP` is a name (`add`) or a number (`0x10`).
- Arguments are `name=value`: integers in decimal or `0x`, bools `true` / `false`, lists `1,2,3`, `bytes` in hex,
  `text` as it is.
- An op whose describe gives no fields (a standard interface's op, say) takes the request as hex bytes (`--text` for a
  string).
- `--wait S` starts a job, waits up to S seconds for its end by status and shows the status.
- The answer shows the result (completed / failed / the reason it was refused), the revision and the fields. `--json`
  too.

### Python and pytest

```python
from oep_client import link, core
from oep_client import call as c

hst = link.open_host("/dev/ttyACM0")
core.take(hst, 10000, owner="my-test")

r = c.call(hst, "io.github.yourname.mine", "add", a=2, b=3)
assert r["sum"] == 5 and r.revision == 1

st = c.call_wait(hst, "io.github.yourname.mine", "draw", color=0xFF0000, timeout=5)   # start the job, wait by status
print(st["start_ns"], st["end_ns"])
```

- Results by field name (`r["sum"]`, `r.sum`); check `r.revision` too. By number with raw bytes as well
  (`c.call(hst, name, 0x10, b"...")`, `r.payload`).
- Errors are exceptions:
  - `CommandRejected`: refused (with the reason).
  - `CommandFailed`: failed (`fail()`'s code and text).
  - `JobFailed`, `JobCancelled`, `JobTimeout`: how a job ended. `JobTimeout` sends a cancel first.
  - `ProbeRestarted`: the probe restarted while waiting (its boot_id changed).
  - `InterfaceMissing`, `RevisionMismatch`, `OpMissing`: all `NotOffered`, for a `pytest.skip`.
- A lock-free op is sent outside the session (`call()` reads the flag from the describe). `call_wait`'s `wait=` takes the
  `wait` a logic capture's `during=steps` gets: the capture keeps going while it waits.

## 8. Examples

- [extras/LibraryTest](../../extras/LibraryTest/README.md): library calls run on the worker, their bus captured, the log's
  lines placed in the waveform (a job, status, `result`, the log).
