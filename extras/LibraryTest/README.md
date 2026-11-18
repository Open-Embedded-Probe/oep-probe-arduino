# LibraryTest: try a library on the probe and tie its waveforms and log together

[日本語](README.ja.md)

An example of a file added to the released firmware (`OepFirmware.h`). It runs calls of a library you want to test on the probe
itself, captures their lines with the same probe's logic capture, and ties each log line the code wrote to a sample of the
waveform.

- `LibraryTest.cpp`: adds `io.github.example.library-test` (`libraryTestRegister()`: `oep::firmware.onSetup` and `add`). It is
  made with [`oep::SimpleInterface`](../../docs/guide/simple-interface.md), so the op names and fields appear in describe (it can
  be called by name with `oep call` or `oep_client.call`).
  - `run` (0x10, `case: u8`): a job that starts the case in the worker and answers at once. Called while one is running it is
    busy; a case that does not exist ends as failed.
  - `status` (0x20) and `cancel` (0x21): the worker's ops (`addWorker`). The state, the failure code and message, the start and
    end times.
  - `result` (0x22, no lock needed): the number and return value of the last case.
  - `log-read` / `log-marks` / `log-clear` (0x11-0x13): the log (`addLog`). Lines inside a case go in with `mark()` / `markf()`.
- `test_library_test.py`: a host example (pytest). It plans a capture on SDA / SCL, starts it with START as the trigger, waits
  for `run` to finish with `call_wait`, reads the segment, and checks each log line placed on a sample. A short write, and a long
  sequence imitating a display (initialization and drawing, about 90 ms).

## Cases run in the worker

Cases run on an `oep::Worker` (on ESP32 / ESP32-P4 a FreeRTOS task pinned to a core other than loop()'s, core 0 on ESP32-P4; on
RP2040 / RP2350 `worker.poll()` from `loop1` on the second core). However long it takes - a display's initialization through
drawing, a power IC's sequence, a `delay` inside the library - the probe keeps answering capture, lease and status meanwhile. The
case code is ordinary sequential code, and markers are written with `mark()` / `markf()` (the log is exclusive only briefly
during writes and reads, so you need not take a lock yourself). The bus is also started on the worker, at the first case. Its
interrupt is attached to the worker's core. CH32 has neither a worker task nor a second core, so this example is not included
there.

## The log is not sent

`mark("## step")` only writes a line into a ring in RAM and puts a mark with the probe's clock (ns) at its head (a few hundred
ns). Unlike markers sent over UART, the work does not stop until sending finishes, and no pin is needed.

- The capture segment's `start_ns` is on the same clock, so a line's sample is `(line ns − start_ns) × rate`
  (`oep_client.probe_log.sample_of`). The accuracy is the segment's `start_uncertainty_ns` (±5 µs on ESP32-P4).
- The host reads the log with common §1's read / marks (`oep_client.probe_log.ProbeLog`) (reading does not erase it). If the
  ring overflows there is a lost mark, and if the mark ring overflows there is a gap in the marks' serial numbers, so losses are
  visible (`probe_log.Log.lost`).
- `probe_log.rows` turns lines into recorder rows (WireSkein's `run.json`): the host's `time.monotonic()`, `probe_ns`,
  `boot_id`, the probe's name. Lines starting with `#` are markers (WireSkein's heading dialect); others are events.

## How to use

1. Put `LibraryTest.cpp` in your sketch folder, make the `.ino` the following form, and build (`sketch.yaml`: §5 of
   [Firmware features](../../docs/guide/firmware-features.md)).

   ```cpp
   #include <OepFirmware.h>
   void libraryTestRegister();

   void setup() {
     libraryTestRegister();
     oep::firmware.begin();
   }

   void loop() { oep::firmware.loop(); }
   ```
2. Replace the case functions with calls of your own library, and write markers with `mark()` / `markf()`.
3. On the host, `OEP_LIBTEST_PORT=<port> OEP_LIBTEST_PINS=<sda>,<scl> pytest test_library_test.py`.

The pins are chosen per chip (ESP32-P4 32 / 33, classic ESP32 25 / 26, RP2040 / RP2350 26 / 27). The sketch holds them with
`kSketch`, so other interfaces cannot plan them and only capture can listen. Even on a bus with nothing connected, a write to an
address nobody answers produces START, the address, NACK and STOP (on a bus without pull-ups the ESP32-P4's `endTransmission`
returns 4).
