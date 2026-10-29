# LibraryTest: a library run on the probe, its wires and its log lined up

[日本語](README.ja.md)

A file to add to the released firmware (`examples/Firmware/OepProbe`): the calls of a library under test run on the probe
itself, the same probe's logic capture takes their lines, and every line the code logged is placed at its sample.

- `LibraryTest.cpp`: adds `io.github.example.library-test` (`oepUserSetup`). Op `run` (0x10) runs one case to its end and
  answers its result; the case's lines go into the log of `OepProbeLog.h`.
- `test_library_test.py`: the host side (pytest). The capture planned on SDA / SCL, started with START as its trigger,
  `run`, the segment read, each log line placed at its sample and checked.

## The log is not sent

`log_->line("## step")` writes the line into a RAM ring and marks its start with the probe's clock (ns) - a few hundred ns.
Unlike markers sent on a UART, nothing waits for a line to go out and no pin is needed.

- A capture segment's `start_ns` is on the same clock: a line's sample is `(its ns - start_ns) x the rate`
  (`oep_client.probe_log.sample_of`), within the segment's `start_uncertainty_ns` (+-5 us on an ESP32-P4).
- The host reads the log with common §1's read / marks (reading does not consume). A full ring leaves a lost mark, a full
  mark ring a jump in the marks' serials: missing lines show (`probe_log.Log.lost`).
- `probe_log.rows` gives the lines as a recorder's rows (WireSkein's `run.json`): the host's `time.monotonic()`,
  `probe_ns`, `boot_id` and the probe's name; a line starting with `#` is a marker (WireSkein's heading dialect), the
  others events.

## Use

1. Copy the `OepProbe` folder, add `LibraryTest.cpp`, point the copy's `sketch.yaml` `libraries: - dir:` at this library
   and build ([firmware features](../../docs/guide/firmware-features.md) §4).
2. Replace the cases with your library's calls, and write lines with `log_->line()` / `log_->printf()`.
3. On the host: `OEP_LIBTEST_PORT=<port> OEP_LIBTEST_PINS=<sda>,<scl> pytest test_library_test.py`.

The pins are chosen per chip (ESP32-P4 32 / 33, classic ESP32 25 / 26, RP2040 / RP2350 26 / 27). The sketch holds them
(`kSketch`): no other interface is planned on them, the capture listens. With nothing wired a write to an absent address
still gives START, the address, a NACK and STOP (on a bus without pull-ups the ESP32-P4's `endTransmission` answers 4).

`run` runs its case inside the request: the probe answers nothing else meanwhile. Run a long case from `oepUserLoop` and
tell its end with another op.
