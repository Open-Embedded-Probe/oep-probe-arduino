# Hardware conformance checks

[日本語](README.ja.md)

This entry point checks the firmware running on a hardware probe against a chosen oep-spec checkout. The checker is the Python client's `oep_client.conformance`. It does not write firmware or change settings.

## How to run

Copy [tests/.env.example](../.env.example) to `tests/.env` and set `OEP_CONFORMANCE_*` and the shared `OEP_HW_LOCK`.

| Variable | Contents |
|---|---|
| `OEP_CONFORMANCE_ADDRESS` | the probe's port (`/dev/ttyACM0`, `tcp://HOST:PORT` and so on) |
| `OEP_CONFORMANCE_UNIT_ID` | the unit ID of the unit to check. Stops on another unit |
| `OEP_CONFORMANCE_SPEC` | the oep-spec checkout to check against |
| `OEP_CONFORMANCE_OUT` | the path of a not yet existing file to write the JSON result to |
| `OEP_HW_LOCK` | an existing lock file shared by everyone who uses the equipment |
| `OEP_CONFORMANCE_FRAMING` | optional. `serial` checks COBS / CRC, wire errors, and that the session survives reopening the same port, independently of the client. `tcp` checks the length and errors |
| `OEP_CONFORMANCE_TCP_PEER` | optional. With `tcp`, the address for a second connection to the same probe, when possible. Adds the two-connection items |

```sh
uv run --project tests --env-file tests/.env pytest tests/equipment --junitxml=/absolute/path/to/new-run.xml
```

When using a client whose checks are still being added, add `--with-editable /absolute/path/to/oep-client-python`.

## Results

- Each core item becomes its own pytest / JUnit result. For interfaces, one result checks the common declarations of all fns. The behavior of each interface is not checked by this entry point.
- The JSON keeps the bytes sent and received, the SPEC commit, dirty state and hash, the checker's hash, the firmware version, and the range not checked.
- It is skip only when nothing is set. Partial settings, a wrong connection or unit, and a mismatch with the specification are FAIL. Items after a transport or cleanup failure are BLOCKED, and the whole is FAIL.
- On finding a restart or a change of unit, it stops there. It does not touch pins, targets or settings. force is tried only during a session the checker opened itself.

What the checker checks is in the client's [conformance checks](https://github.com/Open-Embedded-Probe/oep-client-python/blob/main/docs/conformance-checks.md). Items the client checks with its own models (the virtual probe, logical fixtures) are part of the client's tests and do not run in this entry point.

## Streaming rate (ESP32-P4 / P4X)

`test_stream_rate.py` streams a logic capture at the rate in the table of [Capture](../../docs/guide/capture.md) and checks that nothing is lost.
The rate is 40 MB/s, run as 4 cases: 2 lines at 160 MHz, 4 at 80 MHz, 8 at 40 MHz, 16 at 20 MHz. One more case sends lock-free
requests (the core's clock) one at a time, 400 a second, beside a 4-line 80 MHz stream (the answers and the stream's pushes share the same vendor bulk IN). The received bytes are counted and dropped at once,
so the host's memory and CPU are not part of what is measured.

| Variable | Contents |
|---|---|
| `OEP_STREAM_ADDRESS` | the probe's port (`oep://<unit_id>`, `/dev/ttyACM0` and so on) |
| `OEP_STREAM_UNIT_ID` | that unit's unit ID. Stops on another unit |
| `OEP_STREAM_CHANNELS` | the names of free channels, separated by commas (`GPIO12,GPIO13,...`). The first ones, as many as the line count, are used |
| `OEP_STREAM_SECONDS` | optional. Seconds to run one case. Default 20 |
| `OEP_STREAM_MAX_LOAD` | optional. Above this load average the case is skipped as busy. Default twice the number of cores |

```sh
uv run --project tests --env-file tests/.env pytest tests/equipment/test_stream_rate.py --junitxml=/absolute/path/to/new-run.xml
```

- It is skip when nothing is set. Too few channels and a probe that is not an ESP32-P4 / P4X are skip too.
- When the host's load average is above `OEP_STREAM_MAX_LOAD` (default twice the number of cores), the rate cannot be judged, so it is skip (the reason says "host busy").
- Any of a lost segment, a gap, a missing push, a stop on error, less than 95 % of the bytes arriving, or less than 90 % of the requests is FAIL.
- The JUnit properties keep the load average, the requested and actual rates, the bytes received, the seconds and the number of gaps.
- It takes only a lease (not the bench lock).
