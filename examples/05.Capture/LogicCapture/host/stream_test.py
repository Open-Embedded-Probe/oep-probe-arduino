# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Open Embedded Probe
# /// script
# requires-python = ">=3.11"
# dependencies = ["oep-client-python", "numpy"]
# ///
"""Capture streaming on the LogicCapture example over HS (oep-spec oep-if-capture §3.4, logic-capture §7.9): LEDC squares
of 1 MHz and 250 kHz on GPIO 4 / 5 (io.github.open-embedded-probe.test-signal), captured as 2 channels in mode 3 at
each rate for some seconds.
Checks per rate: the bytes received up to status's write position, position jumps (probe-side drops), push frames
missing by seq, and every rising edge on the ideal grid (±1 sample: LEDC and PARLIO share a source clock, so an edge
on the sampling instant lands on either side). A gap breaks the phase, so each contiguous run is checked alone.

  uv run stream_test.py <usb serial, e.g. 30eda0e343c6-hs> <rate Hz[,rate...]> <seconds>
"""

import sys
import time

import numpy as np

from oep_client import capture, core, link  # noqa: E402

SIGNALS = ((4, 1_000_000), (5, 250_000))


def edges_on_grid(data: bytes, cfg: capture.Config, k: int, period: float) -> tuple[int, int]:
    """(rising edges, edges off the ideal grid) of channel k, 8 MiB at a time."""
    w, pos = cfg.width, cfg.positions[k]
    rises, bad, first = 0, 0, None
    prev = None       # the last sample of the previous chunk
    base = 0          # samples before this chunk
    step = 8 << 20
    for at in range(0, len(data), step):
        bits = np.unpackbits(np.frombuffer(data, np.uint8, min(step, len(data) - at), at), bitorder="little")
        ch = bits.reshape(-1, w)[:, pos].astype(np.int8)
        joined = ch if prev is None else np.concatenate(([prev], ch))
        lead = 0 if prev is None else 1
        idx = np.flatnonzero(np.diff(joined) == 1) + 1 - lead + base   # the sample index of each rise
        prev = ch[-1]
        base += len(ch)
        if len(idx):
            if first is None:
                first = idx[0]
            grid = (idx - first) / period
            bad += int(np.count_nonzero(np.abs(grid - np.round(grid)) * period > 1.0))
            rises += len(idx)
    return rises, bad


def runs(got: capture.Received):
    """The contiguous pieces of got.data between position jumps."""
    cut = [0] + [i for i, _ in got.gaps] + [len(got.data)]
    return [bytes(got.data[a:b]) for a, b in zip(cut, cut[1:]) if b > a]


def main() -> int:
    serial, rates, seconds = sys.argv[1], [int(r) for r in sys.argv[2].split(",")], float(sys.argv[3])
    hst = link.open_usb_host(serial=serial)
    lk = hst.link
    print(f"# transport {lk.transport}")
    hst.open(lease_ms=3000)
    fn = core.find(hst, capture.LogicCapture.NAME)
    sig = core.find(hst, "io.github.open-embedded-probe.test-signal")
    core.plan_apply(hst, [(fn, 0, SIGNALS[0][0]), (fn, 1, SIGNALS[1][0])])
    for pin, hz in SIGNALS:
        hst.call(sig, 0x01, bytes([pin]) + hz.to_bytes(4, "little") + bytes([50]))
    cap = capture.LogicCapture(hst, fn)
    failed = 0
    try:
        for rate in rates:
            cfg = cap.configure(rate=rate, mode=capture.STREAMING, critical={capture.MODE})
            cap.subscribe(0, 5)
            t0 = time.monotonic()
            cap.start()
            got = cap.stream(lk, seconds=seconds)
            cap.stop()
            got = cap.finish(lk, got, timeout=10)
            took = time.monotonic() - t0
            cap.unsubscribe()
            end = cap.status()[2]
            reached = (got.start or 0) + len(got.data) + sum(n for _, n in got.gaps)
            skipped = sum(n for _, n in got.gaps)
            bad = []   # (rises, off grid) per channel; None where the rate is too low to see the square (< 4 per period)
            for k, (_, hz) in enumerate(SIGNALS):
                period = float(cfg.rate) / hz
                if period < 4:
                    bad.append(None)
                    continue
                e = [edges_on_grid(r, cfg, k, period) for r in runs(got)]
                bad.append((sum(x for x, _ in e), sum(y for _, y in e)))
            judged = [x for x in bad if x is not None]
            ok = reached == end and not got.gaps and not got.seq_lost and all(b == 0 and n > 0 for n, b in judged)
            failed += not ok
            print(f"{'ok  ' if ok else 'FAIL'} rate {float(cfg.rate) / 1e6:g} MHz w={cfg.width} pos={cfg.positions}: "
                  f"{len(got.data) / 1e6:.1f} MB in {took:.1f} s ({len(got.data) / took / 1e6:.1f} MB/s), "
                  f"{got.frames} frames, end {'=' if reached == end else '!='} write_pos, gaps {len(got.gaps)} "
                  f"({skipped / 1e6:.2f} MB), seq lost {got.seq_lost}, edges off grid "
                  + ", ".join(f"ch{k} " + ("-" if x is None else f"{x[1]}/{x[0]}") for k, x in enumerate(bad)))
    finally:
        for pin, _ in SIGNALS:
            hst.call(sig, 0x02, bytes([pin]))
        core.plan_release(hst)
        hst.end()
        lk.close()   # the async IN transfers must be cancelled, or libusb's finalizer waits on them at exit
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
