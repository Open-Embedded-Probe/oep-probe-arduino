# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Open Embedded Probe
# /// script
# requires-python = ">=3.11"
# dependencies = ["oep-client-python", "wireskein"]
# ///
"""Capture streaming on the LogicCapture example over HS (oep-spec oep-if-capture §3.4): LEDC squares
of 1 MHz and 250 kHz on GPIO 4 / 5 (io.github.open-embedded-probe.test-signal), captured as 2 channels streaming at
each rate for some seconds (oep_client.acquire), judged by WireSkein.
Checks per rate: no loss (a position jump or a push frame missing by seq stops the capture and fails it), and each
square on its ideal grid within one sample (WireSkein's square with grid_abs_s: LEDC and PARLIO share a source clock,
so an edge on the sampling instant lands on either side).

  uv run stream_test.py <usb serial, e.g. 30eda0e343c6-hs> <rate Hz[,rate...]> <seconds>
"""

import os
import sys
import tempfile
import time

from oep_client import acquire, core, link  # noqa: E402
from wireskein import capture as wc, decode
from wireskein.runlog import square
from wireskein.sources.oep import capture_of
from wireskein.verify import check_square

SIGNALS = (("sq1m", 4, 1_000_000), ("sq250k", 5, 250_000))


def main() -> int:
    serial, rates, seconds = sys.argv[1], [int(r) for r in sys.argv[2].split(",")], float(sys.argv[3])
    hst = link.open_usb_host(serial=serial)
    lk = hst.link
    print(f"# transport {lk.transport}")
    hst.open(lease_ms=3000)
    sig = core.find(hst, "io.github.open-embedded-probe.test-signal")
    for _, pin, hz in SIGNALS:
        hst.call(sig, 0x01, bytes([pin]) + hz.to_bytes(4, "little") + bytes([50]))
    failed = 0
    try:
        for rate in rates:
            req = acquire.Request([(name, f"GPIO{pin}") for name, pin, _ in SIGNALS], rate, 0, mode="stream",
                                  seconds=seconds)
            t0 = time.monotonic()
            res = acquire.capture(hst, req)
            took = time.monotonic() - t0
            got = res.meta.get("stream", {})
            judged = []
            with tempfile.TemporaryDirectory() as d:   # WireSkein's checks read a capture as its file gives it
                cap = decode.load(wc.write(os.path.join(d, "stream.wireskein"), capture_of(res)))
                for name, _, hz in SIGNALS:
                    if cap.rate / hz < 4:          # too slow to see the square
                        judged.append(f"{name} -")
                        continue
                    ok, out, why = check_square(cap, square(name, hz, grid_abs_s=1 / cap.rate))
                    judged.append(f"{name} {'ok' if ok else 'NG'} grid {out.get('grid_max_s', 0) * 1e9:.1f} ns"
                                  + (f" ({why})" if why else ""))
                    failed += not ok
            failed += bool(res.lost)
            nbytes = got.get("bytes", 0)
            print(f"{'FAIL' if res.lost or any(' NG ' in j for j in judged) else 'ok  '} rate {float(cap.rate) / 1e6:g} MHz: "
                  f"{nbytes / 1e6:.1f} MB in {took:.1f} s ({nbytes / took / 1e6:.1f} MB/s), {got.get('frames')} frames, "
                  f"lost {res.lost or 'none'}; " + ", ".join(judged))
    finally:
        for _, pin, _ in SIGNALS:
            hst.call(sig, 0x02, bytes([pin]))
        core.plan_release(hst)
        hst.end()
        lk.close()   # the async IN transfers must be cancelled, or libusb's finalizer waits on them at exit
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
