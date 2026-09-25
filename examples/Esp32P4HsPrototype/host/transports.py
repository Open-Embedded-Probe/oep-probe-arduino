# /// script
# dependencies = ["pyserial>=3.5", "pyusb>=1.3", "libusb1>=3", "hidapi>=0.14"]
# ///
"""P1/P2: one OEP session reached over vendor bulk, HID and CDC of one P4 HS device (Esp32P4HsPrototype)."""
import sys, time
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[4] / "oep-client-python" / "src"))
from oep_client.v1 import core, host as h, link

CDC = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM13"
ok = True
def check(name, cond, extra=""):
    global ok
    ok &= bool(cond)
    print(f"{'PASS' if cond else 'FAIL'} {name} {extra}")

A = link.open_usb_host(transports=("vendor",))
B = link.open_usb_host(transports=("hid",))
C = link.open_host(CDC)
print("transports:", A.link.transport, B.link.transport, type(B.link.stream.backend).__name__)
try:
    check("no lock at start", A.lock_state()[0] is False)
    o = A.open(3000)
    S = A.session
    check("vendor opens", o.lease_ms > 0, f"session {S:#x}")
    for name, X in (("hid", B), ("cdc", C)):
        locked, remaining = X.lock_state()
        check(f"{name} sees the lock", locked and remaining > 0, f"remaining {remaining}")
        try:
            X.open(3000)
            check(f"{name} other session refused", False)
        except h.Locked:
            check(f"{name} other session refused", True)
        r = X.open(3000, session=S)
        check(f"{name} same session resumes", r.resumed)
        X.keepalive()
        check(f"{name} locked request in the session", True)
    # heartbeats go where the subscription came from
    for X in (A, B, C):
        X.link.events.clear(); X.link.pushes.clear()
    B.subscribe(0, 0, 100)
    t = time.monotonic()
    while time.monotonic() - t < 0.6:
        for X in (A, B, C):
            X.link.pump(0.05)
    counts = [len(X.link.events) + len(X.link.pushes) for X in (A, B, C)]
    check("heartbeats only on the subscriber's transport (hid)", counts[1] >= 3 and counts[0] == 0 and counts[2] == 0, str(counts))
    A.subscribe(0, 0, 100)    # re-subscribing from vendor moves them
    for X in (A, B, C):
        X.link.pump(0.1); X.link.events.clear(); X.link.pushes.clear()
    t = time.monotonic()
    while time.monotonic() - t < 0.6:
        for X in (A, B, C):
            X.link.pump(0.05)
    counts = [len(X.link.events) + len(X.link.pushes) for X in (A, B, C)]
    check("after vendor subscribes, heartbeats move to vendor", counts[0] >= 3 and counts[1] == 0 and counts[2] == 0, str(counts))
    A.unsubscribe(0)
    for name, X in (("vendor", A), ("hid", B), ("cdc", C)):
        r = core.link_speed(X, seconds=1.0)
        print(f"speed {name}: in {r['in_mb_s']:.2f} MB/s out {r['out_mb_s']:.2f} MB/s (size {r['size']} x {r['inflight']})")
    C.end()
    check("cdc ends the shared session; vendor sees no lock", A.lock_state()[0] is False)
finally:
    for X in (A, B, C):
        X.link.close()
print("ALL PASS" if ok else "SOME FAILED")
