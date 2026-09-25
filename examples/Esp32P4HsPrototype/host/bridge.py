# /// script
# dependencies = ["pyserial>=3.5", "pyusb>=1.3", "libusb1>=3"]
# ///
"""P3: oep.fixture.uart forwarded to the "UART bridge" CDC port (Esp32P4HsPrototype, TX pad looped back to RX inside)."""
import os, struct, sys, threading, time
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[4] / "oep-client-python" / "src"))
import serial
from oep_client.v1 import console, core, fixture, link

PORT = sys.argv[1]
TX, RX = 20, 21
ok = True
def check(name, cond, extra=""):
    global ok
    ok &= bool(cond)
    print(f"{'PASS' if cond else 'FAIL'} {name} {extra}", flush=True)

def drain(p, want, seconds=3.0):
    got, t = bytearray(), time.monotonic()
    while len(got) < want and time.monotonic() - t < seconds:
        got += p.read(max(1, p.in_waiting))
    return bytes(got)

A = link.open_usb_host(transports=("vendor",))
try:
    A.open(5000)
    fu, fb = core.find(A, "oep.fixture.uart"), core.find(A, "oep.test.bridge")
    core.plan_apply(A, [(fu, 1, RX), (fu, 2, TX)])
    u = fixture.FixtureUart(A, fu)
    baud = u.configure(115200)
    A.call(fb, 0x02, bytes([1, TX]))            # loopback: RX from the TX pad
    A.call(fb, 0x01, b"\x01")                   # bind the bridge port
    start = u.read(console.PositionStream.FROM_NOW, 0, 0).start

    p = serial.Serial(PORT, 115200, timeout=0.05)   # sets line coding 115200 (same)
    time.sleep(0.2); p.reset_input_buffer()
    msg = b"hello bridge\r\n"
    p.write(msg)
    back = drain(p, len(msg))
    check("CDC -> TX -> (loop) RX -> CDC", back == msg, repr(back))
    chunk = u.read_from(start, 1000)
    check("the same bytes in the OEP stream", chunk.data.endswith(msg), repr(chunk.data[-20:]))

    for b in (921600, 2000000):
        p.baudrate = b
        time.sleep(0.2)
        st = struct.unpack("<III", A.call(fb, 0x03).payload[:12])
        check(f"line coding {b} reruns the UART", abs(st[0] - b) <= b * 0.03, f"uart {st[0]} coding {st[2]}")
        p.reset_input_buffer()
        data = os.urandom(65536)
        g0 = st[1]
        t = time.monotonic()
        w = threading.Thread(target=p.write, args=(data,)); w.start()   # read while writing
        back = drain(p, len(data), 10)
        dt = time.monotonic() - t
        w.join()
        g1 = struct.unpack("<III", A.call(fb, 0x03).payload[:12])[1]
        check(f"64 KiB echo at {b}", back == data, f"{len(back)} bytes, {len(back) * 10 / dt / 1e3:.0f} kbaud effective, new gaps {g1 - g0}")

    # while the port is closed: bytes sent through OEP write loop back into the stream; does the port get them later?
    p.close()
    time.sleep(0.2)
    u._call(fixture.FixtureUart.CONFIGURE, struct.pack("<I", 115200)) if False else None
    body = b"while closed\r\n"
    A.call(fu, 0x06, struct.pack("<H", len(body)) + body)
    time.sleep(0.3)
    p = serial.Serial(PORT, 2000000, timeout=0.05)
    later = drain(p, len(body), 1.0)
    print(f"INFO after reopening the port got {later!r} (backlog kept while closed?)")
    p.close()
    A.call(fb, 0x01, b"\x00")
    core.plan_release(A)
    A.end()
finally:
    A.link.close()
print("ALL PASS" if ok else "SOME FAILED")
