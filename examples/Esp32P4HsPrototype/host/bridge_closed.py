# /// script
# dependencies = ["pyserial>=3.5", "pyusb>=1.3", "libusb1>=3"]
# ///
import struct, sys, time
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[4] / "oep-client-python" / "src"))
import serial
from oep_client.v1 import console, core, fixture, link
PORT, TX, RX = sys.argv[1], 20, 21
A = link.open_usb_host(transports=("vendor",))
try:
    A.open(5000)
    fu, fb = core.find(A, "oep.fixture.uart"), core.find(A, "oep.test.bridge")
    core.plan_apply(A, [(fu, 1, RX), (fu, 2, TX)])
    u = fixture.FixtureUart(A, fu)
    u.configure(115200)
    A.call(fb, 0x02, bytes([1, TX])); A.call(fb, 0x01, b"\x01")
    start = u.read(console.PositionStream.FROM_NOW, 0, 0).start
    body = b"while closed\r\n"
    A.call(fu, 0x06, struct.pack("<H", len(body)) + body)     # the port was never opened
    time.sleep(0.3)
    print("stream:", u.read_from(start, 100).data)
    p = serial.Serial(PORT, 115200, timeout=0.05)
    got, t = b"", time.monotonic()
    while time.monotonic() - t < 1.0:
        got += p.read(256)
    print("port after open:", got)
    print("stream after open:", u.read_from(start, 200).data)
    print("status baud/gaps/coding:", struct.unpack("<III", A.call(fb, 0x03).payload[:12]))
    A.call(fu, 0x06, struct.pack("<H", 6) + b"open\r\n"); time.sleep(0.2)
    print("port while open:", p.read(256))
    p.close()
    A.call(fb, 0x01, b"\x00"); core.plan_release(A); A.end()
finally:
    A.link.close()
