# /// script
# dependencies = ["pyserial>=3.5", "pyusb>=1.3", "libusb1>=3"]
# ///
"""P4: oep.probe.config on Esp32P4HsPrototype. One phase per run; the shell reboots / re-attaches in between."""
import binascii, os, struct, sys, threading, time
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[4] / "oep-client-python" / "src"))
import serial
from oep_client.v1 import catalog, core, fixture, link, registry as reg

CFG = reg.PROBE_CONFIG
GET, SET, SAVE, ERASE, REBOOT = (CFG.op[k] for k in ("get", "set", "save", "erase", "reboot"))
T_MODE, T_PLAN, T_BIND = (CFG.tlv["item"][k] for k in ("boot_mode", "plan", "bind"))
TX, RX = 20, 21

def tlv(tag, v): return bytes([tag, len(v)]) + v
def items_split(b):
    out, at = [], 0
    while at < len(b):
        out.append((b[at], b[at + 2:at + 2 + b[at + 1]])); at += 2 + b[at + 1]
    return out
def canonical(items):   # (tag, value) -> sorted by tag, then first key (bind: port)
    return b"".join(tlv(t, v) for t, v in sorted(items, key=lambda i: (i[0], i[1][:1] if i[0] == T_BIND else b"")))

def describe(A, fn):
    data, first = b"", 0
    while True:
        r = A.request(0, 0x03, catalog.pack_describe_request(fn, first), locked=False).payload
        data += r[1:]; first += len(catalog.split_tlv(r[1:]))
        if not r[0] or len(r) == 1: return catalog.split_tlv(data)

def get(A, fc):
    p = A.call(fc, GET, struct.pack("<H", 0), locked=False).payload
    return p[0], struct.unpack_from("<I", p, 1)[0], items_split(p[5:])

ok = True
def check(name, cond, extra=""):
    global ok
    ok &= bool(cond); print(f"{'PASS' if cond else 'FAIL'} {name} {extra}", flush=True)

phase = sys.argv[1]
A = link.open_usb_host(transports=("vendor",))
try:
    A.open(5000)
    fc, fu = core.find(A, "oep.probe.config"), core.find(A, "oep.fixture.uart")
    d = describe(A, fc)
    print("describe:", [(hex(t), v.hex()) for t, v in d])
    more, h, items = get(A, fc)
    print("get:", hex(h), [(hex(t), v.hex()) for t, v in items])
    if phase == "setup":
        A.call(fc, ERASE)   # the saved copy only: clear the current items too
        A.call(fc, SET, tlv(T_MODE, b"") + tlv(T_BIND, b"") + tlv(T_PLAN, b""))
        want = [(T_PLAN, struct.pack("<HBH", fu, 1, RX) + struct.pack("<HBH", fu, 2, TX)),
                (T_BIND, bytes([0, 1, 0, 1]) + struct.pack("<H", fu))]
        h = struct.unpack("<I", A.call(fc, SET, b"".join(tlv(t, v) for t, v in reversed(want))).payload)[0]
        mine = binascii.crc32(canonical(want))
        check("set hash = host's CRC-32 of the canonical form", h == mine, f"{h:#x} {mine:#x}")
        t = time.monotonic(); hs = struct.unpack("<I", A.call(fc, SAVE).payload)[0]
        check("save", hs == h, f"{(time.monotonic() - t) * 1e3:.0f} ms")
        t = time.monotonic(); A.call(fc, SAVE)
        print(f"INFO second save (same content, not written) {(time.monotonic() - t) * 1e3:.1f} ms")
        try:
            A.call(fc, SET, tlv(0x03, b"\x00\x00ab"))
            check("label refused (not in this prototype)", False)
        except Exception as e:
            check("label refused (not in this prototype)", "nsupported" in type(e).__name__ or "Unsupported" in str(type(e)), type(e).__name__)
    elif phase == "check":   # after a reboot: the saved plan and bind back, the bridge working
        check("items back after reboot", {t for t, _ in items} >= {T_PLAN, T_BIND}, hex(h))
        A.call(core.find(A, "oep.test.bridge"), 0x02, bytes([1, TX]))
        port = sys.argv[2]
        p = serial.Serial(port, 460800, timeout=0.05); time.sleep(0.3); p.reset_input_buffer()
        st = struct.unpack("<III", A.call(core.find(A, "oep.test.bridge"), 0x03).payload[:12])
        check("line coding ran the UART (bind flag bit0)", abs(st[0] - 460800) < 20000, str(st))
        p.write(b"after reboot\r\n"); got, t = b"", time.monotonic()
        while len(got) < 14 and time.monotonic() - t < 2: got += p.read(64)
        check("bridge echo after reboot", got == b"after reboot\r\n", repr(got))
        p.close()
    elif phase.startswith("mode"):
        m = int(phase[4:])
        extra = sys.argv[2:]
        body = tlv(T_MODE, bytes([m]))
        if "clearbind" in extra: body += tlv(T_BIND, b"")
        h = struct.unpack("<I", A.call(fc, SET, body).payload)[0]
        A.call(fc, SAVE)
        print(f"INFO set boot_mode {m} saved, hash {h:#x}; rebooting")
        A.call(fc, REBOOT)
    elif phase == "erase":
        A.call(fc, ERASE); A.call(fc, SET, tlv(T_MODE, b"") + tlv(T_BIND, b"") + tlv(T_PLAN, b""))
        print("erased")
    elif phase == "reboot":
        A.call(fc, REBOOT)
    try:
        A.end()
    except Exception:
        pass
finally:
    A.link.close()
print("ALL PASS" if ok else "SOME FAILED")
