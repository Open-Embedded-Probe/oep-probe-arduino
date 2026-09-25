# /// script
# dependencies = ["pyserial>=3.5"]
# ///
"""v1 wire §5.5 on the X035 fixture: a lapsed lease drops the host's use of the connection (it closes when nothing else
uses it; a bind keeps it), and an automatic attach ignores chip_id bits [7:4] (the silicon revision)."""
import pathlib, struct, sys, time
HERE = pathlib.Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[4] / "oep-client-python" / "src"))
import serial  # noqa: E402
from oep_client.v1 import core, link, message as m, registry as reg, riscv  # noqa: E402

PORT = "/run/board-identify/by-id/esp32-series-30eda0e31108"
CFG = reg.PROBE_CONFIG
ok = True


def check(name, cond, extra=""):
    global ok
    ok &= bool(cond)
    print(f"{'PASS' if cond else 'FAIL'} {name} {extra}", flush=True)


def tlv(tag, v):
    return bytes([tag, len(v)]) + v


A = link.open_host(PORT)
fb, fc = core.find(A, "oep.test.console-bind"), core.find(A, "oep.probe.config")
wire = riscv.Wire(A, "oep.wire.rvswd")


def status():
    st, users, conn, open_, chip, gaps, dtr = struct.unpack("<BBBBIIB", A.call(fb, 0x01, locked=False).payload[:13])
    return {"state": st, "users": users, "connected": conn, "chip": chip}


def force_detach():
    if status()["connected"]:
        A.open(3000)
        A.call(wire.fn, 0x03, bytes([1]) + m.tlv(reg.WIRE_RVSWD.tlv["detach"]["force"], b"", critical=True))
        A.end()


force_detach()
# 1. no bind: attach with a 1 s lease, no keepalive
A.open(3000)
A.call(fc, CFG.op["set"], tlv(4, b"") + tlv(5, b""))
A.end()
A.open(1000)
wire.attach(halt=False)
check("attached, the host is a user", status()["users"] == 1, str(status()))
time.sleep(2.0)
A.lock_state()   # any request lets the probe notice the lapse (it also does in its own loop)
st = status()
check("lease lapsed: the host's use dropped, the link closed", st["connected"] == 0 and st["users"] == 0, str(st))
# 2. with a bind (attach 0): the link stays for it
A.open(3000)
A.call(fc, CFG.op["set"], tlv(4, bytes([0, 2, 0, 0]) + struct.pack("<HB", 1, 2)))
A.end()
A.open(1000)
wire.attach(halt=False)
time.sleep(0.3)
check("attached with a bind: host + bind", status()["users"] == 3, str(status()))
time.sleep(2.0)
st = status()
check("lease lapsed: the bind keeps the link", st["connected"] == 1 and st["users"] == 2, str(st))
force_detach()
# 3. attach on open with a target whose revision bits differ
A.open(3000)
chip = 0x035e0601 ^ 0x10
A.call(fc, CFG.op["set"], tlv(4, bytes([0, 2, 1, 0]) + struct.pack("<HB", 1, 2)) + tlv(5, struct.pack("<HI", 1, chip)))
A.end()
p = serial.Serial(sys.argv[1], 115200, timeout=0.05)
time.sleep(1.5)
st = status()
check("chip_id bits [7:4] are not compared", st["state"] == 1 and st["connected"] == 1, f"{st} target {chip:#010x}")
p.close()
force_detach()
A.open(3000)
A.call(fc, CFG.op["set"], tlv(4, b"") + tlv(5, b""))
A.end()
A.link.close()
print("ALL PASS" if ok else "SOME FAILED")
