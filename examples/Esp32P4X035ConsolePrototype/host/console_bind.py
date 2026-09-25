# /// script
# dependencies = ["pyserial>=3.5"]
# ///
"""P6 on the X035 fixture (Esp32P4X035ConsolePrototype): the target's dmseq console forwarded to the HS "Target
console" CDC port by a bind (oep.probe.config source 2), and the connection kept for it across a host's detach and
reset. One phase per run (a reboot needs the shell to re-attach usbipd in between):

  console_bind.py flash            build the core's HelloDMSeq and program it (host-side ch32_flash, oep_smoke's path)
  console_bind.py host <tty>       attach 0: the host attaches -> console on the port; plain detach keeps it; typing
                                   comes back uppercased; a host reset keeps it; detach with force ends it
  console_bind.py open <tty>       attach 1: a wrong target is refused (chip_id), the right one opens on DTR
  console_bind.py boot-set         attach 2 + target saved (then reboot the probe and re-attach)
  console_bind.py boot <tty>       after the reboot: the console flows with no host
  console_bind.py clear            bind, target and the saved copy removed

Needs the ArduinoCore-CH32 checkout beside dev_oep (../../dev_wch/ArduinoCore-CH32) for the build and flash path.
"""
import pathlib, struct, sys, tempfile, time

HERE = pathlib.Path(__file__).resolve()
DEV = HERE.parents[5]
sys.path.insert(0, str(HERE.parents[4] / "oep-client-python" / "src"))
SMOKE = DEV / "dev_wch" / "ArduinoCore-CH32" / "tests" / "manual" / "oep_smoke"
sys.path.insert(0, str(SMOKE))
import serial  # noqa: E402
from oep_client.v1 import core, link, message as m, registry as reg, riscv  # noqa: E402

PORT = "/run/board-identify/by-id/esp32-series-30eda0e31108"
CFG = reg.PROBE_CONFIG
SET, SAVE, ERASE = CFG.op["set"], CFG.op["save"], CFG.op["erase"]
T_BIND, T_TARGET = CFG.tlv["item"]["bind"], CFG.tlv["item"]["target"]
WIRE_FN = 1
ok = True


def check(name, cond, extra=""):
    global ok
    ok &= bool(cond)
    print(f"{'PASS' if cond else 'FAIL'} {name} {extra}", flush=True)


def tlv(tag, v):
    return bytes([tag, len(v)]) + v


def bind(attach, mechanism=2):
    return tlv(T_BIND, bytes([0, 2, attach, 0]) + struct.pack("<HB", WIRE_FN, mechanism))


def status(A):
    st, users, conn, open_, chip, gaps, dtr = struct.unpack("<BBBBIIB", A.call(core.find(A, "oep.test.console-bind"), 0x01, locked=False).payload[:13])
    return {"state": st, "users": users, "connected": conn, "open": open_, "chip": chip, "gaps": gaps, "dtr": dtr}


def listen(p, seconds):
    got, t = b"", time.monotonic()
    while time.monotonic() - t < seconds:
        got += p.read(256)
    return got


phase = sys.argv[1]
if phase == "flash":
    import oep_smoke
    from targets import TARGETS
    profile = TARGETS["x035"]
    src = DEV / "dev_wch" / "ArduinoCore-CH32" / "libraries" / "SerialDMSeq" / "examples" / "HelloDMSeq"
    with tempfile.TemporaryDirectory() as tmp:
        binary = oep_smoke.build("HelloDMSeq", profile["fqbn"], pathlib.Path(tmp), print, source=src)
        bench = oep_smoke.Bench(PORT, profile)
        try:
            outcome, conn, dm = oep_smoke.program(bench, binary.read_bytes(), profile, print)
            print("program:", outcome.as_dict())
            dm.reset(confirm=True)
            bench.wire.detach(conn)
        finally:
            bench.close()
    sys.exit(0)

A = link.open_host(PORT)
try:
    A.open(10000)
    fc = core.find(A, "oep.probe.config")
    wire = riscv.Wire(A, "oep.wire.rvswd")
    print("status at start:", status(A))
    if phase in ("host", "open") and status(A)["connected"]:   # start clean: a link an earlier run left goes
        A.call(wire.fn, 0x03, bytes([1]) + m.tlv(reg.WIRE_RVSWD.tlv["detach"]["force"], b"", critical=True))
    if phase == "host":
        A.call(fc, SET, bind(0) + tlv(T_TARGET, b""))
        p = serial.Serial(sys.argv[2], 115200, timeout=0.05)
        time.sleep(0.3)
        p.reset_input_buffer()
        quiet = listen(p, 1.5)
        check("nothing on the port before a host attaches", b"uptime" not in quiet, repr(quiet[-40:]))
        conn, _ = wire.attach(halt=False)
        time.sleep(0.2)
        got = listen(p, 2.5)
        check("host attach -> console on the port", b"uptime" in got, repr(got[-60:]))
        print("  status:", status(A))
        wire.detach(conn)
        st = status(A)
        check("plain detach keeps the link for the bind", st["connected"] == 1 and st["users"] == 2, str(st))
        got = listen(p, 2.5)
        check("the console goes on after the host's detach", b"uptime" in got, repr(got[-60:]))
        p.write(b"hello\n")
        got = listen(p, 2.0)
        check("typing on the port reaches the target (uppercased back)", b"HELLO" in got, repr(got[-60:]))
        conn, _ = wire.attach(halt=False)   # joins the bind's connection
        riscv.RiscvDm(A, conn).reset(confirm=True)
        got = listen(p, 3.0)
        st = status(A)
        check("a host reset keeps the connection and the console", b"hello from the debug module" in got and st["connected"] == 1,
              f"{st} {got[-80:]!r}")
        A.call(wire.fn, 0x03, bytes([conn]) + m.tlv(reg.WIRE_RVSWD.tlv["detach"]["force"], b"", critical=True))
        st = status(A)
        check("detach with force ends it", st["connected"] == 0, str(st))
        p.reset_input_buffer()
        got = listen(p, 2.0)
        check("no console after the forced detach", b"uptime" not in got, repr(got[-40:]))
        p.close()
    elif phase == "open":
        A.call(fc, SET, bind(1) + tlv(T_TARGET, struct.pack("<HI", WIRE_FN, 0x12345678)))
        p = serial.Serial(sys.argv[2], 115200, timeout=0.05)
        time.sleep(1.0)
        st = status(A)
        check("attach on open, wrong target: refused, link released", st["state"] == 2 and st["connected"] == 0, f"{st} chip {st['chip']:#010x}")
        chip = st["chip"]
        p.close()
        A.call(fc, SET, tlv(T_TARGET, struct.pack("<HI", WIRE_FN, chip)))
        p = serial.Serial(sys.argv[2], 115200, timeout=0.05)
        t0 = time.monotonic()
        got = b""
        while b"uptime" not in got and time.monotonic() - t0 < 10:   # after a refused attach the target's dmseq
            got += listen(p, 0.5)                                     # took 5-6 s to come back (§7.5)
        st = status(A)
        print(f"   first console line {time.monotonic() - t0:.1f} s after opening")
        check("right target: console on DTR", st["state"] == 1 and b"uptime" in got, f"{st} {got[-60:]!r}")
        p.close()
        time.sleep(0.3)
        st = status(A)
        check("closing the port keeps the console reading (SDI must not stall)", st["connected"] == 1 and st["open"] == 1, str(st))
        A.call(wire.fn, 0x03, bytes([1]) + m.tlv(reg.WIRE_RVSWD.tlv["detach"]["force"], b"", critical=True))
    elif phase == "boot-set":
        st = status(A)
        chip = st["chip"] or int(sys.argv[2], 0) if len(sys.argv) > 2 else st["chip"]
        h = struct.unpack("<I", A.call(fc, SET, bind(2) + tlv(T_TARGET, struct.pack("<HI", WIRE_FN, chip))).payload)[0]
        A.call(fc, SAVE)
        print(f"saved attach-at-boot bind, target chip {chip:#010x}, hash {h:#x}; reboot the probe now")
        A.call(fc, CFG.op["reboot"])
    elif phase == "boot":
        st = status(A)
        check("after reboot, attached by itself", st["state"] == 1 and st["connected"] == 1 and st["users"] & 2, str(st))
        p = serial.Serial(sys.argv[2], 115200, timeout=0.05)
        got = listen(p, 2.5)
        check("the console flows with no host", b"uptime" in got, repr(got[-60:]))
        p.close()
    elif phase == "clear":
        A.call(fc, ERASE)
        A.call(fc, SET, tlv(T_BIND, b"") + tlv(T_TARGET, b""))
        print("cleared")
    try:
        A.end()
    except Exception:
        pass
finally:
    A.link.close()
print("ALL PASS" if ok else "SOME FAILED")
