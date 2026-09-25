# /// script
# dependencies = ["pyserial>=3.5", "pyusb>=1.3", "libusb1>=3", "hidapi>=0.14"]
# ///
"""P7: OEP throughput per USB configuration. For each boot mode of Esp32P4HsPrototype: save it, reboot, re-attach
(WSL + usbipd), then measure every way in the mode offers - link_speed both ways (full frames, the probe's in-flight
limit, `seconds` each) and the round trip of a lock-free request.

  usb_configs.py <usbipd busid> <device serial> [modes, default 1,2,3,0,4,5] [seconds]

Prints a Markdown table and the rows as JSON. Leaves the probe in mode 0 with the configuration erased.
"""
import json, os, subprocess, sys, time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "oep-client-python" / "src"))
from oep_client.v1 import core, link, registry as reg  # noqa: E402

BUSID, SERIAL = sys.argv[1], sys.argv[2]
MODES = [int(x) for x in (sys.argv[3] if len(sys.argv) > 3 else "1,2,3,0,4,5").split(",")]
SECONDS = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
CFG = reg.PROBE_CONFIG
NAMES = {0: "vendor+hid+cdc+1 data+msc", 1: "vendor", 2: "vendor+hid", 3: "vendor+hid+cdc", 4: "vendor+hid+cdc+2 data",
         5: "vendor+hid+cdc+3 data"}


def usbipd_state() -> str:
    out = subprocess.run(["usbipd.exe", "list"], capture_output=True, text=True).stdout
    for line in out.splitlines():
        if line.startswith(BUSID + " "):   # "Shared", "Shared (forced)", "Attached"
            return "Attached" if "Attached" in line else "Shared" if "Shared" in line else ""
    return ""


def reattach(timeout=90.0) -> float:
    """Until the probe answers on vendor bulk again: attach it when usbipd shows it Shared (after the Windows restart
    of 2026-09-25 usbipd re-attached it by itself)."""
    t0 = time.monotonic()
    time.sleep(1.5)
    while time.monotonic() - t0 < timeout:
        try:
            h = link.open_usb_host(serial=SERIAL, transports=("vendor",))
            h.link.close()
            return time.monotonic() - t0
        except Exception:
            if usbipd_state() == "Shared":
                subprocess.run(["usbipd.exe", "attach", "--wsl", "--busid", BUSID], capture_output=True)
            time.sleep(0.5)
    raise TimeoutError("the probe did not come back")


def cdc_tty() -> str | None:
    for p in sorted(Path("/dev").glob("ttyACM*")):
        props = subprocess.run(["udevadm", "info", "-q", "property", "-n", str(p)], capture_output=True, text=True).stdout
        if SERIAL in props and "ID_USB_INTERFACE_NUM=02" in props:
            return str(p)
    return None


def measure(hst) -> dict:
    r = core.link_speed(hst, seconds=SECONDS)
    t = time.perf_counter()
    for _ in range(200):
        hst.lock_state()
    return {"in_mb_s": round(r["in_mb_s"], 2), "out_mb_s": round(r["out_mb_s"], 2),
            "rtt_ms": round((time.perf_counter() - t) / 200 * 1e3, 3)}


def set_mode(m: int | None, erase=False):
    A = link.open_usb_host(serial=SERIAL, transports=("vendor",))
    try:
        A.open(5000)
        fc = core.find(A, "oep.probe.config")
        if erase:
            A.call(fc, CFG.op["erase"])
            A.call(fc, CFG.op["set"], bytes([CFG.tlv["item"]["boot_mode"], 0]))
        else:
            A.call(fc, CFG.op["set"], bytes([CFG.tlv["item"]["boot_mode"], 1, m]))
            A.call(fc, CFG.op["save"])
        A.call(fc, CFG.op["reboot"])
    finally:
        A.link.close()


rows = []
reattach()   # the probe may be Shared, not attached, to begin with
for m in MODES:
    set_mode(m)
    back = reattach()
    row = {"mode": m, "config": NAMES.get(m, str(m)), "reboot_s": round(back, 1)}
    for kind in ("vendor", "hid", "cdc"):
        try:
            if kind == "cdc":
                tty = cdc_tty()
                if not tty:
                    continue
                hst = link.open_host(tty)
            else:
                hst = link.open_usb_host(serial=SERIAL, transports=(kind,))
        except Exception as e:
            if kind == "hid" and "no vendor HID" in str(e):
                continue
            row[kind] = {"error": f"{type(e).__name__}: {e}"[:120]}
            continue
        try:
            row[kind] = measure(hst)
        except Exception as e:
            row[kind] = {"error": f"{type(e).__name__}: {e}"[:120]}
        finally:
            hst.link.close()
    print(json.dumps(row), flush=True)
    rows.append(row)

set_mode(None, erase=True)
reattach()


def cell(r, k, f):
    v = r.get(k)
    return "-" if v is None else v.get(f, "err") if isinstance(v, dict) else "-"


print("\n| mode | USB configuration | vendor in / out MB/s, rtt ms | HID in / out, rtt | CDC in / out, rtt |")
print("|---:|---|---|---|---|")
for r in rows:
    cols = [f"{cell(r, k, 'in_mb_s')} / {cell(r, k, 'out_mb_s')}, {cell(r, k, 'rtt_ms')}" if isinstance(r.get(k), dict) and "error" not in r[k]
            else ("-" if k not in r else r[k]["error"]) for k in ("vendor", "hid", "cdc")]
    print(f"| {r['mode']} | {r['config']} | " + " | ".join(cols) + " |")
