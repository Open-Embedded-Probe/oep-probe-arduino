"""HS bulk probe: the dut is the UART console (flash + banner); OEP runs over the vendor bulk pair."""

import json
import subprocess
import time

import pytest

from oep_client.v0 import Client
from oep_client.v0.transport import BulkTransport

VID, PID, SERIAL = 0x303A, 0x4021, "e104-p4-windows-v1"


def _attach_via_usbipd():
    """After a reflash the device re-enumerates on Windows; re-attach it (already bound) by bus id."""
    try:
        state = subprocess.run(["usbipd.exe", "state"], capture_output=True, text=True, timeout=20).stdout
        for d in json.loads(state).get("Devices", []):
            if f"PID_{PID:04X}" in (d.get("InstanceId") or "").upper() and d.get("BusId") and d.get("ClientIPAddress") is None:
                subprocess.run(["usbipd.exe", "attach", "--wsl", "--busid", d["BusId"]], capture_output=True, text=True, timeout=30)
                return True
    except Exception:  # noqa: BLE001
        return False
    return False


@pytest.fixture
def probe(dut):
    import re
    dut.expect(re.compile(rb"# OEP P4 HS probe usb_begin=1"), timeout=30)
    transport = None
    for attempt in range(6):
        try:
            transport = BulkTransport.open(VID, PID, SERIAL, timeout_s=3.0)
            break
        except ConnectionError:
            _attach_via_usbipd()
            time.sleep(1.0)
    if transport is None:
        pytest.skip("HS bulk probe not attached to WSL (usbipd)")
    transport.discard_input()
    client = Client(transport, timeout=3.0)
    try:
        yield client
    finally:
        transport.close()
