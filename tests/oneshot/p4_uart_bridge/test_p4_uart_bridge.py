"""The ESP32-P4 / P4X firmware's UART bridge (UART3 on the chip's default UART pads, Esp32P4.h oepAddBridge), end to end
on a board with a USB-UART there (the bench's 2-3, GUITION JC-ESP32P4-M3-DEV: a CH340C with the auto reset). The plugin
builds the firmware (this folder's sketch includes examples/Firmware/OepProbe) and uploads it through that port (the
boot ROM's download), under its device lock; then OEP must answer on the same port, as a UART bridge, and the bridge's
pins must never be offered. A one-shot test (tests/oneshot, not collected by default): run it by name, naming the
USB-UART's port,

  cd tests && uv run pytest oneshot/p4_uart_bridge --profile esp32p4 --port /dev/ttyUSB2
"""
import struct
import time

import pytest
from oep_client import core, fixture, link, registry
from oep_client.host import Rejected

BRIDGE_PINS = ("GPIO37", "GPIO38")   # the chip's default UART pads (Esp32P4.h kBridgeTx / kBridgeRx)
CHANNEL_NAME = registry.CORE.tlv["describe"]["channel_name"]


def _open(port, seconds=30):
    """The probe once it answers on the port after the upload's reset (the boot, then the bridge's begin)."""
    deadline = time.monotonic() + seconds
    while True:
        try:
            hst = link.open_host(port)
            core.confirm(hst)
            return hst
        except Exception:
            if time.monotonic() >= deadline:
                raise
            time.sleep(1)


def _channels(hst, names):
    found = {}
    for tag, value in core.describe(hst):
        if tag == CHANNEL_NAME and len(value) > 2:
            found[value[2:].decode()] = struct.unpack_from("<H", value)[0]
    return [found[n] for n in names]


@pytest.fixture
def hst(request):
    h = _open(request.config.option.port)
    yield h
    try:
        if h.session is not None:   # only a test that took the lock ends a session
            h.end()
    finally:
        h.link.close()


def test_oep_answers_on_the_uart_bridge(hst):
    here = hst.limits["transport"]
    kinds = {index: kind for index, kind, _ in core.transports(hst)}
    assert here in kinds, f"confirm's transport {here} is not among {kinds}"
    assert kinds[here] == core.TRANSPORT_KIND["uart_bridge"]


def test_the_bridge_pins_are_never_offered(hst):
    core.take(hst, 10_000, owner="oep-probe-arduino uart_bridge")
    gpio = fixture.Gpio(hst)
    line = registry.FIXTURE_GPIO.enum["role"]["line"]
    for channel in _channels(hst, BRIDGE_PINS):
        with pytest.raises(Rejected):
            core.plan_apply(hst, [(gpio.fn, line, channel)])
