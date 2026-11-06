"""The classic ESP32 firmware on its UART bridge (UART0 through the board's USB-UART), end to end: the plugin builds the
firmware (this folder's sketch includes examples/Firmware/OepProbe) and uploads it through that port, under its device
lock; then OEP must answer there as a UART bridge, with the Wi-Fi choice's TCP listener made at boot and listed after
it (its slots' buffers are taken when it is made). Any classic ESP32 board with a USB-UART on UART0 (the bench's 1-1).
A one-shot test (tests/oneshot, not collected by default): run it by name, naming the port,

  cd tests && uv run pytest oneshot/esp32_uart_bridge --profile esp32 --port /dev/ttyUSB0
"""
import time

import pytest
from oep_client import core, link


@pytest.fixture
def hst(request):
    deadline = time.monotonic() + 30
    while True:
        try:
            h = link.open_host(request.config.option.port)
            core.confirm(h)
            break
        except Exception:
            if time.monotonic() >= deadline:
                raise
            time.sleep(1)
    yield h
    h.link.close()


def test_oep_answers_on_the_uart_bridge_and_lists_the_tcp_listener(hst):
    kinds = {index: kind for index, kind, _ in core.transports(hst)}
    assert kinds[hst.limits["transport"]] == core.TRANSPORT_KIND["uart_bridge"]
    assert core.TRANSPORT_KIND["tcp"] in kinds.values()
