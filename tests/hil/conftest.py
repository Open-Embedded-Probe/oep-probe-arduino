"""Hand the harness's serial handle to the OEP client without reopening the port."""

import time

import pytest

from oep_client.v0 import Client, FrameTransport


@pytest.fixture
def probe(dut):
    """OEP v0 client over the dut's own pyserial object. The redirect thread is paused
    because the wire is binary and because opening the port again would reset the P4."""
    dut.serial._redirect_thread.stop_reading()
    time.sleep(0.2)
    stream = dut.serial.proc
    transport = FrameTransport(stream)
    transport.discard_input()
    client = Client(transport, timeout=3.0)
    try:
        yield client
    finally:
        dut.serial._redirect_thread.start_reading()
