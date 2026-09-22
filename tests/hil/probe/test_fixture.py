"""fixture.gpio / fixture.uart with the peer P4 over the 8-wire link (E154 pins)."""

import time

from oep_client.v0 import codec
from oep_client.v0.services import FixtureGpio, FixtureUart
from oep_client.v0 import tlv

UART_RX, UART_TX = 27, 26      # probe side; peer echoes on 26 (its RX) -> 27 (its TX)
MIRROR_IN, MIRROR_OUT = 28, 29  # probe drives 28, peer mirrors onto 29


def test_fixture_gpio_uart_lease(probe, peers):
    peer = peers["p4b"]
    peer.write("?")
    peer.expect_exact("# HIL peer_p4b", timeout=10)
    probe.confirm(); probe.list_functions()
    gpio = FixtureGpio(probe, probe.find(*codec.DEF_FIXTURE_GPIO[:2]).function)
    uart = FixtureUart(probe, probe.find(*codec.DEF_FIXTURE_UART[:2]).function)
    candidates = tlv.channel_candidates(probe.describe(uart.function))
    print(f"\nHIL uart candidates: {len(candidates)} channels, includes 26/27: {26 in candidates and 27 in candidates}")
    assert 2 not in candidates and 54 not in candidates, "reserved RVSWD pins must not be offered"

    # GPIO drive / sample through the peer mirror
    for level, mode in ((1, FixtureGpio.OUTPUT_HIGH), (0, FixtureGpio.OUTPUT_LOW), (1, FixtureGpio.OUTPUT_HIGH)):
        gpio.configure(MIRROR_IN, mode)
        gpio.configure(MIRROR_OUT, FixtureGpio.INPUT_FLOATING)
        time.sleep(0.01)
        assert gpio.read(MIRROR_OUT) == level, f"mirror did not follow level {level}"
    print("HIL gpio mirror 28->29: ok")

    # UART needs a lease; without one, configure is rejected
    assert probe.call(uart.function, codec.FIXTURE_UART_OP_CONFIGURE,
                      codec.FixtureUartConfigureRequest(baud=115200).pack()).detail == codec.REJECT_UNAVAILABLE
    lease, effective = probe.plan_apply(uart.assignments(rx=UART_RX, tx=UART_TX))
    print(f"HIL lease {lease} effective={tlv.decode(effective)}")
    # a second plan while one is active is refused; GPIO may not take a leased pin
    assert probe.call(codec.DEF_CORE_FUNCTION, codec.CORE_OP_PLAN_APPLY,
                      codec.CorePlanApplyRequest(tlv=tlv.encode(tlv.role_assignment(*a) for a in uart.assignments(30, 31))).pack()).detail == codec.REJECT_UNAVAILABLE
    assert probe.call(gpio.function, codec.FIXTURE_GPIO_OP_CONFIGURE,
                      codec.FixtureGpioConfigureRequest(channel=UART_TX, mode=FixtureGpio.OUTPUT_LOW).pack()).detail == codec.REJECT_UNAVAILABLE
    assert uart.configure(115200) == 115200
    uart.read()  # drain
    message = b"PING via OEP fixture.uart\n"
    assert uart.write(message) == len(message)
    echoed = uart.read_until(b"\n")
    print(f"HIL uart echo: {echoed!r}")
    assert echoed == message
    payload = bytes(range(256)) * 2
    assert uart.write(payload) == len(payload)
    time.sleep(0.1)
    back = bytearray()
    while len(back) < len(payload):
        chunk = uart.read(512)
        if not chunk:
            break
        back += chunk
    assert bytes(back) == payload, "512-byte binary echo differs"
    print("HIL uart 512-byte binary echo: ok")

    probe.plan_release(lease)
    assert probe.call(uart.function, codec.FIXTURE_UART_OP_WRITE,
                      codec.FixtureUartWriteRequest(data=b"x").pack()).detail == codec.REJECT_UNAVAILABLE
    # after release the pins are plain fixture GPIO again
    gpio.configure(UART_TX, FixtureGpio.INPUT_FLOATING)
    print("HIL lease release: ok")
