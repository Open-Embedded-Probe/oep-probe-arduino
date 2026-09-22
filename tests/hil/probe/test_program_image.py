"""program_image / verify_image on the X035 fixture: diff, pipelined pages, CRC verify, reset."""

import os

from oep_client.v0.flash_image import Target, program_image, verify_image


def test_program_image_diff_and_verify(probe):
    target = Target(probe)
    part, chip_id, size = target.preflight()
    print(f"\nHIL preflight: {part} 0x{chip_id:08x} {size} bytes")
    assert part == "CH32X035F8U6"
    geometry = target.flash.geometry()
    current = target.memory.read_range(geometry.base, geometry.size)
    # keep the resident image, replace the last 4 KiB with fresh random data
    image = bytearray(current)
    image[-4096:] = os.urandom(4096)
    outcome = program_image(target, bytes(image))
    print(f"HIL program_image: changed={outcome.pages_changed}/{outcome.pages_total} failed={outcome.pages_failed} "
          f"verified={outcome.verified} timings={ {k: round(v, 3) for k, v in outcome.timings.items()} }")
    assert outcome.verified and outcome.pages_changed == 16
    # identical image: no page programmed, still verifies
    again = program_image(target, bytes(image))
    print(f"HIL program_image (same): changed={again.pages_changed} verified={again.verified}")
    assert again.verified and again.pages_changed == 0
    check = verify_image(target, bytes(image))
    print(f"HIL verify_image: verified={check.verified} timings={ {k: round(v, 3) for k, v in check.timings.items()} }")
    assert check.verified
    assert not (target.control.status().flags & 2), "target must be running after reset"
