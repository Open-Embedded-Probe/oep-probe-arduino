"""target.control / memory / flash against the CH32X035F8U6 fixture. Destructive to the last 2 KiB."""

import os
import time

from oep_client.v0 import codec
from oep_client.v0.services import TargetControl, TargetFlash, TargetMemory

X035_ESIG_CHIP_ID = 0x1FFFF704
X035_F8U6 = 0x035E0601


def _services(probe):
    probe.confirm()
    probe.list_functions()
    return (TargetControl(probe, probe.find(*codec.DEF_TARGET_CONTROL[:2]).function),
            TargetMemory(probe, probe.find(*codec.DEF_TARGET_MEMORY[:2]).function),
            TargetFlash(probe, probe.find(*codec.DEF_TARGET_FLASH[:2]).function))


def test_target_read_program_reset(probe):
    control, memory, flash = _services(probe)
    status = control.status()
    print(f"\nHIL status before: flags={status.flags}")
    control.halt()
    status = control.status()
    assert status.flags & 2, "not halted"
    assert memory.read_word(X035_ESIG_CHIP_ID) == X035_F8U6

    geometry = flash.geometry()
    assert (geometry.base, geometry.size, geometry.page) == (0x08000000, 63488, 256)
    t0 = time.perf_counter()
    image = memory.read_range(geometry.base, geometry.size)
    t1 = time.perf_counter()
    crc = flash.verify_crc32(geometry.base, geometry.size)
    t2 = time.perf_counter()
    print(f"HIL read 62 KiB via pipeline: {t1 - t0:.3f} s ({geometry.size / (t1 - t0) / 1000:.0f} kB/s); "
          f"probe verify_crc32: {t2 - t1:.3f} s")
    assert len(image) == geometry.size
    assert TargetFlash.crc32(image) == crc, "host CRC of the pipelined read differs from the probe's CRC"

    # program the last 8 physical pages with a fresh pattern, read them back
    seed = int.from_bytes(os.urandom(2), "little")
    pages = []
    for i in range(8):
        address = geometry.base + geometry.size - 8 * 256 + i * 256
        pages.append((address, bytes(((seed + i * 7 + b * 13) ^ (b >> 3)) & 0xFF for b in range(256))))
    t0 = time.perf_counter()
    failed = flash.program_pages(pages)
    t1 = time.perf_counter()
    print(f"HIL program 8 pages pipelined: {t1 - t0:.3f} s ({(t1 - t0) / 8 * 1000:.1f} ms/page) failed={failed}")
    assert not failed
    back = memory.read_range(pages[0][0], 8 * 256)
    assert back == b"".join(d for _, d in pages), "read-back differs from the programmed pattern"
    assert flash.verify_crc32(pages[0][0], 8 * 256) == TargetFlash.crc32(back)

    # RAM write / read
    ram = 0x20000000 + 0x1000
    data = bytes(range(64))
    assert memory.write(ram, data) == 64
    assert memory.read(ram, 64) == data

    control.reset()
    status = control.status()
    print(f"HIL status after reset: flags={status.flags}")
    assert not (status.flags & 2)
