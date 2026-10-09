#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Write one verified BOOTSEL mount; never scan or write other mounted boards."""
import json
from pathlib import Path
import shutil
import os
import struct
import sys

from probe_upload import verify_uf2_mount


def copy_uf2(image, drive, location):
    verify_uf2_mount(drive, location)
    if not (drive / 'INFO_UF2.TXT').is_file():
        raise RuntimeError('not a BOOTSEL drive')
    with image.open('rb') as source:
        header = source.read(512)
        if len(header) != 512 or image.stat().st_size % 512 or struct.unpack_from('<II', header) != (0x0A324655, 0x9E5D5157):
            raise ValueError('not a complete UF2 image')
        source.seek(0)
        with (drive / 'NEW.UF2').open('wb') as dest:
            shutil.copyfileobj(source, dest, 64 * 1024)
            dest.flush()
            os.fsync(dest.fileno())
    print(f'Wrote {image.stat().st_size} bytes to the verified BOOTSEL mount')


if __name__ == '__main__':
    copy_uf2(Path(sys.argv[1]), Path(sys.argv[2]), json.loads(sys.argv[3]))
