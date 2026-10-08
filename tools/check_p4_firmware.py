#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Open Embedded Probe
"""Check P4/P4X identity in the OepProbe artifacts built with the pinned SDK."""
import argparse
from pathlib import Path
import struct


def check(build: Path, profile: str) -> None:
    variant, min_rev, max_rev, product = {
        "esp32p4": (1, 1, 199, "ESP32-P4"),
        "esp32p4x": (2, 301, 399, "ESP32-P4X"),
    }[profile]
    image = (build / "OepProbe.ino.bin").read_bytes()
    # ESP image header + first segment header + esp_app_desc_t.
    offset = 24 + 8 + 256
    if len(image) < offset + 16 or image[0] != 0xE9:
        raise ValueError("missing or truncated ESP app image")
    if struct.unpack_from("<I", image, 32)[0] != 0xABCD5432:
        raise ValueError("ESP app descriptor is not at the expected offset")
    identity = struct.unpack_from("<IHHHHI", image, offset)
    if identity != (0x4650454F, 1, variant, min_rev, max_rev, 0):
        raise ValueError(f"{profile}: wrong or missing DFU identity: {identity}")
    elf = (build / "OepProbe.ino.elf").read_bytes()
    for value in (profile, f"OEP probe ({product})"):
        if (value + "\0").encode() not in elf:
            raise ValueError(f"{profile}: missing model/product {value!r}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("build", type=Path)
    parser.add_argument("profile", choices=("esp32p4", "esp32p4x"))
    args = parser.parse_args()
    check(args.build, args.profile)
    print(f"{args.profile}: DFU identity, revision bounds, model and USB product verified")
