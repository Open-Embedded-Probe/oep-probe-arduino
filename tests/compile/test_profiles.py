#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Open Embedded Probe
"""Test the release workflow's actual profile discovery and model selection.

--matrix emits its P4/P4X build matrix for the test workflow.
"""
import json
from pathlib import Path
import re
import subprocess
import sys
import textwrap
import unittest

ROOT = Path(__file__).resolve().parents[2]
WORKFLOW = ROOT / ".github/workflows/firmware.yml"


def release_builds():
    scripts = re.findall(r"python3 - <<'PY'[^\n]*\n(.*?)^\s*PY\s*$",
                         WORKFLOW.read_text(), re.MULTILINE | re.DOTALL)
    output = subprocess.check_output([sys.executable, "-c", textwrap.dedent(scripts[0])],
                                     cwd=ROOT, text=True)
    return json.loads(next(line.removeprefix("builds=") for line in output.splitlines()
                           if line.startswith("builds=")))


class BuildProfilesTest(unittest.TestCase):
    def test_p4_and_p4x_cover_the_same_examples(self):
        builds = release_builds()
        p4 = {b["sketch"]: b for b in builds if b["profile"] == "esp32p4"}
        p4x = {b["sketch"]: b for b in builds if b["profile"] == "esp32p4x"}
        self.assertEqual(p4.keys(), p4x.keys())
        self.assertTrue({"examples/Firmware/OepProbe", "examples/03.Transports/MultipleTransports",
                         "examples/04.Debug/RvswdDebugProbe", "examples/05.Capture/LogicCapture"} <= p4.keys())
        for sketch in p4:
            with self.subTest(sketch=sketch):
                a, b = p4[sketch], p4x[sketch]
                self.assertIn("ChipVariant=prev3", a["fqbn"])
                self.assertEqual(a["fqbn"].replace("ChipVariant=prev3", "ChipVariant=postv3"), b["fqbn"])
                self.assertEqual(a["cache"], b["cache"])
                self.assertEqual(a["index_urls"], "https://espressif.github.io/arduino-esp32/package_esp32_index.json")
                self.assertEqual(a["index_urls"], b["index_urls"])
                self.assertNotEqual(a["id"], b["id"])
                self.assertEqual(a["release"], b["release"])
        self.assertTrue(p4x["examples/Firmware/OepProbe"]["release"])

    def test_release_model_distinguishes_chip_variant(self):
        source = WORKFLOW.read_text()
        begin = source.index("          found = dict(re.findall(")
        end = source.index("          # New tags verify", begin)
        code = textwrap.dedent(source[begin:end])
        for profile, props, expected in (
            ("esp32p4", "build.mcu=esp32p4\nbuild.chip_variant=esp32p4_es\n", "esp32p4"),
            ("esp32p4x", "build.mcu=esp32p4\nbuild.chip_variant=esp32p4\n", "esp32p4x"),
            ("promicrorp2350", "build.chip=rp2350\n", "rp2350"),
            ("esp32", "build.mcu=esp32\n", "esp32"),
        ):
            with self.subTest(profile=profile):
                scope = {"re": re, "props": props, "profile": profile}
                exec(code, scope)
                self.assertEqual(scope["chip"], expected)


if __name__ == "__main__":
    if sys.argv[1:] == ["--matrix"]:
        print("builds=" + json.dumps([b for b in release_builds()
                                     if b["profile"] in ("esp32p4", "esp32p4x")]))
    else:
        unittest.main()
