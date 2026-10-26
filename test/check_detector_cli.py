#!/usr/bin/env python3
"""Exercise the actual MVS executable offline, without a camera or Python OpenCV."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def make_scene(path, lights):
    width, height = 960, 480
    pixels = bytearray(width * height * 3)
    for x, rgb in lights:
        for y in range(120, 220):
            begin = (y * width + x) * 3
            pixels[begin : begin + 14 * 3] = bytes(rgb) * 14
    path.write_bytes(f"P6\n{width} {height}\n255\n".encode() + pixels)


def run(binary):
    # tuples describe RGB rectangles, expected detection count and per-armor color.
    red = [(80, (255, 0, 0)), (260, (255, 0, 0))]
    high_red = [(80, (255, 0, 8)), (260, (255, 0, 8))]
    blue = [(550, (0, 160, 255)), (730, (0, 160, 255))]
    mixed = [red[0], (260, (0, 160, 255))]
    cases = [
        ("red", red, ["red"]),
        ("red", high_red, ["red"]),
        ("blue", red, []),
        ("blue", blue, ["blue"]),
        ("red", blue, []),
        ("both", red + blue, ["blue", "red"]),
        ("both", mixed, []),
        ("red", [], []),
    ]
    with tempfile.TemporaryDirectory(prefix="armor-cli-test-") as directory:
        root = Path(directory)
        for index, (mode, lights, expected) in enumerate(cases):
            image = root / f"case-{index}.ppm"
            status = root / f"case-{index}.json"
            make_scene(image, lights)
            env = dict(os.environ, ARMOR_STATUS_FILE=str(status))
            result = subprocess.run(
                [binary, "--image", str(image), "--color", mode],
                env=env, capture_output=True, timeout=10, check=True,
            )
            value = json.loads(status.read_text())
            assert value["mode"] == mode, value
            assert value["colors"] == expected, value
            assert value["armors"] == len(expected), value
            assert len(value["corners"]) == 4 * len(expected), value
            assert value["red_armors"] == expected.count("red"), value
            assert value["blue_armors"] == expected.count("blue"), value
            assert value["detected"] == bool(expected), value
            assert (value["width"], value["height"]) == (960, 480), value
            assert result.stdout.startswith(b"\xff\xd8"), "missing JPEG frame"
            assert result.stdout.endswith(b"\xff\xd9"), "truncated JPEG frame"
        invalid = subprocess.run(
            [binary, "--color", "purple"], capture_output=True, timeout=5,
        )
        assert invalid.returncode != 0
        assert b"--color must be" in invalid.stderr
    print("PASS: 8 actual-executable image/JSON/JPEG cases and invalid color rejected")


if __name__ == "__main__":
    run(sys.argv[1])
