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


def write_test_calibration(path, verified=1):
    template = """%YAML:1.0
---
calibration_verified: VERIFIED
image_width: 960
image_height: 480
camera_matrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [1000., 0., 480., 0., 1000., 240., 0., 0., 1.]
distortion_coefficients: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [0., 0., 0., 0., 0.]
armor_width_m: 0.135
armor_height_m: 0.056
max_reprojection_error_px: 4.
"""
    path.write_text(template.replace("VERIFIED", str(verified)))


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
            assert value["geometry"]["reason"] == "calibration_missing", value
            assert value["geometry"]["poses"] == [], value
            assert value["geometry"]["tracks"] == [], value
            assert value["image_tracking"]["units"] == "pixels", value
            assert len(value["image_tracking"]["tracks"]) == expected.count("blue"), value
            for track in value["image_tracking"]["tracks"]:
                assert track["predicted_center_px"] is None, value
            assert result.stdout.startswith(b"\xff\xd8"), "missing JPEG frame"
            assert result.stdout.endswith(b"\xff\xd9"), "truncated JPEG frame"
        # A known synthetic blue rectangle verifies actual-executable PnP and JSON.
        image = root / "blue-pose.ppm"
        make_scene(image, [(200, (0, 160, 255)), (440, (0, 160, 255))])
        config = root / "calibration.yml"
        for verified in [0, 1]:
            write_test_calibration(config, verified)
            status = root / "blue-pose.json"
            env = dict(os.environ, ARMOR_STATUS_FILE=str(status))
            subprocess.run(
                [binary, "--image", str(image), "--color", "blue",
                 "--calibration", str(config), "--prediction-ms", "80"],
                env=env, capture_output=True, timeout=10, check=True,
            )
            value = json.loads(status.read_text())
            geometry = value["geometry"]
            assert geometry["frame"] == "camera_optical"
            assert geometry["prediction_horizon_ms"] == 80
            if not verified:
                assert geometry["reason"] == "calibration_unverified", value
                assert not geometry["poses"] and not geometry["tracks"], value
            else:
                assert geometry["reason"] == "ready" and len(geometry["poses"]) == 1, value
                pose = geometry["poses"][0]
                assert 0.5 < pose["position_m"][2] < 0.7, value
                assert pose["reprojection_error_px"] < 4, value
                assert len(geometry["tracks"]) == 1
                assert geometry["tracks"][0]["predicted_position_m"] is None, value
                assert not geometry["tracks"][0]["confirmed"], value
        invalid = subprocess.run(
            [binary, "--color", "purple"], capture_output=True, timeout=5,
        )
        assert invalid.returncode != 0
        assert b"--color must be" in invalid.stderr
    print("PASS: image/JSON/JPEG cases, calibrated blue PnP, unverified/absent calibration, tentative tracking, invalid color")


if __name__ == "__main__":
    run(sys.argv[1])
