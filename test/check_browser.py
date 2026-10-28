#!/usr/bin/env python3
"""Check detector-to-browser contracts and stale-frame behavior with an offline frame."""

import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

from check_detector_cli import make_scene, write_test_calibration


def run(binary, browser):
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with tempfile.TemporaryDirectory(prefix="armor-browser-test-") as directory:
        root = Path(directory)
        image = root / "blue.ppm"
        status = root / "blue.json"
        make_scene(image, [(200, (0, 160, 255)), (440, (0, 160, 255))])
        config = root / "calibration.yml"
        write_test_calibration(config)
        env = dict(os.environ, ARMOR_STATUS_FILE=str(status))
        frame = subprocess.run(
            [binary, "--image", str(image), "--color", "blue", "--calibration", str(config)],
            env=env, capture_output=True, check=True, timeout=10,
        ).stdout
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 0))
            port = probe.getsockname()[1]
        env.update(BROWSER_PORT=str(port), BROWSER_BIND="127.0.0.1")
        captures = root / "captures"
        env["ARMOR_CAPTURE_DIR"] = str(captures)
        process = subprocess.Popen(
            [sys.executable, browser], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env,
        )
        base = f"http://127.0.0.1:{port}"
        try:
            assert process.stdin is not None
            process.stdin.write(frame)
            process.stdin.flush()
            deadline = time.monotonic() + 5
            while True:
                try:
                    with opener.open(base + "/status", timeout=1) as response:
                        value = json.load(response)
                    if value["stream_alive"]:
                        break
                except (OSError, urllib.error.URLError):
                    pass
                if time.monotonic() > deadline:
                    raise RuntimeError("browser did not become ready")
                time.sleep(0.05)
            assert value["red_armors"] == 0 and value["blue_armors"] == 1, value
            assert value["mode"] == "blue" and value["colors"] == ["blue"], value
            assert value["detected"] and len(value["corners"]) == 4, value
            assert value["geometry"]["reason"] == "ready", value
            assert len(value["geometry"]["poses"]) == 1, value
            assert len(value["geometry"]["tracks"]) == 1, value
            assert value["image_tracking"]["units"] == "pixels", value
            assert len(value["image_tracking"]["tracks"]) == 1, value
            with opener.open(base, timeout=2) as response:
                assert "RDK 蓝色装甲" in response.read().decode()
            request = urllib.request.Request(base + "/capture", method="POST")
            with opener.open(request, timeout=2) as response:
                assert json.load(response)["pending"]
            assert (captures / "capture.request").exists()
            # The real detector must save the original image before resize/drawing.
            subprocess.run(
                [binary, "--image", str(image), "--width", "480"],
                env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, check=True, timeout=10,
            )
            with opener.open(base + "/capture", timeout=2) as response:
                saved = json.load(response)
            assert saved["saved"] and not saved["pending"] and saved["count"] == 1, saved
            assert (saved["width"], saved["height"]) == (960, 480), saved
            import struct
            png = (captures / saved["filename"]).read_bytes()
            assert png.startswith(b"\x89PNG\r\n\x1a\n")
            assert struct.unpack(">II", png[16:24]) == (960, 480)
            with opener.open(base + "/stream", timeout=2) as response:
                assert b"\xff\xd8" in response.read(256), "missing MJPEG frame"
            time.sleep(2.1)
            with opener.open(base + "/status", timeout=2) as response:
                stale = json.load(response)
            assert not stale["stream_alive"] and not stale["detected"], stale
            assert stale["armors"] == 0 and stale["corners"] == [], stale
            assert stale["geometry"]["reason"] == "stream_stale", stale
            assert stale["geometry"]["poses"] == [] and stale["geometry"]["tracks"] == [], stale
            assert stale["image_tracking"]["tracks"] == [], stale
            try:
                opener.open(request, timeout=2)
                raise AssertionError("capture must reject a stale stream")
            except urllib.error.HTTPError as error:
                assert error.code == 503
                assert "相机画面已断开" in json.load(error)["error"]
            process.stdin.close()
            assert process.wait(timeout=3) == 0
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=3)
    print("PASS: blue counts/corners, PnP/tracking JSON, HTML, MJPEG, stale geometry clearing, clean EOF shutdown")


if __name__ == "__main__":
    run(sys.argv[1], sys.argv[2])
