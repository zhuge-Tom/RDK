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

from check_detector_cli import make_scene


def run(binary, browser):
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with tempfile.TemporaryDirectory(prefix="armor-browser-test-") as directory:
        root = Path(directory)
        image = root / "red.ppm"
        status = root / "red.json"
        make_scene(image, [(80, (255, 0, 0)), (260, (255, 0, 0))])
        env = dict(os.environ, ARMOR_STATUS_FILE=str(status))
        frame = subprocess.run(
            [binary, "--image", str(image), "--color", "red"],
            env=env, capture_output=True, check=True, timeout=10,
        ).stdout
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 0))
            port = probe.getsockname()[1]
        env.update(BROWSER_PORT=str(port), BROWSER_BIND="127.0.0.1")
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
            assert value["red_armors"] == 1 and value["blue_armors"] == 0, value
            assert value["mode"] == "red" and value["colors"] == ["red"], value
            assert value["detected"] and len(value["corners"]) == 4, value
            with opener.open(base, timeout=2) as response:
                assert "红蓝装甲检测" in response.read().decode()
            with opener.open(base + "/stream", timeout=2) as response:
                assert b"\xff\xd8" in response.read(256), "missing MJPEG frame"
            time.sleep(2.1)
            with opener.open(base + "/status", timeout=2) as response:
                stale = json.load(response)
            assert not stale["stream_alive"] and not stale["detected"], stale
            assert stale["armors"] == 0 and stale["corners"] == [], stale
            process.stdin.close()
            assert process.wait(timeout=3) == 0
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=3)
    print("PASS: red counts/corners, HTML, MJPEG, stale-frame clearing, clean EOF shutdown")


if __name__ == "__main__":
    run(sys.argv[1], sys.argv[2])
