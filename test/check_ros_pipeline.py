#!/usr/bin/env python3
"""Exercise real ROS nodes, typed messages, browser and capture service offline."""
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

from check_detector_cli import make_scene, write_test_calibration


def run(binary, browser):
    # Isolate the test graph from the live camera and ordinary ROS deployments.
    os.environ.update(ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID="91")
    sys.path.insert(0, str(Path(binary).resolve().parent / "rosidl_generator_py"))
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rdk_s100_blue_armor_detector.msg import VisionResult
    from sensor_msgs.msg import CameraInfo
    rclpy.init()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with tempfile.TemporaryDirectory(prefix="armor-ros-test-") as directory:
            root = Path(directory)
            image = root / "blue.ppm"
            make_scene(image, [(200, (0, 160, 255)), (440, (0, 160, 255))])
            config = root / "synthetic-calibration.yml"
            write_test_calibration(config)
            for calibrated in (False, True):
                phase = root / ("calibrated" if calibrated else "uncalibrated")
                phase.mkdir()
                captures = phase / "captures"
                with socket.socket() as probe:
                    probe.bind(("127.0.0.1", 0))
                    port = probe.getsockname()[1]
                env = dict(os.environ, BROWSER_PORT=str(port), BROWSER_BIND="127.0.0.1", ARMOR_CAPTURE_DIR=str(captures))
                command = [binary, "--ros-args", "-p", f"test_image:={image}", "-p", "processing_width:=480",
                           "-p", "fps:=20.0", "-p", f"capture_directory:={captures}"]
                if calibrated:
                    command += ["-p", f"calibration_path:={config}"]
                received, infos = [], []
                node = Node("blue_vision_regression")
                subscriptions = [
                    node.create_subscription(VisionResult, "/armor/tracks", received.append, qos_profile_sensor_data),
                    node.create_subscription(CameraInfo, "/camera/camera_info", infos.append, qos_profile_sensor_data),
                ]
                with (phase / "vision.log").open("w+") as vision_log, (phase / "browser.log").open("w+") as browser_log:
                    vision = subprocess.Popen(command, env=env, stdout=vision_log, stderr=subprocess.STDOUT)
                    bridge = subprocess.Popen([sys.executable, browser], env=env, stdout=browser_log, stderr=subprocess.STDOUT)
                    base = f"http://127.0.0.1:{port}"
                    try:
                        deadline = time.monotonic() + 15
                        while time.monotonic() < deadline:
                            assert vision.poll() is None, "ROS C++ pipeline exited"
                            assert bridge.poll() is None, "ROS browser bridge exited"
                            rclpy.spin_once(node, timeout_sec=.1)
                            try:
                                with opener.open(base + "/status", timeout=.3) as response:
                                    status = json.load(response)
                                tracks = status.get("image_tracking", {}).get("tracks", [])
                                if (status.get("stream_alive") and tracks and tracks[0]["confirmed"]
                                        and received and received[-1].image_tracks[0].prediction_valid and infos):
                                    break
                            except (OSError, urllib.error.URLError):
                                pass
                        else:
                            raise AssertionError("ROS topics/browser did not become ready")
                        assert status["runtime"] == "ros2" and status["blue_armors"] == 1, status
                        assert status["red_armors"] == 0 and len(status["corners"]) == 4, status
                        assert status["width"] == 480 and status["height"] == 240, status
                        assert received[-1].header.frame_id == "camera_optical"
                        assert received[-1].header.stamp.sec > 0
                        assert len(received[-1].armors[0].corners) == 4
                        assert received[-1].image_tracks[0].observed
                        assert received[-1].image_tracks[0].confirmed
                        assert infos[-1].width == 480 and infos[-1].height == 240
                        if calibrated:
                            assert status["geometry"]["reason"] == "ready", status
                            assert status["geometry"]["poses"][0]["distance_m"] > 0
                            assert received[-1].poses and received[-1].tracks
                            assert abs(infos[-1].k[0] - 500) < 1e-6
                        else:
                            assert status["geometry"]["reason"] == "calibration_missing", status
                            assert not status["geometry"]["poses"] and not received[-1].poses
                            assert infos[-1].k[0] == 0
                        with opener.open(base + "/stream", timeout=2) as response:
                            assert b"\xff\xd8" in response.read(256)
                        with opener.open(urllib.request.Request(base + "/capture", method="POST"), timeout=5) as response:
                            assert json.load(response)["pending"]
                        with opener.open(base + "/capture", timeout=2) as response:
                            saved = json.load(response)
                        assert saved["saved"] and saved["count"] == 1, saved
                        assert saved["width"] == 960 and saved["height"] == 480, saved
                        png = (captures / saved["filename"]).read_bytes()
                        assert struct.unpack(">II", png[16:24]) == (960, 480)
                        # Repeated saves must complete while the camera keeps streaming.
                        for expected_count in (2, 3):
                            with opener.open(urllib.request.Request(base + "/capture", method="POST"), timeout=5) as response:
                                assert json.load(response)["pending"]
                            with opener.open(base + "/capture", timeout=2) as response:
                                repeated = json.load(response)
                            assert repeated["saved"] and repeated["count"] == expected_count, repeated
                            assert repeated["filename"] != saved["filename"], repeated
                            with opener.open(base + "/status", timeout=2) as response:
                                assert json.load(response)["stream_alive"]
                            saved = repeated
                        # A real filesystem failure must reach the HTTP caller as such.
                        preserved = phase / "preserved-captures"
                        captures.rename(preserved)
                        captures.write_text("not a writable capture directory")
                        try:
                            try:
                                opener.open(urllib.request.Request(base + "/capture", method="POST"), timeout=5)
                                raise AssertionError("capture must report its save failure")
                            except urllib.error.HTTPError as error:
                                assert error.code == 503
                                assert "相机未能保存照片" in json.load(error)["error"]
                        finally:
                            captures.unlink()
                            preserved.rename(captures)
                        vision.terminate()
                        assert vision.wait(timeout=5) == 0
                        deadline = time.monotonic() + 4
                        while time.monotonic() < deadline:
                            with opener.open(base + "/status", timeout=1) as response:
                                stale = json.load(response)
                            if not stale["stream_alive"]:
                                break
                            time.sleep(.15)
                        assert not stale["stream_alive"] and not stale["detected"], stale
                        assert not stale["geometry"]["tracks"] and not stale["image_tracking"]["tracks"], stale
                        bridge.terminate()
                        assert bridge.wait(timeout=5) == 0
                    except BaseException:
                        for log in (vision_log, browser_log):
                            log.seek(0)
                            print(log.read(), file=sys.stderr)
                        raise
                    finally:
                        for process in (vision, bridge):
                            if process.poll() is None:
                                process.terminate()
                                try:
                                    process.wait(timeout=5)
                                except subprocess.TimeoutExpired:
                                    process.kill()
                                    process.wait(timeout=2)
                        for subscription in subscriptions:
                            node.destroy_subscription(subscription)
                        node.destroy_node()
                print("PASS ROS phase:", "calibrated synthetic" if calibrated else "uncalibrated", flush=True)
    finally:
        rclpy.shutdown()
    print("PASS: composed nodes, typed corners/tracks, CameraInfo, metric gating, browser, ROS capture service, stale clearing and shutdown")


if __name__ == "__main__":
    run(sys.argv[1], sys.argv[2])
