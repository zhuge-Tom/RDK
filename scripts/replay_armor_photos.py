#!/usr/bin/env python3
"""Replay catalogued raw photos through the C++ ROS pipeline without a camera."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    repo = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=repo / "docs/measurements/2026-11-static/photos/catalog.json")
    parser.add_argument("--output", type=Path, required=True, help="New output directory; existing directories are refused")
    parser.add_argument("--binary", type=Path, default=repo / "install/lib/rdk_s100_blue_armor_detector/blue_vision_ros")
    parser.add_argument("--params-file", type=Path, default=repo / "config/blue_vision_ros.yaml")
    parser.add_argument("--calibration", type=Path, default=repo / "config/pose_camera.rdk-hikrobot-debug.yml")
    parser.add_argument("--domain-id", type=int, default=96, help="Use an otherwise unused local ROS domain")
    args = parser.parse_args()
    for name in ("manifest", "binary", "params_file", "calibration", "output"):
        setattr(args, name, getattr(args, name).resolve())
    if not 0 <= args.domain_id <= 232:
        parser.error("domain-id must be 0..232")
    catalog = json.loads(args.manifest.read_text())
    for photo in catalog["photos"]:
        image = (args.manifest.parent / photo["image"]).resolve()
        if digest(image) != photo["sha256"]:
            raise RuntimeError(f"Photo hash mismatch: {image}")
    for path in (args.binary, args.params_file, args.calibration):
        if not path.is_file():
            raise FileNotFoundError(path)
    os.environ.update(ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID=str(args.domain_id))
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String

    args.output.mkdir(parents=True, exist_ok=False)
    parameters = args.output / "blue_vision_ros.yaml"
    calibration = args.output / "pose_camera.yml"
    parameters.write_bytes(args.params_file.read_bytes())
    calibration.write_bytes(args.calibration.read_bytes())
    results = []
    rclpy.init()
    try:
        for photo in catalog["photos"]:
            if photo["kind"] != "raw":
                continue
            image = (args.manifest.parent / photo["image"]).resolve()
            node = Node("armor_photo_replay")
            received = []
            subscription = node.create_subscription(
                String, "/armor/status", lambda msg: received.append(json.loads(msg.data)),
                qos_profile_sensor_data)
            command = [str(args.binary), "--ros-args", "--params-file", str(parameters),
                       "-p", f"test_image:={image}", "-p", "color:=blue",
                       "-p", f"calibration_path:={calibration}",
                       "-p", f"capture_directory:={args.output / 'unused-captures'}"]
            process = None
            try:
                with (args.output / (photo["id"] + ".log")).open("w") as log:
                    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
                    deadline = time.monotonic() + 20
                    while not received:
                        if process.poll() is not None:
                            raise RuntimeError(f"Replay exited for {photo['id']}; inspect its log")
                        if time.monotonic() >= deadline:
                            raise TimeoutError(f"No ROS status for {photo['id']}")
                        rclpy.spin_once(node, timeout_sec=.1)
                    status = received[0]
                    if status.get("mode") != "blue" or status.get("runtime") != "ros2":
                        raise RuntimeError("Unexpected publisher in replay domain; select a free domain")
                    record = {"photo_id": photo["id"], "image_sha256": photo["sha256"],
                              "result_type": "new_offline_cpp_ros_replay",
                              "historical_synchronized_measurement": False, "status": status}
                    (args.output / (photo["id"] + ".json")).write_text(
                        json.dumps(record, ensure_ascii=False, indent=2) + "\n")
                    results.append(record)
                    print(photo["id"], "armors=", status["blue_armors"], flush=True)
            finally:
                if process is not None and process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                node.destroy_subscription(subscription)
                node.destroy_node()
    finally:
        rclpy.shutdown()
    manifest = {"schema_version": 1, "result_type": "new_offline_cpp_ros_replay",
                "calibration_sha256": digest(calibration), "parameters_sha256": digest(parameters),
                "source_sha256": {str(p.relative_to(repo)): digest(p)
                                  for p in sorted((repo / "src").glob("*")) if p.is_file()},
                "note": "Raw R/Z/XYZ/corners are replay outputs; no historical frame synchronization is asserted.",
                "results": results}
    (args.output / "results.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")


if __name__ == "__main__":
    main()
