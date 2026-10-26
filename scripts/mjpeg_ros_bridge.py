#!/usr/bin/env python3
"""Publish an MJPEG stdin stream to ROS2 and display C++ detector output."""

from __future__ import annotations

import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import sys
import threading
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud


PAGE = """<!doctype html>
<html lang="zh-CN">
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>电脑 C++ 蓝色装甲检测</title>
<style>
  * { box-sizing: border-box; }
  body { margin: 0; background: #0c0f11; color: white; font: 16px system-ui, sans-serif; }
  header { height: 64px; display: flex; align-items: center; gap: 24px; padding: 0 20px;
           background: #20252a; border-bottom: 1px solid #4b535b; }
  strong { font-size: 20px; }
  #state { padding: 8px 18px; border-radius: 8px; font-weight: 700; }
  .found { background: #087f23; }
  .lost { background: #b42318; }
  #details { color: #d3d9de; }
  main { height: calc(100vh - 64px); display: grid; place-items: center; }
  img { display: block; max-width: 100%; max-height: 100%; object-fit: contain; }
</style>
<header><strong>电脑 C++ 蓝色装甲检测</strong><span id="state" class="lost">等待画面</span>
<span id="details"></span></header>
<main><img src="/stream" alt="实时检测画面"></main>
<script>
async function update() {
  try {
    const value = await (await fetch('/status', {cache: 'no-store'})).json();
    const state = document.getElementById('state');
    state.className = value.detected ? 'found' : 'lost';
    state.textContent = value.detected ? `已识别：${value.armors} 个蓝色装甲板` : '未识别到装甲板';
    document.getElementById('details').textContent =
      `${value.corners} 个角点 · ${value.fps.toFixed(1)} FPS`;
  } catch (_) {
    const state = document.getElementById('state');
    state.className = 'lost'; state.textContent = '连接中断';
  }
}
update(); setInterval(update, 250);
</script>
</html>"""


def make_handler(node: "MjpegRosBridge"):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _format: str, *_args: object) -> None:
            pass

        def send_bytes(self, payload: bytes, content_type: str) -> None:
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self) -> None:
            if self.path == "/":
                self.send_bytes(PAGE.encode("utf-8"), "text/html; charset=utf-8")
                return
            if self.path == "/status":
                with node.web_condition:
                    payload = json.dumps(node.web_status).encode("utf-8")
                self.send_bytes(payload, "application/json")
                return
            if self.path != "/stream":
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seen = 0
            try:
                while True:
                    with node.web_condition:
                        node.web_condition.wait_for(
                            lambda: node.web_sequence > seen, timeout=5.0
                        )
                        jpeg = node.web_jpeg
                        seen = node.web_sequence
                    if jpeg is None:
                        continue
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                        + str(len(jpeg)).encode("ascii")
                        + b"\r\n\r\n"
                        + jpeg
                        + b"\r\n"
                    )
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass

    return Handler


class MjpegRosBridge(Node):
    def __init__(
        self,
        *,
        headless: bool,
        save_directory: Path,
        web_bind: str,
        web_port: int,
        web_width: int,
        web_jpeg_quality: int,
        web_max_fps: float,
    ) -> None:
        super().__init__("rdk_mvs_mjpeg_bridge")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, "/image_raw", qos)
        self.create_subscription(
            Image,
            "/blue_armor_detector/visualization",
            self.on_visualization,
            qos,
        )
        self.create_subscription(
            PointCloud,
            "/blue_armor_detector/armors",
            self.on_armors,
            qos,
        )
        self.headless = headless
        self.save_directory = save_directory
        self.save_directory.mkdir(parents=True, exist_ok=True)
        self.window = "RDK Hikrobot | PC C++ blue armor detector | Q: quit"
        self.lock = threading.Lock()
        self.latest_jpeg: bytes | None = None
        self.sequence = 0
        self.published_sequence = 0
        self.end_of_stream = False
        self.error: str | None = None
        self.last_input = time.monotonic()
        self.last_save = 0.0
        self.armor_count = 0
        self.frames = 0
        self.started = time.monotonic()
        self.web_width = web_width
        self.web_jpeg_quality = web_jpeg_quality
        self.web_max_fps = web_max_fps
        self.visual_condition = threading.Condition()
        self.visual_frame: np.ndarray | None = None
        self.visual_sequence = 0
        self.web_condition = threading.Condition()
        self.web_jpeg: bytes | None = None
        self.web_sequence = 0
        self.web_status: dict[str, int | float | bool] = {
            "detected": False,
            "armors": 0,
            "corners": 0,
            "fps": 0.0,
        }
        self.web_server = ThreadingHTTPServer((web_bind, web_port), make_handler(self))
        self.web_server.daemon_threads = True
        threading.Thread(target=self.web_server.serve_forever, daemon=True).start()
        threading.Thread(target=self.encode_web_frames, daemon=True).start()
        self.get_logger().info(f"Browser preview: http://{web_bind}:{web_port}/")
        if not self.headless:
            cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window, 1280, 720)
        threading.Thread(target=self.read_stdin, daemon=True).start()
        self.create_timer(0.005, self.publish_latest)

    def read_stdin(self) -> None:
        buffer = bytearray()
        try:
            while rclpy.ok():
                chunk = os.read(0, 65536)
                if not chunk:
                    break
                buffer.extend(chunk)
                while True:
                    begin = buffer.find(b"\xff\xd8")
                    if begin < 0:
                        buffer[:] = buffer[-1:]
                        break
                    end = buffer.find(b"\xff\xd9", begin + 2)
                    if end < 0:
                        del buffer[:begin]
                        break
                    jpeg = bytes(buffer[begin : end + 2])
                    del buffer[: end + 2]
                    with self.lock:
                        self.latest_jpeg = jpeg
                        self.sequence += 1
                        self.last_input = time.monotonic()
                if len(buffer) > 32 * 1024 * 1024:
                    raise RuntimeError("invalid MJPEG stream: frame exceeds 32 MiB")
        except Exception as error:  # noqa: BLE001 - report pipe failures through ROS logging
            self.error = str(error)
        finally:
            self.end_of_stream = True

    def publish_latest(self) -> None:
        with self.lock:
            sequence = self.sequence
            jpeg = self.latest_jpeg
        if jpeg is not None and sequence != self.published_sequence:
            frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                self.published_sequence = sequence
                message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                message.header.stamp = self.get_clock().now().to_msg()
                message.header.frame_id = "rdk_hikrobot_camera"
                self.publisher.publish(message)
                if self.frames == 0:
                    cv2.imwrite(str(self.save_directory / "latest-raw.jpg"), frame)
        if not self.headless and cv2.waitKey(1) & 0xFF in (27, ord("q")):
            rclpy.shutdown()
        if self.end_of_stream and time.monotonic() - self.last_input > 1.0:
            if self.error:
                self.get_logger().error(self.error)
            rclpy.shutdown()

    def on_armors(self, message: PointCloud) -> None:
        self.armor_count = len(message.points) // 4

    def on_visualization(self, message: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        self.frames += 1
        detected = self.armor_count > 0
        banner_color = (20, 125, 25) if detected else (20, 20, 185)
        label = (
            f"DETECTED: {self.armor_count} BLUE ARMOR | {self.armor_count * 4} CORNERS"
            if detected
            else "LOST: NO BLUE ARMOR"
        )
        output_height = max(1, round(frame.shape[0] * self.web_width / frame.shape[1]))
        output = cv2.resize(frame, (self.web_width, output_height), interpolation=cv2.INTER_AREA)
        cv2.rectangle(output, (0, 0), (output.shape[1], 48), banner_color, -1)
        cv2.putText(
            output,
            f"PC C++ | {label}",
            (12, 33),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        with self.visual_condition:
            self.visual_frame = output
            self.visual_sequence += 1
            self.visual_condition.notify()
        if not self.headless:
            cv2.imshow(self.window, output)

    def encode_web_frames(self) -> None:
        seen = 0
        next_output = time.monotonic()
        output_times: deque[float] = deque(maxlen=30)
        while rclpy.ok():
            with self.visual_condition:
                self.visual_condition.wait_for(
                    lambda: self.visual_sequence > seen or not rclpy.ok(), timeout=1.0
                )
                if not rclpy.ok():
                    return
                frame = self.visual_frame
                seen = self.visual_sequence
            if frame is None:
                continue
            now = time.monotonic()
            if now < next_output:
                time.sleep(next_output - now)
            next_output = time.monotonic() + 1.0 / self.web_max_fps
            ok, encoded = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.web_jpeg_quality]
            )
            if not ok:
                continue
            now = time.monotonic()
            output_times.append(now)
            fps = 0.0
            if len(output_times) > 1:
                fps = (len(output_times) - 1) / (output_times[-1] - output_times[0])
            with self.web_condition:
                self.web_jpeg = encoded.tobytes()
                self.web_sequence += 1
                self.web_status = {
                    "detected": self.armor_count > 0,
                    "armors": self.armor_count,
                    "corners": self.armor_count * 4,
                    "fps": fps,
                }
                self.web_condition.notify_all()
            if now - self.last_save >= 1.0:
                cv2.imwrite(str(self.save_directory / "latest-visualization.jpg"), frame)
                self.last_save = now

    def destroy_node(self) -> bool:
        self.web_server.shutdown()
        self.web_server.server_close()
        cv2.destroyAllWindows()
        return super().destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--save-directory", type=Path, default=Path("/tmp/rmcs-blue-armor-live"))
    parser.add_argument("--web-bind", default="127.0.0.1")
    parser.add_argument("--web-port", type=int, default=8876)
    parser.add_argument("--web-width", type=int, default=960)
    parser.add_argument("--web-jpeg-quality", type=int, default=75)
    parser.add_argument("--web-max-fps", type=float, default=20.0)
    args = parser.parse_args()
    if args.web_width <= 0:
        parser.error("--web-width must be positive")
    if not 1 <= args.web_jpeg_quality <= 100:
        parser.error("--web-jpeg-quality must be between 1 and 100")
    if args.web_max_fps <= 0:
        parser.error("--web-max-fps must be positive")

    rclpy.init()
    node = MjpegRosBridge(
        headless=args.headless,
        save_directory=args.save_directory,
        web_bind=args.web_bind,
        web_port=args.web_port,
        web_width=args.web_width,
        web_jpeg_quality=args.web_jpeg_quality,
        web_max_fps=args.web_max_fps,
    )
    for handled_signal in (signal.SIGINT, signal.SIGTERM):
        signal.signal(handled_signal, lambda _signum, _frame: rclpy.shutdown())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
