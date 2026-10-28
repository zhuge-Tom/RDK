#!/usr/bin/env python3
"""ROS 2 subscriber/HTTP bridge; no image detection or JPEG re-encoding on the PC."""

import json
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, HistoryPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger

import rdk_mjpeg_browser as browser


class RosBrowser(Node):
    def __init__(self):
        super().__init__("armor_browser_bridge")
        self.latest_status = dict(detected=False, armors=0, corners=[], detect_fps=0., runtime="ros2")
        self.lock = threading.Lock()
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=qos_profile_sensor_data.reliability)
        self.image_subscription = self.create_subscription(
            CompressedImage, "/armor/visualization/compressed", self.on_image, qos)
        self.status_subscription = self.create_subscription(String, "/armor/status", self.on_status, 1)
        self.capture_client = self.create_client(Trigger, "/camera/save_calibration_frame")

    def on_image(self, message):
        browser.state.publish_jpeg(bytes(message.data))

    def on_status(self, message):
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            return
        with self.lock:
            self.latest_status = value

    def get_status(self):
        with self.lock:
            return dict(self.latest_status)

    def capture(self):
        if not self.capture_client.wait_for_service(timeout_sec=.5):
            raise RuntimeError("尚未发现相机采集服务，请等待几秒后重试")
        future = self.capture_client.call_async(Trigger.Request())
        ready = threading.Event()
        future.add_done_callback(lambda _future: ready.set())
        if not future.done():
            ready.wait(timeout=3)
        if not future.done():
            self.capture_client.remove_pending_request(future)
            raise TimeoutError("相机保存响应超时，请检查保存数量后再重试")
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"Capture service failed: {error}")
            raise RuntimeError("相机采集服务调用失败，请检查 RDK 终端日志") from error
        if not response.success:
            raise RuntimeError("相机未能保存照片：" + response.message)
        return True


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = RosBrowser()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    browser.status_provider = node.get_status
    browser.capture_provider = node.capture

    def read_topics():
        while not browser.stopping.is_set() and rclpy.ok():
            executor.spin_once(timeout_sec=.1)

    try:
        return browser.main(reader_target=read_topics)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
