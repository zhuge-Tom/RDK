#!/usr/bin/env python3
"""Stream raw Hikrobot frames as concatenated JPEGs.

This file is piped to ``python3 -`` over SSH, so it is never copied to the RDK.
It relies on the RDK's existing /home/RM_Vision/hik_camera_color.py module.
"""

from __future__ import annotations

import argparse
import ctypes
from pathlib import Path
import signal
import sys
import time

import cv2
import numpy as np


for candidate in (
    Path("/opt/MVS/Samples/64/Python"),
    Path("/opt/MVS/Samples/aarch64/Python"),
    Path("/home/RM_Vision"),
):
    if (candidate / "MvImport").is_dir():
        sys.path.insert(0, str(candidate))
        break

from MvImport.MvCameraControl_class import (  # noqa: E402
    MV_ACCESS_Exclusive,
    MV_CC_DEVICE_INFO,
    MV_CC_DEVICE_INFO_LIST,
    MV_FRAME_OUT,
    MV_USB_DEVICE,
    MvCamera,
)


class HikCamera:
    def __init__(self, exposure_time: float, gain: float) -> None:
        self.cam = MvCamera()
        self.exposure_time = exposure_time
        self.gain = gain

    def open(self) -> None:
        devices = MV_CC_DEVICE_INFO_LIST()
        result = MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE, devices)
        if result != 0 or devices.nDeviceNum == 0:
            raise RuntimeError(
                f"no Hikrobot USB camera, result={result}, count={devices.nDeviceNum}"
            )
        device = ctypes.cast(
            devices.pDeviceInfo[0], ctypes.POINTER(MV_CC_DEVICE_INFO)
        ).contents
        result = self.cam.MV_CC_CreateHandle(device)
        if result != 0:
            raise RuntimeError(f"create camera handle failed: {result}")
        result = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if result != 0:
            raise RuntimeError(f"open camera failed: {result}")
        self.cam.MV_CC_SetEnumValueByString("ExposureAuto", "Off")
        self.cam.MV_CC_SetFloatValue("ExposureTime", self.exposure_time)
        self.cam.MV_CC_SetEnumValueByString("GainAuto", "Off")
        self.cam.MV_CC_SetFloatValue("Gain", self.gain)
        self.cam.MV_CC_SetEnumValueByString("TriggerMode", "Off")
        result = self.cam.MV_CC_StartGrabbing()
        if result != 0:
            raise RuntimeError(f"start grabbing failed: {result}")

    def read(self, timeout_ms: int) -> np.ndarray | None:
        frame = MV_FRAME_OUT()
        if self.cam.MV_CC_GetImageBuffer(frame, timeout_ms) != 0:
            return None
        try:
            width = frame.stFrameInfo.nWidth
            height = frame.stFrameInfo.nHeight
            length = frame.stFrameInfo.nFrameLen
            raw = np.frombuffer(
                ctypes.string_at(frame.pBufAddr, length), dtype=np.uint8
            ).reshape(height, width)
            return cv2.cvtColor(raw, cv2.COLOR_BayerBG2BGR)
        finally:
            self.cam.MV_CC_FreeImageBuffer(frame)

    def close(self) -> None:
        self.cam.MV_CC_StopGrabbing()
        self.cam.MV_CC_CloseDevice()
        self.cam.MV_CC_DestroyHandle()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--exposure", type=float, default=3500.0)
    parser.add_argument("--gain", type=float, default=2.0)
    parser.add_argument("--max-fps", type=float, default=30.0)
    parser.add_argument("--jpeg-quality", type=int, default=78)
    args = parser.parse_args()
    if args.max_fps <= 0:
        parser.error("--max-fps must be positive")
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("--jpeg-quality must be between 1 and 100")

    stopping = False

    def stop(_signum: int, _frame: object) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    camera = HikCamera(exposure_time=args.exposure, gain=args.gain)
    camera.open()
    print(
        f"Hikrobot raw stream: exposure={args.exposure:.0f}us "
        f"gain={args.gain:.1f} max_fps={args.max_fps:.1f}",
        file=sys.stderr,
        flush=True,
    )
    next_output = time.monotonic()
    try:
        while not stopping:
            frame = camera.read(timeout_ms=1000)
            if frame is None:
                continue
            now = time.monotonic()
            if now < next_output:
                continue
            next_output = now + 1.0 / args.max_fps
            ok, encoded = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]
            )
            if not ok:
                continue
            try:
                sys.stdout.buffer.write(encoded.tobytes())
                sys.stdout.buffer.flush()
            except BrokenPipeError:
                break
    finally:
        camera.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
