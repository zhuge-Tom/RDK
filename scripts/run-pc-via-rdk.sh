#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
rdk_host=${RDK_HOST:-sunrise@192.168.1.50}
rdk_python_path=${RDK_MVS_PYTHON_PATH:-/home/RM_Vision}
params=${DETECTOR_PARAMS:-"$repo_root/config/blue_armor_detector.yaml"}

if ! command -v ros2 >/dev/null; then
  echo "ros2 is unavailable. Source your ROS 2 and workspace setup files first." >&2
  exit 2
fi

echo "RDK camera -> SSH MJPEG -> PC ROS 2 C++ detector" >&2
echo "Browser: http://127.0.0.1:8876/" >&2
if command -v xdg-open >/dev/null; then
  (
    for _attempt in $(seq 1 40); do
      if curl --noproxy '*' -fsS --max-time 1 \
        http://127.0.0.1:8876/status >/dev/null; then
        exec xdg-open http://127.0.0.1:8876/
      fi
      sleep 0.25
    done
  ) </dev/null >/tmp/blue-armor-browser-open.log 2>&1 &
fi

ssh -T -o ServerAliveInterval=5 -o ServerAliveCountMax=3 "$rdk_host" \
  "export MVCAM_COMMON_RUNENV=/opt/MVS/lib; export LD_LIBRARY_PATH=/opt/MVS/lib/aarch64:\${LD_LIBRARY_PATH:-}; export PYTHONPATH=$rdk_python_path:\${PYTHONPATH:-}; python3 - --exposure '${EXPOSURE_US:-3500}' --gain '${CAMERA_GAIN:-2}' --max-fps '${CAMERA_FPS:-30}' --jpeg-quality '${JPEG_QUALITY:-78}'" \
  < "$repo_root/scripts/mvs_raw_mjpeg.py" \
  | (
      ros2 run rdk_s100_blue_armor_detector blue_armor_detector_node \
        --ros-args --params-file "$params" &
      detector_pid=$!
      trap 'kill "$detector_pid" 2>/dev/null || true; wait "$detector_pid" 2>/dev/null || true' EXIT INT TERM
      python3 "$repo_root/scripts/mjpeg_ros_bridge.py" \
        --headless --web-bind 127.0.0.1 --web-port 8876 \
        --web-width 960 --web-jpeg-quality 75 --web-max-fps 20
    )
