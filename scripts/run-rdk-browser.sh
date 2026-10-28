#!/usr/bin/env bash
set -euo pipefail

rdk_host=${RDK_HOST:-sunrise@192.168.1.50}
rdk_dir=${RDK_DIR:-/home/sunrise/rdk-s100-blue-armor-detector}
local_port=${BROWSER_PORT:-8876}
color=${ARMOR_COLOR:-blue}
case "$color" in
  blue|red|both) ;;
  *) echo 'ARMOR_COLOR must be blue, red or both' >&2; exit 2 ;;
esac
rdk_dir_q=$(printf '%q' "$rdk_dir")
case "${VISION_BACKEND:-direct}" in
  ros2) rdk_script=./scripts/run-rdk-ros2.sh ;;
  direct) rdk_script=./scripts/run-rdk-local.sh ;;
  *) echo 'VISION_BACKEND must be ros2 or direct' >&2; exit 2 ;;
esac
remote_environment=("ARMOR_COLOR=$color" "POSE_CALIBRATION=${POSE_CALIBRATION:-}"
  "PREDICTION_MS=${PREDICTION_MS:-100}" "ARMOR_CAPTURE_DIR=${ARMOR_CAPTURE_DIR:-$rdk_dir/calibration/captures}"
  "EXPOSURE_US=${EXPOSURE_US:-3500}" "CAMERA_GAIN=${CAMERA_GAIN:-2}"
  "CAMERA_FPS=${CAMERA_FPS:-30}" "PROCESSING_WIDTH=${PROCESSING_WIDTH:-960}"
  "JPEG_QUALITY=${JPEG_QUALITY:-75}" "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}"
  "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}" "DETECTOR_BINARY=./build/blue_armor_mvs_detector")
printf -v remote_env '%q ' "${remote_environment[@]}"

echo "Starting RDK detector; browser: http://127.0.0.1:${local_port}/" >&2
if command -v xdg-open >/dev/null; then
  (
    for _attempt in $(seq 1 40); do
      if curl --noproxy '*' -fsS --max-time 1 \
        "http://127.0.0.1:${local_port}/status" >/dev/null; then
        exec xdg-open "http://127.0.0.1:${local_port}/"
      fi
      sleep 0.25
    done
  ) </dev/null >/tmp/blue-armor-browser-open.log 2>&1 &
fi

exec ssh -tt \
  -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=10 \
  -o ServerAliveCountMax=3 \
  -L "${local_port}:127.0.0.1:8876" \
  "$rdk_host" \
  "cd $rdk_dir_q && exec env $remote_env bash $rdk_script"
