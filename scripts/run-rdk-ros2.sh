#!/usr/bin/env bash
# Four composed C++ ROS nodes plus the ROS subscriber/browser bridge.
set -eo pipefail
repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
if [[ ! -f "$repo_root/install/share/rdk_s100_blue_armor_detector/local_setup.bash" ]]; then
  echo 'ROS install missing; build and install the package first.' >&2
  exit 2
fi
source "$repo_root/install/share/rdk_s100_blue_armor_detector/local_setup.bash"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export BROWSER_PORT="${BROWSER_PORT:-8876}"
export ARMOR_CAPTURE_DIR="${ARMOR_CAPTURE_DIR:-$repo_root/calibration/captures}"
export LD_LIBRARY_PATH="${MVS_ROOT:-/opt/MVS}/lib/aarch64:${MVS_ROOT:-/opt/MVS}/lib/64:${LD_LIBRARY_PATH:-}"
LC_NUMERIC=C printf -v exposure_ros '%.6f' "${EXPOSURE_US:-3500}"
LC_NUMERIC=C printf -v gain_ros '%.6f' "${CAMERA_GAIN:-2}"
LC_NUMERIC=C printf -v fps_ros '%.6f' "${CAMERA_FPS:-30}"
LC_NUMERIC=C printf -v prediction_ros '%.6f' "${PREDICTION_MS:-100}"
arguments=(--ros-args --params-file "$repo_root/config/blue_vision_ros.yaml"
  -p "exposure_us:=$exposure_ros"
  -p "gain:=$gain_ros"
  -p "fps:=$fps_ros"
  -p "processing_width:=${PROCESSING_WIDTH:-960}"
  -p "jpeg_quality:=${JPEG_QUALITY:-75}"
  -p "prediction_ms:=$prediction_ros"
  -p "color:=${ARMOR_COLOR:-blue}"
  -p "capture_directory:=$ARMOR_CAPTURE_DIR")
if [[ -n "${POSE_CALIBRATION:-}" ]]; then arguments+=(-p "calibration_path:=$POSE_CALIBRATION"); fi
if [[ -n "${ROS_TEST_IMAGE:-}" ]]; then arguments+=(-p "test_image:=$ROS_TEST_IMAGE"); fi
processes=()
cleanup() {
  trap - EXIT INT TERM
  for child in "${processes[@]}"; do kill -TERM "$child" 2>/dev/null || true; done
  for child in "${processes[@]}"; do wait "$child" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
cd "$repo_root"
"$repo_root/install/lib/rdk_s100_blue_armor_detector/blue_vision_ros" "${arguments[@]}" &
processes+=("$!")
/usr/bin/python3 "$repo_root/scripts/rdk_ros_browser.py" &
processes+=("$!")
# Stop the remaining process if either acquisition or the browser bridge exits.
set +e
wait -n "${processes[@]}"
result=$?
set -e
exit "$result"
