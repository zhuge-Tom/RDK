#!/usr/bin/env bash
set -eo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
binary=${DETECTOR_BINARY:-"$repo_root/build/blue_armor_mvs_detector"}
port=${BROWSER_PORT:-8876}
export BROWSER_PORT="$port"
export ARMOR_STATUS_FILE="${ARMOR_STATUS_FILE:-/tmp/rmcs-armor-${port}-$$.json}"
export ARMOR_CAPTURE_DIR="${ARMOR_CAPTURE_DIR:-$repo_root/calibration/captures}"
color=${ARMOR_COLOR:-blue}
pose_args=(--prediction-ms "${PREDICTION_MS:-100}")
if [[ -n "${POSE_CALIBRATION:-}" ]]; then
  pose_args+=(--calibration "$POSE_CALIBRATION")
fi
case "$color" in
  blue|red|both) ;;
  *) echo 'ARMOR_COLOR must be blue, red or both' >&2; exit 2 ;;
esac

if [[ ! -x "$binary" ]]; then
  echo "Detector binary not found: $binary" >&2
  echo "Build blue_armor_mvs_detector first." >&2
  exit 2
fi

export LD_LIBRARY_PATH="${MVS_ROOT:-/opt/MVS}/lib/aarch64:${MVS_ROOT:-/opt/MVS}/lib/64:${LD_LIBRARY_PATH:-}"
echo "Browser: http://127.0.0.1:${port}/" >&2

"$binary" \
  --color "$color" \
  "${pose_args[@]}" \
  --exposure "${EXPOSURE_US:-3500}" \
  --gain "${CAMERA_GAIN:-2}" \
  --fps "${CAMERA_FPS:-30}" \
  --width "${PROCESSING_WIDTH:-960}" \
  --quality "${JPEG_QUALITY:-75}" \
  2> >(tee "${DETECTOR_LOG:-/tmp/blue-armor-detector.log}" >&2) \
  | python3 "$repo_root/scripts/rdk_mjpeg_browser.py"
