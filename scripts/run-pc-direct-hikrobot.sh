#!/usr/bin/env bash
set -eo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mvs_root=${MVS_ROOT:-/opt/MVS}
binary=${DETECTOR_BINARY:-"$repo_root/build/blue_armor_mvs_detector"}
port=${BROWSER_PORT:-8876}
export BROWSER_PORT="$port"

if [[ ! -f "$mvs_root/include/MvCameraControl.h" ]]; then
  echo "Hikrobot MVS SDK not found under $mvs_root" >&2
  exit 2
fi
if [[ ! -x "$binary" ]]; then
  echo "Detector binary not found: $binary" >&2
  echo "Build blue_armor_mvs_detector first." >&2
  exit 2
fi

export LD_LIBRARY_PATH="$mvs_root/lib/64:$mvs_root/lib/aarch64:${LD_LIBRARY_PATH:-}"
echo "PC USB Hikrobot camera; browser: http://127.0.0.1:${port}/" >&2

if command -v xdg-open >/dev/null; then
  (
    for _attempt in $(seq 1 40); do
      if curl --noproxy '*' -fsS --max-time 1 \
        "http://127.0.0.1:${port}/status" >/dev/null; then
        exec xdg-open "http://127.0.0.1:${port}/"
      fi
      sleep 0.25
    done
  ) </dev/null >/tmp/blue-armor-browser-open.log 2>&1 &
fi

"$binary" \
  --exposure "${EXPOSURE_US:-3500}" \
  --gain "${CAMERA_GAIN:-2}" \
  --fps "${CAMERA_FPS:-30}" \
  --width "${PROCESSING_WIDTH:-960}" \
  --quality "${JPEG_QUALITY:-75}" \
  2> >(tee "${DETECTOR_LOG:-/tmp/blue-armor-detector.log}" >&2) \
  | python3 "$repo_root/scripts/mjpeg_browser.py"
