#!/usr/bin/env bash
# Capture/detect/localize/predict on the RDK; display only in the PC browser.
set -euo pipefail
script_directory=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export RDK_DIR="${RDK_DIR:-/home/sunrise/rdk_blue_vision}"
export ARMOR_COLOR=blue
export PREDICTION_MS="${PREDICTION_MS:-100}"
export VISION_BACKEND="${VISION_BACKEND:-ros2}"
exec bash "$script_directory/run-rdk-browser.sh"
