#!/usr/bin/env bash
# Deploy the local blue vision module into its own RDK directory.
set -euo pipefail
script_directory=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export RDK_DIR="${RDK_DIR:-/home/sunrise/rdk_blue_vision}"
bash "$script_directory/deploy-to-rdk.sh"
rdk_directory_q=$(printf '%q' "$RDK_DIR")
ssh -tt "${RDK_HOST:-sunrise@192.168.1.50}" \
  "cd $rdk_directory_q && source /opt/ros/humble/setup.bash && ctest --test-dir build --output-on-failure"
