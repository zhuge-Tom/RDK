#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
rdk_host=${RDK_HOST:-sunrise@192.168.1.50}
rdk_dir=${RDK_DIR:-/home/sunrise/rdk-s100-blue-armor-detector}
rdk_dir_q=$(printf '%q' "$rdk_dir")

echo "Uploading sources to ${rdk_host}:${rdk_dir}" >&2
tar -C "$repo_root" --exclude=.git --exclude=build --exclude=install --exclude=log --exclude=calibration --exclude=__pycache__ \
  --exclude='config/pose_camera.yml' --exclude='config/pose_camera.measured*.yml' \
  --exclude='config/pose_camera.measured*.report.json' -czf - . \
  | ssh "$rdk_host" "mkdir -p $rdk_dir_q && tar -xmzf - -C $rdk_dir_q"

ssh -tt "$rdk_host" "cd $rdk_dir_q && source /opt/ros/humble/setup.bash && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DCMAKE_INSTALL_PREFIX=\"\$PWD/install\" && cmake --build build --parallel 4 && cmake --install build"

echo "Deployment complete. Use the matching PC browser launcher for this deployment directory." >&2
