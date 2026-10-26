#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
rdk_host=${RDK_HOST:-sunrise@192.168.1.50}
rdk_dir=${RDK_DIR:-/home/sunrise/rdk-s100-blue-armor-detector}
rdk_dir_q=$(printf '%q' "$rdk_dir")

echo "Uploading sources to ${rdk_host}:${rdk_dir}" >&2
tar -C "$repo_root" --exclude=.git --exclude=build --exclude=install --exclude=log -czf - . \
  | ssh "$rdk_host" "mkdir -p $rdk_dir_q && tar -xzf - -C $rdk_dir_q"

ssh -tt "$rdk_host" "cd $rdk_dir_q && source /opt/ros/humble/setup.bash && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build --parallel 4"

echo "Deployment complete. Start it with scripts/run-rdk-browser.sh on the PC." >&2
