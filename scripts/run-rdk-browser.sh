#!/usr/bin/env bash
set -euo pipefail

rdk_host=${RDK_HOST:-sunrise@192.168.1.50}
rdk_dir=${RDK_DIR:-/home/sunrise/rdk-s100-blue-armor-detector}
local_port=${BROWSER_PORT:-8876}
rdk_dir_q=$(printf '%q' "$rdk_dir")

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
  "cd $rdk_dir_q && DETECTOR_BINARY=./build/blue_armor_mvs_detector exec ./scripts/run-rdk-local.sh"
