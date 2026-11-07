#!/usr/bin/env bash
# The shared RDK ROS 2 pipeline, selecting red targets.
set -euo pipefail
script_directory=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export ARMOR_COLOR=red
exec bash "$script_directory/run-blue-vision-browser.sh"
