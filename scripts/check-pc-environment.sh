#!/usr/bin/env bash
set -u

mvs_root=${MVS_ROOT:-/opt/MVS}
failed=0

check_command() {
  if command -v "$1" >/dev/null; then
    echo "OK   command: $1"
  else
    echo "MISS command: $1"
    failed=1
  fi
}

architecture=$(uname -m)
if [[ "$architecture" == "x86_64" ]]; then
  echo "OK   architecture: $architecture"
else
  echo "MISS expected x86_64 computer, found: $architecture"
  failed=1
fi

for command_name in g++ cmake pkg-config python3 curl lsusb; do
  check_command "$command_name"
done

if pkg-config --exists opencv4 2>/dev/null; then
  echo "OK   OpenCV: $(pkg-config --modversion opencv4)"
else
  echo "MISS OpenCV 4 development package"
  failed=1
fi

if [[ -f "$mvs_root/include/MvCameraControl.h" ]]; then
  echo "OK   MVS header: $mvs_root/include/MvCameraControl.h"
else
  echo "MISS MVS header: $mvs_root/include/MvCameraControl.h"
  failed=1
fi

if [[ -f "$mvs_root/lib/64/libMvCameraControl.so" ]]; then
  echo "OK   MVS library: $mvs_root/lib/64/libMvCameraControl.so"
else
  echo "MISS MVS library: $mvs_root/lib/64/libMvCameraControl.so"
  failed=1
fi

if command -v lsusb >/dev/null \
  && lsusb | grep -Eiq 'Hikrobot|2bdf:'; then
  echo "OK   Hikrobot USB camera detected"
else
  echo "MISS Hikrobot USB camera not detected"
  failed=1
fi

if ((failed)); then
  echo "Environment is not ready. Fix the MISS items and run this check again." >&2
  exit 2
fi

echo "PC direct-camera environment is ready."
