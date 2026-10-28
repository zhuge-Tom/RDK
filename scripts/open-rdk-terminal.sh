#!/usr/bin/env bash
# Run inside a visible PC terminal; wait for the direct Ethernet link, then SSH.
set -euo pipefail
interface=${RDK_INTERFACE:-enx00e04c0a4d30}
host=${RDK_HOST:-sunrise@192.168.1.50}
rdk_address=${RDK_ADDRESS:-192.168.1.50}

echo "RDK 终端：等待 USB 网卡 $interface 和网线连接。"
echo "接好 RDK 电源、USB 网卡与网线后将继续 SSH；Ctrl+C 可退出等待。"
until [[ -e "/sys/class/net/$interface/carrier" ]]; do sleep 2; done
nmcli connection up RDK-direct ifname "$interface"
until [[ "$(cat "/sys/class/net/$interface/carrier" 2>/dev/null || true)" == 1 ]]; do
  sleep 2
done
echo '已检测到物理链路，检查 RDK 网线连通性。'
until ping -I "$interface" -c 1 -W 2 "$rdk_address" >/dev/null 2>&1; do sleep 2; done
if ! ip route get "$rdk_address" | grep -Fq "dev $interface"; then
  echo 'RDK 路由未走网线，请先修复 VPN/路由配置。' >&2
  exit 2
fi
echo "正在 SSH 连接 $host。"
exec ssh -t -o ConnectTimeout=8 -o ServerAliveInterval=10 \
  "$host" 'stty rows 30 cols 100; exec bash -i'
