#!/usr/bin/env bash
set -euo pipefail

iface="${1:-}"
if [[ -z "$iface" ]]; then
  echo "Usage: $0 <network-interface>" >&2
  echo "Example: $0 enp2s0" >&2
  exit 2
fi

if ! command -v ip >/dev/null 2>&1; then
  echo "Error: 'ip' command not found" >&2
  exit 1
fi

# Ensure interface exists
if [[ ! -d "/sys/class/net/$iface" ]]; then
  echo "Error: interface '$iface' not found" >&2
  exit 1
fi

# Needs root to modify routes
if [[ "$(id -u)" -ne 0 ]]; then
  echo "Error: must run as root (try: sudo $0 $iface)" >&2
  exit 1
fi

echo "Adding multicast route 224.0.0.0/4 dev $iface"
ip route replace 224.0.0.0/4 dev "$iface" scope link

echo "Current multicast route:" 
ip route show 224.0.0.0/4 || true

echo "Tip: verify traffic with: tcpdump -ni $iface 'udp and (port 5004 or port 5006)'"