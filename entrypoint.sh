#!/usr/bin/env bash
set -e

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///etc/cyclonedds/local_cyclonedds.xml}"

echo "[entrypoint] CYCLONEDDS_URI=${CYCLONEDDS_URI}"

if command -v dbus-daemon >/dev/null 2>&1; then
  mkdir -p /run/dbus
  if [ ! -S /run/dbus/system_bus_socket ]; then
    echo "[entrypoint] iniciando dbus system bus"
    dbus-daemon --system --fork
  fi
fi

for i in $(seq 1 30); do
  if ip link show ztpp6e24bc >/dev/null 2>&1 && \
     ip -4 addr show dev ztpp6e24bc | grep -q "inet "; then
    break
  fi
  sleep 1
done

if ! ip link show ztpp6e24bc >/dev/null 2>&1; then
  echo "[entrypoint] ERROR: no existe la interfaz ztpp6e24bc"
  exit 1
fi

if ! ip -4 addr show dev ztpp6e24bc | grep -q "inet "; then
  echo "[entrypoint] ERROR: ztpp6e24bc no tiene IPv4"
  exit 1
fi

ZT_IP=$(ip -4 -o addr show dev ztpp6e24bc | awk '{print $4}' | cut -d/ -f1)
export ZT_IP

echo "[entrypoint] ZeroTier IP: ${ZT_IP}"

if [ -f /etc/cyclonedds/local_cyclonedds.template.xml ]; then
  envsubst < /etc/cyclonedds/local_cyclonedds.template.xml > /etc/cyclonedds/local_cyclonedds.xml
  echo "[entrypoint] XML generado:"
  cat /etc/cyclonedds/local_cyclonedds.xml
else
  echo "[entrypoint] ERROR: no existe /etc/cyclonedds/local_cyclonedds.template.xml"
  exit 1
fi

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

exec "$@"