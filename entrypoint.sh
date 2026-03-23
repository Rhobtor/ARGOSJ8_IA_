#!/usr/bin/env bash
# Prepara el entorno y ejecuta el comando del contenedor
set -e
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///etc/cyclonedds/local_cyclonedds.xml}"
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
exec "$@"
