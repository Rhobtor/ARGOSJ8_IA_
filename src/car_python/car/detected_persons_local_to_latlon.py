#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import Imu, NavSatFix


EARTH_RADIUS_M = 6378137.0


def quat_to_euler(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def local_xy_to_latlon(
    x_fwd_m: float,
    y_left_m: float,
    origin_lat_deg: float,
    origin_lon_deg: float,
    heading_deg: float,
) -> Tuple[float, float]:
    yaw_enu_deg = (90.0 - float(heading_deg)) % 360.0
    yaw = math.radians(yaw_enu_deg)

    x_east_m = (math.cos(yaw) * float(x_fwd_m)) - (math.sin(yaw) * float(y_left_m))
    y_north_m = (math.sin(yaw) * float(x_fwd_m)) + (math.cos(yaw) * float(y_left_m))

    lat0 = math.radians(float(origin_lat_deg))
    lon0 = math.radians(float(origin_lon_deg))

    lat = lat0 + (y_north_m / EARTH_RADIUS_M)
    lon = lon0 + (x_east_m / (EARTH_RADIUS_M * max(math.cos(lat0), 1e-9)))
    return (math.degrees(lat), math.degrees(lon))


class DetectedPersonsLocalToLatLon(Node):
    def __init__(self) -> None:
        super().__init__("detected_persons_local_to_latlon")

        self.declare_parameter("input_topic", "detected_persons_local")
        self.declare_parameter("output_topic", "detected_persons_latlon")
        self.declare_parameter("global_output_topic", "/detected_persons_latlon_global")
        self.declare_parameter("fix_topic", "/fixposition/navsatfix")
        self.declare_parameter("imu_topic", "/imu_head/data")
        self.declare_parameter("publish_if_heading_missing", False)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        global_output_topic = str(self.get_parameter("global_output_topic").value)
        fix_topic = str(self.get_parameter("fix_topic").value)
        imu_topic = str(self.get_parameter("imu_topic").value)

        self._latest_fix: Optional[Tuple[float, float, float]] = None
        self._latest_heading_deg: Optional[float] = None
        self._source_namespace = str(self.get_namespace() or '').strip('/') or 'default'

        self._pub = self.create_publisher(PoseArray, output_topic, 10)
        self._pub_global = None
        if global_output_topic:
            self._pub_global = self.create_publisher(PoseArray, global_output_topic, 10)
        self.create_subscription(PoseArray, input_topic, self._on_local_persons, 10)
        self.create_subscription(NavSatFix, fix_topic, self._on_fix, 10)
        self.create_subscription(Imu, imu_topic, self._on_imu, 10)

        self.get_logger().info(
            f"Converting local detected persons {input_topic} -> {output_topic} using {fix_topic} + {imu_topic}"
        )
        if self._pub_global is not None:
            self.get_logger().info(
                f"Mirroring detected persons globally on {global_output_topic} with source namespace {self._source_namespace}"
            )

    def _on_fix(self, msg: NavSatFix) -> None:
        self._latest_fix = (float(msg.latitude), float(msg.longitude), float(msg.altitude))

    def _on_imu(self, msg: Imu) -> None:
        _, _, yaw = quat_to_euler(
            float(msg.orientation.x),
            float(msg.orientation.y),
            float(msg.orientation.z),
            float(msg.orientation.w),
        )
        yaw_deg_enu = math.degrees(yaw) % 360.0
        self._latest_heading_deg = (90.0 - yaw_deg_enu) % 360.0

    def _on_local_persons(self, msg: PoseArray) -> None:
        if self._latest_fix is None:
            self.get_logger().warn("Waiting for GNSS fix before publishing detected persons in lat/lon", throttle_duration_sec=2.0)
            return

        heading_deg = self._latest_heading_deg
        if heading_deg is None and not bool(self.get_parameter("publish_if_heading_missing").value):
            self.get_logger().warn("Waiting for heading before publishing detected persons in lat/lon", throttle_duration_sec=2.0)
            return
        if heading_deg is None:
            heading_deg = 0.0

        origin_lat, origin_lon, _ = self._latest_fix
        out = PoseArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "wgs84"

        for pose in msg.poses:
            lat, lon = local_xy_to_latlon(
                x_fwd_m=float(pose.position.x),
                y_left_m=float(pose.position.y),
                origin_lat_deg=origin_lat,
                origin_lon_deg=origin_lon,
                heading_deg=heading_deg,
            )
            point = Pose()
            point.position.x = lat
            point.position.y = lon
            point.position.z = float(pose.position.z)
            point.orientation = pose.orientation
            out.poses.append(point)

        self._pub.publish(out)
        if self._pub_global is not None:
            global_out = PoseArray()
            global_out.header.stamp = out.header.stamp
            global_out.header.frame_id = f"wgs84/{self._source_namespace}"
            global_out.poses = list(out.poses)
            self._pub_global.publish(global_out)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DetectedPersonsLocalToLatLon()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()