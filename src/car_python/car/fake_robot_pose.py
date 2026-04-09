#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus


class FakeRobotPose(Node):
    def __init__(self) -> None:
        super().__init__("fake_robot_pose")

        self.declare_parameter("fix_topic", "/fixposition/navsatfix")
        self.declare_parameter("imu_topic", "/imu_head/data")
        self.declare_parameter("latitude", 36.721302)
        self.declare_parameter("longitude", -4.421636)
        self.declare_parameter("altitude", 12.0)
        self.declare_parameter("heading_deg", 0.0)
        self.declare_parameter("rate_hz", 5.0)

        self._fix_topic = str(self.get_parameter("fix_topic").value)
        self._imu_topic = str(self.get_parameter("imu_topic").value)
        self._pub_fix = self.create_publisher(NavSatFix, self._fix_topic, 10)
        self._pub_imu = self.create_publisher(Imu, self._imu_topic, 10)

        rate_hz = max(float(self.get_parameter("rate_hz").value), 0.1)
        self.create_timer(1.0 / rate_hz, self._publish)

        self.get_logger().info(f"Publishing fake robot GNSS/IMU on {self._fix_topic} and {self._imu_topic}")

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        lat = float(self.get_parameter("latitude").value)
        lon = float(self.get_parameter("longitude").value)
        alt = float(self.get_parameter("altitude").value)
        heading_deg = float(self.get_parameter("heading_deg").value)

        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = "gps"
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = alt
        self._pub_fix.publish(fix)

        yaw_enu_deg = (90.0 - heading_deg) % 360.0
        yaw = math.radians(yaw_enu_deg)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation.z = math.sin(yaw / 2.0)
        imu.orientation.w = math.cos(yaw / 2.0)
        self._pub_imu.publish(imu)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = FakeRobotPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()