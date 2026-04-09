#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray


class FakePersonLocalDetections(Node):
    def __init__(self) -> None:
        super().__init__("fake_person_local_detections")

        self.declare_parameter("output_topic", "/ARGJ801/detected_persons_local")
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("points_xy", [6.0, 1.5, 10.0, -2.0])

        output_topic = str(self.get_parameter("output_topic").value)
        rate_hz = max(float(self.get_parameter("rate_hz").value), 0.1)

        self._points_xy = [float(v) for v in self.get_parameter("points_xy").value]
        self._pub = self.create_publisher(PoseArray, output_topic, 10)
        self.create_timer(1.0 / rate_hz, self._publish)

        self.get_logger().info(f"Publishing fake local detected persons on {output_topic}")

    def _publish(self) -> None:
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        values = self._points_xy
        for index in range(0, len(values) - 1, 2):
            pose = Pose()
            pose.position.x = float(values[index])
            pose.position.y = float(values[index + 1])
            pose.position.z = 0.0
            pose.orientation.w = float((index // 2) + 1)
            msg.poses.append(pose)

        self._pub.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = FakePersonLocalDetections()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()