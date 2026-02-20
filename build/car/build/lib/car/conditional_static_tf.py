import math
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def _quat_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


class ConditionalStaticTF(Node):
    def __init__(self) -> None:
        super().__init__("conditional_static_tf")

        self.declare_parameter("parent_frame", "")
        self.declare_parameter("child_frame", "")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("roll", 0.0)
        self.declare_parameter("pitch", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("wait_for_existing_sec", 1.0)
        self.declare_parameter("check_period_sec", 0.2)
        self.declare_parameter("disable_if_child_has_parent", True)

        self._parent_frame = str(self.get_parameter("parent_frame").value).strip()
        self._child_frame = str(self.get_parameter("child_frame").value).strip()

        if not self._parent_frame:
            raise ValueError("parent_frame is empty")
        if not self._child_frame:
            raise ValueError("child_frame is empty")
        if self._parent_frame == self._child_frame:
            raise ValueError("parent_frame == child_frame")

        self._x = float(self.get_parameter("x").value)
        self._y = float(self.get_parameter("y").value)
        self._z = float(self.get_parameter("z").value)
        self._roll = float(self.get_parameter("roll").value)
        self._pitch = float(self.get_parameter("pitch").value)
        self._yaw = float(self.get_parameter("yaw").value)

        self._wait_for_existing = float(self.get_parameter("wait_for_existing_sec").value)
        self._check_period = float(self.get_parameter("check_period_sec").value)
        self._disable_if_child_has_parent = bool(self.get_parameter("disable_if_child_has_parent").value)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._broadcaster = StaticTransformBroadcaster(self)

        # Track whether this child already appears in the TF graph.
        # If it does, publishing a second parent->child edge can split the TF tree.
        self._child_parent_seen: Optional[str] = None
        self._sub_tf_static = self.create_subscription(TFMessage, "/tf_static", self._on_tf_msg, 10)
        self._sub_tf = self.create_subscription(TFMessage, "/tf", self._on_tf_msg, 50)

        self._start_time = self.get_clock().now()
        self._published = False

        self.get_logger().info(
            f"Will publish static TF {self._parent_frame} -> {self._child_frame} only if missing "
            f"(wait_for_existing_sec={self._wait_for_existing:.2f})"
        )

        self._timer = self.create_timer(self._check_period, self._tick)

    def _on_tf_msg(self, msg: TFMessage) -> None:
        # Capture any parent that already owns this child.
        for t in msg.transforms:
            try:
                child = str(t.child_frame_id).strip()
                parent = str(t.header.frame_id).strip()
            except Exception:
                continue

            if not child or not parent:
                continue

            if child != self._child_frame:
                continue

            # If the graph already contains child under some parent, remember it.
            # (Ignore the exact same edge we might later publish.)
            if parent != self._parent_frame:
                self._child_parent_seen = parent

    def _has_existing_tf(self) -> bool:
        try:
            return self._tf_buffer.can_transform(
                self._parent_frame,
                self._child_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.0),
            )
        except Exception:
            return False

    def _tick(self) -> None:
        if self._published:
            return

        if self._disable_if_child_has_parent and self._child_parent_seen is not None:
            self.get_logger().warn(
                f"Child frame '{self._child_frame}' already has a parent '{self._child_parent_seen}'; not publishing "
                f"{self._parent_frame}->{self._child_frame} to avoid multiple parents"
            )
            self._published = True
            self._timer.cancel()
            return

        if self._has_existing_tf():
            self.get_logger().info(
                f"TF already exists for {self._parent_frame} -> {self._child_frame}; not publishing"
            )
            self._published = True
            self._timer.cancel()
            return

        elapsed = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9
        if elapsed < self._wait_for_existing:
            return

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._parent_frame
        msg.child_frame_id = self._child_frame
        msg.transform.translation.x = self._x
        msg.transform.translation.y = self._y
        msg.transform.translation.z = self._z

        qx, qy, qz, qw = _quat_from_euler(self._roll, self._pitch, self._yaw)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw

        self._broadcaster.sendTransform(msg)
        self.get_logger().info(f"Published static TF {self._parent_frame} -> {self._child_frame}")
        self._published = True
        self._timer.cancel()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    try:
        node = ConditionalStaticTF()
    except Exception as exc:
        # Ensure a clean error for launch logs
        rclpy.shutdown()
        raise exc

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
