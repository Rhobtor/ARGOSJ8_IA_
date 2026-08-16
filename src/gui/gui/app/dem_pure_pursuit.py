from __future__ import annotations

import math
from collections.abc import Sequence


def compute_command(
    path: Sequence[tuple[float, float]],
    robot_pose: tuple[float, float, float],
    lookahead_distance: float,
    forward_speed: float,
    max_angular_speed: float,
    goal_tolerance: float = 0.8,
) -> tuple[float, float, bool]:
    if not path:
        return 0.0, 0.0, False

    robot_x, robot_y, robot_yaw = robot_pose
    goal_x, goal_y = path[-1]
    if math.hypot(goal_x - robot_x, goal_y - robot_y) <= goal_tolerance:
        return 0.0, 0.0, True

    closest_index = min(
        range(len(path)),
        key=lambda index: math.hypot(path[index][0] - robot_x, path[index][1] - robot_y),
    )
    target_x, target_y = path[-1]
    for point_x, point_y in path[closest_index:]:
        if math.hypot(point_x - robot_x, point_y - robot_y) >= lookahead_distance:
            target_x, target_y = point_x, point_y
            break

    delta_x = target_x - robot_x
    delta_y = target_y - robot_y
    local_x = math.cos(robot_yaw) * delta_x + math.sin(robot_yaw) * delta_y
    local_y = -math.sin(robot_yaw) * delta_x + math.cos(robot_yaw) * delta_y
    distance_squared = max(delta_x * delta_x + delta_y * delta_y, 1e-6)
    curvature = 2.0 * local_y / distance_squared

    linear = max(0.0, float(forward_speed))
    angular_limit = max(0.0, float(max_angular_speed))
    angular = max(-angular_limit, min(angular_limit, linear * curvature))

    if local_x < 0.0:
        linear *= 0.2
    elif angular_limit > 0.0:
        linear *= max(0.25, 1.0 - 0.65 * abs(angular) / angular_limit)

    return linear, angular, False