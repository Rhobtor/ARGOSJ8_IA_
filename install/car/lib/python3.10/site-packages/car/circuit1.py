#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SequentialGoalSelector (circuito secuencial)
===========================================
Nodo ROS 2 que publica metas de navegación **en orden** (circuito) tomadas
de una lista fija de puntos (x, y, z) definida como parámetro ROS
(`goal_points`). Incluye gestión de colisiones, reinicio, progreso y señal
fin de circuito.

Características principales:
- Re-publica la meta activa a `republish_hz`.
- Comprueba colisión con `OccupancyGrid` y corrige con una BFS hasta
  `MAX_SEARCH_M`, manteniendo el **z** original y la orientación.
- Avanza al **siguiente** punto cuando llega `goal_reached: True`.
- Se **resetea al primer punto** cuando en `/env/episode_done` llega un
  mensaje cuyo `data` esté en `reset_values` (p. ej., "STUCK").
- Opcional: orienta el yaw mirando hacia el **siguiente** punto del circuito.
- Publica el número de **puntos visitados** en `/circuit/visited` (UInt32).
- Publica **fin de circuito** en `/circuit/done` (Bool) al alcanzar el último
  punto con `loop: false` y `signal_done_on_last: true`. Con `hold_when_done: true`
  se detiene tras señalizar el fin.

Parámetros YAML (ejemplo):
--------------------------------
points_goal:
  ros__parameters:
    goal_points: [ 12.0, 4.0, 0.0,
                    7.0,-4.0, 0.0,
                   -3.0, 3.0, 0.0 ]
    republish_hz: 2.0
    loop: false
    compute_yaw_from_path: true
    goal_frame_id: "map"
    reset_values: ["STUCK", "RESET"]
    signal_done_on_last: true
    hold_when_done: true

Publica:
  * `goal` (PoseArray) – un único Pose con la meta
  * `goal_marker` (Marker) – esfera roja para RViz
  * `/circuit/visited` (UInt32) – contador de puntos alcanzados
  * `/circuit/done` (Bool) – señal de fin de circuito

Suscribe:
  * `/occupancy_grid` (nav_msgs/OccupancyGrid)
  * `goal_reached` (std_msgs/Bool)
  * `/env/episode_done` (std_msgs/String) – reinicio de circuito

Autor: ChatGPT — 2025-11
"""

import math
from collections import deque
from typing import List

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool, String, UInt32
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf_transformations import quaternion_from_euler

# ---------- Umbrales de colisión y búsqueda ----------
OBSTACLE_COST_TH = 50        # ≥50 ⇒ ocupado
UNKNOWN_COST_TH  = -1        # -1 ⇒ desconocido → trátalo como obstáculo
CLEARANCE_M      = 0.30      # [m] radio libre alrededor del goal
MAX_SEARCH_M     = 2.0       # [m] radio máx. para "rescatar" el goal

class SequentialGoalSelector(Node):
    def __init__(self):
        super().__init__('sequential_goal_selector')

        # --- Parámetros de usuario ---
        self.declare_parameter('goal_points', [0.0, 0.0, 0.0])
        self.declare_parameter('republish_hz', 2.0)
        self.declare_parameter('loop', True)
        self.declare_parameter('compute_yaw_from_path', True)
        self.declare_parameter('goal_frame_id', 'map')
        self.declare_parameter('reset_values', ['STUCK'])
        self.declare_parameter('signal_done_on_last', True)
        self.declare_parameter('hold_when_done', True)

        raw_points = (
            self.get_parameter('goal_points').get_parameter_value()
                                  .double_array_value
        )
        if len(raw_points) % 3 != 0:
            raise ValueError(
                f"El parámetro 'goal_points' debe tener longitud múltiplo de 3 (x,y,z). "
                f"Recibido: {len(raw_points)} elementos."
            )
        self.points: List[Pose] = []
        for i in range(0, len(raw_points), 3):
            x, y, z = raw_points[i:i+3]
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.position.z = float(z)
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, 0.0)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
            self.points.append(p)

        self.loop                   = bool(self.get_parameter('loop').value)
        self.compute_yaw_from_path  = bool(self.get_parameter('compute_yaw_from_path').value)
        self.frame_id               = str(self.get_parameter('goal_frame_id').value)
        self.reset_values           = list(self.get_parameter('reset_values').value)
        self.signal_done_on_last    = bool(self.get_parameter('signal_done_on_last').value)
        self.hold_when_done         = bool(self.get_parameter('hold_when_done').value)

        hz = float(self.get_parameter('republish_hz').value)
        if hz <= 0:
            hz = 2.0
            self.get_logger().warn('republish_hz <= 0; usando 2.0 Hz por defecto.')

        # --- QoS ---
        qos_done = QoSProfile(depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # --- Suscriptores ---
        self.map_sub = self.create_subscription(OccupancyGrid, '/occupancy_grid', self.map_cb, 10)
        self.goal_reached_sub = self.create_subscription(Bool, 'goal_reached', self.goal_reached_cb, 10)
        self.episode_done_sub = self.create_subscription(String, '/env/episode_done', self.episode_done_cb, 10)

        # --- Publicadores ---
        self.goal_pub   = self.create_publisher(PoseArray, 'goal', 10)
        self.marker_pub = self.create_publisher(Marker, 'goal_marker', 10)
        self.visited_pub = self.create_publisher(UInt32, '/circuit/visited', 10)
        self.done_pub    = self.create_publisher(Bool, '/circuit/done', qos_done)

        # --- Timer ---
        self.timer = self.create_timer(1.0 / hz, self.timer_cb)

        # --- Estado ---
        self.idx = 0
        self.current_goal: Pose | None = None
        self.goal_active = False
        self.grid_data = None
        self.grid_info = None
        self.visited = 0

        self.get_logger().info(f"✔️  Cargados {len(self.points)} puntos. Inicio en el punto #0.")
        self.publish_index(self.idx, initial=True)

    # ==================== Callbacks ====================
    def map_cb(self, msg: OccupancyGrid):
        self.grid_data = msg.data
        self.grid_info = msg.info
        if self.goal_active and self.current_goal and self.is_goal_in_collision(self.current_goal):
            self.get_logger().info('⚠️  La meta actual ha pasado a estar en colisión; corrigiendo…')
            fixed = self.closest_free_pose(self.current_goal)
            if fixed:
                self.current_goal = fixed
                self.publish_pose(fixed)

    def goal_reached_cb(self, msg: Bool):
        if not msg.data:
            return
        # Incrementa contador de puntos visitados
        self.visited += 1
        self.visited_pub.publish(UInt32(data=self.visited))

        # ¿Último punto?
        at_last = (self.idx == len(self.points) - 1)
        if (not self.loop) and at_last and self.signal_done_on_last:
            self.get_logger().info('🏁 Fin de circuito alcanzado.')
            self.done_pub.publish(Bool(data=True))
            if self.hold_when_done:
                self.goal_active = False   # deja de republicar
                return
            # Si no hacemos hold, simplemente no avanzamos más porque loop=False
            return

        # En otro caso, avanzamos
        self.advance_to_next()

    def episode_done_cb(self, msg: String):
        if msg.data in self.reset_values:
            self.get_logger().info(f"🔄 Reset por episodio: '{msg.data}'. Volviendo al primer punto…")
            self.reset_circuit()

    def timer_cb(self):
        if self.goal_active and self.current_goal:
            self.publish_pose(self.current_goal)

    # ==================== Lógica de circuito ====================
    def reset_circuit(self):
        self.idx = 0
        self.visited = 0
        self.publish_index(self.idx)

    def advance_to_next(self):
        if not self.points:
            return
        next_idx = self.idx + 1
        if next_idx >= len(self.points):
            if self.loop:
                next_idx = 0
            else:
                self.get_logger().info('🏁 Fin de lista y loop=False. No publico más metas.')
                self.goal_active = False
                return
        self.idx = next_idx
        self.publish_index(self.idx)

    def publish_index(self, index: int, initial: bool = False):
        base_pose = self.copy_pose(self.points[index])
        if self.compute_yaw_from_path and len(self.points) >= 2:
            nxt = (index + 1) % len(self.points) if self.loop else min(index + 1, len(self.points) - 1)
            yaw = math.atan2(self.points[nxt].position.y - base_pose.position.y,
                             self.points[nxt].position.x - base_pose.position.x)
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
            base_pose.orientation.x, base_pose.orientation.y, base_pose.orientation.z, base_pose.orientation.w = qx, qy, qz, qw

        pose_to_publish = base_pose
        if self.grid_info is not None:
            if self.is_goal_in_collision(base_pose):
                fixed = self.closest_free_pose(base_pose)
                if fixed:
                    pose_to_publish = fixed
                else:
                    self.get_logger().warn('No pude encontrar celda libre cercana; publicaré el punto original igualmente.')

        self.current_goal = pose_to_publish
        self.goal_active = True
        self.publish_pose(pose_to_publish)

        if not initial:
            self.get_logger().info(
                f"➡️  Meta #{index} publicada: x={pose_to_publish.position.x:.2f}, "
                f"y={pose_to_publish.position.y:.2f}, z={pose_to_publish.position.z:.2f}")

    # ==================== Publicación ====================
    def publish_pose(self, pose: Pose):
        arr = PoseArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = self.frame_id
        arr.poses.append(pose)
        self.goal_pub.publish(arr)

        m = Marker()
        m.header = arr.header
        m.ns = 'goal_marker'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = pose
        m.scale.x = m.scale.y = m.scale.z = 2.0
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 1.0
        self.marker_pub.publish(m)

    # ==================== Utilidades Occupancy ====================
    def world_to_grid(self, pose: Pose):
        if not self.grid_info:
            return None
        res = self.grid_info.resolution
        gx = int((pose.position.x - self.grid_info.origin.position.x) / res)
        gy = int((pose.position.y - self.grid_info.origin.position.y) / res)
        if 0 <= gx < self.grid_info.width and 0 <= gy < self.grid_info.height:
            return (gx, gy)
        return None

    def grid_to_world_pose(self, gx: int, gy: int, ref_pose: Pose):
        res = self.grid_info.resolution
        p = Pose()
        p.position.x = self.grid_info.origin.position.x + (gx + 0.5) * res
        p.position.y = self.grid_info.origin.position.y + (gy + 0.5) * res
        p.position.z = ref_pose.position.z
        p.orientation = ref_pose.orientation
        return p

    def is_cell_occupied(self, gx: int, gy: int) -> bool:
        idx = gy * self.grid_info.width + gx
        if idx < 0 or idx >= len(self.grid_data):
            return True
        cost = self.grid_data[idx]
        return (cost >= OBSTACLE_COST_TH) or (cost == UNKNOWN_COST_TH)

    def has_clearance(self, cx: int, cy: int, rad_cells: int) -> bool:
        res = self.grid_info.resolution
        for dx in range(-rad_cells, rad_cells + 1):
            for dy in range(-rad_cells, rad_cells + 1):
                if math.hypot(dx, dy) * res >= CLEARANCE_M:
                    continue
                nx, ny = cx + dx, cy + dy
                if (0 <= nx < self.grid_info.width and
                    0 <= ny < self.grid_info.height and
                    self.is_cell_occupied(nx, ny)):
                    return False
        return True

    def is_goal_in_collision(self, pose: Pose) -> bool:
        cell = self.world_to_grid(pose)
        return (cell is not None) and self.is_cell_occupied(cell[0], cell[1])

    def closest_free_pose(self, origin_pose: Pose):
        start = self.world_to_grid(origin_pose)
        if start is None:
            return None
        res = self.grid_info.resolution
        max_cells = int(MAX_SEARCH_M / res)
        clearance_cells = int(CLEARANCE_M / res)
        q = deque([(start[0], start[1], 0)])
        visited = { (start[0], start[1]) }
        neigh = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
        while q:
            x, y, d = q.popleft()
            if d > max_cells:
                break
            if (not self.is_cell_occupied(x, y) and
                self.has_clearance(x, y, clearance_cells)):
                return self.grid_to_world_pose(x, y, origin_pose)
            for dx, dy in neigh:
                nx, ny = x + dx, y + dy
                if (0 <= nx < self.grid_info.width and
                    0 <= ny < self.grid_info.height and
                    (nx, ny) not in visited):
                    visited.add((nx, ny))
                    q.append((nx, ny, d + 1))
        return None

    # ==================== Miscelánea ====================
    @staticmethod
    def copy_pose(p: Pose) -> Pose:
        q = Pose()
        q.position.x = p.position.x
        q.position.y = p.position.y
        q.position.z = p.position.z
        q.orientation = p.orientation
        return q


def main(args=None):
    rclpy.init(args=args)
    node = SequentialGoalSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
