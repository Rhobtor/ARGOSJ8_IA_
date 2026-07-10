#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path as FilePath
from typing import Optional, Tuple
import xml.etree.ElementTree as ET
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from path_manager_interfaces.srv import ReadPathFromFile, ReturnRobotPath, RobotPath, WritePathToFile


def namespaced(namespace: str, name: str) -> str:
    if not name:
        return name
    if name.startswith('/'):
        return name
    return f'/{namespace}/{name}'


class ExternalPathController(Node):
    """Expose the GUI external-path contract and publish GNSS and local goals sequentially."""

    def __init__(self) -> None:
        super().__init__('external_path_controller')

        self.declare_parameter('namespace', 'ARGJ801')
        self.declare_parameter('receive_path_service', 'receive_external_path')
        self.declare_parameter('read_path_service', 'read_external_path_file')
        self.declare_parameter('write_path_service', 'write_external_path_file')
        self.declare_parameter('return_path_service', 'get_external_robot_Path')
        self.declare_parameter('command_topic', 'external_path_command')

        self.declare_parameter('goal_topic', '/goal_gnss')
        self.declare_parameter('local_goal_topic', '/goal_local')
        self.declare_parameter('goal_reached_topic', 'goal_reached')
        self.declare_parameter('goal_frame_id', 'gps')
        self.declare_parameter('local_fix_topic', '/fixposition/navsatfix')
        self.declare_parameter('local_odom_topic', '/fixposition/odometry_enu')
        self.declare_parameter('local_goal_mode', 'map')
        self.declare_parameter('local_goal_frame_id', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('local_fixed_frame', 'FP_ENU0')
        self.declare_parameter('ecef_frame_id', 'ECEF')
        self.declare_parameter('local_goal_use_altitude', False)
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('republish_hz', 2.0)
        self.declare_parameter('loop', False)
        self.declare_parameter('hold_when_done', True)
        self.declare_parameter('storage_dir', str(FilePath.home() / '.ros' / 'car_external_paths'))

        self._path = NavPath()
        self._path.header.frame_id = 'wgs84'
        self._current_index = 0
        self._active = False
        self._finished = False

        self._loop = bool(self.get_parameter('loop').value)
        self._hold_when_done = bool(self.get_parameter('hold_when_done').value)
        self._goal_frame_id = str(self.get_parameter('goal_frame_id').value)
        self._local_goal_mode = str(self.get_parameter('local_goal_mode').value or 'body').strip().lower()
        self._local_goal_frame_id = str(self.get_parameter('local_goal_frame_id').value)
        self._map_frame = str(self.get_parameter('map_frame').value or 'map').strip() or 'map'
        self._local_fixed_frame = str(self.get_parameter('local_fixed_frame').value or 'FP_ENU0').strip() or 'FP_ENU0'
        self._ecef_frame_id = str(self.get_parameter('ecef_frame_id').value or 'ECEF').strip() or 'ECEF'
        self._local_goal_use_altitude = bool(self.get_parameter('local_goal_use_altitude').value)
        self._robot_frame = str(self.get_parameter('robot_frame').value or 'base_link').strip() or 'base_link'
        self._namespace = str(self.get_parameter('namespace').value or 'ARGJ801').strip().strip('/') or 'ARGJ801'
        self._storage_dir = FilePath(str(self.get_parameter('storage_dir').value)).expanduser()
        self._storage_dir.mkdir(parents=True, exist_ok=True)
        self._latest_fix: Optional[NavSatFix] = None
        self._latest_local_odom: Optional[Odometry] = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._ecef_transformer = self._create_ecef_transformer()

        goal_topic = str(self.get_parameter('goal_topic').value)
        local_goal_topic = str(self.get_parameter('local_goal_topic').value)
        goal_reached_topic = str(self.get_parameter('goal_reached_topic').value)
        local_fix_topic = str(self.get_parameter('local_fix_topic').value)
        local_odom_topic = str(self.get_parameter('local_odom_topic').value)
        command_topic = namespaced(self._namespace, str(self.get_parameter('command_topic').value))

        self._goal_pub = self.create_publisher(NavSatFix, goal_topic, qos_profile_sensor_data)
        self._local_goal_pub = self.create_publisher(PoseArray, local_goal_topic, 10)
        self.create_subscription(Bool, goal_reached_topic, self._on_goal_reached, 10)
        self.create_subscription(String, command_topic, self._on_command, 10)
        self.create_subscription(NavSatFix, local_fix_topic, self._on_fix, qos_profile_sensor_data)
        self.create_subscription(Odometry, local_odom_topic, self._on_local_odom, 10)

        receive_path_service = namespaced(self._namespace, str(self.get_parameter('receive_path_service').value))
        read_path_service = namespaced(self._namespace, str(self.get_parameter('read_path_service').value))
        write_path_service = namespaced(self._namespace, str(self.get_parameter('write_path_service').value))
        return_path_service = namespaced(self._namespace, str(self.get_parameter('return_path_service').value))

        self.create_service(RobotPath, receive_path_service, self._receive_path)
        self.create_service(ReadPathFromFile, read_path_service, self._read_path)
        self.create_service(WritePathToFile, write_path_service, self._write_path)
        self.create_service(ReturnRobotPath, return_path_service, self._return_path)

        hz = float(self.get_parameter('republish_hz').value)
        if hz > 0.0:
            self.create_timer(1.0 / hz, self._republish_current_goal)

        self.get_logger().info(
            f'external_path_controller listo. namespace={self._namespace} goal_topic={goal_topic} '
            f'local_goal_topic={local_goal_topic} local_goal_mode={self._local_goal_mode} '
            f'goal_reached={goal_reached_topic} storage_dir={self._storage_dir}'
        )

    def _on_fix(self, msg: NavSatFix) -> None:
        self._latest_fix = msg

    def _on_local_odom(self, msg: Odometry) -> None:
        self._latest_local_odom = msg

    def _receive_path(self, req: RobotPath.Request, res: RobotPath.Response) -> RobotPath.Response:
        self._path = self._clone_path(req.path)
        if not self._path.header.frame_id:
            self._path.header.frame_id = 'wgs84'
        self._reset_runtime(keep_path=True)
        res.ack = True
        self.get_logger().info(f'Ruta externa recibida: {len(self._path.poses)} waypoints')
        return res

    def _read_path(self, req: ReadPathFromFile.Request, res: ReadPathFromFile.Response) -> ReadPathFromFile.Response:
        filename = self._normalize_filename(req.filename)
        if not filename:
            res.success = False
            return res

        try:
            self._path = self._load_gpx(filename)
            self._reset_runtime(keep_path=True)
            res.success = True
            self.get_logger().info(f'Ruta externa cargada: {filename} ({len(self._path.poses)} waypoints)')
        except Exception as exc:
            self.get_logger().error(f'No se pudo leer la ruta externa {filename}: {exc}')
            res.success = False
        return res

    def _write_path(self, req: WritePathToFile.Request, res: WritePathToFile.Response) -> WritePathToFile.Response:
        filename = self._normalize_filename(req.filename)
        if not filename or not self._path.poses:
            res.success = False
            return res

        try:
            self._save_gpx(filename, self._path)
            self.get_logger().info(f'Ruta externa guardada: {filename}')
            res.success = True
        except Exception as exc:
            self.get_logger().error(f'No se pudo escribir la ruta externa {filename}: {exc}')
            res.success = False
        return res

    def _return_path(self, req: ReturnRobotPath.Request, res: ReturnRobotPath.Response) -> ReturnRobotPath.Response:
        del req
        res.path = self._clone_path(self._path)
        return res

    def _on_command(self, msg: String) -> None:
        command = str(msg.data or '').strip().lower()
        if command == 'start':
            self._start_path()
            return
        if command == 'stop':
            self._active = False
            self.get_logger().info('Ruta externa detenida')
            return
        if command == 'clear':
            self._path = NavPath()
            self._path.header.frame_id = 'wgs84'
            self._reset_runtime(keep_path=True)
            self.get_logger().info('Ruta externa borrada')
            return
        if command:
            self.get_logger().warn(f'Comando externo desconocido: {command}')

    def _start_path(self) -> None:
        if not self._path.poses:
            self.get_logger().warn('No hay ruta externa cargada para arrancar')
            return

        if self._finished or self._current_index >= len(self._path.poses):
            self._current_index = 0
            self._finished = False

        self._active = True
        self._publish_current_goal(announce=True)

    def _on_goal_reached(self, msg: Bool) -> None:
        if not msg.data or not self._active or not self._path.poses:
            return

        at_last = self._current_index >= len(self._path.poses) - 1
        if at_last and not self._loop:
            self._finished = True
            self._active = not self._hold_when_done
            self.get_logger().info('Ultimo waypoint externo alcanzado')
            return

        self._current_index += 1
        if self._current_index >= len(self._path.poses):
            self._current_index = 0
        self._publish_current_goal(announce=True)

    def _republish_current_goal(self) -> None:
        if self._active:
            self._publish_current_goal(announce=False)

    def _publish_current_goal(self, announce: bool) -> None:
        if not self._path.poses:
            return

        pose_stamped = self._path.poses[self._current_index]
        lat_deg, lon_deg, alt_m = self._extract_lat_lon_alt(pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z)

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._goal_frame_id
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = 0
        msg.latitude = lat_deg
        msg.longitude = lon_deg
        msg.altitude = alt_m
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self._goal_pub.publish(msg)
        self._publish_current_local_goal(lat_deg, lon_deg, alt_m, announce)
        if announce:
            self.get_logger().info(
                f'Waypoint externo #{self._current_index} -> lat={lat_deg:.8f} lon={lon_deg:.8f} alt={alt_m:.2f}'
            )

    def _publish_current_local_goal(self, lat_deg: float, lon_deg: float, alt_m: float, announce: bool) -> None:
        needs_odom = self._local_goal_mode in {'map', 'body', 'map_local'}
        if self._latest_fix is None or (needs_odom and self._latest_local_odom is None):
            if announce:
                self.get_logger().warn(
                    'Sin referencia local disponible para publicar /goal_local '
                    f'(fix={self._latest_fix is not None} odom={self._latest_local_odom is not None})'
                )
            return

        east_m, north_m, up_m = self._latlon_to_enu_delta(lat_deg, lon_deg, alt_m)
        local_goal = Pose()
        local_goal.orientation.w = 1.0

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        if self._local_goal_mode == 'map':
            x_fwd_m, y_left_m = self._enu_to_body_delta(east_m, north_m)
            try:
                robot_in_map = self._tf_buffer.lookup_transform(
                    self._map_frame,
                    self._robot_frame,
                    rclpy.time.Time(),
                )
            except Exception as exc:
                if announce:
                    self.get_logger().warn(
                        f'Lookup TF fallido {self._map_frame} <- {self._robot_frame}: {exc}',
                        throttle_duration_sec=1.0,
                    )
                return

            map_delta = self._rotate_vector_by_quaternion(
                (
                    float(robot_in_map.transform.rotation.x),
                    float(robot_in_map.transform.rotation.y),
                    float(robot_in_map.transform.rotation.z),
                    float(robot_in_map.transform.rotation.w),
                ),
                (
                    x_fwd_m,
                    y_left_m,
                    up_m if self._local_goal_use_altitude else 0.0,
                ),
            )
            local_goal.position.x = float(robot_in_map.transform.translation.x) + map_delta[0]
            local_goal.position.y = float(robot_in_map.transform.translation.y) + map_delta[1]
            local_goal.position.z = (
                float(robot_in_map.transform.translation.z) + map_delta[2]
            ) if self._local_goal_use_altitude else 0.0
            msg.header.frame_id = self._local_goal_frame_id or self._map_frame
        elif self._local_goal_mode == 'map_local':
            x_fwd_m, y_left_m = self._enu_to_body_delta(east_m, north_m)
            local_goal.position.x = x_fwd_m
            local_goal.position.y = y_left_m
            local_goal.position.z = up_m if self._local_goal_use_altitude else 0.0
            msg.header.frame_id = self._local_goal_frame_id or self._map_frame
        elif self._local_goal_mode == 'map_axes':
            robot_alt_m = float(self._latest_fix.altitude)
            goal_alt_m = float(alt_m) if self._local_goal_use_altitude else robot_alt_m

            goal_map_xyz = self._map_point_from_fix(lat_deg, lon_deg, goal_alt_m)
            robot_map_xyz = self._map_point_from_fix(
                float(self._latest_fix.latitude),
                float(self._latest_fix.longitude),
                robot_alt_m,
            )

            if goal_map_xyz is not None and robot_map_xyz is not None:
                local_goal.position.x = goal_map_xyz[0] - robot_map_xyz[0]
                local_goal.position.y = goal_map_xyz[1] - robot_map_xyz[1]
                local_goal.position.z = (goal_map_xyz[2] - robot_map_xyz[2]) if self._local_goal_use_altitude else 0.0
            else:
                if announce:
                    self.get_logger().warn(
                        f'TF global {self._map_frame}<-{self._ecef_frame_id} no disponible; usando aproximacion ENU local',
                        throttle_duration_sec=1.0,
                    )
                source_frame = self._local_fixed_frame
                map_xyz = self._transform_vector((east_m, north_m, up_m), source_frame, self._map_frame)
                if map_xyz is None:
                    if announce:
                        self.get_logger().warn(
                            f'No se pudo calcular el goal local en {self._map_frame} ni con TF global ni con ENU local'
                        )
                    return

                local_goal.position.x = map_xyz[0]
                local_goal.position.y = map_xyz[1]
                local_goal.position.z = map_xyz[2]
            msg.header.frame_id = self._local_goal_frame_id or self._map_frame
        elif self._local_goal_mode in {'enu', 'world'}:
            goal_xyz = self._goal_in_local_fixed_frame(east_m, north_m, up_m)
            local_goal.position.x = goal_xyz[0]
            local_goal.position.y = goal_xyz[1]
            local_goal.position.z = goal_xyz[2]
            msg.header.frame_id = self._local_goal_frame_id or self._latest_local_odom.header.frame_id or self._local_fixed_frame
        else:
            x_fwd_m, y_left_m = self._enu_to_body_delta(east_m, north_m)
            local_goal.position.x = x_fwd_m
            local_goal.position.y = y_left_m
            local_goal.position.z = up_m
            msg.header.frame_id = self._local_goal_frame_id or self._latest_local_odom.child_frame_id or 'base_link'

        msg.poses.append(local_goal)

        self._local_goal_pub.publish(msg)
        if announce:
            self.get_logger().info(
                f'Waypoint local #{self._current_index} -> x={local_goal.position.x:.3f} '
                f'y={local_goal.position.y:.3f} z={local_goal.position.z:.3f} frame={msg.header.frame_id}'
            )

    def _extract_lat_lon_alt(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        frame_id = str(self._path.header.frame_id or '').strip().lower()

        if frame_id in {'ll', 'latlon', 'lat_lon'}:
            lat_deg = float(x)
            lon_deg = float(y)
        else:
            lon_deg = float(x)
            lat_deg = float(y)

        if abs(lat_deg) > 90.0 and abs(lon_deg) <= 90.0:
            lat_deg, lon_deg = lon_deg, lat_deg

        return (lat_deg, lon_deg, float(z))

    @staticmethod
    def _create_ecef_transformer():
        try:
            from pyproj import Transformer
        except Exception as exc:  # pragma: no cover
            raise RuntimeError(
                'No se pudo importar pyproj. Instala el paquete (ej: python3-pyproj o pip install pyproj).'
            ) from exc

        return Transformer.from_crs('EPSG:4326', 'EPSG:4978', always_xy=True)

    def _map_point_from_fix(self, lat_deg: float, lon_deg: float, alt_m: float) -> Optional[Tuple[float, float, float]]:
        x_ecef, y_ecef, z_ecef = self._ecef_transformer.transform(float(lon_deg), float(lat_deg), float(alt_m))
        return self._transform_point((float(x_ecef), float(y_ecef), float(z_ecef)), self._ecef_frame_id, self._map_frame)

    def _latlon_to_enu_delta(self, lat_deg: float, lon_deg: float, alt_m: float) -> Tuple[float, float, float]:
        if self._latest_fix is None:
            return (0.0, 0.0, 0.0)

        ref_lat = float(self._latest_fix.latitude)
        ref_lon = float(self._latest_fix.longitude)
        ref_alt = float(self._latest_fix.altitude)
        radius_m = 6378137.0

        north_m = math.radians(float(lat_deg) - ref_lat) * radius_m
        east_m = (
            math.radians(float(lon_deg) - ref_lon)
            * radius_m
            * max(math.cos(math.radians(ref_lat)), 1e-9)
        )
        up_m = float(alt_m) - ref_alt
        return (east_m, north_m, up_m)

    def _enu_to_body_delta(self, east_m: float, north_m: float) -> Tuple[float, float]:
        if self._latest_local_odom is None:
            return (east_m, north_m)

        orientation = self._latest_local_odom.pose.pose.orientation
        yaw = self._yaw_from_quaternion(
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        x_fwd_m = (cos_yaw * east_m) + (sin_yaw * north_m)
        y_left_m = (-sin_yaw * east_m) + (cos_yaw * north_m)
        return (x_fwd_m, y_left_m)

    def _goal_in_local_fixed_frame(self, east_m: float, north_m: float, up_m: float) -> Tuple[float, float, float]:
        if self._latest_local_odom is None:
            return (east_m, north_m, up_m)

        odom_pose = self._latest_local_odom.pose.pose.position
        return (
            float(odom_pose.x) + east_m,
            float(odom_pose.y) + north_m,
            float(odom_pose.z) + up_m,
        )

    def _transform_point(
        self,
        point_xyz: Tuple[float, float, float],
        source_frame: str,
        target_frame: str,
    ) -> Optional[Tuple[float, float, float]]:
        if source_frame == target_frame:
            return point_xyz

        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
            )
        except Exception as exc:
            self.get_logger().warn(
                f'Lookup TF fallido {target_frame} <- {source_frame}: {exc}',
                throttle_duration_sec=1.0,
            )
            return None

        tx = float(transform.transform.translation.x)
        ty = float(transform.transform.translation.y)
        tz = float(transform.transform.translation.z)
        qx = float(transform.transform.rotation.x)
        qy = float(transform.transform.rotation.y)
        qz = float(transform.transform.rotation.z)
        qw = float(transform.transform.rotation.w)
        rx, ry, rz = self._rotate_vector_by_quaternion((qx, qy, qz, qw), point_xyz)
        return (tx + rx, ty + ry, tz + rz)

    def _transform_vector(
        self,
        vector_xyz: Tuple[float, float, float],
        source_frame: str,
        target_frame: str,
    ) -> Optional[Tuple[float, float, float]]:
        if source_frame == target_frame:
            return vector_xyz

        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
            )
        except Exception as exc:
            self.get_logger().warn(
                f'Lookup TF fallido {target_frame} <- {source_frame}: {exc}',
                throttle_duration_sec=1.0,
            )
            return None

        qx = float(transform.transform.rotation.x)
        qy = float(transform.transform.rotation.y)
        qz = float(transform.transform.rotation.z)
        qw = float(transform.transform.rotation.w)
        return self._rotate_vector_by_quaternion((qx, qy, qz, qw), vector_xyz)

    def _rotate_vector_by_quaternion(
        self,
        quaternion: Tuple[float, float, float, float],
        vector_xyz: Tuple[float, float, float],
    ) -> Tuple[float, float, float]:
        qx, qy, qz, qw = quaternion
        vx, vy, vz = vector_xyz
        tx = 2.0 * ((qy * vz) - (qz * vy))
        ty = 2.0 * ((qz * vx) - (qx * vz))
        tz = 2.0 * ((qx * vy) - (qy * vx))
        cx = (qy * tz) - (qz * ty)
        cy = (qz * tx) - (qx * tz)
        cz = (qx * ty) - (qy * tx)
        return (
            vx + (qw * tx) + cx,
            vy + (qw * ty) + cy,
            vz + (qw * tz) + cz,
        )

    def _yaw_from_quaternion(self, x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * ((w * z) + (x * y))
        cosy_cosp = 1.0 - 2.0 * ((y * y) + (z * z))
        return math.atan2(siny_cosp, cosy_cosp)

    def _clone_path(self, msg: NavPath) -> NavPath:
        out = NavPath()
        out.header = msg.header
        out.poses = list(msg.poses)
        return out

    def _reset_runtime(self, keep_path: bool) -> None:
        self._active = False
        self._finished = False
        self._current_index = 0
        if not keep_path:
            self._path = NavPath()
            self._path.header.frame_id = 'wgs84'

    def _normalize_filename(self, filename: str) -> Optional[str]:
        name = str(filename or '').strip()
        if not name:
            self.get_logger().warn('Nombre de fichero vacío')
            return None
        return FilePath(name).stem

    def _path_file(self, filename: str) -> FilePath:
        return self._storage_dir / f'{filename}.gpx'

    def _save_gpx(self, filename: str, path: NavPath) -> None:
        root = ET.Element('gpx', version='1.1', creator='ExternalPathController - ROS2')
        trk = ET.SubElement(root, 'trk')
        ET.SubElement(trk, 'name').text = filename
        trkseg = ET.SubElement(trk, 'trkseg')

        for pose_stamped in path.poses:
            lat_deg, lon_deg, alt_m = self._extract_lat_lon_alt(
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                pose_stamped.pose.position.z,
            )
            trkpt = ET.SubElement(trkseg, 'trkpt', lat=f'{lat_deg:.8f}', lon=f'{lon_deg:.8f}')
            ET.SubElement(trkpt, 'ele').text = f'{alt_m:.3f}'

        ET.ElementTree(root).write(self._path_file(filename), encoding='utf-8', xml_declaration=True)

    def _load_gpx(self, filename: str) -> NavPath:
        tree = ET.parse(self._path_file(filename))
        root = tree.getroot()

        path = NavPath()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'wgs84'

        for elem in root.iter():
            if not elem.tag.endswith('trkpt'):
                continue

            lat_attr = elem.attrib.get('lat')
            lon_attr = elem.attrib.get('lon')
            if lat_attr is None or lon_attr is None:
                continue

            alt_m = 0.0
            for child in elem:
                if child.tag.endswith('ele') and child.text:
                    alt_m = float(child.text)
                    break

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(lon_attr)
            pose.pose.position.y = float(lat_attr)
            pose.pose.position.z = float(alt_m)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        return path


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ExternalPathController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()