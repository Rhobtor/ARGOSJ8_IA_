#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path as FilePath
from typing import Optional, Tuple
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, String

from path_manager_interfaces.srv import ReadPathFromFile, ReturnRobotPath, RobotPath, WritePathToFile


def namespaced(namespace: str, name: str) -> str:
    if not name:
        return name
    if name.startswith('/'):
        return name
    return f'/{namespace}/{name}'


class ExternalPathController(Node):
    """Expose the GUI external-path contract and publish GNSS goals sequentially."""

    def __init__(self) -> None:
        super().__init__('external_path_controller')

        self.declare_parameter('namespace', 'ARGJ801')
        self.declare_parameter('receive_path_service', 'receive_external_path')
        self.declare_parameter('read_path_service', 'read_external_path_file')
        self.declare_parameter('write_path_service', 'write_external_path_file')
        self.declare_parameter('return_path_service', 'get_external_robot_Path')
        self.declare_parameter('command_topic', 'external_path_command')

        self.declare_parameter('goal_topic', '/goal_gnss')
        self.declare_parameter('goal_reached_topic', 'goal_reached')
        self.declare_parameter('goal_frame_id', 'gps')
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
        self._namespace = str(self.get_parameter('namespace').value or 'ARGJ801').strip().strip('/') or 'ARGJ801'
        self._storage_dir = FilePath(str(self.get_parameter('storage_dir').value)).expanduser()
        self._storage_dir.mkdir(parents=True, exist_ok=True)

        goal_topic = str(self.get_parameter('goal_topic').value)
        goal_reached_topic = str(self.get_parameter('goal_reached_topic').value)
        command_topic = namespaced(self._namespace, str(self.get_parameter('command_topic').value))

        self._goal_pub = self.create_publisher(NavSatFix, goal_topic, qos_profile_sensor_data)
        self.create_subscription(Bool, goal_reached_topic, self._on_goal_reached, 10)
        self.create_subscription(String, command_topic, self._on_command, 10)

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
            f'goal_reached={goal_reached_topic} storage_dir={self._storage_dir}'
        )

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
        if announce:
            self.get_logger().info(
                f'Waypoint externo #{self._current_index} -> lat={lat_deg:.8f} lon={lon_deg:.8f} alt={alt_m:.2f}'
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