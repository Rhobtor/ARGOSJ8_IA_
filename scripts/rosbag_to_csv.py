#!/usr/bin/env python3
"""Export ROS 2 rosbag2 topics to CSV.

This script is intentionally simple and focused: it reads a rosbag2 and writes
CSV files per topic. It supports clean, explicit schemas for the common topics
used in this project:

- /cmd_vel (geometry_msgs/msg/Twist)
- /fixposition/odometry (nav_msgs/msg/Odometry)
- /goal (geometry_msgs/msg/PoseArray) – assumes first pose is the active goal
- /circuit/done (std_msgs/msg/Bool)

If you export a topic with an unknown type, it falls back to a single `json`
column with the full message payload.

Examples
--------

Export the usual topics to ./csv_out:

  python3 scripts/rosbag_to_csv.py \
    --bag /path/to/bag_dir \
    --out csv_out \
    --topic /cmd_vel --topic /fixposition/odometry --topic /goal --topic /circuit/done

List topics/types recorded in a bag:

  python3 scripts/rosbag_to_csv.py --bag /path/to/bag_dir --list-topics

Notes
-----
- `--bag` may be the bag directory, the `metadata.yaml`, or the `.db3/.mcap` file.
- Output is one CSV file per topic, with a sanitized filename.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


def _require_ros2_imports():
    try:
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore
        from rclpy.serialization import deserialize_message  # type: ignore

        return SequentialReader, StorageOptions, ConverterOptions, get_message, deserialize_message
    except Exception as exc:  # pragma: no cover
        raise RuntimeError(
            "ROS 2 Python libs not available. Source ROS 2 and your workspace overlay first. "
            "Example: `source /opt/ros/<distro>/setup.bash && source install/setup.bash`. "
            f"Original import error: {exc}"
        )


def normalize_bag_path(path: str) -> str:
    p = os.path.abspath(os.path.expanduser(path))
    if os.path.isdir(p):
        return p
    if os.path.isfile(p):
        # metadata.yaml or storage file
        return os.path.dirname(p)
    return p


def is_rosbag_dir(path: str) -> bool:
    return os.path.isdir(path) and os.path.isfile(os.path.join(path, "metadata.yaml"))


def sanitize_topic_to_filename(topic: str) -> str:
    # '/fixposition/odometry' -> 'fixposition_odometry'
    s = topic.strip().strip("/")
    if not s:
        return "topic"
    out = []
    for ch in s:
        if ch.isalnum() or ch in ("-", "_"):
            out.append(ch)
        else:
            out.append("_")
    return "".join(out)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def message_to_jsonable(obj: Any) -> Any:
    """Best-effort conversion of ROS messages to JSON-able objects."""
    # Common fast-path for primitive-like objects
    if obj is None:
        return None
    if isinstance(obj, (bool, int, float, str)):
        return obj
    if isinstance(obj, (list, tuple)):
        return [message_to_jsonable(x) for x in obj]
    if isinstance(obj, dict):
        return {str(k): message_to_jsonable(v) for k, v in obj.items()}

    # Try rosidl_runtime_py conversion if available
    try:
        from rosidl_runtime_py.convert import message_to_ordereddict  # type: ignore

        return message_to_ordereddict(obj)
    except Exception:
        pass

    # Fallback: reflect over fields
    fields = getattr(obj, "__slots__", None)
    if fields:
        out: Dict[str, Any] = {}
        for f in fields:
            try:
                out[f] = message_to_jsonable(getattr(obj, f))
            except Exception:
                out[f] = None
        return out

    return str(obj)


@dataclass
class TopicInfo:
    name: str
    type: str


def open_reader(bag_dir: str, storage_id: str) -> Tuple[Any, Dict[str, str]]:
    SequentialReader, StorageOptions, ConverterOptions, get_message, deserialize_message = _require_ros2_imports()

    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_dir, storage_id=str(storage_id))
    converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)

    type_map: Dict[str, str] = {}
    for t in reader.get_all_topics_and_types():
        type_map[t.name] = t.type
    return reader, type_map


def list_topics(bag_dir: str, storage_id: str) -> List[TopicInfo]:
    reader, type_map = open_reader(bag_dir, storage_id)
    del reader
    return [TopicInfo(name=k, type=v) for k, v in sorted(type_map.items(), key=lambda kv: kv[0])]


def ensure_out_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def write_csv_header(writer: csv.writer, columns: Sequence[str]) -> None:
    writer.writerow(list(columns))


def export_bag(
    bag_dir: str,
    out_dir: str,
    topics: Sequence[str],
    storage_id: str,
) -> None:
    SequentialReader, StorageOptions, ConverterOptions, get_message, deserialize_message = _require_ros2_imports()

    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_dir, storage_id=str(storage_id))
    converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)

    type_map: Dict[str, str] = {}
    for t in reader.get_all_topics_and_types():
        type_map[t.name] = t.type

    wanted = set(topics)
    missing = [tp for tp in topics if tp not in type_map]
    if missing:
        available = ", ".join(sorted(type_map.keys()))
        raise RuntimeError(f"Missing topics in bag: {missing}. Available: {available}")

    msg_types = {tp: get_message(type_map[tp]) for tp in topics}

    ensure_out_dir(out_dir)

    # Prepare one CSV per topic
    files: Dict[str, Any] = {}
    writers: Dict[str, csv.writer] = {}
    t0_ns_by_topic: Dict[str, Optional[int]] = {tp: None for tp in topics}
    schema_by_topic: Dict[str, str] = {}

    def schema_for_type(type_name: str) -> str:
        if type_name.endswith("geometry_msgs/msg/Twist"):
            return "twist"
        if type_name.endswith("nav_msgs/msg/Odometry"):
            return "odometry"
        if type_name.endswith("geometry_msgs/msg/PoseArray"):
            return "posearray_goal"
        if type_name.endswith("std_msgs/msg/Bool"):
            return "bool"
        return "json"

    for tp in topics:
        schema_by_topic[tp] = schema_for_type(type_map[tp])
        fname = sanitize_topic_to_filename(tp) + ".csv"
        fpath = os.path.join(out_dir, fname)
        f = open(fpath, "w", newline="")
        files[tp] = f
        writers[tp] = csv.writer(f)

        schema = schema_by_topic[tp]
        if schema == "twist":
            write_csv_header(
                writers[tp],
                [
                    "t_ns",
                    "t_s",
                    "linear_x",
                    "linear_y",
                    "linear_z",
                    "angular_x",
                    "angular_y",
                    "angular_z",
                ],
            )
        elif schema == "odometry":
            write_csv_header(
                writers[tp],
                [
                    "t_ns",
                    "t_s",
                    "x",
                    "y",
                    "z",
                    "qx",
                    "qy",
                    "qz",
                    "qw",
                    "yaw",
                    "vx",
                    "vy",
                    "vz",
                    "wx",
                    "wy",
                    "wz",
                ],
            )
        elif schema == "posearray_goal":
            write_csv_header(
                writers[tp],
                [
                    "t_ns",
                    "t_s",
                    "goal_x",
                    "goal_y",
                    "goal_z",
                ],
            )
        elif schema == "bool":
            write_csv_header(writers[tp], ["t_ns", "t_s", "data"])
        else:
            write_csv_header(writers[tp], ["t_ns", "t_s", "json"])

    try:
        while reader.has_next():
            topic, data, t_ns = reader.read_next()
            if topic not in wanted:
                continue

            if t0_ns_by_topic[topic] is None:
                t0_ns_by_topic[topic] = int(t_ns)
            t_s = (float(t_ns) - float(t0_ns_by_topic[topic])) / 1e9

            msg = deserialize_message(data, msg_types[topic])
            schema = schema_by_topic[topic]
            w = writers[topic]

            if schema == "twist":
                w.writerow(
                    [
                        int(t_ns),
                        t_s,
                        float(msg.linear.x),
                        float(msg.linear.y),
                        float(msg.linear.z),
                        float(msg.angular.x),
                        float(msg.angular.y),
                        float(msg.angular.z),
                    ]
                )
            elif schema == "odometry":
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                yaw = yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w))
                tw = msg.twist.twist
                w.writerow(
                    [
                        int(t_ns),
                        t_s,
                        float(p.x),
                        float(p.y),
                        float(p.z),
                        float(q.x),
                        float(q.y),
                        float(q.z),
                        float(q.w),
                        float(yaw),
                        float(tw.linear.x),
                        float(tw.linear.y),
                        float(tw.linear.z),
                        float(tw.angular.x),
                        float(tw.angular.y),
                        float(tw.angular.z),
                    ]
                )
            elif schema == "posearray_goal":
                poses = getattr(msg, "poses", [])
                if poses:
                    p0 = poses[0].position
                    gx, gy, gz = float(p0.x), float(p0.y), float(p0.z)
                else:
                    gx = gy = gz = float("nan")
                w.writerow([int(t_ns), t_s, gx, gy, gz])
            elif schema == "bool":
                w.writerow([int(t_ns), t_s, bool(getattr(msg, "data", False))])
            else:
                payload = message_to_jsonable(msg)
                w.writerow([int(t_ns), t_s, json.dumps(payload, ensure_ascii=False)])
    finally:
        for f in files.values():
            try:
                f.close()
            except Exception:
                pass


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="Bag directory, metadata.yaml, or storage file (.db3/.mcap)")
    ap.add_argument("--out", default="csv_out", help="Output directory")
    ap.add_argument(
        "--storage-id",
        default="sqlite3",
        help="rosbag2 storage id (commonly 'sqlite3' or 'mcap').",
    )
    ap.add_argument("--list-topics", action="store_true")
    ap.add_argument(
        "--topic",
        action="append",
        default=[],
        help="Topic name to export (repeatable). If omitted, exports the common set.",
    )

    args = ap.parse_args(argv)
    bag_dir = normalize_bag_path(str(args.bag))
    if not is_rosbag_dir(bag_dir):
        raise FileNotFoundError(
            "Not a rosbag2 directory (missing metadata.yaml). "
            f"Got: {bag_dir}"
        )

    if args.list_topics:
        for ti in list_topics(bag_dir, storage_id=str(args.storage_id)):
            print(f"{ti.name}\t{ti.type}")
        return 0

    topics = list(args.topic)
    if not topics:
        topics = ["/cmd_vel", "/fixposition/odometry", "/goal", "/circuit/done"]

    export_bag(bag_dir, out_dir=str(args.out), topics=topics, storage_id=str(args.storage_id))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
