#!/usr/bin/env python3
"""Generate paper-style plots from ROS 2 rosbag2 trials.

Implements the two plots described in the request:

Plot A (smoothness / dithering):
  - Steering effort: S_w = ∫ |w(t)| dt  from /cmd_vel (angular.z)
  - Oscillation index: Z_w = number of sign changes of w

Plot B (progress to waypoint):
  - Distance to current waypoint d_wp(t) from /fixposition/odometry + /goal
  - Heading error |Δθ(t)| to current waypoint

The script prefers reading the current waypoint from the recorded /goal topic
(published by the node `circuit1`), so it stays consistent with what the
supervisor/circuit actually used.

Usage examples
--------------

Plot A across methods/trials (one figure):

  python3 scripts/paper_plots_rosbag.py \
    --plot A \
    --trial OCRA:/path/to/ocra_trial_01 \
    --trial OCRA:/path/to/ocra_trial_02 \
    --trial Baseline:/path/to/base_trial_01 \
    --out plots

Plot B for one representative trial per method:

  python3 scripts/paper_plots_rosbag.py \
    --plot B \
    --trial OCRA:/path/to/ocra_trial_01 \
    --trial Baseline:/path/to/base_trial_01 \
    --b-trial-index 0 \
    --out plots

Notes
-----
- `--trial` expects a rosbag2 directory containing metadata.yaml.
- If `/circuit/done` exists in the bag, metrics are computed until the first
  True; otherwise they use the full bag span for the topic.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


def _require_ros2_imports():
    try:
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore
        from rclpy.serialization import deserialize_message  # type: ignore

        return SequentialReader, StorageOptions, ConverterOptions, get_message, deserialize_message
    except Exception as exc:  # pragma: no cover
        raise RuntimeError(
            "ROS 2 Python libs not available. Source your ROS 2 + workspace overlay first. "
            "Example: `source /opt/ros/<distro>/setup.bash && source install/setup.bash`. "
            f"Original import error: {exc}"
        )


def wrap_to_pi(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    # ZYX yaw from quaternion.
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass(frozen=True)
class TrialSpec:
    method: str
    bag_path: str


@dataclass
class TimeSeries:
    t: np.ndarray  # seconds (float64)
    v: np.ndarray  # values (float64)


@dataclass
class OdomSeries:
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray


@dataclass
class GoalSeries:
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray


def _is_rosbag_dir(path: str) -> bool:
    return os.path.isdir(path) and os.path.isfile(os.path.join(path, "metadata.yaml"))


def normalize_bag_path(path: str) -> str:
    """Accept bag directory, metadata.yaml, or a storage file and return the bag directory."""
    p = os.path.expanduser(path)
    p = os.path.abspath(p)

    if os.path.isdir(p):
        return p

    # If user points directly to metadata.yaml
    if os.path.isfile(p) and os.path.basename(p) == "metadata.yaml":
        return os.path.dirname(p)

    # If user points to a storage file inside the bag directory
    if os.path.isfile(p):
        return os.path.dirname(p)

    return p


def _read_rosbag_topics(
    bag_path: str,
    topics: Sequence[str],
    storage_id: str = "sqlite3",
) -> Tuple[Dict[str, List[int]], Dict[str, List[object]]]:
    SequentialReader, StorageOptions, ConverterOptions, get_message, deserialize_message = _require_ros2_imports()

    bag_path = normalize_bag_path(bag_path)
    if not _is_rosbag_dir(bag_path):
        raise FileNotFoundError(
            "Not a rosbag2 directory (missing metadata.yaml). "
            "Pass the bag folder (containing metadata.yaml), or the metadata.yaml/.db3 file inside it. "
            f"Got: {bag_path}"
        )

    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id=str(storage_id))
    converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)

    type_map: Dict[str, str] = {}
    for t in reader.get_all_topics_and_types():
        # t.name, t.type
        type_map[t.name] = t.type

    wanted = set(topics)
    missing = [tp for tp in topics if tp not in type_map]
    if missing:
        available = ", ".join(sorted(type_map.keys()))
        raise RuntimeError(
            "Some requested topics are not present in the bag. "
            f"Missing: {missing}. Available: {available}"
        )

    msg_types = {tp: get_message(type_map[tp]) for tp in topics}
    out_times_ns: Dict[str, List[int]] = {tp: [] for tp in topics}
    out_msgs: Dict[str, List[object]] = {tp: [] for tp in topics}

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in wanted:
            continue
        msg = deserialize_message(data, msg_types[topic])
        out_times_ns[topic].append(int(t_ns))
        out_msgs[topic].append(msg)

    return out_times_ns, out_msgs


def list_bag_topics(bag_path: str, storage_id: str = "sqlite3") -> List[Tuple[str, str]]:
    SequentialReader, StorageOptions, ConverterOptions, get_message, deserialize_message = _require_ros2_imports()

    bag_path = normalize_bag_path(bag_path)
    if not _is_rosbag_dir(bag_path):
        raise FileNotFoundError(
            "Not a rosbag2 directory (missing metadata.yaml). "
            f"Got: {bag_path}"
        )

    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id=str(storage_id))
    converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)
    return sorted([(t.name, t.type) for t in reader.get_all_topics_and_types()], key=lambda x: x[0])


def _first_true_time_ns(times_ns: List[int], bool_msgs: List[object]) -> Optional[int]:
    # std_msgs/Bool has field `.data`
    for t_ns, m in zip(times_ns, bool_msgs):
        if getattr(m, "data", False):
            return int(t_ns)
    return None


def _to_seconds(times_ns: Sequence[int], t0_ns: int) -> np.ndarray:
    return (np.asarray(times_ns, dtype=np.float64) - float(t0_ns)) / 1e9


def read_cmd_vel_series(
    bag_path: str,
    cmd_vel_topic: str,
    end_time_ns: Optional[int],
    storage_id: str,
) -> TimeSeries:
    times_ns, msgs = _read_rosbag_topics(bag_path, [cmd_vel_topic], storage_id=storage_id)
    t_ns = times_ns[cmd_vel_topic]
    m = msgs[cmd_vel_topic]
    if not t_ns:
        return TimeSeries(t=np.array([]), v=np.array([]))

    if end_time_ns is not None:
        keep = [i for i, tn in enumerate(t_ns) if tn <= end_time_ns]
        t_ns = [t_ns[i] for i in keep]
        m = [m[i] for i in keep]

    t0_ns = t_ns[0]
    t = _to_seconds(t_ns, t0_ns)
    omega = np.asarray([float(mm.angular.z) for mm in m], dtype=np.float64)
    return TimeSeries(t=t, v=omega)


def read_odom_series(
    bag_path: str,
    odom_topic: str,
    end_time_ns: Optional[int],
    storage_id: str,
) -> OdomSeries:
    times_ns, msgs = _read_rosbag_topics(bag_path, [odom_topic], storage_id=storage_id)
    t_ns = times_ns[odom_topic]
    m = msgs[odom_topic]
    if not t_ns:
        return OdomSeries(t=np.array([]), x=np.array([]), y=np.array([]), yaw=np.array([]))

    if end_time_ns is not None:
        keep = [i for i, tn in enumerate(t_ns) if tn <= end_time_ns]
        t_ns = [t_ns[i] for i in keep]
        m = [m[i] for i in keep]

    t0_ns = t_ns[0]
    t = _to_seconds(t_ns, t0_ns)
    x = np.asarray([float(mm.pose.pose.position.x) for mm in m], dtype=np.float64)
    y = np.asarray([float(mm.pose.pose.position.y) for mm in m], dtype=np.float64)
    yaw = np.asarray(
        [
            float(
                yaw_from_quaternion(
                    mm.pose.pose.orientation.x,
                    mm.pose.pose.orientation.y,
                    mm.pose.pose.orientation.z,
                    mm.pose.pose.orientation.w,
                )
            )
            for mm in m
        ],
        dtype=np.float64,
    )
    return OdomSeries(t=t, x=x, y=y, yaw=yaw)


def read_goal_series(
    bag_path: str,
    goal_topic: str,
    end_time_ns: Optional[int],
    storage_id: str,
) -> GoalSeries:
    times_ns, msgs = _read_rosbag_topics(bag_path, [goal_topic], storage_id=storage_id)
    t_ns = times_ns[goal_topic]
    m = msgs[goal_topic]
    if not t_ns:
        return GoalSeries(t=np.array([]), x=np.array([]), y=np.array([]))

    if end_time_ns is not None:
        keep = [i for i, tn in enumerate(t_ns) if tn <= end_time_ns]
        t_ns = [t_ns[i] for i in keep]
        m = [m[i] for i in keep]

    t0_ns = t_ns[0]
    t = _to_seconds(t_ns, t0_ns)

    # geometry_msgs/PoseArray with a single Pose
    gx: List[float] = []
    gy: List[float] = []
    for mm in m:
        poses = getattr(mm, "poses", [])
        if not poses:
            gx.append(float("nan"))
            gy.append(float("nan"))
            continue
        p0 = poses[0]
        gx.append(float(p0.position.x))
        gy.append(float(p0.position.y))

    return GoalSeries(t=t, x=np.asarray(gx, dtype=np.float64), y=np.asarray(gy, dtype=np.float64))


def find_trial_end_time_ns(
    bag_path: str,
    done_topic: str,
    storage_id: str,
) -> Optional[int]:
    try:
        times_ns, msgs = _read_rosbag_topics(bag_path, [done_topic], storage_id=storage_id)
    except Exception:
        return None
    return _first_true_time_ns(times_ns[done_topic], msgs[done_topic])


def steering_effort(cmd: TimeSeries) -> float:
    if cmd.t.size < 2:
        return 0.0
    return float(np.trapz(np.abs(cmd.v), cmd.t))


def oscillation_index(cmd: TimeSeries, eps: float = 1e-3) -> int:
    if cmd.v.size < 2:
        return 0
    w = np.asarray(cmd.v, dtype=np.float64)
    s = np.sign(w)
    s[np.abs(w) < eps] = 0.0
    s = s[s != 0.0]
    if s.size < 2:
        return 0
    return int(np.sum(s[1:] != s[:-1]))


def align_goal_to_odom(odom: OdomSeries, goals: GoalSeries) -> Tuple[np.ndarray, np.ndarray]:
    """Return (gx(t_odom), gy(t_odom)) piecewise constant from goals."""
    if odom.t.size == 0 or goals.t.size == 0:
        return np.full_like(odom.t, np.nan), np.full_like(odom.t, np.nan)

    gx = np.full_like(odom.t, np.nan)
    gy = np.full_like(odom.t, np.nan)
    j = 0
    current_x = goals.x[0]
    current_y = goals.y[0]
    for i, t in enumerate(odom.t):
        while j + 1 < goals.t.size and goals.t[j + 1] <= t:
            j += 1
            current_x = goals.x[j]
            current_y = goals.y[j]
        gx[i] = current_x
        gy[i] = current_y
    return gx, gy


def compute_progress_series(
    odom: OdomSeries,
    goals: GoalSeries,
) -> Tuple[TimeSeries, TimeSeries]:
    """Return (d_wp(t), abs_heading_error(t)) over odom timestamps."""
    gx, gy = align_goal_to_odom(odom, goals)
    dx = gx - odom.x
    dy = gy - odom.y
    d = np.sqrt(dx * dx + dy * dy)

    bearing = np.arctan2(dy, dx)
    dtheta = np.vectorize(wrap_to_pi)(bearing - odom.yaw)
    return TimeSeries(t=odom.t, v=d), TimeSeries(t=odom.t, v=np.abs(dtheta))


def _ensure_out_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def _safe_stem(s: str) -> str:
    out = []
    for ch in s:
        if ch.isalnum() or ch in ("-", "_", "."):
            out.append(ch)
        else:
            out.append("_")
    stem = "".join(out).strip("_.")
    return stem or "trial"


def _bag_name_for_files(bag_path: str) -> str:
    bag_dir = normalize_bag_path(bag_path)
    return os.path.basename(os.path.normpath(bag_dir))


def _save_metrics_csv(path: str, rows: List[Dict[str, object]]) -> None:
    if not rows:
        return
    fieldnames = list(rows[0].keys())
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)


def export_timeseries_csv(
    out_path: str,
    t: np.ndarray,
    columns: Dict[str, np.ndarray],
) -> None:
    keys = list(columns.keys())
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s"] + keys)
        for i in range(int(t.size)):
            row = [float(t[i])] + [float(columns[k][i]) for k in keys]
            w.writerow(row)


def plot_a_smoothness(
    per_method: Dict[str, Dict[str, List[float]]],
    out_dir: str,
    style: str,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    methods = list(per_method.keys())
    s_vals = [per_method[m]["S_w"] for m in methods]
    z_vals = [per_method[m]["Z_w"] for m in methods]

    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.0), constrained_layout=True)
    ax0, ax1 = axes

    def _plot(ax, data, ylabel):
        if style == "violin":
            parts = ax.violinplot(data, showmeans=True, showextrema=True)
            for pc in parts["bodies"]:
                pc.set_alpha(0.7)
        else:
            ax.boxplot(data, showmeans=True)
        ax.set_xticks(range(1, len(methods) + 1), methods, rotation=20, ha="right")
        ax.set_ylabel(ylabel)
        ax.grid(True, axis="y", alpha=0.3)

    _plot(ax0, s_vals, r"Steering effort $S_\omega=\int |\omega(t)| dt$")
    _plot(ax1, z_vals, r"Yaw oscillation index $Z_\omega$ (# sign changes)")

    fig.suptitle("Smoothness / dithering metrics (10 trials distribution)")
    png = os.path.join(out_dir, "plotA_smoothness.png")
    pdf = os.path.join(out_dir, "plotA_smoothness.pdf")
    fig.savefig(png, dpi=200)
    fig.savefig(pdf)
    plt.close(fig)


def plot_b_progress(
    reps: List[Tuple[str, TimeSeries, TimeSeries]],
    out_dir: str,
    xlabel: str = "t [s]",
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(10.0, 6.0), sharex=True, constrained_layout=True)

    for label, d_wp, abs_dtheta in reps:
        ax0.plot(d_wp.t, d_wp.v, label=label, linewidth=1.6)
        ax1.plot(abs_dtheta.t, abs_dtheta.v, label=label, linewidth=1.2)

    ax0.set_ylabel(r"Distance to current waypoint $d_{wp}(t)$ [m]")
    ax0.grid(True, alpha=0.3)
    ax0.legend(loc="best")

    ax1.set_ylabel(r"Heading error $|\Delta\theta(t)|$ [rad]")
    ax1.set_xlabel(xlabel)
    ax1.grid(True, alpha=0.3)

    fig.suptitle("Progress to waypoint")
    png = os.path.join(out_dir, "plotB_waypoint_progress.png")
    pdf = os.path.join(out_dir, "plotB_waypoint_progress.pdf")
    fig.savefig(png, dpi=200)
    fig.savefig(pdf)
    plt.close(fig)


def parse_trial_arg(s: str) -> TrialSpec:
    if ":" not in s:
        raise argparse.ArgumentTypeError("--trial must be like Method:/path/to/bag")
    method, path = s.split(":", 1)
    method = method.strip()
    path = path.strip()
    if not method:
        raise argparse.ArgumentTypeError("Empty method name in --trial")
    if not path:
        raise argparse.ArgumentTypeError("Empty bag path in --trial")
    return TrialSpec(method=method, bag_path=path)


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", choices=["A", "B", "both"], default="A")
    ap.add_argument("--trial", action="append", type=parse_trial_arg, required=True)
    ap.add_argument("--out", default="plots")

    ap.add_argument(
        "--storage-id",
        default="sqlite3",
        help="rosbag2 storage id (commonly 'sqlite3' or 'mcap').",
    )
    ap.add_argument(
        "--list-topics",
        action="store_true",
        help="List topics/types in the first provided bag and exit.",
    )

    ap.add_argument(
        "--export",
        choices=["none", "metrics", "timeseries", "all"],
        default="metrics",
        help=(
            "CSV export mode. 'metrics' writes per-trial S_w/Z_w; 'timeseries' writes cmd_vel/progress series; "
            "'all' writes both; 'none' disables CSV output."
        ),
    )

    ap.add_argument(
        "--figures",
        choices=["on", "off"],
        default="on",
        help="Generate figure files (png/pdf). Use 'off' for CSV-only extraction.",
    )

    ap.add_argument("--cmd-vel-topic", default="/cmd_vel")
    ap.add_argument("--odom-topic", default="/fixposition/odometry")
    ap.add_argument("--goal-topic", default="/goal")
    ap.add_argument("--done-topic", default="/circuit/done")

    ap.add_argument("--style", choices=["violin", "box"], default="violin")
    ap.add_argument("--osc-eps", type=float, default=1e-3)

    ap.add_argument(
        "--b-trial-index",
        type=int,
        default=0,
        help="Which trial (per method) to use for Plot B when multiple are provided.",
    )

    args = ap.parse_args(argv)
    out_dir = args.out
    _ensure_out_dir(out_dir)

    if args.list_topics:
        first_bag = args.trial[0].bag_path
        for name, typ in list_bag_topics(first_bag, storage_id=str(args.storage_id)):
            print(f"{name}\t{typ}")
        return 0

    # Group trials by method.
    by_method: Dict[str, List[str]] = {}
    for t in args.trial:
        by_method.setdefault(t.method, []).append(t.bag_path)

    metrics_rows: List[Dict[str, object]] = []
    per_method: Dict[str, Dict[str, List[float]]] = {
        m: {"S_w": [], "Z_w": []} for m in by_method.keys()
    }

    if args.plot in ("A", "both"):
        for method, bag_paths in by_method.items():
            for bag_path in bag_paths:
                end_ns = find_trial_end_time_ns(bag_path, args.done_topic, storage_id=str(args.storage_id))
                cmd = read_cmd_vel_series(
                    bag_path, args.cmd_vel_topic, end_ns, storage_id=str(args.storage_id)
                )
                s_w = steering_effort(cmd)
                z_w = oscillation_index(cmd, eps=float(args.osc_eps))
                per_method[method]["S_w"].append(s_w)
                per_method[method]["Z_w"].append(float(z_w))

                if args.export in ("timeseries", "all"):
                    bag_name = _safe_stem(_bag_name_for_files(bag_path))
                    method_name = _safe_stem(method)
                    out_csv = os.path.join(out_dir, f"timeseries_cmd_vel__{method_name}__{bag_name}.csv")
                    export_timeseries_csv(out_csv, cmd.t, {"omega": cmd.v})

                metrics_rows.append(
                    {
                        "method": method,
                        "bag": bag_path,
                        "S_w": s_w,
                        "Z_w": z_w,
                        "end_time_from_done_topic": bool(end_ns is not None),
                    }
                )

        if args.figures == "on":
            plot_a_smoothness(per_method, out_dir=out_dir, style=str(args.style))
        if args.export in ("metrics", "all"):
            _save_metrics_csv(os.path.join(out_dir, "plotA_metrics.csv"), metrics_rows)

    if args.plot in ("B", "both"):
        reps: List[Tuple[str, TimeSeries, TimeSeries]] = []
        for method, bag_paths in by_method.items():
            if not bag_paths:
                continue
            idx = int(args.b_trial_index)
            idx = max(0, min(idx, len(bag_paths) - 1))
            bag_path = bag_paths[idx]

            end_ns = find_trial_end_time_ns(bag_path, args.done_topic, storage_id=str(args.storage_id))
            odom = read_odom_series(
                bag_path, args.odom_topic, end_ns, storage_id=str(args.storage_id)
            )
            goals = read_goal_series(
                bag_path, args.goal_topic, end_ns, storage_id=str(args.storage_id)
            )
            d_wp, abs_dtheta = compute_progress_series(odom, goals)
            reps.append((method, d_wp, abs_dtheta))

            if args.export in ("timeseries", "all"):
                bag_name = _safe_stem(_bag_name_for_files(bag_path))
                method_name = _safe_stem(method)
                out_csv = os.path.join(out_dir, f"timeseries_progress__{method_name}__{bag_name}.csv")
                export_timeseries_csv(out_csv, d_wp.t, {"d_wp": d_wp.v, "abs_dtheta": abs_dtheta.v})

        if reps and args.figures == "on":
            plot_b_progress(reps, out_dir=out_dir)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
