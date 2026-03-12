#!/usr/bin/env python3

"""Generate paper-style plots from previously exported CSV files.

This script complements `rosbag_to_csv.py`:
  1) Export rosbag2 topics to CSV.
  2) Run this script on the CSV folder(s) to compute metrics and plot.

Expected CSV schemas (as produced by scripts/rosbag_to_csv.py):
  - cmd_vel.csv: columns include t_ns, angular_z
  - fixposition_odometry.csv: columns include t_ns, x, y, yaw
  - goal.csv: columns include t_ns, goal_x, goal_y

Plot A (smoothness / dithering) from /cmd_vel:
  - S_w = ∫ |omega(t)| dt
  - Z_w = number of sign changes of omega

Plot B (progress to waypoint) from odometry + goal:
  - d_wp(t) distance to current goal
  - |Δθ(t)| heading error to current goal
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


def wrap_to_pi(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class TrialSpec:
    method: str
    csv_dir: str


@dataclass
class TimeSeries:
    t: np.ndarray
    v: np.ndarray


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


def compute_waypoint_index_from_goal_series(
    goal_x_on_odom: np.ndarray,
    goal_y_on_odom: np.ndarray,
    change_tol_m: float = 1e-3,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return (wp_idx, wp_change) arrays aligned with odom timeline.

    wp_idx starts at 1 and increments when the goal position changes by more
    than change_tol_m.
    """
    n = int(goal_x_on_odom.size)
    wp_idx = np.ones(n, dtype=np.int64)
    wp_change = np.zeros(n, dtype=np.int64)
    if n == 0:
        return wp_idx, wp_change

    cur = 1
    last_x = float(goal_x_on_odom[0])
    last_y = float(goal_y_on_odom[0])
    for i in range(1, n):
        x = float(goal_x_on_odom[i])
        y = float(goal_y_on_odom[i])
        if np.isnan(x) or np.isnan(y) or np.isnan(last_x) or np.isnan(last_y):
            wp_idx[i] = cur
            continue
        if math.hypot(x - last_x, y - last_y) > float(change_tol_m):
            cur += 1
            wp_change[i] = 1
            last_x = x
            last_y = y
        wp_idx[i] = cur
    return wp_idx, wp_change


def _cumulative_trapz_abs(t: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Cumulative integral of |v| over t using trapezoidal rule."""
    if t.size == 0:
        return np.array([], dtype=np.float64)
    out = np.zeros_like(t, dtype=np.float64)
    if t.size < 2:
        return out
    av = np.abs(v.astype(np.float64))
    dt = np.diff(t)
    out[1:] = np.cumsum(0.5 * (av[1:] + av[:-1]) * dt)
    return out


def _cumulative_oscillation_count(v: np.ndarray, eps: float = 1e-3) -> np.ndarray:
    """Cumulative count of sign changes in v (ignoring |v|<eps)."""
    if v.size == 0:
        return np.array([], dtype=np.int64)
    w = v.astype(np.float64)
    s = np.sign(w)
    s[np.abs(w) < eps] = 0.0

    out = np.zeros(v.shape, dtype=np.int64)
    last = 0.0
    count = 0
    for i in range(int(s.size)):
        si = float(s[i])
        if si == 0.0:
            out[i] = count
            continue
        if last != 0.0 and si != last:
            count += 1
        last = si
        out[i] = count
    return out


def _safe_stem(s: str) -> str:
    out = []
    for ch in s:
        if ch.isalnum() or ch in ("-", "_", "."):
            out.append(ch)
        else:
            out.append("_")
    stem = "".join(out).strip("_.")
    return stem or "trial"


def _ensure_out_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def parse_trial_arg(s: str) -> TrialSpec:
    if ":" not in s:
        raise argparse.ArgumentTypeError("--trial must be like Method:/path/to/csv_dir")
    method, path = s.split(":", 1)
    method = method.strip()
    path = os.path.abspath(os.path.expanduser(path.strip()))
    if not method:
        raise argparse.ArgumentTypeError("Empty method name in --trial")
    if not path:
        raise argparse.ArgumentTypeError("Empty csv_dir in --trial")
    if not os.path.isdir(path):
        raise argparse.ArgumentTypeError(f"csv_dir does not exist: {path}")
    return TrialSpec(method=method, csv_dir=path)


def _read_csv_columns(path: str, columns: Sequence[str]) -> Dict[str, np.ndarray]:
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise RuntimeError(f"CSV has no header: {path}")

        missing = [c for c in columns if c not in reader.fieldnames]
        if missing:
            raise RuntimeError(
                f"CSV missing columns {missing} in {path}. Available: {reader.fieldnames}"
            )

        out_lists: Dict[str, List[float]] = {c: [] for c in columns}
        for row in reader:
            for c in columns:
                val = row.get(c, "")
                if val is None or val == "":
                    out_lists[c].append(float("nan"))
                else:
                    out_lists[c].append(float(val))

    return {k: np.asarray(v, dtype=np.float64) for k, v in out_lists.items()}


def _trial_time_zero_ns(*arrays_t_ns: np.ndarray) -> int:
    mins = [int(np.nanmin(a)) for a in arrays_t_ns if a.size > 0]
    if not mins:
        return 0
    return int(min(mins))


def _to_seconds_from_ns(t_ns: np.ndarray, t0_ns: int) -> np.ndarray:
    return (t_ns.astype(np.float64) - float(t0_ns)) / 1e9


def read_cmd_vel(csv_dir: str, filename: str = "cmd_vel.csv") -> Tuple[np.ndarray, np.ndarray]:
    path = os.path.join(csv_dir, filename)
    cols = _read_csv_columns(path, ["t_ns", "angular_z"])
    return cols["t_ns"].astype(np.int64), cols["angular_z"]


def read_odom(csv_dir: str, filename: str = "fixposition_odometry.csv") -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    path = os.path.join(csv_dir, filename)
    cols = _read_csv_columns(path, ["t_ns", "x", "y", "yaw"])
    return (
        cols["t_ns"].astype(np.int64),
        cols["x"],
        cols["y"],
        cols["yaw"],
    )


def read_goal(csv_dir: str, filename: str = "goal.csv") -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    path = os.path.join(csv_dir, filename)
    cols = _read_csv_columns(path, ["t_ns", "goal_x", "goal_y"])
    return (
        cols["t_ns"].astype(np.int64),
        cols["goal_x"],
        cols["goal_y"],
    )


def read_done(csv_dir: str, filename: str = "circuit_done.csv") -> Optional[int]:
    """Return first t_ns where done==True, else None."""
    path = os.path.join(csv_dir, filename)
    if not os.path.isfile(path):
        return None
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None or "t_ns" not in reader.fieldnames or "data" not in reader.fieldnames:
            raise RuntimeError(f"circuit_done.csv missing required columns t_ns,data: {path}")
        for row in reader:
            tn_s = (row.get("t_ns") or "").strip()
            d_s = (row.get("data") or "").strip().lower()
            if not tn_s:
                continue
            tn = int(float(tn_s))
            if d_s in ("true", "1", "yes", "y"):
                return tn
    return None


def _truncate_by_end_ns(t_ns: np.ndarray, end_ns: Optional[int]) -> np.ndarray:
    if end_ns is None or t_ns.size == 0:
        return np.arange(t_ns.size)
    return np.where(t_ns <= int(end_ns))[0]


def steering_effort(t: np.ndarray, omega: np.ndarray) -> float:
    if t.size < 2:
        return 0.0
    return float(np.trapz(np.abs(omega), t))


def oscillation_index(omega: np.ndarray, eps: float = 1e-3) -> int:
    if omega.size < 2:
        return 0
    w = np.asarray(omega, dtype=np.float64)
    s = np.sign(w)
    s[np.abs(w) < eps] = 0.0
    s = s[s != 0.0]
    if s.size < 2:
        return 0
    return int(np.sum(s[1:] != s[:-1]))


def align_goal_piecewise_constant(odom_t: np.ndarray, goal_t: np.ndarray, goal_x: np.ndarray, goal_y: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    if odom_t.size == 0 or goal_t.size == 0:
        return np.full_like(odom_t, np.nan, dtype=np.float64), np.full_like(odom_t, np.nan, dtype=np.float64)

    # Ensure goal series is sorted by time
    order = np.argsort(goal_t)
    gt = goal_t[order]
    gx = goal_x[order]
    gy = goal_y[order]

    out_x = np.full(odom_t.shape, np.nan, dtype=np.float64)
    out_y = np.full(odom_t.shape, np.nan, dtype=np.float64)
    j = 0
    curx = gx[0]
    cury = gy[0]
    for i, t in enumerate(odom_t):
        while j + 1 < gt.size and gt[j + 1] <= t:
            j += 1
            curx = gx[j]
            cury = gy[j]
        out_x[i] = curx
        out_y[i] = cury
    return out_x, out_y


def compute_progress(odom: OdomSeries, goal: GoalSeries) -> Tuple[TimeSeries, TimeSeries]:
    gx, gy = align_goal_piecewise_constant(odom.t, goal.t, goal.x, goal.y)
    dx = gx - odom.x
    dy = gy - odom.y
    d = np.sqrt(dx * dx + dy * dy)
    bearing = np.arctan2(dy, dx)
    dtheta = np.vectorize(wrap_to_pi)(bearing - odom.yaw)
    return TimeSeries(t=odom.t, v=d), TimeSeries(t=odom.t, v=np.abs(dtheta))


def export_single_trial_trace_csv(
    out_path: str,
    cmd_t_s: np.ndarray,
    cmd_omega: np.ndarray,
    odom: OdomSeries,
    goal: GoalSeries,
    osc_eps: float,
    wp_change_tol_m: float,
) -> None:
    """Export a detailed, time-aligned trace (odom timeline as base)."""
    # Base timeline: odom
    t = odom.t

    # Interpolate omega onto odom timeline (cmd_vel is usually higher rate)
    if cmd_t_s.size >= 2:
        order = np.argsort(cmd_t_s)
        ct = cmd_t_s[order]
        cw = cmd_omega[order]
        omega_on_odom = np.interp(t, ct, cw, left=np.nan, right=np.nan)
    elif cmd_t_s.size == 1 and t.size > 0:
        omega_on_odom = np.full_like(t, float(cmd_omega[0]), dtype=np.float64)
    else:
        omega_on_odom = np.full_like(t, np.nan, dtype=np.float64)

    # Align goal on odom timeline (piecewise constant)
    gx, gy = align_goal_piecewise_constant(odom.t, goal.t, goal.x, goal.y)
    wp_idx, wp_change = compute_waypoint_index_from_goal_series(gx, gy, change_tol_m=float(wp_change_tol_m))
    dx = gx - odom.x
    dy = gy - odom.y
    d_wp = np.sqrt(dx * dx + dy * dy)
    bearing = np.arctan2(dy, dx)
    dtheta = np.vectorize(wrap_to_pi)(bearing - odom.yaw)
    abs_dtheta = np.abs(dtheta)

    cum_Sw = _cumulative_trapz_abs(t, omega_on_odom)
    cum_Zw = _cumulative_oscillation_count(omega_on_odom, eps=float(osc_eps))

    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "t_s",
                "x",
                "y",
                "yaw",
                "goal_x",
                "goal_y",
                "wp_idx",
                "wp_change",
                "d_wp",
                "abs_dtheta",
                "omega",
                "cum_Sw",
                "cum_Zw",
            ]
        )
        for i in range(int(t.size)):
            w.writerow(
                [
                    float(t[i]),
                    float(odom.x[i]),
                    float(odom.y[i]),
                    float(odom.yaw[i]),
                    float(gx[i]),
                    float(gy[i]),
                    int(wp_idx[i]),
                    int(wp_change[i]),
                    float(d_wp[i]),
                    float(abs_dtheta[i]),
                    float(omega_on_odom[i]) if not np.isnan(omega_on_odom[i]) else "",
                    float(cum_Sw[i]),
                    int(cum_Zw[i]),
                ]
            )


def plot_single_trial_progress_with_waypoints(
    label: str,
    odom: OdomSeries,
    goal: GoalSeries,
    cmd_t_s: np.ndarray,
    cmd_omega: np.ndarray,
    out_dir: str,
    osc_eps: float,
    wp_change_tol_m: float,
    label_every: int = 1,
    bottom: str = "omega",
) -> None:
    """Single-trial Plot B with waypoint transition markers."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    gx, gy = align_goal_piecewise_constant(odom.t, goal.t, goal.x, goal.y)
    wp_idx, wp_change = compute_waypoint_index_from_goal_series(gx, gy, change_tol_m=float(wp_change_tol_m))

    dx = gx - odom.x
    dy = gy - odom.y
    d_wp = np.sqrt(dx * dx + dy * dy)
    bearing = np.arctan2(dy, dx)
    dtheta = np.vectorize(wrap_to_pi)(bearing - odom.yaw)
    abs_dtheta = np.abs(dtheta)

    # interpolate omega
    if cmd_t_s.size >= 2:
        order = np.argsort(cmd_t_s)
        ct = cmd_t_s[order]
        cw = cmd_omega[order]
        omega_on_odom = np.interp(odom.t, ct, cw, left=np.nan, right=np.nan)
    elif cmd_t_s.size == 1 and odom.t.size > 0:
        omega_on_odom = np.full_like(odom.t, float(cmd_omega[0]), dtype=np.float64)
    else:
        omega_on_odom = np.full_like(odom.t, np.nan, dtype=np.float64)

    fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(10.0, 6.0), sharex=True, constrained_layout=True)
    ax0.plot(odom.t, d_wp, label=label, linewidth=1.6)

    if bottom == "abs_dtheta":
        ax1.plot(odom.t, abs_dtheta, label=label, linewidth=1.2)
        ax1.set_ylabel(r"Heading error $|\Delta\theta(t)|$ [rad]")
    else:
        ax1.plot(odom.t, omega_on_odom, label=label, linewidth=1.2)
        ax1.set_ylabel(r"Yaw rate $\omega(t)$ [rad/s]")

    ax0.set_ylabel(r"Distance to waypoint $d_{wp}(t)$ [m]")
    ax0.grid(True, alpha=0.3)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlabel("t [s]")

    # Waypoint transition markers
    change_idx = np.where(wp_change == 1)[0]
    for n, i in enumerate(change_idx.tolist()):
        t_change = float(odom.t[i])
        ax0.axvline(t_change, color="k", alpha=0.15, linewidth=1.0)
        ax1.axvline(t_change, color="k", alpha=0.15, linewidth=1.0)

        if label_every > 0 and (n % int(label_every) == 0):
            prev_idx = int(wp_idx[i] - 1)
            cur_idx = int(wp_idx[i])
            txt = f"{prev_idx}→{cur_idx}"
            ax0.text(t_change, ax0.get_ylim()[1], txt, rotation=90, va="top", ha="right", fontsize=8, alpha=0.7)

    fig.suptitle("Progress to waypoint (annotated)")
    out_png = os.path.join(out_dir, f"plotB_single_trial_annotated__{_safe_stem(label)}.png")
    out_pdf = os.path.join(out_dir, f"plotB_single_trial_annotated__{_safe_stem(label)}.pdf")
    fig.savefig(out_png, dpi=200)
    fig.savefig(out_pdf)
    plt.close(fig)


def plot_a(per_method: Dict[str, Dict[str, List[float]]], out_dir: str, style: str) -> None:
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

    fig.suptitle("Smoothness / dithering metrics")
    fig.savefig(os.path.join(out_dir, "plotA_smoothness_from_csv.png"), dpi=200)
    fig.savefig(os.path.join(out_dir, "plotA_smoothness_from_csv.pdf"))
    plt.close(fig)


def plot_b(reps: List[Tuple[str, TimeSeries, TimeSeries]], out_dir: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(10.0, 6.0), sharex=True, constrained_layout=True)

    for label, d_wp, abs_dtheta in reps:
        ax0.plot(d_wp.t, d_wp.v, label=label, linewidth=1.6)
        ax1.plot(abs_dtheta.t, abs_dtheta.v, label=label, linewidth=1.2)

    ax0.set_ylabel(r"Distance to waypoint $d_{wp}(t)$ [m]")
    ax0.grid(True, alpha=0.3)
    ax0.legend(loc="best")

    ax1.set_ylabel(r"Heading error $|\Delta\theta(t)|$ [rad]")
    ax1.set_xlabel("t [s]")
    ax1.grid(True, alpha=0.3)

    fig.suptitle("Progress to waypoint")
    fig.savefig(os.path.join(out_dir, "plotB_progress_from_csv.png"), dpi=200)
    fig.savefig(os.path.join(out_dir, "plotB_progress_from_csv.pdf"))
    plt.close(fig)


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", choices=["A", "B", "both"], default="both")
    ap.add_argument("--trial", action="append", type=parse_trial_arg, required=True)
    ap.add_argument("--out", default="plots_from_csv")

    ap.add_argument("--style", choices=["violin", "box"], default="violin")
    ap.add_argument("--osc-eps", type=float, default=1e-3)
    ap.add_argument("--figures", choices=["on", "off"], default="on")
    ap.add_argument("--b-trial-index", type=int, default=0)
    ap.add_argument(
        "--truncate-on-done",
        choices=["on", "off"],
        default="on",
        help="If circuit_done.csv exists, truncate data at first done=True.",
    )

    ap.add_argument(
        "--export-trace",
        choices=["on", "off"],
        default="off",
        help="Export a time-aligned per-trial trace CSV (single-trial analysis).",
    )

    ap.add_argument(
        "--wp-change-tol-m",
        type=float,
        default=1e-3,
        help="Tolerance (meters) to detect goal/waypoint changes from goal.csv.",
    )
    ap.add_argument(
        "--annotate-waypoints",
        choices=["on", "off"],
        default="off",
        help="For single-trial Plot B, draw vertical lines + labels at waypoint changes.",
    )
    ap.add_argument(
        "--wp-label-every",
        type=int,
        default=1,
        help="Label every N-th waypoint transition (avoid clutter).",
    )
    ap.add_argument(
        "--plotb-bottom",
        choices=["omega", "abs_dtheta"],
        default="omega",
        help="What to draw in Plot B bottom subplot for single-trial annotated plot.",
    )

    args = ap.parse_args(argv)
    out_dir = os.path.abspath(os.path.expanduser(args.out))
    _ensure_out_dir(out_dir)

    # Group trials by method.
    by_method: Dict[str, List[str]] = {}
    for t in args.trial:
        by_method.setdefault(t.method, []).append(t.csv_dir)

    # Optional: export per-trial trace (one file per trial)
    if args.export_trace == "on":
        for method, csv_dirs in by_method.items():
            for csv_dir in csv_dirs:
                # Read raw series
                cmd_t_ns, omega = read_cmd_vel(csv_dir)
                odom_t_ns, x, y, yaw = read_odom(csv_dir)
                goal_t_ns, gx, gy = read_goal(csv_dir)

                end_ns = read_done(csv_dir) if args.truncate_on_done == "on" else None
                keep_c = _truncate_by_end_ns(cmd_t_ns, end_ns)
                keep_o = _truncate_by_end_ns(odom_t_ns, end_ns)
                keep_g = _truncate_by_end_ns(goal_t_ns, end_ns)

                cmd_t_ns_k = cmd_t_ns[keep_c]
                omega_k = omega[keep_c]

                odom_t_ns_k = odom_t_ns[keep_o]
                x_k = x[keep_o]
                y_k = y[keep_o]
                yaw_k = yaw[keep_o]

                goal_t_ns_k = goal_t_ns[keep_g]
                gx_k = gx[keep_g]
                gy_k = gy[keep_g]

                t0 = _trial_time_zero_ns(cmd_t_ns_k, odom_t_ns_k, goal_t_ns_k)
                cmd_t_s = _to_seconds_from_ns(cmd_t_ns_k, t0)
                odom = OdomSeries(
                    t=_to_seconds_from_ns(odom_t_ns_k, t0),
                    x=x_k,
                    y=y_k,
                    yaw=yaw_k,
                )
                goal = GoalSeries(
                    t=_to_seconds_from_ns(goal_t_ns_k, t0),
                    x=gx_k,
                    y=gy_k,
                )

                method_name = _safe_stem(method)
                trial_name = _safe_stem(os.path.basename(os.path.normpath(csv_dir)))
                out_trace = os.path.join(out_dir, f"trial_trace__{method_name}__{trial_name}.csv")
                export_single_trial_trace_csv(
                    out_trace,
                    cmd_t_s=cmd_t_s,
                    cmd_omega=omega_k,
                    odom=odom,
                    goal=goal,
                    osc_eps=float(args.osc_eps),
                    wp_change_tol_m=float(args.wp_change_tol_m),
                )

                if args.annotate_waypoints == "on" and args.figures == "on":
                    plot_single_trial_progress_with_waypoints(
                        label=f"{method_name}__{trial_name}",
                        odom=odom,
                        goal=goal,
                        cmd_t_s=cmd_t_s,
                        cmd_omega=omega_k,
                        out_dir=out_dir,
                        osc_eps=float(args.osc_eps),
                        wp_change_tol_m=float(args.wp_change_tol_m),
                        label_every=int(args.wp_label_every),
                        bottom=str(args.plotb_bottom),
                    )

    per_method: Dict[str, Dict[str, List[float]]] = {
        m: {"S_w": [], "Z_w": []} for m in by_method.keys()
    }
    metrics_rows: List[Dict[str, object]] = []

    # Plot A metrics from cmd_vel
    if args.plot in ("A", "both"):
        for method, csv_dirs in by_method.items():
            for csv_dir in csv_dirs:
                t_ns, omega = read_cmd_vel(csv_dir)
                end_ns = read_done(csv_dir) if args.truncate_on_done == "on" else None
                keep = _truncate_by_end_ns(t_ns, end_ns)
                t_ns_k = t_ns[keep]
                omega_k = omega[keep]

                t0 = _trial_time_zero_ns(t_ns_k)
                t = _to_seconds_from_ns(t_ns_k, t0)
                s_w = steering_effort(t, omega_k)
                z_w = oscillation_index(omega_k, eps=float(args.osc_eps))
                per_method[method]["S_w"].append(s_w)
                per_method[method]["Z_w"].append(float(z_w))
                metrics_rows.append(
                    {
                        "method": method,
                        "csv_dir": csv_dir,
                        "S_w": s_w,
                        "Z_w": z_w,
                        "truncated_on_done": bool(end_ns is not None),
                    }
                )

        # Save metrics CSV
        metrics_path = os.path.join(out_dir, "plotA_metrics_from_csv.csv")
        with open(metrics_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(metrics_rows[0].keys()) if metrics_rows else ["method", "csv_dir", "S_w", "Z_w"])
            w.writeheader()
            w.writerows(metrics_rows)

        if args.figures == "on":
            plot_a(per_method, out_dir=out_dir, style=str(args.style))

    # Plot B progress from odom + goal
    if args.plot in ("B", "both"):
        reps: List[Tuple[str, TimeSeries, TimeSeries]] = []
        for method, csv_dirs in by_method.items():
            if not csv_dirs:
                continue
            idx = max(0, min(int(args.b_trial_index), len(csv_dirs) - 1))
            csv_dir = csv_dirs[idx]

            odom_t_ns, x, y, yaw = read_odom(csv_dir)
            goal_t_ns, gx, gy = read_goal(csv_dir)

            end_ns = read_done(csv_dir) if args.truncate_on_done == "on" else None
            keep_o = _truncate_by_end_ns(odom_t_ns, end_ns)
            keep_g = _truncate_by_end_ns(goal_t_ns, end_ns)

            odom_t_ns_k = odom_t_ns[keep_o]
            x_k = x[keep_o]
            y_k = y[keep_o]
            yaw_k = yaw[keep_o]

            goal_t_ns_k = goal_t_ns[keep_g]
            gx_k = gx[keep_g]
            gy_k = gy[keep_g]

            t0 = _trial_time_zero_ns(odom_t_ns_k, goal_t_ns_k)

            odom = OdomSeries(
                t=_to_seconds_from_ns(odom_t_ns_k, t0),
                x=x_k,
                y=y_k,
                yaw=yaw_k,
            )
            goal = GoalSeries(
                t=_to_seconds_from_ns(goal_t_ns_k, t0),
                x=gx_k,
                y=gy_k,
            )
            d_wp, abs_dtheta = compute_progress(odom, goal)
            reps.append((method, d_wp, abs_dtheta))

        if reps and args.figures == "on":
            plot_b(reps, out_dir=out_dir)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
