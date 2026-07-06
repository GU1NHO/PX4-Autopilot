#!/usr/bin/env python3
"""
Post-flight trajectory analyser.

Usage:
    python3 trajectory_plotter.py flight_20240617_143000.csv

Outputs:
  - 3D trajectory plot (desired vs actual)
  - 2D top-down plot (N x E plane)
  - Tracking error over time
  - Console summary: max error, RMS, per-segment mean error
"""

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

_GPS_FIX_LABELS = {0: "NO_GPS", 1: "NO_FIX", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK_F", 6: "RTK"}


def _load_csv(path: Path) -> list:
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: float(v) for k, v in row.items()})
    return rows


def _distance_to_segment(a: np.ndarray, b: np.ndarray, p: np.ndarray) -> float:
    """Perpendicular distance from point p to segment [a, b].
    Adapted from flight_path_assertion.py."""
    v = b - a
    w = p - a
    c1 = np.dot(w, v)
    if c1 <= 0:
        return float(np.linalg.norm(p - a))
    c2 = np.dot(v, v)
    if c2 <= c1:
        return float(np.linalg.norm(p - b))
    proj = a + (c1 / c2) * v
    return float(np.linalg.norm(p - proj))


def _compute_perpendicular_errors(rows: list) -> list:
    """For each log row compute perpendicular distance to the current desired segment."""
    if not rows:
        return []

    errors = []
    waypoints = []
    prev_desired = None

    for row in rows:
        desired = np.array([row["desired_N_m"], row["desired_E_m"], row["desired_D_m"]])
        actual  = np.array([row["actual_N_m"],  row["actual_E_m"],  row["actual_D_m"]])

        if prev_desired is not None and not np.allclose(desired, prev_desired):
            waypoints.append(prev_desired.copy())
        prev_desired = desired

        if len(waypoints) == 0:
            errors.append(float(np.linalg.norm(actual - desired)))
        else:
            seg_a = waypoints[-1]
            seg_b = desired
            errors.append(_distance_to_segment(seg_a, seg_b, actual))

    return errors


def _build_waypoint_sequence(rows: list) -> list:
    """Extract unique sequential desired positions (waypoints)."""
    waypoints = []
    prev = None
    for row in rows:
        d = (row["desired_N_m"], row["desired_E_m"], row["desired_D_m"])
        if d != prev:
            waypoints.append(d)
            prev = d
    return waypoints


def _segment_stats(rows: list, perp_errors: list) -> list:
    """Return per-waypoint-segment mean perpendicular error."""
    waypoints = _build_waypoint_sequence(rows)
    if len(waypoints) < 2:
        return []

    segments = []
    seg_idx = 0
    seg_errors = []
    prev_desired = None

    for row, err in zip(rows, perp_errors):
        desired = (row["desired_N_m"], row["desired_E_m"], row["desired_D_m"])
        if prev_desired is not None and desired != prev_desired:
            if seg_errors:
                segments.append({
                    "from": prev_desired,
                    "to": desired,
                    "mean_err": float(np.mean(seg_errors)),
                    "max_err": float(np.max(seg_errors)),
                })
            seg_errors = []
            seg_idx += 1
        seg_errors.append(err)
        prev_desired = desired

    if seg_errors:
        segments.append({
            "from": prev_desired,
            "to": prev_desired,
            "mean_err": float(np.mean(seg_errors)),
            "max_err": float(np.max(seg_errors)),
        })

    return segments


def plot_and_analyse(csv_path: Path) -> None:
    rows = _load_csv(csv_path)
    if not rows:
        print("No data found in CSV.")
        sys.exit(1)

    t  = np.array([r["timestamp_s"] for r in rows]) - rows[0]["timestamp_s"]
    dn = np.array([r["desired_N_m"] for r in rows])
    de = np.array([r["desired_E_m"] for r in rows])
    dd = np.array([r["desired_D_m"] for r in rows])
    an = np.array([r["actual_N_m"]  for r in rows])
    ae = np.array([r["actual_E_m"]  for r in rows])
    ad = np.array([r["actual_D_m"]  for r in rows])

    perp_errors = np.array(_compute_perpendicular_errors(rows))
    e3d = np.array([r["error_3d_m"] for r in rows])

    # ---- Console summary ----
    print(f"\n=== Trajectory Analysis: {csv_path.name} ===")
    print(f"  Duration:       {t[-1]:.1f} s  ({len(rows)} samples)")
    print(f"  Max 3D error:   {e3d.max():.3f} m")
    print(f"  RMS 3D error:   {math.sqrt(np.mean(e3d**2)):.3f} m")
    print(f"  Mean perp error:{perp_errors.mean():.3f} m")
    print(f"  Max  perp error:{perp_errors.max():.3f} m")
    if rows and "gps_num_satellites" in rows[0]:
        gps_sat_arr = np.array([r["gps_num_satellites"] for r in rows])
        print(f"  GPS satellites: min={int(gps_sat_arr.min())}  mean={gps_sat_arr.mean():.1f}  max={int(gps_sat_arr.max())}")

    segments = _segment_stats(rows, list(perp_errors))
    if segments:
        print("\n  Per-segment perpendicular error:")
        for i, seg in enumerate(segments):
            print(f"    Seg {i+1}: mean={seg['mean_err']:.3f}m  max={seg['max_err']:.3f}m")

    # ---- Plots ----
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(f"Flight Path Analysis — {csv_path.name}", fontsize=13)

    # GPS signal quality (top-left)
    ax_gps = fig.add_subplot(2, 2, 1)
    has_gps_cols = rows and "gps_num_satellites" in rows[0]
    if has_gps_cols:
        gps_sat = np.array([r["gps_num_satellites"] for r in rows])
        gps_fix = np.array([r["gps_fix_type"] for r in rows])
        ax_gps.plot(t, gps_sat, color="steelblue", linewidth=1.2, label="Satellites")
        ax_gps.set_ylabel("Num Satellites", color="steelblue")
        ax_gps.tick_params(axis="y", labelcolor="steelblue")
        ax_gps.set_xlabel("Time (s)")
        ax_gps.set_title("GPS Signal Quality")
        ax_gps.grid(True)
        ax_gps2 = ax_gps.twinx()
        ax_gps2.step(t, gps_fix, color="darkorange", linewidth=1.0, where="post", label="Fix type", alpha=0.7)
        ax_gps2.set_yticks(list(_GPS_FIX_LABELS.keys()))
        ax_gps2.set_yticklabels(list(_GPS_FIX_LABELS.values()), fontsize=7)
        ax_gps2.set_ylabel("Fix Type", color="darkorange")
        ax_gps2.tick_params(axis="y", labelcolor="darkorange")
        lines1, labels1 = ax_gps.get_legend_handles_labels()
        lines2, labels2 = ax_gps2.get_legend_handles_labels()
        ax_gps.legend(lines1 + lines2, labels1 + labels2, fontsize=8)
    else:
        ax_gps.text(0.5, 0.5, "GPS data not in CSV\n(re-run with updated script)",
                    ha="center", va="center", transform=ax_gps.transAxes,
                    fontsize=10, color="gray")
        ax_gps.set_title("GPS Signal Quality")
        ax_gps.axis("off")

    # Top-down (N x E)
    ax2d = fig.add_subplot(2, 2, 2)
    ax2d.plot(de, dn, "b--", linewidth=1.5, label="Desired")
    ax2d.plot(ae, an, "r-",  linewidth=1.5, label="Actual")
    ax2d.set_xlabel("East (m)")
    ax2d.set_ylabel("North (m)")
    ax2d.set_title("Top-down (N × E)")
    ax2d.legend()
    ax2d.set_aspect("equal")
    ax2d.grid(True)

    # Altitude over time
    ax_alt = fig.add_subplot(2, 2, 3)
    ax_alt.plot(t, -dd, "b--", linewidth=1.2, label="Desired")
    ax_alt.plot(t, -ad, "r-",  linewidth=1.2, label="Actual")
    ax_alt.set_xlabel("Time (s)")
    ax_alt.set_ylabel("Altitude (m)")
    ax_alt.set_title("Altitude over Time")
    ax_alt.legend()
    ax_alt.grid(True)

    # Tracking error over time
    ax_err = fig.add_subplot(2, 2, 4)
    ax_err.plot(t, e3d,         color="orange", linewidth=1.2, label="3D error")
    ax_err.plot(t, perp_errors, color="purple", linewidth=1.2, label="Perp. error")
    ax_err.axhline(e3d.mean(),         color="orange", linestyle=":", linewidth=1)
    ax_err.axhline(perp_errors.mean(), color="purple", linestyle=":", linewidth=1)
    ax_err.set_xlabel("Time (s)")
    ax_err.set_ylabel("Error (m)")
    ax_err.set_title("Tracking Error over Time")
    ax_err.legend()
    ax_err.grid(True)

    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Trajectory analysis and plotting")
    parser.add_argument("csv_file", help="CSV log file from flight_path_follower.py")
    args = parser.parse_args()

    csv_path = Path(args.csv_file)
    if not csv_path.exists():
        print(f"File not found: {csv_path}")
        sys.exit(1)

    plot_and_analyse(csv_path)


if __name__ == "__main__":
    main()
