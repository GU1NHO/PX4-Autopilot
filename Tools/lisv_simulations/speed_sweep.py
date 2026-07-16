#!/usr/bin/env python3
"""
Controller speed-sweep test for PX4 Offboard -- run entirely in ONE
continuous flight/Offboard session, no re-arming or re-switching modes
between trials.

Purpose: compare "our controller" (the external position-interpolation
loop, same technique as flight_path_follower.py) across different cruise
speeds under identical conditions, to build an error-vs-speed curve. Run
the same speed values through mission_path_follower.py separately
(manually, one speed at a time) to compare against PX4's native
controller.

Workflow:
  1. Run this script before or after arming.
  2. Arm and take off manually in Position mode, climb to your working
     altitude.
  3. Switch to Offboard mode via QGC or RC switch -- reference pose
     (position + heading) is captured ONCE, for the entire sweep, not
     re-captured per trial.
  4. For each speed in --speeds, the full waypoint list is flown --
     repeated --repeats times -- logging one CSV per trial. Because the
     path in waypoints_sweep.yaml both starts and ends at [0,0,0] relative
     to that single captured origin, every trial begins and ends at the
     same physical spot, so trials stay comparable across the whole sweep.
  5. Land manually after the whole sweep completes.
  6. Use trajectory_plotter.py on each trial's CSV, and compare the
     RMS/max error it reports across speeds.

Difference from flight_path_follower.py: no acceptance_radius gate --
advancing to the next waypoint depends only on the commanded
interpolation reaching its target (dist_to_target < 1e-3), not on the
vehicle's measured error. This makes each trial's duration a pure
function of distance/speed, so tracking error is purely an outcome
metric, not something that also changes how long a trial runs.

Connection strings: see flight_path_follower.py docstring.
"""

import asyncio
import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import yaml
from mavsdk import System
from mavsdk.offboard import PositionNedYaw


SETPOINT_RATE_HZ = 20  # PX4 requires > 2 Hz to stay in Offboard mode
DT = 1.0 / SETPOINT_RATE_HZ


def _distance_3d(a: tuple, b: tuple) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def _to_body(n: float, e: float, n0: float, e0: float, cos_y: float, sin_y: float) -> tuple:
    """Convert absolute NED position to body-frame (fwd, right) relative to origin."""
    rel_n = n - n0
    rel_e = e - e0
    return rel_n * cos_y + rel_e * sin_y, -rel_n * sin_y + rel_e * cos_y


def _transform_waypoints(raw_waypoints: list, origin_ned: tuple, yaw_deg: float) -> list:
    """Identical transform to flight_path_follower.py / mission_path_follower.py."""
    n0, e0, d0 = origin_ned
    yaw = math.radians(yaw_deg)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)

    ned = []
    for wp in raw_waypoints:
        fwd, right, up = float(wp[0]), float(wp[1]), float(wp[2])
        ned.append((
            n0 + fwd * cos_y - right * sin_y,
            e0 + fwd * sin_y + right * cos_y,
            d0 - up,
        ))
    return ned


async def _wait_for_gps(drone: System) -> None:
    print("Waiting for GPS lock...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("GPS lock acquired.")
            return


async def _wait_for_airborne(drone: System) -> None:
    print("Waiting for drone to be armed and airborne...")
    print("  → Arm and take off manually in Position mode.")
    async for is_armed in drone.telemetry.armed():
        if is_armed:
            break

    async for pos in drone.telemetry.position_velocity_ned():
        alt = -pos.position.down_m
        if alt > 0.3:
            print("\nAirborne. Ready.")
            return


async def _wait_for_offboard(drone: System) -> None:
    """Send the current position as setpoint every tick until Offboard mode activates."""
    print("Sending hold setpoints... Switch to Offboard mode via QGC or RC switch.")

    in_offboard = asyncio.Event()
    cur = {"n": 0.0, "e": 0.0, "d": 0.0, "yaw": 0.0}

    async def _watch_mode():
        async for fm in drone.telemetry.flight_mode():
            if "OFFBOARD" in str(fm):
                in_offboard.set()
                return

    async def _watch_pos():
        async for pv in drone.telemetry.position_velocity_ned():
            cur["n"] = pv.position.north_m
            cur["e"] = pv.position.east_m
            cur["d"] = pv.position.down_m

    async def _watch_yaw():
        async for att in drone.telemetry.attitude_euler():
            cur["yaw"] = att.yaw_deg

    mode_task = asyncio.create_task(_watch_mode())
    pos_task  = asyncio.create_task(_watch_pos())
    yaw_task  = asyncio.create_task(_watch_yaw())

    await asyncio.sleep(0.1)

    try:
        while not in_offboard.is_set():
            await drone.offboard.set_position_ned(
                PositionNedYaw(cur["n"], cur["e"], cur["d"], cur["yaw"])
            )
            await asyncio.sleep(DT)
    finally:
        mode_task.cancel()
        pos_task.cancel()
        yaw_task.cancel()


async def _execute_trial(
    drone: System,
    ned_waypoints: list,
    origin_ned: tuple,
    yaw_deg: float,
    speed_ms: float,
    csv_path: Path,
    abort: asyncio.Event,
) -> None:
    """One full pass through ned_waypoints at speed_ms, logging to csv_path.

    No acceptance_radius gate: advances once the commanded interpolation
    reaches each target (dist_to_target < 1e-3), regardless of measured
    tracking error -- by design, so trial duration is purely a function
    of distance/speed and doesn't itself depend on tracking quality.
    """
    n0, e0, _ = origin_ned
    yaw_rad = math.radians(yaw_deg)
    cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)

    actual_pos = {"n": 0.0, "e": 0.0, "d": 0.0}
    gps_info   = {"num_sat": 0, "fix_type": 0}
    stop_telem = asyncio.Event()

    async def _telem():
        async for pv in drone.telemetry.position_velocity_ned():
            actual_pos["n"] = pv.position.north_m
            actual_pos["e"] = pv.position.east_m
            actual_pos["d"] = pv.position.down_m
            if stop_telem.is_set():
                return

    async def _gps_telem():
        async for gi in drone.telemetry.gps_info():
            gps_info["num_sat"]  = gi.num_satellites
            gps_info["fix_type"] = int(gi.fix_type)
            if stop_telem.is_set():
                return

    telem_task     = asyncio.create_task(_telem())
    gps_telem_task = asyncio.create_task(_gps_telem())
    try:
        await asyncio.sleep(0.2)
        interp = [actual_pos["n"], actual_pos["e"], actual_pos["d"]]

        with open(csv_path, "w", newline="") as f:
            f.write(f"# speed_ms: {speed_ms:.3f}\n")
            f.write(f"# origin_ned_m: N={n0:.4f},E={e0:.4f},D={origin_ned[2]:.4f}\n")
            for i, wp in enumerate(ned_waypoints):
                f.write(f"# waypoint_{i + 1:02d}_ned_m: N={wp[0]:.4f},E={wp[1]:.4f},D={wp[2]:.4f}\n")

            writer = csv.writer(f)
            writer.writerow([
                "timestamp_s",
                "desired_N_m", "desired_E_m", "desired_D_m",
                "actual_N_m",  "actual_E_m",  "actual_D_m",
                "error_3d_m",
                "gps_num_satellites", "gps_fix_type",
                "desired_fwd_m", "desired_right_m",
                "actual_fwd_m",  "actual_right_m",
            ])

            for i, target in enumerate(ned_waypoints):
                if abort.is_set():
                    print("  Trial aborted.")
                    return

                print(f"  Waypoint {i + 1}/{len(ned_waypoints)}: "
                      f"N={target[0]:.2f} E={target[1]:.2f} D={target[2]:.2f}")

                while not abort.is_set():
                    diff = [target[j] - interp[j] for j in range(3)]
                    dist_to_target = math.sqrt(sum(d ** 2 for d in diff))
                    step = min(speed_ms * DT, dist_to_target)
                    if dist_to_target > 1e-6:
                        interp = [interp[j] + step * diff[j] / dist_to_target
                                  for j in range(3)]

                    await drone.offboard.set_position_ned(
                        PositionNedYaw(interp[0], interp[1], interp[2], yaw_deg)
                    )

                    an, ae, ad = actual_pos["n"], actual_pos["e"], actual_pos["d"]
                    error = _distance_3d((an, ae, ad), target)

                    d_fwd, d_right = _to_body(target[0], target[1], n0, e0, cos_y, sin_y)
                    a_fwd, a_right = _to_body(an, ae, n0, e0, cos_y, sin_y)
                    writer.writerow([
                        f"{time.time():.3f}",
                        f"{target[0]:.4f}", f"{target[1]:.4f}", f"{target[2]:.4f}",
                        f"{an:.4f}", f"{ae:.4f}", f"{ad:.4f}",
                        f"{error:.4f}",
                        gps_info["num_sat"], gps_info["fix_type"],
                        f"{d_fwd:.4f}", f"{d_right:.4f}",
                        f"{a_fwd:.4f}", f"{a_right:.4f}",
                    ])

                    # No acceptance_radius: advance once the commanded
                    # interpolation itself has reached the target.
                    if dist_to_target < 1e-3:
                        break

                    await asyncio.sleep(DT)

    finally:
        stop_telem.set()
        telem_task.cancel()
        gps_telem_task.cancel()


async def run(
    connection: str,
    waypoints_path: Path,
    speeds: list,
    repeats: int,
    output_dir: Path,
) -> None:
    drone = System()
    print(f"Connecting to {connection} ...")
    await drone.connect(system_address=connection)

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected.")
            break

    await _wait_for_gps(drone)
    await _wait_for_airborne(drone)
    await _wait_for_offboard(drone)

    async for pos in drone.telemetry.position_velocity_ned():
        origin = (pos.position.north_m, pos.position.east_m, pos.position.down_m)
        break
    async for att in drone.telemetry.attitude_euler():
        yaw_deg = att.yaw_deg
        break

    with open(waypoints_path) as f:
        config = yaml.safe_load(f)
    raw_waypoints = config["waypoints"]
    ned_waypoints = _transform_waypoints(raw_waypoints, origin, yaw_deg)

    total_trials = len(speeds) * repeats
    print("Offboard active — reference captured for the WHOLE sweep:")
    print(f"  Position: N={origin[0]:.2f} E={origin[1]:.2f} D={origin[2]:.2f} m")
    print(f"  Heading:  {yaw_deg:.1f}°  (0=North, 90=East)")
    print(f"  Speeds: {speeds}  |  Repeats: {repeats}  |  Total trials: {total_trials}\n")

    abort = asyncio.Event()
    gps_lost = asyncio.Event()
    left_offboard = asyncio.Event()

    async def _gps_monitor():
        async for health in drone.telemetry.health():
            if not health.is_global_position_ok and not gps_lost.is_set():
                gps_lost.set()
                abort.set()
                print("\nWARNING: GPS position lost — aborting sweep.")

    async def _mode_monitor():
        async for fm in drone.telemetry.flight_mode():
            if "OFFBOARD" not in str(fm) and not left_offboard.is_set():
                left_offboard.set()
                abort.set()
                print(f"\nLeft Offboard mode ({fm}) — aborting sweep.")

    gps_task  = asyncio.create_task(_gps_monitor())
    mode_task = asyncio.create_task(_mode_monitor())

    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    try:
        trial_num = 0
        for speed_ms in speeds:
            for rep in range(1, repeats + 1):
                if abort.is_set():
                    break
                trial_num += 1
                csv_path = output_dir / f"sweep_{stamp}_v{speed_ms:.2f}_r{rep}.csv"
                print(f"[Trial {trial_num}/{total_trials}] speed={speed_ms} m/s  rep={rep}")
                await _execute_trial(drone, ned_waypoints, origin, yaw_deg, speed_ms, csv_path, abort)
                print(f"  Saved: {csv_path}\n")
            if abort.is_set():
                break

        if abort.is_set():
            print("Sweep aborted early — data up to the last completed trial is saved.")
        else:
            print("Sweep complete. Holding last position.")
            last_n, last_e, last_d = ned_waypoints[-1]
            hold = PositionNedYaw(last_n, last_e, last_d, yaw_deg)
            while not abort.is_set():
                await drone.offboard.set_position_ned(hold)
                await asyncio.sleep(DT)

        print("Land manually when ready.")

    finally:
        gps_task.cancel()
        mode_task.cancel()


def main():
    parser = argparse.ArgumentParser(description="PX4 Offboard controller speed-sweep test")
    parser.add_argument(
        "--connection", default="udpin://0.0.0.0:14540",
        help="MAVLink connection string (SITL: udpin://0.0.0.0:14540 | QGC forward: udpin://0.0.0.0:14550 | HITL via mavproxy: udpin://0.0.0.0:14552)",
    )
    parser.add_argument(
        "--waypoints", default="waypoints_sweep.yaml",
        help="YAML file with body-relative waypoints [forward, right, up] in metres (should start/end at [0,0,0])",
    )
    parser.add_argument(
        "--speeds", default="0.4,0.8,1.2,1.5",
        help="Comma-separated list of cruise speeds to test, in m/s",
    )
    parser.add_argument(
        "--repeats", type=int, default=1,
        help="Number of repetitions per speed (default: 1)",
    )
    parser.add_argument(
        "--output-dir", default=".", type=Path,
        help="Directory for the CSV logs (default: current directory)",
    )
    args = parser.parse_args()

    wp_path = Path(args.waypoints)
    if not wp_path.exists():
        raise FileNotFoundError(f"Waypoints file not found: {wp_path}")

    speeds = [float(s.strip()) for s in args.speeds.split(",")]

    try:
        asyncio.run(run(
            connection=args.connection,
            waypoints_path=wp_path,
            speeds=speeds,
            repeats=args.repeats,
            output_dir=args.output_dir,
        ))
    except KeyboardInterrupt:
        print("\nInterrupted — CSVs saved up to the last completed trial.")


if __name__ == "__main__":
    main()
