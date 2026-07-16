#!/usr/bin/env python3
"""
Companion computer script for PX4 autonomous flight path following using
Mission mode instead of Offboard.

Key difference from flight_path_follower.py: instead of streaming position
setpoints continuously (which requires an unbroken >2Hz link and drops out
of Offboard on any hiccup), this uploads the whole path as a PX4 mission
once. PX4 then flies it autonomously from the FMU -- the mission keeps
running even if the companion computer or telemetry link drops afterwards.
Trade-off: the path is fixed at upload time, no per-tick reactive control
like Offboard offers.

Workflow:
  1. Run this script before or after arming.
  2. Arm and take off manually in Position mode, climb to your working
     altitude and hold roughly still -- this is the pose the path is
     captured relative to.
  3. Script captures position + heading, converts the body-relative
     waypoints ([forward_m, right_m, up_m]) to a lat/lon mission, and
     uploads it to the vehicle. Avoid drifting far from that pose between
     "Airborne. Ready." and switching modes -- the mission is fixed at
     upload time, unlike Offboard's continuously-updated hold setpoint.
  4. Switch to Mission mode via QGC or RC switch -- PX4 starts flying the
     uploaded waypoints immediately, no further script involvement needed
     to keep it going.
  5. Script logs desired (current mission leg target) vs actual position
     while the mission runs.
  6. Mission holds at the last waypoint when finished (RTL-after-mission
     is disabled) -- land manually.
  7. Use trajectory_plotter.py to analyse the saved CSV, same as before.

Waypoint format in YAML (identical to flight_path_follower.py):
  [forward_m, right_m, up_m]  body-relative offsets at capture time.

Connection strings:
  SITL (MAVSDK default):  udpin://0.0.0.0:14540
  QGC forward (real hw):  udpin://0.0.0.0:14550
  HITL via mavproxy:      udpin://0.0.0.0:14552
  Serial (direct):        serial:///dev/ttyUSB0:57600
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
from mavsdk.mission import MissionItem, MissionPlan, MissionError


_EARTH_RADIUS_M = 6378137.0  # WGS84 semi-major axis; fine for local-scale paths


def _distance_3d(a: tuple, b: tuple) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def _to_body(n: float, e: float, n0: float, e0: float, cos_y: float, sin_y: float) -> tuple:
    """Convert absolute NED position to body-frame (fwd, right) relative to origin."""
    rel_n = n - n0
    rel_e = e - e0
    return rel_n * cos_y + rel_e * sin_y, -rel_n * sin_y + rel_e * cos_y


def _transform_waypoints(
    raw_waypoints: list,
    origin_ned: tuple,
    yaw_deg: float,
) -> list:
    """Convert body-relative [forward, right, up] offsets to absolute NED.
    Identical transform to flight_path_follower.py, so both scripts treat
    the waypoints YAML the same way."""
    n0, e0, d0 = origin_ned
    yaw = math.radians(yaw_deg)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)

    ned = []
    for wp in raw_waypoints:
        fwd, right, up = float(wp[0]), float(wp[1]), float(wp[2])
        ned.append((
            n0 + fwd * cos_y - right * sin_y,
            e0 + fwd * sin_y + right * cos_y,
            d0 - up,                            # up → subtract from D
        ))
    return ned


def _ned_offset_to_global(
    origin_lat_deg: float,
    origin_lon_deg: float,
    north_m: float,
    east_m: float,
) -> tuple:
    """Flat-earth local projection -- accurate enough for paths of a few
    hundred metres, which is what this project targets. PX4 missions need
    absolute lat/lon, unlike Offboard's local NED setpoints."""
    lat = origin_lat_deg + (north_m / _EARTH_RADIUS_M) * (180.0 / math.pi)
    lon = origin_lon_deg + (east_m / (_EARTH_RADIUS_M * math.cos(math.radians(origin_lat_deg)))) * (180.0 / math.pi)
    return lat, lon


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


async def _capture_origin(drone: System) -> tuple:
    """Capture NED origin, global lat/lon, and heading -- read once each,
    as close together as the telemetry streams allow."""
    async for pos in drone.telemetry.position_velocity_ned():
        ned_origin = (pos.position.north_m, pos.position.east_m, pos.position.down_m)
        break
    async for gpos in drone.telemetry.position():
        origin_lat, origin_lon = gpos.latitude_deg, gpos.longitude_deg
        break
    async for att in drone.telemetry.attitude_euler():
        yaw_deg = att.yaw_deg
        break
    return ned_origin, origin_lat, origin_lon, yaw_deg


def _build_mission(
    ned_waypoints: list,
    origin_lat: float,
    origin_lon: float,
    yaw_deg: float,
    speed_ms: float,
    acceptance_radius: float,
) -> MissionPlan:
    items = []
    for n, e, d in ned_waypoints:
        lat, lon = _ned_offset_to_global(origin_lat, origin_lon, n, e)
        items.append(MissionItem(
            lat, lon,
            relative_altitude_m=-d,              # D negative-up → altitude positive-up
            speed_m_s=speed_ms,
            is_fly_through=False,
            gimbal_pitch_deg=0.0,
            gimbal_yaw_deg=0.0,
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=0.0,
            camera_photo_interval_s=0.0,
            acceptance_radius_m=acceptance_radius,
            yaw_deg=yaw_deg,                     # fixed heading, same as flight_path_follower.py
            camera_photo_distance_m=0.0,
            vehicle_action=MissionItem.VehicleAction.NONE,
        ))
    return MissionPlan(items)


async def _wait_for_mission_mode(drone: System) -> None:
    print("Mission uploaded. Switch to Mission mode via QGC or RC switch to start.")
    async for fm in drone.telemetry.flight_mode():
        if "MISSION" in str(fm):
            print("Mission mode active — PX4 is flying the uploaded path.\n")
            return


async def _log_mission(
    drone: System,
    ned_waypoints: list,
    ned_origin: tuple,
    yaw_deg: float,
    csv_path: Path,
    abort: asyncio.Event,
) -> None:
    """Log desired (current mission leg target) vs actual position while
    PX4 flies the mission on its own. Aborts early if abort is set (GPS
    failure or mode change). CSV columns match flight_path_follower.py's
    so trajectory_plotter.py works unchanged on either script's output."""
    n0, e0, _ = ned_origin
    yaw_rad = math.radians(yaw_deg)
    cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)

    actual_pos = {"n": 0.0, "e": 0.0, "d": 0.0}
    gps_info   = {"num_sat": 0, "fix_type": 0}
    progress   = {"current": 0, "total": len(ned_waypoints)}
    finished   = asyncio.Event()
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

    async def _progress_telem():
        async for mp in drone.mission.mission_progress():
            progress["current"] = mp.current
            progress["total"] = mp.total
            if mp.current >= mp.total:
                finished.set()
                return
            if stop_telem.is_set():
                return

    telem_task    = asyncio.create_task(_telem())
    gps_telem_task = asyncio.create_task(_gps_telem())
    progress_task = asyncio.create_task(_progress_telem())

    try:
        await asyncio.sleep(0.2)
        print(f"Logging to {csv_path}\n")

        with open(csv_path, "w", newline="") as f:
            f.write(f"# origin_ned_m: N={n0:.4f},E={e0:.4f},D={ned_origin[2]:.4f}\n")
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

            while not abort.is_set() and not finished.is_set():
                idx = min(progress["current"], len(ned_waypoints) - 1)
                target = ned_waypoints[idx]
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

                await asyncio.sleep(0.05)  # 20 Hz, matches flight_path_follower.py

            if finished.is_set():
                print("Mission finished — holding last waypoint.")
            elif abort.is_set():
                print("Mission logging aborted.")

    finally:
        stop_telem.set()
        telem_task.cancel()
        gps_telem_task.cancel()
        progress_task.cancel()


async def run(
    connection: str,
    waypoints: list,
    acceptance_radius: float,
    speed_ms: float,
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

    ned_origin, origin_lat, origin_lon, yaw_deg = await _capture_origin(drone)
    ned_waypoints = _transform_waypoints(waypoints, ned_origin, yaw_deg)

    print("Reference captured:")
    print(f"  Position: N={ned_origin[0]:.2f} E={ned_origin[1]:.2f} D={ned_origin[2]:.2f} m")
    print(f"  Lat/Lon:  {origin_lat:.7f}, {origin_lon:.7f}")
    print(f"  Heading:  {yaw_deg:.1f}°  (0=North, 90=East)")
    print("  Waypoints transformed to NED:")
    for i, wp in enumerate(ned_waypoints):
        print(f"    {i+1}: N={wp[0]:.2f} E={wp[1]:.2f} D={wp[2]:.2f}")
    print()

    mission_plan = _build_mission(
        ned_waypoints, origin_lat, origin_lon, yaw_deg, speed_ms, acceptance_radius
    )

    # A mission item's speed_m_s only takes effect for legs AFTER that item,
    # not the leg leading up to it -- so the first leg would otherwise fly
    # at PX4's default cruise speed instead of speed_ms. Set the cruise speed
    # itself so the first leg already matches; restored once done.
    default_cruise_speed = await drone.param.get_param_float("MPC_XY_CRUISE")
    await drone.param.set_param_float("MPC_XY_CRUISE", speed_ms)

    print("Uploading mission...")
    try:
        await drone.mission.upload_mission(mission_plan)
        await drone.mission.set_return_to_launch_after_mission(False)
    except MissionError as error:
        print(f"Mission upload failed: {error}")
        await drone.param.set_param_float("MPC_XY_CRUISE", default_cruise_speed)
        return

    await _wait_for_mission_mode(drone)

    # --- Shared abort event (GPS failure OR mode change) ---
    abort = asyncio.Event()
    gps_lost = asyncio.Event()
    left_mission = asyncio.Event()

    async def _gps_monitor():
        async for health in drone.telemetry.health():
            if not health.is_global_position_ok and not gps_lost.is_set():
                gps_lost.set()
                abort.set()
                print("\nWARNING: GPS position lost — stopping mission logging.")
                print("         PX4 failsafe will activate per its own configured behaviour.")

    async def _mode_monitor():
        async for fm in drone.telemetry.flight_mode():
            if "MISSION" not in str(fm) and not left_mission.is_set():
                left_mission.set()
                abort.set()
                print(f"\nLeft Mission mode ({fm}).")

    gps_task  = asyncio.create_task(_gps_monitor())
    mode_task = asyncio.create_task(_mode_monitor())

    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / f"mission_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    try:
        await _log_mission(drone, ned_waypoints, ned_origin, yaw_deg, csv_path, abort)
        print(f"\nData saved: {csv_path}")
        print(f"Run: python3 trajectory_plotter.py {csv_path}")
        print("\nLand manually when ready.")
    finally:
        gps_task.cancel()
        mode_task.cancel()
        await drone.param.set_param_float("MPC_XY_CRUISE", default_cruise_speed)


def main():
    parser = argparse.ArgumentParser(description="PX4 mission-mode path follower")
    parser.add_argument(
        "--connection", default="udpin://0.0.0.0:14540",
        help="MAVLink connection string (SITL: udpin://0.0.0.0:14540 | QGC forward: udpin://0.0.0.0:14550 | HITL via mavproxy: udpin://0.0.0.0:14552)",
    )
    parser.add_argument(
        "--waypoints", default="waypoints_example.yaml",
        help="YAML file with body-relative waypoints [forward, right, up] in metres",
    )
    parser.add_argument(
        "--output-dir", default=".", type=Path,
        help="Directory for the CSV log (default: current directory)",
    )
    args = parser.parse_args()

    wp_path = Path(args.waypoints)
    if not wp_path.exists():
        raise FileNotFoundError(f"Waypoints file not found: {wp_path}")

    with open(wp_path) as f:
        config = yaml.safe_load(f)

    try:
        asyncio.run(run(
            connection=args.connection,
            waypoints=config["waypoints"],
            acceptance_radius=float(config.get("acceptance_radius", 0.5)),
            speed_ms=float(config.get("speed_ms", 0.5)),
            output_dir=args.output_dir,
        ))
    except KeyboardInterrupt:
        print("\nInterrupted — CSV saved up to last completed row.")


if __name__ == "__main__":
    main()
