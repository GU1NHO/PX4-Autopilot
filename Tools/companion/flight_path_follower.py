#!/usr/bin/env python3
"""
Companion computer script for PX4 autonomous flight path following.

Workflow:
  1. Run this script before or after arming.
  2. Arm and take off manually in Position mode.
  3. Switch to Offboard mode via QGC or RC switch.
     At that moment the script captures position and heading and transforms
     the waypoints: [forward_m, right_m, up_m] relative to where the drone
     is and which way it is pointing.
  4. Script executes waypoints slowly and logs desired vs actual.
  5. Land manually after path completes.
  6. Use trajectory_plotter.py to analyse the saved CSV.

Waypoint format in YAML  (body-relative offsets at Offboard activation):
  [forward_m, right_m, up_m]
  forward > 0 → ahead of drone     right > 0 → right of drone
  up > 0      → climb              up = 0    → maintain current altitude

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
from mavsdk.offboard import OffboardError, PositionNedYaw


SETPOINT_RATE_HZ = 20  # PX4 requires > 2 Hz to stay in Offboard mode
DT = 1.0 / SETPOINT_RATE_HZ


def _distance_3d(a: tuple, b: tuple) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def _to_body(n: float, e: float, n0: float, e0: float, cos_y: float, sin_y: float) -> tuple:
    """Convert absolute NED position to body-frame (fwd, right) relative to origin."""
    rel_n = n - n0
    rel_e = e - e0
    return rel_n * cos_y + rel_e * sin_y, -rel_n * sin_y + rel_e * cos_y


async def _wait_for_gps(drone: System) -> None:
    print("Waiting for GPS lock...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("GPS lock acquired.")
            return


async def _get_current_position_ned(drone: System) -> tuple:
    async for pos in drone.telemetry.position_velocity_ned():
        return (pos.position.north_m, pos.position.east_m, pos.position.down_m)


async def _get_current_yaw_deg(drone: System) -> float:
    async for att in drone.telemetry.attitude_euler():
        return att.yaw_deg


def _transform_waypoints(
    raw_waypoints: list,
    origin_ned: tuple,
    yaw_deg: float,
) -> list:
    """Convert body-relative [forward, right, up] offsets to absolute NED.

    Rotation: body forward/right → NED North/East using the drone's yaw at
    the moment Offboard mode was activated.  'up' becomes a negative D offset
    (NED Down is positive downward).
    """
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


async def _wait_for_airborne(drone: System) -> None:
    print("Waiting for drone to be armed and airborne...")
    print("  → Arm and take off manually in Position mode.")
    async for is_armed in drone.telemetry.armed():
        if is_armed:
            break

    async for pos in drone.telemetry.position_velocity_ned():
        alt = -pos.position.down_m
        if alt > 0.3:
            print(f"\nAirborne. Ready.")
            return


async def _wait_for_offboard(drone: System) -> None:
    """Send the current position as setpoint every tick until Offboard mode activates.

    The setpoint is updated on every cycle, so if the operator climbs or
    moves before switching to Offboard the hold target is always the live
    position — no jump occurs when the mode change happens.
    """
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

    await asyncio.sleep(0.1)  # let streams populate cur with initial values

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


async def _execute_waypoints(
    drone: System,
    waypoints: list,
    acceptance_radius: float,
    speed_ms: float,
    yaw_deg: float,
    origin_ned: tuple,
    csv_path: Path,
    abort: asyncio.Event,
) -> None:
    """Execute waypoints, writing every row to csv_path.
    Aborts early if abort is set (GPS failure or mode change).
    CSV is always flushed on exit via the 'with open' context manager.
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

        print(f"Logging to {csv_path}")
        print(f"Waypoints: {len(waypoints)}  |  speed: {speed_ms} m/s  |  acceptance: {acceptance_radius} m\n")

        with open(csv_path, "w", newline="") as f:
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

            for i, wp in enumerate(waypoints):
                if abort.is_set():
                    print("  Waypoint execution aborted.")
                    return

                target = (float(wp[0]), float(wp[1]), float(wp[2]))
                print(f"Waypoint {i+1}/{len(waypoints)}: "
                      f"N={target[0]:.1f} E={target[1]:.1f} D={target[2]:.1f}")

                while not abort.is_set():
                    diff = [target[j] - interp[j] for j in range(3)]
                    dist_to_target = math.sqrt(sum(d**2 for d in diff))
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

                    if error < acceptance_radius and dist_to_target < 1e-3:
                        print(f"  Reached (error={error:.2f}m)")
                        break

                    await asyncio.sleep(DT)

    finally:
        stop_telem.set()
        telem_task.cancel()
        gps_telem_task.cancel()


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

    await _wait_for_offboard(drone)

    origin = await _get_current_position_ned(drone)
    yaw_deg = await _get_current_yaw_deg(drone)
    ned_waypoints = _transform_waypoints(waypoints, origin, yaw_deg)

    print("Offboard active — reference captured:")
    print(f"  Position: N={origin[0]:.2f} E={origin[1]:.2f} D={origin[2]:.2f} m")
    print(f"  Heading:  {yaw_deg:.1f}°  (0=North, 90=East)")
    print("  Waypoints transformed to NED:")
    for i, wp in enumerate(ned_waypoints):
        print(f"    {i+1}: N={wp[0]:.2f} E={wp[1]:.2f} D={wp[2]:.2f}")
    print()

    # --- Shared abort event (GPS failure OR mode change) ---
    abort = asyncio.Event()
    gps_lost = asyncio.Event()
    left_offboard = asyncio.Event()

    async def _gps_monitor():
        async for health in drone.telemetry.health():
            if not health.is_global_position_ok and not gps_lost.is_set():
                gps_lost.set()
                abort.set()
                print("\nWARNING: GPS position lost — stopping Offboard setpoints.")
                print("         PX4 failsafe will activate (COM_OBL_RC_ACT → Altitude mode).")

    async def _mode_monitor():
        async for fm in drone.telemetry.flight_mode():
            if "OFFBOARD" not in str(fm) and not left_offboard.is_set():
                left_offboard.set()
                abort.set()
                print(f"\nLeft Offboard mode ({fm}).")

    gps_task  = asyncio.create_task(_gps_monitor())
    mode_task = asyncio.create_task(_mode_monitor())

    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / f"flight_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    try:
        await _execute_waypoints(
            drone, ned_waypoints, acceptance_radius, speed_ms, yaw_deg, origin, csv_path, abort
        )

        # CSV is already saved (file closed by 'with open' inside _execute_waypoints)
        print(f"\nData saved: {csv_path}")
        print(f"Run: python3 trajectory_plotter.py {csv_path}")

        if abort.is_set():
            return  # GPS failed or mode changed — don't enter hold loop

        # Hold last position until user exits Offboard or GPS fails
        print("Holding last position. Switch out of Offboard mode (QGC or RC) to end.")
        last_n, last_e, last_d = ned_waypoints[-1]
        hold_end = PositionNedYaw(last_n, last_e, last_d, yaw_deg)

        while not abort.is_set():
            await drone.offboard.set_position_ned(hold_end)
            await asyncio.sleep(DT)

        if gps_lost.is_set():
            print("GPS failure: setpoints stopped. PX4 failsafe active.")
        else:
            print("Script done.")

    finally:
        gps_task.cancel()
        mode_task.cancel()


def main():
    parser = argparse.ArgumentParser(description="PX4 flight path follower")
    parser.add_argument(
        "--connection", default="udpin://0.0.0.0:14540",
        help="MAVLink connection string (SITL: udpin://0.0.0.0:14540 | QGC forward: udpin://0.0.0.0:14550 | HITL via mavproxy: udpin://0.0.0.0:14552)",
    )
    parser.add_argument(
        "--waypoints", default="waypoints_example.yaml",
        help="YAML file with waypoints [N, E, D] in metres",
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
