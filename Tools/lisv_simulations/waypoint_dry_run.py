#!/usr/bin/env python3
"""
Ground-truth validator for the body-relative waypoint transform -- no
flight required, no commands sent to the vehicle.

Connects to the vehicle wherever it currently is (on the ground, armed or
not), reads its real position and heading, and prints exactly what
flight_path_follower.py / mission_path_follower.py would compute for each
waypoint in the YAML -- as NED deltas, absolute lat/lon, and (most useful
in the field) distance + compass bearing from the current position, so
you can sanity-check with a handheld compass and a tape measure.

This is telemetry read-only: it never calls drone.offboard.* or
drone.mission.*, and never arms the vehicle. Safe to run at any time,
motors on or off, on the bench or in the field.

Usage:
    python waypoint_dry_run.py --connection udpin://0.0.0.0:14552 --waypoints waypoints_example.yaml
"""

import argparse
import asyncio
import math
from pathlib import Path

import yaml
from mavsdk import System


_EARTH_RADIUS_M = 6378137.0  # WGS84 semi-major axis; fine for local-scale paths


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


def _ned_offset_to_global(origin_lat_deg: float, origin_lon_deg: float, north_m: float, east_m: float) -> tuple:
    """Identical projection to mission_path_follower.py."""
    lat = origin_lat_deg + (north_m / _EARTH_RADIUS_M) * (180.0 / math.pi)
    lon = origin_lon_deg + (east_m / (_EARTH_RADIUS_M * math.cos(math.radians(origin_lat_deg)))) * (180.0 / math.pi)
    return lat, lon


def _bearing_and_distance(dn: float, de: float) -> tuple:
    """Compass bearing (0=N, 90=E, clockwise) and distance for a NED offset --
    point a handheld/phone compass along this bearing to sanity-check direction."""
    distance = math.sqrt(dn ** 2 + de ** 2)
    bearing = math.degrees(math.atan2(de, dn)) % 360.0
    return distance, bearing


async def run(connection: str, waypoints_path: Path) -> None:
    drone = System()
    print(f"Connecting to {connection} ...")
    await drone.connect(system_address=connection)

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected.")
            break

    print("Waiting for GPS lock (safe to run disarmed, on the ground)...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("GPS lock acquired.\n")
            break

    async for pos in drone.telemetry.position_velocity_ned():
        ned_origin = (pos.position.north_m, pos.position.east_m, pos.position.down_m)
        break
    async for gpos in drone.telemetry.position():
        origin_lat, origin_lon = gpos.latitude_deg, gpos.longitude_deg
        break
    async for att in drone.telemetry.attitude_euler():
        yaw_deg = att.yaw_deg
        break

    print(f"Current position:       N={ned_origin[0]:.2f} E={ned_origin[1]:.2f} D={ned_origin[2]:.2f} m")
    print(f"Current lat/lon:        {origin_lat:.7f}, {origin_lon:.7f}")
    print(f"Drone heading (compass): {yaw_deg:.1f}°  <- point a phone compass this way to check the nose")
    print()

    with open(waypoints_path) as f:
        config = yaml.safe_load(f)
    raw_waypoints = config["waypoints"]

    ned_waypoints = _transform_waypoints(raw_waypoints, ned_origin, yaw_deg)

    # Same "# origin_ned_m:" / "# waypoint_NN_ned_m:" format written into the
    # flight CSV logs, so this dry-run output can be diffed directly against
    # a real flight's log header to confirm the same absolute path was used.
    print(f"# origin_ned_m: N={ned_origin[0]:.4f},E={ned_origin[1]:.4f},D={ned_origin[2]:.4f}")
    for i, wp in enumerate(ned_waypoints):
        print(f"# waypoint_{i + 1:02d}_ned_m: N={wp[0]:.4f},E={wp[1]:.4f},D={wp[2]:.4f}")
    print()

    header = f"{'#':<3} {'[fwd,right,up]':<18} {'dN(m)':>7} {'dE(m)':>7} {'dist(m)':>9} {'bearing':>9}  lat,lon"
    print(header)
    print("-" * len(header))
    for i, (wp, ned) in enumerate(zip(raw_waypoints, ned_waypoints)):
        dn = ned[0] - ned_origin[0]
        de = ned[1] - ned_origin[1]
        distance, bearing = _bearing_and_distance(dn, de)
        lat, lon = _ned_offset_to_global(origin_lat, origin_lon, ned[0], ned[1])
        print(f"{i + 1:<3} {str(wp):<18} {dn:7.2f} {de:7.2f} {distance:9.2f} {bearing:9.1f}  {lat:.7f},{lon:.7f}")

    print()
    print("No commands were sent to the vehicle -- telemetry read-only, nothing armed or moved.")
    print("Sanity check: stand by the vehicle, sight each 'bearing' above with a compass,")
    print("and confirm it points where you'd expect (e.g. waypoint 1 with right=0 should")
    print("point the same way as the drone's nose). You can also paste the lat/lon into")
    print("QGroundControl's map to visually confirm the point lands where expected.")


def main():
    parser = argparse.ArgumentParser(
        description="Dry-run validator for body-relative waypoints -- read-only, no flight required"
    )
    parser.add_argument(
        "--connection", default="udpin://0.0.0.0:14540",
        help="MAVLink connection string (SITL: udpin://0.0.0.0:14540 | QGC forward: udpin://0.0.0.0:14550 | HITL via mavproxy: udpin://0.0.0.0:14552)",
    )
    parser.add_argument(
        "--waypoints", default="waypoints_example.yaml",
        help="YAML file with body-relative waypoints [forward, right, up] in metres",
    )
    args = parser.parse_args()

    wp_path = Path(args.waypoints)
    if not wp_path.exists():
        raise FileNotFoundError(f"Waypoints file not found: {wp_path}")

    try:
        asyncio.run(run(args.connection, wp_path))
    except KeyboardInterrupt:
        print("\nInterrupted.")


if __name__ == "__main__":
    main()
