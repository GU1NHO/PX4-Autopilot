#!/usr/bin/env python3
"""
Bench test: verify the Offboard link works end-to-end WITHOUT flying.

Connects, streams a zero-velocity body setpoint, then asks PX4 to switch
into Offboard mode (drone.offboard.start()). PX4 must reply ACCEPTED or
DENIED to that request over the same link the real flight script would
use -- so this proves (or disproves) the round trip without ever needing
to arm or take off.

Safety: refuses to run any offboard command if the vehicle is armed.
Motors do not spin while disarmed regardless of flight mode, so this is
safe to run with the drone sitting on a table, propellers on or off.

Usage:
    python offboard_bench_test.py --connection udpin://0.0.0.0:14552
"""

import argparse
import asyncio

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

HOLD_SETPOINT = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)


async def _first(agen):
    async for item in agen:
        return item


async def run(connection: str, duration_s: float) -> None:
    drone = System()
    print(f"Connecting to {connection} ...")
    await drone.connect(system_address=connection)

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected.")
            break

    print("\nReading vehicle status (5s timeout per field)...")
    try:
        armed = await asyncio.wait_for(_first(drone.telemetry.armed()), timeout=5)
        mode = await asyncio.wait_for(_first(drone.telemetry.flight_mode()), timeout=5)
        health = await asyncio.wait_for(_first(drone.telemetry.health()), timeout=5)
        print(f"  Armed:        {armed}")
        print(f"  Flight mode:  {mode}")
        print(f"  GPS ok:       {health.is_global_position_ok}")
        print(f"  Home set:     {health.is_home_position_ok}")
    except asyncio.TimeoutError:
        print("  No telemetry received within 5s.")
        print("  -> The link may be receive-only from here (setpoints/commands")
        print("     you send might not be reaching the vehicle at all).")
        return

    if armed:
        print("\nABORT: vehicle is armed. Disarm it before running this bench test.")
        return

    print(f"\nStreaming zero-velocity setpoints, then requesting Offboard mode...")
    await drone.offboard.set_velocity_body(HOLD_SETPOINT)

    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"\nRESULT: PX4 REJECTED the Offboard request.")
        print(f"        {error}")
        print("        This still proves the command reached the vehicle and it")
        print("        replied -- the reason above tells you what's blocking entry")
        print("        (e.g. no valid position estimate, no GPS, RC required, etc).")
        return

    print("\nRESULT: PX4 ACCEPTED the Offboard request.")
    print("        Round trip confirmed: your setpoints are reaching the FMU.")

    print(f"\nHolding zero-velocity Offboard for {duration_s:.0f}s (still disarmed, safe)...")
    end_time = asyncio.get_event_loop().time() + duration_s
    while asyncio.get_event_loop().time() < end_time:
        await drone.offboard.set_velocity_body(HOLD_SETPOINT)
        await asyncio.sleep(0.05)  # 20 Hz, matches flight_path_follower.py

    try:
        await drone.offboard.stop()
        print("Offboard stopped cleanly.")
    except OffboardError as error:
        print(f"Warning: could not stop Offboard cleanly: {error}")


def main():
    parser = argparse.ArgumentParser(description="Bench test for the Offboard link (no flight required)")
    parser.add_argument(
        "--connection", default="udpin://0.0.0.0:14552",
        help="MAVLink connection string (same one flight_path_follower.py will use)",
    )
    parser.add_argument(
        "--duration", type=float, default=5.0,
        help="Seconds to hold Offboard mode once accepted (default: 5)",
    )
    args = parser.parse_args()

    try:
        asyncio.run(run(args.connection, args.duration))
    except KeyboardInterrupt:
        print("\nInterrupted.")


if __name__ == "__main__":
    main()
