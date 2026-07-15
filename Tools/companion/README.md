# Flight Path Follower

Companion computer scripts to send a predefined flight path to a PX4 drone in Offboard mode and compare the desired vs actual trajectory.

## Requirements

```bash
pip install mavsdk pyyaml matplotlib numpy
```

## How it works

| Script | Role |
|--------|------|
| `flight_path_follower.py` | Connects to PX4, sends position setpoints, logs desired+actual to CSV |
| `trajectory_plotter.py`   | Reads CSV and generates plots + tracking error metrics |
| `waypoints_example.yaml`  | Example square path (5m × 5m) in body frame |

## Connection

| Environment | Command |
|-------------|---------|
| SITL (Gazebo) | `python3 flight_path_follower.py` (default `udpin://0.0.0.0:14540`) |
| Real hardware | `python3 flight_path_follower.py --connection udpin://0.0.0.0:14552` |

**Real hardware setup:** enable MAVLink forwarding in QGC to port 14552:
*Application Settings → MAVLink → Forward MAVLink to Host → `localhost:14552`*

## Step-by-step flight procedure

1. Connect the drone, open QGroundControl.
2. **Start the script** (before or after arming):
   ```bash
   # SITL
   python3 flight_path_follower.py --waypoints waypoints_example.yaml

   # Real hardware
   python3 flight_path_follower.py --connection udpin://0.0.0.0:14552 --waypoints waypoints_example.yaml
   ```
3. Script prints `Waiting for GPS lock...` then `Waiting for drone to be armed and airborne`.
4. **Arm and take off manually** in Position mode. Climb to the desired altitude.
5. **Switch to Offboard mode** via QGC (*Flight Mode → Offboard*) or RC aux switch.
   - At that moment the script captures position and heading and transforms the waypoints.
6. Script executes waypoints and prints real-time progress + error.
7. After the last waypoint is reached, script holds that position — **land manually**.
8. Script saves `flight_YYYYMMDD_HHMMSS.csv` in the current directory.

## Define your own path

Edit `waypoints_example.yaml`. Waypoints are in **drone body frame** at the moment Offboard is activated:

```yaml
acceptance_radius: 0.5   # metres — when to advance to next waypoint
speed_ms: 0.4            # setpoint speed (m/s) — keep low for precision

waypoints:
  # [forward_m, right_m, up_m]
  #   forward > 0 → ahead    right > 0 → right    up > 0 → climb
  - [5.0,  0.0, 0.0]   # 5m ahead
  - [5.0,  5.0, 0.0]   # 5m ahead, 5m right
  - [0.0,  5.0, 0.0]   # 5m right
  - [0.0,  0.0, 0.0]   # return to start
```

`up = 0` maintains current altitude. The script rotates the path using the drone's heading at Offboard activation, so the square always executes in front of the drone regardless of which way it is pointing.

## Analyse the trajectory

```bash
python3 trajectory_plotter.py flight_YYYYMMDD_HHMMSS.csv
```

Outputs:
- **GPS signal quality**: satellites count and fix type over time
- **Top-down plot** in body frame (Forward × Right) — the path always appears aligned
- **Altitude over time**
- **Tracking error over time**: 3D Euclidean distance and perpendicular distance to desired segment
- **Console summary**: max error, RMS, per-segment mean/max, GPS satellite stats

## Safety notes

- Always keep the RC transmitter ready to switch back to Position mode.
- The script **does not arm, disarm, or take off** — these are always manual.
- If GPS position is lost, the script stops sending setpoints and PX4 activates the failsafe configured in `COM_OBL_RC_ACT` (recommended: Altitude mode).
- If the script exits or is killed, PX4 detects loss of Offboard setpoints after `COM_OF_LOSS_T` seconds and switches to failsafe.
- Test in a safe open area at low altitude first.
