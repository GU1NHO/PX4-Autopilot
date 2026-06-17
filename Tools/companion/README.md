# Flight Path Follower

Companion computer scripts to send a predefined NED flight path to a PX4 drone in Offboard mode and compare the desired vs actual trajectory.

## Requirements

```bash
pip install mavsdk pyyaml matplotlib numpy
```

## How it works

| Script | Role |
|--------|------|
| `flight_path_follower.py` | Connects to PX4, sends position setpoints, logs desired+actual to CSV |
| `trajectory_plotter.py`   | Reads CSV and generates 3D/2D plots + tracking error metrics |
| `waypoints_example.yaml`  | Example square path (5m × 5m at 2m altitude) |

## Connection

The radio telemetry on TELEM1 is already used by QGroundControl.
QGC forwards MAVLink to UDP 14550 by default — the script connects there:

```bash
python3 flight_path_follower.py  # default: udp://:14550
```

If QGC does **not** forward (check in *Application Settings → MAVLink → Forward MAVLink*), use `mavlink-router` to multiplex the serial port:

```bash
# Install: sudo apt install mavlink-router
mavlink-router /dev/ttyUSB0:57600 \
  --endpoint udp-server:0.0.0.0:14550 \
  --endpoint udp-server:0.0.0.0:14551
# QGC connects to 14550, script connects to 14551:
python3 flight_path_follower.py --connection udp://:14551
```

## Step-by-step flight procedure

1. Connect the drone, open QGroundControl.
2. **Start the script** (before arming):
   ```bash
   python3 flight_path_follower.py --waypoints waypoints_example.yaml
   ```
3. Script prints `Waiting for GPS lock...` then `Waiting for drone to be armed and airborne`.
4. **Arm and take off manually** in Position mode. Climb to the desired altitude (≥ 2 m for the example path).
5. **Switch to Offboard mode** via QGC (*Flight Mode → Offboard*) or your RC aux switch.
6. Script starts executing waypoints and prints real-time progress + error.
7. After the last waypoint is reached, script holds that position — **land manually**.
8. Script saves `flight_YYYYMMDD_HHMMSS.csv` in the current directory.

## Analyse the trajectory

```bash
python3 trajectory_plotter.py flight_20240617_143000.csv
```

Outputs:
- **3D plot**: desired (blue dashed) vs actual (red solid)
- **Top-down plot** (N × E)
- **Altitude over time**
- **Tracking error over time**: 3D Euclidean distance and perpendicular distance to desired segment
- **Console summary**: max error, RMS, per-segment mean/max

## Define your own path

Edit `waypoints_example.yaml` or create a new YAML file:

```yaml
acceptance_radius: 0.5   # metres — when to advance to next waypoint

waypoints:
  # [North_m, East_m, Down_m]   Down is NEGATIVE to go UP (NED convention)
  - [10.0,  0.0, -3.0]   # 10m North, 3m altitude
  - [10.0, 10.0, -3.0]
  - [ 0.0, 10.0, -3.0]
  - [ 0.0,  0.0, -3.0]
```

Pass it with `--waypoints my_path.yaml`.

## Safety notes

- Always keep the RC transmitter ready to switch back to Position mode.
- The script **does not arm or disarm** the drone — arming/takeoff/landing are always manual.
- If the script is killed, PX4 will detect the loss of Offboard setpoints and switch to failsafe (default: Hold mode). Tune `COM_OF_LOSS_T` in QGC to adjust the timeout.
- Test in a safe open area at low altitude first.
