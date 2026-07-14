# Running on the Real Jackal (cpr-j100-0540)

Status as checked on 2026-07-13: the robot already runs everything sensor-side.
**No driver installation is needed.**

| What | Topic | Frame | Status |
|---|---|---|---|
| Jackal base (odom, EKF, twist_mux) | `/odometry/filtered`, `/cmd_vel` | `base_link` | running |
| Hokuyo (urg_node) | `/scan` | `laser` | running |
| Mid-360 (Livox driver) | `/livox/lidar` (+`/livox/imu`) | `livox_frame` (verify) | running |

Robot geometry: Mid-360 tilt **38.73° nose-down, sensor 0.440 m above the
floor** — measured with `rosrun fyp_jackal_navigation measure_tilt.py`
(floor-plane fit, 2026-07-13; agrees with the ~0.455 m tape measure). The
45° static TF from the peer's amr_system is mis-calibrated by ~6°. Robot's
tallest point ~0.50 m; Hokuyo optical centre ~0.31 m (per its TF). Real
bridge: ~0.75 m clearance × ~1.0 m wide. Real tripod: box top at ~1.07 m,
legs ~0.26 m spread at the floor.

## 1. Pre-flight checks (5 commands on the robot)

```bash
rostopic type /livox/lidar
# want: sensor_msgs/PointCloud2. If it says livox_ros_driver2/CustomMsg,
# the driver's xfer_format must be changed to 0 (tell Claude).

rostopic echo /livox/lidar -n 1 | grep frame_id
# note the frame name; if it is not "livox_frame", pass
# mid360_frame:=<name> to nav_real.launch.

rosrun tf tf_echo base_link laser
# If this prints a transform -> good, laser TF exists (your amr_bringup
# probably publishes it). If it errors, add publish_laser_tf:=true to
# nav_real.launch.

rostopic hz /scan          # ~40 Hz
rostopic hz /livox/lidar   # ~10 Hz

rosrun tf tf_echo base_link livox_frame
# This transform is NOT part of the robot's permanent bringup - it only
# exists while the peer's amr_system launch is running. nav_real.launch
# therefore publishes it itself (publish_mid360_tf defaults to true) with
# the same calibrated values: xyz [0.179, 0, 0.394], pitch 45.0 deg.
# (Actual tilt is ~45 deg, not the 38.22 deg design figure.)
```

## 2. Get the code onto the robot

```bash
mkdir -p ~/fyp_ws/src && cd ~/fyp_ws/src
git clone https://github.com/NaimYz1/LastTask.git fyp_jackal
# Do NOT clone livox_laser_simulation here - it is only for Gazebo.
cd ~/fyp_ws && catkin_make
echo "source ~/fyp_ws/devel/setup.bash" >> ~/.bashrc
```

The `myroom` map is included in the repo
(`fyp_jackal_navigation/maps/myroom.yaml`), so nothing to copy.

## 3. How the Mid-360 reaches the costmap (and why)

Another system on this robot also publishes a pose for `livox_frame`
(observed: the costmap once saw the sensor at z = −0.14, below the floor,
4.5 m away from the robot — TF was flip-flopping between publishers, and
phantom obstacles walled off the whole map). Navigation is therefore
**TF-free** for this sensor: the `mid360_filter` node (started by
`nav_real.launch`) applies the fixed mount transform itself (launch args
`mid360_x/y/z/pitch`), keeps only points in the obstacle band
(`band_min_z`/`band_max_z`, relative to base_link), and publishes
`/mid360/obstacles` in base_link frame. The costmap consumes that.

**Pitch calibration (do this before navigating):** in RViz enable the
"Mid360 Obstacles (filtered, real)" display (magenta) and point the robot
at OPEN floor with a wall a few metres away:

- Correct: magenta points only on the wall (0.19–0.57 m heights); the open
  floor between robot and wall is EMPTY.
- Pitch too small / too large: a carpet of magenta points appears on the
  bare floor a few metres ahead. Adjust `mid360_pitch:=` in steps of ±0.02
  (rad) until the floor is clean. Current default 0.7854 (45°).

## 4. Navigate — the two-terminal workflow

**Terminal 1 — SSH into the robot** (runs navigation):

```bash
ssh administrator@192.168.1.124
roslaunch fyp_jackal_navigation nav_real.launch                     # 3D fusion ON
# or: roslaunch fyp_jackal_navigation nav_real.launch use_mid360:=false   # 2D-only baseline
```

**Terminal 2 — on the VM** (runs RViz). IMPORTANT: plain `rviz` opens the
DEFAULT config with none of our displays — always load ours via
`view.launch` (keep the repo on the VM up to date with `git pull` so the
RViz config matches):

```bash
export ROS_MASTER_URI=http://192.168.1.124:11311   # robot's WiFi IP
export ROS_IP=<your_vm_ip>                          # e.g. 192.168.1.109
roslaunch fyp_jackal_navigation view.launch
```

(Equivalent: `rviz -d ~/fyp_ws/src/fyp_jackal/fyp_jackal_navigation/rviz/fyp_nav.rviz`.
Displays to keep ON for the real robot: "Mid360 Obstacles (filtered, real)"
(magenta - what the costmap consumes), "Hokuyo Real (/scan)", map/costmaps/
paths. Keep "Mid360 Real (PointCloud2)" OFF normally - it is the raw cloud
including floor and CEILING, only useful for whole-bridge screenshots.
Disable the sim displays. If TF complains about time, sync clocks:
`sudo apt install chrony` on both machines.)

**2D Pose Estimate** on the map first, small test goal second, then the tasks.

Known harmless warnings on this robot:
- `TF_OLD_DATA ... frame livox_frame ... unknown_publisher`: another system
  on the robot publishes that frame with stale stamps. Navigation does not
  use it (the mid360_filter node replaces TF), so ignore.
- The EKF odometry drifts in z (observed −0.22 m). The local costmap uses
  the 2D ObstacleLayer (`local_costmap_real.yaml`) exactly so that this
  cannot break obstacle marking or clearing.

## 5. Task physics with YOUR measurements — read this

- **Bridge** (0.75 m clearance vs 0.50 m robot): fits. With `clearance` =
  0.60, the deck (0.75) is ignored and the robot drives under. The 1.0 m
  width is the tight dimension: only ~0.28 m spare per side, which is why the
  real config uses smaller padding/inflation. If move_base hesitates at the
  entrance, lower `footprint_padding` to 0.03 in `costmap_common_real.yaml`.
- **Tripod**: your box spans ~0.85–1.07 m — *higher than the bridge deck
  (0.75 m)*. Therefore NO height filter can mark the box while letting the
  bridge pass: physically, if the robot fits under a 0.75 m bridge, it also
  fits under a 1.07 m box. The robot will avoid the tripod anyway because the
  Mid-360 sees the **legs and centre stick** (below 0.60 m) far more reliably
  than the Hokuyo does. If you want the "3D detects the box" demonstration
  specifically, shorten the tripod so the box bottom sits below ~0.55 m.
- Safety: first bridge run with a hand on the e-stop; max speed is 0.5 m/s.

## 6. Debugging failed goals: the black-box recorder

When a goal aborts ("spins then stops"), run this in a third terminal
DURING the tests:

```bash
rosrun fyp_jackal_navigation nav_debug.py
```

It records everything relevant to `~/.ros/nav_debug/nav_debug_*.log`
(2-second status snapshots: pose, localization std, lethal cells around the
robot, mid360 point count, cmd_vel) and, at the moment move_base aborts,
prints a **VERDICT** naming the most likely cause:

- **MISLOCALIZED** — AMCL covariance blew up (the "works after re-setting
  2D Pose Estimate" signature). Fix: fresh pose estimate, not a restart.
- **PINNED / BOXED IN** — lethal cells inside/around the footprint.
  Screenshot the local costmap + magenta cloud: real object or phantom?
- **SENSOR STALL** — a topic went silent (driver died / battery / network).
- none of the above — tight-gap maneuver failure; place goals in open
  space and approach gaps head-on.

It also raises move_base's internal loggers to DEBUG, so per-cycle planner
detail (every scored trajectory, every costmap update) is captured in
`~/.ros/log/latest/move_base*.log` for deep dives.

## 7. Why the cloud looks "cut off" above ~0.75 m (it is not a bug)

The Mid-360's vertical FOV is −7°..+52° relative to itself; tilted 38.22°
nose-down that is **−45°..+14° relative to the floor**. From 0.455 m height,
the highest visible point at distance d is `0.455 + d·tan(14°)`:
~0.70 m at 1 m, ~0.95 m at 2 m, ~1.20 m at 3 m. The sensor sees the whole
bridge **from 2–3 m away**; up close, the top exits the FOV upward — by
design, in exchange for good ground coverage. For navigation it is
irrelevant: the costmap only uses points below the 0.60 m clearance.
For "whole bridge" screenshots, set the RViz cloud Decay Time to ~5 s and
approach slowly from 3 m.

**Do NOT feed `amr_system`'s `projection.launch` scans into navigation.**
Its obstacle converter flattens everything up to `max_height: 1.5` into a 2D
scan — the bridge deck becomes a wall and the robot will refuse to drive
under. Our `nav_real.launch` consumes `/livox/lidar` directly with proper
height filtering; no converter is needed.

## 8. Troubleshooting

| Symptom | Fix |
|---|---|
| Floor ahead marks as obstacle while driving | Raise `mid360/min_obstacle_height` 0.15 → 0.20; re-check pitch calibration. |
| Robot clips thin obstacles (tripod legs, stool poles) | The filter node accumulates 1.5 s of odometry-compensated points precisely for this; raise `accumulate:=2.5` for even stickier marks (at the cost of slower clearing of moving obstacles). |
| Robot clips LOW structures (stool star-base, low boxes) | Lower `band_min_z` (default 0.05 = ~0.12 m above floor). If lowering further, watch /mid360/obstacles for floor false-positives. |
| AMCL lost / jumps | Re-set 2D Pose Estimate; check the map matches the room's current furniture. |
| move_base won't enter the bridge | Costmap too fat for the 1.0 m opening: lower `footprint_padding` / `inflation_radius` slightly. |
| No `map -> odom` TF | AMCL gets no scans: check `/scan` and that `scan_topic` arg matches. |
