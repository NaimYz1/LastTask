# Running on the Real Jackal (cpr-j100-0540)

Status as checked on 2026-07-13: the robot already runs everything sensor-side.
**No driver installation is needed.**

| What | Topic | Frame | Status |
|---|---|---|---|
| Jackal base (odom, EKF, twist_mux) | `/odometry/filtered`, `/cmd_vel` | `base_link` | running |
| Hokuyo (urg_node) | `/scan` | `laser` | running |
| Mid-360 (Livox driver) | `/livox/lidar` (+`/livox/imu`) | `livox_frame` (verify) | running |

Robot geometry (measured 2026-07-13): Mid-360 dome centre ~0.455 m above the
floor (tallest point ~0.50 m), Hokuyo optical centre ~0.355 m, tilt 38.22°
nose-down (mount design value). Real bridge: ~0.75 m clearance × ~1.0 m wide.
Real tripod: box top at ~1.07 m, legs ~0.26 m spread at the floor.

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
# Run WITHOUT nav_real running. If a transform already exists, the robot's
# own bringup publishes it -> launch nav_real with publish_mid360_tf:=false
# so two publishers don't fight over the frame.
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

## 3. TF calibration (10 minutes, once)

`nav_real.launch` publishes `base_link -> livox_frame` from measured values.
Already set: `mid360_z 0.39` (0.455 − 0.065 base_link height), pitch 0.6671
(38.22°). Still to measure: **the forward offset** — measure the horizontal
distance from the dome centre to the FRONT edge of the top plate; then
`mid360_x = 0.21 − that distance` (plate front edge ≈ 0.21 m ahead of robot
centre). A few cm of error is acceptable.

**Verification (do this before navigating):**

```bash
roslaunch fyp_jackal_navigation nav_real.launch
```

In RViz (fixed frame `base_link`), enable the "Mid360 Real (PointCloud2)"
display: the floor must be a flat sheet of points near z = 0 (colour-coded
blue/low), walls vertical. If the floor tilts up/down ahead of the robot,
adjust `mid360_pitch` by small steps (±0.02 rad) until flat.

## 4. Navigate

```bash
roslaunch fyp_jackal_navigation nav_real.launch          # 3D fusion ON
roslaunch fyp_jackal_navigation nav_real.launch use_mid360:=false   # 2D-only baseline
```

RViz from the VM/laptop on the same WiFi:

```bash
export ROS_MASTER_URI=http://192.168.1.124:11311   # robot's WiFi IP
export ROS_IP=<your_vm_ip>
roslaunch fyp_jackal_navigation view.launch
```

(Enable the "... Real" displays, disable the sim ones. If TF complains about
time, sync clocks: `sudo apt install chrony` on both.)

**2D Pose Estimate** on the map first, small test goal second, then the tasks.

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

## 6. Why the cloud looks "cut off" above ~0.75 m (it is not a bug)

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

## 7. Troubleshooting

| Symptom | Fix |
|---|---|
| Floor ahead marks as obstacle while driving | Raise `mid360/min_obstacle_height` 0.15 → 0.20; re-check pitch calibration. |
| Robot ignores the tripod legs until close | Marks accumulate over ~1 s of Mid-360 pattern; approach slower, or ask Claude for a cloud-accumulator node. |
| AMCL lost / jumps | Re-set 2D Pose Estimate; check the map matches the room's current furniture. |
| move_base won't enter the bridge | Costmap too fat for the 1.0 m opening: lower `footprint_padding` / `inflation_radius` slightly. |
| No `map -> odom` TF | AMCL gets no scans: check `/scan` and that `scan_topic` arg matches. |
