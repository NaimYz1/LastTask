# Task Procedures & Tuning

Robot spawns at world/map origin **(0, 0)** facing **+x** (towards the
bridge). Coordinates below are in the `map` frame — read them off the RViz
grid (1 m cells).

## Task 1 — Drive under the bridge

The arena is split by a wall at x = 2.5 with a single doorway; the bridge
(1.5 m wide opening, **0.85 m clearance**, robot is ~0.60 m tall to the top
of the Mid-360 dome) sits in that doorway. The bridge is **not** in the
static map.

1. Start the three launch files (see README).
2. In RViz confirm the Mid-360 cloud shows the deck as the robot faces it,
   and that the **costmap shows the two supports but NOT the deck**.
3. **2D Nav Goal** at ~(5, 0).
4. Expected: the global plan goes through the bridge opening; the robot
   drives underneath and reaches the goal.

Evidence for the report:
- Screenshot: point cloud showing the full bridge vs costmap showing only
  supports (this is the height filter working).
- The naive-3D ablation:

  ```bash
  roslaunch fyp_jackal_navigation nav.launch clearance:=2.0
  ```

  Now the deck DOES mark → the doorway is blocked → move_base finds no valid
  plan (or plans forever). This proves that naively adding 3D points to a
  costmap is wrong, and that the height filter is the fix.

### Low-bridge crash demo (the strongest evidence)

```bash
roslaunch fyp_jackal_gazebo fyp_world.launch low_bridge:=true
```

This world's bridge deck is at **0.50 m** — above the Hokuyo scan plane
(0.37 m) but below the Mid-360 tower (0.60 m):

- `nav.launch use_mid360:=false` + goal (5, 0): the doorway looks free to
  the 2D lidar, the robot drives in, and the sensor tower **physically hits
  the deck** in Gazebo. This is the failure that motivates the whole FYP.
- `nav.launch` (3D on): the deck is below the 0.70 m clearance, so it marks
  as an obstacle, the doorway is blocked, and move_base refuses to send the
  robot under — the correct, safe behaviour.

Record both runs (screen capture + `rosbag record /front/scan /mid360/points
/move_base/local_costmap/costmap /tf /tf_static`) for the report.

## Task 2 — Avoid the tripod

Tripod at (-3, 0): three 12 mm legs, thin centre stick, and a
0.28 × 0.28 × 0.22 m box spanning z = 0.42–0.64 m (above the ~0.37 m scan
plane, below the 0.70 m clearance → it marks as an obstacle). At 0.37 m the
Hokuyo can only clip the converged tops of the legs — a few centimetres of
target — so the 2D view of the tripod is almost nothing.

1. **2D Nav Goal** at ~(-5, 0) — the straight path goes through the tripod.
2. Expected: with the Mid-360, the box footprint appears as a solid obstacle
   in the local costmap well before arrival and the robot swings around it.

Baseline comparison:

```bash
roslaunch fyp_jackal_navigation nav.launch use_mid360:=false
```

With only the Hokuyo, the costmap gets at most three thin flickering dots
from the legs (in reality, thin dark legs are often missed entirely) and
knows nothing about the box overhang. Compare costmap screenshots at the
same robot pose. Note in sim the ideal ray-cast laser sees the legs more
reliably than a real Hokuyo does — say so in the report.

## Tuning table (sim ↔ real robot)

| What | Where | Default |
|---|---|---|
| Robot clearance (max height a 3D point may mark) | `clearance` arg of `nav.launch` → `obstacles_layer/mid360/max_obstacle_height` | 0.70 m |
| Ground-return cutoff | `mid360/min_obstacle_height` in `costmap_common.yaml` | 0.08 m |
| Mid-360 tilt | `mid360_tilt_deg` arg of `fyp_world.launch` | 38.22° |
| Mid-360 position on tower (from `mid_mount`, top plate centre) | `mid360_xyz` arg | `-0.02 0 0.30` |
| Hokuyo position on riser box (from `mid_mount`) | `hokuyo_xyz` xacro arg in `fyp_jackal.urdf.xacro` | `0.08 0 0.075` |
| Lidar rays per update (sim speed) | `mid360_samples` arg | 10000 |
| Bridge/tripod dimensions | `fyp_jackal_gazebo/worlds/fyp_arena.world` (commented) | see world |
| Arena map | regenerate with `python scripts/generate_map.py` after editing the world's *permanent* structure | — |

**Measure on the real robot when you get access:** exact Mid-360 position
and tilt (update `mid360_xyz` / `mid360_tilt_deg`), total robot height
including the sensor (set `clearance` = height + ~0.10 m margin), the real
bridge's clearance and width, and the real tripod/box heights (mirror them
in the world file so the sim predicts the real behaviour).

## Real-robot notes (phase 2, after sim works)

Full step-by-step guide: [REAL_ROBOT.md](REAL_ROBOT.md). In short: install
Livox-SDK2 + livox_ros_driver2 on the onboard PC, configure the lidar's
Ethernet IPs, measure the mount and run
`roslaunch fyp_jackal_navigation nav_real.launch map_file:=<your map>` —
it reuses these exact configs with the real driver's PointCloud2 topic.
