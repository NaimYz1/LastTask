# Task Procedures & Tuning

Robot spawns at world/map origin **(0, 0)** facing **+x** (towards the
bridge). Coordinates below are in the `map` frame — read them off the RViz
grid (1 m cells).

## Task 1 — Drive under the bridge

The arena is split by a wall at x = 2.5 with a single doorway; the bridge
(1.5 m wide opening, **0.85 m clearance**, robot is ~0.44 m tall) sits in
that doorway. The bridge is **not** in the static map.

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

## Task 2 — Avoid the tripod

Tripod at (-3, 0): three 12 mm legs, thin centre stick, and a
0.28 × 0.28 × 0.22 m box spanning z = 0.42–0.64 m (above the 0.19 m scan
plane, below the 0.55 m clearance → it marks as an obstacle).

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
| Robot clearance (max height a 3D point may mark) | `clearance` arg of `nav.launch` → `obstacles_layer/mid360/max_obstacle_height` | 0.55 m |
| Ground-return cutoff | `mid360/min_obstacle_height` in `costmap_common.yaml` | 0.08 m |
| Mid-360 tilt | `mid360_tilt_deg` arg of `fyp_world.launch` | 38.22° |
| Mid-360 position on top plate (from `mid_mount`) | `mid360_xyz` arg | `0.17 0 0.14` |
| Lidar rays per update (sim speed) | `mid360_samples` arg | 10000 |
| Bridge/tripod dimensions | `fyp_jackal_gazebo/worlds/fyp_arena.world` (commented) | see world |
| Arena map | regenerate with `python scripts/generate_map.py` after editing the world's *permanent* structure | — |

**Measure on the real robot when you get access:** exact Mid-360 position
and tilt (update `mid360_xyz` / `mid360_tilt_deg`), total robot height
including the sensor (set `clearance` = height + ~0.10 m margin), the real
bridge's clearance and width, and the real tripod/box heights (mirror them
in the world file so the sim predicts the real behaviour).

## Real-robot notes (phase 2, after sim works)

- The Mid-360 publishes via `livox_ros_driver`/`livox_ros_driver2` as
  **PointCloud2** — change the `mid360` source in `costmap_common.yaml`:
  `data_type: PointCloud2`, `topic:` the driver's topic, `sensor_frame:` the
  driver's TF frame (make sure a static TF from the robot to that frame
  matches the measured mount).
- The Mid-360's non-repetitive pattern is sparse per single message —
  if marking looks thin, aggregate a few scans or lower the costmap
  `update_frequency` gap by feeding with a small relay that accumulates
  2–3 clouds. (In sim this is not needed.)
- Keep AMCL on the Hokuyo exactly as in sim; your existing real-world map
  keeps working.
- On the floor, z=0 in the map frame is the floor **only if** the map/odom
  frames start on the floor (they do for Jackal bringup). If the real
  Mid-360 marks the floor, raise `min_obstacle_height` slightly (0.10–0.15).
