# FYP — 3D LiDAR-Aided Navigation for Jackal J100

Navigation for a Clearpath Jackal J100 with a **Hokuyo UST-10LX** (2D, front
mount) and a **Livox Mid-360** (3D, tilted ~38.22° nose-down), on
**Ubuntu 18.04 / ROS Melodic / Gazebo 9**.

A 2D lidar only sees a single horizontal slice of the world (~0.37 m above
the floor with our top-plate mount). Two situations break it:

| Task | Problem for the 2D lidar | Solution |
|------|--------------------------|----------|
| **1. Bridge** (hollow underneath) | Scan plane passes through the open space — the robot is blind to the deck overhead. Is there clearance to drive under? | Mid-360 sees the full structure. Points are **height-filtered** in the costmap: anything above the robot's clearance (0.70 m) is ignored, so the robot confidently drives **under** the bridge while still avoiding its side supports. |
| **2. Tripod** (thin legs, box on top) | Legs are near-invisible (thin), the box floats above the scan plane. | Mid-360 marks the box and legs in the costmap → robot avoids it. |

## How it works

```
Hokuyo /front/scan  ──►  AMCL (localization)
                    ──►  move_base costmaps (obstacle source "scan")
Mid-360 /mid360/points ─►  move_base costmaps (obstacle source "mid360",
                            VoxelLayer, 0.08 m < z < 0.55 m height filter)
```

The single most important parameter is
`obstacles_layer/mid360/max_obstacle_height` (**0.70 m** = robot height
including the Mid-360 dome, ~0.60 m, + margin) in
[costmap_common.yaml](fyp_jackal_navigation/params/costmap_common.yaml)
— 3D points above it are treated as passable overhead structure, points
below it as real obstacles. It is exposed as the `clearance` launch argument.

## Packages

- **fyp_jackal_description** — Jackal URDF + UST-10 + Mid-360 (mount pose and
  tilt are xacro args; simulated with the
  [livox_laser_simulation](https://github.com/Livox-SDK/livox_laser_simulation) plugin)
- **fyp_jackal_gazebo** — arena world (bridge + tripod), spawn launch files
- **fyp_jackal_navigation** — AMCL + move_base configs, pre-built map, RViz config
- **scripts** — VM setup script, map generator

## Quick start (on the Ubuntu 18.04 VM)

```bash
mkdir -p ~/fyp_ws/src && cd ~/fyp_ws/src
git clone <THIS_REPO_URL> fyp_jackal
bash fyp_jackal/scripts/vm_setup.sh        # gazebo upgrade + deps + build
source ~/fyp_ws/devel/setup.bash
```

Run (three terminals):

```bash
roslaunch fyp_jackal_gazebo fyp_world.launch      # simulation
roslaunch fyp_jackal_navigation nav.launch        # localization + planning
roslaunch fyp_jackal_navigation view.launch       # RViz
```

In RViz use **2D Nav Goal**:
- **Task 1 (bridge):** goal at roughly (5, 0) — the robot must pass under the
  bridge (the only doorway to the east half).
- **Task 2 (tripod):** goal at roughly (-5, 0) — the straight-line path goes
  through the tripod; the robot must go around it.

### Demonstrations / ablations for the report

```bash
roslaunch fyp_jackal_navigation nav.launch use_mid360:=false   # 2D-only baseline
roslaunch fyp_jackal_navigation nav.launch clearance:=2.0      # naive 3D: bridge deck
                                                               # marks as obstacle and
                                                               # blocks the doorway
```

See [docs/TASKS.md](docs/TASKS.md) for full demo procedures and
[docs/VM_SETUP.md](docs/VM_SETUP.md) for detailed setup/troubleshooting.

## Matching the real robot

All sim-vs-reality knobs are launch args / yaml values — see the tuning table
in [docs/TASKS.md](docs/TASKS.md). On the real robot the same nav configs are
reused; only the Mid-360 observation source changes (`data_type: PointCloud2`,
the livox_ros_driver topic, and the real TF frame).
