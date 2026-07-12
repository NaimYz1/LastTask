# VM Setup — Ubuntu 18.04 + ROS Melodic

One-time setup of the simulation environment inside your VMware VM.

## 1. Get the code

```bash
mkdir -p ~/fyp_ws/src
cd ~/fyp_ws/src
git clone <THIS_REPO_URL> fyp_jackal
```

## 2. Run the setup script

```bash
bash ~/fyp_ws/src/fyp_jackal/scripts/vm_setup.sh
```

It does four things (you can also run them manually, see below):

1. **Upgrades Gazebo 9.0 → latest 9.x** from the official OSRF repository.
   Melodic ships Gazebo 9.0; the Livox plugin needs a newer 9.x
   (its README targets 9.18+). This is an in-series upgrade — safe, and all
   ROS gazebo packages keep working.
2. Installs the Jackal + navigation ROS packages.
3. Clones [Livox-SDK/livox_laser_simulation](https://github.com/Livox-SDK/livox_laser_simulation)
   into the workspace (this provides `liblivox_laser_simulation.so` and the
   `mid360.csv` scan pattern).
4. Builds with `catkin_make` and sources the workspace in `~/.bashrc`.

## 3. Verify

```bash
roslaunch fyp_jackal_gazebo fyp_world.launch
```

You should see the arena with the bridge (right/east) and the orange-boxed
tripod (left/west), and the Jackal at the centre. Then check:

```bash
rostopic hz /front/scan        # ~50 Hz laser
rostopic hz /mid360/points     # ~10 Hz point cloud
rostopic info /mid360/points   # type: sensor_msgs/PointCloud
rosrun tf tf_echo base_link mid360_link   # should show the 38.22 deg pitch
```

## Troubleshooting

| Symptom | Fix |
|---|---|
| `livox_laser_simulation` fails to **build** with Gazebo API errors | Gazebo was not actually upgraded — `gazebo --version` should print 9.1x. Re-run step 1, then `catkin_make clean && catkin_make`. |
| Plugin builds but Gazebo prints `Failed to load plugin liblivox_laser_simulation.so` | Open a new terminal so `devel/setup.bash` is sourced (it adds the plugin dir to `GAZEBO_PLUGIN_PATH` via the catkin env). |
| `/mid360/points` exists but RViz shows nothing | The display must be **rviz/PointCloud** (legacy type), not PointCloud2. The provided `view.launch` already uses the right one. |
| Your clone of the livox plugin publishes **PointCloud2** instead (forks differ) | Change `data_type: PointCloud` → `PointCloud2` for the `mid360` source in `fyp_jackal_navigation/params/costmap_common*.yaml`, and switch the RViz display class. |
| Simulation runs far below real-time in the VM | Lower the lidar load: `roslaunch fyp_jackal_gazebo fyp_world.launch mid360_samples:=5000`. Also try `gui:=false` and use only RViz. Give the VM more cores; enable 3D acceleration in VMware settings. |
| Robot spawns but does not move / no `/odom` | Check the controllers loaded: `rostopic echo /jackal_velocity_controller/odom -n 1`. If missing, `ros-melodic-jackal-simulator`/`ros-melodic-robot-localization` are not installed. |
| TF error `mid360_link does not exist` | `robot_state_publisher` is started by our `spawn_jackal.launch` — make sure you launched `fyp_world.launch` from this repo, not Clearpath's `jackal_world.launch`. |

## Manual install (what the script does)

```bash
# Gazebo 9 latest
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install gazebo9 libgazebo9-dev

# ROS packages
sudo apt-get install ros-melodic-jackal-simulator ros-melodic-jackal-desktop \
  ros-melodic-jackal-navigation ros-melodic-navigation ros-melodic-amcl \
  ros-melodic-map-server ros-melodic-gmapping ros-melodic-robot-localization \
  ros-melodic-twist-mux ros-melodic-interactive-marker-twist-server

# Livox plugin + build
cd ~/fyp_ws/src && git clone https://github.com/Livox-SDK/livox_laser_simulation.git
cd ~/fyp_ws && catkin_make -DCMAKE_BUILD_TYPE=Release
```
