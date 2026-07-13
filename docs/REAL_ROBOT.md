# Running on the Real Jackal

Phase 2: same navigation stack, real sensors. Everything below happens on the
**Jackal's onboard PC** (Ubuntu 18.04 / Melodic) unless stated otherwise.

## 0. What stays the same

- Jackal base bring-up (odometry, `base_link` TF, `cmd_vel`) — your existing setup.
- Hokuyo publishing `/front/scan` — your existing setup. Verify:
  `rostopic hz /front/scan` and note the frame:
  `rostopic echo /front/scan/header/frame_id -n 1`.
  If it is not `front_laser`, edit the `scan:` block in
  `params/costmap_common_real.yaml` to match.
- AMCL + move_base configs — identical to sim, loaded by `nav_real.launch`.
- Your existing map of the environment — passed as `map_file`.

## 1. Get the code onto the robot

```bash
mkdir -p ~/fyp_ws/src && cd ~/fyp_ws/src
git clone https://github.com/NaimYz1/LastTask.git fyp_jackal
# Do NOT clone livox_laser_simulation here - it is only for Gazebo.
cd ~/fyp_ws && catkin_make
echo "source ~/fyp_ws/devel/setup.bash" >> ~/.bashrc
```

## 2. Mid-360 driver (livox_ros_driver2)

The Mid-360 is only supported by **livox_ros_driver2** (not the old
livox_ros_driver), which needs **Livox-SDK2** first:

```bash
# SDK2
git clone https://github.com/Livox-SDK/Livox-SDK2.git ~/Livox-SDK2
cd ~/Livox-SDK2 && mkdir build && cd build
cmake .. && make -j4 && sudo make install

# driver (note: cloned with the exact folder name the build script expects)
cd ~/fyp_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2
./build.sh ROS1
```

If `build.sh ROS1` fails on Melodic (it officially targets Noetic), paste the
error — there are known small fixes.

### Network

The Mid-360 talks over Ethernet with **fixed IPs**:

- Lidar IP: `192.168.1.1XX` where `XX` = last two digits of the serial number
  on the lidar's sticker.
- Give the Jackal's Ethernet port a static IP on that subnet, e.g.:
  `sudo ip addr add 192.168.1.50/24 dev eth0` (make it permanent via
  netplan/interfaces later). Test with `ping 192.168.1.1XX`.
- Edit `livox_ros_driver2/config/MID360_config.json`: set every `host_..._ip`
  field to `192.168.1.50` and the lidar `ip` to your `192.168.1.1XX`.

### Run and verify

```bash
roslaunch livox_ros_driver2 rviz_MID360.launch   # publishes /livox/lidar + opens rviz
```

You should see the cloud. For headless use later, use `msg_MID360.launch` but
make sure `xfer_format` is **0** (PointCloud2) in the launch file — the costmap
needs PointCloud2, not the Livox custom message. Check:
`rostopic info /livox/lidar` → `sensor_msgs/PointCloud2`, and
`rostopic echo /livox/lidar/header/frame_id -n 1` → `livox_frame`.

## 3. Mid-360 TF (measure the mount!)

`nav_real.launch` publishes a static transform `base_link -> livox_frame`.
Measure on the real robot and pass the values:

- `mid360_z`: height of the Mid-360 optical centre above the floor **minus
  0.065 m** (base_link sits ~0.065 m above the floor).
- `mid360_x`: forward offset from the robot centre (negative = behind centre).
- `mid360_pitch`: nose-down tilt in **radians** (38.22° = 0.6671).

**Calibration check:** with the driver and TF running, open RViz (fixed frame
`base_link`), add the `/livox/lidar` PointCloud2 display, and look at a wall
and the floor: floor points must lie flat near z = 0 (not a tilted plane) and
walls must be vertical. Tune `mid360_pitch` until they are.

## 4. Run navigation

```bash
roslaunch fyp_jackal_navigation nav_real.launch \
    map_file:=/path/to/your_map.yaml \
    mid360_z:=<measured> mid360_pitch:=<measured>
```

From your laptop/VM on the same network you can run RViz remotely:

```bash
export ROS_MASTER_URI=http://<jackal_ip>:11311
export ROS_IP=<your_vm_ip>
roslaunch fyp_jackal_navigation view.launch
```

(Enable the "Mid360 Real (PointCloud2)" display, disable the sim one.
Clocks must roughly agree — install `chrony` if TF complains about time.)

Then: **2D Pose Estimate** on the map, small test goal first, then the bridge
and tripod runs.

## 5. Real-world checklist

- [ ] `clearance` = measured robot height (top of Mid-360) + ~0.10 m.
- [ ] Real bridge clearance comfortably above the robot? If it is LOWER than
      the robot, the correct outcome is the robot *refusing* to drive under.
- [ ] First bridge run: hand on the e-stop, walk alongside.
- [ ] Floor marks in the costmap while driving? Raise
      `mid360/min_obstacle_height` (0.15 → 0.20) in `costmap_common_real.yaml`.
- [ ] Sparse/flickery marking on the tripod? The Mid-360 pattern is
      non-repetitive; marks accumulate over ~0.5–1 s. Approach slower, or ask
      me to add a small cloud-accumulator node.
