#!/bin/bash
# One-time setup for the VMware Ubuntu 18.04 / ROS Melodic machine.
#
# Assumes this repository has already been cloned into ~/fyp_ws/src, e.g.:
#   mkdir -p ~/fyp_ws/src && cd ~/fyp_ws/src
#   git clone <YOUR_GITHUB_URL> fyp_jackal
#   bash fyp_jackal/scripts/vm_setup.sh
set -e

echo "==> [0/4] Working around the Ubuntu 18.04 appstreamcli bug (crashes 'apt-get update')"
# Known bionic bug: appstreamcli segfaults in apt's Post-Invoke-Success hook.
# appstream only feeds the Ubuntu Software GUI store; safe to remove on a
# robotics VM.
if dpkg -s appstream >/dev/null 2>&1; then
    sudo apt-get purge -y appstream
fi

echo "==> [1/4] Upgrading Gazebo 9 to the latest 9.x (livox plugin needs newer than the stock 9.0)"
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y gazebo9 libgazebo9-dev

# Gazebo 9.19 needs matching ignition/sdformat libraries from the same OSRF
# repo; apt does not always pull them in, which leaves gzserver failing with
# "undefined symbol: ...SetUserAgent...". Upgrade whatever is installed.
IGN_PKGS=$(dpkg -l | awk '/^ii  (libignition|libsdformat)/{print $2}')
if [ -n "$IGN_PKGS" ]; then
    sudo apt-get install -y --only-upgrade $IGN_PKGS
fi

echo "==> [2/4] Installing ROS packages"
sudo apt-get install -y \
    ros-melodic-jackal-simulator \
    ros-melodic-jackal-desktop \
    ros-melodic-jackal-navigation \
    ros-melodic-navigation \
    ros-melodic-amcl \
    ros-melodic-map-server \
    ros-melodic-gmapping \
    ros-melodic-robot-localization \
    ros-melodic-twist-mux \
    ros-melodic-interactive-marker-twist-server

echo "==> [3/4] Cloning the Livox Gazebo plugin"
cd ~/fyp_ws/src
if [ ! -d livox_laser_simulation ]; then
    git clone https://github.com/Livox-SDK/livox_laser_simulation.git
fi

echo "==> [4/4] Building the workspace"
cd ~/fyp_ws
source /opt/ros/melodic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release

if ! grep -q "fyp_ws/devel/setup.bash" ~/.bashrc; then
    echo "source ~/fyp_ws/devel/setup.bash" >> ~/.bashrc
fi

echo ""
echo "Done. Open a NEW terminal (or 'source ~/fyp_ws/devel/setup.bash') and run:"
echo "  roslaunch fyp_jackal_gazebo fyp_world.launch"
echo "  roslaunch fyp_jackal_navigation nav.launch      (second terminal)"
echo "  roslaunch fyp_jackal_navigation view.launch     (third terminal)"
