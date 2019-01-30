#!/bin/bash
. /opt/ros/kinetic/setup.bash
cd ws/robot_ws
apt update
rosdep install --from-paths src --ignore-src -r -y
colcon build --build-base armhf_build --install-base armhf_install
colcon bundle --build-base armhf_build --install-base armhf_install --bundle-base armhf_bundle --apt-sources-list /opt/cross/apt-sources.yaml --apt-package-blacklist src/blacklist
exit