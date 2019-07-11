#!/bin/bash
curl https://install.husarnet.com/install.sh | sudo bash
echo "export ROS_IPV6=on" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://master:11311" >> ~/.bashrc
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros.list'
sudo apt update
sudo apt install -y ros-kinetic-desktop
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source /opt/ros/kinetic/setup.bash
mkdir ~/ros_workspace
mkdir ~/ros_workspace/src
cd ~/ros_workspace/src
catkin_init_workspace
cd ~/ros_workspace
catkin_make
echo "source ~/ros_workspace/devel/setup.sh" >> ~/.bashrc
source ~/ros_workspace/devel/setup.sh
cd ~/ros_workspace/src/
git clone https://github.com/husarion/tutorial_pkg.git
cd ~/ros_workspace/
rosdep install --from-paths src --ignore-src -r -y
catkin_make
