#!/bin/bash
set -x
#set -e

echo "----------- start ------------"

APT_GET="sudo apt-get"

echo "deb http://packages.ros.org/ros/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros-focal.list

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

$APT_GET update

$APT_GET install -y ros-noetic-ros-base
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

$APT_GET install -y python3-rosdep python3-rosinstall-generator python3-wstool python3-rosinstall build-essential cmake

sudo rosdep init

rosdep update

$APT_GET install -y ros-noetic-mavros ros-noetic-mavros-extras

wget http://wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh

chmod +x install_geographiclib_datasets.sh

sudo bash ./install_geographiclib_datasets.sh

rm install_geographiclib_datasets.sh