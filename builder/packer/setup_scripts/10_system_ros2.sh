#!/bin/bash
set -eu

# Copyright 2019-2022 Tiryoh
# https://github.com/Tiryoh/ros2_setup_scripts_ubuntu
# Licensed under the Apache License, Version 2.0
#
# REF: https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
# by Open Robotics, licensed under CC-BY-4.0
# source: https://github.com/ros2/ros2_documentation

CHOOSE_ROS_DISTRO=humble
INSTALL_PACKAGE=ros-base
TARGET_OS=jammy

# Check OS version
if ! which lsb_release > /dev/null ; then
	sudo apt-get update
	sudo apt-get install -y curl lsb-release
fi

if [[ "$(lsb_release -sc)" == "$TARGET_OS" ]]; then
	echo "OS Check Passed"
else
	printf '\033[33m%s\033[m\n' "=================================================="
	printf '\033[33m%s\033[m\n' "ERROR: This OS (version: $(lsb_release -sc)) is not supported"
	printf '\033[33m%s\033[m\n' "=================================================="
	exit 1
fi

if ! dpkg --print-architecture | grep -q 64; then
	printf '\033[33m%s\033[m\n' "=================================================="
	printf '\033[33m%s\033[m\n' "ERROR: This architecture ($(dpkg --print-architecture)) is not supported"
	printf '\033[33m%s\033[m\n' "See https://www.ros.org/reps/rep-2000.html"
	printf '\033[33m%s\033[m\n' "=================================================="
	exit 1
fi

# Install
printf "\n\n################ ADDING LINUX DEPENDENCIES ###########################\n\n"
sudo apt-get update
sudo apt-get install -y software-properties-common
sudo add-apt-repository -y universe
sudo apt-get install -y curl gnupg2 lsb-release build-essential

printf "\n\n################ ADDING ROS APT REPO ###########################\n\n"

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update

printf "\n\n################ ROSBASE INSTALL ###########################\n\n"

sudo apt-get install -y ros-$CHOOSE_ROS_DISTRO-$INSTALL_PACKAGE
sudo apt-get install -y python3-argcomplete 
sudo apt-get install -y python3-colcon-common-extensions
sudo apt-get install -y python3-rosdep python3-vcstool # https://index.ros.org/doc/ros2/Installation/Linux-Development-Setup/
[ -e /etc/ros/rosdep/sources.list.d/20-default.list ] ||

printf "\n\n################ ROSDEP INIT & UPDATE ###########################\n\n"

sudo rosdep init
rosdep update
grep -F "source /opt/ros/$CHOOSE_ROS_DISTRO/setup.bash" ~/.bashrc ||
echo "source /opt/ros/$CHOOSE_ROS_DISTRO/setup.bash" >> ~/.bashrc


echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc

set +u

printf "\n\n################ INSTALLING USMA RRC CONFIG ###########################\n\n"

# Add from @pratman - https://github.com/westpoint-robotics/os-setup/blob/master/ubuntu22_ros2.md
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ros/ros_tutorials.git -b humble-devel
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
AMENT_TRACE_SETUP_FILES=1 source /opt/ros/humble/setup.bash 
colcon build
. install/local_setup.bash
echo "source $HOME/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "success installing ROS2 $CHOOSE_ROS_DISTRO"
echo "Run 'source /opt/ros/$CHOOSE_ROS_DISTRO/setup.bash'"

