#!/bin/bash

# Name: install_ros.sh
# Author: Joshua Eckels
# Contact: eckelsjd@rose-hulman.edu
# Date: August 4, 2019
# System: Ubuntu 18.04
# ROS: Melodic
# Doc: Shell script to install ROS Melodic 
# References: http://wiki.ros.org/melodic/Installation/Ubuntu
#	      

# variables
ROS_version=melodic
shell_setup=~/.bashrc

# setup sources
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# update system
sudo apt update

# install
sudo apt install ros-$ROS_version-desktop-full

# rosdep
sudo rosdep init
rosdep update

# setup environment
echo -e "\n# Setup ROS-$ROS_version workspace" >> $shell_setup
echo "alias rossource='source /opt/ros/melodic/setup.bash'" >> $shell_setup
source $shell_setup
rossource

# install package-building dependencies
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
