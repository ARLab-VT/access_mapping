#!/bin/bash

# Name: install_catkin.sh
# Author: Joshua Eckels
# Contact: eckelsjd@rose-hulman.edu
# Date: August 4, 2019
# System: Ubuntu 18.04
# ROS: Melodic
# Doc: Shell script to install catkin_tools and setup catkin workspace
# References: https://catkin-tools.readthedocs.io/en/latest/installing.html
#	      

# variables
ROS_version=melodic
shell_setup=~/.bashrc
catkin_ws=catkin_ws_cb
pkg=access_mapping

# insall catkin_tools with apt-get
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python-catkin-tools

# setup catkin workspace
source opt/ros/$ROS_version/setup.bash
cd ~
mkdir -p $catkin_ws/src
cd $catkin_ws
catkin init

# create package inside workspace
cd src
catkin create pkg $pkg
cd ..
catkin build

# source catkin workspace
echo "alias catsource='source ~/$catkin_ws/devel/setup.bash'" >> $shell_setup
source $shell_setup
catsource
