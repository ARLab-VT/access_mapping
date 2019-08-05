#!/bin/bash

# Name: install_cv_bridge.sh
# Author: Joshua Eckels
# Contact: eckelsjd@rose-hulman.edu
# Date: August 4, 2019
# System: Ubuntu 18.04
# ROS: Melodic
# Doc: Shell script to build cv_bridge,tf,tf2_ros for Python 3
# Prerequisites: Virtual environment for Python 3 as setup in install_cv.sh
#		 Catkin workspace as setup in install_catkin.sh
# References: https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674
#	      https://github.com/ros/geometry2/issues/259
#	      https://github.com/ros/geometry2/issues/293

# variables
ROS_version=melodic
shell_setup=~/.bashrc
catkin_ws=catkin_ws_cb
pkg=access_mapping
virtual_env=cv

# install required ROS packages to use Python 3
workon $virtual_env
sudo apt-get install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg

# build cv_bridge for Python 3
sudo apt-get install python-catkin-tools python3-dev python3-numpy
cd ~/$catkin_ws
catkin config -DPYTHON_EXECUTABLE=~/.virtualenvs/$virtual_env/bin/python -DPYTHON_INCLUDE_DIR=~/.virtualenvs/$virtual_env/include/python3.6m 
catkin config --no-install
cd src
git clone -b $ROS_version https://github.com/ros-perception/vision_opencv.git
cd ~/$catkin_ws
catkin build cv_bridge

# source setup file
# echo "alias cvsource='source ~/$catkin_ws/install/setup.bash --extend'" >> $shell_setup
# source $shell_setup
# source ~/$catkin_ws/install/setup.bash --extend # cvsource

# install more dependencies (if not already installed)
workon $virtual_env
pip install catkin_pkg pyyaml empy rospkg numpy

# build tf and tf2_py from source for Python 3
cd ~/$catkin_ws/src
git clone https://github.com/ros/geometry
git clone https://github.com/ros/geometry2
cd ..
catkin build
source devel/setup.bash # catsource
