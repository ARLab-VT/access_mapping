#!/bin/bash

# File: 	install.sh
# Author: 	Joshua Eckels
# Contact: 	eckelsjd@rose-hulman.edu
# Date: 	August 4, 2019
# System: 	Ubuntu 18.04
# ROS: 		Melodic
# Catkin:	catkin_tools
# OpenCV: 	Version 3.4.7
# Doc: 		Shell script to setup workspace for use with ROS package access_mapping.
#		Assumes a fresh install of Ubuntu 18.04. Script was built and tested
#		on Ubuntu 18.04 dual-boot with Windows 10. See tutorials for dual-boot
#		setup. Note: catkin workspace was setup using catkin_tools package; this is
#		different than procedure followed on the ROS wiki. Please see references
#		below. Project undertaken as part of NSF-REU program at Virginia Tech.
# References: 	https://www.pyimagesearch.com/2018/05/28/ubuntu-18-04-how-to-install-opencv/
#		https://github.com/opencv/opencv/issues/14868
#	      	https://realpython.com/python-virtual-environments-a-primer/
#		http://wiki.ros.org/melodic/Installation/Ubuntu
#		https://catkin-tools.readthedocs.io/en/latest/quick_start.html
#		https://catkin-tools.readthedocs.io/en/latest/history.html
#		https://answers.ros.org/question/243192/catkin_make-vs-catkin-build/
#		https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674
#		https://github.com/ros/geometry2/issues/259
#		https://github.com/ros/geometry2/issues/293
#		https://itsfoss.com/install-ubuntu-1404-dual-boot-mode-windows-8-81-uefi/
#		https://github.com/git-lfs/git-lfs/wiki/Installation
#		https://git-lfs.github.com/
#		https://towardsdatascience.com/uploading-large-files-to-github-dbef518fa1a

# variables
cv_version=3.4.7	# desired version of OpenCV
ros_version=melodic	# desired version of ROS
shell_setup=~/.bashrc	# absolute path to shell setup rc file
virtual_env=cv		# desired name of python 3 virtual environment
catkin_ws=catkin_ws_cb	# desired name of catkin workspace
pkg=access_mapping	# name of ROS package to build

############################################################################
# SETUP CATKIN WORKSPACE (USING CATKIN_TOOLS)
############################################################################
# install catkin_tools with apt-get
# note: This should be done separately from the tutorials on ROS wiki.
#       catkin_tools work seperately from default catkin tools (catkin_make,etc.)
#       See https://catkin-tools.readthedocs.io/en/latest/history.html
#       See https://answers.ros.org/question/243192/catkin_make-vs-catkin-build/
#       for options in combining catkin build and catkin_make (not recommended).
source $shell_setup
source /usr/local/bin/virtualenvwrapper.sh
cd ~
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python-catkin-tools
sudo apt-get install python3-pip python3-yaml
sudo apt-get install python3-dev python3-numpy

############################################################################
# BUILD CV_BRIDGE, GEOMETRY, GEOMETRY2, ACCESS_MAPPING for ROS in PYTHON3
############################################################################
# install required ROS packages to use Python 3 in ROS
# note: These steps were taken to resolve issues in running python3 scripts
#       with python2 dependencies: cv_bridge, tf, tf2_ros.
#       Similar steps might be taken to build other python2 packages for use
#       in python3 scripts or nodes.
#       See https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674
#       See https://github.com/ros/geometry2/issues/259
#       See https://github.com/ros/geometry2/issues/293
# setup catkin workspace
rossource
cd ~
mkdir -p $catkin_ws/src
cd $catkin_ws/src
git clone https://github.com/ros/geometry
git clone https://github.com/ros/geometry2
git clone -b $ros_version https://github.com/ros-perception/vision_opencv.git
git clone https://github.com/eckelsjd/access_mapping.git
cd ..
workon $virtual_env
pip install rospkg catkin_pkg pyyaml empy
catkin config -DPYTHON_EXECUTABLE=~/.virtualenvs/$virtual_env/bin/python -DPYTHON_INCLUDE_DIR=~/.virtualenvs/$virtual_env/include/python3.6m
catkin config --no-install
catkin build
source devel/setup.bash

# source catkin workspace
echo "alias catsource='source ~/$catkin_ws/devel/setup.bash'" >> $shell_setup
source $shell_setup
rossource
catsource
