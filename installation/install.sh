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
# INSTALL OPENCV (FROM SOURCE)
############################################################################
# update system
sudo apt-get update
sudo apt-get upgrade

# install developer tools
sudo apt-get install build-essential cmake unzip pkg-config

# install OpenCV-specific prerequisites
sudo apt-get install libjpeg-dev libpng-dev libtiff-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get install libxvidcore-dev libx264-dev

# install GTK and other good libraries for OpenCV
sudo apt-get install libgtk-3-dev
sudo apt-get install libatlas-base-dev gfortran

# install Python 3 headers and libraries
sudo apt-get install python3-dev

# download opencv and opencv_contrib source code
cd ~
wget -O opencv.zip https://github.com/opencv/opencv/archive/$cv_version.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/$cv_version.zip
unzip opencv.zip
unzip opencv_contrib.zip
mv opencv-$cv_version opencv
mv opencv_contrib-$cv_version opencv_contrib

# install pip
wget https://bootstrap.pypa.io/get-pip.py
sudo python3 get-pip.py

# install virtualenv and virtualenvwrapper
sudo pip install virtualenv virtualenvwrapper
sudo rm -rf ~/get-pip.py ~/.cache/pip

# setup python3 virtual environment
echo -e "\n# virtualenv and virtualenvwrapper" >> $shell_setup
echo "export WORKON_HOME=$HOME/.virtualenvs" >> $shell_setup
echo "export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3" >> $shell_setup
echo "source /usr/local/bin/virtualenvwrapper.sh" >> $shell_setup
source $shell_setup
source /usr/local/bin/virtualenvwrapper.sh
mkvirtualenv $virtual_env -p python3
workon $virtual_env

# install numpy in virtual environment
pip install numpy

# Reference wahyubram82' answer: https://github.com/opencv/opencv/issues/14868
# Edit the file located at opencv/modules/core/include/opencv2/core/private.hpp
# edit the line:
# include <Eigen/Core>
# to be:
# include <eigen3/Eigen/Core>
sed -i 's!include <Eigen/Core>!include <eigen3/Eigen/Core>!' ~/opencv/modules/core/include/opencv2/core/private.hpp

# configure OpenCV with CMake and compile; will take some time
cd ~/opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr/local \
	-D INSTALL_PYTHON_EXAMPLES=ON \
	-D INSTALL_C_EXAMPLES=OFF \
	-D OPENCV_ENABLE_NONFREE=ON \
	-D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
	-D PYTHON_EXECUTABLE=~/.virtualenvs/$virtual_env/bin/python \
	-D BUILD_EXAMPLES=ON ..
make

# install
sudo make install
sudo ldconfig

# check the install (optional)
# pkg-config --modversion opencv

# sym-link OpenCV bindings into virtual environment
cd $(dirname $(find /usr -name cv2.cpython-36m-x86_64-linux-gnu.so))
sudo mv cv2.cpython-36m-x86_64-linux-gnu.so cv2.so
cd ~/.virtualenvs/$virtual_env/lib/python3.6/site-packages/
ln -s $(find /usr -name cv2.so) cv2.so
deactivate

# remove files/folders (optional)
# cd ~
# rm opencv.zip opencv_contrib.zip
# rm -rf opencv opencv_contrib

############################################################################
# INSTALL ROS (SEE ROS WIKI)
############################################################################
# setup sources
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# update system
sudo apt update

# install
sudo apt install ros-$ros_version-desktop-full

# rosdep
sudo rosdep init
rosdep update

# source ROS environment
echo -e "\n# Setup ROS-$ros_version workspace" >> $shell_setup
echo "alias rossource='source /opt/ros/$ros_version/setup.bash'" >> $shell_setup
source $shell_setup
rossource

# install package-building dependencies
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

############################################################################
# SETUP CATKIN WORKSPACE (USING CATKIN_TOOLS)
############################################################################
# install catkin_tools with apt-get
# note: This should be done separately from the tutorials on ROS wiki.
#	catkin_tools work seperately from default catkin tools (catkin_make,etc.)
#	See https://catkin-tools.readthedocs.io/en/latest/history.html
#	See https://answers.ros.org/question/243192/catkin_make-vs-catkin-build/
#	for options in combining catkin build and catkin_make (not recommended).
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python-catkin-tools

# setup catkin workspace
source /opt/ros/$ros_version/setup.bash # rossource
cd ~
mkdir -p $catkin_ws/src
cd $catkin_ws
catkin init
catkin build

# source catkin workspace
echo "alias catsource='source ~/$catkin_ws/devel/setup.bash'" >> $shell_setup
source $shell_setup
catsource

############################################################################
# BUILD CV_BRIDGE, GEOMETRY, GEOMETRY2 for ROS in PYTHON3
############################################################################
# install required ROS packages to use Python 3 in ROS
# note:	These steps were taken to resolve issues in running python3 scripts
#	with python2 dependencies: cv_bridge, tf, tf2_ros.
#	Similar steps might be taken to build other python2 packages for use
#	in python3 scripts or nodes.
#	See https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674
#	See https://github.com/ros/geometry2/issues/259
#	See https://github.com/ros/geometry2/issues/293
workon $virtual_env
sudo apt-get install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg
sudo pip install pyyaml empy 

# build cv_bridge for Python 3
sudo apt-get install python-catkin-tools python3-dev python3-numpy
cd ~/$catkin_ws
catkin config -DPYTHON_EXECUTABLE=~/.virtualenvs/$virtual_env/bin/python -DPYTHON_INCLUDE_DIR=~/.virtualenvs/$virtual_env/include/python3.6m
catkin config --no-install
cd src
git clone -b $ros_version https://github.com/ros-perception/vision_opencv.git
cd ~/$catkin_ws
catkin build cv_bridge

# build tf and tf2_ros from source for Python 3
cd ~/$catkin_ws/src
git clone https://github.com/ros/geometry
git clone https://github.com/ros/geometry2
cd ..
catkin build
source devel/setup.bash # catsource

############################################################################
# SETUP AND BUILD ACCESS_MAPPING PACKAGE FROM GIT
############################################################################
# use git-lfs to store yolov3.weights file on github repository
# See https://git-lfs.github.com/
# See https://github.com/git-lfs/git-lfs/wiki/Installation
echo -e "\n# Source ROS and catkin environments" >> $shell_setup
echo "rossource && catsource" >> $shell_setup
source $shell_setup
workon $virtual_env

# setup and install git-lfs (for large files)
sudo apt-get install git-lfs

# build access_mapping package
cd ~/$catkin_ws/src
git clone https://github.com/eckelsjd/access_mapping.git
git lfs install
cd ..
catkin build access_mapping
source devel/setup.bash
