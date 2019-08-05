#!/bin/bash

# Name: install_cv.sh
# Author: Joshua Eckels
# Contact: eckelsjd@rose-hulman.edu
# Date: August 4, 2019
# System: Ubuntu 18.04
# OpenCV: Version 3.4.7
# Doc: Shell script to install OpenCV in a Python 3 virtual environment
# References: https://www.pyimagesearch.com/2018/05/28/ubuntu-18-04-how-to-install-opencv/
#	      https://realpython.com/python-virtual-environments-a-primer/

# variables
version=3.4.7
shell_setup=~/.bashrc
virtual_env=cv

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
wget -O opencv.zip https://github.com/opencv/opencv/archive/$version.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/$version.zip
unzip opencv.zip
unzip opencv_contrib.zip
mv opencv-$version opencv
mv opencv_contrib-$version opencv_contrib

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
mkvirtualenv $virtual_env -p python3
workon $virtual_env

# install numpy in virtual environment
pip install numpy
workon $virtual_env

# configure OpenCV with CMake and compile
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

# check install
# pkg-config --modversion opencv 

# sym-link OpenCV bindings into virtual environment
cd $(dirname $(find /usr -name cv2.cpython-36m-x86_64-linux-gnu.so))
sudo mv cv2.cpython-36m-x86_64-linux-gnu.so cv2.so
cd ~/.virtualenvs/$virtual_env/lib/python3.6/site-packages/
ln -s $(find /usr -name cv2.so) cv2.so

# optionally remove files/folders
# cd ~
# rm opencv.zip opencv_contrib.zip
# rm -rf opencv opencv_contrib

