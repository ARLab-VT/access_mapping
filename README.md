# access_mapping
## Project
### Motivation
The `access_mapping` repository  is designed as a ROS (Robotic Operating System) package to be built inside a catkin workspace as setup in the installation section below. The goal of this package is to provide ROS nodes capable of identifying potential barriers or constraints in the environment for the purpose of autonomous robot or handicapped/accessibility navigation and traversability. A long term use of these capabilities would be the potential to generate global maps of the environment (using existing Simultaneous Localization and Mapping (SLAM)-based approaches) that are annotated and labeled with different barriers. These global maps, that have been annotated with the location of barriers such as stairs, doors, and handicap-accessible blue buttons, could then be exploited for front-end use, allowing users to plan an optimized path from point A to point B that accounts for all personally potential barriers, whether the user is a handicapped wheelchair user or a small autonomous rover. 
### Context
All work on the `access_mapping` package is part of ongoing academic research at the Assistive Robotics Lab at Virginia Tech. The idea for the project stemmed from the social engineering principles surrounding handicapped navigation on the Virginia Tech campus and grew out of its applicability to both fields of autonomous navigation and social engineering. All research was funded by the National Science Foundation (NSF) as part of a Research Experience for Undergraduates (REU) program at Virginia Tech in Automotive Engineering.


## System Setup
### System
All installation steps below were performed and tested on a fresh Ubuntu 18.04 installation. Support for other platforms is not available at this time. Instructions below assume a fresh Ubuntu 18.04 installation, and it is advised to gain access to one if not already available. All testing on installation was performed on a Ubuntu 18.04 installation dual-booted with Windows 10.
**See [dual-boot](https://itsfoss.com/install-ubuntu-1404-dual-boot-mode-windows-8-81-uefi/) options and instructions.**
**[EaseUS](https://www.easeus.com/?ad&gclid=EAIaIQobChMI1aiA9rnx4wIVEp6fCh3-7Qe3EAAYASAAEgJ06vD_BwE) is recommended for disk partition management.**
**[WinDirStat](https://windirstat.net/) is recommended for hard disk space management.**
A hard disk partition of 30GB or more will be sufficient for installation of Ubuntu and all required software for the `access_mapping` package.
### Required software
All required software will be installed with associated dependencies in the installation section below. The `access_mapping` package is being developed as a possible plug-in map-annotator for the existing SLAM package: `rtabmap_ros`. See their ROS wiki page [here](http://wiki.ros.org/rtabmap_ros).
#### ROS Melodic
Procedure to install ROS Melodic version was followed on the [ROS wiki](http://wiki.ros.org/melodic/Installation/Ubuntu).
#### Catkin build tools
Tutorials on the ROS wiki setup a catkin workspace with the legacy `catkin_make` tools, as seen [here](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).
However, for this installation, the still-in-beta [catkin build tools](https://catkin-tools.readthedocs.io/en/latest/index.html) will be used to build the workspace.
#### Python 3 virtual environment
A python 3 virtual environment will be set up to install all required python 3 packages and modules. Learn more about python [virtual environments](https://realpython.com/python-virtual-environments-a-primer/).
#### OpenCV 3.4.7
In order to utilize open source computer vision software (OpenCV)'s `dnn` neural network module for object detection, OpenCV 3.4.7 will be installed inside the python 3 virtual environment. This tutorial on [pyimagesearch.com](https://www.pyimagesearch.com/2018/05/28/ubuntu-18-04-how-to-install-opencv/
) was followed to download and compile OpenCV 3.4.7 from source.
#### ZED camera SDK
This is not a requirement of the `access_mapping` package. But all rosbag files used by `access_mapping` should include (at least) the following ZED topics:
```bash
/zed/zed_node/rgb/image_rect_color
/zed/zed_node/rgb/camera_info
/zed/zed_node/depth/depth_registered
/zed/zed_node/depth/camera_info
/zed/zed_node/odom
/tf
/tf_static
```
A ZED camera with its Software Development Kit (SDK) is required to record all compatible rosbag files. See the ZED [website](https://www.stereolabs.com/zed/) for more information, as well as the Access_SLAMNav_deployment/mapper [repository](https://github.com/ARLab-VT/Access_SLAMNav_deployment/tree/master/mapper), for more information on recording rosbag files with a ZED camera. 


## Installation
### Info
All installation steps below were performed and tested on a fresh Ubuntu 18.04 installation. Development for other platforms are not supported at this time. All software as described above will be installed.
### Install
1. **Open a new terminal window:** `Ctrl+Alt+t`
2. **Get installation scripts:** Navigate to home directory and clone from `Access_SLAMNav_deployment` repository by typing the following commands.
```bash
sudo apt update
sudo apt -y install git
cd ~
git clone https://github.com/ARLab-VT/Access_SLAMNav_deployment.git
cd Access_SLAMNav_deployment.git/slam_annotator
```
3. **Allow execution of both install scripts:** 
```bash
chmod +x install_first.sh
chmod +x install_second.sh
```
4. **Run the first installation script:**
```bash
./install_first.sh
```
Allow the  script to run to completion. Confirm installation of all required packages as the script runs by pressing enter or typing password as prompted.`install_first.sh` downloads ROS Melodic and builds OpenCV 3.4.7 from source into a python 3 virtual environment.

5. **Open a new terminal window:**`Ctrl+Alt+t`

6. **Run the second installation script by entering the following into the new terminal:**
```bash
./install_second.sh
```
Allow the script to run to completion and confirm all installation requests. `install_second.sh` sets up a catkin workspace and builds several ROS packages for use with python 3. It also clones the `access_mapping` repository and builds it for use with python 3.

7. **Download the YOLOv3 neural network model for use with OpenCV's `dnn` neural network module.** Navigate to the link below and save the `yolo-coco` folder in the source directory of the `access_mapping` package. Download [yolo-coco]( https://tinyurl.com/yolo-files) as a .zip file.
```bash
cd <your_downloads_folder>
mv <your_yolo_file.zip> ~/catkin_ws_cb/src/access_mapping
unzip <your_yolo_file.zip>
rm <your_yolo_file.zip>   
```
Example:
```bash
cd ~/Downloads
mv yolo-coco.zip ~/catkin_ws_cb/src/access_mapping
cd ~/catkin_ws_cb/src/access_mapping
unzip yolo-coco.zip
rm yolo-coco.zip
```

8. **Ensure proper system setup by sourcing  .setup files:**
`rossource` (alias created during install for sourcing ROS workspace, or `source /opt/ros/melodic/setup.bash`)
`catsource` (alias created during install for sourcing catkin workspace, or `source ~/catkin_ws_cb/devel/setup.bash`)
**Optional:** add the two sourcing commands above to your `.bashrc` file, (or other shell setup file) to run every time your terminal opens:
```bash
echo -e "\n# Source ROS and catkin workspace" >> ~/.bashrc
echo "rossource && catsource" >> ~/.bashrc
```


## Deployment
1. **Open a new terminal window and source setup files:**
```bash
rossource
catsource
```
2. **Run ROS master:**
```bash
roscore
```

3. **Open another terminal window and source setup files.**

4. **Run `gate.py` node in `access_mapping` package with default values:**
```bash
cd ~/catkin_ws_cb
roscd access_mapping
cd nodes
rosrun access_mapping gate.py --rosbag <path_to_rosbag_file>
```
Demo rosbag file is available here: [link]. Move the rosbag file to `<path_to_rosbag_file>` as mentioned above.

5. **Open new terminal window and source setup files.**

6. **Run `object_detect.py` node in `access_mapping` package with default values:**
```
rosrun access_mapping object_detect.py --yolo ../yolo-coco
```
Note: required `--yolo` argument asks for path to folder containing YOLO configuration files, as downloaded in step 6 of install section.

7. **Quitting:** Click on terminal window where `gate.py` node is running and press `'q'`. Alternatively, each node can be shutdown with keyboard-interrupt `ctrl+c`.

8. Please see project detail in Contribute section for more info on each node and optional command-line flags.


## Demo
### Object detection
Following the default deployment method described above, the output of the `object_detect` node is shown below.

[video not currently available]

Parameters can be changed to detect multiple kinds of objects as well:

[video not currently available]

### Locate in 3D global space
The following output of the `object_detect` node gives the location of a detected object in global coordinates:

[output not currently available]

### Occupancy grid
The 3D global coordinates of objects can be aggregated in a list and utilized in many ways. The output below shows the projection of these points onto a 2D occupancy grid for data visualization:

[output not currently available]


## Contribute
### Project detail
The goal of this section is to provide documentation on overall usage of the `access_mapping` package.
#### Nodes
#### Launch files
#### Message files

### Current bugs
#### Step-mode
#### Global transform
#### Occupancy grid

### Features not currently supported
#### System support
#### Launch files
#### Global transform
#### Data aggregation and export
#### Data visualization
#### Integration with rtabmap_ros

### Future work
#### Alternate neural networks
#### Training neural network
#### User input
#### Rosbag v. live recording
#### Alternate recording setups
#### GUI for easy visualization
#### Non-visual recognizers
#### Multi-session global mapping
#### Front-end 


## References

