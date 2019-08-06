# access_mapping
## Installation
### Info
All installation steps below were performed and tested on a fresh Ubuntu 18.04 installation. Development for other platforms are not supported at this time. Instructions below assume a fresh Ubuntu 18.04 installation, as of August 6, 2019.
### Install
1. **Download installation scripts:** https://tinyurl.com/install-access-mapping by navigating to the link and saving the two .sh files in Downloads folder.

2. **Open a new terminal window:**
    `Ctrl+Alt+t`

3. **Navigate to Downloads folder and move installation scripts to home directory:**
    `cd Downloads`
    `mv install_first.sh install_second.sh ~ `

  `cd ~`

4. **Allow execution of both scripts:**

  `chmod +x install_first.sh`

  `chmod +x install_second.sh`

5. **Run the first installation script by typing the following into the terminal:**

  `./install_first.sh`

  Allow the  script to run to completion. Confirm installation of all required packages as the script runs by pressing enter or typing password as prompted.

  `install_first.sh` downloads ROS melodic into the system and builds OpenCV v.3.4.7 from source into a python 3 virtual environment.

6. **Run the second installation script by typing the following into the terminal:**

  `./install_second.sh`

  Allow the script to run to completion and confirm all installation requests.

  `install_second.sh` sets up a catkin workspace and builds several ROS packages for use with python 3. It also clones the `access_mapping` repository and builds it for use with python 3.

7. **Download the YOLOv3 neural network model for use with OpenCV's `dnn` neural network module:** https://tinyurl.com/yolo-files  by navigating to the link and saving the `yolo-coco` folder in the source directory of the `access_mapping` package.

  `mv <your_yolo_zip_file.zip> ~/catkin_ws_cb/src/access_mapping`

  `cd ~/catkin_ws_cb/src/access_mapping`

  `unzip <your_yolo_zip_file.zip>`

8. **Ensure proper system setup by sourcing  .setup files:**

  `rossource` (alias created during install for sourcing ROS workspace, or `source /opt/ros/melodic/setup.bash`)

  `catsource` (alias created during install for sourcing catkin workspace, or `source ~/catkin_ws_cb/devel/setup.bash`)

### Testing install
1. **Open a new terminal window and source setup files:**

   `Ctrl+alt+t`

   `rossource && catsource`

2. **Run ROS master:**

   `roscore`

3. **Open new terminal window and source setup files.**

4. **Run `gate.py` node in `access_mapping` package:**

   `cd ~/catkin_ws_cb`

   `roscd access_mapping`

   `cd nodes`

   `rosrun access_mapping gate.py --rosbag <path_to_rosbag_file>`

   Demo rosbag files are available here: ___link

5. **Open new terminal window and source setup files.**

6. **Run `object_detect.py` node in `access_mapping` package with default values:**

   `rosrun access_mapping object_detect.py --yolo ../yolo-coco`

   Note: required `--yolo` argument asks for path to folder containing YOLO configuration files, as downloaded in step 7 of install.

7. **Quitting:** Click on terminal window where `gate.py` node is running and press `'q'`. Alternatively, each node can be shutdown with keyboard-interrupt `ctrl+c`.

  