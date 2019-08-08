# Launch files

Current launch file does not run due to the following error:

`object_detect.py` node requires python 3 within `cv` virtual environment.

`gate.py` node requires python 2 within ROS environment under `/opt/ros/melodic` path.

Needs fixing.

Usage:

```bash
roslaunch SLAM_annotator.launch rosbag:=../rosbag/example.bag yolo:=~/catkin_ws_cb/src/access_mapping/yolo-coco
```

