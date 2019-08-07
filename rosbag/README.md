# Rosbag directory
This is an example directory to store rosbag files.
Example:

```bash
source /opt/ros/melodic/setup.bash # rossource
source ~/catkin_ws_cb/devel/setup.bash # catsource
roscd access_mapping
cd nodes

# make sure roscore is running in separate terminal

# run gate.py node
rosrun access_mapping gate.py --rosbag ../rosbag/example_rosbag.bag

```



