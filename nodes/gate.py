#!/usr/bin/env python
"""
gate.py:            Python 2 script for running a ROS node
Contributors:       Joshua Eckels, Enea Dushaj
Organization:       Virginia Tech NSF-REU
Date:               August 7, 2019
Note:               The object_detect ROS node only works in tandem with this gate.py node in 
                    the access_mapping ROS package. This gate.py node must be run first to
                    initialize a rosbag for playback. This node should be executed in an 
                    environment with a python 2 interpreter.

Doc:    This node opens a rosbag file from a specified path and iterates through each message,
        building up a queue of 6 ROS topic messages to publish all at once. The node decides when
        to publish a queue of messages based on the availability of the messages and the
        current status of all connected nodes. The role of the gate node is to carry out the same
        functionality as the 'rosbag play' command-line tool, with the added control over
        the speed and timing of publishing messages contained in the rosbag.

Command-line arguments:
    -r  /  --rosbag     : base path to rosbag file [REQUIRED]
    -o  /  --objects    : list of objects being tracked by object_detect node
                          format: --objects person,tvmonitor,chair,etc
    -z  /  --topics     : list of the following 6 required ROS topics to listen to.
                          format: img,inf,depth,infd,tf,odom
                          default topics for Zed camera are shown below, but may be replaced
                          with equivalent topics for other systems.
                          (note: must be in the order indicated)
                          rgb image topic:    /zed/zed_node/rgb/image_rect_color
                          rgb camera info:    /zed/zed_node/rgb/camera_info
                          depth image topic:  /zed/zed_node/depth/depth_registered
                          depth camera info:  /zed/zed_node/depth/camera_info
                          transform frame:    /tf
                          camera odometry:    /zed/zed_node/odom
    -s  /  --speed      : value to toggle the loop playback speed of this node

Usage: rosrun access_mapping gate.py --rosbag ../rosbag/example.bag [-o person,chair] 
        [-z img,inf,depth,infd,tf,odom] [-s 0.01]

"""

import rospy, rosbag
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import argparse
import sys, select, tty, termios

"""
Class:   NodeTracker
Args:    objects: list of strings representing all objects being tracked by object_detect node
Fields:  A NodeTracker object initializes 6 publishers to ROS topics
         and 6 variables located in a dictionary to track the status of messages loaded
         on these topics.These topics correspond to the 6 required topics for successful
         object detection and image processing. The default values
         are set up for processing from a ZED stereo camera rosbag recording:
         rgb image topic:    /zed/zed_node/rgb/image_rect_color
         rgb camera info:    /zed/zed_node/rgb/camera_info
         depth image topic:  /zed/zed_node/depth/depth_registered
         depth camera info:  /zed/zed_node/depth/camera_info
         transform frame:    /tf
         camera odometry:    /zed/zed_node/odom
         A NodeTracker has a dictionary holding the names of all objects currently being
         tracked by the object_detect node(s) and their 'ready' status.
         A Nodetracker communicates with the object_detect node(s) over the /ready
         and /end topics.
Methods: close_nodes:       sends shutdown signal to all connected nodes
         node_callback:     "triggers" gate to change the ready status for a given node object
         ready:             checks all nodes and message statuses to determine if publish ready
         check_nodes:       checks all nodes for readiness status
         add_message:       adds a message from the rosbag to the message queue
         publish_queue:     publishes and emptys all messages loaded on the message queue
"""
class NodeTracker:
    def __init__(self, objects):
        # storage dictionary to track the loading of the 6 ROS topics messages
        self.msg_status = { topics["img_topic"] : False,
                topics["img_info_topic"] : False,
                topics["depth_topic"] : False,
                topics["depth_info_topic"] : False,
                topics["tf_topic"] : False,
                topics["odom_topic"] : False }

        # storage dictionary to track the ready status of the object_detect node(s)
        self.node_status = {key: False for key in objects} # nodes not ready initially 
        self.is_paused = False

        # message queue to build up one of each ROS topic before sending
        self.msg_queue = {}

        # communication lines via Subscriber/Publisher to object_detect node(s)
        self.node_sub = rospy.Subscriber("/ready", String, self.node_callback)
        self.node_pub = rospy.Publisher("/ready", String, queue_size=10)
        self.node_end = rospy.Publisher("/end", String, queue_size=10)

        # 6 ROS topic publishers
        self.publishers = { topics["img_topic"]:rospy.Publisher(topics["img_topic"],Image,queue_size=10),
                topics["img_info_topic"]:rospy.Publisher(topics["img_info_topic"], CameraInfo,queue_size=10),
                topics["depth_topic"]:rospy.Publisher(topics["depth_topic"],Image,queue_size=10),
                topics["depth_info_topic"]:rospy.Publisher(topics["depth_info_topic"],CameraInfo,queue_size=10),
                topics["tf_topic"]:rospy.Publisher(topics["tf_topic"],TFMessage,queue_size=10),
                topics["odom_topic"] : rospy.Publisher(topics["odom_topic"],Odometry,queue_size=10) }

    # send shutdown signal to all connected nodes
    def close_nodes(self):
        self.node_end.publish("shutdown")

    # handles trigger_gate call from object_detect node
    def node_callback(self, msg):
        if (msg.data == "start"):
            return
        else:
            print("triggering gate. . . ")
            self.node_status[msg.data] = False if self.node_status.get(msg.data) else True

    # returns a boolean if gate is overall ready to publish the message queue
    def ready(self):
        for status in self.msg_status.values():
            if not status:
                return False
        return self.check_nodes()

    def check_nodes(self):
        for status in self.node_status.values():
            if not status:
                return False
        return not self.is_paused

    # add a message to the message queue
    def add_message(self, topic, msg):
        self.msg_queue[topic] = msg
        self.msg_status[topic] = True

    # publish and clear all messages stored on the message queue
    def publish_queue(self):
        for topic in self.msg_queue.keys():
            pub = self.publishers.get(topic)
            msg = self.msg_queue.get(topic)
            pub.publish(msg)
        for key in self.msg_status.keys():
            self.msg_status[key] = False
        self.msg_queue.clear()
        self.node_pub.publish("start") # start node processing callback

# close out of program
def shutdown():
    if rospy.core.is_shutdown():
        print("Closing gate program only . . .")
        sys.exit()
    else:
        print("Sending shutdown signal to nodes . . .")
        tracker.close_nodes()
        rospy.rostime.wallsleep(1)
        print("Closing gate program . . .")
        rospy.signal_shutdown("closing gate")
        sys.exit()

# for use with detecting keyboard input
def isData():
    return select.select([sys.stdin],[],[],0) == ([sys.stdin],[],[])

# main block of code execution. Gate node is initialized and begins loading
# messages from the rosbag based on the status of all nodes and messages
def start_node():
    rospy.init_node('gate')
    rospy.loginfo('gate node started')
    print("Press 'q' to close gate program and all nodes.")
    print("Press 'p' to pause gate internally.")
    print("Keyboard interrupt (ctrl+c) shuts gate node down only.")
    rospy.rostime.wallsleep(2)

    # handle tf_static messages
    tf_static_pub = rospy.Publisher(tf_static_topic,TFMessage,queue_size=10)  
    tf_static_msgs = bag.read_messages(topics=tf_static_topic)
    for topic,msg,t in tf_static_msgs:
        print("Topic:{}. Message:{}".format(topic,msg))
        tf_static_pub.publish(msg)

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno()) 

        # rospy.spin() main loop
        while not rospy.core.is_shutdown():
            # publish messages when all types have been received
            if tracker.ready():
                tracker.publish_queue()
                print("Gate publishing queue. . .")
            
            # load new messages if all nodes are done processing
            if tracker.check_nodes():
                topic, msg, t = messages.next()
                if (t.secs <= end_time-1):
                    tracker.add_message(topic, msg)
                    print("Loading message: {}".format(topic))
                else:
                    break
            
            # listen for keyboard input (non-blocking)
            if isData():
                char = sys.stdin.read(1)
                if char == 'q':
                    break
                if char == 'p':
                    tracker.is_paused = False if tracker.is_paused else True

            print("Gate waiting")
            rospy.rostime.wallsleep(delay)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    shutdown()

if __name__ == '__main__':
    try:
         # parse command-line arguments
        ap = argparse.ArgumentParser()
        ap.add_argument("-r", "--rosbag", required=True,
            help ="base path to rosbag file")
        ap.add_argument("-o", "--objects",default=["person"],
            type=lambda s: [item for item in s.split(',')],
            help ="list of objects being detected. -o obj1,obj2,obj3...")
        ap.add_argument("-z", "--topics",default=None,
            type=lambda s: [item for item in s.split(',')],
            help = "List of ros topics to listen to. Order must be: camera rgb image topic,\
                    rgb camera info topic,depth image topic,depth camera info topic,transform\
                    topic,odometry topic")
        ap.add_argument("-s", "--speed",type=float, default=0.005,
            help ="speed of main while loop in (sec)")
        args = vars(ap.parse_args(rospy.myargv()[1:]))

        # default ZED node topics
        topics = { "img_topic" : "/zed/zed_node/rgb/image_rect_color",
                "img_info_topic" : "/zed/zed_node/rgb/camera_info",
                "depth_topic" : "/zed/zed_node/depth/depth_registered",
                "depth_info_topic" : "/zed/zed_node/depth/camera_info",
                "tf_topic" : "/tf",
                "odom_topic" : "/zed/zed_node/odom" }

        # handle non-default topics
        if args["topics"]:
            tlist = args["topics"]
            if (len(tlist) != len(topics.keys())):
                print("Invalid topics list. Must be {} entries in order.".format(len(topics.keys())))
                print("List of ros topics to listen to. Order must be: camera rgb image topic,\
                        rgb camera info topic, depth image topic, depth camera info topic, transform\
                        topic, odometry topic")
                sys.exit()

            topics = { "img_topic" : tlist[0],
            "img_info_topic" : tlist[1],
            "depth_topic" : tlist[2],
            "depth_info_topic" : tlist[3],
            "tf_topic" : tlist[4],
            "odom_topic" : tlist[5] }

        topics_to_read = topics.values()
        tf_static_topic = "/tf_static"

        # load bagfile messages
        bagfile = args["rosbag"]
        bag = rosbag.Bag(bagfile, "r")

        messages = bag.read_messages(topics=topics_to_read)
        end_time = bag.get_end_time()

        # initialize node tracker
        tracker = NodeTracker(args["objects"])
        delay = args["speed"]

        # initialize gate node
        start_node()

    except rospy.ROSInterruptException:
        pass
