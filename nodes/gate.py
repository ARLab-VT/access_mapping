#!/usr/bin/env python

import rospy, rosbag
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import argparse
import sys, select, tty, termios

class NodeTracker:
    def __init__(self, nodes):
        self.msg_status = { img_topic : False,
                img_info_topic : False,
                depth_topic : False,
                depth_info_topic : False,
                tf_topic : False,
                odom_topic : False }
        self.node_status = {key: False for key in nodes} # nodes not ready initially 
        self.is_paused = False
        self.msg_queue = {}
        self.node_sub = rospy.Subscriber("/ready", String, self.node_callback)
        self.node_pub = rospy.Publisher("/ready", String, queue_size=10)
        self.node_end = rospy.Publisher("/end", String, queue_size=10)
        self.publishers = { img_topic:rospy.Publisher(img_topic,Image,queue_size=10),
                img_info_topic : rospy.Publisher(img_info_topic, CameraInfo,queue_size=10),
                depth_topic : rospy.Publisher(depth_topic,Image,queue_size=10),
                depth_info_topic : rospy.Publisher(depth_info_topic,CameraInfo,queue_size=10),
                tf_topic : rospy.Publisher(tf_topic,TFMessage,queue_size=10),
                odom_topic : rospy.Publisher(odom_topic,Odometry,queue_size=10) }

    def close_nodes(self):
        self.node_end.publish("shutdown")

    def node_callback(self, msg):
        if (msg.data == "start"):
            return
        else:
            print("triggering gate. . . ")
            self.node_status[msg.data] = False if self.node_status.get(msg.data) else True

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

    def add_message(self, topic, msg):
        self.msg_queue[topic] = msg
        self.msg_status[topic] = True

    def publish_queue(self):
        for topic in self.msg_queue.keys():
            pub = self.publishers.get(topic)
            msg = self.msg_queue.get(topic)
            pub.publish(msg)
        for key in self.msg_status.keys():
            self.msg_status[key] = False
        self.msg_queue.clear()
        self.node_pub.publish("start") # start node processing callback

def shutdown():
    # close out of program
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

def isData():
    return select.select([sys.stdin],[],[],0) == ([sys.stdin],[],[])

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
        # topics
        img_topic = "/zed/zed_node/rgb/image_rect_color"
        img_info_topic = "/zed/zed_node/rgb/camera_info"
        depth_topic = "/zed/zed_node/depth/depth_registered"
        depth_info_topic = "/zed/zed_node/depth/camera_info"
        tf_topic = "/tf"
        tf_static_topic = "/tf_static"
        odom_topic = "/zed/zed_node/odom"
        topics_to_read = [img_topic, img_info_topic, depth_topic, depth_info_topic,
            tf_topic, odom_topic]

         # parse command-line arguments
        ap = argparse.ArgumentParser()
        ap.add_argument("-r", "--rosbag", required=True,
            help ="base path to rosbag file")
        ap.add_argument("-n", "--nodes",default=["person"],
            type=lambda s: [item for item in s.split(',')],
            help ="list of objects being detected. -n obj1,obj2,obj3...")
        ap.add_argument("-s", "--speed",type=float, default=0.005,
            help ="speed of main while loop in (sec)")
        args = vars(ap.parse_args(rospy.myargv()[1:]))

        # load bagfile messages
        bagfile = args["rosbag"]
        bag = rosbag.Bag(bagfile, "r")

        messages = bag.read_messages(topics=topics_to_read)
        end_time = bag.get_end_time()

        # initialize node tracker
        tracker = NodeTracker(args["nodes"])
        delay = args["speed"]
        start_node()

    except rospy.ROSInterruptException:
        pass
