#!/usr/bin/env python3 
"""
object_detect.py:   Python 3 script for running a ROS node
Contributors:       Joshua Eckels, Enea Dushaj
Organization:       Virginia Tech NSF-REU
Date:               August 7, 2019
Note:               This python ROS node only works in tandem with the gate.py node in 
                    the access_mapping ROS package. The gate.py node must be run first to
                    initialize a rosbag for playback. This node must be executed while
                    in the python 3 virtual environment where OpenCV was installed.
                    Activate virtualenvironment using: workon cv

Doc:    This node receives image and transform messages from ROS topics, passes an image
        through a pre-trained YOLOv3 neural network for object detection, obtains localized center
        points for each object detected, determines each objects' location in 3D space relative
        to the ZED camera, transforms the locations to a global map, and exports a list of all
        detected objects and locations for later use in global map annotation.

Command-line arguments:
    -y  /  --yolo       : base path to folder containing YOLO config and weights [REQUIRED]
    -o  /  --objects    : list of objects within pre-trained coco dataset to be detected by node
                          format: --objects person,tvmonitor,chair,etc
    -p  /  --playback   : character to determine playback mode (see GateListener)
    -z  /  --topics     : list of 6 ROS topics to listen to (see GateListener)
                          note: must be in the order indicated in docstring below
    -v  /  --video      : name of video file to write in current directory (defaults to no video)
    -s  /  --speed      : value to toggle the loop playback speed of this node
    -t  /  --threshold  : Threshold when applying non-maxima suppression (for YOLO use)
    -c  /  --confidence : Minimum probability to filter weak detections (for YOLO use)

Usage: rosrun access_mapping object_detect.py --yolo ../yolo-coco [-o person,chair] 
        [-p s] [-z img,inf,depth,infd,tf,odom] [-v output.avi] [-s 0.01] [-t 0.4] [-c 0.6]

"""

import rospy
import tf, tf2_ros 
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from std_msgs.msg import String, Header
from geometry_msgs.msg import Point, PointStamped, Quaternion, Pose
from tf.transformations import quaternion_from_euler
from cv_bridge import CvBridge
import cv2
import numpy as np
import argparse
import sys, termios, tty, os, time, select

"""
Class:   GateListener
Args:    None
Fields:  A GateListener object initializes 6 subscribers to ROS topics 
         and 6 storage variables for messages received on these topics. 
         These topics correspond to the 6 required topics for successful
         object detection and image processing. The default values
         are set up for processing from a ZED stereo camera rosbag recording:
         rgb image topic:    /zed/zed_node/rgb/image_rect_color
         rgb camera info:    /zed/zed_node/rgb/camera_info
         depth image topic:  /zed/zed_node/depth/depth_registered
         depth camera info:  /zed/zed_node/depth/camera_info
         transform frame:    /tf
         camera odometry:    /zed/zed_node/odom
         A GateListener object holds the boolean 'play_mode', set by the user, 
         that determines how the object_detect node interacts with the gate node.
         There are 4 playback modes:
           Continuous: gate will publish messages independent of object_detect node
           Step: gate will only publish one message upon user keyboard input
           Auto-step: gate sends messages only when object_detect node has completed processing
           Fast-forward: gate sends messages and no image processing is performed
         A GateListener object holds 3 storage variables for transforms and occupancy grids
Methods: A GateListener has 6 callback methods for storing ROS topic messages, a
         method for toggling the gate node, and a method for node shutdown.
"""
class GateListener:
    def __init__(self):
        # storage variables for the 6 ROS topics 
        self.image = None
        self.image_info = None
        self.depth = None
        self.depth_info = None
        self.tf = None
        self.odom = None

        # play mode determines how node interacts with the gate
        self.play_mode = None
        self.stepping = False

        # storage variables for use with transforms and occupancy grid
        self.tf_listener = None	
        self.grid_pub = None
        self.occ_grid = None

        # communication lines via Subscriber/Publisher to gate node
        self.gate_sub = None
        self.gate_pub = rospy.Publisher("/ready", String, queue_size=10)
        self.gate_end = rospy.Subscriber("/end", String, self.shutdown)

        # 6 ROS topic listeners
        self.topic_sub = { "image_sub" : rospy.Subscriber(topics["img_topic"], Image,
                self.image_callback),
                "image_info_sub" : rospy.Subscriber(topics["img_info_topic"], CameraInfo,
                self.image_info_callback),
                "depth_sub" : rospy.Subscriber(topics["depth_topic"], Image,
                self.depth_callback),
                "depth_info_sub" : rospy.Subscriber(topics["depth_info_topic"], CameraInfo,
                self.depth_info_callback),
                "tf_sub" : rospy.Subscriber(topics["tf_topic"], TFMessage, self.tf_callback),
                "odom_sub" : rospy.Subscriber(topics["odom_topic"], Odometry,
                self.odom_callback) }

    # 6 callback methods to store ROS topic messages internally
    def image_callback(self, msg):
        self.image = msg

    def depth_callback(self, msg):
        self.depth = msg

    def image_info_callback(self, msg):
        self.image_info = msg

    def depth_info_callback(self, msg):
        self.depth_info = msg

    def odom_callback(self, msg):
        self.odom = msg

    def tf_callback(self, msg):
        self.tf = msg

    # publishes the name of each node_object to the gate node on the /ready topic
    # this alerts the gate node that this object_node has finished processing a set of data
    def trigger_gate(self):
        for obj in node_objects:
            self.gate_pub.publish(obj)

    # close out of program
    def shutdown(self,msg):
        if (msg.data == "shutdown"):
            print("Closing program on gate shutdown . . .")
            rospy.signal_shutdown("closing node")
            
# for reading keyboard input
def isData():
    return select.select([sys.stdin],[],[],0) == ([sys.stdin],[],[])

# saving object detection video
def write_video():
    if not video_array:
        pass
    else:
        print("\nTotal number of images gathered: {}".format(len(video_array)))
        print("Writing video . . .")
        out = cv2.VideoWriter(video_name,cv2.VideoWriter_fourcc(*'XVID'), 28, (W,H))
        for i in range(len(video_array)):
            out.write(video_array[i])
        out.release()
        print("Done writing video.")

# pass a ROS sensor_msgs/Image message through object detection
# return an array of objects detected with their center value
def object_detection():
    try:
        image = bridge.imgmsg_to_cv2(listener.image, "bgr8")
        global W
        global H
        (H, W) = image.shape[:2]

        # array to fill with desired detected objects
        # format: [(label, centerX, centerY, width, height), (.....), ...]
        detect_array = []

        # determine only the *output* layer names that we need from YOLO
        ln = net.getLayerNames()
        ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]

        # construct a blob from the input image and then perform a forward
        # pass of the YOLO object detector, giving us our bounding boxes and
        # associated probabilities
        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416),
        swapRB=True, crop=False)
        net.setInput(blob)
        start = time.time()
        layerOutputs = net.forward(ln)
        end = time.time()

        # show timing information on YOLO
        print("[INFO] YOLO took {:.6f} seconds".format(end - start))

        # initialize our lists of detected bounding boxes, confidences, and
        # class IDs, respectively
        boxes = []
        confidences = []
        classIDs = []

        # loop over each of the layer outputs
        for output in layerOutputs:
            # loop over each of the detections
            for detection in output:
            # extract the class ID and confidence (i.e., probability) of
            # the current object detection
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]

                # filter out weak predictions by ensuring the detected
                # probability is greater than the minimum probability
                if confidence > args["confidence"]:
                    # scale the bounding box coordinates back relative to the
                    # size of the image, keeping in mind that YOLO actually
                    # returns the center (x, y)-coordinates of the bounding
                    # box followed by the boxes' width and height
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")

                    # use the center (x, y)-coordinates to derive the top and
                    # and left corner of the bounding box
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))

                    # update our list of bounding box coordinates, confidences,
                    # and class IDs
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)

        # apply non-maxima suppression to suppress weak, overlapping bounding
        # boxes
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, args["confidence"],args["threshold"])

        # ensure at least one detection exists
        if len(idxs) > 0:
            # loop over the indexes we are keeping
            for i in idxs.flatten():
                # extract the bounding box coordinates
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])

                # draw a bounding box rectangle and label on the image
                color = [int(c) for c in COLORS[classIDs[i]]]
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                text = "{}: {:.4f}".format(LABELS[classIDs[i]], confidences[i])
                cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, color, 2)
                        
                # add desired objects to return list
                # example shown below: take all objects in node_objects list
                for obj in node_objects:
                    if (LABELS[classIDs[i]] == obj):
                        tuple_to_append = (LABELS[classIDs[i]], int(x+(w/2)), int(y+(h/2)), 
                                w, h)
                        detect_array.append(tuple_to_append)

        # show the output image and return
        cv2.imshow("Object detection image", image)
        cv2.waitKey(1)
        video_array.append(image)
        return detect_array

    except Exception as err:
        print(err)
        return None

# this function obtains depth values for center image pixels
def rgb_to_depth(objs):
    depth_image = bridge.imgmsg_to_cv2(listener.depth, "32FC1")
    normalizedImg = np.array(depth_image, dtype=np.float)
    normalizedImg = cv2.normalize(normalizedImg, normalizedImg, 1, 0, cv2.NORM_MINMAX,
        dtype=cv2.CV_32F)
 
    # array to fill with desired objects
    # format: [(label, depth, centerX, centerY), (....), ...]
    # depth is given in meters in data format '32FC1'
    depth_array = []
 
    # retrieve center pixel information
    if objs:
        for obj in objs:
            label = obj[0]
            (col, row) = obj[1:3] # col = centerX; row = centerY
            (w, h) = obj[3:5]
            depth = depth_image[row,col]
            depth_array.append((label, depth, col, row))

            # draw bounding boxes on normalized depth image
            (cornerX, cornerY) = (int(col-(w/2)),int(row-(h/2)))
            cv2.rectangle(normalizedImg, (cornerX, cornerY), (cornerX + w, cornerY + h), 1, 2)
            cv2.putText(normalizedImg, label, (cornerX, cornerY - 5), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, 1, 2)
    
    # display depth image and return array
    cv2.imshow("Normalized depth image", normalizedImg)
    cv2.waitKey(1)
    return depth_array

# this function utilizes physical camera parameters and geometry to obtain
# 3D coordinates of an object in an image at a know depth
def convert3d(arr):
    # array to fill with desired object 3D coordinates
    # format: [(label,PointStamped), (..), ...]
    obj_arr = []
    
    # get intrinsic camera properties
    # all values are in units of pixels
    p_mat = listener.image_info.P
    fx, fy, cx, cy = p_mat[0], p_mat[5], p_mat[2], p_mat[6]
    
    if arr:
        for obj in arr:
            # values for obj as returned by rgb_to_depth function
            label, d, c, r = obj[0], obj[1], obj[2], obj[3]

            # calculate 3D point in left_camera's frame from geometry
            x = ((c-cx)*d)/fx
            y = ((r-cy)*d)/fy
            z = d
            point3d = Point(x,y,z)
            print("Left camera (X,Y,Z): [{},({}, {}, {})]".format(label,round(x,5),round(y,5),round(z,5)))

            # Configure 3DPointStamped
            h = Header()
            h.stamp = rospy.Time.now()
            h.frame_id = 'zed_left_camera_optical_frame'
            point3d_stamped = PointStamped(h, point3d)
            obj_arr.append((label, point3d_stamped))

            
    return obj_arr
 
# this function converts a 3DPoint from ZED camera frame into global frame
def tf_global(arr, target_frame='map'):
    tf_buffer = tf2_ros.Buffer()
    tf2_listener = tf2_ros.TransformListener(tf_buffer)
    transform = tf_buffer.lookup_transform(target_frame,
                                           "zed_left_camera_optical_frame",
                                           rospy.Time.now(), 
                                           rospy.Duration(1.0))        

    # array to fill with desired objects
    # format: [(label, 3DPointStamped), (..), ...]
    global3d_array = []

    if arr:
        for obj in arr:
            label = obj[0]
            point_stamped = obj[1]
            global_point = listener.transformPoint(target_frame, point_stamped)
            print ("Global (X,Y,Z) = [{},({}, {}, {})]".format(label,global_point.point.x,global_point.point.y,global_point.point.z))
            global3d_array.append((label,global_point))

    return global3d_array

# this function will return an OccupancyGrid object with the specified values 
def make_grid(width = 100, height = 100, resolution = 0.05):
    # Configure the Header message file
    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = 'map'
    map_load_time = rospy.Time(0)

    # resolution is in unit m/cell
    # width      is in unit cells
    # height     is in unit cells
    q = quaternion_from_euler(0, np.pi,-np.pi/2)  # (rot about x, rot about y, rot about z)
    xq = q[0]
    yq = q[1]
    zq = q[2]
    wq = q[3]
    orientation = Quaternion(xq,yq,zq,wq)
    origin  = Pose(Point(0,0,0), orientation)
    info = MapMetaData(map_load_time, resolution, width, height, origin)
    n = width*height
    data = np.zeros(n)

    return OccupancyGrid(h,info,data)

# this function takes an array of 3D points and puts their projection onto an
# occupancy grid for data visualization
def modify_grid(arr):
    width = listener.occ_grid.MapMetaData.width
    height = listener.occ_grid.MapMetaData.height
    resolution = listener.occ_grid.MapMetaData.resolution
    data = listener.occ_grid.data
    if arr:
        for obj in arr:
            point_stamped = obj[1]
            if not (math.isnan(point_stamped.point.x) and math.isnan(point_stamped.point.y)):
                x = int(point_stamped.point.x/resolution)

                #shift the coordinate system over to only deal with positive y values within the grid
                y_corrected = point_stamped.point.y + width/2 
                y = int(y_corrected/resolution)
                index = y + x*width
                data[index] = 100
                listener.occ_grid.data[index] = data[index]
                listener.grid_pub.publish(listener.occ_grid)
   
# this function receives a message on the /ready topic and responds to
# handle the "stepping" functionality through the rosbag
def handle_step(msg):
    if (listener.play_mode == "continuous"):
        return

    if (msg.data == "start"):
        listener.stepping = False # closes step gate
        process_callback()

# the main processing center of the node. All other image processing functions are called
# from here. process_callback will signal to the gate node when it starts processing
# and when it finishes by "closing" and "opening" the gate, respectively
def process_callback():
    try:
        if (listener.play_mode == "auto_step"):
            listener.trigger_gate() # close gate

        # object detection -> append to video_array, list of boxes
        objects = object_detection() if listener.image else None
        print("Objects: {}".format(objects)) if objects else print("No objects detected")

        # rgb_to_depth     -> list of boxes with depth value at center
        depths = rgb_to_depth(objects) if listener.depth else None
        print("Depths: {}".format(depths)) if depths else print("No depths obtained")

        # convert3d        -> convert 2d pixel value to 3d coordinates in camera's frame
        points_3d = convert3d(depths) if depths else None

#######################################################################################
# NOT FUNCTIONAL YET -> TF and OccupancyGrid Visualization
#######################################################################################
        # tf_global        -> (X,Y,Z) and label in global frame
        # global_3d = tf_global(points_3d) if points_3d else None
        
        # modify_grid      -> data visualization
        #modify_grid(points_3d) if points_3d else None

        # aggr             -> [(obj,X,Y,Z,W,H)...] accumulated, aggregated list
#######################################################################################
        
        print("\n")

        if (listener.play_mode == "auto_step"):
            listener.trigger_gate() # open gate

    except Exception as err:
        print(err)

# initialize object_detect.py ROS node. This is where the main loop of the code
# executes continuously. An occupancy grid is initialized. Decisions are made
# to process callback based on keyboard input and playback mode once the 
# rospy.spin() main loop has been entered.
def start_node():
    node_name = "detect"
    for obj in node_objects:
        node_name = obj + "_" + node_name
    rospy.init_node(node_name)
    rospy.loginfo('{} node started'.format(node_name))
    old_settings = termios.tcgetattr(sys.stdin)
    listener.tf_listener = tf.TransformListener()

    # Make occupancy grid
    listener.occ_grid = make_grid()
    listener.grid_pub = rospy.Publisher('GRID', OccupancyGrid, queue_size=10)
    listener.grid_pub.publish(listener.occ_grid)

    # open gate initially for continuous, auto-step, and fast-forward modes
    if (listener.play_mode != "step"):
        listener.trigger_gate()

    # rospy.spin() main loop
    try:
        tty.setcbreak(sys.stdin.fileno())
        while not rospy.core.is_shutdown():
            # continuous mode (break on ctrl+c)
            if (listener.play_mode == "continuous"):
                process_callback()

            # step mode (break on q)
            elif (listener.play_mode == "step"):
                if not listener.gate_sub:
                    listener.gate_sub = rospy.Subscriber("/ready", String, handle_step)
                else:
                    if isData():
                        char = sys.stdin.read(1)
                        if char == 's':
                            # allow gate to pass msgs through
                            listener.trigger_gate() # open gate
                            listener.stepping = True
                            while listener.stepping:
                                pass # block execution
                            listener.trigger_gate() # close gate
                        elif char == 'q':
                            print("Quitting step mode . . .")
                            break

            # auto-step mode (break on ctrl+c)
            elif (listener.play_mode == "auto_step"):
                if not listener.gate_sub:
                    listener.gate_sub = rospy.Subscriber("/ready", String, handle_step)
                else:
                    pass

            # fast-forward mode (no object detection)
            elif (listener.play_mode == "fast_forward"):
                if listener.image:
                    image = bridge.imgmsg_to_cv2(listener.image, "bgr8")
                    cv2.imshow("Fast-forward image", image)
                    cv2.waitKey(1)
                    if isData():
                        char = sys.stdin.read(1)
                        if char == 'q':
                            print("Quitting fast-forward mode . . .")
                            listener.trigger_gate() # close gate
                            break
                else:
                    pass

            # invalid mode
            else:
                print("Not a valid mode: quitting process . . .")
                break

            # 'delay' controls the speed of code execution in this loop
            rospy.rostime.wallsleep(delay) 
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    # write video and cleanup
    cv2.destroyAllWindows()
    print("Closing on rospy.core shutdown . . .")
    if video_write:
        write_video()
    print("Exiting . . .")
    sys.exit()

"""Run this node as a python 3 script. Code execution will start here."""
if __name__ == '__main__':
    try:
        # parse command-line arguments
        ap = argparse.ArgumentParser()
        ap.add_argument("-y", "--yolo", required=True,
            help="Base path to YOLO directory")
        ap.add_argument("-o", "--objects",
            type=lambda s: [item for item in s.split(',')],
            default=["person"],
            help="Node-specific objects for detection")
        ap.add_argument("-c", "--confidence", type=float, default=0.5,
            help="Minimum probability to filter weak detections")
        ap.add_argument("-t", "--threshold", type=float, default=0.3,
            help="Threshold when applying non-maxima suppression")
        ap.add_argument("-v", "--video", default="None",
            help="Name of video file to write")
        ap.add_argument("-s", "--speed", type=float, default=0.005,
            help="Speed of main while loop")
        ap.add_argument("-p", "--playback", default='a',
            help="Playback mode: (c)ontinuous, (s)tep, (a)uto-step, (f)ast-forward")
        ap.add_argument("-z", "--topics",
            type=lambda s: [item for item in s.split(',')],
            default=None,
            help="List of ros topics to listen to. Order must be: camera rgb image topic,\
                    rgb camera info topic,depth image topic,depth camera info topic,transform\
                    topic,odometry topic")
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

        # create listener object to store info from gate node
        listener = GateListener()

        # set playback mode
        s = args["playback"]
        if (s == 'c'):
            print("starting continuous mode . . .")
            print("Use keyboard interrupt to close (ctrl+c).")
            listener.play_mode = "continuous"
            rospy.rostime.wallsleep(2)
        elif (s == 's'):
            print("starting step mode . . .")
            print("Press 's' to step. Press 'q' to close.")
            listener.play_mode = "step"
            rospy.rostime.wallsleep(2)
        elif (s == 'a'):
            print("starting auto-step mode . . .")
            print("Use keyboard interrupt to close (ctrl+c).")
            listener.play_mode = "auto_step"
            rospy.rostime.wallsleep(2)
        elif (s == 'f'):
            print("staring fast-forward mode . . .")
            print("Press 'q' to quit properly.")
            listener.play_mode = "fast_forward"
            rospy.rostime.wallsleep(2)
        else:
            print("Invalid mode: {}. Try again. . .".format(s))
            sys.exit()

        """
        All objects defined here will be used globally throughout the script.
        LABELS: labels for neural network from coco dataset
        COLORS: colors to use for displaying output of neural network
        net: YOLO neural network model itself
        node_objects: all objects that will be detected and processed by this node
        topics: dictionary storing the 6 required ROS topic types
        listener: data storage object to communicate with the gate node
        bridge: cv_bridge object for converting ROS image messages to OpenCV usage 
        video_name: object detection output video file name 
        video_array: global accumulator for images processed during object detection
        video_write: whether or not the node should make a video upon exiting
        """

        # load the COCO class labels our YOLO model was trained on
        labelsPath = os.path.sep.join([args["yolo"], "coco.names"])
        LABELS = open(labelsPath).read().strip().split("\n")

        # initialize a list of colors to represent each possible class label
        np.random.seed(42)
        COLORS = np.random.randint(0, 255, size=(len(LABELS), 3),
        dtype="uint8")

        # derive the paths to the YOLO weights and model configuration
        weightsPath = os.path.sep.join([args["yolo"], "yolov3.weights"])
        configPath = os.path.sep.join([args["yolo"], "yolov3.cfg"])

        # load our YOLO object detector trained on COCO dataset (80 classes)
        print("[INFO] loading YOLO from disk...")
        net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)

        # Initialize node
        node_objects = args["objects"]
        video_name = args["video"]
        video_array = []
        video_write = False if (video_name == "None") else True
        delay = args["speed"]
        bridge = CvBridge()
        start_node()

    except rospy.ROSInterruptException:
        pass
