#!/usr/bin/env python3 

# Usage: rosrun ros_cv object_detect.py --yolo ../path_to_yolo_dir --objects
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

class GateListener:
    def __init__(self):
        self.image = None
        self.image_info = None
        self.depth = None
        self.depth_info = None
        self.tf = None
        self.odom = None
        self.play_mode = None
        self.tf_listener = None
        self.gate_sub = None
        self.gate_end = rospy.Subscriber("/end", String, self.shutdown)
        self.stepping = False
        self.gate_pub = rospy.Publisher("/ready", String, queue_size=10)
        self.image_sub = rospy.Subscriber(img_topic, Image,
                self.image_callback)
        self.image_info_sub = rospy.Subscriber(img_info_topic, CameraInfo,
                self.image_info_callback)
        self.depth_sub = rospy.Subscriber(depth_topic, Image,
                self.depth_callback)
        self.depth_info_sub = rospy.Subscriber(depth_info_topic, CameraInfo,
                self.depth_info_callback)
        self.tf_sub = rospy.Subscriber(tf_topic, TFMessage, self.tf_callback)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry,
                self.odom_callback)

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

    def trigger_gate(self):
        for obj in node_objects:
            self.gate_pub.publish(obj)

    def shutdown(self,msg):
        # close out of program
        if (msg.data == "shutdown"):
            print("Closing program on gate shutdown . . .")
            rospy.signal_shutdown("closing node")
            
def isData():
    return select.select([sys.stdin],[],[],0) == ([sys.stdin],[],[])

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

#                        publish message to /detect topic
#                        detect_msg = Detect()
#                        detect_msg.label = obj
#                        h = std_msgs.msg.Header()
#                        h.frame_id = "/detection"
#                        h.stamp = rospy.Time.now()
#                        detect_msg.header = h
#                        detect_msg.header.stamp = rospy.Time.now()
#                          
#                        point_stamped = geometry_msgs.msg.PointStamped()
#                        header = std_msgs.msg.Header()
#                        header.stamp = rospy.Time.now()
#                        header.frame_id = "zed_left_camera_optical_frame"
#                        point_x = int(x+(w/2)) 
#                        point_y = int(y+(h/2))
#                        point_z = 0
#                        point = geometry_msgs.msg.Point(point_x, point_y, point_z)
#                        point_stamped = geometry_msgs.msg.PointStamped(header, point)
#                        print("Time: {}.{}".format(detect_msg.header.stamp.secs, detect_msg.header.stamp.nsecs))
#                        detect_msg.box = [int(x+(w/2)),int(y+(h/2)),w,h]
#                        detect_msg.confidence = confidences[i]
#                        pub.publish(point_stamped)

        # show the output image and return
        cv2.imshow("Object detection image", image)
        cv2.waitKey(1)
        video_array.append(image)
        return detect_array

    except Exception as err:
        print(err)
        return None

def rgb_to_depth(objs):
    depth_image = bridge.imgmsg_to_cv2(listener.depth, "32FC1")
    normalizedImg = np.array(depth_image, dtype=np.float)
    normalizedImg = cv2.normalize(normalizedImg, normalizedImg, 0, 1, cv2.NORM_MINMAX,
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

def convert3d(arr):
    # Get camera intrinsic parameters
    camera_info = listener.image_info
    cx = camera_info.K[2]
    cy = camera_info.K[5]
    fx = camera_info.K[0] # X focal length of ZED in pixels
    fy = camera_info.K[4] # Y focal length of ZED in pixels

#    # The ZED has 0.004 mm per pixel at 1280x720 resolution
#    focal_length_mm = 2.8 # Focal Length of ZED in mm 
#    focal_length_pix = camera_info.K[0] # Focal Length of ZED in pixels
#    # mm/pixel of ZED. If running at 1280x720 res, should be ~0.004 mm
#    mm_per_pixel = focal_length_mm/focal_length_pix  
#
#    # FOV_d = 110                              # Field of View of the ZED in degrees
#    # FOV_r = 110*np.pi/180                    # Field of View of the ZED in radians
#    """
#    F = f*(W/w); if you know the camera's digital sensor has a width W in millimiters,
#    and the image width in pixels is w, you can convert the focal length f to world units.
#    F is in mm, f is in pixel size, W is sensor width in whatever length unit you can get
#    it in, and w is in pixels.
#    """
#    # Get field of view (fov) angles
#    img_width_mm = mm_per_pixel * W
#    theta_horizontal = np.arctan((img_width_mm/2)/focal_length_mm)
#    fov_horizontal = 2*theta_horizontal
#    print ("Horizontal Field of View in Degrees = {}".format(fov_horizontal*180/np.pi))
#    img_height_mm = mm_per_pixel * H
#    theta_vertical = np.arctan((img_height_mm/2)/focal_length_mm)
#    fov_vertical = 2*theta_vertical
#    print ("Vertical Field of View in Degrees = {}".format(fov_vertical*180/np.pi))
#
#    alpha_horizontal = (np.pi - fov_horizontal)/2
#    alpha_vertical = 2*np.pi - (fov_vertical)/2

    # array to fill with desired objects
    # format: [(label, 3DPointStamped), (..), ...]
    point3d_array = []

    # retrieve center pixel information
    if arr:
        for obj in arr:
            label = obj[0]
            depth = obj[1]
            (centerX, centerY) = obj[2:4] # col = centerX; row = centerY
            x_new = (centerX - cx) * depth / fx
            y_new = (centerY - cy) * depth / fy
            z_new = depth
            P3D = Point(x_new,y_new,z_new)

            # Configure the Header message file
            h = Header()
            # Header.seq is handled by the publisher
            h.stamp = rospy.Time.now()
            h.frame_id = 'zed_left_camera_optical_frame'

            # Create a PointStamped message file which consists of a Header.msg and Point.msg
            P3D_Stamped = PointStamped(h, P3D)
            point3d_array.append((label, P3D_Stamped))

    return point3d_array 

def tf_global(arr, target_frame='map'):
    """
    This function takes a PointStamped Message and converts its coordinates into the
    target_frame coordinates. This is done by tf.TransformerListener.TransformPoint,
    however timing needs to be implemented in order for this code to work
    """
    tf_buffer = tf2_ros.Buffer()
    tf2_listener = tf2_ros.TransformListener(tf_buffer)
    transform = tf_buffer.lookup_transform(target_frame,"zed_left_camera_optical_frame", rospy.Time.now(), rospy.Duration(1.0))        

    # array to fill with desired objects
    # format: [(label, 3DPointStamped), (..), ...]
    global3d_array = []

    if arr:
        for obj in arr:
            label = obj[0]
            point_stamped = obj[1]
            global_point = listener.transformPoint(target_frame, point_stamped)
            print ("Global (X,Y,Z) = ({}, {}, {})".format(global_point.point.x,global_point.point.y,global_point.point.z))
            global3d_array.append((label,global_point))

    return global3d_array

def make_grid(width = 100, height = 100, resolution = 0.05):
     ### MAKE AN OCCUPANCY GRID ###

    # Configure the Header message file
    h = Header()
    # Header.seq is handled by the publisher
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

def modify_grid(arr):
    width = occ_grid.MapMetaData.width
    height = occ_grid.MapMetaData.height
    resolution = occ_grid.MapMetaData.resolution
    data = occ_grid.data
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
                occ_grid.data[index] = data[index]
                self.grid_pub.publish(occ_grid)

#def convert_3d(arr):
#    # array to fill with desired object 3D coordinates
#    # format: [(label,x,y,z), (....), ...]
#    obj_arr = []
#    
#    # get intrinsic camera properties
#    # all values are in units of pixels
#    p_mat = listener.image_info.P
#    fx, fy, cx, cy = p_mat[0], p_mat[5], p_mat[2], p_mat[6]
#    
#    if arr:
#        for obj in arr:
#            # values for obj as returned by rgb_to_depth function
#            label, d, c, r = obj[0], obj[1], obj[2], obj[3]
#
#            # calculate 3D point in left_camera's frame from geometry
#            x = ((c-cx)*d)/fx
#            y = ((r-cy)*d)/fy
#            z = d
#            obj_arr.append((label,round(x,5),round(y,5),round(z,5)))
#            
#    return obj_arr
    
def handle_step(msg):
    if (listener.play_mode == "continuous"):
        return

    if (msg.data == "start"):
        listener.stepping = False # closes step gate
        process_callback()

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

        # Enea's code:
        points_3d = convert3d(depths) if depths else None
        print("3D Points in left camera frame: {}".format(points_3d)) if points_3d else print("No 3D points obtained")

        #global_3d = tf_global(points_3d) if points_3d else None
        #print("3D Points in global frame: {}".format(global_3d)) if global_3d else print("No global 3D points obtained")
        
        # modify_grid(points_3d) if points_3d else None

        # 2d_to_3d         -> (X,Y,Z) and label in camera's ref frame
        # points_3d = convert_3d(depths) if depths else None
        # print("3D Points: {}".format(points_3d)) if points_3d else print("No 3D points obtained")
        # tf_global        -> (X,Y,Z) and label in global frame
        # aggr             -> [(obj,X,Y,Z,W,H)...] accumulated, aggregated list
        
        print("\n")

        if (listener.play_mode == "auto_step"):
            listener.trigger_gate() # open gate

    except Exception as err:
        print(err)

def start_node():
    node_name = "detect"
    for obj in node_objects:
        node_name = obj + "_" + node_name
    rospy.init_node(node_name)
    rospy.loginfo('{} node started'.format(node_name))
    old_settings = termios.tcgetattr(sys.stdin)
    listener.tf_listener = tf.TransformListener()

    # Make occupancy grid
    occ_grid = make_grid()
    grid_pub = rospy.Publisher('GRID', OccupancyGrid, queue_size=10)
    grid_pub.publish(occ_grid)

    # open gate initially for continuous, auto-step, and fast-forward modes
    if (listener.play_mode != "step"):
        listener.trigger_gate()

    # rospy.spin()
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

if __name__ == '__main__':
    try:
        # topics
        img_topic = "/zed/zed_node/rgb/image_rect_color"
        img_info_topic = "/zed/zed_node/rgb/camera_info"
        depth_topic = "/zed/zed_node/depth/depth_registered"
        depth_info_topic = "/zed/zed_node/depth/camera_info"
        tf_topic = "/tf"
        odom_topic = "/zed/zed_node/odom"

        # create listener object to store info from gate node
        listener = GateListener()

        # parse command-line arguments
        ap = argparse.ArgumentParser()
        ap.add_argument("-y", "--yolo", required=True,
            help="base path to YOLO directory")
        ap.add_argument("-o", "--objects",
            type=lambda s: [item for item in s.split(',')],
            default=["person"],
            help="node-specific objects for detection")
        ap.add_argument("-c", "--confidence", type=float, default=0.5,
            help="minimum probability to filter weak detections")
        ap.add_argument("-t", "--threshold", type=float, default=0.3,
            help="threshold when applyong non-maxima suppression")
        ap.add_argument("-v", "--video", default="None",
            help="name of video file to write")
        ap.add_argument("-s", "--speed", type=float, default=0.005,
            help="speed of main while loop")
        ap.add_argument("-p", "--playback", default='a',
            help="playback mode: (c)ontinuous, (s)tep, (a)uto-step, (f)ast-forward")
        args = vars(ap.parse_args(rospy.myargv()[1:]))

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

        # global accumulators
        video_array = []

        # Initialize node
        node_objects = args["objects"]
        video_name = args["video"]
        delay = args["speed"]
        video_write = False if (video_name == "None") else True
#        pub = rospy.Publisher("/detect", geometry_msgs.msg.PointStamped, queue_size=10)
        bridge = CvBridge()
        start_node()

    except rospy.ROSInterruptException:
        pass
