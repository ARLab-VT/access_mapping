#!/usr/bin/env python

# This script displays a depth image from a ROS topic. It allows you to determine the 
# x, y, and z locations of a point in an image with respect to the optical frame that 
# the depth image was taken in. It also converts that coordinate to a coordinate in a 
# global frame for use in mapping applications.
# The Equations to convert a pixel in an image to a 3D point were taken from:
# https://elcharolin.wordpress.com/2017/09/06/transforming-a-depth-map-into-a-3d-point-cloud/

from __future__ import print_function

import roslib
roslib.load_manifest('ros_cv')
import sys
import time
import numpy as np
import rospy
import cv2
import tf
import tf2_ros
from ros_cv.msg import Detect
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Point
import std_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
import message_filters


class Point_Transformer:


  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)
    self.point_pub = rospy.Publisher('Points', PointStamped, queue_size=10)

    self.bridge = CvBridge()
    self.listener = tf.TransformListener()

    self.depth_image_sub = message_filters.Subscriber("/zed/zed_node/depth/depth_registered",Image)#,self.callback)
    self.camera_info_sub = message_filters.Subscriber("/zed/zed_node/depth/camera_info", CameraInfo)#, self.callback)
    self.rgb_image_sub = message_filters.Subscriber("/zed/zed_node/rgb/image_rect_color",Image) 
    self.object_location_sub = message_filters.Subscriber("/detect", Detect)                        #!!!!!!!!!!!!!!!!!!!#

    sub_list = [self.depth_image_sub, self.camera_info_sub, self.rgb_image_sub, self.object_location_sub]    #!!!!!!!!!!!!!!!!!!!#
    self.ts = message_filters.ApproximateTimeSynchronizer(sub_list, 20,0.1)
    self.ts.registerCallback(self.callback)
    print("Subscribed. init done")



  def callback(self,depth_img_data, camera_info, rgb_img_data, object_location_info):      #!!!!!!!!!!!!!!!!!!!!!!!!#
    try:
      print("Here")
      depth_image = self.bridge.imgmsg_to_cv2(depth_img_data, "32FC1")
      rgb_image = self.bridge.imgmsg_to_cv2(rgb_img_data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    (rows,cols) = depth_image.shape
    norm_depth_image = cv2.normalize(depth_image, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)

    def ChangeFrame(PointStamped, target_frame = 'map'):

	# This function takes a PointStamped Message and converts its coordinates into the target_frame coordinates. 
        # This is done by tf.TransformerROS.transformPoint, however timing needs to be implemented in order for this code to work
        tf_buffer = tf2_ros.Buffer()
        tf2_listener = tf2_ros.TransformListener(tf_buffer)
        transform = tf_buffer.lookup_transform(target_frame,
                                               "zed_left_camera_optical_frame",
                                               rospy.Time.now(),
                                               rospy.Duration(1.0))        
        GlobalPoint = self.listener.transformPoint(target_frame, PointStamped)
        print ("Global (X,Y,Z)= (%s, %s, %s)" % (GlobalPoint.point.x,GlobalPoint.point.y,GlobalPoint.point.z))
	return GlobalPoint





    def DepthImageToXYZCoordinates(mdpt_x_of_object, mdpt_y_of_object, target_frame):
        
        cv2.circle(norm_depth_image,(mdpt_x_of_object,mdpt_y_of_object), 2, (0,0,255))

        # The ZED has 0.004 mm per pixel at 1280x720 resolution
        w = cols                                                   # image width in pixels
        h = rows                                                   # image height in pixels
        Focal_length_in_mm = 2.8                                   # Focal Length of ZED in mm 
	focal_length_in_pixels = camera_info.K[0]                  # Focal Length of ZED in pixels
        mm_per_pixel = Focal_length_in_mm/focal_length_in_pixels   # mm/pixel of the ZED. If running at 720x 1280 resolution, should be ~ 0.004 mm
 
        # FOV_d = 110                              # Field of View of the ZED in degrees
        # FOV_r = 110*np.pi/180                    # Field of View of the ZED in radians
    
        ### F = f*(W/w) #if you know the camera's digital sensor has a width W in millimiters, and the image width in pixels is w, you can convert the focal length f to world units
                #F is in mm, f is in pixel size, W is sensor width in whatever length unit you can get it in, and w is in pixels.


        image_width_in_mm = mm_per_pixel * w
        Theta_horizontal = np.arctan ((image_width_in_mm/2)/Focal_length_in_mm)
        FOV_horizontal = 2*Theta_horizontal
        #print ("Horizontal Field of View in Degrees = %s" % (FOV_horizontal*180/np.pi))

        image_height_in_mm = mm_per_pixel * h
        Theta_vertical = np.arctan ((image_height_in_mm/2)/Focal_length_in_mm)
        FOV_vertical = 2*Theta_vertical
        #print ("Vertical Field of View in Degrees = %s" % (FOV_vertical*180/np.pi))
 
      
        [i,j] = [mdpt_x_of_object, mdpt_y_of_object]# = [i,j]  #will be received from message from Josh
        depth = depth_image[j, i] #in meters
        cx_d = camera_info.K[2]
	cy_d = camera_info.K[5]
	fx_d = camera_info.K[0]                                    # Focal Length of ZED in pixels
	fy_d = camera_info.K[4]                                    # Focal Length of ZED in pixels   

        alpha_horizontal = (np.pi - FOV_horizontal)/2
        gamma_horizontal = alpha_horizontal + (i)*FOV_horizontal/(w)     #Not sure if should be i, or number of pixels from center of image
        x = -depth/(np.tan(gamma_horizontal))
        if abs(x)<0.001:
            x = 0

        alpha_vertical = 2*np.pi - (FOV_vertical)/2
        gamma_vertical = alpha_vertical + (j)*FOV_vertical/(h)   #Not sure if should be j, or number of pixels from center of image
        y = depth*np.tan(gamma_vertical)
        if abs(y)<0.001:
                y = 0
	 
        z = depth
        if abs(z)< 0.01:
                z = 0


	x_d = mdpt_x_of_object
	y_d = mdpt_y_of_object
	x_new = (x_d - cx_d) * depth / fx_d
	y_new = (y_d - cy_d) * depth / fy_d
	z_new = depth
	P3D = Point(x_new,y_new,z_new)

        #print ("   Local (X, Y, Z) = (%s,%s,%s)" % (x,y,z))
	#print ("   Global (X, Y, Z) = (%s,%s,%s)" % (P3D.x,P3D.y,P3D.z))

        # Configure the Header message file
        Header = std_msgs.msg.Header()
        # Header.seq is handled by the publisher
        Header.stamp = rospy.Time.now()
        Header.frame_id = 'zed_left_camera_optical_frame'

        # Use the points extracted from the depth image to fill in the Point message file
        point_msg = Point(float(x),float(y),float(z))

        # Create a PointStamped message file which consists of a Header.msg and Point.msg
        point_stamped_msg = PointStamped(Header, point_msg)
        P3D_Stamped = PointStamped(Header, P3D)

        return ChangeFrame(P3D_Stamped, target_frame)#, ChangeFrame(point_stamped_msg, target_frame)
    
    self.point_pub.publish(DepthImageToXYZCoordinates(int(object_location_info.x),int(object_location_info.y),'map'))  #!!!!!!!!!!!!!!!!!!!!#
    #cv2.imshow('mouseRGB', rgb_image)                                                     				#!!!!!!!!!!!!!!!!!!!!#
    cv2.waitKey(3)
    cv2.destroyAllWindows()    




def main(args):
  rospy.init_node('Point_Transformer', anonymous=True)
  print("\n")
  PT = Point_Transformer()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
