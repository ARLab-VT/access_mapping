#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
import random
import message_filters
import numpy as np
import std_msgs.msg
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler


class OccupancyGridder():


  def __init__(self):
    self.grid_pub = rospy.Publisher('GRID', OccupancyGrid, queue_size=10)

    self.point_sub = message_filters.Subscriber("Points", PointStamped) 

    sub_list = [self.point_sub]
    self.ts = message_filters.ApproximateTimeSynchronizer(sub_list, 20,0.1)
    self.ts.registerCallback(self.callback)


  def callback(self,point_stamped_data):

	def GridMaker(PointStamped):
	    rate = rospy.Rate(10) # 10hz
	    while not rospy.is_shutdown():
	
        	x = int(PointStamped.point.x/resolution)
		
		y_corrected = PointStamped.point.y + 2.5
                print("x = %s" % (x))      
	        y = int(y_corrected/resolution)
		print("y = %s" % (y))
                
	        index = int(y + x*width)
		print("index = %s" % (index))
                
	        #data[index] = random.randint(1,101)
		data[index] = 100
       
                #n = (width*height - 1)/2
                #for i in range(1,n):
		#    data[2*i+1] = 100

                
       
	        return index, data[index]
       

	Occupancy_Grid.data[GridMaker(point_stamped_data)[0]] = GridMaker(point_stamped_data)[1] 
        self.grid_pub.publish(Occupancy_Grid)
    

def main(args):
        OG = OccupancyGridder()
        try:
          rospy.spin()
        except KeyboardInterrupt:
          print("Shutting down")


if __name__ == '__main__':

        rospy.init_node('OccupancyGrid', anonymous=True)

	### MAKE AN OCCUPANCY GRID ###	
	# Configure the Header message file
	header = std_msgs.msg.Header()
	# Header.seq is handled by the publisher
	header.stamp = rospy.Time.now()
	header.frame_id = 'map'
	

	map_load_time = rospy.Time(0)

	resolution = .05     #m/cell

	width = 100
	height = 100

	x = 0
	y = 0
	z = 0
	position = Point(x,y,z)

	q = quaternion_from_euler(0, np.pi,-np.pi/2)  # (rot about x, rot about y, rot about z)
	xq = q[0]
	yq = q[1]
	zq = q[2]
	wq = q[3]
	orientation = Quaternion(xq,yq,zq,wq)

	origin  = Pose(position, orientation)

	info = MapMetaData(map_load_time, resolution, width, height, origin)

	n = width*height
	data = np.empty(shape=[n])

	for i in range(9999):
	        data[i] = 0
        for i in range(50):
                data[2*i] = 100
		data[2*i+1] = 50

	Occupancy_Grid = OccupancyGrid(header, info, data)
	##############################



	main(sys.argv)


