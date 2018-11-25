#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from robot_decapsuleur_lidar.msg import LidarPoint

def get_lidar_data(laserscan):
	minimum   = min(laserscan.ranges)
	data = LidarPoint()
	data.angle = laserscan.ranges.index(minimum)*laserscan.angle_increment
	data.distance = minimum
	pub.publish(data)



if __name__ == '__main__':
	try:
		rospy.init_node('lidar_min_node', anonymous=True)
		rospy.Subscriber("/lidar/scan", LaserScan, get_lidar_data)
		pub = rospy.Publisher('/lidar/minimum', LidarPoint, queue_size=10)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
