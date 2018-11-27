#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from robot_decapsuleur_lidar.msg import LidarPoint
import numpy as np

def get_lidar_data(laserscan):
	ranges = list(laserscan.ranges)
	ranges[len(laserscan.ranges)//3:2*len(laserscan.ranges)//3] = [np.inf for _ in range(len(laserscan.ranges)//3)]
	minimum   = min(ranges)
	data = LidarPoint()
	data.angle = (ranges.index(minimum)*laserscan.angle_increment - math.pi)%(2*math.pi)
	data.distance = minimum
	# print(ranges.index(minimum), (ranges.index(minimum)*laserscan.angle_increment - math.pi)%(2*math.pi))
	pub.publish(data)



if __name__ == '__main__':
	try:
		rospy.init_node('lidar_min_node', anonymous=True)
		rospy.Subscriber("/lidar/scan", LaserScan, get_lidar_data)
		pub = rospy.Publisher('/lidar/minimum', LidarPoint, queue_size=1)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
