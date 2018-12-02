#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from robot_decapsuleur_lidar.msg import LidarPoint
import numpy as np

def get_lidar_data(laserscan):
	ranges = list(laserscan.ranges)

	# Remove a third in the back
	length = len(ranges)
	for (i, r) in enumerate(ranges):
		if (i > len(ranges)//3 and i < 2*len(ranges)//3) or ranges[i] < 0.3 or ranges[i] > 0.6:
			ranges[i] = np.inf


	minimum   = min(ranges)
	if minimum != np.inf:
		data = LidarPoint()
		data.angle = (ranges.index(minimum)*laserscan.angle_increment - math.pi)%(2*math.pi)
		data.distance = minimum
		pub.publish(data)



if __name__ == '__main__':
	try:
		rospy.init_node('lidar_min_node', anonymous=True)
		rospy.Subscriber("/lidar/scan", LaserScan, get_lidar_data)
		pub = rospy.Publisher('/lidar/minimum', LidarPoint, queue_size=1)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
