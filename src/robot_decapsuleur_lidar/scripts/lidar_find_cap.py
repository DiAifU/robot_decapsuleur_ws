#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from robot_decapsuleur_lidar.msg import LidarPoint
from robot_decapsuleur_lidar.srv import LidarCapPoint
import numpy as np


class FindCap:

    min_point = None
    current_arm_angle = None
    d_angle_up = 0.05
    d_angle_down = 0.05
    d_dist_stop = 0.3
    d_angle_stop = 0.5

    def __init__(self):
        rospy.init_node('lidar_find_cap_node', anonymous=True)
        rospy.Subscriber("/lidar/scan", LaserScan, self.get_lidar_data)
        rospy.Service('/lidar/find_cap', LidarCapPoint, self.execute_find_cap)
        self.arm = rospy.Publisher('/joint3_controller/command', Float64, queue_size=1)
        rospy.Subscriber('/joint3_controller/state', JointState, self.get_arm_state)
        rospy.spin()

    def get_lidar_data(self, laserscan):
    	ranges = list(laserscan.ranges)

    	# Remove a third in the back
    	length = len(ranges)
    	for (i, r) in enumerate(ranges):
    		if (i > len(ranges)//3 and i < 2*len(ranges)//3) or ranges[i] < 0.3 or ranges[i] > 0.8:
    			ranges[i] = np.inf


    	minimum   = min(ranges)
        if minimum != np.inf:
            self.min_point = LidarPoint()
            self.min_point.angle = (ranges.index(minimum)*laserscan.angle_increment - math.pi)%(2*math.pi)
            self.min_point.distance = minimum

    def move_robot(self, up):
        if self.current_arm_angle == None:
            return

        if up:
            self.arm.publish(self.current_arm_angle - self.d_angle_up)
        else:
            self.arm.publish(self.current_arm_angle + self.d_angle_down)
        rospy.sleep(0.25)


    def get_arm_state(self, state):
            #print("DOING SHIIIIT")
        self.current_arm_angle = state.current_pos


    def execute_find_cap(self, unused):
        print("UNUSSSEEEEED ARG IS :", unused)
        current_min_point = self.min_point
        found = False
        while not found:
            self.move_robot(True)
            print(current_min_point.distance, self.min_point.distance, abs(current_min_point.distance - self.min_point.distance))
            if  abs(current_min_point.distance - self.min_point.distance) > self.d_dist_stop or \
                abs(self.min_point.angle - current_min_point.angle) > self.d_angle_stop:
                found = True


        return [self.min_point, True]



if __name__ == '__main__':
    find_cap = FindCap()
