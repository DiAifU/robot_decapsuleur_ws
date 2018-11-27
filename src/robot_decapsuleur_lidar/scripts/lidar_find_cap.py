#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from robot_decapsuleur_lidar.msg import LidarPoint
from robot_decapsuleur_lidar.srv import LidarCapPoint


class FindCap:

    min_point = None
    current_arm_angle = None
    d_angle_up = 0.05
    d_angle_down = 0.05
    d_dist_stop = 1.5
    d_angle_stop = 0.5

    def __init__(self):
        rospy.init_node('lidar_find_cap_node', anonymous=True)
        rospy.Subscriber("/lidar/minimum", LidarPoint, self.get_lidar_data)
        rospy.Service('/lidar/find_cap', LidarCapPoint, self.execute_find_cap)
        self.arm = rospy.Publisher('/joint3_controller/command', Float64, queue_size=1)
        rospy.Subscriber('/joint3_controller/state', JointState, self.get_arm_state)
        rospy.spin()

    def get_lidar_data(self, point):
        self.min_point = point

    def move_robot(self, up):
        if self.current_arm_angle == None:
            return

        if up:
            self.arm.publish(self.current_arm_angle - self.d_angle_up)
        else:
            self.arm.publish(self.current_arm_angle + self.d_angle_down)
        rospy.sleep(0.1)


    def get_arm_state(self, state):
            #print("DOING SHIIIIT")
        self.current_arm_angle = state.current_pos


    def execute_find_cap(self, unused):
        print("UNUSSSEEEEED ARG IS :", unused)
        current_min_point = self.min_point
        last_valid_min_point = self.min_point
        found = False
        while not found:
            self.move_robot(True)
            if self.min_point.distance > self.d_dist_stop or \
                abs(self.min_point.angle - current_min_point.angle) > self.d_angle_stop:
                found = True
            else:
                last_valid_min_point = self.min_point


        return [last_valid_min_point, True]



if __name__ == '__main__':
    find_cap = FindCap()
