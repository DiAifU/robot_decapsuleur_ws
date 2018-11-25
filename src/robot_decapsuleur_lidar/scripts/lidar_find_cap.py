#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from dynamixel_msgs.msg import JointState
from robot_decapsuleur_lidar.msg import LidarPoint


class FindCap:

    min_data = None
    current_arm_angle = None
    d_angle_up = 0.1
    d_angle_down = 0.05
    d_dist_stop = 0.3
    d_angle_stop = 0.5

    def __init__(self):
        rospy.init_node('lidar_find_cap_node', anonymous=True)
        rospy.Subscriber("/lidar/minimum", LidarPoint, self.get_lidar_data)
        rospy.Service('/lidar/find_cap', Trigger, self.execute_find_cap)
        self.arm = rospy.Publisher('/joint3_controller/command', Float64, queue_size=0)
        rospy.Subscriber('/joint3_controller/state', JointState, self.get_arm_state)
        rospy.spin()

    def get_lidar_data(self, data):
            self.min_data = [(data.angle + math.pi)%(2*math.pi), data.distance]

    def move_robot(self, up):
        if self.current_arm_angle == None:
            return

        if up:
            self.arm.publish(self.current_arm_angle - self.d_angle_up)
        else:
            self.arm.publish(self.current_arm_angle + self.d_angle_down)
        rospy.sleep(0.1)


    def get_arm_state(self, state):
        if state.current_pos > 0.4 or state.current_pos < -3:
            print("DOING SHIIIIT")
            exit(-1)
        self.current_arm_angle = state.current_pos


    def execute_find_cap(self, ok):
        current_min_data = self.min_data
        found = False
        while not found:
            self.move_robot(True)
            if self.min_data[1] > d_dist_stop or abs(self.min_data[0] - current_min_data[0]) > d_angle_stop:
                found = True
        return [True, str(self.min_data)]



if __name__ == '__main__':
    try:
        find_cap = FindCap()
    except rospy.ROSInterruptException:
        pass
