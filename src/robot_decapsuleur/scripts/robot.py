#!/usr/bin/env python

import rospy
from enum import Enum
from std_msgs.msg import Float64
from darknet_detector.srv import ObjectInfo
import math
import time
from std_srvs.srv import Trigger
from robot_decapsuleur_lidar.msg import LidarPoint



class State(Enum):
    FIND_WITH_LIDAR = 0
    VERIFY_WITH_CAMERA = 1
    FIND_CAP = 2
    PLACE_ARM = 3
    REMOVE_CAP = 4

# min motor: -2.12, max: 2.62, zero: -2.35

class Robot:
    def __init__(self):
        # Constants
        self.ANGLE_ZERO_ROT = -2.35
        self.ANGLE_ZERO_ARM1 = -1.57
        self.ANGLE_ZERO_ARM2 = 0
        self.ERROR_STOP = 0.2

        # Variables
        self.state = State.FIND_WITH_LIDAR
        self.lidar_sub = None
        self.rot_pub = rospy.Publisher('/joint1_controller/command', Float64, queue_size=0)
        self.arm1_pub = rospy.Publisher('/joint2_controller/command', Float64, queue_size=0)
        self.arm2_pub = rospy.Publisher('/joint3_controller/command', Float64, queue_size=0)


    def debug(self, d):
        if self.debug: print(d)


    def lidar_found(self, data):
        if self.state != State.FIND_WITH_LIDAR: # unregister is taking some time
            return

        if data.distance > 0.3:
            return

        angle = (data.angle + self.ANGLE_ZERO_ROT - math.pi)%(2*math.pi) - math.pi
        self.debug(angle - self.ANGLE_ZERO_ROT)
        if abs(angle - self.ANGLE_ZERO_ROT) > self.ERROR_STOP :
            self.rot_pub.publish(angle)
        else:
            self.debug("Objet detecte")
            self.switch_state(State.VERIFY_WITH_CAMERA)


    def verify_with_cam(self):
        self.debug("Waiting for object detection service")
        rospy.wait_for_service('darknet/find_object')
        self.debug("Service found")
        contains_obj_srv = rospy.ServiceProxy('darknet/find_object', ObjectInfo)
        attempt = 0
        found = False
        while attempt < 5 and found == False:
            found = contains_obj_srv('person').found
            attempt = attempt + 1
            self.debug("Bottle found: {}, attempt {}".format(found, attempt))
        return found

    def find_cap(self):
        self.debug("Waiting for find cap service")
        rospy.wait_for_service('lidar/find_cap')
        self.debug("Service found")
        find_cap_srv = rospy.ServiceProxy('lidar/find_cap', Trigger)
        res = find_cap_srv()
        self.debug("Cap found at data: {}".format(res.message))

    def switch_state(self, new_state):
        self.debug("Switch to new state : {}".format(new_state))
        self.state = new_state

        # Reset all the state-specific variables
        if self.lidar_sub != None:
            self.lidar_sub.unregister()
            self.lidar_sub = None

    def start(self):
        rospy.init_node('robot_decapsuleur')
        while not rospy.is_shutdown():
            if self.state == State.FIND_WITH_LIDAR:
                if self.lidar_sub == None:
                    self.debug("Initialisation du robot")
                    time.sleep(3)
                    self.rot_pub.publish(self.ANGLE_ZERO_ROT)
                    self.arm1_pub.publish(self.ANGLE_ZERO_ARM1)
                    self.arm2_pub.publish(self.ANGLE_ZERO_ARM2)
                    time.sleep(3)
                    self.lidar_sub = rospy.Subscriber('/lidar/minimum', LidarPoint, self.lidar_found)
            elif self.state == State.VERIFY_WITH_CAMERA:
                if self.verify_with_cam():
                    self.switch_state(State.FIND_CAP)
                else:
                    self.switch_state(State.FIND_WITH_LIDAR)
            elif self.state == State.FIND_CAP:
                rospy.sleep(1)
                self.find_cap()
                self.switch_state(State.PLACE_ARM)
            elif self.state == State.PLACE_ARM:
                None
            elif self.state == State.REMOVE_CAP:
                None



if __name__ == '__main__':
    robot = Robot()
    robot.start()
