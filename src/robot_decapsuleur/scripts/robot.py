#!/usr/bin/env python

import rospy
from enum import Enum
from std_msgs.msg import Float64, Bool
from darknet_detector.srv import ObjectInfo
import math
import time
from robot_decapsuleur_lidar.msg import LidarPoint
from robot_decapsuleur_lidar.srv import LidarCapPoint
import tf
from tf.msg import tfMessage
from geometry_msgs.msg import TransformStamped, PoseStamped
from dynamixel_controllers.srv import SetSpeed
import moveit_commander
import sys

class State(Enum):
    FIND_WITH_LIDAR = 0
    VERIFY_WITH_CAMERA = 1
    FIND_CAP = 2
    PLACE_ARM = 3
    REMOVE_CAP = 4
    SLEEP = 5

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
        self.cap_point = None
        rospy.Subscriber("/sleep_mode", Bool, self.set_sleep_mode)

        self.rot_pub = rospy.Publisher('/joint1_controller/command', Float64, queue_size=1)
        self.arm1_pub = rospy.Publisher('/joint2_controller/command', Float64, queue_size=1)
        self.arm2_pub = rospy.Publisher('/joint3_controller/command', Float64, queue_size=1)

        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=1)
        self.tf_listener = tf.TransformListener()

        rospy.sleep(90)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")


    def debug(self, d):
        if self.debug: print(d)

    def set_sleep_mode(self, data):
        if self.state == State.SLEEP and not data.data:
            self.switch_state(State.FIND_WITH_LIDAR)
        elif self.state != State.SLEEP and data.data:
            self.switch_state(State.SLEEP)


    def lidar_found(self, data):
        if self.state != State.FIND_WITH_LIDAR: # unregister is taking some time
            return

        if data.distance < 0.3 or data.distance > 0.8:
            return

        angle = (data.angle + self.ANGLE_ZERO_ROT)%(2*math.pi) - math.pi

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
        find_cap_srv = rospy.ServiceProxy('lidar/find_cap', LidarCapPoint)
        res = find_cap_srv()
        self.debug("Cap found at data: {}".format(res))
        return res

    def switch_state(self, new_state):
        self.debug("Switch to new state : {}".format(new_state))
        self.state = new_state

        # Reset all the state-specific variables
        if self.lidar_sub != None:
            self.lidar_sub.unregister()
            self.lidar_sub = None

    def set_motors_speed(self, speed):
        for i in range(1, 4):
            topic = '/joint{}_controller/set_speed'.format(i)
            rospy.wait_for_service(topic)
            set_speed = rospy.ServiceProxy(topic, SetSpeed)
            set_speed(speed)

    def start(self):
        rospy.init_node('robot_decapsuleur')
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            if self.cap_point != None:
                t = TransformStamped()
                t.header.frame_id = "lidar"
                t.header.stamp = rospy.Time.now()
                t.child_frame_id = "cap"
                t.transform.translation.x = self.cap_point.distance * math.cos(self.cap_point.angle - math.pi/2)
                t.transform.translation.y = self.cap_point.distance * math.sin(self.cap_point.angle - math.pi/2)
                t.transform.rotation.w = 1.0

                tfm = tfMessage([t])
                self.pub_tf.publish(tfm)


            if self.state == State.FIND_WITH_LIDAR:
                if self.lidar_sub == None:
                    self.debug("Initialisation du robot")
                    self.set_motors_speed(1.0)
                    self.rot_pub.publish(self.ANGLE_ZERO_ROT)
                    self.arm1_pub.publish(self.ANGLE_ZERO_ARM1)
                    self.arm2_pub.publish(self.ANGLE_ZERO_ARM2)
                    time.sleep(3)
                    self.set_motors_speed(0.3)
                    self.lidar_sub = rospy.Subscriber('/lidar/minimum', LidarPoint, self.lidar_found)
            elif self.state == State.VERIFY_WITH_CAMERA:
                if self.verify_with_cam():
                    self.switch_state(State.FIND_CAP)
                else:
                    self.switch_state(State.FIND_WITH_LIDAR)
            elif self.state == State.FIND_CAP:
                rospy.sleep(1)
                cap_data = self.find_cap()
                if not cap_data.success:
                    self.switch_state(State.FIND_WITH_LIDAR)
                else:
                    self.cap_point = cap_data.point
                    self.switch_state(State.PLACE_ARM)
            elif self.state == State.PLACE_ARM:
                # Compute pose in lidar frame
                p = PoseStamped()
                p.header.frame_id = "lidar"
                p.header.stamp = rospy.Time.now()
                p.pose.orientation.w = 1.0
                p.pose.position.x = self.cap_point.distance * math.cos(self.cap_point.angle - math.pi/2)
                p.pose.position.y = self.cap_point.distance * math.sin(self.cap_point.angle - math.pi/2)

                self.group.set_joint_value_target(p, True)
                self.group.plan()
                self.group.go(wait=True)


            elif self.state == State.REMOVE_CAP:
                None
            elif self.state == State.SLEEP:
                None



if __name__ == '__main__':
    robot = Robot()
    robot.start()
