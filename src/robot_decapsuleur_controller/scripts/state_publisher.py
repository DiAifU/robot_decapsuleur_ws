#!/usr/bin/env python

import rospy

from sensor_msgs.msg import JointState as JointStateMoveIt
from dynamixel_msgs.msg import JointState as JointStateDynamixel
from std_msgs.msg import Int16, Header

JOINT_1_INDEX = 0
JOINT_2_INDEX = 1

class JointStatePublisher():

    def __init__(self):
        rospy.init_node('state_publisher')

        rate = 15 # Hz
        r = rospy.Rate(rate)


        self.name = ['', '', '', '']
        self.header = Header()
        self.current_pos = [0, 0, 0, 0]
        self.velocity = [0, 0, 0, 0]
        self.load = [0, 0, 0, 0]


        # Start controller state subscribers
        for i in range(len(self.name)):
            rospy.Subscriber('/joint{}_controller/state'.format(i+1), JointStateDynamixel, self.joint_state_callback(i))

        # Start publisher
        self.joint_states_pub = rospy.Publisher('/joint_states', JointStateMoveIt, queue_size=10)

        while self.name[0] == '' or self.name[1] == '' or self.name[2] == '' or self.name[3] == '':
            r.sleep()

        #r.sleep()

        rospy.loginfo("Publishing joint_state at " + str(rate) + "Hz")

        while not rospy.is_shutdown():
            self.publish_joint_states()
            r.sleep()



    def joint_state_callback(self, index):
        def joint_state_handler(msg):
            self.name[index] = msg.name
            self.header = msg.header
            self.current_pos[index] = msg.current_pos
            self.velocity[index] = msg.velocity
            self.load[index] = msg.load
        return joint_state_handler


    def publish_joint_states(self):
        s = JointStateMoveIt()
        s.header = self.header
        s.name = self.name
        s.position = self.current_pos
        s.velocity = self.velocity
        s.effort = self.load
        self.joint_states_pub.publish(s)


if __name__ == '__main__':
    JointStatePublisher()
