#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from darknet_detector.srv import ObjectInfo
from sensor_msgs.msg import Image

if __name__ == "__main__":
    global bridge
    rospy.init_node("camerasim")
    cap = cv2.VideoCapture(0)
    bridge = CvBridge()
    pub = rospy.Publisher("/cv_camera_node/image_raw", Image, queue_size=1)
    rospy.wait_for_service('/darknet/contains_object')
    contains_obj_srv = rospy.ServiceProxy('/darknet/contains_object', ObjectInfo)

    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            imgmsg = bridge.cv2_to_imgmsg(frame, "bgr8")
            pub.publish(imgmsg)

            reponse = contains_obj_srv('person')
            print(reponse)
        r.sleep()
