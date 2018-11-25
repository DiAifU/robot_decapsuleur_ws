#!/usr/bin/env python

import rospy
from darknet_detector.srv import ObjectInfo
from sensor_msgs.msg import Image
import os
import sys
darknet_path = os.path.dirname(os.path.realpath(__file__)).replace('scripts', 'darknet_files')
os.chdir(darknet_path)
sys.path.insert(0, darknet_path)
import darknet
from cv_bridge import CvBridge, CvBridgeError

cur_img = None

def contains_object(obj_info):
    return cur_img is not None and darknet.get_object(cur_img, obj_info.type) is not None

def image_received(image):
    global cur_img
    cur_img = bridge.imgmsg_to_cv2(image, "bgr8")


if __name__ == "__main__":
    global bridge
    rospy.init_node("darknet_find_object")
    bridge = CvBridge()
    darknet.load()
    rospy.Subscriber("/cv_camera_node/image_raw", Image, image_received)
    rospy.Service('darknet/find_object', ObjectInfo, contains_object)
    rospy.spin()
