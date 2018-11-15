#!/usr/bin/env python

import rospy
import sys
import os
from darknet_detector.msg import Object
from sensor_msgs.msg import Image
darknet_path = os.path.dirname(os.path.realpath(__file__)).replace('scripts', 'darknet_files')
sys.path.insert(0, darknet_path)
os.chdir(darknet_path)
import darknet
from cv_bridge import CvBridge, CvBridgeError


class ImageDetector:
    def __init__(self):
        rospy.init_node("darknet_detector")
        darknet.load()
        self.bridge = CvBridge()
        if sys.argv[1] == "all":
            fct = self.get_first_object
        else:
            fct = self.get_specific_object
        rospy.Subscriber("rgb/image_raw", Image, fct)
        self.pub = rospy.Publisher('darknet/detector', Object, queue_size=1)

    def publish_obj(self, obj_infos):
        obj = Object()
        obj.name = obj_infos[0]
        obj.proba = obj_infos[1]
        obj.x1 = obj_infos[2][0]
        obj.y1 = obj_infos[2][1]
        obj.x2 = obj_infos[3][0]
        obj.y2 = obj_infos[3][1]
        self.pub.publish(obj)

    def get_first_object(self, image):
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        obj_infos = darknet.get_first_object(cv_image)
        if obj_infos: self.publish_obj(obj_infos)


    def get_specific_object(self, image):
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        obj_infos = darknet.get_object(cv_image, sys.argv[1])
        if obj_infos: self.publish_obj(obj_infos)


if __name__ == "__main__":
    ImageDetector()
    rospy.spin()
