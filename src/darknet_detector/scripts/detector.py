#!/usr/bin/env python

import rospy
import sys
import os
from darknet_detector.msg import Object
from sensor_msgs.msg import Image
darknet_path = os.path.dirname(os.path.realpath(__file__)).replace('scripts', 'darknet_files')
sys.path.insert(0, darknet_path)
import darknet
from cv_bridge import CvBridge, CvBridgeError

# ENABLE_TESTS=1
# if ENABLE_TESTS: import cv2



class ImageDetector:
    def __init__(self):
        rospy.init_node("darknet_detector")
        darknet.load()
        self.bridge = CvBridge()
        rospy.Subscriber("rgb/camera_info", Image, self.image_received)
        self.pub = rospy.Publisher('darknet/detector', Object, queue_size=1)
        # if ENABLE_TESTS: self.image_received(self.bridge.cv2_to_imgmsg(cv2.imread('/home/nicolas/Documents/TP Vision/Darknet/darknet/data/dog.jpg'), "bgr8"))

    def image_received(self, image):
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        obj_infos = darknet.get_first_object(cv_image)

        # if ENABLE_TESTS:
        #     print(obj_infos)
        #     while True:
        #         cv2.imshow("Image window", cv_image)
        #         if cv2.waitKey(0) & 0xFF == ord('q'):
        #             break

        if obj_infos:
            obj = Object()
            obj.name = obj_infos[0]
            obj.proba = obj_infos[1]
            obj.x1 = obj_infos[2][0]
            obj.y1 = obj_infos[2][1]
            obj.x2 = obj_infos[3][0]
            obj.y2 = obj_infos[3][1]
            self.pub.publish(obj)

if __name__ == "__main__":
    ImageDetector()
    # rospy.spin()
