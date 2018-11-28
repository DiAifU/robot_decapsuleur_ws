#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


def image_received(imgmsg):
    cur_img = bridge.imgmsg_to_cv2(imgmsg, "bgr8")
    (h, w) = cur_img.shape[:2]
    # calculate the center of the image
    center = (w / 2, h / 2)

    M = cv2.getRotationMatrix2D(center, 90, 1.0)
    cur_img = cv2.warpAffine(cur_img, M, (w, h))

    imgmsg2 = bridge.cv2_to_imgmsg(cur_img, "bgr8")
    img_pub.publish(imgmsg2)

if __name__ == "__main__":
    global img_pub, bridge
    rospy.init_node("rotate_camera")
    bridge = CvBridge()

    img_pub = rospy.Publisher("camera/image_raw_rotated", Image, queue_size=1)
    rospy.Subscriber("cv_camera/image_raw", Image, image_received)
    rospy.spin()
