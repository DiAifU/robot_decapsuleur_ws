#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from darknet_detector.msg import Object
from sensor_msgs.msg import Image

def object_received(obj):
    frame = bridge.imgmsg_to_cv2(imgmsg, "bgr8")


    # Calcul de la couleur en fonction du label trouve
    name_sum = sum(ord(c) for c in obj.name)
    color = (name_sum%256, (name_sum*10)%256, (name_sum*50)%256)
    cv2.rectangle(frame, (obj.x1, obj.y1), (obj.x2, obj.y2), color, 1)
    cv2.putText(frame, obj.name, (obj.x1, obj.y1), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 1, cv2.LINE_AA)
    cv2.imshow("Image", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        exit(0)

if __name__ == "__main__":
    global bridge
    rospy.init_node("camerasim")
    cap = cv2.VideoCapture(0)
    bridge = CvBridge()
    pub = rospy.Publisher("rgb/image_raw", Image, queue_size=1)
    rospy.Subscriber("/darknet/detector", Object, object_received)

    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            imgmsg = bridge.cv2_to_imgmsg(frame, "bgr8")
            pub.publish(imgmsg)
        r.sleep()
