#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
import laser_geometry.laser_geometry as lg

class LidarPointCloud:
    def __init__(self):
        rospy.init_node('lidar_pointcloud_node')
        self.lp = lg.LaserProjection()
        self.pc_pub = rospy.Publisher("/lidar/depth/points2", PointCloud2, queue_size=1)
        rospy.Subscriber("/lidar/scan", LaserScan, self.scan_cb, queue_size=1)

    def scan_cb(self, msg):
        pc2_msg = self.lp.projectLaser(msg)
        self.pc_pub.publish(pc2_msg)

if __name__ == '__main__':
    lidar_pointcloud = LidarPointCloud()
    rospy.spin()
