#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg
import tf
import pcl_ros


class LidarPointCloud:
    def __init__(self):
        rospy.init_node('lidar_pointcloud_node')
        self.lp = lg.LaserProjection()
        self.pc_pub = rospy.Publisher("/lidar/depth/points2", PointCloud2, queue_size=1)
        self.listener = tf.TransformListener()

        rospy.Subscriber("/lidar/scan", LaserScan, self.scan_cb, queue_size=1)

    def scan_cb(self, msg):
        pc2_msg = self.lp.projectLaser(msg)
        print(pc2_msg)
        self.listener.waitForTransform("lidar", "base", rospy.Time(0),rospy.Duration(1.0))
        self.pc_pub.publish(pc2_msg)

if __name__ == '__main__':
    lidar_pointcloud = LidarPointCloud()
    rospy.spin()
