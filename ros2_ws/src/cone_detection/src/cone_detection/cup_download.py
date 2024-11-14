#!/usr/bin/env python3

from time import sleep
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs_py import point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
import subprocess
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Point
import threading


class ConeDetectionNode(Node):
    def __init__(self):
        super().__init__('cone_detection_node')
        self.subscription_image = self.create_subscription(
            Image,
            '/D435/color/image_raw',
            self.image_callback,
            1)

        self.publisher = self.create_publisher(Point, 'cone_position', 10)
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
        self.bridge = CvBridge()
        self.depthImage = None
        self.fx = 192.72604370117188
        self.fy = 192.72604370117188
        self.cx = 162.08334350585938
        self.cy = 117.73448944091797
        self.run_once=True
        self.cv_image = None

    def image_callback(self, msg):
        print("hi")
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        print("Color Image dimension (Row,Col):", self.cv_image.shape[0], "x", self.cv_image.shape[1])
        image_filename = "1_no_cup.jpg"
        cv2.imwrite(image_filename, self.cv_image)

def main(args=None):
    rclpy.init(args=args)
    cone_detection_node = ConeDetectionNode()
    rclpy.spin(cone_detection_node)
    cone_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
