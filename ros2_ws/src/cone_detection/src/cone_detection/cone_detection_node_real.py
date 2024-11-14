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

        sleep(5)
        
        self.depth_image = self.create_subscription(
            Image,
            '/D435/depth/image_rect_raw',
            self.di_callback,
            10)

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
        image_filename = "cone_detection_result.jpg"
        cv2.imwrite(image_filename, self.cv_image)

    def di_callback(self, msg):
        cv_image_depth = CvBridge().imgmsg_to_cv2(msg)
        self.depthImage = cv_image_depth
        print("Shape of the image:", cv_image_depth.shape)
        if self.run_once:
            print("Depth Image dimension (Row,Col):", cv_image_depth.shape[0], "x", cv_image_depth.shape[1])
            self.run_once = False
        
        if(self.cv_image is not None and self.depthImage is not None):
            centroid_x = 210
            centroid_y = 120

            distance_val2 = self.depthImage[centroid_y, centroid_x] #True depth of the object
            #img = cv2.rectangle(self.depthImage, (centroid_x, centroid_y), (centroid_x+1, centroid_y+1), (0, 0, 255), 2)
            print("Distance value2:", distance_val2, "m")

            #cv2.imshow("Cone Detection", img)
            #cv2.waitKey(1)

            x_final = distance_val2*(centroid_x - self.cx)/self.fx
            y_final = distance_val2*(centroid_y - self.cy)/self.fy
            z_final = distance_val2

            print(x_final, y_final, z_final)

            #subprocess.run(["ros2", "run", "tf2_ros", "static_transform_publisher",str(x_final), str(y_final), str(z_final), "0", "0", "0", "1","D435_depth_optical_frame", "coffee"])

            """
            try:
                # Create a point stamped message
                point_stamped = PointStamped()
                point_stamped.header.frame_id = 'D435_depth_optical_frame'
                point_stamped.point.x = x_final
                point_stamped.point.y = y_final
                point_stamped.point.z = float(z_final)

                
                target_pt = self.tf_buf.transform(point_stamped, "base_link")
                # Transform point to base_link frame
                #transformed_point = self.tf_buffer.transform(point_stamped, 'base_link')

                print(target_pt.point.x, target_pt.point.y, target_pt.point.z)
                subprocess.run(["ros2", "run", "tf2_ros", "static_transform_publisher",
                    str(target_pt.point.x), str(target_pt.point.y), str(target_pt.point.z),
                    "0", "0", "0", "1",
                    "base_link", "coffee"])

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    print("Exception occurred:", e)
            """



def main(args=None):
    rclpy.init(args=args)
    cone_detection_node = ConeDetectionNode()
    rclpy.spin(cone_detection_node)
    cone_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
