#!/usr/bin/env python3

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
            '/wrist_rgbd_depth_sensor/image_raw',
            self.image_callback,
            10)

        self.depth_image = self.create_subscription(
            Image,
            '/wrist_rgbd_depth_sensor/depth/image_raw',
            self.di_callback,
            10)

        self.publisher = self.create_publisher(Point, 'cone_position', 10)

    
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        self.bridge = CvBridge()
        self.depthImage = None
        self.fx = 520.7813804684724
        self.fy = 521
        self.cx = 520.7813804684724
        self.cy = 240.5
        self.run_once=True
        self.cv_image = None

    def start_image_display_thread(self):
        display_thread = threading.Thread(target=self.display_loop)
        display_thread.daemon = True  # Make sure the thread terminates when the main program exits
        display_thread.start()
    
    def display_loop(self):
        while True:
            if self.cv_image is not None:
                cv2.imshow("Cone Detection", self.cv_image)
                cv2.waitKey(1)


    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Thresholding to separate object from background
        _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)

        # Contour detection
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Finding the contour with maximum area
        max_contour = max(contours, key=cv2.contourArea)

        # Getting the bounding box of the contour
        x, y, w, h = cv2.boundingRect(max_contour)

        # Drawing the bounding box
        
        cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # Calculate centroid
        centroid_x = (x + x + w) // 2
        
        centroid_y = (y + y + h) // 2

        cv2.rectangle(cv_image, (centroid_x, centroid_y), (centroid_x+1, centroid_y+1), (0, 0, 255), 2)

        max_val = cv2.minMaxLoc(self.depthImage)[1]
        print("Max value:", max_val)

        # Get global min depth value
        min_val = cv2.minMaxLoc(self.depthImage)[0]
        print("Min value:", min_val)

        if(max_val == 0 and min_val == 0):
            print("Can't find depth, Retrying...")
        else:
            # Get depth value at a point
            distance_val1 = self.depthImage[centroid_x, centroid_y]
            print("Distance value1:", distance_val1, "m")

            distance_val2 = self.depthImage[centroid_y, centroid_x] #True depth of the object
            print("Distance value2:", distance_val2, "m")

            # Displaying the image
            self.cv_image = cv_image
            x_final = distance_val2*(centroid_x - self.cx)/self.fx + 0.20
            y_final = distance_val2*(centroid_y - self.cy)/self.fy
            z_final = distance_val2
            # Transform to base_link frame
            try:
                # Create a point stamped message
                point_stamped = PointStamped()
                point_stamped.header.frame_id = 'wrist_rgbd_camera_depth_optical_frame'
                point_stamped.point.x = x_final
                point_stamped.point.y = y_final
                point_stamped.point.z = float(z_final)

                
                target_pt = self.tf_buf.transform(point_stamped, "base_link")
                # Transform point to base_link frame
                #transformed_point = self.tf_buffer.transform(point_stamped, 'base_link')

                print(target_pt.point.x, target_pt.point.y, target_pt.point.z)
                error_x_cup = 0.0409587541
                error_y_cup = 0.0468214836

                # Publish the transformed point
                while(True):
                    point_msg = Point()
                    point_msg.x = round(target_pt.point.x - error_x_cup, 3)
                    point_msg.y = round(target_pt.point.y - error_y_cup, 3)
                    point_msg.z = round(target_pt.point.z, 3)
                    self.publisher.publish(point_msg)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print("Exception occurred:", e)

    def di_callback(self, msg):
        cv_image_depth = CvBridge().imgmsg_to_cv2(msg)
        self.depthImage = cv_image_depth
        if self.run_once:
            print("Image dimension (Row,Col):", cv_image_depth.shape[0], "x", cv_image_depth.shape[1])
            self.run_once = False


def main(args=None):
    rclpy.init(args=args)
    cone_detection_node = ConeDetectionNode()
    cone_detection_node.start_image_display_thread()
    rclpy.spin(cone_detection_node)
    cone_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
