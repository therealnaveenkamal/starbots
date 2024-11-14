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
from tensorflow.keras.models import load_model
import cv2
import numpy as np


class ConeDetectionNode(Node):
    def __init__(self):
        super().__init__('cone_detection_node')
        
        self.subscription_image = self.create_subscription(
            Image,
            'website/color/image',
            self.image_callback,
            10)
        
        self.depth_image = self.create_subscription(
            Image,
            'website/depth/image',
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
        self.point_got=True
        self.point_msg = Point()
        self.cv_image = None
        self.raw_image = None
        self.x_offset = -15
        self.y_offset = 22
        self.path = "/home/user/ros2_ws/src/cone_detection/src/cone_detection/"
        self.model = load_model(self.path+'starbots_cup_detection.h5')
        #self.model = load_model(self.path+'starbots_new_model.h5')
        self.web_publisher = self.create_publisher(Image, 'website/image', 10)

    def start_image_display_thread(self):
        display_thread = threading.Thread(target=self.display_loop)
        display_thread.daemon = True  # Make sure the thread terminates when the main program exits
        display_thread.start()
    
    def display_loop(self):
        while True:
            if self.cv_image is not None and self.raw_image is not None:
                cv2.namedWindow("Depth Image", cv2.WINDOW_NORMAL)
                cv2.namedWindow("Raw Image", cv2.WINDOW_NORMAL)
                cv2.imshow("Depth Image", self.raw_image)
                cv2.imshow("Raw Image", self.cv_image)
                cv2.waitKey(1)
                    


    def image_callback(self, msg):
        if(self.cv_image is None):
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            print("Color Image dimension (Row,Col):", self.cv_image.shape[0], "x", self.cv_image.shape[1])
            image_filename = self.path+"cone_detection_image.jpg"
            cv2.imwrite(image_filename, self.cv_image)
            #image_filename = self.path+"CUP_CLOSE_RAW.jpg"
            #cv2.imwrite(image_filename, self.cv_image)

    def di_callback(self, msg):
        cv_image_depth = CvBridge().imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depthImage = cv_image_depth
        print("Shape of the image:", cv_image_depth.shape)
        if self.run_once:
            print("Depth Image dimension (Row,Col):", cv_image_depth.shape[0], "x", cv_image_depth.shape[1])
            self.run_once = False
        
        
        if(self.cv_image is not None and self.depthImage is not None):
            data = msg.data
            depth_array = np.frombuffer(data, dtype=np.uint16)
            image_array = depth_array.reshape((240, 320))
            image = np.zeros((240, 320, 3), dtype=np.uint8)
            image[:, :, 0] = image_array[:,:]  # Red channel
            image[:, :, 1] = image_array[:,:]  # Green channel
            image[:, :, 2] = image_array[:,:]  # Blue channel

            im_msg = CvBridge().cv2_to_imgmsg(image, encoding='bgr8')
            self.web_publisher.publish(im_msg)

            #image_filename = self.path+"CUP_CLOSE_DEPTH.jpg"
            #cv2.imwrite(image_filename, image)
            
            new_image_path = self.path+'cone_detection_image.jpg'
            new_image = cv2.imread(new_image_path)
            new_image = new_image / 255.0
            new_image = np.expand_dims(new_image, axis=0)
            predicted_bbox = self.model.predict(new_image)
            predicted_w, predicted_h, predicted_x, predicted_y = predicted_bbox[0]

            pred_image = cv2.imread(new_image_path)
            w = int(predicted_w)
            h = int(predicted_h)
            x = int(predicted_x)
            y = int(predicted_y)
            
            centroid_x = (x + x + w) // 2
            centroid_y = (y + y + h) // 2

            raw_c_x = (x + x + w) // 2
            raw_c_y = (y + y + h) // 2
            
        

            centroid_x += self.x_offset
            centroid_y += self.y_offset


            #centroid_x = 210
            #centroid_y = 110
            #distance_val2 = 0.353
            print(centroid_x, centroid_y)
            distance_val2 = image_array[centroid_y, centroid_x]  # assuming the first channel holds the depth information
            distance_val2 = distance_val2 * 0.001
            print("Distance value2:", distance_val2, "m")

            if(distance_val2 == 0):
                print("CUP NOT DETECTED")
                cv2.destroyAllWindows()
            else:
                # Draw a circle at the specified point
                cv2.circle(image, (centroid_x, centroid_y), 5, (0, 0, 255), -1)
                cv2.circle(self.cv_image, (raw_c_x, raw_c_y), 5, (0, 0, 255), -1)
                
                x_final = distance_val2*(centroid_x - self.cx)/self.fx
                y_final = distance_val2*(centroid_y - self.cy)/self.fy - 0.29
                z_final = distance_val2

                #print(x_final, y_final, z_final)
                self.raw_image = image
                cv2.namedWindow("Depth Image", cv2.WINDOW_NORMAL)
                cv2.namedWindow("Raw Image", cv2.WINDOW_NORMAL)
                cv2.imshow("Depth Image", self.raw_image)
                cv2.imshow("Raw Image", self.cv_image)
                cv2.waitKey(1)
                
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

                    #print(target_pt.point.x, target_pt.point.y, target_pt.point.z)

                    error_correction = 0.003

                    
                    # Publish the transformed point
                    if(self.point_got):
                        self.point_msg.x = round(target_pt.point.x - error_correction, 3)
                        self.point_msg.y = round(target_pt.point.y - error_correction, 3)
                        self.point_msg.z = round(target_pt.point.z, 3)
                        self.point_got = False
                    
                    self.publisher.publish(self.point_msg)


                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    print("Exception occurred:", e)
            

def main(args=None):
    rclpy.init(args=args)
    cone_detection_node = ConeDetectionNode()
    #cone_detection_node.start_image_display_thread()
    rclpy.spin(cone_detection_node)
    cone_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
