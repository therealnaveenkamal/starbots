#!/usr/bin/env python3

from time import sleep
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import subprocess


class RawBroadcast(Node):
    def __init__(self):
        super().__init__('raw_broadcast')
        self.subscription_image_raw = self.create_subscription(
            Image,
            '/D435/color/image_raw',
            self.image_raw_callback,
            1)
        self.subscription_image_raw_flag = True
        self.subscription_image_rect_raw = None

        self.web_publisher_color = self.create_publisher(Image, 'website/color/image', 10)
        self.web_publisher_depth = self.create_publisher(Image, 'website/depth/image', 10)

    def switch_subscription(self):
        if self.subscription_image_raw_flag:
            if self.subscription_image_rect_raw is None:
                self.subscription_image_rect_raw = self.create_subscription(
                    Image,
                    '/D435/depth/image_rect_raw',
                    self.image_rect_raw_callback,
                    1)
                
                self.subscription_image_raw = None
        else:
            if self.subscription_image_raw is None:
                self.subscription_image_raw = self.create_subscription(
                    Image,
                    '/D435/color/image_raw',
                    self.image_raw_callback,
                    1)
                
                self.subscription_image_rect_raw = None

        self.subscription_image_raw_flag = not self.subscription_image_raw_flag


    def image_raw_callback(self, msg):
        print("Received image_raw")
        self.web_publisher_color.publish(msg)
        self.destroy_subscription(self.subscription_image_raw)
        self.switch_subscription()

    def image_rect_raw_callback(self, msg):
        print("Received image_rect_raw")
        self.web_publisher_depth.publish(msg)
        self.destroy_subscription(self.subscription_image_rect_raw)
        self.switch_subscription()


def main(args=None):
    rclpy.init(args=args)
    cone_detection_node = RawBroadcast()
    rclpy.spin(cone_detection_node)
    cone_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
