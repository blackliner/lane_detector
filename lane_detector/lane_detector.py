import rclpy
import argparse
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from datetime import datetime
import cv2
import os
import yaml
import numpy as np


class LaneDetector(Node):
    def __init__(self, **kwargs):
        super().__init__("lane_detector")

        self.br = CvBridge()

        image_topic_ = self.declare_parameter("image_topic", "/image/image_raw").value

        self.frame_id_ = self.declare_parameter("frame_id", "camera").value

        self.image_subscriber = self.create_subscription(Image, image_topic_, self.image_callback, 10)

        # self.lane_publisher_ = self.create_publisher(Image, image_topic_, 5)

    def image_callback(self, msg):
        # self.get_logger().info(f"Image received: {msg.width}x{msg.height}")

        img = self.br.imgmsg_to_cv2(msg)

        height, width, channels = img.shape
        # self.get_logger().info(f"OpenCV: {width}x{height}:{channels}")

        cv2.imshow("Received", img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    cv2.namedWindow("Received", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Received", 800, 400)
    cv2.moveWindow("Received", 20, 20)

    lane_detector = LaneDetector()

    rclpy.spin(lane_detector)

    lane_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
