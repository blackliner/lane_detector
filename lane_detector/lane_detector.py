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
from natsort import natsorted


class LaneDetector(Node):
    def __init__(self, **kwargs):
        super().__init__("lane_detector")

        image_topic_ = self.declare_parameter("image_topic", "/image/image_raw").value

        self.frame_id_ = self.declare_parameter("frame_id", "camera").value

        self.image_publisher_ = self.create_publisher(Image, image_topic_, 5)

    def image_callback(self, image_path=None):
        if self.type == "video":
            rval, image = self.vc.read()
        elif image_path:
            image = cv2.imread(image_path)
        else:
            self.get_logger().error("Image path is none.")
            raise ValueError()

        time_msg = self.get_time_msg()
        img_msg = self.get_image_msg(image, time_msg)

        self.image_publisher_.publish(img_msg)

    def get_image_msg(self, image, time):

        img_msg = CvBridge().cv2_to_imgmsg(image, encoding="bgr8")
        img_msg.header.stamp = time
        img_msg.header.frame_id = self.frame_id_
        return img_msg


def main(args=None):
    rclpy.init(args=args)

    lane_detector = LaneDetector()

    rclpy.spin(lane_detector)

    lane_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
