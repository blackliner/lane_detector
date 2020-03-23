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
from argparse import Namespace


def nothing(x):
    pass


trackbar_list = [
    ("h_low", 0, 0, 179),
    ("h_high", 179, 0, 179),
    ("s_low", 125, 0, 255),
    ("s_high", 255, 0, 255),
    ("v_low", 29, 0, 255),
    ("v_high", 142, 0, 255),
    ("g_low", 0, 0, 255),
    ("g_high", 0, 0, 255),
    ("x1", 980, 0, 1920),
    ("w1", 700, 0, 3840),
    ("y1", 800, 0, 1080),
    ("x2", 960, 0, 1920),
    ("w2", 700, 0, 3840),
    ("y2", 1080, 0, 1080),
]


def addTrackBars(window_name, trackbar_list):
    for name, value, min_val, max_val in trackbar_list:
        cv2.createTrackbar(name, window_name, min_val, max_val, nothing)
        cv2.setTrackbarPos(name, window_name, value)


def getTrackBars(window_name, trackbar_list):
    retval = {}
    for name, value, min_val, max_val in trackbar_list:
        retval[name] = cv2.getTrackbarPos(name, window_name)

    return Namespace(**retval)


class LaneDetector(Node):
    def __init__(self, **kwargs):
        super().__init__("lane_detector")

        self.br = CvBridge()

        image_topic_ = self.declare_parameter("image_topic", "/ImageRAW").value

        self.frame_id_ = self.declare_parameter("frame_id", "camera").value

        self.image_raw = None

        self.image_subscriber = self.create_subscription(Image, image_topic_, self.image_callback, 1)

        # self.lane_publisher_ = self.create_publisher(Image, image_topic_, 5)

    def image_callback(self, msg):
        self.image_raw = msg
        # self.get_logger().info(f"Image received: {msg.width}x{msg.height}")

    def execute(self):
        if self.image_raw is None:
            return
        img = self.br.imgmsg_to_cv2(self.image_raw)

        bar_data = getTrackBars("controls", trackbar_list)

        height, width, channels = img.shape

        top_left = [bar_data.x1 - bar_data.w1 / 2, bar_data.y1]
        bot_left = [bar_data.x2 - bar_data.w2 / 2, bar_data.y2]
        bot_right = [bar_data.x2 + bar_data.w2 / 2, bar_data.y2]
        top_right = [bar_data.x1 + bar_data.w1 / 2, bar_data.y1]

        src = np.float32([[0, 1080], [1920, 1080], top_right, top_left])
        dst = np.float32([bot_left, bot_right, [1920, 0], [0, 0]])
        M = cv2.getPerspectiveTransform(src, dst)  # The transformation matrix
        Minv = cv2.getPerspectiveTransform(dst, src)  # Inverse transformation

        bird = cv2.warpPerspective(img, M, (width, height))  # Image warping

        hsv = cv2.cvtColor(bird, cv2.COLOR_BGR2HLS)
        # hsv = img.copy()

        low_val = (0, 0, 0)
        high_val = (179, 255, 100)

        low_val = (bar_data.h_low, bar_data.s_low, bar_data.v_low)
        high_val = (bar_data.h_high, bar_data.s_high, bar_data.v_high)

        mask = cv2.inRange(hsv, low_val, high_val)

        # cv2.rectangle(mask, (0, 0), (1920, 800), (0, 0, 0), -1)

        # close mask
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, kernel=np.ones((bar_data.g_high, bar_data.g_high), dtype=np.uint8)
        )

        # remove noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel=np.ones((bar_data.g_low, bar_data.g_low), dtype=np.uint8))

        # # improve mask by drawing the convexhull
        # contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # for cnt in contours:
        #     hull = cv2.convexHull(cnt)
        #     cv2.drawContours(mask, [hull], 0, (255), -1)
        # # erode mask a bit to migitate mask bleed of convexhull
        # mask = cv2.morphologyEx(mask, cv2.MORPH_ERODE, kernel=np.ones((64, 64), dtype=np.uint8))

        # remove this line, used to show intermediate result of masked road
        road = cv2.bitwise_and(bird, bird, mask=mask)

        # apply mask to hsv image
        road_hsv = cv2.bitwise_and(hsv, hsv, mask=mask)

        # set lower and upper color limits
        low_val = (1, 1, 1)
        high_val = (255, 255, 255)

        # Threshold the HSV image
        mask2 = cv2.inRange(road_hsv, low_val, high_val)
        # apply mask to original image
        result = cv2.bitwise_and(bird, bird, mask=mask2)

        # Find the edges in the image using canny detector
        # edges = cv2.Canny(gray, 50, 200)

        # Detect points that form a line
        lines = cv2.HoughLinesP(mask, 1, np.pi / 180, 50, minLineLength=100, maxLineGap=20)
        # Draw lines on the image
        try:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(bird, (x1, y1), (x2, y2), (255, 0, 0), 3)

        except:
            pass
        # Show result

        # cv2.imshow("controls", img)
        cv2.imshow("img", img)
        cv2.imshow("bird", bird)
        cv2.imshow("mask", mask)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    cv2.namedWindow("controls", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("controls", 600, 400)
    addTrackBars("controls", trackbar_list)

    cv2.namedWindow("img", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("img", 600, 400)

    cv2.namedWindow("bird", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("bird", 600, 400)

    cv2.namedWindow("mask", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("mask", 600, 400)

    lane_detector = LaneDetector()

    while True:
        rclpy.spin_once(lane_detector, timeout_sec=0.05)
        lane_detector.execute()

    lane_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
