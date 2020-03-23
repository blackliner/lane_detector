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


def nothing(x):
    pass


class LaneDetector(Node):
    def __init__(self, **kwargs):
        super().__init__("lane_detector")


        self.br = CvBridge()

        image_topic_ = self.declare_parameter("image_topic", "/ImageRAW").value

        self.frame_id_ = self.declare_parameter("frame_id", "camera").value

        self.image_raw = None

        self.image_subscriber = self.create_subscription(Image, image_topic_, self.image_callback, 10)

        # self.lane_publisher_ = self.create_publisher(Image, image_topic_, 5)

    def image_callback(self, msg):
        self.image_raw = msg
        # self.get_logger().info(f"Image received: {msg.width}x{msg.height}")

    def execute(self):
        if self.image_raw is None:
            return
        img = self.br.imgmsg_to_cv2(self.image_raw)

        height, width, channels = img.shape
        # self.get_logger().info(f"OpenCV: {width}x{height}:{channels}")

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # hsv = img.copy()
        h_low = cv2.getTrackbarPos('H_low','img')
        h_high = cv2.getTrackbarPos('H_high','img')
        s_low = cv2.getTrackbarPos('S_low','img')
        s_high = cv2.getTrackbarPos('S_high','img')
        v_low = cv2.getTrackbarPos('V_low','img')
        v_high = cv2.getTrackbarPos('V_high','img')
        low_val = (0,10,121)
        high_val = (67,96,255)


        mask = cv2.inRange(hsv, low_val,high_val)

        # remove noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel=np.ones((16,16),dtype=np.uint8))
        # close mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel=np.ones((32,32),dtype=np.uint8))

        # # improve mask by drawing the convexhull 
        # contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # for cnt in contours:
        #     hull = cv2.convexHull(cnt)
        #     cv2.drawContours(mask,[hull],0,(255), -1)
        # # erode mask a bit to migitate mask bleed of convexhull
        # mask = cv2.morphologyEx(mask, cv2.MORPH_ERODE, kernel=np.ones((5,5),dtype=np.uint8))

        # remove this line, used to show intermediate result of masked road
        road = cv2.bitwise_and(img, img,mask=mask)

        # apply mask to hsv image
        road_hsv = cv2.bitwise_and(hsv, hsv,mask=mask)
        # set lower and upper color limits
        low_val = (0,0,0)
        high_val = (255,255,255)


        # low_val = (h_low,s_low,v_low)
        # high_val = (h_high,s_high,v_high)

        # Threshold the HSV image 
        mask2 = cv2.inRange(road_hsv, low_val,high_val)
        # apply mask to original image
        result = cv2.bitwise_and(img, img,mask=mask2)

        # Convert the image to gray-scale
        gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        
        low_val = (h_low)
        high_val = (h_high)
        # Threshold the HSV image         
        gray = cv2.inRange(gray, low_val,high_val)
        
        # Find the edges in the image using canny detector
        # edges = cv2.Canny(gray, 50, 200)

        # Detect points that form a line
        lines = cv2.HoughLinesP(gray, 1, np.pi/180, 50, minLineLength=100, maxLineGap=20)
        # Draw lines on the image
        final = img.copy()
        try:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(final, (x1, y1), (x2, y2), (255, 0, 0), 3)
        except:
            pass
        # Show result

        # cv2.imshow("img", img)
        cv2.imshow("result", result)
        cv2.imshow("road", road)
        cv2.imshow("gray", gray)
        cv2.imshow("final", final)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    cv2.namedWindow("img", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("img", 600, 400)
    cv2.createTrackbar("H_low",  "img" , 0, 255, nothing)
    cv2.createTrackbar("H_high",  "img" , 0, 255, nothing)
    cv2.createTrackbar("S_low",  "img" , 0, 255, nothing)
    cv2.createTrackbar("S_high",  "img" , 0, 255, nothing)
    cv2.createTrackbar("V_low",  "img" , 0, 255, nothing)
    cv2.createTrackbar("V_high",  "img" , 0, 255, nothing)

    cv2.namedWindow("result", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("result", 600, 400)

    cv2.namedWindow("road", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("road", 600, 400)

    cv2.namedWindow("gray", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("gray", 600, 400)

    cv2.namedWindow("final", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("final", 600, 400)

    lane_detector = LaneDetector()

    while(True):
        rclpy.spin_once(lane_detector, timeout_sec=0.05)
        lane_detector.execute()


    lane_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
