#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from rcl_interfaces.msg import SetParametersResult


class TrafficLightColorDetector(Node):
    """
    Determines whether a YOLO-detected traffic light crop is red.

    Subscribes:
        /traffic_light_crop : Image

    Publishes:
        /traffic_light_red : Bool
    """

    def __init__(self):
        super().__init__("traffic_light_color_detector")

        self.declare_parameter("red_pixel_ratio_threshold", 0.02)
        self.declare_parameter("min_red_area", 20)

        self.red_pixel_ratio_threshold = (
            self.get_parameter("red_pixel_ratio_threshold")
            .get_parameter_value()
            .double_value
        )

        self.min_red_area = (
            self.get_parameter("min_red_area")
            .get_parameter_value()
            .integer_value
        )

        self.bridge = CvBridge()

        self.create_subscription(
            Image,
            "/traffic_light_crop",
            self.crop_callback,
            10
        )

        self.red_pub = self.create_publisher(
            Bool,
            "/traffic_light_red",
            10
        )

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info("Traffic Light Color Detector Initialized")

    def crop_callback(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        is_red = self.is_red_light(bgr)

        out_msg = Bool()
        out_msg.data = is_red
        self.red_pub.publish(out_msg)

        if is_red:
            self.get_logger().info("Red traffic light detected")

    def is_red_light(self, bgr: np.ndarray) -> bool:
        if bgr is None or bgr.size == 0:
            return False

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        #Red wraps around the HSV hue range, so use two masks.
        lower_red_1 = np.array([0, 80, 80])
        upper_red_1 = np.array([10, 255, 255])

        lower_red_2 = np.array([170, 80, 80])
        upper_red_2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)

        red_mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((3, 3), np.uint8)
        red_mask = cv2.erode(red_mask, kernel, iterations=1)
        red_mask = cv2.dilate(red_mask, kernel, iterations=2)

        red_pixels = cv2.countNonZero(red_mask)
        total_pixels = bgr.shape[0] * bgr.shape[1]
        red_ratio = red_pixels / float(total_pixels)

        contours, _ = cv2.findContours(
            red_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        max_area = 0
        if contours:
            max_area = max(cv2.contourArea(c) for c in contours)

        return (
            red_ratio >= self.red_pixel_ratio_threshold
            and max_area >= self.min_red_area
        )

    def parameters_callback(self, params):
        for param in params:
            if param.name == "red_pixel_ratio_threshold":
                self.red_pixel_ratio_threshold = param.value
                self.get_logger().info(
                    f"Updated red_pixel_ratio_threshold to {self.red_pixel_ratio_threshold}"
                )

            elif param.name == "min_red_area":
                self.min_red_area = param.value
                self.get_logger().info(
                    f"Updated min_red_area to {self.min_red_area}"
                )

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightColorDetector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

