#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

import cv2

from cv_bridge import CvBridge, CvBridgeError

from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from vs_msgs.msg import ConeLocationPixel
from final_challenge.track_segmentation import cd_color_segmentation

class TrackDetector(Node):
    """
    A class for applying your track detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_track_px (ConeLocationPixel) : the coordinates of the track in the image frame (units are pixels).
    """

    def __init__(self):
        super().__init__("track_detector")

        self.declare_parameter("target_smoothing_alpha", 0.9)
        self.declare_parameter("max_target_jump_ratio", 0.12)
        self.declare_parameter("max_center_offset_ratio", 0.35)
        self.target_smoothing_alpha = (
            self.get_parameter("target_smoothing_alpha")
            .get_parameter_value()
            .double_value
        )
        self.max_target_jump_ratio = (
            self.get_parameter("max_target_jump_ratio")
            .get_parameter_value()
            .double_value
        )
        self.max_center_offset_ratio = (
            self.get_parameter("max_center_offset_ratio")
            .get_parameter_value()
            .double_value
        )
        self.filtered_target = None

        # Subscribe to ZED camera RGB frames
        self.track_pub = self.create_publisher(ConeLocationPixel, "/relative_track_px", 10)
        self.debug_pub = self.create_publisher(Image, "/track_debug_img", 10)
        self.image_sub = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 10)
        self.bridge = CvBridge()  # Converts between ROS images and OpenCV Images

        self.get_logger().info("Track Detector Initialized")

    def image_callback(self, image_msg):
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_track_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        # YOUR CODE HERE
        # detect the track and publish its
        # pixel location in the image.
        # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        #################################
        # Convert ROS image message to OpenCV image
        try:
            image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Get track from color segmentation
        # The function returns (px,py)
        drive_point = cd_color_segmentation(image)

        # Create message to publish
        track_px_msg = ConeLocationPixel()
        h, w = image.shape[:2]
        if drive_point is not None:
            x, y, lines = drive_point

            measured_target = np.array([float(x), float(y)])
            max_jump_px = self.max_target_jump_ratio * w
            max_center_offset_px = self.max_center_offset_ratio * w
            center_offset = abs(measured_target[0] - (w / 2.0))

            if center_offset > max_center_offset_px:
                self.get_logger().warn(
                    f"Ignoring lane target {center_offset:.1f}px from image center "
                    f"(limit {max_center_offset_px:.1f}px)"
                )
                drive_point = None

            if drive_point is not None and self.filtered_target is None:
                self.filtered_target = measured_target
            elif drive_point is not None:
                jump = np.linalg.norm(measured_target - self.filtered_target)
                if jump <= max_jump_px:
                    self.filtered_target = (
                        self.target_smoothing_alpha * self.filtered_target
                        + (1.0 - self.target_smoothing_alpha) * measured_target
                    )
                else:
                    self.get_logger().warn(
                        f"Ignoring lane target jump of {jump:.1f}px "
                        f"(limit {max_jump_px:.1f}px)"
                    )

            if self.filtered_target is None:
                track_px_msg.u = -1.0
                track_px_msg.v = -1.0
                self.get_logger().info("No stable track target yet")
                self.track_pub.publish(track_px_msg)
                return

            x = int(self.filtered_target[0])
            y = int(self.filtered_target[1])

            track_px_msg.u = float(x)
            track_px_msg.v = float(y)

            cv2.circle(image, (x, y), 5, (0, 0, 255), -1)

            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(image, (x1, y1), (x2, y2), (255, 0, 0), 3)

            self.get_logger().info(f"Detected lane-center target at pixel: ({x}, {y})")
        else:
            # No track detected, publish sentinel values
            track_px_msg.u = -1.0
            track_px_msg.v = -1.0

            self.get_logger().info("No track detected")

        # Publish track pixel location
        self.track_pub.publish(track_px_msg)

        # Publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.debug_pub.publish(debug_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert debug image: {e}")

def main(args=None):
    rclpy.init(args=args)
    track_detector = TrackDetector()
    rclpy.spin(track_detector)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
