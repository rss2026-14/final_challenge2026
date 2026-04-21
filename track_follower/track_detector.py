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
from track_segmentation import cd_color_segmentation

class TrackDetector(Node):
    """
    A class for applying your track detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_track_px (ConeLocationPixel) : the coordinates of the track in the image frame (units are pixels).
    """

    def __init__(self):
        super().__init__("track_detector")

        # Subscribe to ZED camera RGB frames
        self.track_pub = self.create_publisher(ConeLocationPixel, "/relative_track_px", 10)
        self.debug_pub = self.create_publisher(Image, "/track_debug_img", 10)
        self.image_sub = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 5)
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

        # Get bounding box from color segmentation
        # The function returns ((x1, y1), (x2, y2))
        bounding_box = cd_color_segmentation(image)

        # Create message to publish
        track_px_msg = ConeLocationPixel()

        if bounding_box is not None:
            # Extract bounding box coordinates from the tuple of tuples format
            (x_min, y_min), (x_max, y_max) = bounding_box

            # Calculate bottom center pixel
            # This point is on the ground plane where the track is
            u = (x_min + x_max) // 2  # center x coordinate
            v = y_max  # bottom y coordinate

            # Fill the message
            track_px_msg.u = float(u)
            track_px_msg.v = float(v)

            # Draw debug visualization
            # Draw bounding box
            cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            # Draw bottom center point
            cv2.circle(image, (u, v), 5, (0, 0, 255), -1)
            # Add text
            cv2.putText(image, f"({u}, {v})", (u + 10, v - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            self.get_logger().info(f"Detected track at pixel: ({u}, {v})")
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
