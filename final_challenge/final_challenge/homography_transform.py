#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
from vs_msgs.msg import ConeLocation, ConeLocationPixel

# The following collection of pixel locations and corresponding relative
# ground plane locations are used to compute our homography matrix

# PTS_IMAGE_PLANE units are in pixels
# see README.md for coordinate frame description

######################################################
# DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
PTS_IMAGE_PLANE = [[315, 200],
                   [175,233],
                   [496,200],
                   [334,146]]  # dummy points
######################################################

# PTS_GROUND_PLANE units are in inches
# car looks along positive x axis with positive y axis to left

######################################################
# DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
PTS_GROUND_PLANE = [[35,5],
                    [19,15.5],
                    [38,-18],
                    [192,0]]  # dummy points
######################################################

METERS_PER_INCH = 0.0254


class HomographyTransformer(Node):
    def __init__(self):
        super().__init__("homography_transformer")

        self.track_pub = self.create_publisher(ConeLocation, "/relative_track", 10)
        self.marker_pub = self.create_publisher(Marker, "/track_marker", 1)
        self.track_px_sub = self.create_subscription(ConeLocationPixel, "/relative_track_px", self.track_detection_callback, 1)

        if not len(PTS_GROUND_PLANE) == len(PTS_IMAGE_PLANE):
            rclpy.logerr("ERROR: PTS_GROUND_PLANE and PTS_IMAGE_PLANE should be of same length")


        mouse_topic = (
            self.declare_parameter("mouse_click_topic", "/cone_debug_image_mouse_left")
            .get_parameter_value()
            .string_value
        )
        self.mouse_sub = self.create_subscription(
            PointStamped, "/cone_debug_image_mouse_left", self.mouse_callback, 1
        )
        self.get_logger().info(f"Subscribed to mouse clicks on: {mouse_topic}")

        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * METERS_PER_INCH
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

        self.get_logger().info("Homography Transformer Initialized")

    def mouse_callback(self, msg):
        self.get_logger().info("click registered")
        u = msg.point.x
        v = msg.point.y
        self.get_logger().info(f"Mouse click received: u={u:.0f}, v={v:.0f}")
        x, y = self.transformUvToXy(u, v)  # transformUvToXy also calls draw_marker

        # Also publish to /relative_track so clicking can be used to test the homography pipeline
        relative_xy_msg = ConeLocation()
        relative_xy_msg.x_pos = x
        relative_xy_msg.y_pos = y
        self.track_pub.publish(relative_xy_msg)

    def track_detection_callback(self, msg):
        # Extract information from message
        u = msg.u
        v = msg.v

        # Call to main function
        x, y = self.transformUvToXy(u, v)

        # Publish relative xy position of object in real world
        relative_xy_msg = ConeLocation()
        relative_xy_msg.x_pos = x
        relative_xy_msg.y_pos = y

        self.track_pub.publish(relative_xy_msg)
        self.draw_marker(x,y, "zed_camera_link")

    def transformUvToXy(self, u, v):
        """
        u and v are pixel coordinates.
        The top left pixel is the origin, u axis increases to right, and v axis
        increases down.

        Returns a normal non-np 1x2 matrix of xy displacement vector from the
        camera to the point on the ground plane.
        Camera points along positive x axis and y axis increases to the left of
        the camera.

        Units are in meters.
        """
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        self.get_logger().info(f"Mouse click received")
        self.draw_marker(x,y,"zed_camera_link")
        return x, y

    def draw_marker(self, track_x, track_y, message_frame):
        """
        Publish a marker to represent the track in rviz.
        (Call this function if you want)
        """
        marker = Marker()
        marker.header.frame_id = message_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = .5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = track_x
        marker.pose.position.y = track_y
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    homography_transformer = HomographyTransformer()
    rclpy.spin(homography_transformer)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
