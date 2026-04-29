#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from vs_msgs.msg import ConeLocation


class PurePursuit(Node):
    """
    Pure Pursuit controller that follows a single moving target point
    coming from the homography transformer (/relative_track).

    Assumes:
        x = forward (meters)
        y = left (meters)
    """

    def __init__(self):
        super().__init__("pure_pursuit_point_follower")

        self.declare_parameter('odom_topic', "/odom")
        self.declare_parameter('drive_topic', "/drive")

        self.odom_topic = self.get_parameter('odom_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value

        self.wheelbase_length = 0.25 #in meters

        self.base_speed = 2.0  #m/s
        self.min_speed = 1.0

        self.target_point = None

        self.alpha = 0.7  # 0 = no smoothing, 1 = very smooth

        self.pose_sub = self.create_subscription(Odometry, self.odom_topic, self.pose_callback, 1)
        self.point_track_sub = self.create_subscription(ConeLocation, "/relative_track", self.track_callback, 1)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 1)

        self.get_logger().info("Point Follower Initialized")

    def track_callback(self, msg):
        """
        Receives (x, y) in vehicle frame from homography.
        Applies smoothing to reduce jitter.
        """
        track_point = (msg.x_pos, msg.y_pos)

        if self.target_point is None:
            self.target_point = track_point
        else:
            self.target_point = (
                self.alpha * self.target_point[0] + (1 - self.alpha) * track_point[0],
                self.alpha * self.target_point[1] + (1 - self.alpha) * track_point[1]
            ) #???

    def pose_callback(self, odometry_msg):
        """
        Main control loop.
        Computes steering using Pure Pursuit.
        """
        if self.target_point is None:
            self._publish_drive_command(0.0, 0.0)
            return

        target_x, target_y = self.target_point

        if target_x <= 0.05:
            self.get_logger().warn("Target too close or behind. Stopping.")
            self._publish_drive_command(0.0, 0.0)
            return

        if target_x < 0.3:
            target_x = 0.3

        L2 = target_x**2 + target_y**2 #squaring distance?

        if L2 < 1e-6:
            self._publish_drive_command(0.0, 0.0)
            return

        steering_angle = np.arctan2(2.0 * self.wheelbase_length * target_y, L2)
        steering_angle = np.clip(steering_angle, -0.34, 0.34)

        self._publish_drive_command(steering_angle)

    def _publish_drive_command(self, speed, steering_angle):
        """
        Publish Ackermann command.
        """
        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = self.get_clock().now().to_msg()
        drive_cmd.header.frame_id = 'base_link'

        drive_cmd.drive.speed = float(speed)
        drive_cmd.drive.steering_angle = float(steering_angle)

        self.drive_pub.publish(self.base_speed, drive_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
