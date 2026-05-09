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

        self.declare_parameter('odom_topic', "/vesc/odom")
        self.declare_parameter('drive_topic', "/vesc/input/navigation")
        self.declare_parameter('derivative_gain', 0.0)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.derivative_gain = self.get_parameter('derivative_gain').value

        self.wheelbase_length = 0.33 #in meters
        self.base_speed = 4.0

        self.target_point = None

        self.alpha = 0.0  # 0 = no smoothing, 1 = very smooth
        self.last_desired_steering_angle = None
        self.last_steering_angle = 0.0
        self.last_control_time = None
        self.max_derivative_term = 0.12
        self.max_steering_delta = 0.06

        self.pose_sub = self.create_subscription(Odometry, self.odom_topic, self.pose_callback, 10)
        self.point_track_sub = self.create_subscription(ConeLocation, "/relative_track", self.track_callback, 10)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)

        self.get_logger().info("Point Follower Initialized")

    def track_callback(self, msg):
        """
        Receives (x, y) in vehicle frame from homography.
        Applies smoothing to reduce jitter.
        """
        if msg.x_pos <= 0.05 or not np.isfinite(msg.x_pos) or not np.isfinite(msg.y_pos):
            return

        track_point = (msg.x_pos, msg.y_pos)

        if self.target_point is None:
            self.target_point = track_point
        #smoothing
        else:
            self.target_point = (
                self.alpha * self.target_point[0] + (1 - self.alpha) * track_point[0],
                self.alpha * self.target_point[1] + (1 - self.alpha) * track_point[1]
            )

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
            self._publish_drive_command(0.0, 0.0)
            return

        dist = np.sqrt(target_x**2 + target_y**2)

        if dist < 1e-6:
            self._publish_drive_command(0.0, 0.0)
            return

        # if target_x <= 0.05:
        #     self.get_logger().warn("Target too close or behind. Stopping.")
        #     self._publish_drive_command(0.0, 0.0)
        #     return

        min_dist = 0.3

        if dist < min_dist:
            scale = min_dist / dist
            target_x *= scale
            target_y *= scale

        # if target_x < 0.3:
        #     target_x = 0.3

        actual_lookahead_sq = target_x**2 + target_y**2
        steering_angle = np.arctan2(2*self.wheelbase_length*target_y, actual_lookahead_sq)

        if abs(steering_angle) > 0.15:
            steering_angle *= 0.2
        steering_angle=np.clip(steering_angle,-0.34,0.34)

        now = self.get_clock().now().nanoseconds * 1e-9
        derivative_term = 0.0
        if self.last_desired_steering_angle is not None and self.last_control_time is not None:
            dt = max(now - self.last_control_time, 1e-3)
            steering_rate = (steering_angle - self.last_desired_steering_angle) / dt
            derivative_term = -self.derivative_gain * steering_rate
            derivative_term = np.clip(
                derivative_term,
                -self.max_derivative_term,
                self.max_derivative_term
            )

        self.last_desired_steering_angle = steering_angle
        self.last_control_time = now
        steering_angle = np.clip(steering_angle + derivative_term, -0.34, 0.34)
        steering_angle = float(np.clip(
            steering_angle,
            self.last_steering_angle - self.max_steering_delta,
            self.last_steering_angle + self.max_steering_delta
        ))
        self.last_steering_angle = steering_angle
        speed=max(1.0, self.base_speed-6.0*abs(steering_angle))

        self._publish_drive_command(speed, steering_angle)

    def _publish_drive_command(self, speed, steering_angle):
        """
        Publish Ackermann command.
        """

        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = self.get_clock().now().to_msg()
        drive_cmd.header.frame_id = 'base_link'
        drive_cmd.drive.speed = float(speed)
        drive_cmd.drive.steering_angle = float(steering_angle)

        self.drive_pub.publish(drive_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
