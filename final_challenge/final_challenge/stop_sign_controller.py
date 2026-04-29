#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import time

from rcl_interfaces.msg import SetParametersResult
from vs_msgs.msg import ConeLocation
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String, Bool


class StopSignController(Node):
    """
    Controller for stopping at a detected stop sign.

    Subscribes:
        /relative_stop_sign : ConeLocation
        /mission_state      : String

    Publishes:
        drive_topic         : AckermannDriveStamped
        /stop_sign_success  : Bool
    """

    def __init__(self):
        super().__init__("stop_sign_controller")

        self.declare_parameter("drive_topic", "/vesc/low_level/input/navigation")
        self.declare_parameter("stop_distance", 1.0)
        self.declare_parameter("stop_time", 3.0)

        self.drive_topic = (
            self.get_parameter("drive_topic")
            .get_parameter_value()
            .string_value
        )

        self.stop_distance = (
            self.get_parameter("stop_distance")
            .get_parameter_value()
            .double_value
        )

        self.stop_time = (
            self.get_parameter("stop_time")
            .get_parameter_value()
            .double_value
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.drive_topic,
            10
        )

        self.success_pub = self.create_publisher(
            Bool,
            "/stop_sign_success",
            10
        )

        self.create_subscription(
            ConeLocation,
            "/relative_stop_sign",
            self.relative_callback,
            1
        )

        self.current_state = "WAITING"
        self.create_subscription(
            String,
            "/mission_state",
            self.state_callback,
            10
        )

        self.is_stopping = False
        self.stop_start_time = None
        self.has_completed_stop = False

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info("Stop Sign Controller Initialized")

    def state_callback(self, msg: String):
        self.current_state = msg.data
        #Reset once we leave the stop sign state, so the controller can be reused later.
        if self.current_state != "STOP_SIGN":
            self.is_stopping = False
            self.stop_start_time = None
            self.has_completed_stop = False

    def relative_callback(self, msg: ConeLocation):
        """
        Called whenever homography publishes the relative stop sign location.
        """
        #Only act when the executive/state machine tells this node to handle a stop sign.
        if self.current_state != "STOP_SIGN":
            return

        if self.has_completed_stop:
            return

        relative_x = msg.x_pos
        relative_y = msg.y_pos

        distance = np.sqrt(relative_x**2 + relative_y**2)

        if distance <= self.stop_distance and not self.is_stopping:
            self.is_stopping = True
            self.stop_start_time = time.time()
            self.get_logger().info(
                f"Stop sign within {distance:.2f} m. Starting stop for {self.stop_time:.2f} s."
            )

    def timer_callback(self):
        """
        Handles holding the stop command for stop_time seconds.
        """

        if not self.is_stopping:
            return

        self.publish_stop_command()

        elapsed = time.time() - self.stop_start_time

        if elapsed >= self.stop_time:
            self.is_stopping = False
            self.has_completed_stop = True

            success_msg = Bool()
            success_msg.data = True
            self.success_pub.publish(success_msg)

            self.get_logger().info("Stop sign stop complete.")

    def publish_stop_command(self):
        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = self.get_clock().now().to_msg()
        drive_cmd.header.frame_id = "base_link"
        drive_cmd.drive.speed = 0.0
        drive_cmd.drive.steering_angle = 0.0

        self.drive_pub.publish(drive_cmd)

    def parameters_callback(self, params):
        for param in params:
            if param.name == "stop_distance":
                self.stop_distance = param.value
                self.get_logger().info(
                    f"Updated stop_distance to {self.stop_distance}"
                )
            elif param.name == "stop_time":
                self.stop_time = param.value
                self.get_logger().info(
                    f"Updated stop_time to {self.stop_time}"
                )
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = StopSignController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

