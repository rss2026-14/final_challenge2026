#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String, Bool
from rcl_interfaces.msg import SetParametersResult


class TrafficLightController(Node):
    """
    Stops the car while a red traffic light is detected.

    Subscribes:
        /traffic_light_red : Bool
        /mission_state     : String

    Publishes:
        drive_topic        : AckermannDriveStamped
    """

    def __init__(self):
        super().__init__("traffic_light_controller")

        self.declare_parameter("drive_topic", "/vesc/low_level/input/navigation")
        self.declare_parameter("red_timeout", 0.5)

        self.drive_topic = (
            self.get_parameter("drive_topic")
            .get_parameter_value()
            .string_value
        )

        self.red_timeout = (
            self.get_parameter("red_timeout")
            .get_parameter_value()
            .double_value
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.drive_topic,
            10
        )

        self.create_subscription(
            Bool,
            "/traffic_light_red",
            self.red_callback,
            10
        )

        self.current_state = "WAITING"
        self.create_subscription(
            String,
            "/mission_state",
            self.state_callback,
            10
        )

        self.last_red_time = None

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info("Traffic Light Controller Initialized")

    def state_callback(self, msg: String):
        self.current_state = msg.data

    def red_callback(self, msg: Bool):
        if msg.data:
            self.last_red_time = time.time()

    def timer_callback(self):
        red_is_recent = False

        if self.last_red_time is not None:
            elapsed = time.time() - self.last_red_time
            red_is_recent = elapsed <= self.red_timeout

        if not red_is_recent:
            return

        # Stop only during active driving states.
        if self.current_state not in ["NAVIGATING", "METER_SEARCH", "PARKING", "STOP_SIGN"]:
            return

        self.publish_stop_command()

    def publish_stop_command(self):
        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = self.get_clock().now().to_msg()
        drive_cmd.header.frame_id = "base_link"
        drive_cmd.drive.speed = 0.0
        drive_cmd.drive.steering_angle = 0.0

        self.drive_pub.publish(drive_cmd)

    def parameters_callback(self, params):
        for param in params:
            if param.name == "red_timeout":
                self.red_timeout = param.value
                self.get_logger().info(
                    f"Updated red_timeout to {self.red_timeout}"
                )

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
