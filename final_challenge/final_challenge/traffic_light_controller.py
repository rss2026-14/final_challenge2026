#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import String, Bool
from rcl_interfaces.msg import SetParametersResult


class TrafficLightController(Node):
    """
    Publishes obstacle alert while a red traffic light is detected.

    Subscribes:
        /traffic_light_red : Bool
        /mission_state     : String

    Publishes:
        /safety/obstacle_alert : Bool
    """

    def __init__(self):
        super().__init__("traffic_light_controller")

        self.declare_parameter("red_timeout", 0.5)

        self.red_timeout = (
            self.get_parameter("red_timeout")
            .get_parameter_value()
            .double_value
        )

        self.obstacle_pub = self.create_publisher(
            Bool,
            "/traffic_light_obstacle_alert",
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

        active_state = self.current_state in [
            "NAVIGATING",
            "METER_SEARCH",
            "PARKING",
            "STOP_SIGN",
        ]

        alert_msg = Bool()
        alert_msg.data = red_is_recent and active_state
        self.obstacle_pub.publish(alert_msg)

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
