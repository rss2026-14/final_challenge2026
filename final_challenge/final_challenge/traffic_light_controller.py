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

        self.declare_parameter("red_timeout", 1.0)
        self.declare_parameter("min_red_detections", 1)
        self.declare_parameter("log_debug", False)

        self.red_timeout = (
            self.get_parameter("red_timeout")
            .get_parameter_value()
            .double_value
        )
        self.min_red_detections = (
            self.get_parameter("min_red_detections")
            .get_parameter_value()
            .integer_value
        )
        self.log_debug = (
            self.get_parameter("log_debug")
            .get_parameter_value()
            .bool_value
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
        self.red_detection_count = 0

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info("Traffic Light Controller Initialized")

    def state_callback(self, msg: String):
        self.current_state = msg.data

    def red_callback(self, msg: Bool):
        if self.log_debug:
            self.get_logger().info(f"Traffic light red message received: red={msg.data}")

        if msg.data:
            self.red_detection_count += 1

            if self.red_detection_count >= self.min_red_detections:
                self.last_red_time = time.time()

            if self.log_debug:
                self.get_logger().info(
                    f"Red detection count={self.red_detection_count}, "
                    f"required={self.min_red_detections}"
                )
        else:
            self.red_detection_count = 0

    def timer_callback(self):
        red_is_recent = False

        if self.last_red_time is not None:
            elapsed = time.time() - self.last_red_time
            red_is_recent = elapsed <= self.red_timeout

        active_state = self.current_state in [
            "NAVIGATING",
            "METER_SEARCH",
            "PARKING",
            "OBSTACLE_PAUSE",
        ]

        alert_msg = Bool()
        alert_msg.data = red_is_recent and active_state
        if self.log_debug:
            self.get_logger().info(
                f"Traffic light controller debug: "
                f"state={self.current_state}, "
                f"red_recent={red_is_recent}, "
                f"active_state={active_state}, "
                f"alert={alert_msg.data}"
            )
        self.obstacle_pub.publish(alert_msg)

    def parameters_callback(self, params):
        for param in params:
            if param.name == "red_timeout":
                self.red_timeout = param.value
                self.get_logger().info(
                    f"Updated red_timeout to {self.red_timeout}"
                )
            elif param.name == "min_red_detections":
                self.min_red_detections = param.value
                self.get_logger().info(
                    f"Updated min_red_detections to {self.min_red_detections}"
                )
            elif param.name == "log_debug":
                self.log_debug = param.value
                self.get_logger().info(
                    f"Updated log_debug to {self.log_debug}"
                )

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
