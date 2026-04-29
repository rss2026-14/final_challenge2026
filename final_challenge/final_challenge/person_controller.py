#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from rcl_interfaces.msg import SetParametersResult
from vs_msgs.msg import ConeLocation
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String, Bool


class PersonController(Node):
    """
    Controller that stops the car while a person is detected within a stop distance.

    Subscribes:
        /relative_person : ConeLocation
        /mission_state   : String

    Publishes:
        drive_topic      : AckermannDriveStamped
        /person_detected : Bool
    """

    def __init__(self):
        super().__init__("person_controller")
        self.declare_parameter("drive_topic", "/vesc/low_level/input/navigation")
        self.declare_parameter("person_timeout", 0.5)
        self.declare_parameter("person_stop_distance", 2.0)
        self.drive_topic = (
            self.get_parameter("drive_topic")
            .get_parameter_value()
            .string_value
        )
        self.person_timeout = (
            self.get_parameter("person_timeout")
            .get_parameter_value()
            .double_value
        )
        self.person_stop_distance = (
            self.get_parameter("person_stop_distance")
            .get_parameter_value()
            .double_value
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.drive_topic,
            10
        )
        self.person_detected_pub = self.create_publisher(
            Bool,
            "/person_detected",
            10
        )

        self.create_subscription(
            ConeLocation,
            "/relative_person",
            self.person_callback,
            1
        )

        self.current_state = "WAITING"
        self.create_subscription(
            String,
            "/mission_state",
            self.state_callback,
            10
        )

        self.last_person_time = None
        self.last_person_distance = None

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info("Person Controller Initialized")

    def state_callback(self, msg: String):
        self.current_state = msg.data

    def person_callback(self, msg: ConeLocation):
        """
        Called whenever homography publishes a person location.
        Receiving the message means a person was detected.
        """

        self.last_person_time = time.time()
        relative_x = msg.x_pos
        relative_y = msg.y_pos
        self.last_person_distance = (relative_x**2 + relative_y**2) ** 0.5

    def timer_callback(self):
        """
        Stop as long as a person has been detected recently and is close enough.
        """

        person_is_recent = False

        if self.last_person_time is not None:
            elapsed = time.time() - self.last_person_time
            person_is_recent = elapsed <= self.person_timeout

        person_is_close = (
            self.last_person_distance is not None
            and self.last_person_distance <= self.person_stop_distance
        )

        detected_msg = Bool()
        detected_msg.data = person_is_recent and person_is_close
        self.person_detected_pub.publish(detected_msg)

        if not person_is_recent:
            return

        if not person_is_close:
            return

        # Only stop the car during active mission states.
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
            if param.name == "person_timeout":
                self.person_timeout = param.value
                self.get_logger().info(
                    f"Updated person_timeout to {self.person_timeout}"
                )

            elif param.name == "person_stop_distance":
                self.person_stop_distance = param.value
                self.get_logger().info(
                    f"Updated person_stop_distance to {self.person_stop_distance}"
                )

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = PersonController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
