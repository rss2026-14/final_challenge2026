#!/usr/bin/env python3

from enum import Enum

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String


class ReversePhase(Enum):
    BACK_UP = 1
    K_TURN_FORWARD = 2
    K_TURN_REVERSE = 3
    COMPLETE = 4


class ReverseController(Node):
    """
    Owns the low-level recovery maneuver after parking.

    The executive switches into REVERSE, this node backs out and performs
    K-turn cycles, then tells the executive it can publish the next goal.
    """

    def __init__(self):
        super().__init__("reverse_controller")

        self.declare_parameter("drive_topic", "/vesc/high_level/input/nav_0")
        self.declare_parameter("back_up_speed", -1.0)
        self.declare_parameter("k_turn_speed", 0.8)
        self.declare_parameter("max_steering_angle", 0.34)
        self.declare_parameter("back_up_duration", 2.0)
        self.declare_parameter("k_turn_forward_duration", 1.6)
        self.declare_parameter("k_turn_reverse_duration", 1.6)
        self.declare_parameter("k_turn_cycles", 2)

        self.drive_topic = self.get_parameter("drive_topic").value
        self.back_up_speed = float(self.get_parameter("back_up_speed").value)
        self.k_turn_speed = float(self.get_parameter("k_turn_speed").value)
        self.max_steering_angle = float(
            self.get_parameter("max_steering_angle").value
        )
        self.back_up_duration = float(
            self.get_parameter("back_up_duration").value
        )
        self.k_turn_forward_duration = float(
            self.get_parameter("k_turn_forward_duration").value
        )
        self.k_turn_reverse_duration = float(
            self.get_parameter("k_turn_reverse_duration").value
        )
        self.k_turn_cycles = int(self.get_parameter("k_turn_cycles").value)

        self.current_state = "WAITING"
        self.phase = ReversePhase.COMPLETE
        self.phase_start_time = self.now_sec()
        self.completed_cycles = 0
        self.completion_sent = False

        self.create_subscription(
            String,
            "/mission_state",
            self.state_callback,
            10,
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.drive_topic,
            10,
        )
        self.complete_pub = self.create_publisher(
            Bool,
            "/reverse_complete",
            10,
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Reverse Controller Initialized")

    def state_callback(self, msg):
        previous_state = self.current_state
        self.current_state = msg.data

        if previous_state != "REVERSE" and self.current_state == "REVERSE":
            self.reset_maneuver()
            self.get_logger().info("Reverse maneuver active.")

        if previous_state == "REVERSE" and self.current_state != "REVERSE":
            self.publish_drive_command(0.0, 0.0)

    def control_loop(self):
        if self.current_state != "REVERSE":
            return

        if self.phase == ReversePhase.BACK_UP:
            self.back_up()
        elif self.phase == ReversePhase.K_TURN_FORWARD:
            self.k_turn_forward()
        elif self.phase == ReversePhase.K_TURN_REVERSE:
            self.k_turn_reverse()
        elif self.phase == ReversePhase.COMPLETE:
            self.publish_drive_command(0.0, 0.0)
            self.publish_complete_once()

    def back_up(self):
        if self.elapsed_phase_time() >= self.back_up_duration:
            self.set_phase(ReversePhase.K_TURN_FORWARD)
            return

        self.publish_drive_command(self.back_up_speed, 0.0)

    def k_turn_forward(self):
        if self.elapsed_phase_time() >= self.k_turn_forward_duration:
            self.set_phase(ReversePhase.K_TURN_REVERSE)
            return

        self.publish_drive_command(
            self.k_turn_speed,
            self.max_steering_angle,
        )

    def k_turn_reverse(self):
        if self.elapsed_phase_time() >= self.k_turn_reverse_duration:
            self.completed_cycles += 1

            if self.completed_cycles >= self.k_turn_cycles:
                self.set_phase(ReversePhase.COMPLETE)
            else:
                self.set_phase(ReversePhase.K_TURN_FORWARD)

            return

        self.publish_drive_command(
            -self.k_turn_speed,
            -self.max_steering_angle,
        )

    def reset_maneuver(self):
        self.phase = ReversePhase.BACK_UP
        self.phase_start_time = self.now_sec()
        self.completed_cycles = 0
        self.completion_sent = False

    def set_phase(self, phase):
        self.phase = phase
        self.phase_start_time = self.now_sec()

        self.get_logger().info(f"Reverse phase -> {phase.name}")

    def publish_complete_once(self):
        if self.completion_sent:
            return

        msg = Bool()
        msg.data = True
        self.complete_pub.publish(msg)

        self.completion_sent = True
        self.get_logger().info("Reverse maneuver complete.")

    def publish_drive_command(self, speed, steering_angle):
        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = self.get_clock().now().to_msg()
        drive_cmd.header.frame_id = "base_link"
        drive_cmd.drive.speed = float(speed)
        drive_cmd.drive.steering_angle = float(steering_angle)

        self.drive_pub.publish(drive_cmd)

    def elapsed_phase_time(self):
        return self.now_sec() - self.phase_start_time

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    controller = ReverseController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
