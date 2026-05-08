#!/usr/bin/env python3

import math
from enum import Enum

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class SearchPhase(Enum):
    APPROACH_WALL = 1
    PAUSE_BEFORE_REVERSE = 2
    BACK_UP = 3
    K_TURN_FORWARD = 4
    K_TURN_REVERSE = 5
    FAILED = 6


class MeterSearchController(Node):
    """
    Owns low-level motion during METER_SEARCH.

    Strategy:
    1. Use lidar to find the closest wall in the forward half-plane.
    2. Drive toward that wall by steering toward the closest lidar bearing.
    3. Once 0.5 m away, stop briefly before reversing.
    4. Back up until 1.0 m away.
    5. If still no parking-meter detection/state transition happened, perform
       repeated K-turn motions and eventually report search failure.
    """

    def __init__(self):
        super().__init__("meter_search_controller")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/vesc/input/navigation")

        self.declare_parameter("near_wall_distance", 0.5)
        self.declare_parameter("far_wall_distance", 1.0)

        self.declare_parameter("approach_speed", 0.9)
        self.declare_parameter("back_up_speed", -0.9)
        self.declare_parameter("k_turn_speed", 0.9)

        self.declare_parameter("max_steering_angle", 0.34)
        self.declare_parameter("wall_angle_gain", 0.8)

        self.declare_parameter("min_valid_range", 0.08)
        self.declare_parameter("max_valid_range", 8.0)

        self.declare_parameter("forward_search_angle", math.pi / 4.0)

        self.declare_parameter("k_turn_forward_duration", 1.0)
        self.declare_parameter("k_turn_reverse_duration", 1.0)

        self.declare_parameter("reverse_pause_duration", 1.0)

        self.declare_parameter("max_k_turn_cycles", 3)

        self.scan_topic = self.get_parameter("scan_topic").value
        self.drive_topic = self.get_parameter("drive_topic").value

        self.near_wall_distance = float(
            self.get_parameter("near_wall_distance").value
        )
        self.far_wall_distance = float(
            self.get_parameter("far_wall_distance").value
        )

        self.approach_speed = float(
            self.get_parameter("approach_speed").value
        )

        self.back_up_speed = float(
            self.get_parameter("back_up_speed").value
        )

        self.k_turn_speed = float(
            self.get_parameter("k_turn_speed").value
        )

        self.max_steering_angle = float(
            self.get_parameter("max_steering_angle").value
        )

        self.wall_angle_gain = float(
            self.get_parameter("wall_angle_gain").value
        )

        self.min_valid_range = float(
            self.get_parameter("min_valid_range").value
        )

        self.max_valid_range = float(
            self.get_parameter("max_valid_range").value
        )

        self.forward_search_angle = float(
            self.get_parameter("forward_search_angle").value
        )

        self.k_turn_forward_duration = float(
            self.get_parameter("k_turn_forward_duration").value
        )

        self.k_turn_reverse_duration = float(
            self.get_parameter("k_turn_reverse_duration").value
        )

        self.reverse_pause_duration = float(
            self.get_parameter("reverse_pause_duration").value
        )

        self.max_k_turn_cycles = int(
            self.get_parameter("max_k_turn_cycles").value
        )

        self.current_state = "WAITING"

        self.phase = SearchPhase.APPROACH_WALL

        self.closest_wall_distance = None
        self.closest_wall_angle = 0.0

        self.phase_start_time = self.now_sec()

        self.k_turn_cycles = 0
        self.failure_sent = False

        self.create_subscription(
            String,
            "/mission_state",
            self.state_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            5
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.drive_topic,
            10
        )

        self.failed_pub = self.create_publisher(
            Bool,
            "/meter_search_failed",
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            "Meter Search Controller Initialized"
        )

    def state_callback(self, msg):
        previous_state = self.current_state
        self.current_state = msg.data

        if (
            previous_state != "METER_SEARCH"
            and self.current_state == "METER_SEARCH"
        ):
            self.reset_search()

            self.get_logger().info(
                "Meter search active."
            )

        if (
            previous_state == "METER_SEARCH"
            and self.current_state != "METER_SEARCH"
        ):
            self.publish_drive_command(0.0, 0.0)

    def scan_callback(self, scan):
        ranges = np.asarray(scan.ranges, dtype=np.float32)

        if ranges.size == 0:
            return

        angles = (
            scan.angle_min
            + scan.angle_increment * np.arange(ranges.size)
        )

        valid = (
            np.isfinite(ranges)
            & (ranges >= self.min_valid_range)
            & (ranges <= self.max_valid_range)
            & (np.abs(angles) <= self.forward_search_angle)
        )

        if not np.any(valid):
            self.closest_wall_distance = None
            return

        valid_ranges = ranges[valid]
        valid_angles = angles[valid]

        closest_idx = int(np.argmin(valid_ranges))

        self.closest_wall_distance = float(
            valid_ranges[closest_idx]
        )

        self.closest_wall_angle = float(
            valid_angles[closest_idx]
        )

    def control_loop(self):
        if self.current_state != "METER_SEARCH":
            return

        if self.phase == SearchPhase.APPROACH_WALL:
            self.approach_wall()

        elif self.phase == SearchPhase.PAUSE_BEFORE_REVERSE:
            self.pause_before_reverse()

        elif self.phase == SearchPhase.BACK_UP:
            self.back_up_from_wall()

        elif self.phase == SearchPhase.K_TURN_FORWARD:
            self.k_turn_forward()

        elif self.phase == SearchPhase.K_TURN_REVERSE:
            self.k_turn_reverse()

        elif self.phase == SearchPhase.FAILED:
            self.publish_drive_command(0.0, 0.0)
            self.publish_failure_once()

    def approach_wall(self):
        if self.closest_wall_distance is None:
            self.publish_drive_command(0.0, 0.0)

            self.get_logger().warn(
                "Meter search waiting for valid lidar wall distance."
            )

            return

        if self.closest_wall_distance <= self.near_wall_distance:
            self.set_phase(SearchPhase.PAUSE_BEFORE_REVERSE)
            return

        steering = np.clip(
            self.wall_angle_gain * self.closest_wall_angle,
            -self.max_steering_angle,
            self.max_steering_angle,
        )

        self.publish_drive_command(
            self.approach_speed,
            steering
        )

    def pause_before_reverse(self):
        self.publish_drive_command(0.0, 0.0)

        elapsed = self.now_sec() - self.phase_start_time

        if elapsed >= self.reverse_pause_duration:
            self.set_phase(SearchPhase.BACK_UP)

    def back_up_from_wall(self):
        if self.closest_wall_distance is None:
            self.publish_drive_command(0.0, 0.0)

            self.get_logger().warn(
                "Lost wall during reverse."
            )

            return

        if self.closest_wall_distance >= self.far_wall_distance:
            self.set_phase(SearchPhase.K_TURN_FORWARD)
            return

        # IMPORTANT:
        # Reverse steering should usually keep the same sign.
        # The vehicle dynamics naturally invert motion.

        steering = np.clip(
            self.wall_angle_gain * self.closest_wall_angle,
            -self.max_steering_angle,
            self.max_steering_angle,
        )

        self.get_logger().info(
            f"BACKING UP | "
            f"dist={self.closest_wall_distance:.2f} | "
            f"angle={self.closest_wall_angle:.2f} | "
            f"steering={steering:.2f}"
        )

        self.publish_drive_command(
            self.back_up_speed,
            steering
        )

    def k_turn_forward(self):
        elapsed = self.now_sec() - self.phase_start_time

        if elapsed >= self.k_turn_forward_duration:
            self.set_phase(SearchPhase.K_TURN_REVERSE)
            return

        self.publish_drive_command(
            self.k_turn_speed,
            self.max_steering_angle
        )

    def k_turn_reverse(self):
        elapsed = self.now_sec() - self.phase_start_time

        if elapsed >= self.k_turn_reverse_duration:
            self.k_turn_cycles += 1

            if self.k_turn_cycles >= self.max_k_turn_cycles:
                self.set_phase(SearchPhase.FAILED)
            else:
                self.set_phase(SearchPhase.K_TURN_FORWARD)

            return

        self.publish_drive_command(
            -self.k_turn_speed,
            -self.max_steering_angle
        )

    def reset_search(self):
        self.phase = SearchPhase.APPROACH_WALL

        self.phase_start_time = self.now_sec()

        self.k_turn_cycles = 0

        self.failure_sent = False

        self.closest_wall_distance = None
        self.closest_wall_angle = 0.0

    def set_phase(self, phase):
        self.phase = phase

        self.phase_start_time = self.now_sec()

        self.get_logger().info(
            f"Meter search phase -> {phase.name}"
        )

    def publish_failure_once(self):
        if self.failure_sent:
            return

        msg = Bool()
        msg.data = True

        self.failed_pub.publish(msg)

        self.failure_sent = True

        self.get_logger().warn(
            "Meter search failed after one search rotation."
        )

    def publish_drive_command(self, speed, steering_angle):
        drive_cmd = AckermannDriveStamped()

        drive_cmd.header.stamp = (
            self.get_clock().now().to_msg()
        )

        drive_cmd.header.frame_id = "base_link"

        drive_cmd.drive.speed = float(speed)
        drive_cmd.drive.steering_angle = float(steering_angle)

        self.drive_pub.publish(drive_cmd)

    def now_sec(self):
        return (
            self.get_clock().now().nanoseconds * 1e-9
        )


def main(args=None):
    rclpy.init(args=args)

    node = MeterSearchController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

# import math
# from enum import Enum

# import numpy as np
# import rclpy
# from ackermann_msgs.msg import AckermannDriveStamped
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# from std_msgs.msg import Bool, String


# class SearchPhase(Enum):
#     SWEEP_FORWARD = 1
#     SWEEP_REVERSE = 2
#     FAILED = 3


# class MeterSearchController(Node):
#     """
#     Owns low-level motion during METER_SEARCH.

#     Strategy (Tight Space Scanner):
#     Because Ackermann vehicles cannot spin in place, this node executes a
#     "Star Pattern". It alternates between driving forward-left and reverse-right.
#     This keeps the vehicle centrally located while rotating its yaw to scan
#     the environment for the parking meter.

#     The LiDAR is used purely as a safety bubble to prevent collisions while sweeping.
#     """

#     def __init__(self):
#         super().__init__("meter_search_controller")

#         self.declare_parameter("scan_topic", "/scan")
#         self.declare_parameter("drive_topic", "/vesc/input/navigation")

#         self.declare_parameter("sweep_speed", 0.8)
#         self.declare_parameter("max_steering_angle", 0.34)

#         self.declare_parameter("sweep_duration", 2.0) # Seconds to spend in each direction
#         self.declare_parameter("max_sweep_cycles", 4) # How many times to go back and forth before giving up

#         self.declare_parameter("safety_bubble_radius", 0.4) # Meters. If anything gets closer than this, swap directions

#         self.scan_topic = self.get_parameter("scan_topic").value
#         self.drive_topic = self.get_parameter("drive_topic").value

#         self.sweep_speed = float(self.get_parameter("sweep_speed").value)
#         self.max_steering_angle = float(self.get_parameter("max_steering_angle").value)
#         self.sweep_duration = float(self.get_parameter("sweep_duration").value)
#         self.max_sweep_cycles = int(self.get_parameter("max_sweep_cycles").value)
#         self.safety_bubble_radius = float(self.get_parameter("safety_bubble_radius").value)

#         self.current_state = "WAITING"
#         self.phase = SearchPhase.SWEEP_FORWARD
#         self.phase_start_time = self.now_sec()

#         self.sweep_cycles = 0
#         self.failure_sent = False

#         # Track front and rear distances for safety checking
#         self.min_distance_front = float('inf')
#         self.min_distance_rear = float('inf')

#         self.create_subscription(String, "/mission_state", self.state_callback, 10)
#         self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 5)

#         self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
#         self.failed_pub = self.create_publisher(Bool, "/meter_search_failed", 10)

#         self.timer = self.create_timer(0.1, self.control_loop)

#         self.get_logger().info("Tight Sweep Meter Search Controller Initialized")

#     def state_callback(self, msg):
#         previous_state = self.current_state
#         self.current_state = msg.data

#         if previous_state != "METER_SEARCH" and self.current_state == "METER_SEARCH":
#             self.reset_search()
#             self.get_logger().info("Meter search active. Initiating star pattern sweep.")

#         if previous_state == "METER_SEARCH" and self.current_state != "METER_SEARCH":
#             self.publish_drive_command(0.0, 0.0)

#     def scan_callback(self, scan):
#         # We only care about collisions, so we check raw distances
#         ranges = np.asarray(scan.ranges, dtype=np.float32)

#         if ranges.size == 0:
#             return

#         angles = scan.angle_min + scan.angle_increment * np.arange(ranges.size)
#         valid = np.isfinite(ranges) & (ranges > 0.05) # Ignore immediate self-hits

#         if not np.any(valid):
#             return

#         valid_ranges = ranges[valid]
#         valid_angles = angles[valid]

#         # Check front half-plane (between -90 and 90 degrees)
#         front_mask = np.abs(valid_angles) <= (math.pi / 2.0)
#         if np.any(front_mask):
#             self.min_distance_front = float(np.min(valid_ranges[front_mask]))
#         else:
#             self.min_distance_front = float('inf')

#         # Check rear half-plane (greater than 90 or less than -90)
#         rear_mask = ~front_mask
#         if np.any(rear_mask):
#             self.min_distance_rear = float(np.min(valid_ranges[rear_mask]))
#         else:
#             self.min_distance_rear = float('inf')

#     def control_loop(self):
#         if self.current_state != "METER_SEARCH":
#             return

#         if self.phase == SearchPhase.SWEEP_FORWARD:
#             self.sweep_forward()

#         elif self.phase == SearchPhase.SWEEP_REVERSE:
#             self.sweep_reverse()

#         elif self.phase == SearchPhase.FAILED:
#             self.publish_drive_command(0.0, 0.0)
#             self.publish_failure_once()

#     def sweep_forward(self):
#         elapsed = self.now_sec() - self.phase_start_time

#         # If time is up OR we are about to hit something in the front, switch to reverse
#         if elapsed >= self.sweep_duration or self.min_distance_front < self.safety_bubble_radius:
#             if self.min_distance_front < self.safety_bubble_radius:
#                  self.get_logger().warn(f"Obstacle in front ({self.min_distance_front:.2f}m). Reversing early.")
#             self.set_phase(SearchPhase.SWEEP_REVERSE)
#             return

#         # Hard left forward
#         self.publish_drive_command(self.sweep_speed, self.max_steering_angle)

#     def sweep_reverse(self):
#         elapsed = self.now_sec() - self.phase_start_time

#         # If time is up OR we are about to hit something in the rear, switch to forward
#         if elapsed >= self.sweep_duration or self.min_distance_rear < self.safety_bubble_radius:
#             if self.min_distance_rear < self.safety_bubble_radius:
#                  self.get_logger().warn(f"Obstacle in rear ({self.min_distance_rear:.2f}m). Moving forward early.")

#             self.sweep_cycles += 1

#             if self.sweep_cycles >= self.max_sweep_cycles:
#                 self.set_phase(SearchPhase.FAILED)
#             else:
#                 self.set_phase(SearchPhase.SWEEP_FORWARD)
#             return

#         # Hard right reverse (keeps the nose turning left)
#         self.publish_drive_command(-self.sweep_speed, -self.max_steering_angle)

#     def reset_search(self):
#         self.phase = SearchPhase.SWEEP_FORWARD
#         self.phase_start_time = self.now_sec()
#         self.sweep_cycles = 0
#         self.failure_sent = False
#         self.min_distance_front = float('inf')
#         self.min_distance_rear = float('inf')

#     def set_phase(self, phase):
#         self.phase = phase
#         self.phase_start_time = self.now_sec()
#         self.get_logger().info(f"Sweep Phase -> {phase.name}")

#     def publish_failure_once(self):
#         if self.failure_sent:
#             return

#         msg = Bool()
#         msg.data = True
#         self.failed_pub.publish(msg)
#         self.failure_sent = True
#         self.get_logger().warn("Meter search failed after completing all sweep cycles.")

#     def publish_drive_command(self, speed, steering_angle):
#         drive_cmd = AckermannDriveStamped()
#         drive_cmd.header.stamp = self.get_clock().now().to_msg()
#         drive_cmd.header.frame_id = "base_link"
#         drive_cmd.drive.speed = float(speed)
#         drive_cmd.drive.steering_angle = float(steering_angle)

#         self.drive_pub.publish(drive_cmd)

#     def now_sec(self):
#         return self.get_clock().now().nanoseconds * 1e-9


# def main(args=None):
#     rclpy.init(args=args)
#     node = MeterSearchController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()
