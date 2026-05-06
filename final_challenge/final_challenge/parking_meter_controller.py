#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from rcl_interfaces.msg import SetParametersResult
from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool

class ParkingController(Node):
    """
    A controller for parking in front of a parking meter.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """

    def __init__(self):
        super().__init__("parking_controller")

        self.declare_parameter("drive_topic", "/vesc/low_level/input/navigation")
        DRIVE_TOPIC = self.get_parameter("drive_topic").get_parameter_value().string_value  # set in launch file; different for simulator vs racecar

        self.declare_parameter("odom_topic", "/pf/pose/odom")
        ODOM_TOPIC = self.get_parameter("odom_topic").get_parameter_value().string_value

        self.declare_parameter("parking_distance", 0.3)
        self.parking_distance = self.get_parameter("parking_distance").get_parameter_value().double_value

        self.declare_parameter("angle_multiplier", 2.5)
        self.declare_parameter("velocity", 0.7)
        self.declare_parameter("reverse_range", 0.1)
        self.declare_parameter("distance_sensitivity", 0.1)
        self.distance_sensitivity = self.get_parameter("distance_sensitivity").get_parameter_value().double_value
        self.angle_multiplier = self.get_parameter("angle_multiplier").get_parameter_value().double_value
        self.reverse_range = self.get_parameter("reverse_range").get_parameter_value().double_value
        self.velocity = self.get_parameter("velocity").get_parameter_value().double_value

        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)

        self.success_pub = self.create_publisher(Bool, "/parking_success", 10)

        self.create_subscription(
            ConeLocation, "/relative_parking_meter", self.relative_callback, 1)
        self.create_subscription(
            Odometry, ODOM_TOPIC, self.odom_callback, 1)

        self.current_state = "WAITING"
        self.create_subscription(String, "/mission_state", self.state_callback, 10)

        self.relative_x = 0.0
        self.relative_y = 0.0
        self.has_meter_target = False
        self.current_pose = None
        self.locked_meter_map_x = None
        self.locked_meter_map_y = None
        self.success_sent = False
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info("Parking Controller Initialized")

    def state_callback(self, msg):
        previous_state = self.current_state
        self.current_state = msg.data

        if previous_state != "PARKING" and self.current_state == "PARKING":
            self.success_sent = False
            self.locked_meter_map_x = None
            self.locked_meter_map_y = None
            self.get_logger().info("Parking state active; locking first meter target in map frame.")

    def relative_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        self.has_meter_target = True

        if self.current_state == "PARKING" and not self.has_locked_meter_target():
            self.lock_meter_target()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def control_loop(self):
        # Only the executive is allowed to activate parking control.
        if self.current_state != "PARKING":
            return

        if not self.has_meter_target:
            self.get_logger().warn("Parking active, but no parking meter target received yet.")
            return

        if not self.has_locked_meter_target():
            self.lock_meter_target()

        if not self.has_locked_meter_target():
            self.get_logger().warn("Parking active, but cannot lock meter target without odometry.")
            return

        self.update_relative_error_from_locked_target()

        drive_cmd = AckermannDriveStamped()

        angle = np.arctan2(self.relative_y, self.relative_x)
        current_distance = np.sqrt(self.relative_x**2 + self.relative_y**2)
        distance_error = current_distance - self.parking_distance

        self.get_logger().info(
            f"Parking meter distance={current_distance:.3f} m, "
            f"x={self.relative_x:.3f} m, "
            f"y={self.relative_y:.3f} m, "
            f"target={self.parking_distance:.3f} m, "
            f"error={distance_error:.3f} m"
        )

        if abs(distance_error) < self.distance_sensitivity:
            if abs(angle) < self.reverse_range:
                # WE HAVE SUCCESSFULLY PARKED!
                steering_angle = 0.0
                velocity = 0.0

                # Tell the Executive we did it
                if not self.success_sent:
                    success_msg = Bool()
                    success_msg.data = True
                    self.success_pub.publish(success_msg)
                    self.success_sent = True

            else:
                steering_angle = -angle * self.angle_multiplier
                velocity = -self.velocity
        elif distance_error < 0:
            if abs(angle) < self.reverse_range:
                steering_angle = 0.0
                velocity = -self.velocity
            else:
                steering_angle = -angle * self.angle_multiplier
                velocity = -self.velocity
        else:
            if abs(angle) < self.reverse_range:
                steering_angle = 0.0
                velocity = self.velocity
            else:
                steering_angle = angle * self.angle_multiplier
                velocity = self.velocity

        steering_angle = np.clip(steering_angle, -0.34, 0.34)

        drive_cmd.header.stamp = self.get_clock().now().to_msg()
        drive_cmd.header.frame_id = 'base_link'
        drive_cmd.drive.speed = float(velocity)
        drive_cmd.drive.steering_angle = float(steering_angle)

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def has_locked_meter_target(self):
        return self.locked_meter_map_x is not None and self.locked_meter_map_y is not None

    def lock_meter_target(self):
        if self.current_pose is None or not self.has_meter_target:
            return

        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        yaw = self.get_yaw(self.current_pose.orientation)

        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        self.locked_meter_map_x = robot_x + cos_yaw * self.relative_x - sin_yaw * self.relative_y
        self.locked_meter_map_y = robot_y + sin_yaw * self.relative_x + cos_yaw * self.relative_y

        self.get_logger().info(
            f"Locked parking meter map target: "
            f"x={self.locked_meter_map_x:.3f}, "
            f"y={self.locked_meter_map_y:.3f}"
        )

    def update_relative_error_from_locked_target(self):
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        yaw = self.get_yaw(self.current_pose.orientation)

        dx = self.locked_meter_map_x - robot_x
        dy = self.locked_meter_map_y - robot_y

        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        self.relative_x = cos_yaw * dx + sin_yaw * dy
        self.relative_y = -sin_yaw * dx + cos_yaw * dy

    def get_yaw(self, orientation):
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)

        #################################

        error_msg.x_error = self.relative_x - self.parking_distance
        error_msg.y_error = self.relative_y
        error_msg.distance_error = np.sqrt(self.relative_x**2 + self.relative_y**2) - self.parking_distance

        self.error_pub.publish(error_msg)

    def parameters_callback(self, params):
        """
        Dynamically updates parameters when modified via 'ros2 param set'.
        """
        for param in params:
            if param.name == 'velocity':
                self.velocity = param.value
                self.speed = self.velocity # Sync fallback speed
                self.get_logger().info(f"Updated velocity to {self.velocity}")
            elif param.name == 'parking_distance':
                self.parking_distance = param.value
                self.get_logger().info(f"Updated parking_distance to {self.parking_distance}")
            elif param.name == 'angle_multiplier':
                self.angle_multiplier = param.value
                self.get_logger().info(f"Updated angle_multiplier to {self.angle_multiplier}")
            elif param.name == 'reverse_range':
                self.reverse_range = param.value
                self.get_logger().info(f"Updated reverse_range to {self.reverse_range}")
            elif param.name == 'distance_sensitivity':
                self.distance_sensitivity = param.value
                self.get_logger().info(f"Updated distance_sensitivity to {self.distance_sensitivity}")

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    pc = ParkingController()
    rclpy.spin(pc)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
