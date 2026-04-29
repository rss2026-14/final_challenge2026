#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from enum import Enum
import math
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from ackermann_msgs.msg import AckermannDriveStamped
from vs_msgs.msg import ConeLocation


class State(Enum):
    WAITING = 1
    NAVIGATING = 2
    OBSTACLE_PAUSE = 3
    METER_SEARCH = 4
    PARKING = 5
    PARKED = 6
    DONE = 7


class BoatingExecutive(Node):
    def __init__(self):
        super().__init__("boating_executive")

        self.state = State.WAITING
        self.previous_state = State.WAITING

        self.current_pose = None
        self.goals = []
        self.current_goal = None

        self.park_start_time = 0.0

        self.traffic_light_obstacle = False
        self.person_obstacle = False

        self.goal_pub = self.create_publisher(
            PoseStamped,
            "/planner/goal",
            10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            "/vesc/low_level/input/navigation",
            1
        )

        self.state_pub = self.create_publisher(
            String,
            "/mission_state",
            10
        )

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_callback,
            10
        )

        self.create_subscription(
            Bool,
            "/parking_success",
            self.parking_success_callback,
            10
        )

        self.create_subscription(
            Bool,
            "/traffic_light_obstacle_alert",
            self.traffic_light_obstacle_callback,
            10
        )

        self.create_subscription(
            Bool,
            "/person_obstacle_alert",
            self.person_obstacle_callback,
            10
        )

        self.create_subscription(
            ConeLocation,
            "/relative_parking_meter",
            self.parking_meter_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info("State Decider Initialized")

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg: PoseStamped):
        self.goals.append(msg)

        if self.state == State.WAITING:
            self.current_goal = self.goals.pop(0)
            self.state = State.NAVIGATING
            self.goal_pub.publish(self.current_goal)

            self.get_logger().info(
                f"Goal received. Starting navigation. {len(self.goals)} goals left in queue."
            )

    def parking_success_callback(self, msg: Bool):
        if msg.data and self.state == State.PARKING:
            self.get_logger().info(
                "Parking controller confirmed success. Holding for 5 seconds."
            )
            self.state = State.PARKED
            self.park_start_time = time.time()

    def parking_meter_callback(self, msg: ConeLocation):
        if self.state == State.METER_SEARCH:
            self.get_logger().info(
                f"Parking meter found at x={msg.x_pos:.2f}, y={msg.y_pos:.2f}. Switching to PARKING."
            )
            self.state = State.PARKING

    def traffic_light_obstacle_callback(self, msg: Bool):
        self.traffic_light_obstacle = msg.data
        self.update_obstacle_state()

    def person_obstacle_callback(self, msg: Bool):
        self.person_obstacle = msg.data
        self.update_obstacle_state()

    def update_obstacle_state(self):
        obstacle_detected = self.traffic_light_obstacle or self.person_obstacle

        if obstacle_detected:
            if self.state in [State.NAVIGATING, State.METER_SEARCH, State.PARKING]:
                self.previous_state = self.state
                self.state = State.OBSTACLE_PAUSE
                self.get_logger().warn("Obstacle detected. Pausing mission.")
                self.hit_the_brakes()

        else:
            if self.state == State.OBSTACLE_PAUSE:
                self.get_logger().info("Obstacle cleared. Resuming previous state.")
                self.state = self.previous_state

    def loop(self):
        self.publish_state()

        if self.state == State.WAITING:
            self.hit_the_brakes()

        elif self.state == State.NAVIGATING:
            # Keep publishing the goal so the planner receives it even if it started late.
            if self.current_goal is not None:
                self.goal_pub.publish(self.current_goal)

            dist = self.distance_to_goal()

            if dist < 2.0:
                self.get_logger().info(
                    f"Within 2m of goal. Distance: {dist:.2f}. Starting meter search."
                )
                self.state = State.METER_SEARCH

        elif self.state == State.METER_SEARCH:
            pass

        elif self.state == State.PARKING:
            pass

        elif self.state == State.PARKED:
            self.hit_the_brakes()

            elapsed_time = time.time() - self.park_start_time

            if elapsed_time >= 5.0:
                self.get_logger().info("Finished 5 second parking hold.")

                if len(self.goals) > 0:
                    self.current_goal = self.goals.pop(0)
                    self.state = State.NAVIGATING
                    self.goal_pub.publish(self.current_goal)

                    self.get_logger().info(
                        f"Moving to next goal. {len(self.goals)} goals left in queue."
                    )
                else:
                    self.state = State.DONE
                    self.get_logger().info("Course complete.")

        elif self.state == State.OBSTACLE_PAUSE:
            self.hit_the_brakes()

        elif self.state == State.DONE:
            self.hit_the_brakes()

    def publish_state(self):
        state_msg = String()
        state_msg.data = self.state.name
        self.state_pub.publish(state_msg)

    def distance_to_goal(self):
        if self.current_pose is None or self.current_goal is None:
            return float("inf")

        dx = self.current_goal.pose.position.x - self.current_pose.position.x
        dy = self.current_goal.pose.position.y - self.current_pose.position.y

        return math.sqrt(dx**2 + dy**2)

    def hit_the_brakes(self):
        self.publish_drive_command(0.0, 0.0)

    def publish_drive_command(self, speed, steering_angle):
        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = self.get_clock().now().to_msg()
        drive_cmd.header.frame_id = "base_link"
        drive_cmd.drive.speed = float(speed)
        drive_cmd.drive.steering_angle = float(steering_angle)

        self.drive_pub.publish(drive_cmd)


def main(args=None):
    rclpy.init(args=args)

    node = BoatingExecutive()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
