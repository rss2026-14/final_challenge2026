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
    REVERSE = 7
    DONE = 8


class BoatingExecutive(Node):
    def __init__(self):
        super().__init__("boating_executive")

        self.state = State.WAITING
        self.previous_state = State.WAITING
        self.state_just_changed = False
        self.resume_state = State.WAITING

        self.current_pose = None
        self.goals = []
        self.current_goal = None

        self.park_start_time = None
        self.reverse_start_time = None
        self.last_goal_publish_time = 0.0

        self.declare_parameter("meter_search_speed", 1.0)
        self.declare_parameter("meter_search_steering_angle", 0.34)
        self.meter_search_speed = (
            self.get_parameter("meter_search_speed")
            .get_parameter_value()
            .double_value
        )
        self.meter_search_steering_angle = (
            self.get_parameter("meter_search_steering_angle")
            .get_parameter_value()
            .double_value
        )

        self.traffic_light_obstacle = False
        self.goal_pub = self.create_publisher(
            PoseStamped,
            "/planner/goal",
            10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            "/vesc/high_level/input/nav_0",
            1
        )

        self.state_pub = self.create_publisher(
            String,
            "/mission_state",
            10
        )

        self.create_subscription(
            Odometry,
            "/pf/pose/odom",
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
            ConeLocation,
            "/relative_parking_meter",
            self.parking_meter_callback,
            10
        )
        self.create_subscription(
            Bool,
            "/meter_search_failed",
            self.meter_search_failed_callback,
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
            self.set_state(State.NAVIGATING)
            self.goal_pub.publish(self.current_goal)

            self.get_logger().info(
                f"Goal received. Starting navigation. {len(self.goals)} goals left in queue."
            )

    def parking_success_callback(self, msg: Bool):
        if msg.data and self.state == State.PARKING:
            self.get_logger().info(
                "Parking controller confirmed success. Holding for 5 seconds."
            )
            self.set_state(State.PARKED)
            # self.park_start_time = self.get_clock().now()

    def parking_meter_callback(self, msg: ConeLocation):
        if self.state == State.METER_SEARCH:
            self.get_logger().info(
                f"Parking meter found at x={msg.x_pos:.2f}, y={msg.y_pos:.2f}. Switching to PARKING."
            )
            self.set_state(State.PARKING)

    def meter_search_failed_callback(self, msg: Bool):
        if msg.data and self.state == State.METER_SEARCH:
            self.get_logger().warn("Meter search failed. Ending mission.")
            self.set_state(State.DONE)

    def traffic_light_obstacle_callback(self, msg: Bool):
        self.traffic_light_obstacle = msg.data
        self.update_obstacle_state()

    def update_obstacle_state(self):
        obstacle_detected = self.traffic_light_obstacle

        if obstacle_detected:
            if self.state in [State.NAVIGATING, State.METER_SEARCH, State.PARKING]:
                self.resume_state = self.state
                self.set_state(State.OBSTACLE_PAUSE)

                self.get_logger().warn("Obstacle detected. Pausing mission.")
                self.hit_the_brakes()

        else:
            if self.state == State.OBSTACLE_PAUSE:
                self.get_logger().info("Obstacle cleared. Resuming previous state.")
                self.set_state(self.resume_state)

    def loop(self):
        self.publish_state()

        if self.state == State.WAITING:
            self.hit_the_brakes()

        elif self.state == State.NAVIGATING:
            # Keep publishing the goal so the planner receives it even if it started late.
            # if self.current_goal is not None:
            #     self.goal_pub.publish(self.current_goal)

            # Try publish once
            # if self.state_just_changed:
            #     if self.current_goal is not None:
            #         self.get_logger().info("Publishing goal ONCE")
            #         self.goal_pub.publish(self.current_goal)

            if self.state_just_changed or time.time() - self.last_goal_publish_time > 2.0:
                if self.current_goal is not None:
                    self.get_logger().info("Publishing goal")
                    self.goal_pub.publish(self.current_goal)
                    self.last_goal_publish_time = time.time()

            self.state_just_changed = False

            dist = self.distance_to_goal()

            if dist < 1.0:
                self.get_logger().info(
                    f"Within 1.0m of goal. Distance: {dist:.2f}. Starting meter search."
                )
                self.set_state(State.METER_SEARCH)

        elif self.state == State.METER_SEARCH:
            pass

        elif self.state == State.PARKING:
            pass

        elif self.state == State.PARKED:
            self.hit_the_brakes()

            elapsed_time_parking = (self.get_clock().now() - self.park_start_time).nanoseconds * 1e-9
            self.get_logger().info(
                f"Holding parked state: {elapsed_time_parking:.2f}s"
            )

            if elapsed_time_parking >= 5.0:
                self.get_logger().info("Finished 5 second parking hold.")
                self.set_state(State.REVERSE)

            #     if len(self.goals) > 0:
            #         self.current_goal = self.goals.pop(0)
            #         # self.set_state(State.NAVIGATING)
            #         # self.goal_pub.publish(self.current_goal)

            #         self.get_logger().info(
            #             f"Moving to next goal. {len(self.goals)} goals left in queue."
            #         )
            #     else:
            #         self.set_state(State.DONE)
            #         self.get_logger().info("Course complete.")

                # self.reverse_time = self.get_clock().now()

                # elapsed_time_reversed = (self.get_clock().now() - self.reverse_time).nanoseconds * 1e-9
                # if elapsed_time_reversed <= 2.0:
                #     self.publish_drive_command(-0.7, 0.0)

        elif self.state == State.REVERSE:

            elapsed_reverse = (
                self.get_clock().now() - self.reverse_start_time
            ).nanoseconds * 1e-9

            if elapsed_reverse < 5.0:
                self.publish_drive_command(-1.0, 0.0)

            else:
                self.hit_the_brakes()

                if len(self.goals) > 0:
                    self.current_goal = self.goals.pop(0)

                    self.get_logger().info(
                        f"Moving to next goal. {len(self.goals)} goals left."
                    )

                    self.set_state(State.NAVIGATING)

                else:
                    self.get_logger().info("Course complete.")
                    self.set_state(State.DONE)

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
        drive_cmd.drive.speed = speed
        drive_cmd.drive.steering_angle = float(steering_angle)

        self.drive_pub.publish(drive_cmd)

    def set_state(self, new_state):
        allowed = {
            State.WAITING: [State.NAVIGATING],
            State.NAVIGATING: [State.METER_SEARCH, State.OBSTACLE_PAUSE],
            State.METER_SEARCH: [State.PARKING],
            State.PARKING: [State.PARKED],
            State.PARKED: [State.REVERSE],
            State.REVERSE: [State.NAVIGATING, State.DONE],
            State.OBSTACLE_PAUSE: [State.NAVIGATING],
            State.DONE: []
        }

        if new_state not in allowed[self.state]:
            self.get_logger().warn(f"INVALID TRANSITION {self.state} -> {new_state}")
            return

        if new_state == State.PARKED:
            self.park_start_time = self.get_clock().now()
        if new_state == State.REVERSE:
            self.reverse_start_time = self.get_clock().now()

        self.get_logger().info(f"{self.state.name} -> {new_state.name}")
        self.previous_state = self.state
        self.state = new_state
        self.state_just_changed = True


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
