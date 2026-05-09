#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from enum import Enum
import math
import time
import copy

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
    THREE_POINT_TURN = 8


class BoatingExecutive(Node):
    def __init__(self):
        super().__init__("boating_executive")

        self.state = State.WAITING
        self.previous_state = State.WAITING
        self.state_just_changed = False
        self.resume_state = State.WAITING

        self.current_pose = None

        # Stores the robot's original starting pose as a PoseStamped.
        self.start_pose_goal = None

        # Stores only clicked outbound goals.
        self.outbound_goals = []

        # Stores [start_pose, goal_1, goal_2, ..., final_goal].
        self.start_and_goal_points = []

        # Current route being followed.
        # Outbound: [goal_1, goal_2, ..., final_goal]
        # Return: [previous_goal, ..., goal_1, start_pose]
        self.route_goals = []
        self.route_index = 0
        self.current_goal = None

        self.is_returning = False
        self.mission_started = False

        self.turn_start_time = None
        self.last_goal_publish_time = 0.0

        self.declare_parameter("min_outbound_goals", 2)
        self.min_outbound_goals = (
            self.get_parameter("min_outbound_goals")
            .get_parameter_value()
            .integer_value
        )

        self.declare_parameter("goal_reach_distance", 0.15)
        self.goal_reach_distance = (
            self.get_parameter("goal_reach_distance")
            .get_parameter_value()
            .double_value
        )

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
            "/vesc/input/navigation",
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

        self.create_subscription(
            Bool,
            "/trajectory/reached",
            self.trajectory_reached_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info("State Decider Initialized: Indexed Route Mode")

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

        # Latch the very first odometry pose as the fixed start forever.
        if self.start_pose_goal is None:
            self.start_pose_goal = self.pose_to_pose_stamped(msg)
            self.start_and_goal_points = [self.start_pose_goal]

            self.get_logger().info(
                f"Latched fixed start pose: "
                f"x={self.start_pose_goal.pose.position.x:.2f}, "
                f"y={self.start_pose_goal.pose.position.y:.2f}"
            )

    def pose_to_pose_stamped(self, odom_msg: Odometry):
        pose_stamped = PoseStamped()
        pose_stamped.header = odom_msg.header
        pose_stamped.pose = copy.deepcopy(odom_msg.pose.pose)
        return pose_stamped

    def goal_callback(self, msg: PoseStamped):
        if self.is_returning:
            self.get_logger().warn("Ignoring new goal because robot is already returning.")
            return

        if self.start_pose_goal is None:
            self.get_logger().warn(
                "Received goal before start pose was available. Waiting for odometry."
            )
            return

        # Save every clicked goal. Do not pop/remove anything.
        self.outbound_goals.append(msg)
        self.start_and_goal_points.append(msg)

        self.get_logger().info(
            f"Saved outbound goal {len(self.outbound_goals)}. "
            f"Total saved points including start: {len(self.start_and_goal_points)}"
        )

        # If this is the first goal, start navigating.
        if self.state == State.WAITING:
            self.mission_started = True
            self.route_goals = self.outbound_goals
            self.route_index = 0
            self.current_goal = self.route_goals[self.route_index]

            self.set_state(State.NAVIGATING)
            self.publish_current_goal()

            self.get_logger().info("Starting outbound navigation.")

    def parking_success_callback(self, msg: Bool):
        # Parking bypassed for this version.
        pass

    def parking_meter_callback(self, msg: ConeLocation):
        # Parking bypassed for this version.
        pass

    def meter_search_failed_callback(self, msg: Bool):
        # Parking bypassed for this version.
        pass

    def trajectory_reached_callback(self, msg: Bool):
        if msg.data and self.state == State.NAVIGATING:
            self.get_logger().info("Trajectory follower reported goal reached.")
            self.advance_to_next_route_point()

    def advance_to_next_route_point(self):
        if self.state != State.NAVIGATING:
            return

        self.route_index += 1

        if self.route_index < len(self.route_goals):
            self.current_goal = self.route_goals[self.route_index]

            self.get_logger().info(
                f"Advancing to route point {self.route_index + 1}/"
                f"{len(self.route_goals)}."
            )

            self.set_state(State.NAVIGATING)
            self.publish_current_goal()
            return

        # Finished current route.
        if not self.is_returning:
            if len(self.outbound_goals) < self.min_outbound_goals:
                self.get_logger().info(
                    f"Reached current final goal, but only "
                    f"{len(self.outbound_goals)}/{self.min_outbound_goals} "
                    "outbound goals have been received. Waiting for more goals."
                )

                self.route_index = max(0, len(self.route_goals) - 1)
                self.hit_the_brakes()
                return

            self.get_logger().info(
                "Reached final outbound goal. Starting three-point turn."
            )
            self.set_state(State.THREE_POINT_TURN)
        else:
            self.get_logger().info("Returned to start point. Mission complete.")
            self.set_state(State.DONE)

    def traffic_light_obstacle_callback(self, msg: Bool):
        self.traffic_light_obstacle = msg.data
        self.update_obstacle_state()

    def update_obstacle_state(self):
        obstacle_detected = self.traffic_light_obstacle

        if obstacle_detected:
            if self.state in [State.NAVIGATING, State.THREE_POINT_TURN]:
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
            if self.state_just_changed or time.time() - self.last_goal_publish_time > 2.0:
                self.publish_current_goal()

            self.state_just_changed = False

            dist = self.distance_to_goal()

            if dist < self.goal_reach_distance:
                self.get_logger().info(
                    f"Within {self.goal_reach_distance:.2f}m of current goal. "
                    f"Distance: {dist:.2f}."
                )
                self.advance_to_next_route_point()

        elif self.state == State.THREE_POINT_TURN:
            if self.turn_start_time is None:
                self.turn_start_time = self.get_clock().now()

            elapsed = (
                self.get_clock().now() - self.turn_start_time
            ).nanoseconds * 1e-9

            if elapsed < 1.0:
                # Phase 1: forward left
                self.publish_drive_command(0.85, 0.34)

            elif elapsed < 2.5:
                # Phase 2: reverse right
                self.publish_drive_command(-1.0, -0.34)

            elif elapsed < 4.0:
                # Phase 3: forward straight
                self.publish_drive_command(1.0, 0.0)

            else:
                self.hit_the_brakes()

                self.get_logger().info(
                    "Three-point turn complete."
                )

                #self.is_returning = True

                ## start_and_goal_points = [start, goal_1, goal_2, ..., final_goal]
                ## Robot is currently at final_goal, so return through everything before it:
                ## [previous_goal, ..., goal_1, start]
                #self.route_goals = list(reversed(self.start_and_goal_points[:-1]))
                #self.route_index = 0

                #if len(self.route_goals) > 0:
                #   self.current_goal = self.route_goals[self.route_index]
                #   self.set_state(State.NAVIGATING)
                #   self.publish_current_goal()

                #    self.get_logger().info(
                #        f"Beginning return route with {len(self.route_goals)} points."
                #    )
                #else:
                #    self.get_logger().warn(
                #        "No return route points available. Ending mission."
                #    )
                #    self.set_state(State.DONE)

        elif self.state == State.OBSTACLE_PAUSE:
            self.hit_the_brakes()

        elif self.state == State.DONE:
            self.hit_the_brakes()

    def publish_current_goal(self):
        if self.current_goal is None:
            return

        self.goal_pub.publish(self.current_goal)
        self.last_goal_publish_time = time.time()

        self.get_logger().info(
            f"Publishing goal: x={self.current_goal.pose.position.x:.2f}, "
            f"y={self.current_goal.pose.position.y:.2f}"
        )

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

    def set_state(self, new_state):
        if new_state == self.state:
            self.state_just_changed = True
            return

        allowed = {
            State.WAITING: [
                State.NAVIGATING
            ],
            State.NAVIGATING: [
                State.NAVIGATING,
                State.THREE_POINT_TURN,
                State.OBSTACLE_PAUSE,
                State.DONE
            ],
            State.THREE_POINT_TURN: [
                State.NAVIGATING,
                State.OBSTACLE_PAUSE,
                State.DONE
            ],
            State.OBSTACLE_PAUSE: [
                State.NAVIGATING,
                State.THREE_POINT_TURN
            ],
            State.METER_SEARCH: [],
            State.PARKING: [],
            State.PARKED: [],
            State.DONE: []
        }

        if new_state not in allowed[self.state]:
            self.get_logger().warn(
                f"INVALID TRANSITION {self.state.name} -> {new_state.name}"
            )
            return

        if new_state == State.THREE_POINT_TURN:
            self.turn_start_time = self.get_clock().now()

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
