import rclpy
from rclpy.node import Node
from enum import Enum
import math
import time

# ROS2 Standard Messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, String


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
        super().__init__('boating_executive')

        self.state = State.WAITING
        self.previous_state = State.WAITING

        self.current_pose = None

        self.goals = []
        self.current_goal = None
        self.park_start_time = 0.0

        # --- Publishers ---
        # Publish goals to your path planner (e.g., Pure Pursuit)
        self.goal_pub = self.create_publisher(PoseStamped, '/planner/goal', 10)
        # Emergency brakes
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Broadcast our state to the rest of the car
        self.state_pub = self.create_publisher(String, '/mission_state', 10)

        # --- Subscribers ---
        # 1. Real Odometry for localization
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 2. THE INTERRUPT: Listens to your separate red light / pedestrian node
        # Expecting a True (obstacle) or False (clear)
        self.create_subscription(Bool, '/safety/obstacle_alert', self.obstacle_callback, 10)

        # 3. Getting the target locations
        self.create_subscription(PoseStamped, '/basement_point_publisher', self.goal_callback, 10)

        self.create_subscription(Bool, '/parking_success', self.parking_success_callback, 10)
        self.park_start_time = 0.0

        # Main loop for standard state management
        self.timer = self.create_timer(0.1, self.loop)

    # ==========================================
    #             THE INTERRUPT
    # ==========================================
    def obstacle_callback(self, msg: Bool):
        """ This triggers instantly when the obstacle node publishes. """
        is_obstacle_detected = msg.data

        if is_obstacle_detected:
            # Only interrupt if we are actually moving
            if self.state in [State.NAVIGATING, State.METER_SEARCH, State.PARKING]:
                self.get_logger().warn("🔴")
                self.previous_state = self.state
                self.state = State.OBSTACLE_PAUSE
                self.hit_the_brakes()

        else:
            # If the coast is clear and we are paused, resume
            if self.state == State.OBSTACLE_PAUSE:
                self.get_logger().info("🟢")
                self.state = self.previous_state

    def odom_callback(self, msg: Odometry):
        # Save the actual position of the robot
        self.current_pose = msg.pose.pose

    def distance_to_goal(self):
        if not self.current_pose or not self.current_goal:
            return float('inf')

        dx = self.current_goal.pose.position.x - self.current_pose.position.x
        dy = self.current_goal.pose.position.y - self.current_pose.position.y
        return math.sqrt(dx**2 + dy**2)


    def parking_success_callback(self, msg: Bool):
        if msg.data and self.state == State.PARKING:
            self.get_logger().info("Parking node confirmed success! Waiting 5 seconds...")
            # SAVE IMAGE
            # e.g., self.save_bounding_box_image()

            self.state = State.PARKED
            self.park_start_time = time.time()

    def goal_callback(self, msg: PoseStamped):
        if self.state == State.WAITING:
            self.current_goal = msg
            self.state = State.NAVIGATING
            self.get_logger().info("Goal received")

    def loop(self):
        # Broadcast current state so other nodes know what's happening
        state_msg = String()
        state_msg.data = self.state.name
        self.state_pub.publish(state_msg)

        if self.state == State.NAVIGATING:
            # 1. Keep telling the path planner where to go
            self.goal_pub.publish(self.current_goal)

            # 2. Check Odometry distance to see if we arrived
            dist = self.distance_to_goal()
            if dist < 2.0: # Close enough to the point!
                self.get_logger().info(f"Within 2m (Distance: {dist:.2f}). Starting Meter Search!")
                self.state = State.METER_SEARCH

        elif self.state == State.METER_SEARCH:
            # checking for the YOLO parking meter flag
            pass

        elif self.state == State.OBSTACLE_PAUSE:
            # Continuously spam the brakes just to be safe
            self.hit_the_brakes()


    # TODO: stop the car
    def hit_the_brakes(self):
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BoatingExecutive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
