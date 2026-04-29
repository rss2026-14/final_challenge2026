import rclpy
from rclpy.node import Node
from enum import Enum
import math
import time
from cv_bridge import CvBridge
import cv2
# ROS2 Standard Messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, String
from ackermann_msgs.msg import AckermannDriveStamped



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
        self.drive_pub = self.create_publisher(AckermannDriveStamped,
                                               self.drive_topic,
                                               1)
        # Broadcast our state to the rest of the car
        self.state_pub = self.create_publisher(String, '/mission_state', 10)
        self.count = 0

        # --- Subscribers ---
        # 1. Real Odometry for localization
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 2. THE INTERRUPT: Listens to your separate red light / pedestrian node
        # Expecting a True (obstacle) or False (clear)
        self.create_subscription(Bool, '/safety/obstacle_alert', self.obstacle_callback, 10)

        # 3. Getting the target locations
        # self.create_subscription(PoseStamped, '/basement_point_publisher', self.goal_callback, 10)
        self.create_subscription(PoseStamped, '/clicked_point', self.goal_callback, 10)

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
            self.count+=1
            bridge = CvBridge()
            # Convert ROS Image message to OpenCV image
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            cv2.imwrite(f'successful_park{self.count}.png', cv_image)

            self.state = State.PARKED
            self.park_start_time = time.time()

    def goal_callback(self, msg: PoseStamped):
        self.goals.append(msg)

        # If we were waiting for a goal to start the mission, get moving!
        if self.state == State.WAITING:
            self.current_goal = self.goals.pop(0)
            self.state = State.NAVIGATING
            self.get_logger().info(f"Goal received! {len(self.goals)} more in queue.")
    def loop(self):
        # Broadcast current state so other nodes know what's happening
        state_msg = String()
        state_msg.data = self.state.name
        self.state_pub.publish(state_msg)

        if self.state == State.NAVIGATING:
            self.goal_pub.publish(self.current_goal)
            dist = self.distance_to_goal()
            if dist < 2.0:
                self.get_logger().info(f"Within 2m (Distance: {dist:.2f}). Starting Meter Search!")
                self.state = State.METER_SEARCH

        elif self.state == State.METER_SEARCH:
            # TODO: Listen to YOLO. Once bounding box is found, switch to State.PARKING
            # self.state = State.PARKING
            pass

        elif self.state == State.PARKING:
            # We don't publish cmd_vel here. The ParkingController node takes over.
            pass

        elif self.state == State.PARKED:
            self.hit_the_brakes() # Keep the car locked in place

            elapsed_time = time.time() - self.park_start_time
            if elapsed_time >= 5.0:
                self.get_logger().info("⏰ 5 seconds elapsed!")

                # Check if we have another goal (loc2 or start)
                if len(self.goals) > 0:
                    self.current_goal = self.goals.pop(0)
                    self.state = State.NAVIGATING
                    self.get_logger().info("Moving to next goal...")
                else:
                    self.state = State.DONE
                    self.get_logger().info("🏁 Course complete! License acquired.")

        elif self.state == State.OBSTACLE_PAUSE:
            self.hit_the_brakes()

        elif self.state == State.DONE:
            self.hit_the_brakes()


    # TODO: stop the car
    def hit_the_brakes(self):
        self._publish_drive_command(0.0, 0.0)

    def _publish_drive_command(self, speed, steering_angle):
        """ Helper function to construct and publish the Ackermann message. """
        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = self.get_clock().now().to_msg()
        drive_cmd.header.frame_id = 'base_link'
        drive_cmd.drive.speed = float(speed)
        drive_cmd.drive.steering_angle = float(steering_angle)

        self.drive_pub.publish(drive_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BoatingExecutive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
