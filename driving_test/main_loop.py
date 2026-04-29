import rclpy
from rclpy.node import Node
from enum import Enum
import time
import math

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String # Placeholder YOLO/Point messages

class State(Enum):
    WAITING_FOR_GOALS = 1
    NAVIGATING = 2
    OBSTACLE_PAUSE = 3
    METER_SEARCH = 4
    PARKING_APPROACH = 5
    PARKED = 6
    MISSION_COMPLETE = 7

class BoatingSchoolExecutive(Node):
    def __init__(self):
        super().__init__('boating_school_executive')

        # State Machine Variables
        self.state = State.WAITING_FOR_GOALS
        self.previous_state = None

        # Navigation & Goal Variables
        self.goals = []          # List to hold loc1, loc2, and start
        self.current_goal = None
        self.start_pose = None   # Save where we started for the return trip

        # Sensor Data Storage
        self.current_pose = None
        self.front_distance = float('inf')
        self.sees_red_light = False
        self.sees_pedestrian = False
        self.meter_bounding_box = None

        # Timers, Flags
        self.park_start_time = 0.0
        self.camera_image = None

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10) # Or your pure pursuit topic

        # Subscribers
        # 1. Basement Points (Target locations)
        self.create_subscription(String, '/basement_point_publisher', self.points_callback, 10)

        # 2. YOLO Detections
        self.create_subscription(String, '/yolo/detections', self.yolo_callback, 10)

        # 3. Camera Image (for saving upon parking)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # 4. LiDAR (for the 1-meter parking rule)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # 5. Odometry/Localization
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Main Control Loop
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("SpongeBob Can't Drive Node Initialized. Awaiting locations...")

    def points_callback(self, msg):
        if self.state == State.WAITING_FOR_GOALS:
            # TODO: Parse loc1 and loc2 from message
            loc1 = PoseStamped() # Extract from actual msg
            loc2 = PoseStamped()

            # Save our current start pose for the bonus points return trip
            self.start_pose = self.current_pose

            # Load up the queue
            self.goals = [loc1, loc2, self.start_pose]
            self.current_goal = self.goals.pop(0)

            self.get_logger().info("Goals received! Starting Mrs. Puff's Exam...")
            self.transition_to(State.NAVIGATING)

    def yolo_callback(self, msg):
        # TODO: yolo message receive and interpret
        # TODO: update self.sees_red_light, self.sees_pedestrian, self.meter_bounding_box
        pass

    def scan_callback(self, msg):
        front_index = len(msg.ranges) // 2
        self.front_distance = msg.ranges[front_index]

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def image_callback(self, msg):
        self.camera_image = msg

    # ==========================================
    #            MAIN CONTROL LOOP
    # ==========================================

    def control_loop(self):

        # If navigating or searching, and we see an obstacle, stop
        if self.state in [State.NAVIGATING, State.METER_SEARCH, State.PARKING_APPROACH]:
            if self.sees_red_light or self.sees_pedestrian: # sep states
                self.get_logger().warn("Obstacle detected, please stop Ghost")
                self.previous_state = self.state
                self.transition_to(State.OBSTACLE_PAUSE)

        # State Machine
        if self.state == State.WAITING_FOR_GOALS:
            self.stop_boat()

        elif self.state == State.NAVIGATING:
            # Publish goal to your path planner
            self.goal_pub.publish(self.current_goal)

            # Check if we are close to the goal to start looking for the meter
            if self.distance_to_goal() < 2.0:
                self.get_logger().info("Approaching target area. Searching for parking meter...")
                self.transition_to(State.METER_SEARCH)

        elif self.state == State.OBSTACLE_PAUSE:
            self.stop_boat()
            # "you can go once you stop seeing the red light"
            if not self.sees_red_light and not self.sees_pedestrian:
                self.get_logger().info("Path clear. Continuing...")
                self.transition_to(self.previous_state)

        elif self.state == State.METER_SEARCH:
            # Slow down the path planner or cap velocity here
            self.goal_pub.publish(self.current_goal)

            if self.meter_bounding_box is not None:
                self.get_logger().info("Parking meter identified! Initiating parking approach...")
                self.transition_to(State.PARKING_APPROACH)

        elif self.state == State.PARKING_APPROACH:
            # PARK THAT GHOST!!!!! WOOO
            approach_twist = Twist()
            # approach_twist.linear.x = ...
            # approach_twist.angular.z = ...
            self.cmd_vel_pub.publish(approach_twist)

            # Check 1-meter rule
            if self.front_distance <= 1.0:
                self.get_logger().info("Reached 1 meter from meter. Parking!")
                self.park_start_time = time.time()
                self.save_parking_image()
                self.transition_to(State.PARKED)

        elif self.state == State.PARKED:
            self.stop_boat()
            # Wait for 5 seconds
            if (time.time() - self.park_start_time) >= 5.0:
                if len(self.goals) > 0:
                    self.current_goal = self.goals.pop(0)
                    self.get_logger().info("Parking complete. Heading to next goal...")
                    self.transition_to(State.NAVIGATING)
                else:
                    self.get_logger().info("Course complete! Hopefully we didn't crash.")
                    self.transition_to(State.MISSION_COMPLETE)

        elif self.state == State.MISSION_COMPLETE:
            self.stop_boat()

    def transition_to(self, new_state):
        if self.state != new_state:
            self.state = new_state

    def stop_boat(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def distance_to_goal(self):
        if not self.current_pose or not self.current_goal:
            return float('inf')
        dx = self.current_pose.position.x - self.current_goal.pose.position.x
        dy = self.current_pose.position.y - self.current_goal.pose.position.y
        return math.sqrt(dx**2 + dy**2)

    def save_parking_image(self):
        # TODO: Implement image saving logic using cv_bridge
        # cv2.imwrite("parking_spot.jpg", cv_image)???
        self.get_logger().info("IMAGE SAVED with bounding box!!! Yay")

def main(args=None):
    rclpy.init(args=args)
    node = BoatingSchoolExecutive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
