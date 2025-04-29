#! /usr/bin/env python3
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from tf_transformations import euler_from_quaternion
import time


class ObstacleDetection(Node):
    """
    Simple obstacle detection node that stops the robot when obstacles are too close.
    Uses a circular detection zone around the robot.
    
    TODO: Implement the detect_obstacle method to avoid obstacles!
    """
    def __init__(self):
        super().__init__("obstacle_detection")

        #wait for rviz
        time.sleep(5)
        
        # Safety parameters - use ROS parameter
        self.declare_parameter("stop_distance", 0.35)  # Default if not specified
        self.stop_distance = self.get_parameter("stop_distance").get_parameter_value().double_value
        self.get_logger().info(f"Using stop_distance: {self.stop_distance}m")
        self.pose = Pose()
        self.odom_sub = self.create_subscription(Odometry, "odom", self.get_odom_callback, qos_profile=qos_profile_sensor_data)
        # Store received data
        self.scan_ranges = []
        self.has_scan_received = False
        
        # Set up goal point
        self.x_g = 5.0 
        self.y_g = 5.0

        # Default motion command (slow forward)
        self.tele_twist = Twist()
        self.tele_twist.linear.x = 0.0
        self.tele_twist.angular.z = 0.0
        self.yaw = 0

        # Movement parameters
        self.max_linear_speed = 1.5
        self.min_linear_speed = 0.3
        self.angular_speed_factor = 4.0
        self.p_regulator = 1

        # Set up quality of service
        qos = QoSProfile(depth=10)

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", qos)
        
        # Subscribe to laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan, "scan", self.scan_callback, qos_profile=qos_profile_sensor_data
        )

        # Subscribe to teleop commands
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist, "cmd_vel_raw", self.cmd_vel_raw_callback, 
            qos_profile=qos_profile_sensor_data
        )

        # Set up timer for regular checking
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def get_odom_callback(self, msg):
        self.pose = msg.pose.pose
        oriList = [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(oriList)
        self.yaw = yaw
        self.get_logger().info(f"Robot state  {self.pose.position.x, self.pose.position.y, yaw}")
 
    def scan_callback(self, msg):
        """Store laser scan data when received"""
        self.scan_ranges = msg.ranges
        self.has_scan_received = True

    def cmd_vel_raw_callback(self, msg):
        """Store teleop commands when received"""
        self.tele_twist = msg

    def timer_callback(self):
        """Regular function to check for obstacles"""
        if self.has_scan_received:
            self.detect_obstacle()

    # NOTE taget från gotogoal 
    def calculate_steering_angle(self):
        """Angle toward goal"""
        return math.atan2(self.y_g - self.pose.position.y, self.x_g - self.pose.position.x)

    def calculate_linear_velocity(self):
        """Calculate forward speed with deceleration near goal"""
        distance = self.euclidean_distance()

        # Deceleration zone is twice the tolerance
        decel_zone = self.distance_tolerance * 2.0

        if distance < decel_zone:
            # Decelerate as we approach the goal
            speed = self.min_linear_speed + (
                self.max_linear_speed - self.min_linear_speed
            ) * (distance / decel_zone)
        else:
            # Normal speed
            speed = self.max_linear_speed

        # Ensure speed stays within bounds
        return max(self.min_linear_speed, min(speed, self.max_linear_speed))
    
    def euclidean_distance(self):
        """Distance between current position and goal"""
        # !FIXME: Mathematical Error in Distance Calculation
        return math.sqrt(
            (self.x_g - self.pose.position.x) ** 2
            + (self.y_g - self.pose.position.y) ** 2
        )
    
    # NOTE Egna funktioner
    def calculate_goal_angle(self):
        # return angle towards goal in relataion to the robot
        goal_angle = self.calculate_steering_angle()

        diff_angle = goal_angle - self.yaw
        diff_angle = math.atan2(math.sin(diff_angle), math.cos(diff_angle))

        return self.p_regulator * diff_angle

    def detect_obstacle(self):
        """
        TODO: Implement obstacle detection and avoidance!
        
        MAIN TASK:
        - Detect if any obstacle is too close to the robot (closer than self.stop_distance)
        - If an obstacle is detected, stop the robot's forward motion
        
        UNDERSTANDING LASER SCAN DATA:
        - self.scan_ranges contains distances to obstacles in meters
        - Each value represents distance at a different angle around the robot
        - Values less than self.stop_distance indicate a close obstacle
        
        CONTROLLING THE ROBOT (Twist message):
        - twist.linear.x: Forward/backward (positive = forward, negative = backward)
        - twist.angular.z: Rotation (positive = left, negative = right)
        - To stop: set twist.linear.x = 0.0 (you can keep angular.z to allow turning)
        
        EXTENSION (optional):
        - For obstacle avoidance, implement turning when obstacles are detected
        - Try to find clear paths by checking different parts of the scan data
        """

        # Filter out invalid readings (very small values, infinity, or NaN)
        valid_ranges = [r for r in self.scan_ranges if not math.isinf(r) and not math.isnan(r) and r > 0.01]
        
        # If no valid readings, assume no obstacles
        if not valid_ranges:
            self.cmd_vel_pub.publish(self.tele_twist)
            return
            
        # Find the closest obstacle in any direction (full 360° scan)
        obstacle_distance = min(valid_ranges)

        # !TODO: Implement your obstacle detection logic here!
        # Remember to use obstacle_distance and self.stop_distance in your implementation.
        min_index = self.scan_ranges.index(obstacle_distance)
        self.get_logger().info(f"Closest obstacle at index {min_index} with distance {obstacle_distance:.2f}m")

        # set angle towards goal
        angle_goal = self.calculate_goal_angle()
        self.get_logger().info(f"{angle_goal=}")
        
        # For now, just use the teleop command (unsafe - replace with your code)
        twist = self.tele_twist

        if self.pose.position.x >= 0:
            self.get_logger().info("############ STOPPED")
            twist.linear.x = 0.0

        #set steering angle
        if angle_goal >= 0.05:
            self.get_logger().info("########## steering")
            twist.angular.z = angle_goal
        else:
            self.get_logger().info("########## ANGLE REACHED")
            twist.angular.z = 0.0


        # Publish the velocity command
        self.cmd_vel_pub.publish(twist)

    def destroy_node(self):
        """Publish zero velocity when node is destroyed"""
        self.get_logger().info("Shutting down, stopping robot...")
        stop_twist = Twist() # Default Twist has all zeros
        self.cmd_vel_pub.publish(stop_twist)
        super().destroy_node() # Call the parent class's destroy_node


def main(args=None):
    rclpy.init(args=args)
    obstacle_detection = ObstacleDetection()
    try:
        rclpy.spin(obstacle_detection)
    except KeyboardInterrupt:
        obstacle_detection.get_logger().info('KeyboardInterrupt caught, allowing rclpy to shutdown.')
    finally:
        obstacle_detection.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
