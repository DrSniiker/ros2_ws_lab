#!/usr/bin/env python3
import threading
import time
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from math import sqrt, atan2, pi
from geometry_msgs.msg import Twist # !FIXME: Missing Twist msg type


class TurtleBot(Node):
    def __init__(self):
        super().__init__("turtlesim_goal")

        # !FIXME: Publisher for velocity commands
        self.velocity_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        # !FIXME: Subscriber for turtle's position
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.update_pose, 10
        )

        # Movement parameters
        self.max_linear_speed = 1.5
        self.min_linear_speed = 0.3
        self.angular_speed_factor = 4.0

        # Setup control timer (10Hz)
        self.timer = self.create_timer(0.1, self.controller_callback)

        # Setup turtle state
        self.pose = Pose()
        self.goal_pose = Pose()
        self.moving_to_goal = False
        self.distance_tolerance = 0.5
        self.waypoint_idx = 0


        # For position logging
        self.last_log_time = 0.0

        # Waypoint parameter
        self.declare_parameter('waypoints', '[[1.0, 1.0], [8.8, 7.0], [1.0, 3.5]]')

        self.waypoints = self.get_parameter('waypoints').value

        self.get_logger().info(f'Using waypoints: {self.waypoints}')

    def update_pose(self, data):
        """Store the turtle's current position"""
        self.pose = data

    def euclidean_distance(self):
        """Distance between current position and goal"""
        # !FIXME: Mathematical Error in Distance Calculation
        return sqrt(
            (self.goal_pose.x - self.pose.x) ** 2
            + (self.goal_pose.y - self.pose.y) ** 2
        )

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

    def calculate_steering_angle(self):
        """Angle toward goal"""
        return atan2(self.goal_pose.y - self.pose.y, self.goal_pose.x - self.pose.x)

    def calculate_angular_velocity(self):
        """Calculate rotational speed for steering"""
        # Angle difference between current heading and goal
        angle_diff = self.calculate_steering_angle() - self.pose.theta

        # Normalize angle between -pi and pi
        while angle_diff > pi:
            angle_diff -= 2 * pi
        while angle_diff < -pi:
            angle_diff += 2 * pi

        # Apply proportional control with stronger corrections for larger angles
        if abs(angle_diff) < 0.1:  # Small corrections
            return angle_diff * self.angular_speed_factor * 0.8
        elif abs(angle_diff) < 0.5:  # Medium corrections
            return angle_diff * self.angular_speed_factor
        else:  # Large corrections
            return angle_diff * self.angular_speed_factor * 1.2

    def controller_callback(self):
        """Waypoints stuff"""
        waypoints_list = eval(self.waypoints)
        waypoints_len = len(waypoints_list)

        self.get_logger().info(f"WAYPOINTS_IDX: {self.waypoint_idx}")
        self.get_logger().info(f"WAYPOINTS_LEN: {waypoints_len}")

        if self.waypoint_idx < waypoints_len:
            self.goal_pose.x = waypoints_list[self.waypoint_idx][0]
            self.goal_pose.y = waypoints_list[self.waypoint_idx][1]

        """Main control loop - called 10 times per second"""
        if not self.moving_to_goal:
            return

        # Log current position periodically (once per second)
        current_time = time.time()
        if current_time - self.last_log_time >= 1.0:
            self.get_logger().info(
                f"Current position: x={self.pose.x:.2f}, y={self.pose.y:.2f}, distance_to_goal={self.euclidean_distance():.2f}"
            )
            self.last_log_time = current_time

        # If we're close enough to the goal, stop moving
        self.get_logger().info(f"----------------------------")
        if self.euclidean_distance() < self.distance_tolerance:
            # Stop the turtle
            vel_msg = Twist()
            self.velocity_publisher.publish(vel_msg)
            self.waypoint_idx += 1

            self.get_logger().info(f"WAYPOINTS_IDX: {self.waypoint_idx}")
            self.get_logger().info(f"Waypoint reached")

            # Mark goal as reached
            if self.waypoint_idx >= waypoints_len:
                self.moving_to_goal = False
                self.get_logger().info(f"Goal reached! x={self.pose.x:.2f}, y={self.pose.y:.2f}")
                return

        # We need to keep moving toward the goal
        vel_msg = Twist()

        # Calculate forward speed
        linear_velocity = self.calculate_linear_velocity()

        # Adjust speed during turns
        angular_diff = abs(self.calculate_steering_angle() - self.pose.theta)
        while angular_diff > pi:
            angular_diff = abs(angular_diff - 2 * pi)

        # Slow down when turning sharply
        turn_factor = 1.0
        if angular_diff > 0.5:  # ~30 degrees
            turn_factor = max(0.4, 1.0 - angular_diff / pi)

        # Set the velocity commands
        vel_msg.linear.x = linear_velocity * turn_factor
        vel_msg.angular.z = self.calculate_angular_velocity()

        # Send command to the turtle
        self.velocity_publisher.publish(vel_msg)


def main():
    # Initialize ROS
    rclpy.init()
    turtlebot = TurtleBot()

    # Run ROS in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(turtlebot,))
    spin_thread.daemon = True
    spin_thread.start()

    try:
        while True:
            try:
                turtlebot.moving_to_goal = True

                # Wait for movement to complete
                while turtlebot.moving_to_goal:
                    time.sleep(0.5)
                
                if turtlebot.moving_to_goal == False:
                    break

            except ValueError:
                print("ERROR!")

    except KeyboardInterrupt:
        pass

    finally:
        turtlebot.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
