#! /usr/bin/env python3
from action_msgs.msg import GoalStatus
from example_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class MinimalActionClient(Node):
    def __init__(self):
        super().__init__("minimal_action_client")
        self._action_client = ActionClient(self, Fibonacci, "fibonacci")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info(
            "Received feedback: {0}".format(list(feedback.feedback.sequence))
        )

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                "Goal succeeded! Result: {0}".format(list(result.sequence))
            )
        else:
            self.get_logger().info("Goal failed with status: {0}".format(status))

        # Shutdown after receiving a result
        rclpy.shutdown()

    def send_goal(self):
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()

        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10

        self.get_logger().info("Sending goal request...")

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)


def main(args=None):
    try:
        rclpy.init(args=args)
        action_client = MinimalActionClient()

        action_client.send_goal()

        rclpy.spin(action_client)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Destroy the node explicitly
        if 'action_client' in locals():
            action_client.destroy_node()
        # rclpy.shutdown() is called in the get_result_callback


if __name__ == "__main__":
    main()
