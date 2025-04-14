#! /usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import random

class Talker(Node):
    def __init__(self):
        super().__init__("martin")
        #self.i = 0 
        self.pub = self.create_publisher(String, "topic_1", 10)
        timer_period = 1.0
        self.tmr = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = "Jag gick i skolan i {0} år!".format(random.randint(0,100))
        #self.i += 1
        self.get_logger().info('Publishing: "{0}"'.format(msg.data))
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = Talker()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
