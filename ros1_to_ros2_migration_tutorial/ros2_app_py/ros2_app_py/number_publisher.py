#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")
        self.number_ = 4

        self.pub_ = self.create_publisher(Int64, "number", 10)

        self.number_timer_ = self.create_timer(0.5, self.publish_number)
        self.get_logger().info("Number publisher has been started!!!")

    def print_number(self):
        self.get_logger().info(str(self.number_))

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)  # init ROS2 communication (no node created here)

    ## node = Node('number_publisher')  # ROS2 node creation - basic way

    node = NumberPublisherNode()  # ROS2 node creation - OOP way

    rclpy.spin(node)  # note that we pass to spin() the name of the node we want to spin --> in ROS2 a script can contain multiple nodes

    rclpy.shutdown()  # shutdown ROS2 communication

if __name__ == '__main__':
    main()