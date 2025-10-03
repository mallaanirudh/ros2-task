#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from example_interfaces.srv import Trigger
import time

class MoveCircleServer(Node):
    def __init__(self):
        super().__init__('move_circle_server')
        self.srv = self.create_service(Trigger, 'move_circle', self.handle_move_circle)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info("MoveCircle Server is ready!")

    def handle_move_circle(self, request, response):
        self.get_logger().info("Received request: Moving in a circle")

        twist = Twist()
        twist.linear.x = 2.7
        twist.angular.z = 1.0
        duration = 6.0
        rate = 10
        interval = 1.0 / rate

        for _ in range(int(duration * rate)):
            self.pub.publish(twist)
            time.sleep(interval)

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)

        response.success = True
        response.message = "Circle completed"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MoveCircleServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
