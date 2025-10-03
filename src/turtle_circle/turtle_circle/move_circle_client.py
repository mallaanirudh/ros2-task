#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger

class MoveCircleClient(Node):
    def __init__(self):
        super().__init__('move_circle_client')
        self.client = self.create_client(Trigger, 'move_circle')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for move_circle service...")
        self.req = Trigger.Request()

    def send_request(self):
        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    node = MoveCircleClient()
    response = node.send_request()
    node.get_logger().info(response.message)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
