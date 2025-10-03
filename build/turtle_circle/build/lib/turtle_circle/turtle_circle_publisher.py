#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleGoal(Node):
    def __init__(self):
        super().__init__('turtle_circle')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.target_theta = math.radians(90)
        self.targets = [(5.5, 8.0), 'circle', (8.0, 8.0)]
        self.current_target_index = 0
        self.Kp_lin = 1.5
        self.Kp_ang = 6.0
        self.current_pose = None
        self.moving = True 
        self.client = self.create_client(Trigger, 'move_circle')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for move_circle service...')

    def pose_callback(self, msg: Pose):
        self.current_pose = msg
        if self.moving and self.current_target_index < len(self.targets):
            self.move()
        
    def move(self,x,y):
        target = self.targets[self.current_target_index]

        
        if target == 'circle':
            self.get_logger().info("Calling circle service...")
            req = Trigger.Request()
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info(future.result().message)
            self.current_target_index += 1  
            return

        dx = x- self.current_pose.x
        dy = y - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)

        desired_angle = math.atan2(dy, dx)
        angle_error = desired_angle - self.current_pose.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        orientation_error = self.target_theta - self.current_pose.theta
        orientation_error = math.atan2(math.sin(orientation_error), math.cos(orientation_error))

        twist = Twist()

        if distance > 0.1:
            twist.linear.x = self.Kp_lin * distance
            twist.angular.z = self.Kp_ang * angle_error
        elif abs(orientation_error) > 0.05:
            twist.linear.x = 0.0
            twist.angular.z = self.Kp_ang * orientation_error
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
