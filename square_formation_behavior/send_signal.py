#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Empty
import sys

class SignalPublisher(Node):
    def __init__(self):
        super().__init__('signal_publisher')
        self.left_pub = self.create_publisher(Empty, '/left', 10)
        self.right_pub = self.create_publisher(Empty, '/right', 10)
    
    def send_left(self):
        msg = Empty()
        self.left_pub.publish(msg)
        self.get_logger().info('Sent LEFT signal')
    
    def send_right(self):
        msg = Empty()
        self.right_pub.publish(msg)
        self.get_logger().info('Sent RIGHT signal')

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: ros2 run square_formation_behavior send_signal [left|right]")
        return
    
    direction = sys.argv[1].lower()
    
    if direction not in ['left', 'right']:
        print("Direction must be 'left' or 'right'")
        return
    
    publisher = SignalPublisher()
    
    # Give time for publishers to establish
    import time
    time.sleep(0.5)
    
    if direction == 'left':
        publisher.send_left()
    else:
        publisher.send_right()
    
    # Keep node alive briefly to ensure message is sent
    rclpy.spin_once(publisher, timeout_sec=1.0)
    
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
