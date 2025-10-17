#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from example_interfaces.msg import Empty
import math
import time

class SquareFormationController(Node):
    def __init__(self):
        super().__init__('square_formation_controller')
        
        # Define square positions (2m x 2m square)
        self.square_positions = {
            'robot1': {'x': 1.0, 'y': 1.0},   # Top-right
            'robot2': {'x': 1.0, 'y': -1.0},  # Bottom-right
            'robot3': {'x': -1.0, 'y': -1.0}, # Bottom-left
            'robot4': {'x': -1.0, 'y': 1.0}   # Top-left
        }
        
        # Current positions from odometry
        self.current_positions = {
            'robot1': None,
            'robot2': None,
            'robot3': None,
            'robot4': None
        }
        
        # Target positions for movement
        self.target_positions = {}
        
        # Movement state
        self.is_moving = False
        
        # Velocity publishers for each robot
        self.vel_publishers = {
            'robot1': self.create_publisher(Twist, '/robot1/cmd_vel', 10),
            'robot2': self.create_publisher(Twist, '/robot2/cmd_vel', 10),
            'robot3': self.create_publisher(Twist, '/robot3/cmd_vel', 10),
            'robot4': self.create_publisher(Twist, '/robot4/cmd_vel', 10)
        }
        
        # Odometry subscribers for each robot
        self.odom_subscribers = {
            'robot1': self.create_subscription(Odometry, '/robot1/odom', 
                                              lambda msg: self.odom_callback(msg, 'robot1'), 10),
            'robot2': self.create_subscription(Odometry, '/robot2/odom',
                                              lambda msg: self.odom_callback(msg, 'robot2'), 10),
            'robot3': self.create_subscription(Odometry, '/robot3/odom',
                                              lambda msg: self.odom_callback(msg, 'robot3'), 10),
            'robot4': self.create_subscription(Odometry, '/robot4/odom',
                                              lambda msg: self.odom_callback(msg, 'robot4'), 10)
        }
        
        # Signal subscribers
        self.left_sub = self.create_subscription(Empty, '/left', self.left_callback, 10)
        self.right_sub = self.create_subscription(Empty, '/right', self.right_callback, 10)
        
        # Control loop timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('Square Formation Controller initialized')
        self.get_logger().info('Waiting for /left or /right signals...')
    
    def odom_callback(self, msg, robot_name):
        """Update current position from odometry"""
        self.current_positions[robot_name] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        }
    
    def get_yaw_from_quaternion(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def left_callback(self, msg):
        """Handle left rotation signal (counter-clockwise)"""
        if self.is_moving:
            self.get_logger().warn('Already moving, ignoring signal')
            return
        
        self.get_logger().info('Received LEFT signal - rotating counter-clockwise')
        self.rotate_formation(direction='left')
    
    def right_callback(self, msg):
        """Handle right rotation signal (clockwise)"""
        if self.is_moving:
            self.get_logger().warn('Already moving, ignoring signal')
            return
        
        self.get_logger().info('Received RIGHT signal - rotating clockwise')
        self.rotate_formation(direction='right')
    
    def rotate_formation(self, direction):
        """Calculate target positions for rotation"""
        if direction == 'right':  # Clockwise
            # robot1 -> robot2's position
            # robot2 -> robot3's position
            # robot3 -> robot4's position
            # robot4 -> robot1's position
            self.target_positions = {
                'robot1': self.square_positions['robot2'].copy(),
                'robot2': self.square_positions['robot3'].copy(),
                'robot3': self.square_positions['robot4'].copy(),
                'robot4': self.square_positions['robot1'].copy()
            }
        else:  # Counter-clockwise (left)
            # robot1 -> robot4's position
            # robot4 -> robot3's position
            # robot3 -> robot2's position
            # robot2 -> robot1's position
            self.target_positions = {
                'robot1': self.square_positions['robot4'].copy(),
                'robot2': self.square_positions['robot1'].copy(),
                'robot3': self.square_positions['robot2'].copy(),
                'robot4': self.square_positions['robot3'].copy()
            }
        
        # Update square positions for next rotation
        self.square_positions = self.target_positions.copy()
        self.is_moving = True
        self.get_logger().info(f'Starting {direction} rotation')
    
    def control_loop(self):
        """Main control loop - moves robots to target positions"""
        if not self.is_moving:
            return
        
        # Check if all robots have valid positions
        if any(pos is None for pos in self.current_positions.values()):
            return
        
        all_reached = True
        
        for robot_name in ['robot1', 'robot2', 'robot3', 'robot4']:
            current = self.current_positions[robot_name]
            target = self.target_positions[robot_name]
            
            # Calculate distance to target
            dx = target['x'] - current['x']
            dy = target['y'] - current['y']
            distance = math.sqrt(dx**2 + dy**2)
            
            # Calculate target angle
            target_angle = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(target_angle - current['theta'])
            
            # Create velocity command
            cmd = Twist()
            
            # Position tolerance
            if distance > 0.05:  # 5cm tolerance
                all_reached = False
                
                # First align with target direction
                if abs(angle_diff) > 0.1:  # ~6 degrees tolerance
                    cmd.angular.z = 1.0 * angle_diff  # P controller for rotation
                else:
                    # Move forward once aligned
                    cmd.linear.x = min(0.2, distance * 0.5)  # P controller for linear motion
                    cmd.angular.z = 0.5 * angle_diff  # Minor corrections
            else:
                # Stop at target
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            
            # Publish velocity command
            self.vel_publishers[robot_name].publish(cmd)
        
        # Check if all robots reached their targets
        if all_reached:
            self.get_logger().info('All robots reached target positions!')
            self.stop_all_robots()
            self.is_moving = False
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def stop_all_robots(self):
        """Send stop command to all robots"""
        stop_cmd = Twist()
        for pub in self.vel_publishers.values():
            pub.publish(stop_cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = SquareFormationController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
