#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from example_interfaces.msg import Empty
import math

class SquareFormationController(Node):
    def __init__(self):
        super().__init__('square_formation_controller')
        
        # Declare parameters for robot namespaces
        self.declare_parameter('robot_namespaces', ['aire1', 'agua2', 'tierra3', 'fuego4'])
        self.declare_parameter('square_size', 1.0)  # meters
        self.declare_parameter('position_tolerance', 0.08)  # meters
        self.declare_parameter('angle_tolerance', 0.15)  # radians (~8.6 degrees)
        
        # Get parameters
        robot_list = self.get_parameter('robot_namespaces').value
        square_size = self.get_parameter('square_size').value
        self.position_tol = self.get_parameter('position_tolerance').value
        self.angle_tol = self.get_parameter('angle_tolerance').value
        
        if len(robot_list) != 4:
            self.get_logger().error('Exactly 4 robots required!')
            raise ValueError('Must provide exactly 4 robot namespaces')
        
        # Store robot names
        self.robot_names = robot_list
        
        # Define square positions (configurable size)
        half_size = square_size / 2.0
        self.square_positions = {
            robot_list[0]: {'x': half_size, 'y': half_size},      # Top-right
            robot_list[1]: {'x': half_size, 'y': -half_size},     # Bottom-right
            robot_list[2]: {'x': -half_size, 'y': -half_size},    # Bottom-left
            robot_list[3]: {'x': -half_size, 'y': half_size}      # Top-left
        }
        
        # Current positions from odometry
        self.current_positions = {name: None for name in robot_list}
        
        # Target positions for movement
        self.target_positions = {}
        
        # Movement state
        self.is_moving = False
        
        # Velocity publishers for each robot
        self.vel_publishers = {}
        for robot in robot_list:
            topic = f'/{robot}/cmd_vel'
            self.vel_publishers[robot] = self.create_publisher(Twist, topic, 10)
            self.get_logger().info(f'Publishing to {topic}')
        
        # Odometry subscribers for each robot
        self.odom_subscribers = {}
        for robot in robot_list:
            topic = f'/{robot}/odom'
            self.odom_subscribers[robot] = self.create_subscription(
                Odometry, 
                topic,
                lambda msg, r=robot: self.odom_callback(msg, r), 
                10
            )
            self.get_logger().info(f'Subscribing to {topic}')
        
        # Signal subscribers
        self.left_sub = self.create_subscription(Empty, '/left', self.left_callback, 10)
        self.right_sub = self.create_subscription(Empty, '/right', self.right_callback, 10)
        
        # Control loop timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('=== Square Formation Controller (Real Robots) ===')
        self.get_logger().info(f'Robots: {robot_list}')
        self.get_logger().info(f'Square size: {square_size}m x {square_size}m')
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
        robots = self.robot_names
        
        if direction == 'right':  # Clockwise
            self.target_positions = {
                robots[0]: self.square_positions[robots[1]].copy(),
                robots[1]: self.square_positions[robots[2]].copy(),
                robots[2]: self.square_positions[robots[3]].copy(),
                robots[3]: self.square_positions[robots[0]].copy()
            }
        else:  # Counter-clockwise (left)
            self.target_positions = {
                robots[0]: self.square_positions[robots[3]].copy(),
                robots[1]: self.square_positions[robots[0]].copy(),
                robots[2]: self.square_positions[robots[1]].copy(),
                robots[3]: self.square_positions[robots[2]].copy()
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
        
        for robot_name in self.robot_names:
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
            if distance > self.position_tol:
                all_reached = False
                
                # First align with target direction
                if abs(angle_diff) > self.angle_tol:
                    # Rotation only
                    cmd.angular.z = max(-0.5, min(0.5, 0.8 * angle_diff))
                else:
                    # Move forward once aligned (slower for real robots)
                    cmd.linear.x = max(0.0, min(0.15, distance * 0.4))
                    cmd.angular.z = max(-0.3, min(0.3, 0.4 * angle_diff))
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
