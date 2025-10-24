#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Robot namespaces (modify these to match your actual robots)
    # Using only 2 robots for now: tierra3 (205) and fuego4 (206)
    robot_namespaces = ['tierra3', 'fuego4', 'aire1', 'agua2']  # Update when you add more robots
    
    # Square formation controller node
    formation_controller = Node(
        package='square_formation_behavior',
        executable='square_formation_controller',
        name='formation_controller',
        output='screen',
        parameters=[{
            'robot_namespaces': robot_namespaces,
            'square_size': 1.0,  # 1m x 1m square
            'position_tolerance': 0.10,
            'angle_tolerance': 0.20,
            'min_robots': 2
        }]
    )
    
    # Static transforms: Connect all robot odom frames to tierra3/odom (first robot as root)
    # This creates a common reference frame for all robots
    root_frame = f'{robot_namespaces[0]}/odom'
    
    static_transforms = []
    for robot in robot_namespaces[1:]:
        static_transforms.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_tf_{root_frame.replace("/", "_")}_to_{robot.replace("/", "_")}_odom',
                arguments=['0', '0', '0', '0', '0', '0', root_frame, f'{robot}/odom'],
                output='screen'
            )
        )
    
    return LaunchDescription([
        formation_controller,
        *static_transforms
    ])