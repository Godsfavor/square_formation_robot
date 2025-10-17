#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Robot namespaces (modify these to match your actual robots)
    robot_namespaces = ['tb_202', 'tb_204', 'tb_205', 'tb_206']
    
    # Square formation controller node
    formation_controller = Node(
        package='square_formation_behavior',
        executable='square_formation_controller',
        name='formation_controller',
        output='screen',
        parameters=[{
            'robot_namespaces': robot_namespaces,
            'square_size': 1.0,  # 1m x 1m square
            'position_tolerance': 0.08,
            'angle_tolerance': 0.15
        }]
    )
    
    # Static transforms: Connect all robot odom frames to tb_202/odom (first robot as root)
    # This creates a common reference frame for all robots
    root_frame = f'{robot_namespaces[0]}/odom'
    
    static_transforms = []
    for robot in robot_namespaces[1:]:
        static_transforms.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'{root_frame}_to_{robot}_odom',
                arguments=['0', '0', '0', '0', '0', '0', root_frame, f'{robot}/odom'],
                output='screen'
            )
        )
    
    return LaunchDescription([
        formation_controller,
        *static_transforms
    ])
