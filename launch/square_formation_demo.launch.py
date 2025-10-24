#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Get package directories
    pkg_multi_tb = get_package_share_directory('multi_turtlebot_sim')
    
    # World file path
    world_path = os.path.join(pkg_multi_tb, 'worlds', 'empty_world.world')
    
    # Start Gazebo server
    start_gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_path, '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # Start Gazebo client
    start_gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    
    # Connect robot2, robot3, robot4 odom frames to robot1/odom
    # This makes robot1/odom the root reference frame
    robot1_to_robot2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot1_to_robot2',
        arguments=['0', '0', '0', '0', '0', '0', 'robot1/odom', 'robot2/odom'],
        output='screen'
    )
    
    robot1_to_robot3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot1_to_robot3',
        arguments=['0', '0', '0', '0', '0', '0', 'robot1/odom', 'robot3/odom'],
        output='screen'
    )
    
    robot1_to_robot4 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot1_to_robot4',
        arguments=['0', '0', '0', '0', '0', '0', 'robot1/odom', 'robot4/odom'],
        output='screen'
    )
    
    # Spawn robots
    spawn_robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_multi_tb, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'robot_prefix': 'robot1',
            'x_pose': '1.0',
            'y_pose': '1.0',
            'use_sim_time': 'true'
        }.items()
    )
    
    spawn_robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_multi_tb, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'robot_prefix': 'robot2',
            'x_pose': '1.0',
            'y_pose': '-1.0',
            'use_sim_time': 'true'
        }.items()
    )
    
    spawn_robot3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_multi_tb, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'robot_prefix': 'robot3',
            'x_pose': '-1.0',
            'y_pose': '-1.0',
            'use_sim_time': 'true'
        }.items()
    )
    
    spawn_robot4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_multi_tb, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'robot_prefix': 'robot4',
            'x_pose': '-1.0',
            'y_pose': '1.0',
            'use_sim_time': 'true'
        }.items()
    )
    
    # Square formation controller node
    formation_controller = Node(
        package='square_formation_behavior',
        executable='square_formation_controller',
        name='formation_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        start_gazebo_server,
        start_gazebo_client,
        robot1_to_robot2,
        robot1_to_robot3,
        robot1_to_robot4,
        spawn_robot1,
        spawn_robot2,
        spawn_robot3,
        spawn_robot4,
        formation_controller
    ])