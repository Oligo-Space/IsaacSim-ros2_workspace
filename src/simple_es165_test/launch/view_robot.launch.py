#!/usr/bin/env python3
"""
Simple launch file to view the single-joint robot in RViz.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Get package share directory
    package_share = get_package_share_directory("simple_es165_test")
    
    # Define paths
    xacro_path = os.path.join(package_share, "config", "simple_es165.xacro")
    controller_path = os.path.join(package_share, "config", "joint_velocities.yaml")
    
    # Process xacro file to get robot description
    robot_description_config = xacro.process_file(
        xacro_path,
        mappings={'joint_velocities_file': controller_path}
    )
    robot_description = robot_description_config.toxml()
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )
    
    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])

