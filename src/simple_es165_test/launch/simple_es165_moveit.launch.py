#!/usr/bin/env python3
"""
Launch MoveIt for simple single-joint ES165 test robot.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package paths
    ros2_controllers_path = os.path.join(
        get_package_share_directory("simple_es165_test_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    moveit_config = (
        MoveItConfigsBuilder("simple_es165_test")
        .robot_description(
            file_path="config/simple_es165.xacro",
        )
        .robot_description_semantic(file_path="config/simple_es165.srdf")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path="config/controller.yaml")
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .to_moveit_configs()
    )

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    # ROS2 Control Node (with mock hardware)
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ros2_controllers_path,
            moveit_config.robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    # Spawn joint_state_broadcaster (required for publishing joint states)
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
            "--service-call-timeout",
            "30",
        ],
        output="screen",
    )

    # Spawn arm_controller
    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
            "--service-call-timeout",
            "30",
        ],
        output="screen",
    )

    # MoveIt move_group
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            moveit_config.robot_description_kinematics,  # Explicitly add kinematics
        ],
    )

    # RViz (optional)
    rviz_config_path = os.path.join(
        moveit_config.package_path,
        "config",
        "moveit.rviz"
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,  # Add kinematics for RViz
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Event handlers to ensure proper startup sequence
    # Start ros2_control after robot_state_publisher is ready
    ros2_control_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[ros2_control],
        )
    )
    
    # Spawn controllers with delay after ros2_control starts
    # Using TimerAction to give ros2_control_node time to initialize services
    delayed_joint_state_broadcaster = TimerAction(
        period=2.0,  # Wait 2 seconds for ros2_control_node to initialize
        actions=[spawn_joint_state_broadcaster],
    )
    
    delayed_arm_controller = TimerAction(
        period=3.0,  # Wait 3 seconds, start after joint_state_broadcaster
        actions=[spawn_arm_controller],
    )
    
    delayed_move_group = TimerAction(
        period=5.0,  # Wait 5 seconds for controllers to be ready
        actions=[move_group],
    )
    
    delayed_rviz = TimerAction(
        period=6.0,  # Wait 6 seconds for move_group to be ready
        actions=[rviz],
    )
    
    joint_state_broadcaster_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control,
            on_start=[delayed_joint_state_broadcaster],
        )
    )

    arm_controller_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control,
            on_start=[delayed_arm_controller],
        )
    )

    move_group_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control,
            on_start=[delayed_move_group],
        )
    )

    rviz_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control,
            on_start=[delayed_rviz],
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
        ros2_control_handler,  # Start ros2_control after robot_state_publisher
        joint_state_broadcaster_handler,  # Spawn joint_state_broadcaster after ros2_control
        arm_controller_handler,  # Spawn arm_controller after ros2_control
        move_group_handler,  # Start move_group after controllers are ready
        rviz_handler,  # Start RViz after move_group is ready
    ])

