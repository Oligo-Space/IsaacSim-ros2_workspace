from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os
import xacro

LOG_LEVEL = "INFO"

def generate_launch_description():

    ros2_controllers_path = os.path.join(
        get_package_share_directory("es165_moveit_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    moveit_config = (
        MoveItConfigsBuilder("es165_moveit")
        .robot_description(
            file_path="config/es165.xacro",
        )
        .robot_description_semantic(file_path="config/es165d.srdf")
        .planning_scene_monitor(
            publish_robot_description=False, publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path="config/controller.yaml")
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .to_moveit_configs()
    )

    # Declare use_sim_time argument
    sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true",
    )

    # Declare log_level argument
    log_level = DeclareLaunchArgument(
        "log_level",
        default_value=LOG_LEVEL,
        description="ROS 2 logging level",
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description, {'use_sim_time': True}],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    # MoveIt move_group
    moveit_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    # ROS2 Control Node
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ros2_controllers_path,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
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
            "--ros-args",
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
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
            "--ros-args",
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
    )

    # RViz
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
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
        period=10.0,  # Wait 10 seconds for ros2_control_node to fully initialize
        actions=[spawn_joint_state_broadcaster],
    )
    
    delayed_arm_controller = TimerAction(
        period=12.0,  # Wait 12 seconds, start after joint_state_broadcaster
        actions=[spawn_arm_controller],
    )
    
    delayed_move_group = TimerAction(
        period=15.0,  # Wait 15 seconds for controllers to be ready
        actions=[moveit_group],
    )
    
    delayed_rviz = TimerAction(
        period=17.0,  # Wait 17 seconds for move_group to be ready
        actions=[rviz2],
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
        sim_time,
        log_level,
        robot_state_publisher,
        ros2_control_handler,
        joint_state_broadcaster_handler,
        arm_controller_handler,
        move_group_handler,
        rviz_handler,
    ])
