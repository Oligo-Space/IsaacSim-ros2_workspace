from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import LaunchConfiguration
from rclpy.impl.implementation_singleton import package
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch_param_builder import ParameterBuilder
import os

LOG_LEVEL = "ERROR"

def generate_launch_description():
    '''
    Launches the robot to interface with MoveIt2 Servo
    Relies on /torque_input being published for robot control
    Only handles attitude control, no linear transformations
    '''

    # ROS2 Controller setup
    ros2_controllers_path = os.path.join(
        get_package_share_directory("es165_moveit_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="es165_arm",package_name="es165_moveit_moveit_config") #This references the es165_moveit_moveit_config package, local files there
        .robot_description(
            file_path="config/es165.xacro",
        )
        .robot_description_semantic(file_path="config/es165d.srdf")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        ) #Keep track of robot descriptions through the moveit config
        .trajectory_execution(file_path="config/controller.yaml")
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .to_moveit_configs()
    )

    servo_params = {
        "moveit_servo": ParameterBuilder("es165_moveit_moveit_config")
        .yaml("config/servo_params.yaml")
        .to_dict()
    }
 
    planning_group_name = {"planning_group_name": "arm"}

    # Declare use_sim_time argument, this allows the robot to interface with the Isaac Sim sim time publisher
    sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true",
    )

    # Declare log_level argument used for debugging, based on global variable LOG_LEVEL
    log_level = DeclareLaunchArgument(
        "log_level",
        default_value=LOG_LEVEL,
        description="ROS 2 logging level",
    )

    # Robot State Publisher, publishes robot description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            moveit_config.robot_description, #use the already processed robot description to avoid publishing twice
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    # ROS2 Control Node, runs the actual robot controller in sim
    # Future use would be a custom controller with the actual control scheme utilized in dx100
    # For now, this is just a simple controller that uses PID to control position
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ros2_controllers_path, 
            moveit_config.robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
        arguments=[
            "--ros-args",
            "--log-level", LaunchConfiguration("log_level"),
        ],
    )

    # Spawn joint_state_broadcaster
    # Transforms /isaac_joint_states to /joint_states to communicate with MoveIt
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
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
        ],
        output="screen",
    )

    # MoveIt move_group
    moveit_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            servo_params,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        arguments=[
            "--ros-args",
            "--log-level", LaunchConfiguration("log_level"),
        ],
        output="screen",
    )

    # RViz config file path
    rviz_config_path = os.path.join(
        get_package_share_directory("es165_moveit"),
        "rviz",
        "default.rviz",
    )

    # Zero-G MoveIt Controller
    zero_g_controller = Node(
        package="es165_moveit",
        executable="zero_g_servo",
        output="both",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # RViz
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        arguments=["-d", rviz_config_path, "--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    # Spawn joint_state_broadcaster after ros2_control starts
    spawn_jsb_on_start = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control,
            on_start=[spawn_joint_state_broadcaster],
        )
    )

    # Spawn arm_controller after joint_state_broadcaster finishes loading
    # Ensures that both controllers don't try to spawn at the same time
    spawn_arm_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_arm_controller],
        )
    )

    # Spawn zero_g_controller after arm_controller finishes loading
    spawn_zero_g_after_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_arm_controller,
            on_exit=[zero_g_controller],
        )
    )



    # Test Control Node
    test_control = Node(
        package="es165_moveit",
        executable="torque_publisher",
        output="both",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    torque_pub = RegisterEventHandler(
        OnProcessStart(
            target_action=zero_g_controller,
            on_start=[test_control],
        )
    )

    return LaunchDescription([
        sim_time,
        log_level,
        robot_state_publisher,
        ros2_control,
        spawn_jsb_on_start,
        spawn_arm_after_jsb,
        moveit_group,
        servo_node,
        spawn_zero_g_after_arm,
        torque_pub,
        # rviz2,
    ])
