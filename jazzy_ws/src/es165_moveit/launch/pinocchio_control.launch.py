from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

LOG_LEVEL = "ERROR"

def generate_launch_description():
    '''
    Launches the robot controlled by the Pinocchio differential-IK controller
    (controller_pinnochio) instead of MoveIt2 Servo.

    move_group and servo_node are intentionally NOT launched: the Pinocchio
    controller solves the IK itself and streams joint trajectories directly to
    arm_controller. robot_state_publisher still publishes the latched
    /robot_description that the controller builds its Pinocchio model from.

    Relies on /torque_input being published for robot control.
    Only handles attitude control, no linear transformations.
    '''

    # ROS2 Controller setup
    ros2_controllers_path = os.path.join(
        get_package_share_directory("es165_moveit_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    # Process es165.xacro directly into a robot_description. The Pinocchio
    # controller only needs the kinematic description on the latched
    # /robot_description topic; the MoveIt planning/servo stack is not launched.
    es165_xacro = PathJoinSubstitution([
        get_package_share_directory("es165_moveit_moveit_config"),
        "config",
        "es165.xacro",
    ])
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", es165_xacro]),
            value_type=str,
        )
    }

    # Declare use_sim_time argument, this allows the robot to interface with the Isaac Sim sim time publisher
    sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    # Declare log_level argument used for debugging, based on global variable LOG_LEVEL
    log_level = DeclareLaunchArgument(
        "log_level",
        default_value=LOG_LEVEL,
        description="ROS 2 logging level",
    )

    # Robot State Publisher, publishes the latched /robot_description that the
    # Pinocchio controller subscribes to (transient_local QoS)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    # ROS2 Control Node, runs the actual robot controller in sim
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ros2_controllers_path,
            robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
        arguments=[
            "--ros-args",
            "--log-level", LaunchConfiguration("log_level"),
        ],
    )

    # Spawn joint_state_broadcaster
    # Transforms /isaac_joint_states to /joint_states
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

    # Pinocchio differential-IK controller (replaces zero_g_servo + MoveIt Servo)
    pinocchio_controller = Node(
        package="es165_moveit",
        executable="controller_pinnochio",
        output="both",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # Spawn joint_state_broadcaster after ros2_control starts
    spawn_jsb_on_start = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control,
            on_start=[spawn_joint_state_broadcaster],
        )
    )

    # Spawn arm_controller after joint_state_broadcaster finishes loading
    spawn_arm_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_arm_controller],
        )
    )

    # Start the Pinocchio controller after arm_controller finishes loading
    spawn_controller_after_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_arm_controller,
            on_exit=[pinocchio_controller],
        )
    )

    torque_profile_pub = Node(
        package="es165_moveit",
        executable="publish_torque_profile",
        output="screen"
    )

    gui = Node(
        package="es165_moveit",
        executable="gui",
        output="screen"
    )

    return LaunchDescription([
        sim_time,
        log_level,
        robot_state_publisher,
        ros2_control,
        spawn_jsb_on_start,
        spawn_arm_after_jsb,
        spawn_controller_after_arm,
        torque_profile_pub,
        gui,
    ])
