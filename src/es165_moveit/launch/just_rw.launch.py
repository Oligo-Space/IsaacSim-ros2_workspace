from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import LaunchConfiguration
from rclpy.impl.implementation_singleton import package
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch_param_builder import ParameterBuilder
import os
LOG_LEVEL = "ERROR"
UPDATE_RATE = 30 #Hz

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

    rw1_servo_params = {
        "moveit_servo": ParameterBuilder("es165_moveit_moveit_config")
        .yaml("config/rw1_servo_params.yaml")
        .to_dict()
    }

    rw2_servo_params = {
        "moveit_servo": ParameterBuilder("es165_moveit_moveit_config")
        .yaml("config/rw2_servo_params.yaml")
        .to_dict()
    }

    rw3_servo_params = {
        "moveit_servo": ParameterBuilder("es165_moveit_moveit_config")
        .yaml("config/rw3_servo_params.yaml")
        .to_dict()
    }

    rw4_servo_params = {
        "moveit_servo": ParameterBuilder("es165_moveit_moveit_config")
        .yaml("config/rw4_servo_params.yaml")
        .to_dict()
    }

    arm_servo_params = {
        "moveit_servo": ParameterBuilder("es165_moveit_moveit_config")
        .yaml("config/servo_params.yaml")
        .to_dict()
    }


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
    spawn_rw1_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rw1_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    spawn_rw2_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rw2_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    spawn_rw3_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rw3_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    spawn_rw4_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rw4_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

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

    arm_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            arm_servo_params,
            {"planning_group_name": "arm"},
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

    rw1_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="rw1_servo_node",
        parameters=[
            rw1_servo_params,
            {"planning_group_name":"rw1_ctrl"},
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

    rw2_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="rw2_servo_node",
        parameters=[
            rw2_servo_params,
            {"planning_group_name":"rw2_ctrl"},
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

    rw3_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="rw3_servo_node",
        parameters=[
            rw3_servo_params,
            {"planning_group_name":"rw3_ctrl"},
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

    rw4_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="rw4_servo_node",
        parameters=[
            rw4_servo_params,
            {"planning_group_name":"rw4_ctrl"},
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
    # Zero-G MoveIt Controller
    rw_controller = Node(
        package="es165_moveit",
        executable="control_rw",
        output="both",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    arm_servo = Node(
        package="es165_moveit",
        executable="zero_g_servo",
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

    # Spawn arm_controller after joint_state_broadcaster finishes loadin

    angle_intercepter = Node(
        package="es165_moveit",
        executable="angle_intercepter",
        output="both",
    )

    angular_acc_pub = Node(
        package="es165_moveit",
        executable="pub_angular_acc",
        output="both",
    )

    torque_publisher = Node(
        package="es165_moveit",
        executable="torque_from_rw",
        output="both",
    )

    rw_controller_action = TimerAction(
        period=1.0,
        actions=[rw_controller,],
    )

    return LaunchDescription([
        sim_time,
        log_level,
        robot_state_publisher,
        ros2_control,
        spawn_jsb_on_start,
        moveit_group,
        arm_servo_node,
        rw1_servo_node,
        rw2_servo_node,
        rw3_servo_node,
        rw4_servo_node,
        arm_servo,
        spawn_rw1_controller,
        spawn_rw2_controller,
        spawn_rw3_controller,
        spawn_rw4_controller,
        spawn_arm_controller,
        angle_intercepter,
        angular_acc_pub,
        # torque_publisher,
        rw_controller_action,
        # rviz2,
    ])
