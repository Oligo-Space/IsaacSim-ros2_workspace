from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launches RViz with:
    - PointCloud subscription to /pcd_points
    - Robot URDF from es165_blender.urdf
    - Scene mesh visualization
    """
    
    # Get package share directory
    package_share_dir = get_package_share_directory('es165_moveit')
    
    # Declare use_sim_time argument
    sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true",
    )
    
    # Declare log_level argument
    log_level = DeclareLaunchArgument(
        "log_level",
        default_value="INFO",
        description="ROS 2 logging level",
    )
    
    # Path to URDF file
    urdf_path = os.path.join(
        package_share_dir,
        'urdf',
        'es165_blender_absolute_paths.urdf'
    )
    
    # Read URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    
    # Robot State Publisher - publishes robot description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )
    
    # Scene Mesh Publisher - publishes the scene mesh as a Marker
    scene_mesh_publisher = Node(
        package='es165_moveit',
        executable='publish_scene_mesh',
        name='scene_mesh_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )
    
    # RViz config file path (create a basic one if it doesn't exist)
    rviz_config_path = os.path.join(
        package_share_dir,
        "rviz",
        "pcd_visualization.rviz"
    )
    
    # If config doesn't exist, use default
    if not os.path.exists(rviz_config_path):
        rviz_config_path = os.path.join(
            package_share_dir,
            "rviz",
            "default.rviz"
        )
    
    # RViz
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': robot_desc},
        ],
        # arguments=["-d", rviz_config_path, "--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    pcd_node = Node(
        package='es165_moveit',
        executable='visualize_pcd',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )
    
    return LaunchDescription([
        sim_time,
        log_level,
        robot_state_publisher,
        # scene_mesh_publisher,
        pcd_node,
        rviz2,
    ])
