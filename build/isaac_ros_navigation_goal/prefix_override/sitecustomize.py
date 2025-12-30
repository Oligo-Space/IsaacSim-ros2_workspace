import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/IsaacSim-ros2_workspace/install/isaac_ros_navigation_goal'
