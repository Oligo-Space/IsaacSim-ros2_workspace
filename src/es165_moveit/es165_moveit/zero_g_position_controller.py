import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, String
import sensor_msgs
from tf2_ros import Buffer
from tf2_msgs.msg import TFMessage
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from threading import Lock
from ament_index_python.packages import get_package_share_directory

from curobo.cuda_robot_model.cuda_robot_model import CudaRobotGeneratorConfig, CudaRobotGenerator
from curobo.types.base import TensorDeviceType
from curobo.util_file import get_robot_configs_path, join_path, load_yaml
from curobo.types.robot import RobotConfig
from curobo.types.math import Pose
from curobo.wrap.model.robot_world import RobotWorld, RobotWorldConfig
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
from curobo.geom.types import WorldConfig, Mesh, Cuboid
from curobo.geom.sdf.world import CollisionQueryBuffer
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.types.state import JointState

import torch
import numpy as np

class ZeroGPositionController(Node):
    def __init__(self):
        super().__init__("zero_g_position_controller")
        self.get_logger().info("ZeroGPositionController initialized")

#Subscribers
        self.create_subscription(TFMessage, '/tf_sim', self.update_pose, 10)
        self.create_subscription(Float32MultiArray, '/torque_input', self.update_goal, 10)
        self.create_subscription(sensor_msgs.msg.JointState, '/joint_states', self.update_state, 10)
        self.create_subscription(Bool, '/reset', self.reset_position, 10)
        self.create_subscription(Float32MultiArray, "/update_position", self.reset_position,10)
        # self.create_subscription(String, '/measured_torques', self.calc_ee_from_efforts, 10)
        #Publishers
        self.joint_trajectory_publisher = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.init_position_publisher = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)

        #parameters
        self.declare_parameter("hz", 30)
        self.dt = 1/self.get_parameter("hz").value

        self.joint_names = ["joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"]


        # Globals
        self.current_joint_state = None
        self.current_state = None
        self.last_t = None
        self.last_planning_t = None
        # self.start_pos_deg = np.rad2deg(np.array([ 0.7563, -1.2798,  1.4244, -5.6094,  0.2952,  6.2832]))
        self.start_pos_deg = np.array([-11,-61,19,-8,-46,109])
        self.move_lock = Lock()
        self.is_enabled = True
        self.received_js = False

        self.ee_intertia = np.array(
            [
                [0.669, 0.0, 0.0],
                [0.0, 2.609, 0.0],
                [0.0, 0.0, 3.261]
            ]
        )
        # self.ee_intertia = np.eye(3)
        robot_file = get_package_share_directory("es165_moveit") + "/urdf/robot_description.yaml"
        world_file = get_package_share_directory("es165_moveit") + "/urdf/ground_plane_only.yaml"
        urdf = get_package_share_directory("es165_moveit") + "/urdf/es165_isaac.urdf"
        self.min_height = 0.2
        self.board_width = 0.51
        self.board_length = 1.17
        self.tensor_args = TensorDeviceType()

        # tensor conversionstensor
        self.tensor_to_array = lambda a: torch.from_numpy(np.asarray(a, dtype=np.float32)).to(self.tensor_args.device)

        world_config = WorldConfig(
            cuboid=[Cuboid(
                name="ground_plane",
                pose=[0.0,0.0,-0.05,0.0,0.0,0.0,1.0],
                dims=[10.0,10.0,0.01]
            )]
        )

        kin_cfg = load_yaml(robot_file)
        base_link = kin_cfg["robot_cfg"]["kinematics"]["base_link"]
        ee_link = kin_cfg["robot_cfg"]["kinematics"]["ee_link"]
        ik_cfg = RobotConfig.from_basic(
            urdf,
            base_link,
            ee_link,
            self.tensor_args
        )   
        # self.ik_solver = IKSolverConfig.load_from_robot_config(
        #     ik_cfg,
        #     world_config,
        #     rotation_threshold=0.05,
        #     position_threshold=0.02,
        #     num_seeds=20,
        #     self_collision_check=True,
        #     self_collision_opt=True,
        #     tensor_args=self.tensor_args,
        #     use_cuda_graph=True
        # )

        motion_gen_config = MotionGenConfig.load_from_robot_config(
            ik_cfg,
            world_model = world_config,
            interpolation_dt=self.dt/32,
            use_cuda_graph=True,
            num_trajopt_seeds=6,
            num_ik_seeds=6,
            position_threshold=0.02,
            rotation_threshold=0.1,
        )
        self.motion_gen = MotionGen(motion_gen_config)



        self.init_position()

    def reset_position(self,msg):
        if isinstance(msg, Float32MultiArray):
            self.init_position(msg.data)
        if msg.data:
            self.init_position()

    def init_position(self,pos=None):
        '''
        Initializes robot to non-zero state
        '''
        if pos is not None:
            self.start_pos_deg = np.rad2deg(pos)

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.joint_names = ["joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"]
        msg.points = [
            JointTrajectoryPoint(positions=list(np.deg2rad(self.start_pos_deg)), velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=Duration(sec=0, nanosec=100000000))
        ]
        self.init_position_publisher.publish(msg)
    
    def update_state(self,msg):
        self.current_state = msg
        self.received_js = True

    def update_pose(self,msg):
        '''
        Pose of the end effector in the inertial frame
        '''


        transform = msg.transforms[0] #Only one arm so take first transform in tree
        t = transform.header.stamp.nanosec/1e9 #Use timestamp of message to ensure accurate sync
        x,y,z = (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z)
        q = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
        roll, pitch, yaw = np.array(euler_from_quaternion(q))
        
        #Update pose
        if self.last_t is not None and self.current_joint_state is not None:
            dt = t - self.last_t
            wx, wy, wz = (np.array([roll,pitch,yaw]) - self.current_joint_state[3:6]) / (dt+1e-12)
            self.current_joint_state = np.array([x,y,z,roll,pitch,yaw,wx,wy,wz])
            self.last_t = t
        else:
            self.current_joint_state = np.array([x,y,z,roll,pitch,yaw,0,0,0])
            self.last_t = t

    def update_goal(self, msg):
        with self.move_lock:
            if not self.is_enabled or not self.received_js:
                self.get_logger().error("Servo is not enabled, skipping move request")
                return
                
            torque = np.array(msg.data,dtype=np.float32) #tx, ty, tz in the end effector body frame

            # self.get_logger().info(f"Current joint state: {self.current_joint_state}")
            self.get_logger().info(f"Current Pose: {self.current_joint_state[:6]}")

            w = np.linalg.inv(self.ee_intertia)@torque*self.dt + self.current_joint_state[6:9]
            p = w*self.dt + self.current_joint_state[3:6]
            p = (p + np.pi) % (2*np.pi) - np.pi
            q = quaternion_from_euler(p[0], p[1], p[2])
            self.get_logger().info(f"Goal Pose: {q}")

            curr_position = self.tensor_to_array(self.current_joint_state[0:3])
            goal_q = q
            

            goal_pose = Pose.from_list([curr_position[0], curr_position[1], curr_position[2], goal_q[0], goal_q[1], goal_q[2], goal_q[3]])
            start_state = JointState.from_numpy(
                position=np.array(self.current_state.position, dtype=np.float32).reshape(1,6),
                velocity=np.array(self.current_state.velocity, dtype=np.float32).reshape(1,6),
                joint_names=self.current_state.name
            )
            # result = self.motion_gen.solve_ik(goal_pose)
            result = self.motion_gen.plan_single(start_state, goal_pose)
            if result.success:
                self.get_logger().info(f"Result: {result.solve_time}")
                # accelerations = list(p[12:18])

                traj = result.get_interpolated_plan()

                traj_msg = JointTrajectory()
                traj_msg.header.stamp = rclpy.time.Time().to_msg()
                traj_msg.header.frame_id = "base_link"
                traj_msg.joint_names = self.joint_names
                traj_msg.points = []
                # self.get_logger().info(f"Trajectory points: {len(traj.position)}")
                # timestep = int(self.dt*1e9/len(traj.position))
                dt_step = self.dt/len(traj.position)
                
                for i, pt in enumerate(zip(traj.position, traj.velocity, traj.acceleration)):
                    self.get_logger().info(f"Trajectory Point: {pt[0]}")
                    self.get_logger().info(f"{int(i*dt_step*1e9)}")
                    # t = Duration(sec=0, nanosec=int((i+1)*timestep))
                    p = JointTrajectoryPoint(
                        positions = pt[0].tolist(),
                        velocities = pt[1].tolist(),
                        accelerations = pt[2].tolist(),
                        time_from_start = Duration(sec=0, nanosec=int(i*dt_step*1e9))
                    )
                    traj_msg.points.append(p)
                self.joint_trajectory_publisher.publish(traj_msg)



            

def main(args=None):
    rclpy.init(args=args)
    node = ZeroGPositionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()