import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, 
    Constraints, 
    PositionConstraint, 
    OrientationConstraint,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np  
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool

class ZeroGController(Node):
    def __init__(self):
        super().__init__("zero_g_controller")

        self.current_joint_state = None
        self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)
        self._move_client = ActionClient(self, MoveGroup, '/move_action')

        self.create_subscription(TFMessage, '/tf', self.update_pose, 10)
        self.create_subscription(Float32MultiArray, '/torque_input', self.update_goal, 10)
        self.create_subscription(Float32MultiArray,"/point_test",self.test_point,10)
        self.create_subscription(Bool, '/reset', self.reset_position, 10)

        self.init_position_publisher = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.start_pos_deg = np.array([-11,-61,19,-8,-46,109])

        self.last_t = None
        self.last_planning_t = None

        #Logging purposes
        self.og_quat = None
        self.og_pose = None

        self.moving = False

    def test_point(self,msg):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = self.current_joint_state[0]
        pose.pose.position.y = self.current_joint_state[1]
        pose.pose.position.z = self.current_joint_state[2]
        pose.pose.orientation.x = msg.data[3]
        pose.pose.orientation.y = msg.data[4]
        pose.pose.orientation.z = msg.data[5]
        pose.pose.orientation.w = msg.data[6]
        self.move_to_pose(pose, "ee_base_link")
        
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
    

    def update_pose(self,msg):
        '''
        Pose of the end effector in the inertial frame
        '''

        transform = msg.transforms[0]
        t = transform.header.stamp.nanosec/1e9
        x,y,z = (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z)
        q = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
        self.og_quat = q
        roll, pitch, yaw = np.array(euler_from_quaternion(q))
        self.og_pose = np.array([x,y,z,roll,pitch,yaw])
        
        # DEBUG: Log Isaac Sim's ee_base_link orientation (uncomment to debug)
        # self.get_logger().info(f"Isaac Sim ee_base_link: roll={np.degrees(roll):.1f}° pitch={np.degrees(pitch):.1f}° yaw={np.degrees(yaw):.1f}°")
        
        if self.last_t is not None:
            dt = t - self.last_t
            wx, wy, wz = (np.array([roll,pitch,yaw]) - self.current_joint_state[3:6]) / dt
            self.current_joint_state = np.array([x,y,z,roll,pitch,yaw,wx,wy,wz])
            self.last_t = t
        else:
            self.current_joint_state = np.array([x,y,z,roll,pitch,yaw,0,0,0])
            self.last_t = t

    
    def update_goal(self, msg):
        self.get_logger().info("Received move request")
        t = self.get_clock().now().nanoseconds/1e9
        if self.moving:
            self.get_logger().info("Already moving, skipping move request")
            return

        torque = np.array(msg.data) #tx, ty, tz in the end effector body frame
        # self.get_logger().info(f"torque: {torque}")

        if self.last_planning_t is not None:
            dt = t - self.last_planning_t
            self.last_planning_t = t
            # Double integrate to get orientation
            self.get_logger().info(f"euler angles: {self.current_joint_state[3:6]}")
            angular_velocity = torque*dt + self.current_joint_state[6:9] #angular velocity
            angle = angular_velocity*dt + self.current_joint_state[3:6] # angle
            angle_update = (angle + np.pi) % (2*np.pi) - np.pi # wrap angle
            self.get_logger().info(f'Goal angles: {angle_update}')
            # This logic is correct
            # self.get_logger().info(f"angle: {angle}") 
            # self.get_logger().info(f"dt: {self.dt}")
            # self.get_logger().info(f"angle_update: {angle_update}")
            # self.get_logger().info(f"angular_velocity: {angular_velocity}")


            position = Pose() #keep current position, only change orientation
            position.position.x = self.current_joint_state[0]
            position.position.y = self.current_joint_state[1]
            position.position.z = self.current_joint_state[2]

            quaternion = np.array(quaternion_from_euler(angle_update[0], angle_update[1], angle_update[2]))
            
            self.get_logger().info(f"Original Quaternion: {self.og_quat}")
            self.get_logger().info(f"Goal Quaternion: {quaternion}")
            self.get_logger().info(f"RPY: {self.og_pose[3:6]}")
            self.get_logger().info(f"Goal RPY: {angle_update}")
            position.orientation.x = quaternion[0]
            position.orientation.y = quaternion[1]
            position.orientation.z = quaternion[2]
            position.orientation.w = quaternion[3]
            pose_update = PoseStamped()
            pose_update.pose = position
            pose_update.header.stamp = self.get_clock().now().to_msg()
            pose_update.header.frame_id = "base_link"  # Position is in base_link frame!
            if not self.moving:
                self.move_to_pose(pose_update, "ee_base_link")
        else:
            self.last_planning_t = t
    
    def move_to_pose(self, pose_goal: PoseStamped, end_effector_link: str = "ee_base_link"):
        """
        Move to a Cartesian pose goal.
        
        Equivalent to MoveItPy:
            arm.set_start_state_to_current_state()
            arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=end_effector_link)
            plan_and_execute(...)
        """
        self.get_logger().info(f"Goal pose msg: {pose_goal}")
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        
        # Set planning group
        req.group_name = "arm"
        req.num_planning_attempts = 30
        req.allowed_planning_time = 1.0
        
        # Start state = current state (empty means use current)
        # req.start_state is left empty to use current state
        
        
        # Build pose constraint
        constraints = Constraints()
        
        # Position constraint - keep end effector at current position
        position_constraint = PositionConstraint()
        position_constraint.header = pose_goal.header
        position_constraint.link_name = end_effector_link
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        # position_constraint.target_point_offset.x = self.current_joint_state[0]
        # position_constraint.target_point_offset.y = self.current_joint_state[1]
        # position_constraint.target_point_offset.z = self.current_joint_state[2]

        
        # Orientation constraint, move to the goal orientation and keep position the same
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose_goal.header
        orientation_constraint.link_name = end_effector_link
        orientation_constraint.orientation = pose_goal.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.2  # radians
        orientation_constraint.absolute_y_axis_tolerance = 0.2
        orientation_constraint.absolute_z_axis_tolerance = 0.2
        orientation_constraint.weight = 1.0
        
        
        # Bounding volume is the tolerance around the goal position
        bounding_volume = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.1]  # 1cm tolerance sphere
        bounding_volume.primitives.append(primitive)
        
        # Primitive pose: WHERE the sphere is (position only, identity orientation)
        sphere_pose = Pose()
        sphere_pose.position = pose_goal.pose.position  # Current ee_base_link position
        sphere_pose.orientation.w = 1.0  # Identity quaternion (sphere doesn't need orientation)
        bounding_volume.primitive_poses.append(sphere_pose)
        
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0

        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        req.goal_constraints.append(constraints)
        
        # PATH CONSTRAINTS - enforce position constraint throughout entire trajectory
        path_constraints = Constraints()
        
        # Create a separate position constraint for path (same as goal constraint)
        path_position_constraint = PositionConstraint()
        path_position_constraint.header = pose_goal.header
        path_position_constraint.link_name = end_effector_link
        path_position_constraint.target_point_offset.x = 0.0
        path_position_constraint.target_point_offset.y = 0.0
        path_position_constraint.target_point_offset.z = 0.0
        
        
        # Same bounding volume - keep ee_base_link within 1cm sphere throughout motion
        path_bounding_volume = BoundingVolume()
        path_primitive = SolidPrimitive()
        path_primitive.type = SolidPrimitive.SPHERE
        path_primitive.dimensions = [0.1]  # 1cm tolerance sphere
        path_bounding_volume.primitives.append(path_primitive)
        
        path_sphere_pose = Pose()
        path_sphere_pose.position = pose_goal.pose.position
        path_sphere_pose.orientation.w = 1.0
        path_bounding_volume.primitive_poses.append(path_sphere_pose)
        
        path_position_constraint.constraint_region = path_bounding_volume
        path_position_constraint.weight = 1.0
        
        path_constraints.position_constraints.append(path_position_constraint)
        req.path_constraints = path_constraints
        
        # Plan and execute (not plan only)
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        self.moving = True

        future = self._move_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return
        self.get_logger().info("Goal accepted, executing...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)
    
    def _result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("Motion completed successfully!")
        else:
            self.get_logger().error(f"Motion failed with error code: {result.error_code.val}")
        self.moving = False
    

def main(args=None):
    rclpy.init(args=args)
    node = ZeroGController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
