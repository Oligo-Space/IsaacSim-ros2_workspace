import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import Pose, PoseStamped
from tf2_ros import Buffer
from tf2_msgs.msg import TFMessage
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.task import Future
from std_srvs.srv import Trigger
from threading import Lock
import numpy as np

class ZeroGController(Node):
    def __init__(self):
        super().__init__("zero_g_controller")
        
        #Subscribers
        self.create_subscription(TFMessage, '/tf', self.update_pose, 10)
        self.create_subscription(Float32MultiArray, '/torque_input', self.update_goal, 10)
        self.create_subscription(Bool, '/reset', self.reset_position, 10)

        #Publishers
        self.twist_publisher = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.init_position_publisher = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)

        #Services
        self.start_service = self.create_client(
            Trigger,
            "/servo_node/start_servo",
        )

        self.stop_service = self.create_client(
            Trigger,
            "/servo_node/stop_servo",
        )


        # Globals
        self.current_joint_state = None
        self.last_t = None
        self.last_planning_t = None
        self.start_pos_deg = np.array([-11,-61,19,-8,-46,109])
        self.move_lock = Lock()
        self.is_enabled = True

        self.init_position()
        self.start_servo()

    def __del__(self):
        '''
        Called when node is destroyed, stops the servo client
        '''
        self.stop_servo()

    def start_servo(self):
        future = self.start_service.call_async(Trigger.Request())
        future.add_done_callback(self.start_servo_callback)

    def start_servo_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Servo started: {response.message}")
            self.is_enabled = True
        except Exception as e:
            self.get_logger().error(f"Servo start failed: {e}")

    def stop_servo(self):
        future = self.stop_service.call_async(Trigger.Request())
        future.add_done_callback(self.stop_servo_callback)

    def stop_servo_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Servo stopped: {response.message}")
            self.is_enabled = False
        except Exception as e:
            self.get_logger().error(f"Servo stop failed: {e}")

    def reset_position(self,msg):
        if msg.data:
            self.init_position()

    def init_position(self):
        '''
        Initializes robot to non-zero state
        '''
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
        t = self.get_clock().now().nanoseconds/1e9
        if not self.is_enabled:
            self.get_logger().error("Servo is not enabled, skipping move request")
            return
            
        with self.move_lock:
            torque = np.array(msg.data,dtype=np.float64) #tx, ty, tz in the end effector body frame

            if self.last_planning_t is not None and self.current_joint_state is not None:
                dt = t - self.last_planning_t
                self.last_planning_t = t
                # Simple step-wise integration (dt will be based on update rate of the torque subscriber)
                # In the future, this might want to be constant if the torque publisher does not publish consistently
                angular_velocity = (torque*dt + self.current_joint_state[6:9]).astype(np.float64) #angular velocity  
                
                # Publish twist command in the end effector body frame
                # This will propogate using the IK plugin and ensure proper servo motion
                twist = TwistStamped()
                twist.header.stamp = self.get_clock().now().to_msg()
                twist.header.frame_id = "tool0"
                twist.twist.angular.x = angular_velocity[0]
                twist.twist.angular.y = angular_velocity[1]
                twist.twist.angular.z = angular_velocity[2]
                # Keep linear velocity 0 so ee stays in place
                twist.twist.linear.x = 0.0
                twist.twist.linear.y = 0.0
                twist.twist.linear.z = 0.0
                self.twist_publisher.publish(twist)

            else:
                self.last_planning_t = t
    
def main(args=None):
    rclpy.init(args=args)
    node = ZeroGController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
