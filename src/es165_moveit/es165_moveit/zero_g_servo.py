import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, String
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import Pose, PoseStamped
from tf2_ros import Buffer
from tf2_msgs.msg import TFMessage
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from rclpy.task import Future
from std_srvs.srv import Trigger
from threading import Lock
import numpy as np
import time

class ZeroGController(Node):
    '''
    Uses basic servo control to move the arm to a desired position
    based on the torque input.
    '''
    def __init__(self):
        super().__init__("zero_g_controller")
        
        #Subscribers
        self.create_subscription(Float32MultiArray, '/torque_input', self.update_goal, 10)
        self.create_subscription(Bool, '/reset', self.reset_position, 10)
        self.create_subscription(Float32MultiArray, "/update_position", self.reset_position,10)
        self.create_subscription(JointState, '/joint_states', self.check_joint_states, 10)

        #Publishers
        self.twist_publisher = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.init_position_publisher = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.arm_initialized_pub = self.create_publisher(Bool, '/arm_initialized', 10)


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
        self.curr_velocity = None
        self.last_t = None
        self.last_planning_t = None
        self.is_enabled = False
        self.start_pos_deg = np.array([-11,-61,19,-8,-46,109])
        self.arm_initialized = True
        
        # Thread lock
        self.move_lock = Lock()


        self.ee_inertia = np.array(
            [
                [0.669, 0.0, 0.0],
                [0.0, 2.609, 0.0],
                [0.0, 0.0, 3.261]
            ]
        )

        self.start_servo()
        self.dt = 1/30
        self.create_timer(0.5, self.check_servo_status)
        self.init_position()

    def __del__(self):
        '''
        Called when node is destroyed, stops the servo client
        '''
        self.stop_servo()
    
    def check_joint_states(self, msg):
        '''
        Not in use - checks if the arm has reached the initial start position
        '''
        # self.get_logger().info(f"Joint states: {msg.position}")
        if not self.arm_initialized:
            if np.linalg.norm(np.array(msg.position) - np.array(self.start_pos_deg)) > 0.01:
                self.init_position()
            else:
                self.arm_initialized = True
                self.arm_initialized_pub.publish(Bool(data=True))

    def check_servo_status(self):
        '''
        Checks if the servo is enabled and starts it if it is not
        Runs on a timer
        '''
        if not self.is_enabled:
            self.get_logger().info("Servo is not enabled, starting servo")
            self.start_servo()
        else:
            return

    def start_servo(self):
        '''
        Starts the servo client
        '''
        future = self.start_service.call_async(Trigger.Request())
        future.add_done_callback(self.start_servo_callback)

    def start_servo_callback(self, future):
        '''
        Callback for the start servo service
        '''
        try:
            response = future.result()
            self.get_logger().info(f"Servo started: {response.message}")
            self.is_enabled = True
        except Exception as e:
            self.get_logger().error(f"Servo start failed: {e}")

    def stop_servo(self):
        '''
        Stops the servo client
        '''
        future = self.stop_service.call_async(Trigger.Request())
        future.add_done_callback(self.stop_servo_callback)

    def stop_servo_callback(self, future):
        '''
        Callback for the stop servo service
        '''
        try:
            response = future.result()
            self.get_logger().info(f"Servo stopped: {response.message}")
            self.is_enabled = False
        except Exception as e:
            self.get_logger().error(f"Servo stop failed: {e}")

    def reset_position(self,msg):
        '''
        Resets the position of the arm to the initial start position
        or a custom position based on the msg value (Bool vs. Array)
        '''
        if isinstance(msg, Float32MultiArray):
            self.init_position(msg.data)
        elif msg.data:
            self.init_position()

    def init_position(self,pos=None):
        '''
        Initializes robot to non-zero state
        '''
        with self.move_lock:
            if pos is not None:
                self.start_pos_deg = np.rad2deg(pos)

            msg = JointTrajectory()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.joint_names = ["joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"]
            msg.points = [
                JointTrajectoryPoint(positions=list(np.deg2rad(self.start_pos_deg)), velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=Duration(sec=0, nanosec=100000000))
            ]
            for i in range(1,10): # Do this multiple times to ensure that the message gets published
                msg.points = [
                    JointTrajectoryPoint(positions=list(np.deg2rad(self.start_pos_deg)), \
                        velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=Duration(sec=0, nanosec=i*100000000))
                ]
                self.init_position_publisher.publish(msg)
                time.sleep(0.1)
            self.create_timer(self.dt,lambda: self.update_goal(None))
            time.sleep(1.0)


    def update_goal(self, msg):
        if not self.is_enabled:
            self.get_logger().error("Servo is not enabled, skipping move request")
            return

        # update current velocity based on torque
        if self.curr_velocity is None:
            self.curr_velocity = np.zeros(3)
        if msg:
            self.curr_velocity = (np.linalg.inv(self.ee_inertia) @ msg.data*self.dt + self.curr_velocity)
        
        # Publish twist command in the end effector body frame
        # This will propogate using the IK plugin and ensure proper servo motion
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg() #timestamp of current time
        twist.header.frame_id = "base_link"
        twist.twist.angular.x = self.curr_velocity[0]
        twist.twist.angular.y = self.curr_velocity[1]
        twist.twist.angular.z = self.curr_velocity[2]
        # Keep linear velocity 0 so ee stays in place
        twist.twist.linear.x = 0.0
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        self.twist_publisher.publish(twist)

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

if __name__ == '__main__':
    main()
