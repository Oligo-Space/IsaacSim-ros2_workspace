import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray, Bool, String
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import Pose, PoseStamped
from tf2_ros import Buffer, TransformBroadcaster
from tf2_msgs.msg import TFMessage
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, quaternion_inverse
from geometry_msgs.msg import TwistStamped, TransformStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from rclpy.task import Future
from std_srvs.srv import SetBool
from moveit_msgs.srv import ServoCommandType
import threading
import numpy as np
from collections import deque
import time

class ZeroGController(Node):
    '''
    Uses basic servo control to move the arm to a desired position
    based on the torque input.
    '''
    def __init__(self):
        super().__init__("zero_g_controller")
        
        #Subscribers
        self.create_subscription(Float32MultiArray, '/torque_input', self.update_speed, 10)
        self.create_subscription(Bool, '/reset', self.reset_position, 10)
        self.create_subscription(Float32MultiArray, "/update_position", self.reset_position,10)
        # /joint_states subscription removed: check_joint_states is unused and the
        # 200 Hz callback starved the 60 Hz control timer (measured median twist
        # gap 21.9 ms vs 16.7 ms target, gaps up to 1.26 s)
        self.create_subscription(Bool, "reset_speed", self.reset_speed, 10)

        # Telemetry subcriber
        self.create_subscription(TFMessage, '/tf_sim', self.update_ee_pose, 10)

        #Publishers
        self.twist_publisher = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.init_position_publisher = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.arm_initialized_pub = self.create_publisher(Bool, '/arm_initialized', 10)

        # Telemetry Publisher
        self.ee_pose_pub = self.create_publisher(Float32MultiArray, '/ee_pose', 10)

        #Services (Jazzy MoveIt Servo API)
        self.pause_service = self.create_client(
            SetBool,
            "/servo_node/pause_servo",
        )

        self.switch_command_type_service = self.create_client(
            ServoCommandType,
            "/servo_node/switch_command_type",
        )

        # Telemetry tracking
        self.last_t = None
        self.last_ee_poses = []
        self.window_size = 10


        # Globals
        self.curr_velocity = np.zeros(3)
        self.last_planning_t = None
        self.is_enabled = False
        self.arm_initialized = True
        self.lock = threading.Lock()
        self.apply = deque([])
        
        self.start_pos_deg = np.array([-11,-61,19,-8,-46,109])
        self.start_pos_transform = np.array([2.628, -0.488, 1.304])
        self.dt = 1/60
        self.apply_time = 0.1

        self.halt_timer = False
        self.timer=None
        self.last_tick_t = None
        self.last_ts = None
        # self.ee_inertia = np.array(
        #     [
        #         [0.669, 0.0, 0.0],
        #         [0.0, 2.609, 0.0],
        #         [0.0, 0.0, 3.261]
        #     ]
        # )

        self.ee_inertia = np.eye(3)

        self.init_position()
        self.create_timer(0.1, self.check_servo_status)

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
        Checks if the servo is enabled and starts it if it is not.
        Runs on a timer.
        '''
        if not self.is_enabled and not self.halt_timer:
            self.get_logger().info("Servo is not enabled, starting servo")
            self.start_servo()

    def start_servo(self):
        '''
        Unpauses servo and sets command type to TWIST.
        '''
        if not self.pause_service.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("pause_servo service not available yet, will retry")
            return
        if not self.switch_command_type_service.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("switch_command_type service not available yet, will retry")
            return

        cmd_req = ServoCommandType.Request()
        cmd_req.command_type = ServoCommandType.Request.TWIST
        future_cmd = self.switch_command_type_service.call_async(cmd_req)
        future_cmd.add_done_callback(self._on_command_type_set)

    def _on_command_type_set(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Servo command type set to TWIST")
                pause_req = SetBool.Request()
                pause_req.data = False
                future_unpause = self.pause_service.call_async(pause_req)
                future_unpause.add_done_callback(self._on_servo_unpaused)
            else:
                self.get_logger().error("Failed to set servo command type")
        except Exception as e:
            self.get_logger().error(f"switch_command_type failed: {e}")

    def _on_servo_unpaused(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Servo unpaused: {response.message}")
            self.is_enabled = True
        except Exception as e:
            self.get_logger().error(f"Servo unpause failed: {e}")

    def stop_servo(self):
        '''
        Pauses the servo.
        '''
        if not self.pause_service.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("pause_servo service not available yet")
            return
        pause_req = SetBool.Request()
        pause_req.data = True
        future = self.pause_service.call_async(pause_req)
        future.add_done_callback(self._on_servo_paused)

    def _on_servo_paused(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Servo paused: {response.message}")
            self.is_enabled = False
        except Exception as e:
            self.get_logger().error(f"Servo pause failed: {e}")

    def reset_position(self,msg):
        '''
        Resets the position of the arm to the initial start position
        or a custom position based on the msg value (Bool vs. Array)
        '''
        if isinstance(msg, Float32MultiArray):
            self.init_position(msg.data)
        elif msg.data:
            self.init_position()
    
    def reset_speed(self,msg):
        '''
        Resets the speed of the arm to the initial start speed
        '''
        if msg.data:
            self.curr_velocity = np.zeros(3)

    def update_ee_pose(self, msg):
        '''
        Updates the EE pose. Publishes [x,y,z, r,p,yaw, vx,vy,vz, wx,wy,wz] where
        w is the body-frame angular velocity from the quaternion delta (euler-angle
        rates are NOT angular velocity and read unequal even for equal body rates).
        '''
        # Select the EE transform by frame name; transforms[0] is whatever link
        # Isaac happens to list first
        transform = None
        for tf in msg.transforms:
            if "ee_base_link" in tf.child_frame_id:
                transform = tf
                break
        if transform is None:
            if not getattr(self, "_warned_no_ee_tf", False):
                self._warned_no_ee_tf = True
                self.get_logger().warn(
                    f"No ee_base_link in /tf_sim (frames: {[t.child_frame_id for t in msg.transforms]}), "
                    "falling back to transforms[0]")
            transform = msg.transforms[0]

        x,y,z = (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z)
        q = np.array([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
        r,p,yaw = euler_from_quaternion(q)
        t = transform.header.stamp.sec + transform.header.stamp.nanosec/1e9

        if self.last_t and len(self.last_ee_poses) < self.window_size and len(self.last_ee_poses) > 0:
            dt = t - self.last_t
            if dt <= 0:
                return
            vx,vy,vz = (np.array([x,y,z]) - self.last_ee_poses[-1][:3]) / dt
            # Body-frame angular velocity: q_rel = q_prev^-1 * q_curr, w = axis*angle/dt
            q_rel = quaternion_multiply(quaternion_inverse(self.last_q), q)
            if q_rel[3] < 0:
                q_rel = tuple(-np.array(q_rel))  # shortest rotation
            v_norm = np.linalg.norm(q_rel[:3])
            angle = 2.0 * np.arctan2(v_norm, q_rel[3])
            w_body = (q_rel[:3] / v_norm) * angle / dt if v_norm > 1e-9 else np.zeros(3)
            self.last_ee_poses.append(np.array([x,y,z,r,p,yaw,vx,vy,vz,*w_body]))
        elif len(self.last_ee_poses) == self.window_size:
            self.ee_pose_pub.publish(Float32MultiArray(data=np.mean(self.last_ee_poses, axis=0)))
            self.last_ee_poses = []
        else:
            self.last_ee_poses.append(np.array([x,y,z,r,p,yaw,0,0,0,0,0,0]))
        self.last_t = t
        self.last_q = q


    def init_position(self,pos=None):
        '''0
        Initializes robot to non-zero state
        '''
        if pos is not None:
            self.start_pos_deg = np.rad2deg(pos)
        with self.lock:
            if self.is_enabled:
                # Servo needs to stop otherwise it will overwrite the reset
                self.stop_servo()
                self.is_enabled = False

            self.halt_timer = True
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
                time.sleep(0.1) #sleeping with rclpy clock kills the node since there is a timer running i think
            # A reset discards pending torques; replaying them from the new pose
            # makes no physical sense and keeps the arm moving after a reset
            self.apply.clear()
            self.last_tick_t = None
            if self.timer is None:
                self.timer = self.create_timer(self.dt,self.update_vis)
            self.halt_timer = False
            if not self.is_enabled:
                self.start_servo()

    def update_speed(self,msg):
        # update current velocity based on torque
        # self.get_logger().info(f'{msg}')
        if len(msg.data) == 4:
            torque = msg.data[:3]
            apply_time = msg.data[-1]
        else:
            torque = msg.data
            apply_time = self.apply_time
        
        # round, not int: truncation drops up to one tick of impulse per command
        self.apply.extend([torque]*round(apply_time/self.dt))

    def update_vis(self):
        if not self.is_enabled or self.halt_timer:
            self.get_logger().error("Servo is not enabled, skipping move request")
            return
        # Publish twist command in the end effector body frame
        # This will propogate using the IK plugin and ensure proper servo motion

        now_clock = self.get_clock().now()
        now = now_clock.nanoseconds / 1e9
        if self.last_tick_t is None:
            n_ticks = 1
            self.last_tick_t = now
        else:
            # floor + advance by consumed ticks so fractional time carries over
            n_ticks = min(int((now - self.last_tick_t) / self.dt), 60)
            self.last_tick_t += n_ticks * self.dt
        impulse = np.zeros(3)
        with self.lock:
            for _ in range(n_ticks):
                if len(self.apply) == 0:
                    break
                impulse += np.asarray(self.apply.popleft(), dtype=float)
            if len(self.apply) == 0:
                # no backlog left: drop any accumulated time deficit so a future
                # command ramps over its ticks instead of being consumed in a burst
                self.last_tick_t = now

        ts = self.get_clock().now().to_msg()
        
        if self.last_ts is None or ts != self.last_ts:
            self.curr_velocity = (np.linalg.inv(self.ee_inertia) @ impulse*self.dt + self.curr_velocity)
            speed = self.curr_velocity
            twist = TwistStamped()
            twist.header.stamp = ts #timestamp of current time
            twist.header.frame_id = "base_link"
            twist.twist.angular.x = speed[0]
            twist.twist.angular.y = speed[1]
            twist.twist.angular.z = speed[2]
            # Keep linear velocity 0 so ee stays in place
            twist.twist.linear.x = 0.0
            twist.twist.linear.y = 0.0
            twist.twist.linear.z = 0.0
            self.twist_publisher.publish(twist)

        self.last_ts = ts

def main(args=None):
    rclpy.init(args=args)
    node = ZeroGController()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
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
