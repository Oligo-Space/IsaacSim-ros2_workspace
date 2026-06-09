import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32MultiArray, Bool, String
from tf2_msgs.msg import TFMessage
from tf_transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import pinocchio as pin
import threading
import numpy as np
from collections import deque
import time


class PinocchioZeroGController(Node):
    '''
    Zero-G attitude controller that mirrors ZeroGController (zero_g_servo.py) but performs
    its own differential (resolved-rate) inverse kinematics with Pinocchio instead of
    handing a TwistStamped to MoveIt2 Servo.

    Torque -> body-frame angular velocity (via ee_inertia) -> 6D spatial twist in the
    ee_base_link LOCAL frame -> damped-least-squares Jacobian solve -> joint velocities ->
    integrated joint positions -> /arm_controller/joint_trajectory.
    '''
    def __init__(self):
        super().__init__("pinocchio_zero_g_controller")

        # Subscribers
        self.create_subscription(Float32MultiArray, '/torque_input', self.update_speed, 10)
        self.create_subscription(Bool, '/reset', self.reset_position, 10)
        self.create_subscription(Float32MultiArray, "/update_position", self.reset_position, 10)
        self.create_subscription(JointState, '/joint_states', self.check_joint_states, 10)
        self.create_subscription(Bool, "reset_speed", self.reset_speed, 10)

        # Telemetry subscriber
        self.create_subscription(TFMessage, '/tf_sim', self.update_ee_pose, 10)

        # robot_description is published latched (transient_local) by robot_state_publisher
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(String, '/robot_description', self.on_robot_description, latched_qos)

        # Publishers
        self.init_position_publisher = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.arm_initialized_pub = self.create_publisher(Bool, '/arm_initialized', 10)

        # Telemetry Publisher
        self.ee_pose_pub = self.create_publisher(Float32MultiArray, '/ee_pose', 10)

        # Telemetry tracking
        self.last_t = None
        self.last_ee_poses = []
        self.window_size = 10

        # Globals
        self.curr_velocity = np.zeros(3)
        self.arm_initialized = True
        self.lock = threading.Lock()
        self.apply = deque([])

        self.arm_joint_names = ["joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"]
        self.start_pos_deg = np.array([-11, -61, 19, -8, -46, 109])
        self.dt = 1/60
        self.apply_time = 0.1
        # Lookahead horizon for the streamed trajectory point. The controller is
        # always given a target several ticks ahead so it never reaches the end of
        # a segment and decelerates between messages (which causes jitter). Only the
        # first dt of each segment is executed before it is replaced next tick.
        self.lookahead = 0.05

        self.halt_timer = False
        self.timer = None
        # Dedicated callback group so the control loop runs on its own executor
        # thread and is not starved by the 200 Hz /joint_states + /tf_sim callbacks
        # (which would make publishing irregular and the arm stutter).
        self.control_cb_group = MutuallyExclusiveCallbackGroup()

        self.ee_inertia = np.array(
            [
                [0.669, 0.0, 0.0],
                [0.0, 2.609, 0.0],
                [0.0, 0.0, 3.261]
            ]
        )

        # Pinocchio model state (built asynchronously from /robot_description)
        self.frame_str = "ee_base_link"
        self.damping = 0.05          # lambda for damped least squares
        self.model = None
        self.data = None
        self.model_ready = False
        self.ee_frame_id = None
        self.q = None                # internal open-loop configuration (reduced-model layout)
        self.q_idx = None            # ROS-order -> model q index map
        self.v_idx = None            # ROS-order -> model v index map
        self.lower = None            # joint position lower limits, ROS order
        self.upper = None            # joint position upper limits, ROS order

        # --- Diagnostics: measure actual rates with a monotonic wall clock ---
        self._diag_lock = threading.Lock()
        self._js_count = 0            # /joint_states callbacks since last report
        self._tf_count = 0            # /tf_sim callbacks since last report
        self._torque_count = 0        # /torque_input callbacks since last report
        self._loop_count = 0          # update_vis executions since last report
        self._loop_last_t = None      # monotonic time of previous update_vis entry
        self._loop_intervals = []     # inter-call intervals (s) — control-loop jitter
        self._loop_compute = []       # time spent in update_vis body (s)
        self._loop_lockwait = []      # time blocked acquiring self.lock (s)
        self._diag_last_t = time.monotonic()
        self.create_timer(1.0, self._report_diagnostics)

        # Bootstrap once the model arrives: publishes start pose and starts the control loop
        self.bootstrap_timer = self.create_timer(0.1, self._try_bootstrap)

    @staticmethod
    def _stats_ms(arr):
        '''min/mean/max of a list of seconds, expressed in milliseconds.'''
        if not arr:
            return (0.0, 0.0, 0.0)
        a = np.asarray(arr) * 1000.0
        return (float(a.min()), float(a.mean()), float(a.max()))

    def _report_diagnostics(self):
        now = time.monotonic()
        with self._diag_lock:
            elapsed = now - self._diag_last_t
            js, tf, tq, loops = self._js_count, self._tf_count, self._torque_count, self._loop_count
            intervals = self._loop_intervals
            computes = self._loop_compute
            lockwaits = self._loop_lockwait
            self._js_count = self._tf_count = self._torque_count = self._loop_count = 0
            self._loop_intervals, self._loop_compute, self._loop_lockwait = [], [], []
            self._diag_last_t = now

        if elapsed <= 0:
            return
        i_min, i_mean, i_max = self._stats_ms(intervals)
        c_min, c_mean, c_max = self._stats_ms(computes)
        l_min, l_mean, l_max = self._stats_ms(lockwaits)
        self.get_logger().info(
            f"[diag {elapsed:.2f}s] loop={loops/elapsed:5.1f}Hz (target {1/self.dt:.0f}) "
            f"interval ms[min/avg/max]={i_min:5.1f}/{i_mean:5.1f}/{i_max:6.1f} "
            f"compute={c_min:4.1f}/{c_mean:4.1f}/{c_max:5.1f} "
            f"lockwait={l_min:4.1f}/{l_mean:4.1f}/{l_max:6.1f} | "
            f"/joint_states={js/elapsed:6.1f}Hz /tf_sim={tf/elapsed:6.1f}Hz "
            f"/torque_input={tq/elapsed:4.1f}Hz"
        )

    def on_robot_description(self, msg):
        '''
        Builds the Pinocchio model from the latched robot_description (URDF XML string).
        Fires once; subsequent messages are ignored.
        '''
        if self.model_ready:
            return
        try:
            self.build_model(msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to build Pinocchio model: {e}")

    def build_model(self, urdf_string):
        full_model = pin.buildModelFromXML(urdf_string)

        # Lock the reaction-wheel joints (continuous joints would inflate nq/nv)
        lock_names = [n for n in ["rw1_joint", "rw2_joint", "rw3_joint", "rw4_joint"]
                      if full_model.existJointName(n)]
        lock_ids = [full_model.getJointId(n) for n in lock_names]
        self.model = pin.buildReducedModel(full_model, lock_ids, pin.neutral(full_model))
        self.data = self.model.createData()

        # ee_base_link is fixed to link_t (a surviving link), so its frame is preserved
        assert self.model.existFrame(self.frame_str), f"frame {self.frame_str} missing after reduction"
        self.ee_frame_id = self.model.getFrameId(self.frame_str)

        # Build ROS-order <-> Pinocchio q/v index maps (do not assume joint ordering)
        self.q_idx = np.zeros(6, dtype=int)
        self.v_idx = np.zeros(6, dtype=int)
        for i, name in enumerate(self.arm_joint_names):
            jid = self.model.getJointId(name)
            joint = self.model.joints[jid]
            self.q_idx[i] = joint.idx_q
            self.v_idx[i] = joint.idx_v

        # Joint limits in ROS order
        self.lower = np.array(self.model.lowerPositionLimit)[self.q_idx]
        self.upper = np.array(self.model.upperPositionLimit)[self.q_idx]

        # Seed internal configuration from the start pose
        self.q = pin.neutral(self.model)
        self.q[self.q_idx] = np.clip(np.deg2rad(self.start_pos_deg), self.lower, self.upper)

        self.model_ready = True
        self.get_logger().info(
            f"Pinocchio model built (nq={self.model.nq}, nv={self.model.nv}), "
            f"ee frame '{self.frame_str}' found"
        )

    def _try_bootstrap(self):
        '''
        Once the model is built, publish the start pose and create the control timer.
        Runs as a one-shot gated retry on a 0.1s timer.
        '''
        if not self.model_ready or self.timer is not None:
            return
        self.init_position()
        self.bootstrap_timer.cancel()

    def check_joint_states(self, msg):
        '''
        Not in use - checks if the arm has reached the initial start position
        '''
        with self._diag_lock:
            self._js_count += 1
        if not self.arm_initialized:
            if np.linalg.norm(np.array(msg.position) - np.array(self.start_pos_deg)) > 0.01:
                self.init_position()
            else:
                self.arm_initialized = True
                self.arm_initialized_pub.publish(Bool(data=True))

    def reset_position(self, msg):
        '''
        Resets the position of the arm to the initial start position
        or a custom position based on the msg value (Bool vs. Array)
        '''
        if isinstance(msg, Float32MultiArray):
            self.init_position(msg.data)
        elif msg.data:
            self.init_position()

    def reset_speed(self, msg):
        '''
        Resets the speed of the arm to the initial start speed
        '''
        if msg.data:
            self.curr_velocity = np.zeros(3)

    def update_ee_pose(self, msg):
        '''
        Updates the EE pose telemetry and publishes an averaged pose+velocity window.
        '''
        with self._diag_lock:
            self._tf_count += 1
        transform = msg.transforms[0]
        x, y, z = (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z)
        r, p, yaw = euler_from_quaternion([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
        t = msg.transforms[0].header.stamp.sec + msg.transforms[0].header.stamp.nanosec/1e9
        if self.last_t and len(self.last_ee_poses) < self.window_size and len(self.last_ee_poses) > 0:
            dt = t - self.last_t
            vx, vy, vz, wx, wy, wz = (self.last_ee_poses[-1][:6] - np.array([x, y, z, r, p, yaw])) / dt
            self.last_ee_poses.append(np.array([x, y, z, r, p, yaw, vx, vy, vz, wx, wy, wz]))
        elif len(self.last_ee_poses) == self.window_size:
            self.ee_pose_pub.publish(Float32MultiArray(data=np.mean(self.last_ee_poses, axis=0)))
            self.last_ee_poses = []
        else:
            self.last_ee_poses.append(np.array([x, y, z, r, p, yaw, 0, 0, 0, 0, 0, 0]))
        self.last_t = t

    def init_position(self, pos=None):
        '''
        Initializes robot to non-zero state and (re)starts the control loop.
        '''
        if pos is not None:
            self.start_pos_deg = np.rad2deg(pos)
        with self.lock:
            self.halt_timer = True
            msg = JointTrajectory()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "ee_base_link"
            msg.joint_names = self.arm_joint_names
            for i in range(1, 10):  # Do this multiple times to ensure that the message gets published
                msg.points = [
                    JointTrajectoryPoint(positions=list(np.deg2rad(self.start_pos_deg)),
                                         velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                         time_from_start=Duration(sec=0, nanosec=i*100000000))
                ]
                self.init_position_publisher.publish(msg)
                time.sleep(0.1)  # sleeping with rclpy clock kills the node since there is a timer running i think

            # Re-seed open-loop configuration so integration restarts from the start pose
            if self.model_ready:
                self.q[self.q_idx] = np.clip(np.deg2rad(self.start_pos_deg), self.lower, self.upper)

            if self.timer is None:
                self.timer = self.create_timer(self.dt, self.update_vis, callback_group=self.control_cb_group)
            self.halt_timer = False

    def update_speed(self, msg):
        # update current velocity based on torque
        with self._diag_lock:
            self._torque_count += 1
        if len(msg.data) == 4:
            torque = msg.data[:3]
            apply_time = msg.data[-1]
        else:
            torque = msg.data
            apply_time = self.apply_time

        self.apply.extend([torque]*int(apply_time/self.dt))

    def update_vis(self):
        if self.halt_timer or not self.model_ready:
            return

        # Diagnostics: record control-loop interval (jitter) and start compute timer
        entry_t = time.monotonic()
        with self._diag_lock:
            if self._loop_last_t is not None:
                self._loop_intervals.append(entry_t - self._loop_last_t)
            self._loop_last_t = entry_t
            self._loop_count += 1

        # Pop the next queued torque (or zero when the queue is empty)
        with self.lock:
            if len(self.apply) > 0:
                torque = self.apply.popleft()
            else:
                torque = np.zeros(3)

        # Integrate torque into body-frame angular velocity (persists; no decay)
        self.curr_velocity = (np.linalg.inv(self.ee_inertia) @ np.asarray(torque, dtype=float)*self.dt + self.curr_velocity)

        # Desired 6D spatial twist in the ee_base_link LOCAL frame.
        # Pinocchio convention: rows 0:3 linear, 3:6 angular. Keep linear 0 so the EE holds position.
        v_des = np.zeros(6)
        v_des[3:6] = self.curr_velocity

        lock_req_t = time.monotonic()
        with self.lock:
            lock_wait = time.monotonic() - lock_req_t
            # Frame Jacobian in the LOCAL frame (matches body-frame angular velocity)
            pin.forwardKinematics(self.model, self.data, self.q)
            pin.updateFramePlacements(self.model, self.data)
            J = pin.computeFrameJacobian(self.model, self.data, self.q, self.ee_frame_id, pin.ReferenceFrame.LOCAL)

            # Damped least squares: qdot = J^T (J J^T + lambda^2 I)^-1 v_des
            lam2 = self.damping ** 2
            JJt = J @ J.T
            qdot = J.T @ np.linalg.solve(JJt + lam2 * np.eye(6), v_des)

            # Lookahead target the controller integrates toward (gives it runway so
            # it never runs out of trajectory between messages -> removes jitter).
            q_target_full = pin.integrate(self.model, self.q, qdot * self.lookahead)
            # Advance the internal open-loop command by exactly one control tick;
            # this is the state the next segment starts from.
            self.q = pin.integrate(self.model, self.q, qdot * self.dt)

            # Clamp both the command state and the published target to joint limits
            self.q[self.q_idx] = np.clip(self.q[self.q_idx], self.lower, self.upper)
            q_target = np.clip(q_target_full[self.q_idx], self.lower, self.upper)
            qdot_arm = qdot[self.v_idx]

        msg = JointTrajectory()
        # Leave header.stamp at 0 so arm_controller starts the trajectory on receipt
        # (an absolute current-time stamp + tiny time_from_start ends in the past).
        msg.header.frame_id = "ee_base_link"
        msg.joint_names = self.arm_joint_names
        msg.points = [
            JointTrajectoryPoint(
                positions=list(map(float, q_target)),
                velocities=list(map(float, qdot_arm)),
                time_from_start=Duration(sec=int(self.lookahead),
                                         nanosec=int((self.lookahead % 1) * 1e9)),
            )
        ]
        self.init_position_publisher.publish(msg)

        # Diagnostics: record lock-wait and total compute time for this tick
        with self._diag_lock:
            self._loop_lockwait.append(lock_wait)
            self._loop_compute.append(time.monotonic() - entry_t)


def main(args=None):
    rclpy.init(args=args)
    node = PinocchioZeroGController()

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
