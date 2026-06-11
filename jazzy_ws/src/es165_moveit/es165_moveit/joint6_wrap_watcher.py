"""Companion to scripts/isaac_joint6_wrap.py (which runs inside Isaac Sim).

When the Isaac-side script teleports joint_6_t by one full revolution (same
physical orientation, fresh travel range), this node sees the >pi jump on
/isaac_joint_states and re-seats the open-loop JointTrajectoryController at the
new position so it doesn't command the joint back to the pre-wrap angle.

Sequence on jump detection: pause servo -> publish single-point trajectory at
the post-wrap joint positions -> unpause servo. The zero-g controller's
integrated velocity is untouched, so rotation resumes seamlessly.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


WRAP_JOINT = "joint_6_t"
JUMP_THRESHOLD = math.pi  # rad; the Isaac script teleports by exactly 2*pi
UNPAUSE_DELAY = 0.1  # s; give the JTC a cycle to latch the re-seat trajectory


class Joint6WrapWatcher(Node):
    def __init__(self):
        super().__init__("joint6_wrap_watcher")
        self.create_subscription(JointState, "/isaac_joint_states", self.on_joint_states, 10)
        self.trajectory_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.pause_client = self.create_client(SetBool, "/servo_node/pause_servo")
        self.last_pos = None
        self.wrapping = False
        self.unpause_timer = None

    def on_joint_states(self, msg):
        if WRAP_JOINT not in msg.name:
            return
        idx = msg.name.index(WRAP_JOINT)
        pos = msg.position[idx]
        last = self.last_pos
        self.last_pos = pos
        if last is None or self.wrapping:
            return
        if abs(pos - last) > JUMP_THRESHOLD:
            self.wrapping = True
            self.get_logger().info(
                f"{WRAP_JOINT} wrapped {math.degrees(last):.1f} -> {math.degrees(pos):.1f} deg; re-seating controller")
            self.set_servo_paused(True)
            self.reseat_controller(msg)
            # unpause after the JTC has consumed the re-seat point
            self.unpause_timer = self.create_timer(UNPAUSE_DELAY, self.finish_wrap)

    def reseat_controller(self, joint_state):
        traj = JointTrajectory()
        traj.joint_names = list(joint_state.name)
        point = JointTrajectoryPoint()
        point.positions = list(joint_state.position)
        point.velocities = [0.0] * len(joint_state.name)
        point.time_from_start.nanosec = 10_000_000  # 10 ms: effectively instant
        traj.points = [point]
        self.trajectory_pub.publish(traj)

    def finish_wrap(self):
        self.unpause_timer.cancel()
        self.unpause_timer = None
        self.set_servo_paused(False)
        self.wrapping = False

    def set_servo_paused(self, paused):
        if not self.pause_client.service_is_ready():
            self.get_logger().warn("pause_servo service not ready; skipping pause toggle")
            return
        req = SetBool.Request()
        req.data = paused
        self.pause_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = Joint6WrapWatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
