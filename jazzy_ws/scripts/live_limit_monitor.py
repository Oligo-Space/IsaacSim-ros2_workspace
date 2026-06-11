#!/usr/bin/env python3
"""Live monitor: logs joint positions vs their limits + servo status.

Prints a line whenever any joint comes within MARGIN_DEG of its URDF limit,
when servo status changes, and a heartbeat snapshot every 2 s.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.msg import ServoStatus

LIMITS_DEG = {  # from arm_w_mm.urdf
    "joint_1_s": (-180, 180),
    "joint_2_l": (-130, 80),
    "joint_3_u": (-112, 208),
    "joint_4_r": (-360, 360),
    "joint_5_b": (-130, 130),
    "joint_6_t": (-360, 360),
}
MARGIN_DEG = 15.0
STATUS_NAMES = {0: "OK", 1: "DECEL_SINGULARITY", 2: "HALT_SINGULARITY",
                3: "DECEL_LEAVING_SING", 4: "DECEL_COLLISION", 5: "HALT_COLLISION", 6: "JOINT_BOUND"}


class Monitor(Node):
    def __init__(self):
        super().__init__("live_limit_monitor")
        self.create_subscription(JointState, "/isaac_joint_states", self.on_js, 10)
        self.create_subscription(ServoStatus, "/servo_node/status", self.on_status, 10)
        self.last_status = None
        self.near = set()
        self.latest = {}
        self.create_timer(2.0, self.heartbeat)

    def on_js(self, msg):
        for name, pos in zip(msg.name, msg.position):
            deg = math.degrees(pos)
            self.latest[name] = deg
            lo, hi = LIMITS_DEG.get(name, (-1e9, 1e9))
            close = deg < lo + MARGIN_DEG or deg > hi - MARGIN_DEG
            if close and name not in self.near:
                self.near.add(name)
                print(f"!! {name} NEAR LIMIT: {deg:8.2f} deg (limits [{lo}, {hi}])", flush=True)
            elif not close and name in self.near:
                self.near.discard(name)
                print(f"   {name} back in range: {deg:8.2f} deg", flush=True)

    def on_status(self, msg):
        if msg.code != self.last_status:
            print(f"** servo status -> {msg.code} ({STATUS_NAMES.get(msg.code, '?')})", flush=True)
            self.last_status = msg.code

    def heartbeat(self):
        snap = "  ".join(f"{j.split('_')[1]}:{self.latest.get(j, float('nan')):7.1f}" for j in LIMITS_DEG)
        print(f"   [{snap}]", flush=True)


rclpy.init()
rclpy.spin(Monitor())
