#!/usr/bin/env python3
"""Analyze jitter_onset bag: find regressions in command/state streams around torque events."""
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = "/home/pj/IsaacSim-ros2_workspace/jazzy_ws/jitter_onset"
TOPICS = {
    "/torque_input": "std_msgs/msg/Float32MultiArray",
    "/arm_controller/joint_trajectory": "trajectory_msgs/msg/JointTrajectory",
    "/isaac_joint_commands": "sensor_msgs/msg/JointState",
    "/isaac_joint_states": "sensor_msgs/msg/JointState",
    "/servo_node/status": "moveit_msgs/msg/ServoStatus",
    "/servo_node/delta_twist_cmds": "geometry_msgs/msg/TwistStamped",
    "/clock": "rosgraph_msgs/msg/Clock",
}

reader = SequentialReader()
reader.open(StorageOptions(uri=BAG, storage_id="mcap"), ConverterOptions("", ""))
type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

data = {k: [] for k in TOPICS}
while reader.has_next():
    topic, raw, t_ns = reader.read_next()
    if topic not in TOPICS:
        continue
    msg = deserialize_message(raw, get_message(type_map[topic]))
    t = t_ns / 1e9
    if topic == "/torque_input":
        data[topic].append((t, list(msg.data)))
    elif topic == "/arm_controller/joint_trajectory":
        if msg.points:
            names = list(msg.joint_names)
            data[topic].append((t, names, np.array(msg.points[0].positions),
                                np.array(msg.points[0].velocities) if msg.points[0].velocities else None))
    elif topic in ("/isaac_joint_commands", "/isaac_joint_states"):
        names = list(msg.name)
        data[topic].append((t, names, np.array(msg.position),
                            np.array(msg.velocity) if msg.velocity else None))
    elif topic == "/servo_node/status":
        data[topic].append((t, msg.code))
    elif topic == "/servo_node/delta_twist_cmds":
        tw = msg.twist
        data[topic].append((t, np.array([tw.angular.x, tw.angular.y, tw.angular.z])))
    elif topic == "/clock":
        data[topic].append((t, msg.clock.sec + msg.clock.nanosec / 1e9))

t0 = min(v[0][0] for v in data.values() if v)
print(f"=== bag t0 = {t0:.3f} (all times relative) ===\n")

print("=== TORQUE EVENTS ===")
for t, d in data["/torque_input"]:
    print(f"  t={t-t0:8.3f}s  torque={d}")

# RTF from /clock
cl = data["/clock"]
if len(cl) > 10:
    wall = np.array([c[0] for c in cl]); sim = np.array([c[1] for c in cl])
    rtf = np.polyfit(wall, sim, 1)[0]
    print(f"\n=== CLOCK ===  {len(cl)} msgs, rate={len(cl)/(wall[-1]-wall[0]):.1f} Hz wall, RTF(sim/wall)={rtf:.3f}")

# Twist command gaps
tw = data["/servo_node/delta_twist_cmds"]
tt = np.array([x[0] for x in tw])
gaps = np.diff(tt)
print(f"\n=== TWIST CMDS ===  {len(tw)} msgs, span {tt[0]-t0:.2f}..{tt[-1]-t0:.2f}s, "
      f"median gap={np.median(gaps)*1000:.1f}ms, max gap={gaps.max()*1000:.1f}ms")
big = np.where(gaps > 0.05)[0]
print(f"  gaps > 50ms (servo max_expected_latency): {len(big)}")
for i in big[:15]:
    print(f"    t={tt[i]-t0:8.3f}s gap={gaps[i]*1000:6.1f}ms")

# Servo status changes
st = data["/servo_node/status"]
print("\n=== SERVO STATUS CHANGES ===  (0=NO_WARNING)")
last = None
for t, c in st:
    if c != last:
        print(f"  t={t-t0:8.3f}s  code={c}")
        last = c

# Regression detection helper: for each stream, per joint, find times where position
# moves AGAINST the locally dominant direction by more than eps.
def find_regressions(stream, label, eps=1e-4):
    if not stream:
        return
    # build per-joint series using name order of first msg
    names = stream[0][1]
    T = np.array([s[0] for s in stream])
    # align positions by name
    P = np.full((len(stream), len(names)), np.nan)
    for i, (_, nm, pos, _) in enumerate(stream):
        for j, n in enumerate(names):
            if n in nm:
                P[i, j] = pos[nm.index(n)]
    print(f"\n=== REGRESSIONS in {label} ===  ({len(stream)} msgs)")
    total = 0
    for j, n in enumerate(names):
        p = P[:, j]
        dp = np.diff(p)
        # dominant direction over 0.5s window
        for i in range(len(dp)):
            if abs(dp[i]) < eps:
                continue
            w0 = np.searchsorted(T, T[i] - 0.25)
            w1 = np.searchsorted(T, T[i] + 0.25)
            trend = p[min(w1, len(p)-1)] - p[w0]
            if abs(trend) > 5 * eps and np.sign(dp[i]) != np.sign(trend) and abs(dp[i]) > eps:
                total += 1
                if total <= 25:
                    print(f"  t={T[i]-t0:8.3f}s {n}: step={dp[i]:+.5f} rad against trend {trend:+.4f} "
                          f"(pos {p[i]:.4f}->{p[i+1]:.4f})")
    print(f"  total regression steps: {total}")

find_regressions(data["/arm_controller/joint_trajectory"], "/arm_controller/joint_trajectory (servo output)")
find_regressions(data["/isaac_joint_commands"], "/isaac_joint_commands (topic_based output)")
find_regressions(data["/isaac_joint_states"], "/isaac_joint_states (measured sim)")
